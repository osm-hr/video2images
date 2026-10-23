#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v2i_common.py -- shared helpers for the ``*-v2i.py`` camera video exporters.

The three exporter scripts in this repository

    * ``virb30-v2i.py``     (Garmin VIRB Ultra 30)
    * ``max360-v2i.py``     (GoPro MAX 360)
    * ``blackvue900-v21.py`` (BlackVue DR900)

all do the same job for a different camera: walk a telemetry track, pick a
frame every ``--distance`` metres of travel and write it as a georeferenced
still image for Mapillary / Panoramax.  This module holds what they share so
the behaviour cannot drift apart:

    * camera detection from container metadata / file names,
    * video discovery for ``--input`` (file *or* directory),
    * the common command-line arguments (identical names, defaults and help
      text in every script),
    * small geodesy / track interpolation helpers,
    * generic ffmpeg frame extraction and exiftool batching.

Python standard library only; ``ffprobe``/``ffmpeg``/``exiftool`` are expected
on ``PATH`` at run time.
"""

from __future__ import annotations

import argparse
import datetime as dt
import math
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

__version__ = "2.0.0"

FFPROBE = os.environ.get("V2I_FFPROBE", "ffprobe")

# ---------------------------------------------------------------------------
# Camera identities
# ---------------------------------------------------------------------------
# Stable identifiers used by every script when they ask "whose file is this?".
CAMERA_VIRB = "garmin-virb-ultra-30"
CAMERA_MAX = "gopro-max-360"
CAMERA_BLACKVUE = "blackvue-dr900"

CAMERA_LABELS = {
    CAMERA_VIRB: "Garmin VIRB Ultra 30",
    CAMERA_MAX: "GoPro MAX 360",
    CAMERA_BLACKVUE: "BlackVue DR900",
}

# Default scanned extensions.  Deliberately broad: every script scans the same
# set and then uses :func:`detect_camera` to keep only the files its camera
# recorded, logging a reason for the ones it drops.
DEFAULT_EXTENSIONS = ".mp4,.mov,.m4v,.360"

# File-name conventions used by the cameras (supporting evidence only -- the
# container metadata below is the authoritative signal).
_RE_VIRB_NAME = re.compile(r"^VIRB[0-9]+\.(mp4|mov)$", re.IGNORECASE)
_RE_MAX_NAME = re.compile(r"^GS[0-9]+\.360$", re.IGNORECASE)
_RE_BLACKVUE_NAME = re.compile(r"^[0-9]{8}_[0-9]{6}_[A-Za-z]{2}\.(mp4|mov)$",
                               re.IGNORECASE)

# Container markers searched for when ffprobe handler names are inconclusive.
_VIRB_MARKERS = (b"virbactioncamera", b"garmin")
_MAX_MARKERS = (b"gopro",)
_BLACKVUE_MARKERS = (b"pittasoft", b"blackvue")


# ---------------------------------------------------------------------------
# Errors
# ---------------------------------------------------------------------------
class VideoError(Exception):
    """Base class for problems with an input video."""


class UnreadableVideo(VideoError):
    """The file exists but cannot be probed as a video."""


# ---------------------------------------------------------------------------
# Metadata probing and camera detection
# ---------------------------------------------------------------------------
@dataclass
class VideoMeta:
    """The metadata ffprobe gives us for one file."""

    path: str
    format_tags: Dict[str, str] = field(default_factory=dict)
    stream_tags: List[Dict[str, str]] = field(default_factory=list)
    handler_names: List[str] = field(default_factory=list)
    codec_tags: List[str] = field(default_factory=list)
    video_streams: int = 0
    has_gpmd: bool = False
    duration: Optional[float] = None

    @property
    def handler_text(self) -> str:
        return " ".join(self.handler_names).lower()

    @property
    def format_text(self) -> str:
        return " ".join(f"{k}={v}" for k, v in self.format_tags.items()).lower()


@dataclass
class Detection:
    """Result of :func:`detect_camera`."""

    camera: Optional[str]
    label: Optional[str]
    evidence: Dict[str, List[str]]
    reason: Optional[str] = None

    @property
    def ok(self) -> bool:
        return self.camera is not None


def _run(cmd: Sequence[str]) -> Tuple[int, bytes, bytes]:
    try:
        proc = subprocess.run(list(cmd), stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE)
    except FileNotFoundError:
        raise UnreadableVideo("required program not found: %s" % cmd[0])
    return proc.returncode, proc.stdout, proc.stderr


def probe_metadata(path: str, ffprobe: str = FFPROBE) -> VideoMeta:
    """Run ``ffprobe`` and summarise the container metadata of *path*."""
    rc, out, err = _run([ffprobe, "-v", "error", "-print_format", "json",
                         "-show_format", "-show_streams", path])
    if rc != 0:
        tail = err.decode("utf-8", "replace").strip().splitlines()
        raise UnreadableVideo(tail[-1] if tail else "ffprobe failed")
    try:
        import json
        data = json.loads(out.decode("utf-8", "replace"))
    except ValueError:
        raise UnreadableVideo("ffprobe returned unreadable output")

    streams = data.get("streams", [])
    if not any(s.get("codec_type") == "video" for s in streams):
        raise UnreadableVideo("no video stream (not a playable video file)")

    meta = VideoMeta(path=path)
    meta.format_tags = {str(k): str(v)
                        for k, v in (data.get("format", {}).get("tags") or {}).items()}
    for stream in streams:
        tags = {str(k): str(v) for k, v in (stream.get("tags") or {}).items()}
        meta.stream_tags.append(tags)
        if tags.get("handler_name"):
            meta.handler_names.append(tags["handler_name"])
        if stream.get("codec_tag_string"):
            meta.codec_tags.append(stream["codec_tag_string"])
        if stream.get("codec_type") == "video":
            meta.video_streams += 1
        if (stream.get("codec_type") == "data"
                and stream.get("codec_tag_string") == "gpmd"):
            meta.has_gpmd = True
    try:
        meta.duration = float(data.get("format", {}).get("duration") or 0.0)
    except (TypeError, ValueError):
        meta.duration = None
    return meta


def _scan_markers(path: str, needles: Iterable[bytes],
                  head: int = 8 * 1024 * 1024,
                  tail: int = 8 * 1024 * 1024) -> set:
    """Return which of *needles* appear in the head/tail of *path*.

    Container metadata (the Garmin ``uuid`` copyright atom, the BlackVue
    ``cprt`` atom, ...) sits near the start or the end of a recording, so
    scanning both ends is enough to identify the camera without reading a
    multi-gigabyte file twice.
    """
    found = set()
    needles = list(needles)
    try:
        size = os.path.getsize(path)
        with open(path, "rb") as fh:
            chunks = [fh.read(head)]
            if size > head + tail:
                fh.seek(size - tail)
                chunks.append(fh.read(tail))
        blob = b"".join(chunks).lower()
    except OSError:
        return found
    for needle in needles:
        if needle in blob:
            found.add(needle)
    return found


def detect_camera(path: str, *, ffprobe: str = FFPROBE) -> Detection:
    """Identify the camera that recorded *path* from its metadata.

    Returns a :class:`Detection`.  ``camera`` is ``None`` when the file is
    unreadable, when no known camera can be identified, or when the evidence is
    ambiguous; ``reason`` then explains why.  Detection never guesses a camera
    merely because metadata is missing.
    """
    name = os.path.basename(path)
    try:
        meta = probe_metadata(path, ffprobe)
    except UnreadableVideo as exc:
        return Detection(None, None, {}, "invalid or unreadable video (%s)" % exc)

    evidence: Dict[str, List[str]] = {
        CAMERA_VIRB: [], CAMERA_MAX: [], CAMERA_BLACKVUE: [],
    }
    handlers = meta.handler_text
    if "garmin" in handlers:
        evidence[CAMERA_VIRB].append("Garmin handler name in container metadata")
    if "pittasoft" in handlers or "blackvue" in handlers:
        evidence[CAMERA_BLACKVUE].append(
            "PittaSoft/BlackVue handler name in container metadata")
    if "gopro" in handlers:
        evidence[CAMERA_MAX].append("GoPro handler name in container metadata")
    if meta.has_gpmd and meta.video_streams >= 2:
        evidence[CAMERA_MAX].append("GoPro gpmd telemetry track")

    if _RE_VIRB_NAME.match(name):
        evidence[CAMERA_VIRB].append("VIRB file name (%s)" % name)
    if _RE_MAX_NAME.match(name):
        evidence[CAMERA_MAX].append("GoPro MAX file name (%s)" % name)
    if _RE_BLACKVUE_NAME.match(name):
        evidence[CAMERA_BLACKVUE].append("BlackVue file name (%s)" % name)

    # Only consult the raw container when the cheap signals were inconclusive.
    if not any(evidence.values()):
        markers = _scan_markers(path, _VIRB_MARKERS + _MAX_MARKERS + _BLACKVUE_MARKERS)
        if _VIRB_MARKERS[0] in markers or _VIRB_MARKERS[1] in markers:
            evidence[CAMERA_VIRB].append("Garmin container metadata")
        if _MAX_MARKERS[0] in markers:
            evidence[CAMERA_MAX].append("GoPro container metadata")
        if _BLACKVUE_MARKERS[0] in markers or _BLACKVUE_MARKERS[1] in markers:
            evidence[CAMERA_BLACKVUE].append("BlackVue container metadata")

    identified = [cam for cam, why in evidence.items() if why]
    if len(identified) == 1:
        camera = identified[0]
        return Detection(camera, CAMERA_LABELS[camera], evidence, None)
    if not identified:
        return Detection(None, None, evidence,
                         "unsupported camera (missing camera metadata)")
    labels = ", ".join(CAMERA_LABELS[c] for c in identified)
    return Detection(None, None, evidence, "ambiguous camera metadata (%s)" % labels)


def skip_reason(detection: Detection, expected: str) -> Optional[str]:
    """Human-readable reason to skip *detection*, or ``None`` to process it.

    ``expected`` is the camera the running script supports.
    """
    if detection.camera == expected:
        return None
    if not detection.ok:
        return detection.reason or "unsupported camera"
    return "recorded by %s, not %s" % (detection.label,
                                       CAMERA_LABELS[expected])


# ---------------------------------------------------------------------------
# Video discovery
# ---------------------------------------------------------------------------
def split_extensions(text: str) -> Tuple[str, ...]:
    """Parse ``--extensions`` into a normalised, lower-case tuple."""
    parts = re.split(r"[,\s]+", text or "")
    exts = []
    for part in parts:
        part = part.strip().lower()
        if not part:
            continue
        if not part.startswith("."):
            part = "." + part
        if part not in exts:
            exts.append(part)
    return tuple(exts)


def glob_to_extensions(pattern: str) -> str:
    """Best-effort conversion of a legacy ``--video-glob`` to an extension list."""
    pieces = []
    for part in re.split(r"[,\s]+", pattern or ""):
        part = part.strip()
        if part.startswith("*."):
            pieces.append(part[1:])
        elif part.startswith("."):
            pieces.append(part)
        elif part:
            pieces.append(part)
    return ",".join(pieces) if pieces else pattern


def discover_videos(input_path: str, extensions: Sequence[str],
                    recursive: bool) -> List[str]:
    """Return the video files under *input_path*.

    *input_path* may be a single file (used verbatim, regardless of its
    extension) or a directory.  Extension matching is case-insensitive.
    """
    input_path = os.path.abspath(input_path)
    if os.path.isfile(input_path):
        return [os.path.normpath(input_path)]
    if not os.path.isdir(input_path):
        return []

    exts = tuple(e.lower() for e in extensions)
    found: List[str] = []
    if recursive:
        for current, dirs, files in os.walk(input_path):
            dirs.sort()
            for name in sorted(files):
                if os.path.splitext(name)[1].lower() in exts:
                    found.append(os.path.join(current, name))
    else:
        try:
            names = sorted(os.listdir(input_path))
        except OSError:
            return []
        for name in names:
            full = os.path.join(input_path, name)
            if os.path.isfile(full) and os.path.splitext(name)[1].lower() in exts:
                found.append(full)
    return sorted({os.path.normpath(p) for p in found})


# ---------------------------------------------------------------------------
# Shared command-line interface
# ---------------------------------------------------------------------------
_LOG_LEVELS = ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL")


def add_common_arguments(parser: argparse.ArgumentParser, *,
                         default_extensions: str = DEFAULT_EXTENSIONS) -> None:
    """Register the arguments every exporter shares.

    Every script calls this, so the names, defaults and help text of the
    common options are identical by construction.
    """
    parser.add_argument("-i", "--input", required=True, metavar="PATH",
                        help="input video file or directory to scan (read only)")
    parser.add_argument("-o", "--output", required=True, metavar="PATH",
                        help="output directory; created automatically if missing")
    parser.add_argument("-r", "--recursive", dest="recursive",
                        action="store_true", default=True,
                        help="search input directories recursively (default)")
    parser.add_argument("--no-recursive", dest="recursive",
                        action="store_false",
                        help="do not descend into sub-directories")
    parser.add_argument("-d", "--distance", type=float, default=5.0,
                        metavar="METERS",
                        help="distance between exported frames in meters")
    parser.add_argument("--start-distance", type=float, default=0.0,
                        metavar="METERS",
                        help="skip this many meters of travel after the start "
                             "of the track before the first frame")
    parser.add_argument("--max-images", type=int, default=None, metavar="N",
                        help="maximum number of images to export per video")
    parser.add_argument("--extensions", default=default_extensions,
                        metavar="LIST",
                        help="comma-separated video file extensions to scan")
    parser.add_argument("--heading-offset", dest="heading_offset", type=float,
                        default=0.0, metavar="DEGREES",
                        help="degrees added to the GPS course to derive the "
                             "image heading (-180..180)")
    parser.add_argument("--offset", dest="heading_offset", type=float,
                        help=argparse.SUPPRESS)          # legacy VIRB alias
    parser.add_argument("--timezone", dest="timezone", default=None,
                        metavar="OFFSET",
                        help="override the camera UTC offset, e.g. +02:00 "
                             "(default: detect from the video)")
    parser.add_argument("--tz", dest="timezone",
                        help=argparse.SUPPRESS)          # legacy MAX alias
    parser.add_argument("--time-base", choices=("local", "utc"), default="local",
                        help="wall clock used for file names and EXIF timestamps")
    parser.add_argument("--format", choices=("jpg", "png"), default="jpg",
                        help="output image format")
    parser.add_argument("--quality", type=int, default=95, metavar="1-100",
                        help="image quality, 100 = best")
    parser.add_argument("--overwrite", action="store_true",
                        help="overwrite existing images instead of keeping them")
    parser.add_argument("--dry-run", action="store_true",
                        help="report what would be exported without writing anything")
    parser.add_argument("--fail-fast", action="store_true",
                        help="stop on the first file that errors")
    parser.add_argument("--manifest", default=None, metavar="FILE",
                        help="write a JSON manifest of all exported points")
    parser.add_argument("--workers", dest="workers", type=int, default=1,
                        metavar="N",
                        help="number of frames to extract in parallel")
    parser.add_argument("--jobs", dest="workers", type=int,
                        help=argparse.SUPPRESS)          # legacy MAX alias
    parser.add_argument("--threads", type=int, default=0, metavar="N",
                        help="ffmpeg threads per extraction job (0 = auto)")
    parser.add_argument("--no-exif", action="store_true",
                        help="do not write EXIF/XMP metadata")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="verbose (debug) logging")
    parser.add_argument("-q", "--quiet", action="store_true",
                        help="only report errors")
    parser.add_argument("--log-level", default="INFO", type=str.upper,
                        choices=_LOG_LEVELS,
                        help="logging verbosity")
    parser.add_argument("--version", action="version",
                        version="%(prog)s " + __version__)


def add_gps_filter_arguments(parser: argparse.ArgumentParser) -> None:
    """GPS-noise options shared by the GPMF (MAX) and NMEA (BlackVue) readers."""
    group = parser.add_argument_group("GPS handling")
    group.add_argument("--stationary-speed", type=float, default=0.3,
                       metavar="MPS",
                       help="speed below which the camera is treated as parked "
                            "and no distance is accumulated")
    group.add_argument("--smooth", type=int, default=9, metavar="N",
                       help="moving-average window (samples) applied to GPS "
                            "positions")
    group.add_argument("--jitter", type=float, default=0.5, metavar="METERS",
                       help="GPS steps shorter than this are treated as noise "
                            "when the video carries no speed data")


def add_track_export_arguments(parser: argparse.ArgumentParser) -> None:
    """Track/export options shared by the FIT (VIRB) and NMEA (BlackVue) jobs."""
    parser.add_argument("--distance-mode", choices=("2d", "3d"), default="2d",
                        help="measure distance travelled on the ground (default) "
                             "or in 3D")
    parser.add_argument("--sidecars", action="store_true",
                        help="also write a <sequence>.track.jsonl manifest")
    parser.add_argument("--fallback-interval", type=float, default=0.0,
                        metavar="SECONDS",
                        help="if a video has no telemetry, sample every N seconds "
                             "instead of skipping it (default 0 = skip)")


def resolve_extensions(args: argparse.Namespace) -> Tuple[str, ...]:
    """Resolve ``--extensions`` (or a legacy ``--video-glob``) to a tuple."""
    text = getattr(args, "extensions", None)
    legacy = getattr(args, "video_glob", None)
    if legacy and (not text or text == DEFAULT_EXTENSIONS):
        text = glob_to_extensions(legacy)
    return split_extensions(text or DEFAULT_EXTENSIONS)


def parse_timezone(text: Optional[str]) -> Optional[int]:
    """Parse a timezone offset into seconds, or ``None`` when not given.

    Accepts ``+02:00``, ``02:00``, ``+2``, ``-5`` and decimal hours (``2.5``).
    """
    if text is None:
        return None
    text = str(text).strip()
    if not text:
        return None
    match = re.match(r"^([+-]?)(\d{1,2}):(\d{2})$", text)
    if match:
        sign = -1 if match.group(1) == "-" else 1
        return sign * (int(match.group(2)) * 3600 + int(match.group(3)) * 60)
    match = re.match(r"^([+-]?)(\d{1,2})$", text)
    if match:
        sign = -1 if match.group(1) == "-" else 1
        return sign * int(match.group(2)) * 3600
    try:
        hours = float(text)
    except ValueError:
        raise ValueError("bad timezone offset: %s (use e.g. +02:00)" % text)
    return int(round(hours * 3600))


def validate_common_args(parser: argparse.ArgumentParser,
                         args: argparse.Namespace) -> None:
    """Validate the shared arguments, calling ``parser.error`` on failure."""
    if args.distance <= 0:
        parser.error("--distance must be greater than zero (got %g)" % args.distance)
    if args.start_distance < 0:
        parser.error("--start-distance must not be negative (got %g)"
                     % args.start_distance)
    if args.max_images is not None and args.max_images < 1:
        parser.error("--max-images must be at least 1 (got %d)" % args.max_images)
    if not (1 <= args.quality <= 100):
        parser.error("--quality must be between 1 and 100 (got %d)" % args.quality)
    if args.workers < 1:
        parser.error("--workers must be at least 1 (got %d)" % args.workers)
    if args.threads < 0:
        parser.error("--threads must not be negative (got %d)" % args.threads)
    if not (-180.0 <= args.heading_offset <= 180.0):
        parser.error("--heading-offset must be between -180 and +180 degrees "
                     "(got %g)" % args.heading_offset)
    try:
        parse_timezone(args.timezone)
    except ValueError as exc:
        parser.error(str(exc))
    if not resolve_extensions(args):
        parser.error("--extensions must list at least one extension")
    if not os.path.exists(args.input):
        parser.error("--input does not exist: %s" % args.input)
    if os.path.exists(args.output) and not os.path.isdir(args.output):
        parser.error("--output exists and is not a directory: %s" % args.output)
    output = os.path.abspath(args.output)
    input_path = os.path.abspath(args.input)
    if os.path.isdir(input_path) and (output == input_path or
                                      output.startswith(input_path + os.sep)):
        LOG.warning("output directory is inside the input tree; make sure it "
                    "holds no videos (%s)", output)


def output_inside_input(args: argparse.Namespace) -> bool:
    input_path = os.path.abspath(args.input)
    output = os.path.abspath(args.output)
    return os.path.isdir(input_path) and (
        output == input_path or output.startswith(input_path + os.sep))


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
LOG = __import__("logging").getLogger("v2i")


def setup_logging(level: str = "INFO", *, verbose: bool = False,
                  quiet: bool = False) -> None:
    """Configure root logging for the scripts."""
    import logging
    if verbose:
        level = "DEBUG"
    elif quiet:
        level = "ERROR"
    root = logging.getLogger()
    for handler in list(root.handlers):
        root.removeHandler(handler)
    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))
    root.addHandler(handler)
    root.setLevel(level)


def log_level(args: argparse.Namespace) -> str:
    if getattr(args, "verbose", False):
        return "DEBUG"
    if getattr(args, "quiet", False):
        return "ERROR"
    return getattr(args, "log_level", "INFO")


# ---------------------------------------------------------------------------
# Geodesy / track interpolation (shared by the VIRB and BlackVue jobs)
# ---------------------------------------------------------------------------
EARTH_RADIUS_M = 6371008.8


def haversine(lat1, lon1, lat2, lon2):
    radius = EARTH_RADIUS_M
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * radius * math.asin(min(1.0, math.sqrt(a)))


def _segment_distance(a, b, mode):
    flat = haversine(a["lat"], a["lon"], b["lat"], b["lon"])
    if mode == "3d":
        dz = (b.get("alt") or 0.0) - (a.get("alt") or 0.0)
        return math.hypot(flat, dz)
    return flat


class Track(object):
    """A GPS track with a cumulative-distance curve and interpolated access.

    *samples* are dicts carrying at least ``t``, ``lat`` and ``lon``; optional
    ``alt``, ``speed``, ``heading`` and ``velocity`` keys are interpolated too.
    """

    def __init__(self, samples, mode="2d"):
        self.samples = samples
        self.mode = mode
        self.cum = [0.0]
        for a, b in zip(samples, samples[1:]):
            self.cum.append(self.cum[-1] + _segment_distance(a, b, mode))
        self.length = self.cum[-1] if samples else 0.0

    def _locate(self, distance):
        samples = self.samples
        count = len(samples)
        if count == 1:
            return 0, 0.0
        if distance <= 0.0:
            return 0, 0.0
        if distance >= self.cum[-1]:
            return count - 2, 1.0
        lo, hi = 0, count - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.cum[mid] <= distance:
                lo = mid
            else:
                hi = mid
        span = self.cum[hi] - self.cum[lo]
        fraction = 0.0 if span <= 0 else (distance - self.cum[lo]) / span
        return lo, fraction

    @staticmethod
    def _interp(a, b, fraction):
        out = {"t": a["t"] + fraction * (b["t"] - a["t"])}
        for key in ("lat", "lon", "alt", "speed", "heading"):
            va, vb = a.get(key), b.get(key)
            if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
                if key == "heading":
                    delta = (vb - va + 540.0) % 360.0 - 180.0
                    out[key] = (va + fraction * delta) % 360.0
                else:
                    out[key] = va + fraction * (vb - va)
            else:
                out[key] = va if va is not None else vb
        va, vb = a.get("velocity"), b.get("velocity")
        if isinstance(va, (list, tuple)) and isinstance(vb, (list, tuple)):
            out["velocity"] = [va[i] + fraction * (vb[i] - va[i])
                               for i in range(min(3, len(va), len(vb)))]
        return out

    def at_distance(self, distance):
        if not self.samples:
            return None
        if len(self.samples) == 1:
            point = dict(self.samples[0])
        else:
            lo, fraction = self._locate(distance)
            point = self._interp(self.samples[lo], self.samples[lo + 1], fraction)
        point["distance"] = distance
        point["grade"] = self.grade_degrees(point)
        return point

    @staticmethod
    def grade_degrees(point):
        velocity = point.get("velocity")
        if not velocity:
            return None
        horizontal = math.hypot(velocity[0], velocity[1])
        if horizontal < 0.5:
            return None
        return math.degrees(math.atan2(velocity[2], horizontal))


def plan_shots(track: Track, interval: float, *, start_distance: float = 0.0,
               max_shots: Optional[int] = None) -> List[dict]:
    """Target points every *interval* metres along *track*.

    Targets sit at ``start_distance, start_distance+interval, ...``; at a
    ``start_distance`` of ``0`` the first point is the track origin.
    """
    shots: List[dict] = []
    if not track.samples or interval <= 0:
        return shots
    if track.length <= 0.0:
        point = track.at_distance(0.0)
        return [point] if point else shots
    base = max(0.0, start_distance)
    if base > track.length:
        return shots
    count = int(math.floor((track.length - base) / interval + 1e-9))
    for index in range(count + 1):
        point = track.at_distance(base + index * interval)
        if point is not None:
            shots.append(point)
        if max_shots is not None and len(shots) >= max_shots:
            break
    return shots


# ---------------------------------------------------------------------------
# Frame extraction (VIRB / BlackVue -- a plain seek + single frame)
# ---------------------------------------------------------------------------
def ffmpeg_quality_args(image_format: str, quality: int) -> List[str]:
    """Map a 1..100 quality to ffmpeg encoder options (higher = better)."""
    quality = max(1, min(100, quality))
    if image_format == "png":
        level = int(round((100 - quality) / 100.0 * 9))
        return ["-compression_level", str(max(0, min(9, level)))]
    qscale = int(round(31 - (quality / 100.0) * 30))
    return ["-q:v", str(max(1, min(31, qscale)))]


def extract_frames(video: str, jobs: Sequence[dict], tmp_dir: str, *,
                   image_format: str, quality: int, workers: int, fps: float,
                   threads: int = 0) -> Dict[int, str]:
    """Extract each job's frame with ffmpeg into *tmp_dir*.

    *jobs* is a list of dicts with ``index`` (unique), ``pts`` (seconds) and
    (optionally) ``dest``.  Returns ``{index: temp_path}`` for the frames that
    were extracted.
    """
    from concurrent.futures import ThreadPoolExecutor

    preroll = 0.25 / max(fps or 1.0, 1.0)
    quality_args = ffmpeg_quality_args(image_format, quality)

    def extract_one(job):
        out_path = os.path.join(tmp_dir, "f_%06d.%s" % (job["index"], image_format))
        seek = max(0.0, job["pts"] - preroll)
        cmd = ["ffmpeg", "-v", "error", "-y", "-ss", "%.6f" % seek,
               "-i", video, "-frames:v", "1"]
        cmd += quality_args
        if threads:
            cmd += ["-threads", str(threads)]
        cmd.append(out_path)
        proc = subprocess.run(cmd, stdout=subprocess.DEVNULL,
                              stderr=subprocess.PIPE)
        if proc.returncode == 0 and os.path.exists(out_path):
            return job["index"], out_path
        LOG.warning("frame %d (%.3f s) could not be extracted: %s",
                    job["index"], job["pts"],
                    proc.stderr.decode("utf-8", "replace").strip().splitlines()[-1:]
                    or "")
        return job["index"], None

    extracted: Dict[int, str] = {}
    if workers > 1 and len(jobs) > 1:
        with ThreadPoolExecutor(max_workers=workers) as pool:
            for index, path in pool.map(extract_one, jobs):
                if path:
                    extracted[index] = path
    else:
        for job in jobs:
            index, path = extract_one(job)
            if path:
                extracted[index] = path
    return extracted


# ---------------------------------------------------------------------------
# Metadata helpers
# ---------------------------------------------------------------------------
def exif_datetime(value: dt.datetime) -> str:
    return value.strftime("%Y:%m:%d %H:%M:%S")


def offset_string(seconds: int) -> str:
    sign = "+" if seconds >= 0 else "-"
    seconds = abs(int(seconds))
    return "%s%02d:%02d" % (sign, seconds // 3600, (seconds % 3600) // 60)


def write_metadata(pairs: Sequence[Tuple[str, Sequence[str]]], *,
                   batch: int = 40) -> None:
    """Write metadata for ``[(path, exiftool_args), ...]``.

    ``exiftool_args`` must not include the file name; images are batched into
    as few exiftool processes as possible.
    """
    if not pairs:
        return
    for start in range(0, len(pairs), batch):
        chunk = pairs[start:start + batch]
        cmd = ["exiftool", "-q", "-q", "-overwrite_original"]
        for path, tag_args in chunk:
            cmd += list(tag_args)
            cmd.append(path)
            cmd.append("-execute")
        cmd.append("-execute0")
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if proc.returncode != 0:
            LOG.error("exiftool failed: %s",
                      proc.stderr.decode("utf-8", "replace").strip())


# ---------------------------------------------------------------------------
# Sequence naming
# ---------------------------------------------------------------------------
def sanitize(text: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9._-]+", "-", text)
    return clean.strip("-") or "video"


def build_sequence_ids(videos: Sequence[str], input_dir: str) -> Dict[str, str]:
    """Map each video to a collision-free sequence directory name."""
    import collections

    stems: Dict[str, Tuple[str, str]] = {}
    for video in videos:
        stem = os.path.splitext(os.path.basename(video))[0]
        rel_dir = os.path.relpath(os.path.dirname(os.path.abspath(video)),
                                  input_dir)
        stems[video] = (stem, rel_dir)

    counts = collections.Counter(stem.lower() for stem, _ in stems.values())
    result: Dict[str, str] = {}
    used = set()
    for video in videos:
        stem, rel_dir = stems[video]
        nested = rel_dir not in (".", "")
        if counts[stem.lower()] > 1 and nested:
            name = sanitize(rel_dir) + "__" + stem
        else:
            name = stem
        name = sanitize(name)
        unique = name
        suffix = 2
        while unique.lower() in used:
            unique = "%s-%d" % (name, suffix)
            suffix += 1
        used.add(unique.lower())
        result[video] = unique
    return result
