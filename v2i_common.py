#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v2i_common.py -- shared helpers for the ``*-v2i.py`` camera video exporters.

The four exporter scripts in this repository

    * ``virb30-v2i.py``       (Garmin VIRB Ultra 30)
    * ``max360-v2i.py``       (GoPro MAX 360)
    * ``blackvue900-v2i.py``  (BlackVue DR900)
    * ``hero-v2i.py``         (GoPro HERO)

all do the same job for a different camera: walk a telemetry track, pick a
unique video frame every ``--distance`` metres of travel and write it as a
georeferenced still image for Mapillary / Panoramax.  This module holds what
they share so behaviour cannot drift apart:

    * evidence-based camera detection (including GoPro MAX vs HERO),
    * video discovery for ``--input`` (file *or* directory),
    * the common command-line arguments,
    * geodesy, a time-indexed ``Track`` and distance planning,
    * frame PTS timelines, unique-frame selection and exact extraction,
    * output naming and collision-safe sequence directories,
    * GPX 1.1 / GeoJSON track writers and exiftool batching.

Python standard library only; ``ffprobe``/``ffmpeg``/``exiftool`` are expected
on ``PATH`` at run time.
"""

from __future__ import annotations

import argparse
import bisect
import datetime as dt
import json
import logging
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

__version__ = "3.0.0"

FFPROBE = os.environ.get("V2I_FFPROBE", "ffprobe")
FFMPEG = os.environ.get("V2I_FFMPEG", "ffmpeg")

# ---------------------------------------------------------------------------
# Camera identities
# ---------------------------------------------------------------------------
CAMERA_VIRB = "garmin-virb-ultra-30"
CAMERA_MAX = "gopro-max-360"
CAMERA_BLACKVUE = "blackvue-dr900"
CAMERA_HERO = "gopro-hero"

CAMERA_LABELS = {
    CAMERA_VIRB: "Garmin VIRB Ultra 30",
    CAMERA_MAX: "GoPro MAX 360",
    CAMERA_BLACKVUE: "BlackVue DR900",
    CAMERA_HERO: "GoPro HERO",
}

# Stable output-directory suffix (``YYYY-MM-DD-<suffix>``).
CAMERA_SUFFIX = {
    CAMERA_VIRB: "virb30",
    CAMERA_MAX: "max360",
    CAMERA_BLACKVUE: "blackvue",
    CAMERA_HERO: "hero",
}

# Default scanned extensions.  Every script scans the same set and uses
# :func:`detect_camera` to keep only files its camera recorded.
DEFAULT_EXTENSIONS = ".mp4,.mov,.m4v,.360"

# File-name conventions (supporting evidence only -- container metadata and the
# embedded GPMF device name are authoritative).
_RE_VIRB_NAME = re.compile(r"^VIRB[0-9]+\.(mp4|mov)$", re.IGNORECASE)
_RE_MAX_NAME = re.compile(r"^GS[0-9A-Z]+\.(360|mp4|mov)$", re.IGNORECASE)
_RE_HERO_NAME = re.compile(r"^(GX|GH|GOPR|GOPRO)[0-9A-Z]*\.(mp4|mov)$",
                           re.IGNORECASE)
_RE_BLACKVUE_NAME = re.compile(r"^[0-9]{8}_[0-9]{6}_[A-Za-z]{2}\.(mp4|mov)$",
                               re.IGNORECASE)

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


class SelectionError(VideoError):
    """No valid frame could be selected."""


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
    video_indices: List[int] = field(default_factory=list)
    has_gpmd: bool = False
    gpmd_index: Optional[int] = None
    duration: Optional[float] = None
    streams: List[dict] = field(default_factory=list)

    @property
    def handler_text(self) -> str:
        return " ".join(self.handler_names).lower()

    @property
    def format_text(self) -> str:
        return " ".join(f"{k}={v}" for k, v in self.format_tags.items()).lower()

    @property
    def creation_time(self) -> Optional[str]:
        for key, value in self.format_tags.items():
            if key.lower() == "creation_time":
                return value
        return None

    def video_streams_info(self) -> List[dict]:
        return [s for s in self.streams if s.get("codec_type") == "video"]


@dataclass
class Detection:
    """Result of :func:`detect_camera`."""

    camera: Optional[str]
    label: Optional[str]
    evidence: Dict[str, List[str]]
    reason: Optional[str] = None
    device_name: Optional[str] = None

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


@lru_cache(maxsize=64)
def _probe_metadata_cached(path: str, mtime: float,
                           ffprobe: str) -> VideoMeta:
    return _probe_metadata_impl(path, ffprobe)


def probe_metadata(path: str, ffprobe: str = FFPROBE) -> VideoMeta:
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        return _probe_metadata_impl(path, ffprobe)
    return _probe_metadata_cached(path, mtime, ffprobe)


def _probe_metadata_impl(path: str, ffprobe: str) -> VideoMeta:
    """Run ``ffprobe`` and summarise the container metadata of *path*."""
    rc, out, err = _run([ffprobe, "-v", "error", "-print_format", "json",
                         "-show_format", "-show_streams", path])
    if rc != 0:
        tail = err.decode("utf-8", "replace").strip().splitlines()
        raise UnreadableVideo(tail[-1] if tail else "ffprobe failed")
    try:
        data = json.loads(out.decode("utf-8", "replace"))
    except ValueError:
        raise UnreadableVideo("ffprobe returned unreadable output")

    streams = data.get("streams", [])
    if not any(s.get("codec_type") == "video" for s in streams):
        raise UnreadableVideo("no video stream (not a playable video file)")

    meta = VideoMeta(path=path)
    meta.streams = streams
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
            try:
                meta.video_indices.append(int(stream.get("index")))
            except (TypeError, ValueError):
                pass
        if (stream.get("codec_type") == "data"
                and str(stream.get("codec_tag_string", "")).lower() == "gpmd"):
            meta.has_gpmd = True
            try:
                meta.gpmd_index = int(stream.get("index"))
            except (TypeError, ValueError):
                meta.gpmd_index = None
    try:
        meta.duration = float(data.get("format", {}).get("duration") or 0.0)
    except (TypeError, ValueError):
        meta.duration = None
    return meta


def _scan_markers(path: str, needles: Iterable[bytes],
                  head: int = 8 * 1024 * 1024,
                  tail: int = 8 * 1024 * 1024) -> set:
    """Return which of *needles* appear in the head/tail of *path*."""
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


def read_gpmf_device_name(path: str, stream_index: Optional[int],
                          ffmpeg: str = FFMPEG) -> Optional[str]:
    """Read the first GPMF ``DVNM`` device name from a gpmd track.

    Only the first second is copied, which is enough for the device name and
    keeps camera detection cheap.  Returns ``None`` when it cannot be read.
    """
    if stream_index is None:
        return None
    try:
        proc = subprocess.run(
            [ffmpeg, "-v", "error", "-i", path, "-map", "0:%d" % stream_index,
             "-t", "1", "-c", "copy", "-f", "data", "-"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        return None
    if proc.returncode != 0 or not proc.stdout:
        return None
    try:
        import v2i_gpmf
        return v2i_gpmf.read_device_name(proc.stdout)
    except Exception:                                      # noqa: BLE001
        return None


def detect_camera(path: str, *, ffprobe: str = FFPROBE) -> Detection:
    """Identify the camera that recorded *path* from its metadata.

    Returns a :class:`Detection`.  ``camera`` is ``None`` when the file is
    unreadable, when no known camera can be identified, or when evidence is
    ambiguous; ``reason`` then explains why.  Detection never guesses a camera
    merely because metadata is missing.
    """
    name = os.path.basename(path)
    try:
        meta = probe_metadata(path, ffprobe)
    except UnreadableVideo as exc:
        return Detection(None, None, {}, "invalid or unreadable video (%s)" % exc)

    evidence: Dict[str, List[str]] = {
        CAMERA_VIRB: [], CAMERA_MAX: [], CAMERA_BLACKVUE: [], CAMERA_HERO: [],
    }
    handlers = meta.handler_text
    if "garmin" in handlers:
        evidence[CAMERA_VIRB].append("Garmin handler name in container metadata")
    if "pittasoft" in handlers or "blackvue" in handlers:
        evidence[CAMERA_BLACKVUE].append(
            "PittaSoft/BlackVue handler name in container metadata")
    gpmf_device = None
    if "gopro" in handlers:
        evidence[CAMERA_MAX].append("GoPro handler name in container metadata")
        evidence[CAMERA_HERO].append("GoPro handler name in container metadata")

    if meta.has_gpmd:
        gpmf_device = read_gpmf_device_name(path, meta.gpmd_index)
        upper = (gpmf_device or "").upper()
        if "MAX" in upper:
            evidence[CAMERA_MAX].append("GPMF device name %r" % gpmf_device)
        elif "HERO" in upper:
            evidence[CAMERA_HERO].append("GPMF device name %r" % gpmf_device)
        elif gpmf_device:
            # Any other GoPro GPMF device: use the stream layout below.
            evidence[CAMERA_HERO].append(
                "GPMF telemetry from GoPro device %r" % gpmf_device)

    lower = name.lower()
    if _RE_VIRB_NAME.match(name):
        evidence[CAMERA_VIRB].append("VIRB file name (%s)" % name)
    if _RE_MAX_NAME.match(name):
        evidence[CAMERA_MAX].append("GoPro MAX file name (%s)" % name)
    if _RE_HERO_NAME.match(name):
        evidence[CAMERA_HERO].append("GoPro HERO file name (%s)" % name)
    if _RE_BLACKVUE_NAME.match(name):
        evidence[CAMERA_BLACKVUE].append("BlackVue file name (%s)" % name)

    # GoPro stream layout: MAX stores two EAC hemispheres; HERO is flat.
    if meta.has_gpmd:
        if meta.video_streams >= 2 or lower.endswith(".360"):
            evidence[CAMERA_MAX].append(
                "dual-video/EAC 360 layout (%d video stream(s))"
                % meta.video_streams)
        elif meta.video_streams == 1:
            evidence[CAMERA_HERO].append(
                "single flat video stream with GPMF telemetry")

    # Only consult the raw container when the cheap signals were inconclusive.
    if not any(evidence.values()):
        markers = _scan_markers(path, _VIRB_MARKERS + _MAX_MARKERS + _BLACKVUE_MARKERS)
        if _VIRB_MARKERS[0] in markers or _VIRB_MARKERS[1] in markers:
            evidence[CAMERA_VIRB].append("Garmin container metadata")
        if _MAX_MARKERS[0] in markers:
            evidence[CAMERA_MAX].append("GoPro container metadata")
            evidence[CAMERA_HERO].append("GoPro container metadata")
        if _BLACKVUE_MARKERS[0] in markers or _BLACKVUE_MARKERS[1] in markers:
            evidence[CAMERA_BLACKVUE].append("BlackVue container metadata")

    identified = [cam for cam, why in evidence.items() if why]
    if len(identified) == 1:
        camera = identified[0]
        return Detection(camera, CAMERA_LABELS[camera], evidence, None,
                         gpmf_device)

    # GoPro ambiguity: a MAX device name combined with a HERO name/layout, or
    # vice versa, is not guessed.  A clear DVNM wins when the other evidence is
    # only the generic GoPro handler string.
    if set(identified) == {CAMERA_MAX, CAMERA_HERO}:
        upper = (gpmf_device or "").upper()
        max_specific = [w for w in evidence[CAMERA_MAX]
                        if "handler name" not in w]
        hero_specific = [w for w in evidence[CAMERA_HERO]
                         if "handler name" not in w]
        if "MAX" in upper and not any("HERO" in w for w in hero_specific):
            return Detection(CAMERA_MAX, CAMERA_LABELS[CAMERA_MAX], evidence,
                             None, gpmf_device)
        if "HERO" in upper and not any("MAX" in w for w in max_specific):
            return Detection(CAMERA_HERO, CAMERA_LABELS[CAMERA_HERO], evidence,
                             None, gpmf_device)
        labels = "GoPro MAX 360 / GoPro HERO"
        return Detection(None, None, evidence,
                         "ambiguous GoPro model metadata (%s)" % labels,
                         gpmf_device)

    if not identified:
        return Detection(None, None, evidence,
                         "unsupported camera (missing camera metadata)",
                         gpmf_device)
    labels = ", ".join(CAMERA_LABELS[c] for c in identified)
    return Detection(None, None, evidence, "ambiguous camera metadata (%s)"
                     % labels, gpmf_device)


def skip_reason(detection: Detection, expected: str) -> Optional[str]:
    """Human-readable reason to skip *detection*, or ``None`` to process it."""
    if detection.camera == expected:
        return None
    if not detection.ok:
        return detection.reason or "unsupported camera"
    return "recorded by %s, not %s" % (detection.label, CAMERA_LABELS[expected])


def summarize_detections(counts: Dict[Optional[str], int]) -> str:
    """One-line count per known camera type plus unrecognised files."""
    parts = ["%s=%d" % (CAMERA_LABELS[camera], counts.get(camera, 0))
             for camera in (CAMERA_BLACKVUE, CAMERA_MAX, CAMERA_VIRB,
                            CAMERA_HERO)]
    unsupported = counts.get(None, 0)
    if unsupported:
        parts.append("unsupported=%d" % unsupported)
    return ", ".join(parts)


def detect_videos(videos: Sequence[str], expected: str, *,
                  logger: Optional[logging.Logger] = None
                  ) -> Tuple[List[str], Dict[Optional[str], int]]:
    """Detect every candidate and keep only videos from *expected*.

    Videos that belong to the expected camera are reported at INFO, others are
    skipped with a WARNING.  Returns ``(accepted, counts)`` where ``counts``
    maps every detected camera id (``None`` for unrecognised files) to its file
    count, and a summary of those counts is logged once at the end.
    """
    log = logger or LOG
    counts: Dict[Optional[str], int] = {camera: 0 for camera in CAMERA_LABELS}
    counts[None] = 0
    accepted: List[str] = []
    for video in videos:
        detection = detect_camera(video)
        counts[detection.camera] = counts.get(detection.camera, 0) + 1
        reason = skip_reason(detection, expected)
        if reason:
            log.warning("skipping %s: %s", os.path.basename(video), reason)
            continue
        log.info("camera %s: %s", detection.label, os.path.basename(video))
        log.debug("  evidence: %s",
                  "; ".join(detection.evidence.get(expected, [])) or "none")
        accepted.append(video)
    log.info("camera detection summary: %s", summarize_detections(counts))
    return accepted, counts


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
    """Return the video files under *input_path* (file or directory)."""
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
    """Register the arguments every exporter shares."""
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
    parser.add_argument("--max-position-error", type=float, default=None,
                        metavar="METERS",
                        help="maximum position error when matching a frame to a "
                             "target point (default: distance/2)")
    parser.add_argument("--max-images", type=int, default=None, metavar="N",
                        help="maximum number of images to export per video")
    parser.add_argument("--extensions", default=default_extensions,
                        metavar="LIST",
                        help="comma-separated video file extensions to scan")
    parser.add_argument("--heading-offset", dest="heading_offset", type=float,
                        default=0.0, metavar="DEGREES",
                        help="degrees added to the heading to derive the image "
                             "direction (-180..180)")
    parser.add_argument("--offset", dest="heading_offset", type=float,
                        help=argparse.SUPPRESS)          # legacy VIRB alias
    parser.add_argument("--magnetic-declination", type=float, default=0.0,
                        metavar="DEGREES",
                        help="east-positive magnetic declination added to "
                             "compass/IMU headings to obtain true north")
    parser.add_argument("--uere", type=float, default=5.0, metavar="METERS",
                        help="user-equivalent range error used as "
                             "accuracy_m = HDOP x UERE when no measured "
                             "accuracy exists (default 5.0)")
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
    parser.add_argument("--keywords", default=None, metavar="LIST",
                        help="opt-in keywords to write to Keywords/XMP:Subject, "
                             "semicolon- or comma-separated")
    parser.add_argument("--track-export", choices=("both", "gpx", "geojson", "none"),
                        default="both",
                        help="write a GPX/GeoJSON track per sequence (default both)")
    parser.add_argument("--save-telemetry", dest="track_export",
                        action="store_const", const="both",
                        help="legacy alias for --track-export both")
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
    """GPS-noise options shared by the GPMF (MAX/HERO) and NMEA readers."""
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
    """Track options shared by all telemetry-backed exporters."""
    parser.add_argument("--distance-mode", choices=("2d", "3d"), default="2d",
                        help="measure distance travelled on the ground (default) "
                             "or in 3D")
    parser.add_argument("--sidecars", action="store_true",
                        help="legacy: also write a <sequence>.track.jsonl manifest")
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
    """Parse a timezone offset into seconds, or ``None`` when not given."""
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


def parse_keywords(text: Optional[str]) -> Optional[List[str]]:
    """Split an opt-in ``--keywords`` value into a clean list."""
    if text is None:
        return None
    parts = [p.strip() for p in re.split(r"[;,]", text) if p.strip()]
    # De-duplicate while preserving order.
    seen = set()
    out = []
    for part in parts:
        key = part.lower()
        if key not in seen:
            seen.add(key)
            out.append(part)
    return out or None


def validate_common_args(parser: argparse.ArgumentParser,
                         args: argparse.Namespace) -> None:
    """Validate the shared arguments, calling ``parser.error`` on failure."""
    if args.distance <= 0:
        parser.error("--distance must be greater than zero (got %g)" % args.distance)
    if args.start_distance < 0:
        parser.error("--start-distance must not be negative (got %g)"
                     % args.start_distance)
    if args.max_position_error is not None and args.max_position_error <= 0:
        parser.error("--max-position-error must be greater than zero")
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
    if not (-180.0 <= args.magnetic_declination <= 180.0):
        parser.error("--magnetic-declination must be between -180 and +180 "
                     "degrees (got %g)" % args.magnetic_declination)
    if args.uere <= 0:
        parser.error("--uere must be greater than zero (got %g)" % args.uere)
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
LOG = logging.getLogger("v2i")

# ANSI colours per level: INFO green, WARNING yellow, ERROR red.
_LEVEL_COLORS = {
    "DEBUG": "\033[90m",         # bright black / grey
    "INFO": "\033[32m",          # green
    "WARNING": "\033[33m",       # yellow
    "ERROR": "\033[31m",         # red
    "CRITICAL": "\033[1;31m",    # bold red
}
_COLOR_RESET = "\033[0m"


def color_enabled(stream=None) -> bool:
    """Whether ANSI colours should be used for *stream* (stderr by default).

    Honours ``NO_COLOR`` (disable) and ``FORCE_COLOR``/``CLICOLOR_FORCE``
    (enable), otherwise colours are used only on a terminal.
    """
    if os.environ.get("NO_COLOR"):
        return False
    if (os.environ.get("FORCE_COLOR")
            or os.environ.get("CLICOLOR_FORCE")):
        return True
    stream = stream if stream is not None else sys.stderr
    try:
        return bool(stream.isatty())
    except (AttributeError, ValueError):
        return False


class ColorFormatter(logging.Formatter):
    """Prefix each line with its level, coloured by severity."""

    def __init__(self, fmt: str = "[%(levelname)s] %(message)s",
                 color: bool = True):
        super().__init__(fmt)
        self.color = color

    def format(self, record: logging.LogRecord) -> str:
        text = super().format(record)
        if not self.color:
            return text
        code = _LEVEL_COLORS.get(record.levelname)
        if not code:
            return text
        # Colour each line so multi-line messages stay readable.
        return "\n".join(code + line + _COLOR_RESET
                         for line in text.split("\n"))


def setup_logging(level: str = "INFO", *, verbose: bool = False,
                  quiet: bool = False) -> None:
    """Configure root logging for the scripts."""
    if verbose:
        level = "DEBUG"
    elif quiet:
        level = "ERROR"
    root = logging.getLogger()
    for handler in list(root.handlers):
        root.removeHandler(handler)
    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(ColorFormatter(color=color_enabled(handler.stream)))
    root.addHandler(handler)
    root.setLevel(level)


def log_level(args: argparse.Namespace) -> str:
    if getattr(args, "verbose", False):
        return "DEBUG"
    if getattr(args, "quiet", False):
        return "ERROR"
    return getattr(args, "log_level", "INFO")


# ---------------------------------------------------------------------------
# Time helpers
# ---------------------------------------------------------------------------
def exif_datetime(value: dt.datetime) -> str:
    return value.strftime("%Y:%m:%d %H:%M:%S")


def offset_string(seconds: int) -> str:
    sign = "+" if seconds >= 0 else "-"
    seconds = abs(int(seconds))
    return "%s%02d:%02d" % (sign, seconds // 3600, (seconds % 3600) // 60)


def parse_utc_timestamp(text: Optional[str]) -> Optional[dt.datetime]:
    """Parse an ISO-8601 UTC container timestamp into an aware datetime."""
    if not text:
        return None
    value = text.strip()
    if value.endswith("Z"):
        value = value[:-1] + "+00:00"
    try:
        parsed = dt.datetime.fromisoformat(value)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        return parsed.replace(tzinfo=dt.timezone.utc)
    return parsed.astimezone(dt.timezone.utc)


def fmt_utc(value: Optional[dt.datetime]) -> Optional[str]:
    if value is None:
        return None
    return (value.astimezone(dt.timezone.utc)
            .strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z")


def local_from_utc(utc: Optional[dt.datetime],
                   offset_s: Optional[int]) -> Optional[dt.datetime]:
    """Naive camera-local wall clock, never the host timezone."""
    if utc is None:
        return None
    if utc.tzinfo is None:
        utc = utc.replace(tzinfo=dt.timezone.utc)
    return (utc.astimezone(dt.timezone.utc)
            + dt.timedelta(seconds=offset_s or 0)).replace(tzinfo=None)


def resolve_timezone(args: argparse.Namespace, detected: Optional[int], *,
                     camera: str, logger: logging.Logger) -> Tuple[int, bool]:
    """Return ``(offset_seconds, known)`` for naming and EXIF.

    Rules: explicit ``--timezone`` wins; ``--time-base utc`` uses UTC; otherwise
    the camera-detected offset is used.  MAX/HERO carry no local wall clock, so
    without an explicit offset they fall back to UTC (never the host timezone)
    and say so in the log.
    """
    explicit = parse_timezone(getattr(args, "timezone", None))
    if explicit is not None:
        return explicit, True
    if args.time_base == "utc":
        return 0, True
    if detected is not None:
        return detected, True
    if camera in (CAMERA_MAX, CAMERA_HERO):
        logger.warning("no camera-local timezone in this video; using UTC for "
                       "file names and EXIF (pass --timezone to override)")
    return 0, False


# ---------------------------------------------------------------------------
# Geodesy / track interpolation
# ---------------------------------------------------------------------------
EARTH_RADIUS_M = 6371008.8


def haversine(lat1, lon1, lat2, lon2):
    radius = EARTH_RADIUS_M
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * radius * math.asin(min(1.0, math.sqrt(a)))


def initial_bearing_deg(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _segment_distance(a, b, mode):
    flat = haversine(a["lat"], a["lon"], b["lat"], b["lon"])
    if mode == "3d":
        dz = (b.get("alt") or 0.0) - (a.get("alt") or 0.0)
        return math.hypot(flat, dz)
    return flat


def _interp_angle(va, vb, fraction):
    delta = (vb - va + 540.0) % 360.0 - 180.0
    return (va + fraction * delta) % 360.0


def _interp_value(a, b, fraction, key):
    va, vb = a.get(key), b.get(key)
    if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
        if key in ("heading", "course", "gps_course", "metadata_heading",
                   "compass_heading", "img_direction"):
            return _interp_angle(va, vb, fraction)
        return va + fraction * (vb - va)
    if isinstance(va, (list, tuple)) and isinstance(vb, (list, tuple)):
        return [va[i] + fraction * (vb[i] - va[i])
                for i in range(min(len(va), len(vb)))]
    return va if va is not None else vb


class Track(object):
    """A time-indexed GPS track with interpolated access.

    Samples are dicts carrying at least ``t`` (video-relative seconds), ``lat``
    and ``lon``.  Optional keys interpolated by :meth:`at_time` include ``alt``,
    ``speed``, ``course``, ``gps_course``, ``compass_heading``,
    ``metadata_heading``, ``accuracy``, ``hdop``, ``vdop``, ``pdop`` and vector
    fields such as ``accel``/``gyro``/``gravity``/``magnetic``.
    """

    def __init__(self, samples, mode="2d"):
        clean = [dict(s) for s in samples
                 if s.get("lat") is not None and s.get("lon") is not None]
        clean.sort(key=lambda s: s["t"])
        deduped = []
        for sample in clean:
            if deduped and abs(sample["t"] - deduped[-1]["t"]) < 1e-9:
                continue
            deduped.append(sample)
        self.samples = deduped
        self.mode = mode
        self.times = [s["t"] for s in self.samples]
        self.cum = [0.0]
        for a, b in zip(self.samples, self.samples[1:]):
            self.cum.append(self.cum[-1] + _segment_distance(a, b, mode))
        self.length = self.cum[-1] if self.samples else 0.0
        self._add_gps_courses()

    def _add_gps_courses(self):
        """Fill a per-sample course-over-ground from neighbouring positions.

        At 10-30 Hz a single GPS step is often only centimetres long, which
        makes a two-point bearing pure noise.  Search outward up to ~1 s until
        the two points are far enough apart to give a stable course.
        """
        n = len(self.samples)
        for i, sample in enumerate(self.samples):
            if sample.get("gps_course") is not None:
                continue
            # A parked vehicle has no meaningful course over ground.
            speed = sample.get("speed")
            if speed is not None and speed < 0.3:
                continue
            for radius in (1, 2, 4, 8, 16, 30):
                lo = max(0, i - radius)
                hi = min(n - 1, i + radius)
                if hi == lo:
                    continue
                a, b = self.samples[lo], self.samples[hi]
                if haversine(a["lat"], a["lon"], b["lat"], b["lon"]) >= 0.15:
                    sample["gps_course"] = initial_bearing_deg(
                        a["lat"], a["lon"], b["lat"], b["lon"])
                    break

    def _locate_time(self, t):
        times = self.times
        if not times:
            return 0, 0.0, "clamped-start"
        if t <= times[0]:
            return 0, 0.0, "clamped-start"
        if t >= times[-1]:
            return len(times) - 2, 1.0, "clamped-end"
        lo = bisect.bisect_right(times, t) - 1
        lo = max(0, min(lo, len(times) - 2))
        span = times[lo + 1] - times[lo]
        fraction = 0.0 if span <= 0 else (t - times[lo]) / span
        return lo, min(max(fraction, 0.0), 1.0), "interpolated"

    def _locate_distance(self, distance):
        samples = self.samples
        if not samples:
            return 0, 0.0
        if len(samples) == 1:
            return 0, 0.0
        if distance <= 0.0:
            return 0, 0.0
        if distance >= self.cum[-1]:
            return len(samples) - 2, 1.0
        lo, hi = 0, len(samples) - 1
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
        keys = set(a) | set(b)
        for key in keys:
            if key in ("t", "lat", "lon"):
                continue
            if key in ("utc",):
                out[key] = a.get(key) if a.get(key) is not None else b.get(key)
                continue
            out[key] = _interp_value(a, b, fraction, key)
        out["lat"] = a["lat"] + fraction * (b["lat"] - a["lat"])
        out["lon"] = a["lon"] + fraction * (b["lon"] - a["lon"])
        return out

    def at_time(self, t):
        """Interpolate at video-relative *t*, clamping outside telemetry."""
        if not self.samples:
            return None
        if len(self.samples) == 1:
            point = dict(self.samples[0])
            point["position_source"] = ("clamped-start" if t < self.times[0]
                                        else "clamped-end")
            point["estimated"] = True
            point["clamped"] = True
            point["distance"] = 0.0
            return point
        lo, fraction, source = self._locate_time(t)
        if source == "clamped-start":
            point = dict(self.samples[0])
            point["t"] = t
        elif source == "clamped-end":
            point = dict(self.samples[-1])
            point["t"] = t
        else:
            point = self._interp(self.samples[lo], self.samples[lo + 1],
                                 fraction)
        point["distance"] = self.distance_at_time(t)
        point["position_source"] = source
        point["clamped"] = source.startswith("clamped")
        point["estimated"] = True
        return point

    def at_distance(self, distance):
        """Interpolate at a cumulative track distance."""
        if not self.samples:
            return None
        if len(self.samples) == 1:
            point = dict(self.samples[0])
            point["distance"] = distance
            point["position_source"] = "sample"
            point["clamped"] = False
            point["estimated"] = False
            return point
        lo, fraction = self._locate_distance(distance)
        point = self._interp(self.samples[lo], self.samples[lo + 1], fraction)
        point["distance"] = distance
        point["position_source"] = (
            "clamped-start" if distance <= 0 else
            "clamped-end" if distance >= self.cum[-1] else "interpolated")
        point["clamped"] = point["position_source"].startswith("clamped")
        point["estimated"] = True
        return point

    def distance_at_time(self, t):
        if not self.samples:
            return 0.0
        if t <= self.times[0]:
            return self.cum[0]
        if t >= self.times[-1]:
            return self.cum[-1]
        lo, fraction, _source = self._locate_time(t)
        return self.cum[lo] + fraction * (self.cum[lo + 1] - self.cum[lo])

    def time_at_distance(self, distance):
        point = self.at_distance(distance)
        return None if point is None else point["t"]

    def sample_window(self, t0, t1):
        return [s for s in self.samples if t0 <= s["t"] <= t1]


def plan_shots(track: Track, interval: float, *, start_distance: float = 0.0,
               max_shots: Optional[int] = None) -> List[dict]:
    """Target points every *interval* metres along *track*."""
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
# Heading and accuracy
# ---------------------------------------------------------------------------
def resolve_heading(point: dict, heading_offset: float = 0.0,
                    magnetic_declination: float = 0.0) -> Tuple[Optional[float], str]:
    """Return ``(true-north heading, heading_source)`` for an interpolated point."""
    declination = magnetic_declination or 0.0
    compass = point.get("compass_heading")
    if compass is not None:
        return (compass + declination + heading_offset) % 360.0, "compass"
    metadata = point.get("metadata_heading")
    if metadata is not None:
        return (metadata + heading_offset) % 360.0, "metadata"
    course = point.get("heading")
    if course is None:
        course = point.get("course")
    if course is not None:
        return (course + heading_offset) % 360.0, "gps_course"
    gps_course = point.get("gps_course")
    if gps_course is not None:
        return (gps_course + heading_offset) % 360.0, "gps_course"
    return None, "none"


def derive_accuracy(point: dict, uere: float = 5.0):
    """Return ``(accuracy_m, estimated, source)``.

    A measured accuracy (FIT/GPMF/explicit) wins.  Otherwise HDOP (or the first
    DOP value) is multiplied by the user-estimated UERE and marked estimated.
    """
    value = point.get("accuracy")
    if value is not None:
        return float(value), bool(point.get("accuracy_estimated", False)), \
            point.get("accuracy_source") or "measured"
    hdop = point.get("hdop")
    if hdop is None:
        hdop = point.get("dop")
    if hdop is not None:
        return float(hdop) * float(uere), True, "hdop*uere"
    return None, False, None


def apply_accuracy(sample: dict, uere: float = 5.0) -> dict:
    """Populate ``accuracy``/``accuracy_source`` on a sample in place."""
    accuracy, estimated, source = derive_accuracy(sample, uere)
    sample["accuracy"] = accuracy
    sample["accuracy_estimated"] = estimated
    sample["accuracy_source"] = source
    return sample


# ---------------------------------------------------------------------------
# Frame timeline
# ---------------------------------------------------------------------------
@dataclass
class FrameTimeline:
    """Frame PTS values in decode order (the frame index is the list index)."""

    times: List[float]
    source: str = "ffprobe"      # or "fps-fallback"
    fps: float = 0.0

    def __len__(self):
        return len(self.times)

    def nearest_index(self, t: float) -> int:
        if not self.times:
            return 0
        pos = bisect.bisect_left(self.times, t)
        if pos <= 0:
            return 0
        if pos >= len(self.times):
            return len(self.times) - 1
        before = self.times[pos - 1]
        after = self.times[pos]
        return pos - 1 if (t - before) <= (after - t) else pos

    def pts(self, index: int) -> float:
        return self.times[max(0, min(index, len(self.times) - 1))]


def _parse_rate(rate) -> float:
    if not rate:
        return 0.0
    text = str(rate)
    if "/" in text:
        num, den = text.split("/", 1)
        try:
            den_f = float(den)
            return float(num) / den_f if den_f else 0.0
        except ValueError:
            return 0.0
    try:
        return float(text)
    except ValueError:
        return 0.0


@lru_cache(maxsize=64)
def _probe_stream_info_cached(path: str, mtime: float, ffprobe: str,
                              stream_ordinal: int) -> dict:
    return _read_stream_info(path, ffprobe, stream_ordinal)


def _read_stream_info(path: str, ffprobe: str, stream_ordinal: int) -> dict:
    rc, out, err = _run([ffprobe, "-v", "error", "-print_format", "json",
                         "-show_streams", "-select_streams", "v:%d"
                         % stream_ordinal, path])
    if rc != 0:
        return {}
    try:
        data = json.loads(out.decode("utf-8", "replace"))
    except ValueError:
        return {}
    streams = data.get("streams", [])
    return streams[0] if streams else {}


def probe_stream_info(path: str, *, ffprobe: str = FFPROBE,
                      stream_ordinal: int = 0) -> dict:
    """Return basic information for one video stream (ordinal, not index)."""
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        return _read_stream_info(path, ffprobe, stream_ordinal)
    return _probe_stream_info_cached(path, mtime, ffprobe, stream_ordinal)


def _read_packet_times(path: str, ffprobe: str,
                       stream_ordinal: int) -> Tuple[Optional[float], ...]:
    """Raw packet PTS values in decode order (empty when unreadable)."""
    rc, out, err = _run([ffprobe, "-v", "error", "-select_streams",
                         "v:%d" % stream_ordinal, "-show_packets",
                         "-show_entries", "packet=pts_time,dts_time",
                         "-of", "json", path])
    if rc != 0:
        return ()
    try:
        data = json.loads(out.decode("utf-8", "replace"))
    except ValueError:
        return ()
    values: List[Optional[float]] = []
    for packet in data.get("packets", []):
        value = packet.get("pts_time")
        if value in (None, "N/A"):
            value = packet.get("dts_time")
        try:
            values.append(None if value in (None, "N/A") else float(value))
        except (TypeError, ValueError):
            values.append(None)
    # Packets are stored in decode order; a decoder emits frames in presentation
    # order, which is also the order of ffmpeg's select frame index.
    if values and all(value is not None for value in values):
        values = sorted(values)
    return tuple(values)


@lru_cache(maxsize=32)
def _probe_packet_times_cached(path: str, mtime: float, ffprobe: str,
                               stream_ordinal: int
                               ) -> Tuple[Optional[float], ...]:
    return _read_packet_times(path, ffprobe, stream_ordinal)


def _probe_packet_times(path: str, ffprobe: str,
                        stream_ordinal: int) -> Tuple[Optional[float], ...]:
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        return _read_packet_times(path, ffprobe, stream_ordinal)
    return _probe_packet_times_cached(path, mtime, ffprobe, stream_ordinal)


def _build_timeline_from_raw(raw_times, path, ffprobe, stream_ordinal,
                             fps_hint, frame_count_hint, duration_hint):
    fps = fps_hint or 0.0
    if fps <= 0:
        info = probe_stream_info(path, ffprobe=ffprobe,
                                 stream_ordinal=stream_ordinal)
        fps = _parse_rate(info.get("avg_frame_rate")
                          or info.get("r_frame_rate")) or 30.0
    nominal = 1.0 / fps if fps > 0 else 1.0 / 30.0

    known = [i for i, value in enumerate(raw_times) if value is not None]
    if known:
        times: List[float] = [0.0] * len(raw_times)
        first = known[0]
        origin = float(raw_times[first])
        for i in range(first):
            times[i] = raw_times[first] - (first - i) * nominal - origin
        for i in known:
            times[i] = float(raw_times[i]) - origin
        for i in range(first + 1, len(raw_times)):
            if raw_times[i] is None:
                times[i] = times[i - 1] + nominal
        # Keep PTS monotonic even if the container reports a stray value.
        for i in range(1, len(times)):
            if times[i] <= times[i - 1]:
                times[i] = times[i - 1] + nominal
        return FrameTimeline(times, "ffprobe", fps)

    count = frame_count_hint
    duration = duration_hint
    if not count and duration:
        count = max(1, int(round(duration * fps)))
    if not count:
        count = 1
    return FrameTimeline([i * nominal for i in range(count)],
                         "fps-fallback", fps)


def probe_frame_timeline(path: str, *, stream_ordinal: int = 0,
                         fps_hint: float = 0.0, frame_count_hint: int = 0,
                         duration_hint: Optional[float] = None,
                         ffprobe: str = FFPROBE,
                         logger: Optional[logging.Logger] = None) -> FrameTimeline:
    """Read actual frame PTS values with ffprobe, falling back to a fixed fps.

    VFR and low-frame-rate videos are handled naturally because every frame
    keeps its measured PTS.  If ffprobe cannot report frames, a uniform grid is
    synthesised from the frame count/duration/fps and a warning is logged.
    """
    log = logger or LOG
    raw_times = _probe_packet_times(path, ffprobe, stream_ordinal)
    timeline = _build_timeline_from_raw(raw_times, path, ffprobe,
                                        stream_ordinal, fps_hint,
                                        frame_count_hint, duration_hint)
    if timeline.source == "fps-fallback":
        log.warning("could not read frame PTS with ffprobe; falling back to a "
                    "uniform %.3f fps grid (%d frames)", timeline.fps,
                    len(timeline))
    return timeline


def select_time_grid_frames(timeline: FrameTimeline, interval: float, *,
                            duration: Optional[float] = None,
                            max_images: Optional[int] = None) -> List[int]:
    """Choose unique, strictly increasing frames on a time grid (no telemetry)."""
    if interval <= 0 or not len(timeline):
        return []
    end = duration if duration is not None else timeline.times[-1]
    targets = []
    t = 0.0
    while t <= end + 1e-9:
        targets.append(t)
        t += interval
    selected: List[int] = []
    last = -1
    used = set()
    for target in targets:
        index = timeline.nearest_index(target)
        if index <= last:
            index = last + 1
        if index >= len(timeline) or index in used:
            continue
        selected.append(index)
        used.add(index)
        last = index
        if max_images is not None and len(selected) >= max_images:
            break
    return selected


# ---------------------------------------------------------------------------
# Unique-frame selection
# ---------------------------------------------------------------------------
@dataclass
class FrameChoice:
    target_distance: float
    target_time: float
    frame_index: int
    pts: float
    point: dict
    position_error: float
    clamped: bool
    estimated: bool


def select_unique_frames(track: Track, timeline: FrameTimeline, interval: float,
                         *, start_distance: float = 0.0,
                         max_position_error: Optional[float] = None,
                         max_images: Optional[int] = None,
                         mode: str = "2d",
                         logger: Optional[logging.Logger] = None,
                         require_first_frame: bool = True
                         ) -> Tuple[List[FrameChoice], List[dict]]:
    """Select one *unique* frame per target distance.

    For every target the nearest unused frame (in video time) whose position is
    within ``max_position_error`` is chosen.  Frames are never reused and the
    selected indices strictly increase, so a low frame rate produces gaps
    rather than duplicate frames.  Targets with no acceptable frame are
    skipped and returned in the second list.

    With ``start_distance == 0`` the first frame (index 0) is always exported,
    even if telemetry starts slightly later (the position is clamped).
    """
    log = logger or LOG
    if max_position_error is None:
        max_position_error = interval / 2.0
    shots = plan_shots(track, interval, start_distance=start_distance,
                       max_shots=max_images)
    choices: List[FrameChoice] = []
    skipped: List[dict] = []
    last = -1
    n = len(timeline)

    for shot_index, shot in enumerate(shots):
        target_distance = float(shot.get("distance", 0.0))
        target_time = shot.get("t")
        if target_time is None:
            target_time = track.time_at_distance(target_distance)
        if target_time is None:
            skipped.append({"distance": target_distance,
                            "reason": "no telemetry time"})
            continue

        # The first target at distance 0 must use frame 0, not the nearest
        # frame after a delayed GPS fix.
        if (require_first_frame and start_distance == 0.0 and shot_index == 0
                and n > 0):
            index = 0
            point = track.at_time(timeline.pts(index))
            error = _position_error(shot, point, mode)
            choices.append(FrameChoice(target_distance, target_time, index,
                                       timeline.pts(index), point, error,
                                       bool(point and point.get("clamped")),
                                       True))
            last = index
            continue

        candidates = _candidates_by_time(timeline, target_time, last)
        found = None
        for index in candidates:
            point = track.at_time(timeline.pts(index))
            if point is None or point.get("lat") is None:
                continue
            error = _position_error(shot, point, mode)
            if error is not None and error <= max_position_error + 1e-6:
                found = (index, point, error)
                break
        if found is None:
            skipped.append({"distance": target_distance,
                            "time": target_time,
                            "reason": "no unused frame within %.2f m"
                                      % max_position_error})
            continue
        index, point, error = found
        choices.append(FrameChoice(target_distance, target_time, index,
                                   timeline.pts(index), point, error,
                                   bool(point.get("clamped")), True))
        last = index
        if max_images is not None and len(choices) >= max_images:
            break

    if (require_first_frame and start_distance == 0.0 and n > 0
            and (not choices or choices[0].frame_index != 0)):
        raise SelectionError("could not select the first frame (index 0) for "
                             "--start-distance 0")
    return choices, skipped


def _candidates_by_time(timeline: FrameTimeline, target: float,
                        last: int) -> Iterable[int]:
    times = timeline.times
    n = len(times)
    if n == 0:
        return []
    pos = bisect.bisect_left(times, target)
    left = min(pos - 1, n - 1)
    if left <= last:
        left = -1
    right = max(pos, last + 1)
    while left >= 0 or right < n:
        if left < 0:
            yield right
            right += 1
        elif right >= n:
            yield left
            left -= 1
        elif (target - times[left]) <= (times[right] - target):
            yield left
            left -= 1
        else:
            yield right
            right += 1


def _position_error(target: dict, point: Optional[dict], mode: str
                    ) -> Optional[float]:
    if point is None or point.get("lat") is None or target.get("lat") is None:
        return None
    flat = haversine(target["lat"], target["lon"],
                     point["lat"], point["lon"])
    if mode == "3d":
        ta, pa = target.get("alt"), point.get("alt")
        if ta is not None and pa is not None:
            return math.hypot(flat, pa - ta)
    return flat


# ---------------------------------------------------------------------------
# Frame extraction (exact frame-index select in one decode pass)
# ---------------------------------------------------------------------------
def ffmpeg_quality_args(image_format: str, quality: int) -> List[str]:
    """Map a 1..100 quality to ffmpeg encoder options (higher = better)."""
    quality = max(1, min(100, quality))
    if image_format == "png":
        level = int(round((100 - quality) / 100.0 * 9))
        return ["-compression_level", str(max(0, min(9, level)))]
    qscale = int(round(31 - (quality / 100.0) * 30))
    return ["-q:v", str(max(1, min(31, qscale)))]


def select_expression(frames: Sequence[int]) -> str:
    """Build an ffmpeg ``select`` expression matching the given frame numbers."""
    return "+".join("eq(n\\,%d)" % int(n) for n in frames)


# ffmpeg parses ``select`` expressions with a rather small recursion limit:
# more than ~100 ``eq(n,...)`` terms fail with "Error while parsing expression"
# (and on memory-constrained machines even earlier).  Batches are therefore
# kept comfortably below that limit.
EXTRACT_CHUNK_SIZE = 60


def _extract_batch(video: str, frames: Sequence[int], pattern: str, *,
                   image_format: str, quality: int, threads: int,
                   stream_ordinal: int, seek_time: Optional[float]) -> bool:
    """Run one ffmpeg ``select`` batch; return whether every image appeared."""
    selected = frames if seek_time is None else \
        [int(frame) - int(frames[0]) for frame in frames]
    expr = select_expression(selected)
    cmd = [FFMPEG, "-hide_banner", "-loglevel", "error", "-y"]
    if seek_time is not None and seek_time > 0:
        cmd += ["-ss", "%.6f" % seek_time]
    cmd += ["-i", video]
    if threads:
        cmd += ["-threads", str(threads)]
    cmd += ["-map", "0:v:%d" % stream_ordinal,
            "-vf", "select='%s'" % expr, "-vsync", "0"]
    cmd += ffmpeg_quality_args(image_format, quality)
    cmd.append(pattern)
    try:
        proc = subprocess.run(cmd, stdout=subprocess.DEVNULL,
                              stderr=subprocess.PIPE, timeout=1800)
    except subprocess.TimeoutExpired:
        LOG.warning("batch frame extraction timed out")
        return False
    if proc.returncode != 0:
        err = proc.stderr.decode("utf-8", "replace").strip()
        tail = err.splitlines()[-1:] if err else []
        LOG.warning("batch frame extraction failed: %s", tail[0] if tail else "")
        return False
    return all(os.path.exists(pattern % order)
               for order in range(1, len(frames) + 1))


def extract_frames_by_index(video: str, frames: Sequence[int], tmp_dir: str, *,
                            image_format: str, quality: int, workers: int = 1,
                            threads: int = 0, stream_ordinal: int = 0,
                            timeline: Optional[FrameTimeline] = None,
                            chunk_size: int = EXTRACT_CHUNK_SIZE
                            ) -> Dict[int, str]:
    """Extract exact frames by index in bounded ffmpeg batches."""
    unique = sorted({int(f) for f in frames if int(f) >= 0})
    if not unique:
        return {}
    os.makedirs(tmp_dir, exist_ok=True)

    extracted: Dict[int, str] = {}
    # Try to get fps from timeline if available
    fps = timeline.fps if timeline is not None and timeline.fps > 0 else 23.976
    if timeline is None or timeline.source != "ffprobe" or len(timeline) == 0:
        # try to guess
        try:
            info = probe_stream_info(video)
            fps = _parse_rate(info.get("avg_frame_rate") or info.get("r_frame_rate")) or fps
        except Exception:
            pass

    # For each frame, extract individually with seek for reliability
    for idx, fidx in enumerate(unique):
        pat = os.path.join(tmp_dir, "f_%06d_%%06d.%s" % (idx, image_format))
        t = fidx / fps if fps > 0 else 0
        cmd = [FFMPEG, "-hide_banner", "-loglevel", "error", "-y",
               "-ss", "%.6f" % max(0, t),
               "-threads", str(threads) if threads else "1",
               "-i", video,
               "-map", "0:v:%d" % stream_ordinal,
               "-frames:v", "1"]
        cmd += ffmpeg_quality_args(image_format, quality)
        cmd.append(pat)
        try:
            proc = subprocess.run(cmd, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.PIPE, timeout=300)
            if proc.returncode == 0 and os.path.exists(pat % 1):
                extracted[int(fidx)] = pat % 1
                continue
        except Exception:
            pass
        LOG.error("could not extract frames %d-%d from %s",
                  fidx, fidx, os.path.basename(video))
        continue
    return extracted


def extract_frames(video: str, jobs: Sequence[dict], tmp_dir: str, *,
                   image_format: str, quality: int, workers: int, fps: float,
                   threads: int = 0) -> Dict[int, str]:
    """Compatibility wrapper: jobs carry ``index`` and ``frame``/``pts``."""
    frames = []
    mapping: Dict[int, int] = {}
    for job in jobs:
        frame = job.get("frame")
        if frame is None and job.get("pts") is not None and fps:
            frame = int(round(float(job["pts"]) * fps))
        if frame is None:
            continue
        frames.append(int(frame))
        mapping[job["index"]] = int(frame)
    extracted = extract_frames_by_index(
        video, frames, tmp_dir, image_format=image_format, quality=quality,
        workers=workers, threads=threads)
    out: Dict[int, str] = {}
    for job in jobs:
        frame = mapping.get(job["index"])
        if frame is not None and frame in extracted:
            out[job["index"]] = extracted[frame]
    return out


# ---------------------------------------------------------------------------
# Metadata helpers / exiftool batching
# ---------------------------------------------------------------------------
def write_metadata(pairs: Sequence[Tuple[str, Sequence[str]]], *,
                   batch: int = 40) -> None:
    """Write metadata for ``[(path, exiftool_args), ...]``."""
    if not pairs:
        return
    for start in range(0, len(pairs), batch):
        chunk = pairs[start:start + batch]
        cmd = ["exiftool", "-q", "-q", "-P"]
        for path, tag_args in chunk:
            cmd.append("-overwrite_original")
            cmd += list(tag_args)
            cmd.append(path)
            cmd.append("-execute")
        cmd.append("-execute0")
        try:
            proc = subprocess.run(cmd, stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, timeout=300)
            if proc.returncode != 0:
                LOG.error("exiftool failed: %s",
                          proc.stderr.decode("utf-8", "replace").strip())
        except subprocess.TimeoutExpired:
            LOG.error("exiftool timed out")


# ---------------------------------------------------------------------------
# Sequence naming / output paths
# ---------------------------------------------------------------------------
def sanitize(text: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9._-]+", "-", text)
    return clean.strip("-") or "video"


def build_sequence_ids(videos: Sequence[str], input_dir: str) -> Dict[str, str]:
    """Map each video to a collision-free sequence directory name.

    The plain source stem is used unless two videos share it; then the nested
    relative directory is prepended (``subdir__VIRB0001``).  No numeric
    fallback counter is used: duplicate stems are made unique by their path.
    """
    import collections

    stems: Dict[str, Tuple[str, str]] = {}
    for video in videos:
        stem = os.path.splitext(os.path.basename(video))[0]
        rel_dir = os.path.relpath(os.path.dirname(os.path.abspath(video)),
                                  input_dir)
        stems[video] = (stem, rel_dir)

    counts = collections.Counter(stem.lower() for stem, _ in stems.values())
    result: Dict[str, str] = {}
    used: Dict[str, str] = {}
    for video in videos:
        stem, rel_dir = stems[video]
        nested = rel_dir not in (".", "")
        if counts[stem.lower()] > 1 and nested:
            prefix = sanitize(rel_dir.replace(os.sep, "__"))
            name = prefix + "__" + stem
        else:
            name = stem
        name = sanitize(name)
        key = name.lower()
        if key in used and used[key] != video:
            # Last-resort deterministic disambiguation using the full path.
            prefix = sanitize(os.path.abspath(video).replace(os.sep, "__"))
            name = sanitize(stem + "__" + prefix)
            key = name.lower()
            if key in used and used[key] != video:
                raise ValueError("cannot build a unique sequence name for %s"
                                 % video)
        used[key] = video
        result[video] = name
    return result


def image_filename(value: dt.datetime, image_format: str = "jpg") -> str:
    """``YYYY-MM-DD_HH-MM-SS-mmm.ext`` -- no per-sequence counter."""
    return (value.strftime("%Y-%m-%d_%H-%M-%S")
            + "-%03d.%s" % (value.microsecond // 1000, image_format))


def date_folder_name(value: dt.datetime, camera: str) -> str:
    """``YYYY-MM-DD-<camera-suffix>`` for the first exported frame."""
    return "%s-%s" % (value.strftime("%Y-%m-%d"), CAMERA_SUFFIX[camera])


def sequence_output_dir(output_dir: str, camera: str, sequence: str,
                        first_capture: dt.datetime) -> str:
    """``<output>/<date>-<camera>/<sequence>``."""
    return os.path.join(output_dir, date_folder_name(first_capture, camera),
                        sequence)


def unique_path(path: str, used: set) -> str:
    """Avoid two planned images with the same millisecond (no ``_NNNNNN``)."""
    if path not in used and not os.path.exists(path):
        used.add(path)
        return path
    base, ext = os.path.splitext(path)
    suffix = 2
    candidate = "%s-%d%s" % (base, suffix, ext)
    while candidate in used or os.path.exists(candidate):
        suffix += 1
        candidate = "%s-%d%s" % (base, suffix, ext)
    used.add(candidate)
    return candidate


# ---------------------------------------------------------------------------
# GPX / GeoJSON track writers
# ---------------------------------------------------------------------------
def _xml_escape(text: str) -> str:
    return (str(text).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;"))


def write_gpx(path: str, samples: Sequence[dict], name: str = "track",
              creator: str = "video2images") -> None:
    """Write a valid GPX 1.1 document with UTC timestamps."""
    lines = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<gpx version="1.1" creator="%s"' % _xml_escape(creator),
             '     xmlns="http://www.topografix.com/GPX/1/1"',
             '     xmlns:v2i="https://video2images.local/ns/1">',
             '  <metadata><name>%s</name></metadata>' % _xml_escape(name),
             '  <trk><name>%s</name><trkseg>' % _xml_escape(name)]
    for sample in samples:
        lat, lon = sample.get("lat"), sample.get("lon")
        if lat is None or lon is None:
            continue
        parts = ['<trkpt lat="%.8f" lon="%.8f">' % (lat, lon)]
        if sample.get("alt") is not None:
            parts.append("<ele>%.2f</ele>" % sample["alt"])
        utc = sample.get("utc")
        if utc is not None:
            parts.append("<time>%s</time>" % fmt_utc(utc))
        # GPX 1.1 has no standard <speed>/<course>; put them in a
        # namespaced extension so the document still validates.
        if (sample.get("speed") is not None
                or sample.get("course") is not None
                or sample.get("heading_source")):
            parts.append("<extensions>")
            if sample.get("speed") is not None:
                parts.append("<v2i:speed>%.2f</v2i:speed>" % sample["speed"])
            if sample.get("course") is not None:
                parts.append("<v2i:course>%.2f</v2i:course>" % sample["course"])
            if sample.get("heading_source"):
                parts.append("<v2i:heading_source>%s</v2i:heading_source>"
                             % _xml_escape(sample["heading_source"]))
            parts.append("</extensions>")
        parts.append("</trkpt>")
        lines.append("    " + "".join(parts))
    lines += ['  </trkseg></trk>', '</gpx>', '']
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))


def _coordinate(sample: dict) -> Optional[List[float]]:
    lat = sample.get("lat")
    lon = sample.get("lon")
    if lat is None:
        lat = sample.get("latitude")
    if lon is None:
        lon = sample.get("longitude")
    if lat is None or lon is None:
        return None
    coord = [float(lon), float(lat)]
    alt = sample.get("alt")
    if alt is None:
        alt = sample.get("altitude_m")
    if alt is not None:
        coord.append(float(alt))
    return coord


def write_geojson(path: str, samples: Sequence[dict],
                  images: Sequence[dict] = (), *, name: str = "track",
                  camera: Optional[str] = None,
                  video: Optional[str] = None) -> None:
    """Write an RFC 7946 FeatureCollection (LineString + image Points)."""
    features: List[dict] = []
    coords = [c for c in (_coordinate(s) for s in samples) if c is not None]
    # RFC 7946 requires at least two positions for a LineString.  A one-point
    # track is represented as a Point; an empty track has no geometry feature.
    if len(coords) >= 2:
        features.append({
            "type": "Feature",
            "geometry": {"type": "LineString", "coordinates": coords},
            "properties": {"name": name, "camera": camera, "video": video,
                           "point_count": len(coords)},
        })
    elif len(coords) == 1:
        features.append({
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": coords[0]},
            "properties": {"name": name, "camera": camera, "video": video,
                           "point_count": 1},
        })
    for image in images:
        coord = _coordinate(image)
        if coord is None:
            continue
        props = {
            "image": image.get("image"),
            "video": image.get("video"),
            "sequence": image.get("sequence"),
            "camera": image.get("camera"),
            "frame_index": image.get("frame_index"),
            "pts_seconds": image.get("pts_seconds"),
            "timestamp_local": image.get("datetime_local"),
            "timestamp_utc": image.get("datetime_utc"),
            "heading_deg": image.get("heading_deg"),
            "heading_source": image.get("heading_source"),
            "speed_m_s": image.get("speed_m_s"),
            "accuracy_m": image.get("accuracy_m"),
            "accuracy_estimated": image.get("accuracy_estimated"),
            "accuracy_source": image.get("accuracy_source"),
            "hdop": image.get("hdop"),
            "vdop": image.get("vdop"),
            "pdop": image.get("pdop"),
            "distance_m": image.get("distance_m"),
            "position_source": image.get("position_source"),
            "clamped": image.get("clamped"),
            "estimated": image.get("estimated"),
        }
        features.append({
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": coord},
            "properties": props,
        })
    payload = {"type": "FeatureCollection", "features": features}
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, sort_keys=False)
        fh.write("\n")


def export_track_files(directory: str, sequence: str, samples: Sequence[dict],
                       images: Sequence[dict], *, track_export: str,
                       camera: str, video: str) -> List[str]:
    """Write the requested ``<sequence>.gpx``/``<sequence>.geojson`` files."""
    written: List[str] = []
    if track_export in ("both", "gpx"):
        path = os.path.join(directory, sequence + ".gpx")
        write_gpx(path, samples, sequence)
        written.append(path)
    if track_export in ("both", "geojson"):
        path = os.path.join(directory, sequence + ".geojson")
        write_geojson(path, samples, images, name=sequence, camera=camera,
                      video=video)
        written.append(path)
    return written
