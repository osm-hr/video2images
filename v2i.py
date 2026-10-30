#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v2i.py -- one-file geotagged still-image exporter for action cameras.

Everything needed to identify videos, read telemetry, select frames, extract
stills and write metadata lives in this single file.  Supported cameras:

    * Garmin VIRB Ultra 30   (MP4 + sibling FIT telemetry)
    * GoPro MAX 360          (``.360`` EAC hemispheres -> equirectangular)
    * BlackVue DR900         (MP4 with embedded NMEA)
    * GoPro HERO             (flat MP4 with GPMF telemetry)

The camera is detected per file, so one input tree may mix manufacturers.  Each
pipeline walks its telemetry track, picks a unique frame every ``--distance``
metres of travel and writes it with EXIF/XMP metadata suitable for Mapillary
and Panoramax.  Unrecognised files are skipped with a logged reason.

Only the Python standard library is used.  ``ffmpeg``, ``ffprobe`` and
``exiftool`` must be available on ``PATH``; ``max2sphere`` is optional.
"""

from __future__ import annotations

import argparse
import bisect
import datetime as dt
import glob
import json
import logging
import math
import os
import re
import shutil
import statistics
import struct
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Callable, Dict, Iterable, Iterator, List, Optional, Sequence, Tuple, Union

SCRIPT_NAME = os.path.basename(os.path.realpath(__file__))
LOG = logging.getLogger("v2i")
EXIFTOOL = os.environ.get("V2I_EXIFTOOL", "exiftool")


class Fatal(Exception):
    """A problem that stops processing the current file."""


# =========================================================================
# Shared helpers: CLI, detection, timelines, extraction and tracks
# =========================================================================

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

# Default scanned extensions.  The single entry point scans this set and uses
# :func:`detect_camera` to route each file to its camera pipeline.
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
        return read_device_name(proc.stdout)
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
    """Register the arguments the unified CLI shares."""
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
                        default=None, metavar="DEGREES",
                        help="degrees added to the heading to derive the image "
                             "direction; recalculated to 000-360 and appended "
                             "to the camera folder as -NNN")
    parser.add_argument("--offset", dest="heading_offset", type=float,
                        default=None,
                        help="alias for --heading-offset")
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
    parser.add_argument("--telemetry-export",
                        choices=("both", "points", "images", "none"),
                        default="both",
                        help="write detailed telemetry JSON next to each "
                             "sequence: every raw point (points), per-image "
                             "snapshots (images) or both (default)")
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
    offset = args.heading_offset
    if offset is None:
        args.heading_offset_given = False
        args.heading_offset = 0.0
    else:
        if not math.isfinite(offset):
            parser.error("--heading-offset must be a finite number of degrees")
        args.heading_offset_given = True
        args.heading_offset = normalize_heading_offset(offset)
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
    """Configure root logging for the exporter."""
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
def normalize_heading_offset(value: float) -> float:
    """Recalculate a heading offset into the unsigned ``[0, 360]`` range."""
    raw = float(value)
    if not math.isfinite(raw):
        raise ValueError("heading offset must be a finite number of degrees")
    wrapped = raw % 360.0
    # Keep an explicit full turn as 360 rather than collapsing it to 000.
    if wrapped == 0.0 and raw > 0.0:
        return 360.0
    return wrapped


def format_heading_offset(value: float) -> str:
    """Return the three-digit folder label for a heading offset (``000``-``360``)."""
    return "%03d" % int(round(normalize_heading_offset(value)))


def heading_offset_for_folder(args: argparse.Namespace) -> Optional[float]:
    """Offset to append to the camera folder, or ``None`` when not given."""
    if getattr(args, "heading_offset_given", False):
        return args.heading_offset
    return None


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
# Frame extraction (one accurate seek + single frame per requested index)
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
# kept comfortably below that limit.  Used by the MAX360 EAC stitcher, which
# still decodes a bounded batch per ffmpeg call.
EXTRACT_CHUNK_SIZE = 60


def _extract_one(video: str, frame_index: int, seek_time: float, pattern: str,
                 *, image_format: str, quality: int, threads: int,
                 stream_ordinal: int) -> Optional[str]:
    """Extract one frame with an accurate input seek; return its path."""
    cmd = [FFMPEG, "-hide_banner", "-loglevel", "error", "-y",
           "-ss", "%.6f" % max(0.0, seek_time)]
    if threads:
        cmd += ["-threads", str(threads)]
    cmd += ["-i", video,
            "-map", "0:v:%d" % stream_ordinal,
            "-frames:v", "1"]
    cmd += ffmpeg_quality_args(image_format, quality)
    cmd.append(pattern)
    try:
        proc = subprocess.run(cmd, stdout=subprocess.DEVNULL,
                              stderr=subprocess.PIPE, timeout=300)
    except (OSError, subprocess.TimeoutExpired) as exc:
        LOG.warning("frame %d extraction failed: %s", frame_index, exc)
        return None
    if proc.returncode != 0:
        err = proc.stderr.decode("utf-8", "replace").strip()
        tail = err.splitlines()[-1:] if err else []
        LOG.warning("frame %d extraction failed: %s", frame_index,
                    tail[0] if tail else "ffmpeg error")
        return None
    output = pattern % 1
    return output if os.path.exists(output) else None


def extract_frames_by_index(video: str, frames: Sequence[int], tmp_dir: str, *,
                            image_format: str, quality: int, workers: int = 1,
                            threads: int = 0, stream_ordinal: int = 0,
                            timeline: Optional[FrameTimeline] = None
                            ) -> Dict[int, str]:
    """Extract exact frames, one accurate seek per frame.

    Seeking to the frame's real PTS (``timeline.pts``) keeps variable frame
    rate sources correct; a uniform ``index / fps`` grid is only used when no
    usable timeline is available.  ``workers`` independent ffmpeg processes
    are run at a time.
    """
    unique = sorted({int(f) for f in frames if int(f) >= 0})
    if not unique:
        return {}
    os.makedirs(tmp_dir, exist_ok=True)

    # Try to get fps from timeline if available
    fps = timeline.fps if timeline is not None and timeline.fps > 0 else 23.976
    if timeline is None or timeline.source != "ffprobe" or len(timeline) == 0:
        # try to guess
        try:
            info = probe_stream_info(video)
            fps = _parse_rate(info.get("avg_frame_rate") or info.get("r_frame_rate")) or fps
        except Exception:
            pass

    have_timeline = (timeline is not None and timeline.source == "ffprobe"
                     and len(timeline) > 0)

    def seek_for(frame_index: int) -> float:
        if have_timeline and frame_index < len(timeline):
            return float(timeline.pts(frame_index))
        return frame_index / fps if fps > 0 else 0.0

    def run(item: Tuple[int, int]) -> Tuple[int, Optional[str]]:
        order, frame_index = item
        pattern = os.path.join(tmp_dir,
                               "f_%06d_%%06d.%s" % (order, image_format))
        path = _extract_one(video, frame_index, seek_for(frame_index), pattern,
                            image_format=image_format, quality=quality,
                            threads=threads, stream_ordinal=stream_ordinal)
        return frame_index, path

    items = list(enumerate(unique))
    extracted: Dict[int, str] = {}
    if workers > 1 and len(items) > 1:
        with ThreadPoolExecutor(max_workers=min(workers, len(items))) as pool:
            results = list(pool.map(run, items))
    else:
        results = [run(item) for item in items]

    for frame_index, path in results:
        if path is not None:
            extracted[frame_index] = path
        else:
            LOG.error("could not extract frame %d from %s",
                      frame_index, os.path.basename(video))
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


def date_folder_name(value: dt.datetime, camera: str, *,
                     heading_offset: Optional[float] = None) -> str:
    """``YYYY-MM-DD-<camera-suffix>`` for the first exported frame.

    A supplied heading offset is recalculated to ``000``-``360`` and appended
    to the camera folder: ``YYYY-MM-DD-<camera-suffix>-<offset>``.
    """
    name = "%s-%s" % (value.strftime("%Y-%m-%d"), CAMERA_SUFFIX[camera])
    if heading_offset is not None:
        name = "%s-%s" % (name, format_heading_offset(heading_offset))
    return name


def sequence_output_dir(output_dir: str, camera: str, sequence: str,
                        first_capture: dt.datetime, *,
                        heading_offset: Optional[float] = None) -> str:
    """``<output>/<date>-<camera>/<sequence>``.

    When a heading offset was explicitly supplied it is recalculated to
    ``000``-``360`` and appended to the camera folder:
    ``<output>/<date>-<camera>-<offset>/<sequence>``.
    """
    return os.path.join(
        output_dir,
        date_folder_name(first_capture, camera, heading_offset=heading_offset),
        sequence)


def unique_path(path: str, used: set) -> str:
    """Disambiguate two planned images that share the same millisecond.

    ``used`` holds the paths already planned in this run.  Files that merely
    exist on disk are *not* collisions: the caller compares the returned path
    with the filesystem to skip (or overwrite) them, so treating them as
    collisions here would turn every re-run into ``-2`` duplicates.
    """
    if path not in used:
        used.add(path)
        return path
    base, ext = os.path.splitext(path)
    suffix = 2
    candidate = "%s-%d%s" % (base, suffix, ext)
    while candidate in used:
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
                       camera: str, video: str,
                       imu: Optional[Dict[str, List[dict]]] = None,
                       video_start_utc: Optional[dt.datetime] = None,
                       telemetry_export: str = "both",
                       calibration: Optional[Dict[str, dict]] = None,
                       units: Optional[Dict[str, str]] = None,
                       time_scale: float = 1.0,
                       extra: Optional[dict] = None) -> List[str]:
    """Write the requested track and detailed telemetry files."""
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
    if telemetry_export in ("both", "points", "images"):
        os.makedirs(directory, exist_ok=True)
    if telemetry_export in ("both", "points"):
        path = os.path.join(directory, sequence + ".telemetry.json")
        write_points_telemetry(
            path, camera=camera, video=video, sequence=sequence,
            gps=samples, imu=imu, video_start_utc=video_start_utc,
            calibration=calibration, units=units, time_scale=time_scale,
            extra=extra)
        written.append(path)
    if telemetry_export in ("both", "images"):
        path = os.path.join(directory, sequence + ".image-telemetry.json")
        write_images_telemetry(
            path, camera=camera, video=video, sequence=sequence,
            images=images, imu=imu, video_start_utc=video_start_utc,
            time_scale=time_scale, extra=extra)
        written.append(path)
    if written:
        LOG.info("  wrote %d export file(s): %s", len(written),
                 ", ".join(os.path.basename(path) for path in written))
    return written


# ---------------------------------------------------------------------------
# Detailed telemetry JSON writers
# ---------------------------------------------------------------------------
# GPX/GeoJSON only keep positions.  These two writers also export the raw
# telemetry: one file with every sampled point (GPS and IMU) and one with the
# GPS/IMU snapshot nearest to each exported image.
_IMU_COMPONENTS = {
    "ACCL": ["x", "y", "z"],
    "GYRO": ["x", "y", "z"],
    "MAGN": ["x", "y", "z"],
    "GRAV": ["x", "y", "z"],
    "CORI": ["x", "y", "z"],
    "ORIN": ["x", "y", "z"],
    "IORI": ["x", "y"],
}

_IMU_UNITS = {
    "ACCL": "m/s^2",
    "GYRO": "rad/s",
    "MAGN": "uT",
    "GRAV": "g",
}


def _json_default(value):
    """Serialise datetimes and tuples found in telemetry samples."""
    if isinstance(value, dt.datetime):
        return fmt_utc(value)
    if isinstance(value, dt.date):
        return value.isoformat()
    if isinstance(value, dt.timedelta):
        return value.total_seconds()
    if isinstance(value, (tuple, set)):
        return list(value)
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    raise TypeError("cannot serialise %r" % (type(value).__name__,))


def _as_utc(value) -> Optional[dt.datetime]:
    if isinstance(value, dt.datetime):
        if value.tzinfo is None:
            return value.replace(tzinfo=dt.timezone.utc)
        return value.astimezone(dt.timezone.utc)
    if isinstance(value, str):
        return parse_utc_timestamp(value)
    return None


def _component_names(key: str, count: int) -> List[str]:
    known = _IMU_COMPONENTS.get(key)
    if known and len(known) == count:
        return list(known)
    if count == 4:
        return ["w", "x", "y", "z"]
    if count == 3:
        return ["x", "y", "z"]
    if count == 2:
        return ["x", "y"]
    return ["v%d" % index for index in range(count)]


def _clean_telemetry_point(point: dict, video_start_utc=None,
                           time_scale: float = 1.0) -> dict:
    """Drop private keys and make one GPS sample JSON-ready."""
    out: dict = {}
    for key, value in point.items():
        if key.startswith("_") or key in ("point", "naming_time", "path",
                                          "written"):
            continue
        if isinstance(value, dt.datetime):
            out[key] = fmt_utc(value)
        else:
            out[key] = _json_default(value) if isinstance(
                value, (dt.date, dt.timedelta, tuple, set, bytes)) else value
    utc = _as_utc(point.get("utc"))
    if utc is not None:
        out["utc"] = fmt_utc(utc)
        origin = _as_utc(video_start_utc)
        if origin is not None:
            out["t_s"] = round((utc - origin).total_seconds() * time_scale, 6)
    return out


def _clean_imu_point(point: dict, key: str, video_start_utc=None,
                     time_scale: float = 1.0) -> dict:
    """Make one IMU sample JSON-ready, naming x/y/z when possible."""
    out: dict = {}
    utc = _as_utc(point.get("utc"))
    if utc is not None:
        out["utc"] = fmt_utc(utc)
        origin = _as_utc(video_start_utc)
        if origin is not None:
            out["t_s"] = round((utc - origin).total_seconds() * time_scale, 6)
    if point.get("stmp_us") is not None:
        out["stmp_us"] = point["stmp_us"]
    if point.get("fit_s") is not None:
        out["fit_s"] = round(float(point["fit_s"]), 6)
    values = point.get("values") or []
    names = _component_names(key, len(values))
    for name, value in zip(names, values):
        out[name] = value
    if point.get("raw") is not None:
        out["raw"] = list(point["raw"])
    for field_name in ("unit", "source"):
        if point.get(field_name) is not None:
            out[field_name] = point[field_name]
    return out


def _imu_stream_entry(key: str, rows: Sequence[dict], video_start_utc=None,
                      units=None, time_scale: float = 1.0) -> dict:
    size = 0
    for row in rows:
        size = max(size, len(row.get("values") or ()))
    unit = None
    if units and units.get(key):
        unit = units[key]
    elif key in _IMU_UNITS:
        unit = _IMU_UNITS[key]
    return {
        "unit": unit,
        "components": _component_names(key, size),
        "point_count": len(rows),
        "points": [_clean_imu_point(row, key, video_start_utc, time_scale)
                   for row in rows],
    }


def write_points_telemetry(path: str, *, camera: str, video: str,
                           sequence: str, gps: Sequence[dict],
                           imu: Optional[Dict[str, List[dict]]] = None,
                           video_start_utc=None,
                           calibration: Optional[Dict[str, dict]] = None,
                           units: Optional[Dict[str, str]] = None,
                           time_scale: float = 1.0,
                           extra: Optional[dict] = None) -> None:
    """Write every telemetry point (GPS and IMU) for one sequence."""
    streams: Dict[str, dict] = {}
    if gps:
        streams["gps"] = {
            "point_count": len(gps),
            "points": [_clean_telemetry_point(point, video_start_utc,
                                              time_scale) for point in gps],
        }
    for key in sorted(imu or {}):
        rows = imu[key]
        if rows:
            streams[key] = _imu_stream_entry(
                key, rows, video_start_utc, units, time_scale)
    payload = {
        "generator": SCRIPT_NAME,
        "camera": camera,
        "camera_label": CAMERA_LABELS.get(camera, camera),
        "video": video,
        "sequence": sequence,
        "video_start_utc": fmt_utc(_as_utc(video_start_utc)),
        "streams": streams,
    }
    if calibration:
        payload["calibration"] = calibration
    if extra:
        payload.update(extra)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, default=_json_default,
                  sort_keys=False)
        fh.write("\n")


def _nearest_timed(times: Sequence[dt.datetime], rows: Sequence[dict],
                   target: Optional[dt.datetime],
                   max_delta_s: float = 1.0):
    if target is None or not times:
        return None
    if target.tzinfo is None:
        target = target.replace(tzinfo=dt.timezone.utc)
    index = bisect.bisect_left(times, target)
    best = None
    for candidate in (index - 1, index):
        if 0 <= candidate < len(times):
            delta = abs((times[candidate] - target).total_seconds())
            if best is None or delta < best[0]:
                best = (delta, candidate)
    if best is None or best[0] > max_delta_s:
        return None
    return rows[best[1]], best[0]


def write_images_telemetry(path: str, *, camera: str, video: str,
                           sequence: str, images: Sequence[dict],
                           imu: Optional[Dict[str, List[dict]]] = None,
                           video_start_utc=None, time_scale: float = 1.0,
                           extra: Optional[dict] = None) -> None:
    """Write the GPS/IMU snapshot nearest to every exported image."""
    indexes: Dict[str, Tuple[List[dt.datetime], List[dict]]] = {}
    for key, rows in (imu or {}).items():
        timed = []
        for row in rows:
            utc = _as_utc(row.get("utc"))
            if utc is not None:
                timed.append((utc, row))
        timed.sort(key=lambda item: item[0])
        indexes[key] = ([item[0] for item in timed],
                        [item[1] for item in timed])

    entries = []
    for record in images:
        point = record.get("point") or {}
        target = _as_utc(record.get("datetime_utc"))
        point_utc = _as_utc(point.get("utc"))
        if target is None:
            target = point_utc
        entry = {key: value for key, value in record.items()
                 if key not in ("point", "naming_time", "path", "written")}
        entry = {key: _json_default(value) if isinstance(
            value, (dt.date, dt.timedelta, tuple, set, bytes)) else value
            for key, value in entry.items()}
        entry["gps"] = (_clean_telemetry_point(point, video_start_utc,
                                               time_scale) if point else None)
        snapshot: Dict[str, Optional[dict]] = {}
        for key, (times, rows) in indexes.items():
            match = _nearest_timed(times, rows, target)
            if match is None and point_utc is not None and point_utc != target:
                match = _nearest_timed(times, rows, point_utc)
            if match is None:
                snapshot[key] = None
                continue
            row, delta = match
            sample = _clean_imu_point(row, key, video_start_utc, time_scale)
            sample["delta_s"] = round(delta, 6)
            snapshot[key] = sample
        entry["imu"] = snapshot
        entries.append(entry)

    payload = {
        "generator": SCRIPT_NAME,
        "camera": camera,
        "camera_label": CAMERA_LABELS.get(camera, camera),
        "video": video,
        "sequence": sequence,
        "video_start_utc": fmt_utc(_as_utc(video_start_utc)),
        "image_count": len(entries),
        "images": entries,
    }
    if extra:
        payload.update(extra)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, default=_json_default,
                  sort_keys=False)
        fh.write("\n")


# =========================================================================
# GoPro GPMF telemetry reader (MAX 360 and HERO)
# =========================================================================

TYPE_STRING = ord("c")
TYPE_INT8 = ord("b")
TYPE_UINT8 = ord("B")
TYPE_INT16 = ord("s")
TYPE_UINT16 = ord("S")
TYPE_INT32 = ord("l")
TYPE_UINT32 = ord("L")
TYPE_INT64 = ord("j")
TYPE_UINT64 = ord("J")
TYPE_FLOAT = ord("f")
TYPE_DOUBLE = ord("d")
TYPE_Q15_16 = ord("q")
TYPE_Q31_32 = ord("Q")
TYPE_FOURCC = ord("F")
TYPE_UTC = ord("U")
TYPE_UTF8 = ord("u")
TYPE_GUID = ord("G")
TYPE_COMPLEX = ord("?")
TYPE_COMPRESSED = ord("#")
TYPE_NEST = 0x00

_NUMERIC = {
    TYPE_INT8: ("b", 1), TYPE_UINT8: ("B", 1),
    TYPE_INT16: ("h", 2), TYPE_UINT16: ("H", 2),
    TYPE_INT32: ("i", 4), TYPE_UINT32: ("I", 4),
    TYPE_INT64: ("q", 8), TYPE_UINT64: ("Q", 8),
    TYPE_FLOAT: ("f", 4), TYPE_DOUBLE: ("d", 8),
    TYPE_Q15_16: ("i", 4), TYPE_Q31_32: ("q", 8),
}
_BYTES = {TYPE_STRING, TYPE_FOURCC, TYPE_UTC, TYPE_UTF8, TYPE_GUID,
          TYPE_COMPLEX, TYPE_COMPRESSED}
_KNOWN_BYTES = set(_NUMERIC) | _BYTES | {TYPE_NEST}

GPS5_SCALE = [1e7, 1e7, 1e3, 1e3, 1e2]       # lat, lon, alt m, m/s, m/s
GPS9_SCALE = [1e7, 1e7, 1e3, 1e3, 1e2, 1.0, 1e3, 1e2, 1.0]
GPS9_EPOCH = dt.datetime(2000, 1, 1, tzinfo=dt.timezone.utc)


class GpmfError(Exception):
    """Raised when the GPMF stream cannot be parsed."""


@dataclass
class Element:
    level: int
    key: str
    typ: int
    struct_size: int
    repeat: int
    offset: int
    length: int
    stream: str = ""

    @property
    def end(self) -> int:
        return self.offset + self.length

    @property
    def type_char(self) -> str:
        return chr(self.typ) if 32 <= self.typ < 127 else "\\x%02x" % self.typ

    @property
    def is_compressed(self) -> bool:
        return self.typ == TYPE_COMPRESSED


def iter_elements(buf: bytes, start: int = 0, end: Optional[int] = None,
                  level: int = 0, stream: str = "") -> Iterator[Element]:
    """Yield every GPMF element in ``buf`` in document (pre-)order."""
    if end is None:
        end = len(buf)
    pos = start
    while pos + 8 <= end:
        key = buf[pos:pos + 4]
        typ = buf[pos + 4]
        struct_size = buf[pos + 5]
        repeat = struct.unpack_from(">H", buf, pos + 6)[0]

        if key == b"\x00\x00\x00\x00":
            pos += 4
            continue

        if typ not in _KNOWN_BYTES:
            raise GpmfError(
                "unknown GPMF type 0x%02x for key %r at offset %d"
                % (typ, key.decode("ascii", "replace"), pos))

        name = key.decode("ascii", "replace")
        if typ == TYPE_NEST:
            payload_len = repeat
            yield Element(level, name, typ, struct_size, repeat, pos + 8,
                          payload_len, stream)
            child_stream = stream
            if level == 1 and name == "STRM":
                child_stream = ""
            yield from iter_elements(buf, pos + 8, pos + 8 + payload_len,
                                     level + 1, child_stream)
        else:
            payload_len = struct_size * repeat
            yield Element(level, name, typ, struct_size, repeat, pos + 8,
                          payload_len, stream)

        pos += 8 + payload_len
        pos = (pos + 3) & ~3


def decode(buf: bytes, elem: Element) -> Union[List, bytes]:
    """Decode an element payload (numbers as a list, strings as bytes)."""
    data = buf[elem.offset:elem.end]
    if elem.typ in _BYTES:
        return b"" if elem.typ == TYPE_COMPRESSED else data
    fmt, esz = _NUMERIC[elem.typ]
    n = elem.length // esz
    return list(struct.unpack(">" + fmt * n, data[:n * esz]))


def as_text(buf: bytes, elem: Element) -> str:
    """Return a decoded element as a NUL-trimmed ASCII string."""
    raw = buf[elem.offset:elem.end]
    return raw.split(b"\x00", 1)[0].decode("ascii", "replace")


def scaled(values: Sequence[float], scale: Optional[Sequence[float]]
           ) -> List[float]:
    """Apply a GPMF SCAL divisor per position (``value / scale``)."""
    if not scale:
        return list(values)
    out = []
    for i, value in enumerate(values):
        divisor = scale[i % len(scale)]
        out.append(value / divisor if divisor else value)
    return out


def read_device_name(buf: bytes) -> Optional[str]:
    """Return the first GPMF ``DVNM`` device name, or ``None``."""
    for elem in iter_elements(buf):
        if elem.key == "DVNM":
            text = as_text(buf, elem).strip()
            if text:
                return text
    return None


# ---------------------------------------------------------------------------
# Timestamps
# ---------------------------------------------------------------------------
def parse_gpsu(text: str) -> dt.datetime:
    """Parse GPMF ``GPSU`` ``yymmddhhmmss.sss`` as UTC."""
    yy = int(text[0:2])
    mo = int(text[2:4])
    dd = int(text[4:6])
    hh = int(text[6:8])
    mi = int(text[8:10])
    sec = float(text[10:])
    whole = int(sec)
    frac_us = int(round((sec - whole) * 1_000_000))
    if frac_us >= 1_000_000:
        whole += 1
        frac_us -= 1_000_000
    return dt.datetime(2000 + yy, mo, dd, hh, mi, whole, frac_us,
                       tzinfo=dt.timezone.utc)


def _epoch_us(value: dt.datetime) -> int:
    return int(round(value.timestamp() * 1_000_000))


def _utc_from_gps9(days, seconds) -> Optional[dt.datetime]:
    if days is None or seconds is None:
        return None
    try:
        days_i = int(days)
        secs = float(seconds)
        # GPS9 stores milliseconds in some firmware, seconds in others.
        if secs > 86400:
            secs = secs / 1000.0
        return GPS9_EPOCH + dt.timedelta(days=days_i, seconds=secs)
    except (TypeError, ValueError, OverflowError):
        return None


# ---------------------------------------------------------------------------
# Blocks / streams
# ---------------------------------------------------------------------------
@dataclass
class _Block:
    stream: str = ""
    scale: Optional[List[float]] = None
    stmp_us: Optional[int] = None
    utc: Optional[dt.datetime] = None
    fix: Optional[int] = None
    dop: Optional[List[float]] = None
    keys: Dict[str, list] = field(default_factory=dict)

    def add(self, key: str, values: list) -> None:
        self.keys.setdefault(key, []).extend(values)


def _collect_blocks(buf: bytes) -> Dict[str, List[_Block]]:
    """Group every ``STRM`` block by its ``STNM`` stream name."""
    blocks: Dict[str, List[_Block]] = {}
    current: Optional[_Block] = None
    device: Optional[str] = None

    def flush() -> None:
        nonlocal current
        if current is not None:
            blocks.setdefault(current.stream or "unknown", []).append(current)
        current = None

    for elem in iter_elements(buf):
        if elem.level == 0 and elem.key == "DEVC":
            flush()
            device = None
        elif elem.level == 1 and elem.key == "DVNM" and current is None:
            text = as_text(buf, elem).strip()
            if text:
                device = text
        elif elem.level == 1 and elem.key == "STRM":
            flush()
            current = _Block()
        elif elem.level == 2 and current is not None:
            if elem.key == "STNM":
                current.stream = as_text(buf, elem).strip()
            elif elem.key == "SCAL":
                values = decode(buf, elem)
                if isinstance(values, list):
                    current.scale = values
            elif elem.key == "STMP":
                values = decode(buf, elem)
                if isinstance(values, list) and values:
                    current.stmp_us = int(values[0])
            elif elem.key == "GPSU":
                text = as_text(buf, elem)
                if len(text) >= 12:
                    try:
                        current.utc = parse_gpsu(text)
                    except ValueError:
                        pass
            elif elem.key == "GPSF":
                values = decode(buf, elem)
                if isinstance(values, list) and values:
                    current.fix = int(values[0])
            elif elem.key == "GPSP":
                values = decode(buf, elem)
                if isinstance(values, list):
                    current.dop = [float(v) / 100.0 for v in values]
            elif elem.typ == TYPE_COMPRESSED:
                # Huffman-compressed payloads are not decoded here.
                current.keys.setdefault("_compressed", []).append(elem.key)
            elif elem.typ in _NUMERIC:
                values = decode(buf, elem)
                if isinstance(values, list):
                    current.add(elem.key, values)
    flush()
    return blocks


def _block_period(blocks: List[_Block], index: int, count: int) -> float:
    """Seconds per sample for a block (microseconds actually)."""
    block = blocks[index]
    if block.stmp_us is None:
        return 1_000_000.0 / 30.0
    if index + 1 < len(blocks) and blocks[index + 1].stmp_us is not None:
        return (blocks[index + 1].stmp_us - block.stmp_us) / max(1, count)
    if index > 0 and blocks[index - 1].stmp_us is not None:
        return (block.stmp_us - blocks[index - 1].stmp_us) / max(1, count)
    return 1_000_000.0 / 30.0


def _group(values: Sequence[float], size: int) -> List[Tuple[float, ...]]:
    return [tuple(values[i:i + size])
            for i in range(0, len(values) - size + 1, size)]


def _gps_rows(block: _Block) -> Tuple[List[Tuple[float, ...]], str]:
    if "GPS5" in block.keys:
        return _group(block.keys["GPS5"], 5), "GPS5"
    if "GPS9" in block.keys:
        return _group(block.keys["GPS9"], 9), "GPS9"
    return [], ""


def _make_gps_sample(row: Sequence[float], scale: Sequence[float],
                     kind: str, block: _Block, utc: Optional[dt.datetime],
                     stmp_us: Optional[int]) -> Optional[dict]:
    defaults = GPS5_SCALE if kind == "GPS5" else GPS9_SCALE
    divisors = list(scale) if scale else list(defaults)
    if len(divisors) < len(row):
        divisors += defaults[len(divisors):]
    vals = [row[i] / divisors[i] if divisors[i] else row[i]
            for i in range(len(row))]
    lat, lon = vals[0], vals[1]
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    if lat == 0.0 and lon == 0.0:
        return None
    alt = vals[2] if len(vals) > 2 else None
    speed2d = vals[3] if len(vals) > 3 else None
    speed3d = vals[4] if len(vals) > 4 else None
    if kind == "GPS9":
        if utc is None and len(row) >= 7:
            utc = _utc_from_gps9(row[5], row[6])
        dop = vals[7] if len(vals) > 7 else None
        fix = int(row[8]) if len(row) > 8 else block.fix
    else:
        dop = None
        fix = block.fix
    dop_values = block.dop or []
    hdop = dop_values[0] if len(dop_values) >= 1 else dop
    vdop = dop_values[1] if len(dop_values) >= 2 else None
    return {
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "speed": speed2d,
        "speed2d": speed2d,
        "speed3d": speed3d,
        "fix": fix,
        "hdop": hdop,
        "vdop": vdop,
        "pdop": None,
        "dop": hdop,
        "utc": utc,
        "stmp_us": stmp_us,
    }


def _build_gps_samples(blocks: List[_Block]
                       ) -> Tuple[List[dict], float, int]:
    gps_blocks = [b for b in blocks if _gps_rows(b)[0]]
    if not gps_blocks:
        return [], 0.0, 0

    offsets = []
    for block in gps_blocks:
        if block.stmp_us is not None and block.utc is not None:
            offsets.append(_epoch_us(block.utc) - block.stmp_us)
    median_offset = int(statistics.median(offsets)) if offsets else 0
    spread = ((max(offsets) - min(offsets)) / 1e6
              if len(offsets) > 1 else 0.0)

    samples: List[dict] = []
    for bi, block in enumerate(gps_blocks):
        rows, kind = _gps_rows(block)
        if not rows:
            continue
        scale = block.scale or (GPS5_SCALE if kind == "GPS5"
                                else GPS9_SCALE)
        period = _block_period(gps_blocks, bi, len(rows))
        for i, row in enumerate(rows):
            stmp = (int(block.stmp_us + i * period)
                    if block.stmp_us is not None else None)
            if stmp is not None and median_offset:
                utc = dt.datetime.fromtimestamp(
                    (stmp + median_offset) / 1e6, dt.timezone.utc)
            else:
                utc = block.utc
            sample = _make_gps_sample(row, scale, kind, block, utc, stmp)
            if sample is not None:
                if sample["utc"] is None and block.utc is not None:
                    sample["utc"] = block.utc
                samples.append(sample)
    samples.sort(key=lambda s: (s["utc"] or dt.datetime.min.replace(
        tzinfo=dt.timezone.utc), s["stmp_us"] or 0))
    return samples, spread, median_offset


def _build_imu_streams(blocks_by_name: Dict[str, List[_Block]],
                       median_offset: int = 0) -> Dict[str, List[dict]]:
    """Decode IMU-ish streams into ``{KEY: [{stmp_us, utc, values}]}``.

    GoPro's ``STNM`` is a human-readable description ("Magnetometer"), while
    the actual payload keys are ``MAGN``/``ACCL``/...; indexing by the payload
    key keeps this independent of firmware naming.
    """
    imu: Dict[str, List[dict]] = {}
    wanted = {"MAGN": 3, "ACCL": 3, "GYRO": 3, "GRAV": 3,
              "CORI": 3, "IORI": 2, "ORIN": 3}
    for _name, blocks in blocks_by_name.items():
        for key, size in wanted.items():
            present = [block for block in blocks if block.keys.get(key)]
            if not present:
                continue
            rows_out = imu.setdefault(key, [])
            for block in present:
                values = block.keys[key]
                rows = _group(values, size)
                scale = block.scale
                bi = blocks.index(block)
                period = _block_period(blocks, bi, max(1, len(rows)))
                for i, row in enumerate(rows):
                    vals = scaled(row, scale) if scale else list(row)
                    stmp = (int(block.stmp_us + i * period)
                            if block.stmp_us is not None else None)
                    utc = None
                    if stmp is not None and median_offset:
                        utc = dt.datetime.fromtimestamp(
                            (stmp + median_offset) / 1e6, dt.timezone.utc)
                    elif block.utc is not None:
                        utc = block.utc
                        if i:
                            utc = utc + dt.timedelta(microseconds=i * period)
                    rows_out.append({"stmp_us": stmp, "utc": utc,
                                     "values": list(vals)})
    return imu


def _nearest_imu(rows: Sequence[dict], stmp_us: Optional[int],
                 utc: Optional[dt.datetime], max_delta_us: int = 500000
                 ) -> Optional[dict]:
    if not rows:
        return None
    if stmp_us is not None and rows[0].get("stmp_us") is not None:
        best = min(rows, key=lambda r: abs((r.get("stmp_us") or 0) - stmp_us))
        if abs((best.get("stmp_us") or 0) - stmp_us) <= max_delta_us:
            return best
        return None
    if utc is not None and rows[0].get("utc") is not None:
        best = min(rows, key=lambda r: abs(
            (r.get("utc") - utc).total_seconds()))
        if abs((best.get("utc") - utc).total_seconds()) <= max_delta_us / 1e6:
            return best
    return None


def _tilt_compensated_heading(magnetic: Sequence[float],
                              gravity: Sequence[float]) -> Optional[float]:
    """Approximate magnetic heading from device-frame MAGN + GRAV/ACCL."""
    if len(magnetic) < 3 or len(gravity) < 3:
        return None
    mx, my, mz = magnetic[:3]
    gx, gy, gz = gravity[:3]
    gnorm = (gx * gx + gy * gy + gz * gz) ** 0.5
    if gnorm < 1e-9:
        return None
    gx, gy, gz = gx / gnorm, gy / gnorm, gz / gnorm
    # East = magnetic x gravity, North = gravity x East.
    ex = my * gz - mz * gy
    ey = mz * gx - mx * gz
    ez = mx * gy - my * gx
    nx = gy * ez - gz * ey
    ny = gz * ex - gx * ez
    nz = gx * ey - gy * ex
    en = (ex * ex + ey * ey + ez * ez) ** 0.5
    nn = (nx * nx + ny * ny + nz * nz) ** 0.5
    if en < 1e-9 or nn < 1e-9:
        return None
    import math
    return (math.degrees(math.atan2(ex, nx)) + 360.0) % 360.0


def _attach_compass(gps_samples: List[dict], imu: Dict[str, List[dict]]
                    ) -> None:
    mags = imu.get("MAGN", [])
    gravity = imu.get("GRAV") or imu.get("ACCL") or []
    if not mags or not gravity:
        return
    for sample in gps_samples:
        mag = _nearest_imu(mags, sample.get("stmp_us"), sample.get("utc"))
        grav = _nearest_imu(gravity, sample.get("stmp_us"), sample.get("utc"))
        if mag is None or grav is None:
            continue
        heading = _tilt_compensated_heading(mag["values"], grav["values"])
        if heading is not None:
            sample["compass_heading"] = heading
            sample["heading_source"] = "compass"


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
@dataclass
class GpmfData:
    device_name: Optional[str]
    streams: Dict[str, dict]
    gps_samples: List[dict]
    imu: Dict[str, List[dict]]
    has_gps: bool
    clock_spread_s: float = 0.0
    origin_utc: Optional[dt.datetime] = None
    time_scale: float = 1.0


def _anchor_imu_utc(imu: Dict[str, List[dict]],
                    origin: Optional[dt.datetime]) -> None:
    """Give IMU rows a UTC clock when the file has no GPS time reference.

    GoPro ``STMP`` is microseconds since the start of the recording, so the
    container creation time anchors the IMU samples to wall clock even for
    unpositioned HERO clips.  Rows that already have a UTC value are kept.
    """
    if origin is None:
        return
    if origin.tzinfo is None:
        origin = origin.replace(tzinfo=dt.timezone.utc)
    for rows in imu.values():
        for row in rows:
            if row.get("utc") is None and row.get("stmp_us") is not None:
                row["utc"] = origin + dt.timedelta(
                    microseconds=int(row["stmp_us"]))


def parse_gpmf(buf: bytes, *, imu_heading: bool = False) -> GpmfData:
    """Parse a raw gpmd payload into GPS samples and IMU streams."""
    if not buf:
        raise GpmfError("empty GPMF payload")
    device_name = read_device_name(buf)
    blocks = _collect_blocks(buf)
    # Some GoPro files contain unrelated Huffman-compressed streams (for
    # example DISP/DisparityMatrix).  They are irrelevant here; ignore them
    # unless the GPS stream itself is compressed, in which case there is no
    # usable position data.
    streams: Dict[str, dict] = {}
    for name, items in blocks.items():
        keys = sorted({key for block in items for key in block.keys
                       if not key.startswith("_")})
        compressed = sorted({key for block in items
                             for key in block.keys.get("_compressed", [])})
        if compressed:
            keys.append("compressed:" + ",".join(compressed))
        streams[name] = {"blocks": len(items), "keys": keys}

    if "GPS5" in blocks:
        gps_compressed = any(block.keys.get("_compressed") for block in blocks["GPS5"])
        gps_decoded = any(block.keys.get("GPS5") for block in blocks["GPS5"])
        if gps_compressed and not gps_decoded:
            raise GpmfError("GPS5 is Huffman-compressed (QUAN), which is not "
                            "supported")

    all_blocks = [block for items in blocks.values() for block in items]
    gps_samples, spread, median_offset = _build_gps_samples(all_blocks)
    imu = _build_imu_streams(blocks, median_offset)
    if imu_heading:
        _attach_compass(gps_samples, imu)
    return GpmfData(device_name=device_name, streams=streams,
                    gps_samples=gps_samples, imu=imu,
                    has_gps=bool(gps_samples), clock_spread_s=spread)


def extract_gpmf(path: str, stream_index: int, *, ffmpeg: str = FFMPEG
                 ) -> bytes:
    """Copy a gpmd stream out of the container with ffmpeg."""
    try:
        proc = subprocess.run(
            [ffmpeg, "-v", "error", "-i", path, "-map",
             "0:%d" % stream_index, "-c", "copy", "-f", "data", "-"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError as exc:
        raise GpmfError("required program not found: ffmpeg") from exc
    if proc.returncode != 0:
        tail = proc.stderr.decode("utf-8", "replace").strip().splitlines()
        raise GpmfError("could not extract gpmd stream: %s"
                        % (tail[-1] if tail else "ffmpeg failed"))
    return proc.stdout

# =========================================================================
# EXIF / XMP metadata construction
# =========================================================================

@dataclass
class FrameMetadata:
    camera: str
    video: str
    sequence: str
    frame_index: int
    pts: float
    distance: Optional[float]
    script_name: str

    make: Optional[str] = None
    model: Optional[str] = None
    software: Optional[str] = None
    serial: Optional[str] = None

    captured_utc: Optional[dt.datetime] = None       # aware UTC
    captured_local: Optional[dt.datetime] = None     # naive camera-local
    utc_offset_s: Optional[int] = None               # None = unknown

    lat: Optional[float] = None
    lon: Optional[float] = None
    alt: Optional[float] = None
    speed: Optional[float] = None                    # m/s
    course: Optional[float] = None                   # true north
    img_direction: Optional[float] = None            # true north
    heading_source: str = "none"

    accuracy: Optional[float] = None                 # metres
    accuracy_estimated: bool = False
    accuracy_source: Optional[str] = None
    hdop: Optional[float] = None
    vdop: Optional[float] = None
    pdop: Optional[float] = None

    position_source: Optional[str] = None
    clamped: bool = False
    estimated: bool = False

    image_width: Optional[int] = None
    image_height: Optional[int] = None
    panorama: bool = False
    pose_heading: Optional[float] = None
    pose_pitch: float = 0.0
    pose_roll: float = 0.0

    keywords: Optional[Sequence[str]] = None
    extra: dict = field(default_factory=dict)


def _fmt_local(value: Optional[dt.datetime]) -> Optional[str]:
    if value is None:
        return None
    return value.strftime("%Y:%m:%d %H:%M:%S")


def _fmt_utc(value: Optional[dt.datetime]) -> Optional[str]:
    if value is None:
        return None
    if value.tzinfo is None:
        value = value.replace(tzinfo=dt.timezone.utc)
    return (value.astimezone(dt.timezone.utc)
            .strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z")


def _split_utc(value: Optional[dt.datetime]):
    if value is None:
        return None, None, None
    if value.tzinfo is None:
        value = value.replace(tzinfo=dt.timezone.utc)
    value = value.astimezone(dt.timezone.utc)
    return (value.strftime("%Y:%m:%d"), value.strftime("%H:%M:%S"),
            _fmt_utc(value))


def provenance(meta: FrameMetadata) -> str:
    """Human-readable provenance (never a keyword tag)."""
    parts = [
        "%s via %s" % (meta.camera, meta.script_name),
        "video=%s" % meta.video,
        "frame=%d" % meta.frame_index,
        "pts=%.3fs" % meta.pts,
    ]
    if meta.distance is not None:
        parts.append("distance=%.2fm" % meta.distance)
    parts.append("heading_source=%s" % meta.heading_source)
    if meta.position_source:
        parts.append("position_source=%s" % meta.position_source)
    if meta.accuracy is not None:
        parts.append("accuracy=%.2fm%s" % (
            meta.accuracy, " (estimated)" if meta.accuracy_estimated else ""))
    if meta.hdop is not None:
        parts.append("hdop=%.2f" % meta.hdop)
    if meta.vdop is not None:
        parts.append("vdop=%.2f" % meta.vdop)
    if meta.pdop is not None:
        parts.append("pdop=%.2f" % meta.pdop)
    if meta.extra:
        for key, value in sorted(meta.extra.items()):
            if value is not None:
                parts.append("%s=%s" % (key, value))
    return " | ".join(parts)


def build_exiftool_args(meta: FrameMetadata) -> List[str]:
    """Return exiftool arguments (without the file name)."""
    tags: List[str] = ["-n", "-charset", "filename=utf8", "-Orientation=1"]

    local = meta.captured_local
    if local is None and meta.captured_utc is not None:
        utc = meta.captured_utc
        if utc.tzinfo is None:
            utc = utc.replace(tzinfo=dt.timezone.utc)
        local = utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
    stamped = _fmt_local(local)
    if stamped:
        tags += ["-DateTimeOriginal=" + stamped,
                 "-CreateDate=" + stamped,
                 "-ModifyDate=" + stamped,
                 "-SubSecTimeOriginal=%03d" % (local.microsecond // 1000)]
    if meta.utc_offset_s is not None:
        off = offset_string(meta.utc_offset_s)
        tags += ["-OffsetTimeOriginal=" + off, "-OffsetTime=" + off]

    # --- camera identity -------------------------------------------------
    if meta.make:
        tags.append("-Make=" + meta.make)
    if meta.model:
        tags.append("-Model=" + meta.model)
    if meta.software:
        tags.append("-Software=" + meta.software)
    if meta.serial:
        tags += ["-SerialNumber=%s" % meta.serial,
                 "-CameraSerialNumber=%s" % meta.serial]

    # --- EXIF GPS (UTC date/time, true-north directions) -----------------
    if meta.lat is not None and meta.lon is not None:
        tags += ["-GPSVersionID=2.3.0.0",
                 "-GPSLatitude=%r" % abs(meta.lat),
                 "-GPSLatitudeRef=%s" % ("N" if meta.lat >= 0 else "S"),
                 "-GPSLongitude=%r" % abs(meta.lon),
                 "-GPSLongitudeRef=%s" % ("E" if meta.lon >= 0 else "W")]
        if meta.alt is not None:
            tags += ["-GPSAltitude=%r" % abs(meta.alt),
                     "-GPSAltitudeRef#=%d" % (0 if meta.alt >= 0 else 1)]
        if meta.speed is not None:
            tags += ["-GPSSpeed=%r" % (meta.speed * 3.6), "-GPSSpeedRef=K"]
        if meta.img_direction is not None:
            tags += ["-GPSImgDirection=%r" % meta.img_direction,
                     "-GPSImgDirectionRef=True North"]
        if meta.course is not None:
            tags += ["-GPSTrack=%r" % meta.course,
                     "-GPSTrackRef=True North"]
        if meta.accuracy is not None:
            tags += ["-GPSHPositioningError=%r" % meta.accuracy,
                     "-XMP:GPSHPositioningError=%r" % meta.accuracy]
        date_stamp, time_stamp, gps_datetime = _split_utc(meta.captured_utc)
        if date_stamp:
            tags += ["-GPSDateStamp=" + date_stamp,
                     "-GPSTimeStamp=" + time_stamp,
                     "-GPSDateTime=" + gps_datetime]
        tags += ["-XMP:GPSLatitude=%r" % meta.lat,
                 "-XMP:GPSLongitude=%r" % meta.lon]
        if meta.alt is not None:
            tags += ["-XMP:GPSAltitude=%r" % abs(meta.alt),
                     "-XMP:GPSAltitudeRef#=%d" % (0 if meta.alt >= 0 else 1)]
        if meta.img_direction is not None:
            tags.append("-XMP:GPSImgDirection=%r" % meta.img_direction)
        if date_stamp:
            tags.append("-XMP:GPSDateTime=" + gps_datetime)

    # --- provenance (never a keyword tag) --------------------------------
    text = provenance(meta)
    tags += ["-ImageDescription=" + text,
             "-UserComment=" + text,
             "-XPComment=" + text,
             "-XMP:Description=" + text,
             "-XMP:CreatorTool=" + meta.script_name,
             "-XMP:Title=%s %s" % (meta.video,
                                   local.strftime("%Y-%m-%d %H:%M:%S")
                                   if local else "")]
    if local is not None:
        tags += ["-XMP:CreateDate=" + local.strftime("%Y:%m:%d %H:%M:%S"),
                 "-XMP:ModifyDate=" + local.strftime("%Y:%m:%d %H:%M:%S")]
    if meta.captured_utc is not None:
        tags.append("-XMP:MetadataDate=" + _fmt_utc(meta.captured_utc))

    # --- opt-in keywords --------------------------------------------------
    if meta.keywords:
        value = "; ".join(meta.keywords)
        tags += ["-Keywords=" + value, "-XMP:Subject=" + value]

    # --- XMP-GPano for panoramic images only ------------------------------
    if meta.panorama:
        width = meta.image_width or 0
        height = meta.image_height or 0
        heading = meta.pose_heading
        if heading is None:
            heading = meta.img_direction or 0.0
        first_date = _fmt_local(local) or _fmt_utc(meta.captured_utc)
        tags += [
            "-XMP-GPano:UsePanoramaViewer=True",
            "-XMP-GPano:CaptureSoftware=%s" % (meta.model or ""),
            "-XMP-GPano:StitchingSoftware=%s" % meta.script_name,
            "-XMP-GPano:ProjectionType=equirectangular",
            "-XMP-GPano:PoseHeadingDegrees=%r" % (heading % 360.0),
            "-XMP-GPano:PosePitchDegrees=%r" % meta.pose_pitch,
            "-XMP-GPano:PoseRollDegrees=%r" % meta.pose_roll,
            "-XMP-GPano:InitialViewHeadingDegrees=%r" % (heading % 360.0),
            "-XMP-GPano:InitialViewPitchDegrees=0",
            "-XMP-GPano:InitialViewRollDegrees=0",
            "-XMP-GPano:InitialHorizontalFOVDegrees=0",
            "-XMP-GPano:InitialVerticalFOVDegrees=0",
            "-XMP-GPano:CroppedAreaLeftPixels=0",
            "-XMP-GPano:CroppedAreaTopPixels=0",
            "-XMP-GPano:CroppedAreaImageWidthPixels=%d" % width,
            "-XMP-GPano:CroppedAreaImageHeightPixels=%d" % height,
            "-XMP-GPano:FullPanoWidthPixels=%d" % width,
            "-XMP-GPano:FullPanoHeightPixels=%d" % height,
            "-XMP-GPano:FirstPhotoDate=%s" % first_date,
            "-XMP-GPano:LastPhotoDate=%s" % first_date,
            "-XMP-GPano:SourcePhotosCount=2",
        ]
    return tags


def _parse_number(value) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text:
        return None
    if "/" in text:
        try:
            numerator, denominator = text.split("/", 1)
            return float(numerator) / float(denominator)
        except (ValueError, ZeroDivisionError):
            return None
    match = re.match(r"[-+]?\d+(?:\.\d+)?", text)
    return float(match.group(0)) if match else None


def accuracy_from_tags(tags: dict) -> dict:
    """Extract accuracy/DOP values from EXIF/XMP-style tag dictionaries.

    Recognised keys are matched case-insensitively and may be prefixed with
    ``EXIF:``/``XMP:``.  Returns ``{"accuracy", "accuracy_source", "hdop",
    "vdop", "pdop"}`` with ``None`` for absent values.
    """
    folded = {str(key).lower().split(":")[-1]: value
              for key, value in (tags or {}).items()}
    accuracy = None
    for key in ("gpshpositioningerror", "gpspositioningerror",
                "positioningerror"):
        if key in folded:
            accuracy = _parse_number(folded[key])
            if accuracy is not None:
                break
    return {
        "accuracy": accuracy,
        "accuracy_source": "exif" if accuracy is not None else None,
        "hdop": _parse_number(folded.get("gpshdop")),
        "vdop": _parse_number(folded.get("gpsvdop")),
        "pdop": _parse_number(folded.get("gpsdop")
                              or folded.get("gpspdop")),
    }

# =========================================================================
# BlackVue DR900 pipeline
# =========================================================================

blackvue_KNOTS_TO_MPS = 0.514444
blackvue_DEFAULT_MODEL = "DR900"
blackvue_DEFAULT_MAKE = "BlackVue"

blackvue__RE_VIDEO_NAME = re.compile(r"^(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})_[A-Za-z]{2}\.")
blackvue__RE_NMEA = re.compile(rb"\[(\d{10,})\]\$([A-Z]{2}[A-Z]{3}),([^\r\n*]*)")




# ===========================================================================
# MP4 container inspection (camera identity + NMEA telemetry)
# ===========================================================================

def blackvue__iter_top_atoms(path):
    """Yield ``(type, offset, payload_size)`` for the top-level MP4 atoms."""
    size = os.path.getsize(path)
    with open(path, "rb") as fh:
        pos = 0
        while pos + 8 <= size:
            fh.seek(pos)
            head = fh.read(8)
            if len(head) < 8:
                break
            atom_size, atom_type = struct.unpack(">I4s", head)
            header = 8
            if atom_size == 1:
                ext = fh.read(8)
                if len(ext) < 8:
                    break
                atom_size = struct.unpack(">Q", ext)[0]
                header = 16
            elif atom_size == 0:
                atom_size = size - pos
            if atom_size < header or pos + atom_size > size:
                break
            yield atom_type, pos + header, atom_size - header
            pos += atom_size


def blackvue_read_cprt(path):
    """Read the BlackVue ``cprt`` copyright atom.

    Returns a dict with ``make``, ``model``, ``firmware`` and ``serial``; the
    fields are ``;``-separated, e.g.
    ``Pittasoft Co., Ltd.;DR900M-1CH;0.901;English;1;D90SS1HAE00736;...``.
    """
    info = {"make": None, "model": None, "firmware": None, "serial": None}
    try:
        with open(path, "rb") as fh:
            for atom_type, offset, size in blackvue__iter_top_atoms(path):
                if size < 4 or size > 1 << 20:
                    continue
                fh.seek(offset)
                payload = fh.read(size)
                if b"Pittasoft" not in payload and b"DR900" not in payload:
                    continue
                start = payload.find(b"Pittasoft")
                if start < 0:
                    start = payload.find(b"DR900")
                text = payload[start:].split(b"\x00", 1)[0]
                text = text.decode("utf-8", "replace").strip()
                parts = [p.strip() for p in text.split(";")]
                if len(parts) >= 3:
                    info["make"] = blackvue_DEFAULT_MAKE
                    info["model"] = parts[1] or None
                    info["firmware"] = parts[2] or None
                if len(parts) >= 6:
                    info["serial"] = parts[5] or None
                if info["model"]:
                    return info
    except OSError:
        pass
    return info


def blackvue_extract_nmea(path, max_payload=8 * 1024 * 1024):
    """Return ``[(bracket_ms, sentence_type, [fields]), ...]`` from *path*.

    NMEA lives in top-level ``free`` atoms; each recording may carry the same
    log twice (a duplicate in the trailing concatenated segment), so identical
    sentences are de-duplicated while keeping the original order.
    """
    sentences = []
    seen = set()
    payloads = []
    try:
        with open(path, "rb") as fh:
            for atom_type, offset, size in blackvue__iter_top_atoms(path):
                if atom_type != b"free" or size == 0:
                    continue
                fh.seek(offset)
                payloads.append(fh.read(min(size, max_payload)))
    except OSError:
        return sentences

    for payload in payloads:
        for match in blackvue__RE_NMEA.finditer(payload):
            key = match.group(0)
            if key in seen:
                continue
            seen.add(key)
            kind = match.group(2).decode("ascii", "replace")
            body = match.group(3).decode("ascii", "replace")
            sentences.append((int(match.group(1)), kind,
                              [kind] + body.split(",")))
    if not sentences:
        # Fall back to a bounded head+tail raw scan for unusual layouts.
        try:
            size = os.path.getsize(path)
            with open(path, "rb") as fh:
                blob = fh.read(min(size, max_payload))
                if size > max_payload:
                    fh.seek(size - max_payload)
                    blob += fh.read(max_payload)
            for match in blackvue__RE_NMEA.finditer(blob):
                key = match.group(0)
                if key in seen:
                    continue
                seen.add(key)
                kind = match.group(2).decode("ascii", "replace")
                body = match.group(3).decode("ascii", "replace")
                sentences.append((int(match.group(1)), kind,
                                  [kind] + body.split(",")))
        except OSError:
            pass
    return sentences


# ===========================================================================
# NMEA parsing
# ===========================================================================

def blackvue__to_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def blackvue__parse_time(text):
    """``hhmmss.ss`` -> (hour, minute, second_float) or None."""
    if not text or len(text) < 6:
        return None
    try:
        return int(text[0:2]), int(text[2:4]), float(text[4:])
    except ValueError:
        return None


def blackvue__parse_date(text):
    """``ddmmyy`` -> (year, month, day) or None."""
    if not text or len(text) < 6:
        return None
    try:
        day, month, year = int(text[0:2]), int(text[2:4]), int(text[4:6])
    except ValueError:
        return None
    return 2000 + year, month, day


def blackvue__nmea_decimal(value, hemisphere):
    """``ddmm.mmmm`` + ``N``/``S``/``E``/``W`` -> signed degrees."""
    number = blackvue__to_float(value)
    if number is None:
        return None
    degrees = int(number // 100)
    minutes = number - degrees * 100
    result = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        result = -result
    return result


def blackvue__parse_gga(fields):
    # $GPGGA,time,lat,N,lon,E,fix,sats,hdop,alt,M,geoid,M,age,station
    if len(fields) < 10:
        return None
    return {
        "time": blackvue__parse_time(fields[1]),
        "lat": blackvue__nmea_decimal(fields[2], fields[3]) if len(fields) > 3 else None,
        "lon": blackvue__nmea_decimal(fields[4], fields[5]) if len(fields) > 5 else None,
        "fix": int(fields[6]) if fields[6].isdigit() else 0,
        "sats": int(fields[7]) if len(fields) > 7 and fields[7].isdigit() else None,
        "hdop": blackvue__to_float(fields[8]) if len(fields) > 8 else None,
        "alt": blackvue__to_float(fields[9]) if len(fields) > 9 else None,
    }


def blackvue__parse_gsa(fields):
    # $GPGSA,mode,fix,sv,sv,...,pdop,hdop,vdop
    if len(fields) < 18:
        return None
    return {
        "fix": int(fields[2]) if len(fields) > 2 and fields[2].isdigit() else None,
        "pdop": blackvue__to_float(fields[15]),
        "hdop": blackvue__to_float(fields[16]),
        "vdop": blackvue__to_float(fields[17]),
    }


def blackvue__parse_rmc(fields):
    # $GPRMC,time,status,lat,N,lon,E,speed_knots,course,date,...
    if len(fields) < 10:
        return None
    return {
        "time": blackvue__parse_time(fields[1]),
        "status": fields[2],
        "lat": blackvue__nmea_decimal(fields[3], fields[4]) if len(fields) > 4 else None,
        "lon": blackvue__nmea_decimal(fields[5], fields[6]) if len(fields) > 6 else None,
        "speed_knots": blackvue__to_float(fields[7]) if len(fields) > 7 else None,
        "course": blackvue__to_float(fields[8]) if len(fields) > 8 else None,
        "date": blackvue__parse_date(fields[9]) if len(fields) > 9 else None,
    }


def blackvue__parse_vtg(fields):
    # $GPVTG,course_true,T,course_mag,M,speed_knots,N,speed_kmh,K,mode
    if len(fields) < 8:
        return None
    return {
        "course": blackvue__to_float(fields[1]),
        "speed_knots": blackvue__to_float(fields[5]) if len(fields) > 5 else None,
        "speed_kmh": blackvue__to_float(fields[7]) if len(fields) > 7 else None,
    }


def blackvue_parse_telemetry(sentences, video_start_local):
    """Turn raw NMEA sentences into GPS samples for ``Track``.

    Returns ``(samples, tz_offset_s)``.  Each sample is a dict with ``t``
    (video-relative seconds), ``lat``, ``lon``, ``alt``, ``speed``, ``course``,
    DOP fields and ``utc`` (timezone-aware UTC).  ``tz_offset_s`` is the
    quarter-hour snapped camera-local minus UTC offset, or ``None`` when it
    cannot be derived.
    """
    records = []
    current = None
    current_bracket = None
    for bracket, kind, fields in sentences:
        if bracket != current_bracket:
            if current is not None:
                records.append(current)
            current = {"bracket": bracket, "gga": None, "rmc": None,
                       "vtg": None, "gsa": None}
            current_bracket = bracket
        if kind.endswith("GGA"):
            current["gga"] = blackvue__parse_gga(fields)
        elif kind.endswith("RMC"):
            current["rmc"] = blackvue__parse_rmc(fields)
        elif kind.endswith("VTG"):
            current["vtg"] = blackvue__parse_vtg(fields)
        elif kind.endswith("GSA"):
            current["gsa"] = blackvue__parse_gsa(fields)
    if current is not None:
        records.append(current)

    # Relative times need an anchor: the file-name start when available,
    # otherwise the first logged fix.  Both come from the camera's own clock so
    # the differences (and therefore every ``t``) stay exact either way.
    if video_start_local is not None:
        anchor = video_start_local
    elif records:
        anchor = dt.datetime.fromtimestamp(
            records[0]["bracket"] / 1000.0, dt.timezone.utc).replace(tzinfo=None)
    else:
        anchor = dt.datetime(1970, 1, 1)

    samples = []
    last_date = None
    for record in records:
        rmc = record["rmc"]
        gga = record["gga"]
        vtg = record["vtg"]
        gsa = record["gsa"]
        if rmc and rmc.get("date"):
            last_date = rmc["date"]
        date = last_date or (video_start_local.date()
                             if video_start_local is not None else None)

        lat = lon = alt = speed = course = None
        utc = None
        if rmc and rmc.get("status") == "A" and rmc.get("lat") is not None:
            lat, lon = rmc["lat"], rmc["lon"]
            if rmc.get("speed_knots") is not None:
                speed = rmc["speed_knots"] * blackvue_KNOTS_TO_MPS
            course = rmc.get("course")
            if rmc.get("time") and date is not None:
                utc = blackvue__utc_from(date, rmc["time"])
        elif gga and gga.get("fix", 0) and gga.get("lat") is not None:
            lat, lon = gga["lat"], gga["lon"]
            if gga.get("time") and date is not None:
                utc = blackvue__utc_from(date, gga["time"])
        else:
            continue

        if gga and gga.get("alt") is not None:
            alt = gga["alt"]
        if speed is None and vtg and vtg.get("speed_knots") is not None:
            speed = vtg["speed_knots"] * blackvue_KNOTS_TO_MPS
        if course is None and vtg and vtg.get("course") is not None:
            course = vtg["course"]
        if utc is None:
            continue
        if lat is None or lon is None or not (-90 <= lat <= 90)\
                or not (-180 <= lon <= 180) or (lat == 0.0 and lon == 0.0):
            continue

        local_wall = dt.datetime.fromtimestamp(
            record["bracket"] / 1000.0, dt.timezone.utc).replace(tzinfo=None)
        t = (local_wall - anchor).total_seconds()
        hdop = vdop = pdop = None
        if gsa:
            hdop, vdop, pdop = gsa.get("hdop"), gsa.get("vdop"), gsa.get("pdop")
        if hdop is None and gga:
            hdop = gga.get("hdop")
        samples.append({
            "t": t,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "speed": speed,
            "course": course,
            "hdop": hdop,
            "vdop": vdop,
            "pdop": pdop,
            "satellites": gga.get("sats") if gga else None,
            "utc": utc,
            "_local_wall": local_wall,
        })

    if not samples:
        return [], None
    samples.sort(key=lambda s: s["t"])

    deltas = [s["_local_wall"].replace(tzinfo=dt.timezone.utc).timestamp()
              - s["utc"].timestamp() for s in samples]
    median = statistics.median(deltas)
    tz_offset = int(round(median / 900.0) * 900)
    return samples, tz_offset


def blackvue__utc_from(date, time):
    year, month, day = date
    hour, minute, second = time
    whole = int(second)
    micro = int(round((second - whole) * 1_000_000))
    if micro >= 1_000_000:
        whole += 1
        micro -= 1_000_000
    try:
        return dt.datetime(year, month, day, hour, minute, whole, micro,
                           tzinfo=dt.timezone.utc)
    except ValueError:
        return None


def blackvue_parse_video_start_local(path):
    """Recording start time from the file name (``YYYYMMDD_HHMMSS_XX``)."""
    match = blackvue__RE_VIDEO_NAME.match(os.path.basename(path))
    if not match:
        return None
    try:
        return dt.datetime(*(int(x) for x in match.groups()))
    except ValueError:
        return None


# ===========================================================================
# Video probing
# ===========================================================================

def blackvue_probe_video(path):
    """Return fps, frame count, duration, dimensions and codec via ffprobe."""
    try:
        proc = subprocess.run(
            ["ffprobe", "-v", "error", "-print_format", "json",
             "-show_format", "-show_streams", path],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        raise Fatal("required program not found: ffprobe")
    if proc.returncode != 0:
        tail = proc.stderr.decode("utf-8", "replace").strip().splitlines()
        raise Fatal(tail[-1] if tail else "ffprobe failed")
    try:
        data = json.loads(proc.stdout.decode("utf-8", "replace"))
    except ValueError:
        raise Fatal("ffprobe returned unreadable output for %s" % path)

    video = None
    for stream in data.get("streams", []):
        if stream.get("codec_type") == "video":
            video = stream
            break
    if video is None:
        raise Fatal("no video stream in %s (not a playable video file)" % path)

    rate = video.get("avg_frame_rate") or video.get("r_frame_rate") or "0/0"
    try:
        num, den = rate.split("/")
        fps = float(num) / float(den) if float(den) else 0.0
    except (ValueError, ZeroDivisionError):
        fps = 0.0

    duration = None
    if video.get("duration"):
        duration = float(video["duration"])
    elif data.get("format", {}).get("duration"):
        duration = float(data["format"]["duration"])

    return {
        "fps": fps,
        "frames": int(video.get("nb_frames") or 0),
        "duration": duration,
        "width": video.get("width"),
        "height": video.get("height"),
        "codec": video.get("codec_name"),
        "creation_time": data.get("format", {}).get("tags", {}).get("creation_time"),
    }


# ===========================================================================
# Records, metadata and per-video processing
# ===========================================================================

def blackvue__record_from_choice(choice, args, video_start_utc, video_start_local,
                        tz_offset, tz_known, camera, sequence, video_name):
    point = choice.point or {}
    pts = max(0.0, float(choice.pts))
    captured_utc = (video_start_utc + dt.timedelta(seconds=pts)
                    if video_start_utc is not None else None)
    captured_local = None
    if video_start_local is not None:
        captured_local = video_start_local + dt.timedelta(seconds=pts)
    elif tz_known and captured_utc is not None:
        captured_local = local_from_utc(captured_utc, tz_offset)
    if args.time_base == "utc" and captured_utc is not None:
        naming = captured_utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
    else:
        naming = captured_local
        if naming is None and captured_utc is not None:
            naming = captured_utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
        if naming is None and video_start_local is not None:
            naming = video_start_local + dt.timedelta(seconds=pts)
    if naming is None:
        naming = dt.datetime(1970, 1, 1)

    heading, heading_source = resolve_heading(
        point, args.heading_offset, args.magnetic_declination)
    accuracy, accuracy_estimated, accuracy_source = derive_accuracy(
        point, args.uere)
    course = point.get("course")
    if course is None:
        course = point.get("gps_course")
    speed = point.get("speed")
    if speed is None:
        speed = point.get("speed2d")

    filename = image_filename(naming, args.format)
    return {
        "sequence": sequence, "video": video_name, "camera": camera,
        "image": filename, "frame_index": int(choice.frame_index),
        "pts_seconds": round(pts, 3),
        "datetime_local": captured_local.isoformat() if captured_local else None,
        "datetime_utc": fmt_utc(captured_utc),
        "utc_offset_s": tz_offset if tz_known else None,
        "latitude": point.get("lat"), "longitude": point.get("lon"),
        "altitude_m": point.get("alt"), "speed_m_s": speed,
        "course_deg": course, "heading_deg": heading,
        "heading_source": heading_source, "img_direction_deg": heading,
        "accuracy_m": accuracy, "accuracy_estimated": accuracy_estimated,
        "accuracy_source": accuracy_source,
        "hdop": point.get("hdop"), "vdop": point.get("vdop"),
        "pdop": point.get("pdop"),
        "distance_m": float(choice.target_distance),
        "track_distance_m": point.get("distance"),
        "position_source": point.get("position_source", "none"),
        "clamped": bool(point.get("clamped")),
        "estimated": bool(point.get("estimated")),
        "naming_time": naming, "point": point,
    }


def blackvue__unpositioned_choices(timeline, interval, duration, max_images):
    frames = select_time_grid_frames(
        timeline, interval, duration=duration, max_images=max_images)
    out = []
    for index in frames:
        pts = timeline.pts(index)
        out.append(FrameChoice(
            target_distance=pts, target_time=pts, frame_index=index, pts=pts,
            point={}, position_error=None, clamped=False, estimated=True))
    return out


def blackvue__metadata_for(record, source, args, camera):
    return FrameMetadata(
        camera=CAMERA_LABELS[camera], video=record["video"],
        sequence=record["sequence"], frame_index=record["frame_index"],
        pts=record["pts_seconds"], distance=record.get("distance_m"),
        script_name=SCRIPT_NAME, make=source.get("make"),
        model=source.get("model"), software=source.get("software"),
        serial=source.get("serial"),
        captured_utc=parse_utc_timestamp(record.get("datetime_utc")),
        captured_local=(dt.datetime.fromisoformat(record["datetime_local"])
                        if record.get("datetime_local") else None),
        utc_offset_s=record.get("utc_offset_s"),
        lat=record.get("latitude"), lon=record.get("longitude"),
        alt=record.get("altitude_m"), speed=record.get("speed_m_s"),
        course=record.get("course_deg"), img_direction=record.get("img_direction_deg"),
        heading_source=record.get("heading_source", "none"),
        accuracy=record.get("accuracy_m"),
        accuracy_estimated=record.get("accuracy_estimated", False),
        accuracy_source=record.get("accuracy_source"),
        hdop=record.get("hdop"), vdop=record.get("vdop"),
        pdop=record.get("pdop"),
        position_source=record.get("position_source"),
        clamped=record.get("clamped"), estimated=record.get("estimated"),
        keywords=parse_keywords(getattr(args, "keywords", None)),
        extra={"firmware": source.get("firmware")},
    )


def blackvue__write_sidecar(path, sequence, video_name, records, source, args):
    meta = {
        "generator": SCRIPT_NAME, "sequence": sequence,
        "source_video": video_name,
        "camera": {"make": source.get("make"), "model": source.get("model"),
                   "serial": source.get("serial"),
                   "software": source.get("software")},
        "distance_interval_m": args.distance,
        "heading_offset_deg": args.heading_offset,
        "time_base": args.time_base,
        "utc_offset_s": source.get("utc_offset_s"),
        "image_count": len(records),
    }
    with open(path, "w") as fh:
        fh.write(json.dumps({"_meta": meta}) + "\n")
        for record in records:
            row = {k: v for k, v in record.items()
                   if k not in ("point", "naming_time")}
            fh.write(json.dumps(row) + "\n")


def blackvue_process_video(video, sequence, output_dir, args, have_exiftool):
    """Export one video sequence. Returns a summary dict; never raises Fatal."""
    name = os.path.basename(video)
    LOG.info("--- %s -> %s", name, sequence)

    info = blackvue_probe_video(video)
    LOG.info("  video: %sx%s %s %.3f fps, %s frames, %.2f s",
             info["width"], info["height"], info["codec"], info["fps"],
             info["frames"], info["duration"] or 0.0)
    timeline = probe_frame_timeline(
        video, fps_hint=info["fps"], frame_count_hint=info["frames"],
        duration_hint=info["duration"], logger=LOG)
    LOG.info("  frame timeline: %d frame(s), source=%s",
             len(timeline), timeline.source)

    cprt = blackvue_read_cprt(video)
    source = {
        "video": name,
        "make": cprt.get("make") or blackvue_DEFAULT_MAKE,
        "model": cprt.get("model") or blackvue_DEFAULT_MODEL,
        "firmware": cprt.get("firmware"),
        "serial": cprt.get("serial"),
    }

    filename_start = blackvue_parse_video_start_local(video)
    if filename_start is None:
        LOG.warning("  file name does not carry a start time; deriving it from "
                    "the GPS log")

    sentences = blackvue_extract_nmea(video)
    LOG.debug("  %d NMEA sentence(s) found", len(sentences))
    samples, detected_tz = blackvue_parse_telemetry(sentences, filename_start)

    tz_offset, tz_known = resolve_timezone(
        args, detected_tz, camera=CAMERA_BLACKVUE, logger=LOG)
    if tz_known:
        LOG.info("  camera-local time offset: %s", offset_string(tz_offset))

    video_start_utc = None
    video_start_local = filename_start
    if samples:
        first = samples[0]
        video_start_utc = first["utc"] - dt.timedelta(seconds=first["t"])
        if tz_known:
            video_start_local = local_from_utc(video_start_utc, tz_offset)
            if filename_start is not None:
                skew = (video_start_local - filename_start).total_seconds()
                if abs(skew) >= 1.0:
                    LOG.info("  camera clock is %+.0f s off GPS; times follow GPS",
                             skew)
        LOG.info("  telemetry: %d GPS samples", len(samples))
        LOG.info("  video start (UTC): %s", fmt_utc(video_start_utc))
    else:
        LOG.warning("  no GPS data in this video")
        if filename_start is not None and tz_known:
            video_start_utc = (filename_start.replace(tzinfo=dt.timezone.utc)
                               - dt.timedelta(seconds=tz_offset))

    track = None
    if samples:
        for sample in samples:
            apply_accuracy(sample, args.uere)
        track = Track(samples, args.distance_mode)
        LOG.info("  track: %d sample(s), %.0f m of travel",
                 len(samples), track.length)

    choices = []
    skipped = []
    if track is not None:
        choices, skipped = select_unique_frames(
            track, timeline, args.distance,
            start_distance=args.start_distance,
            max_position_error=args.max_position_error,
            max_images=args.max_images, mode=args.distance_mode, logger=LOG)
        if skipped:
            LOG.info("  skipped %d target(s) with no acceptable unused frame",
                     len(skipped))

    if not choices:
        if args.fallback_interval > 0 and info["duration"]:
            choices = blackvue__unpositioned_choices(
                timeline, args.fallback_interval, info["duration"],
                args.max_images)
            LOG.warning("  falling back to a %g s time grid (%d images, no position)",
                        args.fallback_interval, len(choices))
        else:
            if samples:
                LOG.error("  no distance-based capture points; skipping this video")
            else:
                LOG.error("  no GPS data and no fallback requested; skipping this video")
            return {"video": name, "sequence": sequence, "images": 0,
                    "planned": 0, "skipped": 0, "failed": True}

    source["software"] = "BlackVue %s FW %s" % (
        source["model"], source["firmware"] or "unknown")
    source["utc_offset_s"] = tz_offset if tz_known else None

    records = [blackvue__record_from_choice(
        choice, args, video_start_utc, video_start_local, tz_offset, tz_known,
        CAMERA_BLACKVUE, sequence, name) for choice in choices]

    first_time = records[0]["naming_time"]
    sequence_dir = sequence_output_dir(
        output_dir, CAMERA_BLACKVUE, sequence, first_time,
        heading_offset=heading_offset_for_folder(args))
    used_paths = set()
    for record in records:
        record["path"] = unique_path(
            os.path.join(sequence_dir, record["image"]), used_paths)
        record["image"] = os.path.basename(record["path"])

    LOG.info("  %d capture point(s) -> %s", len(records), sequence_dir)
    if args.dry_run:
        LOG.info("  dry run: %d image(s) would be written", len(records))
        return {"video": name, "sequence": sequence, "images": 0,
                "planned": len(records), "skipped": 0, "dir": sequence_dir}

    to_extract = []
    skipped_existing = 0
    for record in records:
        if os.path.exists(record["path"]) and not args.overwrite:
            LOG.debug("  exists, skipping: %s", record["image"])
            skipped_existing += 1
            continue
        to_extract.append(record)

    written_paths = []
    failed_frames = []
    temp_dir = tempfile.mkdtemp(prefix="blackvue_")
    try:
        extracted = extract_frames_by_index(
            video, [r["frame_index"] for r in to_extract], temp_dir,
            image_format=args.format, quality=args.quality,
            workers=args.workers, threads=args.threads, timeline=timeline)
        for record in to_extract:
            temp_path = extracted.get(record["frame_index"])
            if not temp_path:
                failed_frames.append(record["frame_index"])
                continue
            os.makedirs(os.path.dirname(record["path"]), exist_ok=True)
            shutil.copyfile(temp_path, record["path"])
            record["written"] = True
            written_paths.append(record["path"])
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)

    if (args.start_distance == 0 and records
            and records[0]["frame_index"] == 0
            and records[0]["frame_index"] in failed_frames):
        LOG.error("  first frame (index 0) could not be extracted; failing %s",
                  name)
        return {"video": name, "sequence": sequence, "images": 0,
                "planned": len(records), "skipped": skipped_existing,
                "failed": True, "dir": sequence_dir}

    LOG.info("  wrote %d image(s), skipped %d existing%s",
             len(written_paths), skipped_existing,
             (", %d failed" % len(failed_frames)) if failed_frames else "")

    if have_exiftool and not args.no_exif and written_paths:
        pairs = [(record["path"], build_exiftool_args(
            blackvue__metadata_for(record, source, args, CAMERA_BLACKVUE)))
            for record in records if record.get("written")]
        write_metadata(pairs)

    export_samples = track.samples if track is not None else []
    export_track_files(
        sequence_dir, sequence, export_samples, records,
        track_export=args.track_export, camera=CAMERA_BLACKVUE, video=name,
        video_start_utc=video_start_utc,
        telemetry_export=args.telemetry_export,
        extra={"distance_interval_m": args.distance,
               "time_base": args.time_base})

    if args.sidecars:
        os.makedirs(sequence_dir, exist_ok=True)
        blackvue__write_sidecar(os.path.join(sequence_dir, sequence + ".track.jsonl"),
                       sequence, name, records, source, args)

    entries = [{k: v for k, v in record.items()
                if k not in ("point", "naming_time")}
               for record in records if record.get("written")]
    return {"video": name, "sequence": sequence, "images": len(written_paths),
            "planned": len(records), "skipped": skipped_existing,
            "entries": entries, "dir": sequence_dir}


# =========================================================================
# GoPro MAX 360 pipeline
# =========================================================================

max360_HEMI_W, max360_HEMI_H = 4096, 1344
max360_DEFAULT_WIDTH, max360_DEFAULT_HEIGHT = 5376, 2688




class max360_BackendError(Fatal):
    """ffmpeg/ffprobe/max2sphere failure."""


# ---------------------------------------------------------------------------
# Probing
# ---------------------------------------------------------------------------
@dataclass
class max360_VideoInfo:
    path: str
    duration: float
    fps: float
    frame_count: int
    video_ordinals: Dict[int, int]      # absolute index -> v:N ordinal
    hemi_a: int                         # absolute stream index
    hemi_b: Optional[int]
    gpmd: Optional[int]
    width: int                          # native hemisphere width
    height: int                         # native hemisphere height
    creation_time: Optional[dt.datetime]
    already_equirect: bool = False
    firmware: Optional[str] = None
    make: str = "GoPro"
    model: str = "GoPro Max"
    serial: Optional[str] = None


def max360__parse_rate(rate) -> float:
    if not rate:
        return 0.0
    text = str(rate)
    if "/" in text:
        try:
            num, den = text.split("/", 1)
            return float(num) / float(den) if float(den) else 0.0
        except ValueError:
            return 0.0
    try:
        return float(text)
    except ValueError:
        return 0.0


def max360__frame_count(stream: dict, duration: float, fps: float) -> int:
    try:
        count = int(stream.get("nb_frames"))
        if count > 0:
            return count
    except (TypeError, ValueError):
        pass
    return max(1, int(round(duration * fps))) if duration and fps else 1


def max360_probe(path: str) -> max360_VideoInfo:
    """Inspect a GoPro MAX file and describe its hemisphere/telemetry layout."""
    meta = probe_metadata(path, FFPROBE)
    videos = meta.video_streams_info()
    if not videos:
        raise max360_BackendError("%s: no video stream" % os.path.basename(path))
    fmt_duration = meta.duration or 0.0
    creation = parse_utc_timestamp(meta.creation_time)

    if len(videos) >= 2:
        # Prefer the largest pair of equally-sized video streams; sorted by
        # absolute index so the top hemisphere comes first.
        pairs = []
        for i in range(len(videos)):
            for j in range(i + 1, len(videos)):
                a, b = videos[i], videos[j]
                if (a.get("width") == b.get("width")
                        and a.get("height") == b.get("height")):
                    area = int(a.get("width") or 0) * int(a.get("height") or 0)
                    pairs.append((area, a, b))
        if not pairs:
            raise max360_BackendError(
                "%s: found %d video streams but no matching EAC hemisphere pair"
                % (os.path.basename(path), len(videos)))
        _area, hemi_a, hemi_b = max(pairs, key=lambda item: item[0])
        if hemi_a.get("index", 0) > hemi_b.get("index", 0):
            hemi_a, hemi_b = hemi_b, hemi_a
        width = int(hemi_a.get("width") or max360_HEMI_W)
        height = int(hemi_a.get("height") or max360_HEMI_H)
        fps = max360__parse_rate(hemi_a.get("avg_frame_rate")
                          or hemi_a.get("r_frame_rate")) or 30000.0 / 1001.0
        duration = float(hemi_a.get("duration") or fmt_duration or 0.0)
        count_a = max360__frame_count(hemi_a, duration, fps)
        count_b = max360__frame_count(hemi_b, duration, fps)
        frame_count = min(count_a, count_b)
        a_idx, b_idx = int(hemi_a["index"]), int(hemi_b["index"])
        already = False
    elif len(videos) == 1:
        stream = videos[0]
        width = int(stream.get("width") or 0)
        height = int(stream.get("height") or 0)
        aspect = (width / height) if height else 0.0
        if aspect < 1.85:
            raise max360_BackendError(
                "%s: one %dx%d video stream is not an EAC hemisphere pair or "
                "an equirectangular frame" % (os.path.basename(path), width,
                                              height))
        hemi_a = int(stream["index"])
        hemi_b = None
        fps = max360__parse_rate(stream.get("avg_frame_rate")
                          or stream.get("r_frame_rate")) or 30000.0 / 1001.0
        duration = float(stream.get("duration") or fmt_duration or 0.0)
        frame_count = max360__frame_count(stream, duration, fps)
        a_idx, b_idx = hemi_a, None
        already = True
    else:
        raise max360_BackendError("%s: no video stream" % os.path.basename(path))

    ordinals = {}
    for ordinal, stream in enumerate(videos):
        try:
            ordinals[int(stream["index"])] = ordinal
        except (KeyError, TypeError, ValueError):
            pass
    return max360_VideoInfo(
        path=path, duration=duration, fps=fps, frame_count=frame_count,
        video_ordinals=ordinals, hemi_a=a_idx, hemi_b=b_idx, gpmd=meta.gpmd_index,
        width=width, height=height, creation_time=creation,
        already_equirect=already,
        firmware=meta.format_tags.get("firmware"),
        serial=meta.format_tags.get("serial_number") or meta.format_tags.get("serial"),
    )


# ---------------------------------------------------------------------------
# EAC stitching filter
# ---------------------------------------------------------------------------
# Known GoPro MAX hemisphere layouts: (strip width, strip height) ->
# (side width, centre width, blend width).  A strip is side + centre + side
# with no gaps; each side region contains its own 2*blend overlap seam.
max360_EAC_LAYOUTS = {
    (4096, 1344): (1376, 1344, 32),
    (2272, 736): (768, 736, 16),
}


def max360__eac_layout(width: int, height: int) -> Tuple[int, int, int]:
    """Return ``(side, centre, blend)`` for a native hemisphere strip."""
    key = (int(width), int(height))
    if key in max360_EAC_LAYOUTS:
        return max360_EAC_LAYOUTS[key]
    # Unknown layout: assume the same equal-gap cube net as the known strips.
    gap = (int(width) - 3 * int(height)) // 2
    if gap < 2 or int(width) != 3 * int(height) + 2 * gap:
        raise max360_BackendError("unsupported EAC hemisphere layout %dx%d"
                           % (width, height))
    return int(height) + gap, int(height), gap


def max360_build_eac_filter(a: int, b: Optional[int], width: int, height: int,
                     select: Optional[str] = None,
                     interp: str = "cubic",
                     already_equirect: bool = False,
                     hemi_width: int = max360_HEMI_W,
                     hemi_height: int = max360_HEMI_H) -> str:
    """Build the ffmpeg filter graph for one EAC hemisphere pair.

    A native 4096x1344 strip is side 1376 + centre 1344 + side 1376 with no
    gaps.  The outer halves of the left/right side regions overlap by 32 px,
    so they must be cross-blended instead of simply cropped (max2sphere does
    the same).  The six faces are assembled into a 3x2 EAC net and v360
    reprojects it to equirectangular.  The 2272x736 (side 768, centre 736,
    blend 16) layout is handled by the same code.
    """
    lines: List[str] = []
    if already_equirect:
        if select:
            lines.append("[0:%d]select='%s'[src]" % (a, select))
        else:
            lines.append("[0:%d]null[src]" % a)
        lines.append("[src]scale=%d:%d[OUT]" % (width, height))
        return ";\n".join(lines)

    if b is None:
        raise max360_BackendError("EAC stitching needs two hemisphere streams")
    side, centre, blend = max360__eac_layout(hemi_width, hemi_height)
    unique_src = side // 2 - 2 * blend      # one outer half of a side region
    blend_chunk = 2 * blend                 # width of each overlap crop
    out_unique = (centre - blend_chunk) // 2
    if (unique_src <= 0 or blend_chunk <= 1 or out_unique <= 0
            or 2 * out_unique + blend_chunk != centre):
        raise max360_BackendError("unsupported EAC hemisphere layout %dx%d"
                           % (hemi_width, hemi_height))

    for stream, tag in ((a, "A"), (b, "B")):
        if select:
            head = "[0:%d]select='%s',format=rgb24" % (stream, select)
        else:
            head = "[0:%d]format=rgb24" % stream
        lines.append("%s,split=3[%sL][%sC][%sR]"
                     % (head, tag, tag, tag))
        lines.append("[%sL]split=4[%sL0][%sL1][%sL2][%sL3]"
                     % (tag, tag, tag, tag, tag))
        lines.append("[%sR]split=4[%sR0][%sR1][%sR2][%sR3]"
                     % (tag, tag, tag, tag, tag))

    def fit(source: str, destination: str) -> None:
        if unique_src == out_unique:
            lines.append("[%s]null[%s]" % (source, destination))
        else:
            lines.append("[%s]scale=%d:%d:flags=lanczos[%s]"
                         % (source, out_unique, centre, destination))

    def side_face(name: str, pads: Sequence[str], x0: int,
                  near_first: bool) -> None:
        """Assemble one left/right side face with a 2*blend overlap seam."""
        near_x = x0 if near_first else x0 + unique_src + 2 * blend_chunk
        far_x = (x0 + unique_src + 2 * blend_chunk
                 if near_first else x0)
        ov1_x = x0 + unique_src
        ov2_x = x0 + unique_src + blend_chunk
        lines.append("[%s]crop=%d:%d:%d:0[%s_near_src]"
                     % (pads[0], unique_src, hemi_height, near_x, name))
        lines.append("[%s]crop=%d:%d:%d:0[%s_ov1]"
                     % (pads[1], blend_chunk, hemi_height, ov1_x, name))
        lines.append("[%s]crop=%d:%d:%d:0[%s_ov2]"
                     % (pads[2], blend_chunk, hemi_height, ov2_x, name))
        lines.append("[%s]crop=%d:%d:%d:0[%s_far_src]"
                     % (pads[3], unique_src, hemi_height, far_x, name))
        fit("%s_near_src" % name, "%s_near" % name)
        fit("%s_far_src" % name, "%s_far" % name)
        lines.append(
            "[%s_ov1][%s_ov2]blend=all_expr='A*(1-X/%d)+B*(X/%d)'[%s_ov]"
            % (name, name, blend_chunk - 1, blend_chunk - 1, name))
        order = (("%s_near" % name, "%s_ov" % name, "%s_far" % name)
                 if near_first else
                 ("%s_far" % name, "%s_ov" % name, "%s_near" % name))
        lines.append("%shstack=inputs=3[%s]"
                     % ("".join("[%s]" % item for item in order), name))

    side_face("LEFT", ["AL0", "AL1", "AL2", "AL3"], 0, near_first=False)
    side_face("RIGHT", ["AR0", "AR1", "AR2", "AR3"],
              side + centre, near_first=True)
    side_face("DOWN", ["BL0", "BL1", "BL2", "BL3"], 0, near_first=False)
    side_face("UP", ["BR0", "BR1", "BR2", "BR3"],
              side + centre, near_first=True)
    lines.append("[AC]crop=%d:%d:%d:0[FRONT]" % (centre, hemi_height, side))
    lines.append("[BC]crop=%d:%d:%d:0[BACK]" % (centre, hemi_height, side))
    lines.append("[LEFT][FRONT][RIGHT]hstack=inputs=3[ROWA]")
    lines.append("[DOWN][BACK][UP]hstack=inputs=3[ROWB]")
    lines.append("[ROWA][ROWB]vstack=inputs=2[eac]")
    lines.append("[eac]v360=eac:equirect:interp=%s:w=%d:h=%d[OUT]"
                 % (interp, width, height))
    return ";\n".join(lines)


def max360_max2sphere_command(video: str, output: str, width: int, height: int,
                       fps: float) -> List[str]:
    """Command line for the optional max2sphere backend (experimental)."""
    return ["max2sphere", "-i", video, "-o", output,
            "-w", str(width), "-h", str(height), "-f", "%.6f" % fps]


def max360__run(cmd: Sequence[str]) -> None:
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE)
    except FileNotFoundError as exc:
        raise max360_BackendError("required program not found: %s" % cmd[0]) from exc
    if proc.returncode != 0:
        stderr = proc.stderr.decode("utf-8", "replace").strip()
        tail = stderr.splitlines()
        raise max360_BackendError("command failed (%d): %s"
                           % (proc.returncode, tail[-1] if tail else "unknown"))


def max360__resolve_backend(name: str) -> str:
    if name == "auto":
        if shutil.which(FFMPEG):
            return "ffmpeg"
        if shutil.which("max2sphere"):
            return "max2sphere"
        raise max360_BackendError("neither ffmpeg nor max2sphere is on PATH")
    if name == "ffmpeg":
        if not shutil.which(FFMPEG):
            raise max360_BackendError("ffmpeg is not on PATH")
        return "ffmpeg"
    if not shutil.which(name):
        raise max360_BackendError("%s is not on PATH (install it or use "
                           "--stitch-backend ffmpeg)" % name)
    return name


def max360__copy_outputs(temp_files: Sequence[str], outputs: Sequence[str]) -> None:
    if len(temp_files) < len(outputs):
        raise max360_BackendError("ffmpeg produced %d image(s) for %d selected frames"
                           % (len(temp_files), len(outputs)))
    for temp, dest in zip(temp_files, outputs):
        parent = os.path.dirname(dest)
        if parent:
            os.makedirs(parent, exist_ok=True)
        shutil.copyfile(temp, dest)


def max360__kept_filter_path(base: str, index: int, total: int) -> str:
    """Per-batch name for ``--keep-filter`` (plain name for a single batch)."""
    if total <= 1:
        return base
    root, ext = os.path.splitext(base)
    return "%s.%02d%s" % (root, index, ext or ".filter")


def max360__stitch_batch(info: max360_VideoInfo, frames: Sequence[int],
                  by_frame: Dict[int, str], temp_dir: str, batch_index: int,
                  *, width: int, height: int, quality: int,
                  image_format: str, interp: str, threads: int,
                  keep_filter: Optional[str], total_batches: int,
                  seek_time: Optional[float]) -> None:
    """Run one bounded EAC/v360 batch and copy its images to their targets."""
    ordered = [int(frame) for frame in frames]
    selected = ordered if seek_time is None else\
        [frame - ordered[0] for frame in ordered]
    select = select_expression(selected)
    script = max360_build_eac_filter(
        info.hemi_a, info.hemi_b, width, height, select=select,
        interp=interp, already_equirect=info.already_equirect,
        hemi_width=info.width, hemi_height=info.height)
    attempt = "seek" if seek_time is not None else "full"
    script_path = os.path.join(
        temp_dir, "stitch_%03d_%s.filter" % (batch_index, attempt))
    with open(script_path, "w") as fh:
        fh.write(script + "\n")
    if keep_filter:
        dest = max360__kept_filter_path(keep_filter, batch_index, total_batches)
        parent = os.path.dirname(dest)
        if parent:
            os.makedirs(parent, exist_ok=True)
        shutil.copyfile(script_path, dest)

    ext = "png" if image_format == "png" else "jpg"
    prefix = "img_%03d_%s_" % (batch_index, attempt)
    pattern = os.path.join(temp_dir, prefix + "%06d." + ext)
    pix_fmt = "rgb24" if image_format == "png" else "yuvj420p"
    cmd = [FFMPEG, "-v", "error", "-y"]
    if seek_time is not None:
        cmd += ["-ss", "%.6f" % seek_time]
    cmd += ["-i", info.path, "-filter_complex_script", script_path,
            "-map", "[OUT]", "-an", "-sn", "-dn", "-vsync", "0",
            "-frames:v", str(len(ordered)),
            "-pix_fmt", pix_fmt]
    cmd += ffmpeg_quality_args(image_format, quality)
    if threads:
        cmd += ["-threads", str(threads)]
    cmd.append(pattern)
    max360__run(cmd)
    temp_files = sorted(glob.glob(
        os.path.join(temp_dir, prefix + "*." + ext)))
    if len(temp_files) != len(ordered):
        raise max360_BackendError(
            "ffmpeg produced %d image(s) for %d selected frames"
            % (len(temp_files), len(ordered)))
    for frame, temp in zip(ordered, temp_files):
        dest = by_frame[frame]
        parent = os.path.dirname(dest)
        if parent:
            os.makedirs(parent, exist_ok=True)
        shutil.copyfile(temp, dest)


def max360_stitch_ffmpeg(info: max360_VideoInfo, frames: Sequence[int],
                  outputs: Sequence[str], *, width: int, height: int,
                  quality: int, image_format: str, interp: str,
                  threads: int, keep_filter: Optional[str],
                  timeline: Optional[FrameTimeline] = None) -> None:
    """Stitch selected frames in bounded batches (seek-assisted when possible).

    A single ``select`` expression cannot hold hundreds of frame numbers, so
    frames are processed in batches.  Real ffprobe PTS values allow each batch
    to seek to its first frame; a failed seek batch is retried from the start
    of the file.  This keeps long MAX recordings both correct and fast.
    """
    ordered_frames = sorted({int(f) for f in frames})
    if not ordered_frames:
        return
    # outputs correspond to the original (already increasing) choices
    by_frame = {int(frame): output for frame, output in zip(frames, outputs)}
    size = EXTRACT_CHUNK_SIZE
    chunks = [ordered_frames[i:i + size]
              for i in range(0, len(ordered_frames), size)]
    temp_dir = tempfile.mkdtemp(prefix="max360_")
    try:
        for batch_index, chunk in enumerate(chunks):
            seek_time = None
            if (timeline is not None and timeline.source == "ffprobe"
                    and len(timeline) and chunk[-1] < len(timeline)):
                seek_time = float(timeline.pts(chunk[0]))
            if seek_time is not None:
                try:
                    max360__stitch_batch(
                        info, chunk, by_frame, temp_dir, batch_index,
                        width=width, height=height, quality=quality,
                        image_format=image_format, interp=interp,
                        threads=threads, keep_filter=keep_filter,
                        total_batches=len(chunks), seek_time=seek_time)
                    continue
                except max360_BackendError as exc:
                    LOG.warning("seek-based stitching failed for frames "
                                "%d-%d (%s); retrying with a full decode",
                                chunk[0], chunk[-1], exc)
            max360__stitch_batch(
                info, chunk, by_frame, temp_dir, batch_index,
                width=width, height=height, quality=quality,
                image_format=image_format, interp=interp, threads=threads,
                keep_filter=keep_filter, total_batches=len(chunks),
                seek_time=None)
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def max360_stitch_max2sphere(info: max360_VideoInfo, frames: Sequence[int],
                      outputs: Sequence[str], *, width: int, height: int,
                      quality: int, image_format: str, threads: int) -> None:
    """Stitch with the optional max2sphere binary, then extract frames."""
    temp_dir = tempfile.mkdtemp(prefix="max360_m2s_")
    video_out = os.path.join(temp_dir, "equirect.mp4")
    try:
        max360__run(max360_max2sphere_command(info.path, video_out, width, height, info.fps))
        timeline = probe_frame_timeline(
            video_out, fps_hint=info.fps, duration_hint=info.duration,
            logger=LOG)
        extracted = extract_frames_by_index(
            video_out, frames, os.path.join(temp_dir, "frames"),
            image_format=image_format, quality=quality, threads=threads,
            timeline=timeline)
        for frame, dest in zip(frames, outputs):
            temp = extracted.get(int(frame))
            if temp is None:
                raise max360_BackendError("max2sphere/ffmpeg did not produce frame %d"
                                   % frame)
            parent = os.path.dirname(dest)
            if parent:
                os.makedirs(parent, exist_ok=True)
            shutil.copyfile(temp, dest)
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


# ---------------------------------------------------------------------------
# GPMF track
# ---------------------------------------------------------------------------
def max360__smooth(values: Sequence[float], window: int) -> List[float]:
    if window <= 1 or len(values) < 3:
        return list(values)
    half = window // 2
    prefix = [0.0]
    for value in values:
        prefix.append(prefix[-1] + value)
    out = [0.0] * len(values)
    n = len(values)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out[i] = (prefix[hi] - prefix[lo]) / (hi - lo)
    return out


def max360_build_track(path: str, info: max360_VideoInfo, args) -> Tuple[Track, GpmfData]:
    if info.gpmd is None:
        raise max360_BackendError("no GPMF (gpmd) telemetry track in file")
    payload = extract_gpmf(path, info.gpmd, ffmpeg=FFMPEG)
    data = parse_gpmf(payload, imu_heading=args.imu_heading)
    LOG.info("  GPMF device: %s; streams: %s", data.device_name,
             ", ".join(sorted(data.streams)))
    if not data.has_gps:
        raise max360_BackendError("no GPS5/GPS9 data found in the GPMF stream")

    timed = [s for s in data.gps_samples if s.get("utc") is not None]
    if not timed:
        raise max360_BackendError("GPMF GPS samples have no usable timestamps")
    first_gps = min(s["utc"] for s in timed)
    last_gps = max(s["utc"] for s in timed)
    gps_span = (last_gps - first_gps).total_seconds()

    # GoPro MAX writes creation_time as camera-local wall clock in some
    # firmware, even though the field looks like UTC.  The GPMF GPS clock is
    # authoritative, so use the first fix unless creation_time agrees with it
    # to within five minutes.
    origin = info.creation_time
    origin_is_gps = False
    if origin is None or abs((first_gps - origin).total_seconds()) > 300:
        if origin is not None:
            LOG.warning("  container creation_time is %.0f s from GPS; using "
                        "the first GPS fix as the video clock",
                        (first_gps - origin).total_seconds())
        origin = first_gps
        origin_is_gps = True

    # TimeWarp (and some slow-motion modes) record real GPS time but play the
    # video back at a different rate.  Map the GPS span onto the video span so
    # frame selection and per-frame timestamps stay aligned, and never emit the
    # same frame twice.
    scale = 1.0
    if info.duration > 0.2 and gps_span > 0.2:
        ratio = gps_span / info.duration
        if ratio > 1.2 or ratio < 0.8:
            scale = info.duration / gps_span
            origin = first_gps
            origin_is_gps = True
            LOG.warning("  TimeWarp/rate change detected: GPS span %.2f s -> "
                        "video span %.2f s (scale %.4f)", gps_span,
                        info.duration, scale)

    samples = []
    for sample in data.gps_samples:
        utc = sample.get("utc")
        if utc is None:
            continue
        fix = sample.get("fix")
        if fix is not None and fix < 2:
            continue
        item = dict(sample)
        item["t"] = (utc - origin).total_seconds() * scale
        item["course"] = None
        apply_accuracy(item, args.uere)
        samples.append(item)
    if not samples:
        raise max360_BackendError("no GPMF GPS sample has a valid fix")
    samples.sort(key=lambda s: s["t"])

    if args.smooth and args.smooth > 1:
        lats = max360__smooth([s["lat"] for s in samples], args.smooth)
        lons = max360__smooth([s["lon"] for s in samples], args.smooth)
        for sample, lat, lon in zip(samples, lats, lons):
            sample["lat"], sample["lon"] = lat, lon

    track = Track(samples, args.distance_mode)
    LOG.info("  telemetry: %d GPS sample(s), %.1f m of travel, "
             "clock spread %.0f ms", len(samples), track.length,
             data.clock_spread_s * 1000)
    data.origin_utc = origin
    data.time_scale = scale
    return track, data


# ---------------------------------------------------------------------------
# Records
# ---------------------------------------------------------------------------
def max360__metadata_for(record, info: max360_VideoInfo, args) -> FrameMetadata:
    return FrameMetadata(
        camera=CAMERA_LABELS[CAMERA_MAX], video=record["video"],
        sequence=record["sequence"], frame_index=record["frame_index"],
        pts=record["pts_seconds"], distance=record.get("distance_m"),
        script_name=SCRIPT_NAME, make=info.make, model=info.model,
        software=info.firmware, serial=info.serial,
        captured_utc=parse_utc_timestamp(record.get("datetime_utc")),
        captured_local=(dt.datetime.fromisoformat(record["datetime_local"])
                        if record.get("datetime_local") else None),
        utc_offset_s=record.get("utc_offset_s"),
        lat=record.get("latitude"), lon=record.get("longitude"),
        alt=record.get("altitude_m"), speed=record.get("speed_m_s"),
        course=record.get("course_deg"), img_direction=record.get("img_direction_deg"),
        heading_source=record.get("heading_source", "none"),
        accuracy=record.get("accuracy_m"),
        accuracy_estimated=record.get("accuracy_estimated", False),
        accuracy_source=record.get("accuracy_source"),
        hdop=record.get("hdop"), vdop=record.get("vdop"),
        pdop=record.get("pdop"),
        position_source=record.get("position_source"),
        clamped=record.get("clamped"), estimated=record.get("estimated"),
        image_width=args.width, image_height=args.height, panorama=True,
        pose_heading=record.get("heading_deg"),
        keywords=parse_keywords(args.keywords),
    )


def max360__record(choice, track, args, info, tz_offset, tz_known, sequence,
            video_name):
    point = choice.point or {}
    pts = max(0.0, float(choice.pts))
    utc = point.get("utc")
    if utc is None:
        origin = info.creation_time
        utc = (origin + dt.timedelta(seconds=pts)
               if origin is not None else None)
    if utc.tzinfo is None:
        utc = utc.replace(tzinfo=dt.timezone.utc)
    local = local_from_utc(utc, tz_offset) if tz_known else None
    if args.time_base == "utc" and utc is not None:
        naming = utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
    else:
        naming = local or (utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
                           if utc is not None else None)
    if naming is None:
        naming = dt.datetime(1970, 1, 1)

    heading, heading_source = resolve_heading(
        point, args.heading_offset, args.magnetic_declination)
    accuracy, accuracy_estimated, accuracy_source = derive_accuracy(
        point, args.uere)
    course = point.get("course")
    if course is None:
        course = point.get("gps_course")
    speed = point.get("speed")
    if speed is None:
        speed = point.get("speed2d")
    return {
        "sequence": sequence, "video": video_name, "camera": CAMERA_MAX,
        "image": image_filename(naming, args.format),
        "frame_index": int(choice.frame_index), "pts_seconds": round(pts, 3),
        "datetime_local": local.isoformat() if local else None,
        "datetime_utc": fmt_utc(utc),
        "utc_offset_s": tz_offset if tz_known else None,
        "latitude": point.get("lat"), "longitude": point.get("lon"),
        "altitude_m": point.get("alt"), "speed_m_s": speed,
        "course_deg": course, "heading_deg": heading,
        "heading_source": heading_source, "img_direction_deg": heading,
        "accuracy_m": accuracy, "accuracy_estimated": accuracy_estimated,
        "accuracy_source": accuracy_source,
        "hdop": point.get("hdop"), "vdop": point.get("vdop"),
        "pdop": point.get("pdop"),
        "distance_m": float(choice.target_distance),
        "track_distance_m": point.get("distance"),
        "position_source": point.get("position_source", "none"),
        "clamped": bool(point.get("clamped")),
        "estimated": bool(point.get("estimated")),
        "naming_time": naming, "point": point,
    }


# ---------------------------------------------------------------------------
# Per-file processing
# ---------------------------------------------------------------------------
def max360_process_video(path: str, sequence: str, args) -> dict:
    name = os.path.basename(path)
    result = {"video": name, "sequence": sequence, "images": 0,
              "planned": 0, "failed": False, "entries": [], "dir": None}
    LOG.info("--- %s -> %s", name, sequence)
    info = max360_probe(path)
    LOG.info("  video: %dx%d %.4f fps, %d frame(s), %.2f s%s",
             info.width, info.height, info.fps, info.frame_count,
             info.duration, " (already equirectangular)"
             if info.already_equirect else " (EAC hemispheres)")

    ordinal = info.video_ordinals.get(info.hemi_a, 0)
    timeline = probe_frame_timeline(
        path, stream_ordinal=ordinal, fps_hint=info.fps,
        frame_count_hint=info.frame_count, duration_hint=info.duration,
        logger=LOG)
    if len(timeline) > info.frame_count:
        timeline = FrameTimeline(timeline.times[:info.frame_count],
                                     timeline.source, timeline.fps)
    LOG.info("  frame timeline: %d frame(s), source=%s",
             len(timeline), timeline.source)

    track, gpmf = max360_build_track(path, info, args)
    detected_offset = None       # GoPro MAX stores UTC, not local wall clock
    tz_offset, tz_known = resolve_timezone(
        args, detected_offset, camera=CAMERA_MAX, logger=LOG)

    choices, skipped = select_unique_frames(
        track, timeline, args.distance, start_distance=args.start_distance,
        max_position_error=args.max_position_error, max_images=args.max_images,
        mode=args.distance_mode, logger=LOG)
    if skipped:
        LOG.info("  skipped %d target(s) with no acceptable unused frame",
                 len(skipped))
    if not choices:
        LOG.error("  no capture point could be matched to a unique frame")
        result["failed"] = True
        return result

    records = [max360__record(c, track, args, info, tz_offset, tz_known, sequence,
                       name) for c in choices]
    first_time = records[0]["naming_time"]
    sequence_dir = sequence_output_dir(
        args.output, CAMERA_MAX, sequence, first_time,
        heading_offset=heading_offset_for_folder(args))
    used_paths = set()
    for record in records:
        record["path"] = unique_path(
            os.path.join(sequence_dir, record["image"]), used_paths)
        record["image"] = os.path.basename(record["path"])
    result["planned"] = len(records)
    result["dir"] = sequence_dir
    LOG.info("  %d capture point(s) -> %s", len(records), sequence_dir)

    if args.dry_run:
        for record in records:
            LOG.info("    frame=%d pts=%.3fs %sZ %.6f,%.6f hdg=%s d=%.1fm -> %s",
                     record["frame_index"], record["pts_seconds"],
                     (record["datetime_utc"] or "")[11:23],
                     record["latitude"] or 0.0, record["longitude"] or 0.0,
                     record["heading_deg"], record["distance_m"],
                     record["image"])
        return result

    to_extract = []
    skipped_existing = 0
    for record in records:
        if os.path.exists(record["path"]) and not args.overwrite:
            skipped_existing += 1
            continue
        to_extract.append(record)

    backend = max360__resolve_backend(args.stitch_backend)
    LOG.info("  stitching backend: %s", backend)
    if to_extract:
        outputs = [record["path"] for record in to_extract]
        frames = [record["frame_index"] for record in to_extract]
        keep_filter = (os.path.join(sequence_dir, sequence + ".filter")
                       if args.keep_filter else None)
        if backend == "max2sphere":
            max360_stitch_max2sphere(info, frames, outputs, width=args.width,
                              height=args.height, quality=args.quality,
                              image_format=args.format, threads=args.threads)
        else:
            max360_stitch_ffmpeg(info, frames, outputs, width=args.width,
                          height=args.height, quality=args.quality,
                          image_format=args.format, interp=args.interp,
                          threads=args.threads, keep_filter=keep_filter,
                          timeline=timeline)
        for record in to_extract:
            record["written"] = os.path.exists(record["path"])
    if (args.start_distance == 0 and records
            and records[0]["frame_index"] == 0
            and not os.path.exists(records[0]["path"])):
        LOG.error("  first frame (index 0) could not be extracted; failing %s",
                  name)
        result["failed"] = True
        return result
    written = [record for record in records if record.get("written")]
    result["images"] = len(written)
    LOG.info("  wrote %d image(s), skipped %d existing",
             len(written), skipped_existing)

    if written and not args.no_exif:
        pairs = [(record["path"], build_exiftool_args(
            max360__metadata_for(record, info, args))) for record in written]
        write_metadata(pairs)

    export_track_files(
        sequence_dir, sequence, track.samples, records,
        track_export=args.track_export, camera=CAMERA_MAX, video=name,
        imu=gpmf.imu, video_start_utc=gpmf.origin_utc,
        telemetry_export=args.telemetry_export,
        time_scale=gpmf.time_scale,
        extra={"distance_interval_m": args.distance,
               "time_base": args.time_base})
    result["entries"] = [{k: v for k, v in record.items()
                          if k not in ("point", "naming_time")}
                         for record in written]
    return result


# =========================================================================
# Garmin VIRB Ultra 30 pipeline
# =========================================================================

virb_FIT_EPOCH = dt.datetime(1989, 12, 31, tzinfo=dt.timezone.utc)
virb_SEMICIRCLE = 180.0 / (2 ** 31)

virb_FIT_EXT = ".fit"
virb_DEFAULT_VIDEO_GLOB = "*.MP4"
virb_DEFAULT_FIT_DIR = "gmetrix"
virb_DEFAULT_DISTANCE = 5.0
virb_DEFAULT_QUALITY = 95




def virb_run(cmd):
    """Run a command and return its stdout as text. Raises Fatal on failure."""
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        raise Fatal("required program not found: %s" % cmd[0])
    if proc.returncode != 0:
        raise Fatal("command failed (%d): %s\n%s" % (
            proc.returncode, " ".join(cmd),
            proc.stderr.decode("utf-8", "replace").strip()))
    return proc.stdout.decode("utf-8", "replace")


# ===========================================================================
# Minimal, dependency-free Garmin FIT decoder
# ===========================================================================
# Garmin stores multi-byte integers little-endian inside a record but the
# header keeps profile_version big-endian.  The conversion factors below are
# written out explicitly because a FIT "scale" is a divisor for the raw value.

virb_BASE_TYPES = {
    0x00: ("enum", "B", 1), 0x01: ("sint8", "b", 1), 0x02: ("uint8", "B", 1),
    0x83: ("sint16", "h", 2), 0x84: ("uint16", "H", 2), 0x85: ("sint32", "i", 4),
    0x86: ("uint32", "I", 4), 0x07: ("string", None, 1), 0x88: ("float32", "f", 4),
    0x89: ("float64", "d", 8), 0x0A: ("uint8z", "B", 1), 0x8B: ("uint16z", "H", 2),
    0x8C: ("uint32z", "I", 4), 0x0D: ("byte", None, 1), 0x8E: ("sint64", "q", 8),
    0x8F: ("uint64", "Q", 8), 0x90: ("uint64z", "Q", 8),
}

virb_INVALID = {
    "uint8": 0xFF, "uint8z": 0x00, "uint16": 0xFFFF, "uint16z": 0x0000,
    "uint32": 0xFFFFFFFF, "uint32z": 0x00000000, "uint64": 0xFFFFFFFFFFFFFFFF,
    "uint64z": 0, "sint8": 0x7F, "sint16": 0x7FFF, "sint32": 0x7FFFFFFF,
    "sint64": 0x7FFFFFFFFFFFFFFF,
}

virb_MSG_FILE_ID = 0
virb_MSG_RECORD = 20
virb_MSG_DEVICE_INFO = 23
virb_MSG_GPS_METADATA = 160
virb_MSG_CAMERA_EVENT = 161
virb_MSG_GYROSCOPE = 164
virb_MSG_ACCELEROMETER = 165
virb_MSG_SENSOR_CALIBRATION = 167
virb_MSG_MAGNETOMETER = 208

virb_IMU_MESSAGES = {
    virb_MSG_GYROSCOPE: "GYRO",
    virb_MSG_ACCELEROMETER: "ACCL",
    virb_MSG_MAGNETOMETER: "MAGN",
}
virb_SENSOR_TYPES = {0: "ACCL", 1: "GYRO", 2: "MAGN"}
virb_IMU_UNITS = {"ACCL": "g", "GYRO": "deg/s", "MAGN": "G"}

virb_CAMERA_EVENT_TYPES = {
    0: "video_start", 1: "video_split", 2: "video_end", 3: "photo_taken",
    4: "video_second_stream_start", 5: "video_second_stream_split",
    6: "video_second_stream_end", 7: "video_split_start",
    8: "video_second_stream_split_start", 11: "video_pause",
    12: "video_second_stream_pause", 13: "video_resume",
    14: "video_second_stream_resume",
}

virb_CAMERA_ORIENTATION = {0: 0, 1: 90, 2: 180, 3: 270}

# Camera events that begin a recorded video segment and that end one.
virb_VIDEO_START_EVENTS = ("video_start", "video_split_start", "video_split")
virb_VIDEO_END_EVENTS = ("video_end", "video_second_stream_end")


class virb_FitMessage(object):
    __slots__ = ("num", "fields")

    def __init__(self, num, fields):
        self.num = num
        self.fields = fields

    def get(self, num):
        return self.fields.get(num)


def virb_parse_fit(path):
    """Decode a FIT file into a list of FitMessage.

    Damaged files never raise: parsing stops at the first inconsistency.
    """
    with open(path, "rb") as fh:
        data = fh.read()
    if len(data) < 14 or data[8:12] != b".FIT":
        raise Fatal("%s is not a Garmin FIT file" % path)

    header_size = data[0]
    data_size = struct.unpack("<I", data[4:8])[0]
    pos = header_size
    end = min(header_size + data_size, len(data))

    local_defs = {}
    out = []
    while pos < end:
        hdr = data[pos]
        pos += 1

        # Bit 7 marks a data message carrying a compressed timestamp.
        if hdr & 0x80:
            local = (hdr >> 5) & 0x03
            definition = local_defs.get(local)
            if definition is None or pos + definition["size_ct"] > end:
                break
            out.append(virb_FitMessage(
                definition["num"],
                virb__read_fields(data, pos, definition, definition["endian"], True, end)))
            pos += definition["size_ct"]
            continue

        is_def = bool(hdr & 0x40)
        is_dev = bool(hdr & 0x20)
        local = hdr & 0x0F
        # NB: a header byte of 0x00 is a valid data message for local type 0.

        if is_def:
            if pos + 5 > end:
                break
            arch = data[pos + 1]
            endian = "<" if arch == 0 else ">"
            num = struct.unpack(endian + "H", data[pos + 2:pos + 4])[0]
            nfields = data[pos + 4]
            p = pos + 5
            fields = []
            for _ in range(nfields):
                if p + 3 > end:
                    fields = None
                    break
                fields.append((data[p], data[p + 1], data[p + 2]))
                p += 3
            dev_fields = []
            if fields is not None and is_dev:
                if p + 1 > end:
                    fields = None
                else:
                    ndev = data[p]
                    p += 1
                    for _ in range(ndev):
                        if p + 3 > end:
                            fields = None
                            break
                        dev_fields.append((data[p], data[p + 1], data[p + 2]))
                        p += 3
            if fields is None:
                break                       # truncated definition
            local_defs[local] = {
                "num": num, "endian": endian, "fields": fields,
                "size": sum(f[1] for f in fields + dev_fields),
                "size_ct": sum(f[1] for f in fields + dev_fields if f[0] != 254),
            }
            pos = p
        else:
            definition = local_defs.get(local)
            if definition is None or pos + definition["size"] > end:
                break
            out.append(virb_FitMessage(
                definition["num"],
                virb__read_fields(data, pos, definition, definition["endian"], False, end)))
            pos += definition["size"]

    return out


def virb__read_fields(data, pos, definition, endian, compressed, end):
    p = pos
    out = {}
    for fnum, size, base in definition["fields"]:
        if p + size > end:
            break                           # truncated payload
        if compressed and fnum == 254:
            continue                        # time_offset lives in the header
        kind, fmt, bsize = virb_BASE_TYPES.get(base, ("byte", None, 1))
        n = size // bsize
        if fmt is None:
            chunk = data[p:p + size]
            p += size
            if kind == "string":
                out[fnum] = chunk.split(b"\x00")[0].decode("utf-8", "replace")
            else:
                out[fnum] = chunk
        else:
            values = struct.unpack(endian + str(n) + fmt, data[p:p + size])
            p += size
            bad = virb_INVALID.get(kind)
            if bad is not None:
                values = tuple(None if v == bad else v for v in values)
            out[fnum] = values[0] if n == 1 else values
    return out


# ===========================================================================
# High level telemetry
# ===========================================================================

def virb__num(value, default=None):
    if value is None:
        return default
    if isinstance(value, tuple):
        return value[0] if value else default
    return value


def virb_parse_fit_name_local(name):
    """GMetrix files are named '<YYYY-MM-DD-HH-MM-SS>.fit' in camera-local
    time. Returns a naive datetime or None."""
    match = re.match(r"(\d{4})-(\d{2})-(\d{2})-(\d{2})-(\d{2})-(\d{2})",
                     os.path.basename(name))
    if not match:
        return None
    try:
        return dt.datetime(*(int(x) for x in match.groups()))
    except ValueError:
        return None


class virb_Telemetry(object):
    """Everything the exporter needs from one FIT file."""

    def __init__(self, path):
        self.path = path
        self.samples = []          # one dict per gps_metadata record
        self.records = []          # one dict per record message (odometer)
        self.camera_events = []    # (fit_seconds, type_name, orientation_deg)
        self.device = {}
        self.time_created_utc = None
        self.utc_offset_s = None   # device time -> UTC correction
        self.track_distance = None
        self.imu_available = False
        self.imu = {}              # {ACCL/GYRO/MAGN: [sample dicts]}
        self.sensor_calibration = {}
        self._raw_imu = {}
        self._load()

    def _load(self):
        for msg in virb_parse_fit(self.path):
            num = msg.num
            if num == virb_MSG_GPS_METADATA:
                self._add_gps(msg.fields)
            elif num == virb_MSG_CAMERA_EVENT:
                self._add_camera_event(msg.fields)
            elif num == virb_MSG_DEVICE_INFO:
                self._add_device(msg.fields)
            elif num == virb_MSG_FILE_ID:
                self._add_file_id(msg.fields)
            elif num == virb_MSG_SENSOR_CALIBRATION:
                self._add_sensor_calibration(msg.fields)
            elif num == virb_MSG_RECORD:
                fit_s = virb__num(msg.get(253))
                distance = msg.get(5)
                if fit_s is None or not isinstance(distance, (int, float)):
                    continue
                self.records.append({"t": fit_s, "distance": distance / 100.0})
                self.track_distance = distance / 100.0
            elif num in virb_IMU_MESSAGES:
                self._add_imu(num, msg.fields)

        self.samples.sort(key=lambda s: s["t"])
        self._dedupe()
        self.records.sort(key=lambda r: r["t"])
        for sample in self.samples:
            if sample["utc_fit"] is not None:
                self.utc_offset_s = sample["utc_fit"] - sample["fit_s"]
                break
        self._finalize_imu()

    def _add_gps(self, fields):
        fit_s = virb__num(fields.get(253))
        if fit_s is None:
            return
        lat_raw = virb__num(fields.get(1))
        lon_raw = virb__num(fields.get(2))
        if lat_raw is None or lon_raw is None:
            return
        lat = lat_raw * virb_SEMICIRCLE
        lon = lon_raw * virb_SEMICIRCLE
        if abs(lat) > 90 or abs(lon) > 180:
            return
        ms = virb__num(fields.get(0), 0) or 0
        alt_raw = virb__num(fields.get(3))
        spd_raw = virb__num(fields.get(4))
        hdg_raw = virb__num(fields.get(5))
        utc_raw = virb__num(fields.get(6))
        vel = fields.get(7)
        accuracy_raw = virb__num(fields.get(8))
        hdop_raw = virb__num(fields.get(11))
        vdop_raw = virb__num(fields.get(12))
        sample = {
            "fit_s": fit_s,
            "t": fit_s + ms / 1000.0,
            "utc_fit": utc_raw,
            "lat": lat,
            "lon": lon,
            # Garmin encodes enhanced_altitude as raw/5 - 500 metres.
            "alt": (alt_raw / 5.0 - 500.0) if alt_raw is not None else None,
            "speed": (spd_raw / 1000.0) if spd_raw is not None else None,
            "course": (hdg_raw / 100.0) if hdg_raw is not None else None,
            "velocity": [v / 100.0 for v in vel] if isinstance(vel, tuple) else None,
        }
        # FIT gps_metadata field 8 behaves like a measured accuracy in metres;
        # fields 11/12 look like scaled HDOP/VDOP but are not in the public SDK
        # profile, so they are used only as a documented fallback.
        if accuracy_raw is not None:
            sample["accuracy"] = float(accuracy_raw)
            sample["accuracy_estimated"] = False
            sample["accuracy_source"] = "fit-gps_metadata-field8"
        if hdop_raw is not None:
            sample["hdop"] = hdop_raw / 100.0
        if vdop_raw is not None:
            sample["vdop"] = vdop_raw / 100.0
        self.samples.append(sample)

    def _add_camera_event(self, fields):
        fit_s = virb__num(fields.get(253))
        if fit_s is None:
            return
        ms = virb__num(fields.get(0), 0) or 0
        etype = virb__num(fields.get(1))
        orient = virb_CAMERA_ORIENTATION.get(virb__num(fields.get(3)))
        self.camera_events.append((
            fit_s + ms / 1000.0,
            virb_CAMERA_EVENT_TYPES.get(etype, "type_%s" % etype),
            orient))

    def _add_device(self, fields):
        if virb__num(fields.get(0), 0) not in (0, None):   # only the creator device
            return
        self.device = {
            "manufacturer": virb__num(fields.get(2)),
            "serial": virb__num(fields.get(3)),
            "product": virb__num(fields.get(4)),
            "software_raw": virb__num(fields.get(5)),
        }

    def _add_file_id(self, fields):
        created = virb__num(fields.get(4))
        if created is not None:
            self.time_created_utc = virb_FIT_EPOCH + dt.timedelta(seconds=created)

    def _add_sensor_calibration(self, fields):
        """Remember the calibration that converts raw IMU counts to units."""
        key = virb_SENSOR_TYPES.get(virb__num(fields.get(0)))
        if key is None:
            return
        calibration = {
            "factor": virb__num(fields.get(1)),
            "divisor": virb__num(fields.get(2)),
            "level_shift": virb__num(fields.get(3)),
            "unit": virb_IMU_UNITS.get(key),
        }
        offset = fields.get(4)
        matrix = fields.get(5)
        if isinstance(offset, (tuple, list)):
            calibration["offset_cal"] = list(offset)
        if isinstance(matrix, (tuple, list)):
            calibration["orientation_matrix"] = list(matrix)
        self.sensor_calibration[key] = calibration

    def _add_imu(self, num, fields):
        """Collect one IMU message (up to 30 samples) as raw ADC counts."""
        key = virb_IMU_MESSAGES.get(num)
        if key is None:
            return
        timestamp = virb__num(fields.get(253))
        if timestamp is None:
            return
        millis = virb__num(fields.get(0), 0) or 0
        offsets = fields.get(1)
        if not isinstance(offsets, (tuple, list)):
            offsets = ()
        axes = [fields.get(2), fields.get(3), fields.get(4)]
        if any(not isinstance(axis, (tuple, list)) for axis in axes):
            return
        count = min(len(axes[0]), len(axes[1]), len(axes[2]))
        rows = self._raw_imu.setdefault(key, [])
        for index in range(count):
            offset = offsets[index] if index < len(offsets) else 0
            fit_s = timestamp + (millis + (offset or 0)) / 1000.0
            rows.append({"fit_s": fit_s,
                         "raw": [axes[0][index], axes[1][index],
                                 axes[2][index]]})

    def _finalize_imu(self):
        """Apply the FIT 3D-sensor calibration to the collected raw counts."""
        raw = self._raw_imu
        self._raw_imu = {}
        for key, rows in raw.items():
            calibration = self.sensor_calibration.get(key, {})
            factor = calibration.get("factor")
            divisor = calibration.get("divisor")
            shift = calibration.get("level_shift")
            factor = 1.0 if factor is None else float(factor)
            divisor = 1.0 if not divisor else float(divisor)
            shift = 0.0 if shift is None else float(shift)
            points = []
            for row in rows:
                values = [((value - shift) / divisor) * factor
                          for value in row["raw"]]
                points.append({
                    "fit_s": row["fit_s"],
                    "utc": self.utc_of(row["fit_s"]),
                    "raw": list(row["raw"]),
                    "values": values,
                })
            points.sort(key=lambda point: point["fit_s"])
            self.imu[key] = points
        self.imu_available = bool(self.imu)

    def _dedupe(self):
        out = []
        last = None
        for sample in self.samples:
            if last is not None and abs(sample["t"] - last) < 1e-9:
                continue
            out.append(sample)
            last = sample["t"]
        self.samples = out

    # -- derived helpers ---------------------------------------------------

    def utc_of(self, fit_seconds):
        """Absolute UTC (aware) of a FIT timestamp, or None if unknown."""
        if self.utc_offset_s is not None:
            return virb_FIT_EPOCH + dt.timedelta(seconds=fit_seconds + self.utc_offset_s)
        if self.time_created_utc is not None and self.samples:
            # file_id.time_created is real UTC; anchor on the first GPS sample.
            delta = fit_seconds - self.samples[0]["fit_s"]
            return self.time_created_utc + dt.timedelta(seconds=delta)
        return None

    def local_time_of(self, fit_seconds, tz_offset):
        """Camera-local naive datetime of a FIT timestamp, or None."""
        utc = self.utc_of(fit_seconds)
        if utc is not None:
            if tz_offset is not None:
                return (utc + dt.timedelta(seconds=tz_offset)).replace(tzinfo=None)
            return utc.replace(tzinfo=None)
        # Last resort: anchor on the FIT file name local time.
        local = virb_parse_fit_name_local(self.path)
        if local and self.samples:
            return local + dt.timedelta(seconds=fit_seconds - self.samples[0]["fit_s"])
        return None

    def local_utc_offset(self):
        """Camera-local wall clock minus UTC in seconds, or None.

        The GMetrix file name holds the local time at which the camera created
        the file while file_id.time_created is real UTC, so their difference is
        the camera's UTC offset.  Snapped to a quarter hour.
        """
        local = virb_parse_fit_name_local(self.path)
        if local and self.time_created_utc:
            delta = local.replace(tzinfo=dt.timezone.utc) - self.time_created_utc
            return int(round(delta.total_seconds() / 900.0) * 900)
        return None

    def camera_orientation(self):
        for _t, _name, orient in sorted(self.camera_events):
            if orient is not None:
                return orient
        return None

    def segments(self):
        """Video segments recorded inside this FIT as {start, end} FIT seconds."""
        events = sorted(self.camera_events)
        starts = []
        for t, name, _orient in events:
            if name in virb_VIDEO_START_EVENTS:
                if not starts or t - starts[-1] > 0.5:
                    starts.append(t)
        ends = [t for t, name, _o in events if name in virb_VIDEO_END_EVENTS]
        segments = []
        for index, start in enumerate(starts):
            if index + 1 < len(starts):
                end = starts[index + 1]
            elif ends:
                end = ends[-1]
            else:
                end = None
            segments.append({"start": start, "end": end})
        return segments

    def sample_window(self, t0, t1):
        return [s for s in self.samples if t0 <= s["t"] <= t1]

    def odometer_window(self, t0, t1):
        """Distance recorded by the device odometer between t0 and t1, or None."""
        before = [r for r in self.records if r["t"] <= t0]
        after = [r for r in self.records if r["t"] >= t1]
        if not before or not after:
            return None
        return max(0.0, after[0]["distance"] - before[-1]["distance"])



# ===========================================================================
# MP4 inspection
# ===========================================================================

def virb_read_mp4_atoms(path):
    """Read the small Garmin user-data atoms stored in moov/udta.

    Returns the embedded FIT file name (authoritative video<->telemetry link),
    the camera settings string and the size of the 'hmtp' telemetry atom.
    """
    info = {"fit_reference": None, "settings": None,
            "hmtp_bytes": 0, "hmtp_nonzero": 0}
    containers = {b"moov", b"udta", b"trak", b"mdia", b"minf", b"stbl"}
    try:
        size = os.path.getsize(path)
    except OSError:
        return info

    with open(path, "rb") as fh:
        stack = [(0, size)]
        found = 0
        while stack and found < 64:
            start, end = stack.pop()
            pos = start
            while pos + 8 <= end:
                fh.seek(pos)
                head = fh.read(8)
                if len(head) < 8:
                    break
                atom_size, atom_type = struct.unpack(">I4s", head)
                header = 8
                if atom_size == 1:
                    ext = fh.read(8)
                    if len(ext) < 8:
                        break
                    atom_size = struct.unpack(">Q", ext)[0]
                    header = 16
                elif atom_size == 0:
                    atom_size = end - pos
                if atom_size < header or pos + atom_size > end:
                    break
                if atom_type in containers:
                    stack.append((pos + header, pos + atom_size))
                elif atom_type in (b"uuid", b"pmcc", b"hmtp"):
                    fh.seek(pos + header)
                    payload = fh.read(min(atom_size - header, 4096))
                    found += 1
                    if atom_type == b"uuid":
                        text = payload.rstrip(b"\x00").decode("utf-8", "replace")
                        match = re.search(
                            r"([0-9]{4}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}\.fit)",
                            text)
                        if match:
                            info["fit_reference"] = match.group(1)
                    elif atom_type == b"pmcc":
                        info["settings"] = payload[4:].rstrip(b"\x00").decode("utf-8", "replace")
                    else:
                        info["hmtp_bytes"] = atom_size - header
                        info["hmtp_nonzero"] = sum(1 for b in payload[8:] if b)
                pos += atom_size
    return info


def virb_parse_creation_time(text):
    """QuickTime creation stamp -> naive datetime.

    Garmin writes this as camera-LOCAL wall clock time (verified against the
    GMetrix video_start event), so the trailing 'Z' is deliberately ignored.
    """
    if not text:
        return None
    for fmt in ("%Y-%m-%dT%H:%M:%S.%fZ", "%Y-%m-%dT%H:%M:%SZ"):
        try:
            return dt.datetime.strptime(text, fmt)
        except ValueError:
            continue
    return None


def virb_probe_video(path):
    """Return fps, frame count, duration, dimensions, codec and creation time."""
    out = virb_run(["ffprobe", "-v", "error", "-print_format", "json",
               "-show_format", "-show_streams", path])
    try:
        data = json.loads(out)
    except ValueError:
        raise Fatal("ffprobe returned unreadable output for %s" % path)

    video = None
    for stream in data.get("streams", []):
        if stream.get("codec_type") == "video":
            video = stream
            break
    if video is None:
        raise Fatal("no video stream in %s (not a playable video file)" % path)

    rate = video.get("avg_frame_rate") or video.get("r_frame_rate") or "0/0"
    try:
        num, den = rate.split("/")
        fps = float(num) / float(den) if float(den) else 0.0
    except (ValueError, ZeroDivisionError):
        fps = 0.0

    duration = None
    if video.get("duration"):
        duration = float(video["duration"])
    elif data.get("format", {}).get("duration"):
        duration = float(data["format"]["duration"])

    return {
        "fps": fps,
        "frames": int(video.get("nb_frames") or 0),
        "duration": duration,
        "width": video.get("width"),
        "height": video.get("height"),
        "codec": video.get("codec_name"),
        "creation_time": data.get("format", {}).get("tags", {}).get("creation_time"),
        "size": os.path.getsize(path),
    }


# ===========================================================================
# FIT discovery and matching
# ===========================================================================

def virb_walk_files(root, exclude_dirs=()):
    """Yield (directory, sorted_file_names), skipping excluded absolute dirs."""
    exclude = {os.path.abspath(d) for d in exclude_dirs}
    for current, dirs, files in os.walk(root):
        dirs[:] = sorted(d for d in dirs
                         if os.path.abspath(os.path.join(current, d)) not in exclude)
        yield current, sorted(files)


def virb_find_fits(input_dir, exclude_dirs=()):
    out = []
    for current, files in virb_walk_files(input_dir, exclude_dirs):
        for name in files:
            if name.lower().endswith(virb_FIT_EXT):
                out.append(os.path.join(current, name))
    return out


def virb_fits_for_video(video, all_fits, fit_dir_name):
    """FIT candidates for one video.

    Preferred: a sibling `<fit_dir_name>/` folder (case-insensitive), then any
    `.fit` sitting directly next to the video, then every discovered `.fit` as a
    last resort.
    """
    video_dir = os.path.dirname(os.path.abspath(video))
    local_dirs = []
    try:
        for entry in os.listdir(video_dir):
            candidate = os.path.join(video_dir, entry)
            if os.path.isdir(candidate) and entry.lower() == fit_dir_name.lower():
                local_dirs.append(candidate)
    except OSError:
        pass

    local = []
    for directory in local_dirs:
        for current, files in virb_walk_files(directory):
            for name in files:
                if name.lower().endswith(virb_FIT_EXT):
                    local.append(os.path.join(current, name))

    direct = [f for f in all_fits
              if os.path.dirname(os.path.abspath(f)) == video_dir]

    candidates = sorted(set(local + direct))
    return candidates if candidates else sorted(all_fits)


def virb_pick_fit(atoms, candidates, created_local, explicit=None):
    """Choose the FIT that belongs to a video.

    Returns (path, reference, problem).  Priority:

      1. an explicit --fit (the user takes responsibility),
      2. an exact match on the FIT file name the MP4 stores internally,
      3. the FIT whose file-name start time is closest to the video's creation
         stamp (same clock, so this is a strong signal),
      4. the only candidate, if there is exactly one.
    """
    reference = atoms.get("fit_reference")

    if explicit:
        note = None
        if reference and os.path.basename(explicit).lower() != reference.lower():
            note = "using --fit %s although the video references %s" % (
                os.path.basename(explicit), reference)
        return explicit, reference, note

    if reference:
        for fit in candidates:
            if os.path.basename(fit).lower() == reference.lower():
                return fit, reference, None
        return None, reference, ("the FIT file this video references (%s) was not "
                                 "found near the video" % reference)

    if created_local:
        scored = []
        for fit in candidates:
            fit_local = virb_parse_fit_name_local(fit)
            if fit_local is not None:
                scored.append((abs((fit_local - created_local).total_seconds()), fit))
        if scored:
            scored.sort(key=lambda item: item[0])
            delta, fit = scored[0]
            if delta < 21600.0:             # same recording session (< 6 h)
                return fit, os.path.basename(fit), None

    if len(candidates) == 1:
        return candidates[0], os.path.basename(candidates[0]), None
    return None, reference, None


# ===========================================================================
# Records, metadata and per-video processing
# ===========================================================================

def virb__record_from_choice(choice, track, args, video_start_utc, tz_offset,
                        tz_known, camera, sequence, video_name):
    """Turn a selected frame into a manifest/metadata record."""
    point = choice.point or {}
    pts = max(0.0, float(choice.pts))
    # Derive the capture time from the video timeline so the file name keeps
    # sub-second precision.  FIT/GPS samples only carry whole seconds, which
    # would otherwise turn every image into a ``-000`` millisecond name.
    captured_utc = None
    if video_start_utc is not None:
        captured_utc = video_start_utc + dt.timedelta(seconds=pts)
    else:
        captured_utc = point.get("utc")
        if captured_utc is not None and captured_utc.tzinfo is None:
            captured_utc = captured_utc.replace(tzinfo=dt.timezone.utc)
    captured_local = (local_from_utc(captured_utc, tz_offset)
                      if tz_known else None)
    if args.time_base == "utc" and captured_utc is not None:
        naming = captured_utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
    else:
        naming = captured_local
        if naming is None and captured_utc is not None:
            naming = captured_utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
    if naming is None:
        naming = dt.datetime(1970, 1, 1)

    heading, heading_source = resolve_heading(
        point, args.heading_offset, args.magnetic_declination)
    accuracy, accuracy_estimated, accuracy_source = derive_accuracy(
        point, args.uere)
    course = point.get("course")
    if course is None:
        course = point.get("gps_course")
    speed = point.get("speed")
    if speed is None and point.get("speed2d") is not None:
        speed = point["speed2d"]

    filename = image_filename(naming, args.format)
    return {
        "sequence": sequence,
        "video": video_name,
        "camera": camera,
        "image": filename,
        "frame_index": int(choice.frame_index),
        "pts_seconds": round(pts, 3),
        "datetime_local": captured_local.isoformat() if captured_local else None,
        "datetime_utc": fmt_utc(captured_utc),
        "utc_offset_s": tz_offset if tz_known else None,
        "latitude": point.get("lat"),
        "longitude": point.get("lon"),
        "altitude_m": point.get("alt"),
        "speed_m_s": speed,
        "course_deg": course,
        "heading_deg": heading,
        "heading_source": heading_source,
        "img_direction_deg": heading,
        "accuracy_m": accuracy,
        "accuracy_estimated": accuracy_estimated,
        "accuracy_source": accuracy_source,
        "hdop": point.get("hdop"),
        "vdop": point.get("vdop"),
        "pdop": point.get("pdop"),
        "distance_m": float(choice.target_distance),
        "track_distance_m": point.get("distance"),
        "position_source": point.get("position_source", "none"),
        "clamped": bool(point.get("clamped")),
        "estimated": bool(point.get("estimated")),
        "naming_time": naming,
        "point": point,
    }


def virb__unpositioned_choices(timeline, interval, duration, max_images):
    frames = select_time_grid_frames(
        timeline, interval, duration=duration, max_images=max_images)
    out = []
    for index in frames:
        pts = timeline.pts(index)
        out.append(FrameChoice(
            target_distance=pts, target_time=pts, frame_index=index, pts=pts,
            point={}, position_error=None, clamped=False, estimated=True))
    return out


def virb__write_sidecar(path, sequence, video_name, records, source, args):
    meta = {
        "generator": SCRIPT_NAME,
        "sequence": sequence,
        "source_video": video_name,
        "camera": {
            "make": source.get("make"), "model": source.get("model"),
            "serial": source.get("serial"),
            "software": source.get("software"),
        },
        "distance_interval_m": args.distance,
        "heading_offset_deg": args.heading_offset,
        "time_base": args.time_base,
        "utc_offset_s": source.get("utc_offset_s"),
        "image_count": len(records),
    }
    with open(path, "w") as fh:
        fh.write(json.dumps({"_meta": meta}) + "\n")
        for record in records:
            row = {k: v for k, v in record.items() if k != "point"}
            row.pop("naming_time", None)
            fh.write(json.dumps(row) + "\n")


def virb__metadata_for(record, source, args, camera):
    return FrameMetadata(
        camera=CAMERA_LABELS[camera],
        video=record["video"], sequence=record["sequence"],
        frame_index=record["frame_index"], pts=record["pts_seconds"],
        distance=record.get("distance_m"), script_name=SCRIPT_NAME,
        make=source.get("make"), model=source.get("model"),
        software=source.get("software"), serial=source.get("serial"),
        captured_utc=parse_utc_timestamp(record.get("datetime_utc")),
        captured_local=(dt.datetime.fromisoformat(record["datetime_local"])
                        if record.get("datetime_local") else None),
        utc_offset_s=record.get("utc_offset_s"),
        lat=record.get("latitude"), lon=record.get("longitude"),
        alt=record.get("altitude_m"), speed=record.get("speed_m_s"),
        course=record.get("course_deg"), img_direction=record.get("img_direction_deg"),
        heading_source=record.get("heading_source", "none"),
        accuracy=record.get("accuracy_m"),
        accuracy_estimated=record.get("accuracy_estimated", False),
        accuracy_source=record.get("accuracy_source"),
        hdop=record.get("hdop"), vdop=record.get("vdop"),
        pdop=record.get("pdop"),
        position_source=record.get("position_source"),
        clamped=record.get("clamped"), estimated=record.get("estimated"),
        keywords=parse_keywords(getattr(args, "keywords", None)),
        extra={"fit": source.get("fit"),
               "camera_orientation_deg": source.get("orientation_deg")},
    )


def virb_choose_segment(telemetry, created_local, duration, tz_offset):
    """Pick the FIT camera-event segment that belongs to *video*.

    Garmin writes several videos into one FIT file.  The container's
    creation_time is camera-local wall clock, so the segment whose local start
    is closest to it wins; if that is unavailable the segment with the closest
    duration is used.  Returns ``(start_fit_s, end_fit_s)`` (either may be
    ``None`` only when no camera events exist).
    """
    segments = telemetry.segments()
    if not segments:
        start = telemetry.samples[0]["t"] if telemetry.samples else None
        end = start + duration if start is not None and duration else None
        return start, end

    best = None
    best_score = None
    for segment in segments:
        start = segment.get("start")
        end = segment.get("end")
        if start is None:
            continue
        score = None
        if created_local is not None:
            utc = telemetry.utc_of(start)
            if utc is not None:
                local = local_from_utc(utc, tz_offset)
                if local is not None:
                    score = abs((local - created_local).total_seconds())
        if score is None and duration:
            length = ((end - start) if end is not None else duration)
            score = abs(length - duration)
        if score is None:
            score = 0.0
        if best_score is None or score < best_score:
            best, best_score = segment, score
    if best is None:
        start = telemetry.samples[0]["t"] if telemetry.samples else None
        return start, (start + duration if start is not None and duration else None)
    start = best.get("start")
    end = best.get("end")
    if end is None and duration:
        end = start + duration
    return start, end


def virb_process_video(video, sequence, output_dir, all_fits, args, telemetry_cache,
                  have_exiftool):
    """Export one video sequence. Returns a summary dict; never raises Fatal."""
    name = os.path.basename(video)
    LOG.info("--- %s -> %s", name, sequence)

    info = virb_probe_video(video)
    LOG.info("  video: %sx%s %s %.3f fps, %s frames, %.2f s",
             info["width"], info["height"], info["codec"], info["fps"],
             info["frames"], info["duration"] or 0.0)
    timeline = probe_frame_timeline(
        video, fps_hint=info["fps"], frame_count_hint=info["frames"],
        duration_hint=info["duration"], logger=LOG)
    LOG.info("  frame timeline: %d frame(s), source=%s",
             len(timeline), timeline.source)

    atoms = virb_read_mp4_atoms(video)
    if atoms.get("fit_reference"):
        LOG.info("  video references FIT: %s", atoms["fit_reference"])
    created_local = virb_parse_creation_time(info.get("creation_time"))

    explicit = args.fit[0] if len(args.fit) == 1 else None
    candidates = ([os.path.abspath(p) for p in args.fit] if args.fit
                  else virb_fits_for_video(video, all_fits, args.fit_directory_name))
    fit_path, fit_ref, problem = virb_pick_fit(atoms, candidates, created_local, explicit)
    if problem:
        LOG.warning("  %s", problem)

    telemetry = None
    if fit_path:
        if fit_path in telemetry_cache:
            telemetry = telemetry_cache[fit_path]
        else:
            try:
                telemetry = virb_Telemetry(fit_path)
            except Exception as exc:                       # noqa: BLE001
                LOG.warning("  could not read %s: %s", fit_path, exc)
                telemetry = None
            telemetry_cache[fit_path] = telemetry
    if telemetry is not None:
        LOG.info("  telemetry: %s (%d GPS samples, IMU %s)",
                 os.path.basename(fit_path), len(telemetry.samples),
                 "present" if telemetry.imu_available else "absent")
    else:
        fit_path = None
        LOG.warning("  no usable FIT telemetry for this video")

    detected_offset = telemetry.local_utc_offset() if telemetry else None
    tz_offset, tz_known = resolve_timezone(
        args, detected_offset, camera=CAMERA_VIRB, logger=LOG)
    if tz_known:
        LOG.info("  camera-local time offset: %s", offset_string(tz_offset))
    else:
        LOG.warning("  camera-local offset unknown; using UTC")

    # ---- align video and telemetry ---------------------------------------
    video_start_fit = None
    video_end_fit = None
    duration = info["duration"]
    if telemetry is not None and telemetry.samples:
        video_start_fit, video_end_fit = virb_choose_segment(
            telemetry, created_local, duration, tz_offset)
        if video_start_fit is not None:
            LOG.info("  video starts at FIT t=%.3f s", video_start_fit)

    video_start_utc = (telemetry.utc_of(video_start_fit)
                       if telemetry and video_start_fit is not None else None)
    if video_start_utc is None and created_local is not None:
        # Garmin writes local wall clock in creation_time; use it as a last
        # resort but never consult the host timezone.
        video_start_utc = (created_local.replace(tzinfo=dt.timezone.utc)
                           - dt.timedelta(seconds=tz_offset if tz_known else 0))
    if video_start_utc is not None:
        LOG.info("  video start (UTC): %s", fmt_utc(video_start_utc))

    # ---- build the time-indexed track ------------------------------------
    track = None
    samples = []
    if telemetry is not None and video_start_fit is not None:
        # A normal clip covers the same FIT span as its playback duration.
        if duration:
            t_end = video_start_fit + duration
            if video_end_fit is not None:
                t_end = min(t_end, video_end_fit)
        elif video_end_fit is not None:
            t_end = video_end_fit
        else:
            t_end = telemetry.samples[-1]["t"]
        for sample in telemetry.sample_window(video_start_fit, t_end):
            item = dict(sample)
            item["t"] = (sample["t"] - video_start_fit)
            item["utc"] = telemetry.utc_of(sample["fit_s"])
            apply_accuracy(item, args.uere)
            samples.append(item)
        if samples:
            track = Track(samples, args.distance_mode)
            LOG.info("  track: %d sample(s), %.0f m of travel",
                     len(samples), track.length)

    choices = []
    skipped = []
    if track is not None and track.samples:
        choices, skipped = select_unique_frames(
            track, timeline, args.distance,
            start_distance=args.start_distance,
            max_position_error=args.max_position_error,
            max_images=args.max_images, mode=args.distance_mode, logger=LOG)
        if skipped:
            LOG.info("  skipped %d target(s) with no acceptable unused frame",
                     len(skipped))

    if not choices:
        if args.fallback_interval > 0 and duration:
            choices = virb__unpositioned_choices(
                timeline, args.fallback_interval, duration, args.max_images)
            LOG.warning("  falling back to a %g s time grid (%d images, no position)",
                        args.fallback_interval, len(choices))
        else:
            LOG.error("  no distance-based capture points; skipping this video")
            return {"video": name, "sequence": sequence, "images": 0,
                    "planned": 0, "skipped": 0, "failed": True}

    firmware = None
    if telemetry is not None and telemetry.device.get("software_raw"):
        firmware = "%.2f" % (telemetry.device["software_raw"] / 100.0)
    elif atoms.get("settings"):
        tail = atoms["settings"].rsplit("_", 1)[-1].strip()
        if re.match(r"^\d+\.\d+$", tail):
            firmware = tail
    source = {
        "video": name,
        "fit": os.path.basename(fit_path) if fit_path else None,
        "fit_ref": fit_ref,
        "make": "Garmin",
        "model": "VIRB Ultra 30",
        "serial": telemetry.device.get("serial") if telemetry else None,
        "software": "VIRB Ultra 30 FW %s" % (firmware or "unknown"),
        "orientation_deg": telemetry.camera_orientation() if telemetry else None,
        "utc_offset_s": tz_offset if tz_known else None,
    }

    records = []
    for choice in choices:
        point = choice.point if choice.point else {}
        if not point and track is not None:
            point = track.at_time(choice.pts)
            choice.point = point
        records.append(virb__record_from_choice(
            choice, track, args, video_start_utc, tz_offset, tz_known,
            CAMERA_VIRB, sequence, name))

    first_time = records[0]["naming_time"]
    sequence_dir = sequence_output_dir(
        output_dir, CAMERA_VIRB, sequence, first_time,
        heading_offset=heading_offset_for_folder(args))
    used_paths = set()
    for record in records:
        record["path"] = unique_path(
            os.path.join(sequence_dir, record["image"]), used_paths)
        record["image"] = os.path.basename(record["path"])

    LOG.info("  %d capture point(s) -> %s", len(records), sequence_dir)
    if args.dry_run:
        LOG.info("  dry run: %d image(s) would be written", len(records))
        return {"video": name, "sequence": sequence, "images": 0,
                "planned": len(records), "skipped": 0, "dir": sequence_dir}

    to_extract = []
    skipped_existing = 0
    for record in records:
        if os.path.exists(record["path"]) and not args.overwrite:
            LOG.debug("  exists, skipping: %s", record["image"])
            skipped_existing += 1
            continue
        to_extract.append(record)

    written_paths = []
    failed_frames = []
    temp_dir = tempfile.mkdtemp(prefix="virb30_")
    try:
        extracted = extract_frames_by_index(
            video, [r["frame_index"] for r in to_extract], temp_dir,
            image_format=args.format, quality=args.quality,
            workers=args.workers, threads=args.threads, timeline=timeline)
        for record in to_extract:
            temp_path = extracted.get(record["frame_index"])
            if not temp_path:
                failed_frames.append(record["frame_index"])
                continue
            os.makedirs(os.path.dirname(record["path"]), exist_ok=True)
            shutil.copyfile(temp_path, record["path"])
            record["written"] = True
            written_paths.append(record["path"])
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)

    if (args.start_distance == 0 and records
            and records[0]["frame_index"] == 0
            and records[0]["frame_index"] in failed_frames):
        LOG.error("  first frame (index 0) could not be extracted; failing %s",
                  name)
        return {"video": name, "sequence": sequence, "images": 0,
                "planned": len(records), "skipped": skipped_existing,
                "failed": True, "dir": sequence_dir}

    LOG.info("  wrote %d image(s), skipped %d existing%s",
             len(written_paths), skipped_existing,
             (", %d failed" % len(failed_frames)) if failed_frames else "")

    if have_exiftool and not args.no_exif and written_paths:
        pairs = [(record["path"], build_exiftool_args(
            virb__metadata_for(record, source, args, CAMERA_VIRB)))
            for record in records if record.get("written")]
        write_metadata(pairs)

    export_samples = (track.samples if track is not None else [])
    export_imu = telemetry.imu if telemetry is not None else None
    if export_imu and video_start_utc is not None and duration:
        window_start = video_start_utc - dt.timedelta(seconds=1.0)
        window_end = video_start_utc + dt.timedelta(seconds=duration + 1.0)

        def in_window(row):
            utc = row.get("utc")
            return utc is None or window_start <= utc <= window_end

        export_imu = {key: [row for row in rows if in_window(row)]
                      for key, rows in export_imu.items()}
    export_track_files(
        sequence_dir, sequence, export_samples, records,
        track_export=args.track_export, camera=CAMERA_VIRB, video=name,
        imu=export_imu, video_start_utc=video_start_utc,
        telemetry_export=args.telemetry_export,
        calibration=(telemetry.sensor_calibration if telemetry else None),
        units=virb_IMU_UNITS,
        extra={"distance_interval_m": args.distance,
               "time_base": args.time_base})

    if args.sidecars:
        os.makedirs(sequence_dir, exist_ok=True)
        virb__write_sidecar(os.path.join(sequence_dir, sequence + ".track.jsonl"),
                       sequence, name, records, source, args)

    entries = [{k: v for k, v in record.items()
                if k not in ("point", "naming_time")}
               for record in records if record.get("written")]
    return {"video": name, "sequence": sequence, "images": len(written_paths),
            "planned": len(records), "skipped": skipped_existing,
            "entries": entries, "dir": sequence_dir}


# =========================================================================
# GoPro HERO pipeline
# =========================================================================

@dataclass
class hero_VideoInfo:
    path: str
    duration: float
    fps: float
    frame_count: int
    gpmd: Optional[int]
    width: int
    height: int
    creation_time: Optional[dt.datetime]
    device_name: Optional[str] = None
    firmware: Optional[str] = None
    serial: Optional[str] = None


def hero__parse_rate(rate) -> float:
    if not rate:
        return 0.0
    text = str(rate)
    if "/" in text:
        try:
            num, den = text.split("/", 1)
            return float(num) / float(den) if float(den) else 0.0
        except ValueError:
            return 0.0
    try:
        return float(text)
    except ValueError:
        return 0.0


def hero_probe(path: str) -> hero_VideoInfo:
    meta = probe_metadata(path, FFPROBE)
    videos = meta.video_streams_info()
    if len(videos) != 1:
        raise Fatal("expected one flat video stream, found %d (GoPro MAX EAC "
                    "files are handled by the MAX pipeline)" % len(videos))
    stream = videos[0]
    width = int(stream.get("width") or 0)
    height = int(stream.get("height") or 0)
    fps = hero__parse_rate(stream.get("avg_frame_rate")
                      or stream.get("r_frame_rate")) or 30.0
    duration = float(stream.get("duration") or meta.duration or 0.0)
    try:
        frame_count = int(stream.get("nb_frames"))
    except (TypeError, ValueError):
        frame_count = 0
    if frame_count <= 0:
        frame_count = max(1, int(round(duration * fps))) if duration else 1
    return hero_VideoInfo(
        path=path, duration=duration, fps=fps, frame_count=frame_count,
        gpmd=meta.gpmd_index, width=width, height=height,
        creation_time=parse_utc_timestamp(meta.creation_time),
        firmware=meta.format_tags.get("firmware"),
        serial=meta.format_tags.get("serial_number") or meta.format_tags.get("serial"),
    )


def hero__gpmf_index(info: hero_VideoInfo, args) -> Optional[int]:
    if args.gpmf_stream is not None:
        return int(args.gpmf_stream)
    return info.gpmd


def hero_build_track(info: hero_VideoInfo, args):
    "Return ``(track, gpmf_data)`` or ``(None, None)`` when there is no GPS."
    index = hero__gpmf_index(info, args)
    if index is None:
        return None, None
    payload = extract_gpmf(info.path, index, ffmpeg=FFMPEG)
    data = parse_gpmf(payload, imu_heading=args.imu_heading)
    info.device_name = info.device_name or data.device_name
    LOG.info("  GPMF device: %s; streams: %s", data.device_name,
             ", ".join(sorted(data.streams)) or "none")
    if not data.has_gps:
        data.origin_utc = info.creation_time
        _anchor_imu_utc(data.imu, data.origin_utc)
        return None, data

    origin = info.creation_time
    if origin is None:
        origin = next((sample["utc"] for sample in data.gps_samples
                       if sample.get("utc") is not None), None)
    if origin is None:
        raise Fatal("GPMF GPS samples have no usable timestamps")
    samples = []
    for sample in data.gps_samples:
        utc = sample.get("utc")
        if utc is None:
            continue
        fix = sample.get("fix")
        if fix is not None and fix < 2:
            continue
        item = dict(sample)
        item["t"] = (utc - origin).total_seconds()
        apply_accuracy(item, args.uere)
        samples.append(item)
    if not samples:
        return None, data
    samples.sort(key=lambda sample: sample["t"])
    track = Track(samples, args.distance_mode)
    LOG.info("  telemetry: %d GPS sample(s), %.1f m of travel",
             len(samples), track.length)
    data.origin_utc = origin
    data.time_scale = 1.0
    return track, data


def hero__unpositioned_choices(timeline, interval, duration, max_images):
    frames = select_time_grid_frames(
        timeline, interval, duration=duration, max_images=max_images)
    out = []
    for index in frames:
        pts = timeline.pts(index)
        out.append(FrameChoice(
            target_distance=pts, target_time=pts, frame_index=index, pts=pts,
            point={}, position_error=None, clamped=False, estimated=True))
    return out


def hero__record(choice, args, info: hero_VideoInfo, tz_offset: int, tz_known: bool,
            sequence: str) -> dict:
    point = choice.point or {}
    pts = max(0.0, float(choice.pts))
    utc = None
    if info.creation_time is not None:
        utc = info.creation_time + dt.timedelta(seconds=pts)
    elif point.get("utc") is not None:
        utc = point["utc"]
    if utc is not None and utc.tzinfo is None:
        utc = utc.replace(tzinfo=dt.timezone.utc)
    local = local_from_utc(utc, tz_offset) if tz_known else None
    if args.time_base == "utc" and utc is not None:
        naming = utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
    else:
        naming = local or (utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
                           if utc is not None else None)
    if naming is None:
        naming = dt.datetime(1970, 1, 1)

    heading, heading_source = resolve_heading(
        point, args.heading_offset, args.magnetic_declination)
    accuracy, accuracy_estimated, accuracy_source = derive_accuracy(
        point, args.uere)
    course = point.get("course")
    if course is None:
        course = point.get("gps_course")
    speed = point.get("speed")
    if speed is None:
        speed = point.get("speed2d")
    unpositioned = point.get("lat") is None
    return {
        "sequence": sequence, "video": os.path.basename(info.path),
        "camera": CAMERA_HERO,
        "image": image_filename(naming, args.format),
        "frame_index": int(choice.frame_index), "pts_seconds": round(pts, 3),
        "datetime_local": local.isoformat() if local else None,
        "datetime_utc": fmt_utc(utc) if utc else None,
        "utc_offset_s": tz_offset if tz_known else None,
        "latitude": point.get("lat"), "longitude": point.get("lon"),
        "altitude_m": point.get("alt"), "speed_m_s": speed,
        "course_deg": course, "heading_deg": heading,
        "heading_source": heading_source, "img_direction_deg": heading,
        "accuracy_m": accuracy, "accuracy_estimated": accuracy_estimated,
        "accuracy_source": accuracy_source,
        "hdop": point.get("hdop"), "vdop": point.get("vdop"),
        "pdop": point.get("pdop"),
        "distance_m": None if unpositioned else float(choice.target_distance),
        "track_distance_m": point.get("distance"),
        "position_source": point.get("position_source", "unpositioned"),
        "clamped": bool(point.get("clamped")),
        "estimated": bool(point.get("estimated")),
        "unpositioned": unpositioned,
        "naming_time": naming, "point": point,
    }


def hero__metadata_for(record, info: hero_VideoInfo, args):
    model = args.device_model or info.device_name or "GoPro HERO"
    return FrameMetadata(
        camera=CAMERA_LABELS[CAMERA_HERO], video=record["video"],
        sequence=record["sequence"], frame_index=record["frame_index"],
        pts=record["pts_seconds"], distance=record.get("distance_m"),
        script_name=SCRIPT_NAME, make="GoPro", model=model,
        software=info.firmware, serial=info.serial,
        captured_utc=parse_utc_timestamp(record.get("datetime_utc")),
        captured_local=(dt.datetime.fromisoformat(record["datetime_local"])
                        if record.get("datetime_local") else None),
        utc_offset_s=record.get("utc_offset_s"),
        lat=record.get("latitude"), lon=record.get("longitude"),
        alt=record.get("altitude_m"), speed=record.get("speed_m_s"),
        course=record.get("course_deg"), img_direction=record.get("img_direction_deg"),
        heading_source=record.get("heading_source", "none"),
        accuracy=record.get("accuracy_m"),
        accuracy_estimated=record.get("accuracy_estimated", False),
        accuracy_source=record.get("accuracy_source"),
        hdop=record.get("hdop"), vdop=record.get("vdop"),
        pdop=record.get("pdop"),
        position_source=record.get("position_source"),
        clamped=record.get("clamped"), estimated=record.get("estimated"),
        image_width=info.width, image_height=info.height, panorama=False,
        keywords=parse_keywords(args.keywords),
        extra={"unpositioned": record.get("unpositioned")},
    )


def hero_process_video(path: str, sequence: str, args) -> dict:
    name = os.path.basename(path)
    result = {"video": name, "sequence": sequence, "images": 0,
              "planned": 0, "failed": False, "entries": [], "dir": None}
    LOG.info("--- %s -> %s", name, sequence)
    info = hero_probe(path)
    LOG.info("  video: %dx%d %.4f fps, %d frame(s), %.2f s", info.width,
             info.height, info.fps, info.frame_count, info.duration)
    timeline = probe_frame_timeline(
        path, fps_hint=info.fps, frame_count_hint=info.frame_count,
        duration_hint=info.duration, logger=LOG)
    if len(timeline) > info.frame_count:
        timeline = FrameTimeline(timeline.times[:info.frame_count],
                                     timeline.source, timeline.fps)
    LOG.info("  frame timeline: %d frame(s), source=%s", len(timeline),
             timeline.source)

    try:
        track, gpmf = hero_build_track(info, args)
    except (Fatal, GpmfError) as exc:
        LOG.error("  %s", exc)
        result["failed"] = True
        return result

    tz_offset, tz_known = resolve_timezone(
        args, None, camera=CAMERA_HERO, logger=LOG)

    if track is None:
        if not args.allow_missing_gps:
            LOG.error("  no GPS telemetry; use --allow-missing-gps "
                      "--fallback-interval SECONDS to export unpositioned frames")
            result["failed"] = True
            return result
        if args.fallback_interval <= 0:
            LOG.error("  --allow-missing-gps requires a positive "
                      "--fallback-interval")
            result["failed"] = True
            return result
        choices = hero__unpositioned_choices(
            timeline, args.fallback_interval, info.duration, args.max_images)
        LOG.warning("  exporting %d unpositioned frame(s) on a %g s time grid",
                    len(choices), args.fallback_interval)
        skipped = []
    else:
        choices, skipped = select_unique_frames(
            track, timeline, args.distance,
            start_distance=args.start_distance,
            max_position_error=args.max_position_error,
            max_images=args.max_images, mode=args.distance_mode, logger=LOG)
        if skipped:
            LOG.info("  skipped %d target(s) with no acceptable unused frame",
                     len(skipped))
        if not choices:
            LOG.error("  no capture point could be matched to a unique frame")
            result["failed"] = True
            return result

    records = [hero__record(choice, args, info, tz_offset, tz_known, sequence)
               for choice in choices]
    first_time = records[0]["naming_time"]
    sequence_dir = sequence_output_dir(
        args.output, CAMERA_HERO, sequence, first_time,
        heading_offset=heading_offset_for_folder(args))
    used_paths = set()
    for record in records:
        record["path"] = unique_path(
            os.path.join(sequence_dir, record["image"]), used_paths)
        record["image"] = os.path.basename(record["path"])
    result["planned"] = len(records)
    result["dir"] = sequence_dir
    LOG.info("  %d capture point(s) -> %s", len(records), sequence_dir)

    if args.dry_run:
        for record in records:
            LOG.info("    frame=%d pts=%.3fs %s hdg=%s d=%s -> %s",
                     record["frame_index"], record["pts_seconds"],
                     (record["datetime_utc"] or "no-utc")[11:23],
                     record["heading_deg"],
                     ("unpositioned" if record.get("unpositioned")
                      else ("%.1fm" % record["distance_m"]
                            if record.get("distance_m") is not None
                            else "-")),
                     record["image"])
        return result

    to_extract = []
    skipped_existing = 0
    for record in records:
        if os.path.exists(record["path"]) and not args.overwrite:
            skipped_existing += 1
            continue
        to_extract.append(record)

    written = []
    failed_frames = []
    temp_dir = tempfile.mkdtemp(prefix="hero_")
    try:
        extracted = extract_frames_by_index(
            path, [record["frame_index"] for record in to_extract], temp_dir,
            image_format=args.format, quality=args.quality,
            workers=args.workers, threads=args.threads, timeline=timeline)
        for record in to_extract:
            temp = extracted.get(record["frame_index"])
            if not temp:
                failed_frames.append(record["frame_index"])
                continue
            os.makedirs(os.path.dirname(record["path"]), exist_ok=True)
            shutil.copyfile(temp, record["path"])
            record["written"] = True
            written.append(record)
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)

    if (args.start_distance == 0 and records
            and records[0]["frame_index"] == 0
            and records[0]["frame_index"] in failed_frames):
        LOG.error("  first frame (index 0) could not be extracted; failing %s",
                  name)
        result["failed"] = True
        return result
    result["images"] = len(written)
    LOG.info("  wrote %d image(s), skipped %d existing%s", len(written),
             skipped_existing,
             (", %d failed" % len(failed_frames)) if failed_frames else "")

    if written and not args.no_exif:
        pairs = [(record["path"], build_exiftool_args(
            hero__metadata_for(record, info, args))) for record in written]
        write_metadata(pairs)

    export_track_files(
        sequence_dir, sequence, track.samples if track else [], records,
        track_export=args.track_export, camera=CAMERA_HERO, video=name,
        imu=gpmf.imu if gpmf else None,
        video_start_utc=gpmf.origin_utc if gpmf else None,
        telemetry_export=args.telemetry_export,
        extra={"distance_interval_m": args.distance,
               "time_base": args.time_base})
    result["entries"] = [{k: v for k, v in record.items()
                          if k not in ("point", "naming_time")}
                         for record in written]
    return result




# ===========================================================================
# Command-line interface
# ===========================================================================
CAMERA_ORDER = (CAMERA_BLACKVUE, CAMERA_MAX, CAMERA_VIRB,
                CAMERA_HERO)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=SCRIPT_NAME,
        description="Export geotagged still images from supported action-camera "
                    "videos every N metres of travel. The camera is detected "
                    "per file (Garmin VIRB Ultra 30, GoPro MAX 360, BlackVue "
                    "DR900 or GoPro HERO), so one input tree may mix cameras.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=("Examples:\n"
                "  %(prog)s -i video/ -o images/ -d 7\n"
                "  %(prog)s -i video/2026-10-18-virb-4k -o images/ -d 8\n"
                "  %(prog)s -i video/hero -o images/ --allow-missing-gps "
                "--fallback-interval 1\n\n"
                "Only files recorded by a supported camera are processed; "
                "everything else found under --input is skipped with a logged "
                "reason.\n\n"
                "Time base: file names and folders use camera-local time by "
                "default; use --time-base utc for UTC."))
    add_common_arguments(parser)
    add_gps_filter_arguments(parser)
    add_track_export_arguments(parser)

    hero = parser.add_argument_group("GoPro HERO telemetry")
    hero.add_argument("--gpmf-stream", type=int, default=None, metavar="INDEX",
                      help="absolute ffmpeg stream index of the gpmd telemetry "
                           "track (default: auto-detect)")
    hero.add_argument("--device-model", default=None, metavar="NAME",
                      help="override the EXIF camera model for HERO videos")
    hero.add_argument("--allow-missing-gps", action="store_true",
                      help="export unpositioned time-lapse frames when a HERO "
                           "video has no GPS (requires --fallback-interval)")

    geometry = parser.add_argument_group("GoPro MAX 360 output geometry")
    geometry.add_argument("--width", type=int, default=max360_DEFAULT_WIDTH,
                          help="MAX output image width (2:1 with --height)")
    geometry.add_argument("--height", type=int, default=max360_DEFAULT_HEIGHT,
                          help="MAX output image height")
    geometry.add_argument("--interp", default="cubic",
                          choices=["cubic", "linear", "nearest", "lanczos"],
                          help="MAX v360 reprojection interpolation")
    geometry.add_argument("--stitch-backend",
                          choices=("auto", "ffmpeg", "max2sphere"),
                          default="auto",
                          help="MAX EAC stitching backend (default: ffmpeg "
                               "when available)")
    geometry.add_argument("--heading-window", type=int, default=4,
                          help="MAX GPS samples either side used for heading "
                               "smoothing (compatibility option)")
    geometry.add_argument("--keep-filter", action="store_true",
                          help="keep the generated MAX ffmpeg filter graph")
    parser.add_argument("--imu-heading", action="store_true",
                        help="derive compass heading from MAGN + GRAV streams "
                             "instead of GPS course (HERO/MAX)")

    virb = parser.add_argument_group("Garmin VIRB telemetry")
    virb.add_argument("--fit", action="append", default=[], metavar="FILE",
                      help="use this FIT file (repeatable); one value applies "
                           "to all VIRB videos")
    virb.add_argument("--fit-directory-name", default=virb_DEFAULT_FIT_DIR,
                      metavar="NAME",
                      help="sibling folder holding FIT telemetry (default "
                           "'%s')" % virb_DEFAULT_FIT_DIR)
    virb.add_argument("--video-glob", dest="video_glob", default=None,
                      metavar="PATTERN", help=argparse.SUPPRESS)

    # Backwards compatibility: the input directory may be positional.
    for action in parser._actions:
        if action.dest == "input" and action.option_strings:
            action.required = False
    parser.add_argument("input_alias", nargs="?", default=None, metavar="INPUT",
                        help=argparse.SUPPRESS)
    return parser


def validate(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    validate_common_args(parser, args)
    if args.fallback_interval < 0:
        parser.error("--fallback-interval must not be negative")
    if args.gpmf_stream is not None and args.gpmf_stream < 0:
        parser.error("--gpmf-stream must not be negative")
    if args.allow_missing_gps and args.fallback_interval <= 0:
        parser.error("--allow-missing-gps requires --fallback-interval SECONDS")
    if args.width < 2 or args.height < 2:
        parser.error("--width/--height must be at least 2")
    if args.width % 2 or args.height % 2:
        parser.error("--width/--height must be even for yuv output")
    if args.heading_window < 1:
        parser.error("--heading-window must be at least 1")
    for path in args.fit:
        if not os.path.isfile(path):
            parser.error("--fit file not found: %s" % path)


def detect_groups(videos: Sequence[str]) -> Dict[str, List[str]]:
    """Detect every candidate and group the accepted files by camera."""
    groups: Dict[str, List[str]] = {camera: [] for camera in CAMERA_ORDER}
    counts: Dict[Optional[str], int] = {camera: 0 for camera in CAMERA_ORDER}
    counts[None] = 0
    for video in videos:
        detection = detect_camera(video, ffprobe=FFPROBE)
        counts[detection.camera] = counts.get(detection.camera, 0) + 1
        name = os.path.basename(video)
        if detection.camera is None:
            LOG.warning("skipping %s: %s", name,
                        detection.reason or "unsupported camera")
            continue
        LOG.info("camera %s: %s", detection.label, name)
        LOG.debug("  evidence: %s",
                  "; ".join(detection.evidence.get(detection.camera, []))
                  or "none")
        groups[detection.camera].append(video)
    LOG.info("camera detection summary: %s", summarize_detections(counts))
    return groups


def _failure(video: str, sequence: str) -> dict:
    return {"video": os.path.basename(video), "sequence": sequence,
            "images": 0, "planned": 0, "skipped": 0, "failed": True,
            "entries": [], "dir": None}


def process_group(camera: str, videos: Sequence[str], root: str,
                  output_dir: str, args: argparse.Namespace,
                  have_exiftool: bool) -> Tuple[List[dict], List[dict], int]:
    """Run one camera pipeline over its accepted videos."""
    sequence_ids = build_sequence_ids(videos, root)
    fits: List[str] = []
    telemetry_cache: dict = {}
    if camera == CAMERA_VIRB:
        fits = virb_find_fits(root, exclude_dirs=[output_dir])
        LOG.info("found %d %s video(s) and %d FIT file(s) under %s",
                 len(videos), CAMERA_LABELS[camera], len(fits), root)
    else:
        LOG.info("found %d %s video(s) under %s", len(videos),
                 CAMERA_LABELS[camera], root)

    summaries: List[dict] = []
    entries: List[dict] = []
    for video in videos:
        sequence = sequence_ids[video]
        try:
            if camera == CAMERA_BLACKVUE:
                summary = blackvue_process_video(video, sequence, output_dir,
                                                 args, have_exiftool)
            elif camera == CAMERA_MAX:
                summary = max360_process_video(video, sequence, args)
            elif camera == CAMERA_VIRB:
                summary = virb_process_video(video, sequence, output_dir, fits,
                                             args, telemetry_cache,
                                             have_exiftool)
            else:
                summary = hero_process_video(video, sequence, args)
            summaries.append(summary)
            entries.extend(summary.get("entries", []))
        except Fatal as exc:
            LOG.error("%s: %s", os.path.basename(video), exc)
            summaries.append(_failure(video, sequence))
            if args.fail_fast:
                break
        except Exception as exc:                            # noqa: BLE001
            LOG.exception("%s: unexpected error: %s",
                          os.path.basename(video), exc)
            summaries.append(_failure(video, sequence))
            if args.fail_fast:
                break
    return summaries, entries, len(videos)


def write_manifest(path: str, entries: Sequence[dict],
                   summaries: Sequence[dict], args: argparse.Namespace,
                   cameras: Sequence[str]) -> None:
    single = cameras[0] if len(cameras) == 1 else "mixed"
    label = (CAMERA_LABELS[single] if single in CAMERA_LABELS
             else "Mixed cameras")
    payload = {
        "generator": SCRIPT_NAME,
        "camera": single,
        "camera_label": label,
        "cameras": list(cameras),
        "distance_interval_m": args.distance,
        "image_count": len(entries),
        "images": list(entries),
        "videos": [{k: v for k, v in summary.items() if k != "entries"}
                   for summary in summaries],
    }
    with open(path, "w") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
        fh.write("\n")


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.input is None:
        args.input = args.input_alias
    if args.input is None:
        parser.error("--input is required (or give a positional input path)")
    validate(parser, args)
    setup_logging(log_level(args))

    for tool in (FFMPEG, FFPROBE):
        if shutil.which(tool) is None:
            LOG.error("required program not found on PATH: %s", tool)
            return 2
    have_exiftool = shutil.which(EXIFTOOL) is not None
    if not have_exiftool and not args.no_exif:
        LOG.warning("exiftool not found on PATH; images will have no metadata")

    input_arg = os.path.abspath(args.input)
    root = input_arg if os.path.isdir(input_arg) else os.path.dirname(input_arg)
    output_dir = os.path.abspath(args.output)
    args.output = output_dir

    extensions = resolve_extensions(args)
    videos = discover_videos(input_arg, extensions, args.recursive)
    if not videos:
        LOG.error("no video files matching %s found under %s",
                  ",".join(extensions), input_arg)
        return 2

    groups = detect_groups(videos)
    found = sum(len(group) for group in groups.values())
    if not found:
        LOG.error("no supported camera video found among %d candidate file(s)",
                  len(videos))
        return 2

    if not args.dry_run:
        try:
            os.makedirs(output_dir, exist_ok=True)
        except OSError as exc:
            LOG.error("cannot create output directory %s: %s", output_dir, exc)
            return 2

    summaries: List[dict] = []
    entries: List[dict] = []
    attempted = 0
    cameras: List[str] = []
    for camera in CAMERA_ORDER:
        if not groups[camera]:
            continue
        cameras.append(camera)
        group_summaries, group_entries, group_attempted = process_group(
            camera, groups[camera], root, output_dir, args, have_exiftool)
        summaries.extend(group_summaries)
        entries.extend(group_entries)
        attempted += group_attempted
        if args.fail_fast and any(s.get("failed") for s in group_summaries):
            break

    total_images = sum(summary.get("images", 0) for summary in summaries)
    total_planned = sum(summary.get("planned", 0) for summary in summaries)
    failed = [summary for summary in summaries if summary.get("failed")]
    LOG.info("done: %d image(s) from %d/%d video(s)%s -> %s",
             total_images, len(summaries) - len(failed), attempted,
             (", %d failure(s)" % len(failed)) if failed else "", output_dir)
    if args.dry_run:
        LOG.info("dry run: %d image(s) would be written", total_planned)

    if args.manifest and not args.dry_run:
        try:
            write_manifest(args.manifest, entries, summaries, args, cameras)
            LOG.info("wrote manifest: %s (%d image(s))",
                     args.manifest, len(entries))
        except OSError as exc:
            LOG.error("cannot write manifest %s: %s", args.manifest, exc)

    if failed:
        LOG.warning("%d video(s) produced no images (see messages above)",
                    len(failed))
        return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Fatal as exc:
        LOG.error("%s", exc)
        sys.exit(2)
    except KeyboardInterrupt:
        sys.stderr.write("\ninterrupted\n")
        sys.exit(130)
