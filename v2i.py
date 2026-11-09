#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v2i.py -- one-file geotagged still-image exporter for action cameras.

Everything needed to identify videos, read telemetry, select frames, extract
stills and write metadata lives in this single file.  Supported cameras:

    * Garmin VIRB Ultra 30   (MP4 + sibling FIT telemetry)
    * GoPro Max, GoPro Max2  (``.360`` EAC hemispheres -> equirectangular)
    * BlackVue DR900         (MP4 with embedded NMEA)
    * GoPro HERO             (flat MP4 with GPMF telemetry)

The camera is detected per file, so one input tree may mix manufacturers.  Each
pipeline walks its telemetry track, picks a unique frame every ``--distance``
metres of travel and writes it with EXIF/XMP metadata suitable for Mapillary
and Panoramax.  Unrecognised files are skipped with a logged reason.

Only the Python standard library is used.  ``ffmpeg``, ``ffprobe`` and
``exiftool`` must be available on ``PATH``; ``max2sphere`` is optional.

Notice: this code was written largely by AI coding assistants under the
direction of the project owner, and tested on the owner's recordings.  It has
not been independently reviewed line by line; check the output (positions,
timestamps, headings, EXIF) before publishing images.
"""

from __future__ import annotations

import argparse
import bisect
import copy
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
from typing import (Callable, Dict, Iterable, Iterator, List, Mapping,
                    Optional, Sequence, Tuple, Union)

SCRIPT_NAME = os.path.basename(os.path.realpath(__file__))
LOG = logging.getLogger("v2i")
EXIFTOOL = os.environ.get("V2I_EXIFTOOL", "exiftool")


class Fatal(Exception):
    """A problem that stops processing the current file."""


# =========================================================================
# Shared helpers: CLI, detection, timelines, extraction and tracks
# =========================================================================

__version__ = "0.0.4"

# Written to EXIF ``Software``, ``XMP:CreatorTool`` and the GPano
# ``StitchingSoftware`` tag of every exported image, e.g. ``v2i.py v0.0.4``.
SOFTWARE_TAG = "%s v%s" % (SCRIPT_NAME, __version__)

# Values ``--level-horizon`` accepted before it became an on/off switch; they
# are rejected with an explanation instead of a bare argparse error.
LEVEL_HORIZON_OLD_VALUES = ("no", "rotate", "exif")

FFPROBE = os.environ.get("V2I_FFPROBE", "ffprobe")
FFMPEG = os.environ.get("V2I_FFMPEG", "ffmpeg")

# ---------------------------------------------------------------------------
# Camera identities
# ---------------------------------------------------------------------------
CAMERA_VIRB = "garmin-virb-ultra-30"
CAMERA_MAX = "gopro-max"
CAMERA_MAX2 = "gopro-max2"
CAMERA_BLACKVUE = "blackvue-dr900"
CAMERA_HERO = "gopro-hero"

CAMERA_LABELS = {
    CAMERA_VIRB: "Garmin VIRB Ultra 30",
    CAMERA_MAX: "GoPro Max",
    CAMERA_MAX2: "GoPro Max2",
    CAMERA_BLACKVUE: "BlackVue DR900",
    CAMERA_HERO: "GoPro HERO",
}

# Stable output-directory suffix (``YYYY-MM-DD-<suffix>``).
CAMERA_SUFFIX = {
    CAMERA_VIRB: "virb30",
    CAMERA_MAX: "max",
    CAMERA_MAX2: "max2",
    CAMERA_BLACKVUE: "blackvue",
    CAMERA_HERO: "hero",
}

# GoPro 360 cameras: separate cameras (labels, counts, output folders) that
# share the EAC stitching pipeline (``max360_*`` functions).
CAMERA_360 = (CAMERA_MAX, CAMERA_MAX2)

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


@dataclass
class GoProSettings:
    """Camera facts from the ``moov/udta/GPMF`` "Global Settings" block.

    Every GoPro file carries this block in its container header (the same
    place the camera's own photos take their EXIF from).  ``tz_offset_s`` is
    the camera's configured UTC offset (``TZON``, minutes, daylight saving
    included) and ``local_created`` the camera-local wall clock (``CDAT``);
    the GoPro Max does not write either, the MAX2 and HERO11 do.
    """
    model: Optional[str] = None             # MINF, e.g. "GoPro Max", "MAX2"
    firmware: Optional[str] = None          # FMWR
    serial: Optional[str] = None            # CASN
    tz_offset_s: Optional[int] = None       # TZON * 60
    local_created: Optional[dt.datetime] = None   # CDAT (naive local)


def _udta_gpmf(path: str) -> Optional[bytes]:
    """Return the raw ``moov/udta/GPMF`` payload, or ``None``.

    Only atom headers are read while walking, so this is cheap even for
    multi-gigabyte files whose ``moov`` sits at the end.
    """
    try:
        size = os.path.getsize(path)
        with open(path, "rb") as fh:
            def walk(start: int, end: int, depth: int) -> Optional[bytes]:
                pos = start
                while pos + 8 <= end:
                    fh.seek(pos)
                    head = fh.read(16)
                    if len(head) < 8:
                        return None
                    atom_size, kind = struct.unpack(">I4s", head[:8])
                    header = 8
                    if atom_size == 1 and len(head) >= 16:
                        atom_size = struct.unpack(">Q", head[8:16])[0]
                        header = 16
                    elif atom_size == 0:
                        atom_size = end - pos
                    if atom_size < header:
                        return None
                    if depth == 0 and kind == b"moov":
                        return walk(pos + header, pos + atom_size, 1)
                    if depth == 1 and kind == b"udta":
                        return walk(pos + header, pos + atom_size, 2)
                    if depth == 2 and kind == b"GPMF":
                        length = min(atom_size - header, 1 << 20)
                        fh.seek(pos + header)
                        return fh.read(length)
                    pos += atom_size
                return None
            return walk(0, size, 0)
    except (OSError, struct.error):
        return None


@lru_cache(maxsize=64)
def read_gopro_settings(path: str) -> GoProSettings:
    """Read :class:`GoProSettings`; missing fields stay ``None``."""
    settings = GoProSettings()
    buf = _udta_gpmf(path)
    if not buf:
        return settings
    try:
        for elem in iter_elements(buf):
            if elem.typ == TYPE_NEST:
                continue
            if elem.key == "MINF" and settings.model is None:
                settings.model = as_text(buf, elem).strip() or None
            elif elem.key == "FMWR" and settings.firmware is None:
                settings.firmware = as_text(buf, elem).strip() or None
            elif elem.key == "CASN" and settings.serial is None:
                settings.serial = as_text(buf, elem).strip() or None
            elif elem.key == "TZON" and settings.tz_offset_s is None:
                values = decode(buf, elem)
                if isinstance(values, list) and values:
                    minutes = int(values[0])
                    if abs(minutes) <= 14 * 60:
                        settings.tz_offset_s = minutes * 60
            elif elem.key == "CDAT" and settings.local_created is None:
                values = decode(buf, elem)
                if isinstance(values, list) and values and values[0] > 0:
                    settings.local_created = dt.datetime.fromtimestamp(
                        int(values[0]), dt.timezone.utc).replace(tzinfo=None)
    except (GpmfError, struct.error, ValueError, OverflowError, OSError):
        pass
    return settings


# EXIF ``Model`` for the GoPro 360 cameras.  The MAX writes "GoPro Max" into
# its own photos; the MAX2 reports only "MAX2", so it gets the same title case
# ("GoPro Max2") for consistency.  Other models (HERO) keep the camera's text.
GOPRO_360_MODEL_NAMES = {"MAX": "GoPro Max", "MAX2": "GoPro Max2"}


def gopro_model_name(minf: Optional[str], fallback: str) -> str:
    """EXIF ``Model``: "GoPro Max" / "GoPro Max2", else the camera's text."""
    text = (minf or "").strip()
    if not text:
        return fallback
    key = re.sub(r"^GOPRO\s*", "", text.upper()).replace(" ", "")
    if key in GOPRO_360_MODEL_NAMES:
        return GOPRO_360_MODEL_NAMES[key]
    return text if text.lower().startswith("gopro") else "GoPro " + text


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


# GoPro Max2 hemisphere strip size, used when no device name is available.
MAX2_STRIP_SIZE = (5888, 1920)


def gopro_360_camera(path: str, device_name: Optional[str] = None,
                     streams: Optional[Sequence[dict]] = None
                     ) -> Tuple[str, str]:
    """Tell a GoPro Max from a MAX2: ``(camera, evidence)``.

    The GPMF device name (``MAX2`` / ``GoPro Max``) decides, then the model
    in the file's Global Settings block (``MINF``), then the hemisphere size
    (MAX2 strips are 5888x1920).  The default is the MAX.
    """
    for source, value in (("GPMF device name", device_name),
                          ("camera model", read_gopro_settings(path).model)):
        text = (value or "").upper().replace(" ", "")
        if "MAX2" in text:
            return CAMERA_MAX2, "%s %r" % (source, value)
        if "MAX" in text:
            return CAMERA_MAX, "%s %r" % (source, value)
    for stream in streams or []:
        size = (int(stream.get("width") or 0), int(stream.get("height") or 0))
        if size == MAX2_STRIP_SIZE:
            return CAMERA_MAX2, "%dx%d MAX2 hemisphere layout" % size
    return CAMERA_MAX, "dual EAC layout of the GoPro Max"


def detect_camera(path: str, *, ffprobe: str = FFPROBE) -> Detection:
    """Identify the camera that recorded *path* from its metadata.

    GoPro 360 files are first recognised as a 360 GoPro, then told apart
    (MAX or MAX2) by :func:`gopro_360_camera`.
    """
    detection = _detect_camera_family(path, ffprobe=ffprobe)
    if detection.camera != CAMERA_MAX:
        return detection
    streams: List[dict] = []
    if not detection.device_name:
        try:
            streams = probe_metadata(path, ffprobe).video_streams_info()
        except UnreadableVideo:
            streams = []
    camera, why = gopro_360_camera(path, detection.device_name, streams)
    evidence = dict(detection.evidence)
    if camera == CAMERA_MAX2:
        evidence[CAMERA_MAX2] = list(evidence.get(CAMERA_MAX, [])) + [why]
        evidence[CAMERA_MAX] = []
    else:
        evidence[CAMERA_MAX] = list(evidence.get(CAMERA_MAX, []))
    return Detection(camera, CAMERA_LABELS[camera], evidence, None,
                     detection.device_name)


def _detect_camera_family(path: str, *, ffprobe: str = FFPROBE) -> Detection:
    """Identify the camera family; GoPro 360 files all report ``CAMERA_MAX``.

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
        CAMERA_VIRB: [], CAMERA_MAX: [], CAMERA_MAX2: [], CAMERA_BLACKVUE: [],
        CAMERA_HERO: [],
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
        evidence[CAMERA_MAX].append("GoPro Max file name (%s)" % name)
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
        labels = "GoPro Max / GoPro HERO"
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


DETECT_WORKERS = 8


def detect_all(videos: Sequence[str]) -> List[Detection]:
    """Run :func:`detect_camera` for every video, several at a time.

    Detection is dominated by ffprobe/ffmpeg start-up and I/O, so a small
    thread pool speeds up large folders.  Results keep the input order.
    """
    workers = min(DETECT_WORKERS, os.cpu_count() or 1, len(videos))
    if workers <= 1:
        return [detect_camera(video, ffprobe=FFPROBE) for video in videos]
    with ThreadPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(
            lambda video: detect_camera(video, ffprobe=FFPROBE), videos))


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
             for camera in CAMERA_ORDER]
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
    for video, detection in zip(videos, detect_all(videos)):
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
    parser.add_argument("--heading-offset", dest="heading_offset",
                        default=None, metavar="DEGREES|auto",
                        help="clockwise angle (0-360) from the direction of "
                             "travel to the camera's front lens, e.g. 90 when "
                             "the lens faces right; 'auto' reads it from each "
                             "video's folder name (a separate 3-digit part, "
                             "e.g. 2026-11-05-max-090-30fps).  360 videos "
                             "are rotated so the direction of travel is the "
                             "image centre (see --no-recenter-forward); flat "
                             "videos keep the pixels, add the offset to the "
                             "heading and get -NNN on their output folder")
    parser.add_argument("--offset", dest="heading_offset", default=None,
                        metavar="DEGREES|auto",
                        help="alias for --heading-offset")
    parser.add_argument("--recenter-forward", dest="recenter_forward",
                        action=argparse.BooleanOptionalAction, default=True,
                        help="with a heading offset, rotate 360 (MAX) images "
                             "so the direction of travel is the image centre "
                             "(default); --no-recenter-forward keeps the "
                             "front lens at the centre, adds the offset to "
                             "the heading and appends -NNN to the output "
                             "folder.  Ignored for flat videos")
    parser.add_argument("--magnetic-declination", type=float, default=0.0,
                        metavar="DEGREES",
                        help="east-positive magnetic declination added to "
                             "compass/IMU headings to obtain true north")
    parser.add_argument("--uere", type=float, default=5.0, metavar="METERS",
                        help="user-equivalent range error used as "
                             "accuracy_m = HDOP x UERE when no measured "
                             "accuracy exists (default 5.0)")
    parser.add_argument("--max-dop", type=float, default=GPS_MAX_DOP,
                        metavar="DOP",
                        help="drop GoPro GPS readings whose dilution of "
                             "precision is above DOP (default %g; 0 keeps "
                             "every reading).  While the receiver is still "
                             "acquiring, GoPro reports a 2D fix with DOP "
                             "99.99 and positions that jump hundreds of "
                             "metres" % GPS_MAX_DOP)
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
    parser.add_argument("--level-horizon", "--horizon", dest="level_horizon",
                        action="store_true",
                        help="level 360 (MAX) photospheres: rotate the sphere "
                             "so the horizon is level, using the camera's own "
                             "gravity sensor smoothed over --level-window.  "
                             "Off by default; ignored for flat videos")
    parser.add_argument("--horizon-gradient", "--level-gradient",
                        dest="horizon_gradient", action="store_true",
                        help="opt-in, with --level-horizon: follow the road "
                             "gradient from GPS altitude instead of averaging "
                             "it away, so hills do not leave the horizon "
                             "tilted (uses the heading offset as the mount "
                             "angle); see README section 6.5")
    parser.add_argument("--level-window", "--horizon-window",
                        dest="level_window", type=float,
                        default=LEVEL_WINDOW_DEFAULT, metavar="SECONDS",
                        help="real-time window over which gravity samples are "
                             "averaged (Gaussian weights) for each leveled "
                             "image (default %g).  A vehicle's turns and "
                             "braking swing GoPro's gravity estimate by "
                             "several degrees, so keep this long for mounted "
                             "cameras; 0 uses the nearest sample (handheld)"
                             % LEVEL_WINDOW_DEFAULT)
    parser.add_argument("--track-export", choices=("both", "gpx", "geojson", "none"),
                        default="both",
                        help="write a GPX/GeoJSON track per sequence (default both)")
    parser.add_argument("--save-telemetry", dest="track_export",
                        action="store_const", const="both",
                        help="legacy alias for --track-export both")
    parser.add_argument("--telemetry-export",
                        choices=("both", "points", "images", "none"),
                        default="both",
                        help="write detailed telemetry GeoJSON next to each "
                             "sequence: every raw GPS/IMU point (points), "
                             "per-image snapshots (images) or both (default)")
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
                        help="if a video has no usable GPS position, sample every "
                             "N seconds instead of skipping it, for any camera "
                             "(default 0 = skip)")


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
    try:
        args.heading_offset_auto, args.heading_offset_value = \
            parse_heading_offset_arg(args.heading_offset)
    except ValueError as exc:
        parser.error(str(exc))
    # Per-video values (folder offset, re-centring) are filled in later by
    # video_args(); these defaults describe "no offset".
    args.heading_offset_given = False
    args.heading_offset = 0.0
    args.recenter_deg = 0.0
    if not (-180.0 <= args.magnetic_declination <= 180.0):
        parser.error("--magnetic-declination must be between -180 and +180 "
                     "degrees (got %g)" % args.magnetic_declination)
    if args.uere <= 0:
        parser.error("--uere must be greater than zero (got %g)" % args.uere)
    if args.max_dop < 0:
        parser.error("--max-dop must be 0 (off) or positive (got %g)"
                     % args.max_dop)
    if args.horizon_gradient and not args.level_horizon:
        parser.error("--horizon-gradient needs --level-horizon")
    if args.level_window < 0:
        parser.error("--level-window must not be negative (got %g)"
                     % args.level_window)
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


def timezone_from_clock_skew(skew_s: Optional[float]) -> Optional[int]:
    """Infer a camera UTC offset from ``wall clock - GPS UTC``.

    GoPro firmware may write the camera's *local* wall clock into the MP4
    ``creation_time`` even though the field looks like UTC.  Compared with the
    authoritative GPS clock the difference is the UTC offset plus a few seconds
    of GPS-lock/stream-start delay, so round it to the nearest quarter hour
    (every real-world offset is a multiple of 15 minutes).  Skews outside the
    real-world UTC offset range are treated as a broken clock, not a timezone.
    """
    if skew_s is None:
        return None
    offset = int(round(float(skew_s) / 900.0) * 900)
    if abs(offset) > 14 * 3600:
        return None
    return offset


# A camera UTC offset further than this from the solar offset of the GPS
# longitude (longitude / 15 h) is almost certainly a wrong camera setting.
# Real-world zones stay within about 3 h of solar time (western China, Spain
# in summer); 3.5 h leaves room for those without hiding a wrong continent.
TZ_SOLAR_TOLERANCE_H = 3.5


def check_timezone_plausible(offset_s: int, longitude: Optional[float],
                             source: str, logger: logging.Logger) -> bool:
    """Warn when *offset_s* cannot be local time at *longitude*.

    A dependency-free sanity check of the camera's timezone against the GPS
    position: it catches a camera still set to another continent, not a
    one-hour daylight-saving slip.  Returns ``True`` when plausible or when
    there is no position to compare with.
    """
    if longitude is None or not -180.0 <= float(longitude) <= 180.0:
        return True
    solar_h = float(longitude) / 15.0
    diff_h = abs(offset_s / 3600.0 - solar_h)
    if diff_h <= TZ_SOLAR_TOLERANCE_H:
        logger.debug("  timezone %s (%s) is plausible for longitude %.3f "
                     "(solar offset %+.1f h)", offset_string(offset_s),
                     source, longitude, solar_h)
        return True
    logger.warning("  timezone %s from %s does not fit the GPS position "
                   "(longitude %.3f, solar offset %+.1f h): the camera clock "
                   "may still be set for another region; pass --timezone to "
                   "override", offset_string(offset_s), source, longitude,
                   solar_h)
    return False


def resolve_timezone(args: argparse.Namespace, detected: Optional[int], *,
                     camera: str, logger: logging.Logger) -> Tuple[int, bool]:
    """Return ``(offset_seconds, known)`` for naming and EXIF.

    Rules: explicit ``--timezone`` wins; ``--time-base utc`` uses UTC; otherwise
    the camera-detected offset is used.  A MAX recording reports the offset via
    its container clock vs GPS (see :func:`timezone_from_clock_skew`), other
    GoPro files may carry none, in which case they fall back to UTC (never the
    host timezone) and say so in the log.
    """
    explicit = parse_timezone(getattr(args, "timezone", None))
    if explicit is not None:
        return explicit, True
    if args.time_base == "utc":
        return 0, True
    if detected is not None:
        return detected, True
    if camera in CAMERA_360 + (CAMERA_HERO,):
        logger.warning("no camera-local timezone in this video; using UTC for "
                       "file names and EXIF (pass --timezone to override)")
    return 0, False


def resolve_gopro_timezone(args: argparse.Namespace, tzon_s: Optional[int],
                           other_s: Optional[int], other_source: str,
                           track, *, camera: str,
                           logger: logging.Logger) -> Tuple[int, bool]:
    """Timezone for a GoPro video, logged with its source and sanity-checked.

    Order: ``--timezone`` / ``--time-base utc`` (see
    :func:`resolve_timezone`), then the camera's own ``TZON`` setting
    (MAX2, HERO11+), then *other_s* (the MAX clock-skew estimate), else UTC
    with a warning.  A camera-derived offset is compared with the GPS
    longitude of the first fix (:func:`check_timezone_plausible`).
    """
    if tzon_s is not None:
        detected, source = tzon_s, "camera TZON setting"
    elif other_s is not None:
        detected, source = other_s, other_source
    else:
        detected, source = None, ""
    offset, known = resolve_timezone(args, detected, camera=camera,
                                     logger=logger)
    explicit = parse_timezone(getattr(args, "timezone", None)) is not None
    if explicit:
        source = "--timezone"
    elif args.time_base == "utc":
        source = "--time-base utc"
    if known:
        logger.info("  camera-local time offset: %s (%s)",
                    offset_string(offset), source)
        if not explicit and args.time_base != "utc" and detected is not None:
            samples = getattr(track, "samples", None) or []
            longitude = samples[0].get("lon") if samples else None
            check_timezone_plausible(offset, longitude, source, logger)
    return offset, known


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


# Course over ground from GPS positions (see Track._add_gps_courses).
COURSE_BASELINE_M = 4.0     # positions this far apart give a stable bearing
COURSE_MAX_SPAN_S = 10.0    # never look further than this (real seconds)
COURSE_MIN_SPEED = 0.5      # m/s; slower samples inherit a neighbour's course


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
        makes a two-point bearing pure noise: GPS jitter of a metre turns a
        0.15 m baseline into a random direction.  The window around each
        sample therefore grows until the two ends are COURSE_BASELINE_M apart
        (a few GPS error radii), but never beyond COURSE_MAX_SPAN_S of real
        time, so a slow crawl or a stop does not borrow a course from far
        away.  At driving speed the baseline is reached within a fraction of
        a second, so turns stay sharp.
        """
        samples = self.samples
        n = len(samples)

        def span_s(a, b):
            ua, ub = a.get("utc"), b.get("utc")
            if ua is not None and ub is not None:
                return abs((ub - ua).total_seconds())
            return abs(b.get("t", 0.0) - a.get("t", 0.0))

        for i, sample in enumerate(samples):
            if sample.get("gps_course") is not None:
                continue
            # A parked vehicle has no meaningful course over ground.
            speed = sample.get("speed")
            if speed is not None and speed < COURSE_MIN_SPEED:
                continue
            lo = hi = i
            while lo > 0 or hi < n - 1:
                new_lo, new_hi = max(0, lo - 1), min(n - 1, hi + 1)
                if span_s(samples[new_lo], samples[new_hi]) > COURSE_MAX_SPAN_S:
                    break
                lo, hi = new_lo, new_hi
                a, b = samples[lo], samples[hi]
                if (haversine(a["lat"], a["lon"], b["lat"], b["lon"])
                        >= COURSE_BASELINE_M):
                    sample["gps_course"] = initial_bearing_deg(
                        a["lat"], a["lon"], b["lat"], b["lon"])
                    break
        # A mounted camera keeps facing the same way while the vehicle is
        # parked (typically at the start of a recording), so give samples
        # without a course the nearest known one instead of leaving the
        # image direction undefined.
        known = [i for i, s in enumerate(self.samples)
                 if s.get("gps_course") is not None]
        if known and len(known) < n:
            for i, sample in enumerate(self.samples):
                if sample.get("gps_course") is not None:
                    continue
                pos = bisect.bisect_left(known, i)
                candidates = [known[j] for j in (pos - 1, pos)
                              if 0 <= j < len(known)]
                nearest = min(candidates, key=lambda j: abs(j - i))
                sample["gps_course"] = self.samples[nearest]["gps_course"]
                sample["gps_course_filled"] = True

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
                # Interpolate the wall clock too: with sparse GPS (TimeWarp
                # keeps ~2 Hz of real time) reusing the earlier sample's UTC
                # would mis-time the frame by up to one sample, which is a
                # whole exported frame in timelapse footage.
                ua, ub = a.get(key), b.get(key)
                if (isinstance(ua, dt.datetime) and isinstance(ub, dt.datetime)
                        and (ua.tzinfo is None) == (ub.tzinfo is None)):
                    out[key] = ua + (ub - ua) * fraction
                else:
                    out[key] = ua if ua is not None else ub
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
        if source in ("clamped-start", "clamped-end"):
            anchor = self.samples[0 if source == "clamped-start" else -1]
            point = dict(anchor)
            point["t"] = t
            # The position is held at the first/last fix, but the capture
            # time is the frame's own: shift the fix's clock by the frame's
            # distance from it (in real seconds, so TimeWarp is respected).
            utc = point.get("utc")
            if isinstance(utc, dt.datetime):
                point["utc"] = utc + dt.timedelta(
                    seconds=(t - anchor["t"]) * self.real_seconds_per_t())
        else:
            point = self._interp(self.samples[lo], self.samples[lo + 1],
                                 fraction)
        point["distance"] = self.distance_at_time(t)
        point["position_source"] = source
        point["clamped"] = source.startswith("clamped")
        point["estimated"] = True
        return point

    def real_seconds_per_t(self) -> float:
        """Real (UTC) seconds per unit of track time ``t``.

        1.0 for normal video; the time-lapse/TimeWarp speed-up when ``t`` is
        playback time.  Falls back to 1.0 without usable UTC stamps.
        """
        if len(self.samples) < 2:
            return 1.0
        first, last = self.samples[0], self.samples[-1]
        span_t = last.get("t", 0.0) - first.get("t", 0.0)
        ua, ub = first.get("utc"), last.get("utc")
        if (span_t > 0 and isinstance(ua, dt.datetime)
                and isinstance(ub, dt.datetime)):
            rate = (ub - ua).total_seconds() / span_t
            if rate > 0:
                return rate
        return 1.0

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

    def position_at_time(self, t):
        """``{lat, lon, alt}`` at *t*, exactly as :meth:`at_time` gives them.

        A cheap variant for callers that only need the position (frame
        matching, placing IMU samples): no other key is interpolated.
        """
        samples = self.samples
        if not samples:
            return None
        if len(samples) == 1:
            a = samples[0]
            return {"lat": a["lat"], "lon": a["lon"], "alt": a.get("alt")}
        lo, fraction, source = self._locate_time(t)
        if source != "interpolated":
            a = samples[0] if source == "clamped-start" else samples[-1]
            return {"lat": a["lat"], "lon": a["lon"], "alt": a.get("alt")}
        a, b = samples[lo], samples[lo + 1]
        return {"lat": a["lat"] + fraction * (b["lat"] - a["lat"]),
                "lon": a["lon"] + fraction * (b["lon"] - a["lon"]),
                "alt": _interp_value(a, b, fraction, "alt")}

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


HEADING_OFFSET_AUTO = "auto"


def parse_heading_offset_arg(value: Optional[str]
                             ) -> Tuple[bool, Optional[float]]:
    """Parse ``--heading-offset``: ``(is_auto, degrees or None)``.

    Only ``auto`` or a number from 0 to 360 is accepted; anything else
    (negative, above 360, not a number) raises ValueError.
    """
    if value is None:
        return False, None
    text = str(value).strip()
    if text.lower() == HEADING_OFFSET_AUTO:
        return True, None
    try:
        degrees = float(text)
    except ValueError:
        raise ValueError("--heading-offset must be a number from 0 to 360 "
                         "or 'auto' (got %r)" % value) from None
    if not math.isfinite(degrees) or not 0.0 <= degrees <= 360.0:
        raise ValueError("--heading-offset must be from 0 to 360 degrees "
                         "(got %s)" % text)
    return False, degrees


def folder_heading_offset(name: str) -> Optional[float]:
    """Heading offset encoded in a folder name, or ``None`` when there is none.

    The name is split on ``-`` and ``_``; the offset is a part that consists
    of exactly three digits (``000``-``360``) and is not the first part, so it
    has a separator before it and a separator or the end of the name after
    it.  ``2026-11-05-max-090-30fps`` -> 90.  Camera names and formats
    such as ``max360``, ``30fps``, ``120fps``, ``1080p``, ``4k`` or
    ``timelapse05`` are never separate 3-digit parts, and dates use 4- and
    2-digit parts.  Raises ValueError for a 3-digit part above 360 or for two
    different offsets in one name.
    """
    parts = re.split(r"[-_]", name)[1:]
    values = []
    for part in parts:
        if re.fullmatch(r"[0-9]{3}", part):
            degrees = float(part)
            if degrees > 360.0:
                raise ValueError("folder %r: %s is not a heading offset "
                                 "(000-360)" % (name, part))
            if degrees not in values:
                values.append(degrees)
    if len(values) > 1:
        raise ValueError("folder %r has more than one heading offset (%s)"
                         % (name, ", ".join("%03d" % v for v in values)))
    return values[0] if values else None


def video_heading_offset(args: argparse.Namespace, video: str
                         ) -> Optional[float]:
    """The heading offset that applies to *video* (``None`` = no offset)."""
    if getattr(args, "heading_offset_auto", False):
        folder = os.path.basename(os.path.dirname(os.path.abspath(video)))
        return folder_heading_offset(folder)
    return getattr(args, "heading_offset_value", None)


def video_args(args: argparse.Namespace, video: str,
               camera: str) -> argparse.Namespace:
    """Per-video copy of *args* with the heading offset resolved.

    For 360 (MAX) videos with re-centring on (the default), the offset is
    applied to the pixels instead of the heading: ``recenter_deg`` is the
    offset, ``heading_offset`` is 0 and the output folder gets no -NNN label.
    Otherwise (flat videos, or --no-recenter-forward) the offset is added to
    the heading and labels the output folder as before.
    """
    vargs = copy.copy(args)
    offset = video_heading_offset(args, video)
    recenter = camera in CAMERA_360 and getattr(args, "recenter_forward", True)
    vargs.recenter_deg = (offset % 360.0) if (recenter and offset) else 0.0
    vargs.heading_offset = 0.0 if (recenter or offset is None) else offset
    vargs.heading_offset_given = offset is not None and not recenter
    vargs.heading_offset_note = None
    if offset is not None:
        vargs.heading_offset_note = "heading offset %03d%s%s" % (
            int(round(offset)),
            " (from folder name)" if args.heading_offset_auto else "",
            ": sphere rotated so the direction of travel is the image centre"
            if vargs.recenter_deg else
            ": added to the heading" if offset else "")
    return vargs


def log_heading_offset(args: argparse.Namespace) -> None:
    note = getattr(args, "heading_offset_note", None)
    if note:
        LOG.info("  %s", note)


def recenter_yaw(recenter_deg: float) -> float:
    """v360 ``yaw`` that brings the direction of travel to the image centre.

    v360 shows the input azimuth ``yaw`` at the output centre (verified with
    marker images: ``yaw=90`` centres what was 90 degrees to the right).  The
    front lens points *recenter_deg* clockwise of travel, so travel sits at
    ``-recenter_deg`` in the stitched image.
    """
    yaw = (-float(recenter_deg) + 180.0) % 360.0 - 180.0
    return 0.0 if abs(yaw) < 1e-9 else yaw


def resolve_heading(point: dict, heading_offset: float = 0.0,
                    magnetic_declination: float = 0.0,
                    recenter_deg: float = 0.0) -> Tuple[Optional[float], str]:
    """Return ``(true-north heading, heading_source)`` for an interpolated point.

    Priority: an explicit heading from the source metadata, then the GPS
    course over ground, and only then the IMU compass.  The GoPro magnetometer
    is uncalibrated (hard-iron offsets of ~100 uT, twice Earth's field, from
    the camera body and the vehicle), and on real MAX recordings its heading
    disagreed with the GPS course by 40-250 degrees, so it must never
    override a course; it only fills in when GPS gives no course at all.
    """
    declination = magnetic_declination or 0.0
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
    # The compass measures where the front lens points, i.e. the image centre
    # as stitched; a heading offset (travel -> lens) does not apply to it, but
    # re-centring turns the image centre back by the offset.
    compass = point.get("compass_heading")
    if compass is not None:
        return (compass + declination - recenter_deg) % 360.0, "compass"
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


# GPS readings rejected before a track is built (see reject_bad_gps).
GPS_MAX_DOP = 10.0          # default --max-dop; GoPro writes 99.99 = no fix
GPS_MAX_SPEED = 70.0        # m/s (252 km/h); faster jumps are receiver noise
GPS_JUMP_RESET_S = 5.0      # after this long rejecting, trust the new reading


def reject_bad_gps(samples: Sequence[dict], max_dop: float = GPS_MAX_DOP,
                   max_speed: float = GPS_MAX_SPEED
                   ) -> Tuple[List[dict], int, int]:
    """Drop unusable GPS readings; return ``(kept, n_dop, n_jump)``.

    *samples* must be in time order and carry ``utc``, ``lat`` and ``lon``.
    Two checks, both needed on real GoPro footage:

    * DOP: while acquiring, the receiver already reports a 2D fix but with
      DOP 99.99 and positions that wander by hundreds of metres per second.
      ``max_dop <= 0`` disables this check.
    * Speed: a reading that implies more than *max_speed* from the previous
      kept reading is a jump (typically where the DOP first improves).  If
      readings keep being rejected for GPS_JUMP_RESET_S, the earlier kept
      reading was the bad one, so the current reading is accepted instead of
      throwing the rest of the track away.
    """
    kept: List[dict] = []
    n_dop = n_jump = 0
    rejecting_since = None
    for sample in samples:
        dop = sample.get("hdop")
        if dop is None:
            dop = sample.get("dop")
        if max_dop > 0 and dop is not None and dop > max_dop:
            n_dop += 1
            continue
        if kept and max_speed > 0:
            prev = kept[-1]
            span = (sample["utc"] - prev["utc"]).total_seconds()
            moved = haversine(prev["lat"], prev["lon"],
                              sample["lat"], sample["lon"])
            if moved > max_speed * max(span, 1e-3):
                if rejecting_since is None:
                    rejecting_since = sample["utc"]
                if ((sample["utc"] - rejecting_since).total_seconds()
                        < GPS_JUMP_RESET_S):
                    n_jump += 1
                    continue
        rejecting_since = None
        kept.append(sample)
    return kept, n_dop, n_jump


def log_rejected_gps(n_dop: int, n_jump: int, max_dop: float) -> None:
    if n_dop or n_jump:
        parts = []
        if n_dop:
            parts.append("%d with DOP above %g" % (n_dop, max_dop))
        if n_jump:
            parts.append("%d implausible jump(s) faster than %.0f km/h"
                         % (n_jump, GPS_MAX_SPEED * 3.6))
        LOG.info("  rejected GPS reading(s): %s", ", ".join(parts))


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


# Longest real-time gap between recording start and the first usable GPS fix
# for which frame 0 is still exported (at the first fix's position).
FIRST_FIX_MAX_DELAY_S = 2.0


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

    With ``start_distance == 0`` the first frame (index 0) is exported even if
    telemetry starts slightly later (position clamped to the first fix,
    capture time still the frame's own), but only when that delay is at most
    FIRST_FIX_MAX_DELAY_S of real time; otherwise the sequence starts at the
    first fix.
    """
    log = logger or LOG
    if max_position_error is None:
        max_position_error = interval / 2.0
    shots = plan_shots(track, interval, start_distance=start_distance,
                       max_shots=max_images)
    # Frame 0 is forced only when GPS was already fixed at (or within
    # FIRST_FIX_MAX_DELAY_S of) the recording start.  When the camera started
    # recording before its GPS locked, frame 0 was taken somewhere the track
    # never saw, so pinning it to the first fix would misplace it.
    if require_first_frame and start_distance == 0.0 and len(timeline) > 0:
        delay = ((track.times[0] - timeline.pts(0))
                 * track.real_seconds_per_t()) if track.times else 0.0
        if delay > FIRST_FIX_MAX_DELAY_S:
            log.warning("  GPS fix starts %.0f s after the recording start; "
                        "the footage before it has no position and is not "
                        "exported (wait for GPS lock before recording)",
                        delay)
            require_first_frame = False
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
            # Only the position is needed to accept or reject a candidate;
            # the full interpolation is done once for the chosen frame.
            position = track.position_at_time(timeline.pts(index))
            if position is None or position.get("lat") is None:
                continue
            error = _position_error(shot, position, mode)
            if error is not None and error <= max_position_error + 1e-6:
                found = (index, track.at_time(timeline.pts(index)), error)
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


def unpositioned_choices(timeline: FrameTimeline, interval: float,
                         duration: Optional[float],
                         max_images: Optional[int]) -> List[FrameChoice]:
    """Time-grid frames without a position (the ``--fallback-interval`` path)."""
    frames = select_time_grid_frames(
        timeline, interval, duration=duration, max_images=max_images)
    out = []
    for index in frames:
        pts = timeline.pts(index)
        out.append(FrameChoice(
            target_distance=pts, target_time=pts, frame_index=index, pts=pts,
            point={}, position_error=None, clamped=False, estimated=True))
    return out


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

# Fallback path only: when a single-pass leveled run fails, its frames are
# stitched to lossless PNG intermediates and rotated by a second v360 pass.
# Smaller batches bound the temporary disk use (a 5376x2688 RGB PNG is
# roughly 40 MB).
MAX360_LEVEL_CHUNK_SIZE = 8
# Leveled frames whose roll and pitch are all within this many degrees of a
# common value share one single-pass rotation (see max360_level_groups).
# 0.2 degrees is ~3 px at the edge of a 5376 px panorama; on four test drives
# it gave runs of 4-5 images and made leveled exports ~2.5x faster than one
# extra rotation pass per frame.
LEVEL_GROUP_TOLERANCE_DEG = 0.2

# At most this many MAX stitch batches run at the same time (bounded by
# --workers).  Every batch decodes two 4K hemisphere streams through a large
# filter graph, and one batch already keeps all cores of a 4-core machine
# busy (measured: 2 at once gave no speedup), so batches run one by one.
# Raise this only on machines with many cores and plenty of memory.
MAX360_PARALLEL_BATCHES = 1


def make_temp_dir(prefix: str, outputs: Sequence[str] = ()) -> str:
    """Temporary directory, next to the outputs when possible.

    Keeping temporary frames on the same filesystem as their destination
    lets :func:`move_file` rename them into place instead of copying.
    """
    for output in outputs:
        parent = os.path.dirname(os.path.abspath(output))
        try:
            os.makedirs(parent, exist_ok=True)
            return tempfile.mkdtemp(prefix=".v2i-" + prefix, dir=parent)
        except OSError:
            break
    return tempfile.mkdtemp(prefix=prefix)


def move_file(src: str, dest: str) -> None:
    """Move *src* to *dest*: a rename when possible, else a copy."""
    try:
        os.replace(src, dest)
    except OSError:
        shutil.copyfile(src, dest)


def place_extracted(extracted: Mapping[int, str], records: Sequence[dict]
                    ) -> Tuple[List[dict], List[int]]:
    """Move extracted temporary frames to their record paths.

    Returns ``(written_records, failed_frame_indices)`` and marks every
    written record with ``written = True``.
    """
    written: List[dict] = []
    failed: List[int] = []
    placed: Dict[int, str] = {}
    for record in records:
        frame = record["frame_index"]
        temp = extracted.get(frame)
        if not temp:
            failed.append(frame)
            continue
        os.makedirs(os.path.dirname(record["path"]), exist_ok=True)
        if frame in placed:
            # Two records on one frame: the temp file has already moved.
            shutil.copyfile(placed[frame], record["path"])
        else:
            move_file(temp, record["path"])
            placed[frame] = record["path"]
        record["written"] = True
        written.append(record)
    return written, failed


def max360_rotation_opts(yaw: float = 0.0, roll_deg: float = 0.0,
                         pitch_deg: float = 0.0) -> str:
    """``v360`` rotation options for re-centring and leveling, or ``""``.

    Same convention as :func:`max360_level_filter` (``roll=-roll_deg``,
    ``rorder=pry``: leveling in the camera frame, then the yaw).  Placed in
    the stitching ``v360=eac:equirect`` it gives the same image as stitching
    first and rotating afterwards (36 dB PSNR on a real frame, the normal
    difference between one and two resamplings) at no extra cost.
    """
    if max(abs(float(yaw)), abs(float(roll_deg)), abs(float(pitch_deg))) < 0.05:
        return ""
    return ":yaw=%.6f:pitch=%.6f:roll=%.6f:rorder=pry" % (
        float(yaw), float(pitch_deg), -float(roll_deg))


def max360_level_groups(frames: Sequence[int],
                        levels: Mapping[int, Tuple[float, float]],
                        tolerance: float, max_size: int
                        ) -> List[Tuple[List[int], Tuple[float, float]]]:
    """Split frames into runs that can share one leveling rotation.

    Consecutive frames stay in a run while every frame's roll and pitch are
    within *tolerance* degrees of the run's midpoint.  Frames without an
    angle count as (0, 0), i.e. only re-centred.  Returns
    ``[(frames, (roll, pitch)), ...]`` with the midpoint per run.
    """
    runs: List[Tuple[List[int], Tuple[float, float]]] = []
    current: List[int] = []
    lo = hi = (0.0, 0.0)
    for frame in frames:
        angle = levels.get(frame) or (0.0, 0.0)
        if current:
            new_lo = (min(lo[0], angle[0]), min(lo[1], angle[1]))
            new_hi = (max(hi[0], angle[0]), max(hi[1], angle[1]))
            if (len(current) < max_size
                    and new_hi[0] - new_lo[0] <= 2 * tolerance
                    and new_hi[1] - new_lo[1] <= 2 * tolerance):
                current.append(frame)
                lo, hi = new_lo, new_hi
                continue
            runs.append((current, ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2)))
        current, lo, hi = [frame], angle, angle
    if current:
        runs.append((current, ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2)))
    return runs


def max360_level_filter(roll_deg: float, pitch_deg: float, width: int,
                        height: int, interp: str = "cubic",
                        yaw: float = 0.0) -> str:
    """Return the v360 filter that levels an equirectangular MAX frame.

    ``roll_deg``/``pitch_deg`` use the camera convention: positive roll is
    clockwise (right side down) and positive pitch means the camera was
    tilted down.  v360's signs were established empirically on deliberately
    tilted GoPro Max footage: ``roll=+R`` turns the picture clockwise, so a
    clockwise camera roll is undone with ``roll=-roll_deg``; ``pitch=+P``
    makes the view look *up* by P (content moves down), which is exactly what
    levels a camera tilted down, so pitch is passed through unchanged.

    *yaw* re-centres the sphere (see :func:`recenter_yaw`).  Rotation order
    ``pry`` applies the leveling in the camera's own frame first and the yaw
    last: with marker images it matched "level, then a separate yaw pass" to
    57 dB PSNR, while the default ``ypr`` (yaw first) tilts about the wrong
    axes.  With ``yaw = 0`` it is the same rotation as before.
    """
    return ("v360=e:e:yaw=%.6f:pitch=%.6f:roll=%.6f:rorder=pry:w=%d:h=%d:"
            "interp=%s" % (float(yaw), float(pitch_deg), -float(roll_deg),
                           int(width), int(height), interp))


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


def _extract_run(video: str, run: Sequence[int], seek_time: float,
                 tmp_dir: str, run_index: int, *, image_format: str,
                 quality: int, threads: int, stream_ordinal: int
                 ) -> Dict[int, str]:
    """Extract several nearby frames with one seek and one sequential decode.

    After the accurate input seek the first decoded frame is ``run[0]``, so
    the ``select`` expression uses frame numbers relative to it.  Returns an
    empty dict when ffmpeg fails or does not produce every frame, so the
    caller can fall back to one seek per frame.
    """
    first = run[0]
    prefix = "r_%05d_" % run_index
    pattern = os.path.join(tmp_dir, prefix + "%06d." + image_format)
    cmd = [FFMPEG, "-hide_banner", "-loglevel", "error", "-y",
           "-ss", "%.6f" % max(0.0, seek_time)]
    if threads:
        cmd += ["-threads", str(threads)]
    cmd += ["-i", video,
            "-map", "0:v:%d" % stream_ordinal,
            "-vf", "select='%s'" % select_expression(
                [frame - first for frame in run]),
            "-vsync", "0",
            "-frames:v", str(len(run))]
    cmd += ffmpeg_quality_args(image_format, quality)
    cmd.append(pattern)
    try:
        proc = subprocess.run(cmd, stdout=subprocess.DEVNULL,
                              stderr=subprocess.PIPE, timeout=600)
    except (OSError, subprocess.TimeoutExpired) as exc:
        LOG.debug("frames %d-%d batch extraction failed: %s",
                  run[0], run[-1], exc)
        return {}
    outputs = [pattern % (i + 1) for i in range(len(run))]
    if proc.returncode != 0 or not all(os.path.exists(p) for p in outputs):
        err = proc.stderr.decode("utf-8", "replace").strip().splitlines()
        LOG.debug("frames %d-%d batch extraction failed: %s", run[0], run[-1],
                  err[-1] if err else "missing output")
        return {}
    return dict(zip(run, outputs))


# Selected frames closer together than this much video time are decoded in
# one sequential ffmpeg pass: decoding the frames in between is cheaper than
# starting a new ffmpeg process and re-decoding from the previous keyframe.
EXTRACT_RUN_GAP_S = 1.0


def _plan_extract_runs(frames: Sequence[int], fps: float,
                       workers: int) -> List[List[int]]:
    """Split sorted frame indices into runs worth one sequential decode."""
    gap = max(1, int(round(EXTRACT_RUN_GAP_S * (fps if fps > 0 else 30.0))))
    max_len = EXTRACT_CHUNK_SIZE
    if workers > 1:
        # Keep enough runs to occupy every worker.
        max_len = max(1, min(max_len, -(-len(frames) // workers)))
    runs: List[List[int]] = []
    for frame in frames:
        if (runs and frame - runs[-1][-1] <= gap
                and len(runs[-1]) < max_len):
            runs[-1].append(frame)
        else:
            runs.append([frame])
    return runs


def extract_frames_by_index(video: str, frames: Sequence[int], tmp_dir: str, *,
                            image_format: str, quality: int, workers: int = 1,
                            threads: int = 0, stream_ordinal: int = 0,
                            timeline: Optional[FrameTimeline] = None
                            ) -> Dict[int, str]:
    """Extract exact frames with accurate seeks.

    Seeking to the frame's real PTS (``timeline.pts``) keeps variable frame
    rate sources correct; a uniform ``index / fps`` grid is only used when no
    usable timeline is available.  Frames that lie close together are
    decoded as one run (one seek, one sequential decode, ``select`` by frame
    number); isolated frames get their own seek.  A run that fails is retried
    frame by frame.  ``workers`` independent ffmpeg processes are run at a
    time.  Flat frames are written exactly as decoded; horizon leveling only
    exists for 360 imagery.
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

    def single(frame_index: int, tag: str) -> Tuple[int, Optional[str]]:
        pattern = os.path.join(tmp_dir,
                               "f_%s_%%06d.%s" % (tag, image_format))
        path = _extract_one(video, frame_index, seek_for(frame_index), pattern,
                            image_format=image_format, quality=quality,
                            threads=threads, stream_ordinal=stream_ordinal)
        return frame_index, path

    def run(item: Tuple[int, List[int]]) -> List[Tuple[int, Optional[str]]]:
        order, frames_in_run = item
        if len(frames_in_run) == 1:
            return [single(frames_in_run[0], "%06d" % order)]
        got = _extract_run(video, frames_in_run, seek_for(frames_in_run[0]),
                           tmp_dir, order, image_format=image_format,
                           quality=quality, threads=threads,
                           stream_ordinal=stream_ordinal)
        if got:
            return [(frame, got[frame]) for frame in frames_in_run]
        LOG.debug("retrying frames %d-%d with one seek per frame",
                  frames_in_run[0], frames_in_run[-1])
        return [single(frame, "%06d_%03d" % (order, i))
                for i, frame in enumerate(frames_in_run)]

    items = list(enumerate(_plan_extract_runs(unique, fps, workers)))
    extracted: Dict[int, str] = {}
    if workers > 1 and len(items) > 1:
        with ThreadPoolExecutor(max_workers=min(workers, len(items))) as pool:
            results = [pair for part in pool.map(run, items) for pair in part]
    else:
        results = [pair for item in items for pair in run(item)]

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
def _exiftool_batches(pairs: Sequence[Tuple[str, Sequence[str]]], batch: int,
                      max_chars: int) -> Iterator[List[str]]:
    """Group per-file argument lists into ``-execute``-separated commands."""
    current: List[str] = []
    files = 0
    size = 0
    for path, tag_args in pairs:
        group = list(tag_args) + [path]
        group_size = sum(len(arg) + 1 for arg in group) + len("-execute") + 1
        if current and (files >= batch or size + group_size > max_chars):
            yield current
            current, files, size = [], 0, 0
        if current:
            current.append("-execute")
        current += group
        files += 1
        size += group_size
    if current:
        yield current


def write_metadata(pairs: Sequence[Tuple[str, Sequence[str]]], *,
                   batch: int = 250, max_chars: int = 100_000) -> None:
    """Write metadata for ``[(path, exiftool_args), ...]``.

    One exiftool process handles up to *batch* files (bounded by the total
    command-line length).  ``-common_args`` makes the quiet, preserve-date
    and overwrite options apply to every ``-execute`` group, not only the
    first one.
    """
    if not pairs:
        return
    for group_args in _exiftool_batches(pairs, batch, max_chars):
        cmd = ([EXIFTOOL] + group_args
               + ["-common_args", "-q", "-q", "-P", "-overwrite_original"])
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
        path = os.path.join(directory, sequence + ".telemetry.geojson")
        write_points_telemetry(
            path, camera=camera, video=video, sequence=sequence,
            gps=samples, imu=imu, video_start_utc=video_start_utc,
            calibration=calibration, units=units, time_scale=time_scale,
            extra=extra)
        written.append(path)
    if telemetry_export in ("both", "images"):
        path = os.path.join(directory, sequence + ".image-telemetry.geojson")
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
# Detailed telemetry GeoJSON writers
# ---------------------------------------------------------------------------
# The plain ``<sequence>.geojson`` track only keeps positions.  These two
# writers export the raw telemetry as RFC 7946 FeatureCollections: one file
# with every sampled point (GPS and IMU) and one with the GPS/IMU snapshot
# nearest to each exported image.  Every sample becomes a ``Feature`` whose
# ``properties`` carry the measurement; IMU samples are placed on the GPS
# track at their timestamp and samples without a usable position are
# written as unlocated features (``"geometry": null``), which the RFC allows.
_IMU_COMPONENTS = {
    "ACCL": ["x", "y", "z"],
    "GYRO": ["x", "y", "z"],
    "MAGN": ["x", "y", "z"],
    "GRAV": ["x", "y", "z"],
    "CORI": ["w", "x", "y", "z"],
    "ORIN": ["x", "y", "z"],
    "IORI": ["w", "x", "y", "z"],
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
    roll = point.get("roll_deg")
    pitch = point.get("pitch_deg")
    if roll is None and key == "GRAV":
        roll, pitch = _gravity_roll_pitch(values)
    if roll is not None and pitch is not None:
        out["roll_deg"] = round(float(roll), 3)
        out["pitch_deg"] = round(float(pitch), 3)
        out["angle_source"] = key
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


def _feature(geometry: Optional[dict], properties: dict) -> dict:
    return {"type": "Feature", "geometry": geometry, "properties": properties}


def _point_geometry(sample: dict) -> Optional[dict]:
    coord = _coordinate(sample)
    return None if coord is None else {"type": "Point", "coordinates": coord}


def _feature_collection(features: List[dict], *, camera: str, video: str,
                        sequence: str, video_start_utc, head: dict,
                        extra: Optional[dict] = None) -> dict:
    """Assemble a FeatureCollection with the run metadata as foreign members."""
    payload = {
        "type": "FeatureCollection",
        "generator": SCRIPT_NAME,
        "generator_version": __version__,
        "camera": camera,
        "camera_label": CAMERA_LABELS.get(camera, camera),
        "video": video,
        "sequence": sequence,
        "video_start_utc": fmt_utc(_as_utc(video_start_utc)),
    }
    payload.update(head)
    if extra:
        payload.update(extra)
    payload["feature_count"] = len(features)
    payload["features"] = features
    return payload


def _track_locator(points: Sequence[dict]) -> Optional[Track]:
    """A :class:`Track` keyed on ``t_s`` so IMU rows can be placed on it."""
    timed = []
    for point in points:
        if (point.get("t_s") is not None and point.get("lat") is not None
                and point.get("lon") is not None):
            timed.append({"t": float(point["t_s"]), "lat": point["lat"],
                          "lon": point["lon"], "alt": point.get("alt")})
    if len(timed) < 2:
        return None
    return Track(timed)


def _locate_on_track(locator: Optional[Track], t_s) -> Optional[dict]:
    """Point geometry for ``t_s`` while it lies inside the GPS track."""
    if locator is None or t_s is None:
        return None
    if not (locator.times[0] <= float(t_s) <= locator.times[-1]):
        return None
    point = locator.position_at_time(float(t_s))
    if point is None:
        return None
    return _point_geometry(point)


def write_points_telemetry(path: str, *, camera: str, video: str,
                           sequence: str, gps: Sequence[dict],
                           imu: Optional[Dict[str, List[dict]]] = None,
                           video_start_utc=None,
                           calibration: Optional[Dict[str, dict]] = None,
                           units: Optional[Dict[str, str]] = None,
                           time_scale: float = 1.0,
                           extra: Optional[dict] = None) -> None:
    """Write every telemetry point (GPS and IMU) as a GeoJSON FeatureCollection.

    The GPS stream becomes one ``LineString`` track feature followed by one
    ``Point`` feature per sample.  Each IMU stream contributes one feature
    per sample, positioned by interpolating the GPS track at the sample time
    (``position_source = "interpolated"``) or unlocated when the sample lies
    outside the track or no track exists.  The ``streams`` member summarises
    units, component names and counts per stream.
    """
    features: List[dict] = []
    streams: Dict[str, dict] = {}

    gps_points = [_clean_telemetry_point(point, video_start_utc, time_scale)
                  for point in gps]
    if gps_points:
        coords = [c for c in (_coordinate(p) for p in gps_points)
                  if c is not None]
        streams["gps"] = {"point_count": len(gps_points),
                          "track_point_count": len(coords)}
        if len(coords) >= 2:
            features.append(_feature(
                {"type": "LineString", "coordinates": coords},
                {"stream": "gps", "kind": "track", "name": sequence,
                 "point_count": len(coords)}))
        for point in gps_points:
            props = {"stream": "gps", "kind": "sample"}
            props.update(point)
            features.append(_feature(_point_geometry(point), props))
    locator = _track_locator(gps_points)

    for key in sorted(imu or {}):
        rows = imu[key]
        if not rows:
            continue
        entry = _imu_stream_entry(key, rows, video_start_utc, units,
                                  time_scale)
        samples = entry.pop("points")
        streams[key] = entry
        located = 0
        for sample in samples:
            props = {"stream": key, "kind": "sample"}
            if entry.get("unit") is not None:
                props["unit"] = entry["unit"]
            props.update(sample)
            geometry = _locate_on_track(locator, sample.get("t_s"))
            if geometry is not None:
                props["position_source"] = "interpolated"
                located += 1
            features.append(_feature(geometry, props))
        entry["located_point_count"] = located

    head: dict = {"streams": streams}
    if calibration:
        head["calibration"] = calibration
    payload = _feature_collection(
        features, camera=camera, video=video, sequence=sequence,
        video_start_utc=video_start_utc, head=head, extra=extra)
    with open(path, "w", encoding="utf-8") as fh:
        # Compact: these files hold one feature per raw GPS/IMU sample.
        json.dump(payload, fh, default=_json_default, sort_keys=False,
                  separators=(",", ":"))
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
    """Write one GeoJSON ``Point`` feature per exported image.

    Each feature's ``properties`` hold the image record plus the GPS sample
    and the IMU snapshot nearest to its capture time.  Unpositioned images
    (fallback time grid) are written with ``"geometry": null``.
    """
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
        gravity = snapshot.get("GRAV") or snapshot.get("ACCL")
        if gravity and gravity.get("roll_deg") is not None:
            entry["roll_deg"] = gravity["roll_deg"]
            entry["pitch_deg"] = gravity["pitch_deg"]
            entry["angle_source"] = gravity.get("angle_source")
        entries.append(_feature(_point_geometry(entry), entry))

    payload = _feature_collection(
        entries, camera=camera, video=video, sequence=sequence,
        video_start_utc=video_start_utc, head={"image_count": len(entries)},
        extra=extra)
    with open(path, "w", encoding="utf-8") as fh:
        # Compact: these files hold one feature per raw GPS/IMU sample.
        json.dump(payload, fh, default=_json_default, sort_keys=False,
                  separators=(",", ":"))
        fh.write("\n")


# =========================================================================
# GoPro GPMF telemetry reader (MAX, MAX2 and HERO)
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


@lru_cache(maxsize=1024)
def _big_endian_struct(fmt: str, count: int) -> struct.Struct:
    return struct.Struct(">%d%s" % (count, fmt))


def decode(buf: bytes, elem: Element) -> Union[List, bytes]:
    """Decode an element payload (numbers as a list, strings as bytes)."""
    if elem.typ in _BYTES:
        return b"" if elem.typ == TYPE_COMPRESSED else buf[elem.offset:elem.end]
    fmt, esz = _NUMERIC[elem.typ]
    n = min(elem.length, max(0, len(buf) - elem.offset)) // esz
    return list(_big_endian_struct(fmt, n).unpack_from(buf, elem.offset))


def decode_complex(buf: bytes, elem: Element, type_string: str
                   ) -> Optional[List]:
    """Decode a ``?`` (complex) element with the stream's ``TYPE`` string.

    Newer GoPros (MAX2, HERO11+) store ``GPS9`` as one structure per sample,
    e.g. ``TYPE`` ``lllllllSS`` = seven int32 plus two uint16.  Returns the
    values flattened sample after sample, or ``None`` when the TYPE contains
    a non-numeric member or does not match the structure size.
    """
    fmt = ""
    for char in type_string:
        entry = _NUMERIC.get(ord(char))
        if entry is None:
            return None
        fmt += entry[0]
    sample = struct.Struct(">" + fmt)
    if not fmt or sample.size != elem.struct_size:
        return None
    values: List = []
    end = min(elem.end, len(buf))
    for pos in range(elem.offset, end - sample.size + 1, sample.size):
        values.extend(sample.unpack_from(buf, pos))
    return values


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
        # Callers pass the SCAL-scaled value, i.e. seconds since midnight.
        # Guessing "milliseconds" from the magnitude would misread the first
        # 86.4 s after midnight UTC, so only clearly impossible values are
        # treated as unscaled milliseconds.
        secs = float(seconds)
        if secs >= 86400.0 + 1.0:
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
    type_string: Optional[str] = None                # TYPE for ``?`` data
    keys: Dict[str, list] = field(default_factory=dict)

    def add(self, key: str, values: list) -> None:
        self.keys.setdefault(key, []).extend(values)


def _collect_blocks(buf: bytes, device_names: Optional[List[str]] = None
                    ) -> Dict[str, List[_Block]]:
    """Group every ``STRM`` block by its ``STNM`` stream name.

    When *device_names* is given, the first non-empty ``DVNM`` (the value
    :func:`read_device_name` returns) is appended to it, so callers do not
    need a second pass over the buffer.
    """
    blocks: Dict[str, List[_Block]] = {}
    current: Optional[_Block] = None
    device: Optional[str] = None

    def flush() -> None:
        nonlocal current
        if current is not None:
            blocks.setdefault(current.stream or "unknown", []).append(current)
        current = None

    for elem in iter_elements(buf):
        if (device_names is not None and not device_names
                and elem.key == "DVNM"):
            text = as_text(buf, elem).strip()
            if text:
                device_names.append(text)
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
            elif elem.key == "TYPE":
                current.type_string = as_text(buf, elem)
            elif elem.typ == TYPE_COMPLEX:
                values = (decode_complex(buf, elem, current.type_string)
                          if current.type_string else None)
                if values:
                    current.add(elem.key, values)
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
        if utc is None and len(vals) >= 7:
            utc = _utc_from_gps9(vals[5], vals[6])
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


# GPS time stamps further than this from the median camera-clock offset are
# stale (GoPro's first payload) and left out of the reported clock spread.
GPS_CLOCK_OUTLIER_S = 1.0


def _build_gps_samples(blocks: List[_Block]
                       ) -> Tuple[List[dict], float, int, int]:
    """Return ``(samples, clock_spread_s, median_offset_us, stale_stamps)``.

    Each block's GPSU time minus its STMP gives the camera-clock offset; the
    median is used.  The spread is reported over offsets within
    GPS_CLOCK_OUTLIER_S of that median: GoPro's first payload often carries a
    GPS time a few seconds stale, which would otherwise dominate the number
    while having no effect on any position.
    """
    gps_blocks = [b for b in blocks if _gps_rows(b)[0]]
    if not gps_blocks:
        return [], 0.0, 0, 0

    # GPS9 (MAX2, HERO11+) has no GPSU block time; each sample carries its
    # own days/seconds instead.  The first sample of a block is taken at the
    # block's STMP, so its time anchors the camera clock exactly like GPSU.
    for block in gps_blocks:
        if block.utc is not None:
            continue
        rows, kind = _gps_rows(block)
        if kind != "GPS9" or not rows or len(rows[0]) < 7:
            continue
        divisors = list(block.scale or GPS9_SCALE)
        if len(divisors) < 7:
            divisors += GPS9_SCALE[len(divisors):]
        if (len(rows[0]) > 8 and int(rows[0][8]) < 2) or not divisors[6]:
            continue                                     # no fix: no time
        block.utc = _utc_from_gps9(rows[0][5] / (divisors[5] or 1.0),
                                   rows[0][6] / divisors[6])

    offsets = []
    for block in gps_blocks:
        if block.stmp_us is not None and block.utc is not None:
            offsets.append(_epoch_us(block.utc) - block.stmp_us)
    median_offset = int(statistics.median(offsets)) if offsets else 0
    close = [o for o in offsets
             if abs(o - median_offset) <= GPS_CLOCK_OUTLIER_S * 1e6]
    stale = len(offsets) - len(close)
    spread = (max(close) - min(close)) / 1e6 if len(close) > 1 else 0.0

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
            if kind == "GPS9":
                utc = None              # every GPS9 row carries its fix time
            elif stmp is not None and median_offset:
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
    return samples, spread, median_offset, stale


def _build_imu_streams(blocks_by_name: Dict[str, List[_Block]],
                       median_offset: int = 0,
                       wanted: Optional[Dict[str, int]] = None
                       ) -> Dict[str, List[dict]]:
    """Decode IMU-ish streams into ``{KEY: [{stmp_us, utc, values}]}``.

    GoPro's ``STNM`` is a human-readable description ("Magnetometer"), while
    the actual payload keys are ``MAGN``/``ACCL``/...; indexing by the payload
    key keeps this independent of firmware naming.
    """
    imu: Dict[str, List[dict]] = {}
    if wanted is None:
        wanted = {"MAGN": 3, "ACCL": 3, "GYRO": 3, "GRAV": 3,
                  "CORI": 4, "IORI": 4, "ORIN": 3}
    for _name, blocks in blocks_by_name.items():
        for key, size in wanted.items():
            present = [block for block in blocks if block.keys.get(key)]
            if not present:
                continue
            rows_out = imu.setdefault(key, [])
            for bi, block in enumerate(blocks):
                values = block.keys.get(key)
                if not values:
                    continue
                rows = _group(values, size)
                scale = block.scale
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


def _roll_pitch_from_body(x: float, y: float, z: float
                          ) -> Tuple[Optional[float], Optional[float]]:
    """Roll/pitch from an up vector in a ``(right, back, up)`` body frame.

    Positive roll is a clockwise roll (right side down); positive pitch means
    the camera is tilted down.  A level camera reads ``(0, 0, 1)``.
    """
    if abs(x) < 1e-12 and abs(z) < 1e-12:
        return None, None
    return (math.degrees(math.atan2(x, z)),
            math.degrees(math.atan2(y, z)))


def _gravity_roll_pitch(values: Sequence[float]
                        ) -> Tuple[Optional[float], Optional[float]]:
    """Derive absolute roll/pitch from a GoPro ``GRAV`` sample.

    Verified on deliberately tilted MAX footage: GRAV behaves as the *down*
    direction in a ``(right, down, forward)`` camera frame, so a level camera
    reads approximately ``[0, 1, 0]``, the first component goes negative when
    the camera's left side dips, and the third goes positive when the lens
    tilts down.  Fed to :func:`_roll_pitch_from_body` as ``(x, forward,
    down)`` this yields positive roll = clockwise (right side down) and
    positive pitch = camera tilted down.  Gravity has no yaw component, so
    heading still comes from the magnetometer or GPS course.

    Note that GRAV is GoPro's low-passed *apparent* gravity: in a moving
    vehicle it also contains the vehicle's own acceleration (turns, braking),
    which is why the leveling code takes a windowed median over time instead
    of trusting a single sample.
    """
    if len(values) < 3:
        return None, None
    return _roll_pitch_from_body(values[0], values[2], values[1])


def _fit_sensor_to_body(values: Sequence[float], calibration: dict
                        ) -> List[float]:
    """Rotate a FIT sensor sample into device axes with message 167 data.

    ``orientation_matrix`` is row-major and normally scaled by 65535; some
    files already store unit-length values, so the scale is detected instead
    of assumed.
    """
    return _apply_fit_matrix(_fit_matrix(calibration), values)


def _fit_matrix(calibration: dict) -> Optional[List[float]]:
    """The unit-scaled FIT orientation matrix, or ``None`` when absent."""
    matrix = calibration.get("orientation_matrix")
    if not matrix or len(matrix) < 9:
        return None
    scale = 65535.0 if any(abs(float(value)) > 2.0 for value in matrix) else 1.0
    return [float(value) / scale for value in matrix]


def _apply_fit_matrix(m: Optional[List[float]], values: Sequence[float]
                      ) -> List[float]:
    if m is None:
        return list(values[:3])
    x, y, z = values[:3]
    return [
        m[0] * x + m[1] * y + m[2] * z,
        m[3] * x + m[4] * y + m[5] * z,
        m[6] * x + m[7] * y + m[8] * z,
    ]


def _virb_image_up(body: Sequence[float], orientation_deg: Optional[float]
                   ) -> List[float]:
    """Rotate a device-frame up vector into the stored video's image frame.

    Garmin cameras rotate the sensor image by the FIT camera orientation so
    the recorded video is upright.  Applying the same rotation to the
    accelerometer's up vector makes roll/pitch relative to the exported image.
    """
    x, y, z = body[:3]
    if not orientation_deg:
        return [x, y, z]
    theta = math.radians(float(orientation_deg))
    c, s = math.cos(theta), math.sin(theta)
    return [x * c + z * s, y, -x * s + z * c]


# Default real-time window (seconds) over which gravity samples are combined
# for one image.  GoPro's GRAV is low-passed *apparent* gravity, so in a
# vehicle it swings by several degrees through every turn and braking phase;
# a 20 s median averages those out while a rigid mount's true tilt survives.
LEVEL_WINDOW_DEFAULT = 25.0
# Largest change of the leveling roll/pitch between consecutive images.  With
# a 25 s Gaussian window this kept every image-to-image step at or below the
# cap on the test drives (the uncapped median used before jumped up to 3.3
# degrees around stops), while following real slope changes.
LEVEL_MAX_STEP_DEG = 0.5
# CANDIDATE FOR REMOVAL (kept for now, off): gravity samples taken while the
# vehicle accelerates, brakes or turns are down-weighted by
# 1 / (1 + (a / LEVEL_ACCEL_REF)^2), a from GPS (m/s^2).  On four test drives
# it changed the angles by ~0.4 degrees but made image-to-image steps
# slightly larger, with no evidence of a more accurate horizon.  A later
# analysis showed why: on hilly roads most of the tilt variation is the real
# road gradient (GPS altitude explained 46-61% of it), which down-weighting
# cannot separate from acceleration.  Remove together with
# _vehicle_acceleration once --horizon-gradient (or a GPS-aided attitude
# filter, see README "Further research") has settled.
LEVEL_ACCEL_WEIGHTING = False
LEVEL_ACCEL_REF = 0.5
# --horizon-gradient: the road gradient from GPS altitude is taken out of the
# gravity samples before smoothing and added back per image, so a vehicle
# pitching on hills is followed without the smoothing lag.  Altitude is
# averaged over +-LEVEL_GRADIENT_ALT_SMOOTH_S and differenced over
# +-LEVEL_GRADIENT_BASELINE_M of travel; |gradient| is clamped.
LEVEL_GRADIENT_BASELINE_M = 30.0
LEVEL_GRADIENT_ALT_SMOOTH_S = 2.0
LEVEL_GRADIENT_MAX = 0.3
# The smoothed part keeps LEVEL_MAX_STEP_DEG; the total (smoothed + gradient)
# may change by up to this much per image, so crests are not delayed.
LEVEL_GRADIENT_MAX_STEP_DEG = 1.0


def _angle_unit_vector(row: dict, key: str) -> Optional[Tuple[float, float, float]]:
    """Return the (right, down, forward) unit vector behind one IMU sample.

    GRAV rows carry the vector directly; rows that only know ``roll_deg`` /
    ``pitch_deg`` (VIRB ``ACCL`` after calibration) are rebuilt from the two
    ``atan2`` angles, which is exact for the (roll, pitch) convention used
    throughout this file.
    """
    values = row.get("values") or ()
    if key == "GRAV" and len(values) >= 3:
        x, down, forward = float(values[0]), float(values[1]), float(values[2])
    else:
        roll = row.get("roll_deg")
        pitch = row.get("pitch_deg")
        if roll is None or pitch is None:
            return None
        r, q = math.radians(float(roll)), math.radians(float(pitch))
        if abs(math.cos(r)) < 1e-9 or abs(math.cos(q)) < 1e-9:
            return None
        x, down, forward = math.tan(r), 1.0, math.tan(q)
    norm = math.sqrt(x * x + down * down + forward * forward)
    if norm < 1e-9:
        return None
    return x / norm, down / norm, forward / norm


def _weighted_vector_angles(vectors: Sequence[Tuple[float, float, float]],
                            weights: Sequence[float]
                            ) -> Optional[Tuple[float, float]]:
    """Roll/pitch of the weighted mean of unit gravity vectors.

    Averaging vector components (not angles) avoids any wrap-around trouble.
    Unlike the median used before, a weighted mean changes smoothly as the
    window slides, so consecutive images do not step between values.
    """
    total = sum(weights)
    if not vectors or total <= 0:
        return None
    x = sum(w * v[0] for v, w in zip(vectors, weights)) / total
    down = sum(w * v[1] for v, w in zip(vectors, weights)) / total
    forward = sum(w * v[2] for v, w in zip(vectors, weights)) / total
    roll, pitch = _roll_pitch_from_body(x, forward, down)
    if roll is None or pitch is None:
        return None
    return roll, pitch


def _vehicle_acceleration(track) -> Tuple[List[dt.datetime], List[float]]:
    """Horizontal acceleration (m/s^2) of the vehicle at each GPS sample.

    Longitudinal from the change of speed, lateral from speed times the turn
    rate, both over about 1.5 s.  Empty without a usable track.
    """
    samples = [s for s in (getattr(track, "samples", None) or [])
               if isinstance(s.get("utc"), dt.datetime)]
    times = [s["utc"] for s in samples]
    accel: List[float] = []
    for i in range(len(samples)):
        lo = bisect.bisect_left(times, times[i] - dt.timedelta(seconds=0.75))
        hi = min(len(samples) - 1, bisect.bisect_right(
            times, times[i] + dt.timedelta(seconds=0.75)) - 1)
        a, b = samples[lo], samples[hi]
        span = (b["utc"] - a["utc"]).total_seconds()
        va, vb = a.get("speed"), b.get("speed")
        if span < 0.2 or va is None or vb is None:
            accel.append(0.0)
            continue
        along = (vb - va) / span
        lateral = 0.0
        ca, cb = a.get("gps_course"), b.get("gps_course")
        if ca is not None and cb is not None and min(va, vb) > 1.0:
            turn = math.radians((cb - ca + 540.0) % 360.0 - 180.0) / span
            lateral = 0.5 * (va + vb) * turn
        accel.append(math.hypot(along, lateral))
    return times, accel


def _road_gradient(track) -> Tuple[List[dt.datetime], List[float]]:
    """Road gradient (rise over run) at each GPS sample, from GPS altitude.

    Altitude is averaged over +-LEVEL_GRADIENT_ALT_SMOOTH_S real seconds, then
    differenced over +-LEVEL_GRADIENT_BASELINE_M of travel.  Shorter spans
    (stops, the ends of the track) give 0.  Empty without a usable track.
    """
    samples = getattr(track, "samples", None) or []
    cum = getattr(track, "cum", None) or []
    idx = [i for i, s in enumerate(samples)
           if isinstance(s.get("utc"), dt.datetime) and s.get("alt") is not None
           and i < len(cum)]
    times = [samples[i]["utc"] for i in idx]
    dist = [cum[i] for i in idx]
    alts = [float(samples[i]["alt"]) for i in idx]
    prefix = [0.0]
    for value in alts:
        prefix.append(prefix[-1] + value)
    smooth = []
    window = dt.timedelta(seconds=LEVEL_GRADIENT_ALT_SMOOTH_S)
    for k, when in enumerate(times):
        lo = bisect.bisect_left(times, when - window)
        hi = bisect.bisect_right(times, when + window)
        smooth.append((prefix[hi] - prefix[lo]) / max(1, hi - lo))
    gradient = []
    base = LEVEL_GRADIENT_BASELINE_M
    for k in range(len(times)):
        lo = bisect.bisect_left(dist, dist[k] - base)
        hi = min(len(dist) - 1, bisect.bisect_right(dist, dist[k] + base) - 1)
        run = dist[hi] - dist[lo]
        if run < 1.3 * base:
            gradient.append(0.0)
            continue
        value = (smooth[hi] - smooth[lo]) / run
        gradient.append(max(-LEVEL_GRADIENT_MAX, min(LEVEL_GRADIENT_MAX, value)))
    return times, gradient


def _gradient_tilt(gradient: float, mount_deg: float) -> Tuple[float, float]:
    """Camera ``(roll, pitch)`` caused by driving on *gradient*.

    Uphill raises the vehicle's nose by atan(gradient).  For a camera whose
    front lens points *mount_deg* clockwise of travel (the heading offset)
    that is ``pitch = -g cos(mount)`` and ``roll = +g sin(mount)`` (camera
    pitch positive = looking down, roll positive = right side down).  The
    signs match regressions of GRAV against GPS gradient on 000/090/270
    mounts (pitch for 000, roll with opposite signs for 090 and 270).
    """
    angle = math.degrees(math.atan(gradient))
    mount = math.radians(mount_deg or 0.0)
    return angle * math.sin(mount), -angle * math.cos(mount)


def _vector_from_angles(roll: float, pitch: float
                        ) -> Optional[Tuple[float, float, float]]:
    """Unit gravity vector ``(x, down, forward)`` for camera roll/pitch."""
    r, q = math.radians(roll), math.radians(pitch)
    if abs(math.cos(r)) < 1e-9 or abs(math.cos(q)) < 1e-9:
        return None
    x, forward = math.tan(r), math.tan(q)
    norm = math.sqrt(x * x + 1.0 + forward * forward)
    return x / norm, 1.0 / norm, forward / norm


def _limit_steps(values: Sequence[Tuple[float, float]], max_step: float
                 ) -> List[Tuple[float, float]]:
    """Cap the change between consecutive (roll, pitch) pairs.

    A forward and a backward rate-limited pass are averaged, so the result
    has no systematic lag in either direction; the average of two sequences
    whose steps are all <= max_step also has steps <= max_step.
    """
    if len(values) < 2 or max_step <= 0:
        return list(values)

    def one_way(seq):
        out = [seq[0]]
        for value in seq[1:]:
            prev = out[-1]
            out.append(tuple(prev[k] + max(-max_step,
                                           min(max_step, value[k] - prev[k]))
                             for k in range(2)))
        return out

    forward = one_way(list(values))
    backward = one_way(list(values)[::-1])[::-1]
    return [((f[0] + b[0]) / 2.0, (f[1] + b[1]) / 2.0)
            for f, b in zip(forward, backward)]


def _image_angles(records: Sequence[dict], imu: Optional[Dict[str, List[dict]]],
                  *, window_s: float = LEVEL_WINDOW_DEFAULT,
                  max_delta_s: float = 1.0, track=None,
                  max_step: float = LEVEL_MAX_STEP_DEG,
                  accel_weighting: bool = LEVEL_ACCEL_WEIGHTING,
                  gradient_mount_deg: Optional[float] = None
                  ) -> Tuple[Dict[int, Tuple[float, float]],
                             Dict[int, Tuple[float, float]]]:
    """Map ``frame_index`` to raw and leveling ``(roll, pitch)`` per image.

    Returns ``(raw, level)``.  ``raw`` is the single IMU sample nearest to the
    capture time (within ``max_delta_s``), kept for the telemetry exports.
    ``level`` is what the horizon correction actually uses:

    * a Gaussian-weighted mean of the gravity vectors within ``window_s``
      *real* seconds centred on the capture time (sigma = window / 4);
    * with *accel_weighting* (off; candidate for removal), samples taken
      while the vehicle accelerates, brakes or turns (from the GPS *track*)
      count less;
    * with *gradient_mount_deg* (``--horizon-gradient``; the heading offset,
      0 for a forward lens), the tilt caused by the road gradient (GPS
      altitude of *track*) is removed from every sample before averaging and
      the image's own gradient tilt is added back afterwards, so hills are
      followed without the window's lag;
    * near the start and end of the recording the window is not cut in half:
      the value of the first/last full window is held instead;
    * finally the change between consecutive images is capped at *max_step*
      degrees (forward/backward averaged, so without lag); with the gradient
      term, that cap applies to the smoothed part and the total is capped at
      LEVEL_GRADIENT_MAX_STEP_DEG.

    Working in real (UTC) time rather than in exported-image neighbours makes
    this independent of the frame spacing, so TimeWarp/timelapse footage is
    handled like normal video.  With ``window_s == 0`` the nearest sample is
    used as is (no smoothing, no cap).

    ``GRAV`` is preferred because it is absolute; the ``ACCL`` fallback covers
    streams that only provide calibrated roll/pitch.  Images without any
    sample in reach are omitted from both maps.
    """
    raw: Dict[int, Tuple[float, float]] = {}
    level: Dict[int, Tuple[float, float]] = {}
    if not imu:
        return raw, level
    indexes: Dict[str, Tuple[List[dt.datetime], List[dict]]] = {}
    for key in ("GRAV", "ACCL"):
        timed = []
        for row in imu.get(key) or []:
            utc = _as_utc(row.get("utc"))
            if utc is not None:
                timed.append((utc, row))
        if timed:
            timed.sort(key=lambda item: item[0])
            indexes[key] = ([item[0] for item in timed],
                            [item[1] for item in timed])
    half = max(0.0, float(window_s)) / 2.0
    sigma = half / 2.0
    acc_times, acc_values = (_vehicle_acceleration(track)
                             if half > 0 and accel_weighting else ([], []))
    use_gradient = gradient_mount_deg is not None and half > 0
    grad_times, grad_values = (_road_gradient(track) if use_gradient
                               else ([], []))

    def gradient_term(when: dt.datetime) -> Tuple[float, float]:
        if not grad_times:
            return 0.0, 0.0
        pos = min(len(grad_times) - 1, bisect.bisect_left(grad_times, when))
        if pos > 0 and (abs((grad_times[pos - 1] - when).total_seconds())
                        < abs((grad_times[pos] - when).total_seconds())):
            pos -= 1
        if abs((grad_times[pos] - when).total_seconds()) > 2.0:
            return 0.0, 0.0
        return _gradient_tilt(grad_values[pos], gradient_mount_deg)

    image_terms: Dict[int, Tuple[float, float]] = {}
    vector_cache: Dict[Tuple[str, int], Optional[Tuple[float, float, float]]] = {}

    def accel_weight(when: dt.datetime) -> float:
        if not acc_times:
            return 1.0
        pos = min(len(acc_times) - 1, bisect.bisect_left(acc_times, when))
        if pos > 0 and (abs((acc_times[pos - 1] - when).total_seconds())
                        < abs((acc_times[pos] - when).total_seconds())):
            pos -= 1
        if abs((acc_times[pos] - when).total_seconds()) > 2.0:
            return 1.0
        ratio = acc_values[pos] / LEVEL_ACCEL_REF
        return 1.0 / (1.0 + ratio * ratio)

    weight_cache: Dict[Tuple[str, int], float] = {}
    for record in records:
        target = _as_utc(record.get("datetime_utc"))
        if target is None:
            continue
        frame = int(record["frame_index"])
        for key in ("GRAV", "ACCL"):
            index = indexes.get(key)
            if index is None:
                continue
            times, rows = index
            match = _nearest_timed(times, rows, target, max_delta_s)
            if match is None:
                continue
            row, _delta = match
            roll = row.get("roll_deg")
            pitch = row.get("pitch_deg")
            if roll is None and key == "GRAV":
                roll, pitch = _gravity_roll_pitch(row.get("values") or [])
            if roll is None or pitch is None:
                continue
            raw[frame] = (float(roll), float(pitch))
            if half <= 0.0:
                level[frame] = (round(float(roll), 3), round(float(pitch), 3))
                break
            # Hold the first/last full window near the recording's ends.
            centre = target
            if (times[-1] - times[0]).total_seconds() > 2 * half:
                earliest = times[0] + dt.timedelta(seconds=half)
                latest = times[-1] - dt.timedelta(seconds=half)
                centre = min(max(target, earliest), latest)
            lo = bisect.bisect_left(times, centre - dt.timedelta(seconds=half))
            hi = bisect.bisect_right(times, centre + dt.timedelta(seconds=half))
            vectors, weights = [], []
            for i in range(lo, hi):
                if (key, i) in vector_cache:
                    vector = vector_cache[(key, i)]
                else:
                    vector = _angle_unit_vector(rows[i], key)
                    if vector is not None and grad_times:
                        # Take this sample's road-gradient tilt out first.
                        s_roll, s_pitch = _roll_pitch_from_body(
                            vector[0], vector[2], vector[1])
                        if s_roll is not None and s_pitch is not None:
                            g_roll, g_pitch = gradient_term(times[i])
                            vector = _vector_from_angles(s_roll - g_roll,
                                                         s_pitch - g_pitch)
                    vector_cache[(key, i)] = vector
                if vector is None:
                    continue
                offset_s = (times[i] - centre).total_seconds()
                cached = weight_cache.get((key, i))
                if cached is None:
                    cached = weight_cache[(key, i)] = accel_weight(times[i])
                vectors.append(vector)
                weights.append(math.exp(-0.5 * (offset_s / sigma) ** 2)
                               * cached)
            angles = _weighted_vector_angles(vectors, weights)
            if angles is not None:
                level[frame] = angles
                if grad_times:
                    image_terms[frame] = gradient_term(target)
            break
    if half > 0 and level:
        ordered = sorted((record for record in records
                          if int(record["frame_index"]) in level),
                         key=lambda r: (_as_utc(r.get("datetime_utc"))
                                        or dt.datetime.min.replace(
                                            tzinfo=dt.timezone.utc),
                                        int(r["frame_index"])))
        frames = [int(r["frame_index"]) for r in ordered]
        limited = _limit_steps([level[f] for f in frames], max_step)
        if image_terms:
            limited = _limit_steps(
                [(roll + image_terms.get(f, (0.0, 0.0))[0],
                  pitch + image_terms.get(f, (0.0, 0.0))[1])
                 for f, (roll, pitch) in zip(frames, limited)],
                LEVEL_GRADIENT_MAX_STEP_DEG)
        for frame, (roll, pitch) in zip(frames, limited):
            level[frame] = (round(roll, 3), round(pitch, 3))
    return raw, level


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
    return (math.degrees(math.atan2(ex, nx)) + 360.0) % 360.0


class _NearestImu:
    """Sorted index over IMU rows answering :func:`_nearest_imu` queries.

    Same result as :func:`_nearest_imu` (which scans every row per query)
    but O(log n) per lookup, which matters when every GPS sample of a long
    recording is matched against 200 Hz IMU streams.
    """

    def __init__(self, rows: Sequence[dict]):
        self.rows = rows
        self.has_stmp = bool(rows) and rows[0].get("stmp_us") is not None
        self.has_utc = bool(rows) and rows[0].get("utc") is not None
        self._indexes: Dict[str, Tuple[list, List[int]]] = {}

    def _index(self, name: str):
        if name not in self._indexes:
            rows = self.rows
            if name == "stmp":
                key = lambda i: rows[i].get("stmp_us") or 0  # noqa: E731
            else:
                key = lambda i: rows[i]["utc"]               # noqa: E731
            # Stable sort: equal keys keep row order, like min() does.
            order = sorted(range(len(rows)), key=key)
            self._indexes[name] = ([key(i) for i in order], order)
        return self._indexes[name]

    def _closest(self, name: str, target, distance):
        keys, order = self._index(name)
        pos = bisect.bisect_left(keys, target)
        best = None
        for j in (pos - 1, pos):
            if 0 <= j < len(keys):
                # First row carrying this key, as min() would return.
                j = bisect.bisect_left(keys, keys[j])
                delta = distance(keys[j], target)
                if best is None or delta < best[0]:
                    best = (delta, order[j])
        return best

    def nearest(self, stmp_us: Optional[int], utc: Optional[dt.datetime],
                max_delta_us: int = 500000) -> Optional[dict]:
        if stmp_us is not None and self.has_stmp:
            best = self._closest("stmp", stmp_us, lambda a, b: abs(a - b))
            if best is not None and best[0] <= max_delta_us:
                return self.rows[best[1]]
            return None
        if utc is not None and self.has_utc:
            best = self._closest(
                "utc", utc, lambda a, b: abs((a - b).total_seconds()))
            if best is not None and best[0] <= max_delta_us / 1e6:
                return self.rows[best[1]]
        return None


def _attach_compass(gps_samples: List[dict], imu: Dict[str, List[dict]]
                    ) -> None:
    mags = imu.get("MAGN", [])
    gravity = imu.get("GRAV") or imu.get("ACCL") or []
    if not mags or not gravity:
        return
    mag_index = _NearestImu(mags)
    grav_index = _NearestImu(gravity)
    for sample in gps_samples:
        mag = mag_index.nearest(sample.get("stmp_us"), sample.get("utc"))
        grav = grav_index.nearest(sample.get("stmp_us"), sample.get("utc"))
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
    clock_stale_stamps: int = 0                      # GPSU outliers ignored
    # Per-frame exposure: ``SHUT`` (seconds) and ``ISOE`` rows, same shape
    # as the IMU rows.  Kept apart from ``imu`` so telemetry exports stay as
    # they were.
    exposure: Dict[str, List[dict]] = field(default_factory=dict)
    origin_utc: Optional[dt.datetime] = None
    time_scale: float = 1.0
    camera_utc_offset: Optional[int] = None


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


def attach_exposure(records: Sequence[dict],
                    exposure: Optional[Dict[str, List[dict]]],
                    max_delta_s: float = 1.0) -> int:
    """Copy GoPro's per-frame shutter time and ISO onto the image records.

    ``SHUT``/``ISOE`` hold one value per recorded frame; the row nearest to
    each image's capture time (within *max_delta_s* real seconds) is used.
    Sets ``exposure_time_s`` and ``iso``; returns how many images got one.
    """
    if not exposure:
        return 0
    found = 0
    for key, field_name, convert in (("SHUT", "exposure_time_s", float),
                                     ("ISOE", "iso", int)):
        timed = sorted(((_as_utc(row.get("utc")), row)
                        for row in exposure.get(key) or []
                        if row.get("utc") is not None and row.get("values")),
                       key=lambda item: item[0])
        if not timed:
            continue
        times = [item[0] for item in timed]
        for record in records:
            target = _as_utc(record.get("datetime_utc"))
            if target is None:
                continue
            pos = bisect.bisect_left(times, target)
            best = None
            for j in (pos - 1, pos):
                if 0 <= j < len(times):
                    delta = abs((times[j] - target).total_seconds())
                    if best is None or delta < best[0]:
                        best = (delta, timed[j][1])
            if best is not None and best[0] <= max_delta_s:
                value = best[1]["values"][0]
                if value and value > 0:
                    record[field_name] = convert(value)
                    if key == "SHUT":
                        found += 1
    return found


def parse_gpmf(buf: bytes, *, imu_heading: bool = False) -> GpmfData:
    """Parse a raw gpmd payload into GPS samples and IMU streams."""
    if not buf:
        raise GpmfError("empty GPMF payload")
    names: List[str] = []
    blocks = _collect_blocks(buf, names)
    device_name = names[0] if names else None
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
    gps_samples, spread, median_offset, stale = _build_gps_samples(all_blocks)
    imu = _build_imu_streams(blocks, median_offset)
    exposure = _build_imu_streams(blocks, median_offset,
                                  wanted={"SHUT": 1, "ISOE": 1})
    if imu_heading:
        _attach_compass(gps_samples, imu)
    return GpmfData(device_name=device_name, streams=streams,
                    gps_samples=gps_samples, imu=imu,
                    has_gps=bool(gps_samples), clock_spread_s=spread,
                    clock_stale_stamps=stale, exposure=exposure)


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
    firmware: Optional[str] = None                   # camera firmware string
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
    pose_pitch: float = 0.0                          # XMP-GPano PosePitchDegrees
    pose_roll: float = 0.0                           # XMP-GPano PoseRollDegrees

    exposure_time: Optional[float] = None            # seconds (GPMF SHUT)
    iso: Optional[int] = None                        # GPMF ISOE

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
    # Milliseconds in GPSTimeStamp: time-lapse frames can share a whole second,
    # and readers that order a sequence by GPS time alone would tie them.
    time_stamp = value.strftime("%H:%M:%S") + ".%03d" % (value.microsecond // 1000)
    date_stamp = value.strftime("%Y:%m:%d")
    # Exif-style "YYYY:mm:dd HH:MM:SS.sssZ": under exiftool's -n the ISO form
    # (with "-" and "T") is rejected for XMP date tags and silently dropped.
    return date_stamp, time_stamp, "%s %sZ" % (date_stamp, time_stamp)


def provenance(meta: FrameMetadata) -> str:
    """Short technical note for EXIF ``UserComment`` (never a keyword tag).

    Only facts without an EXIF tag of their own: where the heading and the
    position came from, plus per-camera flags in ``extra``.  Everything else
    has a proper tag (``GPSHPositioningError``, ``GPSDOP``, ``Software`` /
    ``XMP-aux:Firmware``) or is an exporter detail kept in the per-image
    telemetry GeoJSON (video, frame, pts, distance, accuracy source, DOPs).
    """
    parts = ["heading_source=%s" % meta.heading_source]
    if meta.position_source:
        parts.append("position_source=%s" % meta.position_source)
    if meta.extra:
        for key, value in sorted(meta.extra.items()):
            if value is not None:
                parts.append("%s=%s" % (key, value))
    return "; ".join(parts)


def build_exiftool_args(meta: FrameMetadata) -> List[str]:
    """Return exiftool arguments (without the file name)."""
    # ColorSpace: exiftool otherwise creates the EXIF IFD with 65535
    # ("Uncalibrated"), but ffmpeg's JPEG/PNG output is plain sRGB.
    tags: List[str] = ["-n", "-charset", "filename=utf8", "-Orientation=1",
                       "-ColorSpace=1"]

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

    # --- camera identity and generating software -------------------------
    # EXIF ``Software`` is "the software or firmware of the camera", so it
    # carries the camera firmware, exactly as the cameras' own photos do, and
    # Adobe's dedicated ``XMP-aux:Firmware`` repeats it.  The exporter is named
    # in ``XMP:CreatorTool`` (and ``XMP-GPano:StitchingSoftware`` for 360).
    # With no known firmware, ``Software`` falls back to the exporter.
    if meta.make:
        tags.append("-Make=" + meta.make)
    if meta.model:
        tags.append("-Model=" + meta.model)
    tags.append("-Software=" + (meta.firmware or SOFTWARE_TAG))
    if meta.firmware:
        tags.append("-XMP-aux:Firmware=" + meta.firmware)
    if meta.serial:
        tags += ["-SerialNumber=%s" % meta.serial,
                 "-CameraSerialNumber=%s" % meta.serial,
                 "-XMP-aux:SerialNumber=%s" % meta.serial]

    # --- per-frame exposure recorded by the camera (GoPro SHUT/ISOE) ------
    if meta.exposure_time:
        tags.append("-ExposureTime=%r" % meta.exposure_time)
    if meta.iso:
        tags.append("-ISO=%d" % meta.iso)

    # --- EXIF GPS (UTC date/time, true-north directions) -----------------
    if meta.lat is not None and meta.lon is not None:
        tags += ["-GPSVersionID=2 3 0 0",
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
                     "-GPSImgDirectionRef=T"]  # raw value: -n is on
        if meta.course is not None:
            tags += ["-GPSTrack=%r" % meta.course,
                     "-GPSTrackRef=T"]
        if meta.accuracy is not None:
            tags += ["-GPSHPositioningError=%r" % meta.accuracy,
                     "-XMP:GPSHPositioningError=%r" % meta.accuracy]
        if meta.hdop is not None:
            tags += ["-GPSDOP=%r" % meta.hdop, "-XMP:GPSDOP=%r" % meta.hdop]
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
            tags += ["-XMP:GPSImgDirection=%r" % meta.img_direction,
                     "-XMP:GPSImgDirectionRef=T"]
        if meta.course is not None:
            tags += ["-XMP:GPSTrack=%r" % meta.course, "-XMP:GPSTrackRef=T"]
        if date_stamp:
            tags.append("-XMP:GPSDateTime=" + gps_datetime)

    # --- provenance (never a keyword tag) --------------------------------
    # Only UserComment: ImageDescription, XPComment and XMP:Description are
    # shown as the photo's caption by galleries and Panoramax.
    tags += ["-UserComment=" + provenance(meta),
             "-XMP:CreatorTool=" + SOFTWARE_TAG]
    if local is not None:
        tags += ["-XMP:CreateDate=" + local.strftime("%Y:%m:%d %H:%M:%S"),
                 "-XMP:ModifyDate=" + local.strftime("%Y:%m:%d %H:%M:%S")]
    if meta.captured_utc is not None:
        tags.append("-XMP:MetadataDate=" + _split_utc(meta.captured_utc)[2])

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
            heading = meta.img_direction
        first_date = _fmt_local(local) or _fmt_utc(meta.captured_utc)
        pose_roll = round(
            (float(meta.pose_roll or 0.0) + 180.0) % 360.0 - 180.0, 3)
        if pose_roll == -180.0:
            pose_roll = 180.0
        pose_pitch = round(
            max(-90.0, min(90.0, float(meta.pose_pitch or 0.0))), 3)
        tags += [
            "-XMP-GPano:UsePanoramaViewer=True",
            "-XMP-GPano:CaptureSoftware=%s" % (meta.model or ""),
            "-XMP-GPano:StitchingSoftware=%s" % SOFTWARE_TAG,
            "-XMP-GPano:ProjectionType=equirectangular",
            "-XMP-GPano:PosePitchDegrees=%r" % pose_pitch,
            "-XMP-GPano:PoseRollDegrees=%r" % pose_roll,
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
        # An unknown heading must stay absent: 0 would claim "facing north".
        if heading is not None:
            tags += ["-XMP-GPano:PoseHeadingDegrees=%r" % (heading % 360.0),
                     "-XMP-GPano:InitialViewHeadingDegrees=%r"
                     % (heading % 360.0)]
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

def flat_video_info(path):
    """fps, frame count, duration, dimensions, codec and creation time.

    Built from :func:`probe_metadata`, which camera detection has already
    run (and cached) for this file, so no extra ffprobe call is needed.
    """
    try:
        meta = probe_metadata(path, FFPROBE)
    except UnreadableVideo as exc:
        raise Fatal("%s: %s" % (path, exc))
    videos = meta.video_streams_info()
    if not videos:
        raise Fatal("no video stream in %s (not a playable video file)" % path)
    video = videos[0]
    fps = _parse_rate(video.get("avg_frame_rate") or video.get("r_frame_rate"))

    duration = None
    if video.get("duration"):
        duration = float(video["duration"])
    elif meta.duration:
        duration = meta.duration

    return {
        "fps": fps,
        "frames": int(video.get("nb_frames") or 0),
        "duration": duration,
        "width": video.get("width"),
        "height": video.get("height"),
        "codec": video.get("codec_name"),
        "creation_time": meta.format_tags.get("creation_time"),
    }


def blackvue_probe_video(path):
    """Return fps, frame count, duration, dimensions and codec via ffprobe."""
    return flat_video_info(path)


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
        point, args.heading_offset, args.magnetic_declination,
        getattr(args, "recenter_deg", 0.0))
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


blackvue__unpositioned_choices = unpositioned_choices


def blackvue__metadata_for(record, source, args, camera):
    return FrameMetadata(
        camera=CAMERA_LABELS[camera], video=record["video"],
        sequence=record["sequence"], frame_index=record["frame_index"],
        pts=record["pts_seconds"], distance=record.get("distance_m"),
        script_name=SCRIPT_NAME, make=source.get("make"),
        model=source.get("model"), firmware=source.get("software"),
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
    log_heading_offset(args)

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
    if to_extract:
        temp_dir = make_temp_dir("blackvue_", [r["path"] for r in to_extract])
        try:
            extracted = extract_frames_by_index(
                video, [r["frame_index"] for r in to_extract], temp_dir,
                image_format=args.format, quality=args.quality,
                workers=args.workers, threads=args.threads, timeline=timeline)
            written, failed_frames = place_extracted(extracted, to_extract)
            written_paths = [record["path"] for record in written]
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
# GoPro Max / MAX2 (360) pipeline
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
    tz_offset_s: Optional[int] = None   # camera TZON (MAX2), None on the MAX
    camera: str = CAMERA_MAX            # CAMERA_MAX or CAMERA_MAX2
    out_width: int = max360_DEFAULT_WIDTH        # equirectangular output size
    out_height: int = max360_DEFAULT_HEIGHT


def max360_native_size(width: int, height: int,
                       already_equirect: bool) -> Tuple[int, int]:
    """Native equirectangular size: four cube faces around (MAX 5376x2688,
    MAX2 7680x3840).  Equirectangular input keeps the historical default."""
    if already_equirect or not height:
        return max360_DEFAULT_WIDTH, max360_DEFAULT_HEIGHT
    try:
        _side, centre, _blend = max360__eac_layout(width, height)
    except max360_BackendError:
        return max360_DEFAULT_WIDTH, max360_DEFAULT_HEIGHT
    return 4 * centre, 2 * centre


def max360_output_size(info: "max360_VideoInfo", args) -> Tuple[int, int]:
    """``--width``/``--height`` or the camera's native size (2:1)."""
    width, height = getattr(args, "width", None), getattr(args, "height", None)
    if width is None and height is None:
        return max360_native_size(info.width, info.height,
                                  info.already_equirect)
    if width is None:
        width = 2 * height
    if height is None:
        height = width // 2 + (width // 2) % 2
    return width, height


max360__parse_rate = _parse_rate


def max360__frame_count(stream: dict, duration: float, fps: float) -> int:
    try:
        count = int(stream.get("nb_frames"))
        if count > 0:
            return count
    except (TypeError, ValueError):
        pass
    return max(1, int(round(duration * fps))) if duration and fps else 1


def max360_probe(path: str) -> max360_VideoInfo:
    """Inspect a GoPro Max file and describe its hemisphere/telemetry layout."""
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
    settings = read_gopro_settings(path)
    return max360_VideoInfo(
        path=path, duration=duration, fps=fps, frame_count=frame_count,
        video_ordinals=ordinals, hemi_a=a_idx, hemi_b=b_idx, gpmd=meta.gpmd_index,
        width=width, height=height, creation_time=creation,
        already_equirect=already,
        firmware=meta.format_tags.get("firmware") or settings.firmware,
        model=gopro_model_name(settings.model, "GoPro Max"),
        serial=(meta.format_tags.get("serial_number")
                or meta.format_tags.get("serial") or settings.serial),
        tz_offset_s=settings.tz_offset_s,
        camera=gopro_360_camera(path, None, videos)[0],
    )


# ---------------------------------------------------------------------------
# EAC stitching filter
# ---------------------------------------------------------------------------
# Known GoPro Max hemisphere layouts: (strip width, strip height) ->
# (side width, centre width, blend width).  A strip is side + centre + side
# with no gaps; each side region contains its own 2*blend overlap seam.
max360_EAC_LAYOUTS = {
    (4096, 1344): (1376, 1344, 32),
    (2272, 736): (768, 736, 16),
    # GoPro Max2: 1984 + 1920 + 1984.  Measured on GS010037: content repeats
    # 64 px apart across each seam (like the MAX), so the overlap crops are
    # 64 px, not the 128 px the generic gap formula below would assume, and
    # 928 unique + 64 blended + 928 unique fill a 1920 face without scaling.
    (5888, 1920): (1984, 1920, 32),
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
                     hemi_height: int = max360_HEMI_H,
                     yaw: float = 0.0, roll: float = 0.0,
                     pitch: float = 0.0) -> str:
    """Build the ffmpeg filter graph for one EAC hemisphere pair.

    A native 4096x1344 strip is side 1376 + centre 1344 + side 1376 with no
    gaps.  The outer halves of the left/right side regions overlap by 32 px,
    so they must be cross-blended instead of simply cropped (max2sphere does
    the same).  The six faces are assembled into a 3x2 EAC net and v360
    reprojects it to equirectangular.  The 2272x736 (side 768, centre 736,
    blend 16) layout is handled by the same code.  A non-zero *yaw*
    re-centres the sphere and *roll*/*pitch* level it, in the same v360 call
    (see :func:`max360_rotation_opts`).
    """
    yaw_opt = max360_rotation_opts(yaw, roll, pitch)
    lines: List[str] = []
    if already_equirect:
        if select:
            lines.append("[0:%d]select='%s'[src]" % (a, select))
        else:
            lines.append("[0:%d]null[src]" % a)
        if yaw_opt:
            lines.append("[src]v360=e:e%s:interp=%s:w=%d:h=%d[OUT]"
                         % (yaw_opt, interp, width, height))
        else:
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
    lines.append("[eac]v360=eac:equirect%s:interp=%s:w=%d:h=%d[OUT]"
                 % (yaw_opt, interp, width, height))
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


def max360__level_image(src: str, dest: str, roll_deg: float, pitch_deg: float,
                        *, width: int, height: int, image_format: str,
                        quality: int, interp: str, threads: int,
                        yaw: float = 0.0) -> None:
    """Level (and re-centre) one stitched equirectangular image."""
    rotate = (abs(float(roll_deg)) >= 0.05
              or abs(float(pitch_deg)) >= 0.05 or abs(float(yaw)) >= 0.05)
    same_format = (os.path.splitext(src)[1].lower()
                   == os.path.splitext(dest)[1].lower())
    if not rotate and same_format:
        shutil.copyfile(src, dest)
        return
    cmd = [FFMPEG, "-v", "error", "-y", "-i", src]
    if rotate:
        cmd += ["-vf", max360_level_filter(
            roll_deg, pitch_deg, width, height, interp, yaw=yaw)]
    cmd += ["-frames:v", "1",
            "-pix_fmt", "rgb24" if image_format == "png" else "yuvj420p"]
    cmd += ffmpeg_quality_args(image_format, quality)
    if threads:
        cmd += ["-threads", str(threads)]
    cmd.append(dest)
    max360__run(cmd)


def max360__stitch_batch(info: max360_VideoInfo, frames: Sequence[int],
                  by_frame: Dict[int, str], temp_dir: str, batch_index: int,
                  *, width: int, height: int, quality: int,
                  image_format: str, interp: str, threads: int,
                  keep_filter: Optional[str], total_batches: int,
                  seek_time: Optional[float],
                  levels: Optional[Mapping[int, Tuple[float, float]]] = None,
                  workers: int = 1, yaw: float = 0.0,
                  rotation: Optional[Tuple[float, float]] = None) -> None:
    """Run one bounded EAC/v360 batch and copy its images to their targets.

    *rotation* ``(roll, pitch)`` levels every frame of the batch with one
    rotation inside the stitching v360 (single pass, no intermediate).

    When *levels* contains a roll/pitch pair for any selected frame, the batch
    is stitched to lossless PNG intermediates and each frame is then remapped
    on the sphere with its own v360 rotation.  The video is still decoded only
    once per batch.
    """
    ordered = [int(frame) for frame in frames]
    leveled = (rotation is None and levels is not None
               and any(levels.get(frame) is not None for frame in ordered))
    roll, pitch = rotation if rotation is not None else (0.0, 0.0)
    selected = ordered if seek_time is None else\
        [frame - ordered[0] for frame in ordered]
    select = select_expression(selected)
    # Without leveling the re-centring yaw is part of the stitching v360;
    # leveled frames get it in their per-frame rotation instead.
    script = max360_build_eac_filter(
        info.hemi_a, info.hemi_b, width, height, select=select,
        interp=interp, already_equirect=info.already_equirect,
        hemi_width=info.width, hemi_height=info.height,
        yaw=0.0 if leveled else yaw, roll=roll, pitch=pitch)
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

    # A leveled frame needs a lossless intermediate so the sphere remap is the
    # only lossy step in the final encode.  The PNG is stored uncompressed
    # (zlib level 0): it is a throwaway file, so compressing it is wasted time.
    # (PNG rather than BMP because it keeps the stream's sample aspect ratio,
    # which ends up in the final JPEG's JFIF/EXIF resolution tags.)
    if leveled:
        ext, pix_fmt, quality_args = "png", "rgb24", ["-compression_level", "0"]
    else:
        ext = "png" if image_format == "png" else "jpg"
        pix_fmt = "rgb24" if image_format == "png" else "yuvj420p"
        quality_args = ffmpeg_quality_args(image_format, quality)
    prefix = "img_%03d_%s_" % (batch_index, attempt)
    pattern = os.path.join(temp_dir, prefix + "%06d." + ext)
    cmd = [FFMPEG, "-v", "error", "-y"]
    if seek_time is not None:
        cmd += ["-ss", "%.6f" % seek_time]
    cmd += ["-i", info.path, "-filter_complex_script", script_path,
            "-map", "[OUT]", "-an", "-sn", "-dn", "-vsync", "0",
            "-frames:v", str(len(ordered)),
            "-pix_fmt", pix_fmt]
    cmd += quality_args
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

    def finish(item: Tuple[int, str]) -> None:
        frame, temp = item
        dest = by_frame[frame]
        parent = os.path.dirname(dest)
        if parent:
            os.makedirs(parent, exist_ok=True)
        if leveled:
            angle = levels.get(frame) if levels else None
            roll, pitch = angle if angle is not None else (0.0, 0.0)
            max360__level_image(
                temp, dest, roll, pitch, width=width, height=height,
                image_format=image_format, quality=quality, interp=interp,
                threads=threads, yaw=yaw)
        else:
            move_file(temp, dest)

    items = list(zip(ordered, temp_files))
    if leveled and workers > 1 and len(items) > 1:
        with ThreadPoolExecutor(max_workers=min(workers, len(items))) as pool:
            list(pool.map(finish, items))
    else:
        for item in items:
            finish(item)



def max360_stitch_ffmpeg(info: max360_VideoInfo, frames: Sequence[int],
                  outputs: Sequence[str], *, width: int, height: int,
                  quality: int, image_format: str, interp: str,
                  threads: int, keep_filter: Optional[str],
                  timeline: Optional[FrameTimeline] = None,
                  levels: Optional[Mapping[int, Tuple[float, float]]] = None,
                  workers: int = 1, yaw: float = 0.0) -> None:
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
    # Leveled frames are grouped into runs that share one rotation (within
    # LEVEL_GROUP_TOLERANCE_DEG), each stitched and leveled in a single v360
    # pass.  The former two-pass path (PNG intermediate + one rotation per
    # frame) remains as the fallback for a run that fails.
    if levels:
        groups = max360_level_groups(ordered_frames, levels,
                                     LEVEL_GROUP_TOLERANCE_DEG,
                                     EXTRACT_CHUNK_SIZE)
        chunks = [run for run, _ in groups]
        rotations: List[Optional[Tuple[float, float]]] = \
            [angle for _, angle in groups]
        LOG.info("  horizon: %d leveled frame(s) in %d single-pass run(s) "
                 "(angles within %.1f deg)", len(ordered_frames), len(chunks),
                 LEVEL_GROUP_TOLERANCE_DEG)
    else:
        chunks = [ordered_frames[i:i + EXTRACT_CHUNK_SIZE]
                  for i in range(0, len(ordered_frames), EXTRACT_CHUNK_SIZE)]
        rotations = [None] * len(chunks)
    # Batches are independent ffmpeg runs, so several can stitch at once.
    # Each one decodes two hemisphere streams, so memory bounds the count.
    parallel = max(1, min(workers, len(chunks), MAX360_PARALLEL_BATCHES))
    # Per-frame leveling inside a batch only gets the workers left over.
    inner_workers = max(1, workers // parallel)
    temp_dir = make_temp_dir("max360_", outputs)

    def run_batch(batch_index: int, chunk: List[int],
                  rotation: Optional[Tuple[float, float]]) -> None:
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
                    total_batches=len(chunks), seek_time=seek_time,
                    levels=levels, workers=inner_workers, yaw=yaw,
                    rotation=rotation)
                return
            except max360_BackendError as exc:
                LOG.warning("seek-based stitching failed for frames "
                            "%d-%d (%s); retrying with a full decode",
                            chunk[0], chunk[-1], exc)
        max360__stitch_batch(
            info, chunk, by_frame, temp_dir, batch_index,
            width=width, height=height, quality=quality,
            image_format=image_format, interp=interp, threads=threads,
            keep_filter=keep_filter, total_batches=len(chunks),
            seek_time=None, levels=levels, workers=inner_workers, yaw=yaw,
            rotation=rotation)

    def do_batch(item: Tuple[int, List[int]]) -> None:
        batch_index, chunk = item
        rotation = rotations[batch_index]
        if rotation is None:
            run_batch(batch_index, chunk, None)
            return
        try:
            run_batch(batch_index, chunk, rotation)
        except max360_BackendError as exc:
            LOG.warning("single-pass leveling failed for frames %d-%d (%s); "
                        "falling back to one rotation pass per frame",
                        chunk[0], chunk[-1], exc)
            # Distinct batch numbers keep the temporary file names apart.
            for sub, start in enumerate(range(0, len(chunk),
                                              MAX360_LEVEL_CHUNK_SIZE)):
                run_batch(1000 * (batch_index + 1) + sub,
                          chunk[start:start + MAX360_LEVEL_CHUNK_SIZE], None)

    try:
        items = list(enumerate(chunks))
        if parallel > 1:
            with ThreadPoolExecutor(max_workers=parallel) as pool:
                list(pool.map(do_batch, items))
        else:
            for item in items:
                do_batch(item)
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def max360_stitch_max2sphere(info: max360_VideoInfo, frames: Sequence[int],
                      outputs: Sequence[str], *, width: int, height: int,
                      quality: int, image_format: str, interp: str,
                      threads: int,
                      levels: Optional[Mapping[int, Tuple[float, float]]] = None,
                      workers: int = 1, yaw: float = 0.0) -> None:
    """Stitch with the optional max2sphere binary, then extract frames.

    max2sphere cannot rotate, so leveled or re-centred frames take one extra
    v360 pass each (via a lossless PNG).
    """
    temp_dir = tempfile.mkdtemp(prefix="max360_m2s_")
    video_out = os.path.join(temp_dir, "equirect.mp4")
    leveled = (levels is not None and any(
        levels.get(int(frame)) is not None for frame in frames)) \
        or abs(yaw) >= 0.05
    try:
        max360__run(max360_max2sphere_command(info.path, video_out, width, height, info.fps))
        timeline = probe_frame_timeline(
            video_out, fps_hint=info.fps, duration_hint=info.duration,
            logger=LOG)
        # Leveled frames go through a lossless PNG intermediate before the
        # v360 sphere rotation, mirroring the ffmpeg backend.
        extracted = extract_frames_by_index(
            video_out, frames, os.path.join(temp_dir, "frames"),
            image_format="png" if leveled else image_format,
            quality=quality, workers=workers, threads=threads,
            timeline=timeline)

        def finish(pair: Tuple[int, str]) -> None:
            frame, dest = pair
            temp = extracted.get(int(frame))
            if temp is None:
                raise max360_BackendError(
                    "max2sphere/ffmpeg did not produce frame %d" % frame)
            parent = os.path.dirname(dest)
            if parent:
                os.makedirs(parent, exist_ok=True)
            if leveled:
                angle = levels.get(int(frame)) if levels else None
                roll, pitch = angle if angle is not None else (0.0, 0.0)
                max360__level_image(
                    temp, dest, roll, pitch, width=width, height=height,
                    image_format=image_format, quality=quality, interp=interp,
                    threads=threads, yaw=yaw)
            else:
                shutil.copyfile(temp, dest)

        pairs = list(zip(frames, outputs))
        if leveled and workers > 1 and len(pairs) > 1:
            with ThreadPoolExecutor(max_workers=min(workers, len(pairs))) as pool:
                list(pool.map(finish, pairs))
        else:
            for pair in pairs:
                finish(pair)
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


# GoPro capture modes whose fixed rate makes the linear GPS-to-video time
# mapping exact.  Anything else (TimeWarp "Auto" speed, odd rates) is mapped
# linearly too, but deserves a warning.
TIMELAPSE_INTERVALS_S = (0.5, 1.0, 2.0, 5.0, 10.0, 30.0, 60.0)
TIMEWARP_FACTORS = (2.0, 5.0, 10.0, 15.0, 30.0)
SLOWMO_FACTORS = (2.0, 4.0, 8.0)
RATE_MATCH_TOLERANCE = 0.03


def log_time_scale(speedup: float, fps: float, real_span: float,
                   video_span: float) -> None:
    """Describe a video whose playback rate differs from real time."""
    def near(value: float, nominal: float) -> bool:
        return abs(value / nominal - 1.0) <= RATE_MATCH_TOLERANCE

    detail = "%.0f s recorded in %.1f s of video" % (real_span, video_span)
    if speedup > 1.0:
        interval = speedup / fps if fps > 0 else 0.0
        if interval and any(near(interval, n) for n in TIMELAPSE_INTERVALS_S):
            LOG.info("  time-lapse: 1 frame every %.2f s of real time "
                     "(%.1fx speed-up, %s)", interval, speedup, detail)
            return
        factor = next((f for f in TIMEWARP_FACTORS if near(speedup, f)), None)
        if factor is not None:
            LOG.info("  TimeWarp %gx (%s)", factor, detail)
            return
    else:
        factor = next((f for f in SLOWMO_FACTORS
                       if near(1.0 / speedup, f)), None)
        if factor is not None:
            LOG.info("  slow motion %gx (%s)", factor, detail)
            return
    LOG.warning("  unusual playback rate %.2fx real time (%s); frame times "
                "are mapped linearly and may drift if the rate varied "
                "(e.g. TimeWarp Auto speed)", speedup, detail)


def max360_build_track(path: str, info: max360_VideoInfo,
                       args) -> Tuple[Optional[Track], Optional[GpmfData]]:
    """Return ``(track, gpmf)`` and ``(None, gpmf)`` when GPS is unusable.

    The caller decides whether an unpositioned time-lapse export is allowed;
    ``gpmf`` still carries the IMU streams, camera clock and detected timezone
    for that fallback.
    """
    if info.gpmd is None:
        raise max360_BackendError("no GPMF (gpmd) telemetry track in file")
    payload = extract_gpmf(path, info.gpmd, ffmpeg=FFMPEG)
    data = parse_gpmf(payload, imu_heading=args.imu_heading)
    LOG.info("  GPMF device: %s; streams: %s", data.device_name,
             ", ".join(sorted(data.streams)))
    if not data.has_gps:
        data.origin_utc = info.creation_time
        _anchor_imu_utc(data.imu, data.origin_utc)
        _anchor_imu_utc(data.exposure, data.origin_utc)
        return None, data

    timed = [s for s in data.gps_samples if s.get("utc") is not None]
    if not timed:
        data.origin_utc = info.creation_time
        _anchor_imu_utc(data.imu, data.origin_utc)
        _anchor_imu_utc(data.exposure, data.origin_utc)
        return None, data
    first_sample = min(timed, key=lambda sample: sample["utc"])
    first_gps = first_sample["utc"]
    last_gps = max(s["utc"] for s in timed)
    gps_span = (last_gps - first_gps).total_seconds()

    # GoPro Max writes creation_time as camera-local wall clock in some
    # firmware, even though the field looks like UTC.  The GPMF GPS clock is
    # authoritative, so use the first fix unless creation_time agrees with it
    # to within five minutes.  The difference is also the camera's UTC offset
    # (plus a short GPS-lock delay), which is what local file names need.
    # STMP keeps counting across the chapters of one recording, so add it back
    # to compare against the recording's start rather than a later chapter's.
    origin = info.creation_time
    skew_s = None
    if origin is not None:
        skew_s = (origin - first_gps).total_seconds()
        stmp_us = first_sample.get("stmp_us")
        if stmp_us is not None and 0 <= stmp_us <= 12 * 3600 * 1000000:
            skew_s += stmp_us / 1e6
    origin_is_gps = False
    if skew_s is None or abs(skew_s) > 300:
        if origin is not None:
            detected = timezone_from_clock_skew(skew_s)
            if detected is not None:
                data.camera_utc_offset = detected
                LOG.info("  using the first GPS fix as the video clock")
            else:
                LOG.warning("  container creation_time is %.0f s from GPS; "
                            "using the first GPS fix as the video clock",
                            (first_gps - origin).total_seconds())
        origin = first_gps
        origin_is_gps = True
    else:
        data.camera_utc_offset = 0

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
            log_time_scale(ratio, info.fps, gps_span, info.duration)

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
    samples.sort(key=lambda s: s["t"])
    max_dop = getattr(args, "max_dop", GPS_MAX_DOP)
    samples, n_dop, n_jump = reject_bad_gps(samples, max_dop)
    log_rejected_gps(n_dop, n_jump, max_dop)
    if not samples:
        # No usable fix (for example an indoor recording).  Keep the clock and
        # IMU so the caller can still export an unpositioned time-lapse.
        data.origin_utc = origin
        data.time_scale = scale
        return None, data
    samples.sort(key=lambda s: s["t"])

    if args.smooth and args.smooth > 1:
        lats = max360__smooth([s["lat"] for s in samples], args.smooth)
        lons = max360__smooth([s["lon"] for s in samples], args.smooth)
        for sample, lat, lon in zip(samples, lats, lons):
            sample["lat"], sample["lon"] = lat, lon

    track = Track(samples, args.distance_mode)
    stale = data.clock_stale_stamps
    LOG.info("  telemetry: %d GPS sample(s), %.1f m of travel, "
             "clock spread %.0f ms%s", len(samples), track.length,
             data.clock_spread_s * 1000,
             " (%d stale GPS time stamp%s ignored)"
             % (stale, "" if stale == 1 else "s") if stale else "")
    data.origin_utc = origin
    data.time_scale = scale
    return track, data


# ---------------------------------------------------------------------------
# Records
# ---------------------------------------------------------------------------
def max360__metadata_for(record, info: max360_VideoInfo, args) -> FrameMetadata:
    # The GPano pose is always level: --level-horizon rotates the pixels
    # themselves, and without it the source tilt is kept as recorded.  The
    # raw angles remain available in the per-image telemetry.
    extra = {}
    if record.get("recentered_deg"):
        extra["recentered_deg"] = "%03d" % int(round(record["recentered_deg"]))
    return FrameMetadata(
        camera=CAMERA_LABELS[info.camera], video=record["video"],
        sequence=record["sequence"], frame_index=record["frame_index"],
        pts=record["pts_seconds"], distance=record.get("distance_m"),
        script_name=SCRIPT_NAME, make=info.make, model=info.model,
        firmware=info.firmware, serial=info.serial,
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
        image_width=info.out_width, image_height=info.out_height,
        panorama=True,
        pose_heading=record.get("heading_deg"),
        pose_pitch=0.0, pose_roll=0.0,
        exposure_time=record.get("exposure_time_s"), iso=record.get("iso"),
        keywords=parse_keywords(args.keywords), extra=extra,
    )


max360__unpositioned_choices = unpositioned_choices


def max360__record(choice, track, args, info, tz_offset, tz_known, sequence,
            video_name, start_utc=None, time_scale=1.0):
    point = choice.point or {}
    pts = max(0.0, float(choice.pts))
    utc = point.get("utc")
    if utc is None:
        origin = start_utc or info.creation_time
        utc = (origin + dt.timedelta(seconds=pts / (time_scale or 1.0))
               if origin is not None else None)
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
        point, args.heading_offset, args.magnetic_declination,
        getattr(args, "recenter_deg", 0.0))
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
        "sequence": sequence, "video": video_name, "camera": info.camera,
        "recentered_deg": getattr(args, "recenter_deg", 0.0) or None,
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
        "position_source": point.get(
            "position_source", "unpositioned" if unpositioned else "none"),
        "clamped": bool(point.get("clamped")),
        "estimated": bool(point.get("estimated")),
        "unpositioned": unpositioned,
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
    log_heading_offset(args)
    info = max360_probe(path)
    info.out_width, info.out_height = max360_output_size(info, args)
    LOG.info("  video: %s, %dx%d %.4f fps, %d frame(s), %.2f s%s -> %dx%d",
             info.model, info.width, info.height, info.fps, info.frame_count,
             info.duration, " (already equirectangular)"
             if info.already_equirect else " (EAC hemispheres)",
             info.out_width, info.out_height)

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

    track = None
    gpmf = None
    try:
        track, gpmf = max360_build_track(path, info, args)
    except (Fatal, GpmfError) as exc:
        if args.fallback_interval <= 0:
            LOG.error("  %s", exc)
            result["failed"] = True
            return result
        LOG.warning("  %s", exc)

    # The MAX2 writes real UTC into creation_time and its own timezone into
    # TZON; the clock-skew estimate is only meaningful for the MAX, which
    # writes its local wall clock there.
    detected_offset = gpmf.camera_utc_offset if gpmf else None
    tz_offset, tz_known = resolve_gopro_timezone(
        args, info.tz_offset_s, detected_offset,
        "container clock vs GPS", track, camera=info.camera, logger=LOG)

    if track is None:
        if args.fallback_interval <= 0:
            LOG.error("  no usable GPS telemetry; use --fallback-interval "
                      "SECONDS to export unpositioned frames")
            result["failed"] = True
            return result
        choices = max360__unpositioned_choices(
            timeline, args.fallback_interval, info.duration, args.max_images)
        LOG.warning("  exporting %d unpositioned frame(s) on a %g s time grid",
                    len(choices), args.fallback_interval)
        skipped = []
    else:
        choices, skipped = select_unique_frames(
            track, timeline, args.distance, start_distance=args.start_distance,
            max_position_error=args.max_position_error,
            max_images=args.max_images, mode=args.distance_mode, logger=LOG)
        if skipped:
            LOG.info("  skipped %d target(s) with no acceptable unused frame",
                     len(skipped))
        if not choices:
            LOG.error("  no capture point could be matched to a unique frame")
            result["failed"] = True
            return result

    fallback_start = gpmf.origin_utc if (track is None and gpmf) else None
    records = [max360__record(
                   c, track, args, info, tz_offset, tz_known, sequence, name,
                   start_utc=fallback_start,
                   time_scale=gpmf.time_scale if gpmf else 1.0)
               for c in choices]
    attach_exposure(records, gpmf.exposure if gpmf else None)
    level_on = bool(getattr(args, "level_horizon", False))
    if level_on:
        window_s = float(getattr(args, "level_window", LEVEL_WINDOW_DEFAULT))
        gradient_mount = None
        if getattr(args, "horizon_gradient", False) and track is not None:
            # The camera's own frame: the mount angle is the heading offset,
            # whether it is re-centred afterwards or not.
            gradient_mount = (getattr(args, "recenter_deg", 0.0)
                              or getattr(args, "heading_offset", 0.0) or 0.0)
        angles, leveled = _image_angles(
            records, gpmf.imu if gpmf else None, window_s=window_s,
            track=track, gradient_mount_deg=gradient_mount)
        for record in records:
            angle = angles.get(record["frame_index"])
            if angle is not None:
                record["roll_deg"], record["pitch_deg"] = angle
            level = leveled.get(record["frame_index"])
            if level is not None:
                record["level_roll_deg"], record["level_pitch_deg"] = level
                record["level_window_s"] = window_s
        LOG.info("  horizon: leveling %d/%d image(s) (%s, at most %.1f deg "
                 "change between images%s)", len(leveled), len(records),
                 "Gaussian average over a %g s real-time window" % window_s
                 if window_s > 0 else "nearest sample, no smoothing",
                 LEVEL_MAX_STEP_DEG,
                 "; road gradient from GPS altitude followed (mount %03d)"
                 % int(round(gradient_mount)) if gradient_mount is not None
                 else "")
    first_time = records[0]["naming_time"]
    sequence_dir = sequence_output_dir(
        args.output, info.camera, sequence, first_time,
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
            position = ("unpositioned" if record.get("unpositioned")
                        else "%.6f,%.6f" % (record["latitude"],
                                            record["longitude"]))
            distance = ("-" if record.get("distance_m") is None
                        else "%.1fm" % record["distance_m"])
            LOG.info("    frame=%d pts=%.3fs %sZ %s hdg=%s d=%s -> %s",
                     record["frame_index"], record["pts_seconds"],
                     (record["datetime_utc"] or "no-utc")[11:23], position,
                     record["heading_deg"], distance, record["image"])
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
    yaw = recenter_yaw(getattr(args, "recenter_deg", 0.0))
    levels = None
    if level_on:
        levels = {}
        for record in to_extract:
            roll = record.get("level_roll_deg")
            pitch = record.get("level_pitch_deg")
            if roll is not None and pitch is not None:
                levels[record["frame_index"]] = (float(roll), float(pitch))
    if to_extract:
        outputs = [record["path"] for record in to_extract]
        frames = [record["frame_index"] for record in to_extract]
        keep_filter = (os.path.join(sequence_dir, sequence + ".filter")
                       if args.keep_filter else None)
        if backend == "max2sphere":
            max360_stitch_max2sphere(info, frames, outputs,
                              width=info.out_width,
                              height=info.out_height, quality=args.quality,
                              image_format=args.format, interp=args.interp,
                              threads=args.threads, levels=levels,
                              workers=args.workers, yaw=yaw)
        else:
            max360_stitch_ffmpeg(info, frames, outputs, width=info.out_width,
                          height=info.out_height, quality=args.quality,
                          image_format=args.format, interp=args.interp,
                          threads=args.threads, keep_filter=keep_filter,
                          timeline=timeline, levels=levels,
                          workers=args.workers, yaw=yaw)
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
        sequence_dir, sequence, track.samples if track else [], records,
        track_export=args.track_export, camera=info.camera, video=name,
        imu=gpmf.imu if gpmf else None,
        video_start_utc=(gpmf.origin_utc if gpmf else fallback_start),
        telemetry_export=args.telemetry_export,
        time_scale=(gpmf.time_scale if gpmf else 1.0),
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
                "decoders": virb__field_decoders(fields, endian),
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


@lru_cache(maxsize=4096)
def _virb_field_decoder(fnum, size, base, endian):
    kind, fmt, bsize = virb_BASE_TYPES.get(base, ("byte", None, 1))
    n = size // bsize
    unpacker = None
    if fmt is not None:
        if n * bsize != size:
            # Malformed field size: fail at decode time like struct.unpack.
            unpacker = struct.Struct(endian + "%dB" % size)
            kind = "malformed"
        else:
            unpacker = struct.Struct(endian + str(n) + fmt)
    return (fnum, size, kind, unpacker, n, virb_INVALID.get(kind))


def virb__field_decoders(fields, endian):
    """Precompiled ``(fnum, size, kind, Struct, count, invalid)`` per field."""
    return tuple(_virb_field_decoder(fnum, size, base, endian)
                 for fnum, size, base in fields)


def virb__read_fields(data, pos, definition, endian, compressed, end):
    p = pos
    out = {}
    decoders = definition.get("decoders")
    if decoders is None:
        decoders = virb__field_decoders(definition["fields"], endian)
    for fnum, size, kind, unpacker, n, bad in decoders:
        if p + size > end:
            break                           # truncated payload
        if compressed and fnum == 254:
            continue                        # time_offset lives in the header
        if unpacker is None:
            chunk = data[p:p + size]
            p += size
            if kind == "string":
                out[fnum] = chunk.split(b"\x00")[0].decode("utf-8", "replace")
            else:
                out[fnum] = chunk
        elif kind == "malformed":
            raise struct.error("FIT field %d: size %d is not a multiple of "
                               "its base type" % (fnum, size))
        else:
            values = unpacker.unpack_from(data, p)
            p += size
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
            converted = []
            for row in rows:
                values = [((value - shift) / divisor) * factor
                          for value in row["raw"]]
                converted.append((row, values))

            # The accelerometer's up/down sign is a device property, not a
            # user setting.  The camera's own orientation events tell us how
            # the stored image is rotated, so choose the sign that puts the
            # image-frame up vector above the horizon for most samples.
            sign = 1.0
            bodies: List[Tuple[List[float], Optional[float], List[float]]] = []
            if key == "ACCL":
                # Matrix, orientation and the unsigned up vector are computed
                # once per sample and reused for the sign vote and the angles.
                matrix = _fit_matrix(calibration)
                score = 0.0
                for row, values in converted:
                    body = _apply_fit_matrix(matrix, values)
                    orientation = self.orientation_at(row["fit_s"])
                    up = _virb_image_up(body, orientation)
                    bodies.append((body, orientation, up))
                    score += up[2]
                if score < 0:
                    sign = -1.0

            points = []
            for index, (row, values) in enumerate(converted):
                point = {
                    "fit_s": row["fit_s"],
                    "utc": self.utc_of(row["fit_s"]),
                    "raw": list(row["raw"]),
                    "values": values,
                }
                if key == "ACCL":
                    body, orientation, up = bodies[index]
                    if sign != 1.0:
                        up = _virb_image_up(
                            [sign * component for component in body],
                            orientation)
                    # FIT body Y points forward; the shared helper expects
                    # (right, back, up), so the Y component flips.
                    roll, pitch = _roll_pitch_from_body(
                        up[0], -up[1], up[2])
                    if roll is not None and pitch is not None:
                        point["roll_deg"] = round(roll, 3)
                        point["pitch_deg"] = round(pitch, 3)
                points.append(point)
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

    def _sorted_events(self):
        """``camera_events`` sorted, cached until the list changes."""
        events = getattr(self, "camera_events", ())
        cache = getattr(self, "_events_cache", None)
        if cache is None or cache[0] is not events or cache[1] != len(events):
            cache = (events, len(events), sorted(events))
            self._events_cache = cache
        return cache[2]

    def camera_orientation(self):
        for _t, _name, orient in self._sorted_events():
            if orient is not None:
                return orient
        return None

    def orientation_at(self, fit_seconds):
        """Camera orientation (degrees) of the segment covering *fit_seconds*.

        The orientation is recorded on the video-start events; the latest one
        at or before the sample applies.  Falls back to the first known value.
        """
        current = self.camera_orientation()
        for when, name, orient in self._sorted_events():
            if orient is None or when > fit_seconds:
                continue
            if name in virb_VIDEO_START_EVENTS or name == "video_second_stream_start":
                current = orient
        return current

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
    info = flat_video_info(path)
    info["size"] = os.path.getsize(path)
    return info


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
        point, args.heading_offset, args.magnetic_declination,
        getattr(args, "recenter_deg", 0.0))
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


virb__unpositioned_choices = unpositioned_choices


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
        firmware=source.get("software"), serial=source.get("serial"),
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
        extra={"camera_orientation_deg": source.get("orientation_deg")},
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
    log_heading_offset(args)

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

    if args.level_horizon:
        LOG.info("  --level-horizon applies to 360 videos only; flat %s "
                 "frames are exported as recorded", CAMERA_LABELS[CAMERA_VIRB])

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
    if to_extract:
        temp_dir = make_temp_dir("virb30_", [r["path"] for r in to_extract])
        try:
            extracted = extract_frames_by_index(
                video, [r["frame_index"] for r in to_extract], temp_dir,
                image_format=args.format, quality=args.quality,
                workers=args.workers, threads=args.threads, timeline=timeline)
            written, failed_frames = place_extracted(extracted, to_extract)
            written_paths = [record["path"] for record in written]
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
    tz_offset_s: Optional[int] = None   # camera TZON (HERO11 and later)


hero__parse_rate = _parse_rate


def hero_probe(path: str) -> hero_VideoInfo:
    meta = probe_metadata(path, FFPROBE)
    videos = meta.video_streams_info()
    if len(videos) != 1:
        raise Fatal("expected one flat video stream, found %d (GoPro Max EAC "
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
    settings = read_gopro_settings(path)
    return hero_VideoInfo(
        path=path, duration=duration, fps=fps, frame_count=frame_count,
        gpmd=meta.gpmd_index, width=width, height=height,
        creation_time=parse_utc_timestamp(meta.creation_time),
        firmware=meta.format_tags.get("firmware") or settings.firmware,
        serial=(meta.format_tags.get("serial_number")
                or meta.format_tags.get("serial") or settings.serial),
        tz_offset_s=settings.tz_offset_s,
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
        _anchor_imu_utc(data.exposure, data.origin_utc)
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
    samples.sort(key=lambda s: s["t"])
    max_dop = getattr(args, "max_dop", GPS_MAX_DOP)
    samples, n_dop, n_jump = reject_bad_gps(samples, max_dop)
    log_rejected_gps(n_dop, n_jump, max_dop)
    if not samples:
        return None, data
    samples.sort(key=lambda sample: sample["t"])
    track = Track(samples, args.distance_mode)
    LOG.info("  telemetry: %d GPS sample(s), %.1f m of travel",
             len(samples), track.length)
    data.origin_utc = origin
    data.time_scale = 1.0
    return track, data


hero__unpositioned_choices = unpositioned_choices


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
        point, args.heading_offset, args.magnetic_declination,
        getattr(args, "recenter_deg", 0.0))
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
        firmware=info.firmware, serial=info.serial,
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
        exposure_time=record.get("exposure_time_s"), iso=record.get("iso"),
        keywords=parse_keywords(args.keywords),
        extra={"unpositioned": record.get("unpositioned")},
    )


def hero_process_video(path: str, sequence: str, args) -> dict:
    name = os.path.basename(path)
    result = {"video": name, "sequence": sequence, "images": 0,
              "planned": 0, "failed": False, "entries": [], "dir": None}
    LOG.info("--- %s -> %s", name, sequence)
    log_heading_offset(args)
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

    tz_offset, tz_known = resolve_gopro_timezone(
        args, info.tz_offset_s, None, "", track, camera=CAMERA_HERO,
        logger=LOG)

    if track is None:
        if args.fallback_interval <= 0:
            LOG.error("  no GPS telemetry; use --fallback-interval SECONDS "
                      "to export unpositioned frames")
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
    attach_exposure(records, gpmf.exposure if gpmf else None)

    if args.level_horizon:
        LOG.info("  --level-horizon applies to 360 videos only; flat %s "
                 "frames are exported as recorded", CAMERA_LABELS[CAMERA_HERO])

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
    if to_extract:
        temp_dir = make_temp_dir("hero_", [r["path"] for r in to_extract])
        try:
            extracted = extract_frames_by_index(
                path, [record["frame_index"] for record in to_extract],
                temp_dir, image_format=args.format, quality=args.quality,
                workers=args.workers, threads=args.threads, timeline=timeline)
            written, failed_frames = place_extracted(extracted, to_extract)
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
CAMERA_ORDER = (CAMERA_BLACKVUE, CAMERA_MAX, CAMERA_MAX2, CAMERA_VIRB,
                CAMERA_HERO)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=SCRIPT_NAME,
        description="Export geotagged still images from supported action-camera "
                    "videos every N metres of travel. The camera is detected "
                    "per file (Garmin VIRB Ultra 30, GoPro Max, GoPro Max2, BlackVue "
                    "DR900 or GoPro HERO), so one input tree may mix cameras.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=("Examples:\n"
                "  %(prog)s -i video/ -o images/ -d 7\n"
                "  %(prog)s -i video/2026-10-18-virb-4k -o images/ -d 8\n"
                "  %(prog)s -i video/hero -o images/ "
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
                      help="deprecated alias; unpositioned export is enabled by "
                           "--fallback-interval SECONDS")

    geometry = parser.add_argument_group("GoPro Max / Max2 (360) output geometry")
    geometry.add_argument("--width", type=int, default=None,
                          help="360 output image width (default: the "
                               "camera's native size, 5376 for the Max, "
                               "7680 for the Max2; 2:1 with --height)")
    geometry.add_argument("--height", type=int, default=None,
                          help="360 output image height (default: width / 2)")
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
                        help="derive a compass heading from the MAGN + GRAV "
                             "streams (HERO/MAX) as a fallback for recordings "
                             "where GPS gives no course; the GPS course "
                             "always wins because GoPro's magnetometer is "
                             "uncalibrated and unreliable")

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
    for value in (args.width, args.height):
        if value is not None and value < 2:
            parser.error("--width/--height must be at least 2")
        if value is not None and value % 2:
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
    for video, detection in zip(videos, detect_all(videos)):
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
    base_args = args
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
            args = video_args(base_args, video, camera)
            if camera == CAMERA_BLACKVUE:
                summary = blackvue_process_video(video, sequence, output_dir,
                                                 args, have_exiftool)
            elif camera in CAMERA_360:
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


def reject_old_level_horizon(parser: argparse.ArgumentParser,
                             argv: Sequence[str]) -> None:
    """Explain the old ``--level-horizon rotate|exif|no`` syntax."""
    for i, arg in enumerate(argv):
        name, _, value = arg.partition("=")
        if name not in ("--level-horizon", "--horizon"):
            continue
        if not value and i + 1 < len(argv):
            value = argv[i + 1] if argv[i + 1] in LEVEL_HORIZON_OLD_VALUES \
                else ""
        if value:
            parser.error("%s no longer takes a value (got %r): give "
                         "--level-horizon to level the horizon (the former "
                         "'rotate'), leave it out to keep the source tilt; "
                         "'exif' was removed" % (name, value))


def check_videos(videos: Sequence[str], args: argparse.Namespace) -> bool:
    """Validate folder heading offsets and warn about duplicate file names."""
    ok = True
    if getattr(args, "heading_offset_auto", False):
        for folder in sorted({os.path.dirname(os.path.abspath(v))
                              for v in videos}):
            try:
                offset = folder_heading_offset(os.path.basename(folder))
            except ValueError as exc:
                LOG.error("--heading-offset auto: %s", exc)
                ok = False
                continue
            LOG.info("heading offset %s: %s",
                     "%03d" % int(offset) if offset is not None else "none",
                     folder)
    by_name: Dict[str, List[str]] = {}
    for video in videos:
        by_name.setdefault(os.path.basename(video).lower(), []).append(video)
    for name, paths in sorted(by_name.items()):
        folders = sorted({os.path.dirname(os.path.abspath(p)) for p in paths})
        if len(folders) > 1:
            LOG.warning("video file name %s occurs in %d folders (%s); their "
                        "images may share an output folder, where an existing "
                        "image is skipped rather than overwritten",
                        os.path.basename(paths[0]), len(folders),
                        ", ".join(folders))
    return ok


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    reject_old_level_horizon(parser,
                             sys.argv[1:] if argv is None else list(argv))
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

    if not check_videos(videos, args):
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
