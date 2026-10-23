#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""virb30-v2i.py -- export geotagged still images from Garmin VIRB
Ultra 30 action-camera videos at a fixed travelled-distance interval.

The camera records plain MP4 video plus a companion Garmin FIT ("G-Metrix")
telemetry file that holds the GPS track, speed, heading and camera events.  This
tool walks that GPS track, and every `--distance` metres of travel it asks
ffmpeg for the video frame captured at the interpolated moment and writes it as
a still image with full EXIF/XMP geo metadata.

INPUT
-----
`--input` is a root directory holding one or more video folders::

    video/
    |-- 2026-10-05/
    |   |-- VIRB0996.MP4
    |   `-- GMetrix/
    |       `-- 2026-10-05-07-53-07.fit
    `-- 2026-10-07/
        |-- VIRB0772.MP4
        `-- GMetrix/
            `-- 2026-10-07-11-08-28.fit

Videos are discovered recursively using `--video-glob` (default ``*.MP4``).  For
each video the companion FIT files are looked up in a sibling directory named by
`--fit-directory-name` (default ``gmetrix``, matched case-insensitively, so the
camera's ``GMetrix`` folder is found).  The MP4 itself names the exact FIT file
it belongs to, which is the primary matching key; timestamps are used as a
fallback.

OUTPUT
------
Images are written below `--output` as::

    images/
    `-- YYYY-MM-DD/                 <- capture date
        `-- VIRB0996/               <- sequence, derived from the video name
            |-- 2026-10-05_09-59-30-191_000001.jpg
            |-- 2026-10-05_09-59-41-021_000002.jpg
            `-- ...

  * The file name is ``YYYY-MM-DD_HH-MM-SS-mmm_NNNNNN.ext`` with zero padded
    fields and a per-sequence 6-digit counter.
  * The timestamp is camera-*local* wall-clock time by default; pass
    ``--time-base utc`` to use UTC instead.
  * Existing files are never overwritten unless ``--overwrite`` is given.
  * ``--sidecars`` additionally writes a ``<sequence>.track.jsonl`` manifest.

SEQUENCE NAMING / COLLISIONS
----------------------------
The sequence directory is the video file name without its extension, e.g.
``VIRB0996``.  If the same base name occurs in more than one video folder (or in
a sub-folder of the input tree), the relative folder path is prepended with
``__`` separators (``subdir__VIRB0996``) so images from different source folders
can never collide.

DISTANCE SAMPLING
-----------------
For every video sequence the corresponding FIT track is turned into a
cumulative-distance curve and images are taken at target distances
``0, d, 2d, 3d, ...`` (``d`` = ``--distance``).  The GPS position, speed,
heading and time of each target are linearly interpolated between the two
bracketing GPS samples, so sampling is by distance travelled, not by frame
number, and never drifts.

DEPENDENCIES
------------
Python standard library only.  The external programs ``ffmpeg`` and ``ffprobe``
must be on PATH; ``exiftool`` is required to write EXIF metadata (if it is
missing a warning is printed and images are written without metadata).
"""

import argparse
import datetime as dt
import json
import logging
import math
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile

import v2i_common as v2i
from v2i_common import Track, detect_camera, plan_shots, skip_reason

SCRIPT_NAME = os.path.basename(os.path.realpath(__file__))
LOG = logging.getLogger("virb30-v2i")

FIT_EPOCH = dt.datetime(1989, 12, 31, tzinfo=dt.timezone.utc)
SEMICIRCLE = 180.0 / (2 ** 31)

FIT_EXT = ".fit"
DEFAULT_VIDEO_GLOB = "*.MP4"
DEFAULT_FIT_DIR = "gmetrix"
DEFAULT_DISTANCE = 5.0
DEFAULT_QUALITY = 95


class Fatal(Exception):
    """A problem that stops the current file (or the whole run)."""


def run(cmd):
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

BASE_TYPES = {
    0x00: ("enum", "B", 1), 0x01: ("sint8", "b", 1), 0x02: ("uint8", "B", 1),
    0x83: ("sint16", "h", 2), 0x84: ("uint16", "H", 2), 0x85: ("sint32", "i", 4),
    0x86: ("uint32", "I", 4), 0x07: ("string", None, 1), 0x88: ("float32", "f", 4),
    0x89: ("float64", "d", 8), 0x0A: ("uint8z", "B", 1), 0x8B: ("uint16z", "H", 2),
    0x8C: ("uint32z", "I", 4), 0x0D: ("byte", None, 1), 0x8E: ("sint64", "q", 8),
    0x8F: ("uint64", "Q", 8), 0x90: ("uint64z", "Q", 8),
}

INVALID = {
    "uint8": 0xFF, "uint8z": 0x00, "uint16": 0xFFFF, "uint16z": 0x0000,
    "uint32": 0xFFFFFFFF, "uint32z": 0x00000000, "uint64": 0xFFFFFFFFFFFFFFFF,
    "uint64z": 0, "sint8": 0x7F, "sint16": 0x7FFF, "sint32": 0x7FFFFFFF,
    "sint64": 0x7FFFFFFFFFFFFFFF,
}

MSG_FILE_ID = 0
MSG_RECORD = 20
MSG_DEVICE_INFO = 23
MSG_GPS_METADATA = 160
MSG_CAMERA_EVENT = 161
MSG_GYROSCOPE = 164
MSG_ACCELEROMETER = 165
MSG_MAGNETOMETER = 208

CAMERA_EVENT_TYPES = {
    0: "video_start", 1: "video_split", 2: "video_end", 3: "photo_taken",
    4: "video_second_stream_start", 5: "video_second_stream_split",
    6: "video_second_stream_end", 7: "video_split_start",
    8: "video_second_stream_split_start", 11: "video_pause",
    12: "video_second_stream_pause", 13: "video_resume",
    14: "video_second_stream_resume",
}

CAMERA_ORIENTATION = {0: 0, 1: 90, 2: 180, 3: 270}

# Camera events that begin a recorded video segment and that end one.
VIDEO_START_EVENTS = ("video_start", "video_split_start", "video_split")
VIDEO_END_EVENTS = ("video_end", "video_second_stream_end")


class FitMessage(object):
    __slots__ = ("num", "fields")

    def __init__(self, num, fields):
        self.num = num
        self.fields = fields

    def get(self, num):
        return self.fields.get(num)


def parse_fit(path):
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
            out.append(FitMessage(
                definition["num"],
                _read_fields(data, pos, definition, definition["endian"], True, end)))
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
            out.append(FitMessage(
                definition["num"],
                _read_fields(data, pos, definition, definition["endian"], False, end)))
            pos += definition["size"]

    return out


def _read_fields(data, pos, definition, endian, compressed, end):
    p = pos
    out = {}
    for fnum, size, base in definition["fields"]:
        if p + size > end:
            break                           # truncated payload
        if compressed and fnum == 254:
            continue                        # time_offset lives in the header
        kind, fmt, bsize = BASE_TYPES.get(base, ("byte", None, 1))
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
            bad = INVALID.get(kind)
            if bad is not None:
                values = tuple(None if v == bad else v for v in values)
            out[fnum] = values[0] if n == 1 else values
    return out


# ===========================================================================
# High level telemetry
# ===========================================================================

def _num(value, default=None):
    if value is None:
        return default
    if isinstance(value, tuple):
        return value[0] if value else default
    return value


def parse_fit_name_local(name):
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


class Telemetry(object):
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
        self._load()

    def _load(self):
        for msg in parse_fit(self.path):
            num = msg.num
            if num == MSG_GPS_METADATA:
                self._add_gps(msg.fields)
            elif num == MSG_CAMERA_EVENT:
                self._add_camera_event(msg.fields)
            elif num == MSG_DEVICE_INFO:
                self._add_device(msg.fields)
            elif num == MSG_FILE_ID:
                self._add_file_id(msg.fields)
            elif num == MSG_RECORD:
                fit_s = _num(msg.get(253))
                distance = msg.get(5)
                if fit_s is None or not isinstance(distance, (int, float)):
                    continue
                self.records.append({"t": fit_s, "distance": distance / 100.0})
                self.track_distance = distance / 100.0
            elif num in (MSG_ACCELEROMETER, MSG_GYROSCOPE, MSG_MAGNETOMETER):
                self.imu_available = True

        self.samples.sort(key=lambda s: s["t"])
        self._dedupe()
        self.records.sort(key=lambda r: r["t"])
        for sample in self.samples:
            if sample["utc_fit"] is not None:
                self.utc_offset_s = sample["utc_fit"] - sample["fit_s"]
                break

    def _add_gps(self, fields):
        fit_s = _num(fields.get(253))
        if fit_s is None:
            return
        lat_raw = _num(fields.get(1))
        lon_raw = _num(fields.get(2))
        if lat_raw is None or lon_raw is None:
            return
        lat = lat_raw * SEMICIRCLE
        lon = lon_raw * SEMICIRCLE
        if abs(lat) > 90 or abs(lon) > 180:
            return
        ms = _num(fields.get(0), 0) or 0
        alt_raw = _num(fields.get(3))
        spd_raw = _num(fields.get(4))
        hdg_raw = _num(fields.get(5))
        utc_raw = _num(fields.get(6))
        vel = fields.get(7)
        self.samples.append({
            "fit_s": fit_s,
            "t": fit_s + ms / 1000.0,
            "utc_fit": utc_raw,
            "lat": lat,
            "lon": lon,
            # Garmin encodes enhanced_altitude as raw/5 - 500 metres.
            "alt": (alt_raw / 5.0 - 500.0) if alt_raw is not None else None,
            "speed": (spd_raw / 1000.0) if spd_raw is not None else None,
            "heading": (hdg_raw / 100.0) if hdg_raw is not None else None,
            "velocity": [v / 100.0 for v in vel] if isinstance(vel, tuple) else None,
        })

    def _add_camera_event(self, fields):
        fit_s = _num(fields.get(253))
        if fit_s is None:
            return
        ms = _num(fields.get(0), 0) or 0
        etype = _num(fields.get(1))
        orient = CAMERA_ORIENTATION.get(_num(fields.get(3)))
        self.camera_events.append((
            fit_s + ms / 1000.0,
            CAMERA_EVENT_TYPES.get(etype, "type_%s" % etype),
            orient))

    def _add_device(self, fields):
        if _num(fields.get(0), 0) not in (0, None):   # only the creator device
            return
        self.device = {
            "manufacturer": _num(fields.get(2)),
            "serial": _num(fields.get(3)),
            "product": _num(fields.get(4)),
            "software_raw": _num(fields.get(5)),
        }

    def _add_file_id(self, fields):
        created = _num(fields.get(4))
        if created is not None:
            self.time_created_utc = FIT_EPOCH + dt.timedelta(seconds=created)

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
            return FIT_EPOCH + dt.timedelta(seconds=fit_seconds + self.utc_offset_s)
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
        local = parse_fit_name_local(self.path)
        if local and self.samples:
            return local + dt.timedelta(seconds=fit_seconds - self.samples[0]["fit_s"])
        return None

    def local_utc_offset(self):
        """Camera-local wall clock minus UTC in seconds, or None.

        The GMetrix file name holds the local time at which the camera created
        the file while file_id.time_created is real UTC, so their difference is
        the camera's UTC offset.  Snapped to a quarter hour.
        """
        local = parse_fit_name_local(self.path)
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
            if name in VIDEO_START_EVENTS:
                if not starts or t - starts[-1] > 0.5:
                    starts.append(t)
        ends = [t for t, name, _o in events if name in VIDEO_END_EVENTS]
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

def read_mp4_atoms(path):
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


def parse_creation_time(text):
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


def probe_video(path):
    """Return fps, frame count, duration, dimensions, codec and creation time."""
    out = run(["ffprobe", "-v", "error", "-print_format", "json",
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

def walk_files(root, exclude_dirs=()):
    """Yield (directory, sorted_file_names), skipping excluded absolute dirs."""
    exclude = {os.path.abspath(d) for d in exclude_dirs}
    for current, dirs, files in os.walk(root):
        dirs[:] = sorted(d for d in dirs
                         if os.path.abspath(os.path.join(current, d)) not in exclude)
        yield current, sorted(files)


def find_fits(input_dir, exclude_dirs=()):
    out = []
    for current, files in walk_files(input_dir, exclude_dirs):
        for name in files:
            if name.lower().endswith(FIT_EXT):
                out.append(os.path.join(current, name))
    return out


def fits_for_video(video, all_fits, fit_dir_name):
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
        for current, files in walk_files(directory):
            for name in files:
                if name.lower().endswith(FIT_EXT):
                    local.append(os.path.join(current, name))

    direct = [f for f in all_fits
              if os.path.dirname(os.path.abspath(f)) == video_dir]

    candidates = sorted(set(local + direct))
    return candidates if candidates else sorted(all_fits)


def pick_fit(atoms, candidates, created_local, explicit=None):
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
            fit_local = parse_fit_name_local(fit)
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
# Metadata (EXIF / XMP) writing
# ===========================================================================

def build_metadata_args(record, source, args):
    """exiftool arguments (without the file name) describing one image."""
    tags = ["-overwrite_original", "-n", "-charset", "filename=utf8", "-Orientation=1"]

    local = record.get("captured_local")
    if local is not None:
        stamped = v2i.exif_datetime(local)
        tags += ["-DateTimeOriginal=" + stamped,
                 "-CreateDate=" + stamped,
                 "-ModifyDate=" + stamped,
                 "-SubSecTimeOriginal=%03d" % (local.microsecond // 1000)]
    if record.get("utc_offset_s") is not None:
        tags.append("-OffsetTimeOriginal=" + v2i.offset_string(record["utc_offset_s"]))
        tags.append("-OffsetTime=" + v2i.offset_string(record["utc_offset_s"]))

    if record.get("lat") is not None and record.get("lon") is not None:
        tags += ["-GPSLatitude=%r" % record["lat"],
                 "-GPSLongitude=%r" % record["lon"]]
        if record.get("alt") is not None:
            tags += ["-GPSAltitude=%r" % record["alt"],
                     "-GPSAltitudeRef=%d" % (0 if record["alt"] >= 0 else 1)]
        if record.get("speed") is not None:
            tags += ["-GPSSpeed=%r" % (record["speed"] * 3.6), "-GPSSpeedRef=K"]
        # The image direction includes --offset (camera optical axis vs. course);
        # GPSTrack stays the raw course over ground.
        if record.get("img_direction") is not None:
            tags += ["-GPSImgDirection=%r" % record["img_direction"],
                     "-GPSImgDirectionRef=T"]
        if record.get("course") is not None:
            tags += ["-GPSTrack=%r" % record["course"], "-GPSTrackRef=T"]

    tags += ["-Make=Garmin", "-Model=VIRB Ultra 30",
             "-Software=%s" % source["software"]]
    if source.get("serial"):
        tags += ["-SerialNumber=%s" % source["serial"],
                 "-CameraSerialNumber=%s" % source["serial"]]

    orient_text = ""
    if source.get("orientation_deg") is not None:
        orient_text = " | mount_orientation=%ddeg" % source["orientation_deg"]
    fit_name = source.get("fit") or "none"
    if source.get("fit_ref") and source["fit_ref"] != source.get("fit"):
        fit_name += " (video references %s)" % source["fit_ref"]

    comment = ("Garmin VIRB Ultra 30 via %s | video=%s | frame=%d | pts=%.3fs | "
               "distance=%.2fm | heading_offset=%+gdeg | fit=%s%s"
               % (SCRIPT_NAME, source["video"], record["frame"], record["pts"],
                  record.get("distance") or 0.0, args.heading_offset, fit_name,
                  orient_text))
    tags += ["-ImageDescription=" + comment,
             "-UserComment=" + comment,
             "-XPComment=" + comment,
             "-XMP:CreatorTool=" + SCRIPT_NAME,
             "-XMP:Title=%s %s" % (source["video"],
                                   local.strftime("%Y-%m-%d %H:%M:%S") if local else "")]

    extra = {
        "source_video": source["video"],
        "fit_file": source.get("fit"),
        "frame_index": record["frame"],
        "pts_seconds": round(record["pts"], 3),
        "distance_m": record.get("distance"),
        "heading_offset_deg": args.heading_offset,
        "camera_orientation_deg": source.get("orientation_deg"),
        "grade_deg": record.get("grade"),
        "utc": record["captured_utc"].isoformat() if record.get("captured_utc") else None,
    }
    tags += ["-XMP:Subject=%s" % json.dumps(extra, sort_keys=True).replace('"', "'"),
             "-Keywords=virb;ultra30;garmin;streetview;mapillary;panoramax"]
    return tags


def write_sidecar(path, sequence, video_name, records, source, telemetry, args):
    """Write a JSONL manifest describing every planned image of a sequence."""
    meta = {
        "generator": SCRIPT_NAME,
        "sequence": sequence,
        "source_video": video_name,
        "fit_file": source.get("fit"),
        "camera": {
            "make": "Garmin", "model": "VIRB Ultra 30",
            "serial": source.get("serial"),
            "software": source["software"],
            "orientation_deg": source.get("orientation_deg"),
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
            velocity = record.get("velocity") or [None, None, None]
            fh.write(json.dumps({
                "image": record["filename"],
                "written": record.get("written", False),
                "pts_seconds": round(record["pts"], 3),
                "frame_index": record["frame"],
                "datetime_local": (record["captured_local"].isoformat()
                                   if record.get("captured_local") else None),
                "datetime_utc": (record["captured_utc"].isoformat()
                                 if record.get("captured_utc") else None),
                "latitude": record.get("lat"),
                "longitude": record.get("lon"),
                "altitude_m": record.get("alt"),
                "speed_m_s": record.get("speed"),
                "course_deg": record.get("course"),
                "image_direction_deg": record.get("img_direction"),
                "grade_deg": record.get("grade"),
                "distance_m": record.get("distance"),
                "velocity_east_m_s": velocity[0],
                "velocity_north_m_s": velocity[1],
                "velocity_up_m_s": velocity[2],
            }) + "\n")


# ===========================================================================
# Per-video processing
# ===========================================================================

def resolve_utc_offset(args, telemetry):
    """Camera UTC offset in seconds: explicit --timezone wins, else detected."""
    offset = v2i.parse_timezone(getattr(args, "timezone", None))
    if offset is not None:
        return offset
    if telemetry is not None:
        return telemetry.local_utc_offset()
    return None


def choose_segment(telemetry, created_local, duration, tz_offset):
    """Pick the video segment this MP4 corresponds to.

    Returns (start_fit, end_fit) or (None, None).
    """
    segments = telemetry.segments()
    if segments:
        if created_local is not None:
            best = None
            for segment in segments:
                local = telemetry.local_time_of(segment["start"], tz_offset)
                if local is None:
                    continue
                delta = abs((local - created_local).total_seconds())
                if best is None or delta < best[0]:
                    best = (delta, segment)
            if best is not None and best[0] < 43200.0:
                return best[1]["start"], best[1]["end"]
        return segments[0]["start"], segments[0]["end"]

    # No camera events: align the video start to the nearest GPS sample.
    samples = telemetry.samples
    if not samples:
        return None, None
    if created_local is not None and tz_offset is not None:
        target_utc = created_local.replace(tzinfo=dt.timezone.utc) - \
            dt.timedelta(seconds=tz_offset)
        best = None
        for sample in samples:
            utc = telemetry.utc_of(sample["fit_s"])
            if utc is None:
                continue
            delta = abs((utc - target_utc).total_seconds())
            if best is None or delta < best[0]:
                best = (delta, sample)
        if best is not None:
            start = best[1]["fit_s"]
            end = start + duration if duration else samples[-1]["t"]
            return start, end
    return samples[0]["fit_s"], samples[-1]["t"]


def build_records(shots, info, args, video_start_fit, video_start_local,
                  video_start_utc, tz_offset):
    """Turn planned shots into capture records with names and times."""
    fps = info["fps"] or 0.0
    records = []
    for counter, shot in enumerate(shots, start=1):
        if shot.get("t") is not None and video_start_fit is not None:
            pts = shot["t"] - video_start_fit
        else:
            pts = shot.get("pts") or 0.0
        pts = max(0.0, pts)
        if fps:
            frame = max(0, int(round(pts * fps)))
            if info.get("frames"):
                # The last shot can land exactly on the video end; frame
                # indices are zero-based, so never ask for the frame after it.
                frame = min(frame, info["frames"] - 1)
            pts = frame / fps
        else:
            frame = 0

        captured_local = None
        if video_start_local is not None:
            captured_local = video_start_local + dt.timedelta(seconds=pts)
        captured_utc = None
        if video_start_utc is not None:
            captured_utc = (video_start_utc + dt.timedelta(seconds=pts)).replace(tzinfo=None)

        if args.time_base == "utc" and captured_utc is not None:
            naming = captured_utc
        else:
            naming = captured_local if captured_local is not None else captured_utc
        if naming is None:
            naming = dt.datetime(1970, 1, 1)

        stamp = naming.strftime("%Y-%m-%d_%H-%M-%S") + \
            "-%03d" % (naming.microsecond // 1000)
        filename = "%s_%06d.%s" % (stamp, counter, args.format)

        course = shot.get("heading")
        img_direction = None
        if course is not None:
            img_direction = (course + args.heading_offset) % 360.0

        records.append({
            "pts": pts,
            "frame": frame,
            "distance": shot.get("distance"),
            "lat": shot.get("lat"),
            "lon": shot.get("lon"),
            "alt": shot.get("alt"),
            "speed": shot.get("speed"),
            "course": course,
            "img_direction": img_direction,
            "grade": shot.get("grade"),
            "velocity": shot.get("velocity"),
            "captured_local": captured_local,
            "captured_utc": captured_utc,
            "utc_offset_s": tz_offset,
            "date": naming.strftime("%Y-%m-%d"),
            "filename": filename,
        })
    return records


def time_grid_shots(duration, interval):
    """Fallback shot list when no usable telemetry exists."""
    shots = []
    if not duration or interval <= 0:
        return shots
    count = int(duration / interval) + 1
    for index in range(count):
        shots.append({
            "pts": index * interval, "t": None, "distance": index * interval,
            "lat": None, "lon": None, "alt": None, "speed": None,
            "heading": None, "grade": None, "velocity": None,
        })
    return shots


def process_video(video, sequence, output_dir, all_fits, args, telemetry_cache,
                  have_exiftool):
    """Export one video sequence. Returns a summary dict; never raises Fatal."""
    name = os.path.basename(video)
    LOG.info("--- %s -> %s", name, sequence)

    info = probe_video(video)
    LOG.info("  video: %sx%s %s %.3f fps, %s frames, %.2f s",
             info["width"], info["height"], info["codec"], info["fps"],
             info["frames"], info["duration"] or 0.0)

    atoms = read_mp4_atoms(video)
    if atoms.get("fit_reference"):
        LOG.info("  video references FIT: %s", atoms["fit_reference"])
    created_local = parse_creation_time(info.get("creation_time"))

    explicit = args.fit[0] if len(args.fit) == 1 else None
    candidates = ([os.path.abspath(p) for p in args.fit] if args.fit
                  else fits_for_video(video, all_fits, args.fit_directory_name))
    fit_path, fit_ref, problem = pick_fit(atoms, candidates, created_local, explicit)
    if problem:
        LOG.warning("  %s", problem)

    telemetry = None
    if fit_path:
        if fit_path in telemetry_cache:
            telemetry = telemetry_cache[fit_path]
        else:
            try:
                telemetry = Telemetry(fit_path)
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

    tz_offset = resolve_utc_offset(args, telemetry)
    if tz_offset is not None:
        LOG.info("  camera-local time offset: %s", v2i.offset_string(tz_offset))

    # ---- align video and telemetry ---------------------------------------
    video_start_fit = None
    video_end_fit = None
    duration = info["duration"]
    if telemetry is not None and telemetry.samples:
        video_start_fit, video_end_fit = choose_segment(
            telemetry, created_local, duration, tz_offset)
        if video_start_fit is not None:
            LOG.info("  video starts at FIT t=%.3f s", video_start_fit)
        if video_end_fit is not None:
            LOG.info("  video segment ends at FIT t=%.3f s", video_end_fit)

    video_start_utc = telemetry.utc_of(video_start_fit) if (telemetry and video_start_fit is not None) else None
    video_start_local = None
    if video_start_utc is not None and tz_offset is not None:
        video_start_local = (video_start_utc + dt.timedelta(seconds=tz_offset)).replace(tzinfo=None)
    elif video_start_utc is not None:
        video_start_local = video_start_utc.replace(tzinfo=None)
    elif created_local is not None:
        video_start_local = created_local
        if tz_offset is not None:
            video_start_utc = created_local.replace(tzinfo=dt.timezone.utc) - \
                dt.timedelta(seconds=tz_offset)
    if video_start_local is not None:
        LOG.info("  video start (local): %s", video_start_local.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3])

    # ---- plan the distance-sampled shots ---------------------------------
    shots = []
    track = None
    if telemetry is not None and video_start_fit is not None:
        t_end = None
        if duration:
            t_end = video_start_fit + duration
        elif video_end_fit is not None:
            t_end = video_end_fit
        else:
            t_end = telemetry.samples[-1]["t"]
        if video_end_fit is not None:
            t_end = min(t_end, video_end_fit) if duration else video_end_fit
        samples = telemetry.sample_window(video_start_fit, t_end)
        if samples:
            track = Track(samples, args.distance_mode)
            shots = plan_shots(track, args.distance,
                               start_distance=args.start_distance,
                               max_shots=args.max_images)
            LOG.info("  %d capture point(s) every %g m over %.0f m of travel",
                     len(shots), args.distance, track.length)
            odometer = telemetry.odometer_window(video_start_fit, t_end)
            if odometer:
                LOG.debug("  accuracy check: walked %.1f m vs odometer %.1f m (%+.1f%%)",
                          track.length, odometer,
                          100.0 * (track.length / odometer - 1.0))
        else:
            LOG.warning("  no GPS samples inside the video window")

    if not shots:
        if args.fallback_interval > 0 and duration:
            shots = time_grid_shots(duration, args.fallback_interval)
            LOG.warning("  falling back to a %g s time grid (%d images, no position)",
                        args.fallback_interval, len(shots))
        else:
            LOG.error("  no distance-based capture points; skipping this video")
            return {"video": name, "sequence": sequence, "images": 0,
                    "planned": 0, "skipped": 0, "failed": True}

    records = build_records(shots, info, args, video_start_fit,
                            video_start_local, video_start_utc, tz_offset)
    dates = sorted({r["date"] for r in records})
    if len(dates) > 1:
        LOG.warning("  recording spans %d calendar days (%s); each image is filed "
                    "under its own capture date", len(dates), ", ".join(dates))
    else:
        LOG.info("  capture date: %s", dates[0])

    # ---- firmware / source info ------------------------------------------
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
        "serial": telemetry.device.get("serial") if telemetry else None,
        "software": "VIRB Ultra 30 FW %s" % (firmware or "unknown"),
        "orientation_deg": telemetry.camera_orientation() if telemetry else None,
        "utc_offset_s": tz_offset,
    }
    if source["orientation_deg"] is not None:
        LOG.info("  camera mount orientation: %d degrees", source["orientation_deg"])

    # ---- plan output paths -----------------------------------------------
    for record in records:
        record["path"] = os.path.join(output_dir, record["date"], sequence,
                                      record["filename"])

    if args.dry_run:
        LOG.info("  dry run: %d image(s) would be written", len(records))
        return {"video": name, "sequence": sequence, "images": 0,
                "planned": len(records), "skipped": 0,
                "dir": os.path.join(output_dir, records[0]["date"], sequence)}

    # ---- extract ----------------------------------------------------------
    to_extract = []
    skipped = 0
    for index, record in enumerate(records, start=1):
        if os.path.exists(record["path"]) and not args.overwrite:
            LOG.debug("  exists, skipping: %s", record["filename"])
            skipped += 1
            continue
        to_extract.append({"index": index, "pts": record["pts"],
                           "dest": record["path"], "record": record})

    written_paths = []
    temp_dir = tempfile.mkdtemp(prefix="virb30_")
    try:
        jobs = [{"index": job["index"], "pts": job["pts"]} for job in to_extract]
        extracted = v2i.extract_frames(
            video, jobs, temp_dir, image_format=args.format,
            quality=args.quality, workers=args.workers, fps=info["fps"],
            threads=args.threads)
        for job in to_extract:
            temp_path = extracted.get(job["index"])
            if not temp_path:
                continue
            dest = job["dest"]
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            shutil.copyfile(temp_path, dest)
            job["record"]["written"] = True
            written_paths.append(dest)
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)

    LOG.info("  wrote %d image(s), skipped %d existing", len(written_paths), skipped)

    if have_exiftool and not args.no_exif and written_paths:
        pairs = [(record["path"], build_metadata_args(record, source, args))
                 for record in records if record.get("written")]
        v2i.write_metadata(pairs)

    if args.sidecars:
        sidecar = os.path.join(output_dir, records[0]["date"], sequence,
                               "%s.track.jsonl" % sequence)
        os.makedirs(os.path.dirname(sidecar), exist_ok=True)
        write_sidecar(sidecar, sequence, name, records, source, telemetry, args)

    entries = [{
        "video": name, "sequence": sequence, "image": record["filename"],
        "path": record["path"], "latitude": record.get("lat"),
        "longitude": record.get("lon"), "altitude_m": record.get("alt"),
        "distance_m": record.get("distance"),
        "datetime_local": (record["captured_local"].isoformat()
                           if record.get("captured_local") else None),
        "datetime_utc": (record["captured_utc"].isoformat()
                         if record.get("captured_utc") else None),
    } for record in records if record.get("written")]

    return {"video": name, "sequence": sequence, "images": len(written_paths),
            "planned": len(records), "skipped": skipped, "entries": entries,
            "dir": os.path.join(output_dir, records[0]["date"], sequence)}


# ===========================================================================
# CLI
# ===========================================================================

def build_parser():
    parser = argparse.ArgumentParser(
        prog=SCRIPT_NAME,
        description="Export geotagged images from Garmin VIRB Ultra 30 videos "
                    "every N metres of travel, with EXIF suitable for Mapillary "
                    "and Panoramax.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=("Example:\n"
                "  %(prog)s --input video/ --output images/ --distance 5\n\n"
                "Only files recorded by a Garmin VIRB Ultra 30 are processed; any "
                "other video found under --input is skipped with a logged reason.\n\n"
                "Time base: file names and folders use camera-local time by "
                "default; use --time-base utc for UTC.") % {"prog": SCRIPT_NAME})

    # Options shared by every *-v2i.py exporter (identical names/defaults/help).
    v2i.add_common_arguments(parser)
    # Options shared with the BlackVue exporter (FIT/NMEA track jobs).
    v2i.add_track_export_arguments(parser)

    # -- VIRB-specific options --------------------------------------------
    parser.add_argument("--fit", action="append", default=[], metavar="FILE",
                        help="use this FIT file (repeatable); one value applies to all "
                             "videos")
    parser.add_argument("--fit-directory-name", default=DEFAULT_FIT_DIR, metavar="NAME",
                        help="sibling folder holding FIT telemetry (default '%s')"
                             % DEFAULT_FIT_DIR)
    # Legacy alias kept for backwards compatibility with the old CLI.
    parser.add_argument("--video-glob", dest="video_glob", default=None,
                        metavar="PATTERN", help=argparse.SUPPRESS)
    return parser


def validate_args(parser, args):
    v2i.validate_common_args(parser, args)
    if args.fallback_interval < 0:
        parser.error("--fallback-interval must not be negative")
    for path in args.fit:
        if not os.path.isfile(path):
            parser.error("--fit file not found: %s" % path)


def write_manifest(path, entries):
    payload = {
        "generator": SCRIPT_NAME,
        "camera": v2i.CAMERA_VIRB,
        "camera_label": v2i.CAMERA_LABELS[v2i.CAMERA_VIRB],
        "image_count": len(entries),
        "images": entries,
    }
    with open(path, "w") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
        fh.write("\n")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)
    v2i.setup_logging(v2i.log_level(args))

    for tool in ("ffmpeg", "ffprobe"):
        if shutil.which(tool) is None:
            parser.error("required program not found on PATH: %s" % tool)
    have_exiftool = shutil.which("exiftool") is not None
    if not have_exiftool and not args.no_exif:
        LOG.warning("exiftool not found on PATH; images will have no EXIF metadata")

    input_arg = os.path.abspath(args.input)
    root = input_arg if os.path.isdir(input_arg) else os.path.dirname(input_arg)
    output_dir = os.path.abspath(args.output)

    extensions = v2i.resolve_extensions(args)
    videos = v2i.discover_videos(input_arg, extensions, args.recursive)
    if not videos:
        LOG.error("no video files matching %s found under %s",
                  ",".join(extensions), input_arg)
        return 2

    # Keep only files this camera recorded; never touch another camera's video.
    accepted = []
    for video in videos:
        detection = detect_camera(video)
        reason = skip_reason(detection, v2i.CAMERA_VIRB)
        if reason:
            LOG.warning("skipping %s: %s", os.path.basename(video), reason)
            continue
        LOG.debug("accepted %s (%s)", os.path.basename(video),
                  "; ".join(detection.evidence.get(v2i.CAMERA_VIRB, [])))
        accepted.append(video)
    if not accepted:
        LOG.error("no %s video found among %d candidate file(s)",
                  v2i.CAMERA_LABELS[v2i.CAMERA_VIRB], len(videos))
        return 2

    all_fits = find_fits(root, exclude_dirs=[output_dir])
    LOG.info("found %d %s video(s) and %d FIT file(s) under %s",
             len(accepted), v2i.CAMERA_LABELS[v2i.CAMERA_VIRB],
             len(all_fits), root)

    if not args.dry_run:
        try:
            os.makedirs(output_dir, exist_ok=True)
        except OSError as exc:
            LOG.error("cannot create output directory %s: %s", output_dir, exc)
            return 2

    sequence_ids = v2i.build_sequence_ids(accepted, root)
    telemetry_cache = {}
    summaries = []
    manifest_entries = []
    for video in accepted:
        try:
            summary = process_video(video, sequence_ids[video], output_dir,
                                    all_fits, args, telemetry_cache,
                                    have_exiftool)
            summaries.append(summary)
            manifest_entries.extend(summary.get("entries", []))
        except Fatal as exc:
            LOG.error("%s: %s", os.path.basename(video), exc)
            summaries.append({"video": os.path.basename(video),
                              "sequence": sequence_ids[video],
                              "images": 0, "planned": 0, "skipped": 0,
                              "failed": True})
            if args.fail_fast:
                break
        except Exception as exc:                            # noqa: BLE001
            LOG.exception("%s: unexpected error: %s", os.path.basename(video), exc)
            summaries.append({"video": os.path.basename(video),
                              "sequence": sequence_ids[video],
                              "images": 0, "planned": 0, "skipped": 0,
                              "failed": True})
            if args.fail_fast:
                break

    total_images = sum(s["images"] for s in summaries)
    total_planned = sum(s["planned"] for s in summaries)
    failed = [s for s in summaries if s.get("failed")]
    LOG.info("done: %d image(s) from %d video(s) -> %s",
             total_images, len(summaries), output_dir)
    if args.dry_run:
        LOG.info("dry run: %d image(s) would be written", total_planned)
    if args.manifest and not args.dry_run:
        try:
            write_manifest(args.manifest, manifest_entries)
            LOG.info("wrote manifest: %s (%d image(s))",
                     args.manifest, len(manifest_entries))
        except OSError as exc:
            LOG.error("cannot write manifest %s: %s", args.manifest, exc)
    if failed:
        LOG.warning("%d video(s) produced no images (see messages above)", len(failed))
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
