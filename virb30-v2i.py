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
import v2i_metadata
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
        accuracy_raw = _num(fields.get(8))
        hdop_raw = _num(fields.get(11))
        vdop_raw = _num(fields.get(12))
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
# Records, metadata and per-video processing
# ===========================================================================

def _record_from_choice(choice, track, args, video_start_utc, tz_offset,
                        tz_known, camera, sequence, video_name):
    """Turn a selected frame into a manifest/metadata record."""
    point = choice.point or {}
    pts = max(0.0, float(choice.pts))
    captured_utc = (video_start_utc + dt.timedelta(seconds=pts)
                    if video_start_utc is not None else None)
    captured_local = (v2i.local_from_utc(captured_utc, tz_offset)
                      if tz_known else None)
    if args.time_base == "utc" and captured_utc is not None:
        naming = captured_utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
    else:
        naming = captured_local
        if naming is None and captured_utc is not None:
            naming = captured_utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
    if naming is None:
        naming = dt.datetime(1970, 1, 1)

    heading, heading_source = v2i.resolve_heading(
        point, args.heading_offset, args.magnetic_declination)
    accuracy, accuracy_estimated, accuracy_source = v2i.derive_accuracy(
        point, args.uere)
    course = point.get("course")
    if course is None:
        course = point.get("gps_course")
    speed = point.get("speed")
    if speed is None and point.get("speed2d") is not None:
        speed = point["speed2d"]

    filename = v2i.image_filename(naming, args.format)
    return {
        "sequence": sequence,
        "video": video_name,
        "camera": camera,
        "image": filename,
        "frame_index": int(choice.frame_index),
        "pts_seconds": round(pts, 3),
        "datetime_local": captured_local.isoformat() if captured_local else None,
        "datetime_utc": v2i.fmt_utc(captured_utc),
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


def _unpositioned_choices(timeline, interval, duration, max_images):
    frames = v2i.select_time_grid_frames(
        timeline, interval, duration=duration, max_images=max_images)
    out = []
    for index in frames:
        pts = timeline.pts(index)
        out.append(v2i.FrameChoice(
            target_distance=pts, target_time=pts, frame_index=index, pts=pts,
            point={}, position_error=None, clamped=False, estimated=True))
    return out


def _write_sidecar(path, sequence, video_name, records, source, args):
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


def _metadata_for(record, source, args, camera):
    return v2i_metadata.FrameMetadata(
        camera=v2i.CAMERA_LABELS[camera],
        video=record["video"], sequence=record["sequence"],
        frame_index=record["frame_index"], pts=record["pts_seconds"],
        distance=record.get("distance_m"), script_name=SCRIPT_NAME,
        make=source.get("make"), model=source.get("model"),
        software=source.get("software"), serial=source.get("serial"),
        captured_utc=v2i.parse_utc_timestamp(record.get("datetime_utc")),
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
        keywords=v2i.parse_keywords(getattr(args, "keywords", None)),
        extra={"fit": source.get("fit"),
               "camera_orientation_deg": source.get("orientation_deg")},
    )


def choose_segment(telemetry, created_local, duration, tz_offset):
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
                local = v2i.local_from_utc(utc, tz_offset)
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


def process_video(video, sequence, output_dir, all_fits, args, telemetry_cache,
                  have_exiftool):
    """Export one video sequence. Returns a summary dict; never raises Fatal."""
    name = os.path.basename(video)
    LOG.info("--- %s -> %s", name, sequence)

    info = probe_video(video)
    LOG.info("  video: %sx%s %s %.3f fps, %s frames, %.2f s",
             info["width"], info["height"], info["codec"], info["fps"],
             info["frames"], info["duration"] or 0.0)
    timeline = v2i.probe_frame_timeline(
        video, fps_hint=info["fps"], frame_count_hint=info["frames"],
        duration_hint=info["duration"], logger=LOG)
    LOG.info("  frame timeline: %d frame(s), source=%s",
             len(timeline), timeline.source)

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

    detected_offset = telemetry.local_utc_offset() if telemetry else None
    tz_offset, tz_known = v2i.resolve_timezone(
        args, detected_offset, camera=v2i.CAMERA_VIRB, logger=LOG)
    if tz_known:
        LOG.info("  camera-local time offset: %s", v2i.offset_string(tz_offset))
    else:
        LOG.warning("  camera-local offset unknown; using UTC")

    # ---- align video and telemetry ---------------------------------------
    video_start_fit = None
    video_end_fit = None
    duration = info["duration"]
    if telemetry is not None and telemetry.samples:
        video_start_fit, video_end_fit = choose_segment(
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
        LOG.info("  video start (UTC): %s", v2i.fmt_utc(video_start_utc))

    # ---- build the time-indexed track ------------------------------------
    track = None
    samples = []
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
        for sample in telemetry.sample_window(video_start_fit, t_end):
            item = dict(sample)
            item["t"] = sample["t"] - video_start_fit
            item["utc"] = telemetry.utc_of(sample["fit_s"])
            v2i.apply_accuracy(item, args.uere)
            samples.append(item)
        if samples:
            track = v2i.Track(samples, args.distance_mode)
            LOG.info("  track: %d sample(s), %.0f m of travel",
                     len(samples), track.length)

    choices = []
    skipped = []
    if track is not None and track.samples:
        choices, skipped = v2i.select_unique_frames(
            track, timeline, args.distance,
            start_distance=args.start_distance,
            max_position_error=args.max_position_error,
            max_images=args.max_images, mode=args.distance_mode, logger=LOG)
        if skipped:
            LOG.info("  skipped %d target(s) with no acceptable unused frame",
                     len(skipped))

    if not choices:
        if args.fallback_interval > 0 and duration:
            choices = _unpositioned_choices(
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
        records.append(_record_from_choice(
            choice, track, args, video_start_utc, tz_offset, tz_known,
            v2i.CAMERA_VIRB, sequence, name))

    first_time = records[0]["naming_time"]
    sequence_dir = v2i.sequence_output_dir(
        output_dir, v2i.CAMERA_VIRB, sequence, first_time)
    used_paths = set()
    for record in records:
        record["path"] = v2i.unique_path(
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
        extracted = v2i.extract_frames_by_index(
            video, [r["frame_index"] for r in to_extract], temp_dir,
            image_format=args.format, quality=args.quality,
            workers=args.workers, threads=args.threads)
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
        pairs = [(record["path"], v2i_metadata.build_exiftool_args(
            _metadata_for(record, source, args, v2i.CAMERA_VIRB)))
            for record in records if record.get("written")]
        v2i.write_metadata(pairs)

    export_samples = (track.samples if track is not None else [])
    v2i.export_track_files(
        sequence_dir, sequence, export_samples, records,
        track_export=args.track_export, camera=v2i.CAMERA_VIRB, video=name)

    if args.sidecars:
        os.makedirs(sequence_dir, exist_ok=True)
        _write_sidecar(os.path.join(sequence_dir, sequence + ".track.jsonl"),
                       sequence, name, records, source, args)

    entries = [{k: v for k, v in record.items()
                if k not in ("point", "naming_time")}
               for record in records if record.get("written")]
    return {"video": name, "sequence": sequence, "images": len(written_paths),
            "planned": len(records), "skipped": skipped_existing,
            "entries": entries, "dir": sequence_dir}


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
