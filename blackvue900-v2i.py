#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""blackvue900-v2i.py -- export geotagged still images from BlackVue
DR900 dash-camera videos at a fixed travelled-distance interval.

A BlackVue DR900 records ordinary MP4 video, and embeds its GPS log as NMEA
sentences inside ``free`` atoms of the container (one log near the start of a
recording and a duplicate in the trailing concatenated segment).  Each sentence
is prefixed with the camera-local wall clock as an epoch-millisecond stamp::

    [1611739535146]$GPGGA,072434.40,4533.02681,N,01535.93605,E,1,08,1.10,...
    [1611739535146]$GPRMC,072434.60,A,4533.02977,N,01535.93935,E,67.672,...
    [1611739535146]$GPVTG,38.32,T,,M,67.672,N,125.329,K,A*3B

This tool reads that log, walks the GPS track, and every `--distance` metres of
travel asks ffmpeg for the video frame captured at the interpolated moment and
writes it as a still image with full EXIF/XMP geo metadata suitable for
Mapillary / Panoramax.

INPUT
-----
`--input` is either a single video file or a root directory holding one or
more recordings::

    video/
    `-- 2021-01-27/
        |-- 20210127_092534_NF.mp4
        `-- 20210127_092635_NF.mp4

Every candidate file is inspected first: only files recorded by a BlackVue
DR900 are processed, and anything else (a Garmin VIRB MP4, a GoPro MAX .360,
an unreadable file, ...) is skipped with a logged reason, so a folder holding
several cameras' footage can be handed over safely.

OUTPUT
------
Images are written below `--output` as::

    images/
    `-- 2021-01-27/                     <- capture date
        `-- 20210127_092534_NF/         <- sequence, derived from the video name
            |-- 2021-01-27_09-24-33-526_000001.jpg
            `-- ...

  * The file name is ``YYYY-MM-DD_HH-MM-SS-mmm_NNNNNN.ext`` with zero padded
    fields and a per-sequence 6-digit counter.
  * Existing files are never overwritten unless ``--overwrite`` is given.
  * ``--sidecars`` additionally writes a ``<sequence>.track.jsonl`` manifest.

TIMESTAMPS
----------
The bracket stamp is the camera's own wall clock; the NMEA time is true GPS
UTC.  A BlackVue clock typically runs up to a minute ahead of GPS, so the two
disagree by more than the timezone.  This tool therefore:

  * derives the video-relative time of every fix from the bracket stamp minus
    the recording's start time (taken from the file name, e.g. ``092534``);
  * takes the absolute UTC instant from the NMEA date/time fields;
  * derives the timezone as the quarter-hour-snapped difference between the
    bracket clock and GPS UTC (``--timezone`` overrides it).

The result is that ``DateTimeOriginal`` / ``OffsetTimeOriginal`` together point
at the true GPS instant, while file names/ folders carry the same instant in
local time.  Use ``--time-base utc`` to name files in UTC instead.  If the
camera clock is skewed by >= 1 s, the tool logs the offset and uses the
reconciled start time.

DEPENDENCIES
------------
Python standard library only.  ``ffmpeg`` and ``ffprobe`` must be on PATH;
``exiftool`` is required to write EXIF metadata (if it is missing a warning is
printed and images are written without metadata).  The shared command-line
options live in ``v2i_common.py``.
"""

import argparse
import datetime as dt
import json
import logging
import os
import re
import shutil
import statistics
import struct
import subprocess
import sys
import tempfile

import v2i_common as v2i
from v2i_common import Track, detect_camera, plan_shots, skip_reason

SCRIPT_NAME = os.path.basename(os.path.realpath(__file__))
LOG = logging.getLogger("blackvue900-v2i")

KNOTS_TO_MPS = 0.514444
DEFAULT_MODEL = "DR900"
DEFAULT_MAKE = "BlackVue"

_RE_VIDEO_NAME = re.compile(r"^(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})_[A-Za-z]{2}\.")
_RE_NMEA = re.compile(rb"\[(\d{10,})\]\$([A-Z]{2}[A-Z]{3}),([^\r\n*]*)")


class Fatal(Exception):
    """A problem that stops the current file."""


# ===========================================================================
# MP4 container inspection (camera identity + NMEA telemetry)
# ===========================================================================

def _iter_top_atoms(path):
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


def read_cprt(path):
    """Read the BlackVue ``cprt`` copyright atom.

    Returns a dict with ``make``, ``model``, ``firmware`` and ``serial``; the
    fields are ``;``-separated, e.g.
    ``Pittasoft Co., Ltd.;DR900M-1CH;0.901;English;1;D90SS1HAE00736;...``.
    """
    info = {"make": None, "model": None, "firmware": None, "serial": None}
    try:
        with open(path, "rb") as fh:
            for atom_type, offset, size in _iter_top_atoms(path):
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
                    info["make"] = DEFAULT_MAKE
                    info["model"] = parts[1] or None
                    info["firmware"] = parts[2] or None
                if len(parts) >= 6:
                    info["serial"] = parts[5] or None
                if info["model"]:
                    return info
    except OSError:
        pass
    return info


def extract_nmea(path, max_payload=8 * 1024 * 1024):
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
            for atom_type, offset, size in _iter_top_atoms(path):
                if atom_type != b"free" or size == 0:
                    continue
                fh.seek(offset)
                payloads.append(fh.read(min(size, max_payload)))
    except OSError:
        return sentences

    for payload in payloads:
        for match in _RE_NMEA.finditer(payload):
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
            for match in _RE_NMEA.finditer(blob):
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

def _to_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def _parse_time(text):
    """``hhmmss.ss`` -> (hour, minute, second_float) or None."""
    if not text or len(text) < 6:
        return None
    try:
        return int(text[0:2]), int(text[2:4]), float(text[4:])
    except ValueError:
        return None


def _parse_date(text):
    """``ddmmyy`` -> (year, month, day) or None."""
    if not text or len(text) < 6:
        return None
    try:
        day, month, year = int(text[0:2]), int(text[2:4]), int(text[4:6])
    except ValueError:
        return None
    return 2000 + year, month, day


def _nmea_decimal(value, hemisphere):
    """``ddmm.mmmm`` + ``N``/``S``/``E``/``W`` -> signed degrees."""
    number = _to_float(value)
    if number is None:
        return None
    degrees = int(number // 100)
    minutes = number - degrees * 100
    result = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        result = -result
    return result


def _parse_gga(fields):
    # $GPGGA,time,lat,N,lon,E,fix,sats,hdop,alt,M,geoid,M,age,station
    if len(fields) < 10:
        return None
    return {
        "time": _parse_time(fields[1]),
        "lat": _nmea_decimal(fields[2], fields[3]) if len(fields) > 3 else None,
        "lon": _nmea_decimal(fields[4], fields[5]) if len(fields) > 5 else None,
        "fix": int(fields[6]) if fields[6].isdigit() else 0,
        "alt": _to_float(fields[9]) if len(fields) > 9 else None,
    }


def _parse_rmc(fields):
    # $GPRMC,time,status,lat,N,lon,E,speed_knots,course,date,...
    if len(fields) < 10:
        return None
    return {
        "time": _parse_time(fields[1]),
        "status": fields[2],
        "lat": _nmea_decimal(fields[3], fields[4]) if len(fields) > 4 else None,
        "lon": _nmea_decimal(fields[5], fields[6]) if len(fields) > 6 else None,
        "speed_knots": _to_float(fields[7]) if len(fields) > 7 else None,
        "course": _to_float(fields[8]) if len(fields) > 8 else None,
        "date": _parse_date(fields[9]) if len(fields) > 9 else None,
    }


def _parse_vtg(fields):
    # $GPVTG,course_true,T,course_mag,M,speed_knots,N,speed_kmh,K,mode
    if len(fields) < 8:
        return None
    return {
        "course": _to_float(fields[1]),
        "speed_knots": _to_float(fields[5]) if len(fields) > 5 else None,
        "speed_kmh": _to_float(fields[7]) if len(fields) > 7 else None,
    }


def parse_telemetry(sentences, video_start_local):
    """Turn raw NMEA sentences into GPS samples for ``v2i_common.Track``.

    Returns ``(samples, tz_offset_s)``.  Each sample is a dict with ``t``
    (video-relative seconds), ``lat``, ``lon``, ``alt``, ``speed``, ``heading``
    and ``utc`` (timezone-aware UTC).  ``tz_offset_s`` is the quarter-hour
    snapped camera-local minus UTC offset, or ``None`` when it cannot be
    derived.
    """
    records = []
    current = None
    current_bracket = None
    for bracket, kind, fields in sentences:
        if bracket != current_bracket:
            if current is not None:
                records.append(current)
            current = {"bracket": bracket, "gga": None, "rmc": None, "vtg": None}
            current_bracket = bracket
        if kind.endswith("GGA"):
            current["gga"] = _parse_gga(fields)
        elif kind.endswith("RMC"):
            current["rmc"] = _parse_rmc(fields)
        elif kind.endswith("VTG"):
            current["vtg"] = _parse_vtg(fields)
    if current is not None:
        records.append(current)

    # Relative times need an anchor: the file-name start when available,
    # otherwise the first logged fix.  Both come from the camera's own clock so
    # the differences (and therefore every ``t``) stay exact either way.
    if video_start_local is not None:
        anchor = video_start_local
    elif records:
        anchor = dt.datetime.utcfromtimestamp(records[0]["bracket"] / 1000.0)
    else:
        anchor = dt.datetime(1970, 1, 1)

    samples = []
    last_date = None
    for record in records:
        rmc = record["rmc"]
        gga = record["gga"]
        vtg = record["vtg"]
        if rmc and rmc.get("date"):
            last_date = rmc["date"]
        date = last_date or video_start_local.date()

        lat = lon = alt = speed = course = None
        utc = None
        if rmc and rmc.get("status") == "A" and rmc.get("lat") is not None:
            lat, lon = rmc["lat"], rmc["lon"]
            if rmc.get("speed_knots") is not None:
                speed = rmc["speed_knots"] * KNOTS_TO_MPS
            course = rmc.get("course")
            if rmc.get("time"):
                utc = _utc_from(date, rmc["time"])
        elif gga and gga.get("fix", 0) and gga.get("lat") is not None:
            lat, lon = gga["lat"], gga["lon"]
            if gga.get("time"):
                utc = _utc_from(date, gga["time"])
        else:
            continue

        if gga and gga.get("alt") is not None:
            alt = gga["alt"]
        if speed is None and vtg and vtg.get("speed_knots") is not None:
            speed = vtg["speed_knots"] * KNOTS_TO_MPS
        if course is None and vtg and vtg.get("course") is not None:
            course = vtg["course"]
        if utc is None:
            continue
        if lat is None or lon is None or not (-90 <= lat <= 90) \
                or not (-180 <= lon <= 180) or (lat == 0.0 and lon == 0.0):
            continue

        local_wall = dt.datetime.utcfromtimestamp(record["bracket"] / 1000.0)
        t = (local_wall - anchor).total_seconds()
        samples.append({
            "t": t,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "speed": speed,
            "heading": course,
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


def _utc_from(date, time):
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


def parse_video_start_local(path):
    """Recording start time from the file name (``YYYYMMDD_HHMMSS_XX``)."""
    match = _RE_VIDEO_NAME.match(os.path.basename(path))
    if not match:
        return None
    try:
        return dt.datetime(*(int(x) for x in match.groups()))
    except ValueError:
        return None


# ===========================================================================
# Video probing
# ===========================================================================

def probe_video(path):
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
# Metadata (EXIF / XMP) writing
# ===========================================================================

def build_metadata_args(record, source, args):
    """exiftool arguments (without the file name) describing one image."""
    tags = ["-overwrite_original", "-n", "-charset", "filename=utf8",
            "-Orientation=1"]

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
            tags += ["-GPSSpeed=%r" % (record["speed"] * 3.6),
                     "-GPSSpeedRef=K"]
        if record.get("img_direction") is not None:
            tags += ["-GPSImgDirection=%r" % record["img_direction"],
                     "-GPSImgDirectionRef=T"]
        if record.get("course") is not None:
            tags += ["-GPSTrack=%r" % record["course"], "-GPSTrackRef=T"]

    tags += ["-Make=%s" % source["make"], "-Model=%s" % source["model"],
             "-Software=%s" % source["software"]]
    if source.get("serial"):
        tags += ["-SerialNumber=%s" % source["serial"],
                 "-CameraSerialNumber=%s" % source["serial"]]

    comment = ("BlackVue DR900 via %s | video=%s | frame=%d | pts=%.3fs | "
               "distance=%.2fm | heading_offset=%+gdeg"
               % (SCRIPT_NAME, source["video"], record["frame"], record["pts"],
                  record.get("distance") or 0.0, args.heading_offset))
    tags += ["-ImageDescription=" + comment,
             "-UserComment=" + comment,
             "-XPComment=" + comment,
             "-XMP:CreatorTool=" + SCRIPT_NAME,
             "-XMP:Title=%s %s" % (source["video"],
                                   local.strftime("%Y-%m-%d %H:%M:%S")
                                   if local else "")]

    extra = {
        "source_video": source["video"],
        "frame_index": record["frame"],
        "pts_seconds": round(record["pts"], 3),
        "distance_m": record.get("distance"),
        "heading_offset_deg": args.heading_offset,
        "grade_deg": record.get("grade"),
        "utc": record["captured_utc"].isoformat() if record.get("captured_utc") else None,
    }
    tags += ["-XMP:Subject=%s" % json.dumps(extra, sort_keys=True).replace('"', "'"),
             "-Keywords=blackvue;dr900;pittasoft;dashcam;streetview;mapillary;panoramax"]
    return tags


def write_sidecar(path, sequence, video_name, records, source, args):
    """Write a JSONL manifest describing every planned image of a sequence."""
    meta = {
        "generator": SCRIPT_NAME,
        "sequence": sequence,
        "source_video": video_name,
        "camera": {
            "make": source.get("make"),
            "model": source.get("model"),
            "serial": source.get("serial"),
            "software": source["software"],
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
            }) + "\n")


# ===========================================================================
# Per-video processing
# ===========================================================================

def build_records(shots, info, args, video_start_local, video_start_utc,
                  tz_offset):
    """Turn planned shots into capture records with names and times."""
    fps = info["fps"] or 0.0
    duration = info["duration"]
    records = []
    for counter, shot in enumerate(shots, start=1):
        pts = shot.get("t")
        if pts is None:
            pts = shot.get("pts") or 0.0
        pts = max(0.0, float(pts))
        if duration:
            pts = min(pts, duration)
        if fps:
            frame = max(0, int(round(pts * fps)))
            if info.get("frames"):
                frame = min(frame, info["frames"] - 1)
            pts = frame / fps
        else:
            frame = 0

        captured_local = None
        if video_start_local is not None:
            captured_local = video_start_local + dt.timedelta(seconds=pts)
        captured_utc = None
        if video_start_utc is not None:
            captured_utc = (video_start_utc + dt.timedelta(seconds=pts)
                            ).replace(tzinfo=None)

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
            "heading": None, "grade": None,
        })
    return shots


def process_video(video, sequence, output_dir, args, have_exiftool):
    """Export one video sequence. Returns a summary dict; never raises Fatal."""
    name = os.path.basename(video)
    LOG.info("--- %s -> %s", name, sequence)

    info = probe_video(video)
    LOG.info("  video: %sx%s %s %.3f fps, %s frames, %.2f s",
             info["width"], info["height"], info["codec"], info["fps"],
             info["frames"], info["duration"] or 0.0)

    cprt = read_cprt(video)
    source = {
        "video": name,
        "make": cprt.get("make") or DEFAULT_MAKE,
        "model": cprt.get("model") or DEFAULT_MODEL,
        "firmware": cprt.get("firmware"),
        "serial": cprt.get("serial"),
    }

    filename_start = parse_video_start_local(video)
    if filename_start is None:
        LOG.warning("  file name does not carry a start time; deriving it from "
                    "the GPS log")

    sentences = extract_nmea(video)
    LOG.debug("  %d NMEA sentence(s) found", len(sentences))
    samples, detected_tz = parse_telemetry(sentences, filename_start)

    tz_offset = v2i.parse_timezone(getattr(args, "timezone", None))
    if tz_offset is None:
        tz_offset = detected_tz
    if tz_offset is None:
        tz_offset = 0

    video_start_utc = None
    video_start_local = filename_start
    if samples:
        first = samples[0]
        video_start_utc = first["utc"] - dt.timedelta(seconds=first["t"])
        # The camera clock (and thus the file name) can run ahead of GPS.
        # Anchor the local time base to the true GPS instant so that
        # DateTimeOriginal + OffsetTimeOriginal reproduce the exact UTC fix.
        video_start_local = (video_start_utc +
                             dt.timedelta(seconds=tz_offset)).replace(tzinfo=None)
        if filename_start is not None:
            skew = (video_start_local - filename_start).total_seconds()
            if abs(skew) >= 1.0:
                LOG.info("  camera clock is %+.0f s off GPS; times follow GPS",
                         skew)
        LOG.info("  telemetry: %d GPS samples, camera-local offset %s",
                 len(samples), v2i.offset_string(tz_offset))
        LOG.info("  video start (local): %s",
                 video_start_local.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3])
    else:
        LOG.warning("  no GPS data in this video")

    shots = []
    if samples:
        track = Track(samples, args.distance_mode)
        shots = plan_shots(track, args.distance,
                           start_distance=args.start_distance,
                           max_shots=args.max_images)
        LOG.info("  %d capture point(s) every %g m over %.0f m of travel",
                 len(shots), args.distance, track.length)

    if not shots:
        if args.fallback_interval > 0 and info["duration"] \
                and video_start_local is not None:
            shots = time_grid_shots(info["duration"], args.fallback_interval)
            LOG.warning("  falling back to a %g s time grid (%d images, no position)",
                        args.fallback_interval, len(shots))
        else:
            if samples:
                LOG.error("  no distance-based capture points; skipping this video")
            else:
                LOG.error("  no GPS data and no fallback requested; skipping this video")
            return {"video": name, "sequence": sequence, "images": 0,
                    "planned": 0, "skipped": 0, "failed": True}

    records = build_records(shots, info, args, video_start_local,
                            video_start_utc, tz_offset)
    dates = sorted({r["date"] for r in records})
    if len(dates) > 1:
        LOG.warning("  recording spans %d calendar days (%s); each image is "
                    "filed under its own capture date", len(dates),
                    ", ".join(dates))
    else:
        LOG.info("  capture date: %s", dates[0])

    source["software"] = "BlackVue %s FW %s" % (
        source["model"], source["firmware"] or "unknown")
    source["utc_offset_s"] = tz_offset

    for record in records:
        record["path"] = os.path.join(output_dir, record["date"], sequence,
                                      record["filename"])

    if args.dry_run:
        LOG.info("  dry run: %d image(s) would be written", len(records))
        return {"video": name, "sequence": sequence, "images": 0,
                "planned": len(records), "skipped": 0,
                "dir": os.path.join(output_dir, records[0]["date"], sequence)}

    to_extract = []
    skipped = 0
    for index, record in enumerate(records, start=1):
        if os.path.exists(record["path"]) and not args.overwrite:
            LOG.debug("  exists, skipping: %s", record["filename"])
            skipped += 1
            continue
        to_extract.append({"index": index, "pts": record["pts"],
                           "record": record})

    written_paths = []
    temp_dir = tempfile.mkdtemp(prefix="blackvue_")
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
            dest = job["record"]["path"]
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            shutil.copyfile(temp_path, dest)
            job["record"]["written"] = True
            written_paths.append(dest)
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)

    LOG.info("  wrote %d image(s), skipped %d existing", len(written_paths),
             skipped)

    if have_exiftool and not args.no_exif and written_paths:
        pairs = [(record["path"], build_metadata_args(record, source, args))
                 for record in records if record.get("written")]
        v2i.write_metadata(pairs)

    if args.sidecars:
        sidecar = os.path.join(output_dir, records[0]["date"], sequence,
                               "%s.track.jsonl" % sequence)
        os.makedirs(os.path.dirname(sidecar), exist_ok=True)
        write_sidecar(sidecar, sequence, name, records, source, args)

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
        description="Export geotagged images from BlackVue DR900 dash-camera "
                    "videos every N metres of travel, with EXIF suitable for "
                    "Mapillary and Panoramax.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=("Example:\n"
                "  %(prog)s --input video/ --output images/ --distance 5\n\n"
                "Only files recorded by a BlackVue DR900 are processed; any "
                "other video found under --input is skipped with a logged "
                "reason.\n\n"
                "Time base: file names and folders use camera-local time by "
                "default; use --time-base utc for UTC.") % {"prog": SCRIPT_NAME})

    # Options shared by every *-v2i.py exporter (identical names/defaults/help).
    v2i.add_common_arguments(parser)
    # Options shared with the GoPro MAX exporter (NMEA/GPMF GPS handling).
    v2i.add_gps_filter_arguments(parser)
    # Options shared with the VIRB exporter (FIT/NMEA track jobs).
    v2i.add_track_export_arguments(parser)

    # BlackVue DR900 needs no options beyond the shared set.
    return parser


def validate_args(parser, args):
    v2i.validate_common_args(parser, args)
    if args.fallback_interval < 0:
        parser.error("--fallback-interval must not be negative")


def write_manifest(path, entries):
    payload = {
        "generator": SCRIPT_NAME,
        "camera": v2i.CAMERA_BLACKVUE,
        "camera_label": v2i.CAMERA_LABELS[v2i.CAMERA_BLACKVUE],
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
        reason = skip_reason(detection, v2i.CAMERA_BLACKVUE)
        if reason:
            LOG.warning("skipping %s: %s", os.path.basename(video), reason)
            continue
        LOG.debug("accepted %s (%s)", os.path.basename(video),
                  "; ".join(detection.evidence.get(v2i.CAMERA_BLACKVUE, [])))
        accepted.append(video)
    if not accepted:
        LOG.error("no %s video found among %d candidate file(s)",
                  v2i.CAMERA_LABELS[v2i.CAMERA_BLACKVUE], len(videos))
        return 2

    LOG.info("found %d %s video(s) under %s", len(accepted),
             v2i.CAMERA_LABELS[v2i.CAMERA_BLACKVUE], input_arg)

    if not args.dry_run:
        try:
            os.makedirs(output_dir, exist_ok=True)
        except OSError as exc:
            LOG.error("cannot create output directory %s: %s", output_dir, exc)
            return 2

    sequence_ids = v2i.build_sequence_ids(accepted, root)
    summaries = []
    manifest_entries = []
    for video in accepted:
        try:
            summary = process_video(video, sequence_ids[video], output_dir,
                                    args, have_exiftool)
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
