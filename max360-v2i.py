#!/usr/bin/env python3
"""max360-v2i - extract geotagged equirectangular still images from
GoPro MAX ``.360`` videos at a fixed GPS distance interval.

This is a single, self-contained script.  Copy it to any machine that has
``ffmpeg``/``ffprobe`` and ``exiftool`` on ``PATH`` and run it directly::

    ./max360-v2i.py -i video/ -o images/ -d 5

``--input`` (also ``-i``) may be a single ``.360`` file or a directory.  Every
candidate file is inspected first: only files recorded by a GoPro MAX are
processed, and anything else (a Garmin VIRB MP4, a BlackVue clip, an
unreadable file, ...) is skipped with a logged reason, so a folder holding
several cameras' footage can be handed over safely.

Only the Python standard library is used (Python >= 3.8); there is nothing to
``pip install``.  The shared command-line options live in ``v2i_common.py``.

The file is laid out as one section per concern, in dependency order:

    1. geodesy   - spherical-earth distance / bearing helpers
    2. gpmf      - dependency-free GoPro GPMF (KLV) telemetry reader
    3. gps       - GPMF -> time-stamped, distance-annotated GPS track
    4. ffmpeg    - all ffmpeg/ffprobe interaction (EAC stitch + v360 reproject)
    5. metadata  - EXIF / XMP-GPano tags written with exiftool
    6. cli       - argument parsing and orchestration
"""
from __future__ import annotations

__version__ = "1.0.0"

import argparse
import datetime as _dt
import json
import logging
import math
import os
import shutil
import statistics
import struct
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Iterator, List, Optional, Sequence, Union

import v2i_common as v2i

LOG = logging.getLogger("max360-v2i")


# =============================================================================
# 1. Geodesy - spherical-earth helpers (no external dependencies)
# =============================================================================
# Distances use the haversine great-circle formula on a spherical earth.
# This is accurate to better than ~0.5 % versus WGS-84 ellipsoidal distances
# at the scales involved here (a few metres to a few kilometres), which is
# well below the accuracy of consumer GPS / GoPro GPMF positioning.

# IUGG mean earth radius (metres).
EARTH_RADIUS_M = 6_371_008.8


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two WGS-84 coordinates, in metres."""
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2.0) ** 2)
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def initial_bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial great-circle bearing (course over ground) from point 1 to 2.

    Returned as degrees clockwise from true north in [0, 360).
    """
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    y = math.sin(dlam) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlam)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


# =============================================================================
# 2. GPMF - a small, dependency-free reader for GoPro "GoPro Metadata Format"
# =============================================================================
# GPMF is the telemetry container carried in the ``gpmd`` track of GoPro MP4 /
# .360 files.  It is a nested key-length-value (KLV) structure.  Every element
# starts with an 8-byte header::
#
#     key      : 4 x ASCII characters (FourCC)
#     type     : 1 byte  (see TYPE_* below)
#     size     : 1 byte  (number of bytes in a single sample)
#     repeat   : 2 bytes big-endian (number of samples)
#
# Payload length is ``size * repeat`` bytes, padded to a 4-byte boundary.
# Elements whose type is ``0x00`` (``NEST``) are containers: their ``repeat``
# field holds the byte length of the nested payload instead of a sample count.
#
# Reference: https://github.com/gopro/gpmf-parser (GPMF_common.h).

# --- Type tags (subset that matters here) ------------------------------------
TYPE_STRING = ord('c')       # ASCII string / int8
TYPE_INT8 = ord('b')
TYPE_UINT8 = ord('B')
TYPE_INT16 = ord('s')
TYPE_UINT16 = ord('S')
TYPE_INT32 = ord('l')
TYPE_UINT32 = ord('L')
TYPE_INT64 = ord('j')
TYPE_UINT64 = ord('J')
TYPE_FLOAT = ord('f')
TYPE_DOUBLE = ord('d')
TYPE_Q15_16 = ord('q')       # fixed point, scale 2**-16
TYPE_Q31_32 = ord('Q')       # fixed point, scale 2**-32
TYPE_FOURCC = ord('F')
TYPE_UTC = ord('U')
TYPE_UTF8 = ord('u')
TYPE_GUID = ord('G')
TYPE_COMPLEX = ord('?')
TYPE_COMPRESSED = ord('#')   # Huffman-compressed stream payload
TYPE_NEST = 0x00

# type byte -> (struct format char, element size in bytes)
_NUMERIC = {
    TYPE_INT8: ('b', 1), TYPE_UINT8: ('B', 1),
    TYPE_INT16: ('h', 2), TYPE_UINT16: ('H', 2),
    TYPE_INT32: ('i', 4), TYPE_UINT32: ('I', 4),
    TYPE_INT64: ('q', 8), TYPE_UINT64: ('Q', 8),
    TYPE_FLOAT: ('f', 4), TYPE_DOUBLE: ('d', 8),
    TYPE_Q15_16: ('i', 4), TYPE_Q31_32: ('q', 8),
}
_BYTES = {TYPE_STRING, TYPE_FOURCC, TYPE_UTC, TYPE_UTF8, TYPE_GUID,
          TYPE_COMPLEX, TYPE_COMPRESSED}

_KNOWN_BYTES = set(_NUMERIC) | _BYTES | {TYPE_NEST}


class GpmfError(Exception):
    """Raised when the GPMF stream cannot be parsed."""


@dataclass
class Element:
    level: int          # nesting depth (0 = top level / device container)
    key: str            # FourCC as text
    typ: int            # type byte
    struct_size: int    # bytes per sample
    repeat: int         # number of samples (or payload bytes for NEST)
    offset: int         # offset of the payload inside the buffer
    length: int         # payload length in bytes (before 4-byte padding)

    @property
    def end(self) -> int:
        return self.offset + self.length

    @property
    def type_char(self) -> str:
        return chr(self.typ) if 32 <= self.typ < 127 else f"\\x{self.typ:02x}"

    @property
    def is_compressed(self) -> bool:
        return self.typ == TYPE_COMPRESSED


def iter_elements(buf: bytes, start: int = 0, end: Optional[int] = None,
                  level: int = 0) -> Iterator[Element]:
    """Yield every GPMF element in ``buf`` in document (pre-)order."""
    if end is None:
        end = len(buf)
    pos = start
    while pos + 8 <= end:
        key = buf[pos:pos + 4]
        typ = buf[pos + 4]
        struct_size = buf[pos + 5]
        repeat = struct.unpack_from(">H", buf, pos + 6)[0]

        # 4x zero bytes = padding / end marker.
        if key == b"\x00\x00\x00\x00":
            pos += 4
            continue

        if typ not in _KNOWN_BYTES:
            # Unknown type: stop parsing this level rather than desync.
            raise GpmfError(
                f"unknown GPMF type 0x{typ:02x} for key "
                f"{key.decode('ascii', 'replace')!r} at offset {pos}")

        if typ == TYPE_NEST:
            payload_len = repeat
            yield Element(level, key.decode('ascii', 'replace'), typ,
                          struct_size, repeat, pos + 8, payload_len)
            yield from iter_elements(buf, pos + 8, pos + 8 + payload_len,
                                     level + 1)
        else:
            payload_len = struct_size * repeat
            yield Element(level, key.decode('ascii', 'replace'), typ,
                          struct_size, repeat, pos + 8, payload_len)

        pos += 8 + payload_len
        pos = (pos + 3) & ~3  # elements are 4-byte aligned


def decode(buf: bytes, elem: Element) -> Union[List, bytes]:
    """Decode an element's payload.

    Numeric types return a list of numbers; string/fourcc/UTC/GUID types
    return raw ``bytes``.  Compressed payloads return ``b""`` (unsupported).
    """
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


def scaled(values: List[float], scale: Optional[List[float]]) -> List[float]:
    """Apply a GPMF SCAL divisor per position (``value / scale``)."""
    if not scale:
        return list(values)
    out = []
    for i, v in enumerate(values):
        s = scale[i % len(scale)]
        out.append(v / s if s else v)
    return out


# =============================================================================
# 3. GPS - turn raw GPMF bytes into a time-stamped, distance-annotated track
# =============================================================================
# The GoPro ``GPS5`` stream carries latitude, longitude, altitude, 2-D speed
# and 3-D speed at ~18 Hz (18 or 19 samples per one-second GPMF block).  Each
# block also carries a ``GPSU`` UTC timestamp and a ``STMP`` microsecond clock
# reading for its first sample.
#
# Timestamps
# ----------
# ``GPSU`` is the absolute UTC time but is only emitted once per block and is
# occasionally stale (e.g. the very first block after a cold start).  ``STMP``
# is a smooth, monotonic microsecond clock.  For every block we therefore
# compute ``offset = GPSU_epoch_us - STMP_us`` and take the *median* offset as
# the clock correction, which rejects the occasional stale ``GPSU`` while
# keeping sub-100 ms absolute accuracy.  A sample's time is then
# ``STMP_sample + median_offset``.
#
# Distance
# --------
# Latitude/longitude are lightly smoothed with a moving average to remove
# position noise, then consecutive samples are joined with the haversine
# great-circle distance.  Samples without a usable fix (no 2-D/3-D fix, an
# out-of-range or ``(0, 0)`` position) are discarded before smoothing, so a
# stray cold-start fix cannot drag the track.  A step only contributes distance
# while the camera is actually moving: the per-sample Doppler speed is used as a
# stationary gate (``stationary_speed``), which stops a parked camera from
# integrating GPS jitter into phantom distance.  When a file carries no speed, a
# raw step smaller than ``jitter_m`` is treated as noise instead.

GPS5_SCALE = [1e7, 1e7, 1e3, 1e3, 1e2]  # lat, lon, alt(m), 2D spd, 3D spd


class GpsError(Exception):
    """Raised when no usable GPS data can be extracted."""


@dataclass
class GpsSample:
    utc: Optional[_dt.datetime]
    stmp_us: int
    lat: float
    lon: float
    alt: Optional[float]
    speed2d: float
    speed3d: float
    fix: Optional[int] = None
    dop: Optional[float] = None

    @property
    def valid_fix(self) -> bool:
        # GPSF: 0/1 = no fix, 2 = 2-D, 3 = 3-D.  Accept 2-D and 3-D.
        fix_ok = self.fix is None or self.fix >= 2
        pos_ok = (-90.0 <= self.lat <= 90.0 and -180.0 <= self.lon <= 180.0
                  and not (self.lat == 0.0 and self.lon == 0.0))
        return bool(fix_ok and pos_ok)


@dataclass
class SelectedPoint:
    """A frame chosen at a fixed distance interval along the track."""
    index: int                 # 1-based sequence
    utc: _dt.datetime
    video_time: float          # seconds from the first frame of the video
    lat: float
    lon: float
    alt: Optional[float]
    heading: Optional[float]   # degrees clockwise from north
    distance: float            # cumulative track distance (m)


@dataclass
class Track:
    samples: List[GpsSample]
    times: List[Optional[_dt.datetime]]
    slat: List[float]          # smoothed latitudes
    slon: List[float]          # smoothed longitudes
    cumdist: List[float]       # cumulative distance (m), same length
    total_distance: float
    start_utc: _dt.datetime
    end_utc: _dt.datetime
    has_speed: bool = False
    offsets_spread_s: float = field(default=0.0)


def _parse_gpsu(text: str) -> _dt.datetime:
    """Parse GPMF ``U`` time ``yymmddhhmmss.sss`` as UTC."""
    yy = int(text[0:2]); mo = int(text[2:4]); dd = int(text[4:6])
    hh = int(text[6:8]); mi = int(text[8:10]); sec = float(text[10:])
    whole = int(sec)
    frac_us = int(round((sec - whole) * 1_000_000))
    if frac_us >= 1_000_000:  # guard against round-up
        whole += 1
        frac_us -= 1_000_000
    return _dt.datetime(2000 + yy, mo, dd, hh, mi, whole, frac_us,
                        tzinfo=_dt.timezone.utc)


def _epoch_us(dt: _dt.datetime) -> int:
    return int(round(dt.timestamp() * 1_000_000))


@dataclass
class _Block:
    stmp_us: Optional[int]
    utc: Optional[_dt.datetime]
    scale: Optional[List[float]]
    fix: Optional[int]
    dop: Optional[float]
    samples: List[tuple]  # raw 5-tuples


def _collect_blocks(buf: bytes) -> List[_Block]:
    """Walk the GPMF tree and pull out every GPS5 block.

    ``STMP``/``TSMP`` appear *before* ``STNM`` inside every stream, so
    elements are accumulated per stream and only kept once the stream is
    known to be a GPS stream.
    """
    blocks: List[_Block] = []

    def flush(s: Optional[dict]) -> None:
        if s and s.get("is_gps") and s.get("samples"):
            blocks.append(_Block(s.get("stmp"), s.get("utc"), s.get("scale"),
                                 s.get("fix"), s.get("dop"), s["samples"]))

    cur: Optional[dict] = None
    for e in iter_elements(buf):
        if e.level == 0 and e.key == "DEVC":
            flush(cur)
            cur = None
        elif e.level == 1 and e.key == "STRM":
            flush(cur)
            cur = {"stmp": None, "utc": None, "scale": None, "fix": None,
                   "dop": None, "samples": [], "is_gps": False}
        elif e.level == 2 and cur is not None:
            if e.key == "STMP":
                cur["stmp"] = decode(buf, e)[0]
            elif e.key == "STNM":
                nm = as_text(buf, e)
                cur["is_gps"] = bool(nm and "GPS" in nm)
            elif cur["is_gps"]:
                if e.key == "SCAL":
                    cur["scale"] = decode(buf, e)
                elif e.key == "GPSU":
                    txt = as_text(buf, e)
                    if len(txt) >= 12:
                        try:
                            cur["utc"] = _parse_gpsu(txt)
                        except ValueError:
                            pass
                elif e.key == "GPSF":
                    cur["fix"] = decode(buf, e)[0]
                elif e.key == "GPSP":
                    cur["dop"] = decode(buf, e)[0] / 100.0
                elif e.key == "GPS5":
                    if e.is_compressed:
                        raise GpsError("GPS5 stream is Huffman-compressed "
                                       "(QUAN enabled); not supported")
                    vals = decode(buf, e)
                    cur["samples"] = [tuple(vals[i:i + 5])
                                      for i in range(0, len(vals) - 4, 5)]
    flush(cur)
    return blocks


def extract_track(buf: bytes, *, stationary_speed: float = 0.3,
                  smooth: int = 9, jitter_m: float = 0.5) -> Track:
    """Parse a raw gpmd stream into a :class:`Track`."""
    blocks = _collect_blocks(buf)
    if not blocks:
        raise GpsError("no GPS5 data found in GPMF stream")

    scale = next((b.scale for b in blocks if b.scale), GPS5_SCALE)
    scale = (scale + GPS5_SCALE)[:5]

    # --- clock offset (GPSU - STMP), robust against stale GPSU ----------
    offsets = [_epoch_us(b.utc) - b.stmp_us
               for b in blocks if b.stmp_us is not None and b.utc is not None]
    median_offset = int(statistics.median(offsets)) if offsets else 0
    spread = (max(offsets) - min(offsets)) / 1e6 if len(offsets) > 1 else 0.0

    # --- per-sample absolute time ---------------------------------------
    samples: List[GpsSample] = []
    have_stmp = all(b.stmp_us is not None for b in blocks)
    prev_stmp = None
    for bi, b in enumerate(blocks):
        n = len(b.samples)
        if n == 0:
            prev_stmp = b.stmp_us
            continue
        if have_stmp:
            nxt = blocks[bi + 1].stmp_us if bi + 1 < len(blocks) else None
            if nxt is not None:
                period = (nxt - b.stmp_us) / n
            elif prev_stmp is not None:
                period = (b.stmp_us - prev_stmp) / n
            else:
                period = 1_000_000 / 18.0
            prev_stmp = b.stmp_us
            for i, s in enumerate(b.samples):
                stmp = int(b.stmp_us + i * period)
                utc = _dt.datetime.fromtimestamp(
                    (stmp + median_offset) / 1e6, _dt.timezone.utc)
                samples.append(_make_sample(s, scale, stmp, utc, b))
        else:
            base = _epoch_us(b.utc) if b.utc else 0
            nxt = next((_epoch_us(nb.utc) for nb in blocks[bi + 1:]
                        if nb.utc is not None), None)
            step = ((nxt - base) / n) if (nxt and base) else 1_000_000 / 18.0
            for i, s in enumerate(b.samples):
                us = int(base + i * step)
                utc = (_dt.datetime.fromtimestamp(us / 1e6, _dt.timezone.utc)
                       if base else None)
                samples.append(_make_sample(s, scale, us, utc, b))

    samples.sort(key=lambda s: s.stmp_us)

    # Drop samples without a usable fix (bad/absent GPSF, out-of-range or
    # (0,0) positions) *before* smoothing and distance integration, so a
    # stray invalid sample cannot drag the smoothed track towards it.
    samples = [s for s in samples if s.valid_fix]
    if not samples:
        raise GpsError("no GPS sample has a valid fix")

    times = [s.utc for s in samples]
    valid_times = [t for t in times if t is not None]
    if not valid_times:
        raise GpsError("GPS samples have no usable timestamps")

    has_speed = any(s.speed2d > 0.0 for s in samples)
    slat = _smooth([s.lat for s in samples], smooth)
    slon = _smooth([s.lon for s in samples], smooth)
    cum = cumulative_distance(samples, slat, slon,
                              stationary_speed=stationary_speed,
                              jitter_m=jitter_m, has_speed=has_speed)

    return Track(samples=samples, times=times, slat=slat, slon=slon,
                 cumdist=cum, total_distance=cum[-1] if cum else 0.0,
                 start_utc=valid_times[0], end_utc=valid_times[-1],
                 has_speed=has_speed, offsets_spread_s=spread)


def _make_sample(raw, scale, stmp, utc, block) -> GpsSample:
    lat, lon, alt, sp2, sp3 = (raw[i] / scale[i] for i in range(5))
    return GpsSample(utc=utc, stmp_us=stmp, lat=lat, lon=lon, alt=alt,
                     speed2d=sp2, speed3d=sp3, fix=block.fix, dop=block.dop)


def _smooth(values: Sequence[float], window: int) -> List[float]:
    """Centred moving average with a shrinking window at the edges."""
    if window <= 1 or len(values) < 3:
        return list(values)
    half = window // 2
    prefix = [0.0]
    for v in values:
        prefix.append(prefix[-1] + v)
    out = []
    n = len(values)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out.append((prefix[hi] - prefix[lo]) / (hi - lo))
    return out


def cumulative_distance(samples: Sequence[GpsSample], slat: Sequence[float],
                        slon: Sequence[float], *,
                        stationary_speed: float = 0.3,
                        jitter_m: float = 0.5,
                        has_speed: bool = True) -> List[float]:
    """Cumulative along-track distance (m) on the smoothed positions."""
    cum = [0.0]
    for i in range(len(samples) - 1):
        a, b = samples[i], samples[i + 1]
        if has_speed:
            speed = 0.5 * (a.speed2d + b.speed2d)
            moving = speed >= stationary_speed
        else:
            moving = haversine_m(a.lat, a.lon, b.lat, b.lon) >= jitter_m
        d = haversine_m(slat[i], slon[i], slat[i + 1], slon[i + 1]) if moving else 0.0
        cum.append(cum[-1] + d)
    return cum


def _interp_time(track: Track, i: int, frac: float) -> _dt.datetime:
    t0 = track.times[i]
    t1 = track.times[min(i + 1, len(track.times) - 1)]
    if t0 is None:
        return t1
    if t1 is None:
        return t0
    return t0 + (t1 - t0) * frac


def _heading_at(track: Track, i: int, window: int = 4,
                offset: float = 0.0) -> Optional[float]:
    """Course over ground around sample ``i`` (centred window)."""
    lo = max(0, i - window)
    hi = min(len(track.samples) - 1, i + window)
    if hi <= lo:
        hi = min(len(track.samples) - 1, lo + 1)
    if hi == lo:
        return None
    d = haversine_m(track.slat[lo], track.slon[lo],
                    track.slat[hi], track.slon[hi])
    if d < 0.1:  # essentially stationary -> heading undefined
        return None
    return (initial_bearing_deg(track.slat[lo], track.slon[lo],
                                track.slat[hi], track.slon[hi])
            + offset) % 360.0


def select_by_distance(track: Track, interval_m: float, *,
                       start_distance: float = 0.0,
                       first_point: bool = True,
                       heading_offset: float = 0.0,
                       heading_window: int = 4,
                       video_time_origin: Optional[_dt.datetime] = None,
                       max_points: Optional[int] = None) -> List[SelectedPoint]:
    """Choose frame locations every ``interval_m`` metres along the track.

    The first point sits at the first *valid* fix (the track origin) when
    ``first_point`` is true; subsequent points fall at
    ``first_valid + start_distance + k * interval_m``.  Position, altitude and
    time at each chosen distance are linearly interpolated between the two
    bracketing GPS samples.
    """
    if interval_m <= 0:
        raise ValueError("interval must be > 0")

    origin = video_time_origin or next(t for t in track.times if t is not None)
    valid = [i for i, s in enumerate(track.samples) if s.valid_fix]
    if not valid:
        raise GpsError("no GPS sample has a valid fix")
    first_valid = valid[0]
    d0 = track.cumdist[first_valid]

    targets: List[float] = []
    if first_point:
        targets.append(d0)
    d = max(0.0, d0 + start_distance)
    if first_point:
        d = max(d, d0 + interval_m)
    while d <= track.total_distance + 1e-9:
        targets.append(d)
        d += interval_m

    points: List[SelectedPoint] = []
    j = 0
    for k, target in enumerate(targets):
        if max_points is not None and len(points) >= max_points:
            break
        while j + 1 < len(track.cumdist) and track.cumdist[j + 1] < target:
            j += 1
        i = min(j, len(track.samples) - 1)
        nxt = min(i + 1, len(track.samples) - 1)
        span = track.cumdist[nxt] - track.cumdist[i]
        frac = 0.0 if span <= 0 else (target - track.cumdist[i]) / span
        frac = min(max(frac, 0.0), 1.0)

        lat = track.slat[i] + (track.slat[nxt] - track.slat[i]) * frac
        lon = track.slon[i] + (track.slon[nxt] - track.slon[i]) * frac
        a_alt = track.samples[i].alt
        b_alt = track.samples[nxt].alt
        if a_alt is not None and b_alt is not None:
            alt = a_alt + (b_alt - a_alt) * frac
        else:
            alt = a_alt if a_alt is not None else b_alt
        utc = _interp_time(track, i, frac)
        heading = _heading_at(track, i, window=heading_window,
                              offset=heading_offset)
        points.append(SelectedPoint(
            index=k + 1, utc=utc, video_time=(utc - origin).total_seconds(),
            lat=lat, lon=lon, alt=alt, heading=heading, distance=target))
    return points


def _xml_escape(text: str) -> str:
    return (text.replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;"))


def to_gpx(track: Track, name: str = "track") -> str:
    """Serialise the GPS track as a GPX 1.1 document (UTC timestamps)."""
    lines = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<gpx version="1.1" creator="max360-v2i"',
             '     xmlns="http://www.topografix.com/GPX/1/1">',
             f'  <metadata><name>{_xml_escape(name)}</name></metadata>',
             f'  <trk><name>{_xml_escape(name)}</name><trkseg>']
    for s in track.samples:
        if not s.valid_fix or s.utc is None:
            continue
        t = s.utc.astimezone(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
        ele = f"<ele>{s.alt:.2f}</ele>" if s.alt is not None else ""
        lines.append(f'    <trkpt lat="{s.lat:.8f}" lon="{s.lon:.8f}">'
                     f'{ele}<time>{t}</time>'
                     f'<speed>{s.speed2d:.2f}</speed></trkpt>')
    lines += ['  </trkseg></trk>', '</gpx>', '']
    return "\n".join(lines)


# =============================================================================
# 4. ffmpeg - all interaction with the external ``ffmpeg`` / ``ffprobe`` binaries
# =============================================================================
# GoPro MAX ``.360`` files store the immersive video as **two Equi-Angular
# Cubemap (EAC) hemisphere tracks** (4096x1344 HEVC each), not as dual-fisheye
# and not as equirectangular.  To obtain a normal 2:1 equirectangular frame we
# stitch the three EAC faces of both hemisphere tracks back together and let
# ffmpeg's ``v360`` filter reproject ``eac -> equirect``.
#
# The stitching filter graph is a faithful port of the one used by
# https://github.com/baasitsharief/gopro-to-mp4 (``gopro_stitch.sh``), which
# cross-fades the 64-pixel overlaps between EAC faces.

FFMPEG = os.environ.get("MAX360_FFMPEG", "ffmpeg")
FFPROBE = os.environ.get("MAX360_FFPROBE", "ffprobe")

HEMI_W, HEMI_H = 4096, 1344  # native GoPro MAX hemisphere track size
DEFAULT_WIDTH, DEFAULT_HEIGHT = 5376, 2688  # native 2:1 equirect output


class BackendError(Exception):
    """Raised when ffmpeg/ffprobe fail or the file layout is unsupported."""


def check_tools() -> None:
    for tool in (FFMPEG, FFPROBE):
        if shutil.which(tool) is None:
            raise BackendError(
                f"required program {tool!r} not found on PATH "
                "(install ffmpeg, or set MAX360_FFMPEG / MAX360_FFPROBE)")


def _run(cmd: Sequence[str], *, capture: bool = True) -> bytes:
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE)
    except FileNotFoundError as exc:  # pragma: no cover - checked earlier
        raise BackendError(str(exc)) from exc
    if proc.returncode != 0:
        tail = proc.stderr.decode("utf-8", "replace").strip().splitlines()
        tail = " | ".join(tail[-6:]) if tail else ""
        raise BackendError(
            f"command failed ({proc.returncode}): {' '.join(cmd[:6])} ... {tail}")
    return proc.stdout if capture else b""


@dataclass
class VideoInfo:
    path: str
    duration: float
    fps: float
    frame_count: int
    hemi_a: int            # stream index of the "top" hemisphere track
    hemi_b: int            # stream index of the "bottom" hemisphere track
    gpmd: Optional[int]    # stream index of the GPMF telemetry track
    width: int             # native hemisphere width
    height: int            # native hemisphere height
    creation_time: Optional[str] = None
    location: Optional[str] = None
    firmware: Optional[str] = None


def _parse_rate(rate: str) -> float:
    if "/" in rate:
        num, den = rate.split("/")
        den = float(den)
        return float(num) / den if den else 0.0
    return float(rate)


def probe(path: str) -> VideoInfo:
    """Inspect a .360 file and describe its hemisphere + telemetry layout."""
    out = _run([FFPROBE, "-v", "error", "-print_format", "json",
                "-show_streams", "-show_format", path])
    data = json.loads(out.decode("utf-8", "replace"))
    streams = data.get("streams", [])
    fmt = data.get("format", {})

    videos = [s for s in streams if s.get("codec_type") == "video"]
    if len(videos) < 2:
        raise BackendError(
            f"{os.path.basename(path)}: expected 2 video tracks (MAX EAC "
            f"hemispheres), found {len(videos)} - not a GoPro MAX .360 file?")
    # The two hemisphere tracks are the two equally-sized video streams.
    videos.sort(key=lambda s: (s.get("width", 0) * s.get("height", 0),
                               s.get("index")))
    hemi_a, hemi_b = videos[0], videos[1]

    gpmd = None
    for s in streams:
        if s.get("codec_type") == "data" and \
                s.get("codec_tag_string") == "gpmd":
            gpmd = s.get("index")
            break

    duration = float(fmt.get("duration") or hemi_a.get("duration") or 0.0)
    fps = _parse_rate(str(hemi_a.get("r_frame_rate", "0/1"))) or 30000.0 / 1001.0
    counts = [int(s["nb_frames"]) for s in (hemi_a, hemi_b)
              if str(s.get("nb_frames", "")).isdigit()]
    frame_count = min(counts) if counts else int(round(duration * fps))
    tags = fmt.get("tags", {})

    return VideoInfo(
        path=path, duration=duration, fps=fps, frame_count=frame_count,
        hemi_a=hemi_a["index"], hemi_b=hemi_b["index"], gpmd=gpmd,
        width=int(hemi_a.get("width", HEMI_W)),
        height=int(hemi_a.get("height", HEMI_H)),
        creation_time=tags.get("creation_time"),
        location=tags.get("location"),
        firmware=tags.get("firmware"))


def extract_gpmd(path: str, gpmd_index: int) -> bytes:
    """Copy the raw ``gpmd`` payload out of the container."""
    return _run([FFMPEG, "-v", "error", "-i", path,
                 "-map", f"0:{gpmd_index}", "-c", "copy", "-f", "data", "-"])


def _geq() -> str:
    n = 64
    expr = (f"if(between(X, 0, {n - 1}), "
            f"(p((X+{n}),Y)*(((X+1))/{n + 1}))"
            f"+(p(X,Y)*(({n + 1}-((X+1)))/{n + 1})), p(X,Y))")
    return f"geq=lum='{expr}':cb='{expr}':cr='{expr}'"


def build_filter(a: int, b: int, width: int, height: int,
                 select: Optional[str] = None,
                 interp: str = "cubic") -> str:
    """Return an ffmpeg ``filter_complex`` script string.

    ``a``/``b`` are the hemisphere stream indices.  When ``select`` is given
    it is applied to both hemisphere streams followed by a ``split`` so a
    single decode pass can emit many synchronised frames.
    """
    vh = HEMI_H
    lines: List[str] = []
    if select:
        lines.append(f"[0:{a}]select='{select}',split=7[a0][a1][a2][a3][a4][a5][a6]")
        lines.append(f"[0:{b}]select='{select}',split=7[b0][b1][b2][b3][b4][b5][b6]")
        A = [f"[a{i}]" for i in range(7)]
        B = [f"[b{i}]" for i in range(7)]
    else:
        A = [f"[0:{a}]"] * 7
        B = [f"[0:{b}]"] * 7

    def row(src: List[str], tag: str) -> List[str]:
        return [
            f"{src[0]}crop=624:{vh}:x=0:y=0,format=yuvj420p[{tag}L]",
            f"{src[1]}crop=128:{vh}:x=624:y=0,format=yuvj420p,{_geq()},"
            f"crop=64:{vh}:x=0:y=0,scale=96:{vh}[{tag}O]",
            f"{src[2]}crop=624:{vh}:x=752:y=0,format=yuvj420p[{tag}R]",
            f"{src[3]}crop=1344:1344:1376:0[{tag}C]",
            f"{src[4]}crop=624:{vh}:x=2720:y=0,format=yuvj420p[{tag}2L]",
            f"{src[5]}crop=128:{vh}:x=3344:y=0,format=yuvj420p,{_geq()},"
            f"crop=64:{vh}:x=0:y=0,scale=96:{vh}[{tag}2O]",
            f"{src[6]}crop=624:{vh}:x=3472:y=0,format=yuvj420p[{tag}2R]",
            f"[{tag}L][{tag}O]hstack[{tag}a]",
            f"[{tag}a][{tag}R]hstack[{tag}b]",
            f"[{tag}b][{tag}C]hstack[{tag}c]",
            f"[{tag}c][{tag}2L]hstack[{tag}d]",
            f"[{tag}d][{tag}2O]hstack[{tag}e]",
            f"[{tag}e][{tag}2R]hstack[{tag}ROW]",
        ]

    lines += row(A, "T")
    lines += row(B, "B")
    lines.append("[TROW][BROW]vstack[FULL_EAC]")
    lines.append(f"[FULL_EAC]v360=eac:equirect:interp={interp}"
                 f":w={width}:h={height}[OUT]")
    return ",\n".join(lines)


def select_expression(frames: Sequence[int]) -> str:
    """Build an ffmpeg ``select`` expression matching the given frame numbers."""
    return "+".join(f"eq(n\\,{int(n)})" for n in frames)


def stitch_frames(info: VideoInfo, frames: Sequence[int],
                  outputs: Sequence[str], *,
                  width: int, height: int, quality: int = 95,
                  image_format: str = "jpg",
                  interp: str = "cubic", threads: int = 0, workers: int = 1,
                  keep_filter: Optional[str] = None) -> List[str]:
    """Extract equirectangular images for ``frames`` by seeking per frame.

    ``outputs`` holds the destination path for each frame, in the same order
    and with the same length as ``frames``; parent directories are created as
    needed.  Each frame is extracted with an input-side seek plus
    ``-frames:v 1``, which is both faster (no full-file decode) and exactly
    frame-accurate.  The returned list equals ``outputs``.
    """
    outs = list(outputs)
    if not frames:
        return []
    if len(outs) != len(frames):
        raise BackendError(
            "number of output paths must match the number of frames")
    for out in outs:
        parent = os.path.dirname(out)
        if parent:
            os.makedirs(parent, exist_ok=True)

    script = build_filter(info.hemi_a, info.hemi_b, width, height, None, interp)
    fd, script_path = tempfile.mkstemp(prefix=".max360-", suffix=".filter")
    os.close(fd)
    with open(script_path, "w") as fh:
        fh.write(script)
    if keep_filter:
        parent = os.path.dirname(keep_filter)
        if parent:
            os.makedirs(parent, exist_ok=True)
        shutil.copyfile(script_path, keep_filter)

    pix_fmt = "rgb24" if image_format == "png" else "yuvj420p"
    quality_args = v2i.ffmpeg_quality_args(image_format, quality)

    def one(idx_frame):
        i, n = idx_frame
        t = n / info.fps
        cmd = [FFMPEG, "-v", "error", "-y", "-ss", f"{t:.9f}", "-i", info.path,
               "-filter_complex_script", script_path,
               "-map", "[OUT]", "-an", "-sn", "-dn", "-frames:v", "1",
               "-pix_fmt", pix_fmt]
        cmd += quality_args
        if threads:
            cmd += ["-threads", str(threads)]
        cmd.append(outputs[i])
        _run(cmd)

    try:
        if workers and workers > 1:
            with ThreadPoolExecutor(max_workers=workers) as pool:
                list(pool.map(one, enumerate(frames)))
        else:
            for item in enumerate(frames):
                one(item)
    finally:
        if os.path.exists(script_path):
            os.remove(script_path)

    return outs


# =============================================================================
# 5. Metadata - write EXIF + XMP-GPano tags into extracted frames (exiftool)
# =============================================================================
# The tag set mirrors what a GoPro MAX writes into its own equirectangular
# photos (see ``sample/``), extended with the GPS/XMP-GPano fields that
# Mapillary, Panoramax and Google Photos' 360 viewer expect.

EXIFTOOL = "exiftool"

TOOL_NAME = "max360-v2i"


class MetadataError(Exception):
    """Raised when exiftool is missing or fails."""


def check_exiftool() -> str:
    path = shutil.which(EXIFTOOL)
    if path is None:
        raise MetadataError(
            "required program 'exiftool' not found on PATH "
            "(install libimage-exiftool-perl)")
    return path


@dataclass
class FrameMeta:
    utc: _dt.datetime
    lat: float
    lon: float
    alt: Optional[float]
    heading: Optional[float]      # degrees clockwise from north
    width: int
    height: int
    make: str = "GoPro"
    model: str = "GoPro Max"
    software: Optional[str] = None
    tz_offset_hours: float = 0.0
    description: Optional[str] = None


def _fmt(dt: _dt.datetime) -> str:
    return dt.strftime("%Y:%m:%d %H:%M:%S")


def _fmt_sub(dt: _dt.datetime) -> str:
    return dt.strftime("%Y:%m:%d %H:%M:%S.%f")[:-3]


def build_args(meta: FrameMeta) -> List[str]:
    local = meta.utc + _dt.timedelta(hours=meta.tz_offset_hours)
    lat_ref = "North" if meta.lat >= 0 else "South"
    lon_ref = "East" if meta.lon >= 0 else "West"
    args: List[str] = []

    def add(tag: str, value) -> None:
        args.append(f"-{tag}={value}")

    # --- camera identity -------------------------------------------------
    add("Make", meta.make)
    add("Model", meta.model)
    if meta.software:
        add("Software", meta.software)
    add("ModifyDate", _fmt(local))
    add("DateTimeOriginal", _fmt(local))
    add("CreateDate", _fmt(local))
    add("Orientation#", 1)
    if meta.description:
        add("ImageDescription", meta.description)

    # --- EXIF GPS (UTC, as required by the EXIF spec) --------------------
    add("GPSVersionID", "2.3.0.0")
    add("GPSLatitude", abs(meta.lat))
    add("GPSLatitudeRef", lat_ref)
    add("GPSLongitude", abs(meta.lon))
    add("GPSLongitudeRef", lon_ref)
    if meta.alt is not None:
        add("GPSAltitude", abs(meta.alt))
        add("GPSAltitudeRef#", 0 if meta.alt >= 0 else 1)
    add("GPSDateStamp", meta.utc.strftime("%Y:%m:%d"))
    add("GPSTimeStamp", meta.utc.strftime("%H:%M:%S"))
    add("GPSDateTime", _fmt_sub(meta.utc) + "Z")
    if meta.heading is not None:
        add("GPSImgDirection", round(meta.heading, 2))
        add("GPSImgDirectionRef", "True North")

    # --- XMP GPS (decimal degrees; convenient for web tools) -------------
    add("XMP:GPSLatitude", meta.lat)
    add("XMP:GPSLongitude", meta.lon)
    if meta.alt is not None:
        add("XMP:GPSAltitude", abs(meta.alt))
        add("XMP:GPSAltitudeRef#", 0 if meta.alt >= 0 else 1)
    add("XMP:GPSDateTime", _fmt_sub(meta.utc) + "Z")
    if meta.heading is not None:
        add("XMP:GPSImgDirection", round(meta.heading, 2))
    add("XMP:CreateDate", _fmt_sub(local))
    add("XMP:ModifyDate", _fmt_sub(local))
    add("XMP:CreatorTool", TOOL_NAME)

    # --- XMP-GPano spherical panorama -------------------------------
    heading = meta.heading if meta.heading is not None else 0.0
    add("XMP-GPano:UsePanoramaViewer", "True")
    add("XMP-GPano:CaptureSoftware", meta.model)
    add("XMP-GPano:StitchingSoftware", TOOL_NAME)
    add("XMP-GPano:ProjectionType", "equirectangular")
    add("XMP-GPano:PoseHeadingDegrees", round(heading, 2))
    add("XMP-GPano:PosePitchDegrees", 0)
    add("XMP-GPano:PoseRollDegrees", 0)
    add("XMP-GPano:InitialViewHeadingDegrees", round(heading, 2))
    add("XMP-GPano:InitialViewPitchDegrees", 0)
    add("XMP-GPano:InitialViewRollDegrees", 0)
    add("XMP-GPano:InitialHorizontalFOVDegrees", 0)
    add("XMP-GPano:InitialVerticalFOVDegrees", 0)
    add("XMP-GPano:CroppedAreaLeftPixels", 0)
    add("XMP-GPano:CroppedAreaTopPixels", 0)
    add("XMP-GPano:CroppedAreaImageWidthPixels", meta.width)
    add("XMP-GPano:CroppedAreaImageHeightPixels", meta.height)
    add("XMP-GPano:FullPanoWidthPixels", meta.width)
    add("XMP-GPano:FullPanoHeightPixels", meta.height)
    add("XMP-GPano:FirstPhotoDate", _fmt(local))
    add("XMP-GPano:LastPhotoDate", _fmt(local))
    add("XMP-GPano:SourcePhotosCount", 2)
    return args


def write_frame(path: str, meta: FrameMeta) -> None:
    """Inject metadata into an extracted JPEG in place."""
    cmd = [EXIFTOOL, "-q", "-q", "-m", "-overwrite_original", "-P",
           *build_args(meta), path]
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if proc.returncode != 0:
        raise MetadataError(
            f"exiftool failed on {path}: "
            f"{proc.stderr.decode('utf-8', 'replace').strip()}")


# =============================================================================
# 6. CLI - argument parsing and orchestration
# =============================================================================


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="max360-v2i",
        description="Extract geotagged equirectangular still images from "
                    "GoPro MAX .360 videos at a fixed GPS distance interval. "
                    "Only files recorded by a GoPro MAX are processed; any "
                    "other video found under --input is skipped with a logged "
                    "reason.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=("Example:\n"
                "  max360-v2i --input video/ --output images/ --distance 5\n\n"
                "Only files recorded by a GoPro MAX are processed; any other "
                "video found under --input is skipped with a logged reason.\n\n"
                "Time base: file names and folders use camera-local time by "
                "default; use --time-base utc for UTC."))

    # Options shared by every *-v2i.py exporter (identical names/defaults/help).
    v2i.add_common_arguments(parser)
    # Options shared with the BlackVue exporter (GPMF/NMEA GPS handling).
    v2i.add_gps_filter_arguments(parser)

    # -- MAX-specific options ---------------------------------------------
    geometry = parser.add_argument_group("output geometry")
    geometry.add_argument("--width", type=int, default=DEFAULT_WIDTH,
                          help="output image width (2:1 with --height)")
    geometry.add_argument("--height", type=int, default=DEFAULT_HEIGHT,
                          help="output image height")
    geometry.add_argument("--interp", default="cubic",
                          choices=["cubic", "linear", "nearest", "lanczos"],
                          help="v360 reprojection interpolation")
    orientation = parser.add_argument_group("orientation")
    orientation.add_argument("--heading-window", type=int, default=4,
                             help="GPS samples either side used for heading")
    run = parser.add_argument_group("run control")
    run.add_argument("--save-telemetry", action="store_true",
                     help="also write the raw GPMF (.gpmd) and a GPX (.gpx) "
                          "track next to the images")
    run.add_argument("--keep-filter", action="store_true",
                     help="keep the generated ffmpeg filter script")

    # Backwards compatibility: the input directory used to be positional.
    for action in parser._actions:
        if action.dest == "input" and action.option_strings:
            action.required = False
    parser.add_argument("input_alias", nargs="?", default=None, metavar="INPUT",
                        help=argparse.SUPPRESS)
    return parser


def _frame_for(fps: float, video_time: float, frame_count: int) -> int:
    n = int(round(max(0.0, video_time) * fps))
    return max(0, min(n, frame_count - 1))


def _tz_offset_hours(args) -> float:
    """Local-time offset in hours for file names and EXIF.

    GoPro MAX stores no local wall-clock, so the offset defaults to UTC (0 h)
    and is only applied when the user passes ``--timezone`` (or the legacy
    ``--tz`` alias).  ``--time-base utc`` forces UTC.
    """
    if args.time_base == "utc":
        return 0.0
    seconds = v2i.parse_timezone(getattr(args, "timezone", None))
    return 0.0 if seconds is None else seconds / 3600.0


def _local(utc: _dt.datetime, tz_offset_hours: float) -> _dt.datetime:
    """Frame capture time in local time (UTC plus the timezone offset)."""
    return utc.astimezone(_dt.timezone.utc) + _dt.timedelta(hours=tz_offset_hours)


def _image_name(local: _dt.datetime, ext: str = ".jpg") -> str:
    """``YYYY-MM-DD_HH-MM-SS-mmm.ext`` in local time (GoPro-style)."""
    return (local.strftime("%Y-%m-%d_%H-%M-%S")
            + f"-{local.microsecond // 1000:03d}{ext}")


def _image_paths(out_dir: str, stem: str, points,
                 tz_offset_hours: float, ext: str = ".jpg") -> List[str]:
    """Destination paths grouped as ``<output>/<date>/<sequence>/<time>.ext``.

    ``date`` and the filename are the frame's local capture time; ``sequence``
    is the source video's stem.  Collisions (two frames in the same
    millisecond) get a ``-2`` / ``-3`` suffix.
    """
    paths: List[str] = []
    used = set()
    for pt in points:
        local = _local(pt.utc, tz_offset_hours)
        path = os.path.join(out_dir, local.strftime("%Y-%m-%d"), stem,
                            _image_name(local, ext))
        if path in used:
            base, _ext = os.path.splitext(path)
            k = 2
            while f"{base}-{k}{ext}" in used:
                k += 1
            path = f"{base}-{k}{ext}"
        used.add(path)
        paths.append(path)
    return paths


def process_video(path: str, args) -> dict:
    """Process one .360 file. Returns a per-file result summary dict."""
    name = os.path.basename(path)
    result = {"file": path, "points": [], "images": 0,
              "error": None, "total_distance": 0.0}

    LOG.info("%s", name)
    info = probe(path)
    LOG.debug("duration=%.2fs fps=%.4f frames=%d hemi=(%d,%d) gpmd=%s "
              "native=%dx%d", info.duration, info.fps, info.frame_count,
              info.hemi_a, info.hemi_b, info.gpmd, info.width, info.height)

    if info.gpmd is None:
        raise GpsError("no GPMF (gpmd) telemetry track in file")

    gpmd = extract_gpmd(path, info.gpmd)
    LOG.debug("gpmd bytes=%d", len(gpmd))
    track = extract_track(gpmd, stationary_speed=args.stationary_speed,
                          smooth=args.smooth, jitter_m=args.jitter)
    result["total_distance"] = track.total_distance
    n_valid = sum(1 for s in track.samples if s.valid_fix)
    LOG.debug("gps samples=%d valid=%d distance=%.1fm clock-spread=%.0fms",
              len(track.samples), n_valid, track.total_distance,
              track.offsets_spread_s * 1000)

    if n_valid == 0:
        raise GpsError("no GPS sample has a valid fix")

    points = select_by_distance(
        track, args.distance,
        start_distance=args.start_distance,
        heading_offset=args.heading_offset,
        heading_window=args.heading_window,
        max_points=args.max_images)
    LOG.info("  %d image(s) over %.1f m (%sZ -> %sZ)", len(points),
             track.total_distance,
             track.start_utc.astimezone(_dt.timezone.utc).strftime(
                 "%Y-%m-%d %H:%M:%S"),
             track.end_utc.astimezone(_dt.timezone.utc).strftime("%H:%M:%S"))

    # Map each point to a video frame; drop points that collide on a frame.
    planned = []
    seen = set()
    for pt in points:
        n = _frame_for(info.fps, pt.video_time, info.frame_count)
        if n in seen:
            LOG.debug("skip point %d: frame %d already used", pt.index, n)
            continue
        seen.add(n)
        planned.append((n, pt))

    tz_hours = _tz_offset_hours(args)
    ext = ".png" if args.format == "png" else ".jpg"
    stem = os.path.splitext(name)[0]
    outputs = _image_paths(args.output, stem, [pt for _, pt in planned],
                           tz_hours, ext)

    planned_rows = [{
        "index": pt.index,
        "frame": n,
        "video_time": round(pt.video_time, 3),
        "utc": pt.utc.astimezone(_dt.timezone.utc).isoformat(),
        "lat": round(pt.lat, 7), "lon": round(pt.lon, 7),
        "alt": None if pt.alt is None else round(pt.alt, 2),
        "heading": None if pt.heading is None else round(pt.heading, 2),
        "distance": round(pt.distance, 2),
    } for n, pt in planned]
    for r, p in zip(planned_rows, outputs):
        r["output"] = os.path.relpath(p, args.output)
    result["points"] = planned_rows

    if args.dry_run:
        for r in planned_rows:
            LOG.info("    #%4d  f=%5d  %sZ  %.6f,%.6f  alt=%s  hdg=%s  "
                     "d=%sm  -> %s", r["index"], r["frame"], r["utc"][11:23],
                     r["lat"], r["lon"], r["alt"], r["heading"], r["distance"],
                     r["output"])
        result["images"] = len(planned_rows)
        return result

    if args.save_telemetry:
        with open(os.path.join(args.output, stem + ".gpmd"), "wb") as fh:
            fh.write(gpmd)
        with open(os.path.join(args.output, stem + ".gpx"), "w") as fh:
            fh.write(to_gpx(track, stem))

    # Skip frames whose image already exists unless --overwrite was given.
    todo = []
    for (n, pt), out in zip(planned, outputs):
        if os.path.exists(out) and not args.overwrite:
            LOG.debug("exists, skipping: %s", os.path.relpath(out, args.output))
            continue
        todo.append((n, pt, out))

    written = 0
    if todo:
        frames = [n for n, _, _ in todo]
        dests = [out for _, _, out in todo]
        files = stitch_frames(
            info, frames, dests,
            width=args.width, height=args.height, quality=args.quality,
            image_format=args.format, interp=args.interp,
            threads=args.threads, workers=args.workers,
            keep_filter=(os.path.join(args.output, f"{stem}.filter")
                         if args.keep_filter else None))
        if len(files) != len(todo):
            raise BackendError(
                f"ffmpeg produced {len(files)} images but {len(todo)} were "
                "selected")
        if not args.no_exif:
            for (n, pt, _out), fpath in zip(todo, files):
                write_frame(fpath, FrameMeta(
                    utc=pt.utc.astimezone(_dt.timezone.utc),
                    lat=pt.lat, lon=pt.lon, alt=pt.alt, heading=pt.heading,
                    width=args.width, height=args.height,
                    make="GoPro", model="GoPro Max", software=info.firmware,
                    tz_offset_hours=tz_hours,
                    description=f"{name} (frame {n}, {pt.distance:.1f} m)"))
        written = len(files)

    result["images"] = written
    return result


def main(argv: Optional[List[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.input is None:
        args.input = args.input_alias
    if args.input is None:
        parser.error("--input is required (or give a positional input path)")
    v2i.validate_common_args(parser, args)
    v2i.setup_logging(v2i.log_level(args))

    try:
        check_tools()
        if not args.dry_run and not args.no_exif:
            check_exiftool()
    except (BackendError, MetadataError) as exc:
        LOG.error("%s", exc)
        return 2

    input_arg = os.path.abspath(args.input)
    output_dir = os.path.abspath(args.output)

    extensions = v2i.resolve_extensions(args)
    videos = v2i.discover_videos(input_arg, extensions, args.recursive)
    if not videos:
        LOG.error("no video files matching %s found under %s",
                  ",".join(extensions), input_arg)
        return 1

    # Keep only files this camera recorded; never touch another camera's video.
    accepted = []
    for video in videos:
        detection = v2i.detect_camera(video)
        reason = v2i.skip_reason(detection, v2i.CAMERA_MAX)
        if reason:
            LOG.warning("skipping %s: %s", os.path.basename(video), reason)
            continue
        LOG.debug("accepted %s", os.path.basename(video))
        accepted.append(video)
    if not accepted:
        LOG.error("no %s video found among %d candidate file(s)",
                  v2i.CAMERA_LABELS[v2i.CAMERA_MAX], len(videos))
        return 1

    LOG.info("found %d %s video(s) under %s", len(accepted),
             v2i.CAMERA_LABELS[v2i.CAMERA_MAX], input_arg)
    if not args.dry_run:
        os.makedirs(output_dir, exist_ok=True)

    results = []
    failures = 0
    for path in accepted:
        try:
            results.append(process_video(path, args))
        except (BackendError, GpsError, MetadataError) as exc:
            failures += 1
            LOG.error("%s: %s", os.path.basename(path), exc)
            results.append({"file": path, "error": str(exc), "images": 0,
                            "points": [], "total_distance": 0.0})
            if args.fail_fast:
                break

    total_images = sum(r.get("images", 0) for r in results)
    total_dist = sum(r.get("total_distance", 0.0) for r in results)
    LOG.info("done: %d image(s) from %d/%d video(s), %.1f m of track%s",
             total_images, len(results) - failures, len(accepted), total_dist,
             (", %d failure(s)" % failures) if failures else "")

    if args.manifest and not args.dry_run:
        with open(args.manifest, "w") as fh:
            json.dump({"generated": _dt.datetime.now(_dt.timezone.utc).isoformat(),
                       "camera": v2i.CAMERA_MAX,
                       "distance_interval_m": args.distance,
                       "output_dir": output_dir,
                       "videos": results}, fh, indent=2)
        LOG.info("manifest written to %s", args.manifest)

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
