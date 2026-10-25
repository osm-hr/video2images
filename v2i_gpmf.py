#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v2i_gpmf.py -- dependency-free reader for GoPro GPMF telemetry.

GPMF ("GoPro Metadata Format") is the nested key-length-value telemetry carried
in the ``gpmd`` track of GoPro MP4 / ``.360`` files.  This module is shared by
``max360-v2i.py`` and ``hero-v2i.py`` and exposes:

* :func:`iter_elements` / :func:`decode` -- the low-level KLV reader,
* :func:`read_device_name` -- the ``DVNM`` device name used for detection,
* :func:`parse_gpmf` -- GPS5/GPS9 time series plus IMU streams,
* :func:`extract_gpmf` -- copy the raw gpmd payload with ffmpeg.

The parser understands GPS5, GPS9, GPSF, GPSP, GPSU, STMP, MAGN, ACCL, GYRO,
GRAV, CORI, IORI, ORIN and arbitrary device/stream keys.  Only the Python
standard library is used.
"""

from __future__ import annotations

import datetime as dt
import statistics
import struct
import subprocess
from dataclasses import dataclass, field
from typing import Dict, Iterator, List, Optional, Sequence, Tuple, Union

FFMPEG = "ffmpeg"

# --- GPMF type tags ---------------------------------------------------------
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


def _build_gps_samples(blocks: List[_Block]) -> Tuple[List[dict], float]:
    gps_blocks = [b for b in blocks if _gps_rows(b)[0]]
    if not gps_blocks:
        return [], 0.0

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
    return samples, spread


def _build_imu_streams(blocks_by_name: Dict[str, List[_Block]]
                       ) -> Dict[str, List[dict]]:
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
                    utc = block.utc
                    if utc is not None and i:
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
    gps_samples, spread = _build_gps_samples(all_blocks)
    imu = _build_imu_streams(blocks)
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
