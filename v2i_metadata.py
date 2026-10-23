#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v2i_metadata.py -- shared EXIF/XMP metadata construction for video2images.

All four exporters build their exiftool argument list here, so timestamps,
GPS tags, accuracy, keywords and (for 360 video) the full XMP-GPano set cannot
diverge between cameras.

Policy implemented here:

* ``DateTimeOriginal``/``CreateDate``/``ModifyDate`` are camera-local when a
  local offset is known, otherwise UTC (never the host timezone).
* ``OffsetTimeOriginal``/``OffsetTime`` are written only when the offset is
  actually known.
* All GPS date/time tags are UTC.
* Keywords/``XMP:Subject`` are opt-in via ``--keywords``; provenance lives in
  description/comment fields instead.
* The complete XMP-GPano tag set is written only for panoramic (MAX) images.
"""

from __future__ import annotations

import datetime as dt
import re
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

from v2i_common import offset_string


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
