#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""max360-v2i.py -- extract geotagged equirectangular stills from GoPro MAX.

GoPro MAX ``.360`` files store immersive video as two Equi-Angular Cubemap
(EAC) hemisphere tracks.  This tool stitches the six 1344-pixel faces back into
one 4032x2688 EAC frame and reprojects it with ffmpeg's ``v360`` filter.  It
walks the embedded GPMF GPS track and writes a unique frame every
``--distance`` metres with EXIF/XMP-GPano metadata and GPX/GeoJSON tracks.

Only Python's standard library is used.  ``ffmpeg``, ``ffprobe`` and
``exiftool`` must be on ``PATH``; the optional ``max2sphere`` backend is used
only when explicitly requested (or as an ``auto`` fallback when ffmpeg is
missing).
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import logging
import math
import os
import shutil
import statistics
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import v2i_common as v2i
import v2i_gpmf
import v2i_metadata

SCRIPT_NAME = os.path.basename(os.path.realpath(__file__))
LOG = logging.getLogger("max360-v2i")

FFMPEG = os.environ.get("MAX360_FFMPEG", "ffmpeg")
FFPROBE = os.environ.get("MAX360_FFPROBE", "ffprobe")
EXIFTOOL = "exiftool"

HEMI_W, HEMI_H = 4096, 1344
DEFAULT_WIDTH, DEFAULT_HEIGHT = 5376, 2688


class Fatal(Exception):
    """A problem that stops the current file."""


class BackendError(Fatal):
    """ffmpeg/ffprobe/max2sphere failure."""


# ---------------------------------------------------------------------------
# Probing
# ---------------------------------------------------------------------------
@dataclass
class VideoInfo:
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


def _parse_rate(rate) -> float:
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


def _frame_count(stream: dict, duration: float, fps: float) -> int:
    try:
        count = int(stream.get("nb_frames"))
        if count > 0:
            return count
    except (TypeError, ValueError):
        pass
    return max(1, int(round(duration * fps))) if duration and fps else 1


def probe(path: str) -> VideoInfo:
    """Inspect a GoPro MAX file and describe its hemisphere/telemetry layout."""
    meta = v2i.probe_metadata(path, FFPROBE)
    videos = meta.video_streams_info()
    if not videos:
        raise BackendError("%s: no video stream" % os.path.basename(path))
    fmt_duration = meta.duration or 0.0
    creation = v2i.parse_utc_timestamp(meta.creation_time)

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
            raise BackendError(
                "%s: found %d video streams but no matching EAC hemisphere pair"
                % (os.path.basename(path), len(videos)))
        _area, hemi_a, hemi_b = max(pairs, key=lambda item: item[0])
        if hemi_a.get("index", 0) > hemi_b.get("index", 0):
            hemi_a, hemi_b = hemi_b, hemi_a
        width = int(hemi_a.get("width") or HEMI_W)
        height = int(hemi_a.get("height") or HEMI_H)
        fps = _parse_rate(hemi_a.get("avg_frame_rate")
                          or hemi_a.get("r_frame_rate")) or 30000.0 / 1001.0
        duration = float(hemi_a.get("duration") or fmt_duration or 0.0)
        count_a = _frame_count(hemi_a, duration, fps)
        count_b = _frame_count(hemi_b, duration, fps)
        frame_count = min(count_a, count_b)
        a_idx, b_idx = int(hemi_a["index"]), int(hemi_b["index"])
        already = False
    elif len(videos) == 1:
        stream = videos[0]
        width = int(stream.get("width") or 0)
        height = int(stream.get("height") or 0)
        aspect = (width / height) if height else 0.0
        if aspect < 1.85:
            raise BackendError(
                "%s: one %dx%d video stream is not an EAC hemisphere pair or "
                "an equirectangular frame" % (os.path.basename(path), width,
                                              height))
        hemi_a = int(stream["index"])
        hemi_b = None
        fps = _parse_rate(stream.get("avg_frame_rate")
                          or stream.get("r_frame_rate")) or 30000.0 / 1001.0
        duration = float(stream.get("duration") or fmt_duration or 0.0)
        frame_count = _frame_count(stream, duration, fps)
        a_idx, b_idx = hemi_a, None
        already = True
    else:
        raise BackendError("%s: no video stream" % os.path.basename(path))

    ordinals = {}
    for ordinal, stream in enumerate(videos):
        try:
            ordinals[int(stream["index"])] = ordinal
        except (KeyError, TypeError, ValueError):
            pass
    return VideoInfo(
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
def build_eac_filter(a: int, b: Optional[int], width: int, height: int,
                     select: Optional[str] = None,
                     interp: str = "cubic",
                     already_equirect: bool = False) -> str:
    """Build the ffmpeg filter graph for one EAC hemisphere pair.

    Each native 4096x1344 hemisphere contains three 1344-wide faces at x=0,
    x=1376 and x=2752 with 32-pixel gaps.  The gaps are dropped, the three
    faces are hstacked to 4032 pixels, the two hemispheres are vstacked to
    4032x2688, and v360 reprojects EAC -> equirectangular.
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
        raise BackendError("EAC stitching needs two hemisphere streams")
    if select:
        lines.append("[0:%d]select='%s'[ain]" % (a, select))
        lines.append("[0:%d]select='%s'[bin]" % (b, select))
    else:
        lines.append("[0:%d]null[ain]" % a)
        lines.append("[0:%d]null[bin]" % b)

    for tag, source in (("A", "[ain]"), ("B", "[bin]")):
        lines.append("%ssplit=3[%s0][%s1][%s2]" % (source, tag, tag, tag))
        for face, x in enumerate((0, 1376, 2752)):
            lines.append(
                "[%s%d]crop=1344:1344:%d:0,format=yuv420p[%sF%d]"
                % (tag, face, x, tag, face))
        lines.append("[%sF0][%sF1][%sF2]hstack=inputs=3[%sROW]"
                     % (tag, tag, tag, tag))
    lines.append("[AROW][BROW]vstack=inputs=2[eac]")
    lines.append("[eac]v360=eac:equirect:interp=%s:w=%d:h=%d[OUT]"
                 % (interp, width, height))
    return ";\n".join(lines)


def max2sphere_command(video: str, output: str, width: int, height: int,
                       fps: float) -> List[str]:
    """Command line for the optional max2sphere backend (experimental)."""
    return ["max2sphere", "-i", video, "-o", output,
            "-w", str(width), "-h", str(height), "-f", "%.6f" % fps]


def _run(cmd: Sequence[str]) -> None:
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE)
    except FileNotFoundError as exc:
        raise BackendError("required program not found: %s" % cmd[0]) from exc
    if proc.returncode != 0:
        tail = proc.stderr.decode("utf-8", "replace").strip().splitlines()
        raise BackendError("command failed (%d): %s"
                           % (proc.returncode, tail[-1] if tail else "unknown"))


def _resolve_backend(name: str) -> str:
    if name == "auto":
        if shutil.which(FFMPEG):
            return "ffmpeg"
        if shutil.which("max2sphere"):
            return "max2sphere"
        raise BackendError("neither ffmpeg nor max2sphere is on PATH")
    if name == "ffmpeg":
        if not shutil.which(FFMPEG):
            raise BackendError("ffmpeg is not on PATH")
        return "ffmpeg"
    if not shutil.which(name):
        raise BackendError("%s is not on PATH (install it or use "
                           "--stitch-backend ffmpeg)" % name)
    return name


def _copy_outputs(temp_files: Sequence[str], outputs: Sequence[str]) -> None:
    if len(temp_files) < len(outputs):
        raise BackendError("ffmpeg produced %d image(s) for %d selected frames"
                           % (len(temp_files), len(outputs)))
    for temp, dest in zip(temp_files, outputs):
        parent = os.path.dirname(dest)
        if parent:
            os.makedirs(parent, exist_ok=True)
        shutil.copyfile(temp, dest)


def stitch_ffmpeg(info: VideoInfo, frames: Sequence[int],
                  outputs: Sequence[str], *, width: int, height: int,
                  quality: int, image_format: str, interp: str,
                  threads: int, keep_filter: Optional[str]) -> None:
    """Extract selected frames through the EAC/v360 graph in one decode pass."""
    ordered_frames = sorted(int(f) for f in frames)
    if not ordered_frames:
        return
    # outputs correspond to the original (already increasing) choices
    by_frame = {int(frame): output for frame, output in zip(frames, outputs)}
    temp_dir = tempfile.mkdtemp(prefix="max360_")
    script_path = os.path.join(temp_dir, "stitch.filter")
    select = v2i.select_expression(ordered_frames)
    script = build_eac_filter(
        info.hemi_a, info.hemi_b, width, height, select=select,
        interp=interp, already_equirect=info.already_equirect)
    with open(script_path, "w") as fh:
        fh.write(script + "\n")
    if keep_filter:
        parent = os.path.dirname(keep_filter)
        if parent:
            os.makedirs(parent, exist_ok=True)
        shutil.copyfile(script_path, keep_filter)

    ext = "png" if image_format == "png" else "jpg"
    pattern = os.path.join(temp_dir, "img_%06d." + ext)
    pix_fmt = "rgb24" if image_format == "png" else "yuvj420p"
    cmd = [FFMPEG, "-v", "error", "-y", "-i", info.path,
           "-filter_complex_script", script_path, "-map", "[OUT]",
           "-an", "-sn", "-dn", "-vsync", "0", "-pix_fmt", pix_fmt]
    cmd += v2i.ffmpeg_quality_args(image_format, quality)
    if threads:
        cmd += ["-threads", str(threads)]
    cmd.append(pattern)
    try:
        _run(cmd)
        temp_files = sorted(glob.glob(os.path.join(temp_dir, "img_*." + ext)))
        if len(temp_files) != len(ordered_frames):
            raise BackendError(
                "ffmpeg produced %d image(s) for %d selected frames"
                % (len(temp_files), len(ordered_frames)))
        for frame, temp in zip(ordered_frames, temp_files):
            dest = by_frame[frame]
            parent = os.path.dirname(dest)
            if parent:
                os.makedirs(parent, exist_ok=True)
            shutil.copyfile(temp, dest)
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def stitch_max2sphere(info: VideoInfo, frames: Sequence[int],
                      outputs: Sequence[str], *, width: int, height: int,
                      quality: int, image_format: str, threads: int) -> None:
    """Stitch with the optional max2sphere binary, then extract frames."""
    temp_dir = tempfile.mkdtemp(prefix="max360_m2s_")
    video_out = os.path.join(temp_dir, "equirect.mp4")
    try:
        _run(max2sphere_command(info.path, video_out, width, height, info.fps))
        extracted = v2i.extract_frames_by_index(
            video_out, frames, os.path.join(temp_dir, "frames"),
            image_format=image_format, quality=quality, threads=threads)
        for frame, dest in zip(frames, outputs):
            temp = extracted.get(int(frame))
            if temp is None:
                raise BackendError("max2sphere/ffmpeg did not produce frame %d"
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
def _smooth(values: Sequence[float], window: int) -> List[float]:
    if window <= 1 or len(values) < 3:
        return list(values)
    half = window // 2
    prefix = [0.0]
    for value in values:
        prefix.append(prefix[-1] + value)
    out = []
    n = len(values)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out.append((prefix[hi] - prefix[lo]) / (hi - lo))
    return out


def build_track(path: str, info: VideoInfo, args) -> Tuple[v2i.Track, list]:
    if info.gpmd is None:
        raise BackendError("no GPMF (gpmd) telemetry track in file")
    payload = v2i_gpmf.extract_gpmf(path, info.gpmd, ffmpeg=FFMPEG)
    data = v2i_gpmf.parse_gpmf(payload, imu_heading=args.imu_heading)
    LOG.info("  GPMF device: %s; streams: %s", data.device_name,
             ", ".join(sorted(data.streams)))
    if not data.has_gps:
        raise BackendError("no GPS5/GPS9 data found in the GPMF stream")

    timed = [s for s in data.gps_samples if s.get("utc") is not None]
    if not timed:
        raise BackendError("GPMF GPS samples have no usable timestamps")
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
        v2i.apply_accuracy(item, args.uere)
        samples.append(item)
    if not samples:
        raise BackendError("no GPMF GPS sample has a valid fix")
    samples.sort(key=lambda s: s["t"])

    if args.smooth and args.smooth > 1:
        lats = _smooth([s["lat"] for s in samples], args.smooth)
        lons = _smooth([s["lon"] for s in samples], args.smooth)
        for sample, lat, lon in zip(samples, lats, lons):
            sample["lat"], sample["lon"] = lat, lon

    track = v2i.Track(samples, args.distance_mode)
    LOG.info("  telemetry: %d GPS sample(s), %.1f m of travel, "
             "clock spread %.0f ms", len(samples), track.length,
             data.clock_spread_s * 1000)
    return track, data.gps_samples


# ---------------------------------------------------------------------------
# Records
# ---------------------------------------------------------------------------
def _metadata_for(record, info: VideoInfo, args) -> v2i_metadata.FrameMetadata:
    return v2i_metadata.FrameMetadata(
        camera=v2i.CAMERA_LABELS[v2i.CAMERA_MAX], video=record["video"],
        sequence=record["sequence"], frame_index=record["frame_index"],
        pts=record["pts_seconds"], distance=record.get("distance_m"),
        script_name=SCRIPT_NAME, make=info.make, model=info.model,
        software=info.firmware, serial=info.serial,
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
        image_width=args.width, image_height=args.height, panorama=True,
        pose_heading=record.get("heading_deg"),
        keywords=v2i.parse_keywords(args.keywords),
    )


def _record(choice, track, args, info, tz_offset, tz_known, sequence,
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
    local = v2i.local_from_utc(utc, tz_offset) if tz_known else None
    if args.time_base == "utc" and utc is not None:
        naming = utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
    else:
        naming = local or (utc.astimezone(dt.timezone.utc).replace(tzinfo=None)
                           if utc is not None else None)
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
    if speed is None:
        speed = point.get("speed2d")
    return {
        "sequence": sequence, "video": video_name, "camera": v2i.CAMERA_MAX,
        "image": v2i.image_filename(naming, args.format),
        "frame_index": int(choice.frame_index), "pts_seconds": round(pts, 3),
        "datetime_local": local.isoformat() if local else None,
        "datetime_utc": v2i.fmt_utc(utc),
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
def process_video(path: str, sequence: str, args) -> dict:
    name = os.path.basename(path)
    result = {"video": name, "sequence": sequence, "images": 0,
              "planned": 0, "failed": False, "entries": [], "dir": None}
    LOG.info("--- %s -> %s", name, sequence)
    info = probe(path)
    LOG.info("  video: %dx%d %.4f fps, %d frame(s), %.2f s%s",
             info.width, info.height, info.fps, info.frame_count,
             info.duration, " (already equirectangular)"
             if info.already_equirect else " (EAC hemispheres)")

    ordinal = info.video_ordinals.get(info.hemi_a, 0)
    timeline = v2i.probe_frame_timeline(
        path, stream_ordinal=ordinal, fps_hint=info.fps,
        frame_count_hint=info.frame_count, duration_hint=info.duration,
        logger=LOG)
    if len(timeline) > info.frame_count:
        timeline = v2i.FrameTimeline(timeline.times[:info.frame_count],
                                     timeline.source, timeline.fps)
    LOG.info("  frame timeline: %d frame(s), source=%s",
             len(timeline), timeline.source)

    track, _gps = build_track(path, info, args)
    detected_offset = None       # GoPro MAX stores UTC, not local wall clock
    tz_offset, tz_known = v2i.resolve_timezone(
        args, detected_offset, camera=v2i.CAMERA_MAX, logger=LOG)

    choices, skipped = v2i.select_unique_frames(
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

    records = [_record(c, track, args, info, tz_offset, tz_known, sequence,
                       name) for c in choices]
    first_time = records[0]["naming_time"]
    sequence_dir = v2i.sequence_output_dir(
        args.output, v2i.CAMERA_MAX, sequence, first_time)
    used_paths = set()
    for record in records:
        record["path"] = v2i.unique_path(
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

    backend = _resolve_backend(args.stitch_backend)
    LOG.info("  stitching backend: %s", backend)
    if to_extract:
        outputs = [record["path"] for record in to_extract]
        frames = [record["frame_index"] for record in to_extract]
        keep_filter = (os.path.join(sequence_dir, sequence + ".filter")
                       if args.keep_filter else None)
        if backend == "max2sphere":
            stitch_max2sphere(info, frames, outputs, width=args.width,
                              height=args.height, quality=args.quality,
                              image_format=args.format, threads=args.threads)
        else:
            stitch_ffmpeg(info, frames, outputs, width=args.width,
                          height=args.height, quality=args.quality,
                          image_format=args.format, interp=args.interp,
                          threads=args.threads, keep_filter=keep_filter)
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
        pairs = [(record["path"], v2i_metadata.build_exiftool_args(
            _metadata_for(record, info, args))) for record in written]
        v2i.write_metadata(pairs)

    v2i.export_track_files(
        sequence_dir, sequence, track.samples, records,
        track_export=args.track_export, camera=v2i.CAMERA_MAX, video=name)
    result["entries"] = [{k: v for k, v in record.items()
                          if k not in ("point", "naming_time")}
                         for record in written]
    return result


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=SCRIPT_NAME,
        description="Extract geotagged equirectangular still images from GoPro "
                    "MAX .360 videos at a fixed GPS distance interval. Only "
                    "files recorded by a GoPro MAX are processed.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=("Example:\n"
                "  %(prog)s -i video/ -o images/ -d 5\n\n"
                "Time base: GoPro MAX stores UTC and no local wall clock, so "
                "file names/EXIF use UTC unless --timezone is given."))
    v2i.add_common_arguments(parser)
    v2i.add_gps_filter_arguments(parser)
    v2i.add_track_export_arguments(parser)

    geometry = parser.add_argument_group("output geometry")
    geometry.add_argument("--width", type=int, default=DEFAULT_WIDTH,
                          help="output image width (2:1 with --height)")
    geometry.add_argument("--height", type=int, default=DEFAULT_HEIGHT,
                          help="output image height")
    geometry.add_argument("--interp", default="cubic",
                          choices=["cubic", "linear", "nearest", "lanczos"],
                          help="v360 reprojection interpolation")
    geometry.add_argument("--stitch-backend",
                          choices=("auto", "ffmpeg", "max2sphere"),
                          default="auto",
                          help="EAC stitching backend (default: ffmpeg when "
                               "available)")
    orientation = parser.add_argument_group("orientation")
    orientation.add_argument("--heading-window", type=int, default=4,
                             help="GPS samples either side used for heading "
                                  "smoothing (compatibility option)")
    orientation.add_argument("--imu-heading", action="store_true",
                             help="derive compass heading from MAGN + GRAV "
                                  "instead of GPS course (experimental)")
    run = parser.add_argument_group("run control")
    run.add_argument("--keep-filter", action="store_true",
                     help="keep the generated ffmpeg filter graph")

    # Backwards compatibility: the input directory used to be positional.
    for action in parser._actions:
        if action.dest == "input" and action.option_strings:
            action.required = False
    parser.add_argument("input_alias", nargs="?", default=None, metavar="INPUT",
                        help=argparse.SUPPRESS)
    return parser


def validate(parser, args):
    v2i.validate_common_args(parser, args)
    if args.fallback_interval < 0:
        parser.error("--fallback-interval must not be negative")
    if args.width < 2 or args.height < 2:
        parser.error("--width/--height must be at least 2")
    if args.width % 2 or args.height % 2:
        parser.error("--width/--height must be even for yuv output")
    if args.heading_window < 1:
        parser.error("--heading-window must be at least 1")


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.input is None:
        args.input = args.input_alias
    if args.input is None:
        parser.error("--input is required (or give a positional input path)")
    validate(parser, args)
    v2i.setup_logging(v2i.log_level(args))

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

    extensions = v2i.resolve_extensions(args)
    videos = v2i.discover_videos(input_arg, extensions, args.recursive)
    if not videos:
        LOG.error("no video files matching %s found under %s",
                  ",".join(extensions), input_arg)
        return 1

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

    sequence_ids = v2i.build_sequence_ids(accepted, root)
    results = []
    failures = 0
    for path in accepted:
        try:
            result = process_video(path, sequence_ids[path], args)
            results.append(result)
            if result.get("failed"):
                failures += 1
                if args.fail_fast:
                    break
        except (Fatal, OSError) as exc:
            failures += 1
            LOG.error("%s: %s", os.path.basename(path), exc)
            results.append({"video": os.path.basename(path),
                            "sequence": sequence_ids[path], "images": 0,
                            "planned": 0, "failed": True, "entries": [],
                            "dir": None})
            if args.fail_fast:
                break

    total_images = sum(r.get("images", 0) for r in results)
    LOG.info("done: %d image(s) from %d/%d video(s)%s", total_images,
             len(results) - failures, len(accepted),
             (", %d failure(s)" % failures) if failures else "")

    if args.manifest and not args.dry_run:
        entries = [entry for result in results
                   for entry in result.get("entries", [])]
        with open(args.manifest, "w") as fh:
            json.dump({"generator": SCRIPT_NAME, "camera": v2i.CAMERA_MAX,
                       "camera_label": v2i.CAMERA_LABELS[v2i.CAMERA_MAX],
                       "distance_interval_m": args.distance,
                       "image_count": len(entries), "images": entries,
                       "videos": [{k: v for k, v in result.items()
                                   if k != "entries"} for result in results]},
                      fh, indent=2)
        LOG.info("manifest written to %s", args.manifest)
    return 1 if failures else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.stderr.write("\ninterrupted\n")
        sys.exit(130)
