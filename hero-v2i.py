#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hero-v2i.py -- extract geotagged stills from GoPro HERO videos.

GoPro HERO cameras record flat (non-360) MP4 files with optional GPMF
telemetry.  This exporter uses the embedded GPS fix (GPS5/GPS9), timestamps
(GPSU/STMP), DOP (GPSP) and optional IMU streams (MAGN/ACCL/GYRO/GRAV/...) to
pick a unique frame every ``--distance`` metres.

HERO files without GPS cannot produce distance-based captures.  They are
skipped by default.  ``--allow-missing-gps --fallback-interval SECONDS``
exports an explicitly unpositioned time-lapse instead; those images carry no
GPS tags and are marked as unpositioned in the manifest/GeoJSON.

Only the Python standard library is used.  ``ffmpeg``/``ffprobe``/``exiftool``
must be available on ``PATH``.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import logging
import os
import shutil
import sys
import tempfile
from dataclasses import dataclass
from typing import List, Optional

import v2i_common as v2i
import v2i_gpmf
import v2i_metadata

SCRIPT_NAME = os.path.basename(os.path.realpath(__file__))
LOG = logging.getLogger("hero-v2i")

FFMPEG = os.environ.get("HERO_FFMPEG", "ffmpeg")
FFPROBE = os.environ.get("HERO_FFPROBE", "ffprobe")
EXIFTOOL = "exiftool"


class Fatal(Exception):
    """A problem that stops processing the current file."""


@dataclass
class VideoInfo:
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


def probe(path: str) -> VideoInfo:
    meta = v2i.probe_metadata(path, FFPROBE)
    videos = meta.video_streams_info()
    if len(videos) != 1:
        raise Fatal("expected one flat video stream, found %d (use max360-v2i.py "
                    "for MAX EAC files)" % len(videos))
    stream = videos[0]
    width = int(stream.get("width") or 0)
    height = int(stream.get("height") or 0)
    fps = _parse_rate(stream.get("avg_frame_rate")
                      or stream.get("r_frame_rate")) or 30.0
    duration = float(stream.get("duration") or meta.duration or 0.0)
    try:
        frame_count = int(stream.get("nb_frames"))
    except (TypeError, ValueError):
        frame_count = 0
    if frame_count <= 0:
        frame_count = max(1, int(round(duration * fps))) if duration else 1
    return VideoInfo(
        path=path, duration=duration, fps=fps, frame_count=frame_count,
        gpmd=meta.gpmd_index, width=width, height=height,
        creation_time=v2i.parse_utc_timestamp(meta.creation_time),
        firmware=meta.format_tags.get("firmware"),
        serial=meta.format_tags.get("serial_number") or meta.format_tags.get("serial"),
    )


def _gpmf_index(info: VideoInfo, args) -> Optional[int]:
    if args.gpmf_stream is not None:
        return int(args.gpmf_stream)
    return info.gpmd


def build_track(info: VideoInfo, args):
    "Return ``(track, gpmf_data)`` or ``(None, None)`` when there is no GPS."
    index = _gpmf_index(info, args)
    if index is None:
        return None, None
    payload = v2i_gpmf.extract_gpmf(info.path, index, ffmpeg=FFMPEG)
    data = v2i_gpmf.parse_gpmf(payload, imu_heading=args.imu_heading)
    info.device_name = info.device_name or data.device_name
    LOG.info("  GPMF device: %s; streams: %s", data.device_name,
             ", ".join(sorted(data.streams)) or "none")
    if not data.has_gps:
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
        v2i.apply_accuracy(item, args.uere)
        samples.append(item)
    if not samples:
        return None, data
    samples.sort(key=lambda sample: sample["t"])
    track = v2i.Track(samples, args.distance_mode)
    LOG.info("  telemetry: %d GPS sample(s), %.1f m of travel",
             len(samples), track.length)
    return track, data


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


def _record(choice, args, info: VideoInfo, tz_offset: int, tz_known: bool,
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
    unpositioned = point.get("lat") is None
    return {
        "sequence": sequence, "video": os.path.basename(info.path),
        "camera": v2i.CAMERA_HERO,
        "image": v2i.image_filename(naming, args.format),
        "frame_index": int(choice.frame_index), "pts_seconds": round(pts, 3),
        "datetime_local": local.isoformat() if local else None,
        "datetime_utc": v2i.fmt_utc(utc) if utc else None,
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


def _metadata_for(record, info: VideoInfo, args):
    model = args.device_model or info.device_name or "GoPro HERO"
    return v2i_metadata.FrameMetadata(
        camera=v2i.CAMERA_LABELS[v2i.CAMERA_HERO], video=record["video"],
        sequence=record["sequence"], frame_index=record["frame_index"],
        pts=record["pts_seconds"], distance=record.get("distance_m"),
        script_name=SCRIPT_NAME, make="GoPro", model=model,
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
        image_width=info.width, image_height=info.height, panorama=False,
        keywords=v2i.parse_keywords(args.keywords),
        extra={"unpositioned": record.get("unpositioned")},
    )


def process_video(path: str, sequence: str, args) -> dict:
    name = os.path.basename(path)
    result = {"video": name, "sequence": sequence, "images": 0,
              "planned": 0, "failed": False, "entries": [], "dir": None}
    LOG.info("--- %s -> %s", name, sequence)
    info = probe(path)
    LOG.info("  video: %dx%d %.4f fps, %d frame(s), %.2f s", info.width,
             info.height, info.fps, info.frame_count, info.duration)
    timeline = v2i.probe_frame_timeline(
        path, fps_hint=info.fps, frame_count_hint=info.frame_count,
        duration_hint=info.duration, logger=LOG)
    if len(timeline) > info.frame_count:
        timeline = v2i.FrameTimeline(timeline.times[:info.frame_count],
                                     timeline.source, timeline.fps)
    LOG.info("  frame timeline: %d frame(s), source=%s", len(timeline),
             timeline.source)

    try:
        track, gpmf = build_track(info, args)
    except (Fatal, v2i_gpmf.GpmfError) as exc:
        LOG.error("  %s", exc)
        result["failed"] = True
        return result

    tz_offset, tz_known = v2i.resolve_timezone(
        args, None, camera=v2i.CAMERA_HERO, logger=LOG)

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
        choices = _unpositioned_choices(
            timeline, args.fallback_interval, info.duration, args.max_images)
        LOG.warning("  exporting %d unpositioned frame(s) on a %g s time grid",
                    len(choices), args.fallback_interval)
        skipped = []
    else:
        choices, skipped = v2i.select_unique_frames(
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

    records = [_record(choice, args, info, tz_offset, tz_known, sequence)
               for choice in choices]
    first_time = records[0]["naming_time"]
    sequence_dir = v2i.sequence_output_dir(
        args.output, v2i.CAMERA_HERO, sequence, first_time)
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
        extracted = v2i.extract_frames_by_index(
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
        pairs = [(record["path"], v2i_metadata.build_exiftool_args(
            _metadata_for(record, info, args))) for record in written]
        v2i.write_metadata(pairs)

    v2i.export_track_files(
        sequence_dir, sequence, track.samples if track else [], records,
        track_export=args.track_export, camera=v2i.CAMERA_HERO, video=name)
    result["entries"] = [{k: v for k, v in record.items()
                          if k not in ("point", "naming_time")}
                         for record in written]
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=SCRIPT_NAME,
        description="Extract geotagged still images from GoPro HERO videos at "
                    "a fixed GPS distance interval.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=("Example:\n  %(prog)s -i video/2026-07-14-hero -o images/ -d 5\n\n"
                "A HERO file without GPS is skipped unless both "
                "--allow-missing-gps and --fallback-interval are given."))
    v2i.add_common_arguments(parser)
    v2i.add_gps_filter_arguments(parser)
    v2i.add_track_export_arguments(parser)
    group = parser.add_argument_group("GoPro HERO telemetry")
    group.add_argument("--gpmf-stream", type=int, default=None, metavar="INDEX",
                       help="absolute ffmpeg stream index of the gpmd telemetry "
                            "track (default: auto-detect)")
    group.add_argument("--device-model", default=None, metavar="NAME",
                       help="override the EXIF camera model")
    group.add_argument("--allow-missing-gps", action="store_true",
                       help="export unpositioned time-lapse frames when the "
                            "video has no GPS (requires --fallback-interval)")
    group.add_argument("--imu-heading", action="store_true",
                       help="derive compass heading from MAGN + GRAV streams "
                            "instead of GPS course")
    return parser


def validate(parser, args):
    v2i.validate_common_args(parser, args)
    if args.fallback_interval < 0:
        parser.error("--fallback-interval must not be negative")
    if args.gpmf_stream is not None and args.gpmf_stream < 0:
        parser.error("--gpmf-stream must not be negative")
    if args.allow_missing_gps and args.fallback_interval <= 0:
        parser.error("--allow-missing-gps requires --fallback-interval SECONDS")


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate(parser, args)
    v2i.setup_logging(v2i.log_level(args))

    for tool in (FFMPEG, FFPROBE):
        if shutil.which(tool) is None:
            LOG.error("required program not found on PATH: %s", tool)
            return 2
    if not shutil.which(EXIFTOOL) and not args.no_exif:
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
        reason = v2i.skip_reason(detection, v2i.CAMERA_HERO)
        if reason:
            LOG.warning("skipping %s: %s", os.path.basename(video), reason)
            continue
        accepted.append(video)
    if not accepted:
        LOG.error("no %s video found among %d candidate file(s)",
                  v2i.CAMERA_LABELS[v2i.CAMERA_HERO], len(videos))
        return 1

    LOG.info("found %d %s video(s) under %s", len(accepted),
             v2i.CAMERA_LABELS[v2i.CAMERA_HERO], input_arg)
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
        except (Fatal, OSError, v2i_gpmf.GpmfError) as exc:
            failures += 1
            LOG.error("%s: %s", os.path.basename(path), exc)
            results.append({"video": os.path.basename(path),
                            "sequence": sequence_ids[path], "images": 0,
                            "planned": 0, "failed": True, "entries": [],
                            "dir": None})
            if args.fail_fast:
                break

    total = sum(result.get("images", 0) for result in results)
    LOG.info("done: %d image(s) from %d/%d video(s)%s", total,
             len(results) - failures, len(accepted),
             (", %d failure(s)" % failures) if failures else "")

    if args.manifest and not args.dry_run:
        entries = [entry for result in results
                   for entry in result.get("entries", [])]
        with open(args.manifest, "w") as fh:
            json.dump({"generator": SCRIPT_NAME, "camera": v2i.CAMERA_HERO,
                       "camera_label": v2i.CAMERA_LABELS[v2i.CAMERA_HERO],
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
