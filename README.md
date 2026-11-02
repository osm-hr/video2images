# video2images (`v2i.py`)

`v2i.py` turns action-camera footage into a distance-spaced, geotagged still-image sequence. It is a single-file, standard-library-only CLI that reads the camera's own telemetry, selects one unique frame per distance target, optionally levels the horizon from the IMU, and writes EXIF/XMP metadata plus GPX, GeoJSON and detailed JSON telemetry.

No separate per-camera modules, no database, no service dependencies. `ffmpeg`, `ffprobe` and `exiftool` are external tools; `max2sphere` is optional.

**Version:** `0.0.3` · **Python:** 3.9+ · **Validated with:** ffmpeg/ffprobe 5.1.9, exiftool 12.x

---

## Table of contents

1. [Executive overview](#1-executive-overview)
2. [Architecture](#2-architecture)
3. [Supported cameras and hardware](#3-supported-cameras-and-hardware)
4. [Raw telemetry ingestion](#4-raw-telemetry-ingestion)
5. [Configuration and parameters](#5-configuration-and-parameters)
6. [Processing internals](#6-processing-internals)
7. [Export profiles and formats](#7-export-profiles-and-formats)
8. [Operational guide](#8-operational-guide)
9. [Tests and verification](#9-tests-and-verification)
10. [Appendix](#10-appendix)

---

## 1. Executive overview

### What the pipeline does

1. **Discovers** candidate video files under an input file or directory.
2. **Identifies** the recording camera per file from container handlers, GPMF device names, stream layout and filename evidence.
3. **Groups** accepted files by camera and runs the matching pipeline in one pass.
4. **Probes** the container and reads a frame-accurate presentation timeline from real PTS values.
5. **Extracts telemetry** from embedded GPMF (`gpmd`), embedded NMEA (`free` atoms) or a sibling Garmin FIT file.
6. **Builds a time-indexed track**, interpolates positions and measures cumulative travel distance.
7. **Selects unique frames** at fixed distance intervals, never reusing a frame and never going backwards.
8. **Extracts stills** with accurate input seek, or stitches MAX 360 hemispheres to equirectangular in bounded batches.
9. **Optionally levels** the horizon using smoothed IMU roll/pitch from the camera.
10. **Writes metadata** (EXIF/XMP, GPS, provenance, GPano pose) with `exiftool`.
11. **Exports tracks and telemetry** as GPX, GeoJSON and two JSON files per sequence.
12. **Optionally writes a run manifest** covering every exported image and video summary.

### Design principles

| Principle | Implementation |
|---|---|
| Single entry point | All code lives in `v2i.py`|
| Standard library only | No third-party Python packages |
| Evidence-based detection | Ambiguous GoPro metadata is rejected with a reason, never guessed |
| Camera-authoritative clocks | GPS/FIT/clip clocks are used; the host timezone is never consulted |
| Deterministic selection | Targets are distance-based; frame indices strictly increase |
| Idempotent runs | Existing images are skipped unless `--overwrite` is passed |
| Explicit opt-in | Horizon leveling and keywords are off by default |
| Bounded resource use | MAX batches, parallel workers, temporary files cleaned up |

### Non-goals

- Video transcoding or re-encoding of source files.
- Lens-distortion, rolling-shutter or keystone correction.
- Uploading to Mapillary, Panoramax or any mapping service.
- Georeferencing without camera telemetry (the fallback exports unpositioned images only).
- Reading arbitrary third-party telemetry formats beyond GPMF, Garmin FIT and NMEA.

### Minimum capabilities

| Capability | Requirement |
|---|---|
| Runtime | Python 3.9 or newer |
| Video probing/extraction | `ffprobe` and `ffmpeg` on `PATH` |
| EXIF/XMP metadata | `exiftool` on `PATH` (optional; images still export without it) |
| 360 stitching | `ffmpeg` with `v360`, or `max2sphere` |
| Parallelism | `--workers N`; MAX leveling scales with CPU cores |

---

## 2. Architecture

### 2.1 End-to-end data flow

```text
input file or tree
      │
      │ discover_videos()
      ▼
candidate files (.mp4/.mov/.m4v/.360)
      │
      │ detect_camera()  ── ffprobe handlers + GPMF DVNM + stream layout + filename
      ▼
camera groups
      │
      │ process_group()  ── CAMERA_ORDER = BlackVue → MAX → VIRB → HERO
      ▼
per-video probe
      ├── frame timeline (ffprobe packet PTS, presentation order)
      ├── telemetry (GPMF / FIT / NMEA)
      └── time base (GPS clock, container clock, detected UTC offset)
      │
      │ Track(samples)  ── haversine cumulative distance
      ▼
plan_shots() ──► select_unique_frames()
      │
      ▼
FrameChoice[]  ── frame index, PTS, interpolated point, distance, error
      │
      ├── flat cameras: extract_frames_by_index()  (one accurate seek per frame)
      └── MAX 360: max360_stitch_ffmpeg() / max360_stitch_max2sphere()
      │
      │ optional horizon leveling (smoothed IMU roll/pitch)
      ▼
JPEG/PNG files
      │
      │ exiftool batch
      ▼
EXIF + XMP + GPano tags
      │
      │ export_track_files()
      ▼
<sequence>.gpx
<sequence>.geojson
<sequence>.telemetry.json
<sequence>.image-telemetry.json
      │
      │ --manifest
      ▼
run manifest JSON
```

### 2.2 Component responsibilities

| Layer | Key functions | Responsibility |
|---|---|---|
| Discovery | `discover_videos`, `resolve_extensions` | Resolve extensions and walk the input tree |
| Detection | `probe_metadata`, `detect_camera`, `detect_groups` | Identify camera per file and group accepted videos |
| Timeline | `probe_frame_timeline`, `_build_timeline_from_raw` | Build frame-index → PTS map from packet timestamps |
| Track | `Track`, `haversine`, `plan_shots` | Interpolate position/time and accumulate distance |
| Selection | `select_unique_frames`, `select_time_grid_frames` | Pick unique frames at distance targets or time-grid fallback |
| Extraction | `extract_frames_by_index`, `_extract_one` | Accurate per-frame seek and parallel ffmpeg workers |
| MAX 360 | `max360_probe`, `max360_build_eac_filter`, `max360_stitch_*` | EAC layout assembly, v360 reprojection, optional sphere rotation |
| Telemetry | `parse_gpmf`, `virb_Telemetry`, `blackvue_parse_telemetry` | Decode GPMF, FIT and NMEA samples |
| Leveling | `_image_angles`, `_smooth_angles`, `max360_level_filter`, `_flat_level_filter` | Derive and apply smoothed roll/pitch |
| Metadata | `FrameMetadata`, `build_exiftool_args`, `write_metadata` | Build and apply EXIF/XMP tags |
| Export | `write_gpx`, `write_geojson`, `write_points_telemetry`, `write_images_telemetry`, `write_manifest` | Serialise tracks, telemetry and run summaries |
| Runtime | `build_parser`, `validate`, `process_group`, `main` | CLI, validation, orchestration and exit codes |

### 2.3 Execution model

- **Sequential videos, parallel frames.** Videos are processed one at a time. Within a video, `--workers N` runs up to `N` independent ffmpeg extraction jobs.
- **MAX batch decode.** MAX stitches frames in bounded batches (`EXTRACT_CHUNK_SIZE = 60`; `MAX360_LEVEL_CHUNK_SIZE = 8` when leveling). Each batch decodes the source once; a failed seek batch retries from the start of the file.
- **Leveled MAX two-pass.** The EAC → equirectangular result is written as a lossless PNG, then each frame is rotated on the sphere with its own `v360` filter and encoded to the requested format. This keeps the sphere remap as the only lossy step.
- **Telemetry caching.** VIRB FIT files are parsed once and reused across videos in the same run (`telemetry_cache`).
- **Failure isolation.** A `Fatal` error fails one video, not the run. `--fail-fast` stops at the first failure.

---

## 3. Supported cameras and hardware

### 3.1 Camera classes

| Class | Camera | Container / media | Video | Telemetry carrier | IMU | 360 | Leveling |
|---|---|---|---|---|---|---|---|
| `garmin-virb-ultra-30` | Garmin VIRB Ultra 30 | MP4/MOV, H.264 | Flat 4K/1080 | Sibling `.FIT` in `GMetrix/` or matched by name/time | `ACCL`, `GYRO`, `MAGN` in FIT | No | Flat frame rotation |
| `gopro-max-360` | GoPro MAX 360 | `.360` / MP4 / MOV, H.264/H.265 | Dual EAC hemispheres (4096×1344 or 2272×736) | Embedded GPMF `gpmd` | `ACCL`, `GYRO`, `MAGN`, `GRAV`, `CORI`, `IORI` | Yes | Pixel-level sphere rotation |
| `blackvue-dr900` | BlackVue DR900 series | MP4, H.264 | Flat dashcam | NMEA sentences in top-level `free` atoms | None | No | None |
| `gopro-hero` | GoPro HERO (flat) | MP4/MOV, H.264/H.265 | Single flat stream | Embedded GPMF `gpmd` | `ACCL`, `GYRO`, `MAGN`, `GRAV`, `CORI`, `IORI` | No | Flat frame rotation |

### 3.2 Garmin VIRB Ultra 30

- **Format requirements:** MP4/MOV with a single H.264 video stream. The camera writes FIT telemetry separately, usually in a sibling `GMetrix/` folder.
- **Telemetry linking order:**
  1. An explicit `--fit FILE`.
  2. The exact FIT filename stored in the MP4 container (`fit_reference`).
  3. The FIT whose filename start time is closest to the video's local creation stamp (within six hours).
  4. The only FIT candidate, if exactly one exists.
- **Segment alignment:** One FIT file may describe several recorded clips. Camera events (`video_start`, `video_split`, `video_end`) define segments. The segment whose local start time is closest to the MP4 creation time wins; duration proximity is the fallback.
- **Per-video timeline:** Video start is `video_start_fit`; capture time is `video_start_utc + pts`, including real milliseconds. The active telemetry window is `video_start_fit → video_start_fit + duration`.
- **Camera orientation:** FIT camera-orientation events (0/90/180/270) are applied to the accelerometer vector so exported flat frames are leveled in the stored video's image frame. The accelerometer up/down sign is derived automatically from the camera data.
- **Validated frame rates:** 24 and 30 fps classic recording. Variable-frame-rate sources are handled because selection uses real PTS.
- **Filename convention:** `VIRB[0-9]+.(mp4|mov)`.

### 3.3 GoPro MAX 360

- **Format requirements:** `.360`, MP4 or MOV with two matching EAC hemisphere streams and an embedded GPMF (`gpmd`) track. A single wide equirectangular stream is also accepted.
- **Known layouts:**
  - `4096×1344`: side 1376 + centre 1344 + side 1376, 32 px blend.
  - `2272×736`: side 768 + centre 736 + side 768, 16 px blend.
- **Stitching:** The filter cross-blends the overlapping side seams, assembles a 3×2 EAC net, and reprojects with `v360=eac:equirect`. Default output is 5376×2688.
- **TimeWarp / rate changes:** GPS span and video span are compared. A ratio outside 0.8–1.2 maps real GPS time onto playback time (`scale = video_span / gps_span`), so frame selection and timestamps stay aligned and no frame is emitted twice.
- **Validated frame rates:** 24 and 30 fps, plus TimeWarp/timelapse recordings that play back at 30 fps.
- **Leveling:** Pixel-level rotation of the equirectangular sphere from `GRAV` roll/pitch. Yaw is never changed. Pose EXIF angles are written as zero to avoid double correction.
- **No valid GPS fix:** `--fallback-interval SECONDS` exports an unpositioned time grid while retaining the GPMF IMU, camera clock and TimeWarp scale.
- **Filename convention:** `GS[0-9A-Z]+.(360|mp4|mov)`; device name usually contains `MAX`.

### 3.4 BlackVue DR900 series

- **Format requirements:** MP4 with a single H.264 video stream. NMEA sentences are embedded in top-level `free` atoms; the `cprt` atom supplies make, model, firmware and serial.
- **Telemetry:** `$GPGGA`, `$GPRMC`, `$GPVTG`, `$GPGSA`. Duplicate NMEA blocks are de-duplicated while preserving order.
- **Clock reconciliation:** The filename carries camera-local start time (`YYYYMMDD_HHMMSS_XX`). GPS time is authoritative; the camera-vs-GPS skew is logged when it exceeds one second.
- **Timezone:** Derived by snapping the local-vs-UTC difference to a quarter-hour step.
- **IMU:** None. Images are never rotated; telemetry contains GPS only.
- **Validated frame rates:** 2 fps and 30 fps time-lapse modes.
- **Filename convention:** `YYYYMMDD_HHMMSS_XX.(mp4|mov)`.

### 3.5 GoPro HERO

- **Format requirements:** MP4/MOV with exactly one flat video stream and an embedded GPMF (`gpmd`) track.
- **Telemetry:** `GPS5`/`GPS9` positions, `GPSU`/`STMP` timestamps, `GPSF`/`GPSP` accuracy/DOP, plus IMU streams.
- **Clock anchoring:** GPS UTC is authoritative. Without GPS, `STMP` is anchored to the container `creation_time`; if no local offset is known, filenames and EXIF use UTC.
- **Validated frame rates:** 24 and 30 fps classic recording.
- **No GPS:** `--fallback-interval SECONDS` exports an unpositioned time grid with IMU snapshots when the clock is available.
- **Filename convention:** `(GX|GH|GOPR|GOPRO)[0-9A-Z]*.(mp4|mov)`; device name usually contains `HERO`.

### 3.6 Detection evidence

Detection never guesses. Accepted evidence includes:

| Camera | Evidence |
|---|---|
| VIRB | Garmin container handler; `VIRB*` filename; Garmin container markers |
| MAX 360 | GoPro handler plus `DVNM` containing `MAX`; `.360`; dual-video/EAC layout; `GS*` filename |
| BlackVue | PittaSoft/BlackVue handler; `YYYYMMDD_HHMMSS_XX` filename; PittaSoft/BlackVue markers |
| HERO | GoPro handler plus `DVNM` containing `HERO`; single flat video stream with GPMF; `GX*`/`GH*`/`GOPR*` filename |

Ambiguous GoPro files (for example a generic GoPro handler with conflicting MAX/HERO evidence) are skipped with `ambiguous GoPro model metadata`. Unrecognised files are skipped with `unsupported camera (missing camera metadata)`.

### 3.7 Frame-rate and timelapse scope

| Camera | Modes validated | Notes |
|---|---|---|
| Garmin VIRB Ultra 30 | 24 fps, 30 fps | Classic recording; VFR handled by real PTS |
| GoPro MAX 360 | 24 fps, 30 fps, TimeWarp/timelapse at 30 fps playback | GPS span is scaled to video span |
| BlackVue DR900 | 2 fps, 30 fps | Timelapse and normal drive modes |
| GoPro HERO | 24 fps, 30 fps | Classic recording |

These are tested operating modes, not hard-coded rejections. Any container whose frame PTS values are readable can be processed.

### 3.8 Runtime environment

| Variable | Default | Purpose |
|---|---|---|
| `V2I_FFMPEG` | `ffmpeg` | Override the ffmpeg executable |
| `V2I_FFPROBE` | `ffprobe` | Override the ffprobe executable |
| `V2I_EXIFTOOL` | `exiftool` | Override the exiftool executable |
| `NO_COLOR` | unset | Disable ANSI colours when set |
| `FORCE_COLOR` / `CLICOLOR_FORCE` | unset | Force ANSI colours when set |

---

## 4. Raw telemetry ingestion

### 4.1 Telemetry availability matrix

| Telemetry group | GoPro MAX | GoPro HERO | Garmin VIRB | BlackVue |
|---|---|---|---|---|
| GPS position | GPMF `GPS5`/`GPS9` | GPMF `GPS5`/`GPS9` | FIT message 160 | NMEA GGA/RMC |
| Altitude | GPMF | GPMF | FIT enhanced altitude | NMEA GGA |
| Speed | GPMF 2D/3D | GPMF 2D/3D | FIT speed + velocity | RMC/VTG |
| Course / bearing | GPMF course + computed | GPMF course + computed | FIT heading | RMC/VTG |
| Accelerometer | `ACCL` | `ACCL` | FIT message 165 (`ACCL`) | — |
| Gyroscope | `GYRO` | `GYRO` | FIT message 164 (`GYRO`) | — |
| Magnetometer | `MAGN` | `MAGN` | FIT message 208 (`MAGN`) | — |
| Gravity / tilt | `GRAV` | `GRAV` | Derived from `ACCL` | — |
| Orientation quaternions | `CORI`, `IORI`, `ORIN` | `CORI`, `IORI`, `ORIN` | — | — |
| Camera events | — | — | FIT message 161 | — |
| Device metadata | GPMF `DVNM` | GPMF `DVNM` | FIT device info | `cprt` atom |
| DOP / accuracy | `GPSF`, `GPSP`, `GPS9` | `GPSF`, `GPSP`, `GPS9` | FIT fields 8/11/12 | GGA/GSA |

### 4.2 Spatial and kinematic data

| Field | Units | Source behavior |
|---|---|---|
| `latitude`, `longitude` | decimal degrees | GPMF scaled by 1e7; FIT semicircles; NMEA `ddmm.mmmm` |
| `alt` | metres | GPMF scaled by 1e3; FIT `raw/5 - 500`; NMEA GGA |
| `speed` / `speed2d` | m/s | GPMF 2D speed; FIT `raw/1000`; NMEA knots × 0.514444 |
| `speed3d` | m/s | GPMF 3D speed where present |
| `course` | degrees true | GPMF/FIT heading; NMEA RMC/VTG |
| `velocity` | m/s vector | FIT enhanced speed vector |
| `accel` (`ACCL`) | GoPro `m/s^2`, VIRB `g` | 3-axis accelerometer |
| `gyro` (`GYRO`) | GoPro `rad/s`, VIRB `deg/s` | 3-axis gyroscope |
| `magnetic` (`MAGN`) | GoPro `µT`, VIRB `G` | 3-axis magnetometer |
| `gravity` (`GRAV`) | `g` | Gravity/up vector; derives roll and pitch |
| `CORI`, `IORI` | quaternion `[w,x,y,z]` | Camera/image rotation quaternions |
| `ORIN` | 3 components | Orientation vector |

GoPro stores `GRAV` in `(X, Z, Y)` order: +X right, +Z up, +Y backwards. A level camera reads approximately `[0, 1, 0]`.

### 4.3 Temporal data

| Field | Resolution | Source |
|---|---|---|
| Frame timeline | container PTS | `ffprobe -show_packets` in presentation order |
| Fallback timeline | nominal `1/fps` | Uniform grid when PTS is unreadable |
| `pts_seconds` | microseconds | Per-frame presentation timestamp |
| `stmp_us` | microseconds | GPMF `STMP`, recording-relative |
| `fit_s` | seconds (float) | FIT timestamp field 253 |
| `utc` | milliseconds | GPS UTC, FIT UTC or anchored camera clock |
| `t_s` | seconds | Video-relative time after TimeWarp scaling |
| `video_start_utc` | milliseconds | Anchor for `t_s`, telemetry and filenames |
| `utc_offset_s` | seconds | Camera-local minus UTC offset |

**Epoch mapping**

- **GPMF:** `STMP` microseconds are combined with the median `UTC − STMP` offset across GPS blocks. `GPSU` provides absolute UTC (`YYMMDDhhmmss.sss`). `GPS9` days/seconds are counted from `2000-01-01T00:00:00Z`. The spread of the clock offsets is reported as `clock_spread_s`.
- **FIT:** the epoch is `1989-12-31T00:00:00Z`. Message field 253 carries the FIT timestamp; field 0 adds milliseconds; field 6 carries UTC.
- **NMEA:** sentences carry `hhmmss.ss` and `ddmmyy`. The embedded bracket timestamp supplies millisecond ordering. The video start is `first_gps_utc − first_gps_t`.
- **HERO without GPS:** `creation_time + STMP` anchors the IMU and fallback frames.

### 4.4 Optical and sensor data

The pipeline does **not** export lens or exposure metadata as structured telemetry. The following are the factual boundaries:

| Field | Availability | Pipeline behavior |
|---|---|---|
| Shutter / exposure time | Present as a GPMF stream name on some GoPro firmware | Listed in the GPMF stream summary; not parsed into samples |
| ISO | Present as a GPMF stream name | Listed only |
| White balance gains / temperature | Present as GPMF stream names | Listed only |
| Focal length | Not present in supported streams | Not exported |
| Aperture | Not present in supported streams | Not exported |
| Lens distortion coefficients | Not present in supported streams | Not exported; stitching assumes the native EAC/equirect layout |
| Sensor temperature | Not present in supported streams | Not exported |

Do not expect EXIF `FocalLength`, `FNumber`, `ExposureTime` or `LensModel` values from this tool. Camera identity, GPS, orientation and provenance are the supported metadata surface.

### 4.5 Accuracy and quality flags

| Flag | Meaning |
|---|---|
| `accuracy_source = "fit-gps_metadata-field8"` | Measured VIRB accuracy in metres |
| `accuracy_source = "hdop*uere"` | Estimated as `HDOP × --uere` |
| `accuracy_estimated = true` | Accuracy derived from DOP, not measured |
| `position_source = "interpolated"` | Position interpolated between two samples |
| `position_source = "clamped-start"` | Target earlier than telemetry; first sample reused |
| `position_source = "clamped-end"` | Target later than telemetry; last sample reused |
| `position_source = "sample"` | Single-sample track |
| `position_source = "unpositioned"` | Fallback export with no GPS fix |
| `clamped` | `true` for clamped-start/clamped-end |
| `estimated` | `true` for interpolated or time-grid fallback points |

### 4.6 Timezone and clock policy

The host timezone is never consulted.

| Camera | Clock rule |
|---|---|
| MAX | Container `creation_time` is compared with the first GPS UTC. `STMP` is added so later chapters compare against recording start. The difference is rounded to the nearest 15 minutes and rejected beyond ±14 hours. |
| HERO | GPS UTC is authoritative. Without GPS, the container clock anchors `STMP`; an unknown offset falls back to UTC with a warning. |
| VIRB | FIT UTC is authoritative. Container `creation_time` is camera-local wall clock and is used for segment matching and as a last-resort anchor. |
| BlackVue | GPS UTC is authoritative. Filename local time is snapped to a quarter-hour offset and checked against GPS. |

An explicit `--timezone` always wins. `--time-base utc` forces UTC naming regardless of detection.

---

## 5. Configuration and parameters

### 5.1 Global input, output and selection

| Option | Type | Default | Valid range / values | Functional description |
|---|---|---|---|---|
| `-i`, `--input PATH` | path | required | existing file or directory | Root to scan. A file is processed directly; a directory is walked according to `--recursive`. |
| `-o`, `--output PATH` | path | required | writable directory | Root for images, tracks and telemetry. Created automatically unless `--dry-run`. |
| `-r`, `--recursive` | flag | on | on/off | Descend into subdirectories during discovery. |
| `--no-recursive` | flag | off | on/off | Restrict discovery to the input directory's immediate files. |
| `-d`, `--distance METERS` | float | `5.0` | `> 0` | Target spacing along the cumulative track. Each target becomes one image. |
| `--start-distance METERS` | float | `0.0` | `>= 0` | Skip this much travel before the first target. With `0`, frame index 0 is always exported, even if telemetry starts later. |
| `--max-position-error METERS` | float | `distance / 2` | `> 0` | Maximum ground (or 3D) distance allowed when matching a frame to its target. Targets with no acceptable unused frame are skipped. |
| `--max-images N` | int | unlimited | `>= 1` | Cap exported images per video. Also caps fallback time-grid frames. |
| `--extensions LIST` | comma list | `.mp4,.mov,.m4v,.360` | at least one extension | Case-insensitive file extensions to scan. Legacy `--video-glob` is converted to this list. |
| `--distance-mode {2d,3d}` | enum | `2d` | `2d`, `3d` | Measure travel on the ground or include altitude changes. |
| `--fallback-interval SECONDS` | float | `0.0` | `>= 0` | When a video has no usable GPS position, export a time grid at this interval instead of skipping it. Applies to every camera. `0` disables the fallback. |
| `--workers N` | int | `1` | `>= 1` | Number of parallel frame-extraction jobs. Also parallelises leveled MAX sphere rotation. |
| `--threads N` | int | `0` | `>= 0` | ffmpeg threads per job. `0` leaves ffmpeg's automatic choice. |
| `--overwrite` | flag | off | on/off | Rewrite existing images in place. Without it, existing paths are skipped and counted. |
| `--dry-run` | flag | off | on/off | Run detection, telemetry, selection and reporting; write nothing and create no output directory. |
| `--fail-fast` | flag | off | on/off | Stop after the first video that fails. |
| `--manifest FILE` | path | none | writable path | Write a JSON run manifest with every exported entry and per-video summary. Suppressed by `--dry-run`. |

### 5.2 Timing, naming and metadata

| Option | Type | Default | Valid range / values | Functional description |
|---|---|---|---|---|
| `--timezone OFFSET` | string | auto-detect | `±HH:MM`, `±H`, or decimal hours | Override the camera UTC offset used for local filenames and EXIF. Detection reads the camera clock when possible. |
| `--tz OFFSET` | string | auto-detect | same as `--timezone` | Legacy alias for `--timezone`. |
| `--time-base {local,utc}` | enum | `local` | `local`, `utc` | Choose camera-local or UTC wall clock for filenames, folders and EXIF timestamps. |
| `--format {jpg,png}` | enum | `jpg` | `jpg`, `png` | Output image codec and file extension. |
| `--quality 1-100` | int | `95` | `1`–`100` | Quality mapping: JPEG uses `-q:v`; PNG uses `-compression_level`. Higher means better quality and larger files. |
| `--keywords LIST` | string | none | semicolon or comma separated | Opt-in `Keywords` and `XMP:Subject` values, de-duplicated case-insensitively. |
| `--heading-offset DEGREES` | float | none | finite number | Added to the resolved heading to produce the image direction. Normalised to unsigned `000`–`360`; when supplied, appended to the camera folder as `-NNN`. |
| `--offset DEGREES` | float | none | finite number | Alias for `--heading-offset`. |
| `--magnetic-declination DEGREES` | float | `0.0` | `-180`–`+180` | East-positive declination added to compass/IMU headings to obtain true north. |
| `--uere METERS` | float | `5.0` | `> 0` | User-equivalent range error used to estimate accuracy as `HDOP × UERE` when no measured accuracy exists. |
| `--level-horizon` / `--no-level-horizon` | bool | off | on/off | Enable or disable IMU-based horizon leveling. Off preserves the source tilt. |
| `--imu-heading` | flag | off | on/off | Derive a tilt-compensated compass heading from `MAGN` + `GRAV` instead of GPS course (MAX/HERO). |

### 5.3 Export and logging

| Option | Type | Default | Valid range / values | Functional description |
|---|---|---|---|---|
| `--track-export {both,gpx,geojson,none}` | enum | `both` | `both`, `gpx`, `geojson`, `none` | Select the spatial track files written per sequence. `none` still allows telemetry JSON. |
| `--save-telemetry` | flag | off | on/off | Legacy alias for `--track-export both`. |
| `--telemetry-export {both,points,images,none}` | enum | `both` | `both`, `points`, `images`, `none` | Write every telemetry point, per-image snapshots, both, or neither. |
| `--sidecars` | flag | off | on/off | Legacy: also write `<sequence>.track.jsonl` (one JSON header plus one JSON object per image). |
| `--no-exif` | flag | off | on/off | Skip the `exiftool` pass entirely. Images still export. |
| `-v`, `--verbose` | flag | off | on/off | Set log level to `DEBUG`. |
| `-q`, `--quiet` | flag | off | on/off | Set log level to `ERROR`. |
| `--log-level LEVEL` | enum | `INFO` | `DEBUG`, `INFO`, `WARNING`, `ERROR`, `CRITICAL` | Explicit log level. `-v`/`-q` take precedence. |
| `--version` | flag | — | — | Print the version and exit. |
| `-h`, `--help` | flag | — | — | Print the grouped help text and exit. |

### 5.4 GPS handling

| Option | Type | Default | Valid range / values | Functional description |
|---|---|---|---|---|
| `--smooth N` | int | `9` | `>= 1` | Moving-average window applied to MAX GPS latitude/longitude samples before track construction. `1` disables smoothing. |
| `--stationary-speed MPS` | float | `0.3` | `>= 0` | Compatibility option. Accepted so existing command lines keep working; the current track suppresses stationary course noise below 0.3 m/s internally, but changing this flag does not alter the threshold. |
| `--jitter METERS` | float | `0.5` | `>= 0` | Compatibility option. Accepted for legacy scripts; the unified track does not use a separate jitter cutoff. |

### 5.5 GoPro MAX 360 options

| Option | Type | Default | Valid range / values | Functional description |
|---|---|---|---|---|
| `--width WIDTH` | int | `5376` | `>= 2`, even | Equirectangular output width. Must be even for yuv output. |
| `--height HEIGHT` | int | `2688` | `>= 2`, even | Equirectangular output height. Use 2:1 with `--width`. |
| `--interp MODE` | enum | `cubic` | `cubic`, `linear`, `nearest`, `lanczos` | `v360` reprojection interpolation used for stitching and sphere leveling. |
| `--stitch-backend {auto,ffmpeg,max2sphere}` | enum | `auto` | `auto`, `ffmpeg`, `max2sphere` | `auto` prefers ffmpeg and falls back to `max2sphere`. `max2sphere` must be installed to select it explicitly. |
| `--keep-filter` | flag | off | on/off | Copy each generated ffmpeg filter graph next to the sequence. Multiple batches get `.00`, `.01`, … suffixes. |
| `--heading-window N` | int | `4` | `>= 1` | Compatibility option for legacy MAX heading smoothing. Validated but not used by the current unified track. |
| `--smooth N` | int | `9` | `>= 1` | Position smoothing window (see GPS handling). |

### 5.6 GoPro HERO options

| Option | Type | Default | Valid range / values | Functional description |
|---|---|---|---|---|
| `--gpmf-stream INDEX` | int | auto-detect | `>= 0` | Absolute ffmpeg stream index of the `gpmd` telemetry track. Auto-detection uses container metadata. |
| `--device-model NAME` | string | GPMF `DVNM` or `GoPro HERO` | any string | Override the EXIF `Model` value for HERO images. |
| `--allow-missing-gps` | flag | off | on/off | Deprecated alias. Unpositioned export is enabled by `--fallback-interval SECONDS`; if this flag is supplied without a positive interval, validation fails. |

### 5.7 Garmin VIRB options

| Option | Type | Default | Valid range / values | Functional description |
|---|---|---|---|---|
| `--fit FILE` | repeatable path | auto-detect | existing FIT file | Use this FIT file. One value applies to every VIRB video in the run. Multiple values form a candidate pool. |
| `--fit-directory-name NAME` | string | `gmetrix` | directory name | Sibling folder searched for FIT telemetry, case-insensitively. |

### 5.8 Legacy and compatibility aliases

| Alias | Canonical option | Behavior |
|---|---|---|
| `--offset` | `--heading-offset` | Same argument, same normalisation |
| `--tz` | `--timezone` | Hidden in help, accepted for old MAX command lines |
| `--jobs` | `--workers` | Hidden in help, accepted for old MAX command lines |
| `--video-glob PATTERN` | `--extensions` | Converted to an extension list |
| `--save-telemetry` | `--track-export both` | Writes GPX and GeoJSON |
| `--allow-missing-gps` | `--fallback-interval` | Requires a positive interval; kept for compatibility |
| positional `INPUT` | `--input` | One positional input path is accepted |

---

## 6. Processing internals

### 6.1 Frame timeline

- `ffprobe -show_packets` returns packet PTS in decode order. The pipeline sorts numeric PTS into presentation order, which matches ffmpeg's frame index.
- Missing PTS values are filled from neighbouring packets and the nominal frame interval.
- The timeline is forced monotonic; stray non-increasing values are shifted forward.
- If no packet timestamps are usable, a uniform `1/fps` grid is synthesised from frame count/duration and a warning is logged (`fps-fallback`).
- Extraction seeks to the real PTS of each selected frame. This keeps VFR and timelapse footage aligned without renumbering frames.

### 6.2 Distance track and target planning

- `Track` keeps samples with valid latitude/longitude, sorts by video-relative time and drops duplicate timestamps.
- Cumulative distance uses haversine segments (`2d`) or `hypot(ground, Δaltitude)` (`3d`).
- `plan_shots()` emits targets at `start_distance + k × distance` while the target remains within track length.
- A zero-length track yields one target at distance zero.

### 6.3 Unique-frame selection

For each target:

1. Interpolate the target time from cumulative distance.
2. Search outward from the target time for candidate frames whose position is within `--max-position-error`.
3. Reject frames at or before the last selected index, so indices strictly increase and are never reused.
4. Accept the nearest acceptable candidate; otherwise skip the target with a logged reason.
5. With `--start-distance 0`, force frame index 0 for the first target. If frame 0 cannot be extracted, the video fails.

Gaps are allowed. Low frame rates and VFR sources produce fewer images rather than duplicate frames. `--max-images` truncates the plan.

### 6.4 Heading resolution

`resolve_heading()` applies this precedence:

1. `compass_heading` — tilt-compensated IMU/MAGN heading (`heading_source = "compass"`).
2. `metadata_heading` — camera/container heading (`heading_source = "metadata"`).
3. `course` or `heading` — recorded course over ground (`heading_source = "gps_course"`).
4. `gps_course` — computed from neighbouring track positions (`heading_source = "gps_course"`).
5. `none`.

`--magnetic-declination` is added only to compass headings. `--heading-offset` is added to every resolved heading and then normalised. The EXIF `GPSImgDirection` uses the final true-north value; `GPSTrack` carries course over ground.

### 6.5 Horizon leveling

| Camera | Method |
|---|---|
| MAX 360 | Two-pass: stitch to equirectangular PNG, then rotate the sphere with `v360=e:e:roll=-roll:pitch=-pitch`. Yaw is untouched. EXIF pose roll/pitch stay `0`. |
| HERO | Rotate the flat frame clockwise by roll using an expanded canvas: `rotate=a=…:ow=rotw(…):oh=roth(…):fillcolor=black`. |
| VIRB | Calibrate `ACCL` with FIT message 167, apply the orientation matrix, rotate into the stored video frame, then rotate the flat image like HERO. |
| BlackVue | No IMU; images stay unrotated. |

**Angle convention**

- Positive roll = clockwise / right side down.
- Positive pitch = camera tilted down.
- `v360` uses the inverse screen rotation, so MAX applies `roll=-roll_deg` and `pitch=-pitch_deg`.

**Smoothing**

- `_smooth_angles()` computes a centered moving average over up to two exported positions before and after the current one (five images total).
- Roll is averaged as a circular quantity; pitch uses an arithmetic mean.
- Images without their own IMU sample are omitted from the average.
- Raw `roll_deg`/`pitch_deg` remain in telemetry; `level_roll_deg`/`level_pitch_deg` record the applied value.
- Rotations below `0.05°` are skipped to avoid needless resampling.
- Leveling happens during export. Existing images are skipped unless `--overwrite` is used.

### 6.6 MAX 360 stitching pipeline

1. Probe the file; select the largest matching EAC hemisphere pair, or a single wide equirectangular stream.
2. Build a filter graph:
   - Split each hemisphere into side/centre/side regions.
   - Cross-blend the 32 px (or 16 px) overlap seams.
   - Assemble the six faces into a 3×2 EAC net.
   - Reproject with `v360=eac:equirect:interp=…:w=…:h=…`.
3. Process selected frames in batches:
   - Seek to the first PTS of each batch when the timeline allows it.
   - Retry the batch from the start of the file if the seek produces a bad result.
4. Without leveling, encode directly to the target format.
5. With leveling, write lossless PNG intermediates and run one `v360=e:e` sphere rotation per frame, then encode.
6. `--workers` parallelises the per-frame sphere rotations.
7. `max2sphere` shares the same two-pass leveled path after it produces the equirectangular video.

`--keep-filter` copies each batch graph to `<sequence>.filter` (or `<sequence>.NN.filter` when there are multiple batches).

### 6.7 EXIF/XMP policy

- `DateTimeOriginal`, `CreateDate`, `ModifyDate` are camera-local when the offset is known, otherwise UTC.
- `SubSecTimeOriginal` carries the capture millisecond.
- `OffsetTimeOriginal`/`OffsetTime` are written only when the offset is known.
- All GPS date/time tags are UTC.
- `GPSHPositioningError` is written when an accuracy exists.
- Provenance goes to `ImageDescription`, `UserComment`, `XPComment` and `XMP:Description`; it never goes to keywords.
- `Keywords`/`XMP:Subject` are written only when `--keywords` is passed.
- GPano/XMP-GPano tags are written only for MAX panoramic images.
- `PoseRollDegrees`/`PosePitchDegrees` are always `0`; the pixels are either already leveled or deliberately untouched.

---

## 7. Export profiles and formats

### 7.1 Output tree

```text
<output>/
└── YYYY-MM-DD-<camera-suffix>[-<offset>]/
    └── <sequence>/
        ├── YYYY-MM-DD_HH-MM-SS-mmm.jpg
        ├── YYYY-MM-DD_HH-MM-SS-mmm.png
        ├── <sequence>.gpx
        ├── <sequence>.geojson
        ├── <sequence>.telemetry.json
        ├── <sequence>.image-telemetry.json
        ├── <sequence>.track.jsonl        # only with --sidecars
        └── <sequence>.filter[.NN]        # only with --keep-filter
```

- The date folder is based on the **first exported frame**, not on the source filename.
- Camera suffixes: `virb30`, `max360`, `blackvue`, `hero`.
- The sequence name is the source stem. Duplicate stems in nested folders are prefixed with the relative directory (`subdir__VIRB0001`); no numeric `_NNNNNN` counter is used.
- An explicitly supplied heading offset is appended to the camera folder: `YYYY-MM-DD-virb30-090`.
- Images that cross midnight stay in the same sequence folder; their filenames carry the date.
- A `-2`/`-3` suffix is used only when two frames selected in the same run share a millisecond.

### 7.2 Export format matrix

| Format | File | When written | Content |
|---|---|---|---|
| JPEG / PNG | `*.jpg`, `*.png` | Always (subject to skip/overwrite) | Geotagged still image |
| EXIF/XMP | Inside image | Unless `--no-exif` or exiftool missing | Camera, GPS, time, provenance, GPano |
| GPX 1.1 | `<sequence>.gpx` | `--track-export both\|gpx` | Track points with UTC time, elevation, speed/course extensions |
| GeoJSON RFC 7946 | `<sequence>.geojson` | `--track-export both\|geojson` | LineString track plus one Point feature per image |
| Points telemetry | `<sequence>.telemetry.json` | `--telemetry-export both\|points` | Every GPS and IMU sample |
| Image telemetry | `<sequence>.image-telemetry.json` | `--telemetry-export both\|images` | Per-image GPS/IMU snapshots |
| Run manifest | `--manifest FILE` | `--manifest` and not `--dry-run` | Every exported entry and video summary |
| Legacy sidecar | `<sequence>.track.jsonl` | `--sidecars` | JSON-lines header plus one record per image |

### 7.3 GPX schema

```xml
<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="video2images"
     xmlns="http://www.topografix.com/GPX/1/1"
     xmlns:v2i="https://video2images.local/ns/1">
  <metadata><name>GS013338</name></metadata>
  <trk><name>GS013338</name><trkseg>
    <trkpt lat="46.12345678" lon="16.12345678">
      <ele>123.45</ele>
      <time>2026-10-31T10:13:24.821Z</time>
      <extensions>
        <v2i:speed>5.20</v2i:speed>
        <v2i:course>87.40</v2i:course>
        <v2i:heading_source>gps_course</v2i:heading_source>
      </extensions>
    </trkpt>
  </trkseg></trk>
</gpx>
```

### 7.4 GeoJSON schema

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "LineString",
        "coordinates": [
          [16.12345678, 46.12345678, 123.45],
          [16.12351234, 46.12350123, 123.60]
        ]
      },
      "properties": {
        "name": "GS013338",
        "camera": "gopro-max-360",
        "video": "GS013338.360",
        "point_count": 2
      }
    },
    {
      "type": "Feature",
      "geometry": {
        "type": "Point",
        "coordinates": [16.12345678, 46.12345678, 123.45]
      },
      "properties": {
        "image": "2026-10-31_12-13-24-821.jpg",
        "video": "GS013338.360",
        "sequence": "GS013338",
        "camera": "gopro-max-360",
        "frame_index": 0,
        "pts_seconds": 0.0,
        "timestamp_local": "2026-10-31T12:13:24.821",
        "timestamp_utc": "2026-10-31T10:13:24.821Z",
        "heading_deg": 87.4,
        "heading_source": "gps_course",
        "speed_m_s": 5.2,
        "accuracy_m": 2.5,
        "accuracy_estimated": true,
        "accuracy_source": "hdop*uere",
        "hdop": 0.5,
        "vdop": 0.7,
        "pdop": null,
        "distance_m": 0.0,
        "position_source": "interpolated",
        "clamped": false,
        "estimated": true
      }
    }
  ]
}
```

A one-point track is emitted as a `Point` feature. An empty track emits no geometry feature but still emits image features when positions exist.

### 7.5 Points telemetry schema

```json
{
  "generator": "v2i.py",
  "camera": "gopro-max-360",
  "camera_label": "GoPro MAX 360",
  "video": "GS013338.360",
  "sequence": "GS013338",
  "video_start_utc": "2026-10-31T10:13:24.821Z",
  "streams": {
    "gps": {
      "point_count": 1,
      "points": [
        {
          "lat": 46.12345678,
          "lon": 16.12345678,
          "alt": 123.45,
          "speed": 5.2,
          "speed2d": 5.2,
          "speed3d": 5.3,
          "fix": 3,
          "hdop": 0.5,
          "vdop": 0.7,
          "pdop": null,
          "dop": 0.5,
          "utc": "2026-10-31T10:13:24.821Z",
          "stmp_us": 1234567,
          "t_s": 0.0
        }
      ]
    },
    "ACCL": {
      "unit": "m/s^2",
      "components": ["x", "y", "z"],
      "point_count": 1,
      "points": [
        {
          "utc": "2026-10-31T10:13:24.821Z",
          "t_s": 0.0,
          "stmp_us": 1234567,
          "x": 0.12,
          "y": 9.79,
          "z": -0.04
        }
      ]
    },
    "GRAV": {
      "unit": "g",
      "components": ["x", "y", "z"],
      "point_count": 1,
      "points": [
        {
          "utc": "2026-10-31T10:13:24.821Z",
          "t_s": 0.0,
          "stmp_us": 1234567,
          "x": 0.01,
          "y": 0.02,
          "z": 0.99,
          "roll_deg": -1.157,
          "pitch_deg": 0.579,
          "angle_source": "GRAV"
        }
      ]
    }
  }
}
```

VIRB files add a `calibration` object keyed by sensor (`ACCL`, `GYRO`, `MAGN`) with `factor`, `divisor`, `level_shift`, `unit`, optional `offset_cal` and optional `orientation_matrix`.

### 7.6 Image telemetry schema

```json
{
  "generator": "v2i.py",
  "camera": "gopro-max-360",
  "camera_label": "GoPro MAX 360",
  "video": "GS013338.360",
  "sequence": "GS013338",
  "video_start_utc": "2026-10-31T10:13:24.821Z",
  "image_count": 1,
  "distance_interval_m": 5.0,
  "time_base": "local",
  "images": [
    {
      "sequence": "GS013338",
      "video": "GS013338.360",
      "camera": "gopro-max-360",
      "image": "2026-10-31_12-13-24-821.jpg",
      "frame_index": 0,
      "pts_seconds": 0.0,
      "datetime_local": "2026-10-31T12:13:24.821",
      "datetime_utc": "2026-10-31T10:13:24.821Z",
      "utc_offset_s": 7200,
      "latitude": 46.12345678,
      "longitude": 16.12345678,
      "altitude_m": 123.45,
      "speed_m_s": 5.2,
      "course_deg": 87.4,
      "heading_deg": 87.4,
      "heading_source": "gps_course",
      "img_direction_deg": 87.4,
      "accuracy_m": 2.5,
      "accuracy_estimated": true,
      "accuracy_source": "hdop*uere",
      "hdop": 0.5,
      "vdop": 0.7,
      "pdop": null,
      "distance_m": 0.0,
      "track_distance_m": 0.0,
      "position_source": "interpolated",
      "clamped": false,
      "estimated": true,
      "gps": {
        "lat": 46.12345678,
        "lon": 16.12345678,
        "utc": "2026-10-31T10:13:24.821Z",
        "t_s": 0.0
      },
      "imu": {
        "GRAV": {
          "utc": "2026-10-31T10:13:24.821Z",
          "t_s": 0.0,
          "x": 0.01,
          "y": 0.02,
          "z": 0.99,
          "roll_deg": -1.157,
          "pitch_deg": 0.579,
          "angle_source": "GRAV",
          "delta_s": 0.012
        },
        "ACCL": {
          "utc": "2026-10-31T10:13:24.821Z",
          "t_s": 0.0,
          "x": 0.12,
          "y": 9.79,
          "z": -0.04,
          "delta_s": 0.012
        }
      },
      "roll_deg": -1.157,
      "pitch_deg": 0.579,
      "angle_source": "GRAV",
      "level_roll_deg": -1.2,
      "level_pitch_deg": 0.6
    }
  ]
}
```

`level_roll_deg`/`level_pitch_deg` appear only when leveling ran and an angle was applied. IMU snapshots outside a one-second window are `null`. BlackVue uses `"imu": {}` because the hardware has no IMU.

### 7.7 Run manifest schema

```json
{
  "generator": "v2i.py",
  "camera": "mixed",
  "camera_label": "Mixed cameras",
  "cameras": ["blackvue-dr900", "gopro-max-360"],
  "distance_interval_m": 7.0,
  "image_count": 2,
  "images": [
    {
      "sequence": "20190712_120000_XX",
      "video": "20190712_120000_XX.mp4",
      "camera": "blackvue-dr900",
      "image": "2019-07-12_12-00-00-000.jpg",
      "frame_index": 0,
      "pts_seconds": 0.0,
      "datetime_utc": "2019-07-12T10:00:00.000Z",
      "latitude": 46.1234,
      "longitude": 16.1234,
      "distance_m": 0.0
    }
  ],
  "videos": [
    {
      "video": "20190712_120000_XX.mp4",
      "sequence": "20190712_120000_XX",
      "images": 1,
      "planned": 1,
      "skipped": 0,
      "failed": false,
      "dir": "/output/2019-07-12-blackvue/20190712_120000_XX"
    }
  ]
}
```

### 7.8 Legacy JSON-lines sidecar

```text
{"_meta":{"generator":"v2i.py","sequence":"VIRB0001","source_video":"VIRB0001.MP4","camera":{"make":"Garmin","model":"VIRB Ultra 30","serial":123456789,"software":"VIRB Ultra 30 FW 2.80"},"distance_interval_m":5.0,"heading_offset_deg":90.0,"time_base":"local","utc_offset_s":7200,"image_count":1}}
{"sequence":"VIRB0001","video":"VIRB0001.MP4","camera":"garmin-virb-ultra-30","image":"2026-10-18_16-18-37-111.jpg","frame_index":0,"pts_seconds":0.0,"datetime_utc":"2026-10-18T14:18:37.111Z", ...}
```

### 7.9 Unpositioned fallback export

When `--fallback-interval SECONDS` is active and no usable position exists:

- Images use the normal `YYYY-MM-DD_HH-MM-SS-mmm.ext` naming.
- `latitude`, `longitude`, `altitude_m`, `speed_m_s`, `course_deg` and `distance_m` are `null`.
- `position_source` is `"unpositioned"` and `unpositioned` is `true`.
- The GPX track is empty.
- The points telemetry contains an empty GPS stream when the camera has no GPS, plus any available IMU streams.
- Image telemetry still carries IMU snapshots when the camera clock can anchor them.
- MAX retains the GPMF IMU, camera clock and TimeWarp scale.
- HERO anchors IMU to container `creation_time`; without it, snapshot times are `null`.

---

## 8. Operational guide

### 8.1 Quick start

```bash
# Everything under video/, camera detected per file
python3 v2i.py -i video/ -o images/ -d 7

# Garmin VIRB (FIT discovered in GMetrix/)
python3 v2i.py -i video/2026-10-05-virb-1080-24fps -o images/ -d 5 --start-distance 0

# GoPro MAX 360 TimeWarp / timelapse
python3 v2i.py -i video/2026-10-26-max360-000-timelapse05-30fps -o images/ -d 2 --start-distance 0

# BlackVue DR900
python3 v2i.py -i video/2019-07-12-blackvue-standard -o images/ -d 5 --start-distance 0

# HERO with no GPS: unpositioned time-lapse
python3 v2i.py -i video/2025-08-16-hero-standard-nogps -o images/ --fallback-interval 30

# Keep the source tilt (default) or level the horizon
python3 v2i.py -i video/ -o images/ -d 7
python3 v2i.py -i video/ -o images/ -d 7 --level-horizon

# Leveled MAX with parallel sphere rotation
python3 v2i.py -i video/2026-10-31-max360-000-horizont -o images/ -d 2 --level-horizon --workers 4

# Dry run and manifest
python3 v2i.py -i video/ -o images/ -d 7 --dry-run
python3 v2i.py -i video/ -o images/ -d 7 --manifest run.json
```

### 8.2 Re-runs and overwrite

- Without `--overwrite`, an image whose path already exists is skipped and counted in the summary.
- `--overwrite` rewrites the same path in place; it never creates suffixed duplicates.
- Run with `--overwrite` after changing leveling options or time base to regenerate existing images.
- A `-2`/`-3` suffix is reserved for two frames selected in the same run that share a millisecond.

### 8.3 Performance tuning

| Goal | Setting |
|---|---|
| Faster flat extraction | `--workers N` (one ffmpeg per frame) |
| Lower CPU | `--workers 1`, `--threads 1` |
| Faster MAX leveling | `--workers N`; each frame's sphere rotation runs in parallel |
| Lower temporary disk use | MAX leveling batches at 8 frames; unleveled batches at 60 |
| Less ffmpeg memory | `--threads` |
| Avoid resampling | Leave `--level-horizon` off when the source is already level |
| Quick inventory | `--dry-run` still reads telemetry and plans every frame |

### 8.4 Logging and colours

- Format: `[LEVEL] message`.
- `INFO` green, `WARNING` yellow, `ERROR` red, `CRITICAL` bold red, `DEBUG` grey.
- Colours are disabled when stderr is not a TTY. `NO_COLOR=1` forces them off; `FORCE_COLOR=1` or `CLICOLOR_FORCE=1` forces them on.

### 8.5 Exit codes

| Code | Meaning |
|---|---|
| `0` | Every discovered video processed successfully |
| `1` | At least one video failed or produced no images |
| `2` | CLI/setup error, missing ffmpeg/ffprobe, no videos, or no supported camera |
| `130` | Interrupted with Ctrl-C |

### 8.6 Troubleshooting

| Symptom | Cause | Resolution |
|---|---|---|
| `unsupported camera (missing camera metadata)` | File is not a recognised camera or metadata is absent | Confirm the camera class; do not force a pipeline |
| `ambiguous GoPro model metadata` | Conflicting MAX/HERO evidence | Re-export from the camera or separate MAX `.360` files |
| `no GPS telemetry; use --fallback-interval` | No usable fix and no fallback requested | Add `--fallback-interval SECONDS` |
| `no usable GPS telemetry; use --fallback-interval` (MAX) | GPS present but no valid fix | Add `--fallback-interval SECONDS`; IMU and clock still export |
| `could not select the first frame (index 0)` | Frame 0 could not be extracted with `--start-distance 0` | Check codec/seek support; the run fails deliberately |
| `exiftool not found on PATH` | Metadata tool not installed | Install exiftool or pass `--no-exif` |
| `the FIT file this video references … was not found` | MP4 points at a missing FIT | Place the FIT in the expected folder or pass `--fit` |
| Images keep the old tilt | Existing files were skipped | Re-run with `--overwrite` |
| Local filenames are UTC | Camera has no local clock evidence | Pass `--timezone +HH:MM` |
| MAX leveling is slow | Per-frame `v360` sphere rotation on CPU | Raise `--workers`, reduce output size, or leave leveling off |

---

## 9. Tests and verification

Run the standard-library suite from the repository root:

```bash
python3 -m unittest discover -s tests -v
```

Coverage includes:

- Unified camera detection and grouping.
- Coloured logging and level filtering.
- Filename milliseconds, collision handling and timezone naming.
- VIRB capture-time naming and FIT calibration.
- MAX container-clock timezone detection, including STMP chapter correction.
- MAX EAC batch command construction and fallback records.
- Seek-based extraction compared against a decode/select pass.
- Horizon leveling: CLI opt-in, neighbour smoothing, flat rotation filter, MAX sphere rotation and zero pose values, VIRB orientation/sign handling.
- Detailed GPS/IMU telemetry exporters: stream contents, per-image nearest-sample matching, null/empty handling and `--telemetry-export`.

Representative validation command:

```bash
python3 ./v2i.py -i video/ -d 7 -o images/
```

The suite is standard-library `unittest` only. No pytest dependency.

---

## 10. Appendix

### 10.1 GPMF streams parsed

| Key | Components | Unit | Notes |
|---|---|---|---|
| `GPS5` | lat, lon, alt, speed2d, speed3d | deg, m, m/s | Scaled by `[1e7, 1e7, 1e3, 1e3, 1e2]` |
| `GPS9` | lat, lon, alt, speed2d, speed3d, days, seconds, DOP, fix | mixed | Scaled by `[1e7, 1e7, 1e3, 1e3, 1e2, 1, 1e3, 1e2, 1]` |
| `GPSU` | UTC text | `YYMMDDhhmmss.sss` | Parsed as UTC |
| `STMP` | recording clock | microseconds | Anchors sample times |
| `GPSF` | fix / accuracy | camera-defined | Used where present |
| `GPSP` | DOP values | camera-defined | HDOP/VDOP fallback |
| `ACCL` | x, y, z | m/s² | 3-axis accelerometer |
| `GYRO` | x, y, z | rad/s | 3-axis gyroscope |
| `MAGN` | x, y, z | µT | 3-axis magnetometer |
| `GRAV` | x, y, z | g | Gravity vector; roll/pitch source |
| `CORI` | w, x, y, z | quaternion | Camera orientation |
| `IORI` | w, x, y, z | quaternion | Image orientation |
| `ORIN` | x, y, z | vector | Orientation vector |
| `DVNM` | text | — | Device name used by detection |

### 10.2 Garmin FIT messages parsed

| Message | Purpose | Key fields |
|---|---|---|
| `0` file_id | Recording creation time | field 4 → FIT epoch |
| `20` record | Odometer / track distance | field 253 timestamp, field 5 distance (cm) |
| `23` device_info | Device identity | manufacturer, serial, product, software |
| `160` gps_metadata | GPS fix and kinematics | 0 ms, 1 lat, 2 lon, 3 alt, 4 speed, 5 heading, 6 UTC, 7 velocity, 8 accuracy, 11 HDOP, 12 VDOP, 253 timestamp |
| `161` camera_event | Segment and orientation events | 0 ms, 1 event type, 3 orientation |
| `164` gyroscope | Angular rate | 3 axes |
| `165` accelerometer | Specific force | 3 axes |
| `167` sensor_calibration | Raw-to-unit calibration | 0 sensor type, 1 factor, 2 divisor, 3 level shift, 4 offset, 5 orientation matrix |
| `208` magnetometer | Magnetic field | 3 axes |

FIT epoch: `1989-12-31T00:00:00Z`. Latitude/longitude use semicircles (`180 / 2^31`).

### 10.3 NMEA sentences parsed

| Sentence | Fields used |
|---|---|
| `$GPGGA` | time, latitude, N/S, longitude, E/W, fix quality, satellites, HDOP, altitude |
| `$GPGSA` | fix type, PDOP, HDOP, VDOP |
| `$GPRMC` | time, status, latitude, N/S, longitude, E/W, speed (knots), course, date |
| `$GPVTG` | true course, speed (knots), speed (km/h) |

Speed conversion: knots × `0.514444` = m/s.

### 10.4 Angle and unit conventions

| Quantity | Convention |
|---|---|
| Roll | Positive = clockwise / right side down |
| Pitch | Positive = camera tilted down |
| Heading / course | Degrees clockwise from true north |
| Magnetic declination | East-positive |
| GoPro `GRAV` | `(X, Z, Y)`: +X right, +Z up, +Y backwards |
| VIRB `ACCL` | Raw counts → `(raw − level_shift) / divisor × factor`, then orientation matrix |
| MAX `v360` leveling | Inverse rotation: `roll=-roll_deg`, `pitch=-pitch_deg` |
| Flat image rotation | Clockwise by `roll_deg` with expanded canvas |

### 10.5 Selection and naming constants

| Constant | Value | Meaning |
|---|---|---|
| `DEFAULT_EXTENSIONS` | `.mp4,.mov,.m4v,.360` | Files scanned by default |
| `EXTRACT_CHUNK_SIZE` | `60` | MAX unleveled batch size |
| `MAX360_LEVEL_CHUNK_SIZE` | `8` | MAX leveled batch size |
| `max360_DEFAULT_WIDTH` | `5376` | Default equirectangular width |
| `max360_DEFAULT_HEIGHT` | `2688` | Default equirectangular height |
| `virb_DEFAULT_FIT_DIR` | `gmetrix` | Default FIT folder name |
| `virb_DEFAULT_DISTANCE` | `5.0` | Historical VIRB default (CLI default is also 5.0) |
| `virb_DEFAULT_QUALITY` | `95` | Historical VIRB default (CLI default is also 95) |
| `EARTH_RADIUS_M` | `6371008.8` | Haversine radius |

### 10.6 Measured sample evidence

| Sample | Notes |
|---|---|
| `video/2026-10-26-max360-000-timelapse05-30fps/GS013336.360` | GoPro Max, DVNM `GoPro Max`, dual EAC 4096×1344, 1208 frames / 40.34 s; GPS span 604.57 s scaled to video span; equirect 5376×2688 with full GPano tags |
| `video/2025-08-16-hero-standard-nogps/GX011785.MP4` | HERO11 Black, GPMF IMU but no GPS5/GPSU/GPSF/GPSP → skipped by default; fallback exports UTC millisecond names |
| `video/2026-10-05-virb-1080-24fps/VIRB0996.MP4` + `GMetrix/2026-10-05-07-53-07.fit` | 9108 GPS samples, IMU present; camera events align the segment; offset `+02:00` |
| `video/2026-10-18-virb-090-4k-30fps/VIRB0782.MP4` | VIRB Ultra 30; real capture milliseconds in filenames (e.g. `2026-10-18_16-18-37-111.jpg`) |
| `video/2026-10-18-virb-270-4k-24fps/VIRB0008.MP4` | VIRB Ultra 30 at 24 fps; offset `+02:00` |
| `video/2026-10-31-max360-000-horizont-nogps/GS013338.360` | No valid GPS fix; `--fallback-interval` exports unpositioned frames with full IMU telemetry and an empty GPS track |
| BlackVue samples | NMEA in `free` atoms; GGA/RMC/VTG/GSA parsed; clock reconciled to GPS; course from RMC/VTG |

### 10.7 Version history

| Version | Highlight |
|---|---|
| `0.0.3` | Unified single-file pipelines, per-camera telemetry, smoothed opt-in leveling, MAX sphere rotation, timezone detection, universal unpositioned fallback |
| Earlier | Per-camera scripts and modules, merged into `v2i.py` |
