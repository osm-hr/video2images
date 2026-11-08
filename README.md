# video2images (`v2i.py`)

`v2i.py` turns action-camera footage into a distance-spaced, geotagged still-image sequence. It is a single-file, standard-library-only CLI that reads the camera's own telemetry, selects one unique frame per distance target, optionally levels 360 photospheres from the IMU (by rotating the pixels or by writing the XMP-GPano pose), and writes EXIF/XMP metadata plus GPX and GeoJSON tracks and detailed GeoJSON telemetry.

No separate per-camera modules, no database, no service dependencies. `ffmpeg`, `ffprobe` and `exiftool` are external tools; `max2sphere` is optional.

**Version:** `0.0.4` · **Python:** 3.9+ · **Validated with:** ffmpeg/ffprobe 5.1.9, exiftool 12.x

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
11. [Further research](#11-further-research)

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
8. **Extracts stills** with accurate input seek, or stitches MAX/MAX2 hemispheres to equirectangular in bounded batches.
9. **Optionally levels** 360 photospheres using smoothed IMU roll/pitch from the camera — either by rotating the sphere pixels or by writing the pose into XMP-GPano tags. Classic flat videos are never rotated.
10. **Writes metadata** (EXIF/XMP, GPS, camera firmware in `Software`, exporter version in `XMP:CreatorTool`, GPano pose) with `exiftool`.
11. **Exports tracks and telemetry** as GPX plus three GeoJSON files per sequence (track, raw telemetry points, per-image snapshots).
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
| Explicit opt-in | Horizon leveling (360 only) and keywords are off by default |
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
      ├── flat cameras: extract_frames_by_index()  (one accurate seek per frame, never rotated)
      └── MAX 360: max360_stitch_ffmpeg() / max360_stitch_max2sphere()
                   │ --level-horizon (smoothed IMU roll/pitch), re-centring yaw
                   ├── no:     pixels and GPano pose untouched (default)
                   ├── rotate: v360 sphere rotation, GPano pose = 0
                   └── exif:   pixels untouched, GPano pose = roll/pitch
      ▼
JPEG/PNG files
      │
      │ exiftool batch  (Software = "v2i.py v0.0.4")
      ▼
EXIF + XMP + GPano tags
      │
      │ export_track_files()
      ▼
<sequence>.gpx
<sequence>.geojson
<sequence>.telemetry.geojson
<sequence>.image-telemetry.geojson
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
| Leveling (360 only) | `_image_angles`, `_weighted_vector_angles`, `_limit_steps`, `max360_level_filter` | Derive a smoothed, step-capped roll/pitch and rotate the sphere pixels (with the re-centring yaw) |
| Metadata | `FrameMetadata`, `build_exiftool_args`, `write_metadata`, `SOFTWARE_TAG` | Build and apply EXIF/XMP tags, including the exporter version |
| Export | `write_gpx`, `write_geojson`, `write_points_telemetry`, `write_images_telemetry`, `write_manifest` | Serialise GPX/GeoJSON tracks, GeoJSON telemetry and run summaries |
| Runtime | `build_parser`, `validate`, `process_group`, `main` | CLI, validation, orchestration and exit codes |

### 2.3 Execution model

- **Sequential videos, parallel frames.** Videos are processed one at a time. Within a video, `--workers N` runs up to `N` independent ffmpeg extraction jobs.
- **MAX batch decode.** MAX stitches frames in bounded batches (`EXTRACT_CHUNK_SIZE = 60`; `MAX360_LEVEL_CHUNK_SIZE = 8` when leveling in `rotate` mode). Each batch decodes the source once; a failed seek batch retries from the start of the file.
- **Leveled MAX single pass (`--level-horizon`).** Consecutive frames whose leveling angles agree within `LEVEL_GROUP_TOLERANCE_DEG` (0.2°) form a run; each run is stitched with its roll, pitch and re-centring yaw inside the stitching `v360`, so leveling costs no extra pass. If a run fails, its frames fall back to the former two-pass path (lossless PNG, then one `v360=e:e` rotation per frame). Without leveling, the re-centring yaw is part of the stitching `v360` as well.
- **Telemetry caching.** VIRB FIT files are parsed once and reused across videos in the same run (`telemetry_cache`).
- **Failure isolation.** A `Fatal` error fails one video, not the run. `--fail-fast` stops at the first failure.

---

## 3. Supported cameras and hardware

### 3.1 Camera classes

| Class | Camera | Container / media | Video | Telemetry carrier | IMU | 360 | Leveling |
|---|---|---|---|---|---|---|---|
| `garmin-virb-ultra-30` | Garmin VIRB Ultra 30 | MP4/MOV, H.264 | Flat 4K/1080 | Sibling `.FIT` in `GMetrix/` or matched by name/time | `ACCL`, `GYRO`, `MAGN` in FIT | No | None (`--level-horizon` ignored) |
| `gopro-max` | GoPro Max | `.360` / MP4 / MOV, H.264/H.265 | Dual EAC hemispheres (4096×1344 or 2272×736) | Embedded GPMF `gpmd` | `ACCL`, `GYRO`, `MAGN`, `GRAV`, `CORI`, `IORI` | Yes | Sphere rotation **or** XMP-GPano pose |
| `gopro-max2` | GoPro Max2 | `.360`, H.265 | Dual EAC hemispheres (5888×1920) | Embedded GPMF `gpmd` (`GPS9`) | `ACCL`, `GYRO`, `MAGN`, `GRAV`, `CORI`, `IORI`, `MNOR` | Yes | Sphere rotation |
| `blackvue-dr900` | BlackVue DR900 series | MP4, H.264 | Flat dashcam | NMEA sentences in top-level `free` atoms | None | No | None |
| `gopro-hero` | GoPro HERO (flat) | MP4/MOV, H.264/H.265 | Single flat stream | Embedded GPMF `gpmd` | `ACCL`, `GYRO`, `MAGN`, `GRAV`, `CORI`, `IORI` | No | None (`--level-horizon` ignored) |

### 3.2 Garmin VIRB Ultra 30

- **Format requirements:** MP4/MOV with a single H.264 video stream. The camera writes FIT telemetry separately, usually in a sibling `GMetrix/` folder.
- **Telemetry linking order:**
  1. An explicit `--fit FILE`.
  2. The exact FIT filename stored in the MP4 container (`fit_reference`).
  3. The FIT whose filename start time is closest to the video's local creation stamp (within six hours).
  4. The only FIT candidate, if exactly one exists.
- **Segment alignment:** One FIT file may describe several recorded clips. Camera events (`video_start`, `video_split`, `video_end`) define segments. The segment whose local start time is closest to the MP4 creation time wins; duration proximity is the fallback.
- **Per-video timeline:** Video start is `video_start_fit`; capture time is `video_start_utc + pts`, including real milliseconds. The active telemetry window is `video_start_fit → video_start_fit + duration`.
- **Camera orientation:** FIT camera-orientation events (0/90/180/270) are applied to the accelerometer vector so the exported `ACCL` telemetry is expressed in the stored video's image frame. The accelerometer up/down sign is derived automatically from the camera data. Frames themselves are never rotated: `--level-horizon` is ignored for this flat camera.
- **Validated frame rates:** 24 and 30 fps classic recording. Variable-frame-rate sources are handled because selection uses real PTS.
- **Filename convention:** `VIRB[0-9]+.(mp4|mov)`.

### 3.3 GoPro Max and GoPro Max2

The MAX and the MAX2 are **separate cameras**: their own detection label, run-summary count, processing pass, output-folder suffix (`max`, `max2`) and camera id in the exports (`gopro-max`, `gopro-max2`). Both use the same 360 pipeline (EAC stitching, re-centring, leveling, heading offset); everything below applies to both unless the MAX2 is named.

- **Format requirements:** `.360`, MP4 or MOV with two matching EAC hemisphere streams and an embedded GPMF (`gpmd`) track. A single wide equirectangular stream is also accepted.
- **Known layouts:**
  - `4096×1344` (MAX): side 1376 + centre 1344 + side 1376, 32 px blend (64 px overlap crops).
  - `2272×736` (MAX): side 768 + centre 736 + side 768, 16 px blend.
  - `5888×1920` (MAX2): side 1984 + centre 1920 + side 1984, 32 px blend (64 px overlap crops). Measured on a MAX2 recording: content repeats 64 px apart across each lens seam, as on the MAX. The generic fallback for unknown sizes would assume 128 px here and doubles every seam, so this size has its own table entry. 928 unique + 64 blended + 928 unique pixels fill a 1920 px face exactly, so unlike the MAX no 624→640 stretch is needed.
- **Stitching:** The filter cross-blends the overlapping side seams, assembles a 3×2 EAC net, and reprojects with `v360=eac:equirect`. The default output is the camera's native size, four cube faces around: **5376×2688** for the MAX and **7680×3840** (8K) for the MAX2. `--width`/`--height` override it (one of them alone keeps 2:1). Equirectangular input keeps 5376×2688.
- **GoPro Max2:** detected as a MAX (device name `MAX2`); EXIF `Model` (and XMP `GPano:CaptureSoftware`) is `GoPro Max2`: the MAX writes `GoPro Max` into its own photos, and the MAX2 (which reports only `MAX2`) gets the same title case for consistency. The camera labels in logs and exports are `GoPro Max` / `GoPro Max2` too. Differences from the MAX:
  - GPS is `GPS9`, a GPMF "complex" (`?`) structure described by the stream's `TYPE` (`lllllllSS`: lat, lon, alt, 2D speed, 3D speed, days since 2000, seconds, DOP, fix), one structure per sample. There is no `GPSU`/`GPSF`/`GPSP`; each sample has its own fix time, fix type and DOP, and the first sample of a block anchors the camera clock (`STMP`) to UTC for the IMU and exposure streams.
  - `creation_time` is real UTC, and the camera's timezone is stored separately (`TZON`, see §4.6).
  - In time-lapse there is one GPMF payload and one GPS sample per video frame (0.5 s apart in the 0.5 s mode).
  - Extra streams (ignored): magnetic-north orientation `MNOR` with confidence `OCNF`, `DisparityMatrix`, face data, logs, compression scores.
  - The native 8K output costs time: a 6-image test spread over a 140 s time-lapse took 7.7 min, mostly decoding two 5888×1920 HEVC streams up to the last frame; one image near the start took 18 s.
- **TimeWarp / rate changes:** GPS span and video span are compared. A ratio outside 0.8–1.2 maps real GPS time onto playback time (`scale = video_span / gps_span`), so frame selection and timestamps stay aligned and no frame is emitted twice. The log names the mode: a standard time-lapse interval (0.5–60 s) or TimeWarp factor (2–30×) is reported as INFO (e.g. `time-lapse: 1 frame every 0.50 s of real time (15.0x speed-up, …)`); any other rate is a WARNING, because a variable rate such as TimeWarp Auto speed is still mapped linearly and frame times may drift.
- **Bad GPS readings (MAX and HERO):** while the receiver is acquiring, GoPro already reports a 2D fix but with DOP 99.99 and positions that jump hundreds of metres per second. Readings with DOP above `--max-dop` (default 10) are dropped, then any reading implying more than 70 m/s from the previous kept one (if rejections last 5 s, the new position is trusted instead). The log reports how many were rejected. Frames before the first kept reading are handled like any frame before the first GPS fix.
- **Clock spread:** the logged spread of GPS time vs camera clock ignores time stamps more than 1 s from the median (GoPro's first payload is often a few seconds stale) and says how many were ignored; the median offset is used either way.
- **Validated frame rates:** 24 and 30 fps, plus TimeWarp/timelapse recordings that play back at 30 fps.
- **Leveling:** The only camera that honours `--level-horizon` (off by default). Roll/pitch come from `GRAV`; the sphere pixels are rotated and the GPano pose stays zero. See §6.5.
- **Re-centring:** With a heading offset, the sphere is rotated so the direction of travel is the image centre (default; `--no-recenter-forward` keeps the lens view). See §6.4.
- **No valid GPS fix:** `--fallback-interval SECONDS` exports an unpositioned time grid while retaining the GPMF IMU, camera clock and TimeWarp scale.
- **Filename convention:** `GS[0-9A-Z]+.(360|mp4|mov)`; device name usually contains `MAX` (`GoPro Max`, `MAX2`).

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
- **Clock anchoring:** GPS UTC is authoritative. Without GPS, `STMP` is anchored to the container `creation_time`. The local offset comes from the camera's `TZON` setting (HERO11 and later, §4.6); without it, filenames and EXIF use UTC with a warning.
- **Validated frame rates:** 24 and 30 fps classic recording.
- **No GPS:** `--fallback-interval SECONDS` exports an unpositioned time grid with IMU snapshots when the clock is available.
- **Leveling:** None. `--level-horizon` is accepted but ignored for this flat camera; IMU roll/pitch still appear in the telemetry exports.
- **Filename convention:** `(GX|GH|GOPR|GOPRO)[0-9A-Z]*.(mp4|mov)`; device name usually contains `HERO`.

### 3.6 Detection evidence

Detection never guesses. Accepted evidence includes:

| Camera | Evidence |
|---|---|
| VIRB | Garmin container handler; `VIRB*` filename; Garmin container markers |
| MAX / MAX2 | GoPro handler plus `DVNM` containing `MAX`; `.360`; dual-video/EAC layout; `GS*` filename. The two are then told apart by `DVNM` (`MAX2` vs `GoPro Max`), else the camera model `MINF` in the file's Global Settings block, else the hemisphere size (5888×1920 = MAX2); the default is the MAX. |
| BlackVue | PittaSoft/BlackVue handler; `YYYYMMDD_HHMMSS_XX` filename; PittaSoft/BlackVue markers |
| HERO | GoPro handler plus `DVNM` containing `HERO`; single flat video stream with GPMF; `GX*`/`GH*`/`GOPR*` filename |

Ambiguous GoPro files (for example a generic GoPro handler with conflicting MAX/HERO evidence) are skipped with `ambiguous GoPro model metadata`. Unrecognised files are skipped with `unsupported camera (missing camera metadata)`.

### 3.7 Frame-rate and timelapse scope

| Camera | Modes validated | Notes |
|---|---|---|
| Garmin VIRB Ultra 30 | 24 fps, 30 fps | Classic recording; VFR handled by real PTS |
| GoPro Max | 24 fps, 30 fps, TimeWarp/timelapse at 30 fps playback | GPS span is scaled to video span |
| GoPro Max2 | 0.5 s time-lapse at 30 fps playback | One GPS sample per frame |
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

| Telemetry group | GoPro Max | GoPro HERO | Garmin VIRB | BlackVue |
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

GoPro's `GRAV` behaves as the *down* direction in a `(right, down, forward)` camera frame (verified on deliberately tilted MAX footage): a level camera reads approximately `[0, 1, 0]`, the first component goes negative when the camera's left side dips, and the third goes positive when the lens tilts down. It is a low-passed **apparent** gravity, so in a moving vehicle it also contains the vehicle's own acceleration (turns, braking) — see §6.5.

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

The pipeline does not export lens metadata. GoPro's per-frame shutter time and ISO are the only exposure values written. The following are the factual boundaries:

| Field | Availability | Pipeline behavior |
|---|---|---|
| Shutter / exposure time | GPMF `SHUT`, one value per frame (MAX, HERO) | Written to EXIF `ExposureTime` for each image |
| ISO | GPMF `ISOE`, one value per frame (MAX, HERO) | Written to EXIF `ISO` for each image |
| White balance gains / temperature | Present as GPMF stream names | Listed only |
| Focal length | Not present in supported streams | Not exported |
| Aperture | Not present in supported streams | Not exported |
| Lens distortion coefficients | Not present in supported streams | Not exported; stitching assumes the native EAC/equirect layout |
| Sensor temperature | Not present in supported streams | Not exported |

Do not expect EXIF `FocalLength`, `FNumber` or `LensModel` values from this tool (`ExposureTime`/`ISO` only for GoPro). Camera identity, GPS, orientation and provenance are the supported metadata surface.

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
| MAX2 (and any GoPro with `TZON`) | The camera's own timezone setting `TZON` (minutes, daylight saving included) from the "Global Settings" GPMF block in `moov/udta/GPMF` is the default source. `creation_time` is real UTC; `CDAT` there is the matching local wall clock. |
| MAX | No `TZON`. Container `creation_time` (camera-local wall clock) is compared with the first GPS UTC. `STMP` is added so later chapters compare against recording start. The difference is rounded to the nearest 15 minutes and rejected beyond ±14 hours. |
| HERO | GPS UTC is authoritative. The offset is `TZON` (HERO11 and later); older models without it fall back to UTC with a warning. |
| VIRB | FIT UTC is authoritative. Container `creation_time` is camera-local wall clock and is used for segment matching and as a last-resort anchor. |
| BlackVue | GPS UTC is authoritative. Filename local time is snapped to a quarter-hour offset and checked against GPS. |

An explicit `--timezone` always wins. `--time-base utc` forces UTC naming regardless of detection (opt-in; local time is the default). When nothing gives an offset, names and EXIF stay in UTC and a warning says so.

The log names the source, e.g. `camera-local time offset: +02:00 (camera TZON setting)` or `(container clock vs GPS)`.

**GPS sanity check.** A camera-derived offset (TZON or MAX clock) is compared with the solar offset of the first GPS fix (longitude / 15 h). More than `TZ_SOLAR_TOLERANCE_H` (3.5 h) apart means the camera is still set for another region, and a warning suggests `--timezone`. Real zones stay within about 3 h of solar time (western China, Spain in summer), so this catches a wrong continent, not a one-hour daylight-saving slip. An exact lookup of the timezone from the position (e.g. the `tzfpy` package plus `zoneinfo`) would also catch that, but needs an extra dependency and is not used.

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
| `--start-distance METERS` | float | `0.0` | `>= 0` | Skip this much travel before the first target. With `0`, frame index 0 is exported even if telemetry starts slightly later, as long as the first usable GPS fix comes within 2 s (real time) of the recording start; otherwise the sequence starts at the first fix and a warning is logged. |
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
| `--heading-offset DEGREES\|auto` | float or `auto` | none | `0`–`360`, `auto` | Clockwise angle from the direction of travel to the front lens. 360 videos are re-centred to the direction of travel; flat videos add it to the heading and get `-NNN` on their output folder. `auto` reads it from each video's folder name. See §6.4. |
| `--offset DEGREES\|auto` | float or `auto` | none | `0`–`360`, `auto` | Alias for `--heading-offset`. |
| `--recenter-forward` / `--no-recenter-forward` | flag | on | on/off | With a heading offset, rotate 360 images so the direction of travel is the image centre (no offset in the folder name). `--no-recenter-forward` keeps the lens view, adds the offset to the heading and appends `-NNN` to the output folder. Ignored for flat videos. |
| `--magnetic-declination DEGREES` | float | `0.0` | `-180`–`+180` | East-positive declination added to compass/IMU headings to obtain true north. |
| `--uere METERS` | float | `5.0` | `> 0` | User-equivalent range error used to estimate accuracy as `HDOP × UERE` when no measured accuracy exists. |
| `--max-dop DOP` | float | `10` | `>= 0` | Drop GoPro (MAX/HERO) GPS readings with DOP above this value; `0` keeps every reading. Raise it (e.g. `15`) to keep low-quality but consistent fixes. |
| `--level-horizon`, `--horizon` | flag | off | on/off | Level **360 (MAX)** photospheres: rotate the sphere so the horizon is level (GPano pose stays `0`). Flat videos ignore it and log that they export as recorded. The former values `rotate`/`exif`/`no` are rejected with an explanation. See §6.5. |
| `--horizon-gradient`, `--level-gradient` | flag | off | on/off | Opt-in, needs `--level-horizon`: follow the road gradient from GPS altitude so hills do not leave the horizon tilted. Uses the heading offset as the mount angle. See §6.5. |
| `--level-window SECONDS`, `--horizon-window` | float | `25.0` | `>= 0` | Real-time window of the Gaussian-weighted gravity average for each leveled image; changes between consecutive images are capped at 0.5°. Keep it long for mounted cameras; `0` uses the nearest single sample (handheld footage). |
| `--imu-heading` | flag | off | on/off | Derive a tilt-compensated compass heading from `MAGN` + `GRAV` (MAX/HERO), used only as a fallback where GPS gives no course; the GPS course always wins (see §6.4). |

### 5.3 Export and logging

| Option | Type | Default | Valid range / values | Functional description |
|---|---|---|---|---|
| `--track-export {both,gpx,geojson,none}` | enum | `both` | `both`, `gpx`, `geojson`, `none` | Select the spatial track files written per sequence. `none` still allows the detailed telemetry GeoJSON. |
| `--save-telemetry` | flag | off | on/off | Legacy alias for `--track-export both`. |
| `--telemetry-export {both,points,images,none}` | enum | `both` | `both`, `points`, `images`, `none` | Write the detailed telemetry as GeoJSON: every GPS/IMU point (`<sequence>.telemetry.geojson`), per-image snapshots (`<sequence>.image-telemetry.geojson`), both, or neither. |
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
| `--stationary-speed MPS` | float | `0.3` | `>= 0` | Compatibility option. Accepted so existing command lines keep working; the current track suppresses stationary course noise below 0.5 m/s internally (`COURSE_MIN_SPEED`), but changing this flag does not alter the threshold. |
| `--jitter METERS` | float | `0.5` | `>= 0` | Compatibility option. Accepted for legacy scripts; the unified track does not use a separate jitter cutoff. |

### 5.5 GoPro Max / Max2 (360) options

| Option | Type | Default | Valid range / values | Functional description |
|---|---|---|---|---|
| `--width WIDTH` | int | native (`5376` MAX, `7680` MAX2) | `>= 2`, even | Equirectangular output width. Must be even for yuv output. Alone, the height is width / 2. |
| `--height HEIGHT` | int | native (`2688` MAX, `3840` MAX2) | `>= 2`, even | Equirectangular output height. Use 2:1 with `--width`. Alone, the width is 2 × height. |
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
| `--offset` | `--heading-offset` | Same argument, same validation |
| `--horizon` | `--level-horizon` | Same switch |
| `--horizon-window` | `--level-window` | Same value |
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
5. With `--start-distance 0`, force frame index 0 for the first target when the first usable GPS fix is at most `FIRST_FIX_MAX_DELAY_S` (2 s, real time) after the recording start. Its position is held at the first fix, but its capture time is the frame's own (frames clamped before the first or after the last fix always get their own time, scaled for TimeWarp). If GPS locked later (recording started before GPS lock), frame 0 was taken somewhere the track never saw: the footage before the first fix is not exported and a warning says how long it was. If a forced frame 0 cannot be extracted, the video fails.

Gaps are allowed. Low frame rates and VFR sources produce fewer images rather than duplicate frames. `--max-images` truncates the plan.

### 6.4 Heading resolution

`resolve_heading()` applies this precedence:

1. `metadata_heading` — camera/container heading (`heading_source = "metadata"`).
2. `course` or `heading` — recorded course over ground (`heading_source = "gps_course"`).
3. `gps_course` — computed from neighbouring track positions (`heading_source = "gps_course"`). The window around each sample grows until its ends are `COURSE_BASELINE_M` (4 m) apart, capped at `COURSE_MAX_SPAN_S` (10 s of real time), so GPS jitter at walking pace or in a crawl does not turn into a random bearing. Samples slower than `COURSE_MIN_SPEED` (0.5 m/s), or with no course of their own (vehicle parked, typically at the start of a recording), take the nearest known course, since a mounted camera keeps facing the same way; such samples carry `gps_course_filled = true`.
4. `compass_heading` — tilt-compensated IMU/MAGN heading (`heading_source = "compass"`), only with `--imu-heading` and only where GPS gives no course at all. GoPro's magnetometer is uncalibrated (hard-iron offsets around 100 µT, about twice Earth's field); on test MAX recordings its heading disagreed with the GPS course by 40–250°, so it never overrides a course.
5. `none`.

`--magnetic-declination` is added only to compass headings. The EXIF `GPSImgDirection` uses the final true-north value of the image centre; `GPSTrack` carries course over ground.

#### Heading offset (camera not facing the direction of travel)

`--heading-offset DEGREES` is the clockwise angle from the direction of travel to the camera's front lens: `090` when the lens faces right (e.g. a MAX mounted with its screen to the left, as Mapillary recommends for bikes), `270` when it faces left, `180` backwards. Only `0`–`360` or `auto` is accepted; negative values, values above 360 and anything else are an error.

| Camera | What the offset does | Output folder |
|---|---|---|
| 360 (MAX, MAX2), default | The sphere is rotated (`v360` `yaw`, in the stitching pass, no extra cost) so the **direction of travel is the image centre**. The heading is then simply the course; the offset is not added to it. `UserComment` records `recentered_deg=NNN`. | `2026-11-05-max` (no offset) |
| 360 with `--no-recenter-forward` | Pixels keep the front lens at the centre; the offset is added to the heading. | `2026-11-05-max-NNN` |
| Flat (BlackVue, VIRB, HERO) | Pixels unchanged; the offset is added to the heading. Re-centring does not apply. | `2026-10-18-virb30-NNN` |

The compass fallback measures where the front lens points, so the offset is not added to it; with re-centring it is turned back by the offset.

**`--heading-offset auto`** reads the offset from **the folder each video is in**, so recordings must be sorted into folders by mount direction. The folder name is split on `-` and `_`; the offset is a part of exactly three digits (`000`–`360`) that is not the first part. It therefore needs a `-`/`_` before it and a `-`/`_` or the end of the name after it:

| Folder name | Offset |
|---|---|
| `2026-11-05-max-090` | 090 |
| `2026-11-05-max` | none (0) |
| `2026-11-05-max-270-30fps` | 270 |
| `2026-11-05_max_090_120fps` | 090 |
| `2026-10-31-max-090-standard-30fps-varazdin-horizont` | 090 |
| `2026-10-18-virb-270-4k-24fps` | 270 |

Camera names and formats such as `max`, `max2`, `max360`, `30fps`, `120fps`, `1080p`, `4k` or `timelapse05` are never separate 3-digit parts, and dates use 4- and 2-digit parts, so they are ignored. A folder whose name *starts* with three digits (`090-trip`) has no offset. A 3-digit part above 360 (`-400-`) or two different offsets in one name (`-090-…-270`) stop the run with an error before anything is exported. Each video's folder offset is logged at the start. When the same video file name occurs in several folders, a warning is logged: their 360 images can land in the same output folder, where an existing image is skipped rather than overwritten.

### 6.5 Horizon leveling (360 only)

`--level-horizon` (alias `--horizon`) is an on/off switch, **off by default**. When on, every MAX photosphere is rotated on the sphere so the horizon is level, using the camera's own gravity sensor. The pixels carry the correction, so `XMP-GPano:PoseRollDegrees`/`PosePitchDegrees` are always `0` and every viewer shows a level horizon. Older command lines with a value (`--level-horizon rotate|exif|no`) stop with an explanation: `rotate` is now plain `--level-horizon`, and `exif` was removed.

| Camera | Behaviour |
|---|---|
| MAX, MAX2 | Leveled when the switch is on. |
| HERO | Switch ignored (logged); IMU roll/pitch still appear in telemetry. |
| VIRB | Switch ignored (logged); calibrated `ACCL` (FIT message 167 + orientation matrix) still appears in telemetry. |
| BlackVue | No IMU; switch ignored. |

**Angle convention**

- Camera roll: positive = clockwise / right side down. Camera pitch: positive = camera tilted down.
- Both angles are independent `atan2` angles of the gravity vector: `roll = atan2(right, down)`, `pitch = atan2(forward, down)`.
- `v360`'s signs were established empirically on deliberately tilted MAX footage: a clockwise camera roll is undone with `roll=-roll_deg`, and `pitch=+P` makes the view look *up*, which levels a camera tilted down.
- With re-centring (§6.4) the same `v360` call also gets the `yaw`, with rotation order `rorder=pry`: leveling happens in the camera's own frame first and the yaw last. With marker images this matched "level, then a separate yaw pass" to 57 dB PSNR; the default order tilts about the wrong axes once a yaw is present.
- `PoseHeadingDegrees`/`InitialViewHeadingDegrees` are written only when a heading is known; an unknown heading is left absent rather than written as `0` (north).

**Smoothing (`--level-window`, alias `--horizon-window`)**

`GRAV` is GoPro's low-passed *apparent* gravity. In a vehicle it also contains the vehicle's own acceleration: a turn or braking phase shows up as a tilt of several degrees for a few seconds, even though a mounted camera has not moved. One sample per image is therefore not a usable tilt.

- For every image, the `GRAV` samples within `--level-window SECONDS` of **real (UTC) time** centred on the capture time are combined as a **Gaussian-weighted mean of the unit gravity vectors** (sigma = window / 4). Unlike the median used before, the mean changes smoothly as the window slides. Default `25` s.
- Near the start and end of a recording, the value of the first/last full window is held instead of using a one-sided window.
- The change between consecutive images is capped at `LEVEL_MAX_STEP_DEG` (`0.5°`). A forward and a backward rate-limited pass are averaged, so the correction does not lag in either direction.
- On four test drives (two 0.5 s time-lapses, two normal videos, 5 m spacing), the largest image-to-image change went from 1.9–3.3° (20 s median) to 0.5°, with 95% of the changes at or below about 0.3°. The largest jumps used to occur around stops, where images are seconds apart.
- Down-weighting samples taken while the vehicle accelerates, brakes or turns (from GPS) is implemented but off (`LEVEL_ACCEL_WEIGHTING`) and **marked for possible removal**. On the same drives it changed the angles by about 0.4° and made image-to-image changes slightly larger, without evidence of a more accurate horizon. A later analysis showed why: on hilly roads most of the tilt variation is the real road gradient (below), which down-weighting cannot tell apart from acceleration. Only the flat GS010153 drive was dominated by acceleration (R² 0.77).
- The window is in real time, not image count, so TimeWarp and normal video behave identically. Use `--level-window 0` (nearest sample, no smoothing, no cap) for handheld footage where the tilt genuinely changes from frame to frame.
- Raw `roll_deg`/`pitch_deg` (nearest single sample) remain in telemetry; `level_roll_deg`/`level_pitch_deg` record the applied value and `level_window_s` the window used.
- Rotations below `0.05°` are skipped to avoid needless resampling.
- **Single pass.** The rotation is applied inside the stitching `v360=eac:equirect` (same options and `rorder=pry` as the old second pass; one frame matched the two-pass result at 36 dB PSNR, the normal difference between one and two resamplings). Because one `v360` call can only hold one rotation, consecutive frames whose roll and pitch agree within `LEVEL_GROUP_TOLERANCE_DEG` (0.2°, about 3 px at the edge of a 5376 px panorama) form a run that shares the run's mid value. On four drives that gave runs of 4–5 images. Twelve leveled, re-centred frames of GS010153 took 89 s instead of 215 s (2.4× faster), and the output was visually identical, slightly sharper. A run that fails falls back to the old path (lossless PNG, then one `v360=e:e` rotation per frame, batches of 8). The log shows `N leveled frame(s) in M single-pass run(s)`.
- Per-frame `v360` commands (`sendcmd`) were tried first and dropped: `v360` rebuilds its remap tables on every command (about 40% slower), and the result did not match.

**Road gradient (`--horizon-gradient`, opt-in)**

On hilly roads the vehicle, and the camera with it, really pitches: GPS altitude explained 46–61% of the tilt variation on three drives with gradients up to ±15%, with a size matching the geometry (1% gradient ≈ 0.57° of tilt), on the camera axis the mount angle predicts. The 25 s window and the 0.5° cap partly average this real tilt away, so on hills the horizon stays a little tilted. With `--horizon-gradient`:

- The road gradient is computed from GPS altitude (averaged over ±2 s, differenced over ±30 m of travel, clamped to ±30%; spans under 39 m count as flat).
- The tilt it causes is projected onto the camera with the mount angle (the heading offset; 0 when none is given): `pitch = −atan(g)·cos(mount)`, `roll = +atan(g)·sin(mount)`. The signs match regressions on the 000, 090 and 270 drives.
- That tilt is removed from every gravity sample before the 25 s average and the image's own gradient tilt is added back afterwards, so hills are followed without the window's lag.
- The 0.5° cap applies to the smoothed part; the total may change by up to `LEVEL_GRADIENT_MAX_STEP_DEG` (1.0°) per image so crests are not delayed.

Measured on images taken during steady driving (|acceleration| < 0.3 m/s²), against a 3 s gravity average as reference (the camera's own reading, not ground truth):

| Drive (mount) | Default: median / p90 / max | `--horizon-gradient` | Largest image-to-image change |
|---|---|---|---|
| GS013342 (270, time-lapse) | 0.68 / 1.41 / 2.54° | 0.65 / 1.15 / 2.54° | 0.5° → 1.0° |
| GS013336 (000, time-lapse) | 0.58 / 1.27 / 2.39° | 0.49 / 1.02 / **3.67°** | 0.5° → 1.0° |
| GS013324 (270, video, steepest) | 0.83 / 1.83 / **7.19°** | 0.85 / 1.84 / **3.94°** | 0.5° → 1.0° |
| GS010153 (090, flat) | 1.16 / 1.85 / 2.08° | 1.27 / 1.63 / 2.06° | 0.5° → 1.0° |

It helps most on steep roads and in the 90th percentile, but it is not better everywhere (GS013336's worst image got worse), and the GoPro's GPS altitude noise is its limit. That is why it is opt-in; try it on hilly routes and check a few images on crests. Doppler vertical speed (3D minus 2D GPS speed) was tested as a cleaner gradient source and is not usable (correlation 0.2–0.28, about 0 in time-lapse).
- Leveling happens during export. Existing images are skipped unless `--overwrite` is used.

### 6.6 MAX / MAX2 stitching pipeline

1. Probe the file; select the largest matching EAC hemisphere pair, or a single wide equirectangular stream.
2. Build a filter graph:
   - Split each hemisphere into side/centre/side regions.
   - Cross-blend the overlap seams (64 px crops on the MAX 4096×1344 and the MAX2, 32 px on 2272×736).
   - Assemble the six faces into a 3×2 EAC net.
   - Reproject with `v360=eac:equirect:interp=…:w=…:h=…`, including the re-centring `yaw` when there is one (§6.4).
3. Process selected frames in batches:
   - Seek to the first PTS of each batch when the timeline allows it.
   - Retry the batch from the start of the file if the seek produces a bad result.
4. Without `--level-horizon`, encode directly to the target format.
5. With `--level-horizon`, group consecutive frames whose angles agree within 0.2° and stitch each group with its roll, pitch and re-centring yaw in the same `v360` (single pass). A group that fails falls back to lossless PNG intermediates and one `v360=e:e` rotation per frame.
6. `--workers` parallelises the per-frame sphere rotations.
7. `max2sphere` shares the same two-pass leveled path after it produces the equirectangular video.

`--keep-filter` copies each batch graph to `<sequence>.filter` (or `<sequence>.NN.filter` when there are multiple batches).

### 6.7 EXIF/XMP policy

- `DateTimeOriginal`, `CreateDate`, `ModifyDate` are camera-local when the offset is known, otherwise UTC.
- `SubSecTimeOriginal` carries the capture millisecond.
- `OffsetTimeOriginal`/`OffsetTime` are written only when the offset is known.
- All GPS date/time tags are UTC. `GPSTimeStamp`, `XMP-exif:GPSDateTime` and `XMP:MetadataDate` carry milliseconds, so time-lapse frames within one second still sort correctly.
- `GPSImgDirection`/`GPSTrack` (EXIF and XMP-exif) are written with `Ref = T` (true north). `GPSImgDirection` and the GPano `PoseHeadingDegrees`/`InitialViewHeadingDegrees` carry the same value: the compass direction of the image centre (for MAX, the front lens).
- `GPSHPositioningError` is written when an accuracy exists; `GPSDOP` when an HDOP exists.
- `ColorSpace = 1` (sRGB).
- GoPro (MAX/HERO): `ExposureTime` and `ISO` come from the camera's per-frame `SHUT`/`ISOE` telemetry (nearest frame within 1 s of the capture time). They are real per-frame values, e.g. useful to spot motion blur; other lens/exposure tags are not written.
- All tags are written with exiftool `-n`, so values must be raw (`T`, `K`, `2 3 0 0`, `YYYY:mm:dd HH:MM:SS.sssZ`); exiftool drops print-form values such as `True North` with only a warning.
- **Camera firmware:** EXIF `Software` (defined as "software or firmware of the camera") and `XMP-aux:Firmware` carry the camera firmware, as the cameras' own photos do (e.g. `H19.03.02.02.00`, `VIRB Ultra 30 FW 4.80`). When no firmware is known, `Software` falls back to the exporter version. The serial number goes to `SerialNumber`, `CameraSerialNumber` and `XMP-aux:SerialNumber`. For GoPro (MAX, MAX2, HERO) it is `CASN` from the file's "Global Settings" block, as on the camera's own photos (e.g. `C3351325080196`); ffprobe does not show it. Note that Panoramax publishes EXIF, so the serial is public.
- **Exporter version:** `XMP:CreatorTool` and `XMP-GPano:StitchingSoftware` carry `SOFTWARE_TAG`, built from `__version__` as `v2i.py v0.0.4`.
- **Provenance note:** only `UserComment`, e.g. `heading_source=gps_course; position_source=interpolated`, plus per-camera flags (`camera_orientation_deg` for VIRB, `unpositioned` for HERO). It holds only facts without an EXIF tag of their own. `ImageDescription`, `XPComment` and `XMP:Description` and `XMP:Title` are not written, because galleries and Panoramax show them as the photo caption or title. Video, frame, PTS, distance, accuracy source and DOPs are in the per-image telemetry GeoJSON. Provenance never goes to keywords.
- `Keywords`/`XMP:Subject` are written only when `--keywords` is passed.
- GPano/XMP-GPano tags are written only for MAX panoramic images.
- `PoseRollDegrees`/`PosePitchDegrees` are always `0`: with `--level-horizon` the pixels are leveled, without it the source tilt is kept as recorded.

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
        ├── <sequence>.geojson                 # positions only (track + image points)
        ├── <sequence>.telemetry.geojson       # every GPS and IMU sample
        ├── <sequence>.image-telemetry.geojson # per-image GPS/IMU snapshot
        ├── <sequence>.track.jsonl        # only with --sidecars
        └── <sequence>.filter[.NN]        # only with --keep-filter
```

- The date folder is based on the **first exported frame**, not on the source filename.
- Camera suffixes: `virb30`, `max`, `max2`, `blackvue`, `hero`.
- The sequence name is the source stem. Duplicate stems in nested folders are prefixed with the relative directory (`subdir__VIRB0001`); no numeric `_NNNNNN` counter is used.
- An explicitly supplied heading offset is appended to the camera folder: `YYYY-MM-DD-virb30-090`.
- Images that cross midnight stay in the same sequence folder; their filenames carry the date.
- A `-2`/`-3` suffix is used only when two frames selected in the same run share a millisecond.

### 7.2 Export format matrix

| Format | File | When written | Content |
|---|---|---|---|
| JPEG / PNG | `*.jpg`, `*.png` | Always (subject to skip/overwrite) | Geotagged still image |
| EXIF/XMP | Inside image | Unless `--no-exif` or exiftool missing | Camera, firmware (`Software`), GPS, time, provenance note (`UserComment`), `CreatorTool = v2i.py v<version>`, GPano |
| GPX 1.1 | `<sequence>.gpx` | `--track-export both\|gpx` | Track points with UTC time, elevation, speed/course extensions |
| GeoJSON RFC 7946 | `<sequence>.geojson` | `--track-export both\|geojson` | LineString track plus one Point feature per image |
| Points telemetry (GeoJSON) | `<sequence>.telemetry.geojson` | `--telemetry-export both\|points` | FeatureCollection: GPS LineString, one Point per GPS sample, one feature per IMU sample positioned on the track |
| Image telemetry (GeoJSON) | `<sequence>.image-telemetry.geojson` | `--telemetry-export both\|images` | FeatureCollection: one Point feature per image with GPS/IMU snapshot properties |
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
        "camera": "gopro-max",
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
        "camera": "gopro-max",
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

### 7.5 Points telemetry GeoJSON schema

`<sequence>.telemetry.geojson` is an RFC 7946 `FeatureCollection`. Run metadata is carried as top-level foreign members (allowed by RFC 7946 §6.1), so standard GIS tools load the file as-is while the summary stays available.

- **GPS stream:** one `LineString` feature (`kind = "track"`, only when ≥ 2 positions exist) followed by one `Point` feature per raw GPS sample (`kind = "sample"`).
- **IMU streams** (`ACCL`, `GYRO`, `MAGN`, `GRAV`, `CORI`, `IORI`, `ORIN`): one feature per sample. Each sample is placed on the GPS track by interpolating the position at its `t_s` (`position_source = "interpolated"`). Samples that fall outside the GPS time span, or sequences without a GPS track (fallback export), are written as **unlocated** features with `"geometry": null`, which the RFC permits.
- **`streams`** summarises each stream: `unit`, `components`, `point_count` and, for IMU, `located_point_count`.

```json
{
  "type": "FeatureCollection",
  "generator": "v2i.py",
  "generator_version": "0.0.4",
  "camera": "gopro-max",
  "camera_label": "GoPro Max",
  "video": "GS013338.360",
  "sequence": "GS013338",
  "video_start_utc": "2026-10-31T10:13:24.821Z",
  "streams": {
    "gps":  {"point_count": 2, "track_point_count": 2},
    "GRAV": {"unit": "g", "components": ["x", "y", "z"], "point_count": 1, "located_point_count": 1},
    "ACCL": {"unit": "m/s^2", "components": ["x", "y", "z"], "point_count": 1, "located_point_count": 0}
  },
  "distance_interval_m": 5.0,
  "time_base": "local",
  "feature_count": 5,
  "features": [
    {
      "type": "Feature",
      "geometry": {"type": "LineString", "coordinates": [[16.12345678, 46.12345678, 123.45], [16.12351234, 46.12350123, 123.6]]},
      "properties": {"stream": "gps", "kind": "track", "name": "GS013338", "point_count": 2}
    },
    {
      "type": "Feature",
      "geometry": {"type": "Point", "coordinates": [16.12345678, 46.12345678, 123.45]},
      "properties": {
        "stream": "gps", "kind": "sample",
        "t": 0.0, "lat": 46.12345678, "lon": 16.12345678, "alt": 123.45,
        "speed": 5.2, "speed2d": 5.2, "speed3d": 5.3, "fix": 3,
        "hdop": 0.5, "vdop": 0.7, "pdop": null, "dop": 0.5,
        "utc": "2026-10-31T10:13:24.821Z", "stmp_us": 1234567, "t_s": 0.0
      }
    },
    {
      "type": "Feature",
      "geometry": {"type": "Point", "coordinates": [16.12348456, 46.12347901, 123.52]},
      "properties": {
        "stream": "GRAV", "kind": "sample", "unit": "g",
        "utc": "2026-10-31T10:13:25.321Z", "t_s": 0.5, "stmp_us": 1734567,
        "x": 0.01, "y": 0.02, "z": 0.99,
        "roll_deg": -1.157, "pitch_deg": 0.579, "angle_source": "GRAV",
        "position_source": "interpolated"
      }
    },
    {
      "type": "Feature",
      "geometry": null,
      "properties": {
        "stream": "ACCL", "kind": "sample", "unit": "m/s^2",
        "utc": "2026-10-31T10:13:30.000Z", "t_s": 5.179,
        "x": 0.12, "y": 9.79, "z": -0.04
      }
    }
  ]
}
```

VIRB files add a `calibration` member keyed by sensor (`ACCL`, `GYRO`, `MAGN`) with `factor`, `divisor`, `level_shift`, `unit`, optional `offset_cal` and optional `orientation_matrix`.

### 7.6 Image telemetry GeoJSON schema

`<sequence>.image-telemetry.geojson` is a `FeatureCollection` with exactly one `Point` feature per exported image. The feature `properties` hold the image record, the nearest GPS sample (`gps`) and the IMU snapshot nearest to the capture time (`imu`). Unpositioned images (fallback time grid) are written with `"geometry": null`.

```json
{
  "type": "FeatureCollection",
  "generator": "v2i.py",
  "generator_version": "0.0.4",
  "camera": "gopro-max",
  "camera_label": "GoPro Max",
  "video": "GS013338.360",
  "sequence": "GS013338",
  "video_start_utc": "2026-10-31T10:13:24.821Z",
  "image_count": 1,
  "distance_interval_m": 5.0,
  "time_base": "local",
  "feature_count": 1,
  "features": [
    {
      "type": "Feature",
      "geometry": {"type": "Point", "coordinates": [16.12345678, 46.12345678, 123.45]},
      "properties": {
        "sequence": "GS013338",
        "video": "GS013338.360",
        "camera": "gopro-max",
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
        "level_pitch_deg": 0.6,
        "level_mode": "exif"
      }
    }
  ]
}
```

`level_roll_deg`/`level_pitch_deg` appear only when leveling ran on a MAX video and an angle was applied. IMU snapshots outside a one-second window are `null`. BlackVue uses `"imu": {}` because the hardware has no IMU.

### 7.7 Run manifest schema

```json
{
  "generator": "v2i.py",
  "camera": "mixed",
  "camera_label": "Mixed cameras",
  "cameras": ["blackvue-dr900", "gopro-max"],
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
- The GPX track is empty and `<sequence>.geojson` has no geometry feature.
- `<sequence>.telemetry.geojson` has no GPS features; IMU samples are still exported as unlocated features (`"geometry": null`).
- `<sequence>.image-telemetry.geojson` features have `"geometry": null` but still carry IMU snapshots when the camera clock can anchor them.
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

# GoPro Max TimeWarp / timelapse
python3 v2i.py -i video/2026-10-26-max-000-timelapse05-30fps -o images/ -d 2 --start-distance 0

# BlackVue DR900
python3 v2i.py -i video/2019-07-12-blackvue-standard -o images/ -d 5 --start-distance 0

# HERO with no GPS: unpositioned time-lapse
python3 v2i.py -i video/2025-08-16-hero-standard-nogps -o images/ --fallback-interval 30

# Keep the source tilt (default)
# (flat VIRB/HERO/BlackVue frames always export as recorded)
python3 v2i.py -i video/ -o images/ -d 7

# Leveled MAX: rotate the sphere pixels, parallel v360 passes
python3 v2i.py -i video/2026-10-31-max-000-horizont -o images/ -d 2 --horizon --workers 4

# Mount offsets from the folder names (…-max-090-…): 360 images are
# re-centred to the direction of travel, flat videos get -NNN folders
python3 v2i.py -i video/ --recursive -o images/ -d 5 --heading-offset auto

# Handheld footage whose tilt genuinely changes every frame: no smoothing
python3 v2i.py -i video/2026-10-31-max-000-horizont-nogps -o images/ --fallback-interval 0.2 --level-horizon --level-window 0

# Dry run and manifest
python3 v2i.py -i video/ -o images/ -d 7 --dry-run
python3 v2i.py -i video/ -o images/ -d 7 --manifest run.json
```

### 8.2 Re-runs and overwrite

- Without `--overwrite`, an image whose path already exists is skipped and counted in the summary.
- `--overwrite` rewrites the same path in place; it never creates suffixed duplicates.
- Run with `--overwrite` after changing `--level-horizon`, the heading offset/re-centring or the time base to regenerate existing images.
- A `-2`/`-3` suffix is reserved for two frames selected in the same run that share a millisecond.

### 8.3 Performance tuning

| Goal | Setting |
|---|---|
| Faster flat extraction | `--workers N` (one ffmpeg per frame) |
| Lower CPU | `--workers 1`, `--threads 1` |
| Faster MAX leveling | Already single pass; `--workers N` only matters for the two-pass fallback |
| Lower temporary disk use | MAX batches hold at most 60 frames (8 in the two-pass fallback) |
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
| Leveled vehicle images tilt by a few degrees, differently per image (worst in TimeWarp) | GoPro `GRAV` contains the vehicle's own acceleration from turns/braking | Keep `--level-window` at the default or raise it (e.g. `30`); never use `0` for a mounted camera. Consecutive images never differ by more than 0.5° |
| Leveled handheld images under-correct | The time-window median flattens genuine per-frame tilt | Use `--level-window 0` |
| `--level-horizon applies to 360 videos only` | `--level-horizon` passed for a flat VIRB/HERO/BlackVue video | Expected; flat frames are exported as recorded and the IMU angles stay in telemetry |
| `--level-horizon no longer takes a value` | Old command line `--level-horizon rotate`/`exif`/`no` | Use `--level-horizon` (former `rotate`) or leave it out (former `no`); `exif` was removed |
| `--heading-offset must be from 0 to 360 degrees` | Negative or too large offset | Use `0`–`360`, e.g. `270` instead of `-90` |
| `--heading-offset auto: folder … has more than one heading offset` | Two 3-digit parts in a folder name | Rename the folder so it carries one offset |
| 360 images face sideways / backwards | Wrong or missing heading offset | Check the mount direction: `090` = front lens to the right of travel; use `--heading-offset auto` with labelled folders |
| Local filenames are UTC | Camera has no local clock evidence | Pass `--timezone +HH:MM` |
| MAX leveling is slow | Many single-pass runs (angles change a lot) or the two-pass fallback was used (see the log) | Reduce output size, or leave `--level-horizon` off |
| Leveled images still tilt on hills | The smoothing window averages the real road gradient away | Try `--horizon-gradient` (opt-in) |
| Looking for `*.telemetry.json` | Telemetry exports switched to GeoJSON | Read `<sequence>.telemetry.geojson` / `<sequence>.image-telemetry.geojson` (RFC 7946 FeatureCollections) |

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
- Horizon leveling: `--level-horizon`/`--horizon` switch (old values rejected), 360-only gating (flat cameras ignore it), Gaussian real-time window (`--level-window`) with a 0.5° per-image cap, MAX sphere rotation with zero pose values, VIRB orientation/sign handling in telemetry.
- Heading offset: `0`–`360`/`auto` validation, folder-name parsing, 360 re-centring (`yaw` sign checked with marker images and real footage; `rorder=pry` for combined leveling), `-NNN` folders only for flat videos or `--no-recenter-forward`. Sign conventions were validated visually on `video/2026-10-31-max-000-horizont-nogps` (camera deliberately rolled to ±40° and pitched down ~10–19°).
- Detailed GPS/IMU telemetry GeoJSON exporters: FeatureCollection structure, track LineString, IMU samples positioned on the track, null-geometry handling, per-image nearest-sample matching and `--telemetry-export`.
- EXIF `Software`/`XMP:CreatorTool`/`XMP-GPano:StitchingSoftware` carry `v2i.py v<__version__>`.

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
| GoPro `GRAV` | Down direction in `(right, down, forward)`; level = `[0, 1, 0]`, left side down → first component negative, lens down → third component positive |
| VIRB `ACCL` | Raw counts → `(raw − level_shift) / divisor × factor`, then orientation matrix |
| MAX `v360` leveling (`rotate`) | `roll=-roll_deg`, `pitch=+pitch_deg` (empirically verified) |
| Flat images | Never rotated; `--level-horizon` is ignored for non-360 cameras |

### 10.5 Selection and naming constants

| Constant | Value | Meaning |
|---|---|---|
| `DEFAULT_EXTENSIONS` | `.mp4,.mov,.m4v,.360` | Files scanned by default |
| `SOFTWARE_TAG` | `v2i.py v0.0.4` | `SCRIPT_NAME` + `__version__`; written to EXIF `Software`, `XMP:CreatorTool`, `XMP-GPano:StitchingSoftware` |
| `LEVEL_WINDOW_DEFAULT` | `25.0` | Default `--level-window` in real-time seconds |
| `LEVEL_MAX_STEP_DEG` | `0.5` | Largest leveling change between consecutive images |
| `LEVEL_ACCEL_WEIGHTING` | `False` | Down-weight gravity samples during acceleration (implemented, off, candidate for removal) |
| `LEVEL_GROUP_TOLERANCE_DEG` | `0.2` | Leveled frames within this angle share one single-pass rotation |
| `LEVEL_GRADIENT_BASELINE_M` | `30.0` | `--horizon-gradient`: altitude differenced over ± this travel |
| `LEVEL_GRADIENT_MAX_STEP_DEG` | `1.0` | `--horizon-gradient`: largest total change between images |
| `FIRST_FIX_MAX_DELAY_S` | `2.0` | Frame 0 is forced only if GPS was fixed within this many seconds |
| `EXTRACT_CHUNK_SIZE` | `60` | Unleveled MAX batch size |
| `MAX360_LEVEL_CHUNK_SIZE` | `8` | MAX `rotate`-leveled batch size |
| `max360_DEFAULT_WIDTH` | `5376` | Default equirectangular width |
| `max360_DEFAULT_HEIGHT` | `2688` | Default equirectangular height |
| `virb_DEFAULT_FIT_DIR` | `gmetrix` | Default FIT folder name |
| `virb_DEFAULT_DISTANCE` | `5.0` | Historical VIRB default (CLI default is also 5.0) |
| `virb_DEFAULT_QUALITY` | `95` | Historical VIRB default (CLI default is also 95) |
| `EARTH_RADIUS_M` | `6371008.8` | Haversine radius |

### 10.6 Measured sample evidence

| Sample | Notes |
|---|---|
| `video/2026-10-26-max-000-timelapse05-30fps/GS013336.360` | GoPro Max, DVNM `GoPro Max`, dual EAC 4096×1344, 1208 frames / 40.34 s; GPS span 604.57 s scaled to video span; equirect 5376×2688 with full GPano tags |
| `video/2025-08-16-hero-standard-nogps/GX011785.MP4` | HERO11 Black, GPMF IMU but no GPS5/GPSU/GPSF/GPSP → skipped by default; fallback exports UTC millisecond names |
| `video/2026-10-05-virb-1080-24fps/VIRB0996.MP4` + `GMetrix/2026-10-05-07-53-07.fit` | 9108 GPS samples, IMU present; camera events align the segment; offset `+02:00` |
| `video/2026-10-18-virb-090-4k-30fps/VIRB0782.MP4` | VIRB Ultra 30; real capture milliseconds in filenames (e.g. `2026-10-18_16-18-37-111.jpg`) |
| `video/2026-10-18-virb-270-4k-24fps/VIRB0008.MP4` | VIRB Ultra 30 at 24 fps; offset `+02:00` |
| `video/2026-10-31-max-000-horizont-nogps/GS013338.360` | No valid GPS fix; `--fallback-interval` exports unpositioned frames with full IMU telemetry and an empty GPS track |
| BlackVue samples | NMEA in `free` atoms; GGA/RMC/VTG/GSA parsed; clock reconciled to GPS; course from RMC/VTG |

### 10.7 Version history

| Version | Highlight |
|---|---|
| `0.0.4` | GoPro Max2 support (`GPS9` complex-type decoding, 5888×1920 layout, native 7680×3840 output, EXIF model `GoPro Max2`, title case like the MAX's own `GoPro Max`); GoPro Max and GoPro Max2 are separate cameras (ids `gopro-max`/`gopro-max2`, output folders `-max`/`-max2`; the old `gopro-max-360` id and `-max360` folder suffix are gone); camera timezone from GoPro `TZON` (MAX2, HERO11+) with a GPS-longitude sanity check; GoPro serial number (`CASN`) in EXIF. MAX: `--heading-offset` 0–360 or `auto` (from folder name), recentre-forward by default, `--level-horizon`/`--horizon` opt-in (single pass, smoothed) with optional `--horizon-gradient`; bad-GPS rejection (`--max-dop`); camera firmware in `Software`/`XMP-aux:Firmware`, provenance only in `UserComment`, `XMP:Title` removed; exposure time/ISO from GPMF; frame-0/clamped-time fix |
| `0.0.3` | Unified single-file pipelines, per-camera telemetry, timezone detection, universal unpositioned fallback. Horizon leveling restricted to 360 (MAX) videos via `--level-horizon {no,rotate,exif}`; telemetry exports switched from `*.json` to RFC 7946 `*.geojson`; exporter version written to EXIF `Software` |
| Earlier | Per-camera scripts and modules, merged into `v2i.py` |

---

## 11. Further research

### 11.1 GPS-aided attitude filter (horizon leveling)

Today's leveling smooths GoPro's `GRAV`, an apparent gravity that also contains the vehicle's own acceleration, and optionally adds the road gradient from GPS altitude (§6.5). The principled replacement is a GPS-aided attitude filter:

- **Propagate** the camera's attitude with the gyroscope (`GYRO`, ~200 Hz in normal video), which follows real tilt changes (hills, crests, bike lean) without lag.
- **Correct** its slow drift with the accelerometer (`ACCL`) after subtracting the vehicle's own acceleration, computed from GNSS: longitudinal (change of speed), lateral (speed × turn rate, from GNSS course or `GYRO` yaw rate) and vertical (from altitude).
- **Benefits:** no smoothing window or step cap is needed; braking, turns and hills are separated physically instead of averaged.
- **Limits:** only for normal video. In time-lapse the camera stores one IMU sample per frame (2 Hz real time), too sparse to integrate the gyroscope, so time-lapse keeps the current method. `CORI` (GoPro's own gyro-fused orientation) might shortcut the propagation step, but its frame convention relative to `GRAV` could not yet be confirmed (3–10° differences in a first comparison).

**RTK GNSS recordings.** RTK GNSS logs recorded alongside future videos are the natural input and reference for this work:

- Centimetre-level position and altitude give a clean road gradient and vehicle acceleration. The GoPro's own GPS altitude noise is what limits `--horizon-gradient` today, and its Doppler vertical speed is unusable.
- They are a reference to validate the existing leveling, the `--horizon-gradient` numbers above, heading (course over ground), and image positions.
- Suggested order: first import RTK tracks as a reference and re-measure the tables in §6.5, then build the filter.

### 11.2 Bike lean

Mapillary recommends 0.5 s time-lapse for bikes. In a turn a bike leans by `atan(a_lateral / g)`, about 14° at 18 km/h in a 10 m-radius turn. GoPro's apparent gravity stays aligned with the leaning bike, so leveling cannot see the lean. The fix is the opposite of acceleration down-weighting: *add* the lateral acceleration back as roll, projected with the mount angle like the road gradient. It needs bike recordings to validate and is deferred until they exist.

