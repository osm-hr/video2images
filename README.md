# video2images

Extract geotagged still images from dash/helmet cameras at fixed GPS distance intervals. Only the Python standard library is used; `ffmpeg`, `ffprobe`, `exiftool` must be on `PATH`. The optional `max2sphere` binary is only used if explicitly requested.

## Supported cameras

`v2i.py` is the single entry point for all supported cameras. It detects the camera per file, groups the videos and runs the matching pipeline, so one input tree may mix manufacturers.

| Camera | Telemetry | 360 stitching |
|---|---|---|
| Garmin VIRB Ultra 30 | FIT (sibling `*.fit` in `GMetrix/` or matched by filename/time) | no |
| GoPro MAX 360 (`.360`/dual-EAC) | GPMF (embedded `gpmd`) | yes (EAC → equirectangular via `ffmpeg v360`, default 5376×2688) |
| BlackVue DR900 series | NMEA in top-level `free` atoms | no |
| GoPro HERO (flat MP4, GPMF) | GPMF GPS5/GPS9; IMU optional | no (no GPano) |

Everything lives in `v2i.py`: camera detection, all four pipelines, GPMF/FIT/NMEA telemetry readers, frame extraction, metadata writing and track export. There are no separate per-camera or helper modules.

## Output layout

- One date folder per source video based on the **first exported frame**: `<output>/YYYY-MM-DD-<camera>/<sequence>/`
- Camera suffixes: `virb30`, `max360`, `blackvue`, `hero`
- Sequence directory name is derived from the source video stem, made collision-safe by prepending a nested path prefix when stems collide (no numeric `-2` counters)
- When `--offset`/`--heading-offset` is supplied, the recalculated `000`-`360` value is appended to the camera folder: `<output>/YYYY-MM-DD-<camera>-<offset>/<sequence>/` (e.g. `2026-10-28-virb30-090`); without it the folder is unchanged
- Filenames: `YYYY-MM-DD_HH-MM-SS-mmm.jpg` or `.png`. Timestamps cross midnight while staying in the same sequence folder
- A `-2`/`-3` suffix is used **only** when two frames selected in the same run share a millisecond. Re-running skips images that already exist; `--overwrite` rewrites them in place (no new suffixed copies)
- Each sequence writes `<sequence>.gpx` and `<sequence>.geojson` by default (`--track-export {both,gpx,geojson,none}`, `--save-telemetry` alias)
- Detailed telemetry is written to `<sequence>.telemetry.json` (every GPS/IMU sample) and `<sequence>.image-telemetry.json` (snapshot per exported image); see [Detailed telemetry](#detailed-telemetry)

## Horizon leveling

Images are **auto-leveled by default** from the camera's own orientation data; use `--no-level-horizon` to keep the source tilt. Leveling never asks for the mounting angle.

| Camera | Leveling method |
|---|---|
| GoPro MAX 360 | `GPano:PoseRollDegrees` / `GPano:PosePitchDegrees` are filled from the per-image `GRAV` sample. The equirectangular pixels are **not** resampled; compliant panorama viewers level the sphere from the pose |
| GoPro HERO | Flat frames are rotated clockwise by `roll_deg` with an expanded canvas (`rotate=...:ow=rotw(iw):oh=roth(ih)`) |
| Garmin VIRB Ultra 30 | `ACCL` is converted with the FIT message 167 calibration and orientation matrix, then rotated into the stored video's image frame using the camera's own orientation events. Flat frames are rotated like HERO |
| BlackVue DR900 | No IMU, so images stay unrotated; telemetry still exports GPS |

Details:

- Positive roll is a **clockwise / right-side-down** tilt; positive pitch means the camera is tilted **down**. MAX pose pitch is negated (`GPano` pitch is positive above the horizon).
- Roll/pitch come from `GRAV` when present (`angle_source: "GRAV"`), otherwise from VIRB `ACCL` (`angle_source: "ACCL"`). Every image entry in the telemetry JSON repeats the value.
- VIRB's accelerometer up/down sign is **derived from the camera data**: the FIT camera-orientation events (0/90/180/270) are applied to the accelerometer vector, and the sign that puts the image-frame up vector above the horizon is chosen automatically. No `--virb-accel-sign` input is required.
- Rotations below `0.05°` are skipped so level footage is not resampled.
- Rotation happens during extraction. Existing images are skipped on re-runs, so use `--overwrite` after changing leveling options to re-level them.
- Pixel rotation expands the canvas and fills the new corners with black; it does not correct lens distortion or pitch keystone.

## Detailed telemetry

Each sequence writes two JSON files next to the GPX/GeoJSON. They are independent of `--track-export` and controlled by `--telemetry-export {both,points,images,none}` (default `both`):

- `<sequence>.telemetry.json` — every GPS and IMU stream. Each stream has `components`, `unit`, `point_count` and `points`
- `<sequence>.image-telemetry.json` — one entry per exported image with the nearest `gps` and `imu` snapshots
- Both include `_meta`/header fields such as camera, video, sequence and `video_start_utc`
- `--telemetry-export points|images|none` selects which file(s) are written; telemetry is written even with `--track-export none`

| Stream | GoPro MAX/HERO | Garmin VIRB Ultra 30 |
|---|---|---|
| `ACCL` | `m/s^2` | `g` |
| `GYRO` | `rad/s` | `deg/s` |
| `MAGN` | `uT` | `G` |
| `GRAV` | `g` | — |

`GRAV` is the absolute gravity/up vector, so every GRAV point also carries `roll_deg` and `pitch_deg`, and each image entry repeats them at the image position with `angle_source: "GRAV"`. VIRB `ACCL` points carry the same angles after calibration/orientation, with `angle_source: "ACCL"`. Roll is the horizon tilt (positive = clockwise/right side down); pitch is the camera tilt (positive = down). Yaw is not part of gravity: use the image `heading_deg`/`course_deg`, or the `CORI` quaternion when a relative rotation is enough.

Every IMU point carries `utc` and video-relative `t_s` (scaled for MAX TimeWarp), `stmp_us` (GPMF) or `fit_s` (FIT), the converted components and, for VIRB, the raw ADC counts in `raw`. VIRB counts are converted with the FIT message 167 calibration: `(raw - level_shift) / divisor * factor`, with the orientation matrix applied on top.

BlackVue has no IMU, so its points file contains only `gps` and its image file has `imu: {}`. GoPro IMU UTC normally comes from the GPS clock; for an unpositioned HERO the IMU clock is anchored to the container creation time, so per-image IMU snapshots are still provided when that timestamp exists (otherwise they are `null`).

## Quick start

```bash
# Everything under video/, detected per file
python3 v2i.py -i video/ -o images/ -d 7

# VIRB (finds FIT in GMetrix/ automatically)
python3 v2i.py -i video/2026-10-05-virb-1080-24fps -o images/ -d 5 --start-distance 0

# GoPro MAX 360 (TimeWarp/variable rates handled)
python3 v2i.py -i video/2026-10-26-max360-000-timelapse05-30fps -o images/ -d 2 --start-distance 0

# BlackVue DR900
python3 v2i.py -i video/2019-07-12-blackvue-standard -o images/ -d 5 --start-distance 0

# GoPro HERO flat (no GPS → skipped unless fallback)
python3 v2i.py -i video/2025-08-16-hero-standard-nogps -o images/ -d 5 --dry-run
# HERO with no GPS, unpositioned time-lapse:
python3 v2i.py -i video/2025-08-16-hero-standard-nogps -o images/ --allow-missing-gps --fallback-interval 30

# Keep the source tilt instead of auto-leveling
python3 v2i.py -i video/ -o images/ -d 7 --no-level-horizon
```

## Detection

Cameras are identified from container handlers, GPMF `DVNM`, stream layout and filenames; ambiguous GoPro metadata is **not guessed** (e.g. generic GoPro handler with no device/filename/layout evidence). Recognised videos are grouped and processed by the matching pipeline; unrecognised or ambiguous files are skipped with a clear reason.

Every candidate file is reported during detection:

- accepted cameras are logged at `INFO`
- skipped files are logged at `WARNING` with the reason
- `[INFO] camera detection summary: BlackVue DR900=N, GoPro MAX 360=N, Garmin VIRB Ultra 30=N, GoPro HERO=N` once per run, counting **all** camera types found under `--input` (an `unsupported=N` entry is added when unknown files are present).

So a single `-i video/` run shows the camera mix of the whole input tree and processes every recognised camera in one pass.

Measured (evidence-based):
- MAX: `DVNM` includes `Max`, dual EAC video streams or `.360`, filename `GS*.360/mp4/mov`
- HERO: `DVNM` includes `HERO`, single flat video stream with GPMF, filename `GX*/GH*/GOPR*/GOPRO*`
- VIRB: Garmin handler/filename `VIRB*`
- BlackVue: PittaSoft/BlackVue handler/filename `YYYYMMDD_HHMMSS_XX`

## Frame timeline & selection

- Frame timeline comes from `ffprobe` actual PTS (`-show_packets`, presentation order) with fallback to a uniform fps grid if unreadable
- Frames are extracted by **exact frame index**, one accurate `-ss <PTS> -frames:v 1` seek per frame (`extract_frames_by_index`); `--workers` runs that many ffmpeg processes in parallel (default 1)
- One unique frame per target distance along the interpolated track; **never reused**, indices **strictly increasing**
- `--start-distance 0` **always exports the first frame (index 0)** even if telemetry starts later (position clamped to start, `position_source=clamped-start`); failing to extract frame 0 with `--start-distance 0` is a hard error
- Nearest unused frame within `--max-position-error` (default `distance/2`); targets with no acceptable frame are **skipped** (gaps allowed)
- Low frame rates / VFR produce gaps rather than duplicates
- `--max-images` caps output; `--distance-mode {2d,3d}`

## Time base & timestamps

- Default `--time-base local` with detected camera offset; `--timezone OFFSET` or `--tz` override
- **MAX/HERO** carry no local wall clock: if offset unknown, filenames/EXIF use **UTC** and a warning is logged (never host timezone)
- EXIF: `DateTimeOriginal`/`CreateDate`/`ModifyDate` are camera-local when known, else UTC; `OffsetTimeOriginal` written **only when offset known**
- All GPS date/time tags are **UTC**
- Track `*.gpx` is valid GPX 1.1 (UTC `<time>`, namespaced extensions); `*.geojson` RFC 7946 (`LineString` + image `Point` features; `Point` if single-sample track)
- Midnight crossing keeps images in the same sequence folder with date-based filenames
- VIRB capture time is `video_start_utc + pts`, including real milliseconds; the telemetry window is `video_start_fit` → `+duration`

## Heading & direction

- Heading source precedence (in `resolve_heading`): `compass_heading` (IMU/MAGN+GRAV) > `metadata_heading` > `course`/`heading` > `gps_course` (computed by `Track` from positions). `heading_source` recorded. `compass_heading` = camera/device pointing direction (tilt-compensated when available); `course`/`heading`/`gps_course` = Course Over Ground (direction of travel). Use `--heading-offset` for mounting angle, `--magnetic-declination` for compass. Per camera: HERO/MAX use GPMF GPS + optional `--imu-heading` compass from MAGN+GRAV; VIRB reads `course/heading` from FIT (usually travel direction) with `camera_orientation_deg` stored separately (not used for `img_direction`); BlackVue uses NMEA RMC/VTG course or Track-computed GPS course.
- `--heading-offset` (`--offset` alias), `--magnetic-declination` applied to true north. Heading offsets are recalculated to unsigned `000`-`360` degrees (`-90` → `270`, `450` → `090`) and, when explicitly supplied, appended to the camera folder name
- Accuracy: measured (FIT/GPMF/EXIF) takes precedence; else `accuracy_m ≈ HDOP × UERE` with `accuracy_estimated=true`, `accuracy_source=hdop*uere` (default `--uere 5.0`)
- VIRB uses FIT GPS accuracy fields (including field 8) and HDOP/VDOP; BlackVue uses GGA HDOP/sats and GSA PDOP/HDOP/VDOP; MAX/HERO use GPMF GPSP/GPSF and DOP

## Metadata policy

- No hard-coded `Keywords`/`XMP:Subject`; `--keywords "a;b;c"` is **opt-in**
- Provenance is written to `ImageDescription`/`UserComment`/`XMP:Description`, not keywords
- **GPano/XMP-GPano** written **only for panoramic (MAX) images** with the full tag set; flat HERO/VIRB/BlackVue have no GPano tags
- MAX `PoseRollDegrees`/`PosePitchDegrees` are filled when leveling is enabled; both stay `0` with `--no-level-horizon`

## GoPro specifics

### MAX 360
- Correct EAC layout: three 1344-wide faces at x=0,1376,2752 per hemisphere, drop 32px gaps, hstack faces → 4032, vstack hemispheres → 4032×2688, `v360=eac:equirect`
- Backends: `--stitch-backend {auto,ffmpeg,max2sphere}` (default `auto`, ffmpeg preferred). `max2sphere` used only if requested/available
- Options: `--width/--height` (default 5376×2688), `--interp {cubic,linear,nearest,lanczos}`, `--keep-filter`, `--imu-heading`, TimeWarp handled by scaling GPS span to video span
- GPMF GPSP raw values used for DOP
- Orientation: `GRAV` gives absolute roll/pitch; `CORI`/`IORI` are 4-component (`w,x,y,z`) rotation quaternions

### HERO (flat)
- Options: `--gpmf-stream INDEX`, `--device-model`, `--allow-missing-gps`, `--imu-heading`, `--fallback-interval`
- **No GPS telemetry → skipped unless both `--allow-missing-gps` and `--fallback-interval` are provided**; fallback exports an **unpositioned** time-lapse with no GPS tags and `position_source=unpositioned`
- Supports GPMF GPS5/GPS9/GPSF/GPSP/GPSU/STMP and IMU/MAGN streams via the built-in GPMF reader
- Flat frames are rotated from `GRAV` roll when leveling is enabled

## Common options

The unified CLI accepts: `-i/--input`, `-o/--output`, `-r/--recursive`, `--no-recursive`, `-d/--distance`, `--start-distance`, `--max-position-error`, `--max-images`, `--extensions`, `--heading-offset`/`--offset`, `--magnetic-declination`, `--uere`, `--timezone`/`--tz`, `--time-base {local,utc}`, `--format {jpg,png}`, `--quality 1-100`, `--keywords`, `--level-horizon`/`--no-level-horizon`, `--track-export {both,gpx,geojson,none}`, `--save-telemetry`, `--telemetry-export {both,points,images,none}`, `--overwrite`, `--dry-run`, `--fail-fast`, `--manifest`, `--workers`/`--jobs`, `--threads`, `--no-exif`, `-v/-q`, `--log-level`, `--version`. Track/GPS helpers from `add_gps_filter_arguments`/`add_track_export_arguments`.

Log lines are colourised on a terminal: INFO green, WARNING yellow, ERROR red (CRITICAL bold red, DEBUG grey). Colours are disabled automatically when stderr is redirected; set `NO_COLOR=1` to force them off or `FORCE_COLOR=1` to force them on.

## Tests

Run the standard-library suite from the repository root:

```bash
python3 -m unittest discover -s tests -v
```

The suite covers unified camera detection/grouping, coloured logging, filename milliseconds and collision handling, VIRB capture-time naming and FIT calibration, MAX batch command construction, seek-based frame extraction compared against a `select` decode pass, horizon leveling (CLI default, flat rotation filter, MAX pose values, VIRB orientation/sign handling) and the detailed GPS/IMU telemetry exporters (stream contents, per-image nearest-sample matching, null/no-GPS handling and `--telemetry-export`).

## Measured sample evidence (for reference)

| Sample | Notes |
|---|---|
| `video/2026-10-26-max360-000-timelapse05-30fps/GS013336.360` | GoPro Max, DVNM GoPro Max, dual EAC 4096×1344, 1208 frames / 40.34 s; GPS span 604.57 s (TimeWarp) scaled to video span; equirect 5376×2688 with full GPano tags |
| `video/2025-08-16-hero-standard-nogps/GX011785.MP4` | HERO11 Black, GPMF IMU but **no GPS5/GPSU/GPSF/GPSP** → skipped by default, fallback unpositioned works with UTC millisecond naming |
| `video/2026-10-05-virb-1080-24fps/VIRB0996.MP4` + `GMetrix/2026-10-05-07-53-07.fit` | 9108 GPS, IMU; camera events align segment, offset +02:00 |
| `video/2026-10-18-virb-090-4k-30fps/VIRB0782.MP4` / `video/2026-10-18-virb-270-4k-24fps/VIRB0008.MP4` | VIRB Ultra 30; filenames carry real capture milliseconds (e.g. `2026-10-18_16-18-37-111.jpg`) |
| BlackVue samples | NMEA in `free` atoms, GGA/RMC/VTG/GSA parsed, clock reconciliation to GPS, course from RMC/VTG |
