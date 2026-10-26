# video2images

Extract geotagged still images from dash/helmet cameras at fixed GPS distance intervals. Only the Python standard library is used; `ffmpeg`, `ffprobe`, `exiftool` must be on `PATH`. The optional `max2sphere` binary is only used if explicitly requested.

## Supported cameras

| Script | Camera | Telemetry | 360 stitching |
|---|---|---|---|
| `virb30-v2i.py` | Garmin VIRB Ultra 30 | FIT (sibling `*.fit` in `GMetrix/` or matched by filename/time) | no |
| `max360-v2i.py` | GoPro MAX 360 (`.360`/dual-EAC) | GPMF (embedded `gpmd`) | yes (EAC → equirectangular via `ffmpeg v360`, default 5376×2688) |
| `blackvue900-v2i.py` (legacy `blackvue900-v21.py` deprecated shim) | BlackVue DR900 series | NMEA in top-level `free` atoms | no |
| `hero-v2i.py` | GoPro HERO (flat MP4, GPMF) | GPMF GPS5/GPS9; IMU optional | no (no GPano) |

All scripts share CLI, output layout, naming, time base, metadata policy, telemetry export and selection rules via `v2i_common.py`, `v2i_gpmf.py`, `v2i_metadata.py`.

## Output layout

- One date folder per source video based on the **first exported frame**: `<output>/YYYY-MM-DD-<camera>/<sequence>/`
- Camera suffixes: `virb30`, `max360`, `blackvue`, `hero`
- Sequence directory name is derived from the source video stem, made collision-safe by prepending a nested path prefix when stems collide (no numeric `-2` counters)
- Filenames: `YYYY-MM-DD_HH-MM-SS-mmm.jpg` or `.png` — **no `_NNNNNN` counter**. Timestamps cross midnight while staying in the same sequence folder
- Each sequence writes `<sequence>.gpx` and `<sequence>.geojson` by default (`--track-export {both,gpx,geojson,none}`, `--save-telemetry` alias)

## Quick start

```bash
# VIRB (finds FIT in GMetrix/ automatically)
python3 virb30-v2i.py -i video/2026-10-05-virb -o images/ -d 5 --start-distance 0

# GoPro MAX 360 (TimeWarp/variable rates handled)
python3 max360-v2i.py -i video/2026-10-17-max360 -o images/ -d 2 --start-distance 0

# BlackVue DR900
python3 blackvue900-v2i.py -i video/2019-07-12-blackvue -o images/ -d 5 --start-distance 0

# GoPro HERO flat (no GPS → skipped unless fallback)
python3 hero-v2i.py -i video/2026-07-14-hero -o images/ -d 5 --dry-run
# HERO with no GPS, unpositioned time-lapse:
python3 hero-v2i.py -i video/2026-07-14-hero -o images/ --allow-missing-gps --fallback-interval 30
```

## Detection

Cameras are identified from container handlers, GPMF `DVNM`, stream layout and filenames; ambiguous GoPro metadata is **not guessed** (e.g. generic GoPro handler with no device/filename/layout evidence). Wrong-camera files are skipped with a clear reason.

Measured (evidence-based):
- MAX: `DVNM` includes `Max`, dual EAC video streams or `.360`, filename `GS*.360/mp4/mov`
- HERO: `DVNM` includes `HERO`, single flat video stream with GPMF, filename `GX*/GH*/GOPR*/GOPRO*`
- VIRB: Garmin handler/filename `VIRB*`
- BlackVue: PittaSoft/BlackVue handler/filename `YYYYMMDD_HHMMSS_XX`

## Frame timeline & selection

- Frame timeline comes from `ffprobe` actual PTS (`-show_packets`, presentation order) with fallback to a uniform fps grid if unreadable
- Frames are extracted by **exact frame index** in one decode pass via `select='eq(n,...)'` with `-vsync 0` (`v2i_common.extract_frames_by_index`)
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

## Headings, accuracy, DOP

- Heading source precedence (in `v2i_common.resolve_heading`): `compass_heading` (IMU/MAGN+GRAV) > `metadata_heading` > `course`/`heading` > `gps_course` (computed by `Track` from positions). `heading_source` recorded. `compass_heading` = camera/device pointing direction (tilt-compensated when available); `course`/`heading`/`gps_course` = Course Over Ground (direction of travel). Use `--heading-offset` for mounting angle, `--magnetic-declination` for compass. Per camera: HERO/MAX use GPMF GPS + optional `--imu-heading` compass from MAGN+GRAV; VIRB reads `course/heading` from FIT (usually travel direction) with `camera_orientation_deg` stored separately (not used for `img_direction`); BlackVue uses NMEA RMC/VTG course or Track-computed GPS course.
- `--heading-offset` (`--offset` alias), `--magnetic-declination` applied to true north
- Accuracy: measured (FIT/GPMF/EXIF) takes precedence; else `accuracy_m ≈ HDOP × UERE` with `accuracy_estimated=true`, `accuracy_source=hdop*uere` (default `--uere 5.0`)
- VIRB uses FIT GPS accuracy fields (including field 8) and HDOP/VDOP; BlackVue uses GGA HDOP/sats and GSA PDOP/HDOP/VDOP; MAX/HERO use GPMF GPSP/GPSF and DOP

## Metadata policy

- No hard-coded `Keywords`/`XMP:Subject`; `--keywords "a;b;c"` is **opt-in**
- Provenance is written to `ImageDescription`/`UserComment`/`XMP:Description`, not keywords
- **GPano/XMP-GPano** written **only for panoramic (MAX) images** with the full tag set; flat HERO/VIRB/BlackVue have no GPano tags

## GoPro specifics

### MAX 360
- Correct EAC layout: three 1344-wide faces at x=0,1376,2752 per hemisphere, drop 32px gaps, hstack faces → 4032, vstack hemispheres → 4032×2688, `v360=eac:equirect`
- Backends: `--stitch-backend {auto,ffmpeg,max2sphere}` (default `auto`, ffmpeg preferred). `max2sphere` used only if requested/available
- Options: `--width/--height` (default 5376×2688), `--interp {cubic,linear,nearest,lanczos}`, `--keep-filter`, `--imu-heading`, TimeWarp handled by scaling GPS span to video span
- GPMF GPSP raw values used for DOP

### HERO (flat)
- Options: `--gpmf-stream INDEX`, `--device-model`, `--allow-missing-gps`, `--imu-heading`, `--fallback-interval`
- **No GPS telemetry → skipped unless both `--allow-missing-gps` and `--fallback-interval` are provided**; fallback exports an **unpositioned** time-lapse with no GPS tags and `position_source=unpositioned`
- Supports GPMF GPS5/GPS9/GPSF/GPSP/GPSU/STMP and IMU/MAGN streams via `v2i_gpmf`

## Common options

All exporters accept: `-i/--input`, `-o/--output`, `-r/--recursive`, `--no-recursive`, `-d/--distance`, `--start-distance`, `--max-position-error`, `--max-images`, `--extensions`, `--heading-offset`/`--offset`, `--magnetic-declination`, `--uere`, `--timezone`/`--tz`, `--time-base {local,utc}`, `--format {jpg,png}`, `--quality 1-100`, `--keywords`, `--track-export {both,gpx,geojson,none}`, `--save-telemetry`, `--overwrite`, `--dry-run`, `--fail-fast`, `--manifest`, `--workers`/`--jobs`, `--threads`, `--no-exif`, `-v/-q`, `--log-level`, `--version`. Track/GPS helpers from `add_gps_filter_arguments`/`add_track_export_arguments`.

Log lines are colourised on a terminal: INFO green, WARNING yellow, ERROR red (CRITICAL bold red, DEBUG grey). Colours are disabled automatically when stderr is redirected; set `NO_COLOR=1` to force them off or `FORCE_COLOR=1` to force them on.

## Measured sample evidence (for reference)

| Sample | Notes |
|---|---|
| `video/2026-10-17-max360/GS013328.360` | GoPro Max, DVNM GoPro Max, dual EAC, GPMF GPS5 194 samples over ~6.44s video but ~32s GPS span (TimeWarp) → scaled; GPSP ~117/120 |
| `video/2026-07-14-hero/GX011785.MP4` | HERO11 Black, GPMF IMU but **no GPS5/GPSU/GPSF/GPSP** → skipped by default, fallback unpositioned works with UTC naming |
| `video/2026-10-05-virb/VIRB0996.MP4` + `GMetrix/2026-10-05-07-53-07.fit` | 9108 GPS, IMU; camera events align segment, offset +02:00 |
| BlackVue samples | NMEA in `free` atoms, GGA/RMC/VTG/GSA parsed, clock reconciliation to GPS, course from RMC/VTG |

## Tests

```bash
python3 -m unittest discover -s tests -v
```

Covers detection, paths/naming/collisions, frame timeline (mock ffprobe packets/2fps/VFR/fallback), selection/low-fps/first-frame/start clamping, headings/declination, accuracy cases, GPX/GeoJSON parsing/validity, metadata keywords/GPano/UTC/offset rules, MAX EAC filter/max2sphere command construction, GPMF synthetic payloads, BlackVue NMEA/DOP.

## Migration

- `blackvue900-v21.py` is now a deprecated shim forwarding to `blackvue900-v2i.py` with a warning
- `USAGE.md` merged into this README
