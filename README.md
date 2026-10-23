# video2images: geotagged still image export from dash/camera videos

This repository contains three command-line tools that extract geotagged still images at fixed travelled-distance intervals from video recordings made by different action/dash cameras. Each tool automatically detects the camera type, processes only matching videos, and skips others with clear reasons. All three share a common CLI surface for consistency.

## Table of Contents

- [Overview](#overview)
- [Supported Cameras](#supported-cameras)
- [Installation & Dependencies](#installation--dependencies)
- [Common CLI Options](#common-cli-options)
- [Output Structure & Naming](#output-structure--naming)
- [Camera-Specific Documentation](#camera-specific-documentation)
  - [Garmin VIRB Ultra 30 (`virb30-v2i.py`)](#garmin-virb-ultra-30-virb30-v2ipy)
  - [GoPro MAX 360 (`max360-v2i.py`)](#gopro-max-360-max360-v2ipy)
  - [BlackVue DR900 (`blackvue900-v21.py`)](#blackvue-dr900-blackvue900-v21py)
- [Examples](#examples)
- [Troubleshooting](#troubleshooting)
- [Notes on Timestamps & GPS](#notes-on-timestamps--gps)

## Overview

All scripts:
- Accept `-i/--input` as either a single video file or a directory (recursive by default).
- Plan capture points every `-d/--distance` meters along the GPS/telemetry track.
- Extract frames via `ffmpeg`, write still images with `exiftool` (if available), and include GPS/EXIF/XMP metadata suitable for Mapillary and Panoramax.
- Detect the camera from container metadata (and filename as supporting evidence); **never process a video belonging to another camera** — mismatches are logged and skipped.
- Share the same logging, sequence naming, manifest/sidecar support, and common flags.

## Supported Cameras

| Script | Camera | File patterns (typical) | Telemetry source | Notes |
|---|---|---|---|---|
| `virb30-v2i.py` | Garmin VIRB Ultra 30 | `VIRB####.MP4`/`.MOV` | FIT files (sibling `gmetrix/` or `--fit`) | Supports `.fit` lookup by default; can also use `--fallback-interval` when telemetry missing. |
| `max360-v2i.py` | GoPro MAX 360 | `GS####.360`, `.MP4` variants | GPMF (embedded in `.360`/MP4) | Reprojects 360° to equirectangular stills; offers geometry/orientation controls; writes GPX/GPmd sidecars via `--save-telemetry`. |
| `blackvue900-v21.py` | BlackVue DR900 (dashcam) | `YYYYMMDD_HHMMSS_NN.mp4` (e.g. `20210127_092534_NF.mp4`) | NMEA sentences embedded in `free` atoms | Reconciles camera-clock skew against GPS UTC; supports `--fallback-interval` when no GPS available. |

## Installation & Dependencies

Required external tools (must be on `PATH`):
- `ffmpeg` and `ffprobe` — frame extraction and probing.
- `exiftool` — write EXIF/XMP/GPS metadata (if missing, images are written without metadata and a warning is logged).

Python dependencies: only the Python 3.11+ standard library. No third-party packages required.

Get the code:
```bash
cd /path/to/video2images
```

All scripts are executable (`chmod +x *.py` if needed). They import `v2i_common.py` from the same directory.

## Common CLI Options

Every script accepts the following shared options (identical names, defaults, and help text semantics):

| Option | Default | Description |
|---|---|---|
| `-i, --input PATH` | (required) | Input video file or directory to scan (read-only). |
| `-o, --output PATH` | (required) | Output directory; created automatically if missing. |
| `-r, --recursive` | `true` | Search input directories recursively. Use `--no-recursive` to disable. |
| `-d, --distance METERS` | `5.0` | Distance between exported frames in meters (along travelled track). |
| `--start-distance METERS` | `0.0` | Skip this many meters after track start before taking the first frame. |
| `--max-images N` | `None` | Maximum number of images to export per video. |
| `--extensions LIST` | `.mp4,.mov,.m4v,.360` | Comma-separated video extensions to scan. |
| `--heading-offset DEGREES` | `0.0` | Degrees added to GPS course to derive image heading (-180..180). Aliases: `--offset` (VIRB legacy). |
| `--timezone OFFSET` | `None` (auto-detect) | Override camera UTC offset, e.g. `+02:00`, `+2`, `-5`, or `2.5`. Aliases: `--tz` (MAX/VIRB legacy). |
| `--time-base {local,utc}` | `local` | Wall clock used for filenames/folders and for naming. EXIF DateTimeOriginal remains local with OffsetTimeOriginal = UTC offset; UTC time is also recorded in sidecars/manifests. |
| `--format {jpg,png}` | `jpg` | Output image format. |
| `--quality 1-100` | `95` | Image quality (100 = best). |
| `--overwrite` | `false` | Overwrite existing images instead of skipping them. |
| `--dry-run` | `false` | Report what would be exported without writing anything. |
| `--fail-fast` | `false` | Stop on the first file that errors. |
| `--manifest FILE` | `None` | Write a JSON manifest listing all exported points. |
| `--workers N` | `1` | Number of frames to extract in parallel. Alias: `--jobs` (MAX legacy). |
| `--threads N` | `0` | ffmpeg threads per extraction job (0 = auto). |
| `--no-exif` | `false` | Do not write EXIF/XMP metadata. |
| `-v, --verbose` / `-q, --quiet` | — | Increase/decrease logging verbosity. |
| `--log-level {DEBUG,INFO,WARNING,ERROR,CRITICAL}` | `INFO` | Explicit logging level. |
| `--version` | — | Show program version and exit. |

Legacy aliases (supported where unambiguous): `--offset`→`--heading-offset`, `--tz`→`--timezone`, `--jobs`→`--workers`, `--video-glob`→`--extensions` (VIRB).

Shared GPS/track behavior (where applicable): `--distance-mode {2d,3d}`, `--sidecars`, `--stationary-speed MPS`, `--smooth N`, `--jitter METERS`, `--fallback-interval SECONDS`.

## Output Structure & Naming

Images are written as:
```
<output>/<YYYY-MM-DD>/<sequence>/<YYYY-MM-DD_HH-MM-SS-mmm_NNNNNN.ext>
```
- `<sequence>` is derived from the source video name (e.g. `VIRB0996`, `20210127_092534_NF`, `GS013287`).
- Timestamp fields use zero-padded components with millisecond precision (`mmm`).
- Counter `NNNNNN` is 6-digit, per sequence (000001, 000002, ...).
- `<YYYY-MM-DD>` is the capture date of the image (by `--time-base` choice: local or UTC date).
- Existing files are never overwritten unless `--overwrite` is given.

When `--sidecars` is enabled, each sequence also gets `<sequence>.track.jsonl` next to the images (one JSON line per planned point, including written status and GPS/timestamps).

When `--manifest` is given, a top-level JSON manifest is written summarizing all exported points across videos.

EXIF metadata (if written):
- `Make`, `Model`, `SerialNumber`/`CameraSerialNumber`, `Software` set from camera/container where available.
- `DateTimeOriginal`, `CreateDate`, `ModifyDate`, `SubSecTimeOriginal` = local capture time (if `--time-base local`) or UTC (if `--time-base utc`), with millisecond precision.
- `OffsetTimeOriginal`, `OffsetTime` = UTC offset in seconds (`+HH:MM`).
- GPS fields: `GPSLatitude`, `GPSLongitude`, `GPSAltitude`, `GPSSpeed`, `GPSTrack`, `GPSImgDirection` (with refs). Speeds are written in km/h for EXIF speed tags where applicable.
- Additional XMP/subject fields include source video, frame/pts, distance, heading offset, etc.

## Camera-Specific Documentation

### Garmin VIRB Ultra 30 (`virb30-v2i.py`)

**Purpose:** Export geotagged stills from VIRB Ultra 30 MP4/MOV videos using associated FIT telemetry.

**Telemetry discovery:**
- Looks for FIT files next to the video, or in a sibling directory named `gmetrix/` (default) or as specified by `--fit-directory-name`.
- `--fit FILE` can be given repeatedly to supply specific FIT files (applies to all videos in that run).
- FIT provides GPS positions, timestamps, and device info.

**Script-specific options:**
| Option | Default | Description |
|---|---|---|
| `--fit FILE` | `[]` | Path to FIT file(s) (repeatable). One value applies to all videos. |
| `--fit-directory-name NAME` | `gmetrix` | Sibling folder holding FIT telemetry files. |
| `--distance-mode {2d,3d}` | `2d` | Distance travelled on ground (2D) or in 3D. |
| `--sidecars` | `false` | Also write `<sequence>.track.jsonl` manifest per sequence. |
| `--fallback-interval SECONDS` | `0.0` | If a video has no usable telemetry/FIT data, sample every N seconds instead of skipping (0 = skip). |

**Behavior notes:**
- VIRB timestamps come from FIT (GPS time base); local time is derived from FIT timestamps and the configured `--timezone` (or detected).
- If no telemetry is found and fallback is enabled, images are time-gridded with no position metadata (Make/Model still written unless `--no-exif`).
- Skips GoPro MAX `.360` and BlackVue `.mp4` files with reasons.

**Example:**
```bash
python3 virb30-v2i.py -i video/2026-10-05 -o images --distance 5 --recursive
```

### GoPro MAX 360 (`max360-v2i.py`)

**Purpose:** Extract geotagged equirectangular still images from GoPro MAX `.360` (and MP4) videos at fixed GPS distance intervals. Only GoPro MAX files are processed.

**Telemetry:** GPMF embedded in the video container.

**Script-specific options:**
| Option | Default | Description |
|---|---|---|
| `--width WIDTH` | `5376` | Output image width (equirectangular). |
| `--height HEIGHT` | `2688` | Output image height. |
| `--interp {cubic,linear,nearest,lanczos}` | `cubic` | v360 reprojection interpolation. |
| `--heading-window HEADING_WINDOW` | `4` | GPS samples either side used for heading smoothing. |
| `--stationary-speed MPS` | `0.3` | Speed below which treated as parked (no distance accumulated). |
| `--smooth N` | `9` | Moving-average window (samples) applied to GPS positions. |
| `--jitter METERS` | `0.5` | GPS steps shorter than this treated as noise when no speed data. |
| `--save-telemetry` | `false` | Also write raw GPMF (`.gpmd`) and GPX (`.gpx`) track next to images. |
| `--keep-filter` | `false` | Keep generated ffmpeg filter script. |

**Behavior notes:**
- Reprojects dual-lens 360° to equirectangular stills using the v360 pipeline.
- Timezone defaults to UTC (0h) unless `--timezone` is specified; `--time-base utc` forces UTC for filenames.
- Positional `INPUT` is accepted as a compatibility alias (e.g. `max360-v2i.py video/ -o images`).
- Skips VIRB and BlackVue files with reasons.

**Example:**
```bash
python3 max360-v2i.py -i video/2026-10-11 -o images --distance 5 --format jpg --quality 95
```

### BlackVue DR900 (`blackvue900-v21.py`)

**Purpose:** Export geotagged stills from BlackVue DR900 dashcam MP4 videos using NMEA GPS sentences embedded in `free` atoms.

**Telemetry:** NMEA sentences (GGA/RMC/VTG) prefixed with camera-local wall-clock epoch-millisecond stamps in `free` atoms; typically present near start and duplicated in trailing concatenated segment.

**Clock reconciliation:** The camera’s wall clock (bracket stamps, matching filename start) can run ahead of GPS UTC. The tool:
1. Computes video-relative time `t` from bracket stamps minus recording start (from filename `YYYYMMDD_HHMMSS`).
2. Derives absolute UTC from NMEA date/time fields.
3. Computes `tz_offset = snapped-quarter-hour(median(bracket_local − nmea_utc))` (or uses `--timezone` override).
4. Sets `video_start_utc = nmea_utc(first) − t(first)` and `video_start_local = video_start_utc + tz_offset`. This ensures `DateTimeOriginal + OffsetTimeOriginal = true GPS UTC`; if skew >= 1s, it logs “camera clock is ±X s off GPS; times follow GPS”.

**Script-specific options:**
| Option | Default | Description |
|---|---|---|
| `--distance-mode {2d,3d}` | `2d` | Distance travelled on ground (2D) or in 3D. |
| `--sidecars` | `false` | Also write `<sequence>.track.jsonl` manifest per sequence. |
| `--fallback-interval SECONDS` | `0.0` | If a video has no GPS data, sample every N seconds instead of skipping (0 = skip). When GPS exists but yields no distance-based capture points (e.g. stationary track), fallback also applies. |

**Behavior notes:**
- Reads `cprt` from container to extract make/model/firmware/serial (recorded as `BlackVue DR900M-1CH FW ...`).
- Deduces and logs camera-local offset and video start (local). Filenames/folders follow `--time-base`.
- If no GPS samples and `--fallback-interval` > 0 and video duration/start known, produces time-gridded images with no position metadata; otherwise skips with reason “no GPS data and no fallback requested” (or “no distance-based capture points”).
- Skips VIRB and GoPro files with reasons.

**Example:**
```bash
python3 blackvue900-v21.py -i video/2021-01-27 -o images --distance 5 --dry-run
python3 blackvue900-v21.py -i video/2021-01-27/20210127_092534_NF.mp4 -o images --max-images 3 --sidecars --manifest images/manifest.json --time-base utc
```

## Examples

### Process a whole mixed tree safely (each script picks its own)
```bash
# VIRB only
python3 virb30-v2i.py -i video/ -o out/virb --distance 5

# GoPro MAX only
python3 max360-v2i.py -i video/ -o out/max --distance 5 --format jpg

# BlackVue only
python3 blackvue900-v21.py -i video/ -o out/bv --distance 5
```

### Dry-run to see what would be planned
```bash
python3 virb30-v2i.py -i video/ -o out --dry-run
```

### Single file with limits
```bash
python3 blackvue900-v21.py -i video/2021-01-27/20210127_092534_NF.mp4 -o out --max-images 10 --start-distance 20
```

### Time-base UTC and custom quality
```bash
python3 max360-v2i.py -i video/ -o out --time-base utc --quality 100 --format png
```

### Provide FIT files explicitly (VIRB)
```bash
python3 virb30-v2i.py -i video/ -o out --fit /path/to/track.fit --fit gmetrix/another.fit
```

### Save telemetry/GPX from MAX
```bash
python3 max360-v2i.py -i video/ -o out --save-telemetry
```

### Fallback when no telemetry/GPS (VIRB/BlackVue)
```bash
python3 virb30-v2i.py -i video/ -o out --fallback-interval 10
python3 blackvue900-v21.py -i video/ -o out --fallback-interval 5
```

## Troubleshooting

- **Missing tools:** Ensure `ffmpeg`, `ffprobe`, `exiftool` are on `PATH`. Scripts check for `ffmpeg`/`ffprobe` and warn if `exiftool` missing.
- **No videos found:** Check `--extensions` matches your files and `--input` path. Use `--recursive` (default) for directories.
- **Skipped files:** Look for WARNING lines like “skipping …: recorded by …, not …” or “unsupported camera/ambiguous/missing camera metadata/invalid or unreadable video/missing GPS data”. This is expected when a folder contains mixed cameras.
- **BlackVue timing looks off:** The tool reports camera-clock skew (e.g. “camera clock is -61 s off GPS; times follow GPS”). Filenames use the reconciled local start (UTC offset applied to true GPS UTC) so EXIF DateTimeOriginal+Offset matches true GPS UTC.
- **Low FPS + small distance:** Multiple capture points may map to the same frame (frame quantization). This is consistent across scripts and produces distinct image files (different counters) with potentially different interpolated GPS at each planned point.
- **MAX 360 reprojection:** Large `--width/--height` increases processing time; defaults are reasonable for Mapillary/Panoramax.
- **Dry-run first:** Use `--dry-run` to verify counts before a real run.

## Notes on Timestamps & GPS

- **VIRB:** Uses FIT telemetry timestamps as ground truth; `video_start_local` derives from FIT + `--timezone`. Skew detection not applicable (FIT is authoritative).
- **MAX:** Uses GPMF GPS time (UTC) where available; timezone handling follows `--timezone`/`--time-base`.
- **BlackVue:** Reconciles bracket (camera local wall clock) vs NMEA (GPS UTC) — this fixes typical camera-clock skew. The logged “video start (local)” is the reconciled local time used for naming/EXIF.
- **Date folders:** Always the capture date per `--time-base`. If a recording spans midnight, each image is filed under its own capture date (warnings logged).
- **Manifest/sidecars:** Contain both `datetime_local` and `datetime_utc` (naive UTC instants in the JSON) plus `utc_offset_s`.