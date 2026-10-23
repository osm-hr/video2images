# Usage Reference

This document provides quick reference tables and examples for all three scripts. For detailed explanations, see [README.md](../README.md).

## Quick Start

```bash
# See help for any script
python3 virb30-v2i.py --help
python3 max360-v2i.py --help
python3 blackvue900-v21.py --help
```

## Common Options Summary

All three share: `-i/--input`, `-o/--output`, `-r/--recursive`, `--no-recursive`, `-d/--distance`, `--start-distance`, `--max-images`, `--extensions`, `--heading-offset`/`--offset`, `--timezone`/`--tz`, `--time-base`, `--format`, `--quality`, `--overwrite`, `--dry-run`, `--fail-fast`, `--manifest`, `--workers`/`--jobs`, `--threads`, `--no-exif`, `-v/-q`, `--log-level`, `--version`, plus track/GPS helpers as applicable.

## Script Comparison

| Feature | virb30-v2i.py | max360-v2i.py | blackvue900-v21.py |
|---|---|---|---|
| Camera | Garmin VIRB Ultra 30 | GoPro MAX 360 | BlackVue DR900 |
| Telemetry | FIT (sibling gmetrix/ or --fit) | GPMF embedded | NMEA in free atoms |
| 360° reprojection | No | Yes (equirectangular) | No |
| Clock skew reconciliation | N/A (FIT authoritative) | N/A | Yes (bracket vs NMEA UTC) |
| Time-grid fallback | Yes (--fallback-interval) | No (GPS required) | Yes (--fallback-interval) |
| Geometry controls | No | width/height/interp/heading-window | No |
| Save telemetry | No (uses FIT) | Yes (--save-telemetry writes GPX+GPmd) | No |

## Examples

### Process directory (dry-run first)
```bash
python3 blackvue900-v21.py -i video/2021-01-27 -o /tmp/out_bv --dry-run
```

### Small extraction with sidecars and manifest
```bash
python3 blackvue900-v21.py -i video/2021-01-27/20210127_092534_NF.mp4 -o /tmp/out_bv --max-images 3 --sidecars --manifest /tmp/out_bv/manifest.json
```

### Force UTC naming/EXIF base
```bash
python3 virb30-v2i.py -i video/ -o /tmp/out_virb --time-base utc
```

### Custom heading and quality
```bash
python3 max360-v2i.py -i video/ -o /tmp/out_max --heading-offset 10 --quality 100 --format png --width 5376 --height 2688
```

### Mixed folder (each script picks its own)
```bash
for s in virb30-v2i.py max360-v2i.py blackvue900-v21.py; do
  out="/tmp/$(basename $s .py)"
  python3 "$s" -i video/ -o "$out" --distance 10 --dry-run
done
```

## Exit Codes

- `0`: success, all processed videos produced images (or all skipped only due to cross-camera detection in a mixed tree? detection skips don't fail the run; only videos that were "accepted" but produced no images count as failed)
- `1`: some accepted videos produced no images
- `2`: no input videos found / cannot create output / configuration error
- `130`: interrupted by user

## Tips

- Use `--dry-run` before large runs.
- Keep `--workers 1` if disk I/O is slow; increase for fast storage/CPUs.
- With 2 fps/low frame rates and small distance intervals, expect some duplicate frames (same extracted frame, different counters/GPS) — this is intentional and matches VIRB behavior.
- BlackVue: if camera clock skew is reported, timestamps are already reconciled to true GPS UTC. This is correct.
