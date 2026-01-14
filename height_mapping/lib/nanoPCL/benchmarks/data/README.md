# nanoPCL Benchmark Data

This directory contains point cloud data for performance evaluation. 
To keep the repository lightweight, large binary files are not tracked by Git.

## Quick Start (Download)

Run the following script from the project root to download the sample KITTI datasets:

```bash
python3 benchmarks/scripts/prepare_kitti_data.py --download
```

## Dataset Details

| Filename | Points | Size | Usage |
|----------|--------|------|-------|
| `scan_000.bin` | ~124K | 2.0 MB | Source scan for filtering and basic ICP |
| `scan_005.bin` | ~124K | 2.0 MB | Target scan (small motion from 000) |
| `scan_010.bin` | ~121K | 1.9 MB | Target scan (large motion from 000) |
| `map_50frames.bin` | ~982K | 15.7 MB | Accumulated map for large-scale registration |

## Manual Generation

If you have the full KITTI Odometry dataset locally, you can generate these samples using:

```bash
python3 benchmarks/scripts/prepare_kitti_data.py --kitti-path /path/to/kitti/sequences
```
