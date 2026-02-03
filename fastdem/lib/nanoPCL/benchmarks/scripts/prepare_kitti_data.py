#!/usr/bin/env python3
"""
prepare_kitti_data.py

KITTI odometry/semanticKITTI에서 벤치마크용 데이터 생성.

Output:
    data/scan_000.bin      # Source scan (filter, normal, search, registration)
    data/scan_005.bin      # Target scan (small motion)
    data/scan_010.bin      # Target scan (large motion)
    data/map_50frames.bin  # Accumulated map

Usage:
    python3 prepare_kitti_data.py --kitti-path /path/to/semanticKITTI/dataset_raw/sequences
"""

import argparse
from pathlib import Path
import numpy as np


def load_poses(pose_file: Path) -> list:
    """Load KITTI pose file -> list of 4x4 transformation matrices"""
    poses = []
    with open(pose_file, 'r') as f:
        for line in f:
            values = list(map(float, line.strip().split()))
            T = np.eye(4)
            T[:3, :] = np.array(values).reshape(3, 4)
            poses.append(T)
    return poses


def load_calib(calib_file: Path) -> np.ndarray:
    """Load Tr (velodyne to camera) from calib.txt"""
    with open(calib_file, 'r') as f:
        for line in f:
            if line.startswith("Tr:"):
                values = list(map(float, line.strip().split()[1:]))
                Tr = np.eye(4)
                Tr[:3, :] = np.array(values).reshape(3, 4)
                return Tr
    raise ValueError("Tr not found in calib file")


def load_kitti_bin(bin_file: Path) -> np.ndarray:
    """Load KITTI .bin -> (N, 4) array [x, y, z, intensity]"""
    return np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)


def save_kitti_bin(points: np.ndarray, output_path: Path):
    """Save as KITTI-format .bin"""
    points.astype(np.float32).tofile(output_path)
    size_mb = output_path.stat().st_size / 1e6
    print(f"  {output_path.name}: {len(points):,} points, {size_mb:.1f} MB")


def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Transform points using 4x4 matrix"""
    xyz = points[:, :3]
    xyz_h = np.hstack([xyz, np.ones((len(xyz), 1))])
    xyz_transformed = (T @ xyz_h.T).T[:, :3]
    return np.hstack([xyz_transformed, points[:, 3:4]])


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Simple voxel grid downsampling"""
    if len(points) == 0:
        return points

    voxel_indices = np.floor(points[:, :3] / voxel_size).astype(np.int32)
    min_idx = voxel_indices.min(axis=0)
    voxel_indices -= min_idx

    max_idx = voxel_indices.max(axis=0) + 1
    keys = (voxel_indices[:, 0] * max_idx[1] * max_idx[2] +
            voxel_indices[:, 1] * max_idx[2] +
            voxel_indices[:, 2])

    _, unique_indices = np.unique(keys, return_index=True)
    return points[unique_indices]


def create_map(seq_path: Path, poses: list, Tr: np.ndarray,
               start: int, num_frames: int, voxel_size: float) -> np.ndarray:
    """Accumulate frames into map"""
    all_points = []
    velodyne_dir = seq_path / "velodyne"

    for i in range(start, min(start + num_frames, len(poses))):
        bin_file = velodyne_dir / f"{i:06d}.bin"
        if not bin_file.exists():
            continue

        points = load_kitti_bin(bin_file)
        T_world_velo = poses[i] @ Tr
        points_world = transform_points(points, T_world_velo)
        all_points.append(points_world)

    merged = np.vstack(all_points)
    print(f"    Raw: {len(merged):,} points")

    downsampled = voxel_downsample(merged, voxel_size)
    print(f"    Downsampled ({voxel_size}m): {len(downsampled):,} points")

    return downsampled


def main():
    parser = argparse.ArgumentParser(description="Prepare KITTI benchmark data")
    parser.add_argument("--kitti-path", required=True,
                        help="Path to KITTI sequences directory")
    parser.add_argument("--output", default="data", help="Output directory")
    parser.add_argument("--sequence", default="00", help="Sequence to use")
    args = parser.parse_args()

    kitti_path = Path(args.kitti_path)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    seq_path = kitti_path / args.sequence
    pose_file = seq_path / "poses.txt"
    calib_file = seq_path / "calib.txt"
    velodyne_dir = seq_path / "velodyne"

    print(f"KITTI: {seq_path}")
    print(f"Output: {output_dir}\n")

    # Load calibration and poses
    Tr = load_calib(calib_file)
    poses = load_poses(pose_file)
    print(f"Loaded {len(poses)} poses\n")

    # 1. Single scans for registration benchmarks
    print("=" * 50)
    print("Single Scans (for registration)")
    print("=" * 50)

    frames = [0, 5, 10]
    for frame in frames:
        bin_file = velodyne_dir / f"{frame:06d}.bin"
        points = load_kitti_bin(bin_file)
        save_kitti_bin(points, output_dir / f"scan_{frame:03d}.bin")
    print()

    # 2. Accumulated map
    print("=" * 50)
    print("Map (50 frames, 0.1m voxel)")
    print("=" * 50)

    map_data = create_map(seq_path, poses, Tr, 0, 50, voxel_size=0.1)
    save_kitti_bin(map_data, output_dir / "map_50frames.bin")
    print()

    # Summary
    print("=" * 50)
    print("Summary")
    print("=" * 50)
    total_size = 0
    for f in sorted(output_dir.glob("*.bin")):
        size_mb = f.stat().st_size / 1e6
        total_size += size_mb
        print(f"  {f.name}: {size_mb:.1f} MB")
    print(f"  Total: {total_size:.1f} MB")
    print()

    # Create archive
    print("Creating archive...")
    import tarfile
    archive_path = output_dir / "benchmark_data.tar.gz"
    with tarfile.open(archive_path, "w:gz") as tar:
        for f in output_dir.glob("*.bin"):
            tar.add(f, arcname=f.name)
    archive_size = archive_path.stat().st_size / 1e6
    print(f"  {archive_path.name}: {archive_size:.1f} MB")


if __name__ == "__main__":
    main()
