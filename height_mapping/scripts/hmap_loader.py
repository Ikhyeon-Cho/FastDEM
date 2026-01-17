#!/usr/bin/env python3
"""
hmap_loader.py - Python loader for .hmap binary files

Usage:
    from hmap_loader import load_hmap, to_torch

    # Load height map
    data = load_hmap("heightmap.hmap")
    print(data['elevation'].shape)  # (rows, cols)
    print(data['metadata'])  # geometry info

    # Convert to PyTorch tensors (optional)
    tensors = to_torch(data)
"""

import struct
from pathlib import Path
from typing import Dict, Optional, Tuple, Union

import numpy as np

# Header format (matches HMapHeader in C++)
# char[4] magic, uint32 version,
# float resolution, uint32 cols, uint32 rows,
# float pos_x, float pos_y,
# uint32 start_idx_x, uint32 start_idx_y,
# uint32 frame_id_len, uint32 num_layers
HEADER_FORMAT = "<4sI f II ff II II"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


def load_hmap(
    path: Union[str, Path], aligned: bool = True
) -> Dict[str, Union[np.ndarray, dict]]:
    """
    Load .hmap binary file into numpy arrays.

    Args:
        path: Path to .hmap file
        aligned: If True, apply np.roll to unroll circular buffer
                 so that the array is world-aligned (default: True)

    Returns:
        Dictionary with:
        - Layer names as keys, numpy arrays as values (float32, shape: rows x cols)
        - 'metadata' key containing geometry information

    Raises:
        FileNotFoundError: If file doesn't exist
        ValueError: If file format is invalid

    Example:
        >>> data = load_hmap("heightmap.hmap")
        >>> elevation = data['elevation']  # np.ndarray (rows, cols)
        >>> resolution = data['metadata']['resolution']
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")

    with open(path, "rb") as f:
        # Read header
        header_bytes = f.read(HEADER_SIZE)
        if len(header_bytes) < HEADER_SIZE:
            raise ValueError("File too small to contain valid header")

        (
            magic,
            version,
            resolution,
            cols,
            rows,
            pos_x,
            pos_y,
            start_idx_x,
            start_idx_y,
            frame_id_len,
            num_layers,
        ) = struct.unpack(HEADER_FORMAT, header_bytes)

        # Validate magic
        if magic != b"HMAP":
            raise ValueError(f"Invalid file format: expected 'HMAP', got {magic!r}")

        if version != 1:
            raise ValueError(f"Unsupported version: {version}")

        # Read frame_id
        frame_id = f.read(frame_id_len).decode("utf-8")

        # Prepare metadata
        metadata = {
            "version": version,
            "resolution": resolution,
            "cols": cols,
            "rows": rows,
            "position": (pos_x, pos_y),
            "start_index": (start_idx_x, start_idx_y),
            "frame_id": frame_id,
            "length": (resolution * cols, resolution * rows),
        }

        result = {"metadata": metadata}

        # Read layers
        for _ in range(num_layers):
            # Read layer name
            (name_len,) = struct.unpack("<I", f.read(4))
            layer_name = f.read(name_len).decode("utf-8")

            # Read layer data (row-major float32)
            data_size = rows * cols * 4  # float32 = 4 bytes
            data_bytes = f.read(data_size)
            if len(data_bytes) < data_size:
                raise ValueError(f"Unexpected EOF while reading layer '{layer_name}'")

            # Convert to numpy array (Eigen uses column-major, but we saved row-major)
            # Actually grid_map::Matrix is Eigen::MatrixXf which is column-major by default
            # So we need to reshape as column-major then transpose
            array = np.frombuffer(data_bytes, dtype=np.float32)
            array = array.reshape((rows, cols), order="F")  # Eigen is column-major

            # Apply circular buffer unrolling if requested
            if aligned and (start_idx_x != 0 or start_idx_y != 0):
                array = np.roll(array, -start_idx_x, axis=0)
                array = np.roll(array, -start_idx_y, axis=1)

            result[layer_name] = array.copy()

    return result


def to_torch(
    data: Dict[str, Union[np.ndarray, dict]], device: Optional[str] = None
) -> Dict[str, "torch.Tensor"]:
    """
    Convert numpy arrays to PyTorch tensors.

    Args:
        data: Dictionary from load_hmap()
        device: Target device ('cpu', 'cuda', 'cuda:0', etc.)
                If None, uses default device.

    Returns:
        Dictionary with same structure, but numpy arrays replaced
        with torch.Tensor. Metadata is preserved as-is.

    Example:
        >>> data = load_hmap("heightmap.hmap")
        >>> tensors = to_torch(data, device='cuda')
        >>> elevation = tensors['elevation']  # torch.Tensor on CUDA
    """
    try:
        import torch
    except ImportError:
        raise ImportError("PyTorch is required for to_torch(). Install with: pip install torch")

    result = {}
    for key, value in data.items():
        if isinstance(value, np.ndarray):
            tensor = torch.from_numpy(value)
            if device is not None:
                tensor = tensor.to(device)
            result[key] = tensor
        else:
            # Keep metadata as-is
            result[key] = value

    return result


def get_info(path: Union[str, Path]) -> dict:
    """
    Get metadata from .hmap file without loading layer data.

    Args:
        path: Path to .hmap file

    Returns:
        Metadata dictionary with geometry information

    Example:
        >>> info = get_info("heightmap.hmap")
        >>> print(f"Resolution: {info['resolution']}m")
        >>> print(f"Size: {info['cols']}x{info['rows']}")
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")

    with open(path, "rb") as f:
        header_bytes = f.read(HEADER_SIZE)
        if len(header_bytes) < HEADER_SIZE:
            raise ValueError("File too small to contain valid header")

        (
            magic,
            version,
            resolution,
            cols,
            rows,
            pos_x,
            pos_y,
            start_idx_x,
            start_idx_y,
            frame_id_len,
            num_layers,
        ) = struct.unpack(HEADER_FORMAT, header_bytes)

        if magic != b"HMAP":
            raise ValueError(f"Invalid file format: expected 'HMAP', got {magic!r}")

        frame_id = f.read(frame_id_len).decode("utf-8")

        # Read layer names only
        layer_names = []
        for _ in range(num_layers):
            (name_len,) = struct.unpack("<I", f.read(4))
            layer_name = f.read(name_len).decode("utf-8")
            layer_names.append(layer_name)
            # Skip layer data
            f.seek(rows * cols * 4, 1)

        return {
            "version": version,
            "resolution": resolution,
            "cols": cols,
            "rows": rows,
            "position": (pos_x, pos_y),
            "start_index": (start_idx_x, start_idx_y),
            "frame_id": frame_id,
            "length": (resolution * cols, resolution * rows),
            "layers": layer_names,
        }


# CLI interface
if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="Load and inspect .hmap files")
    parser.add_argument("file", help="Path to .hmap file")
    parser.add_argument(
        "--info", "-i", action="store_true", help="Show file info only"
    )
    parser.add_argument(
        "--layer", "-l", type=str, help="Show statistics for specific layer"
    )

    args = parser.parse_args()

    try:
        if args.info:
            info = get_info(args.file)
            print(f"File: {args.file}")
            print(f"  Version: {info['version']}")
            print(f"  Frame ID: {info['frame_id']}")
            print(f"  Resolution: {info['resolution']:.4f} m")
            print(f"  Size: {info['cols']} x {info['rows']} cells")
            print(f"  Length: {info['length'][0]:.2f} x {info['length'][1]:.2f} m")
            print(f"  Position: ({info['position'][0]:.2f}, {info['position'][1]:.2f})")
            print(f"  Start Index: ({info['start_index'][0]}, {info['start_index'][1]})")
            print(f"  Layers: {', '.join(info['layers'])}")
        else:
            data = load_hmap(args.file)
            meta = data["metadata"]
            print(f"File: {args.file}")
            print(f"  Frame ID: {meta['frame_id']}")
            print(f"  Resolution: {meta['resolution']:.4f} m")
            print(f"  Size: {meta['cols']} x {meta['rows']} cells")
            print(f"  Layers:")

            for key, value in data.items():
                if key == "metadata":
                    continue
                if args.layer and key != args.layer:
                    continue

                valid = ~np.isnan(value)
                n_valid = np.sum(valid)
                n_total = value.size

                print(f"    {key}:")
                print(f"      Valid cells: {n_valid}/{n_total} ({100*n_valid/n_total:.1f}%)")
                if n_valid > 0:
                    print(f"      Min: {np.nanmin(value):.4f}")
                    print(f"      Max: {np.nanmax(value):.4f}")
                    print(f"      Mean: {np.nanmean(value):.4f}")
                    print(f"      Std: {np.nanstd(value):.4f}")

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
