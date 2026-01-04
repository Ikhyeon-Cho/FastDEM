# nanoPCL Benchmarks

This directory contains benchmarks comparing nanoPCL with PCL (Point Cloud Library).

## Requirements

These benchmarks require PCL to be installed:

```bash
sudo apt install libpcl-dev
```

## Building

Benchmarks are **disabled by default** because they require PCL as an external dependency.

To build benchmarks:

```bash
mkdir build && cd build
cmake .. -DNANOPCL_BUILD_BENCHMARKS=ON
make
```

## Running

```bash
./benchmark_normal
```

## Results

Typical results on a modern CPU (100k points, radius=0.1):

| Library  | Time (ms) | Speedup |
|----------|-----------|---------|
| nanoPCL  | ~50       | **3x**  |
| PCL      | ~150      | 1x      |

*Results may vary based on hardware and data characteristics.*
