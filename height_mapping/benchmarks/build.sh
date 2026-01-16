#!/bin/bash
# Build height_mapping benchmarks
#
# Usage:
#   ./build.sh                    # Build all
#   ./build.sh benchmark_height_update   # Build specific

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Compiler settings
CXX=${CXX:-g++}
CXXFLAGS="-O2 -std=c++17 -fopenmp"

# Include paths
EIGEN_CFLAGS=$(pkg-config --cflags eigen3 2>/dev/null || echo "-I/usr/include/eigen3")
INCLUDES=(
    "-I../lib/nanoPCL/include"
    "-I../include"
    "$EIGEN_CFLAGS"
)

# Get grid_map flags
GRID_MAP_FLAGS=$(pkg-config --cflags --libs grid_map_core 2>/dev/null || echo "-lgrid_map_core")

# Build function
build_benchmark() {
    local name=$1
    echo "Building ${name}..."
    $CXX $CXXFLAGS ${INCLUDES[@]} ${name}.cpp -o ${name} $GRID_MAP_FLAGS
    echo "Built: ${name}"
}

# Main
if [ $# -eq 0 ]; then
    # Build all benchmarks
    for cpp in *.cpp; do
        name="${cpp%.cpp}"
        build_benchmark "$name"
    done
else
    # Build specific benchmark
    build_benchmark "$1"
fi

echo ""
echo "Run with: ./benchmark_height_update [path/to/kitti.bin]"
