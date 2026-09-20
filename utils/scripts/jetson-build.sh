#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Phillip Vu <36169227+phillipvu@users.noreply.github.com>
#
# Configure, build, and (optionally) test/benchmark GNSS-SDR with CUDA on an
# NVIDIA Jetson. Companion to docs/JETSON.md. Run from the source tree root:
#
#   utils/scripts/jetson-build.sh [--deps] [--tests] [--bench] [--install] [-j N]
#
#   --deps     apt-get install the Ubuntu build dependencies (needs sudo)
#   --tests    build and run the CUDA unit tests (run_tests, gtest filter)
#   --bench    build and run benchmark_pcps_grid (CPU baseline vs GPU)
#   --install  cmake --install (needs sudo)
#   -j N       parallel jobs (default: nproc, capped at 6 on <=8 GB boards)

set -euo pipefail

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$SRC_DIR/build}"
DO_DEPS=0
DO_TESTS=0
DO_BENCH=0
DO_INSTALL=0
JOBS=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --deps) DO_DEPS=1 ;;
        --tests) DO_TESTS=1 ;;
        --bench) DO_BENCH=1 ;;
        --install) DO_INSTALL=1 ;;
        -j) JOBS="$2"; shift ;;
        -j*) JOBS="${1#-j}" ;;
        -h|--help) sed -n '5,15p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

export PATH="/usr/local/cuda/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"

if [[ -z "$JOBS" ]]; then
    JOBS="$(nproc)"
    MEM_GB=$(awk '/MemTotal/ {printf "%d", $2/1024/1024}' /proc/meminfo)
    if [[ "$MEM_GB" -le 8 && "$JOBS" -gt 6 ]]; then JOBS=6; fi
fi

echo "== GNSS-SDR CUDA build on $(tr -d '\0' < /proc/device-tree/model 2>/dev/null || hostname)"
echo "   source: $SRC_DIR"
echo "   build:  $BUILD_DIR"
echo "   jobs:   $JOBS"
command -v nvcc >/dev/null || { echo "nvcc not found: is the CUDA toolkit installed under /usr/local/cuda?" >&2; exit 1; }
nvcc --version | tail -n 2

if [[ $DO_DEPS -eq 1 ]]; then
    echo "== Installing build dependencies"
    sudo apt-get update
    sudo apt-get install -y build-essential cmake git pkg-config libboost-dev \
        libboost-date-time-dev libboost-system-dev libboost-filesystem-dev \
        libboost-thread-dev libboost-chrono-dev libboost-serialization-dev \
        libabsl-dev libad9361-dev libarmadillo-dev libblas-dev \
        libgnutls-openssl-dev libgnutls28-dev libgtest-dev liblapack-dev \
        libmatio-dev libpcap-dev libprotobuf-dev libpugixml-dev libssl-dev \
        libuhd-dev gnuradio-dev gr-osmosdr protobuf-compiler python3-mako
fi

CMAKE_ARGS=(-DENABLE_CUDA=ON)
if [[ $DO_TESTS -eq 1 ]]; then CMAKE_ARGS+=(-DENABLE_UNIT_TESTING=ON); fi
if [[ $DO_BENCH -eq 1 ]]; then CMAKE_ARGS+=(-DENABLE_BENCHMARKS=ON); fi

echo "== Configuring: cmake ${CMAKE_ARGS[*]}"
cmake -S "$SRC_DIR" -B "$BUILD_DIR" "${CMAKE_ARGS[@]}"

echo "== Building"
cmake --build "$BUILD_DIR" -j"$JOBS"

if [[ $DO_TESTS -eq 1 ]]; then
    echo "== Running CUDA unit tests"
    "$BUILD_DIR/src/tests/run_tests" \
        --gtest_filter='CudaPcpsEngineTest.*:GpsL1CaPcpsAcquisitionCudaTest.*:GpuMulticorrelatorTest.*'
fi

if [[ $DO_BENCH -eq 1 ]]; then
    echo "== Running benchmark_pcps_grid (CPU baseline vs GPU)"
    if command -v nvpmodel >/dev/null; then
        echo "   (for repeatable numbers: sudo nvpmodel -m 0 && sudo jetson_clocks)"
    fi
    "$BUILD_DIR/tests/benchmarks/benchmark_pcps_grid" --benchmark_counters_tabular=true
fi

if [[ $DO_INSTALL -eq 1 ]]; then
    echo "== Installing"
    sudo cmake --install "$BUILD_DIR"
fi

echo "== Done"
