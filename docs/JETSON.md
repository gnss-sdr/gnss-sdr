<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2026 Phil Vu <dr.phil.vu@orboticsystems.com>
)
<!-- prettier-ignore-end -->

# Building GNSS-SDR with CUDA on NVIDIA Jetson

This guide covers building GNSS-SDR with `-DENABLE_CUDA=ON` on NVIDIA Jetson
modules (Orin AGX / Orin NX / Orin Nano, and older Xavier / TX2 / Nano boards),
what the CUDA build gives you, and how to measure it against the CPU baseline.

It was written and verified on a Jetson Orin running JetPack 6 (L4T r36,
Ubuntu 22.04, CUDA 12.x). Other JetPack releases differ only in package
versions.

## What the CUDA build enables

| Block / feature                       | Selected with                                             | Notes                                                                                                                                                          |
| ------------------------------------- | --------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **GPU acquisition grid (all PCPS blocks)** | `Acquisition_XX.use_cuda=true` (or `GNSS-SDR.use_cuda_acquisition=true` globally) | The Doppler x code-phase search grid of every `*_PCPS_Acquisition` implementation is evaluated with batched cuFFTs. Peak search and statistics are unchanged, so results are identical to the CPU path. Falls back to the CPU automatically if the device cannot be initialised. |
| `GPS_L1_CA_DLL_PLL_Tracking_GPU`      | `Tracking_1C.implementation=GPS_L1_CA_DLL_PLL_Tracking_GPU` | Legacy CUDA multi-correlator tracking block (GPS L1 C/A only, experimental).                                                                                   |

Optional per-block override for multi-GPU hosts: `Acquisition_XX.cuda_device=N`
(`GNSS-SDR.cuda_device=N` globally). Jetson has a single device, so leave it
unset.

## 1. Prerequisites on the Jetson

JetPack already ships the CUDA toolkit, `nvcc`, and cuFFT under
`/usr/local/cuda`. Make sure `nvcc` is on your `PATH`:

```
$ echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
$ echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
$ source ~/.bashrc
$ nvcc --version
```

Install the GNSS-SDR dependencies from the Ubuntu repositories (this is the
Debian/Ubuntu list from the main [README](../README.md), unchanged for arm64):

```
$ sudo apt-get install build-essential cmake git pkg-config libboost-dev \
       libboost-date-time-dev libboost-system-dev libboost-filesystem-dev \
       libboost-thread-dev libboost-chrono-dev libboost-serialization-dev \
       libabsl-dev libad9361-dev libarmadillo-dev libblas-dev \
       libgnutls-openssl-dev libgnutls28-dev libgtest-dev liblapack-dev \
       libmatio-dev libpcap-dev libprotobuf-dev libpugixml-dev libssl-dev \
       libuhd-dev gnuradio-dev gr-osmosdr protobuf-compiler python3-mako
```

Check that CMake is at least 3.17 (`cmake --version`); JetPack 6 ships 3.22.
(CMake >= 3.24 is not required: on Jetson the build detects the SoC from the
device tree and selects the right `sm_XX` automatically; see below.)

## 2. Configure and build

```
$ git clone https://github.com/gnss-sdr/gnss-sdr
$ cd gnss-sdr
$ cmake -S . -B build -DENABLE_CUDA=ON -DENABLE_UNIT_TESTING=ON -DENABLE_BENCHMARKS=ON
$ cmake --build build -j$(nproc)
$ sudo cmake --install build
```

The same steps, plus dependency installation, tests and the benchmark, are
scripted in [utils/scripts/jetson-build.sh](../utils/scripts/jetson-build.sh):

```
$ utils/scripts/jetson-build.sh --deps --tests --bench
```

The configure log should show:

```
-- NVIDIA CUDA GPU Acceleration will be enabled.
--  Target CUDA architecture(s): 87
-- The CUDA compiler identification is NVIDIA 12.x. Standard: C++17.
```

### GPU architecture selection

`CMAKE_CUDA_ARCHITECTURES` is chosen as follows (first match wins):

1. Whatever you pass on the command line, e.g. `-DCMAKE_CUDA_ARCHITECTURES=87`.
2. On Jetson, from `/proc/device-tree/compatible`:
   `tegra234` (Orin) -> 87, `tegra194` (Xavier) -> 72, `tegra186` (TX2) -> 62,
   `tegra210` (Nano/TX1) -> 53.
3. `native` (CMake >= 3.24, any host).
4. Otherwise nvcc's default.

To build a binary that runs on several Jetson generations, pass a list:
`-DCMAKE_CUDA_ARCHITECTURES="72;87"`.

### Build tips for Jetson

- Compile with all cores but watch memory on 8 GB modules: `-j4` is safer than
  `-j$(nproc)` on Orin Nano / Orin NX 8 GB when building the unit tests.
- Put the board in max-performance mode before benchmarking:
  `sudo nvpmodel -m 0 && sudo jetson_clocks`.
- `-DENABLE_UNIT_TESTING=OFF` roughly halves the build time if you only need
  the receiver.

## 3. Verify

Run the CUDA-specific unit tests (they are built into `run_tests` only when
`ENABLE_CUDA=ON`):

```
$ cd build
$ ./src/tests/run_tests --gtest_filter='CudaPcpsEngineTest.*:GpsL1CaPcpsAcquisitionCudaTest.*:GpuMulticorrelatorTest.*'
```

`CudaPcpsEngineTest.*` compares the GPU grid to a CPU reference sample by
sample (single dwell, non-coherent accumulation, bit-transition mode and 4 ms
coherent integration). `GpsL1CaPcpsAcquisitionCudaTest.*` runs the full
`GPS_L1_CA_PCPS_Acquisition` adapter on a real 4 Msps capture with
`use_cuda=true` and checks it returns the same Doppler/delay as the CPU path.

## 4. Benchmark: GPU acquisition vs CPU baseline

`benchmark_pcps_grid` (built with `-DENABLE_BENCHMARKS=ON`) times one complete
PCPS search grid — carrier wipe-off, forward FFT, code multiplication, inverse
FFT and squared magnitude for every Doppler bin — on the CPU (volk + FFTW via
`gr::fft`, the receiver's default) and on the GPU (`CudaPcpsEngine`, including
host <-> device transfers), for a sweep of FFT sizes and Doppler bin counts:

```
$ ./build/tests/benchmarks/benchmark_pcps_grid --benchmark_counters_tabular=true
```

The CPU baseline runs single-threaded, which is how `pcps_acquisition` uses it
(one acquisition worker thread per channel). Counters:

- `dwells/s`: complete search grids per second. This is the figure that
  bounds how many channels can acquire simultaneously in real time
  (`Channels.in_acquisition`).
- `grid_cells/s`: Doppler x code-phase hypotheses evaluated per second.

The `MeasureExecutionTime` case of `CudaPcpsEngineTest` prints a quick
one-line CPU/GPU comparison at the default 4 Msps / 1 ms / 40-bin geometry if
you do not want to build Google Benchmark.

### Reference results

Measured on a Jetson AGX Orin (JetPack 6, `nvpmodel -m 0`, `jetson_clocks`),
`fft_size` = samples per dwell, one channel:

<!-- JETSON_BENCHMARK_TABLE_START -->
_Results are filled in from `benchmark_pcps_grid` on the target board; see the
pull request description for the run that produced the numbers in the
release._
<!-- JETSON_BENCHMARK_TABLE_END -->

## 5. Run the receiver on the GPU

Add to any configuration file:

```
GNSS-SDR.use_cuda_acquisition=true   ; every PCPS acquisition block uses the GPU
```

or per block:

```
Acquisition_1C.use_cuda=true
Acquisition_1B.use_cuda=true
```

Look for this line in the log (`gnss-sdr --log_dir=/tmp` or stderr) to confirm
the GPU path is active:

```
PCPS acquisition grid will be computed on CUDA device Orin (FFT size 4000, up to 21 Doppler bins, ...)
```

If the device cannot be initialised the block logs a `WARNING` and continues on
the CPU, so a configuration that enables `use_cuda` stays usable on a machine
without a GPU (or built without `ENABLE_CUDA`).

A ready-made example is
[conf/Other/gnss-sdr_GPS_L1_gr_complex_gpu.conf](../conf/Other/gnss-sdr_GPS_L1_gr_complex_gpu.conf).

## 6. Notes on the implementation

- `src/algorithms/acquisition/libs/cuda_pcps_engine.{h,cu}` — the engine. One
  CUDA stream and one cuFFT batched plan per acquisition block (per channel),
  so channels acquiring concurrently overlap on the device. All Doppler bins are
  processed in a single batched forward/inverse FFT pair; three small kernels
  do the wipe-off, code multiplication and magnitude/accumulation. Non-coherent
  accumulation across dwells is kept on the device.
- `pcps_acquisition::doppler_grid()` dispatches to the engine when present and
  otherwise to the unchanged CPU loop (`doppler_grid_cpu()`); peak search,
  CFAR statistics, two-step refinement, dumping and the monitor output all run
  on the host as before.
- The engine header contains no CUDA types, so it can be included from plain
  C++ translation units. `CUDA_GPU_ACCEL=1` is defined project-wide when
  `ENABLE_CUDA=ON`.
- Fixes to the existing CUDA tracking block that were needed to build and run
  on JetPack 6 are listed in the [changelog](./CHANGELOG.md).

## Troubleshooting

| Symptom                                                                       | Cause / fix                                                                                                                                  |
| ----------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| `No CMAKE_CUDA_COMPILER could be found`                                       | `nvcc` not on `PATH`; export `/usr/local/cuda/bin` or pass `-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc`.                                 |
| `nvcc fatal : Unsupported gpu architecture 'compute_30'`                      | Stale build directory from an older GNSS-SDR; delete `build/` and reconfigure.                                                               |
| `CUDA acquisition engine could not be initialised (... cudaErrorNoDevice ...)` | The process cannot see the GPU. Check `nvidia-smi`/`tegrastats`, and that the user is in the `video` group on Jetson.                        |
| `CUFFT_ALLOC_FAILED` for large FFTs                                           | Not enough GPU memory for `bins x fft_size`. Reduce `doppler_max`/increase `doppler_step`, or lower `coherent_integration_time_ms`.          |
| `unsupported GNU version! gcc versions later than N are not supported`        | Host compiler newer than the toolkit supports; pass `-DCMAKE_CUDA_HOST_COMPILER=g++-N` (JetPack 6's gcc 11 is supported by CUDA 12).         |
