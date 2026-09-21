<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2026 Phillip Vu <36169227+phillipvu@users.noreply.github.com>
)
<!-- prettier-ignore-end -->

# Building GNSS-SDR with CUDA on NVIDIA Jetson

This guide covers building GNSS-SDR with `-DENABLE_CUDA=ON` on NVIDIA Jetson
modules (Orin AGX / Orin NX / Orin Nano, and older Xavier / TX2 / Nano boards),
what the CUDA build gives you, and how to measure it against the CPU baseline.

It was written and verified on a Jetson Orin Nano Super Developer Kit running
JetPack 7.2 (L4T r39, Ubuntu 24.04, CUDA 13.2, gcc 13). JetPack 6 (L4T r36,
Ubuntu 22.04, CUDA 12.x) differs only in package versions.

## What the CUDA build enables

| Block / feature                       | Selected with                                             | Notes                                                                                                                                                          |
| ------------------------------------- | --------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **GPU acquisition grid (all PCPS blocks)** | `Acquisition_XX.use_cuda=true` (or `GNSS-SDR.use_cuda_acquisition=true` globally) | The Doppler x code-phase search grid of every `*_PCPS_Acquisition` implementation is evaluated with batched cuFFTs. Peak search and statistics are unchanged, so results are identical to the CPU path. Falls back to the CPU automatically if the device cannot be initialized. |
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

Check that CMake is at least 3.17 (`cmake --version`); JetPack 6 ships 3.22
and JetPack 7 ships 3.28.
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
-- The CUDA compiler identification is NVIDIA 13.2.86. Standard: C++17.
```

A full build with unit tests and benchmarks takes about 45 minutes on an Orin
Nano at `-j4` and peaks around 4 GB of RAM.

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
$ ./tests/run_tests --gtest_filter='CudaPcpsEngineTest.*:GpsL1CaPcpsAcquisitionCudaTest.*:GpuMulticorrelatorTest.*'
```

Expected output (Orin Nano Super, JetPack 7.2):

```
CudaPcpsEngineTest.MatchesCpuReferenceSingleDwell
  fft_size=4000 bins=40 dwells=1 max rel. error=2.7e-07 peak@(bin 27, idx 524)
CudaPcpsEngineTest.MatchesCpuReferenceNonCoherentAccumulation
  fft_size=4000 bins=40 dwells=4 max rel. error=2.0e-07
CudaPcpsEngineTest.MatchesCpuReferenceBitTransitionMode
  fft_size=8000 bins=40 dwells=1 max rel. error=2.0e-07
CudaPcpsEngineTest.MatchesCpuReferenceLongCoherentIntegration
  fft_size=16000 bins=100 dwells=1 max rel. error=4.1e-07
GpsL1CaPcpsAcquisitionCudaTest.SameEstimateAsCpu
  CPU:  Doppler=1700 Hz, delay=523 samples
  CUDA: Doppler=1700 Hz, delay=523 samples
GpsL1CaPcpsAcquisitionCudaTest.SameEstimateAsCpuMakeTwoStep
  CPU  (two steps): Doppler=1740 Hz, delay=911 samples
  CUDA (two steps): Doppler=1740 Hz, delay=911 samples
[  PASSED  ] 11 tests.
```

The GPU grid matches the CPU grid to single-precision rounding (relative
error a few 1e-7), and the full adapter returns the same acquisition estimate
on a real capture.

`CudaPcpsEngineTest.*` compares the GPU grid to a CPU reference sample by
sample (single dwell, non-coherent accumulation, bit-transition mode and 4 ms
coherent integration). `GpsL1CaPcpsAcquisitionCudaTest.*` runs the full
`GPS_L1_CA_PCPS_Acquisition` adapter on a real 4 Msps capture with
`use_cuda=true` and checks it returns the same Doppler/delay as the CPU path,
with and without the two-step (fine Doppler) search.

## 4. Benchmark: GPU acquisition vs CPU baseline

`benchmark_pcps_grid` (built with `-DENABLE_BENCHMARKS=ON`) times one complete
PCPS search grid (carrier wipe-off, forward FFT, code multiplication, inverse
FFT and squared magnitude for every Doppler bin) on the CPU (volk + FFTW via
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

Measured on a Jetson Orin Nano Super Developer Kit (8 GB, 6-core Cortex-A78AE,
1024-core Ampere GPU), JetPack 7.2 / CUDA 13.2, default `nvpmodel` 25 W mode
with CPU/GPU frequency scaling left on (i.e. no `jetson_clocks`), board
otherwise idle. Time is wall-clock per complete search grid; the GPU column
includes the host-to-device copy of the input, the device-to-host copy of the
magnitude grid, and the scatter into the acquisition block's per-bin buffers.

| Samples per dwell (`fft_size`) | Doppler bins | CPU (µs) | GPU (µs) | Speed-up | GPU dwells/s |
| -----------------------------: | -----------: | -------: | -------: | -------: | -----------: |
|  2000 (2 Msps, 1 ms)           |           21 |      759 |      128 |     5.9× |         7814 |
|  2000                          |           41 |     1483 |      188 |     7.9× |         5314 |
|  2000                          |           81 |     2934 |      321 |     9.1× |         3113 |
|  4000 (4 Msps, 1 ms)           |           21 |     1581 |      200 |     7.9× |         5006 |
|  4000                          |           41 |     3085 |      336 |     9.2× |         2976 |
|  4000                          |           81 |     6152 |      611 |    10.1× |         1636 |
|  8000 (8 Msps, 1 ms)           |           21 |     3355 |      367 |     9.1× |         2727 |
|  8000                          |           41 |     6695 |      661 |    10.1× |         1513 |
|  8000                          |           81 |    13600 |     1180 |    11.5× |          847 |
| 16000 (4 Msps, 4 ms)           |           21 |     7699 |      756 |    10.2× |         1324 |
| 16000                          |           41 |    15091 |     1334 |    11.3× |          750 |
| 16000                          |           81 |    29888 |     2883 |    10.4× |          347 |
| 20000 (20 Msps, 1 ms)          |           21 |    11962 |      912 |    13.1× |         1097 |
| 20000                          |           41 |    23445 |     1697 |    13.8× |          589 |
| 20000                          |           81 |    46426 |     3097 |    15.0× |          323 |
| 40000 (20 Msps, 2 ms)          |           21 |    26399 |     2231 |    11.8× |          448 |
| 40000                          |           41 |    51341 |     4194 |    12.2× |          238 |
| 40000                          |           81 |   101573 |     8023 |    12.7× |          125 |

Reading the table: with the default GPS L1 configuration (4 Msps, 1 ms,
±5 kHz at 250 Hz = 41 bins) one Orin Nano CPU core sustains ~320 dwells/s,
the GPU ~3000 dwells/s. At 20 Msps the CPU falls below real time for a single
channel with 81 bins (21 dwells/s of a 1 ms signal, i.e. 2% real time) while
the GPU stays at 323 dwells/s. Note the CPU figures are for one core; with
several channels in acquisition the CPU path scales with the cores you give it
while the channels' GPU engines share one device, so the per-channel speed-up
shrinks as `Channels.in_acquisition` grows. The numbers above are single-engine
throughput; concurrent multi-channel GPU throughput has not been characterized.

Run the sweep yourself with:

```
$ ./build/tests/benchmarks/benchmark_pcps_grid --benchmark_counters_tabular=true --benchmark_min_time=0.5s
```

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

If the device cannot be initialized the block logs a `WARNING` and continues on
the CPU, so a configuration that enables `use_cuda` stays usable on a machine
without a GPU (or built without `ENABLE_CUDA`).

A ready-made example is
[conf/Other/gnss-sdr_GPS_L1_gr_complex_gpu.conf](../conf/Other/gnss-sdr_GPS_L1_gr_complex_gpu.conf).

### End-to-end check on a real capture

The public
[2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN](https://sourceforge.net/projects/gnss-sdr/files/data/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.tar.gz)
capture (GPS L1, 4 Msps `ishort`, 100 s) run through
`conf/File_input/GPS/gnss-sdr_GPS_L1_ishort.conf` with `SignalSource.samples=480000000`
(60 s), `Channels.in_acquisition=4`, and `Acquisition_1C.use_cuda` set to
`false` / `true`, on the Orin Nano Super (three runs each, `next` branch):

| Acquisition | Time to first fix (receiver time) | PVT solutions in 60 s | Processing time for 60 s of signal |
| ----------- | --------------------------------- | --------------------: | ---------------------------------: |
| CPU         | 06:23:30.5 (3/3 runs)             |                    69 |                     12.3 to 12.7 s |
| CUDA        | 06:23:30.5 (3/3 runs)             |                    69 |                     10.1 to 10.7 s |

Both give the same position (41.2748 N, 1.9877 E, ~74 m, the CTTC roof). The
processing time is dominated by tracking on the CPU, so the GPU acquisition
path mostly frees CPU time here rather than shortening the run; the benefit
grows with the number of channels in acquisition, the sampling rate, and the
Doppler range.

## 6. Notes on the implementation

- `src/algorithms/acquisition/libs/cuda_pcps_engine.{h,cu}` is the engine. One
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
| `CUDA acquisition engine could not be initialized (... cudaErrorNoDevice ...)` | The process cannot see the GPU. Check `nvidia-smi`/`tegrastats`, and that the user is in the `video` group on Jetson.                        |
| `CUFFT_ALLOC_FAILED` for large FFTs                                           | Not enough GPU memory for `bins x fft_size`. Reduce `doppler_max`/increase `doppler_step`, or lower `coherent_integration_time_ms`.          |
| `unsupported GNU version! gcc versions later than N are not supported`        | Host compiler newer than the toolkit supports; pass `-DCMAKE_CUDA_HOST_COMPILER=g++-N` (JetPack 6's gcc 11 is supported by CUDA 12).         |
