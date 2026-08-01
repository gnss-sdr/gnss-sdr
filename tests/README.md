<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2020 Carles Fernandez-Prades <carles.fernandez@cttc.es>
)
<!-- prettier-ignore-end -->

# GNSS-SDR Tests

This directory contains the test suite for GNSS-SDR, built on [Google Test](https://github.com/google/googletest) (gtest) for unit and system tests, and [Google Benchmark](https://github.com/google/benchmark) for performance benchmarks.

## Directory Layout

```
tests/
├── CMakeLists.txt          # Test build definitions and CTest registration
├── test_main.cc            # Main test runner (all tests compiled into a single binary)
├── single_test_main.cc     # Lightweight runner for individual test executables
├── unit-tests/             # Unit tests organized by subsystem
│   ├── arithmetic/         # Math utilities, FFT, code generation
│   ├── control-plane/      # Configuration, factory, flowgraph
│   ├── signal-processing-blocks/  # Acquisition, tracking, telemetry, PVT, filters
│   └── system-parameters/  # Ephemeris, navigation message decoders
├── system-tests/           # End-to-end integration tests
│   ├── position_test.cc    # Position validation against reference data
│   └── ttff.cc             # Time-to-first-fix test
├── benchmarks/             # Performance benchmarks
│   ├── CMakeLists.txt
│   └── benchmark_*.cc      # Individual benchmark executables
├── common-files/           # Shared test utilities
│   ├── test_flags.h        # CLI flag definitions (gnuplot, plot options)
│   ├── tracking_tests_flags.h
│   ├── signal_generator_flags.h
│   ├── observable_tests_flags.h
│   └── gnuplot_i.h         # Gnuplot interface helper
└── data/                   # Reference data for tests
    ├── config_file_sample.txt
    └── rtklib_test/        # RTKLIB XML test data (obs, ephemeris)
```

## Test Types

### Unit Tests (`unit-tests/`)

Unit tests verify individual components in isolation. They use the standard Google Test `TEST()` and `TEST_F()` macros.

**Categories:**

| Subdirectory | What it tests | Example |
|---|---|---|
| `arithmetic/` | Signal processing primitives | FFT, code generation, correlation, resamplers |
| `control-plane/` | Receiver infrastructure | `InMemoryConfiguration`, `FileConfiguration`, `GNSSFlowgraph`, `GNSSBlockFactory` |
| `signal-processing-blocks/` | Individual signal processing blocks | Acquisition algorithms, tracking loops, telemetry decoders, PVT solvers, filters |
| `system-parameters/` | GNSS data structures | Ephemeris decoders, Reed-Solomon, navigation message parsing |

Test files follow the naming convention `*_test.cc`. Each `.cc` file is `#include`d directly into `test_main.cc` rather than compiled separately.

**Example test** (`unit-tests/control-plane/in_memory_configuration_test.cc`):

```cpp
#include "in_memory_configuration.h"

TEST(InMemoryConfiguration, StoreAndRetrieve)
{
    auto configuration = std::make_unique<InMemoryConfiguration>();
    configuration->set_property("Foo.property1", "value");
    std::string default_value = "default_value";
    std::string value = configuration->property("Foo.property1", std::move(default_value));
    EXPECT_STREQ("value", value.c_str());
}
```

### System Tests (`system-tests/`)

System tests validate the full receiver chain against recorded signal files. They require pre-recorded signal samples, reference orbits, and known position solutions.

- **`position_test.cc`**: Validates computed PVT output against reference positions from SPIRENT or similar simulators
- **`ttff.cc`**: Measures time-to-first-fix

System tests are enabled with `-DENABLE_SYSTEM_TESTING=ON`.

### Benchmarks (`benchmarks/`)

Performance benchmarks measure execution time of critical code paths. Built with [Google Benchmark](https://github.com/google/benchmark). See `benchmarks/README.md` for details.

Enabled with `-DENABLE_BENCHMARKS=ON`.

## Framework: Google Test

Google Test (gtest) is the sole unit testing framework. If not found on the system, CMake downloads and builds it automatically at configure time.

Gtest features used in this project:

- **`TEST()`**: Basic test macro
- **`TEST_F()`**: Test fixture (shared setup/teardown via `::testing::Test`)
- **`EXPECT_*`**: Non-fatal assertions (`EXPECT_EQ`, `EXPECT_TRUE`, `EXPECT_STREQ`, etc.)
- **`ASSERT_*`**: Fatal assertions (`ASSERT_EQ`, `ASSERT_TRUE`, etc.)
- **`::testing::InitGoogleTest()`**: Test initialization in `main()`
- **`RUN_ALL_TESTS()`**: Test execution entry point

## Building Tests

### Prerequisites

Install build dependencies:

```bash
sudo apt-get install build-essential cmake libboost-all-dev libgnuradio-dev libarmadillo-dev
```

### Configure with Tests

```bash
mkdir build && cd build
cmake .. \
    -DENABLE_UNIT_TESTING=ON \
    -DENABLE_SYSTEM_TESTING=OFF \
    -DENABLE_UNIT_TESTING_EXTRA=OFF
```

### Build Options

| CMake Option | Default | Description |
|---|---|---|
| `ENABLE_UNIT_TESTING` | OFF | Build unit tests |
| `ENABLE_UNIT_TESTING_MINIMAL` | OFF | Build only a minimal subset of unit tests |
| `ENABLE_UNIT_TESTING_EXTRA` | OFF | Download external signal files for additional tests |
| `ENABLE_SYSTEM_TESTING` | OFF | Build system-level integration tests |
| `ENABLE_SYSTEM_TESTING_EXTRA` | OFF | Extra system tests requiring GNSSsim |
| `ENABLE_BENCHMARKS` | OFF | Build performance benchmarks |

### Build and Run

```bash
# Build everything including tests
cmake --build build -- -j$(nproc)

# Run all tests via CTest (recommended)
cmake --build build --target check

# Or run the combined test binary directly
./install/run_tests
```

### Running Specific Tests

Use Google Test's `--gtest_filter` flag to run a subset:

```bash
# Run only tests matching a pattern
./install/run_tests --gtest_filter="*Acquisition*"

# Run a specific test case
./install/run_tests --gtest_filter="InMemoryConfiguration.StoreAndRetrieve"

# Run all tests in a test suite
./install/run_tests --gtest_filter="InMemoryConfiguration.*"
```

### Logging

Test logs are written to:

- **glog backend**: Directory specified by `--log_dir` (default: `/tmp`)
- **Abseil backend**: Automatically directed to a file; set `--log_dir` to control location

## How Tests Are Organized

### Registration

All unit tests are registered via `#include` directives in `test_main.cc`. The `main()` function calls:

```cpp
testing::InitGoogleTest(&argc, argv);
RUN_ALL_TESTS();
```

Preprocessor guards control which tests are included:

```cpp
// Core tests (always included when UNIT_TESTING is enabled)
#include "unit-tests/arithmetic/matio_test.cc"
#include "unit-tests/control-plane/in_memory_configuration_test.cc"

// Tests requiring compiled binaries (guarded by EXCLUDE_TESTS_REQUIRING_BINARIES)
#ifndef EXCLUDE_TESTS_REQUIRING_BINARIES
#include "unit-tests/control-plane/gnss_flowgraph_test.cc"
#endif

// Extra tests (guarded by EXTRA_TESTS, enabled with ENABLE_UNIT_TESTING_EXTRA)
#if EXTRA_TESTS
#include "unit-tests/signal-processing-blocks/acquisition/acq_performance_test.cc"
#endif

// Hardware-specific tests
#if FPGA_BLOCKS_TEST
#include "unit-tests/acquisition/gps_l1_ca_pcps_acquisition_test_fpga.cc"
#endif

#if CUDA_BLOCKS_TEST
#include "unit-tests/tracking/gpu_multicorrelator_test.cc"
#endif

#if OPENCL_BLOCKS_TEST
#include "unit-tests/acquisition/gps_l1_ca_pcps_opencl_acquisition_gsoc2013_test.cc"
#endif
```

### Standalone Test Executables

For CI efficiency, some tests are compiled as separate executables using `single_test_main.cc`:

| Executable | Source | Description |
|---|---|---|
| `flowgraph_test` | `gnss_flowgraph_test.cc` | Flowgraph construction tests |
| `gnss_block_test` | Multiple block test files | Signal processing block tests |
| `acq_test` | Acquisition test files | Acquisition algorithm tests |
| `trk_test` | Tracking test files | Tracking loop tests |
| `control_thread_test` | `control_thread_test.cc` | Control plane tests |

These are registered with CTest via `add_test()` for parallel execution:

```bash
ctest -V          # Run all registered tests verbosely
ctest -N          # List tests without running
ctest -R acq      # Run only tests matching "acq"
```

## Test Data

### Signal Samples

Some tests require recorded GNSS signal files. These are downloaded automatically at configure time from SourceForge:

| File | Constellations | MD5 |
|---|---|---|
| `Galileo_E1_ID_1_Fs_4Msps_8ms.dat` | Galileo E1b/c | `d57a02d3...` |
| `GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat` | GPS L1 C/A | `f12ada80...` |
| `GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat` | GPS + Galileo | `b98d6d82...` |
| `NT1065_GLONASS_L1_20160831_fs6625e6_if0e3_4ms.bin` | GLONASS L1 | `0e2dc212...` |

**Extra test data** (with `ENABLE_UNIT_TESTING_EXTRA=ON`):

| File | Constellations | MD5 |
|---|---|---|
| `gps_l2c_m_prn7_5msps.dat` | GPS L2C | `a6fcbefe...` |
| `Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat` | GLONASS L1 C/A | `ffb72fc6...` |
| `BdsB1IStr01_fs25e6_if0_4ms.dat` | BeiDou B1I | `5a4336da...` |
| `BdsB3IStr01_fs50e6_if0_4ms.dat` | BeiDou B3I | `066d0d84...` |
| `Test_vectors.zip` | OSNMA test vectors (GSC) | `8158aebe...` |

Files are downloaded to `${CMAKE_BINARY_DIR}/thirdparty/signal_samples/` at configure time. The `download_test_file()` CMake function (defined in `tests/CMakeLists.txt`) handles download with hash verification and optional extraction.

### XML Reference Data

Test XML files in `tests/data/` are used by RTKLIB-related tests:

- `obs_test1.xml` — Observation data
- `eph_GPS_L1CA_test1.xml` — GPS L1 C/A ephemeris
- `config_file_sample.txt` — Sample configuration for file source tests

These are copied to `${CMAKE_BINARY_DIR}/thirdparty/data/` at configure time.

## Common Test Flags

Test executables accept CLI flags for controlling behavior:

| Flag | Type | Default | Description |
|---|---|---|---|
| `--gnuplot_executable` | string | (auto) | Path to gnuplot binary for plotting |
| `--plot_acq_grid` | bool | false | Plot acquisition grid |
| `--plot_decimate` | int32 | 1 | Decimation factor for plots |

Glog/Abseil logging flags:

| Flag | Type | Default | Description |
|---|---|---|---|
| `--log_dir` | string | `/tmp` | Directory for log output |
| `--minloglevel` | int | 0 | Minimum log severity (0=INFO) |

Usage:

```bash
./install/run_tests --log_dir=/tmp/test_logs --gtest_filter="*Acquisition*"
```

## Adding a New Test

1. **Create the test file** at `tests/unit-tests/<category>/<component>_test.cc` following the naming convention
2. **Use Google Test macros** (`TEST()`, `TEST_F()`, `EXPECT_*`, `ASSERT_*`)
3. **Register the test** by adding an `#include` in `tests/test_main.cc` in the appropriate section
4. **Handle conditional compilation** with the appropriate preprocessor guard if the test has dependencies:
   - Hardware: `#if FPGA_BLOCKS_TEST`, `#if CUDA_BLOCKS_TEST`, `#if OPENCL_BLOCKS_TEST`
   - Binaries: `#ifndef EXCLUDE_TESTS_REQUIRING_BINARIES`
   - Extra data: `#if EXTRA_TESTS`
5. **Build and run**:

```bash
cmake --build build --target run_tests -j$(nproc)
./install/run_tests --gtest_filter="*YourNewTest*"
```

6. **For standalone test executables** (optional, for CI parallelization), create a new `add_executable()` + `add_test()` entry in `tests/CMakeLists.txt` using `single_test_main.cc`

## CI Integration

Tests run in CI via GitHub Actions (`.github/workflows/main.yml`):

- **Ubuntu builds**: Build with `ENABLE_UNIT_TESTING=ON` + `ENABLE_UNIT_TESTING_EXTRA=ON`, run `run_tests` binary and `position_test`
- **macOS builds**: Same test suite
- **Cross-arch builds** (aarch64, armv7, riscv64): Build verification only (no test execution)

The CI workflow compiles the `check` target:

```bash
cmake --build build --target check
```
