<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2011-2024 Carles Fernandez-Prades <carles.fernandez@cttc.es>
)
<!-- prettier-ignore-end -->

# Changelog

All notable changes to GNSS-SDR will be documented in this file.

## [Unreleased](https://github.com/gnss-sdr/gnss-sdr/tree/next)

### Improvements in Interoperability:

- Improved error handling in UDP connections.
- Make it possible to receive multiple constellations using a single channel
  wideband device (HackRF/LimeSDR/USRP). Demonstration:
  https://www.youtube.com/watch?v=ZQs2sFchJ6w
  https://www.youtube.com/watch?v=HnZkKj9a-QM
- Add the following signal sources for use when GNSS-SDR is operating on SoC
  FPGA boards (`-DENABLE_FPGA=ON`):

  - `ADRV9361_Z7035_Signal_Source_FPGA`: Analog Devices ADRV9361-Z7035 board.
  - `FMCOMMS5_Signal_Source_FPGA`: FMCOMMS5 analog front-end.
  - `MAX2771_EVKIT_Signal_Source_FPGA`: MAX2771 evaluation kit analog front-end.
  - `DMA_Signal_Source_FPGA`: FPGA DMA working in post-processing mode.

  When building GNSS-SDR for the SoC FPGA, the following options can be passed
  to CMake with possible values of `ON` or `OFF`, and their default value is
  `OFF`:

  - `-DENABLE_AD9361`: Checks if the IIO driver is installed and builds the
    `ADRV9361_Z7035_Signal_Source_FPGA` and the `FMCOMMS5_Signal_Source_FPGA`
    sources.
  - `-DENABLE_MAX2771`: Checks if the SPIdev driver is installed and builds the
    `MAX2771_EVKIT_Signal_Source_FPGA` source.
  - `-DENABLE_DMA_PROXY`: Checks if the DMA proxy driver is installed for
    controlling the DMA in the FPGA and enables its usage.

### Improvements in Portability:

- Fix building against google-glog 0.7.x.
- Find dependencies in the loongarch64 architecture.
- Soft transition from [GFlags](https://github.com/gflags/gflags) and
  [Google Logging (glog)](https://github.com/google/glog) to Abseil
  [Logging](https://abseil.io/docs/cpp/guides/logging) and
  [Flags](https://abseil.io/docs/cpp/guides/flags) libraries. While gflags and
  glog have dutifully served GNSS-SDR for over a decade, they are now showing
  signs of aging. The latest version of gflags dates back six years now, with
  its last commit in the master branch occurring two years ago. Glog remains
  well maintained, with its latest version v0.7.0 released in February 2024, but
  with no active development of new features and stuck at C++14. Abseil, on the
  other hand, represents a contemporary evolution in software development,
  supports C++17 and C++20, and has absorbed the functionalities of flags and
  logging from its predecessors. Furthermore, as Abseil has become a
  prerequisite for the latest versions of Protocol Buffers, its eventual
  inclusion in GNSS-SDR's indirect dependencies is inevitable. Leveraging Abseil
  allows for eliminating the need for gflags and glog, thereby reducing the
  number of mandatory dependencies for GNSS-SDR in forthcoming GNU/Linux
  distributions. For seamless integration, GNSS-SDR requires a quite recent
  minimum version of Abseil, v20240116. If an older version is detected, the
  library will not be utilized, and GNSS-SDR will fall back to using gflags and
  glog, which still can be used and are fully supported. A new CMake
  configuration option `-DENABLE_GLOG_AND_GFLAGS=ON` is available to force the
  usage of glog and gflags instead of Abseil, even if a valid version of that
  library is present. If the Abseil version installed in your system is too old
  but you still want to try it, you can also force the downloading and building
  of a recent version with the new CMake configuration flag
  `-DENABLE_OWN_ABSEIL=ON` (requires CMake >= 3.24, otherwise it has no effect).
  This change has a downside in maintainability, since the source code becomes
  plagued with preprocessor directives required to maintain compatibility both
  with gflags and glog, and with Abseil.
- Historically, GNSS-SDR linked against the GnuTLS library for cryptographic
  functions. If GnuTLS was not found, then the building system looked for and
  linked against OpenSSL as a fallback. This was due to the OpenSSL 1.x dual
  license scheme, which was incompatible with GPL v3.0 license, preventing it
  from being a mandatory dependency for GNSS-SDR in most GNU/Linux
  distributions. This issue was solved with the release of OpenSSL 3.0.0, which
  transitioned to the Apache License 2.0, fully compatible with GPL v3.0.
  Accordingly, the GNSS-SDR building system now looks for OpenSSL in the first
  place and, if not found, then it looks for GnuTLS as a fallback.

### Improvements in Usability:

- Tidy up the `conf/` folder.

See the definitions of concepts and metrics at
https://gnss-sdr.org/design-forces/

&nbsp;

## [GNSS-SDR v0.0.19.1](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.19.1) - 2024-01-26

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.10579595.svg)](https://doi.org/10.5281/zenodo.10579595)

- Fix formatting of the `CITATION.cff` file.

## [GNSS-SDR v0.0.19](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.19) - 2024-01-23

### Improvements in Efficiency:

- Fixed some performance inefficiencies detected by Coverity Scan.

### Improvements in Interoperability:

- Added a new PVT configuration boolean flag (`flag_geohash_log_out`) that
  enables or disables the Position Geohash tag output in INFO log files. Set to
  `false` by default.
- New fields have been added to the custom output stream defined by
  `monitor_pvt.proto`:
  - `utc_time` (a [RFC 3339](https://www.rfc-editor.org/rfc/rfc3339) datetime
    string),
  - velocity in the local ENU frame (`vel_e`, `vel_n`, and `vel_u`), in m/s,
  - the course over ground, `cog`, in degrees,
  - the status of the Galileo's High Accuracy Service, `galhas_status`:
    - 0: HAS data not available
    - 1: HAS Corrections applied
  - `geohash`, an
    [encoded geographic location](https://en.wikipedia.org/wiki/Geohash).

### Improvements in Maintainability

- Removed useless casts and shadowed variables, improving source code
  readability.

### Improvements in Portability:

- Updated local `cpu_features` library to v0.9.0.
- `volk_gnsssdr`: fix syntax for Python 3.12 without breaking backward
  compatibility with Python 2.7.
- Fixed linking against GNU Radio v3.10.9.1.
- Make use of new API if linking against VOLK >= 3.1.
- Fixed undefined behaviour in `volk_gnsssdr` arising from incompatibility
  between complex numbers in C and C++.
- Now build system paths are not leaked when cross-compiling.
- Enabled building using macOS Sonoma and `arm64` processor architecture.

### Improvements in Repeatability:

- A Kalman filter is now available in the PVT block, smoothing the outputs of a
  simple Least Squares solution and improving the precision of delivered fixes.
  It can be enabled by setting `PVT.enable_pvt_kf=true` in the configuration
  file. The user can set values for the measurement and process noise
  covariances with the following optional parameters (here with their default
  values): `PVT.kf_measures_ecef_pos_sd_m=1.0`, in [m];
  `PVT.kf_measures_ecef_vel_sd_ms=0.1`, in [m/s];
  `PVT.kf_system_ecef_pos_sd_m=2.0`, in [m]; and
  `PVT.kf_system_ecef_vel_sd_ms=0.5`, in [m/s].

### Improvements in Scalability:

- Fixed some potential data race conditions detected by Coverity Scan.

### Improvements in Usability:

- The Galileo E1B Reduced CED parameters usage has been set to `false` by
  default. You can activate its usage with `Galileo_E1B_Telemetry_Decoder=true`
  in your configuration file.
- The generation of Galileo E6B observables has been disabled if the user sets
  `PVT.use_e6_for_pvt=false`, fixing the PVT computation in some multi-band
  configurations.
- Fix bug in the custom binary output (`PVT.enable_monitor=true`) output rate.
  Before this fix, it was outputting data every 20 ms, instead of observing the
  `PVT.output_rate_ms` setting.
- Now the program exits properly if a SIGINT signal is received (_e.g._, the
  user pressing Ctrl+C, or another user application sending an interruption
  signal).
- The estimated CN0 value is now printed in the terminal when navigation data is
  succesfully decoded.
- Fixed GPS navigation message satellite validation.
- Latitude and longitude are now reported in the terminal with six decimal
  places (the sixth decimal place worths up to 0.11 m), instead of the
  overkilling nine (the ninth decimal place worths up to 110 microns).
  Similarly, height in meters is now reported with two decimal places instead of
  three, and velocity in m/s also with two decimal places instead of three.
- Fixed the rate at which KML, GPX, GeoJSON, and NMEA annotations are made. The
  rate is now set by `PVT.output_rate_ms` (`500` ms by default), and can be
  particularized by `PVT.kml_rate_ms`, `PVT.gpx_rate_ms`, `PVT.geojson_rate_ms`,
  and `PVT.nmea_rate_ms`. Those values should be multiples of
  `PVT.output_rate_ms`, or the least common multiple will be taken.

See the definitions of concepts and metrics at
https://gnss-sdr.org/design-forces/

&nbsp;

## [GNSS-SDR v0.0.18](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.18) - 2023-04-06

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.7805514.svg)](https://doi.org/10.5281/zenodo.7805514)

### Improvements in Accuracy:

- Processing and application of the corrections provided by the Galileo High
  Accuracy Service (HAS) to the PVT solution. It requires at least a Galileo E1
  (or E5a) + Galileo E6B configuration. A new configuration parameter
  `PVT.use_has_corrections`, set to `true` by default, can be used to deactivate
  the application of HAS corrections but still retrieve the HAS data if set to
  `false`.

### Improvements in Availability:

- Fixed bug that made the PVT block to not resolve position anymore after a loss
  of samples event.
- Improved non-coherent acquisition when `Acquisition_XX.blocking=false`.
- Implemented processing of BeiDou PRN 34 up to PRN 63 signals.
- Implemented Hamming code correction for Glonass navigation message.
- Now the first iteration of the PVT computation is initialized by the Bancroft
  method. This allows to get PVT fixes in some unusual geometries (_e.g._,
  GNSS-like signals transmitted by LEO satellites). This initialization is
  performed by default. You can opt-out by setting `PVT.bancroft_init=false` in
  your configuration file.

### Improvements in Interoperability:

- Enabled PVT computation in the Galileo E5a + E5b receiver. Observables
  reported in the RINEX file.
- Fixed PVT computation in the Galileo E5b-only receiver.
- Get E6B observables and PVT solutions in the Galileo E1B + E6B receiver.
  Decoding of HAS messages as described in the
  [HAS SIS ICD v1.0](https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_HAS_SIS_ICD_v1.0.pdf).
  Generation of RTCM 3.2 messages from the received HAS messages in the
  [IGS State Space Representation (SSR) Format](https://files.igs.org/pub/data/format/igs_ssr_v1.pdf).
  Specifically, it generates messages of type IGM01 (SSR Orbit Correction),
  IGM02 (SSR Clock Correction), IGM03 (SSR Combined Orbit and Clock Correction),
  and IGM05 (SSR Code Bias).
- Added a `ZMQ_Signal_Source` for working with streams of samples published via
  [ZeroMQ](https://zeromq.org/).
- Fixed register unpacking for Labsat3W files in `Labsat_Signal_Source`. This
  fix is only available if gnss-sdr is linked against Boost >= 1.58.0.

### Improvements in Maintainability:

- The now archived [GPSTk toolkit](https://github.com/SGL-UT/GPSTk), used in
  some optional tests and applications, has been replaced by the new
  [GNSSTk](https://github.com/SGL-UT/gnsstk) C++ Library. Compatibility with the
  former GPSTk toolkit is maintained.

### Improvements in Portability:

- Improved detection of the BLAS library under macOS / Macports (the `lapack`
  port dependency installed with the `+openblas` variant does not install `blas`
  but `openblas`, which is used as a replacement if `blas` is not found).
- Removed duplicated files in the Secure User Plane Location implementation,
  which caused issues when linking with some compilers.
- Added support for Xilinx's Zynq UltraScale+ devices (requires the
  `-DENABLE_FPGA=ON` building option).
- Fixed running time error if the `gnss-sdr` binary and/or the GNU Radio
  libraries were built with the `-D_GLIBCXX_ASSERTIONS` compiler option. This is
  added by default in some GNU/Linux distributions (e.g., ArchLinux and Fedora).
- Fixed linking against libunwind when the glog library is built locally.
- The configuration options at building time `-DENABLE_OWN_GLOG`,
  `-DENABLE_OWN_ARMADILLO`, and `-DENABLE_OWN_GNSSTK` can now be switched `ON`
  and `OFF` without the need to start from an empty buiding folder.
- Improved CMake handling of the spdlog library used by GNU Radio >= 3.10.
- Make use of the C++20 standard if the environment allows for it.
- Improved passing of compiler flags to `volk_gnsssdr` if the corresponding
  environment variables are defined. This fixes warnings in some packaging
  systems.
- Improved support for the RISC-V architecture.
- Test files are now donwloaded at configuration time instead of being included
  in the source tree. This allows for a smaller package and fixes Lintian
  `very-long-line-length-in-source-file` warnings since those files were not
  recognized as binaries. The configuration flag `-DENABLE_PACKAGING=ON` passed
  to CMake deactivates file downloading.
- The `ENABLE_GENERIC_ARCH` building option was removed, simplifying the process
  of buiding the software in non-x86 processor architectures.
- If the Protocol Buffers dependency is not found, it is downloaded, built and
  statically linked at buiding time. If CMake >= 3.13 and the
  [Abseil C++ libraries](https://github.com/abseil/abseil-cpp) >= 20230117 are
  installed on your system, Protocol Buffers v22.2 will be used. If those
  requirements are not met, Protocol Buffers v21.4 will be used instead
  (requires autotools).
- Since Debian 8 "Jessie", which enjoyed Long Term Support until the end of June
  2020, is not anymore in the Debian official repositories, we drop its support.
- Fixes for GCC 13 and Clang 16.

### Improvements in Usability:

- Fixed large GLONASS velocity errors and the extended correlator when using the
  `GLONASS_L1_CA_DLL_PLL_C_Aid_Tracking` and
  `GLONASS_L2_CA_DLL_PLL_C_Aid_Tracking` implementations.
- The `UHD_Signal_Source` learned a new parameter `otw_format` for setting the
  [over-the-wire data format](https://files.ettus.com/manual/page_configuration.html#config_stream_args_otw_format)
  (that is, the format used between the device and the UHD) in some devices,
  thus allowing to select the `sc8` format instead of the default `sc16`. This
  would reduce the dynamic range and increase quantization noise, but also
  reduce the load on the data link and thus allow more bandwidth.
- The `UHD_Signal_Source` learned another two optional parameters:
  `device_recv_frame_size` and `device_num_recv_frames` for overriding
  [transport layer defaults](https://files.ettus.com/manual/page_transport.html).
- Added gain setting and reading for the XTRX board when using the
  `Osmosdr_Signal_Source` implementation of a `SignalSource`.
- The `Osmosdr_Signal_Source` implementation learned a new parameter `if_bw` to
  manually set the bandwidth of the bandpass filter on the radio frontend.
- The new configuration parameter `Channels_XX.RF_channel_ID` allows to specify
  the signal source per channel group.
- New configuration parameter `PVT.use_unhealthy_sats`, set by default to
  `false`, allows processing observables of satellites that report an unhealthy
  status in the navigation message if set to `true`.
- Added the [Geohash](https://en.wikipedia.org/wiki/Geohash) of the PVT solution
  in the internal logs.
- Allowed the CMake project to be a sub-project.

See the definitions of concepts and metrics at
https://gnss-sdr.org/design-forces/

&nbsp;

## [GNSS-SDR v0.0.17](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.17) - 2022-04-20

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.6473244.svg)](https://doi.org/10.5281/zenodo.6473244)

### Improvements in Availability:

- Compute PVT solutions when using GPS L5 signals even if the satellite is
  reported as not healthy in the CNAV message.

### Improvements in Portability:

- Updated `cpu_features` library to v0.7.0. The building option
  `ENABLE_OWN_CPUFEATURES` has been replaced by `ENABLE_CPUFEATURES`, defaulting
  to `ON`.
- Fixed building against GNU Radio v3.10.2.0.

### Improvements in Reliability:

- Fix some defects detected by Coverity Scan 2021.12.1.

### Improvements in Usability:

- Added a script at `src/utils/scripts/download-galileo-almanac.sh` that
  downloads an XML file with the latest Galileo almanac published by the
  European GNSS Service Centre at https://www.gsc-europa.eu/gsc-products/almanac

See the definitions of concepts and metrics at
https://gnss-sdr.org/design-forces/

&nbsp;

## [GNSS-SDR v0.0.16](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.16) - 2022-02-15

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.6090349.svg)](https://doi.org/10.5281/zenodo.6090349)

### Improvements in Availability:

- Added the Galileo E5b receiving chain. The software is now able to compute PVT
  solutions as a standalone Galileo E5b receiver.
- Improved Time-To-First-Fix when using GPS L1 C/A signals, fixing a bug that
  was making the receiver to drop the satellite if the PLL got locked at 180
  degrees, and making some optimizations on bit transition detection.
- Fixed a bug that prevented from obtaining PVT fixes with Galileo E1 OS signals
  if the I/NAV subframe type 0 was the first decoded subframe.

### Improvements in Interoperability:

- Fixed setting of the signal source gain if the AGC is enabled when using the
  AD9361 front-end.
- Fixed the regeneration of Galileo ephemeris from the reduced clock and
  ephemeris data (CED) defined in the Galileo E1B INAV message introduced in
  Galileo OS SIS ICD Issue 2.0.
- Added a `Limesdr_Signal_Source` for interoperability with LimeSDR (requires
  [gr-limesdr](https://github.com/myriadrf/gr-limesdr) and the
  `-DENABLE_LIMESDR=ON` building flag).

### Improvements in Maintainability:

- Rewritten Viterbi decoder for Galileo navigation messages. Encapsulated in a
  class instead of being implemented as free inline functions. This improves
  memory management and source code readability.
- Prefer initialization to assignment in constructors. This improves the
  readability of the code, could potentially increase performance, and allows
  for easier detection of unused data members (see the
  [CppCoreGuidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md/#Rc-initialize)).
  Added the `cppcoreguidelines-prefer-member-initializer` clang-tidy check to
  enforce this policy.
- Non-functional change: Fixed formatting defects detected by clang-format 13.0.
- Non-functional change: Simplified flow graph disconnection.
- Updated GSL implementation to v0.40.0. See the
  [gsl-lite release](https://github.com/gsl-lite/gsl-lite/releases/tag/v0.40.0).
- CI - `cpplint` job on GitHub: Added the `build/include_what_you_use` filter
  for early detection of missing includes.
- CI - `clang-tidy` job on GitHub: More robust detection of LLVM paths installed
  by homebrew.

### Improvements in Portability:

- Fixed building against the new API in the gr-iio component present in GNU
  Radio v3.10.X.Y.
- Fixed building against GNU Radio v3.10.X.Y, which does not support the C++20
  standard.
- Fixed building against GNU Radio v3.10.X.Y, which replaced
  [log4cpp](https://log4cpp.sourceforge.net/) by the
  [spdlog](https://github.com/gabime/spdlog) and
  [fmt](https://github.com/fmtlib/fmt) libraries.
- Updated `cpu_features` library for improved processor detection.

### Improvements in Reliability:

- Fixed some potential buffer overflows.
- Avoid source code lines longer than 512 characters. This was a warning raised
  by Lintian (very-long-line-length-in-source-file). Long lines in source code
  could be used to obfuscate the source code and to hide stuff like backdoors or
  security problems.

### Improvements in Usability:

- Added a new monitor to extract the decoded data bits of the navigation
  messages and send them elsewhere via UDP. Activated by setting
  `NavDataMonitor.enable_monitor=true`,
  `NavDataMonitor.client_addresses=127.0.0.1` and `NavDataMonitor.port=1237` in
  the configuration file. Format described in the `nav_message.proto` file. A
  simple listener application written in C++ is included in
  `src/utils/nav-listener` as an example.
- Extract successful rate of the CRC check in the decoding of navigation
  messages. This can be enabled by setting
  `TelemetryDecoder_XX.dump_crc_stats=true` and, optionally,
  `TelemetryDecoder_XX.dump_crc_stats_filename=./crc_stats` in the configuration
  file. At the end of the processing (or exiting with `q` + `[Enter]`), the CRC
  check success rate will be reported in a file.
- The `UHD_Signal_Source` learned to dump data in folders that do not exist,
  _e.g._, if `SignalSource.dump=true`,
  `SignalSource.dump_filename=./non-existing/data.dat`, and the `non-existing`
  folder does not exist, it will be created if the running user has writing
  permissions. This also works for absolute paths.
- Added a new configuration parameter `PVT.rtk_trace_level` that sets the
  logging verbosity level of the RTKLIB library.
- Added a new output parameter `Flag_PLL_180_deg_phase_locked` in the monitor
  output that indicates if the PLL got locked at 180 degrees, so the symbol sign
  is reversed.
- Fixed a bug in the satellite selection algorithm for configurations with a
  large number of channels. The maximum number of channels per signal is now
  limited to the number of available satellites per system minus one. The number
  of channels performing concurrent acquisition, `Channels.in_acquisition`,
  cannot be larger than the total number of channels. The program will stop if
  those requirements are not met in the configuration file.
- Fixed program termination when using `File_Signal_Source` and extended
  integration times.
- The `Fifo_Signal_Source` Signal Source implementation learned to handle the
  `ibyte` type.
- Added a `CITATION.cff` file.
- Updated version of the Contributor Covenant to version 2.1.

See the definitions of concepts and metrics at
https://gnss-sdr.org/design-forces/

&nbsp;

## [GNSS-SDR v0.0.15](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.15) - 2021-08-23

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.5242839.svg)](https://doi.org/10.5281/zenodo.5242839)

### Improvements in Availability:

- Added the reading of reduced clock and ephemeris data (CED) in the Galileo E1B
  INAV message introduced in Galileo OS SIS ICD Issue 2.0. If the reduced CED is
  available before the full ephemeris set, it is used for PVT computation until
  the full set has not yet been received. This can contribute to shortening the
  Time-To-First-Fix. Still experimental.
- Added the exploitation of the FEC2 Erasure Correction in the Galileo E1B INAV
  message introduced in Galileo OS SIS ICD Issue 2.0. This can contribute to
  shortening the Time-To-First-Fix. Since the added computational cost could
  break some real-time configurations, this feature is disabled by default. It
  can be activated from the configuration file by adding
  `TelemetryDecoder_1B.enable_reed_solomon=true`.
- Reduction of the TTFF in GPS L1 and Galileo E1 by improving the frame
  synchronization mechanism.

### Improvements in Maintainability:

- The Contributor License Agreement (CLA) signing for new contributors has been
  replaced by a
  [Developer's Certificate of Origin (DCO)](https://github.com/gnss-sdr/gnss-sdr/blob/next/.github/DCO.txt),
  which implies that contributed commits in a pull request need to be signed as
  a manifestation that contributors have the right to submit their work under
  the open source license indicated in the contributed file(s) (instead of
  asking them to sign the CLA document).
- Improved handling of changes in GNU Radio 3.9 FFT API.
- Improved handling of the filesystem library.
- Added an abstract class `SignalSourceInterface` and a common base class
  `SignalSourceBase`, which allow removing a lot of duplicated code in Signal
  Source blocks, and further abstract file-based interfaces behind them.
- Do not apply clang-tidy fixes to protobuf-generated headers.
- Refactored private implementation of flow graph connection and disconnection
  for improved source code readability.
- Added a base class for GNSS ephemeris, saving some duplicated code and
  providing a common nomenclature for ephemeris' parameters. New generated XML
  files make use of the new parameters' names.
- Update GSL implementation to 0.38.1. See
  https://github.com/gsl-lite/gsl-lite/releases/tag/v0.38.1
- Update references to the latest GPS ICDs (IS-GPS-200M, IS-GPS-800H,
  IS-GPS-705H) published in May, 2021.

### Improvements in Portability:

- Avoid collision of the `cpu_features` library when installing the
  `volk_gnsssdr` library on its own, and VOLK has already installed its version.
  Added a new building option `ENABLE_OWN_CPUFEATURES`, defaulting to `ON` when
  building `gnss-sdr` but defaulting to `OFF` when building a stand-alone
  version of `volk_gnsssdr`. When this building option is set to `ON`, it forces
  the building of the local version of the `cpu_features` library, regardless of
  whether it is already installed or not.
- CMake's `<policy_max>` version bumped to 3.21. The minimum CMake version is
  2.8.12.
- Fix building when using the Xcode generator, Xcode >= 12 and CMake >= 3.19.
- Fix building of FPGA blocks when linking against GNU Radio >= 3.9 and/or
  Boost >= 1.74.
- Fix linking of the `<filesystem>` library when using GCC 8.x and GNU Radio >=
  3.8.
- If the Matio library is not found, now it is configured and built by CMake
  instead of using autotools.
- Added support for Apple M1 AArch64 architecture processor and for FreeBSD on
  x86, improved AMD microarchitecture detection.
- CMake now selects the C++23 standard if the environment allows for it.
- Improved detection of Gnuplot and `gnss_sim` when cross-compiling.
- NEON kernel implementations of the `volk_gnsssdr` library are now enabled in
  aarch64 architectures.

### Improvements in Reliability

- Bug fix in the Galileo E1/E5 telemetry decoder that produced incorrect timing
  information if a satellite is lost and then readquired.
- Check satellites' health status. If a satellite is marked as not healthy in
  its navigation message, the corresponding observables are not used for
  navigation.

### Improvements in Usability:

- Added a new `Fifo_Signal_Source` implementation that allows using a
  [Unix FIFO](https://en.wikipedia.org/wiki/Named_pipe) as a signal source, thus
  allowing to multiplex signal streams outside of `gnss-sdr`, letting another
  program hold access to the receiver, or allowing signal sources that are not
  supported by `gnss-sdr` but can dump the signal to a FIFO.
- Avoid segmentation faults in the flow graph connection and/or starting due to
  some common inconsistencies in the configuration file.
- Provide hints to the user in case of failed flow graph connection due to
  inconsistencies in the configuration file.
- Fix segmentation fault if the RINEX output was disabled.
- Added a feature that optionally enables the remote monitoring of GPS and
  Galileo ephemeris using UDP and [Protocol Buffers](https://protobuf.dev/).
- Now building the software passing the `-DENABLE_FPGA=ON` to CMake does not
  make the receiver unusable when running on non-FPGA-enabled platforms. On
  FPGA-enabled platforms, now it is possible to run non-FPGA-enabled
  configurations.
- Fix bug that made the Monitor block to always set to 0 the
  `carrier_phase_rads` parameter value.
- The `Labsat_Signal_Source` implementation of the `SignalSource` block now can
  read files in the LabSat 3 Wideband format (`.LS3W`). When using this format,
  this source block can provide multiple RF chain outputs.
- Replace `Receiver.sources_count` configuration parameter name by
  `GNSS-SDR.num_sources`. The former parameter name is still read to ensure
  backward compatibility with configuration files using that nomenclature.
- Fix bug in searching for gr-iio when CMake was re-run several times with
  different settings for the `-DENABLE_FMCOMMS2` or `-DENABLE_PLUTOSDR` building
  options.
- Fix building when using UHD >= v4.0.0.0.
- Fix building for `-DENABLE_FMCOMMS2=ON` and/or `-DENABLE_PLUTOSDR=ON` when the
  built-in gr-iio module introduced in GNU Radio 3.10 is found. This in-tree GNU
  Radio module takes precedence over the gr-iio package provided at
  https://github.com/analogdevicesinc/gr-iio. If the GNU Radio module is found,
  the other one is ignored.
- File `changelog.md` renamed to the more usual `CHANGELOG.md` uppercase name.
- New global configuration parameter `GNSS-SDR.observable_interval_ms`, set by
  default to 20 [ms], allows to control the internal rate at which computed
  observables sets are processed (50 observables sets per second by default).
- Fix bug in the `Monitor.decimation_factor` parameter, which was not working
  properly for other values than 1.

See the definitions of concepts and metrics at
https://gnss-sdr.org/design-forces/

&nbsp;

## [GNSS-SDR v0.0.14](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.14) - 2021-01-08

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4428100.svg)](https://doi.org/10.5281/zenodo.4428100)

### Improvements in Availability:

- Fixed bug in acquisition detection when the configuration parameter
  `Acquisition_XX.threshold` was set but `Acquisition_XX.pfa` was not, causing
  false locks.
- Fixed anti-jamming filters: `Pulse_Blanking_Filter`, `Notch_Filter`, and
  `Notch_Filter_Lite`.

### Improvements in Efficiency:

- Faster `SignalConditioner` block when its implementation is set to
  `Pass_Through`.

### Improvements in Interoperability:

- Added the Galileo E6 B/C signal structure based on E6-B/C Codes Technical
  Note, Issue 1, January 2019, including Acquisition and Tracking blocks. The
  Telemetry Decoder is still empty (only the CRC is checked, based on Galileo
  High Accuracy Service E6-B Signal-In-Space Message Specification v1.2, April
  2020).

### Improvements in Maintainability:

- Added a common shared pointer definition `gnss_shared_ptr`, which allows
  handling the `boost::shared_ptr` to `std::shared_ptr` transition in GNU Radio
  3.9 API more nicely.
- Support new FFT and firdes blocks' API in GNU Radio 3.9.
- Added detection of inconsistent function prototypes in `volk_gnsssdr` library
  kernels at compile time.
- Fixed defects detected by clang-tidy check `bugprone-reserved-identifier`, and
  added to the checks list. This check corresponds to CERT C Coding Standard
  rule
  [DCL37-C](https://wiki.sei.cmu.edu/confluence/display/c/DCL37-C.+Do+not+declare+or+define+a+reserved+identifier)
  as well as its C++ counterpart,
  [DCL51-CPP](https://wiki.sei.cmu.edu/confluence/display/cplusplus/DCL51-CPP.+Do+not+declare+or+define+a+reserved+identifier).
- Applied and added more clang-tidy checks related to readability:
  `readability-make-member-function-const` and `readability-qualified-auto`.

### Improvements in Portability:

- Fixed `-DENABLE_OWN_GLOG=ON` building option when gflags is installed and it
  is older than v2.1.2 (_e.g._, in CentOS 7).
- Improved handling of old gflags versions, minimum version set to 2.1.2.
  Replaced `google::` by `gflags::` namespace when using functions of the gflags
  library.
- Replaced `git://` by `https://` as the used protocol when downloading Gflags,
  so it can work through firewalls requiring authentication.
- Fixed static linking of the matio library when downloaded and built by CMake.
- Improved CPU feature detection by switching to Google's
  [cpu_features](https://github.com/google/cpu_features) library: The
  `volk_gnsssdr` library had its own CPU feature detection methods, which were
  not totally reliable and difficult to implement across compilers and OSes.
  This is now handled by the `cpu_features` library, thus building upon that
  expertise. Since that library has higher dependency version requirements than
  GNSS-SDR, the old method is still used in old development environments. No
  extra dependency is needed. This change is transparent to the user, since
  everything is managed by the CMake scripts.
- The `volk_gnsssdr` library can be built on Microsoft Windows and can execute
  SIMD instructions on that OS.
- Removed all instances of `_mm256_zeroupper()` in the `volk_gnsssdr` library,
  since they are not required and lead to miscompilation with GCC 10.2 and
  optimization level `-O3`.
- Fixed building with `-DENABLE_CUDA=ON` for blocks implemented with CUDA.
- Fixed linking against the ORC library if it is present in the system.
- Fixed a bug introduced in v0.0.13 that prevented getting Galileo-only PVT
  fixes in some environments.
- Fixed duplication of protobuf build tree if it was locally built and then
  installed with `DESTDIR` variable set.

### Improvements in Usability:

- Fixed a bug when enabling pseudorange carrier smoothing in other bands than
  L1.
- If `SignalConditioner.implementation=Pass_Through`, then all the configuration
  parameters for the `DataTypeAdapter`, `InputFilter` and `Resampler` blocks are
  ignored. This was the default behavior in GNSS-SDR v0.0.12, but it changed in
  v0.0.13. This change recovers the old behavior.
- Fixed occasional segmentation fault when exiting with `q` + `[Enter]` keys if
  `Acquisition_XX.blocking=false`.
- Fixed the termination of the receiver with `q` + `[Enter]` keys when using the
  `Osmosdr_Signal_Source` implementation of the `SignalSource` block.
- The `Labsat_Signal_Source` implementation of the `SignalSource` block now can
  be throttled with the new parameters `SignalSource.enable_throttle_control`
  and `SignalSource.throttle_frequency_sps`, thus allowing the emulation of
  real-time operation.
- Improved General Block diagram, both in content and in image resolution.
- The `Custom_UDP_Signal_Source` implementation now accepts
  `SignalSource.sample_type=cfloat`, in addition to the existing 4 and 8-bit
  length sample types.
- Fixed the `obsdiff` and `rinex2assist` utilities when installed if they were
  built with a locally downloaded version of GPSTk.
- The generated HTML documentation now makes use of the Doxygen grouping
  feature.
- Improved rendering of equations in HTML documentation generated by Doxygen.
  Make use of MathJax for equation rendering. Added new building option
  `ENABLE_EXTERNAL_MATHJAX`, set to `ON` by default. If set to `OFF`, it allows
  using a local installation of MathJax 2.
- Improved dumps in Telemetry Decoding blocks. Now they include the raw
  navigation message bits. If `TelemetryDecoder_XX.dump=true`, the resulting
  `.dat` binary file is also delivered in `.mat` format, which is readable from
  Matlab and Python.

See the definitions of concepts and metrics at
https://gnss-sdr.org/design-forces/

&nbsp;

## [GNSS-SDR v0.0.13](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.13) - 2020-07-29

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3965566.svg)](https://doi.org/10.5281/zenodo.3965566)

### Improvements in Efficiency:

- Faster internal handling of `Gnss_Synchro` objects by reducing the amount of
  copying via adding `noexcept` move constructor and move assignment operators,
  so the move semantics are also used in STL containers.
- All `std::endl` have been replaced by the `'\n'` character, since there is no
  need to always flush the stream.
- Performed a stack reordering of class members that seems to offer
  statistically better performance in some processor architectures and/or
  compilers.
- Add building option `ENABLE_STRIP` to generate stripped binaries (that is,
  without debugging symbols), smaller in size and potentially providing better
  performance than non-stripped counterparts. Only for GCC in Release build
  mode. Set to `OFF` by default.

### Improvements in Maintainability:

- Improved usage of smart pointers to better express ownership of resources.
- Add definition of `std::make_unique` for buildings with C++11, and make use of
  it through the source code.
- Private members in headers have been sorted by type and size, minimizing
  padding space in the stack and making the files more readable for humans.
- Simpler, less error-prone design of the `GNSSBlockFactory` class public API
  and internal implementation.
- Simpler API for the `Pvt_Solution` class.
- Improved system constant definition headers, numerical values are only written
  once.
- Improved `const` correctness.
- The software can now be built against the GNU Radio 3.9 API that uses standard
  library's smart pointers instead of Boost's. Minimum GNU Radio required
  version still remains at 3.7.3.
- The software can now be built against Boost <= 1.73 (minimum version: 1.53).
- Fixed building with GCC 10 (gcc-10 and above flipped a default from `-fcommon`
  to `-fno-common`, causing an error due to multiple defined lambda functions).
- Fixed warnings raised by GCC 10 and Clang 10.
- Various improvements in the CMake scripts: better decision on the C++ standard
  to use; simplifications for various API dependency and environment versions
  requirements, with more intuitive naming for variables; fixed the
  `ENABLE_CLANG_TIDY` option; better GFORTRAN module; and broader adoption of
  the modern per-target approach.

### Improvements in Portability:

- The software can now be cross-compiled on Petalinux environments.
- Removed python six module as a dependency if using Python 3.x.
- Make use of `std::span` if the compiler supports it, and use `gsl-lite` as a
  fallback. The latter has been updated to version
  [0.37.0](https://github.com/gsl-lite/gsl-lite/releases/tag/0.37.0).
- Improved finding of `libgfortran` in openSUSE and Fedora distributions.
- Improved interface for FPGA off-loading.
- Allow a random name for the build type. If not recognized, it is set to
  `None`. This allows packaging in some distributions that pass an arbitrary
  name as the build type (e.g., Gentoo) to avoid unexpected compiler flags. The
  building option `ENABLE_PACKAGING` must be set to `ON` when packaging.
- Do not stop the receiver if SysV message queues cannot be created.

### Improvements in Reliability:

- Fixed a bug in GLONASS GNAV CRC computation.
- Fixed a bug in GLONASS time year.
- Fixed a possible buffer overflow in the generation of RTCM messages.
- Fixed bugs which could cause a random crash on receiver stopping.

### Improvements in Reproducibility:

- Improved reproducibility of the `volk_gnsssdr` library: Drop compile-time CPU
  detection.

### Improvements in Testability:

- Add building option `ENABLE_BENCHMARKS`, which activates the building of
  benchmarks for some code snippets, making it easier to developers to benchmark
  different implementations for the same purpose. Set to `OFF` by default.

### Improvements in Usability:

- Do not pollute the source directory if the software is built from an
  out-of-source-tree directory. Downloaded external sources and test raw files
  are now stored in a `./thirdparty` folder under the building directory. In the
  case of an out-of-source-tree build, the generated binaries are stored in an
  `./install` folder, also under the building directory. The old behavior for
  generated binaries is maintained if the building is done from any source tree
  subfolder (for instance, `gnss-sdr/build`): in that case, binaries are stored
  in the source tree (under `gnss-sdr/install`).
- Defined new `GNSS-SDR.GPS_banned_prns`, `GNSS-SDR.Galileo_banned_prns`,
  `GNSS-SDR.Glonass_banned_prns` and `GNSS-SDR.Beidou_banned_prns` configuration
  parameters. The user can specify lists of satellites that will not be
  processed (e.g., `GNSS-SDR.Galileo_banned_prns=14,18` since Galileo E14 and
  E18 satellites are not usable for PVT). Satellites on those lists will never
  be assigned to a processing channel.
- Added acquisition and tracking monitors, with configuration examples.
- Added a Matlab script to quantize the input signal with a given number of bits
  per sample.
- Fixed the building option `-DENABLE_LOG=OFF`, which strips internal logging
  from the binary and can help to reduce its size and ultimately to speed up the
  receiver. In binaries with enabled logging, it still can be disabled by
  passing the command line flag `--minloglevel=3` to `gnss-sdr`. This can be
  relevant in embedded devices with scarce storage capabilities.
- Fixed a bug in the Signal Sources configuration that made the number of
  samples parameter ignored when too large (that is, when set larger than
  2^31-1). Now the `samples` parameter accepts values up to 2^64-1, that is,
  18,446,744,073,709,551,615 samples.
- Fixed a bug in the forwarding of NMEA messages to a serial port (configuration
  of the `PVT.nmea_dump_devname` parameter was ignored).
- Updated version of the Contributor Covenant to version 2.0. Added badge in the
  README.md file.

See the definitions of concepts and metrics at
https://gnss-sdr.org/design-forces/

&nbsp;

## [GNSS-SDR v0.0.12](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.12) - 2020-03-13

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3709089.svg)](https://doi.org/10.5281/zenodo.3709089)

### Improvements in Accuracy:

- Improved accuracy of the C/N0 estimator.

### Improvements in Availability:

- Fixed computation of acquisition threshold when using the `Acquisition_XX.pfa`
  configuration parameter, including non-coherent accumulation
  (`Acquisition_XX.max_dwells` > 1).

### Improvements in Efficiency:

- Faster implementation of the Viterbi decoder for Galileo navigation messages.
- The `-O3` flag is now passed to GCC in `Release` and `RelWithDebInfo` build
  types (instead of `-O2`), thus enabling tree vectorization. Disabled if
  building for Fedora or Gentoo.

### Improvements in Flexibility:

- New Tracking parameters allow the configuration of the C/N0 and lock detector
  smoothers, as well as the activation of the FLL in pull-in and steady-state
  stages.
- Added new Tracking parameter `Tracking_XX.carrier_aiding`, allowing
  enabling/disabling of carrier aiding to the code tracking loop.
- New PVT parameter `PVT.enable_rx_clock_correction` allows to enable or disable
  the continuous application of the Time solution correction (clock steering) to
  the computation of Observables. By default is set to `false` (that is,
  disabled).
- New PVT parameter `PVT.max_clock_offset_ms`: if
  `PVT.enable_rx_clock_correction` is set to `false`, this parameter sets the
  maximum allowed local clock offset with respect to the Time solution. If the
  estimated offset exceeds this parameter, a clock correction is applied to the
  computation of Observables.
- Fixed L5 and E5a receiver chains when tracking the data component.
- Added new PVT configuration parameter `PVT.rinex_name` to specify a custom
  name of the generated RINEX files. A commandline flag `--RINEX_name` is also
  available, and overrides the configuration.

### Improvements in Interoperability:

- Fixed PVT solution in receivers processing L1 plus L2C and/or L5 signals.
- Fixed the initialization of the carrier phase accumulator. Carrier phase
  measurements are now usable to compute integer ambiguity resolution.
- Added carrier phase observable initialization to match the pseudorange length.
- Added RINEX files generation for triple-band configurations (L1 + L2C + L5 and
  L1 + E1 + L2C + L5 + E5a).
- Fixed bugs in the decoding of BeiDou navigation messages.
- Fixed bugs in the generation of RTCM messages.
- Fixed a bug in feeding Galileo channels' observations to RTKLIB, which was
  causing wrong date of PVT fixes in Galileo-only receivers and not considering
  Galileo observations in multi-constellation receivers when using signals after
  the GPS rollover on April 6, 2019.
- Improved management of devices with the AD9361 RF transceiver.
- Fixed bugs in FPGA off-loading.

### Improvements in Maintainability:

- Rewriting of acquisition and tracking adapters, thus avoiding a lot of code
  duplication.
- New CMake option `-DENABLE_ARMA_NO_DEBUG` defines the macro `ARMA_NO_DEBUG`,
  which disables all run-time checks, such as bounds checking, in the Armadillo
  library. This will result in faster code. This option is disabled by default
  during development, but automatically set to `ON` if the option
  `ENABLE_PACKAGING` is set to `ON`.
- All shadowed variables detected by passing `-Wshadow` to the compiler have
  been fixed (see https://rules.sonarsource.com/cpp/RSPEC-1117?search=shadow and
  MISRA C++:2008, 2-10-2 - Identifiers declared in an inner scope shall not hide
  an identifier declared in an outer scope).
- Apply more clang-tidy checks related to readability:
  `readability-avoid-const-params-in-decls`,
  `readability-braces-around-statements`, `readability-isolate-declaration`,
  `readability-redundant-control-flow`, and
  `readability-uppercase-literal-suffix`. Fixed raised warnings.
- Fixed a number of defects detected by `cpplint.py`. Filters applied:
  `+build/class`, `+build/c++14`, `+build/deprecated`,
  `+build/explicit_make_pair`, `+build/include_what_you_use`,
  `+build/printf_format`, `+build/storage_class`, `+readability/constructors`,
  `+readability/namespace`, `+readability/newline`, `+readability/utf8`,
  `+runtime/casting`, `+runtime/explicit`, `+runtime/indentation_namespace`,
  `+runtime/init`, `+runtime/invalid_increment`,
  `+runtime/member_string_references`, `+runtime/memset`, `+runtime/operator`,
  `+runtime/printf`, `+runtime/printf_format`, and `+whitespace/blank_line`.
- `clang-format` can now be applied to the whole code tree without breaking
  compilation.
- Added more check options to `.clang-tidy` file.
- Default Python version is now >= 3.4. Python 2.7 still can be used in systems
  where Python 3 is not available (e.g., CentOS 7, Debian 8, Ubuntu 10.04).
- CMake now passes the `-DCMAKE_BUILD_TYPE` (or configuration in
  multi-configuration generators like Xcode) to modules built along with
  `gnss-sdr` (e.g, the `volk_gnsssdr` library and googletest). Build types
  available: `None`, `Release` (by default), `Debug`, `RelWithDebInfo`,
  `MinSizeRel`, `Coverage`, `NoOptWithASM`, `O2WithASM`, `O3WithASM`, `ASAN`.
- Fix runtime errors when compiling in `Debug` mode on macOS.
- Updated links in comments through the source code and in CMake scripts.
- Update GSL implementation to 0.36.0. See
  https://github.com/gsl-lite/gsl-lite/releases/tag/v0.36.0
- Create a CI job on GitHub to ensure that `clang-tidy` has been applied in most
  of the source code (some optional blocks and tests are left apart).
- Create a CI job on GitHub to ensure that `clang-format` has been applied.
- Create a CI job on GitHub to ensure that `cpplint` filters have been applied.
- Create a CI job on GitHub to ensure compliance with REUSE Specification (see
  https://reuse.software)
- Create a CI job on GitHub using `prettier` (https://prettier.io/) to check
  markdown files formatting.
- Create a CI job on GitHub to check the formatting of CMake scripts using
  `cmakelint` (see https://github.com/richq/cmake-lint).

### Improvements in Openness:

- Make software compliant with REUSE Specification – Version 3.0 (see
  https://reuse.software/spec/).

### Improvements in Portability:

- The CMake scripts now find dependencies in Debian's riscv64 architecture.
- Enable AVX2 kernels of the `volk_gnsssdr` library when using the Clang
  compiler.
- Fixed building in some ARM-based devices. Now Clang and ARMClang can be used
  for native building.
- Added toolchain files for building gnss-sdr and the `volk_gnsssdr` library in
  several ARM processor architectures, including those in Raspberry Pi 3 and 4.
- The software can now be built using Xcode (passing `-GXcode` to CMake) without
  previous manual installation of `volk_gnsssdr`.
- The software can now be built using Xcode (passing `-GXcode` to CMake) without
  gflags, glog, matio, PugiXML, Protocol Buffers or googletest previously
  installed.
- Now the `volk_gnsssdr` library can be built on Microsoft Windows.
- Now the `volk_gnsssdr` library makes use of C11 `aligned_alloc` where
  available.
- Improved CMake script for cross-compilation and for the detection of AVX, AVX2
  and NEON (v7 and v8) instructions.
- Fixed warnings raised by CMake 3.17.
- Fixed warnings raised by `cmake --warn-uninitialized ..`
- Fixed the receiver's availability in systems in which the Sys V message queue
  mechanism is not available.

### Improvements in Reliability:

- Decoding of navigation messages no longer rely on implementation defined
  behavior for shifting left a signed integer.
- Removed usage of functions with insecure API (e.g., `strcpy`, `sprintf`).
- New type alias `volk_gnsssdr::vector` allows both aligned memory allocation
  and automatic deallocation.
- Fixed a memory leak in the generation of Galileo PRN codes.
- Added clang-tidy checks `clang-analyzer-security.*`,
  `clang-analyzer-optin.portability.UnixAPI` clang-tidy checks. Fixed raised
  warnings.
- Fixed `cpplint.py` `runtime/printf` and `runtime/explicit` errors.
- All constructors callable with one argument are marked with the keyword
  explicit. See MISRA C++:2008, 12-1-3 - All constructors that are callable with
  a single argument of fundamental type shall be declared explicit.

### Improvements in Repeatability:

- Added the option to apply carrier smoothing of code pseudoranges with new
  Observables parameter `Observables.enable_carrier_smoothing`.
- Fixed normalization of DLL discriminator in BPSK modulations when the spacing
  between correlators was not 0.5.

### Improvements in Testability:

- Add receiver runtime to `position_test` report.
- Improvements in FPGA unit tests.
- Add new utility tool `obsdiff` to perform single and double differences
  computations from observation RINEX files. Requires GPSTk and Armadillo >=
  9.800.

### Improvements in Usability:

- A new global parameter `GNSS-SDR.pre_2009_file` allows to process raw sample
  files containing GPS L1 C/A signals dated before July 14, 2009.
- Improved DLL-PLL binary dump MATLAB/Octave plot script. Old versions removed.
- Simplified RTKLIB error log.
- Added a Python 3 plotting script to show relative performance of generic
  `volk_gnsssdr` kernels wrt. SIMD fastest versions.
- Added reporting of velocity in the terminal.
- Added reporting of user clock drift estimation, in ppm, in the `Pvt_Monitor`
  and in internal logging (`Debug` mode).
- Updated documentation generated by Doxygen, now the `pdfmanual` target works
  when using ninja.
- CMake now generates an improved summary of required/optional dependency
  packages found and enabled/disabled features, including the building system
  and GNSS-SDR, CMake and compiler versions. This info is also stored in a file
  called `features.log` in the building directory.
- Markdown files have been wrapped to 80 characters line length to make it
  easier to read them from the terminal.

See the definitions of concepts and metrics at
https://gnss-sdr.org/design-forces/

&nbsp;

## [GNSS-SDR v0.0.11](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.11) - 2019-08-04

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3359989.svg)](https://doi.org/10.5281/zenodo.3359989)

This release has several improvements in different dimensions, addition of new
features and bug fixes:

### Improvements in Accuracy:

- Local clock correction based on PVT solution, allowing the delivery of
  continuous observables.
- Fix a bug in broadcast ionospheric parameters usage.

### Improvements in Availability:

- Improved mechanism for false lock detection in the Tracking loops.
- Fixed bug in Galileo INAV/FNAV message decoding when PLL is locked at 180
  degrees, which prevented from correct navigation message decoding in some
  situations.
- Fixed bug that caused a random deadlock in the Observables block, preventing
  the computation of PVT fixes.
- Fixed PVT computation continuity through the TOW rollover.
- Improved signal acquisition and tracking mechanisms in high dynamic scenarios.

### Improvements in Efficiency:

- Added mechanism for assisted acquisition of signals on a secondary band when
  the primary has already been acquired. This allows a great reduction of the
  computational load in multi-frequency configurations.
- Tracking loops now perform bit synchronization, simplifying the decoding
  process in Telemetry blocks and FPGA-offloading.
- Improved preamble detection implementation in the decoding of navigation
  messages (acceleration by x1.6 on average per channel).
- Shortened Acquisition to Tracking transition time.
- Applied clang-tidy checks and fixes related to performance:
  `performance-faster-string-find`, `performance-for-range-copy`,
  `performance-implicit-conversion-in-loop`,
  `performance-inefficient-algorithm`,
  `performance-inefficient-string-concatenation`,
  `performance-inefficient-vector-operation`, `performance-move-const-arg`,
  `performance-move-constructor-init`, `performance-noexcept-move-constructor`,
  `performance-type-promotion-in-math-fn`,
  `performance-unnecessary-copy-initialization`,
  `performance-unnecessary-value-param`, `readability-string-compare`.

### Improvements in Flexibility:

- Rewritten Control Thread and GNSS flow graph for increased control of
  channels' status and smarter assignation of satellites in multi-band
  configurations.
- New Tracking parameters allow the configuration of PLL and DLL filters order.
- Added parameter to enable FLL during pull-in time.
- Configurable pull-in time in the Tracking loops.

### Improvements in Interoperability:

- Added the BeiDou B1I and B3I receiver chains.
- Fix bug in GLONASS dual frequency receiver.
- Added a custom UDP/IP output for PVT data streaming.
- Improved Monitor block with UDP/IP output for internal receiver's data
  streaming.
- Custom output formats described with `.proto` files, making easier to other
  applications reading them in a forward and backward-compatible fashion upon
  future format changes. New dependency: Protocol Buffers >= 3.0.0
- Fixes in RINEX generation: week rollover, annotations are not repeated anymore
  in navigation files. Parameter `PVT.rinexnav_rate_ms` has been removed,
  annotations are made as new ephemeris arrive.
- Fixes in RTCM messages generation: week rollover.

### Improvements in Maintainability:

- The internal communication mechanism based on `gr::msg_queue` has been
  replaced by a thread-safe, built-in asynchronous message passing system based
  on GNU Radio's Polymorphic Types. This change is backwards-compatible and
  prevents from a failure in case of a possible future deprecation or removal of
  the `gr::msg_queue` API.
- Deprecated `boost::asio::io_service` replaced by `boost::asio::io_context` if
  Boost > 1.65
- CMake turns all policies to ON according to the running version up to version
  3.15.
- Usage of clang-tidy integrated into CMake scripts. New option
  `-DENABLE_CLANG_TIDY=ON` executes clang-tidy along with compilation. Requires
  clang compiler.
- Applied clang-tidy checks and fixes related to readability:
  `readability-container-size-empty`, `readability-identifier-naming`,
  `readability-inconsistent-declaration-parameter-name`,
  `readability-named-parameter`, `readability-non-const-parameter`,
  `readability-string-compare`.
- Improved includes selection following suggestions by `include-what-you-use`
  (see https://include-what-you-use.org/), allowing faster compiles, fewer
  recompiles and making refactoring easier.
- Massive reduction of warnings triggered by clang-tidy checks.
- Throughout code cleaning and formatting performed with automated tools in
  order to reduce future commit noise.

### Improvements in Portability:

- Added interfaces for FPGA off-loading in GPS L1 C/A, Galileo E1b/c, GPS L2C,
  GPS L5 and Galileo E5a receiver chains.
- CMake scripts now follow a modern approach (targets and properties) but still
  work with 2.8.12.
- Improvements for macOS users using Homebrew.
- The software builds against GNU Radio >= 3.7.3, including 3.8.0. Automatically
  detected, no user intervention is required.
- The `volk_gnsssdr` library can now be built without requiring Boost if the
  compiler supports C++17 or higher.
- The Boost Filesystem library is not anymore a required dependency in cases
  where it can be replaced by `std::filesystem`. Automatically detected, no user
  intervention is required.
- CMake scripts automatically select among C++11, C++14, C++17, or C++20
  standards, the most recent as possible, depending on compiler and dependencies
  versions.
- Drawback in portability: Protocol Buffers >= 3.0.0 is a new required
  dependency.

### Improvements in Reliability:

- Included the Guidelines Support Library. General improvement of memory
  management, replacement of raw pointers by containers or smart pointers.
- Applied clang-tidy checks and fixes related to High Integrity C++:
  `performance-move-const-arg`, `modernize-use-auto`,
  `modernize-use-equals-default`, `modernize-use-equals-delete`,
  `modernize-use-noexcept`, `modernize-use-nullptr`, `cert-dcl21-cpp`,
  `misc-new-delete-overloads`, `cert-dcl58-cpp`, `cert-err52-cpp`,
  `cert-err60-cpp`, `hicpp-exception-baseclass`, `hicpp-explicit-conversions`.
- Fixed a number of defects detected by Coverity Scan (version June 2019).

### Improvements in Usability:

- The receiver now admits FPGA off-loading, allowing for real time operation in
  embedded systems at high sampling rates and high number of signals and
  channels per signal in multiple bands.
- Fixed program termination (avoiding hangs and segfaults in some
  platforms/configurations).
- The `Labsat_Signal_Source` now terminates the receiver's execution when the
  end of file(s) is reached. It now accepts LabSat 2 filenames and series of
  LabSat 3 files.
- Added configuration parameters to set the annotation rate in KML, GPX, GeoJSON
  and NMEA outputs, set by default to 1 s.
- New parameter `PVT.show_local_time_zone` displays time in the local time zone.
  Subject to the proper system configuration of the machine running the software
  receiver. This feature is not available in old compilers.
- CMake now generates a summary of required/optional dependency packages found
  and enabled/disabled features. This info is also stored in a file called
  features.log in the building directory.
- Improved information provided to the user in case of building configuration
  and runtime failures.
- Remove abandoned building option `-DENABLE_GN3S` and `Gn3s_Signal_Source`
  implementation.

See the definitions of concepts and metrics at
https://gnss-sdr.org/design-forces/

&nbsp;

## [GNSS-SDR v0.0.10](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.10) - 2018-12-14

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.2279988.svg)](https://doi.org/10.5281/zenodo.2279988)

This release has several improvements in different dimensions, addition of new
features and bug fixes:

### Improvements in Accuracy:

- Part of the RTKLIB core library has been integrated into GNSS-SDR. There is
  now a single PVT block implementation which makes use of RTKLIB to deliver PVT
  solutions, including Single and PPP navigation modes.
- Fixed CN0 estimation for other correlation times than 1 ms.
- Improved computation of tracking parameters and GNSS observables.
- Other minor bug fixes.

### Improvements in Availability:

- Internal Finite State Machines rewritten for improved continuity in delivering
  position fixes. This fixes a bug that was stalling the receiver after about
  six hours of continuous operation.
- Redesign of the time counter for enhanced continuity.
- Improved flow graph in multi-system configurations: the receiver does not get
  stalled anymore if no signal is found from the first system.
- Improved acquisition and tracking sensitivity.
- Added mechanisms for Assisted GNSS, thus shortening the Time-To-First-Fix.
  Provision of data via XML files or via SUPL v1.0. Documented at
  https://gnss-sdr.org/docs/sp-blocks/global-parameters/
- Other minor bug fixes.

### Improvements in Efficiency:

- Added the possibility of non-blocking acquisition, which works well when using
  real-time data from an RF front-end.
- Improved flow graph in multi-band configurations: satellites acquired in one
  band are immediately searched in others.
- Complex local codes have been replaced by real codes, alleviating the
  computational burden.
- New `volk_gnsssdr` kernels: `volk_gnsssdr_16i_xn_resampler_16i_xn.h`,
  `volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn.h`,
  `volk_gnsssdr_32f_xn_resampler_32f_xn.h`, and
  `volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn.h`.
- Some AVX2 implementations added to the `volk_gnsssdr` library.
- Improvement in C++ usage: Use of const container calls when result is
  immediately converted to a const iterator. Using these members removes an
  implicit conversion from iterator to const_iterator.
- Output printers can be shut down, with some savings in memory and storage
  requirements.
- A number of code optimizations here and there.

### Improvements in Flexibility:

- A number of new parameters have been exposed to the configuration system.
- Possibility to choose Pilot or Data component for tracking of GPS L5 and
  Galileo E5a signals.
- Enabled extended coherent integration times for signal tracking.
- Configurable coherent and/or non-coherent signal acquisition.
- Some configuration parameters can now be overridden by commandline flags for
  easier use in scripts.

### Improvements in Interoperability:

- Added the GPS L5 receiver chain.
- Added the GLONASS L1 SP receiver chain.
- Added the GLONASS L2 SP receiver chain.
- Improvements in the Galileo E5a and GPS L2C receiver chains.
- Updated list of available GNSS satellites.
- Added five more signal sources: `Fmcomms2_Signal_Source` (requires gr-iio),
  `Plutosdr_Signal Source` (requires gr-iio), `Spir_GSS6450_File_Signal_Source`,
  `Labsat_Signal_Source` and `Custom_UDP_Signal_Source` (requires libpcap).
  Documented in https://gnss-sdr.org/docs/sp-blocks/signal-source/
- Improved support for BladeRF, HackRF and RTL-SDR front-ends.
- Added tools for the interaction with front-ends based on the AD9361 chipset.
- Intermediate results are now saved in MAT-file format (`.mat`), readable from
  Matlab/Octave and from Python via h5py.
- Added the GPX output format.
- Improvements in the generation of KML files.
- Improvements in the NMEA output. The receiver can produce GPGGA, GPRMC, GPGSA,
  GPGSV, GAGSA and GAGSV sentences.
- Improvements in the RTCM server stability.
- Improvements in the correctness of generated RINEX files.
- The receiver can read and make use of Galileo almanac XML files published by
  the European GNSS Service Centre at
  https://www.gsc-europa.eu/gsc-products/almanac
- Own-defined XML schemas for navigation data published at
  https://github.com/gnss-sdr/gnss-sdr/tree/next/docs/xml-schemas
- Added program `rinex2assist` to convert RINEX navigation files into XML files
  usable for Assisted GNSS. Only available building from source. See
  https://github.com/gnss-sdr/gnss-sdr/tree/next/src/utils/rinex2assist

### Improvements in Maintainability:

- Setup of a Continuous Integration system that checks building and runs QA code
  in a wide range of GNU/Linux distributions (Arch Linux, CentOS, Debian,
  Fedora, OpenSUSE, Ubuntu) and releases. See
  https://gitlab.com/gnss-sdr/gnss-sdr
- Creation of multi-system processing blocks, drastically reducing code
  duplication and maintainability time.
- Automated code formatting with clang-format. This tool is widely available and
  easy to integrate into many code editors, and it also can be used from the
  command line. It cuts time spent on adhering to the project's code formatting
  style.
- Improvement in C++ usage: C-style casts have been replaced by C++ casts.
  C-style casts are difficult to search for. C++ casts provide compile time
  checking ability and express programmers' intent better, so they are safer and
  clearer.
- Improvement in C++ usage: The override special identifier is now used when
  overriding a virtual function. This helps the compiler to check for type
  changes in the base class, making the detection of errors easier.
- Improvement in C++ usage: A number of unused includes have been removed. Order
  of includes set to: local (in-source) headers, then library headers, then
  system headers. This helps to detect missing includes.
- Improvement in C++ usage: Enhanced const correctness. Misuses of those
  variables are detected by the compiler.
- Improved code with clang-tidy and generation of a `compile_commands.json` file
  containing the exact compiler calls for all translation units of the project
  in machine-readable form if clang-tidy is detected.
- Applied some style rules to CMake scripts.
- Minimal versions of dependencies identified and detected.

### Improvements in Portability:

- Several CMake scripts improvements, more verbose outputs in case of errors.
  Building configuration has been documented in
  https://gnss-sdr.org/docs/tutorials/configuration-options-building-time/
- Improved SDK for cross-compilation in embedded devices. Documented in
  https://gnss-sdr.org/docs/tutorials/cross-compiling/
- Improved control over minimum required versions for core dependencies.
- The software builds with C++11, C++14 and C++17 standards.
- The software can now be built using GCC >= 4.7.2 or LLVM/Clang >= 3.4.0
  compilers on GNU/Linux, and with Clang/AppleClang on macOS.
- The Ninja build system can be used in replacement of make.
- The `volk_gnsssdr` library can be built using Python 2.7+ or Python 3.6+.
- The `volk_gnsssdr` library is now ready for AArch64 NEON instructions.
- Improved detection of required and optional dependencies in many GNU/Linux
  distributions and processor architectures.
- Improvement in C++ usage: The `<ctime>` library has been replaced by the more
  modern and portable `<chrono>` (except for the interaction with RTKLIB).
- Improvement in C++ usage: The `<stdio.h>` library has been replaced by the
  more modern and portable `<fstream>` for file handling.
- Improvement in C++ usage: C++ libraries preferred over C libraries (e.g.,
  `<cctype>` instead of `<ctype.h>`, `<cmath>` instead of `<math.h>`).
- Fix compatibility with Boost 1.67 (closes Debian bug #911882
  https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=911882)
- Fixes required by Debian packaging.
- Fixes required by Macports packaging.
- A downside in portability: BLAS and LAPACK libraries are now required even in
  ARM devices.
- A downside in portability: the matio library >= 1.5.3 is a new required
  dependency. If not found, it is downloaded and built automatically at building
  time, but this requires libtool, automake and hdf5 already installed in the
  system.
- A downside in portability: the PugiXML library is a new required dependency.
  If not found, it is downloaded and built automatically at building time.

### Improvements in Reliability:

- Introduced 3 new Input Filter implementations for pulsed and narrowband
  interference mitigation: `Pulse_Blanking_Filter`, `Notch_Filter` and
  `Notch_Filter_Lite`. Documented in
  https://gnss-sdr.org/docs/sp-blocks/input-filter/
- Improved flow graph stability.
- Introduction of high-integrity C++ practices into the source code and included
  in the coding style guide. See https://gnss-sdr.org/coding-style/
- Fixed a number of defects detected by Coverity Scan.
- Improvement of QA code and addition of a number of new tests. Documented at
  https://gnss-sdr.org/docs/tutorials/testing-software-receiver-2/
- Improvement in C++ usage: `rand()` function replaced by `<random>` library.
- Improvement in C++ usage: `strlen` and `strncpy` have been replaced by safer
  C++ counterparts.
- Improvement in C++ usage: Some destructors have been fixed, avoiding
  segmentation faults when exiting the program.
- Website switched from http to https. Links in the source tree switched when
  available.

### Improvements in Reproducibility:

- Setup of a Continuous Reproducibility system at GitLab for the automatic
  reproduction of experiments. The concept was introduced in
  https://ieeexplore.ieee.org/document/8331069/ Example added in the
  `src/utils/reproducibility/ieee-access18/` folder.
- Fixes of Lintian warnings related to build reproducibility.

### Improvements in Scalability:

- Improvements in multi-system, multi-band receiver configurations. The receiver
  now accepts any number of channels and systems in the three available bands.
- All possible combinations of signals and integration times are now accepted by
  the Observables block.

### Improvements in Testability:

- Several Unit Tests added. Documentation of testing concepts and available
  tests at https://gnss-sdr.org/docs/tutorials/testing-software-receiver/
- New extra unit test `AcquisitionPerformanceTest` checks the performance of
  Acquisition blocks.
- New extra unit test `TrackingPullInTest` checks acquisition to tracking
  transition.
- New extra unit test `HybridObservablesTest` checks the generation of
  observables.
- Improved system testing: position_test accepts a wide list of parameters and
  can be used with external files.
- Receiver channels can now be fixed to a given satellite.
- Testing integrated in a Continuous Reproducibility system (see above).
- Improved CTest support in `volk_gnsssdr`.

### Improvements in Usability:

- All Observables block implementations have been merged into a single
  implementation for all kinds of GNSS signals, making it easier to configure.
- All PVT block implementations have been merged into a single implementation
  for all kinds of GNSS signals, making it easier to configure.
- Misleading parameter name `GNSS-SDR.internal_fs_hz` has been replaced by
  `GNSS-SDR.internal_fs_sps`. The old parameter name is still read. If found, a
  warning is provided to the user. The old name will be removed in future
  releases.
- Updated and improved online documentation of processing blocks at
  https://gnss-sdr.org/docs/sp-blocks/
- Improved documentation of required dependency packages in several GNU/Linux
  distributions.
- Dump and output files can now be stored anywhere.
- Parameter names with the same role have been harmonized within different block
  implementations.
- Added a changelog, a code of conduct, a contributing guide and a pull-request
  template in the source tree.
- Added colors to the commandline user interface.
- Updated manfiles.
- Updated examples of configuration files under the `conf/` folder.

See the definitions of concepts and metrics at
https://gnss-sdr.org/design-forces/

&nbsp;

## [GNSS-SDR v0.0.9](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.9) - 2017-02-13

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.291371.svg)](https://doi.org/10.5281/zenodo.291371)

This release has several improvements, addition of new features and bug fixes in
many dimensions:

### Improvements in Accuracy:

- Major rewriting in the generation of pseudoranges.
- Fixed bug in Galileo E5a/I codes.
- Fixed bug in Galileo E1 correlator spacing.
- Fixed bug that was causing errors in receivers above the troposphere.
- Fixed 16-bit complex resampler.
- Improved time tracking algorithm.
- Added Bancroft's algorithm implementation for PVT initialization.

### Improvements in Availability:

- Improved numerical stability of the PVT solution. The infamous bug that was
  causing apparently random error peaks has finally been fixed.

### Improvements in Efficiency:

- VOLK_GNSSSDR: Added NEON,AVX and unaligned protokernels for
  `volk_gnsssdr_32f_index_max_32` kernel.
- VOLK_GNSSSDR: Added `volk_gnsssdr-config-info` to the list of generated
  executables.

### Improvements in Flexibility:

- Added maximum number of dwells in the Tong algorithm.

### Improvements in Interoperability:

- Added six new Galileo satellites: FM7, FM10, FM11, FM12, FM13, FM14.
- The Hybrid_Observables and Hybrid_PVT implementations can now handle more
  types of GNSS signals.
- The RINEX printer can now print L2C and E5a observables and navigation files,
  including multiband configurations.
- Added RTCM 3.2 output to more receiver configurations.

### Improvements in Maintainability:

- The VOLK_GNSSSDR library can now be built with Python 3. Switched dependencies
  for VOLK_GNSSDR: from (old, python2.7-only) python-cheetah templates to
  Python3 friendly python-mako and python-six. So, Python-cheetah dependency has
  been dropped, and python-mako and python-six have been added.
- If suitable versions of gflags, glog, armadillo or googletest are not found in
  the system, they will be downloaded and built at compile time (versions 2.2.0,
  0.3.4, 7.600.2 and 1.8.0, respectively).
- Fixed more than 30 defects detected by Coverity Scan.
- Added CMake Python finder and module checker.
- Deleted files related to CPack.
- Fixes, updates and improvements in the documentation.
- Improvements in CMake scripts: General code cleaning and addition of comments.
  Improved user information in case of failure. Improved detection of
  dependencies in more processor architectures (e.g. aarch64).

### Improvements in Marketability:

- Reduced time from a commit to deployment (see virtualization mechanisms in
  Portability).

### Improvements in Portability:

- Now GNSS-SDR can be run in virtual environments through snap packages (see
  https://github.com/carlesfernandez/snapcraft-sandbox) and docker images (see
  https://github.com/carlesfernandez/docker-gnsssdr).
- Now GNSS-SDR is adapted to cross-compiling environments for embedded devices
  (see https://github.com/carlesfernandez/oe-gnss-sdr-manifest).
- BLAS and LAPACK libraries are no longer mandatory on ARM devices.

### Improvements in Scalability:

- Fixed bug in acquisition with rata rates higher than 16 Msps in 4ms code
  periods.

### Improvements in Testability:

- Major QA source code refactoring: they has been split into
  `src/tests/unit-tests` and `src/tests/system-tests` folders. They are
  optionally built with the `ENABLE_UNIT_TESTING=ON` (unit testing QA code),
  `ENABLE_UNIT_TESTING_EXTRA=ON` (unit tests that require extra files downloaded
  at configure time), `ENABLE_SYSTEM_TESTING=ON` (system tests, such as
  measurement of Time-To-First-Fix) and `ENABLE_SYSTEM_TESTING_EXTRA=ON` (extra
  system test requiring external tools, automatically downloaded and built at
  building time) configuration flags. The EXTRA options also download and build
  a custom software-defined signal generator and version 2.9 of GPSTk, if not
  already found on the system. Download and local link of version 2.9 can be
  forced by `ENABLE_OWN_GPSTK=ON` building configuration flag. Only
  `ENABLE_UNIT_TESTING` is set to ON by default.
- Unit tests added: `CPU_multicorrelator_test` and `GPU_multicorrelator_test`
  measure computer performance in multicorrelator setups.
- Unit tests added: `GpsL1CADllPllTracking` and `GpsL1CATelemetryDecoderTest`.
- System test added: `ttff_gps_l1` performs a set of cold / assisted runs of the
  software receiver and computes statistics about the obtained Time To First
  Fix.
- System test added: `obs_gps_l1_system_test` uses an external software-defined
  signal generator to produce raw digital GNSS signal from a RINEX navigation
  file and a position (static or dynamic), processes it with GNSS-SDR, and then
  compares the RINEX observation file produced by the software receiver to that
  produced by the signal generator.
- Software Development Kit provided for embedded devices (see
  https://gnss-sdr.org/docs/tutorials/cross-compiling/).

### Improvements in Usability:

- Now the block factory automatically detects Channel input data type, so it is
  no longer required to specify Channel.input_type in the configuration. An
  error raises if Acquisition and Tracking Blocks are not configured with the
  same input data type.
- Block names changed from L2_M to L2C.
- Documentation available at https://gnss-sdr.org/docs/
- Improved tools for compilation, execution and testing in embedded devices.

See the definitions of concepts and metrics at
https://gnss-sdr.org/design-forces/

&nbsp;

## [GNSS-SDR v0.0.8](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.8) - 2016-07-04

[![DOI](https://zenodo.org/badge/doi/10.5281/zenodo.57022.svg)](http://dx.doi.org/10.5281/zenodo.57022)

This is a maintenance and bug fix release with no relevant new features with
respect to v0.0.7. The main changes are:

- Fixed a bug that broke building when using latest VOLK release
- Updated PYBOMBS instructions
- Added Tests for FFT length
- Added Tests for CUDA-based tracking
- Added Tests for SIMD-based tracking
- Improved CUDA-based correlation.
- Updated documentation
- Fixed building in mips and powerpc architectures.
- `gr-gn3s` and `gr-dbfcttc` moved to its own repository.
- Improved package reproducibility
- VOLK_GNSSSDR: Fixed a bug in AVX2 puppet
- VOLK_GNSSSDR: can now be built using the C98 standard
- VOLK_GNSSSDR: Fixed a bug that broke building when linking to Boost in some
  configurations.
- VOLK_GNSSSDR: Added an option to trigger profiling at building time.
- VOLK_GNSSSDR: Fix the CMake-based check for posix_memalign.

&nbsp;

## [GNSS-SDR v0.0.7](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.7) - 2016-05-15

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.51521.svg)](https://doi.org/10.5281/zenodo.51521)

This release has several improvements, addition of new features and bug fixes:

- Improvements in receiver design: Internal block communication has been
  redesigned to accommodate the addition of new signals, and now upstream and
  downstream communication within blocks is implemented through the GNU Radio
  block’s asynchronous message passing system, leading to a more scalable, more
  robust and cleaner design.
- Improvements in receiver design: Correlators have been rewritten to take full
  advantage of VOLK and VOLK_GNSSSDR, and they are of general use for any
  tracking block. Their API now admit an arbitrary number of correlators, spaced
  in an arbitrary manner, in 16ic and 32fc versions.
- Improvements in receiver design: Block adapters are now all managed by smart
  pointers, ensuring better memory management.
- Improvements in processing speed: The VOLK_GNSSSDR library has been rewritten,
  following current VOLK standards and adding a number of new kernels. This
  approach addresses both efficiency and portability. Now the library provides
  the key kernels for GNSS signal processing in 16ic and 32fc versions,
  including SSE2, SSE3, SSE4.1, AVX, AV2 and NEON implementations. Please
  execute `volk_gnsssdr_profile` and `volk_profile` to use the fastest
  implementation for your host machine.
- New source block: `Two_Bit_Packed_File_Signal_Source`. This block takes 2 bit
  samples that have been packed into bytes or shorts as input and generates a
  byte for each sample.
- Fixes in SUPL assistance (supl.nokia.com removed).
- Improvements in acquisition: Added a non CFAR PCPS acquisition algorithm based
  on the estimation of the post correlation noise floor. If enabled as an option
  in the acquisition configuration, it allows setting more stable thresholds in
  the presence of non-gaussian front-end noise (which is the usual behavior of
  front-ends.)
- Fixes in acquisition: Fixed mismatch between the config files and the
  acquisition code in the specification of the IF. Fixed a bug in the length of
  the FFT of local codes.
- Improvements in tracking sensitivity: Added configuration option to customize
  the extension of the GPS L1 CA correlation length after bit synchronization
  (options are: [1,2,4,5,10,20] ms). Only available in the
  `GPS_L1_CA_DLL_PLL_C_Aid_Tracking` implementation.
- New tracking block introduced: `GPS_L1_CA_DLL_PLL_C_Aid_Tracking` is a GPS L1
  C/A carrier PLL and code DLL with optional carrier-aid feedback. It is
  available in both 32 bits `gr_complex` input samples and in 16 bits short int
  complex samples. The `gr_complex` version has also the capability to extend
  the coherent correlation period from 1 ms to 20 ms using telemetry symbol
  synchronization.
- Increased resolution in CN0 estimator internal variables.
- Fixed a bug in computation of GPS L1 C/A carrier phase observable.
- Fixed a bug in the internal state machine that was blocking the receiver after
  a few hours of usage. Now the receiver can work continually (tested for more
  than one week, no known limit).
- New tracking block introduced: `GPS_L1_CA_DLL_PLL_Tracking_GPU` is a GPS L1
  C/A carrier PLL and code DLL that uses the CUDA-compatible GPU to compute
  carrier wipe off and correlation operations, alleviating the CPU load.
- Obsolete/buggy blocks removed: `GPS_L1_CA_DLL_FLL_PLL_Tracking`,
  `GPS_L1_CA_DLL_PLL_Optim_Tracking`.
- Added a RTCM printer and TCP server in PVT blocks (still experimental). The
  receiver is now able to stream data in real time, serving RTCM 3.2 messages to
  multiple clients. For instance, it can act as a Ntrip Source feeding a Ntrip
  Server, or to be used as data input in RTKLIB, obtaining Precise Point
  Positioning fixes in real-time. The TCP port, Station ID, and rate of
  MT1019/MT1045 and MSM can be configured. `GPS_L1_CA_PVT` serves MT1019 (GPS
  Ephemeris) and MSM7 (MT1077, full GPS pseudoranges, phase ranges, phase range
  rates and CNR - high resolution) messages, while `GALILEO_E1_PVT` serves
  MT1045 (Galileo ephemeris) and MSM7 (MT1097, full Galileo pseudoranges, phase
  ranges, phase range rates and CNR - high resolution).
- Added a GeoJSON printer. Basic (least-squares) position fixes can be now also
  stored in this format, in addition to KML.
- Obsolete block removed: output filter.
- QA code migrated to the new asynchronous message passing system.
- Improvements in documentation: update of `README.md` file, addition of
  documentation for the VOLK_GNSSSDR library, updated links to new ICDs.
- Improvements in documentation: Satellite identification updated to current
  constellation status.
- Updated and cleaner console output. Now Galileo satellites have the ‘E’
  identifier in their PRN number.
- Several improvements in CMake scripts allow to build GNSS-SDR in Linux Debian
  (Jessie, Stretch and Sid), Ubuntu (from 12.04 to 16.04), including amd64,
  i386, armhf and arm64 architectures, and possibly in other GNU/Linux
  distributions, as well as in Mac OS X 10.9 to 10.11. It also works well with
  CMake 3.5 (some problems solved with VOLK_GNSSSDR as a sub-project).
- The software can link either against OpenSSL or against GnuTLS with openssl
  extensions, whatever it is available. This allows buildings in distributions
  such as Fedora or ArchLinux, while being compatible with binary distribution
  through Debian packages.
- Fixed a number of defects detected by Coverity Scan.
- Some fixes required by Debian licensing and packaging system.
- Added a CGRAN (https://www.cgran.org/) manifest
- Lots of code cleaning and fixes of typos and small bugs.

&nbsp;

## [GNSS-SDR v0.0.6](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.6) - 2015-09-02

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.30104.svg)](https://doi.org/10.5281/zenodo.30104)

This release has several improvements and bug fixes:

- Added initial support to multi-band, multi-source configurations (multiple
  signal sources and signal conditioners).
- Updated configuration files to new notation. Old and new configuration
  notations still compatible.
- Added skeleton for mixed (multi-frequency and multi-system) observables block.
- Faster local carrier update (25% of improvement).
- Added initial support to GPS L2C real time tracking and decoding of CNAV
  message with NSL STEREO v2, Fraunhofer’s Flexiband, and USRPx front-ends (the
  latter requiring external clock).
- Added initial support to select the frontend clock reference source in UHD
  signal source (i.e. internal or external clock reference).
- Added 2 bits complex file source for GNSS-SDR GSoC 2015 signal sampler
  designed by Ajith Peter.
- Added a new rtl_tcp signal source, remote access to RTL2832U-based dongles via
  TCP.
- Always build front-end-cal, a calibration tool for some DVB-T receivers based
  on the Realtek's RTL2832U chipset.
- Fixed bug in UTC time computation for GPS signals.
- Updated satellite identification for GPS and Galileo.
- Defined `cbyte` as a new input data type (`std::complex<signed char>`).
- Adding a new data_type_adapter, from interleaved short to
  `std::complex<short>`.
- Adding a filter for complex short streams.
- Adding a fir_filter for `std::complex<signed char>` (aka `cbyte`). It converts
  the data type to floats, filters, and converts back to cbyte.
- Added a resampler for `cbyte`s and `cshort`s.
- First working version of a GPS tracking block implementation using CUDA with
  multi-GPU device support.
- Updating RINEX obs header when leap second is available.
- Updating RINEX nav file when IONO and UTC data are available.
- Include Signal Strength Indicator in RINEX observable files.
- Tests fixed.
- Fixed more than 200 code defects detected by Coverity Scan.
- Updated documentation.
- Updated documentation and CMake scripts for the GN3S v2 driver (Linux-only)
- Armadillo version automatically downloaded and built if it is not present in
  the system is now 5.400.3.
- Updated old links from googlecode to new links at GitHub for Google Test,
  gflags, glog and gperftools.
- gfortran is no longer a required package, but it is used if available.
- Added an option to remove logging.
- Enabled cross-compilation for ARM devices.
- Lots of code cleaning.

&nbsp;

## [GNSS-SDR v0.0.5](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.5) - 2015-01-13

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.13920.svg)](https://doi.org/10.5281/zenodo.13920)

This release has several improvements and bug fixes:

- Now GNSS-SDR can be installed on the system with the usual
  `cmake ../ && make && sudo make install`.
- Added `volk_gnsssdr` library, a volk-like library implementing some specific
  kernels and ensuring portable executables. It comes with a
  `volk_gnsssdr_profile` executable, in the fashion of `volk_profile`. Volk and
  `volk_gnsssdr` are compatible and can be mixed together. This is expected to
  enable faster execution of the software receiver in upcoming versions.
- The former `rtlsdr_signal_source` has been replaced by a more general
  `osmosdr_signal_source` compatible with all those front-ends accessible by the
  OsmoSDR driver (bladeRF, hackRF, etc.) in addition to RTL-based dongles.
- Added manpages when binaries `gnss-sdr`, `volk_gnsssdr_profile` and
  `front-end-cal` are installed.
- Now GNSS-SDR can be built on i386, amd64, armhf, armel and arm64
  architectures.
- Now GNSS-SDR builds on Ubuntu 14.04 and 14.10, Debian jessie/sid and Mac OS X
  10.9 and 10.10.
- Improved detection of dependencies, specially when installed as .deb packages.
- Added a `check` target with some minimal tests.
- Added support for interleaved I/Q byte-size sample files.
- Minor bug fixes, updated documentation and code cleaning.

&nbsp;

## [GNSS-SDR v0.0.4](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.4) - 2014-09-08

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.11628.svg)](https://doi.org/10.5281/zenodo.11628)

This release has several improvements and bug fixes:

- Added hybrid processing GPS L1 C/A and Galileo E1B, providing position fixes
  make use of observables for both constellations.
- Added implementations of the QuickSync algorithm for GPS L1 C/A and Galileo E1
  acquisition.
- Added processing blocks for Galileo E5a: Acquisition, Tracking,
  Telemetry_Decoder (experimental)
- New configuration files allow to configure GPS and Galileo channels in the
  same receiver.
- Added tropospheric corrections to GPS and Galileo PVT solution.
- Improved precision obtained by changing some variables from float to double.
- New building options: `ENABLE_GN3S`, `ENABLE_RTLSDR`, `ENABLE_ARRAY`, and
  `ENABLE_OPENCL`.
- Improved documentation on how to enable optional drivers.
- Fixed bug in memory alignment that caused problems with high data rates.
- Added `ENABLE_GENERIC_ARCH`, an option to build the binary without detecting
  the SIMD instruction set present in the compiling machine, so it can be
  executed in other machines without those specific sets.
- Added `ENABLE_GPERFTOOLS`, which links the executable to tcmalloc and profiler
  if Gperftools is available on the system.
- Added carrier phase, Doppler shift and signal strength observables to the
  RINEX files. Static PPP solutions are available for GPS with RTKLIB via RINEX
  files.
- The executable now produces RINEX files version 3.02 of Galileo Observables,
  Navigation data, and mixed (GPS/Galileo) observables and nav data. RINEX 3.02
  is the default version of RINEX files.
- Armadillo version updated to 4.400.2
- Armadillo now uses OpenBLAS instead of BLAS if the former is available on the
  system.
- Some raw pointers have been changed to smart pointers.
- Minor bug fixes and code cleaning.

&nbsp;

## [GNSS-SDR v0.0.3](https://github.com/gnss-sdr/gnss-sdr/releases/tag/v0.0.3) - 2014-06-30

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.10708.svg)](https://doi.org/10.5281/zenodo.10708)

This release has several improvements and bug fixes, completing the transition
from Subversion to Git. The main changes are:

- Created some missing directories lost in the SVN to Git transition.
- New C++11-ized block factory, flow graph and tests, resulting in better memory
  management and fewer segmentation faults. Several raw pointers converted to
  smart pointers.
- Reorganization of assistance data input and output.
- Fixed memory leak when talking to SUPL servers.
- Improved retrieval of assistance data.
- Fixing an error in a constant value related to Galileo.
- Inform users if the temporal folder is not /tmp.
- Fixes and additions to the documentation.
- README in markdown language so it looks better in Git repositories.
- Fixed a bug that prevented the update of all shared map structures (ephemeris,
  iono parameters, etc…).
- The configuration script now throws error if GCC is older than 4.7 or Boost is
  older than 1.45
- Improved detection / downloading & building if missing of Gflags and Glog.
- Improved detection / downloading & building if missing of Armadillo and
  related dependencies.
- Fixes many warnings that appeared when using CMake 3.0.
- Improved detection of GTEST_DIR variable.
- Include header files in libraries so IDEs such as Xcode can display them.

Enjoy it!
