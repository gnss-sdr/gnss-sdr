## obsdiff

<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2020 Carles Fernandez-Prades <carles.fernandez@cttc.es>
)
<!-- prettier-ignore-end -->

This program computes single-differences and double-differences from RINEX
observation files.

### Building

Requirements:

- [Armadillo](http://arma.sourceforge.net/): A C++ library for linear algebra
  and scientific computing. This program requires version 9.800 or higher. If
  your installed Armadillo version is older, see below.
- [Gflags](https://github.com/gflags/gflags): A C++ library that implements
  command-line flags processing. If not found in your system, the latest version
  will be downloaded, built and linked for you at building time.
- [GNSSTK](https://github.com/SGL-UT/gnsstk): The GNSSTk C++ Library, used for
  RINEX files reading. If not found in your system, the latest version will be
  downloaded, built and linked for you at building time.
- [Matio](https://github.com/tbeu/matio): A MATLAB MAT File I/O Library,
  version >= 1.5.3. If it is not found, or an older version is found, CMake will
  download, build and link a recent version for you at building time.

Optional:

- [Gnuplot](http://www.gnuplot.info/): a portable command-line driven graphing
  utility.

This program is built along with GNSS-SDR if the options
`ENABLE_UNIT_TESTING_EXTRA` or `ENABLE_SYSTEM_TESTING_EXTRA` are set to `ON`
when calling CMake:

```
$ cmake -DENABLE_SYSTEM_TESTING_EXTRA=ON ..
$ make obsdiff
$ sudo make install
```

The last step is optional. Without it, you still will get the executable at
`../install/obsdiff`.

This program requires Armadillo 9.800 or higher. If the available Armadillo
version is older, this program will not be built. If your local Armadillo
installed version is older than 9.800, you can force CMake to download, build
and link a recent one:

```
$ cmake -DENABLE_SYSTEM_TESTING_EXTRA=ON -DENABLE_OWN_ARMADILLO=ON ..
$ make obsdiff
$ sudo make install
```

This later option requires [BLAS](http://www.netlib.org/blas/),
[LAPACK](http://www.netlib.org/lapack/) and
[GFortran](https://gcc.gnu.org/fortran/) already installed in your system.

### Usage

Double differences (Pseudorange, Carrier Phase and Carrier Doppler) within Base
and Rover receivers:

```
$ obsdiff --base_rinex_obs=base.20o --rover_rinex_obs=rover.20o
```

Double differences with receiver clock correction (Pseudorange, Carrier Phase
and Carrier Doppler) within Base and Rover receivers:

```
$ obsdiff --base_rinex_obs=base.20o --rover_rinex_obs=rover.20o --rinex_nav=base.nav --remove_rx_clock_error=true
```

Single difference (Pseudorange, Carrier Phase and Carrier Doppler) with Base
receiver only and a special duplicated satellites simulated scenario:

```
$ obsdiff --rover_rinex_obs=rover.20o --single_diff=true --dupli_sat=true --dupli_sat_prns=1,2,3,4
```

Where the list of duplicated satellites PRN pairs must be specified by
`--dupli_sat_prns` flag (_i.e.,_ `1,2,3,4` indicates that the PRNs 1,2 share the
same orbit. The same applies for PRNs 3,4)

Single difference of Pseudorange Rate vs. Carrier Phase rate for each satellite:

```
$ obsdiff --rover_rinex_obs=rover.20o --single_diff=true
```

There is some flexibility in how command-line flags may be specified. The
following examples are equivalent:

```
$ obsdiff -base_rinex_obs=reference.20o
$ obsdiff --base_rinex_obs=reference.20o
$ obsdiff -base_rinex_obs reference.20o
$ obsdiff --base_rinex_obs reference.20o
```

For boolean flags, the possibilities are slightly different:

```
$ obsdiff --single_diffs
$ obsdiff --nosingle_diffs
$ obsdiff --single_diffs=true
$ obsdiff --single_diffs=false
```

(as well as the single-dash variant on all of these).

Despite this flexibility, we recommend using only a single form:
`--variable=value` for non-boolean flags, and `--variable/--novariable` for
boolean flags.

Available command-line flags:

<!-- prettier-ignore-start -->
| **Command-line flag**     | **Default value** | **Description**  |
|:-------------------------:|:-----------------:|:-----------------|
| `--skip_obs_transitory_s` | `30.0`            | Skip the initial observable outputs to avoid transitory results [s]. |
| `--skip_obs_ends_s`       | `5.0`             | Skip the lasts observable outputs to avoid transitory results [s]. |
| `--single_diffs`          | `false`           | [`true`, `false`]: If `true`, the program also computes the single difference errors for [Carrier Phase](https://gnss-sdr.org/docs/sp-blocks/observables/#carrier-phase-measurement) and [Doppler](https://gnss-sdr.org/docs/sp-blocks/observables/#doppler-shift-measurement) measurements (requires LO synchronization between receivers). |
| `--compare_with_5X`       | `false`           | [`true`, `false`]: If `true`, the program compares the E5a Doppler and Carrier Phases with the E5 full Bw in RINEX (expect discrepancy due to the center frequencies difference). |
| `--dupli_sat`             | `false`           | [`true`, `false`]: If `true`, this flag enables special observable test mode where the scenario contains duplicated satellite orbits. |
| `--dupli_sat_prns`        | `1,2,3,4`         | List of duplicated satellites PRN pairs (_i.e._, `1,2,3,4` indicates that the PRNs 1,2 share the same orbit. The same applies for PRNs 3,4). |
| `--base_rinex_obs`        | `base.obs`        | Filename of reference RINEX observation file. |
| `--rover_rinex_obs`       | `rover.obs`       | Filename of tested RINEX observation file. |
| `--remove_rx_clock_error` | `false`           | Compute and remove the receivers clock error prior to compute observable differences (requires a valid RINEX nav file for both receivers) |
| `--rinex_nav`             | `base.nav`        | Filename of reference RINEX navigation file. Only needed if `remove_rx_clock_error` is set to `true`. |
| `--system`                | `G`               | GNSS satellite system: `G` for GPS, `E` for Galileo. |
| `--signal`                | `1C`              | GNSS signal: `1C` for GPS L1 CA, `1B` for Galileo E1. |
| `--show_plots`            | `true`            | [`true`, `false`]: If `true`, and if [gnuplot](http://www.gnuplot.info/) is found on the system, displays results plots on screen. Please set it to `false` for non-interactive testing. |
<!-- prettier-ignore-end -->
