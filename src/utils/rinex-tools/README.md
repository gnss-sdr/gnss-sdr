obsdiff
-------

[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: Javier Arribas, 2020. <jarribas@cttc.es>
)

This program computes Single-difference and Double-difference from RINEX observation files.

## Building

This program is built along with GNSS-SDR if the options `ENABLE_UNIT_TESTING_EXTRA` or `ENABLE_SYSTEM_TESTING_EXTRA` are set to `ON` when calling CMake:

```
$ cmake -DENABLE_SYSTEM_TESTING_EXTRA=ON ..
$ make obsdiff
$ sudo make install
```

The last step is optional. Without it, you still will get the executable at `../install/obsdiff`.


## Usage

```
$ obsdiff --ref_rinex_obs=reference.20o --test_rinex_obs=rover.20o
```

There is some flexibility in how flags may be specified. The following examples are equivalent:

```
$ obsdiff -ref_rinex_obs=reference.20o
$ obsdiff --ref_rinex_obs=reference.20o
$ obsdiff -ref_rinex_obs reference.20o
$ obsdiff --ref_rinex_obs reference.20o
```



Available command-line flags:

| **Command-line flag**  | **Default value** | **Description**  |
|:-------------------------:|:-----------------:|:-----------------|
| &#8209;`skip_obs_transitory_s`  | `30.0`            | Skip the initial observable outputs to avoid transitory results [s]. |
| `-skip_obs_ends_s`        | `5.0`             | Skip the lasts observable outputs to avoid transitory results [s]. |
| `-single_diffs`           | `false`           | [`true`, `false`]: If `true`, the program also computes the single difference errors for [Carrier Phase](https://gnss-sdr.org/docs/sp-blocks/observables/#carrier-phase-measurement) and [Doppler](https://gnss-sdr.org/docs/sp-blocks/observables/#doppler-shift-measurement) measurements (requires LO synchronization between receivers). |
| `-compare_with_5X`        | `false`           | [`true`, `false`]: If `true`, the program compares the E5a Doppler and Carrier Phases with the E5 full Bw in RINEX (expect discrepancy due to the center frequencies difference). |
| `-dupli_sat`              | `false`           | [`true`, `false`]: If `true`, this flag enables special observable test mode where the scenario contains duplicated satellite orbits. |
| `-dupli_sat_prns`         | `1,2,3,4`         | List of duplicated satellites PRN pairs (_i.e._, `1,2,3,4` indicates that the PRNs 1,2 share the same orbit. The same applies for PRNs 3,4). |
| `-ref_rinex_obs`          | `reference.obs`   | Filename of reference RINEX observation file. |
| `-test_rinex_obs`         | `test.obs`        | Filename of tested RINEX observation file. |
| `-show_plots`             | `true`            | [`true`, `false`]: If `true`, and if [gnuplot](http://www.gnuplot.info/) is found on the system, displays results plots on screen. Please set it to `false` for non-interactive testing. |
