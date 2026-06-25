<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2026 Carles Fernandez-Prades <carles.fernandez@cttc.es>
)
<!-- prettier-ignore-end -->

# GNSS-SDR Python Plot Utilities

These scripts read GNSS-SDR dump files and save diagnostic plots. They are
intended to be run from any directory; input and output locations are
configurable through command-line options.

Every script accepts `-h`/`--help` (printing the full option list and exiting)
and saves PNG figures to a `--fig-path` directory, creating it if needed. Pass
`--show` to also display the figures interactively.

## Common Options

The following options are shared by every script. The per-script tables below
repeat them with the default value that applies to each tool.

- `-h`, `--help`: print the option list and exit.
- `-i`, `--input-path`: directory containing the dump files (default: current
  directory).
- `--conf`: optional GNSS-SDR configuration file. When provided, scripts infer
  missing options such as `--file-prefix`, `--channels`, `--first-channel`,
  `--sampling-frequency`, `--signal-type`, and `--nav-sol-period` from the
  configuration. Explicit command-line options always take precedence over
  values inferred from `--conf`.
- `--file-prefix`: the value of the block's `dump_filename` configuration
  parameter in GNSS-SDR. It may include a directory and an extension; each
  script reconstructs the actual dump filenames from it exactly the way GNSS-SDR
  does — the directory is split off, the basename extension is removed, and the
  block-specific suffix is appended:

  - tracking / telemetry: `<prefix><channel>.dat`
  - observables / PVT: `<prefix>.dat`
  - acquisition:
    `<prefix>_<system>_<signal>_ch_<channel>_<execution>_sat_<PRN>.mat`

  Any directory in `--file-prefix` is resolved relative to `--input-path` (an
  absolute `--file-prefix` path overrides it).

  When `--conf` is provided to the generic tracking and acquisition plotters,
  a multi-signal configuration expands to every enabled signal. Pass
  `--signal-type` to restrict plots to one signal. Single-file modes and
  signal-specific scripts still require or prefer one signal. Channel ranges
  are computed using GNSS-SDR's receiver order:
  `1C, 2S, L5, 1B, 5X, E6, 1G, 2G, B1, B3, 7X, J1, J5`.

- `-o`, `--fig-path`: directory where plots are saved. Defaults to a per-script
  subdirectory under `./plots/`.
- `--show`: display plots interactively after saving them. By default, scripts
  save the PNG files and close the figures. When `--show` is given, all of a
  run's figures are kept open and displayed together with a single blocking
  `plt.show()` at the end; closing the windows lets the script exit.
- `--format`: saved figure format, one of `png` (default), `pdf`, `eps`, `svg`,
  `jpg`. The vector formats (`pdf`, `eps`, `svg`) are written publication-ready
  with embedded fonts — PDF and EPS embed TrueType fonts (font type 42) instead
  of the default Type 3 fonts that many journals reject, so the output is
  suitable for camera-ready figures. Raster formats are saved at 300 DPI. This
  mirrors the `--format` option of `utils/skyplot/skyplot.py`.

## Dependencies

The scripts use Python 3 plus:

- `numpy`, `matplotlib` — all scripts.
- `scipy`, `h5py` — acquisition grid (`plot_acq_grid.py`).
- `pyproj` — PVT UTM coordinate conversion.
- `folium` — PVT map generation (skipped with `--no-position`).

## Acquisition Grid — `plot_acq_grid.py`

Reads acquisition `.mat` dumps and plots the 2D/3D acquisition grid (search
space) of the test statistic.

Input pattern (per dump):

```text
<input-path>/<file-prefix>_<system>_<signal>_ch_<channel>_<execution>_sat_<PRN>.mat
```

Output PNGs (`_2D` and `_3D`):

```text
<fig-path>/<output-basename>_<system>_<signal>_ch_<channel>_<execution>_sat_<PRN>_2D.png
<fig-path>/<output-basename>_<system>_<signal>_ch_<channel>_<execution>_sat_<PRN>_3D.png
```

| Option                                   | Default                  | Description                                                                                                       |
| ---------------------------------------- | ------------------------ | ----------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input-path`                     | `.`                      | Directory containing the acquisition `.mat` dumps.                                                                |
| `--conf`                                 | unset                    | GNSS-SDR configuration used to infer acquisition dump prefixes and signal selectors.                               |
| `--file-prefix`                          | `--conf` or `acquisition` | `Acquisition.dump_filename` value (may include a directory/extension).                                            |
| `-o`, `--fig-path`                       | `plots/acquisition`      | Output directory for PNGs.                                                                                        |
| `--output-basename`, `--output-prefix`   | `--file-prefix` basename | Base name for the saved PNGs (directory and extension stripped).                                                  |
| `--all-files` / `--single-file`          | `--all-files`            | Plot every matching dump, or a single dump chosen by the selectors below.                                         |
| `--positive-only` / `--include-negative` | `--positive-only`        | Plot only positive acquisitions, or positive and negative.                                                        |
| `--lite-view` / `--full-view`            | `--lite-view`            | Interpolated light grid, or the raw acquisition grid.                                                             |
| `--sat`                                  | `1` in single-file mode  | Satellite PRN. Filters in all-files mode; selects the dump in single-file mode.                                   |
| `--channel`                              | `0` in single-file mode  | Acquisition channel number. Filter / selector.                                                                    |
| `--execution`                            | `1` in single-file mode  | Acquisition dump execution index. Filter / selector.                                                              |
| `--signal-type CODE`                     | `--conf` or `1C`         | GNSS-SDR signal code (see table below); case-insensitive. Filters in all-files mode; selects in single-file mode. |
| `--samples-per-chip`                     | `3`                      | Samples per chip used by the light-grid interpolation.                                                            |
| `--samples-per-code`                     | `25000`                  | Samples per code used for the 2D normalization.                                                                   |
| `--input-power`                          | `100.0`                  | Input power used for the 2D normalization.                                                                        |
| `--show`                                 | off                      | Display figures after saving.                                                                                     |
| `--format`                               | `png`                    | Saved figure format: `png` (default), `pdf`, `eps`, `svg`, `jpg`. Vector formats embed fonts (publication-ready). |

Signal codes accepted by `--signal-type`, using the same nomenclature as the
GNSS-SDR configuration and dump filenames (e.g. `--signal-type=E6`):

| Code | System  | Signal | Code length (chips) |
| ---- | ------- | ------ | ------------------- |
| `1C` | GPS     | L1 C/A | 1023                |
| `2S` | GPS     | L2C    | 10230               |
| `L5` | GPS     | L5     | 10230               |
| `1B` | Galileo | E1B    | 4092                |
| `5X` | Galileo | E5a    | 10230               |
| `7X` | Galileo | E5b    | 10230               |
| `E6` | Galileo | E6     | 5115                |
| `1G` | GLONASS | L1     | 511                 |
| `2G` | GLONASS | L2     | 511                 |
| `B1` | BeiDou  | B1I    | 2046                |
| `B3` | BeiDou  | B3I    | 10230               |
| `J1` | QZSS    | L1 C/A | 1023                |
| `J5` | QZSS    | L5     | 10230               |

In all-files mode `--sat`, `--channel`, `--execution`, and `--signal-type`
filter the matching dumps. If `--conf` enables several signals and
`--file-prefix` is not set, all configured acquisition dump prefixes are
scanned. If a `--file-prefix` matches no files, the available dump prefixes
found in the directory are listed.

Examples:

```bash
python3 plot_acq_grid.py
python3 plot_acq_grid.py --input-path ./out --fig-path ./plots/acquisition
python3 plot_acq_grid.py --file-prefix hackrf_l1_acq_dump --channel 0
python3 plot_acq_grid.py --file-prefix hackrf_l1_acq_dump --sat 26
python3 plot_acq_grid.py --signal-type E6
python3 plot_acq_grid.py --conf conf/File_input/Beidou/gnss-sdr_BDS_B3I_byte.conf --signal-type B3 --input-path ./out
python3 plot_acq_grid.py --single-file --signal-type J1 --sat 193 --channel 0 --execution 1
python3 plot_acq_grid.py --single-file --sat 3 --channel 0 --execution 1 --full-view
python3 plot_acq_grid.py --include-negative --output-basename my_run
```

## DLL/PLL VEML Tracking — `dll_pll_veml_plot_sample.py`

Reads per-channel tracking `.dat` dumps and plots VEML tracking diagnostics
(correlator outputs, discriminators, C/N0, ...), plus optional Doppler plots.

Input pattern: `<input-path>/<file-prefix><channel>.dat`.

| Option                 | Default                       | Description                                                                                                       |
| ---------------------- | ----------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input-path`   | `.`                           | Directory containing the tracking `.dat` dumps.                                                                   |
| `--conf`               | unset                         | GNSS-SDR configuration used to infer tracking dump prefix, sampling frequency, and channel range.                  |
| `--file-prefix`        | `--conf` or `track_ch`        | `Tracking.dump_filename` value (may include a directory/extension).                                               |
| `-o`, `--fig-path`     | `plots/dll-pll-veml-tracking` | Output directory for PNGs.                                                                                        |
| `--sampling-frequency` | `--conf` or `3000000.0`       | Signal sampling frequency in Hz (sets the RX-time axis).                                                          |
| `--channels`           | `--conf` or `5`               | Number of channels to read.                                                                                       |
| `--first-channel`      | `--conf` or `0`               | First channel number in the dump filenames.                                                                       |
| `--signal-type CODE`   | inferred if needed            | GNSS-SDR signal code used to restrict channel ranges and dump filenames from `--conf`; omitted means all configured signals. |
| `--plot-last-outputs`  | `0`                           | Only plot the last N outputs; `0` plots all of them.                                                              |
| `--no-doppler`         | Doppler plots on              | Skip the extra Doppler-only plots.                                                                                |
| `--show`               | off                           | Display figures after saving.                                                                                     |
| `--format`             | `png`                         | Saved figure format: `png` (default), `pdf`, `eps`, `svg`, `jpg`. Vector formats embed fonts (publication-ready). |

Examples:

```bash
python3 dll_pll_veml_plot_sample.py --input-path ./out
python3 dll_pll_veml_plot_sample.py --file-prefix tracking_ch_ --channels 8
python3 dll_pll_veml_plot_sample.py --conf conf/File_input/Galileo/gnss-sdr_Galileo_E1_nsr.conf --signal-type 1B --input-path ./out
python3 dll_pll_veml_plot_sample.py --plot-last-outputs 5000 --no-doppler
```

## GPS L1 C/A KF Tracking — `gps_l1_ca_kf_plot_sample.py`

Reads per-channel Kalman-filter tracking `.dat` dumps produced by the
`GPS_L1_CA_KF_Tracking` block and plots tracking plus Kalman-filter variables.

Input pattern: `<input-path>/<file-prefix><channel>.dat`.

| Option                 | Default             | Description                                                                                                       |
| ---------------------- | ------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input-path`   | `.`                 | Directory containing the tracking `.dat` dumps.                                                                   |
| `--conf`               | unset               | GNSS-SDR configuration used to infer tracking dump prefix, sampling frequency, and channel range.                  |
| `--file-prefix`        | `--conf` or `track_ch` | `Tracking.dump_filename` value (may include a directory/extension).                                            |
| `-o`, `--fig-path`     | `plots/kf-tracking` | Output directory for PNGs.                                                                                        |
| `--sampling-frequency` | `--conf` or `4000000.0` | Signal sampling frequency in Hz.                                                                              |
| `--channels`           | `--conf` or `5`     | Number of channels to read.                                                                                       |
| `--first-channel`      | `--conf` or `0`     | First channel number in the dump filenames.                                                                       |
| `--signal-type CODE`   | inferred if needed  | GNSS-SDR signal code used to select channel ranges and dump filenames from `--conf`.                              |
| `--code-period`        | `0.001`             | Code period in seconds.                                                                                           |
| `--show`               | off                 | Display figures after saving.                                                                                     |
| `--format`             | `png`               | Saved figure format: `png` (default), `pdf`, `eps`, `svg`, `jpg`. Vector formats embed fonts (publication-ready). |

> The dumps must come from the `GPS_L1_CA_KF_Tracking` block; standard DLL/PLL
> tracking dumps have a different record layout and will be misread.

Examples:

```bash
python3 gps_l1_ca_kf_plot_sample.py --input-path ./out
python3 gps_l1_ca_kf_plot_sample.py --file-prefix track_ch --channels 8
python3 gps_l1_ca_kf_plot_sample.py --conf conf/File_input/MultiCons/gnss-sdr_labsat_kf.conf --signal-type 1C --input-path ./out
python3 gps_l1_ca_kf_plot_sample.py --sampling-frequency 4000000 --code-period 0.001
```

## Tracking Quality Indicators — `plot_tracking_quality_indicators.py`

Plots the carrier-lock test and C/N0 indicators of all channels on two shared
figures.

Input pattern: `<input-path>/<file-prefix><channel>.dat`.

| Option               | Default                  | Description                                                                                                                                   |
| -------------------- | ------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input-path` | `.`                      | Directory containing the tracking `.dat` dumps.                                                                                               |
| `--conf`             | unset                    | GNSS-SDR configuration used to infer tracking dump prefix, sampling frequency, signal type, and channel range.                                 |
| `--file-prefix`      | `--conf` or `track_ch`   | `Tracking.dump_filename` value (may include a directory/extension).                                                                           |
| `-o`, `--fig-path`   | `plots/tracking-quality` | Output directory for PNGs.                                                                                                                    |
| `--sampling-frequency` | `--conf` or `4000000.0` | Signal sampling frequency in Hz.                                                                                                            |
| `--channels`         | `--conf` or `5`          | Number of channels to read.                                                                                                                   |
| `--first-channel`    | `--conf` or `0`          | First channel number in the dump filenames.                                                                                                   |
| `--signal-type`      | `--conf` or `1C`         | GNSS-SDR signal code used to restrict channel ranges and label traces; omitted with multi-signal `--conf` means all configured signals.       |
| `--show`             | off                      | Display figures after saving.                                                                                                                 |
| `--format`           | `png`                    | Saved figure format: `png` (default), `pdf`, `eps`, `svg`, `jpg`. Vector formats embed fonts (publication-ready).                             |

The C/N0 legend labels each trace as `<signal-type> PRN <PRN>` (e.g.
`1C PRN 9`); the carrier-lock legend labels traces as `SV <signal-type> PRN
<PRN>`.

Examples:

```bash
python3 plot_tracking_quality_indicators.py --input-path ./out
python3 plot_tracking_quality_indicators.py --file-prefix hackrf_l1_tracking_ch_ --channels 8
python3 plot_tracking_quality_indicators.py --conf conf/RealTime_input/gnss-sdr_GPS_L1_USRP_realtime.conf --input-path ./out
python3 plot_tracking_quality_indicators.py --file-prefix hackrf_l1_tracking_ch_ --channels 8 --signal-type 1C
```

## Telemetry — `gps_l1_ca_telemetry_plot_sample.py`

Reads telemetry decoder `.dat` dumps and compares the TOW timing of two
channels.

Input pattern: `<input-path>/<file-prefix><channel>.dat`.

| Option               | Default           | Description                                                                                                       |
| -------------------- | ----------------- | ----------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input-path` | `.`               | Directory containing the telemetry `.dat` dumps.                                                                  |
| `--conf`             | unset             | GNSS-SDR configuration used to infer telemetry dump prefix and channel range.                                      |
| `--file-prefix`      | `--conf` or `telemetry` | `TelemetryDecoder.dump_filename` value (may include a directory/extension).                                |
| `-o`, `--fig-path`   | `plots/telemetry` | Output directory for PNGs.                                                                                        |
| `--channels`         | `--conf` or `18`  | Number of channels to try reading; missing channels are skipped.                                                  |
| `--first-channel`    | `--conf` or `0`   | First channel number in the dump filenames.                                                                       |
| `--signal-type CODE` | inferred if needed | GNSS-SDR signal code used to select channel ranges and dump filenames from `--conf`.                             |
| `--channel-a`        | `--conf` or `0`   | First channel number to plot.                                                                                     |
| `--channel-b`        | `--conf` or `5`   | Second channel number to plot.                                                                                    |
| `--show`             | off               | Display figures after saving.                                                                                     |
| `--format`           | `png`             | Saved figure format: `png` (default), `pdf`, `eps`, `svg`, `jpg`. Vector formats embed fonts (publication-ready). |

Examples:

```bash
python3 gps_l1_ca_telemetry_plot_sample.py --input-path ./out
python3 gps_l1_ca_telemetry_plot_sample.py --conf conf/gnss-sdr.conf --input-path ./out
python3 gps_l1_ca_telemetry_plot_sample.py --file-prefix hackrf_l1_tlm_ch_ --channel-a 0 --channel-b 5
```

## Hybrid Observables — `hybrid_observables_plot_sample.py`

Reads a single observables dump and plots pseudorange, accumulated carrier
phase, Doppler frequency, and captured PRNs across channels.

Input pattern: `<input-path>/<file-prefix>.dat` (single file).

| Option               | Default                    | Description                                                                                                       |
| -------------------- | -------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input-path` | `.`                        | Directory containing the observables dump.                                                                        |
| `--conf`             | unset                      | GNSS-SDR configuration used to infer observables dump filename and total channel count.                            |
| `--file-prefix`      | `--conf` or `observables.dat` | `Observables.dump_filename` value (may include a directory/extension).                                      |
| `-o`, `--fig-path`   | `plots/hybrid-observables` | Output directory for PNGs.                                                                                        |
| `--channels`         | `--conf` or `5`            | Number of observable channels in the dump.                                                                        |
| `--show`             | off                        | Display figures after saving.                                                                                     |
| `--format`           | `png`                      | Saved figure format: `png` (default), `pdf`, `eps`, `svg`, `jpg`. Vector formats embed fonts (publication-ready). |

Examples:

```bash
python3 hybrid_observables_plot_sample.py
python3 hybrid_observables_plot_sample.py --input-path ./out --channels 8
python3 hybrid_observables_plot_sample.py --conf conf/gnss-sdr.conf --input-path ./out
python3 hybrid_observables_plot_sample.py --file-prefix hackrf_l1_observables.dat
```

## PVT Raw Dump — `gps_l1_ca_pvt_raw_plot_sample.py`

Reads a single PVT raw dump and plots navigation/position diagnostics. It also
generates folium HTML maps under `<fig-path>/maps` unless `--no-position` is
given.

Input pattern: `<input-path>/<file-prefix>.dat` (single file).

| Option                              | Default            | Description                                                                                                       |
| ----------------------------------- | ------------------ | ----------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input-path`                | `.`                | Directory containing the PVT dump.                                                                                |
| `--conf`                            | unset              | GNSS-SDR configuration used to infer PVT dump filename and navigation solution period.                             |
| `--file-prefix`                     | `--conf` or `PVT.dat` | `PVT.dump_filename` value (may include a directory/extension).                                                 |
| `-o`, `--fig-path`                  | `plots/pvt`        | Output directory for PNGs and maps.                                                                               |
| `--nav-sol-period`                  | `--conf` or `10.0` | Navigation solution period in milliseconds.                                                                       |
| `--true-position E_UTM N_UTM U_UTM` | unset (NaN)        | Reference receiver position in UTM, used for error plots.                                                         |
| `--plot-skyplot`                    | off                | Try to generate the sky-plot panel.                                                                               |
| `--no-position`                     | position plot on   | Skip the position/map diagnostic (avoids the `folium` dependency).                                                |
| `--one-vs-time NAME`                | `X_vel`, `Tot_Vel` | `navSolutions` variable to plot versus time. Repeatable.                                                          |
| `--show`                            | off                | Display figures after saving.                                                                                     |
| `--format`                          | `png`              | Saved figure format: `png` (default), `pdf`, `eps`, `svg`, `jpg`. Vector formats embed fonts (publication-ready). |
| `--open-maps`                       | off                | Open the generated folium map HTML files in a browser.                                                            |

Examples:

```bash
python3 gps_l1_ca_pvt_raw_plot_sample.py
python3 gps_l1_ca_pvt_raw_plot_sample.py --input-path ./out --file-prefix PVT.dat
python3 gps_l1_ca_pvt_raw_plot_sample.py --conf conf/gnss-sdr.conf --input-path ./out
python3 gps_l1_ca_pvt_raw_plot_sample.py --true-position 431234.0 4587654.0 120.0
python3 gps_l1_ca_pvt_raw_plot_sample.py --one-vs-time X_vel --one-vs-time Y_vel
python3 gps_l1_ca_pvt_raw_plot_sample.py --no-position
```
