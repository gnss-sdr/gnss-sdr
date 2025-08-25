# GNSS Skyplot utility

<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2025 Carles Fernandez-Prades <carles.fernandez@cttc.es>
)
<!-- prettier-ignore-end -->

A Python script that generates polar skyplots from RINEX navigation files,
showing satellite visibility over time.

## Features

- Processes RINEX navigation files.
- Calculates satellite positions using broadcast ephemeris.
- Plots satellite tracks in azimuth-elevation coordinates.
- Customizable observer location.
- Color-codes satellites by constellation (GPS, Galileo, GLONASS, BeiDou).
- Elevation mask set to 5°, configurable via the `--elev-mask` optional
  argument.
- Outputs high-quality image in PDF format. EPS, PNG, and SVG formats are also
  available via the `--format` optional argument.
- Non-interactive mode for CI jobs with the `--no-show` optional argument.
- Constellations to plot can be configured via the `--system` optional argument.
- Optionally, it uses the corresponding RINEX observation file to limit the plot
  to the receiver observation time via the`--use-obs` optional argument.
  - If this argument is set, the tool looks for a matching file following
    standard RINEX naming conventions, and uses it if found.
  - If this argument is set and the observation file is found, the position
    selection logic is:
    1. If user provides latitude, longitude, and altitude as positional
       arguments, that position is always used.
    2. Otherwise, if the provided observation file contains a valid
       `APPROX POSITION XYZ` field in its header, that position is used.
    3. Otherwise, the default position is used.

## Requirements

- Python 3.6+
- Required packages:
  - `numpy`
  - `matplotlib`

## Usage

### Basic Command

```
./skyplot.py <RINEX_FILE> [LATITUDE] [LONGITUDE] [ALTITUDE]
             [--elev-mask ELEV_MASK]
             [--format {pdf,eps,png,svg}]
             [--no-show]
             [--system SYSTEM [SYSTEM ...]]
             [--use-obs]
```

### Arguments

| Argument         | Type     | Units       | Description              | Default  |
| ---------------- | -------- | ----------- | ------------------------ | -------- |
| `RINEX_NAV_FILE` | Required | -           | RINEX nav file path      | -        |
| `LATITUDE`       | Optional | degrees (°) | North/South position     | 41.275°N |
| `LONGITUDE`      | Optional | degrees (°) | East/West position       | 1.9876°E |
| `ALTITUDE`       | Optional | meters (m)  | Height above sea level   | 80.0 m   |
| `--elev-mask`    | Optional | degrees (°) | Elevation mask           | 5°       |
| `--format`       | Optional | -           | Output {pdf,eps,png,svg} | pdf      |
| `--no-show`      | Optional | -           | Do not show plot         | -        |
| `--system`       | Optional | -           | Systems to plot          | All      |
| `--use-obs`      | Optional | -           | Use RINEX obs data       | -        |

### Examples

- Skyplot from default location (Castelldefels, Spain):
  ```
  ./skyplot.py brdc0010.22n
  ```
- Skyplot from custom location (New York City, USA):
  ```
  ./skyplot.py brdc0010.22n 40.7128 -74.0060 10.0
  ```
- Skyplot from custom location (Santiago, Chile) using receiver observation
  time:
  ```
  ./skyplot.py brdc0010.22n -33.4592 -70.6453 520.0 --use-obs
  ```
- Non-interactive mode (for CI jobs):
  ```
  ./skyplot.py brdc0010.22n -33.4592 -70.6453 520.0 --no-show
  ```
- Only plot GPS and Galileo satellites:
  ```
  ./skyplot.py brdc0010.22n -33.4592 -70.6453 520.0 --system GPS Galileo
  ```
  or
  ```
  ./skyplot.py brdc0010.22n -33.4592 -70.6453 520.0 --system G E
  ```
- Get a PNG file:
  ```
  ./skyplot.py brdc0010.22n -33.4592 -70.6453 520.0 --format png
  ```

## Output

The script generates a PDF file named `skyplot_<RINEX_FILE>.pdf` (with dots in
`<RINEX_FILE>` replaced by `_`) with:

- Satellite trajectories over all epochs in the file.
  - NAV file - ephemeris time range (default).
  - Receiver observation time if `--use-obs` is specified and the RINEX OBS file
    is found.
- Color-coded by constellation.
- Observer location in title.
- Time range in footer.
- Embedded fonts that display consistently across all systems and generate
  publication-ready figures.
- EPS, PNG, and SVG output formats available via `--format eps`, `--format png`,
  and `--format svg`.
