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
- Optionally uses an OBS file to limit plot to the receiver active pricessing time (--use-obs).
  - When enabled, the tool looks for a matching file by replacing the last character of the NAV filename with O/o and uses it if found.
- Calculates satellite positions using broadcast ephemeris.
- Plots satellite tracks in azimuth-elevation coordinates.
- Color-codes satellites by constellation (GPS, Galileo, GLONASS, BeiDou).
- Customizable observer location.
- Outputs high-quality image in PDF format.
- Non-interactive mode for CI jobs (with `--no-show` flag).

## Requirements

- Python 3.6+
- Required packages:
  - `numpy`
  - `matplotlib`

## Usage

### Basic Command

```
./skyplot.py <RINEX_FILE> [LATITUDE] [LONGITUDE] [ALTITUDE] [--use-obs] [--no-show]
```

### Arguments

| Argument         | Type     | Units       | Description            | Default  |
| ---------------- | -------- | ----------- | ---------------------- | -------- |
| `RINEX_NAV_FILE` | Required | -           | RINEX nav file path    | -        |
| `LATITUDE`       | Optional | degrees (°) | North/South position   | 41.275°N |
| `LONGITUDE`      | Optional | degrees (°) | East/West position     | 1.9876°E |
| `ALTITUDE`       | Optional | meters (m)  | Height above sea level | 80.0 m   |
| `--use-obs`      | Optional | -           | Use RINEX obs data     | -        |
| `--no-show`      | Optional | -           | Do not show plot       | -        |

### Examples

- Skyplot from default location (Castelldefels, Spain):
  ```
  ./skyplot.py brdc0010.22n
  ```
- Skyplot from custom location (New York City, USA):
  ```
  ./skyplot.py brdc0010.22n 40.7128 -74.0060 10.0
  ```
- Skyplot from custom location (Santiago, Chile) using receiver active window:
  ```
  ./skyplot.py brdc0010.22n -33.4592 -70.6453 520.0 --use-obs
  ```
- Non-interactive mode (for CI jobs):
  ```
  ./skyplot.py brdc0010.22n -33.4592 -70.6453 520.0 --no-show
  ```

## Output

The script generates a PDF file named `skyplot_<RINEX_FILE>.pdf` (with dots in
`<RINEX_FILE>` replaced by `_`) with:

- Satellite trajectories over all epochs in the file.
  - Entire NAV file span (default)
  - Receiver active span if `--use-obs` is specified and OBS file is found
- Color-coded by constellation.
- Observer location in title.
- Time range in footer.
