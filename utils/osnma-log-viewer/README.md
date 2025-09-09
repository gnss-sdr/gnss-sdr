# Galileo OSNMA Authentication Analyzer

<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2025 Carles Fernandez-Prades <carles.fernandez@cttc.es>
)
<!-- prettier-ignore-end -->

A Python script for analyzing and visualizing Galileo OSNMA (Open Service
Navigation Message Authentication) authentication status from GNSS-SDR log
files.

## Features

- **Log parsing**: Extracts OSNMA authentication data from GNSS-SDR log files.
- **Visualization**: Creates timeline plots of OSNMA authentication status.
- **Multiple output formats**: Supports PDF, PNG, SVG, EPS, and JPG formats.

## Prerequisites

- Python 3.6+
- Required Python packages:
  - `pandas`
  - `matplotlib`

## Usage

```
usage: osnma_log_viewer.py [-h] [--localtime] [--no-show] [-o OUTPUT] [--start START] [--end END] [-v] logfile

Generate a Galileo navigation message authentication timeline plot from a GNSS-SDR log file.

positional arguments:
  logfile              GNSS-SDR log file path

options:
  -h, --help           show this help message and exit
  --localtime          Display results in local time instead of UTC
  --no-show            Run without displaying the plot window
  -o, --output OUTPUT  Output file for plot (default: osnma_auth_timeline.pdf)
  --start START        Initial datetime in "YYYY-MM-DD HH:MM:SS" format
  --end END            Final datetime in "YYYY-MM-DD HH:MM:SS" format
  -v, --version        Show program version and exit

Example: osnma_log_viewer.py gnss-sdr.log --output auth_timeline.png --no-show --localtime
```

### Basic Usage

```
osnma_log_viewer.py gnss-sdr.log
```

This will create a plot named `osnma_auth_timeline.pdf` in the current
directory.

### Specify output file name

```
osnma_log_viewer.py gnss-sdr.log -o authentication_timeline.png
```

- Supported output formats
  - `.pdf` - Vector PDF (recommended for publications)
  - `.png` - Raster PNG image
  - `.svg` - Scalable Vector Graphics
  - `.eps` - Encapsulated PostScript
  - `.jpg` - JPEG image

### Select the time range to plot

Plot from a specific start time to the end of the log file:

```
osnma_log_viewer.py gnss-sdr.log --start "2025-08-26 10:35:00"
```

Plot from the beginning of the log file to a specific end time:

```
osnma_log_viewer.py gnss-sdr.log --end "2025-08-26 10:40:00"
```

Plot between a specific start and end time:

```
osnma_log_viewer.py gnss-sdr.log --start "2025-08-26 10:35:00" --end "2025-08-26 10:40:00"
```

If `--localtime` is passed, the `--start` and `--end` date-times are interpreted
in the local time zone.
