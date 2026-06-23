"""
 plot_format.py

 Shared output-format helpers for the GNSS-SDR Python plotting utilities.

 Provides a common --format command-line argument (same set of formats as
 utils/skyplot/skyplot.py) and a matplotlib style that makes the vector
 outputs (PDF/EPS) publication-ready with embedded fonts.

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2026  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

# Output formats offered by every plotting utility, matching the --format
# option of utils/skyplot/skyplot.py. PNG is the default here.
OUTPUT_FORMATS = ["png", "pdf", "eps", "svg", "jpg"]
DEFAULT_OUTPUT_FORMAT = "png"


def add_output_format_argument(parser):
    """Add the shared --format option to an argparse parser."""
    parser.add_argument(
        "--format",
        dest="output_format",
        type=str.lower,
        default=DEFAULT_OUTPUT_FORMAT,
        choices=OUTPUT_FORMATS,
        help="Saved figure format (default: png). Vector formats (pdf, eps, "
        "svg) are written publication-ready with embedded fonts.",
    )


def apply_publication_style():
    """Configure matplotlib so saved figures are publication-ready.

    The key settings embed the fonts in PDF and EPS output (font type 42 =
    TrueType) instead of leaving them as the default Type 3 fonts, which many
    journals reject. SVG text is kept editable, and raster output uses a high
    DPI. The active font family is left untouched, so plots keep their usual
    look; whatever font is in use gets embedded.
    """
    import matplotlib.pyplot as plt

    plt.rcParams["pdf.fonttype"] = 42  # embed TrueType fonts in PDF
    plt.rcParams["ps.fonttype"] = 42  # embed TrueType fonts in EPS/PS
    plt.rcParams["svg.fonttype"] = "none"  # keep SVG text editable
    plt.rcParams["savefig.dpi"] = 300  # high-resolution png/jpg
    plt.rcParams["savefig.bbox"] = "tight"  # trim surrounding whitespace
