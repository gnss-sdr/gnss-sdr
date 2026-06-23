#!/usr/bin/env python3
"""
 plot_tracking_quality_indicators.py

 Reads GNSS-SDR tracking dump binary files and plots C/N0 plus carrier-lock
 quality indicators across channels.

 File format:
   {input_path}/{file_prefix}{channel}.dat

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from lib.dll_pll_veml_read_tracking_dump import dll_pll_veml_read_tracking_dump
from lib.dump_filename import resolve_dump_prefix
from lib.plot_format import add_output_format_argument, apply_publication_style


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot tracking quality indicators from GNSS-SDR dumps."
    )
    parser.add_argument(
        "-i",
        "--input-path",
        type=Path,
        default=Path("."),
        help="Directory containing tracking .dat dumps (default: .).",
    )
    parser.add_argument(
        "-o",
        "--fig-path",
        type=Path,
        default=Path("plots/tracking-quality"),
        help="Directory where plots are saved.",
    )
    parser.add_argument(
        "--file-prefix",
        default="track_ch",
        help="GNSS-SDR Tracking.dump_filename value (default: track_ch). May "
        "include a directory and extension; the matching <prefix><channel>.dat "
        "files are read, resolved against --input-path.",
    )
    parser.add_argument(
        "--channels",
        type=int,
        default=5,
        help="Number of channels to read.",
    )
    parser.add_argument(
        "--first-channel",
        type=int,
        default=0,
        help="First channel number in the dump filenames.",
    )
    parser.add_argument(
        "--signal-type",
        type=str.upper,
        default="1C",
        metavar="CODE",
        help="GNSS-SDR signal code (e.g. 1C, 5X, 7X, E6, J1) used as the "
        "C/N0 legend label. The tracking dump stores only the PRN, so the "
        "signal is supplied here (default: 1C).",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display figures interactively after saving them.",
    )
    add_output_format_argument(parser)
    return parser.parse_args()


def read_tracking_dumps(args):
    directory, base = resolve_dump_prefix(args.file_prefix, args.input_path)
    dumps = []
    for channel in range(args.first_channel, args.first_channel + args.channels):
        tracking_log_path = directory / f"{base}{channel}.dat"
        dumps.append(dll_pll_veml_read_tracking_dump(tracking_log_path))
    return dumps


def mean_prn_label(tracking):
    return str(round(np.mean(tracking["PRN"])))


def main():
    args = parse_args()
    args.fig_path.mkdir(parents=True, exist_ok=True)
    apply_publication_style()

    gnss_tracking = read_tracking_dumps(args)

    plt.figure()
    plt.gcf().canvas.manager.set_window_title(
        "Carrier lock test output for all the channels"
    )
    plt.title("Carrier lock test output for all the channels")
    carrier_lock_names = []
    for tracking in gnss_tracking:
        plt.plot(tracking["carrier_lock_test"])
        carrier_lock_names.append(f"SV {mean_prn_label(tracking)}")
    plt.legend(carrier_lock_names)
    plt.savefig(args.fig_path / f"carrier_lock_test.{args.output_format}")
    if not args.show:
        plt.close()

    plt.figure()
    plt.gcf().canvas.manager.set_window_title("Carrier CN0 output for all channels")
    plt.title("Carrier CN0 output for all channels")
    cn0_names = []
    for tracking in gnss_tracking:
        plt.plot(tracking["CN0_SNV_dB_Hz"])
        cn0_names.append(f"{args.signal_type} PRN {mean_prn_label(tracking)}")
    plt.legend(cn0_names)
    plt.savefig(args.fig_path / f"CN0_SNV_dB_Hz.{args.output_format}")
    if not args.show:
        plt.close()

    # Show all saved figures with a single plt.show() to avoid the repeated
    # show()/close() cycle that can crash interactive backends on macOS.
    if args.show:
        plt.show()


if __name__ == "__main__":
    try:
        main()
    except OSError as exc:
        raise SystemExit(f"Error: {exc}")
