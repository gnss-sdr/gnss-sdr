#!/usr/bin/env python3
"""
 dll_pll_veml_plot_sample.py

 Reads GNSS-SDR DLL/PLL VEML tracking dump binary files and plots internal
 tracking variables.

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
from lib.plotVEMLTracking import plotVEMLTracking


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot GNSS-SDR DLL/PLL VEML tracking dumps."
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
        default=Path("plots/dll-pll-veml-tracking"),
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
        "--sampling-frequency",
        type=float,
        default=3000000.0,
        help="Signal sampling frequency in Hz.",
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
        "--plot-last-outputs",
        type=int,
        default=0,
        help="Only plot the last N outputs; 0 plots all outputs.",
    )
    parser.add_argument(
        "--no-doppler",
        dest="plot_doppler",
        action="store_false",
        help="Do not generate the extra Doppler-only plots.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display figures interactively after saving them.",
    )
    add_output_format_argument(parser)
    parser.set_defaults(plot_doppler=True)
    return parser.parse_args()


def read_tracking_dumps(args):
    directory, base = resolve_dump_prefix(args.file_prefix, args.input_path)
    dumps = []
    for channel in range(args.first_channel, args.first_channel + args.channels):
        tracking_log_path = directory / f"{base}{channel}.dat"
        dumps.append(dll_pll_veml_read_tracking_dump(tracking_log_path))
    return dumps


def main():
    args = parse_args()
    args.fig_path.mkdir(parents=True, exist_ok=True)

    apply_publication_style()

    gnss_tracking = read_tracking_dumps(args)
    track_results = []
    settings = {
        "numberOfChannels": args.channels,
        "fig_path": args.fig_path,
        "show": args.show,
        "output_format": args.output_format,
    }

    for index, tracking in enumerate(gnss_tracking, start=1):
        if 0 < args.plot_last_outputs < len(tracking["code_freq_hz"]):
            start_sample = len(tracking["code_freq_hz"]) - args.plot_last_outputs
        else:
            start_sample = 0

        track_result = {
            "status": "T",
            "codeFreq": np.copy(tracking["code_freq_hz"][start_sample:]),
            "carrFreq": np.copy(tracking["carrier_doppler_hz"][start_sample:]),
            "dllDiscr": np.copy(tracking["code_error"][start_sample:]),
            "dllDiscrFilt": np.copy(tracking["code_nco"][start_sample:]),
            "pllDiscr": np.copy(tracking["carr_error"][start_sample:]),
            "pllDiscrFilt": np.copy(tracking["carr_nco"][start_sample:]),
            "I_P": np.copy(tracking["P"][start_sample:]),
            "Q_P": np.zeros(len(tracking["P"][start_sample:])),
            "I_VE": np.copy(tracking["VE"][start_sample:]),
            "I_E": np.copy(tracking["E"][start_sample:]),
            "I_L": np.copy(tracking["L"][start_sample:]),
            "I_VL": np.copy(tracking["VL"][start_sample:]),
            "Q_VE": np.zeros(len(tracking["VE"][start_sample:])),
            "Q_E": np.zeros(len(tracking["E"][start_sample:])),
            "Q_L": np.zeros(len(tracking["L"][start_sample:])),
            "Q_VL": np.zeros(len(tracking["VL"][start_sample:])),
            "data_I": np.copy(tracking["prompt_I"][start_sample:]),
            "data_Q": np.copy(tracking["prompt_Q"][start_sample:]),
            "PRN": np.copy(tracking["PRN"][start_sample:]),
            "CNo": np.copy(tracking["CN0_SNV_dB_Hz"][start_sample:]),
            "prn_start_time_s": (
                np.copy(tracking["PRN_start_sample"][start_sample:])
                / args.sampling_frequency
            ),
        }
        track_results.append(track_result)

        plotVEMLTracking(index, track_results, settings)

        if args.plot_doppler:
            channel = args.first_channel + index - 1
            plt.figure()
            plt.plot(
                track_result["prn_start_time_s"],
                [x / 1000 for x in tracking["carrier_doppler_hz"][start_sample:]],
            )
            plt.xlabel("Time(s)")
            plt.ylabel("Doppler(KHz)")
            plt.title(f"Doppler frequency channel {channel}")
            plt.savefig(
                args.fig_path
                / f"Doppler_freq_ch_{channel}.{args.output_format}"
            )
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
