#!/usr/bin/env python3
"""
 gps_l1_ca_kf_plot_sample.py

 Reads GPS L1 C/A Kalman-filter tracking dump binary files
 (GPS_L1_CA_KF_Tracking) and plots tracking and Kalman-filter variables.

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

import numpy as np

from lib.dump_filename import resolve_dump_prefix
from lib.gps_l1_ca_kf_read_tracking_dump import gps_l1_ca_kf_read_tracking_dump
from lib.plot_format import add_output_format_argument, apply_publication_style
from lib.plotKalman import plotKalman
from lib.plotTracking import plotTracking


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot GPS L1 C/A Kalman-filter tracking dumps."
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
        default=Path("plots/kf-tracking"),
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
        default=4000000.0,
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
        "--code-period",
        type=float,
        default=0.001,
        help="Code period in seconds.",
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
        dumps.append(gps_l1_ca_kf_read_tracking_dump(tracking_log_path))
    return dumps


def main():
    args = parse_args()
    args.fig_path.mkdir(parents=True, exist_ok=True)

    apply_publication_style()

    gnss_tracking = read_tracking_dumps(args)
    track_results = []
    kalman_results = []

    for index, tracking in enumerate(gnss_tracking, start=1):
        track_result = {
            "status": "T",
            "codeFreq": np.copy(tracking["code_freq_hz"]),
            "carrFreq": np.copy(tracking["carrier_doppler_hz"]),
            "carrFreqRate": np.copy(tracking["carrier_doppler_rate_hz2"]),
            "dllDiscr": np.copy(tracking["code_error"]),
            "dllDiscrFilt": np.copy(tracking["code_nco"]),
            "pllDiscr": np.copy(tracking["carr_error"]),
            "pllDiscrFilt": np.copy(tracking["carr_nco"]),
            "I_P": np.copy(tracking["prompt_I"]),
            "Q_P": np.copy(tracking["prompt_Q"]),
            "I_E": np.copy(tracking["E"]),
            "I_L": np.copy(tracking["L"]),
            "Q_E": np.zeros(len(tracking["E"])),
            "Q_L": np.zeros(len(tracking["L"])),
            "PRN": np.copy(tracking["PRN"]),
            "CNo": np.copy(tracking["CN0_SNV_dB_Hz"]),
            "prn_start_time_s": (
                np.copy(tracking["PRN_start_sample"]) / args.sampling_frequency
            ),
        }

        kalman_result = {
            "PRN": np.copy(tracking["PRN"]),
            "innovation": np.copy(tracking["carr_error"]),
            "state1": np.copy(tracking["carr_nco"]),
            "state2": np.copy(tracking["carrier_doppler_hz"]),
            "state3": np.copy(tracking["carrier_doppler_rate_hz2"]),
            "CNo": np.copy(tracking["CN0_SNV_dB_Hz"]),
        }

        track_results.append(track_result)
        kalman_results.append(kalman_result)

        settings = {
            "numberOfChannels": args.channels,
            "msToProcess": len(tracking["E"]),
            "codePeriod": args.code_period,
            "timeStartInSeconds": 0,
            "fig_path": args.fig_path,
            "show": args.show,
            "output_format": args.output_format,
        }

        plotTracking(index, track_results, settings)
        plotKalman(index, kalman_results, settings)

    # Show all saved figures with a single plt.show() to avoid the repeated
    # show()/close() cycle that can crash interactive backends on macOS.
    if args.show:
        import matplotlib.pyplot as plt

        plt.show()


if __name__ == "__main__":
    try:
        main()
    except OSError as exc:
        raise SystemExit(f"Error: {exc}")
