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
import sys

import numpy as np

from lib.dump_filename import resolve_dump_prefix
from lib.gnss_sdr_conf import (
    ConfigError,
    SIGNAL_TYPES,
    add_conf_argument,
    load_gnss_sdr_conf,
)
from lib.gps_l1_ca_kf_read_tracking_dump import gps_l1_ca_kf_read_tracking_dump
from lib.plot_format import add_output_format_argument, apply_publication_style
from lib.plotKalman import plotKalman
from lib.plotTracking import plotTracking

DEFAULT_FILE_PREFIX = "track_ch"
DEFAULT_SAMPLING_FREQUENCY = 4000000.0
DEFAULT_CHANNELS = 5
DEFAULT_FIRST_CHANNEL = 0
DEFAULT_CODE_PERIOD = 0.001


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot GPS L1 C/A Kalman-filter tracking dumps."
    )
    add_conf_argument(parser)
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
        default=None,
        help="GNSS-SDR Tracking.dump_filename value. May include a directory "
        "and extension; the matching <prefix><channel>.dat files are read, "
        "resolved against --input-path. Defaults to the selected Tracking "
        "dump filename from --conf, or track_ch.",
    )
    parser.add_argument(
        "--sampling-frequency",
        type=float,
        default=None,
        help="Signal sampling frequency in Hz. Defaults to "
        "GNSS-SDR.internal_fs_sps from --conf, or 4000000.0.",
    )
    parser.add_argument(
        "--channels",
        type=int,
        default=None,
        help="Number of channels to read. Defaults to the selected signal's "
        "channel count from --conf, or 5.",
    )
    parser.add_argument(
        "--first-channel",
        type=int,
        default=None,
        help="First channel number in the dump filenames. Defaults to the "
        "selected signal's first absolute channel from --conf, or 0.",
    )
    parser.add_argument(
        "--signal-type",
        type=str.upper,
        choices=sorted(SIGNAL_TYPES),
        default=None,
        metavar="CODE",
        help="GNSS-SDR signal code used to select channel ranges and dump "
        "filenames from --conf (default: infer from --conf, otherwise 1C).",
    )
    parser.add_argument(
        "--code-period",
        type=float,
        default=None,
        help="Code period in seconds.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display figures interactively after saving them.",
    )
    add_output_format_argument(parser)
    args = parser.parse_args()
    try:
        apply_conf_defaults(args)
    except ConfigError as exc:
        parser.error(str(exc))
    return args


def apply_conf_defaults(args):
    conf = load_gnss_sdr_conf(args.conf) if args.conf else None
    signal = None
    if conf is not None and (
        args.signal_type is not None
        or args.file_prefix is None
        or args.channels is None
        or args.first_channel is None
    ):
        signal = conf.select_signal(
            args.signal_type,
            default_signal="1C",
            prefer_default_on_ambiguous=True,
        )
        args.signal_type = signal.signal

        if (
            signal.tracking_implementation
            and "KF" not in signal.tracking_implementation.upper()
        ):
            print(
                "Warning: selected tracking implementation "
                f"{signal.tracking_implementation!r} does not look like a "
                "Kalman-filter tracker; the dump layout may be incompatible.",
                file=sys.stderr,
            )

    if args.file_prefix is None:
        args.file_prefix = (
            signal.tracking_dump_filename
            if signal is not None and signal.tracking_dump_filename
            else DEFAULT_FILE_PREFIX
        )

    if args.sampling_frequency is None:
        if conf is not None:
            if conf.internal_fs_sps is None:
                raise ConfigError(
                    "GNSS-SDR.internal_fs_sps is required to infer "
                    "--sampling-frequency."
                )
            args.sampling_frequency = conf.internal_fs_sps
        else:
            args.sampling_frequency = DEFAULT_SAMPLING_FREQUENCY

    if args.channels is None:
        args.channels = signal.count if signal is not None else DEFAULT_CHANNELS

    if args.first_channel is None:
        args.first_channel = (
            signal.first_channel if signal is not None else DEFAULT_FIRST_CHANNEL
        )

    if args.code_period is None:
        args.code_period = DEFAULT_CODE_PERIOD


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
            "firstChannel": args.first_channel,
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
