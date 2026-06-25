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
from lib.gnss_sdr_conf import (
    ChannelDump,
    ConfigError,
    SIGNAL_TYPES,
    add_conf_argument,
    load_gnss_sdr_conf,
)
from lib.plot_format import add_output_format_argument, apply_publication_style
from lib.plotVEMLTracking import plotVEMLTracking

DEFAULT_FILE_PREFIX = "track_ch"
DEFAULT_SAMPLING_FREQUENCY = 3000000.0
DEFAULT_CHANNELS = 5
DEFAULT_FIRST_CHANNEL = 0
DEFAULT_SIGNAL_TYPE = "1C"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot GNSS-SDR DLL/PLL VEML tracking dumps."
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
        default=Path("plots/dll-pll-veml-tracking"),
        help="Directory where plots are saved.",
    )
    parser.add_argument(
        "--file-prefix",
        default=None,
        help="GNSS-SDR Tracking.dump_filename value. May include a directory "
        "and extension; the matching <prefix><channel>.dat files are read, "
        "resolved against --input-path. Defaults to the configured Tracking "
        "dump filename or filenames from --conf, or track_ch.",
    )
    parser.add_argument(
        "--sampling-frequency",
        type=float,
        default=None,
        help="Signal sampling frequency in Hz. Defaults to "
        "GNSS-SDR.internal_fs_sps from --conf, or 3000000.0.",
    )
    parser.add_argument(
        "--channels",
        type=int,
        default=None,
        help="Number of channels to read. Defaults to all selected channels "
        "from --conf, or 5.",
    )
    parser.add_argument(
        "--first-channel",
        type=int,
        default=None,
        help="First channel number in the dump filenames. Defaults to the "
        "first selected absolute channel from --conf, or 0.",
    )
    parser.add_argument(
        "--signal-type",
        type=str.upper,
        choices=sorted(SIGNAL_TYPES),
        default=None,
        metavar="CODE",
        help="GNSS-SDR signal code used to restrict channel ranges and dump "
        "filenames from --conf (e.g. 1C, 5X, 7X, E6, J1). Omitted with a "
        "multi-signal --conf means all configured signals.",
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
    args = parser.parse_args()
    try:
        apply_conf_defaults(args)
    except ConfigError as exc:
        parser.error(str(exc))
    return args


def apply_conf_defaults(args):
    conf = load_gnss_sdr_conf(args.conf) if args.conf else None
    signals = []
    explicit_range = args.channels is not None or args.first_channel is not None

    if conf is not None:
        signals = conf.select_signals(args.signal_type)
        if args.signal_type is not None:
            args.signal_type = signals[0].signal

    if args.file_prefix is None:
        if conf is None or (len(signals) == 1 and explicit_range):
            signal = signals[0] if signals else None
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

    if conf is not None and not explicit_range:
        args.channel_plan = conf.channel_dump_plan(
            signals=signals,
            dump_filename_attr="tracking_dump_filename",
            default_file_prefix=DEFAULT_FILE_PREFIX,
            override_file_prefix=args.file_prefix,
        )
        args.channels = len(args.channel_plan)
        args.first_channel = args.channel_plan[0].channel
        args.channel_ids = [entry.channel for entry in args.channel_plan]
        if len(signals) == 1:
            args.file_prefix = args.channel_plan[0].file_prefix
        return

    if args.channels is None:
        args.channels = signals[0].count if signals else DEFAULT_CHANNELS

    if args.first_channel is None:
        args.first_channel = (
            signals[0].first_channel if signals else DEFAULT_FIRST_CHANNEL
        )

    if args.file_prefix is None:
        args.file_prefix = DEFAULT_FILE_PREFIX

    manual_signal = args.signal_type or (
        signals[0].signal if signals else DEFAULT_SIGNAL_TYPE
    )
    args.channel_plan = [
        ChannelDump(
            channel=channel,
            signal=manual_signal,
            file_prefix=args.file_prefix,
        )
        for channel in range(args.first_channel, args.first_channel + args.channels)
    ]
    args.channel_ids = [entry.channel for entry in args.channel_plan]


def read_tracking_dumps(args):
    dumps = []
    for entry in args.channel_plan:
        directory, base = resolve_dump_prefix(entry.file_prefix, args.input_path)
        channel = entry.channel
        tracking_log_path = directory / f"{base}{channel}.dat"
        if not tracking_log_path.exists():
            print(f"Skipping channel {channel}: missing {tracking_log_path}")
            continue
        tracking = dll_pll_veml_read_tracking_dump(tracking_log_path)
        if not tracking["PRN"]:
            print(f"Skipping channel {channel}: no samples in {tracking_log_path}")
            continue
        tracking["_channel"] = channel
        tracking["_signal_type"] = entry.signal
        dumps.append(tracking)
    return dumps


def main():
    args = parse_args()
    args.fig_path.mkdir(parents=True, exist_ok=True)

    apply_publication_style()

    gnss_tracking = read_tracking_dumps(args)
    if not gnss_tracking:
        raise SystemExit("Error: no channel dumps found to plot.")

    plotted_channel_ids = [tracking["_channel"] for tracking in gnss_tracking]
    track_results = []
    settings = {
        "numberOfChannels": len(gnss_tracking),
        "firstChannel": plotted_channel_ids[0],
        "channelIds": plotted_channel_ids,
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
            channel = tracking["_channel"]
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
