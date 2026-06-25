#!/usr/bin/env python3
"""
 plot_tracking_quality_indicators.py

 Reads GNSS-SDR tracking dump binary files and plots C/N0 plus carrier-lock
 quality indicators against elapsed time.

 Two layouts are available (see --style):
   per-satellite : one subplot per PRN, merged across channels (default). A
                   satellite that is handed between channels (lost lock and
                   re-acquired) appears as a single continuous trace.
   per-channel   : one line per channel on a single axes.

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
import math
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

DEFAULT_FILE_PREFIX = "track_ch"
DEFAULT_SAMPLING_FREQUENCY = 4000000.0
DEFAULT_CHANNELS = 5
DEFAULT_FIRST_CHANNEL = 0
DEFAULT_SIGNAL_TYPE = "1C"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot tracking quality indicators from GNSS-SDR dumps."
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
        default=Path("plots/tracking-quality"),
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
        "GNSS-SDR.internal_fs_sps from --conf, or 4000000.0.",
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
        help="GNSS-SDR signal code (e.g. 1C, 5X, 7X, E6, J1) used to restrict "
        "channel ranges and label traces. Omitted with a multi-signal --conf "
        "means all configured signals.",
    )
    parser.add_argument(
        "--style",
        choices=("per-satellite", "per-channel"),
        default="per-satellite",
        help="per-satellite: one subplot per PRN, merged across channels "
        "(default). per-channel: one line per channel on a single axes.",
    )
    parser.add_argument(
        "--subplot-cols",
        type=int,
        default=3,
        help="Columns in the per-satellite subplot grid; rows grow to fit "
        "the number of satellites (default: 3).",
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
    signals = []
    explicit_range = args.channels is not None or args.first_channel is not None
    if conf is not None:
        signals = conf.select_signals(args.signal_type)
        if args.signal_type is not None:
            args.signal_type = signals[0].signal
    elif args.signal_type is None:
        args.signal_type = DEFAULT_SIGNAL_TYPE

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
        tracking = dll_pll_veml_read_tracking_dump(tracking_log_path)
        tracking["_channel"] = channel
        tracking["_signal_type"] = entry.signal
        dumps.append(tracking)
    return dumps


def prn_label(tracking):
    # Distinct PRNs the channel tracked, in order of appearance. A channel that
    # is reassigned mid-run (lost lock and re-acquired) tracks more than one
    # satellite.
    prns = dict.fromkeys(int(p) for p in tracking["PRN"] if int(p))
    signal = tracking.get("_signal_type")
    prn_list = "/".join(str(p) for p in prns) or "unknown"
    if signal:
        return f"{signal} PRN {prn_list}"
    return prn_list


def time_seconds(tracking, sampling_frequency):
    return np.asarray(tracking["PRN_start_sample"], dtype=float) / sampling_frequency


def series_by_prn(tracking_list, key, sampling_frequency):
    # Merge each signal/PRN series across all channels, sorted by time, so a
    # satellite handed between channels becomes one continuous series. Assumes
    # no two channels track the same signal/PRN simultaneously.
    parts = {}
    for tracking in tracking_list:
        time_s = time_seconds(tracking, sampling_frequency)
        prn = np.asarray(tracking["PRN"])
        values = np.asarray(tracking[key], dtype=float)
        signal = tracking.get("_signal_type", "")
        for p in np.unique(prn):
            if not p:
                continue
            sel = prn == p
            parts.setdefault((signal, int(p)), []).append(
                (time_s[sel], values[sel])
            )
    series = {}
    for key, chunks in parts.items():
        time_s = np.concatenate([c[0] for c in chunks])
        values = np.concatenate([c[1] for c in chunks])
        order = np.argsort(time_s, kind="stable")
        series[key] = (time_s[order], values[order])
    return series


def satellite_title(series_key):
    signal, prn = series_key
    if signal:
        return f"{signal} PRN {prn}"
    return f"PRN {prn}"


def plot_per_satellite(tracking_list, spec, args):
    series = series_by_prn(tracking_list, spec["key"], args.sampling_frequency)
    prns = sorted(series)
    ncols = max(1, args.subplot_cols)
    nrows = max(1, math.ceil(len(prns) / ncols))
    fig, axes = plt.subplots(
        nrows,
        ncols,
        figsize=(4.0 * ncols, 2.3 * nrows),
        sharex=True,
        squeeze=False,
    )
    fig.canvas.manager.set_window_title(f"{spec['ylabel']} per satellite")
    for ax in axes.flat:  # hide unused cells (e.g. 10 sats in a 4x3 grid)
        ax.set_visible(False)
    for ax, p in zip(axes.flat, prns):
        ax.set_visible(True)
        time_s, values = series[p]
        ax.plot(time_s, values, linewidth=0.8)
        ax.set_title(satellite_title(p), fontsize=9)
        ax.grid(alpha=0.3)
    fig.suptitle(f"{spec['ylabel']} per satellite")
    fig.supxlabel("Time(s)")
    fig.supylabel(spec["ylabel"])
    fig.tight_layout()
    fig.savefig(args.fig_path / f"{spec['stem']}.{args.output_format}")
    if not args.show:
        plt.close(fig)


def plot_per_channel(tracking_list, spec, args):
    fig = plt.figure()
    fig.canvas.manager.set_window_title(f"{spec['ylabel']} per channel")
    plt.title(f"{spec['ylabel']} per channel")
    for tracking in tracking_list:
        time_s = time_seconds(tracking, args.sampling_frequency)
        plt.plot(
            time_s,
            tracking[spec["key"]],
            label=spec["label"](tracking),
        )
    plt.xlabel("Time(s)")
    plt.ylabel(spec["ylabel"])
    plt.legend()
    plt.savefig(args.fig_path / f"{spec['stem']}.{args.output_format}")
    if not args.show:
        plt.close(fig)


def main():
    args = parse_args()
    args.fig_path.mkdir(parents=True, exist_ok=True)
    apply_publication_style()

    gnss_tracking = read_tracking_dumps(args)

    specs = [
        {
            "key": "carrier_lock_test",
            "ylabel": "Carrier lock test",
            "stem": "carrier_lock_test",
            "label": lambda tracking: f"SV {prn_label(tracking)}",
        },
        {
            "key": "CN0_SNV_dB_Hz",
            "ylabel": "C/N0 (dB-Hz)",
            "stem": "CN0_SNV_dB_Hz",
            "label": lambda tracking: prn_label(tracking),
        },
    ]
    for spec in specs:
        if args.style == "per-channel":
            plot_per_channel(gnss_tracking, spec, args)
        else:
            plot_per_satellite(gnss_tracking, spec, args)

    # Show all saved figures with a single plt.show() to avoid the repeated
    # show()/close() cycle that can crash interactive backends on macOS.
    if args.show:
        plt.show()


if __name__ == "__main__":
    try:
        main()
    except OSError as exc:
        raise SystemExit(f"Error: {exc}")
