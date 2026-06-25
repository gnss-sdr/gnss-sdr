#!/usr/bin/env python3
"""
 gps_l1_ca_telemetry_plot_sample.py

 Reads GNSS-SDR GPS L1 C/A telemetry dump binary files and compares telemetry
 timing fields for two channels.

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

from lib.dump_filename import resolve_dump_prefix
from lib.gnss_sdr_conf import (
    ConfigError,
    SIGNAL_TYPES,
    add_conf_argument,
    load_gnss_sdr_conf,
)
from lib.gps_l1_ca_read_telemetry_dump import gps_l1_ca_read_telemetry_dump
from lib.plot_format import add_output_format_argument, apply_publication_style

DEFAULT_FILE_PREFIX = "telemetry"
DEFAULT_CHANNELS = 18
DEFAULT_FIRST_CHANNEL = 0
DEFAULT_CHANNEL_A = 0
DEFAULT_CHANNEL_B = 5


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot GNSS-SDR GPS L1 C/A telemetry dumps."
    )
    add_conf_argument(parser)
    parser.add_argument(
        "-i",
        "--input-path",
        type=Path,
        default=Path("."),
        help="Directory containing telemetry .dat dumps (default: .).",
    )
    parser.add_argument(
        "-o",
        "--fig-path",
        type=Path,
        default=Path("plots/telemetry"),
        help="Directory where plots are saved.",
    )
    parser.add_argument(
        "--file-prefix",
        default=None,
        help="GNSS-SDR TelemetryDecoder.dump_filename value. May include a "
        "directory and extension; the matching <prefix><channel>.dat files "
        "are read, resolved against --input-path. Defaults to the selected "
        "TelemetryDecoder dump filename from --conf, or telemetry.",
    )
    parser.add_argument(
        "--channels",
        type=int,
        default=None,
        help="Number of channels to try reading. Defaults to the selected "
        "signal's channel count from --conf, or 18.",
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
        "filenames from --conf (default: infer from --conf, preferring 1C "
        "when available).",
    )
    parser.add_argument(
        "--channel-a",
        type=int,
        default=None,
        help="First channel number to plot. Defaults to the first selected "
        "channel from --conf, or 0.",
    )
    parser.add_argument(
        "--channel-b",
        type=int,
        default=None,
        help="Second channel number to plot. Defaults to the second selected "
        "channel from --conf, or 5.",
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
        or args.channel_a is None
        or args.channel_b is None
    ):
        signal = conf.select_signal(
            args.signal_type,
            default_signal="1C",
            prefer_default_on_ambiguous=True,
        )
        args.signal_type = signal.signal

    if args.file_prefix is None:
        args.file_prefix = (
            signal.telemetry_dump_filename
            if signal is not None and signal.telemetry_dump_filename
            else DEFAULT_FILE_PREFIX
        )

    if args.channels is None:
        args.channels = signal.count if signal is not None else DEFAULT_CHANNELS

    if args.first_channel is None:
        args.first_channel = (
            signal.first_channel if signal is not None else DEFAULT_FIRST_CHANNEL
        )

    if args.channel_a is None:
        args.channel_a = (
            signal.first_channel if signal is not None else DEFAULT_CHANNEL_A
        )

    if args.channel_b is None:
        if signal is not None and signal.count > 1:
            args.channel_b = signal.first_channel + 1
        elif signal is not None:
            args.channel_b = signal.first_channel
        else:
            args.channel_b = DEFAULT_CHANNEL_B


def read_telemetry_dumps(args):
    directory, base = resolve_dump_prefix(args.file_prefix, args.input_path)
    telemetry = {}
    for channel in range(args.first_channel, args.first_channel + args.channels):
        telemetry_log_path = directory / f"{base}{channel}.dat"
        try:
            telemetry[channel] = gps_l1_ca_read_telemetry_dump(telemetry_log_path)
        except OSError:
            continue
    return telemetry


def require_channel(telemetry, channel):
    if channel not in telemetry:
        available = ", ".join(str(ch) for ch in sorted(telemetry)) or "none"
        raise ValueError(f"Channel {channel} was not read. Available: {available}")
    return telemetry[channel]


def main():
    args = parse_args()
    args.fig_path.mkdir(parents=True, exist_ok=True)

    apply_publication_style()

    telemetry = read_telemetry_dumps(args)
    data_a = require_channel(telemetry, args.channel_a)
    data_b = require_channel(telemetry, args.channel_b)

    plt.figure()
    plt.gcf().canvas.manager.set_window_title(
        f"Telem_Current_Symbol_TOW_{args.channel_a}_{args.channel_b}.png"
    )
    plt.plot(
        data_a["tracking_sample_counter"],
        [x / 1000 for x in data_a["tow_current_symbol_ms"]],
        "b",
    )
    plt.plot(
        data_b["tracking_sample_counter"],
        [x / 1000 for x in data_b["tow_current_symbol_ms"]],
        "r",
    )
    plt.grid(True)
    plt.xlabel("TRK Sampling Counter")
    plt.ylabel("Current Symbol TOW [s]")
    plt.legend([f"CHN-{args.channel_a}", f"CHN-{args.channel_b}"])
    plt.tight_layout()
    plt.savefig(
        args.fig_path
        / f"Telem_Current_Symbol_TOW_{args.channel_a}_{args.channel_b}"
        f".{args.output_format}"
    )
    if not args.show:
        plt.close()

    plt.figure()
    plt.gcf().canvas.manager.set_window_title(
        f"Telem_TRK_Sampling_Counter_{args.channel_a}_{args.channel_b}.png"
    )
    plt.plot(data_a["tracking_sample_counter"], data_a["tow"], "b")
    plt.plot(data_b["tracking_sample_counter"], data_b["tow"], "r")
    plt.grid(True)
    plt.xlabel("TRK Sampling Counter")
    plt.ylabel("Decoded Nav TOW")
    plt.legend([f"CHN-{args.channel_a}", f"CHN-{args.channel_b}"])
    plt.tight_layout()
    plt.savefig(
        args.fig_path
        / f"Telem_TRK_Sampling_Counter_{args.channel_a}_{args.channel_b}"
        f".{args.output_format}"
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
    except (OSError, ValueError) as exc:
        raise SystemExit(f"Error: {exc}")
