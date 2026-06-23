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
from lib.gps_l1_ca_read_telemetry_dump import gps_l1_ca_read_telemetry_dump
from lib.plot_format import add_output_format_argument, apply_publication_style


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot GNSS-SDR GPS L1 C/A telemetry dumps."
    )
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
        default="telemetry",
        help="GNSS-SDR TelemetryDecoder.dump_filename value (default: "
        "telemetry). May include a directory and extension; the matching "
        "<prefix><channel>.dat files are read, resolved against --input-path.",
    )
    parser.add_argument(
        "--channels",
        type=int,
        default=18,
        help="Number of channels to try reading.",
    )
    parser.add_argument(
        "--first-channel",
        type=int,
        default=0,
        help="First channel number in the dump filenames.",
    )
    parser.add_argument(
        "--channel-a",
        type=int,
        default=0,
        help="First channel number to plot.",
    )
    parser.add_argument(
        "--channel-b",
        type=int,
        default=5,
        help="Second channel number to plot.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display figures interactively after saving them.",
    )
    add_output_format_argument(parser)
    return parser.parse_args()


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
