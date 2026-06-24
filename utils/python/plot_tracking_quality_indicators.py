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
        "--signal-type",
        type=str.upper,
        default="1C",
        metavar="CODE",
        help="GNSS-SDR signal code (e.g. 1C, 5X, 7X, E6, J1) used as the "
        "C/N0 legend label. The tracking dump stores only the PRN, so the "
        "signal is supplied here (default: 1C).",
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
    return parser.parse_args()


def read_tracking_dumps(args):
    directory, base = resolve_dump_prefix(args.file_prefix, args.input_path)
    dumps = []
    for channel in range(args.first_channel, args.first_channel + args.channels):
        tracking_log_path = directory / f"{base}{channel}.dat"
        dumps.append(dll_pll_veml_read_tracking_dump(tracking_log_path))
    return dumps


def prn_label(tracking):
    # Distinct PRNs the channel tracked, in order of appearance. A channel that
    # is reassigned mid-run (lost lock and re-acquired) tracks more than one
    # satellite;
    prns = dict.fromkeys(int(p) for p in tracking["PRN"] if int(p))
    return "/".join(str(p) for p in prns)


def time_seconds(tracking, sampling_frequency):
    return np.asarray(tracking["PRN_start_sample"], dtype=float) / sampling_frequency


def series_by_prn(tracking_list, key, sampling_frequency):
    # Merge each PRN's samples across all channels, sorted by time, so a
    # satellite handed between channels becomes one continuous series. 
    # Assumes no two channels track the same PRN simultaneously
    parts = {}
    for tracking in tracking_list:
        time_s = time_seconds(tracking, sampling_frequency)
        prn = np.asarray(tracking["PRN"])
        values = np.asarray(tracking[key], dtype=float)
        for p in np.unique(prn):
            if not p:
                continue
            sel = prn == p
            parts.setdefault(int(p), []).append((time_s[sel], values[sel]))
    series = {}
    for p, chunks in parts.items():
        time_s = np.concatenate([c[0] for c in chunks])
        values = np.concatenate([c[1] for c in chunks])
        order = np.argsort(time_s, kind="stable")
        series[p] = (time_s[order], values[order])
    return series


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
        ax.set_title(f"PRN {p}", fontsize=9)
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
            label=f"{spec['legend_prefix']} {prn_label(tracking)}",
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
            "legend_prefix": "SV",
        },
        {
            "key": "CN0_SNV_dB_Hz",
            "ylabel": "C/N0 (dB-Hz)",
            "stem": "CN0_SNV_dB_Hz",
            "legend_prefix": f"{args.signal_type} PRN",
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
