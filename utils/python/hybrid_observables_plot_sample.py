#!/usr/bin/env python3
"""
 hybrid_observables_plot_sample.py

 Reads a GNSS-SDR hybrid observables raw dump and plots pseudorange, carrier
 phase, Doppler, and PRN values.

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

from lib.dump_filename import resolve_dump_prefix
from lib.plot_format import add_output_format_argument, apply_publication_style
from lib.read_hybrid_observables_dump import read_hybrid_observables_dump


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot GNSS-SDR hybrid observables dump data."
    )
    parser.add_argument(
        "-i",
        "--input-path",
        type=Path,
        default=Path("."),
        help="Directory containing the observables dump (default: .).",
    )
    parser.add_argument(
        "--file-prefix",
        default="observables.dat",
        help="GNSS-SDR Observables.dump_filename value (default: "
        "observables.dat). May include a directory and extension; the "
        "matching <prefix>.dat file is read, resolved against --input-path.",
    )
    parser.add_argument(
        "-o",
        "--fig-path",
        type=Path,
        default=Path("plots/hybrid-observables"),
        help="Directory where plots are saved.",
    )
    parser.add_argument(
        "--channels",
        type=int,
        default=5,
        help="Number of observable channels in the dump.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display figures interactively after saving them.",
    )
    add_output_format_argument(parser)
    return parser.parse_args()


def first_valid_observable(gnss_observables, channels):
    min_tow_idx = None
    obs_idx = 0
    for channel in range(channels):
        valid_indices = np.where(np.array(gnss_observables["valid"][channel]) > 0)[0]
        if len(valid_indices) == 0:
            continue
        idx = valid_indices[0]
        if min_tow_idx is None or idx < min_tow_idx:
            min_tow_idx = idx
            obs_idx = channel

    if min_tow_idx is None:
        raise ValueError("No valid observables found in the dump.")
    return min_tow_idx, obs_idx


def save_figure(fig_path, name, show, output_format):
    plt.tight_layout()
    plt.savefig(fig_path / f"{name}.{output_format}")
    # Close unless it will be shown; main() triggers a single plt.show() at the
    # end. Avoids repeated show()/close() cycles, which can crash interactive
    # matplotlib backends (e.g. macOS) on window close.
    if not show:
        plt.close()


def main():
    args = parse_args()
    args.fig_path.mkdir(parents=True, exist_ok=True)
    apply_publication_style()

    directory, base = resolve_dump_prefix(args.file_prefix, args.input_path)
    observables_file = directory / f"{base}.dat"
    gnss_observables = read_hybrid_observables_dump(args.channels, observables_file)
    min_tow_idx, obs_idx = first_valid_observable(gnss_observables, args.channels)
    time_start = gnss_observables["RX_time"][obs_idx][min_tow_idx] - 100
    time_end = gnss_observables["RX_time"][obs_idx][-1] + 100

    plt.figure()
    plt.title("Pseudorange")
    for channel in range(args.channels):
        plt.scatter(
            gnss_observables["RX_time"][channel][min_tow_idx:],
            gnss_observables["Pseudorange_m"][channel][min_tow_idx:],
            s=1,
            label=f"Channel {channel}",
        )
    plt.xlim(time_start, time_end)
    plt.grid(True)
    plt.xlabel("TOW [s]")
    plt.ylabel("Pseudorange [m]")
    plt.legend()
    plt.gcf().canvas.manager.set_window_title("Pseudorange.png")
    save_figure(args.fig_path, "Pseudorange", args.show, args.output_format)

    plt.figure()
    plt.title("Carrier Phase")
    for channel in range(args.channels):
        plt.scatter(
            gnss_observables["RX_time"][channel][min_tow_idx:],
            gnss_observables["Carrier_phase_hz"][channel][min_tow_idx:],
            s=1,
            label=f"Channel {channel}",
        )
    plt.xlim(time_start, time_end)
    plt.xlabel("TOW [s]")
    plt.ylabel("Accumulated Carrier Phase [cycles]")
    plt.grid(True)
    plt.legend()
    plt.gcf().canvas.manager.set_window_title("AccumulatedCarrierPhase.png")
    save_figure(args.fig_path, "AccumulatedCarrierPhase", args.show, args.output_format)

    plt.figure()
    plt.title("Doppler Effect")
    for channel in range(args.channels):
        plt.scatter(
            gnss_observables["RX_time"][channel][min_tow_idx:],
            gnss_observables["Carrier_Doppler_hz"][channel][min_tow_idx:],
            s=1,
            label=f"Channel {channel}",
        )
    plt.xlim(time_start, time_end)
    plt.xlabel("TOW [s]")
    plt.ylabel("Doppler Frequency [Hz]")
    plt.grid(True)
    plt.legend()
    plt.gcf().canvas.manager.set_window_title("DopplerFrequency.png")
    save_figure(args.fig_path, "DopplerFrequency", args.show, args.output_format)

    plt.figure()
    plt.title("GNSS Channels captured")
    for channel in range(args.channels):
        label = "unknown"
        for prn in gnss_observables["PRN"][channel][min_tow_idx:]:
            if int(prn) != 0:
                label = str(int(prn))
                break
        plt.scatter(
            gnss_observables["RX_time"][channel][min_tow_idx:],
            gnss_observables["PRN"][channel][min_tow_idx:],
            s=1,
            label=f"PRN {channel} = {label}",
        )
    plt.xlim(time_start, time_end)
    plt.xlabel("TOW [s]")
    plt.ylabel("PRN")
    plt.grid(True)
    plt.legend()
    plt.gcf().canvas.manager.set_window_title("PRNs.png")
    save_figure(args.fig_path, "PRNs", args.show, args.output_format)

    # Show all saved figures with a single plt.show() to avoid the repeated
    # show()/close() cycle that can crash interactive backends on macOS.
    if args.show:
        plt.show()


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as exc:
        raise SystemExit(f"Error: {exc}")
