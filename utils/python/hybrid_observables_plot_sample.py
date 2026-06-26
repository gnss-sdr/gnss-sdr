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
from lib.gnss_sdr_conf import (
    ConfigError,
    add_conf_argument,
    load_gnss_sdr_conf,
    signal_pretty_name,
)
from lib.plot_format import add_output_format_argument, apply_publication_style
from lib.read_hybrid_observables_dump import read_hybrid_observables_dump

DEFAULT_FILE_PREFIX = "observables.dat"
DEFAULT_CHANNELS = 5


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot GNSS-SDR hybrid observables dump data."
    )
    add_conf_argument(parser)
    parser.add_argument(
        "-i",
        "--input-path",
        type=Path,
        default=Path("."),
        help="Directory containing the observables dump (default: .).",
    )
    parser.add_argument(
        "--file-prefix",
        default=None,
        help="GNSS-SDR Observables.dump_filename value. May include a "
        "directory and extension; the matching <prefix>.dat file is read, "
        "resolved against --input-path. Defaults to Observables.dump_filename "
        "from --conf, or observables.dat.",
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
        default=None,
        help="Number of observable channels in the dump. Defaults to the "
        "total configured channel count from --conf, or 5.",
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

    if args.file_prefix is None:
        args.file_prefix = (
            conf.observables_dump_filename
            if conf is not None and conf.observables_dump_filename
            else DEFAULT_FILE_PREFIX
        )

    if args.channels is None:
        if conf is not None:
            if conf.total_channels <= 0:
                raise ConfigError(
                    "At least one Channels_<signal>.count value is required "
                    "to infer --channels."
                )
            args.channels = conf.total_channels
        else:
            args.channels = DEFAULT_CHANNELS

    args.channel_signals = channel_signal_labels(conf, args.channels)


def channel_signal_labels(conf, channels):
    labels = [""] * channels
    if conf is None:
        return labels

    for signal in conf.enabled_signals:
        for channel in signal.channels:
            if 0 <= channel < channels:
                labels[channel] = signal.signal
    return labels


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


def valid_channel_data(gnss_observables, channel, observable):
    valid_mask = np.array(gnss_observables["valid"][channel]) > 0
    rx_time = np.array(gnss_observables["RX_time"][channel])
    values = np.array(gnss_observables[observable][channel])
    return rx_time[valid_mask], values[valid_mask]


def valid_channel_samples(gnss_observables, channel, observable):
    valid_mask = np.array(gnss_observables["valid"][channel]) > 0
    rx_time = np.array(gnss_observables["RX_time"][channel])
    values = np.array(gnss_observables[observable][channel])
    prns = np.array(gnss_observables["PRN"][channel])
    return rx_time[valid_mask], values[valid_mask], prns[valid_mask]


def valid_time_bounds(gnss_observables, channels):
    valid_times = []
    for channel in range(channels):
        rx_time, _ = valid_channel_data(gnss_observables, channel, "valid")
        if len(rx_time) > 0:
            valid_times.append(rx_time)

    if not valid_times:
        raise ValueError("No valid observables found in the dump.")

    valid_times = np.concatenate(valid_times)
    return valid_times.min() - 100, valid_times.max() + 100


def series_by_tracked_prn(gnss_observables, channels, observable, channel_signals):
    parts = {}
    for channel in range(channels):
        rx_time, values, prns = valid_channel_samples(
            gnss_observables, channel, observable
        )
        signal = channel_signals[channel] if channel < len(channel_signals) else ""
        for prn in np.unique(prns):
            if int(prn) == 0:
                continue
            selected = prns == prn
            parts.setdefault((signal, int(prn)), []).append(
                (rx_time[selected], values[selected])
            )

    series = {}
    for series_key, chunks in parts.items():
        rx_time = np.concatenate([chunk[0] for chunk in chunks])
        values = np.concatenate([chunk[1] for chunk in chunks])
        order = np.argsort(rx_time, kind="stable")
        series[series_key] = (rx_time[order], values[order])
    return series


def tracked_prn_label(series_key):
    signal, prn = series_key
    if signal:
        return f"{signal_pretty_name(signal)} PRN {prn}"
    return f"PRN {prn}"


def plot_observable_by_tracked_prn(
    gnss_observables,
    args,
    *,
    observable,
    title,
    ylabel,
    window_title,
    output_name,
    time_start,
    time_end,
):
    plt.figure()
    plt.title(title)
    series = series_by_tracked_prn(
        gnss_observables, args.channels, observable, args.channel_signals
    )
    for series_key in sorted(series):
        rx_time, values = series[series_key]
        plt.scatter(
            rx_time,
            values,
            s=1,
            label=tracked_prn_label(series_key),
        )
    plt.xlim(time_start, time_end)
    plt.xlabel("TOW [s]")
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.gcf().canvas.manager.set_window_title(window_title)
    save_figure(args.fig_path, output_name, args.show, args.output_format)


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
    first_valid_observable(gnss_observables, args.channels)
    time_start, time_end = valid_time_bounds(gnss_observables, args.channels)

    plot_observable_by_tracked_prn(
        gnss_observables,
        args,
        observable="Pseudorange_m",
        title="Pseudorange",
        ylabel="Pseudorange [m]",
        window_title="Pseudorange.png",
        output_name="Pseudorange",
        time_start=time_start,
        time_end=time_end,
    )

    plot_observable_by_tracked_prn(
        gnss_observables,
        args,
        observable="Carrier_phase_hz",
        title="Carrier Phase",
        ylabel="Accumulated Carrier Phase [cycles]",
        window_title="AccumulatedCarrierPhase.png",
        output_name="AccumulatedCarrierPhase",
        time_start=time_start,
        time_end=time_end,
    )

    plot_observable_by_tracked_prn(
        gnss_observables,
        args,
        observable="Carrier_Doppler_hz",
        title="Doppler Effect",
        ylabel="Doppler Frequency [Hz]",
        window_title="DopplerFrequency.png",
        output_name="DopplerFrequency",
        time_start=time_start,
        time_end=time_end,
    )

    plt.figure()
    plt.title("GNSS Channels captured")
    for channel in range(args.channels):
        rx_time, prns = valid_channel_data(gnss_observables, channel, "PRN")
        if len(rx_time) == 0:
            continue
        plt.scatter(
            rx_time,
            prns,
            s=1,
            label=f"Channel {channel}",
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
