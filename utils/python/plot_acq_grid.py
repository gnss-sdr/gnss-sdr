#!/usr/bin/env python3
"""
 plot_acq_grid.py

 Reads GNSS-SDR Acquisition dump .mat files and plots acquisition grids.

 Irene Perez Riega, 2023. iperrie@inta.es
 Minhaj Uddin Ahmad, 2026. mahmad12@crimson.ua.edu

 File format:
   {path}/{file_prefix}_{system}_{signal}_ch_{channel}_{execution}_sat_{sat}.mat

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from lib.dump_filename import resolve_dump_prefix
from lib.gnss_sdr_conf import (
    ConfigError,
    N_CHIPS,
    SIGNAL_TYPES,
    add_conf_argument,
    load_gnss_sdr_conf,
)
from lib.plot_format import add_output_format_argument, apply_publication_style

# Heavy numerical and plotting libraries (h5py, numpy, matplotlib, scipy) are
# imported lazily inside the functions that need them. This keeps --help and
# argument-error invocations fast and free of matplotlib font-cache warnings.


DEFAULT_FILE_PREFIX = "acquisition"
DEFAULT_SAT = 1
DEFAULT_CHANNEL = 0
DEFAULT_EXECUTION = 1
DEFAULT_SIGNAL_TYPE = "1C"


@dataclass(frozen=True)
class AcquisitionDumpPlan:
    signal: Optional[str]
    system: Optional[str]
    file_prefix: str


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot GNSS-SDR acquisition dump grids."
    )
    add_conf_argument(parser)
    parser.add_argument(
        "-i",
        "--input-path",
        type=Path,
        default=Path("."),
        help="Directory containing acquisition .mat dumps (default: .).",
    )
    parser.add_argument(
        "-o",
        "--fig-path",
        type=Path,
        default=Path("plots/acquisition"),
        help="Directory where plots are saved (default: ./plots/acquisition).",
    )
    parser.add_argument(
        "--file-prefix",
        default=None,
        help="GNSS-SDR Acquisition.dump_filename value. May include a "
        "directory and extension; matching "
        "<prefix>_<sys>_<sig>_ch_<ch>_<exec>_sat_<PRN>.mat files are read, "
        "resolved against --input-path. Defaults to the configured "
        "Acquisition dump filename or filenames from --conf, or acquisition.",
    )
    parser.add_argument(
        "--output-basename",
        "--output-prefix",
        dest="output_basename",
        default=None,
        help="Base name used for saved plot files (default: the --file-prefix "
        "basename, without directory or extension).",
    )

    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--all-files",
        dest="plot_all_files",
        action="store_true",
        help="Plot all matching dump files (default).",
    )
    mode.add_argument(
        "--single-file",
        dest="plot_all_files",
        action="store_false",
        help="Plot one dump selected by --sat, --channel, --execution, and "
        "--signal-type.",
    )
    parser.set_defaults(plot_all_files=True)

    acq_filter = parser.add_mutually_exclusive_group()
    acq_filter.add_argument(
        "--positive-only",
        dest="plot_positive_acqs",
        action="store_true",
        help="Only plot positive acquisitions (default).",
    )
    acq_filter.add_argument(
        "--include-negative",
        dest="plot_positive_acqs",
        action="store_false",
        help="Plot positive and negative acquisition dumps.",
    )
    parser.set_defaults(plot_positive_acqs=True)

    view = parser.add_mutually_exclusive_group()
    view.add_argument(
        "--lite-view",
        dest="lite_view",
        action="store_true",
        help="Use interpolated light grid representation (default).",
    )
    view.add_argument(
        "--full-view",
        dest="lite_view",
        action="store_false",
        help="Plot the raw acquisition grid.",
    )
    parser.set_defaults(lite_view=True)

    parser.add_argument(
        "--sat",
        type=int,
        default=None,
        help="Satellite PRN. Filters all-files mode; defaults to 1 in "
        "single-file mode.",
    )
    parser.add_argument(
        "--channel",
        type=int,
        default=None,
        help="Acquisition channel number. Filters all-files mode; defaults "
        "to 0 in single-file mode.",
    )
    parser.add_argument(
        "--execution",
        type=int,
        default=None,
        help="Acquisition dump execution. Filters all-files mode; defaults "
        "to 1 in single-file mode.",
    )
    parser.add_argument(
        "--signal-type",
        type=str.upper,
        choices=sorted(SIGNAL_TYPES),
        default=None,
        metavar="CODE",
        help="GNSS-SDR signal code (e.g. 1C, 5X, 7X, E6, J1). Filters "
        "all-files mode; omitted with a multi-signal --conf scans all "
        "configured signals. Defaults to 1C (GPS L1 C/A) in single-file "
        "mode. Choices: " + ", ".join(sorted(SIGNAL_TYPES)) + ".",
    )
    parser.add_argument(
        "--samples-per-chip",
        type=int,
        default=3,
        help="Samples per chip used by the light grid interpolation.",
    )
    parser.add_argument(
        "--samples-per-code",
        type=int,
        default=25000,
        help="Samples per code used for the 2D normalization.",
    )
    parser.add_argument(
        "--input-power",
        type=float,
        default=100.0,
        help="Input power used for the 2D normalization.",
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
    if args.output_basename is None:
        # Default the saved-plot basename to the dump basename, dropping any
        # directory and extension carried by --file-prefix. A multi-signal
        # configuration may use several prefixes, so leave the basename empty
        # and let the system/signal/channel fields identify each output.
        if getattr(args, "acquisition_plans", None) and args.file_prefix is None:
            args.output_basename = None
        else:
            _, args.output_basename = resolve_dump_prefix(args.file_prefix)
    return args


def apply_conf_defaults(args):
    conf = load_gnss_sdr_conf(args.conf) if args.conf else None
    signal = None
    signals = []
    args.acquisition_plans = []

    if conf is not None:
        if args.signal_type is not None:
            signal = conf.require_enabled_signal(args.signal_type)
            signals = [signal]
        elif args.plot_all_files and args.file_prefix is None:
            signals = conf.select_signals()
            if len(signals) == 1:
                signal = signals[0]
                args.signal_type = signal.signal
        elif not args.plot_all_files:
            signal = conf.select_signal()
            signals = [signal]
            args.signal_type = signal.signal

    if conf is not None and args.plot_all_files and args.file_prefix is None:
        args.acquisition_plans = [
            AcquisitionDumpPlan(
                signal=selected.signal,
                system=selected.system,
                file_prefix=(
                    selected.acquisition_dump_filename
                    or DEFAULT_FILE_PREFIX
                ),
            )
            for selected in signals
        ]
        if len(args.acquisition_plans) == 1:
            args.file_prefix = args.acquisition_plans[0].file_prefix

    if args.file_prefix is None and not args.acquisition_plans:
        args.file_prefix = (
            signal.acquisition_dump_filename
            if signal is not None and signal.acquisition_dump_filename
            else DEFAULT_FILE_PREFIX
        )

    if args.signal_type is None and not args.plot_all_files:
        args.signal_type = DEFAULT_SIGNAL_TYPE


def acquisition_plans_for_args(args):
    if getattr(args, "acquisition_plans", None):
        return args.acquisition_plans

    system = None
    if args.signal_type is not None:
        system, _ = SIGNAL_TYPES[args.signal_type]
    return [
        AcquisitionDumpPlan(
            signal=args.signal_type,
            system=system,
            file_prefix=args.file_prefix,
        )
    ]


def positive_acq_value(data):
    for key in ("positive_acq", "d_positive_acq"):
        value = data.get(key)
        if value is not None:
            return value[0]
    return None


def parse_dump_name(file_path):
    parts = file_path.stem.split("_")
    if len(parts) < 8:
        return None
    try:
        if parts[-5] != "ch" or parts[-2] != "sat":
            return None
        return {
            "file_prefix": "_".join(parts[:-7]),
            "system": parts[-7],
            "signal": parts[-6],
            "channel": int(parts[-4]),
            "execution": int(parts[-3]),
            "sat": int(parts[-1]),
        }
    except ValueError:
        return None


def matching_dump_prefixes(input_path):
    prefixes = set()
    for file_path in input_path.glob("*.mat"):
        metadata = parse_dump_name(file_path)
        if metadata is not None:
            prefixes.add(metadata["file_prefix"])
    return sorted(prefixes)


def read_acquisition_dump(file_path, n_chips, positive_only):
    import h5py
    import numpy as np

    with h5py.File(file_path, "r") as data:
        positive_acq = positive_acq_value(data)
        if positive_only:
            if positive_acq is None:
                print(f"Skipping {file_path.name}: no positive_acq variable")
                return None
            if positive_acq != 1:
                return None

        acq_grid = data["acq_grid"][:]
        n_fft, n_dop_bins = acq_grid.shape
        d_max, f_max = np.unravel_index(np.argmax(acq_grid), acq_grid.shape)
        doppler_step = data["doppler_step"][0]
        doppler_max = data["doppler_max"][0]
        freq = np.arange(n_dop_bins) * doppler_step - doppler_max
        delay = np.arange(n_fft) / n_fft * n_chips

    return acq_grid, n_fft, d_max, f_max, doppler_step, doppler_max, freq, delay


def plot_dump(file_path, fig_path, image_name_root, n_chips, args):
    import matplotlib.pyplot as plt
    import numpy as np
    from scipy.interpolate import CubicSpline

    dump = read_acquisition_dump(file_path, n_chips, args.plot_positive_acqs)
    if dump is None:
        return False

    (
        acq_grid,
        n_fft,
        d_max,
        f_max,
        doppler_step,
        doppler_max,
        freq,
        delay,
    ) = dump

    fig = plt.figure()
    plt.gcf().canvas.manager.set_window_title(str(file_path))
    if not args.lite_view:
        ax = fig.add_subplot(111, projection="3d")
        x_axis, y_axis = np.meshgrid(freq, delay)
        ax.plot_surface(x_axis, y_axis, acq_grid, cmap="viridis")
        ax.set_ylim([min(delay), max(delay)])
    else:
        delay_interp = (
            np.arange(args.samples_per_chip * n_chips) / args.samples_per_chip
        )
        spline = CubicSpline(delay, acq_grid)
        grid_interp = spline(delay_interp)
        ax = fig.add_subplot(111, projection="3d")
        x_axis, y_axis = np.meshgrid(freq, delay_interp)
        ax.plot_surface(x_axis, y_axis, grid_interp, cmap="inferno")
        ax.set_ylim([min(delay_interp), max(delay_interp)])

    ax.set_xlabel("Doppler shift (Hz)")
    ax.set_xlim([min(freq), max(freq)])
    ax.set_ylabel("Code delay (chips)")
    ax.set_zlabel("Test Statistics")
    plt.tight_layout()
    plt.savefig(fig_path / f"{image_name_root}_3D.{args.output_format}")
    if not args.show:
        plt.close(fig)

    fig2, axes = plt.subplots(2, 1, figsize=(8, 6))
    plt.gcf().canvas.manager.set_window_title(str(file_path))
    axes[0].plot(freq, acq_grid[d_max, :])
    axes[0].set_xlim([min(freq), max(freq)])
    axes[0].set_xlabel("Doppler shift (Hz)")
    axes[0].set_ylabel("Test statistics")
    axes[0].set_title(
        f"Fixed code delay to {(d_max - 1) / n_fft * n_chips} chips"
    )

    normalization = (args.samples_per_code ** 4) * args.input_power
    axes[1].plot(delay, acq_grid[:, f_max] / normalization)
    axes[1].set_xlim([min(delay), max(delay)])
    axes[1].set_xlabel("Code delay (chips)")
    axes[1].set_ylabel("Test statistics")
    axes[1].set_title(
        f"Doppler wipe-off = {(f_max - 1) * doppler_step - doppler_max} Hz"
    )

    plt.tight_layout()
    plt.savefig(fig_path / f"{image_name_root}_2D.{args.output_format}")
    if not args.show:
        plt.close(fig2)
    return True


def single_file_path(args):
    sat = args.sat if args.sat is not None else DEFAULT_SAT
    channel = args.channel if args.channel is not None else DEFAULT_CHANNEL
    execution = (
        args.execution if args.execution is not None else DEFAULT_EXECUTION
    )
    signal_type = (
        args.signal_type
        if args.signal_type is not None
        else DEFAULT_SIGNAL_TYPE
    )
    signal = signal_type
    system, n_chips = SIGNAL_TYPES[signal_type]
    directory, base = resolve_dump_prefix(args.file_prefix, args.input_path)
    filename = (
        f"{base}_{system}_{signal}_ch_{channel}_"
        f"{execution}_sat_{sat}.mat"
    )
    image_name_root = image_name_from_metadata(
        args.output_basename,
        {
            "system": system,
            "signal": signal,
            "channel": channel,
            "execution": execution,
            "sat": sat,
        },
    )
    return directory / filename, image_name_root, n_chips


def image_name_from_metadata(output_basename, metadata):
    prefix = f"{output_basename}_" if output_basename else ""
    return (
        f"{prefix}{metadata['system']}_{metadata['signal']}_ch_"
        f"{metadata['channel']}_{metadata['execution']}_sat_{metadata['sat']}"
    )


def metadata_matches_filters(metadata, args):
    if args.sat is not None and metadata["sat"] != args.sat:
        return False
    if args.channel is not None and metadata["channel"] != args.channel:
        return False
    if args.execution is not None and metadata["execution"] != args.execution:
        return False
    if args.signal_type is not None:
        system, _ = SIGNAL_TYPES[args.signal_type]
        if metadata["system"] != system or metadata["signal"] != args.signal_type:
            return False
    return True


def metadata_matches_plan(metadata, plan):
    if plan.signal is None:
        return True
    return metadata["system"] == plan.system and metadata["signal"] == plan.signal


def show_figures(args):
    # Display every saved figure with a single plt.show() call. Showing once
    # (instead of once per figure) keeps the GUI event loop to a single entry
    # and avoids closing figures afterwards, which prevents the segfault some
    # macOS matplotlib backends hit on repeated show()/close() cycles. The
    # figures stay open until the user closes all windows, then the process
    # exits and the interpreter tears matplotlib down cleanly.
    if not args.show:
        return
    import matplotlib.pyplot as plt

    plt.show()


def main():
    args = parse_args()
    args.fig_path.mkdir(parents=True, exist_ok=True)
    apply_publication_style()

    if not args.plot_all_files:
        file_path, image_name_root, n_chips = single_file_path(args)
        if not file_path.exists():
            raise FileNotFoundError(f"dump file not found: {file_path}")
        plot_dump(file_path, args.fig_path, image_name_root, n_chips, args)
        show_figures(args)
        return

    plotted = 0
    found_paths = set()
    handled_paths = set()
    searched_prefixes = []
    for plan in acquisition_plans_for_args(args):
        directory, base = resolve_dump_prefix(plan.file_prefix, args.input_path)
        searched_prefixes.append((directory, base))
        file_paths = sorted(directory.glob(f"{base}*.mat"))
        for file_path in file_paths:
            found_paths.add(file_path)
            if file_path in handled_paths:
                continue

            metadata = parse_dump_name(file_path)
            if metadata is None:
                print(f"Skipping {file_path.name}: unexpected filename format")
                continue

            if not metadata_matches_plan(metadata, plan):
                continue

            if not metadata_matches_filters(metadata, args):
                continue

            handled_paths.add(file_path)
            n_chips = N_CHIPS.get((metadata["system"], metadata["signal"]))
            if n_chips is None:
                print(
                    f"Unknown system/signal {metadata['system']}/"
                    f"{metadata['signal']}, skipping: {file_path.name}"
                )
                continue

            image_name_root = image_name_from_metadata(
                args.output_basename,
                metadata,
            )
            if plot_dump(file_path, args.fig_path, image_name_root, n_chips, args):
                plotted += 1

    if not found_paths:
        for directory, base in searched_prefixes:
            print(f"No files matching {base}*.mat in {directory}")
        directories = sorted(
            {directory for directory, _ in searched_prefixes},
            key=str,
        )
        prefixes = []
        for directory in directories:
            prefixes.extend(matching_dump_prefixes(directory))
        prefixes = sorted(set(prefixes))
        if prefixes:
            print("Available acquisition dump prefixes:")
            for prefix in prefixes:
                print(f"  {prefix}")
            print(
                "Use --file-prefix to select the input dump prefix. "
                "--output-basename only changes saved plot filenames."
            )
    elif plotted == 0:
        print("No acquisition dumps were plotted.")

    show_figures(args)


if __name__ == "__main__":
    try:
        main()
    except OSError as exc:
        raise SystemExit(f"Error: {exc}")
