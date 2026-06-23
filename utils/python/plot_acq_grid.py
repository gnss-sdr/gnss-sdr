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
from pathlib import Path

from lib.dump_filename import resolve_dump_prefix
from lib.plot_format import add_output_format_argument, apply_publication_style

# Heavy numerical and plotting libraries (h5py, numpy, matplotlib, scipy) are
# imported lazily inside the functions that need them. This keeps --help and
# argument-error invocations fast and free of matplotlib font-cache warnings.


# GNSS-SDR signal nomenclature: each two-character signal code maps to its
# system character (as written in the dump filename) and primary code length
# in chips. The signal code matches the value used in GNSS-SDR configuration
# (e.g. Galileo.implementation signals) and in the dump filename.
SIGNAL_TYPES = {
    "1C": ("G", 1023),  # GPS L1 C/A
    "2S": ("G", 10230),  # GPS L2C
    "L5": ("G", 10230),  # GPS L5
    "1B": ("E", 4092),  # Galileo E1B
    "5X": ("E", 10230),  # Galileo E5a
    "7X": ("E", 10230),  # Galileo E5b
    "E6": ("E", 5115),  # Galileo E6
    "1G": ("R", 511),  # GLONASS L1
    "2G": ("R", 511),  # GLONASS L2
    "B1": ("C", 2046),  # BeiDou B1I
    "B3": ("C", 10230),  # BeiDou B3I
    "J1": ("J", 1023),  # QZSS L1 C/A
    "J5": ("J", 10230),  # QZSS L5
}
N_CHIPS = {(system, code): nc for code, (system, nc) in SIGNAL_TYPES.items()}
DEFAULT_SAT = 1
DEFAULT_CHANNEL = 0
DEFAULT_EXECUTION = 1
DEFAULT_SIGNAL_TYPE = "1C"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot GNSS-SDR acquisition dump grids."
    )
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
        default="acquisition",
        help="GNSS-SDR Acquisition.dump_filename value (default: "
        "acquisition). May include a directory and extension; matching "
        "<prefix>_<sys>_<sig>_ch_<ch>_<exec>_sat_<PRN>.mat files are read, "
        "resolved against --input-path.",
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
        "all-files mode; defaults to 1C (GPS L1 C/A) in single-file mode. "
        "Choices: " + ", ".join(sorted(SIGNAL_TYPES)) + ".",
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
    if args.output_basename is None:
        # Default the saved-plot basename to the dump basename, dropping any
        # directory and extension carried by --file-prefix.
        _, args.output_basename = resolve_dump_prefix(args.file_prefix)
    return args


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
    directory, base = resolve_dump_prefix(args.file_prefix, args.input_path)
    file_paths = sorted(directory.glob(f"{base}*.mat"))
    for file_path in file_paths:
        metadata = parse_dump_name(file_path)
        if metadata is None:
            print(f"Skipping {file_path.name}: unexpected filename format")
            continue

        if not metadata_matches_filters(metadata, args):
            continue

        n_chips = N_CHIPS.get((metadata["system"], metadata["signal"]))
        if n_chips is None:
            print(
                f"Unknown system/signal {metadata['system']}/"
                f"{metadata['signal']}, skipping: {file_path.name}"
            )
            continue

        image_name_root = image_name_from_metadata(args.output_basename, metadata)
        if plot_dump(file_path, args.fig_path, image_name_root, n_chips, args):
            plotted += 1

    if not file_paths:
        print(f"No files matching {base}*.mat in {directory}")
        prefixes = matching_dump_prefixes(directory)
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
