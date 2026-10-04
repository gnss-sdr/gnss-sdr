#!/usr/bin/env python3
"""
 gps_l1_ca_pvt_raw_plot_sample.py

 Reads a GNSS-SDR PVT raw dump and plots navigation/position diagnostics.

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import argparse
from pathlib import Path

import numpy as np

from lib.dump_filename import resolve_dump_prefix
from lib.gnss_sdr_conf import (
    ConfigError,
    add_conf_argument,
    load_gnss_sdr_conf,
)
from lib.plot_format import add_output_format_argument, apply_publication_style

DEFAULT_FILE_PREFIX = "PVT.dat"
DEFAULT_NAV_SOL_PERIOD = 10.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot GNSS-SDR GPS L1 C/A PVT raw dump data."
    )
    add_conf_argument(parser)
    parser.add_argument(
        "-i",
        "--input-path",
        type=Path,
        default=Path("."),
        help="Directory containing the PVT dump (default: .).",
    )
    parser.add_argument(
        "--file-prefix",
        default=None,
        help="GNSS-SDR PVT.dump_filename value. May include a directory and "
        "extension; the matching <prefix>.dat file is read, resolved against "
        "--input-path. Defaults to PVT.dump_filename from --conf, or PVT.dat.",
    )
    parser.add_argument(
        "-o",
        "--fig-path",
        type=Path,
        default=Path("plots/pvt"),
        help="Directory where plots are saved.",
    )
    parser.add_argument(
        "--nav-sol-period",
        type=float,
        default=None,
        help="Navigation solution period in milliseconds. Defaults to "
        "PVT.output_rate_ms from --conf, or 10.0.",
    )
    parser.add_argument(
        "--true-position",
        type=float,
        nargs=3,
        metavar=("E_UTM", "N_UTM", "U_UTM"),
        default=[np.nan, np.nan, np.nan],
        help="Reference receiver position in UTM coordinates.",
    )
    parser.add_argument(
        "--plot-skyplot",
        action="store_true",
        help="Placeholder: the sky plot panel is not available from a PVT "
        "dump (no azimuth/elevation data). Use utils/skyplot/skyplot.py "
        "instead.",
    )
    parser.add_argument(
        "--no-position",
        dest="plot_position",
        action="store_false",
        help="Skip the position/map diagnostic plot.",
    )
    parser.add_argument(
        "--one-vs-time",
        action="append",
        default=None,
        help="navSolutions variable to plot versus time. Can be repeated.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display figures interactively after saving them.",
    )
    parser.add_argument(
        "--open-maps",
        action="store_true",
        help="Open generated folium map HTML files in a browser.",
    )
    add_output_format_argument(parser)
    parser.set_defaults(plot_position=True)
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
            conf.pvt_dump_filename
            if conf is not None and conf.pvt_dump_filename
            else DEFAULT_FILE_PREFIX
        )

    if args.nav_sol_period is None:
        args.nav_sol_period = (
            conf.pvt_output_rate_ms
            if conf is not None and conf.pvt_output_rate_ms is not None
            else DEFAULT_NAV_SOL_PERIOD
        )


def add_utm_coordinates(nav_solutions):
    try:
        from pyproj import CRS, Transformer
        from pyproj.aoi import AreaOfInterest
        from pyproj.database import query_utm_crs_info
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "gps_l1_ca_pvt_raw_plot_sample.py requires pyproj for PVT "
            "coordinate conversion."
        ) from exc

    input_projection = CRS.from_epsg(4326)
    e_utm = []
    n_utm = []
    for longitude, latitude in zip(
        nav_solutions["longitude"], nav_solutions["latitude"]
    ):
        utm_crs_info = query_utm_crs_info(
            datum_name="WGS 84",
            area_of_interest=AreaOfInterest(
                west_lon_degree=longitude,
                south_lat_degree=latitude,
                east_lon_degree=longitude,
                north_lat_degree=latitude,
            ),
        )
        if utm_crs_info:
            output_projection = CRS.from_epsg(utm_crs_info[0].code)
        else:
            zone = int((longitude + 180) // 6) + 1
            epsg = (32600 if latitude >= 0 else 32700) + zone
            output_projection = CRS.from_epsg(epsg)

        transformer = Transformer.from_crs(
            input_projection, output_projection, always_xy=True)
        east, north = transformer.transform(longitude, latitude)
        e_utm.append(east)
        n_utm.append(north)

    nav_solutions["E_UTM"] = e_utm
    nav_solutions["N_UTM"] = n_utm
    nav_solutions["U_UTM"] = nav_solutions["Z"]


def main():
    args = parse_args()
    args.fig_path.mkdir(parents=True, exist_ok=True)
    apply_publication_style()

    from lib.gps_l1_ca_read_pvt_dump import gps_l1_ca_read_pvt_dump
    from lib.plotNavigation import plotNavigation
    from lib.plotPosition import plot_oneVStime, plot_position

    settings = {
        "true_position": {
            "E_UTM": args.true_position[0],
            "N_UTM": args.true_position[1],
            "U_UTM": args.true_position[2],
        },
        "navSolPeriod": args.nav_sol_period,
        "fig_path": args.fig_path,
        "show": args.show,
        "output_format": args.output_format,
    }

    directory, base = resolve_dump_prefix(args.file_prefix, args.input_path)
    pvt_file = directory / f"{base}.dat"
    nav_solutions = gps_l1_ca_read_pvt_dump(pvt_file)
    add_utm_coordinates(nav_solutions)

    plotNavigation(nav_solutions, settings, int(args.plot_skyplot))

    if args.plot_position:
        plot_position(
            nav_solutions,
            fig_path=args.fig_path,
            show=args.show,
            open_maps=args.open_maps,
            output_format=args.output_format,
        )

    one_vs_time = args.one_vs_time or ["X_vel", "Tot_Vel"]
    for variable_name in one_vs_time:
        plot_oneVStime(
            nav_solutions,
            variable_name,
            fig_path=args.fig_path,
            show=args.show,
            output_format=args.output_format,
        )

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
