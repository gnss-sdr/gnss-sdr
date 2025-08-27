
"""
vtl_plot_data.py

 CLI entry-point to read GNSS-SDR Tracking dump binary file using the provided function and
 plot internal variables

 Pedro Pereira, 2025. pereirapedro@gmail.com

 Example
 --------
 python vtl_plot_data.py --dump /path/to/vtl_dump.dat --channels 20 \
  --galileo 1 --gt 4729369.07 -705574.36 4207012.59 \
  --fig-dir ./PLOTS --groups pos vel clock residuals meas sv activity \
  --interactive --no-show

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import argparse
from typing import Dict, Any
from lib.vtl_read_dump import vtl_read_dump
from lib.plotVTL import plotVTL


def parse_args():
    p = argparse.ArgumentParser(description="Plot GNSS-SDR VTL dump.")
    p.add_argument("--dump", required=True, help="Path to vtl_dump.dat")
    p.add_argument("--channels", type=int, required=True, help="Number of channels in the dump")
    p.add_argument("--galileo", type=int, default=1, choices=[0,1], help="GAL enabled flag recorded in dump")
    p.add_argument("--fig-dir", default="./PLOTS", help="Directory to save figures")
    p.add_argument("--gt", type=float, nargs=3, metavar=("X","Y","Z"),
                   help="Ground-truth ECEF (m): X Y Z")
    p.add_argument("--groups", nargs="+",
                   default=["pos","vel","clock","residuals","meas","sv","activity"],
                   help="Which groups to plot. Use any of: pos vel clock residuals meas sv activity all")
    p.add_argument("--no-save", action="store_true", help="Do not save figures")
    p.add_argument("--show", action="store_true", help="Show aggregate matplotlib figures")
    return p.parse_args()


def main():
    args = parse_args()

    settings: Dict[str, Any] = {
        "numberOfChannels": args.channels,
        "numberOfStates": 8 + (1 if args.galileo == 1 else 0),
        "GAL_en": args.galileo,
        "dump_path": args.dump,
        "fig_path": args.fig_dir,
    }
    if args.gt is not None:
        settings["gt_rx_p_x"], settings["gt_rx_p_y"], settings["gt_rx_p_z"] = args.gt

    # Read dump
    G = vtl_read_dump(settings)

    # Plot
    plotVTL(G, settings,
            groups=args.groups,
            save=not args.no_save,
            show=args.show)


if __name__ == "__main__":
    main()
