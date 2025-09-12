#!/usr/bin/env python
"""
osnma_log_viewer.py

Generate a Galileo navigation message authentication timeline plot from a
GNSS-SDR log file.

-----------------------------------------------------------------------------

GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
This file is part of GNSS-SDR.

SPDX-FileCopyrightText: 2025 Carles Fernandez-Prades cfernandez(at)cttc.es
SPDX-License-Identifier: GPL-3.0-or-later

-----------------------------------------------------------------------------
"""

import argparse
import datetime
import re
import sys
import os
from pathlib import Path

try:
    import pandas as pd
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import matplotlib.dates as mdates
except ImportError:
    print("Error: This script requires matplotlib and pandas.")
    print("Install them with: pip install matplotlib pandas")
    sys.exit(1)

__version__ = "1.0.0"
GALILEO_EPOCH = datetime.datetime(1999, 8, 22, tzinfo=datetime.timezone.utc)
GST_UTC_OFFSET = 18  # seconds


def galileo_to_utc(tow, week, gst_utc_offset=GST_UTC_OFFSET):
    """Convert Galileo week + TOW to UTC datetime."""
    gst_time = GALILEO_EPOCH + datetime.timedelta(weeks=week, seconds=tow)
    # Convert GST to UTC
    return gst_time - datetime.timedelta(seconds=gst_utc_offset)


def parse_osnma_log(log_file):
    """Parse GNSS-SDR OSNMA log lines to extract authentication status.
    Returns:
        DataFrame with columns: ["TOW", "WN", "PRNd", "PRNa", "ADKD", "UTC"]
    """
    tow_pattern = re.compile(r"TOW=(\d+)")
    prnd_pattern = re.compile(r"PRNd=(\d+)")
    prna_pattern = re.compile(r"PRNa=(\d+)")
    adkd_pattern = re.compile(r"ADKD=(\d+)")
    success_pattern = re.compile(r"Tag verification :: SUCCESS")
    wn_pattern = re.compile(r"WN=(\d+)")
    ls_pattern = re.compile(r"Delta_tLS=(\d+)")

    leap_second = GST_UTC_OFFSET
    records = []
    wn = None

    with open(log_file, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if "OSNMA" in line:
                wn_match = wn_pattern.search(line)
                if wn_match:
                    wn = int(wn_match.group(1))

                tow_match = tow_pattern.search(line)
                prnd_match = prnd_pattern.search(line)
                prna_match = prna_pattern.search(line)
                success_match = success_pattern.search(line)
                adkd_match = adkd_pattern.search(line)

                if tow_match and prnd_match and success_match and adkd_match:
                    tow = int(tow_match.group(1))
                    prnd = int(prnd_match.group(1))
                    prna = int(prna_match.group(1))
                    adkd = int(adkd_match.group(1))
                    if wn is not None:
                        utc = galileo_to_utc(tow, wn, leap_second)
                        if adkd == 12:
                            tow -= 300
                        if adkd in (0, 4):
                            tow -= 30
                        records.append((tow, wn, prnd, prna, adkd, utc))

            elif "Galileo leap second" in line:
                ls_match = ls_pattern.search(line)
                if ls_match:
                    leap_second = int(ls_match.group(1))

    df = pd.DataFrame(records, columns=[
                      "TOW", "WN", "PRNd", "PRNa", "ADKD", "UTC"])
    df.sort_values(by=["WN", "TOW"], inplace=True)
    return df


def plot_authentication_bars(
    df, output_file="osnma_auth_timeline.pdf", show_plot=True, use_localtime=False
):
    """Plot horizontal bars for Galileo satellites E01-E36."""
    BAR_DURATION_SEC = 30
    SECS_TO_CHANGE_STYLE = 3600 * 2
    BAR_DURATION_DAY = BAR_DURATION_SEC / (24 * 3600)
    satellites = list(range(1, 37))
    y_labels = [f"E{prn:02d}" for prn in satellites]

    global_start_time = df["UTC"].iloc[0]
    global_end_time = df["UTC"].iloc[-1]
    global_time_span_secs = (
        global_end_time - global_start_time).total_seconds()

    plt.rcParams["pdf.fonttype"] = 42  # TrueType fonts
    plt.rcParams["ps.fonttype"] = 42  # TrueType fonts
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = ["Times New Roman", "Times", "DejaVu Serif"]
    plt.rcParams["mathtext.fontset"] = "dejavuserif"  # For math text
    plt.rcParams["savefig.dpi"] = 300  # for jpg and png
    plt.rcParams["savefig.bbox"] = "tight"  # Always use bbox_inches='tight'
    plt.rcParams["svg.fonttype"] = "none"  # Make SVG text editable
    fig, ax = plt.subplots(figsize=(14, 8.75))

    for idx, prn in enumerate(satellites):
        sat_data = df[df["PRNd"] == prn]
        if sat_data.empty:
            continue

        utc_times = sat_data["UTC"].tolist()
        matplot_dates = [mdates.date2num(dt) for dt in utc_times]
        prna = sat_data["PRNa"].tolist()
        adkd = sat_data["ADKD"].tolist()

        for i in range(len(matplot_dates) - 1):
            start = matplot_dates[i]
            end = start + BAR_DURATION_DAY

            color = "tab:green" if prna[i] == prn else "tab:blue"

            if global_time_span_secs < SECS_TO_CHANGE_STYLE:
                if adkd[i] == 0:
                    hatch = "\\\\\\\\"
                elif adkd[i] == 4:
                    hatch = "////"
                elif adkd[i] == 12:
                    hatch = "----"
                else:
                    hatch = ""
                ax.barh(
                    idx,
                    end - start,
                    left=start,
                    color=color,
                    hatch=hatch,
                    edgecolor="black",
                    height=0.6,
                    alpha=0.5,
                )
            else:
                ax.barh(
                    idx,
                    end - start,
                    left=start,
                    color=color,
                    edgecolor=color,
                    height=0.6,
                )

    # Define x-axis label with UTC offset if needed
    if use_localtime:
        offset = datetime.datetime.now().astimezone().utcoffset()
        if offset is None:
            xlabel = "Local Time (UTC)"
        else:
            total_minutes = int(offset.total_seconds() // 60)
            if total_minutes == 0:
                xlabel = "Local Time"
            else:
                hours, minutes = divmod(abs(total_minutes), 60)
                sign = "+" if total_minutes > 0 else "-"
                if minutes == 0:
                    offset_str = f"{sign}{hours}h"
                else:
                    offset_str = f"{sign}{hours}h{minutes:02d}m"
                xlabel = f"Local Time (UTC {offset_str})"
    else:
        xlabel = "UTC Time"

    # Labels and title
    ax.set_yticks(range(len(satellites)))
    ax.set_yticklabels(y_labels, fontsize=14)
    ax.set_ylabel("Galileo satellites", fontsize=18)
    ax.set_xlabel(xlabel, fontsize=18)
    ax.set_title(
        "Galileo navigation message authentication timeline", pad=25, fontsize=20
    )

    # Format x-axis as dates
    ax.xaxis_date()
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%Y-%m-%d\n%H:%M:%S"))
    ax.tick_params(axis="x", labelsize=14)
    fig.autofmt_xdate()

    # Legend
    if global_time_span_secs < SECS_TO_CHANGE_STYLE:
        legend_patches = [
            mpatches.Patch(
                facecolor="tab:green",
                edgecolor="black",
                hatch="\\\\\\\\",
                alpha=0.5,
                label="Self-authenticated ADKD=0",
            ),
            mpatches.Patch(
                facecolor="tab:green",
                edgecolor="black",
                hatch="////",
                alpha=0.5,
                label="Self-authenticated ADKD=4",
            ),
            mpatches.Patch(
                facecolor="tab:green",
                edgecolor="black",
                hatch="---",
                alpha=0.5,
                label="Self-authenticated ADKD=12",
            ),
            mpatches.Patch(
                facecolor="tab:blue",
                edgecolor="black",
                hatch="\\\\\\\\",
                alpha=0.5,
                label="Cross-authenticated ADKD=0",
            ),
            mpatches.Patch(
                facecolor="tab:blue",
                edgecolor="black",
                hatch="////",
                alpha=0.5,
                label="Cross-authenticated ADKD=4",
            ),
            mpatches.Patch(
                facecolor="tab:blue",
                edgecolor="black",
                hatch="---",
                alpha=0.5,
                label="Cross-authenticated ADKD=12",
            ),
        ]
    else:
        legend_patches = [
            mpatches.Patch(color="tab:green", alpha=0.5,
                           label="Self-authenticated"),
            mpatches.Patch(color="tab:blue", alpha=0.5,
                           label="Cross-authenticated"),
        ]

    ax.legend(handles=legend_patches, loc="upper right", fontsize=18)
    ax.grid(True, axis="x", linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(output_file)
    print(f"Plot saved to {output_file}")
    if show_plot:
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nExecution interrupted by the user. Exiting.")
            plt.close()
    else:
        plt.close()


def main():
    """Generate the OSNMA authentication timeline plot"""
    try:
        parser = argparse.ArgumentParser(
            description="Generate a Galileo navigation message authentication timeline plot from a GNSS-SDR log file.",
            epilog="Example: osnma_log_viewer.py gnss-sdr.log --output auth_timeline.png --no-show --localtime",
        )

        parser.add_argument("logfile", type=str, help="GNSS-SDR log file path")

        parser.add_argument(
            "--localtime",
            action="store_true",
            help="Display results in local time instead of UTC",
        )

        parser.add_argument(
            "--no-show",
            action="store_true",
            help="Run without displaying the plot window",
        )

        parser.add_argument(
            "-o",
            "--output",
            type=str,
            default="osnma_auth_timeline.pdf",
            help="Output file for plot (default: osnma_auth_timeline.pdf)",
        )

        parser.add_argument(
            "--start", type=str, help='Initial datetime in "YYYY-MM-DD HH:MM:SS" format'
        )

        parser.add_argument(
            "--end", type=str, help='Final datetime in "YYYY-MM-DD HH:MM:SS" format'
        )

        parser.add_argument(
            "-v",
            "--version",
            action="version",
            version=f"%(prog)s {__version__}",
            help="Show program version and exit",
        )

        args = parser.parse_args()

        log_file = Path(args.logfile)
        if not log_file.exists():
            print(f"Log file {log_file} not found. Exiting.")
            sys.exit(1)

        valid_extensions = {".pdf", ".png", ".svg", ".eps", ".jpg", ".jpeg"}
        _, ext = os.path.splitext(args.output)
        if ext.lower() not in valid_extensions:
            print(f"Error: Output file '{args.output}' has invalid extension.")
            print(f"Supported extensions: {', '.join(valid_extensions)}")
            sys.exit(1)

        print(f"Reading file {log_file} ...")
        df = parse_osnma_log(log_file)
        if df.empty:
            print("No OSNMA authentication records found in the log file.")
            sys.exit(0)

        successful_total_tags = df.shape[0]
        print(f"Found {successful_total_tags} validated OSNMA tags.")

        # Read time limits
        start_datetime = None
        end_datetime = None
        if args.start:
            try:
                start_datetime = datetime.datetime.strptime(
                    args.start, "%Y-%m-%d %H:%M:%S"
                )
            except ValueError:
                print(
                    'Invalid datetime format for argument --start. Please use "YYYY-MM-DD HH:MM:SS" using double quotes. Ignoring this argument.'
                )
                start_datetime = None
        if args.end:
            try:
                end_datetime = datetime.datetime.strptime(
                    args.end, "%Y-%m-%d %H:%M:%S"
                )
            except ValueError:
                print(
                    'Invalid datetime format for argument --end. Please use "YYYY-MM-DD HH:MM:SS" using double quotes. Ignoring this argument.'
                )
                end_datetime = None

        # Convert data to local time if requested
        if args.localtime:
            df["UTC"] = pd.to_datetime(df["UTC"], utc=True)
            df["UTC"] = df["UTC"].dt.tz_convert(
                tz=datetime.datetime.now().astimezone().tzinfo
            )

        # drop tz info for matplotlib
        df["UTC"] = df["UTC"].dt.tz_localize(None)

        # Apply datetime filtering to data if parameters are provided
        if start_datetime is not None and end_datetime is not None:
            if start_datetime >= end_datetime:
                print("Error: --start datetime must be earlier than --end datetime")
                sys.exit(1)
            else:
                df = df[(df["UTC"] >= start_datetime)
                        & (df["UTC"] <= end_datetime)]
        elif start_datetime is not None:
            df = df[df["UTC"] >= start_datetime]
        elif end_datetime is not None:
            df = df[df["UTC"] <= end_datetime]

        if df.empty:
            print(
                "No OSNMA authentication records found in the log file for the specified time period."
            )
            sys.exit(0)

        tags_in_period = df.shape[0]
        if successful_total_tags != tags_in_period:
            print(f"A total of {tags_in_period} tags will be plotted for the specified time period.")

        print(
            "Generating Galileo's navigation message authentication timeline plot ..."
        )
        plot_authentication_bars(
            df,
            output_file=args.output,
            show_plot=not args.no_show,
            use_localtime=args.localtime,
        )
    except Exception as e:
        print(f"Error: {str(e)}")
        return 1
    return 0


if __name__ == "__main__":
    main()
