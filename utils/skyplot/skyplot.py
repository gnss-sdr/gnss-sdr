#!/usr/bin/env python
"""
 skyplot.py

 Reads a RINEX navigation file and generates a skyplot. Optionally, a RINEX observation file can 
 also be read to match the skyplot to the receiver processing time.

 Usage: python skyplot.py <RINEX_NAV_FILE> [observer_lat] [observer_lon] [observer_alt] [--use-obs] 

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 SPDX-FileCopyrightText: 2025 Carles Fernandez-Prades cfernandez(at)cttc.es
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import argparse
import re
import sys
from datetime import datetime, timedelta
from math import atan2, cos, sin, sqrt
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _read_obs_time_bounds(obs_path):
    """Return (start_time, end_time) from a RINEX 2/3 OBS file by scanning epoch lines.
    If parsing fails or the file is not OBS, return (None, None).
    """
    try:
        with open(obs_path, 'r', encoding='utf-8', errors='ignore') as f:
            # Detect OBS in header and skip to END OF HEADER
            is_obs = False
            for line in f:
                if "RINEX VERSION / TYPE" in line:
                    # Robust OBS detection:
                    # In RINEX 2/3 the file-type letter at col 21 (0-based idx 20)
                    tchar = line[20:21]
                    if tchar == 'O' or 'OBSERVATION DATA' in line:
                        is_obs = True
                if "END OF HEADER" in line:
                    break
            if not is_obs:
                return (None, None)

            start_time = None
            end_time = None

            for line in f:
                if not line.strip():
                    continue
                if line.startswith('>'):  # RINEX 3 epoch line
                    yyyy = int(line[2:6]); mm = int(line[7:9]); dd = int(line[10:12])
                    hh = int(line[13:15]); mi = int(line[16:18]); ss = float(line[19:29])
                    epoch = datetime(yyyy, mm, dd, hh, mi, int(ss), int((ss % 1)*1e6))
                else:
                    # RINEX 2 epoch line
                    try:
                        yy = int(line[1:3]); mm = int(line[4:6]); dd = int(line[7:9])
                        hh = int(line[10:12]); mi = int(line[13:15]); ss = float(line[15:26])
                        yyyy = 1900 + yy if yy >= 80 else 2000 + yy
                        epoch = datetime(yyyy, mm, dd, hh, mi, int(ss), int((ss % 1)*1e6))
                    except Exception:
                        continue
                if start_time is None or epoch < start_time:
                    start_time = epoch
                if end_time is None or epoch > end_time:
                    end_time = epoch
            return (start_time, end_time)
    except Exception:
        return (None, None)


def parse_rinex_float(s):
    """Parse RINEX formatted float string which may contain D or E exponent and compact spacing"""
    # Handle empty string
    if not s.strip():
        return 0.0

    # Replace D exponent with E (some RINEX files use D instead of E)
    s = s.replace('D', 'E').replace('d', 'e')

    # Handle cases where exponent lacks E (e.g., "12345-3")
    if re.match(r'[+-]?\d+[+-]\d+', s.strip()):
        s = s.replace('+', 'E+').replace('-', 'E-')

    try:
        return float(s)
    except ValueError:
        # Handle cases where the number runs into the next field
        # Try to split at the exponent if present
        if 'E' in s:
            base, exp = s.split('E')[:2]
            # Take first character of exponent if needed
            if exp and exp[0] in '+-' and len(exp) > 1:
                return float(base + 'E' + exp[0] + exp[1:].split()[0])
        return 0.0  # Default if parsing fails


def read_rinex_nav(filename):
    """Read RINEX v3.0 navigation file"""
    satellites = {}
    line_number = 0
    with open(filename, 'r', encoding='utf-8') as f:
        # Skip header
        while True:
            line = f.readline()
            line_number += 1
            if not line:
                return satellites  # Empty file
            if "END OF HEADER" in line:
                break

        # Read ephemeris data
        current_line = f.readline()
        line_number += 1
        while current_line:
            if len(current_line) < 23:
                current_line = f.readline()
                line_number += 1
                continue

            prn = current_line[:3].strip()
            system = prn[0]  # G, R, E, etc.

            try:
                # Parse epoch fields
                year = int(current_line[4:8])
                month = int(current_line[9:11])
                day = int(current_line[12:14])
                hour = int(current_line[15:17])
                minute = int(current_line[18:20])
                second = int(float(current_line[21:23]))

                year += 2000 if year < 80 else 0
                epoch = datetime(year, month, day, hour, minute, second)

                # Read the next lines
                lines = [current_line]
                line_count = 4 if system == 'R' else 7
                for _ in range(line_count):
                    next_line = f.readline()
                    line_number += 1
                    if not next_line:
                        break
                    lines.append(next_line)

                if len(lines) < line_count + 1:
                    current_line = f.readline()
                    line_number += 1
                    continue

                if system == 'R':  # GLONASS specific parsing
                    ephemeris = {
                        'prn': prn,
                        'epoch': epoch,
                        'sv_clock_bias': parse_rinex_float(lines[0][23:42]),
                        'sv_relative_freq_bias': parse_rinex_float(lines[0][42:61]),
                        'message_frame_time': parse_rinex_float(lines[0][61:80]),
                        'x': parse_rinex_float(lines[1][4:23]),  # Position (km)
                        'x_vel': parse_rinex_float(lines[1][23:42]),  # Velocity (km/s)
                        'x_acc': parse_rinex_float(lines[1][42:61]),
                        'health': parse_rinex_float(lines[1][61:80]),
                        'y': parse_rinex_float(lines[2][4:23]),
                        'y_vel': parse_rinex_float(lines[2][23:42]),
                        'y_acc': parse_rinex_float(lines[2][42:61]),
                        'freq_num': parse_rinex_float(lines[2][61:80]),
                        'z': parse_rinex_float(lines[3][4:23]),
                        'z_vel': parse_rinex_float(lines[3][23:42]),
                        'z_acc': parse_rinex_float(lines[3][42:61]),
                        'age': parse_rinex_float(lines[3][61:80])
                    }
                else:
                    # Parse all ephemeris parameters
                    ephemeris = {
                        'prn': prn,
                        'epoch': epoch,
                        'sv_clock_bias': parse_rinex_float(lines[0][23:42]),
                        'sv_clock_drift': parse_rinex_float(lines[0][42:61]),
                        'sv_clock_drift_rate': parse_rinex_float(lines[0][61:80]),
                        'iode': parse_rinex_float(lines[1][4:23]),
                        'crs': parse_rinex_float(lines[1][23:42]),
                        'delta_n': parse_rinex_float(lines[1][42:61]),
                        'm0': parse_rinex_float(lines[1][61:80]),
                        'cuc': parse_rinex_float(lines[2][4:23]),
                        'ecc': parse_rinex_float(lines[2][23:42]),
                        'cus': parse_rinex_float(lines[2][42:61]),
                        'sqrt_a': parse_rinex_float(lines[2][61:80]),
                        'toe': parse_rinex_float(lines[3][4:23]),
                        'cic': parse_rinex_float(lines[3][23:42]),
                        'omega0': parse_rinex_float(lines[3][42:61]),
                        'cis': parse_rinex_float(lines[3][61:80]),
                        'i0': parse_rinex_float(lines[4][4:23]),
                        'crc': parse_rinex_float(lines[4][23:42]),
                        'omega': parse_rinex_float(lines[4][42:61]),
                        'omega_dot': parse_rinex_float(lines[4][61:80]),
                        'idot': parse_rinex_float(lines[5][4:23]),
                        'codes_l2': parse_rinex_float(lines[5][23:42]),
                        'gps_week': parse_rinex_float(lines[5][42:61]),
                        'l2p_flag': parse_rinex_float(lines[5][61:80]),
                        'sv_accuracy': parse_rinex_float(lines[6][4:23]),
                        'sv_health': parse_rinex_float(lines[6][23:42]),
                        'tgd': parse_rinex_float(lines[6][42:61]),
                        'iodc': parse_rinex_float(lines[6][61:80]),
                        'transmission_time': parse_rinex_float(lines[7][4:23]),
                        'fit_interval': (
                            parse_rinex_float(lines[7][23:42])) if len(lines[7]) > 23 else 0.0
                    }

                if prn not in satellites:
                    satellites[prn] = []
                satellites[prn].append(ephemeris)

            except (ValueError, IndexError) as e:
                print(f"\nError in file {filename} at line {line_number}:")
                print(f"  PRN: {prn}")
                print(f"  Line content: {current_line.strip()}")
                print(f"  Error type: {type(e).__name__}")
                print(f"  Error details: {str(e)}")
                print("Skipping to next satellite block...\n")
                # Skip to next block by reading until next PRN
                while current_line and not current_line.startswith(prn[0]):
                    current_line = f.readline()
                    line_number += 1
                continue

            current_line = f.readline()
            line_number += 1

    return satellites


def calculate_satellite_position(ephemeris, transmit_time):
    """Calculate satellite position in ECEF coordinates at given transmission time"""
    system = ephemeris['prn'][0]

    if system == 'R':  # GLONASS - use position + velocity * time
        dt = transmit_time
        # Convert km to meters
        xk = (ephemeris['x'] + ephemeris['x_vel'] * dt + 0.5 * ephemeris['x_acc'] * dt**2) * 1000
        yk = (ephemeris['y'] + ephemeris['y_vel'] * dt + 0.5 * ephemeris['y_acc'] * dt**2) * 1000
        zk = (ephemeris['z'] + ephemeris['z_vel'] * dt + 0.5 * ephemeris['z_acc'] * dt**2) * 1000
    else:
        # Constants
        mu = 3.986005e14  # Earth's gravitational constant (m^3/s^2)
        omega_e_dot = 7.2921151467e-5  # Earth rotation rate (rad/s)

        # Semi-major axis
        a = ephemeris['sqrt_a'] ** 2

        # Corrected mean motion
        n0 = sqrt(mu / (a ** 3))
        n = n0 + ephemeris['delta_n']

        # Mean anomaly
        mk = ephemeris['m0'] + n * transmit_time

        # Solve Kepler's equation for eccentric anomaly (Ek)
        ek = mk
        for _ in range(10):
            ek_old = ek
            ek = mk + ephemeris['ecc'] * sin(ek)
            if abs(ek - ek_old) < 1e-12:
                break

        # True anomaly
        nu_k = atan2(sqrt(1 - ephemeris['ecc']**2) * sin(ek), cos(ek) - ephemeris['ecc'])

        # Argument of latitude
        phi_k = nu_k + ephemeris['omega']

        # Second harmonic perturbations
        delta_uk = ephemeris['cus'] * sin(2 * phi_k) + ephemeris['cuc'] * cos(2 * phi_k)
        delta_rk = ephemeris['crs'] * sin(2 * phi_k) + ephemeris['crc'] * cos(2 * phi_k)
        delta_ik = ephemeris['cis'] * sin(2 * phi_k) + ephemeris['cic'] * cos(2 * phi_k)

        # Corrected argument of latitude, radius and inclination
        uk = phi_k + delta_uk
        rk = a * (1 - ephemeris['ecc'] * cos(ek)) + delta_rk
        ik = ephemeris['i0'] + delta_ik + ephemeris['idot'] * transmit_time

        # Positions in orbital plane
        xk_prime = rk * cos(uk)
        yk_prime = rk * sin(uk)

        # Corrected longitude of ascending node
        omega_k = (
            ephemeris['omega0']
            + (ephemeris['omega_dot'] - omega_e_dot) * transmit_time
            - omega_e_dot * ephemeris['toe']
        )

        # Earth-fixed coordinates
        xk = xk_prime * cos(omega_k) - yk_prime * cos(ik) * sin(omega_k)
        yk = xk_prime * sin(omega_k) + yk_prime * cos(ik) * cos(omega_k)
        zk = yk_prime * sin(ik)

    return xk, yk, zk


def calculate_satellite_positions(ephemeris, start_time, end_time, step_min=5):
    """Generate multiple positions over time for a single satellite
    between start_time and end_time.
    """
    positions = []
    current_time = start_time
    system = ephemeris['prn'][0]
    max_valid_time = 1800 if system == 'R' else 14400
    while current_time <= end_time:
        transmit_time = (current_time - ephemeris['epoch']).total_seconds()

        if abs(transmit_time) <= max_valid_time:
            x, y, z = calculate_satellite_position(ephemeris, transmit_time)
            positions.append((current_time, x, y, z))

        current_time += timedelta(minutes=step_min)

    return positions


def ecef_to_az_el(x, y, z, obs_lat, obs_lon, obs_alt):
    """Convert ECEF coordinates to azimuth and elevation"""
    # WGS-84 parameters
    a = 6378137.0  # semi-major axis
    e_sq = 6.69437999014e-3  # first eccentricity squared

    # Convert geodetic coordinates to ECEF
    n = a / sqrt(1 - e_sq * sin(obs_lat)**2)
    obs_x = (n + obs_alt) * cos(obs_lat) * cos(obs_lon)
    obs_y = (n + obs_alt) * cos(obs_lat) * sin(obs_lon)
    obs_z = (n * (1 - e_sq) + obs_alt) * sin(obs_lat)

    # Vector from observer to satellite
    dx = x - obs_x
    dy = y - obs_y
    dz = z - obs_z

    # Convert to local ENU (East, North, Up) coordinates
    enu_x = -sin(obs_lon) * dx + cos(obs_lon) * dy
    enu_y = -sin(obs_lat) * cos(obs_lon) * dx - sin(obs_lat) * sin(obs_lon) * dy + cos(obs_lat) * dz
    enu_z = cos(obs_lat) * cos(obs_lon) * dx + cos(obs_lat) * sin(obs_lon) * dy + sin(obs_lat) * dz

    # Calculate azimuth and elevation
    azimuth = atan2(enu_x, enu_y)
    elevation = atan2(enu_z, sqrt(enu_x**2 + enu_y**2))

    # Convert to degrees and adjust azimuth to 0-360
    azimuth = np.degrees(azimuth) % 360
    elevation = np.degrees(elevation)

    return azimuth, elevation


def plot_satellite_tracks(satellites, obs_lat, obs_lon, obs_alt,
                         footer_text=None, filename=None,
                         show_plot=True, start_time=None, 
                         end_time=None):
    """Plot trajectories for all visible satellites"""
    plt.rcParams["font.family"] = "Times New Roman"
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='polar')
    ax.tick_params(labelsize=16, pad=7)

    # Polar plot setup
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_ylim(0, 90)

    # Elevation ticks
    ax.set_yticks(range(0, 91, 15))
    ax.set_yticklabels(['90°', '', '60°', '', '30°', '', '0°'], fontsize=14)

    # Color scheme by constellation
    system_colors = {
        'G': 'blue',    # GPS
        'E': 'green',   # Galileo
        'R': 'red',     # GLONASS
        'C': 'orange',  # BeiDou
        'J': 'brown',   # QZSS
        'I': 'pink',    # IRNSS
        'S': 'gray'     # SBAS
    }

    # System names mapping
    system_names = {
        'G': 'GPS',
        'E': 'Galileo',
        'R': 'GLONASS',
        'C': 'BeiDou',
        'J': 'QZSS',
        'I': 'IRNSS',
        'S': 'SBAS'
    }

    # Find which systems are actually present
    present_systems = {prn[0] for prn in satellites.keys() if prn[0] in system_colors}

    # Plot each satellite
    for prn, ephemeris_list in satellites.items():
        color = system_colors.get(prn[0], 'purple')  # Default to purple for unknown systems

        if not ephemeris_list:
            continue

        mid_time = start_time + (end_time - start_time) / 2
        prev_eph = [e for e in ephemeris_list if e['epoch'] <= mid_time]
        if prev_eph:
            ephemeris = max(prev_eph, key=lambda e: e['epoch'])
        else:
            ephemeris = min(ephemeris_list, key=lambda e: abs((e['epoch'] - mid_time).total_seconds()))

        if start_time is None or end_time is None:
            all_epochs = sorted({e['epoch'] for prn_data in satellites.values() for e in prn_data})
            start_time = min(all_epochs)
            end_time = max(all_epochs)

        positions = calculate_satellite_positions(ephemeris, start_time, end_time)

        # Split into visible segments
        segments = []
        current_seg_az = []
        current_seg_el = []
        for _, x, y, z in positions:
            azimuth, elevation = ecef_to_az_el(x, y, z, obs_lat, obs_lon, obs_alt)
            if elevation > 5:
                current_seg_az.append(azimuth)
                current_seg_el.append(elevation)
            else:
                if len(current_seg_az) > 1:
                    segments.append((current_seg_az, current_seg_el))
                current_seg_az, current_seg_el = [], []
        if len(current_seg_az) > 1:
            segments.append((current_seg_az, current_seg_el))

        # Plot each segment separately
        for az_seg, el_seg in segments:
            theta = np.radians(az_seg)
            r = 90 - np.array(el_seg)
            ax.plot(theta, r, '-', color=color, alpha=0.7, linewidth=2.5)

            # Arrow at end
            if len(theta) >= 2:
                dx = theta[-1] - theta[-2]
                dy = r[-1] - r[-2]
                arrow_length_factor = 1.3
                extended_theta = theta[-2] + dx * arrow_length_factor
                extended_r = r[-2] + dy * arrow_length_factor
                ax.annotate('',
                            xytext=(theta[-1], r[-1]),
                            xy=(extended_theta, extended_r),
                            arrowprops={
                                'arrowstyle': '->',
                                'color': color,
                                'alpha': 0.9,
                                'linewidth': 1.5,
                                'shrinkA': 0,
                                'shrinkB': 0
                            })

            # Label at midpoint of the segment
            mid_idx = len(theta)//2
            ax.text(theta[mid_idx], r[mid_idx], prn,
                    fontsize=12, ha='center', va='center',
                    bbox={"facecolor": "white", "alpha": 0.8, "pad": 2})

    # Legend for present systems
    legend_elements = [
        plt.Line2D([0], [0], marker='o', color='w',
                  label=f'{system_names[sys]} ({sys})',
                  markerfacecolor=system_colors[sys],
                  markersize=10)
        for sys in present_systems
    ]
    if legend_elements:
        ax.legend(handles=legend_elements,
                  loc='upper right',
                  bbox_to_anchor=(1.3, 1.1),
                  fontsize=14)

    lat_deg = np.degrees(obs_lat)
    lon_deg = np.degrees(obs_lon)
    lat_hemisphere = 'N' if lat_deg >= 0 else 'S'
    lon_hemisphere = 'E' if lon_deg >= 0 else 'W'

    plt.title(
        f"GNSS skyplot from {abs(lat_deg):.2f}° {lat_hemisphere}, "
        f"{abs(lon_deg):.2f}° {lon_hemisphere}",
        pad=25,
        fontsize=20
    )

    if footer_text:
        fig.text(0.42, 0.05, footer_text, ha='center', va='center', fontsize=16)

    plt.tight_layout()

    if filename:
        filename_no_path = Path(filename).name
        filename_no_dots = filename_no_path.replace('.', '_')
        output_name = f"skyplot_{filename_no_dots}.pdf"
    else:
        output_name = "skyplot.pdf"

    plt.savefig(output_name, format='pdf', bbox_inches='tight')
    print(f"Image saved as {output_name}")
    if show_plot:
        plt.show()
    else:
        plt.close()


def main():
    """Generate the skyplot"""
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description='Generate GNSS skyplot from RINEX navigation file',
        add_help=False
    )

    # Add the no-show flag
    parser.add_argument(
        '--no-show', 
        action='store_true',
        help='Run without displaying plot window'
    )
    # Add the use-obs flag
    parser.add_argument(
        '--use-obs',
        action='store_true',
        help='Use corresponding RINEX observation file to bound the skyplot to the receiver time window'
    )

    # Parse known args (this ignores other positional args)
    args, remaining_args = parser.parse_known_args()

    # Handle help manually
    if '-h' in remaining_args or '--help' in remaining_args:
        print("""
Usage: python skyplot.py <RINEX_FILE> [LATITUDE] [LONGITUDE] [ALTITUDE] [--use-obs] [--no-show]

Example:
  python skyplot.py brdc0010.22n 41.275 1.9876 80.0 --use-obs --no-show
""")
        sys.exit(0)

    if len(remaining_args) < 1:
        print("Error: RINEX file required")
        sys.exit(1)

    filename = remaining_args[0]

    # Default observer location (Castelldefels, Barcelona, Spain)
    obs_lat = np.radians(41.2750)
    obs_lon = np.radians(1.9876)
    obs_alt = 80.0

    # Override with command line arguments if provided
    if len(remaining_args) >= 4:
        try:
            obs_lat = np.radians(float(remaining_args[1]))
            obs_lon = np.radians(float(remaining_args[2]))
            if len(remaining_args) >= 5:
                obs_alt = float(remaining_args[3])
        except ValueError:
            print("Invalid observer coordinates. Using defaults.")

    # Read RINEX file
    print(f"Reading {filename}...")
    try:
        satellites = read_rinex_nav(filename)
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return

    if not satellites:
        print("No satellite data found in the file.")
        return

    # Print summary information
    all_epochs = sorted(list(set(
        e['epoch'] for prn, ephemerides in satellites.items() for e in ephemerides
    )))
    print("\nFile contains:")
    print(f"- {len(satellites)} unique satellites")
    print(f"- {len(all_epochs)} unique epochs")
    print(f"- From {all_epochs[0]} to {all_epochs[-1]}")

    # Calculate and print satellite counts by system
    system_counts = {}
    for prn in satellites:
        system = prn[0]
        system_counts[system] = system_counts.get(system, 0) + 1

    print("\nSatellite systems found:")
    for system, count in sorted(system_counts.items()):
        system_name = {
            'G': 'GPS',
            'R': 'GLONASS',
            'E': 'Galileo',
            'C': 'BeiDou'
        }.get(system, 'Unknown')
        print(f"- {system_name} ({system}): {count} satellites")

    # Generate the combined skyplot
    # Time window: OBS bounds if provided; else NAV span
    use_start, use_end = all_epochs[0], all_epochs[-1]
    if args.use_obs:
        tried = []
        obs_path = None
        stem = filename[:-1]

        for s in ('O', 'o'):  # Try uppercase then lowercase
            candidate = stem + s
            tried.append(candidate)
            if Path(candidate).exists():
                obs_path = candidate
                break

        if obs_path:
            obs_start, obs_end = _read_obs_time_bounds(obs_path)
            if obs_start and obs_end:
                use_start, use_end = obs_start, obs_end
                print(f"\nObservation window detected from {obs_path}: {use_start} → {use_end}")
            else:
                print(f"\nWarning: Could not read valid times from {obs_path}. Using NAV span instead.")
        else:
            print(f"\nOBS file not found. Tried: {', '.join(tried)}. Using NAV span instead.")

    # Ensure at least two samples with the default 5-minute step
    if (use_end - use_start) < timedelta(minutes=5):
        use_end = use_start + timedelta(minutes=5)

    # Generate the combined skyplot
    print("\nGenerating skyplot...")
    footer = f"From {use_start} to {use_end} UTC"

    plot_satellite_tracks(
        satellites,
        obs_lat,
        obs_lon,
        obs_alt,
        footer_text=footer,
        filename=filename,
        show_plot=not args.no_show,
        start_time=use_start,
        end_time=use_end
    )


if __name__ == "__main__":
    main()
