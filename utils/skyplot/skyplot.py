#!/usr/bin/env python
"""
skyplot.py

Reads a RINEX navigation file and generates a skyplot. Optionally, a RINEX
observation file can also be read to match the skyplot to the receiver
processing time.

Usage:
   skyplot.py <RINEX_NAV_FILE> [observer_lat] [observer_lon] [observer_alt]
               [--elev-mask ELEV_MASK]
               [--format {pdf,eps,jpg,png,svg}]
               [--no-show]
               [--system SYSTEM [SYSTEM ...]]
               [--use-obs]

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
from typing import Any, Dict, List, Tuple, Optional

try:
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError:
    print("Error: This script requires matplotlib and numpy.")
    print("Install them with: pip install matplotlib numpy")
    sys.exit(1)

__version__ = "1.0.0"

# Default position: Castelldefels, Barcelona
DEFAULT_LAT = 41.275
DEFAULT_LON = 1.9876
DEFAULT_ALT = 80.0


def read_obs_time_bounds(
    obs_path: str,
) -> Tuple[Optional[datetime], Optional[datetime]]:
    """
    Return (start_time, end_time) from a RINEX observation file (v2/3/4)
    by scanning epoch lines. If parsing fails or the file is not OBS, return (None, None).
    """
    start_time = None
    end_time = None

    try:
        obs_file = Path(obs_path)
        if not obs_file.exists():
            return (None, None)

        with obs_file.open("r", encoding="utf-8", errors="ignore") as f:
            # --- Detect OBS file in header ---
            is_obs = False
            for line in f:
                if "RINEX VERSION / TYPE" in line:
                    file_type = line[20:21].upper()
                    if file_type == "O" or "OBSERVATION DATA" in line.upper():
                        is_obs = True
                if "END OF HEADER" in line:
                    break
            if not is_obs:
                return (None, None)

            # --- Scan for epoch lines ---
            for line in f:
                line = line.rstrip()
                if not line:
                    continue

                try:
                    if line.startswith(">"):  # RINEX 3/4 epoch line
                        yyyy = int(line[2:6])
                        mm = int(line[7:9])
                        dd = int(line[10:12])
                        hh = int(line[13:15])
                        mi = int(line[16:18])
                        ss = float(line[19:29])
                    else:  # RINEX 2 epoch line
                        yy = int(line[1:3])
                        mm = int(line[4:6])
                        dd = int(line[7:9])
                        hh = int(line[10:12])
                        mi = int(line[13:15])
                        ss = float(line[15:26])
                        yyyy = 1900 + yy if yy >= 80 else 2000 + yy

                    epoch = datetime(
                        yyyy, mm, dd, hh, mi, int(ss), int((ss % 1) * 1e6)
                    )

                    if start_time is None or epoch < start_time:
                        start_time = epoch
                    if end_time is None or epoch > end_time:
                        end_time = epoch
                except Exception:
                    # Skip malformed lines
                    continue

        return (start_time, end_time)
    except Exception:
        return (None, None)


def find_obs_for_nav(nav_file: str) -> Optional[str]:
    """
    Find the corresponding RINEX observation file for a given navigation file.

    This function attempts to locate the appropriate observation (OBS) file that
    corresponds to a given navigation (NAV) file by applying standard RINEX naming
    conventions across versions 2, 3, and 4.

    Parameters
    ----------
    nav_file : str
        Path to the RINEX navigation file. The function will search for a
        corresponding observation file using various naming patterns.

    Returns
    -------
    Optional[str]
        Path to the found observation file if successful, None otherwise.
        Also prints the list of attempted file paths if no match is found.
    """
    nav_path = Path(nav_file)
    tried = []

    stem = nav_path.stem
    suffix = nav_path.suffix

    # --- RINEX v2 names: replace last letter of extension with 'O' or 'o'
    if suffix and suffix[-1].isalpha():
        for o_type in ("O", "o"):
            candidate = nav_path.with_suffix(suffix[:-1] + o_type)
            tried.append(str(candidate))
            if candidate.exists():
                return str(candidate)

    # --- RINEX v3/v4 names: handle standard extensions and common modifiers
    gnss_patterns = [
        # Mixed constellations
        ("_MN", "_MO"),
        ("_mn", "_mo"),
        ("_MM", "_MO"),
        ("_mm", "_mo"),
        ("_MR", "_MO"),
        ("_mr", "_mo"),
        # Individual constellations
        ("_GN", "_GO"),  # GPS
        ("_gn", "_go"),
        ("_RN", "_RO"),  # GLONASS
        ("_rn", "_ro"),
        ("_EN", "_EO"),  # Galileo
        ("_en", "_eo"),
        ("_CN", "_CO"),  # BeiDou
        ("_cn", "_co"),
        ("_JN", "_JO"),  # QZSS
        ("_jn", "_jo"),
        ("_IN", "_IO"),  # IRNSS
        ("_in", "_io"),
        ("_SN", "_SO"),  # SBAS
        ("_sn", "_so"),
    ]

    for nav_pattern, obs_pattern in gnss_patterns:
        if nav_pattern in stem:
            # Direct replacement (e.g., _MN -> _MO)
            candidate = nav_path.with_name(
                stem.replace(nav_pattern, obs_pattern) + suffix
            )
            tried.append(str(candidate))
            if candidate.exists():
                return str(candidate)

            # Handle sampling rate patterns (e.g., _MN -> _30S_MO)
            sampling_rates = [
                "_30S",
                "_15S",
                "_01S",
                "_05S",
                "_30s",
                "_15s",
                "_01s",
                "_05s",
            ]
            for rate in sampling_rates:
                candidate = nav_path.with_name(
                    stem.replace(nav_pattern, rate + obs_pattern) + suffix
                )
                tried.append(str(candidate))
                if candidate.exists():
                    return str(candidate)

            # Also try with common observation extensions
            for obs_ext in [
                ".rnx",
                ".obs",
                ".OBS",
                ".22O",
                ".23O",
                ".24O",
                ".25O",
            ]:
                if suffix != obs_ext:
                    candidate = nav_path.with_name(
                        stem.replace(nav_pattern, obs_pattern) + obs_ext
                    )
                    tried.append(str(candidate))
                    if candidate.exists():
                        return str(candidate)

    # --- Additional patterns for files with sampling rate modifiers before constellation code
    sampling_rates = [
        "_30S",
        "_15S",
        "_01S",
        "_05S",
        "_30s",
        "_15s",
        "_01s",
        "_05s",
        "_01H",
        "_1H",
        "_01h",
        "_1h",
    ]
    for rate in sampling_rates:
        if rate in stem:
            # Check if this is a navigation file with sampling rate + _MN/_GN/etc.
            for nav_suffix in [
                "_MN",
                "_GN",
                "_RN",
                "_EN",
                "_CN",
                "_JN",
                "_IN",
                "_SN",
                "_mn",
                "_gn",
                "_rn",
                "_en",
                "_cn",
                "_jn",
                "_in",
                "_sn",
            ]:
                if rate + nav_suffix in stem:
                    # Replace navigation with observation (e.g., _30S_MN -> _30S_MO)
                    candidate = nav_path.with_name(
                        stem.replace(
                            rate + nav_suffix,
                            rate
                            + nav_suffix.replace("N", "O").replace("n", "o"),
                        )
                        + suffix
                    )
                    tried.append(str(candidate))
                    if candidate.exists():
                        return str(candidate)

                    # Also try without sampling rate (e.g., _30S_MN -> _MO)
                    candidate = nav_path.with_name(
                        stem.replace(
                            rate + nav_suffix,
                            nav_suffix.replace("N", "O").replace("n", "o"),
                        )
                        + suffix
                    )
                    tried.append(str(candidate))
                    if candidate.exists():
                        return str(candidate)

    print(f"OBS file not found. Tried: {', '.join(tried)}.")
    return None


def ecef_to_geodetic(
    x: float, y: float, z: float
) -> Tuple[float, float, float]:
    """Convert ECEF (X, Y, Z) [m] to (lat [°], lon [°], h [m])."""
    # WGS84 constants
    a = 6378137.0  # semi-major axis
    f = 1 / 298.257223563
    e2 = f * (2 - f)

    lon = atan2(y, x)
    r = sqrt(x * x + y * y)
    lat = atan2(z, r * (1 - e2))  # initial guess

    # Iterative improvement
    for _ in range(5):
        N = a / sqrt(1 - e2 * sin(lat) ** 2)
        h = r / cos(lat) - N
        lat = atan2(z, r * (1 - e2 * (N / (N + h))))

    N = a / sqrt(1 - e2 * sin(lat) ** 2)
    h = r / cos(lat) - N

    return np.degrees(lat), np.degrees(lon), h


def get_approx_position_from_obs(
    obs_file: str,
) -> Optional[Tuple[float, float, float]]:
    """Read APPROX POSITION XYZ from an OBS RINEX file header.
    Returns (lat, lon, h) in degrees/meters or None if not valid."""
    try:
        with open(obs_file, "r") as f:
            for line in f:
                if "APPROX POSITION XYZ" in line:
                    parts = line.split()
                    if len(parts) >= 3:
                        try:
                            x, y, z = map(float, parts[0:3])
                        except ValueError:
                            return None
                        # Check if position is valid (not all zeros)
                        if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
                            return None
                        # Convert ECEF → geodetic
                        return ecef_to_geodetic(x, y, z)
        return None
    except OSError:
        return None


def parse_rinex_float(s: str) -> float:
    """Parse RINEX formatted float string which may contain D or E exponent and compact spacing"""
    # Handle empty string
    if not s.strip():
        return 0.0

    # Replace D exponent with E (some RINEX files use D instead of E)
    s = s.replace("D", "E").replace("d", "e")

    # Handle cases where exponent lacks E (e.g., "12345-3")
    if re.match(r"[+-]?\d+[+-]\d+", s.strip()):
        s = s.replace("+", "E+").replace("-", "E-")

    try:
        return float(s)
    except ValueError:
        # Handle cases where the number runs into the next field
        # Try to split at the exponent if present
        if "E" in s:
            base, exp = s.split("E")[:2]
            # Take first character of exponent if needed
            if exp and exp[0] in "+-" and len(exp) > 1:
                return float(base + "E" + exp[0] + exp[1:].split()[0])
        return 0.0  # Default if parsing fails


def read_rinex_header(filename: str) -> Tuple[str, str]:
    """Return (version_str, file_type_char) from the 'RINEX VERSION / TYPE' header line."""
    with open(filename, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if "RINEX VERSION / TYPE" in line:
                version = line[0:9].strip()  # F9.2 in v2/v3/v4
                # 'N' (GPS nav v2), 'G' (GLO nav v2), 'H' (GEO/SBAS v2), 'N' in v3/4 too
                ftype = line[20:21].upper()
                return version, ftype
            if "END OF HEADER" in line:
                break
    return "", ""


def read_rinex_nav(filename: str) -> Dict[str, List[Dict[str, Any]]]:
    """
    Read RINEX v2/v3/v4 navigation file into a dict { 'Gxx': [eph...], 'Rxx': [...], 'Sxxx': [...] }.
    """
    version_str, ftype = read_rinex_header(filename)
    is_v2 = version_str.startswith("2")

    satellites = {}
    line_number = 0
    with open(filename, "r", encoding="utf-8", errors="ignore") as f:
        # Skip header
        while True:
            line = f.readline()
            line_number += 1
            if not line:
                return satellites
            if "END OF HEADER" in line:
                break

        current_line = f.readline()
        line_number += 1

        # ----------------------------
        # RINEX 2.10 / 2.11 parsing
        # ----------------------------
        if is_v2:
            # File type: 'N' (GPS), 'G' (GLONASS), 'H' (GEO/SBAS)
            v2_system = ftype  # keep original char
            while current_line:
                # Skip empties
                if not current_line.strip():
                    current_line = f.readline()
                    line_number += 1
                    continue

                try:
                    # --- First record line: PRN/EPOCH/CLOCK --------------------
                    # Formats per RINEX 2.11 Table A4 (GPS) and Table A11 (GLONASS)
                    prn_num = int(current_line[0:2])
                    yy = int(current_line[3:5])
                    mm = int(current_line[6:8])
                    dd = int(current_line[9:11])
                    hh = int(current_line[12:14])
                    mi = int(current_line[15:17])
                    ss = float(current_line[18:23])

                    # Year mapping: 80–99 => 1980–1999, 00–79 => 2000–2079
                    yyyy = 1900 + yy if yy >= 80 else 2000 + yy
                    epoch = datetime(
                        yyyy, mm, dd, hh, mi, int(ss), int((ss % 1) * 1e6)
                    )

                    # Map PRN to 'Gxx' / 'Rxx' / 'Sxxx'
                    if v2_system == "N":  # GPS nav
                        prn = f"G{prn_num:02d}"
                    elif v2_system == "G":  # GLONASS nav
                        prn = f"R{prn_num:02d}"
                    elif v2_system == "H":  # GEO/SBAS nav (PRN-100 in file)
                        prn = f"S{prn_num + 100:03d}"
                    else:
                        # Unknown v2 type; skip
                        current_line = f.readline()
                        line_number += 1
                        continue

                    # Collect the lines of this ephemeris block:
                    lines = [current_line]

                    if v2_system == "G" or v2_system == "H":
                        # GLONASS & GEO blocks: 3 more lines (Tables A11/A16) -> total 4 records
                        needed = 3
                    else:
                        # GPS v2 block: 7 more lines (Table A4) -> total 8 records
                        needed = 7

                    for _ in range(needed):
                        next_line = f.readline()
                        line_number += 1
                        if not next_line:
                            break
                        lines.append(next_line)

                    if len(lines) < needed + 1:
                        current_line = f.readline()
                        line_number += 1
                        continue

                    if v2_system == "N":
                        # GPS
                        ephemeris = {
                            "prn": prn,
                            "epoch": epoch,
                            "sv_clock_bias": parse_rinex_float(
                                lines[0][23:41]
                            ),
                            "sv_clock_drift": parse_rinex_float(
                                lines[0][41:61]
                            ),
                            "sv_clock_drift_rate": parse_rinex_float(
                                lines[0][61:80]
                            ),
                            "iode": parse_rinex_float(lines[1][4:22]),
                            "crs": parse_rinex_float(lines[1][22:41]),
                            "delta_n": parse_rinex_float(lines[1][41:60]),
                            "m0": parse_rinex_float(lines[1][61:80]),
                            "cuc": parse_rinex_float(lines[2][4:22]),
                            "ecc": parse_rinex_float(lines[2][22:41]),
                            "cus": parse_rinex_float(lines[2][41:60]),
                            "sqrt_a": parse_rinex_float(lines[2][60:80]),
                            "toe": parse_rinex_float(lines[3][4:22]),
                            "cic": parse_rinex_float(lines[3][22:41]),
                            "omega0": parse_rinex_float(lines[3][41:60]),
                            "cis": parse_rinex_float(lines[3][60:80]),
                            "i0": parse_rinex_float(lines[4][4:22]),
                            "crc": parse_rinex_float(lines[4][22:41]),
                            "omega": parse_rinex_float(lines[4][41:60]),
                            "omega_dot": parse_rinex_float(lines[4][60:80]),
                            "idot": parse_rinex_float(lines[5][4:22]),
                            "codes_l2": parse_rinex_float(lines[5][22:41]),
                            "gps_week": parse_rinex_float(lines[5][41:61]),
                            "l2p_flag": parse_rinex_float(lines[5][61:80]),
                            "sv_accuracy": parse_rinex_float(lines[6][4:22]),
                            "sv_health": parse_rinex_float(lines[6][22:41]),
                            "tgd": parse_rinex_float(lines[6][41:61]),
                            "iodc": parse_rinex_float(lines[6][61:80]),
                            "transmission_time": parse_rinex_float(
                                lines[7][4:22]
                            )
                            if len(lines) > 7
                            else None,
                            "fit_interval": parse_rinex_float(lines[7][22:41])
                            if len(lines) > 7
                            else None,
                            "extra": lines[8:],
                        }
                    elif v2_system == "H":
                        # GEO/SBAS (Table A16)
                        ephemeris = {
                            "prn": prn,
                            "epoch": epoch,
                            "sv_clock_bias": parse_rinex_float(
                                lines[0][23:41]
                            ),
                            "sv_clock_drift": parse_rinex_float(
                                lines[0][41:61]
                            ),
                            "sv_clock_drift_rate": parse_rinex_float(
                                lines[0][61:80]
                            ),
                            "x": parse_rinex_float(lines[1][4:22]),
                            "x_vel": parse_rinex_float(lines[1][22:41]),
                            "x_acc": parse_rinex_float(lines[1][41:60]),
                            "health": parse_rinex_float(lines[1][60:80]),
                            "y": parse_rinex_float(lines[2][4:22]),
                            "y_vel": parse_rinex_float(lines[2][22:41]),
                            "y_acc": parse_rinex_float(lines[2][41:61]),
                            "z": parse_rinex_float(lines[3][4:22]),
                            "z_vel": parse_rinex_float(lines[3][21:41]),
                            "z_acc": parse_rinex_float(lines[3][41:61]),
                            "extra": lines[4:],
                        }
                    elif v2_system == "G":
                        # GLONASS
                        ephemeris = {
                            "prn": prn,
                            "epoch": epoch,
                            "sv_clock_bias": parse_rinex_float(
                                lines[0][23:42]
                            ),
                            "sv_relative_freq_bias": parse_rinex_float(
                                lines[0][42:61]
                            ),
                            "message_frame_time": parse_rinex_float(
                                lines[0][61:80]
                            ),
                            "x": parse_rinex_float(lines[1][4:22]),
                            "x_vel": parse_rinex_float(lines[1][22:41]),
                            "x_acc": parse_rinex_float(lines[1][41:60]),
                            "health": parse_rinex_float(lines[1][60:80]),
                            "y": parse_rinex_float(lines[2][4:22]),
                            "y_vel": parse_rinex_float(lines[2][22:41]),
                            "y_acc": parse_rinex_float(lines[2][41:60]),
                            "freq_num": parse_rinex_float(lines[2][60:80]),
                            "z": parse_rinex_float(lines[3][4:22]),
                            "z_vel": parse_rinex_float(lines[3][22:41]),
                            "z_acc": parse_rinex_float(lines[3][41:60]),
                            "age": parse_rinex_float(lines[3][60:80]),
                            "extra": lines[4:],
                        }
                    else:
                        ephemeris = None

                    if ephemeris:
                        satellites.setdefault(prn, []).append(ephemeris)

                except (ValueError, IndexError):
                    # Skip malformed block; advance
                    current_line = f.readline()
                    line_number += 1
                    continue

                current_line = f.readline()
                line_number += 1

            return satellites  # done with v2

        # ----------------------------
        # RINEX 3 / 4 parsing
        # ----------------------------
        while current_line:
            # Skip short/noise lines
            if len(current_line) < 23:
                current_line = f.readline()
                line_number += 1
                continue

            # Parse the epoch line
            parts = current_line.split()
            if len(parts) < 8:
                current_line = f.readline()
                line_number += 1
                continue

            prn = parts[0].strip()
            system = prn[0]

            try:
                year = int(parts[1])
                month = int(parts[2])
                day = int(parts[3])
                hour = int(parts[4])
                minute = int(parts[5])
                second = float(parts[6][:2])
                epoch = datetime(
                    year,
                    month,
                    day,
                    hour,
                    minute,
                    int(second),
                    int((second % 1) * 1e6),
                )
                lines = [current_line]
                line_count = 4 if system == "R" or system == "S" else 7
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

                if system == "R":  # GLONASS
                    ephemeris = {
                        "prn": prn,
                        "epoch": epoch,
                        "sv_clock_bias": parse_rinex_float(lines[0][23:41]),
                        "sv_relative_freq_bias": parse_rinex_float(
                            lines[0][42:61]
                        ),
                        "message_frame_time": parse_rinex_float(
                            lines[0][61:80]
                        ),
                        "x": parse_rinex_float(lines[1][4:23]),
                        "x_vel": parse_rinex_float(lines[1][23:41]),
                        "x_acc": parse_rinex_float(lines[1][42:61]),
                        "health": parse_rinex_float(lines[1][61:80]),
                        "y": parse_rinex_float(lines[2][4:23]),
                        "y_vel": parse_rinex_float(lines[2][23:41]),
                        "y_acc": parse_rinex_float(lines[2][42:61]),
                        "freq_num": parse_rinex_float(lines[2][61:80]),
                        "z": parse_rinex_float(lines[3][4:23]),
                        "z_vel": parse_rinex_float(lines[3][23:41]),
                        "z_acc": parse_rinex_float(lines[3][42:61]),
                        "age": parse_rinex_float(lines[3][61:80]),
                        "extra": lines[4:],
                    }
                elif system == "S":  # SBAS (RINEX v4 short form)
                    ephemeris = {
                        "prn": prn,
                        "epoch": epoch,
                        "sv_clock_bias": parse_rinex_float(lines[0][23:42]),
                        "sv_clock_drift": parse_rinex_float(lines[0][42:61]),
                        "sv_clock_drift_rate": parse_rinex_float(
                            lines[0][61:80]
                        ),
                        "x": parse_rinex_float(lines[1][4:23])
                        if len(lines) > 1
                        else None,
                        "x_vel": parse_rinex_float(lines[1][23:42])
                        if len(lines) > 1
                        else None,
                        "x_acc": parse_rinex_float(lines[1][42:61])
                        if len(lines) > 1
                        else None,
                        "health": parse_rinex_float(lines[1][61:80])
                        if len(lines) > 1
                        else None,
                        "y": parse_rinex_float(lines[2][4:23])
                        if len(lines) > 2
                        else None,
                        "y_vel": parse_rinex_float(lines[2][23:42])
                        if len(lines) > 2
                        else None,
                        "y_acc": parse_rinex_float(lines[2][42:61])
                        if len(lines) > 2
                        else None,
                        "z": parse_rinex_float(lines[3][4:23])
                        if len(lines) > 3
                        else None,
                        "z_vel": parse_rinex_float(lines[3][23:42])
                        if len(lines) > 3
                        else None,
                        "z_acc": parse_rinex_float(lines[3][42:61])
                        if len(lines) > 3
                        else None,
                        "extra": lines[4:],
                    }
                elif system in ("G", "E", "C", "I", "J"):
                    ephemeris = {
                        "prn": prn,
                        "epoch": epoch,
                        "sv_clock_bias": parse_rinex_float(lines[0][23:42]),
                        "sv_clock_drift": parse_rinex_float(lines[0][42:61]),
                        "sv_clock_drift_rate": parse_rinex_float(
                            lines[0][61:80]
                        ),
                        "iode": parse_rinex_float(lines[1][4:23]),
                        "crs": parse_rinex_float(lines[1][23:42]),
                        "delta_n": parse_rinex_float(lines[1][42:61]),
                        "m0": parse_rinex_float(lines[1][61:80]),
                        "cuc": parse_rinex_float(lines[2][4:23]),
                        "ecc": parse_rinex_float(lines[2][23:42]),
                        "cus": parse_rinex_float(lines[2][42:61]),
                        "sqrt_a": parse_rinex_float(lines[2][61:80]),
                        "toe": parse_rinex_float(lines[3][4:23]),
                        "cic": parse_rinex_float(lines[3][23:42]),
                        "omega0": parse_rinex_float(lines[3][42:61]),
                        "cis": parse_rinex_float(lines[3][61:80]),
                        "i0": parse_rinex_float(lines[4][4:23]),
                        "crc": parse_rinex_float(lines[4][23:42]),
                        "omega": parse_rinex_float(lines[4][42:61]),
                        "omega_dot": parse_rinex_float(lines[4][61:80]),
                        "idot": parse_rinex_float(lines[5][4:23]),
                        "codes_l2": parse_rinex_float(lines[5][23:42]),
                        "gps_week": parse_rinex_float(lines[5][42:61]),
                        "l2p_flag": parse_rinex_float(lines[5][61:80]),
                        "sv_accuracy": parse_rinex_float(lines[6][4:23]),
                        "sv_health": parse_rinex_float(lines[6][23:42]),
                        "tgd": parse_rinex_float(lines[6][42:61]),
                        "iodc": parse_rinex_float(lines[6][61:80]),
                        "transmission_time": parse_rinex_float(lines[7][4:23])
                        if len(lines) > 7
                        else None,
                        "fit_interval": parse_rinex_float(lines[7][23:42])
                        if len(lines) > 7
                        else None,
                        "extra": lines[8:],
                    }
                else:
                    ephemeris = []

                if ephemeris:
                    satellites.setdefault(prn, []).append(ephemeris)

            except (ValueError, IndexError):
                # Skip to next line
                pass

            current_line = f.readline()
            line_number += 1

    return satellites


def calculate_satellite_position(
    ephemeris: Dict[str, Any], transmit_time: float
) -> Tuple[float, float, float]:
    """Calculate satellite position in ECEF coordinates [m], at a given transmission time"""
    system = ephemeris["prn"][0]

    if system in ("R", "S"):  # GLONASS/SBAS
        dt = transmit_time
        # Convert km to meters
        xk = (
            ephemeris["x"]
            + ephemeris["x_vel"] * dt
            + 0.5 * ephemeris["x_acc"] * dt**2
        ) * 1000
        yk = (
            ephemeris["y"]
            + ephemeris["y_vel"] * dt
            + 0.5 * ephemeris["y_acc"] * dt**2
        ) * 1000
        zk = (
            ephemeris["z"]
            + ephemeris["z_vel"] * dt
            + 0.5 * ephemeris["z_acc"] * dt**2
        ) * 1000
    else:
        # Constants
        mu = 3.986005e14  # Earth's gravitational constant (m^3/s^2)
        omega_e_dot = 7.2921151467e-5  # Earth rotation rate (rad/s)

        # Semi-major axis
        a = ephemeris["sqrt_a"] ** 2

        # Corrected mean motion
        n0 = sqrt(mu / (a**3))
        n = n0 + ephemeris["delta_n"]

        # Mean anomaly
        mk = ephemeris["m0"] + n * transmit_time

        # Solve Kepler's equation for eccentric anomaly (Ek)
        ek = mk
        for _ in range(10):
            ek_old = ek
            ek = mk + ephemeris["ecc"] * sin(ek)
            if abs(ek - ek_old) < 1e-12:
                break

        # True anomaly
        nu_k = atan2(
            sqrt(1 - ephemeris["ecc"] ** 2) * sin(ek),
            cos(ek) - ephemeris["ecc"],
        )

        # Argument of latitude
        phi_k = nu_k + ephemeris["omega"]

        # Second harmonic perturbations
        delta_uk = ephemeris["cus"] * sin(2 * phi_k) + ephemeris["cuc"] * cos(
            2 * phi_k
        )
        delta_rk = ephemeris["crs"] * sin(2 * phi_k) + ephemeris["crc"] * cos(
            2 * phi_k
        )
        delta_ik = ephemeris["cis"] * sin(2 * phi_k) + ephemeris["cic"] * cos(
            2 * phi_k
        )

        # Corrected argument of latitude, radius and inclination
        uk = phi_k + delta_uk
        rk = a * (1 - ephemeris["ecc"] * cos(ek)) + delta_rk
        ik = ephemeris["i0"] + delta_ik + ephemeris["idot"] * transmit_time

        # Positions in orbital plane
        xk_prime = rk * cos(uk)
        yk_prime = rk * sin(uk)

        # Corrected longitude of ascending node
        omega_k = (
            ephemeris["omega0"]
            + (ephemeris["omega_dot"] - omega_e_dot) * transmit_time
            - omega_e_dot * ephemeris["toe"]
        )

        # Earth-fixed coordinates
        xk = xk_prime * cos(omega_k) - yk_prime * cos(ik) * sin(omega_k)
        yk = xk_prime * sin(omega_k) + yk_prime * cos(ik) * cos(omega_k)
        zk = yk_prime * sin(ik)

    return xk, yk, zk


def calculate_satellite_positions(
    ephemeris: Dict[str, Any],
    start_time: datetime,
    end_time: datetime,
    step_min: int = 5,
) -> List[Tuple[datetime, float, float, float]]:
    """
    Compute multiple positions over time for a single satellite
    between start_time and end_time.

    Parameters
    ----------
    ephemeris : Dict[str, Any]
        Satellite ephemeris data containing orbital parameters
    start_time : datetime
        Start time for position calculations
    end_time : datetime
        End time for position calculations
    step_min : int, optional
        Time step in minutes between calculations, by default 5

    Returns
    -------
    List[Tuple[datetime, float, float, float]]
        List of tuples containing (timestamp, x, y, z) coordinates
        in ECEF frame for each time step
    """
    positions = []
    current_time = start_time
    system = ephemeris["prn"][0]
    max_valid_time = 1800 if system == "R" else 14400
    while current_time <= end_time:
        transmit_time = (current_time - ephemeris["epoch"]).total_seconds()

        if abs(transmit_time) <= max_valid_time:
            x, y, z = calculate_satellite_position(ephemeris, transmit_time)
            positions.append((current_time, x, y, z))

        current_time += timedelta(minutes=step_min)

    return positions


def ecef_to_az_el(
    x: float,
    y: float,
    z: float,
    obs_lat: float,
    obs_lon: float,
    obs_alt: float,
) -> Tuple[float, float]:
    """
    Convert ECEF coordinates to azimuth and elevation angles.

    Parameters
    ----------
    x : float
        Satellite X coordinate in ECEF frame (meters)
    y : float
        Satellite Y coordinate in ECEF frame (meters)
    z : float
        Satellite Z coordinate in ECEF frame (meters)
    obs_lat : float
        Observer latitude in radians
    obs_lon : float
        Observer longitude in radians
    obs_alt : float
        Observer altitude in meters above WGS-84 ellipsoid

    Returns
    -------
    Tuple[float, float]
        Azimuth and elevation angles in degrees.
        Azimuth: 0° to 360° (0° = North, 90° = East)
        Elevation: 0° to 90° (0° = horizon, 90° = zenith)
    """
    # WGS-84 parameters
    a = 6378137.0  # semi-major axis
    e_sq = 6.69437999014e-3  # first eccentricity squared

    # Convert geodetic coordinates to ECEF
    n = a / sqrt(1 - e_sq * sin(obs_lat) ** 2)
    obs_x = (n + obs_alt) * cos(obs_lat) * cos(obs_lon)
    obs_y = (n + obs_alt) * cos(obs_lat) * sin(obs_lon)
    obs_z = (n * (1 - e_sq) + obs_alt) * sin(obs_lat)

    # Vector from observer to satellite
    dx = x - obs_x
    dy = y - obs_y
    dz = z - obs_z

    # Convert to local ENU (East, North, Up) coordinates
    enu_x = -sin(obs_lon) * dx + cos(obs_lon) * dy
    enu_y = (
        -sin(obs_lat) * cos(obs_lon) * dx
        - sin(obs_lat) * sin(obs_lon) * dy
        + cos(obs_lat) * dz
    )
    enu_z = (
        cos(obs_lat) * cos(obs_lon) * dx
        + cos(obs_lat) * sin(obs_lon) * dy
        + sin(obs_lat) * dz
    )

    # Calculate azimuth and elevation
    azimuth = atan2(enu_x, enu_y)
    elevation = atan2(enu_z, sqrt(enu_x**2 + enu_y**2))

    # Convert to degrees and adjust azimuth to 0-360
    azimuth = np.degrees(azimuth) % 360
    elevation = np.degrees(elevation)

    return azimuth, elevation


def plot_satellite_tracks(
    satellites: Dict[str, List[Dict[str, Any]]],
    obs_lat: float,
    obs_lon: float,
    obs_alt: float,
    footer_text: Optional[str] = None,
    filename: Optional[str] = None,
    show_plot: bool = True,
    start_time: Optional[datetime] = None,
    end_time: Optional[datetime] = None,
    elev_mask: float = 5.0,
    output_format: str = "pdf",
) -> None:
    """
    Plot trajectories for all visible satellites in a polar skyplot.

    Parameters
    ----------
    satellites : Dict[str, List[Dict[str, Any]]]
        Dictionary mapping satellite PRN strings to lists of ephemeris data
    obs_lat : float
        Observer latitude in radians
    obs_lon : float
        Observer longitude in radians
    obs_alt : float
        Observer altitude in meters
    footer_text : Optional[str], optional
        Text to display at bottom of plot, by default None
    filename : Optional[str], optional
        Input filename for naming output, by default None
    show_plot : bool, optional
        Whether to display the plot interactively, by default True
    start_time : Optional[datetime], optional
        Start time for position calculations, by default None (auto-detect)
    end_time : Optional[datetime], optional
        End time for position calculations, by default None (auto-detect)
    elev_mask : float, optional
        Minimum elevation angle in degrees to plot, by default 5.0
    output_format : str, optional
        Output file format, by default "pdf"

    Returns
    -------
    None
        Saves plot to file and optionally displays it
    """
    plt.rcParams["pdf.fonttype"] = 42  # TrueType fonts
    plt.rcParams["ps.fonttype"] = 42  # TrueType fonts
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = ["Times New Roman", "Times", "DejaVu Serif"]
    plt.rcParams["mathtext.fontset"] = "dejavuserif"  # For math text
    plt.rcParams["savefig.dpi"] = 300  # for jpg and png
    plt.rcParams["savefig.bbox"] = "tight"  # Always use bbox_inches='tight'
    plt.rcParams["svg.fonttype"] = "none"  # Make SVG text editable
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="polar")
    ax.tick_params(labelsize=16, pad=7)

    # Polar plot setup
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_ylim(0, 90)

    # Elevation ticks
    ax.set_yticks(range(0, 91, 15))
    ax.set_yticklabels(["90°", "", "60°", "", "30°", "", "0°"], fontsize=14)

    # Color scheme by constellation
    system_colors = {
        "G": "blue",  # GPS
        "E": "green",  # Galileo
        "R": "red",  # GLONASS
        "C": "orange",  # BeiDou
        "J": "brown",  # QZSS
        "I": "pink",  # IRNSS
        "S": "lightgray",  # SBAS
        "L": "cyan",  # LEO (new in RINEX v4)
    }

    # System names mapping
    system_names = {
        "G": "GPS",
        "E": "Galileo",
        "R": "GLONASS",
        "C": "BeiDou",
        "J": "QZSS",
        "I": "IRNSS",
        "S": "SBAS",
        "L": "LEO",
    }

    # Find which systems are actually present
    present_systems = {
        prn[0] for prn in satellites.keys() if prn[0] in system_colors
    }

    # Plot each satellite
    for prn, ephemeris_list in satellites.items():
        # Default to purple for unknown systems
        color = system_colors.get(prn[0], "purple")

        if not ephemeris_list:
            continue

        mid_time = start_time + (end_time - start_time) / 2
        prev_eph = [e for e in ephemeris_list if e["epoch"] <= mid_time]
        if prev_eph:
            ephemeris = max(prev_eph, key=lambda e: e["epoch"])
        else:
            ephemeris = min(
                ephemeris_list,
                key=lambda e: abs((e["epoch"] - mid_time).total_seconds()),
            )

        if start_time is None or end_time is None:
            all_epochs = sorted(
                {
                    e["epoch"]
                    for prn_data in satellites.values()
                    for e in prn_data
                }
            )
            start_time = min(all_epochs)
            end_time = max(all_epochs)

        positions = calculate_satellite_positions(
            ephemeris, start_time, end_time
        )

        # Split into visible segments
        segments = []
        current_seg_az = []
        current_seg_el = []
        for _, x, y, z in positions:
            azimuth, elevation = ecef_to_az_el(
                x, y, z, obs_lat, obs_lon, obs_alt
            )
            if elevation > elev_mask:
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
            ax.plot(
                theta, r, "-", color=color, alpha=0.7, linewidth=2.5, zorder=1
            )

            # Arrow at end
            if len(theta) >= 2:
                dx = theta[-1] - theta[-2]
                dy = r[-1] - r[-2]
                arrow_length_factor = 1.8
                extended_theta = theta[-2] + dx * arrow_length_factor
                extended_r = r[-2] + dy * arrow_length_factor
                ax.annotate(
                    "",
                    xytext=(theta[-1], r[-1]),
                    xy=(extended_theta, extended_r),
                    arrowprops={
                        "arrowstyle": "->",
                        "color": color,
                        "alpha": 0.9,
                        "linewidth": 1.5,
                        "shrinkA": 0,
                        "shrinkB": 0,
                    },
                    zorder=2,
                )

            # Label at midpoint of the segment
            mid_idx = len(theta) // 2
            ax.text(
                theta[mid_idx],
                r[mid_idx],
                prn,
                fontsize=12,
                ha="center",
                va="center",
                bbox={
                    "facecolor": system_colors.get(prn[0], "white"),
                    "alpha": 0.2,
                    "pad": 2,
                },
                zorder=3,
            )

    # Legend for present systems
    legend_elements = [
        plt.Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            label=f"{system_names[sys]} ({sys})",
            markerfacecolor=system_colors[sys],
            markersize=10,
        )
        for sys in present_systems
    ]
    if legend_elements:
        ax.legend(
            handles=legend_elements,
            loc="upper right",
            bbox_to_anchor=(1.3, 1.1),
            fontsize=14,
        )

    lat_deg = np.degrees(obs_lat)
    lon_deg = np.degrees(obs_lon)
    lat_hemisphere = "N" if lat_deg >= 0 else "S"
    lon_hemisphere = "E" if lon_deg >= 0 else "W"

    plt.title(
        f"GNSS skyplot from {abs(lat_deg):.2f}° {lat_hemisphere}, "
        f"{abs(lon_deg):.2f}° {lon_hemisphere}",
        pad=25,
        fontsize=20,
    )

    if footer_text:
        fig.text(
            0.42, 0.05, footer_text, ha="center", va="center", fontsize=16
        )

    plt.tight_layout()

    if filename:
        filename_no_path = Path(filename).name
        filename_no_dots = filename_no_path.replace(".", "_")
        output_name = f"skyplot_{filename_no_dots}.{output_format}"
    else:
        output_name = f"skyplot.{output_format}"

    plt.savefig(output_name, format=output_format)
    print(f"Image saved as {output_name}")
    if show_plot:
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nExecution interrupted by the user. Exiting.")
            plt.close()
    else:
        plt.close()


def main():
    """Generate the skyplot"""
    try:
        # Set system names and codes
        system_name_to_code = {
            "GPS": "G",
            "GLONASS": "R",
            "GALILEO": "E",
            "BEIDOU": "C",
            "QZSS": "J",
            "IRNSS": "I",
            "SBAS": "S",
            "LEO": "L",
        }

        # Set up argument parser
        parser = argparse.ArgumentParser(
            description="Generate a GNSS skyplot from a RINEX navigation file",
            epilog="Example: skyplot.py brdc0010.22n -33.4592 -70.6453 520.0 --format png --system G E --elev-mask 10 --no-show",
        )

        # Positional arguments
        parser.add_argument("filename", help="RINEX navigation file path")

        parser.add_argument(
            "lat",
            nargs="?",
            type=float,
            default=DEFAULT_LAT,
            help=f"Observer latitude in degrees (default: {DEFAULT_LAT}° N)",
        )

        parser.add_argument(
            "lon",
            nargs="?",
            type=float,
            default=DEFAULT_LON,
            help=f"Observer longitude in degrees (default: {DEFAULT_LON}° E)",
        )

        parser.add_argument(
            "alt",
            nargs="?",
            type=float,
            default=DEFAULT_ALT,
            help=f"Observer altitude in meters (default: {DEFAULT_ALT} m)",
        )

        # Optional arguments
        parser.add_argument(
            "--elev-mask",
            type=float,
            default=5.0,
            help="Elevation mask in degrees for plotting satellite tracks (default: 5°)",
        )

        parser.add_argument(
            "--format",
            type=str,
            default="pdf",
            choices=["pdf", "eps", "jpg", "png", "svg"],
            help="Output file format for plot (default: pdf)",
        )

        parser.add_argument(
            "--no-show",
            action="store_true",
            help="Run without displaying plot window",
        )

        parser.add_argument(
            "--system",
            nargs="+",
            help="Only plot satellites from these systems (e.g., G R E or GPS GLONASS Galileo)",
        )

        parser.add_argument(
            "--use-obs",
            action="store_true",
            help="Use corresponding RINEX observation file to bound the skyplot to the receiver time window",
        )

        parser.add_argument(
            "-v",
            "--version",
            action="version",
            version=f"%(prog)s {__version__}",
            help="Show program version and exit",
        )

        # Parse all arguments with full validation
        args = parser.parse_args()

        filename = args.filename
        user_provided_position = (
            (args.lat != DEFAULT_LAT)
            or (args.lon != DEFAULT_LON)
            or (args.alt != DEFAULT_ALT)
        )

        # Convert coordinates to radians
        obs_lat = np.radians(args.lat)
        obs_lon = np.radians(args.lon)
        obs_alt = args.alt

        # Read RINEX file
        print(f"Reading {filename} ...")
        try:
            satellites = read_rinex_nav(filename)
        except FileNotFoundError:
            print(f"Error: File {filename} not found.")
            return 1

        if not satellites:
            print("No satellite data found in the file.")
            return 1

        if args.system:
            systems_upper = set()
            for s in args.system:
                s_upper = s.upper()
                if s_upper in system_name_to_code:
                    systems_upper.add(system_name_to_code[s_upper])
                else:
                    systems_upper.add(s_upper)  # Assume user passed the code

            satellites = {
                prn: eph_list
                for prn, eph_list in satellites.items()
                if prn[0].upper() in systems_upper
            }

            if not satellites:
                print(
                    f"No satellites found for systems: {', '.join(sorted(systems_upper))}"
                )
                return 1

        # Print summary information
        all_epochs = sorted(
            list(
                set(
                    e["epoch"]
                    for prn, ephemerides in satellites.items()
                    for e in ephemerides
                )
            )
        )
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
                "G": "GPS",
                "R": "GLONASS",
                "E": "Galileo",
                "C": "BeiDou",
                "J": "QZSS",
                "I": "IRNSS",
                "S": "SBAS",
                "L": "LEO",
            }.get(system, "Unknown")
            print(f"- {system_name} ({system}): {count} satellites")

        # Generate the combined skyplot
        # Time window: OBS bounds if provided; else NAV span
        use_start, use_end = all_epochs[0], all_epochs[-1]
        if args.use_obs:
            obs_path = find_obs_for_nav(filename)
            if obs_path:
                obs_start, obs_end = read_obs_time_bounds(obs_path)
                if obs_start and obs_end:
                    use_start, use_end = obs_start, obs_end
                    print(
                        f"\nObservation window detected in {obs_path}: from {use_start} to {use_end}"
                    )
                else:
                    print(
                        f"\nWarning: Could not read valid times from {obs_path}. Using NAV span instead."
                    )
                if not user_provided_position:
                    approx_pos = get_approx_position_from_obs(obs_path)
                    if approx_pos is not None:
                        lat, lon, h = approx_pos
                        obs_lat = np.radians(lat)
                        obs_lon = np.radians(lon)
                        obs_alt = h
                        print(
                            f"\nObserver position found in {obs_path}: lat {lat:.2f}°, lon {lon:.2f}°, height {h:.2f} m."
                        )

        # Ensure at least two samples with the default 5-minute step
        if (use_end - use_start) < timedelta(minutes=5):
            use_end = use_start + timedelta(minutes=5)

        # Generate the combined skyplot
        print("\nGenerating skyplot ...")
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
            end_time=use_end,
            elev_mask=args.elev_mask,
            output_format=args.format,
        )
    except Exception as e:
        print(f"Error: {str(e)}")
        return 1

    return 0


if __name__ == "__main__":
    main()
