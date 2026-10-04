#!/usr/bin/env python3

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: 2026 Carles Fernandez-Prades <carles.fernandez@cttc.es>

"""Validate GNSS-SDR PVT XML output files with their corresponding schemas."""

import argparse
import pathlib
import shutil
import subprocess
import sys


SCHEMAS_BY_FILENAME = {
    "gps_ephemeris.xml": "ephemeris_map.xsd",
    "gps_cnav_ephemeris.xml": "cnav_ephemeris_map.xsd",
    "gal_ephemeris.xml": "gal_ephemeris_map.xsd",
    "gal_inav_ephemeris.xml": "gal_ephemeris_map.xsd",
    "gal_fnav_ephemeris.xml": "gal_ephemeris_map.xsd",
    "eph_GLONASS_GNAV.xml": "gnav_ephemeris_map.xsd",
    "glo_gnav_ephemeris.xml": "gnav_ephemeris_map.xsd",
    "bds_dnav_ephemeris.xml": "bds_dnav_ephemeris_map.xsd",
    "bds_cnav1_ephemeris.xml": "bds_cnav1_ephemeris_map.xsd",
    "gps_almanac.xml": "gps_almanac_map.xsd",
    "gal_almanac.xml": "gal_almanac_map.xsd",
    "bds_dnav_almanac.xml": "bds_dnav_almanac_map.xsd",
    "gps_iono.xml": "iono_model.xsd",
    "gps_cnav_iono.xml": "cnav_iono_model.xsd",
    "qzss_iono.xml": "qzss_iono_model.xsd",
    "qzss_cnav_iono.xml": "qzss_cnav_iono_model.xsd",
    "gal_iono.xml": "gal_iono_model.xsd",
    "bds_dnav_iono.xml": "bds_dnav_iono_model.xsd",
    "gps_utc_model.xml": "utc_model.xsd",
    "gps_cnav_utc_model.xml": "cnav_utc_model.xsd",
    "qzss_utc_model.xml": "qzss_utc_model.xsd",
    "qzss_cnav_utc_model.xml": "qzss_cnav_utc_model.xsd",
    "gal_utc_model.xml": "gal_utc_model.xsd",
    "glo_utc_model.xml": "gnav_utc_model.xsd",
    "bds_dnav_utc_model.xml": "bds_dnav_utc_model.xsd",
}


def xml_files(paths):
    """Expand file and directory arguments into a stable list of XML files."""
    result = []
    for path in paths:
        if path.is_dir():
            result.extend(sorted(path.glob("*.xml")))
        else:
            result.append(path)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", type=pathlib.Path, nargs="+",
                        help="XML output files or directories containing them")
    arguments = parser.parse_args()

    xmllint = shutil.which("xmllint")
    if xmllint is None:
        parser.error("xmllint was not found in PATH")

    schema_directory = pathlib.Path(__file__).resolve().parent
    failed = False
    validated = 0
    for xml_file in xml_files(arguments.paths):
        schema_name = SCHEMAS_BY_FILENAME.get(xml_file.name)
        if schema_name is None:
            print(f"Skipping unrecognized XML output: {xml_file}")
            continue
        if not xml_file.is_file():
            print(f"Missing XML file: {xml_file}", file=sys.stderr)
            failed = True
            continue

        command = [xmllint, "--noout", "--schema",
                   str(schema_directory / schema_name), str(xml_file)]
        result = subprocess.run(command, check=False)
        failed = failed or result.returncode != 0
        validated += 1

    if validated == 0:
        print("No recognized GNSS-SDR XML output files were found.", file=sys.stderr)
        return 1
    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
