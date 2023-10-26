"""
 gps_l1_ca_read_telemetry_dump.py
   gps_l1_ca_read_telemetry_dump (filename)

 Open and read GNSS-SDR telemetry binary log files (.dat) into Python, and
 return the contents.

 Irene Pérez Riega, 2023. iperrie@inta.es

      Args:
        filename        - Path to file telemetry[N].dat with the raw data

      Return:
        telemetry       - A dictionary with the processed data

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import struct


def gps_l1_ca_read_telemetry_dump(filename):

    double_size_bytes = 8
    int_size_bytes = 4
    bytes_shift = 0

    tow_current_symbol_ms = []
    tracking_sample_counter = []
    tow = []
    nav_simbols = []
    prn = []

    f = open(filename, 'rb')
    if f is None:
        return None
    else:
        while True:
            f.seek(bytes_shift, 0)

            tow_current_symbol_ms.append(struct.unpack(
                'd', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            tracking_sample_counter.append(struct.unpack(
                'Q', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            tow.append(struct.unpack(
                'd', f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)

            nav_simbols.append(struct.unpack(
                'I', f.read(int_size_bytes))[0])
            bytes_shift += int_size_bytes
            f.seek(bytes_shift, 0)

            prn.append(struct.unpack('I', f.read(int_size_bytes))[0])
            bytes_shift += int_size_bytes
            f.seek(bytes_shift, 0)

            # Check file
            linea = f.readline()
            if not linea:
                break

        telemetry = {
            'tow_current_symbol_ms': tow_current_symbol_ms,
            'tracking_sample_counter': tracking_sample_counter,
            'tow': tow,
            'nav_simbols': nav_simbols,
            'prn': prn
        }

    return telemetry
