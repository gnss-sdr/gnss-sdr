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
                          Empty files return empty lists. An incomplete
                          trailing record is ignored.

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import struct

# Fields are written separately by gps_l1_ca_telemetry_decoder_gs.cc through
# write_value(): native byte order, standard sizes, and no alignment padding.
_RECORD_FORMAT = '=dQdii'
_FIELD_NAMES = [
    'tow_current_symbol_ms', 'tracking_sample_counter', 'tow', 'nav_simbols',
    'prn',
]


def gps_l1_ca_read_telemetry_dump(filename):

    record_size = struct.calcsize(_RECORD_FORMAT)
    columns = [[] for _ in _FIELD_NAMES]

    with open(filename, 'rb') as f:
        while True:
            record = f.read(record_size)
            if len(record) < record_size:
                # Clean end of file (or a trailing partial record).
                break
            for column, value in zip(columns, struct.unpack(_RECORD_FORMAT,
                                                           record)):
                column.append(value)

    return dict(zip(_FIELD_NAMES, columns))
