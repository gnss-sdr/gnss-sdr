"""
 read_hybrid_observables_dump.py

   This function plots the tracking results for the given channel list.

 Irene Pérez Riega, 2023. iperrie@inta.es

 read_hybrid_observables_dump(channels, filename)

   Args:
       channels        - list of channels to be processed
       filename        - path to the observables file

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import struct

def read_hybrid_observables_dump(channels, filename):

    double_size_bytes = 8
    bytes_shift = 0

    RX_time = [[] for _ in range(channels+1)]
    d_TOW_at_current_symbol = [[] for _ in range(channels+1)]
    Carrier_Doppler_hz = [[] for _ in range(channels+1)]
    Carrier_phase_hz = [[] for _ in range(channels+1)]
    Pseudorange_m = [[] for _ in range(channels+1)]
    PRN = [[] for _ in range(channels+1)]
    valid = [[] for _ in range(channels+1)]

    f = open(filename, 'rb')
    if f is None:
        return None
    else:
        while True:
            try:
                # There is an empty channel at the end (Channel-6)
                for N in range(0, channels+1):
                    f.seek(bytes_shift, 0)

                    RX_time[N].append(struct.unpack(
                        'd',f.read(double_size_bytes))[0])
                    bytes_shift += double_size_bytes
                    f.seek(bytes_shift, 0)

                    d_TOW_at_current_symbol[N].append(struct.unpack(
                        'd', f.read(double_size_bytes))[0])
                    bytes_shift += double_size_bytes
                    f.seek(bytes_shift, 0)

                    Carrier_Doppler_hz[N].append(struct.unpack(
                        'd', f.read(double_size_bytes))[0])
                    bytes_shift += double_size_bytes
                    f.seek(bytes_shift, 0)

                    Carrier_phase_hz[N].append(struct.unpack(
                        'd', f.read(double_size_bytes))[0])
                    bytes_shift += double_size_bytes
                    f.seek(bytes_shift, 0)

                    Pseudorange_m[N].append(struct.unpack(
                        'd', f.read(double_size_bytes))[0])
                    bytes_shift += double_size_bytes
                    f.seek(bytes_shift, 0)

                    PRN[N].append(struct.unpack(
                        'd', f.read(double_size_bytes))[0])
                    bytes_shift += double_size_bytes
                    f.seek(bytes_shift, 0)

                    valid[N].append(struct.unpack(
                        'd', f.read(double_size_bytes))[0])
                    bytes_shift += double_size_bytes
                    f.seek(bytes_shift, 0)

            except:
                break

        # Delete last Channel:
        RX_time = [row for i, row in enumerate(RX_time) if i != 5]
        d_TOW_at_current_symbol = [row for i, row in enumerate(
            d_TOW_at_current_symbol)if i != 5]
        Carrier_Doppler_hz = [row for i, row in enumerate(
            Carrier_Doppler_hz) if i != 5]
        Carrier_phase_hz = [row for i, row in enumerate(
            Carrier_phase_hz) if i != 5]
        Pseudorange_m = [row for i, row in enumerate(Pseudorange_m) if i != 5]
        PRN = [row for i, row in enumerate(PRN) if i != 5]
        valid = [row for i, row in enumerate(valid) if i != 5]

        observables = {
            'RX_time': RX_time,
            'd_TOW_at_current_symbol': d_TOW_at_current_symbol,
            'Carrier_Doppler_hz': Carrier_Doppler_hz,
            'Carrier_phase_hz': Carrier_phase_hz,
            'Pseudorange_m': Pseudorange_m,
            'PRN': PRN,
            'valid':valid
        }

    f.close()

    return observables
