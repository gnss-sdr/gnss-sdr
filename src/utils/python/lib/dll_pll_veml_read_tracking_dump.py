"""
 dll_pll_veml_read_tracking_dump.py
   dll_pll_veml_read_tracking_dump (filename)

 Read GNSS-SDR Tracking dump binary file into Python.
 Opens GNSS-SDR tracking binary log file .dat and returns the contents

 Irene Pérez Riega, 2023. iperrie@inta.es

      Args:
        filename: path to file .dat with the raw data

      Return:
        GNSS_tracking: A dictionary with the processed data in lists

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import struct
import sys


def dll_pll_veml_read_tracking_dump (filename):

    v1 = []
    v2 = []
    v3 = []
    v4 = []
    v5 = []
    v6 = []
    v7 = []
    v8 = []
    v9 = []
    v10= []
    v11 = []
    v12 = []
    v13 = []
    v14 = []
    v15 = []
    v16 = []
    v17 = []
    v18 = []
    v19 = []
    v20 = []
    v21 = []
    v22 = []
    GNSS_tracking = {}

    bytes_shift = 0

    if sys.maxsize > 2 ** 36:  # 64 bits computer
        float_size_bytes = 4
        unsigned_long_int_size_bytes = 8
        double_size_bytes = 8
        unsigned_int_size_bytes = 4

    else: # 32 bits
        float_size_bytes = 4
        unsigned_long_int_size_bytes = 4
        double_size_bytes = 8
        unsigned_int_size_bytes = 4

    f = open(filename, 'rb')
    if f is None:
        return None
    else:
        while True:
            f.seek(bytes_shift, 0)
            # VE -> Magnitude of the Very Early correlator.
            v1.append(struct.unpack('f', f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # E -> Magnitude of the Early correlator.
            v2.append(struct.unpack('f', f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # P -> Magnitude of the Prompt correlator.
            v3.append(struct.unpack('f', f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # L -> Magnitude of the Late correlator.
            v4.append(struct.unpack('f', f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # VL -> Magnitude of the Very Late correlator.
            v5.append(struct.unpack('f', f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # promp_I -> Value of the Prompt correlator in the In-phase
            # component.
            v6.append(struct.unpack('f', f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # promp_Q -> Value of the Prompt correlator in the Quadrature
            # component.
            v7.append(struct.unpack('f',
                                    f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # PRN_start_sample -> Sample counter from tracking start.
            if unsigned_long_int_size_bytes == 8:
                v8.append(struct.unpack(
                    'Q', f.read(unsigned_long_int_size_bytes))[0])
                bytes_shift += unsigned_long_int_size_bytes
            else:
                v8.append(struct.unpack('I',
                                        f.read(unsigned_int_size_bytes))[0])
                bytes_shift += unsigned_int_size_bytes
            f.seek(bytes_shift, 0)
            # acc_carrier_phase_rad - > Accumulated carrier phase, in rad.
            v9.append(struct.unpack('f', f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # carrier doppler hz -> Doppler shift, in Hz.
            v10.append(struct.unpack('f',
                                     f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # carrier doppler rate hz s -> Doppler rate, in Hz/s.
            v11.append(struct.unpack('f',
                                     f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # code freq hz -> Code frequency, in chips/s.
            v12.append(struct.unpack('f',
                                     f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # code_freq_rate_hz_s -> Code frequency rate, in chips/s².
            v13.append(struct.unpack('f',
                                     f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # carr_error -> Raw carrier error (unfiltered) at the PLL
            # output, in Hz.
            v14.append(struct.unpack('f',
                                     f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # carr_nco -> Carrier error at the output of the PLL
            # filter, in Hz.
            v15.append(struct.unpack('f',
                                     f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # code error -> Raw code error (unfiltered) at the DLL
            # output, in chips.
            v16.append(struct.unpack('f',
                                     f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # code nco -> Code error at the output of the DLL
            # filter, in chips.
            v17.append(struct.unpack('f',
                                     f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # CN0_SNV_dB_Hz -> C/N0 estimation, in dB-Hz.
            v18.append(struct.unpack('f',
                                     f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # carrier lock test -> Output of the carrier lock test.
            v19.append(struct.unpack('f',
                                     f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # var 1 -> not used ?
            v20.append(struct.unpack('f',
                                     f.read(float_size_bytes))[0])
            bytes_shift += float_size_bytes
            f.seek(bytes_shift, 0)
            # var 2 -> not used ?
            v21.append(struct.unpack('d',
                                     f.read(double_size_bytes))[0])
            bytes_shift += double_size_bytes
            f.seek(bytes_shift, 0)
            # PRN ->  Satellite ID.
            v22.append(struct.unpack('I',
                                     f.read(unsigned_int_size_bytes))[0])
            bytes_shift += unsigned_int_size_bytes
            f.seek(bytes_shift, 0)

            # Check file
            linea = f.readline()
            if not linea:
                break

        f.close()

        GNSS_tracking['VE'] = v1
        GNSS_tracking['E'] = v2
        GNSS_tracking['P'] = v3
        GNSS_tracking['L'] = v4
        GNSS_tracking['VL'] = v5
        GNSS_tracking['prompt_I'] = v6
        GNSS_tracking['prompt_Q'] = v7
        GNSS_tracking['PRN_start_sample'] = v8
        GNSS_tracking['acc_carrier_phase_rad'] = v9
        GNSS_tracking['carrier_doppler_hz'] = v10
        GNSS_tracking['carrier_doppler_rate_hz_s'] = v11
        GNSS_tracking['code_freq_hz'] = v12
        GNSS_tracking['code_freq_rate_hz_s'] = v13
        GNSS_tracking['carr_error'] = v14
        GNSS_tracking['carr_nco'] = v15
        GNSS_tracking['code_error'] = v16
        GNSS_tracking['code_nco'] = v17
        GNSS_tracking['CN0_SNV_dB_Hz'] = v18
        GNSS_tracking['carrier_lock_test'] = v19
        GNSS_tracking['var1'] = v20
        GNSS_tracking['var2'] = v21
        GNSS_tracking['PRN'] = v22

    return GNSS_tracking
