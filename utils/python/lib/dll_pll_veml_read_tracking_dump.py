"""
 dll_pll_veml_read_tracking_dump.py
   dll_pll_veml_read_tracking_dump (filename)

 Read GNSS-SDR Tracking dump binary file into Python.
 Opens GNSS-SDR tracking binary log file .dat and returns the contents

 Irene Pérez Riega, 2023. iperrie@inta.es
 Minhaj Uddin Ahmad, 2026. mahmad12@crimson.ua.edu

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

# Binary layout of one tracking dump record, matching the writer in
# src/algorithms/tracking/gnuradio_blocks/dll_pll_veml_tracking.cc
#
# Fields are written back-to-back.
# '<' selects little-endian / standard sizes / no alignment padding.
_RECORD_FORMAT = '<' + (
    'f'    # VE  -> Magnitude of the Very Early correlator.
    'f'    # E   -> Magnitude of the Early correlator.
    'f'    # P   -> Magnitude of the Prompt correlator.
    'f'    # L   -> Magnitude of the Late correlator.
    'f'    # VL  -> Magnitude of the Very Late correlator.
    'f'    # prompt_I -> Prompt correlator, In-phase component.
    'f'    # prompt_Q -> Prompt correlator, Quadrature component.
    'Q'    # PRN_start_sample -> Sample counter from tracking start.
    'f'    # acc_carrier_phase_rad -> Accumulated carrier phase, in rad.
    'f'    # carrier_doppler_hz -> Doppler shift, in Hz.
    'f'    # carrier_doppler_rate_hz_s -> Doppler rate, in Hz/s.
    'f'    # code_freq_hz -> Code frequency, in chips/s.
    'f'    # code_freq_rate_hz_s -> Code frequency rate, in chips/s^2.
    'f'    # carr_error -> Raw carrier error at the PLL output, in Hz.
    'f'    # carr_nco -> Carrier error at the output of the PLL filter, in Hz.
    'f'    # code_error -> Raw code error at the DLL output, in chips.
    'f'    # code_nco -> Code error at the output of the DLL filter, in chips.
    'f'    # CN0_SNV_dB_Hz -> C/N0 estimation, in dB-Hz.
    'f'    # carrier_lock_test -> Output of the carrier lock test.
    'f'    # var1 -> aux variable.
    'd'    # var2 -> aux variable.
    'I'    # PRN -> Satellite ID.
    'Q'    # TOW_ms -> Time of week, in ms.
    'I'    # WN -> Week number.
)

_FIELD_NAMES = [
    'VE', 'E', 'P', 'L', 'VL', 'prompt_I', 'prompt_Q', 'PRN_start_sample',
    'acc_carrier_phase_rad', 'carrier_doppler_hz', 'carrier_doppler_rate_hz_s',
    'code_freq_hz', 'code_freq_rate_hz_s', 'carr_error', 'carr_nco',
    'code_error', 'code_nco', 'CN0_SNV_dB_Hz', 'carrier_lock_test',
    'var1', 'var2', 'PRN', 'TOW_ms', 'WN',
]


def dll_pll_veml_read_tracking_dump (filename):

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
