"""
 gps_l1_ca_kf_read_tracking_dump.py
   gps_l1_ca_kf_read_tracking_dump (filename)

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

# Binary layout of one GPS_L1_CA_KF_Tracking dump record, matching the writer
# in src/algorithms/tracking/gnuradio_blocks/kf_tracking.cc
#
# Fields are written back-to-back. Unlike the DLL/PLL VEML dump, the KF dump
# ends at PRN (no TOW_ms / WN fields), so a record is 22 fields / 96 bytes.
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
    'f'    # carrier_doppler_hz -> KF carrier Doppler estimate, in Hz.
    'f'    # carrier_doppler_rate_hz2 -> KF carrier Doppler rate, in Hz/s.
    'f'    # code_freq_hz -> KF code frequency, in chips/s.
    'f'    # code_freq_rate_hz_s -> Code phase rate, in chips/s^2.
    'f'    # carr_error -> Carrier phase discriminator output, in Hz.
    'f'    # carr_nco -> KF carrier state estimate.
    'f'    # code_error -> Code discriminator output, in chips.
    'f'    # code_nco -> KF code error estimate, in chips.
    'f'    # CN0_SNV_dB_Hz -> C/N0 estimation, in dB-Hz.
    'f'    # carrier_lock_test -> Output of the carrier lock test.
    'f'    # var1 -> aux variable.
    'd'    # var2 -> aux variable.
    'I'    # PRN -> Satellite ID.
)

_FIELD_NAMES = [
    'VE', 'E', 'P', 'L', 'VL', 'prompt_I', 'prompt_Q', 'PRN_start_sample',
    'acc_carrier_phase_rad', 'carrier_doppler_hz', 'carrier_doppler_rate_hz2',
    'code_freq_hz', 'code_freq_rate_hz_s', 'carr_error', 'carr_nco',
    'code_error', 'code_nco', 'CN0_SNV_dB_Hz', 'carrier_lock_test',
    'var1', 'var2', 'PRN',
]


def gps_l1_ca_kf_read_tracking_dump (filename):

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
