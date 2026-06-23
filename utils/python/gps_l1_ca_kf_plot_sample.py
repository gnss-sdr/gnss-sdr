"""
 gps_l1_ca_kf_plot_sample.py

 Reads a GPS L1 C/A Kalman-filter tracking dump binary file
 (GPS_L1_CA_KF_Tracking) and plots some internal tracking and Kalman-filter
 variables.

 Irene Pérez Riega, 2023. iperrie@inta.es
 Minhaj Uddin Ahmad, 2026. mahmad12@crimson.ua.edu

 Modifiable in the file:
   samplingFreq      - Sampling frequency [Hz]
   channels          - Number of channels
   first_channel     - Number of the first channel
   code_period       - Code period [s]
   path              - Path to folder which contains the tracking dump files
   fig_path          - Path where the plots will be saved
   file_prefix       - Fixed part of the tracking dump file names

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import os
import numpy as np
from lib.gps_l1_ca_kf_read_tracking_dump import gps_l1_ca_kf_read_tracking_dump
from lib.plotTracking import plotTracking
from lib.plotKalman import plotKalman

GNSS_tracking = []
trackResults = []
kalmanResults = []

# ---------- CHANGE HERE:
samplingFreq = 4000000  # must match SignalSource.sampling_frequency in the .conf
channels = 5
first_channel = 0
code_period = 0.001
path = '../../../out/'
fig_path = '../../../PLOTS/KF_Tracking'
file_prefix = 'track_ch'

for N in range(1, channels + 1):
    tracking_log_path = os.path.join(path,
                                     f'{file_prefix}{N-1+first_channel}.dat')
    GNSS_tracking.append(gps_l1_ca_kf_read_tracking_dump(tracking_log_path))

for N in range(1, channels + 1):
    trackResult = {
        'status': 'T',  # fake track
        'codeFreq': np.copy(GNSS_tracking[N-1]["code_freq_hz"]),
        'carrFreq': np.copy(GNSS_tracking[N-1]["carrier_doppler_hz"]),
        'carrFreqRate':
            np.copy(GNSS_tracking[N-1]["carrier_doppler_rate_hz2"]),
        'dllDiscr': np.copy(GNSS_tracking[N-1]["code_error"]),
        'dllDiscrFilt': np.copy(GNSS_tracking[N-1]["code_nco"]),
        'pllDiscr': np.copy(GNSS_tracking[N-1]["carr_error"]),
        'pllDiscrFilt': np.copy(GNSS_tracking[N-1]["carr_nco"]),

        'I_P': np.copy(GNSS_tracking[N-1]["prompt_I"]),
        'Q_P': np.copy(GNSS_tracking[N-1]["prompt_Q"]),

        'I_E': np.copy(GNSS_tracking[N-1]["E"]),
        'I_L': np.copy(GNSS_tracking[N-1]["L"]),
        'Q_E': np.zeros(len(GNSS_tracking[N-1]["E"])),
        'Q_L': np.zeros(len(GNSS_tracking[N-1]["L"])),
        'PRN': np.copy(GNSS_tracking[N-1]["PRN"]),
        'CNo': np.copy(GNSS_tracking[N-1]["CN0_SNV_dB_Hz"]),
        'prn_start_time_s':
            np.copy(GNSS_tracking[N-1]["PRN_start_sample"]) / samplingFreq
    }

    # Kalman-filter internals. The KF dump stores the carrier discriminator
    # (carr_error -> innovation) and the filter states, but no measurement
    # noise covariance, so 'r_noise_cov' is intentionally left out (plotKalman
    # skips that panel when it is absent).
    kalmanResult = {
        'PRN': np.copy(GNSS_tracking[N-1]["PRN"]),
        'innovation': np.copy(GNSS_tracking[N-1]["carr_error"]),
        'state1': np.copy(GNSS_tracking[N-1]["carr_nco"]),
        'state2': np.copy(GNSS_tracking[N-1]["carrier_doppler_hz"]),
        'state3': np.copy(GNSS_tracking[N-1]["carrier_doppler_rate_hz2"]),
        'CNo': np.copy(GNSS_tracking[N-1]["CN0_SNV_dB_Hz"])
    }

    trackResults.append(trackResult)
    kalmanResults.append(kalmanResult)

    settings = {
        'numberOfChannels': channels,
        'msToProcess': len(GNSS_tracking[N-1]['E']),
        'codePeriod': code_period,
        'timeStartInSeconds': 0,
        'fig_path': fig_path,
        'show': False  # set True to display the figures interactively
    }

    # Create and save graphics as PNG
    plotTracking(N, trackResults, settings)
    plotKalman(N, kalmanResults, settings)
