"""
 gps_l1_ca_kf_plot_sample.py

 Reads GNSS-SDR Tracking dump binary file using the provided
 function and plots some internal variables

 Irene Pérez Riega, 2023. iperrie@inta.es

 Modifiable in the file:
   sampling_freq     - Sampling frequency [Hz]
   channels          - Number of channels to check if they exist
   first_channel     - Number of the first channel
   path              - Path to folder which contains raw file
   'trk_dump_ch'     - Fixed part in tracking dump files names

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
# Signal parameters:
samplingFreq = 6625000
channels = 5
first_channel = 0
code_period = 0.001

path = '/home/labnav/Desktop/TEST_IRENE/tracking'

for N in range(1, channels + 1):
    tracking_log_path = os.path.join(path,
                                     f'trk_dump_ch{N-1+first_channel}.dat')
    GNSS_tracking.append(gps_l1_ca_kf_read_tracking_dump(tracking_log_path))

# todo lee lo mismo que dll_pll_velm_plot_sample y guarda diferente v13 y v14
# GNSS-SDR format conversion to Python GPS receiver
for N in range(1, channels + 1):
    trackResult= {
        'status': 'T',  # fake track
        'codeFreq': np.copy(GNSS_tracking[N - 1]["code_freq_hz"]),
        'carrFreq': np.copy(GNSS_tracking[N - 1]["carrier_doppler_hz"]),
        'carrFreqRate':
            np.copy(GNSS_tracking[N - 1]["carrier_doppler_rate_hz2"]),#todo no se usa en dll, carrier_doppler_rate_hz_s segun dll
        'dllDiscr': np.copy(GNSS_tracking[N - 1]["code_error"]),
        'dllDiscrFilt': np.copy(GNSS_tracking[N - 1]["code_nco"]),
        'pllDiscr': np.copy(GNSS_tracking[N - 1]["carr_error"]),#todo code_freq_rate_hz_s segun dll
        'pllDiscrFilt': np.copy(GNSS_tracking[N - 1]["carr_nco"]),

        'I_P': np.copy(GNSS_tracking[N - 1]["prompt_I"]),#todo distinto de dll
        'Q_P': np.copy(GNSS_tracking[N - 1]["prompt_Q"]),#todo distinto de dll

        'I_E': np.copy(GNSS_tracking[N - 1]["E"]),
        'I_L': np.copy(GNSS_tracking[N - 1]["L"]),
        'Q_E': np.zeros(len(GNSS_tracking[N - 1]["E"])),
        'Q_L': np.zeros(len(GNSS_tracking[N - 1]["L"])),
        'PRN': np.copy(GNSS_tracking[N - 1]["PRN"]),
        'CNo': np.copy(GNSS_tracking[N - 1]["CN0_SNV_dB_Hz"])
    }

    kalmanResult= {
        'PRN': np.copy(GNSS_tracking[N - 1]["PRN"]),
        'innovation': np.copy(GNSS_tracking[N - 1]["carr_error"]),#todo code_freq_rate_hz_s segun dll
        'state1': np.copy(GNSS_tracking[N - 1]["carr_nco"]),
        'state2': np.copy(GNSS_tracking[N - 1]["carrier_doppler_hz"]),
        'state3': GNSS_tracking[N - 1]["carrier_doppler_rate_hz2"],#todo segun el dll es carrier_doppler_rate_hz_s
        'r_noise_cov': np.copy(GNSS_tracking[N - 1]["carr_noise_sigma2"]),#todo carr_error segun dll
        'CNo': np.copy(GNSS_tracking[N - 1]["CN0_SNV_dB_Hz"])
    }

    trackResults.append(trackResult)
    kalmanResults.append(kalmanResult)

    settings = {
        'numberOfChannels': channels,
        'msToProcess': len(GNSS_tracking[N-1]['E']),
        'codePeriod': code_period,
        'timeStartInSeconds': 20
    }

    # Create and save graphics as PNG
    plotTracking(N, trackResults, settings)
    plotKalman(N, kalmanResults, settings)
