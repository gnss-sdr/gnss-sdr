"""
 dll_pll_veml_plot_sample.py

 Reads GNSS-SDR Tracking dump binary file using the provided function and
 plots some internal variables

 Irene Pérez Riega, 2023. iperrie@inta.es

 Modifiable in the file:
   sampling_freq     - Sampling frequency [Hz]
   plot_last_outputs - If 0 -> process everything / number of items processed
   channels          - Number of channels
   first_channel     - Number of the first channel
   doppler_opt       - = 1 -> Plot // = 0 -> No plot
   path              - Path to folder which contains raw files
   fig_path          - Path where doppler plots will be save
   'trk_dump_ch'     - Fixed part of the tracking dump files names

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from lib.dll_pll_veml_read_tracking_dump import dll_pll_veml_read_tracking_dump
from lib.plotVEMLTracking import plotVEMLTracking

trackResults = []
settings = {}
GNSS_tracking = []

# ---------- CHANGE HERE:
sampling_freq = 3000000
plot_last_outputs = 0
channels = 5
first_channel = 0
doppler_opt = 1
settings['numberOfChannels'] = channels

path = '/home/labnav/Desktop/TEST_IRENE/tracking'
fig_path = '/home/labnav/Desktop/TEST_IRENE/PLOTS/Doppler'

for N in range(1, channels+1):
    tracking_log_path = os.path.join(path,
                                     f'trk_dump_ch{N-1+first_channel}.dat')
    GNSS_tracking.append(dll_pll_veml_read_tracking_dump(tracking_log_path))

# GNSS-SDR format conversion to Python GPS receiver
for N in range (1, channels+1):
    if 0 < plot_last_outputs < len(GNSS_tracking[N - 1].get("code_freq_hz")):
        start_sample = (len(GNSS_tracking[N-1].get("code_freq_hz")) -
                        plot_last_outputs)
    else:
        start_sample = 0

    trackResult = {
        'status': 'T',  # fake track
        'codeFreq': np.copy(GNSS_tracking[N-1]["code_freq_hz"][start_sample:]),
        'carrFreq': np.copy(GNSS_tracking[N-1]["carrier_doppler_hz"][start_sample:]),
        'dllDiscr': np.copy(GNSS_tracking[N-1]["code_error"][start_sample:]),
        'dllDiscrFilt': np.copy(GNSS_tracking[N-1]["code_nco"][start_sample:]),
        'pllDiscr': np.copy(GNSS_tracking[N-1]["carr_error"][start_sample:]),
        'pllDiscrFilt': np.copy(GNSS_tracking[N-1]["carr_nco"][start_sample:]),

        'I_P': np.copy(GNSS_tracking[N-1]["P"][start_sample:]),
        'Q_P': np.zeros(len(GNSS_tracking[N-1]["P"][start_sample:])),

        'I_VE': np.copy(GNSS_tracking[N-1]["VE"][start_sample:]),
        'I_E': np.copy(GNSS_tracking[N-1]["E"][start_sample:]),
        'I_L': np.copy(GNSS_tracking[N-1]["L"][start_sample:]),
        'I_VL': np.copy(GNSS_tracking[N-1]["VL"][start_sample:]),
        'Q_VE': np.zeros(len(GNSS_tracking[N-1]["VE"][start_sample:])),
        'Q_E': np.zeros(len(GNSS_tracking[N-1]["E"][start_sample:])),
        'Q_L': np.zeros(len(GNSS_tracking[N-1]["L"][start_sample:])),
        'Q_VL': np.zeros(len(GNSS_tracking[N-1]["VL"][start_sample:])),
        'data_I': np.copy(GNSS_tracking[N-1]["prompt_I"][start_sample:]),
        'data_Q': np.copy(GNSS_tracking[N-1]["prompt_Q"][start_sample:]),
        'PRN': np.copy(GNSS_tracking[N-1]["PRN"][start_sample:]),
        'CNo': np.copy(GNSS_tracking[N-1]["CN0_SNV_dB_Hz"][start_sample:]),
        'prn_start_time_s': np.copy(GNSS_tracking[N-1]["PRN_start_sample"]
                                    [start_sample:]) / sampling_freq
    }
    trackResults.append(trackResult)

    # Plot results:
    plotVEMLTracking(N,trackResults,settings)

    # Plot Doppler according to selected in doppler_opt variable:
    if doppler_opt == 1:
        if not os.path.exists(fig_path):
            os.makedirs(fig_path)

        plt.figure()
        plt.plot(trackResults[N - 1]['prn_start_time_s'],
                 [x/1000 for x in GNSS_tracking[N - 1]['carrier_doppler_hz']
                 [start_sample:]])
        plt.xlabel('Time(s)')
        plt.ylabel('Doppler(KHz)')
        plt.title('Doppler frequency channel ' + str(N))

        plt.savefig(os.path.join(fig_path, f'Doppler_freq_ch_{N}.png'))
        plt.show()
