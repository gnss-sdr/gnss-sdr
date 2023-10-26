"""
 gps_l1_ca_telemetry_plot_sample.py

 Reads GNSS-SDR Tracking dump binary file using the provided function and
 plots some internal variables

 Irene Pérez Riega, 2023. iperrie@inta.es

 Modifiable in the file:
   sampling_freq     - Sampling frequency [Hz]
   channels          - Number of channels to check if they exist
   doppler_opt       - = 1 -> Plot // = 0 -> No plot
   path              - Path to folder which contains raw file
   fig_path          - Path where plots will be save
   chn_num_a / b     - Channel which will be plotted

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import os
import matplotlib.pyplot as plt
from lib.gps_l1_ca_read_telemetry_dump import gps_l1_ca_read_telemetry_dump

GNSS_telemetry = []

# ---------- CHANGE HERE:
sampling_freq = 2000000
channels = list(range(18))
path = '/home/labnav/Desktop/TEST_IRENE/'
fig_path = '/home/labnav/Desktop/TEST_IRENE/PLOTS/Telemetry'

if not os.path.exists(fig_path):
    os.makedirs(fig_path)

i = 0
for N in channels:
    try:
        telemetry_log_path = os.path.join(path, f'telemetry{N}.dat')
        telemetry_data = gps_l1_ca_read_telemetry_dump(telemetry_log_path)
        GNSS_telemetry.append(telemetry_data)
        i += 1
    except:
        pass

# ---------- CHANGE HERE:
chn_num_a = 0
chn_num_b = 5

# Plotting values
if chn_num_a in range(i) and chn_num_b in range(i):

    # First Plot:
    plt.figure()
    plt.gcf().canvas.manager.set_window_title(f'Telem_Current_Simbol_TOW_'
                                              f'{chn_num_a}_{chn_num_b}.png')

    plt.plot(GNSS_telemetry[chn_num_a]['tracking_sample_counter'],
             [x / 1000 for x in GNSS_telemetry[chn_num_a]
             ['tow_current_symbol_ms']], 'b')
    plt.plot(GNSS_telemetry[chn_num_b]['tracking_sample_counter'],
             GNSS_telemetry[chn_num_b]['tow_current_symbol_ms'], 'r')

    plt.grid(True)
    plt.xlabel('TRK Sampling Counter')
    plt.ylabel('Current Symbol TOW')
    plt.legend([f'CHN-{chn_num_a-1}', f'CHN-{chn_num_b-1}'])
    plt.tight_layout()

    plt.savefig(os.path.join(fig_path, f'Telem_Current_Simbol_TOW_{chn_num_a}'
                                       f'_{chn_num_b}.png'))
    plt.show()

    # Second Plot:
    plt.figure()
    plt.gcf().canvas.manager.set_window_title(f'Telem_TRK_Sampling_Counter_'
                                              f'{chn_num_a}_{chn_num_b}.png')

    plt.plot(GNSS_telemetry[chn_num_a]['tracking_sample_counter'],
             GNSS_telemetry[chn_num_a]['tow'], 'b')
    plt.plot(GNSS_telemetry[chn_num_b]['tracking_sample_counter'],
             GNSS_telemetry[chn_num_b]['tow'], 'r')

    plt.grid(True)
    plt.xlabel('TRK Sampling Counter')
    plt.ylabel('Decoded Nav TOW')
    plt.legend([f'CHN-{chn_num_a-1}', f'CHN-{chn_num_b-1}'])
    plt.tight_layout()

    plt.savefig(os.path.join(fig_path, f'Telem_TRK_Sampling_Counter_'
                                       f'{chn_num_a}_{chn_num_b}.png'))
    plt.show()
