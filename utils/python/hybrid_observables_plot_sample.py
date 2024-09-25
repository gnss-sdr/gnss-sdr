"""
 hybrid_observables_plot_sample.py

 Reads GNSS-SDR observables raw dump binary file using the provided function
 and plots some internal variables

 Irene Pérez Riega, 2023. iperrie@inta.es

 Modifiable in the file:
   sampling_freq        - Sampling frequency [Hz]
   channels             - Number of channels to check if they exist
   path                 - Path to folder which contains raw file
   fig_path             - Path where plots will be save
   observables_log_path - Completed path to observables raw data file

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import numpy as np
import matplotlib.pyplot as plt
from lib.read_hybrid_observables_dump import read_hybrid_observables_dump
import os

observables = {}
double_size_bytes = 8
bytes_shift = 0

# ---------- CHANGE HERE:
samplingFreq = 2000000
channels = 5
path = '/home/labnav/Desktop/TEST_IRENE/'
fig_path = '/home/labnav/Desktop/TEST_IRENE/PLOTS/HybridObservables/'
observables_log_path = path + 'observables.dat'

if not os.path.exists(fig_path):
    os.makedirs(fig_path)

GNSS_observables = read_hybrid_observables_dump(channels,observables_log_path)

# Plot data
# --- optional: plot since it detect the first satellite connected
min_tow_idx = 1
obs_idx = 1

for n in range(0, channels):

    idx = np.where(np.array(GNSS_observables['valid'][n])>0)[0][0]
    # Find the index from the first satellite connected
    if n == 0:
        min_tow_idx = idx
    if min_tow_idx > idx:
        min_tow_idx = idx
        obs_idx = n

# Plot observables from that index
# First plot
plt.figure()
plt.title('Pseudorange')
for i in range(channels):
    plt.scatter(GNSS_observables['RX_time'][i][min_tow_idx:],
                GNSS_observables['Pseudorange_m'][i][min_tow_idx:],s=1,
                label=f'Channel {i}')
plt.xlim(GNSS_observables['RX_time'][obs_idx][min_tow_idx]-100,
         GNSS_observables['RX_time'][obs_idx][-1]+100)
plt.grid(True)
plt.xlabel('TOW [s]')
plt.ylabel('Pseudorange [m]')
plt.legend()
plt.gcf().canvas.manager.set_window_title('Pseudorange.png')
plt.tight_layout()
plt.savefig(os.path.join(fig_path, 'Pseudorange.png'))
plt.show()

# Second plot
plt.figure()
plt.title('Carrier Phase')
for i in range(channels):
    plt.scatter(GNSS_observables['RX_time'][i][min_tow_idx:],
                GNSS_observables['Carrier_phase_hz'][i][min_tow_idx:],s=1,
                label=f'Channel {i}')
plt.xlim(GNSS_observables['RX_time'][obs_idx][min_tow_idx]-100,
         GNSS_observables['RX_time'][obs_idx][-1]+100)
plt.xlabel('TOW [s]')
plt.ylabel('Accumulated Carrier Phase [cycles]')
plt.grid(True)
plt.legend()
plt.gcf().canvas.manager.set_window_title('AccumulatedCarrierPhase.png')
plt.tight_layout()
plt.savefig(os.path.join(fig_path, 'AccumulatedCarrierPhase.png'))
plt.show()

# Third plot
plt.figure()
plt.title('Doppler Effect')
for i in range(channels):
    plt.scatter(GNSS_observables['RX_time'][i][min_tow_idx:],
                GNSS_observables['Carrier_Doppler_hz'][i][min_tow_idx:],s=1,
                label=f'Channel {i}')
plt.xlim(GNSS_observables['RX_time'][obs_idx][min_tow_idx]-100,
         GNSS_observables['RX_time'][obs_idx][-1]+100)
plt.xlabel('TOW [s]')
plt.ylabel('Doppler Frequency [Hz]')
plt.grid(True)
plt.legend()
plt.gcf().canvas.manager.set_window_title('DopplerFrequency.png')
plt.tight_layout()
plt.savefig(os.path.join(fig_path, 'DopplerFrequency.png'))
plt.show()

# Fourth plot
plt.figure()
plt.title('GNSS Channels captured')
for i in range(channels):
    lab = 0
    a = 0
    while lab == 0:
        lab = int(GNSS_observables["PRN"][i][min_tow_idx+a])
        a += 1
    plt.scatter(GNSS_observables['RX_time'][i][min_tow_idx:],
                GNSS_observables['PRN'][i][min_tow_idx:], s=1,
                label=f'PRN {i} = {lab}')
plt.xlim(GNSS_observables['RX_time'][obs_idx][min_tow_idx]-100,
         GNSS_observables['RX_time'][obs_idx][-1]+100)
plt.xlabel('TOW [s]')
plt.ylabel('PRN')
plt.grid(True)
plt.legend()
plt.gcf().canvas.manager.set_window_title('PRNs.png')
plt.tight_layout()
plt.savefig(os.path.join(fig_path, 'PRNs.png'))
plt.show()