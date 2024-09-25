"""
 plot_tracking_quality_indicators.py



 Irene Pérez Riega, 2023. iperrie@inta.es

 Modifiable in the file:
   channels             - Number of channels
   firs_channel         - Number of the first channel
   path                 - Path to folder which contains raw files
   fig_path             - Path where doppler plots will be save
   'trk_dump_ch'        - Fixed part of the tracking dump files names

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""


import matplotlib.pyplot as plt
import numpy as np
import os
from lib.dll_pll_veml_read_tracking_dump import dll_pll_veml_read_tracking_dump

GNSS_tracking = []
plot_names = []

# ---------- CHANGE HERE:
channels = 5
first_channel = 0
path = '/home/labnav/Desktop/TEST_IRENE/tracking'
fig_path = '/home/labnav/Desktop/TEST_IRENE/PLOTS/TrackingQualityIndicator'

for N in range(1, channels + 1):
    tracking_log_path = os.path.join(path,
                                     f'trk_dump_ch{N-1+first_channel}.dat')
    GNSS_tracking.append(dll_pll_veml_read_tracking_dump(tracking_log_path))

if not os.path.exists(fig_path):
    os.makedirs(fig_path)

# Plot tracking quality indicators
# First plot
plt.figure()
plt.gcf().canvas.manager.set_window_title('Carrier lock test output for all '
                                          'the channels')
plt.title('Carrier lock test output for all the channels')
for n in range(len(GNSS_tracking)):
    plt.plot(GNSS_tracking[n]['carrier_lock_test'])
    plot_names.append(f'SV {str(round(np.mean(GNSS_tracking[n]["PRN"])))}')
plt.legend(plot_names)
plt.savefig(os.path.join(fig_path,
                         f'carrier_lock_test '
                         f'{str(round(np.mean(GNSS_tracking[n]["PRN"])))}'))
plt.show()

# Second plot
plt.figure()
plt.gcf().canvas.manager.set_window_title('Carrier CN0 output for all the '
                                          'channels')
plt.title('Carrier CN0 output for all the channels')
for n in range(len(GNSS_tracking)):
    plt.plot(GNSS_tracking[n]['CN0_SNV_dB_Hz'])
    plot_names.append(f'CN0_SNV_dB_Hz '
                      f'{str(round(np.mean(GNSS_tracking[n]["PRN"])))}')
plt.legend(plot_names)
plt.savefig(os.path.join(
    fig_path, f'SV {str(round(np.mean(GNSS_tracking[n]["PRN"])))}'))
plt.show()
