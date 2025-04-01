"""
 plotKalman.py
   plotKalman (channelNr, trackResults, settings)

 This function plots the tracking results for the given channel list.

 Irene Pérez Riega, 2023. iperrie@inta.es

      Args:
        channelList     - list of channels to be plotted.
        trackResults    - tracking results from the tracking function.
        settings        - receiver settings.

      Modifiable in the file:
        fig_path        - Path where doppler plots will be save

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


def plotKalman(channelNr, trackResults, settings):

    # ---------- CHANGE HERE:
    fig_path = '/home/labnav/Desktop/TEST_IRENE/PLOTS/PlotKalman'

    if not os.path.exists(fig_path):
        os.makedirs(fig_path)

    # Protection - if the list contains incorrect channel numbers
    channelNr = np.intersect1d(channelNr,
                               np.arange(1, settings['numberOfChannels'] + 1))

    for channelNr in channelNr:
        time_start = settings['timeStartInSeconds']
        time_axis_in_seconds = np.arange(1, settings['msToProcess']+1)/1000

        # Plot all figures
        plt.figure(figsize=(1920 / 100, 1080 / 100))
        plt.clf()
        plt.gcf().canvas.set_window_title(
            f'Channel {channelNr} (PRN '
            f'{str(trackResults[channelNr-1]["PRN"][-2])}) results')
        plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1,
                            hspace=0.4, wspace=0.4)
        plt.tight_layout()

        # Row 1
        # ----- CNo for signal -----------------------------------------------
        # Measure of the ratio between carrier signal power and noise power
        plt.subplot(4, 2, 1)
        plt.plot(time_axis_in_seconds,
                 trackResults[channelNr-1]['CNo'][:settings['msToProcess']],
                 'b')
        plt.grid()
        plt.axis('tight')
        plt.xlabel('Time (s)')
        plt.ylabel('CNo (dB-Hz)')
        plt.title('Carrier to Noise Ratio', fontweight='bold')

        # ----- PLL discriminator filtered -----------------------------------
        plt.subplot(4, 2, 2)
        plt.plot(time_axis_in_seconds,
                 trackResults[channelNr-1]['state1']
                 [:settings['msToProcess']], 'b')
        plt.grid()
        plt.axis('tight')
        plt.xlim([time_start, time_axis_in_seconds[-1]])
        plt.xlabel('Time (s)')
        plt.ylabel('Phase Amplitude')
        plt.title('Filtered Carrier Phase', fontweight='bold')

        # Row 2
        # ----- Carrier Frequency --------------------------------------------
        # Filtered carrier frequency of (transmitted by a satellite)
        # for a specific channel
        plt.subplot(4, 2, 3)
        plt.plot(time_axis_in_seconds[1:],
                 trackResults[channelNr-1]['state2']
                 [1:settings['msToProcess']], color=[0.42, 0.25, 0.39])
        plt.grid()
        plt.axis('auto')
        plt.xlim(time_start, time_axis_in_seconds[-1])
        plt.xlabel('Time (s)')
        plt.ylabel('Freq (Hz)')
        plt.title('Filtered Carrier Frequency', fontweight='bold')

        # ----- Carrier Frequency Rate ---------------------------------------
        plt.subplot(4, 2, 4)
        plt.plot(time_axis_in_seconds[1:],
                 trackResults[channelNr-1]['state3']
                 [1:settings['msToProcess']], color=[0.42, 0.25, 0.39])
        plt.grid()
        plt.axis('auto')
        plt.xlim(time_start, time_axis_in_seconds[-1])
        plt.xlabel('Time (s)')
        plt.ylabel('Freq (Hz)')
        plt.title('Filtered Carrier Frequency Rate', fontweight='bold')

        # Row 3
        # ----- PLL discriminator unfiltered----------------------------------
        plt.subplot(4, 2, (5,6))
        plt.plot(time_axis_in_seconds,
                 trackResults[channelNr-1]['innovation'], 'r')
        plt.grid()
        plt.axis('auto')
        plt.xlim(time_start, time_axis_in_seconds[-1])
        plt.xlabel('Time (s)')
        plt.ylabel('Amplitude')
        plt.title('Raw PLL discriminator (Innovation)',fontweight='bold')

        # Row 4
        # ----- PLL discriminator covariance ---------------------------------
        plt.subplot(4, 2, (7,8))
        plt.plot(time_axis_in_seconds,
                 trackResults[channelNr-1]['r_noise_cov'], 'r')
        plt.grid()
        plt.axis('auto')
        plt.xlim(time_start, time_axis_in_seconds[-1])
        plt.xlabel('Time (s)')
        plt.ylabel('Variance')
        plt.title('Estimated Noise Variance', fontweight='bold')

        plt.tight_layout()
        plt.savefig(os.path.join(fig_path,
                                 f'kalman_ch{channelNr}_PRN_'
                                 f'{trackResults[channelNr - 1]["PRN"][-1]}'
                                 f'.png'))
        plt.show()
