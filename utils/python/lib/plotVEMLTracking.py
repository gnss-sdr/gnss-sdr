"""
 plotVEMLTracking.py

   This function plots the tracking results for the given channel list.

 Irene Pérez Riega, 2023. iperrie@inta.es

 plotVEMLTracking(channelNr, trackResults, settings)

   Args:
       channelList     - list of channels to be plotted.
       trackResults    - tracking results from the tracking function.
       settings        - receiver settings.

   Modifiable in the file:
       fig_path        - Path where plots will be save

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


def plotVEMLTracking(channelNr, trackResults, settings):

    # ---------- CHANGE HERE:
    fig_path = '/home/labnav/Desktop/TEST_IRENE/PLOTS/VEMLTracking'
    if not os.path.exists(fig_path):
        os.makedirs(fig_path)

    # Protection - if the list contains incorrect channel numbers
    if channelNr in list(range(1,settings["numberOfChannels"]+1)):

        plt.figure(figsize=(1920 / 120, 1080 / 120))
        plt.clf()
        plt.gcf().canvas.set_window_title(
            f'Channel {channelNr} (PRN '
            f'{trackResults[channelNr-1]["PRN"][0]}) results')
        plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1,
                            hspace=0.4, wspace=0.4)

        # Extract timeAxis and time_label
        if 'prn_start_time_s' in trackResults[channelNr-1]:
            timeAxis = trackResults[channelNr-1]['prn_start_time_s']
            time_label = 'RX Time (s)'
        else:
            timeAxis = np.arange(1, len(trackResults[channelNr-1]['PRN']) + 1)
            time_label = 'Epoch'

        len_dataI = len (trackResults[channelNr-1]["data_I"])
        len_dataQ = len (trackResults[channelNr-1]["data_Q"])

        if len_dataI < len_dataQ:
            dif = len_dataQ - len_dataI
            trackResults[channelNr-1]["data_I"] = np.pad(
                trackResults[channelNr-1]["data_I"], pad_width=(0,dif),
                mode="constant", constant_values=0)
        elif len_dataQ < len_dataI:
            dif = len_dataI - len_dataQ
            trackResults[channelNr-1]["data_Q"] = np.pad(
                trackResults[channelNr-1]["data_Q"], pad_width=(0,dif),
                mode="constant", constant_values=0 )

        # Discrete-Time Scatter Plot
        plt.subplot(3, 3, 1)
        plt.plot(trackResults[channelNr-1]['data_I'],
                 trackResults[channelNr-1]['data_Q'], marker='.',
                 markersize=1, linestyle=' ')
        plt.grid()
        plt.axis('equal')
        plt.title('Discrete-Time Scatter Plot', fontweight='bold')
        plt.xlabel('I prompt')
        plt.ylabel('Q prompt')

        # Nav bits
        plt.subplot(3, 3, (2, 3))
        plt.plot(timeAxis, trackResults[channelNr-1]['data_I'],
                 linewidth=1)
        plt.grid()
        plt.title('Bits of the navigation message', fontweight='bold')
        plt.xlabel(time_label)
        plt.axis('tight')

        # Raw PLL discriminator unfiltered
        plt.subplot(3, 3, 4)
        plt.plot(timeAxis, trackResults[channelNr-1]['pllDiscr'],
                 color='r', linewidth=1)
        plt.grid()
        plt.axis('tight')
        plt.xlabel(time_label)
        plt.ylabel('Amplitude')
        plt.title('Raw PLL discriminator', fontweight='bold')

        # Correlation results
        plt.subplot(3, 3, (5, 6))
        corr_data = [
            np.sqrt(trackResults[channelNr-1]['I_VE'] ** 2 +
                    trackResults[channelNr-1]['Q_VE'] ** 2),
            np.sqrt(trackResults[channelNr-1]['I_E'] ** 2 +
                    trackResults[channelNr-1]['Q_E'] ** 2),
            np.sqrt(trackResults[channelNr-1]['I_P'] ** 2 +
                    trackResults[channelNr-1]['Q_P'] ** 2),
            np.sqrt(trackResults[channelNr-1]['I_L'] ** 2 +
                    trackResults[channelNr-1]['Q_L'] ** 2),
            np.sqrt(trackResults[channelNr-1]['I_VL'] ** 2 +
                    trackResults[channelNr-1]['Q_VL'] ** 2)
        ]

        line = []
        colors = ['b','#FF6600','#FFD700','purple','g']

        for i, data in enumerate(corr_data):
            line.append(plt.plot(timeAxis, data, label=f'Data {i+1}',
                                 color=colors[i], marker='*', linestyle=' ',
                                 linewidth=1))

        plt.grid()
        plt.title('Correlation results',fontweight='bold')
        plt.xlabel(time_label)
        plt.axis('tight')
        plt.legend([r'$\sqrt{I_{VE}^2 + Q_{VE}^2}$',
                    r'$\sqrt{I_{E}^2 + Q_{E}^2}$',
                    r'$\sqrt{I_{P}^2 + Q_{P}^2}$',
                    r'$\sqrt{I_{L}^2 + Q_{L}^2}$',
                    r'$\sqrt{I_{VL}^2 + Q_{VL}^2}$'], loc='best')

        # Filtered PLL discriminator
        plt.subplot(3, 3, 7)
        plt.plot(timeAxis, trackResults[channelNr-1]['pllDiscrFilt'],
                 'b', linewidth=1)
        plt.grid()
        plt.axis('tight')
        plt.xlabel(time_label)
        plt.ylabel('Amplitude')
        plt.title('Filtered PLL discriminator', fontweight='bold')

        # Raw DLL discriminator unfiltered
        plt.subplot(3, 3, 8)
        plt.plot(timeAxis, trackResults[channelNr-1]['dllDiscr'], 'r',
                 linewidth=1)
        plt.grid()
        plt.axis('tight')
        plt.xlabel(time_label)
        plt.ylabel('Amplitude')
        plt.title('Raw DLL discriminator',fontweight='bold')

        # Filtered DLL discriminator
        plt.subplot(3, 3, 9)
        plt.plot(timeAxis, trackResults[channelNr-1]['dllDiscrFilt'],
                 'b', linewidth=1)
        plt.grid()
        plt.axis('tight')
        plt.xlabel(time_label)
        plt.ylabel('Amplitude')
        plt.title('Filtered DLL discriminator',fontweight='bold')

    plt.savefig(os.path.join(fig_path,
                             f'Ch{channelNr}_PRN'
                             f'{trackResults[channelNr-1]["PRN"][0]}'
                             f'_results'))
    plt.show()
