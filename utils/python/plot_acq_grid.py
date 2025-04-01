"""
 plot_acq_grid.py

 Reads GNSS-SDR Acquisition dump .mat file using the provided function and
 plots acquisition grid of acquisition statistic of PRN sat

 Irene Pérez Riega, 2023. iperrie@inta.es

 Modifiable in the file:
   sampling_freq        - Sampling frequency [Hz]
   channels             - Number of channels to check if they exist
   path                 - Path to folder which contains raw file
   fig_path             - Path where plots will be save
   plot_all_files       - Plot all the files in a folder (True/False)
   ----
   file                 - Fixed part in files names. In our case: acq_dump
   sat                  - Satellite. In our case: 1
   channel              - Channel. In our case: 1
   execution            - In our case: 0
   signal_type          - In our case: 1
   ----
   lite_view            - True for light grid representation

 File format:
   {path}/{file}_ch_{system}_{signal}_ch_{channel}_{execution}_sat_{sat}.mat
 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import h5py

# ---------- CHANGE HERE:
path = '/home/labnav/Desktop/TEST_IRENE/acquisition/'
fig_path = '/home/labnav/Desktop/TEST_IRENE/PLOTS/Acquisition/'
plot_all_files = False

if not os.path.exists(fig_path):
    os.makedirs(fig_path)

if not plot_all_files:

    # ---------- CHANGE HERE:
    file = 'acq_dump'
    sat = 1
    channel = 0
    execution = 1
    signal_type = 1
    lite_view = True

    # If lite_view -> sets the number of samples per chip in the graphical
    # representation
    n_samples_per_chip = 3
    d_samples_per_code = 25000

    signal_types = {
        1: ('G', '1C', 1023), # GPS L1
        2: ('G', '2S', 10230), # GPS L2M
        3: ('G', 'L5', 10230), # GPS L5
        4: ('E', '1B', 4092), # Galileo E1B
        5: ('E', '5X', 10230), # Galileo E5
        6: ('R', '1G', 511), # Glonass 1G
        7: ('R', '2G', 511), # Glonass 2G
        8: ('C', 'B1', 2048), # Beidou B1
        9: ('C', 'B3', 10230), # Beidou B3
        10: ('C', '5C', 10230) # Beidou B2a
    }
    system, signal, n_chips = signal_types.get(signal_type)

    # Load data
    filename = (f'{path}{file}_ch_{system}_{signal}_ch_{channel}_{execution}'
                f'_sat_{sat}.mat')
    img_name_root = (f'{fig_path}{file}_ch_{system}_{signal}_ch_{channel}_'
                   f'{execution}_sat_{sat}')

    with h5py.File(filename, 'r') as data:
        acq_grid = data['acq_grid'][:]
        n_fft, n_dop_bins = acq_grid.shape
        d_max, f_max = np.unravel_index(np.argmax(acq_grid), acq_grid.shape)
        doppler_step = data['doppler_step'][0]
        doppler_max = data['doppler_max'][0]
        freq = np.arange(n_dop_bins) * doppler_step - doppler_max
        delay = np.arange(n_fft) / n_fft * n_chips

    # Plot data
    # --- Acquisition grid (3D)
    fig = plt.figure()
    plt.gcf().canvas.manager.set_window_title(filename)
    if not lite_view:
        ax = fig.add_subplot(111, projection='3d')
        X, Y = np.meshgrid(freq, delay)
        ax.plot_surface(X, Y, acq_grid, cmap='viridis')
        ax.set_ylim([min(delay), max(delay)])
    else:
        delay_interp = (np.arange(n_samples_per_chip * n_chips)
                        / n_samples_per_chip)
        spline = CubicSpline(delay, acq_grid)
        grid_interp = spline(delay_interp)
        ax = fig.add_subplot(111, projection='3d')
        X, Y = np.meshgrid(freq, delay_interp)
        ax.plot_surface(X, Y, grid_interp, cmap='inferno')
        ax.set_ylim([min(delay_interp), max(delay_interp)])

    ax.set_xlabel('Doppler shift (Hz)')
    ax.set_xlim([min(freq), max(freq)])
    ax.set_ylabel('Code delay (chips)')
    ax.set_zlabel('Test Statistics')

    plt.tight_layout()
    plt.savefig(img_name_root + '_sample_3D.png')
    plt.show()

    # --- Acquisition grid (2D)
    input_power = 100 # Change Test statistics in Doppler wipe-off plot

    fig2, axes = plt.subplots(2, 1, figsize=(8, 6))
    plt.gcf().canvas.manager.set_window_title(filename)
    axes[0].plot(freq, acq_grid[d_max, :])
    axes[0].set_xlim([min(freq), max(freq)])
    axes[0].set_xlabel('Doppler shift (Hz)')
    axes[0].set_ylabel('Test statistics')
    axes[0].set_title(f'Fixed code delay to {(d_max - 1) / n_fft * n_chips} '
                      f'chips')

    normalization = (d_samples_per_code**4) * input_power
    axes[1].plot(delay, acq_grid[:, f_max] / normalization)
    axes[1].set_xlim([min(delay), max(delay)])
    axes[1].set_xlabel('Code delay (chips)')
    axes[1].set_ylabel('Test statistics')
    axes[1].set_title(f'Doppler wipe-off = '
                      f'{str((f_max-1) * doppler_step - doppler_max)} Hz')

    plt.tight_layout()
    plt.savefig(img_name_root + '_sample_2D.png')
    plt.show()

else:
    # ---------- CHANGE HERE:
    lite_view = True
    # If lite_view -> sets the number of samples per chip in the graphical
    # representation
    n_samples_per_chip = 3
    d_samples_per_code = 25000

    filenames = os.listdir(path)
    for filename in filenames:
        sat = 1
        channel = 0
        execution = 1

        system = filename[12]
        signal = filename[14:16]
        if system == "G":
            if signal == "1C":
                n_chips = 1023
            elif signal == "2S" or "L5":
                n_chips = 10230
            else:
                print("Incorrect files format. Change the code or the "
                      "filenames.")
                sys.exit()
        elif system == "E":
            if signal == "1B":
                n_chips = 4092
            elif signal == "5X":
                n_chips = 10230
            else:
                print("Incorrect files format. Change the code or the "
                      "filenames.")
                sys.exit()
        elif system == "R":
            if signal == "1G" or "2G":
                n_chips = 511
            else:
                print("Incorrect files format. Change the code or the "
                      "filenames.")
                sys.exit()
        elif system == "C":
            if signal == "B1":
                n_chips = 2048
            elif signal == "B3" or "5C":
                n_chips = 10230
            else:
                print("Incorrect files format. Change the code or the "
                      "filenames.")
                sys.exit()

        complete_path = path + filename
        with h5py.File(complete_path, 'r') as data:
            acq_grid = data['acq_grid'][:]
            n_fft, n_dop_bins = acq_grid.shape
            d_max, f_max = np.unravel_index(np.argmax(acq_grid),
                                            acq_grid.shape)
            doppler_step = data['doppler_step'][0]
            doppler_max = data['doppler_max'][0]
            freq = np.arange(n_dop_bins) * doppler_step - doppler_max
            delay = np.arange(n_fft) / n_fft * n_chips

        # Plot data
        # --- Acquisition grid (3D)
        fig = plt.figure()
        plt.gcf().canvas.manager.set_window_title(filename)
        if not lite_view:
            ax = fig.add_subplot(111, projection='3d')
            X, Y = np.meshgrid(freq, delay)
            ax.plot_surface(X, Y, acq_grid, cmap='viridis')
            ax.set_ylim([min(delay), max(delay)])
        else:
            delay_interp = (np.arange(n_samples_per_chip * n_chips)
                            / n_samples_per_chip)
            spline = CubicSpline(delay, acq_grid)
            grid_interp = spline(delay_interp)
            ax = fig.add_subplot(111, projection='3d')
            X, Y = np.meshgrid(freq, delay_interp)
            ax.plot_surface(X, Y, grid_interp, cmap='inferno')
            ax.set_ylim([min(delay_interp), max(delay_interp)])

        ax.set_xlabel('Doppler shift (Hz)')
        ax.set_xlim([min(freq), max(freq)])
        ax.set_ylabel('Code delay (chips)')
        ax.set_zlabel('Test Statistics')

        plt.savefig(os.path.join(fig_path, filename[:-4]) + '_3D.png')

        plt.close()

        # --- Acquisition grid (2D)
        input_power = 100  # Change Test statistics in Doppler wipe-off plot

        fig2, axes = plt.subplots(2, 1, figsize=(8, 6))
        plt.gcf().canvas.manager.set_window_title(filename)
        axes[0].plot(freq, acq_grid[d_max, :])
        axes[0].set_xlim([min(freq), max(freq)])
        axes[0].set_xlabel('Doppler shift (Hz)')
        axes[0].set_ylabel('Test statistics')
        axes[0].set_title(f'Fixed code delay to '
                          f'{(d_max - 1) / n_fft * n_chips} chips')

        normalization = (d_samples_per_code ** 4) * input_power
        axes[1].plot(delay, acq_grid[:, f_max] / normalization)
        axes[1].set_xlim([min(delay), max(delay)])
        axes[1].set_xlabel('Code delay (chips)')
        axes[1].set_ylabel('Test statistics')
        axes[1].set_title(f'Doppler wipe-off = '
                          f'{str((f_max - 1) * doppler_step - doppler_max)} '
                          f'Hz')

        plt.tight_layout()
        plt.savefig(os.path.join(fig_path, filename[:-4]) + '_2D.png')
        # plt.show()
        plt.close()
