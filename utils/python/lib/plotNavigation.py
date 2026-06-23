"""
 plotNavigation.py

 Function plots variations of coordinates over time and a 3D position
 plot. It plots receiver coordinates in UTM system or coordinate offsets if
 the true UTM receiver coordinates are provided.

 Irene Pérez Riega, 2023. iperrie@inta.es

 plotNavigation(navSolutions, settings, plot_skyplot)

   Args:
       navSolutions    - Results from navigation solution function. It
                       contains measured pseudoranges and receiver
                      coordinates.
       settings        - Receiver settings. The true receiver coordinates
                       are contained in this structure.
       plot_skyplot    - If == 1 then use satellite coordinates to plot the
                       satellite positions (not implemented yet TO DO)

   Modifiable in the file:
       fig_path        - Path where plots will be save

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import numpy as np
import matplotlib.pyplot as plt
import os


def plotNavigation(navSolutions, settings, plot_skyplot=0):

    fig_path = settings.get('fig_path', 'plots/pvt')
    output_format = settings.get('output_format', 'png')

    if not os.path.exists(fig_path):
        os.makedirs(fig_path)

    if navSolutions:
        if (np.isnan(settings['true_position']['E_UTM']) or
                np.isnan(settings['true_position']['N_UTM']) or
                np.isnan(settings['true_position']['U_UTM'])):

            # Compute mean values
            ref_coord = {
                'E_UTM': np.nanmean(navSolutions['E_UTM']),
                'N_UTM': np.nanmean(navSolutions['N_UTM']),
                'U_UTM': np.nanmean(navSolutions['U_UTM'])
            }

            mean_latitude = np.nanmean(navSolutions['latitude'])
            mean_longitude = np.nanmean(navSolutions['longitude'])
            mean_height = np.nanmean(navSolutions['height'])
            ref_point_lg_text = (f"Mean Position\nLat: {mean_latitude}º\n"
                                 f"Long: {mean_longitude}º\n"
                                 f"Hgt: {mean_height:+6.1f}")

        else:
            # Compute the mean error for static receiver
            ref_coord = {
                'E_UTM': settings['true_position']['E_UTM'],
                'N_UTM': settings['true_position']['N_UTM'],
                'U_UTM': settings['true_position']['U_UTM']
            }

            mean_position = {
                'E_UTM': np.nanmean(navSolutions['E_UTM']),
                'N_UTM': np.nanmean(navSolutions['N_UTM']),
                'U_UTM': np.nanmean(navSolutions['U_UTM'])
            }

            error_meters = np.sqrt(
                (mean_position['E_UTM'] - ref_coord['E_UTM']) ** 2 +
                (mean_position['N_UTM'] - ref_coord['N_UTM']) ** 2 +
                (mean_position['U_UTM'] - ref_coord['U_UTM']) ** 2)

            ref_point_lg_text = (f"Reference Position, Mean 3D error = "
                                 f"{error_meters} [m]")

        #Create plot and subplots
        plt.figure(figsize=(1920 / 120, 1080 / 120))
        plt.clf()
        plt.title('Navigation solutions',fontweight='bold')

        ax1 = plt.subplot(4, 2, (1, 4))
        ax2 = plt.subplot(4, 2, (5, 7), projection='3d')
        ax3 = plt.subplot(4, 2, (6, 8), projection='3d')

        #  (ax1) Coordinate differences in UTM system from reference point
        # Coerce to arrays so the offset works whether the reference is a
        # float (from --true-position) or a NumPy mean of the solutions.
        e_utm = np.asarray(navSolutions['E_UTM'], dtype=float)
        n_utm = np.asarray(navSolutions['N_UTM'], dtype=float)
        u_utm = np.asarray(navSolutions['U_UTM'], dtype=float)
        d_east = e_utm - ref_coord['E_UTM']
        d_north = n_utm - ref_coord['N_UTM']
        d_up = u_utm - ref_coord['U_UTM']
        ax1.plot(np.vstack([d_east, d_north, d_up]).T)
        ax1.set_title('Coordinates variations in UTM system', fontweight='bold')
        ax1.legend(['E_UTM', 'N_UTM', 'U_UTM'])
        ax1.set_xlabel(f"Measurement period: {settings['navSolPeriod']} ms")
        ax1.set_ylabel('Variations (m)')
        ax1.grid(True)
        ax1.axis('tight')

        # (ax2) Reserved for a satellite sky plot, which cannot be drawn from a
        # PVT dump (it has no per-satellite azimuth/elevation). Hide the empty
        # panel; use utils/skyplot/skyplot.py to plot a skyplot from a RINEX
        # navigation file.
        ax2.axis('off')
        if plot_skyplot:
            print("Warning: the sky plot panel is not available from a PVT "
                  "dump (no azimuth/elevation data). Use "
                  "utils/skyplot/skyplot.py instead.")

        # (ax3) Position plot in UTM system
        ax3.scatter(d_east, d_north, d_up, marker='+')
        ax3.scatter([0], [0], [0], color='r', marker='+', linewidth=1.5)
        ax3.view_init(0, 90)
        ax3.set_box_aspect([1, 1, 1])
        ax3.grid(True, which='minor')
        ax3.legend(['Measurements', ref_point_lg_text])
        ax3.set_title('Positions in UTM system (3D plot)',fontweight='bold')
        ax3.set_xlabel('East (m)')
        ax3.set_ylabel('North (m)')
        ax3.set_zlabel('Upping (m)')

        plt.tight_layout()
        plt.savefig(os.path.join(fig_path, f'measures_UTM.{output_format}'))
        # Close unless it will be shown; the caller triggers a single
        # plt.show() at the end. Avoids repeated show()/close() cycles, which
        # can crash interactive matplotlib backends (e.g. macOS) on close.
        if not settings.get('show', True):
            plt.close()
