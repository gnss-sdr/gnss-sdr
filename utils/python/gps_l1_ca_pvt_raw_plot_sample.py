"""
 gps_l1_ca_pvt_raw_plot_sample.py

 Reads GNSS-SDR PVT raw dump binary file using the provided function and plots
 some internal variables

 Irene Pérez Riega, 2023. iperrie@inta.es

 Modifiable in the file:
   sampling_freq     - Sampling frequency [Hz]
   channels          - Number of channels to check if they exist
   path              - Path to folder which contains raw file
   pvt_raw_log_path  - Completed path to PVT raw data file
   nav_sol_period    - Measurement period [ms]
   plot_skyplot      - = 1 -> Sky Plot (TO DO) // = 0 -> No Sky Plot
   true_position     - In settings, If not known enter all NaN's and mean
                        position will be used as a reference in UTM
                        coordinate system
   plot_position     - Optional function at the end
   plot_oneVStime    - Optional function at the end, select variable to plot

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import utm
import numpy as np
from lib.gps_l1_ca_read_pvt_dump import gps_l1_ca_read_pvt_dump
from lib.plotNavigation import plotNavigation
import pyproj
from lib.plotPosition import plot_position, plot_oneVStime

settings = {}
utm_e = []
utm_n = []
E_UTM = []
N_UTM = []
utm_zone = []

# ---------- CHANGE HERE:
samplingFreq = 64e6 / 16
channels = 5
path = '/home/labnav/Desktop/TEST_IRENE/'
pvt_raw_log_path = path + 'PVT.dat'
nav_sol_period = 10
plot_skyplot = 0

settings['true_position'] = {'E_UTM':np.nan,'N_UTM':np.nan,'U_UTM':np.nan}
settings['navSolPeriod'] = nav_sol_period

navSolutions = gps_l1_ca_read_pvt_dump(pvt_raw_log_path)
X, Y, Z = navSolutions['X'], navSolutions['Y'], navSolutions['Z']

utm_coords = []

for i in range(len(navSolutions['longitude'])):
    utm_coords.append(utm.from_latlon(navSolutions['latitude'][i],
                                      navSolutions['longitude'][i]))



for i in range(len(utm_coords)):
    utm_e.append(utm_coords[i][0])
    utm_n.append(utm_coords[i][1])
    utm_zone.append(utm_coords[i][2])

# Transform from Lat Long degrees to UTM coordinate system
# It's supposed utm_zone and letter will not change during tracking
input_projection = pyproj.CRS.from_string("+proj=longlat "
                                          "+datum=WGS84 +no_defs")

utm_e = []
utm_n = []
for i in range(len(navSolutions['longitude'])):
    output_projection = pyproj.CRS (f"+proj=utm +zone={utm_zone[i]} "
                                    f"+datum=WGS84 +units=m +no_defs")
    transformer = pyproj.Transformer.from_crs(input_projection,
                                              output_projection)
    utm_e, utm_n = transformer.transform(navSolutions['longitude'][i],
                                         navSolutions['latitude'][i])
    E_UTM.append(utm_e)
    N_UTM.append(utm_n)


navSolutions['E_UTM'] = E_UTM
navSolutions['N_UTM'] = N_UTM
navSolutions['U_UTM'] = navSolutions['Z']

plotNavigation(navSolutions,settings,plot_skyplot)

# OPTIONAL: Other plots ->
plot_position(navSolutions)
plot_oneVStime(navSolutions, 'X_vel')
plot_oneVStime(navSolutions, 'Tot_Vel')