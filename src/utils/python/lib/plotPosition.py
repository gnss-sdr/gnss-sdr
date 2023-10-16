"""
 plotPosition.py

 plot_position(navSolutions)
   Graph Latitude-Longitude and X-Y-X as a function of Transmit Time
   Args:
      navSolutions     - A dictionary with the processed information in lists

 plot_oneVStime(navSolutions, name)
   Graph of a variable as a function of transmission time
   Args:
       navSolutions    - A dictionary with the processed information in lists
       name            - navSolutions variable name that we want to plot

 calcularCEFP(percentil, navSolutions, m_lat, m_long)
   Calculate CEFP radio [m] for n percentil.
   Args:
       percentil       - Number of measures that will be inside the circumference
       navSolutions    - A dictionary with the processed information in lists
       m_lat           - Mean latitude measures [º]
       m_long          - Mean longitude measures [º]

 Modifiable in the file:
   fig_path        - Path where plots will be save
   fig_path_maps   - Path where the maps will be save
   filename_map    - Path where map will be save
   filename_map_t  - Path where terrain map will be save

 Irene Pérez Riega, 2023. iperrie@inta.es

 -----------------------------------------------------------------------------

 GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 This file is part of GNSS-SDR.

 Copyright (C) 2022  (see AUTHORS file for a list of contributors)
 SPDX-License-Identifier: GPL-3.0-or-later

 -----------------------------------------------------------------------------
"""

import math
import os.path
import webbrowser
import numpy as np
import matplotlib.pyplot as plt
import folium


def plot_position(navSolutions):

    # ---------- CHANGE HERE:
    fig_path = '/home/labnav/Desktop/TEST_IRENE/PLOTS/PlotPosition/'
    fig_path_maps = fig_path + 'maps/'
    filename_map = 'mapPlotPosition.html'
    filename_map_t = 'mapTerrainPotPosition.html'

    if not os.path.exists(fig_path_maps):
        os.mkdir(fig_path_maps)

    # Statics Positions:
    m_lat = sum(navSolutions['latitude']) / len(navSolutions['latitude'])
    m_long = sum(navSolutions['longitude']) / len(navSolutions['longitude'])

    # CEFP_n -> Include the n% of the dots in the circle
    r_CEFP_95 = calcularCEFP(95, navSolutions, m_lat, m_long)
    r_CEFP_50 = calcularCEFP(50, navSolutions, m_lat, m_long)

    # Generate and save html with the positions
    m = folium.Map(location=[navSolutions['latitude'][0],
                             navSolutions['longitude'][0]], zoom_start=100)
    c_CEFP95 = folium.Circle(location=[m_lat, m_long],
                             radius=r_CEFP_95, color='green', fill=True,
                             fill_color='green', fill_opacity=0.5)
    c_CEFP50 = folium.Circle(location=[m_lat, m_long], radius=r_CEFP_50,
                             color='red', fill=True, fill_color='red',
                             fill_opacity=0.5)

    # POP-UPs
    popup95 = folium.Popup("(Green)CEFP95 diameter: {} "
                           "metres".format(2 * r_CEFP_95))
    popup95.add_to(c_CEFP95)
    popup50 = folium.Popup("(Red)CEFP50 diameter: {} "
                           "metres".format(2 * r_CEFP_50))
    popup50.add_to(c_CEFP50)

    c_CEFP95.add_to(m)
    c_CEFP50.add_to(m)

    # Optional: Plot each point ->
    """
    for i in range(len(navSolutions['latitude'])):
        folium.Marker(location=[navSolutions['latitude'][i],
                                navSolutions['longitude'][i]],
                      icon=folium.Icon(color='red')).add_to(m)
    """

    m.save(fig_path_maps + filename_map)
    webbrowser.open(fig_path_maps + filename_map)

    # Optional: with terrain ->
    """    
    n = folium.Map(location=[navSolutions['latitude'][0],
                             navSolutions['longitude'][0]], zoom_start=100,
                   tiles='Stamen Terrain')
    c_CEFP95.add_to(n)
    c_CEFP50.add_to(n)
    n.save(fig_path_maps + filename_map_t)
    webbrowser.open(fig_path_maps + filename_map_t)
    """

    # Plot ->
    time = []
    for i in range(len(navSolutions['TransmitTime'])):
        time.append(round(navSolutions['TransmitTime'][i] -
                          min(navSolutions['TransmitTime']), 3))

    plt.figure(figsize=(1920 / 120, 1080 / 120))
    plt.clf()
    plt.suptitle(f'Plot file PVT process data results')

    # Latitude and Longitude
    plt.subplot(1, 2, 1)
    scatter = plt.scatter(navSolutions['latitude'], navSolutions['longitude'],
                          c=time, marker='.')
    plt.grid()
    plt.ticklabel_format(style='plain', axis='both', useOffset=False)
    plt.title('Positions latitud-longitud')
    plt.xlabel('Latitude º')
    plt.ylabel('Longitude º')
    plt.axis('tight')

    # Colors
    cmap = plt.get_cmap('viridis')
    norm = plt.Normalize(vmin=min(time), vmax=max(time))
    scatter.set_cmap(cmap)
    scatter.set_norm(norm)
    colors = plt.colorbar(scatter)
    colors.set_label('TransmitTime [s]')

    # X, Y, Z
    ax = plt.subplot(1, 2, 2, projection='3d')
    plt.ticklabel_format(style='plain', axis='both', useOffset=False)
    ax.scatter(navSolutions['X'], navSolutions['Y'], navSolutions['Z'],
               c=time, marker='.')
    ax.set_xlabel('Eje X [m]')
    ax.set_ylabel('Eje Y [m]')
    ax.set_zlabel('Eje Z [m]')
    ax.set_title('Positions x-y-z')

    plt.tight_layout()
    plt.savefig(os.path.join(fig_path, f'PVT_ProcessDataResults.png'))
    plt.show()


def plot_oneVStime(navSolutions, name):

    # ---------- CHANGE HERE:
    fig_path = '/home/labnav/Desktop/TEST_IRENE/PLOTS/PlotPosition/'
    if not os.path.exists(fig_path):
        os.mkdir(fig_path)

    time = []
    for i in range(len(navSolutions['TransmitTime'])):
        time.append(round(navSolutions['TransmitTime'][i] -
                          min(navSolutions['TransmitTime']), 3))

    plt.clf()
    plt.scatter(time, navSolutions[name], marker='.')
    plt.grid()
    plt.title(f'{name} vs Time')
    plt.xlabel('Time [s]')
    plt.ylabel(name)
    plt.axis('tight')
    plt.ticklabel_format(style='plain', axis='both', useOffset=False)

    plt.tight_layout()
    plt.savefig(os.path.join(fig_path, f'{name}VSTime.png'))
    plt.show()

def calcularCEFP(percentil, navSolutions, m_lat, m_long):

    r_earth = 6371000
    lat = []
    long = []
    dlat = []
    dlong = []
    dist = []

    m_lat = math.radians(m_lat)
    m_long = math.radians(m_long)

    for i in range(len(navSolutions['latitude'])):
        lat.append(math.radians(navSolutions['latitude'][i]))
        long.append(math.radians(navSolutions['longitude'][i]))

    for i in range(len(lat)):
        dlat.append(m_lat - lat[i])
        dlong.append(m_long - long[i])
        # Haversine:
        a = (math.sin(dlat[i] / 2) ** 2 +
             math.cos(lat[i]) * math.cos(m_lat) * math.sin(dlong[i] / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist.append(r_earth * c)

    # Radio CEFP
    radio_CEFP_p = np.percentile(dist, percentil)
    return radio_CEFP_p
