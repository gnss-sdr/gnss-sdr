#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
% Miguel Angel Gomez, 2024. gomezlma(at)inta.es
% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------
%
"""

import numpy as np
import h5py
import matplotlib.pyplot as plt

#%%
f = h5py.File('PVT_raw.mat','r')
#data = f.get('rateQualityOutTrim/date')
#data = np.array(data)
print(f.keys()) # gives the name of the variables stored
RX_time = f.get('RX_time')

vel_x = f.get('vel_x')
vel_y = f.get('vel_y')
vel_z = f.get('vel_z')

pos_x = f.get('pos_x')
pos_y = f.get('pos_y')
pos_z = f.get('pos_z')

#kf_state1=f.get('vtl_kf_state_1')

#%%
for keys in f:
    keys=f.get(keys)
#%%
#plt.plot(kf_state1[:,0]-kf_state1[0,0],'o',label='kf_state1')

#plt.show()
#%%
plt.scatter(RX_time,pos_x)
plt.show()
plt.scatter(RX_time,pos_y-pos_y[0])
plt.show()
plt.scatter(RX_time,pos_z-pos_z[0])
plt.show()

#%%
plt.scatter(RX_time,pos_x-pos_x[0])
#plt.ylim([4863484, 4863591])
plt.ylim([-20,110])
plt.ylabel('X [m]')
plt.show()
plt.scatter(RX_time,pos_y-pos_y[0])
plt.ylabel('Y [m]')
plt.ylim([-85, 5])
plt.show()
plt.scatter(RX_time,pos_z-pos_z[0])
plt.ylabel('Z [m]')
plt.ylim([-110,25])
plt.show()

#%%
plt.scatter(RX_time,vel_x)
plt.show()
plt.scatter(RX_time,vel_y-vel_y[0])
plt.show()
plt.scatter(RX_time,vel_z-vel_z[0])
plt.show()

