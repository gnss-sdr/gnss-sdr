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
#%%
import numpy as np
import matplotlib.pyplot as plt
#%%
with open('dump_vtl_file.csv') as f:
    rawdatos=np.genfromtxt(f,dtype=str,delimiter=",")
#%%    
idx_kf_err = np.where(rawdatos[:,0]=='kf_xerr') [0]# se indexa
idx_kf_state = np.where(rawdatos[:,0]=='kf_state')[0]
idx_rtklib = np.where(rawdatos[:,0]=='rtklib_state')[0]
#%%
kf_err=rawdatos[idx_kf_err,1:].astype(float)
kf_estate=rawdatos[idx_kf_state,1:].astype(float)
rtklib=rawdatos[idx_rtklib,1:].astype(float)

#%%
plt.close()
plt.plot(kf_err[:,0],'o',label='kf_err')
plt.plot(kf_estate[:,0]-kf_estate[0,0],'o',label='kf_estate')
plt.plot(rtklib[:,0]-rtklib[0,0],'o',label='rtklib')
plt.xlabel('time U.A.')
plt.ylabel('X [m]')
plt.legend()
plt.show()

plt.plot(kf_err[:,1],'o',label='kf_err')
plt.plot(kf_estate[:,1]-kf_estate[0,1],'o',label='kf_estate')
plt.plot(rtklib[:,1]-rtklib[0,1],'o',label='rtklib')
plt.xlabel('time U.A.')
plt.ylabel('Y [m]')
plt.legend()
plt.show()

plt.plot(kf_err[:,2],'o',label='kf_err')
plt.plot(kf_estate[:,2]-kf_estate[0,2],'o',label='kf_estate')
plt.plot(rtklib[:,2]-rtklib[0,2],'o',label='rtklib')
plt.xlabel('time U.A.')
plt.ylabel('Z [m]')
plt.legend()
plt.show()

#%%
plt.close()
plt.plot(kf_estate[:,0]-kf_estate[0,0]+kf_err[:,0],'o',label='kf_corr_estate')
plt.plot(rtklib[:,0]-rtklib[0,0],'o',label='rtklib')
plt.xlabel('time U.A.')
plt.ylabel('X [m]')
plt.title('X corrected')
plt.legend()
plt.show()

plt.plot(kf_estate[:,1]-kf_estate[0,1]+kf_err[:,1],'o',label='kf_corr_estate')
plt.plot(rtklib[:,1]-rtklib[0,1],'o',label='rtklib')
plt.xlabel('time U.A.')
plt.ylabel('Y [m]')
plt.title('Y corrected')
plt.legend()
plt.show()

plt.plot(kf_estate[:,2]-kf_estate[0,2]+kf_err[:,2],'o',label='kf_corr_estate')
plt.plot(rtklib[:,2]-rtklib[0,2],'o',label='rtklib')
plt.xlabel('time U.A.')
plt.ylabel('Z [m]')
plt.title('Z corrected')
plt.legend()
plt.show()

#%%
plt.close()
plt.plot(kf_err[:,3],'o',label='kf_err')
plt.plot(kf_estate[:,3]-kf_estate[0,3],'o',label='kf_estate')
plt.plot(rtklib[:,3]-rtklib[0,3],'o',label='rtklib')
plt.xlabel('time U.A.')
plt.legend()
plt.show()

plt.plot(kf_err[:,4],'o',label='kf_err')
plt.plot(kf_estate[:,4]-kf_estate[0,4],'o',label='kf_estate')
plt.plot(rtklib[:,4]-rtklib[0,4],'o',label='rtklib')
plt.xlabel('time U.A.')
plt.legend()
plt.show()

plt.plot(kf_err[:,5],'o',label='kf_err')
plt.plot(kf_estate[:,5]-kf_estate[0,5],'o',label='kf_estate')
plt.plot(rtklib[:,5]-rtklib[0,5],'o',label='rtklib')
plt.xlabel('time U.A.')
plt.legend()
plt.show()
