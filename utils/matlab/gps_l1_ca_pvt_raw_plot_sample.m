% Read PVG raw dump
% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------
%

%clear all;

samplingFreq       = 64e6/16;     %[Hz]
channels=4;
path='/home/javier/workspace/gnss-sdr-ref/trunk/install/';
pvt_raw_log_path=[path 'PVT_raw.dat'];
GNSS_PVT_raw= gps_l1_ca_read_pvt_raw_dump(channels,pvt_raw_log_path);
