% Read PVG raw dump
% -------------------------------------------------------------------------
%
% Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
%
% GNSS-SDR is a software defined Global Navigation
%           Satellite Systems receiver
%
% This file is part of GNSS-SDR.
% 
% GNSS-SDR is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% at your option) any later version.
% 
% GNSS-SDR is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
%
% -------------------------------------------------------------------------
%

%clear all;

samplingFreq       = 64e6/16;     %[Hz]
channels=4;
path='/home/javier/workspace/gnss-sdr-ref/trunk/install/';
pvt_raw_log_path=[path 'PVT_raw.dat'];
GNSS_PVT_raw= gps_l1_ca_read_pvt_raw_dump(channels,pvt_raw_log_path);
