% Read GNSS-SDR PVT lib dump binary file into MATLAB. The resulting
% structure is compatible with the K.Borre MATLAB-based receiver.
%
% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Javier Arribas 2011
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------
%

function [navSolutions] = gps_l1_ca_pvt_read_pvt_dump (filename, count)

%% usage: gps_l1_ca_pvt_read_pvt_dump (filename, [count])
%%
%% open GNSS-SDR PVT binary log file .dat and return the contents
%%
%
% //  PVT GPS time
% tmp_double=GPS_current_time;
% d_dump_file.write((char*)&tmp_double, sizeof(double));
% // ECEF User Position East [m]
% tmp_double=mypos(0);
% d_dump_file.write((char*)&tmp_double, sizeof(double));
% // ECEF User Position North [m]
% tmp_double=mypos(1);
% d_dump_file.write((char*)&tmp_double, sizeof(double));
% // ECEF User Position Up [m]
% tmp_double=mypos(2);
% d_dump_file.write((char*)&tmp_double, sizeof(double));
% // User clock offset [s]
% tmp_double=mypos(3);
% d_dump_file.write((char*)&tmp_double, sizeof(double));
% // GEO user position Latitude [deg]
% tmp_double=d_latitude_d;
% d_dump_file.write((char*)&tmp_double, sizeof(double));
% // GEO user position Longitude [deg]
% tmp_double=d_longitude_d;
% d_dump_file.write((char*)&tmp_double, sizeof(double));
% // GEO user position Height [m]
% tmp_double=d_height_m;
% d_dump_file.write((char*)&tmp_double, sizeof(double));

m = nargchk (1,2,nargin);
num_double_vars=8;
double_size_bytes=8;
skip_bytes_each_read=double_size_bytes*num_double_vars;
bytes_shift=0;
if (m)
    usage (m);
end

if (nargin < 3)
    count = Inf;
end
%loops_counter = fread (f, count, 'uint32',4*12);
f = fopen (filename, 'rb');
if (f < 0)
else
    GPS_current_time = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved
    ECEF_X = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved
    ECEF_Y = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved
    ECEF_Z = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved
    Clock_Offset = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved
    Lat = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved
    Long = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved
    Height = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved
    fclose (f);
end

navSolutions.X=ECEF_X.';
navSolutions.Y=ECEF_Y.';
navSolutions.Z=ECEF_Z.';
navSolutions.dt=Clock_Offset.';
navSolutions.latitude=Lat.';
navSolutions.longitude=Long.';
navSolutions.height=Height.';
navSolutions.TransmitTime=GPS_current_time.';
