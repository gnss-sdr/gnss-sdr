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
% count is the maximum number of records to read (all by default).
%
% Each record of the file takes 188 bytes, in the byte order of the machine
% that wrote it:
%
%   uint32   TOW at current symbol [ms]
%   uint32   Week number
%   double   PVT GPS time [s]
%   double   User clock offset [s]
%   double   ECEF user position X, Y, Z [m]
%   double   ECEF user velocity X, Y, Z [m/s]
%   double   Position covariance c_xx, c_yy, c_zz, c_xy, c_yz, c_zx [m^2]
%   double   GEO user position Latitude [deg]
%   double   GEO user position Longitude [deg]
%   double   GEO user position Height [m]
%   uint8    Number of valid satellites
%   uint8    RTKLIB solution status
%   uint8    RTKLIB solution type (0: xyz-ecef, 1: enu-baseline)
%   float    AR ratio factor for validation
%   float    AR ratio threshold for validation
%   double   GDOP, PDOP, HDOP, VDOP
%   uint8    Number of satellites with Galileo HAS corrections
%
% The fields of the output structure are row vectors, with one element per
% record. They are named as the variables of the .mat file that GNSS-SDR
% writes with PVT.dump_mat=true. X, Y, Z, dt and TransmitTime are kept for
% the K.Borre MATLAB-based receiver.

if (nargin < 1 || nargin > 2)
    error ('usage: gps_l1_ca_pvt_read_pvt_dump (filename, [count])');
end

if (nargin < 2)
    count = Inf;
end

record_size_bytes = 188;

f = fopen (filename, 'rb');
if (f < 0)
    error ('gps_l1_ca_pvt_read_pvt_dump: cannot open file %s', filename);
end
raw = fread (f, count * record_size_bytes, 'uint8=>uint8');
fclose (f);

% an incomplete record at the end of the file is left out
num_records = floor (numel (raw) / record_size_bytes);
raw = reshape (raw(1:num_records * record_size_bytes), record_size_bytes, num_records);

navSolutions.TOW_at_current_symbol_ms = read_field (raw, 0, 'uint32');
navSolutions.week = read_field (raw, 4, 'uint32');
navSolutions.RX_time = read_field (raw, 8, 'double');
navSolutions.user_clk_offset = read_field (raw, 16, 'double');
navSolutions.pos_x = read_field (raw, 24, 'double');
navSolutions.pos_y = read_field (raw, 32, 'double');
navSolutions.pos_z = read_field (raw, 40, 'double');
navSolutions.vel_x = read_field (raw, 48, 'double');
navSolutions.vel_y = read_field (raw, 56, 'double');
navSolutions.vel_z = read_field (raw, 64, 'double');
navSolutions.cov_xx = read_field (raw, 72, 'double');
navSolutions.cov_yy = read_field (raw, 80, 'double');
navSolutions.cov_zz = read_field (raw, 88, 'double');
navSolutions.cov_xy = read_field (raw, 96, 'double');
navSolutions.cov_yz = read_field (raw, 104, 'double');
navSolutions.cov_zx = read_field (raw, 112, 'double');
navSolutions.latitude = read_field (raw, 120, 'double');
navSolutions.longitude = read_field (raw, 128, 'double');
navSolutions.height = read_field (raw, 136, 'double');
navSolutions.valid_sats = read_field (raw, 144, 'uint8');
navSolutions.solution_status = read_field (raw, 145, 'uint8');
navSolutions.solution_type = read_field (raw, 146, 'uint8');
navSolutions.AR_ratio_factor = read_field (raw, 147, 'single');
navSolutions.AR_ratio_threshold = read_field (raw, 151, 'single');
navSolutions.gdop = read_field (raw, 155, 'double');
navSolutions.pdop = read_field (raw, 163, 'double');
navSolutions.hdop = read_field (raw, 171, 'double');
navSolutions.vdop = read_field (raw, 179, 'double');
navSolutions.has_corrected_sats = read_field (raw, 187, 'uint8');

% K.Borre MATLAB-based receiver names
navSolutions.X = navSolutions.pos_x;
navSolutions.Y = navSolutions.pos_y;
navSolutions.Z = navSolutions.pos_z;
navSolutions.dt = navSolutions.user_clk_offset;
navSolutions.TransmitTime = navSolutions.RX_time;


function values = read_field (raw, offset, type)
% One field of every record, as a row vector of doubles. offset is the
% position of the field in the record, in bytes, starting at zero.
switch type
    case 'uint8'
        field_size_bytes = 1;
    case {'uint32', 'single'}
        field_size_bytes = 4;
    otherwise
        field_size_bytes = 8;
end
bytes = raw(offset + 1:offset + field_size_bytes, :);
values = double (typecast (bytes(:), type)).';
