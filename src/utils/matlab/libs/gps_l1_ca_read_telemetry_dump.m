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
% Javier Arribas 2011

function [telemetry] = gps_l1_ca_read_telemetry_dump (filename, count)

%% usage: read_tracking_dat (filename, [count])
%%
%% open GNSS-SDR tracking binary log file .dat and return the contents
%%

m = nargchk (1,2,nargin);
num_double_vars=3;
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
    telemetry.preamble_delay_ms = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved
    telemetry.prn_delay_ms = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved
    telemetry.Preamble_symbol_counter = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved
    
    fclose (f);
    
    %%%%%%%% output vars %%%%%%%%
    %           {
    % 				double tmp_double;
    % 				tmp_double = current_synchro_data.Preamble_delay_ms;
    % 				d_dump_file.write((char*)&tmp_double, sizeof(double));
    % 				tmp_double = current_synchro_data.Prn_delay_ms;
    % 				d_dump_file.write((char*)&tmp_double, sizeof(double));
    % 				tmp_double = current_synchro_data.Preamble_symbol_counter;
    % 				d_dump_file.write((char*)&tmp_double, sizeof(double));
    %             }
end

