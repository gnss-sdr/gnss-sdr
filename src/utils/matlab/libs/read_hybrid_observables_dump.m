% Usage: read_tracking_dat (filename, [count])
%
% Opens GNSS-SDR tracking binary log file .dat and returns the contents
%

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
% along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
%
% -------------------------------------------------------------------------
%
% Javier Arribas 2011

function [observables] = read_hybrid_observables_dump (channels, filename, count)

m = nargchk (1,2,nargin);
num_double_vars=7;
double_size_bytes=8;
skip_bytes_each_read=double_size_bytes*num_double_vars*channels;
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
    for N=1:1:channels
        observables.RX_time(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.d_TOW_at_current_symbol(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.Carrier_Doppler_hz(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.Carrier_phase_hz(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.Pseudorange_m(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.PRN(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.valid(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
    end
    
    fclose (f);
    
    %%%%%%%% output vars %%%%%%%%
    %     double tmp_double;
    %     for (unsigned int i = 0; i < d_nchannels; i++)
    %         {
    %             tmp_double = current_gnss_synchro[i].RX_time;
    %             d_dump_file.write((char*)&tmp_double, sizeof(double));
    %             tmp_double = current_gnss_synchro[i].TOW_at_current_symbol_s;
    %             d_dump_file.write((char*)&tmp_double, sizeof(double));
    %             tmp_double = current_gnss_synchro[i].Carrier_Doppler_hz;
    %             d_dump_file.write((char*)&tmp_double, sizeof(double));
    %             tmp_double = current_gnss_synchro[i].Carrier_phase_rads/GPS_TWO_PI;
    %             d_dump_file.write((char*)&tmp_double, sizeof(double));
    %             tmp_double = current_gnss_synchro[i].Pseudorange_m;
    %             d_dump_file.write((char*)&tmp_double, sizeof(double));
    %             tmp_double = current_gnss_synchro[i].PRN;
    %             d_dump_file.write((char*)&tmp_double, sizeof(double));
    %             tmp_double = current_gnss_synchro[i].Flag_valid_pseudorange;
    %             d_dump_file.write((char*)&tmp_double, sizeof(double));
    %         }
end

