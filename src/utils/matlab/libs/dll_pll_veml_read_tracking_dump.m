% Usage: dll_pll_veml_read_tracking_dump (filename, [count])
%
% Opens GNSS-SDR tracking binary log file .dat and returns the contents

% Read GNSS-SDR Tracking dump binary file into MATLAB.
% Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

function [GNSS_tracking] = dll_pll_veml_read_tracking_dump (filename, count)

m = nargchk (1,2,nargin);

num_float_vars = 18;
num_unsigned_long_int_vars = 1;
num_double_vars = 1;
num_unsigned_int_vars = 1;

if(~isempty(strfind(computer('arch'), '64')))
    % 64-bit computer
    double_size_bytes = 8;
    unsigned_long_int_size_bytes = 8;
    float_size_bytes = 4;
    unsigned_int_size_bytes = 4;
else
    double_size_bytes = 8;
    unsigned_long_int_size_bytes = 4;
    float_size_bytes = 4;
    unsigned_int_size_bytes = 4;
end

skip_bytes_each_read = float_size_bytes * num_float_vars + unsigned_long_int_size_bytes * num_unsigned_long_int_vars + ...
    double_size_bytes * num_double_vars + num_unsigned_int_vars*unsigned_int_size_bytes;

bytes_shift = 0;

if (m)
    usage (m);
end

if (nargin < 2)
    count = Inf;
end
%loops_counter = fread (f, count, 'uint32',4*12);
f = fopen (filename, 'rb');
if (f < 0)
else
    v1 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v2 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v3 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v4 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v5 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v6 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v7 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v8 = fread (f, count, 'long', skip_bytes_each_read - unsigned_long_int_size_bytes);
    bytes_shift = bytes_shift + unsigned_long_int_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v9 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v10 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v11 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v12 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v13 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v14 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v15 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v16 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v17 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v18 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v19 = fread (f, count, 'float', skip_bytes_each_read-float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next double
    v20 = fread (f, count, 'double', skip_bytes_each_read - double_size_bytes);
    bytes_shift = bytes_shift + double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next unsigned int
    v21 = fread (f, count, 'uint', skip_bytes_each_read - unsigned_int_size_bytes);
    fclose (f);
    
    GNSS_tracking.VE = v1;
    GNSS_tracking.E = v2;
    GNSS_tracking.P = v3;
    GNSS_tracking.L = v4;
    GNSS_tracking.VL = v5;
    GNSS_tracking.prompt_I = v6;
    GNSS_tracking.prompt_Q = v7;
    GNSS_tracking.PRN_start_sample = v8;
    GNSS_tracking.acc_carrier_phase_rad = v9;
    GNSS_tracking.carrier_doppler_hz = v10;
    GNSS_tracking.carrier_doppler_rate_hz = v11;
    GNSS_tracking.code_freq_hz = v12;
    GNSS_tracking.carr_error = v13;
    GNSS_tracking.carr_nco = v14;
    GNSS_tracking.code_error = v15;
    GNSS_tracking.code_nco = v16;
    GNSS_tracking.CN0_SNV_dB_Hz = v17;
    GNSS_tracking.carrier_lock_test = v18;
    GNSS_tracking.var1 = v19;
    GNSS_tracking.var2 = v20;
    GNSS_tracking.PRN = v21;
end

