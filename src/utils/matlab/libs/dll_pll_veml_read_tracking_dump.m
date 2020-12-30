% Usage: dll_pll_veml_read_tracking_dump (filename, [count])
%
% Read GNSS-SDR Tracking dump binary file into MATLAB.
% Opens GNSS-SDR tracking binary log file .dat and returns the contents

% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Luis Esteve, 2012. luis(at)epsilon-formacion.com
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------

function [GNSS_tracking] = dll_pll_veml_read_tracking_dump (filename, count)

m = nargchk (1,2,nargin);

num_float_vars = 19;
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
    if unsigned_long_int_size_bytes==8
        v8 = fread (f, count, 'uint64', skip_bytes_each_read - unsigned_long_int_size_bytes);
    else
        v8 = fread (f, count, 'uint32', skip_bytes_each_read - unsigned_long_int_size_bytes);
    end
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
    fseek(f,bytes_shift,'bof'); % move to next float
    v18 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v19 = fread (f, count, 'float', skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next float
    v20 = fread (f, count, 'float', skip_bytes_each_read-float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next double
    v21 = fread (f, count, 'double', skip_bytes_each_read - double_size_bytes);
    bytes_shift = bytes_shift + double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next unsigned int
    v22 = fread (f, count, 'uint', skip_bytes_each_read - unsigned_int_size_bytes);
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
    GNSS_tracking.carrier_doppler_rate_hz_s = v11;
    GNSS_tracking.code_freq_hz = v12;
    GNSS_tracking.code_freq_rate_hz_s = v13;
    GNSS_tracking.carr_error = v14;
    GNSS_tracking.carr_nco = v15;
    GNSS_tracking.code_error = v16;
    GNSS_tracking.code_nco = v17;
    GNSS_tracking.CN0_SNV_dB_Hz = v18;
    GNSS_tracking.carrier_lock_test = v19;
    GNSS_tracking.var1 = v20;
    GNSS_tracking.var2 = v21;
    GNSS_tracking.PRN = v22;
end
