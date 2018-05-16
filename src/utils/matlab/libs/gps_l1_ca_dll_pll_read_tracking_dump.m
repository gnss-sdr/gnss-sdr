% Usage: gps_l1_ca_dll_pll_read_tracking_dump_64bits (filename, [count])
%
% Opens GNSS-SDR tracking binary log file .dat and returns the contents

% Read GNSS-SDR Tracking dump binary file into MATLAB.
% Javier Arribas, 2011. jarribas(at)cttc.es
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

function [GNSS_tracking] = gps_l1_ca_dll_pll_read_tracking_dump (filename, count)

m = nargchk (1,2,nargin);
num_float_vars=5;
num_unsigned_long_int_vars=1;
num_double_vars=11;
num_unsigned_int_vars=1;
double_size_bytes=8;
unsigned_long_int_size_bytes=8;
float_size_bytes=4;
long_int_size_bytes=4;

skip_bytes_each_read=float_size_bytes*num_float_vars+unsigned_long_int_size_bytes*num_unsigned_long_int_vars+double_size_bytes*num_double_vars+long_int_size_bytes*num_unsigned_int_vars;
bytes_shift=0;
if (m)
    usage (m);
end

if (nargin < 2)
    %count = Inf;
    file_stats = dir(filename);
    %round num bytes to read to integer number of samples (to protect the script from binary
    %dump end file transitory)
    count = (file_stats.bytes - mod(file_stats.bytes,skip_bytes_each_read))/skip_bytes_each_read;
end
%loops_counter = fread (f, count, 'uint32',4*12);
f = fopen (filename, 'rb');
if (f < 0)
else
    v1 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
    bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v2 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
    bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v3 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
    bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v4 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
    bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v5 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
    bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved unsigned_long_int
    v6 = fread (f, count, 'uint64',skip_bytes_each_read-unsigned_long_int_size_bytes);
    bytes_shift=bytes_shift+unsigned_long_int_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved double
    v7 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved double
    v8 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved double
    v9 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved double
    v10 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved double
    v11 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved double
    v12 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved double
    v13 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved double
    v14 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved double
    v15 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved double
    v16 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved double
    v17 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved double
    v18 = fread (f, count, 'uint32',skip_bytes_each_read-double_size_bytes);
    fclose (f);
    
    %%%%%%%% output vars %%%%%%%%
    
    %                     // EPR
    %                     d_dump_file.write(reinterpret_cast<char*>(&tmp_E), sizeof(float));
    %                     d_dump_file.write(reinterpret_cast<char*>(&tmp_P), sizeof(float));
    %                     d_dump_file.write(reinterpret_cast<char*>(&tmp_L), sizeof(float));
    %                     // PROMPT I and Q (to analyze navigation symbols)
    %                     d_dump_file.write(reinterpret_cast<char*>(&prompt_I), sizeof(float));
    %                     d_dump_file.write(reinterpret_cast<char*>(&prompt_Q), sizeof(float));
    %                     // PRN start sample stamp
    %                     //tmp_float=(float)d_sample_counter;
    %                     d_dump_file.write(reinterpret_cast<char*>(&d_sample_counter), sizeof(unsigned long int));
    %                     // accumulated carrier phase
    %                     d_dump_file.write(reinterpret_cast<char*>(&d_acc_carrier_phase_rad), sizeof(double));
    %
    %                     // carrier and code frequency
    %                     d_dump_file.write(reinterpret_cast<char*>(&d_carrier_doppler_hz), sizeof(double));
    %                     d_dump_file.write(reinterpret_cast<char*>(&d_code_freq_chips), sizeof(double));
    %
    %                     //PLL commands
    %                     d_dump_file.write(reinterpret_cast<char*>(&carr_phase_error_secs_Ti), sizeof(double));
    %                     d_dump_file.write(reinterpret_cast<char*>(&d_carrier_doppler_hz), sizeof(double));
    %
    %                     //DLL commands
    %                     d_dump_file.write(reinterpret_cast<char*>(&code_error_chips_Ti), sizeof(double));
    %                     d_dump_file.write(reinterpret_cast<char*>(&code_error_filt_chips), sizeof(double));
    %
    %                     // CN0 and carrier lock test
    %                     d_dump_file.write(reinterpret_cast<char*>(&d_CN0_SNV_dB_Hz), sizeof(double));
    %                     d_dump_file.write(reinterpret_cast<char*>(&d_carrier_lock_test), sizeof(double));
    %
    %                     // AUX vars (for debug purposes)
    %                     tmp_double = d_rem_code_phase_samples;
    %                     d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
    %                     tmp_double = static_cast<double>(d_sample_counter + d_current_prn_length_samples);
    %                     d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
    %                             // PRN
    %             unsigned int prn_ = d_acquisition_gnss_synchro->PRN;
    %             d_dump_file.write(reinterpret_cast<char*>(&prn_), sizeof(unsigned int));
    E=v1;
    P=v2;
    L=v3;
    prompt_I=v4;
    prompt_Q=v5;
    PRN_start_sample=v6;
    acc_carrier_phase_rad=v7;
    carrier_doppler_hz=v8;
    code_freq_hz=v9;
    carr_error=v10;
    carr_nco=v11;
    code_error=v12;
    code_nco=v13;
    CN0_SNV_dB_Hz=v14;
    carrier_lock_test=v15;
    var1=v16;
    var2=v17;
    PRN=v18;
    
    GNSS_tracking.E=E;
    GNSS_tracking.P=P;
    GNSS_tracking.L=L;
    GNSS_tracking.prompt_I=prompt_I;
    GNSS_tracking.prompt_Q=prompt_Q;
    GNSS_tracking.PRN_start_sample=PRN_start_sample;
    GNSS_tracking.acc_carrier_phase_rad=acc_carrier_phase_rad;
    GNSS_tracking.carrier_doppler_hz=carrier_doppler_hz;
    GNSS_tracking.code_freq_hz=code_freq_hz;
    GNSS_tracking.carr_error=carr_error;
    GNSS_tracking.carr_nco=carr_nco;
    GNSS_tracking.code_error=code_error
    GNSS_tracking.code_nco=code_nco;
    GNSS_tracking.CN0_SNV_dB_Hz=CN0_SNV_dB_Hz;
    GNSS_tracking.carrier_lock_test=carrier_lock_test;
    GNSS_tracking.d_rem_code_phase_samples=var1;
    GNSS_tracking.var2=var2;
    GNSS_tracking.PRN=PRN;
end

