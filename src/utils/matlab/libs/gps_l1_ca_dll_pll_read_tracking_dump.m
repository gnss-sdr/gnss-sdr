% /*!
%  * \file gps_l1_ca_dll_pll_read_tracking_dump.m
%  * \brief Read GNSS-SDR Tracking dump binary file into MATLAB.
%  * \author Javier Arribas, 2011. jarribas(at)cttc.es
%  * -------------------------------------------------------------------------
%  *
%  * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
%  *
%  * GNSS-SDR is a software defined Global Navigation
%  *          Satellite Systems receiver
%  *
%  * This file is part of GNSS-SDR.
%  *
%  * GNSS-SDR is free software: you can redistribute it and/or modify
%  * it under the terms of the GNU General Public License as published by
%  * the Free Software Foundation, either version 3 of the License, or
%  * at your option) any later version.
%  *
%  * GNSS-SDR is distributed in the hope that it will be useful,
%  * but WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  * GNU General Public License for more details.
%  *
%  * You should have received a copy of the GNU General Public License
%  * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
%  *
%  * -------------------------------------------------------------------------
%  */             
function [GNSS_tracking] = gps_l1_ca_dll_pll_read_tracking_dump (filename, count)

  %% usage: gps_l1_ca_dll_pll_read_tracking_dump (filename, [count])
  %%
  %% open GNSS-SDR tracking binary log file .dat and return the contents
  %%

  m = nargchk (1,2,nargin);
  num_float_vars = 5;
  num_double_vars = 11;
  num_ulong_vars = 1;
  num_uint_vars = 1;
  
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

  skip_bytes_each_read = float_size_bytes*num_float_vars + double_size_bytes*num_double_vars + unsigned_int_size_bytes*num_uint_vars + unsigned_long_int_size_bytes*num_ulong_vars;
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
    v1 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v2 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v3 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v4 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v5 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  unsigned long int
    v6 = fread (f, count, 'long',skip_bytes_each_read-unsigned_long_int_size_bytes);
        bytes_shift=bytes_shift+unsigned_long_int_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v7 = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v8 = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v9 = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v10 = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v11 = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v12 = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v13 = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v14 = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v15 = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
            bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next  float
    v16 = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
            bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next double
    v17 = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
            bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next double
    v18 = fread (f, count, 'uint', skip_bytes_each_read - unsigned_int_size_bytes);
    fclose (f);
  

    GNSS_tracking.E = v1;
    GNSS_tracking.P = v2;
    GNSS_tracking.L = v3;
    GNSS_tracking.prompt_I = v4;
    GNSS_tracking.prompt_Q = v5;
    GNSS_tracking.PRN_start_sample = v6;
    GNSS_tracking.acc_carrier_phase_rad = v7;
    GNSS_tracking.carrier_doppler_hz = v8;
    GNSS_tracking.code_freq_hz = v9;
    GNSS_tracking.carr_error = v10;
    GNSS_tracking.carr_nco = v11;
    GNSS_tracking.code_error = v12;
    GNSS_tracking.code_nco = v13;
    GNSS_tracking.CN0_SNV_dB_Hz = v14;
    GNSS_tracking.carrier_lock_test = v15;
    GNSS_tracking.var1 = v16;
    GNSS_tracking.var2 = v17;
    GNSS_tracking.PRN = v18;
  end
  
