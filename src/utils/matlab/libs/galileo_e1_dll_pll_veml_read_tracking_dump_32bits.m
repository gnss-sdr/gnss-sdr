% /*!
%  * \file galileo_e1_dll_pll_veml_read_tracking_dump.m
%  * \brief Read GNSS-SDR Tracking dump binary file into MATLAB.
%  * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
%  * -------------------------------------------------------------------------
%  *
%  * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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
function [GNSS_tracking] = galileo_e1_dll_pll_veml_read_tracking_dump_32bits (filename, count)

  %% usage: galileo_e1_dll_pll_veml_read_tracking_dump (filename, [count])
  %%
  %% open GNSS-SDR tracking binary log file .dat and return the contents
  %%

  m = nargchk (1,2,nargin);
  num_float_vars=17;
  num_unsigned_long_int_vars=1;
  num_double_vars=1;
  double_size_bytes=8;
  unsigned_long_int_size_bytes=4;
  float_size_bytes=4;
  skip_bytes_each_read=float_size_bytes*num_float_vars+unsigned_long_int_size_bytes*num_unsigned_long_int_vars+double_size_bytes*num_double_vars;
  bytes_shift=0;
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
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v6 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v7 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v8 = fread (f, count, 'uint32',skip_bytes_each_read-unsigned_long_int_size_bytes);
        bytes_shift=bytes_shift+unsigned_long_int_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v9 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v10 = fread (f, count, '*float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v11 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v12 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v13 = fread (f, count, '*float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v14 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v15 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v16 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v17 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
            bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v18 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
            bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v19 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    fclose (f);
    
    %%%%%%%% output vars %%%%%%%%
    
%                     // EPR
%                     d_dump_file.write((char*)&tmp_VE, sizeof(float));
%                     d_dump_file.write((char*)&tmp_E, sizeof(float));
%                     d_dump_file.write((char*)&tmp_P, sizeof(float));
%                     d_dump_file.write((char*)&tmp_L, sizeof(float));
%                     d_dump_file.write((char*)&tmp_VL, sizeof(float));
%                     // PROMPT I and Q (to analyze navigation symbols)
%                     d_dump_file.write((char*)&prompt_I, sizeof(float));
%                     d_dump_file.write((char*)&prompt_Q, sizeof(float));
%                     // PRN start sample stamp
%                     //tmp_float=(float)d_sample_counter;
%                     d_dump_file.write((char*)&d_sample_counter, sizeof(unsigned long int));
%                     // accumulated carrier phase
%                     d_dump_file.write((char*)&d_acc_carrier_phase_rad, sizeof(float));
% 
%                     // carrier and code frequency
%                     d_dump_file.write((char*)&d_carrier_doppler_hz, sizeof(float));
%                     d_dump_file.write((char*)&d_code_freq_hz, sizeof(float));
% 
%                     //PLL commands
%                     d_dump_file.write((char*)&carr_error, sizeof(float));
%                     d_dump_file.write((char*)&carr_nco, sizeof(float));
% 
%                     //DLL commands
%                     d_dump_file.write((char*)&code_error, sizeof(float));
%                     d_dump_file.write((char*)&code_nco, sizeof(float));
% 
%                     // CN0 and carrier lock test
%                     d_dump_file.write((char*)&d_CN0_SNV_dB_Hz, sizeof(float));
%                     d_dump_file.write((char*)&d_carrier_lock_test, sizeof(float));
% 
%                     // AUX vars (for debug purposes)
%                     tmp_float = d_rem_code_phase_samples;
%                     d_dump_file.write((char*)&tmp_float, sizeof(float));
%                     tmp_double=(double)(d_sample_counter+d_current_prn_length_samples);
%                     d_dump_file.write((char*)&tmp_double, sizeof(double));
                
    VE=v1;
    E=v2;
    P=v3;
    L=v4;
    VL=v5;
    prompt_I=v6;
    prompt_Q=v7;
    PRN_start_sample=v8;
    acc_carrier_phase_rad=v9;
    carrier_doppler_hz=v10;
    code_freq_hz=v11;
    carr_error=v12;
    carr_nco=v13;
    code_error=v14;
    code_nco=v15;
    CN0_SNV_dB_Hz=v16;
    carrier_lock_test=v17;
    var1=v18;
    var2=v19;
    
    GNSS_tracking.VE=VE;
    GNSS_tracking.E=E;
    GNSS_tracking.P=P;
    GNSS_tracking.L=L;
    GNSS_tracking.VL=VL;
    GNSS_tracking.prompt_I=prompt_I;
    GNSS_tracking.prompt_Q=prompt_Q;
    GNSS_tracking.PRN_start_sample=PRN_start_sample;
    GNSS_tracking.acc_carrier_phase_rad=acc_carrier_phase_rad;
    GNSS_tracking.carrier_doppler_hz=carrier_doppler_hz;
    GNSS_tracking.code_freq_hz=code_freq_hz;
    GNSS_tracking.carr_error=carr_error;
    GNSS_tracking.carr_nco=carr_nco;
    GNSS_tracking.code_error=code_error;
    GNSS_tracking.code_nco=code_nco;
    GNSS_tracking.CN0_SNV_dB_Hz=CN0_SNV_dB_Hz;
    GNSS_tracking.carrier_lock_test=carrier_lock_test;
    GNSS_tracking.var1=var1;
    GNSS_tracking.var2=var2;
  end
  
