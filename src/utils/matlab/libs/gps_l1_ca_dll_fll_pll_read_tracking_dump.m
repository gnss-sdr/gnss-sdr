% /*!
%  * \file gps_l1_ca_dll_fll_pll_read_tracking_dump.m
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
function [GNSS_tracking] = gps_l1_ca_dll_fll_pll_read_tracking_dump (filename, samplingFreq, count)

  %% usage: gps_l1_ca_dll_fll_pll_read_tracking_dump (filename, [count])
  %%
  %% open GNSS-SDR tracking binary log file .dat and return the contents
  %%

  m = nargchk (1,3,nargin);
  num_float_vars=16;
  num_double_vars=1;
  double_size_bytes=8;
  float_size_bytes=4;
  skip_bytes_each_read=float_size_bytes*num_float_vars+double_size_bytes*num_double_vars;
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
    v6 = fread (f, count, 'uint32',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v7 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v8 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v9 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v10 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v11 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v12 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v13 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
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
    v17 = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
    fclose (f);
    
    %%%%%%%% output vars %%%%%%%%

% 			// EPR
% 			d_dump_file.write((char*)&tmp_E, sizeof(float));
% 			d_dump_file.write((char*)&tmp_P, sizeof(float));
% 			d_dump_file.write((char*)&tmp_L, sizeof(float));
% 			// PROMPT I and Q (to analyze navigation symbols)
% 			d_dump_file.write((char*)&prompt_I, sizeof(float));
% 			d_dump_file.write((char*)&prompt_Q, sizeof(float));
% 			// PRN start sample stamp
% 			//tmp_float=(float)d_sample_counter;
% 			d_dump_file.write((char*)&d_sample_counter, sizeof(unsigned long int));
% 			// accumulated carrier phase
% 			d_dump_file.write((char*)&d_acc_carrier_phase_rad, sizeof(float));
% 
% 			// carrier and code frequency
% 			d_dump_file.write((char*)&d_carrier_doppler_hz, sizeof(float));
% 			d_dump_file.write((char*)&d_code_freq_hz, sizeof(float));
% 
% 			//PLL commands
% 			d_dump_file.write((char*)&PLL_discriminator_hz, sizeof(float));
% 			d_dump_file.write((char*)&carr_nco_hz, sizeof(float));
% 
% 			//DLL commands
% 			d_dump_file.write((char*)&code_error_chips, sizeof(float));
% 			d_dump_file.write((char*)&d_code_phase_samples, sizeof(float));
% 
% 			// CN0 and carrier lock test
% 			d_dump_file.write((char*)&d_CN0_SNV_dB_Hz, sizeof(float));
% 			d_dump_file.write((char*)&d_carrier_lock_test, sizeof(float));
% 
% 			// AUX vars (for debug purposes)
% 			tmp_float=0;
% 			d_dump_file.write((char*)&tmp_float, sizeof(float));
% 			d_dump_file.write((char*)&d_sample_counter_seconds, sizeof(double));
            
    E=v1;
    P=v2;
    L=v3;
    prompt_I=v4;
    prompt_Q=v5;
    PRN_start_sample=v6;
    acc_carrier_phase_rad=v7;
    carrier_doppler_hz=v8;
    code_freq_hz=v9;
    PLL_discriminator_hz=v10;
    carr_nco_hz=v11;
    code_error_chips=v12;
    code_phase_samples=v13;
    CN0_SNV_dB_Hz=v14;
    carrier_lock_test=v15;
    var1=v16;
    var2=v17;
    
    GNSS_tracking.E=E;
    GNSS_tracking.P=P;
    GNSS_tracking.L=L;
    GNSS_tracking.prompt_I=prompt_I;
    GNSS_tracking.prompt_Q=prompt_Q;
    GNSS_tracking.PRN_start_sample=PRN_start_sample;
    GNSS_tracking.acc_carrier_phase_rad=acc_carrier_phase_rad;
    GNSS_tracking.carrier_doppler_hz=carrier_doppler_hz;
    GNSS_tracking.code_freq_hz=code_freq_hz;
    GNSS_tracking.PLL_discriminator_hz=PLL_discriminator_hz;
    GNSS_tracking.carr_nco=carr_nco_hz;
    GNSS_tracking.code_error_chips=code_error_chips;
    GNSS_tracking.code_phase_samples=code_phase_samples;
    GNSS_tracking.CN0_SNV_dB_Hz=CN0_SNV_dB_Hz;
    GNSS_tracking.carrier_lock_test=carrier_lock_test;
    GNSS_tracking.var1=var1;
    GNSS_tracking.var2=var2;
    GNSS_tracking.prn_delay_ms=1000*(GNSS_tracking.var2+GNSS_tracking.var1)./samplingFreq;
  end
  
