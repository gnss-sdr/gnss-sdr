% /*!
%  * \file gps_l1_ca_dll_fll_pll_plot_sample.m
%  * \brief Read GNSS-SDR Tracking dump binary file using the provided
%  function and plot some internal variables
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
%close all;
%clear all;
samplingFreq       = 64e6/16;     %[Hz]
channels=4;
path='/home/javier/workspace/gnss-sdr-ref/trunk/install/';
for N=1:1:channels
    tracking_log_path=[path 'tracking_ch_' num2str(N-1) '.dat'];
    GNSS_tracking(N)= gps_l1_ca_dll_fll_pll_read_tracking_dump(tracking_log_path,samplingFreq);   
end

% GNSS-SDR format conversion to MATLAB GPS receiver
channel_PRN_ID=[32,14,20,11];
tracking_loop_start=1;%10001;
tracking_loop_end=70000;
for N=1:1:channels
        trackResults_sdr(N).status='T'; %fake track
        trackResults_sdr(N).codeFreq=GNSS_tracking(N).code_freq_hz(tracking_loop_start:tracking_loop_end).';
        trackResults_sdr(N).carrFreq=GNSS_tracking(N).carrier_doppler_hz(tracking_loop_start:tracking_loop_end).';
        trackResults_sdr(N).dllDiscr       = GNSS_tracking(N).code_error_chips(tracking_loop_start:tracking_loop_end).';
        trackResults_sdr(N).dllDiscrFilt   = GNSS_tracking(N).code_phase_samples(tracking_loop_start:tracking_loop_end).';
        trackResults_sdr(N).pllDiscr       = GNSS_tracking(N).PLL_discriminator_hz(tracking_loop_start:tracking_loop_end).';
        trackResults_sdr(N).pllDiscrFilt   = GNSS_tracking(N).carr_nco(tracking_loop_start:tracking_loop_end).';
        trackResults_sdr(N).absoluteSample = (GNSS_tracking(N).var2(tracking_loop_start:tracking_loop_end)+GNSS_tracking(N).var1(tracking_loop_start:tracking_loop_end)).';
        
        trackResults_sdr(N).prn_delay_ms = 1000*trackResults_sdr(N).absoluteSample/samplingFreq;
        %trackResults_sdr(N).absoluteSample = (GNSS_tracking(N).PRN_start_sample(tracking_loop_start:tracking_loop_end)+GNSS_tracking(N).var1(tracking_loop_start:tracking_loop_end)).';

        trackResults_sdr(N).I_P=GNSS_tracking(N).prompt_I(tracking_loop_start:tracking_loop_end).';
        trackResults_sdr(N).Q_P=GNSS_tracking(N).prompt_Q(tracking_loop_start:tracking_loop_end).';

        trackResults_sdr(N).I_E= GNSS_tracking(N).E(tracking_loop_start:tracking_loop_end).';
        trackResults_sdr(N).I_L = GNSS_tracking(N).L(tracking_loop_start:tracking_loop_end).';
        trackResults_sdr(N).Q_E = zeros(1,tracking_loop_end-tracking_loop_start+1);
        trackResults_sdr(N).Q_L =zeros(1,tracking_loop_end-tracking_loop_start+1);
        trackResults_sdr(N).PRN=channel_PRN_ID(N);
        
        % Use original MATLAB tracking plot function
        settings.numberOfChannels=channels;
        settings.msToProcess=tracking_loop_end-tracking_loop_start+1;
        %plotTracking(N,trackResults_sdr,settings)
end


% for N=1:1:channels
%  figure;
%  plot([GNSS_tracking(N).E,GNSS_tracking(N).P,GNSS_tracking(N).L],'-*');
%  title(['Early, Prompt, and Late correlator absolute value output for channel ' num2str(N)']);
%  figure;
%  plot(GNSS_tracking(N).prompt_I,GNSS_tracking(N).prompt_Q,'+');
%  title(['Navigation constellation plot for channel ' num2str(N)]);
%  figure;
%  
%  plot(GNSS_tracking(N).prompt_Q,'r');
%  hold on;
%  plot(GNSS_tracking(N).prompt_I);
%  title(['Navigation symbols I(red) Q(blue) for channel ' num2str(N)]);
% end


