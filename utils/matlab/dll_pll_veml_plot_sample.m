% Reads GNSS-SDR Tracking dump binary file using the provided
%  function and plots some internal variables

% Javier Arribas, 2011. jarribas(at)cttc.es
% Antonio Ramos,  2018. antonio.ramos(at)cttc.es
% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------
%

close all;
clear all;

if ~exist('dll_pll_veml_read_tracking_dump.m', 'file')
    addpath('./libs')
end

samplingFreq = 3000000;     %[Hz]
plot_last_outputs=0;%1000;

channels = 1;   % Number of channels
first_channel = 0;  % Number of the first channel

path = '/home/javier/git/gnss-sdr/install/test_inta/';  %% CHANGE THIS PATH

for N=1:1:channels
    tracking_log_path = [path 'tracking_ch_' num2str(N+first_channel-1) '.dat']; %% CHANGE track_ch_ BY YOUR dump_filename
    GNSS_tracking(N) = dll_pll_veml_read_tracking_dump(tracking_log_path);
end

% GNSS-SDR format conversion to MATLAB GPS receiver

for N=1:1:channels
    trackResults(N).status = 'T'; %fake track
    if plot_last_outputs>0 && plot_last_outputs<length(GNSS_tracking(N).code_freq_hz)

        start_sample=length(GNSS_tracking(N).code_freq_hz)-plot_last_outputs;
    else
        start_sample=1;
    end
    trackResults(N).codeFreq       = GNSS_tracking(N).code_freq_hz(start_sample:end).';
    trackResults(N).carrFreq       = GNSS_tracking(N).carrier_doppler_hz(start_sample:end).';
    trackResults(N).dllDiscr       = GNSS_tracking(N).code_error(start_sample:end).';
    trackResults(N).dllDiscrFilt   = GNSS_tracking(N).code_nco(start_sample:end).';
    trackResults(N).pllDiscr       = GNSS_tracking(N).carr_error(start_sample:end).';
    trackResults(N).pllDiscrFilt   = GNSS_tracking(N).carr_nco(start_sample:end).';

    trackResults(N).I_P = GNSS_tracking(N).P(start_sample:end).';
    trackResults(N).Q_P = zeros(1,length(GNSS_tracking(N).P(start_sample:end)));

    trackResults(N).I_VE = GNSS_tracking(N).VE(start_sample:end).';
    trackResults(N).I_E = GNSS_tracking(N).E(start_sample:end).';
    trackResults(N).I_L = GNSS_tracking(N).L(start_sample:end).';
    trackResults(N).I_VL = GNSS_tracking(N).VL(start_sample:end).';
    trackResults(N).Q_VE = zeros(1,length(GNSS_tracking(N).VE(start_sample:end)));
    trackResults(N).Q_E = zeros(1,length(GNSS_tracking(N).E(start_sample:end)));
    trackResults(N).Q_L = zeros(1,length(GNSS_tracking(N).L(start_sample:end)));
    trackResults(N).Q_VL = zeros(1,length(GNSS_tracking(N).VL(start_sample:end)));
    trackResults(N).data_I = GNSS_tracking(N).prompt_I(start_sample:end).';
    trackResults(N).data_Q = GNSS_tracking(N).prompt_Q(start_sample:end).';
    trackResults(N).PRN = GNSS_tracking(N).PRN(start_sample:end).';
    trackResults(N).CNo = GNSS_tracking(N).CN0_SNV_dB_Hz(start_sample:end).';
    trackResults(N).prn_start_time_s = GNSS_tracking(N).PRN_start_sample(start_sample:end)/samplingFreq;
    % Use original MATLAB tracking plot function
    settings.numberOfChannels = channels;
    plotVEMLTracking(N, trackResults, settings)
end

%Doppler plot (optional)
% for N=1:1:channels
%     figure;
%     plot(trackResults(N).prn_start_time_s , GNSS_tracking(N).carrier_doppler_hz(start_sample:end).' / 1000);
%     xlabel('Time(s)'); ylabel('Doppler(KHz)'); title(['Doppler frequency channel ' num2str(N)]);
% end
