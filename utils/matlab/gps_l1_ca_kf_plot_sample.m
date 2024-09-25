% Reads GNSS-SDR Tracking dump binary file using the provided
%  function and plots some internal variables
% Javier Arribas, 2011. jarribas(at)cttc.es
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


samplingFreq = 6625000;     %[Hz]
channels = 8;
first_channel = 0;
code_period = 0.001;

path    = '/archive/';  %% CHANGE THIS PATH
figpath = [path];

for N=1:1:channels
    tracking_log_path = [path 'epl_tracking_ch_' num2str(N+first_channel-1) '.dat']; %% CHANGE epl_tracking_ch_ BY YOUR dump_filename
    GNSS_tracking(N) = gps_l1_ca_kf_read_tracking_dump(tracking_log_path);
end

% GNSS-SDR format conversion to MATLAB GPS receiver

for N=1:1:channels
    trackResults(N).status = 'T'; %fake track
    trackResults(N).codeFreq       = GNSS_tracking(N).code_freq_hz.';
    trackResults(N).carrFreq       = GNSS_tracking(N).carrier_doppler_hz.';
    trackResults(N).carrFreqRate   = GNSS_tracking(N).carrier_dopplerrate_hz2.';
    trackResults(N).dllDiscr       = GNSS_tracking(N).code_error.';
    trackResults(N).dllDiscrFilt   = GNSS_tracking(N).code_nco.';
    trackResults(N).pllDiscr       = GNSS_tracking(N).carr_error.';
    trackResults(N).pllDiscrFilt   = GNSS_tracking(N).carr_nco.';

    trackResults(N).I_P = GNSS_tracking(N).prompt_I.';
    trackResults(N).Q_P = GNSS_tracking(N).prompt_Q.';

    trackResults(N).I_E = GNSS_tracking(N).E.';
    trackResults(N).I_L = GNSS_tracking(N).L.';
    trackResults(N).Q_E = zeros(1,length(GNSS_tracking(N).E));
    trackResults(N).Q_L = zeros(1,length(GNSS_tracking(N).E));
    trackResults(N).PRN = GNSS_tracking(N).PRN.';
    trackResults(N).CNo = GNSS_tracking(N).CN0_SNV_dB_Hz.';


    kalmanResults(N).PRN = GNSS_tracking(N).PRN.';
    kalmanResults(N).innovation  = GNSS_tracking(N).carr_error.';
    kalmanResults(N).state1      = GNSS_tracking(N).carr_nco.';
    kalmanResults(N).state2      = GNSS_tracking(N).carrier_doppler_hz.';
    kalmanResults(N).state3      = GNSS_tracking(N).carrier_dopplerrate_hz2.';
    kalmanResults(N).r_noise_cov = GNSS_tracking(N).carr_noise_sigma2.';
    kalmanResults(N).CNo         = GNSS_tracking(N).CN0_SNV_dB_Hz.';

    % Use original MATLAB tracking plot function
    settings.numberOfChannels = channels;
    settings.msToProcess = length(GNSS_tracking(N).E);
    settings.codePeriod  = code_period;
    settings.timeStartInSeconds = 20;

    %plotTracking(N, trackResults, settings)
    plotKalman(N, kalmanResults, settings)

    saveas(gcf, [figpath 'epl_tracking_ch_' num2str(N) '_PRN_' num2str(trackResults(N).PRN(end)) '.png'], 'png')
end
