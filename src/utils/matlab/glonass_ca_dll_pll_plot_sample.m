% Reads GNSS-SDR Tracking dump binary file using the provided
%  function and plots some internal variables
% Damian Miralles, 2017. dmiralles2009(at)gmail.com
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

close all;
clear all;

if ~exist('dll_pll_veml_read_tracking_dump.m', 'file')
    addpath('./libs')
end


samplingFreq = 6625000;     %[Hz]
channels = 5;
first_channel = 0;

path = '/archive/';  %% CHANGE THIS PATH

for N=1:1:channels
    tracking_log_path = [path 'glo_tracking_ch_' num2str(N+first_channel-1) '.dat']; %% CHANGE glo_tracking_ch_ BY YOUR dump_filename
    GNSS_tracking(N) = dll_pll_veml_read_tracking_dump(tracking_log_path);
end

% GNSS-SDR format conversion to MATLAB GPS receiver

for N=1:1:channels
    trackResults(N).status = 'T'; %fake track
    trackResults(N).codeFreq       = GNSS_tracking(N).code_freq_hz.';
    trackResults(N).carrFreq       = GNSS_tracking(N).carrier_freq_hz.';
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
    trackResults(N).CNo = GNSS_tracking(N).CN0_SNV_dB_Hz.';
    trackResults(N).PRN = ones(1,length(GNSS_tracking(N).E));
    
    % Use original MATLAB tracking plot function
    settings.numberOfChannels = channels;
    settings.msToProcess = length(GNSS_tracking(N).E);
    plotTracking(N, trackResults, settings)
end
