% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% This file loads the different output .mat files and displays the
% important parameters of each output block
%
% Output blocks: Acquisition, Tracking, Telemetry decoder, Observables,
% PVT, and Monitor. 
%
% Author: J. Alfaro
% -------------------------------------------------------------------------
%
% clear; clc;
% close all;
%% Output blocks activation flags
acq.flag = 1; % 1: compute acquisition block; 0: ignore acquisition block
trk.flag = 0; % 1: compute tracking block; 0: ignore tracking block
tel.flag = 0; % 1: compute telemetry dec. block; 0: ignore telemetry dec. block
obs.flag = 0; % 1: compute observables block; 0: ignore observables block

%% Acquisition block
% Filename settings
acq.file.path         = 'D:\FGI_GNSS_Spoofing\GPS_acquisition_TGS_L1\'; % Path to acquition file
acq.file.file         = 'D:\FGI_GNSS_Spoofing\TG_SFMC\GPS_acquisition_TGS_L1_241\acq_ch_G_1C_ch_0_2537_sat_15.mat'; % Filename to acquition file
acq.file.dump_channel = 0; % Channel number defined by dump_channel, default 0
acq.file.execution    = 1; % Dump number
acq.file.satellite    = 1; % Targeted satellite’s PRN number
acq.file.signal       = 1; % Signal type
% Signal:
%     1 GPS  L1
%     2 GPS  L2M
%     3 GPS  L5
%     4 Gal. E1B
%     5 Gal. E5
%     6 Glo. 1G
%     7 Glo. 2G
%     8 BDS. B1
%     9 BDS. B3
%    10 BDS. B2a

if acq.flag; acq_results(acq); end % Execute if acq_flag is ON

%% Tracking block
trk.samplingFreq = 4000000; % Signal source sampling frequency [Hz]
% Filename settings
trk.file.file = 'trk_dump'; % Common filename to tracking files
trk.Nchan     = 1;          % Number of channels targeting the signal

if trk.flag; trk_results(trk); end % Execute if trk_flag is ON


%% Telemetry decoder block
% Filename settings
tel.file.file = 'has_data0.mat';
if tel.flag; tel_results(tel); end % Execute if trk_flag is ON


%% Observables block
obs.samplingFreq = 4000000; % Signal source sampling frequency [Hz]
% Filename settings
obs.file.file = 'a';
if obs.flag; obs_results(obs); end % Execute if obs_flag is ON
% %% PVT block
% pvt_filename = 'a';
% 
% %% Monitor block
% mon_filename = 'a';

