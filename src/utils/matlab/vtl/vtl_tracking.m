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
%%
clc
close all
clearvars 

% if ~exist('gps_l1_ca_read_pvt_raw_dump.m', 'file')
%     addpath('./libs')
% end
% 
% if ~exist('cat2geo.m', 'file')
%     addpath('./libs/geoFunctions')
% end
SPEED_OF_LIGHT_M_S=299792458.0;
Lambda_GPS_L1=0.1902937;
%%
trkSolution=trk2struct('dump_trk_file.csv');

%% split by solution type
figure;sgtitle('real doppler')
for chan=0:4
eval(['[indCH' num2str(chan) ',~]= find(trkSolution.dopp.real==chan);'])
eval(['Dopp_real_CH' num2str(chan) '=trkSolution.dopp.real(indCH' num2str(chan) ',2);'])
eval(['subplot(2,3,' num2str(chan+1) ');plot(Dopp_real_CH' num2str(chan) ')'])
end
figure;sgtitle('cmd doppler')
for chan=0:4
eval(['[indCH' num2str(chan) ',~]= find(trkSolution.dopp.cmd==chan);'])
eval(['Dopp_cmd_CH' num2str(chan) '=trkSolution.dopp.cmd(indCH' num2str(chan) ',2);'])
eval(['subplot(2,3,' num2str(chan+1) ');plot(Dopp_cmd_CH' num2str(chan) ')'])
end

%%
% for chan=0:4
% load(['tracking\tracking_raw' num2str(chan) '.mat']);
% tracking_channel(chan+1).PRN=PRN;
% tracking_channel(chan+1).CN0_SNV_dB_Hz=CN0_SNV_dB_Hz;
% tracking_channel(chan+1).carrier_doppler_hz=carrier_doppler_hz;
% tracking_channel(chan+1).carrier_doppler_rate_hz=carrier_doppler_rate_hz;
% tracking_channel(chan+1).code_error_chips=code_error_chips;
% tracking_channel(chan+1).code_freq_chips=code_freq_chips;
% tracking_channel(chan+1).code_freq_rate_chips=code_freq_rate_chips;
% tracking_channel(chan+1).acc_carrier_phase_rad=acc_carrier_phase_rad;
% 
% clearvars -except tracking_channel chan
% end
% %%
% figure
% for chan=1:5
% subplot(2,3,chan);
% plot(tracking_channel(chan).carrier_doppler_hz);
% grid minor
% end
% sgtitle('carrier doppler hz channel')
% %%
% figure
% for chan=1:5
% subplot(2,3,chan);
% plot(tracking_channel(chan).CN0_SNV_dB_Hz);
% grid minor
% end
% sgtitle('CN0 SNV dB Hz channel')
% 
% %%
% figure
% for chan=1:5
% subplot(2,3,chan);
% plot(tracking_channel(chan).acc_carrier_phase_rad);
% grid minor
% end
% sgtitle('acc carrier phase rad')