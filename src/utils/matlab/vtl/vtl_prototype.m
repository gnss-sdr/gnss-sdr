% VTL prototype
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
point_of_closure=6000;
%%
samplingFreq=5000000;
channels=6;
TTFF_sec=41.48;

%% ============================ PARSER TO STRUCT ============================
plot_skyplot=0;
plot_reference=1;

navSolution = GnssSDR2struct('PVT_raw.mat');
refSolution = SpirentMotion2struct('..\log_spirent\motion_V1_SPF_LD_05.csv');
%
load observables\observable_raw.mat
% refSatData = SpirentSatData2struct('..\log_spirent\sat_data_V1A1_SPF_LD_05.csv');
rx_PRN=[28 4 17 15 27 9]; % for SPF_LD_05.
load('PVT_raw.mat','sat_posX_m','sat_posY_m','sat_posZ_m','sat_velX','sat_velY'...
        ,'sat_velZ','sat_prg_m','clk_bias_s','clk_drift','sat_dopp_hz','user_clk_offset')
%%
vtlSolution = Vtl2struct('dump_vtl_file.csv');
%% calculate LOS Rx-sat

% for chan=1:5
%     for t=1:length(navSolution.RX_time)
%         d(chan)=(sat_posX_m(chan,t)-navSolution.X(t))^2;
%         d(chan)=d(chan)+(sat_posY_m(chan,t)-navSolution.Y(t))^2;
%         d(chan)=d(chan)+(sat_posZ_m(chan,t)-navSolution.Z(t))^2;
%         d(chan)=sqrt(d(chan));
%         
%         c_pr_m(chan,t)=d(chan)+clk_bias_s(t)*SPEED_OF_LIGHT_M_S;
%         
%         a_x(chan,t)=-(sat_posX_m(chan,t)-navSolution.X(t))/d(chan);
%         a_y(chan,t)=-(sat_posY_m(chan,t)-navSolution.Y(t))/d(chan);
%         a_z(chan,t)=-(sat_posZ_m(chan,t)-navSolution.Z(t))/d(chan);
%     end
% end
% 
% %%

% %%
% for chan=1:5
%     for t=1:length(navSolution.RX_time)
%         rhoDot_pri(chan,t)=(sat_velX(chan,t)-navSolution.vX(t))*a_x(chan,t)...
%             +(sat_velY(chan,t)-navSolution.vY(t))*a_y(chan,t)...
%             +(sat_velZ(chan,t)-navSolution.vZ(t))*a_z(chan,t);
%         
%         kf_yerr(chan,t)=(sat_dopp_hz(chan,t)*Lambda_GPS_L1)-rhoDot_pri(chan,t);
%     end
% end

%%

kf_prototype

%%
figure;plot(kf_yerr(1:5,:)');title('c_pr_m-error');xlabel('t U.A');ylabel('pr_m [m]');grid minor
legend('PRN 28','PRN 4','PRN 17','PRN 15','PRN 27','Location','eastoutside')
figure;plot(kf_yerr(6:10,:)');title('c_pr_m_dot-error');xlabel('t U.A');ylabel('pr_m_dot [m/s]');grid minor
legend('PRN 28','PRN 4','PRN 17','PRN 15','PRN 27','Location','eastoutside')
%%
% figure;plot([a_x(1,:);a_y(1,:);a_z(1,:)]');
% figure;plot([vtlSolution.LOSx vtlSolution.LOSy vtlSolution.LOSz])
%% ====== FILTERING =======================================================
% moving_avg_factor= 500;
% LAT_FILT = movmean(navSolution.latitude,moving_avg_factor);
% LON_FILT = movmean(navSolution.longitude,moving_avg_factor);
% HEIGH_FILT = movmean(navSolution.height,moving_avg_factor);
% 
% X_FILT = movmean(navSolution.X,moving_avg_factor);
% Y_FILT = movmean(navSolution.Y,moving_avg_factor);
% Z_FILT = movmean(navSolution.Z,moving_avg_factor);
% 
% vX_FILT = movmean(navSolution.vX,moving_avg_factor);
% vY_FILT = movmean(navSolution.vY,moving_avg_factor);
% vZ_FILT = movmean(navSolution.vZ,moving_avg_factor);
% 
%%
%general_raw_plot
vtl_general_plot
%%
% close all
% figure;plot(kf_xerr(7,:),'.');title('clk bias err')
% figure;plot(kf_xerr(8,:),'.');title('clk drift err')
% figure;plot(kf_x(7,:),'.');title('clk bias state')
% figure;plot(kf_x(8,:),'.');title('clk drift state')
% figure;plot(corr_kf_state(7,:),'.');title('clk bias corrected state')
% figure;plot(corr_kf_state(8,:),'.');title('clk drift corrected state')
%%
% close all
% kferr_pos_all=[vtlSolution.kferr.X vtlSolution.kferr.Y vtlSolution.kferr.Z];
% kferr_vel_all=[vtlSolution.kferr.vX vtlSolution.kferr.vY vtlSolution.kferr.vZ];
% figure;plot(kferr_pos_all,'.');title('original pos err') 
% figure;plot(kf_xerr(1:3,:)','.');title('calc pos err')
% figure;plot(kferr_vel_all,'.');title('original vel err')
% figure;plot(kf_xerr(4:6,:)','.');title('calc vel err')
%% ============================================== ==============================================
% time_reference_spirent_obs=129780;%s
% if(load_observables)
% %     figure;plot(Carrier_phase_cycles','.')
% %     figure;plot(Pseudorange_m','.')
%         %%%
%     Carrier_Doppler_hz_sim=zeros(length(refSatData.GPS.SIM_time),6);
%     
%     for i=1:length(refSatData.GPS.SIM_time)
%         Carrier_Doppler_hz_sim(i,1)=refSatData.GPS.series(i).doppler_shift(4);%PRN 28
%         Carrier_Doppler_hz_sim(i,2)=refSatData.GPS.series(i).doppler_shift(1);%PRN 4
%         Carrier_Doppler_hz_sim(i,3)=refSatData.GPS.series(i).doppler_shift(6);%PRN 17
%         Carrier_Doppler_hz_sim(i,4)=refSatData.GPS.series(i).doppler_shift(7);%PRN 15
%         Carrier_Doppler_hz_sim(i,5)=refSatData.GPS.series(i).doppler_shift(8);%PRN 27
%         Carrier_Doppler_hz_sim(i,6)=refSatData.GPS.series(i).doppler_shift(9);%PRN 9
%         
%     end
%     
%     
%     Rx_Dopp_all=figure('Name','RX_Carrier_Doppler_hz');plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz','.')
%     xlim([0,140]);
%     xlabel('')
%     ylabel('Doppler (Hz)')
%     xlabel('time from simulation init (seconds)')
%     grid on
%     hold on
%     legend('PRN 28','PRN 4','PRN 17','PRN 15','PRN 27','PRN 9','Location','eastoutside')
%     plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim','.')
%     legend('PRN 28','PRN 4','PRN 17','PRN 15','PRN 27','PRN 9','Location','eastoutside')
%     hold off
%     grid on
%     
%     Rx_Dopp_one=figure('Name','RX_Carrier_Doppler_hz');plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz(1,:)','.')
%     xlim([0,140]);
%     ylim([-2340,-2220]);
%     xlabel('')
%     ylabel('Doppler (Hz)')
%     xlabel('time from simulation init (seconds)')
%     grid on
%     hold on
%     legend('PRN 28 GNSS-SDR','Location','eastoutside')
%     plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim(:,1)','.','DisplayName','reference')
%     hold off
%     grid on
% end