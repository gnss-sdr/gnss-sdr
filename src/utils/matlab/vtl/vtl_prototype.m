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
point_of_closure=3000;
%%
samplingFreq=5000000;
channels=6;
TTFF_sec=41.48;

plot_skyplot=0;
plot_reference=1;
load_observables=1;

%% ============================ PARSER TO STRUCT ============================

navSolution = GnssSDR2struct('PVT_raw.mat');
refSolution = SpirentMotion2struct('..\log_spirent\motion_V1_SPF_LD_05.csv');
%
load observables\observable_raw.mat
% refSatData = SpirentSatData2struct('..\log_spirent\sat_data_V1A1_SPF_LD_05.csv');
rx_PRN=[28 4 17 15 27 9]; % for SPF_LD_05.
load('PVT_raw.mat','sat_posX_m','sat_posY_m','sat_posZ_m','sat_velX','sat_velY'...
        ,'sat_velZ','sat_prg_m','clk_bias_s','clk_drift','sat_dopp_hz','user_clk_offset')

if(load_observables)
    load observables\observable_raw.mat
    refSatData = SpirentSatData2struct('..\log_spirent\sat_data_V1A1_SPF_LD_05.csv');
end
%%
% vtlSolution = Vtl2struct('dump_vtl_file.csv');
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
figure;plot(kf_yerr(1:5,:)');title('c pr m-error');xlabel('t U.A');ylabel('pr m [m]');grid minor
legend('PRN 28','PRN 4','PRN 17','PRN 15','PRN 27','Location','eastoutside')
figure;plot(kf_yerr(6:10,:)');title('c pr m DOT-error');xlabel('t U.A');ylabel('pr m dot [m/s]');grid minor
legend('PRN 28','PRN 4','PRN 17','PRN 15','PRN 27','Location','eastoutside')

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

%% ============================================== ==============================================
time_reference_spirent_obs=129780;%s

if(load_observables)

    Carrier_Doppler_hz_sim=zeros(length(refSatData.GPS.SIM_time),6);

    for i=1:length(refSatData.GPS.SIM_time)
        Carrier_Doppler_hz_sim(i,1)=refSatData.GPS.series(i).doppler_shift(4);%PRN 28
        Carrier_Doppler_hz_sim(i,2)=refSatData.GPS.series(i).doppler_shift(1);%PRN 4
        Carrier_Doppler_hz_sim(i,3)=refSatData.GPS.series(i).doppler_shift(6);%PRN 17
        Carrier_Doppler_hz_sim(i,4)=refSatData.GPS.series(i).doppler_shift(7);%PRN 15
        Carrier_Doppler_hz_sim(i,5)=refSatData.GPS.series(i).doppler_shift(8);%PRN 27
        Carrier_Doppler_hz_sim(i,6)=refSatData.GPS.series(i).doppler_shift(9);%PRN 9

        Pseudorange_m_sim(i,1)=refSatData.GPS.series(i).pr_m(4);%PRN 28
        Pseudorange_m_sim(i,2)=refSatData.GPS.series(i).pr_m(1);%PRN 4
        Pseudorange_m_sim(i,3)=refSatData.GPS.series(i).pr_m(6);%PRN 17
        Pseudorange_m_sim(i,4)=refSatData.GPS.series(i).pr_m(7);%PRN 15
        Pseudorange_m_sim(i,5)=refSatData.GPS.series(i).pr_m(8);%PRN 27
        Pseudorange_m_sim(i,6)=refSatData.GPS.series(i).pr_m(9);%PRN 9
    end


    Rx_Dopp_all=figure('Name','RX_Carrier_Doppler_hz');plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz','s')
    xlim([0,140]);
    xlabel('')
    ylabel('Doppler (Hz)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim','.')
    plot(navSolution.RX_time(1,:)-time_reference_spirent_obs, sat_dopp_hz_filt,'o')
    legend('PRN 28','PRN 4','PRN 17','PRN 15','PRN 27','PRN 9','Location','eastoutside')
    hold off
    grid on

    Rx_Dopp_one=figure('Name','RX_Carrier_Doppler_hz');plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz(1,:)','s')
    xlim([0,140]);
    ylim([-2340,-2220]);
    xlabel('')
    ylabel('Doppler (Hz)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    legend('PRN 28 GNSS-SDR','Location','eastoutside')
    plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim(:,1)','.','DisplayName','reference')
    plot(navSolution.RX_time(1,:)-time_reference_spirent_obs, sat_dopp_hz_filt(1,:),'o','DisplayName','filtered VTL')
    
    hold off
    grid on
% -------------------------------------
%     Rx_pseudo_all=figure('Name','RX_pr_m');plot(RX_time(1,:)-time_reference_spirent_obs, Pseudorange_m','s')
%     xlim([0,140]);
%     xlabel('')
%     ylabel('Pseudorange (m)')
%     xlabel('time from simulation init (seconds)')
%     grid on
%     hold on
%     plot(refSatData.GPS.SIM_time/1000, Pseudorange_m_sim','.')
%     plot(navSolution.RX_time(1,:)-time_reference_spirent_obs, pr_m_filt,'o')
%     legend('PRN 28','PRN 4','PRN 17','PRN 15','PRN 27','PRN 9','Location','eastoutside')
%     hold off
%     grid on
% 
%     Rx_pseudo_one=figure('Name','RX_pr_m');plot(RX_time(1,:)-time_reference_spirent_obs, Pseudorange_m(1,:)','s')
%     xlim([0,140]);
%     xlabel('')
%     ylabel('Pseudorange (m)')
%     xlabel('time from simulation init (seconds)')
%     grid on
%     hold on
%     legend('PRN 28 GNSS-SDR','Location','eastoutside')
%     plot(refSatData.GPS.SIM_time/1000, Pseudorange_m_sim(:,1)','.','DisplayName','reference')
%     plot(navSolution.RX_time(1,:)-time_reference_spirent_obs, pr_m_filt(1,:),'o','DisplayName','filtered VTL')
%     hold off
%     grid on
end

