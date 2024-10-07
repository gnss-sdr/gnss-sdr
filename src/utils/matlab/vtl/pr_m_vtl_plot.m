% Miguel Angel Gomez, 2024. gomezlma(at)inta.es
% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------
%
%% LOAD

    Pseudorange_m_sim=zeros(length(refSatData.GPS.SIM_time),channels);

    for i=1:length(refSatData.GPS.SIM_time)
        Pseudorange_m_sim(i,1)=refSatData.GPS.series(i).pr_m(4);%PRN 28
        Pseudorange_m_sim(i,2)=refSatData.GPS.series(i).pr_m(3);%PRN 20
        Pseudorange_m_sim(i,3)=refSatData.GPS.series(i).pr_m(8);%PRN 17
        Pseudorange_m_sim(i,4)=refSatData.GPS.series(i).pr_m(6);%PRN 12
        Pseudorange_m_sim(i,5)=refSatData.GPS.series(i).pr_m(5);%PRN 9
        Pseudorange_m_sim(i,6)=refSatData.GPS.series(i).pr_m(2);%PRN 5
        Pseudorange_m_sim(i,7)=refSatData.GPS.series(i).pr_m(1);%PRN 4
        Pseudorange_m_sim(i,8)=refSatData.GPS.series(i).pr_m(7);%PRN 2
    end
%%

    figure('Name','RX_PseudoRange_m');
for channel_cnt=1:channels
    subplot(3,3,channel_cnt)
    plot(linspace(0,tFinal,length(Pseudorange_m(channel_cnt,:)')), Pseudorange_m(channel_cnt,:)','s')
    xlim([0,tFinal]);
%     ylim([min(Pseudorange_m_sim(:,channel_cnt)),max(Pseudorange_m_sim(:,channel_cnt))]);
    xlabel('')
    ylabel('PR [m] (m)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    legend(['PRN' num2str(rx_PRN(channel_cnt)) ' GNSS-SDR'],'Location','eastoutside')% bench
%     plot(refSatData.GPS.SIM_time/1000, Pseudorange_m_sim(:,channel_cnt)','.','DisplayName','SPIRENT reference')
%     plot(navSolution.RX_time(1,:)-time_reference_spirent_obs, sat_dopp_hz_filt(1,:),'o','DisplayName','filtered VTL')
    hold off;grid minor
end
%%
    figure('Name','Rsim_PseudoRange_m');
for channel_cnt=1:channels
    subplot(3,3,channel_cnt)
    plot(linspace(0,tFinal,length(Pseudorange_m(channel_cnt,:)')), Pseudorange_m(channel_cnt,:)','s')
    xlim([0,tFinal]);
    ylim([min(Pseudorange_m_sim(:,channel_cnt)),max(Pseudorange_m_sim(:,channel_cnt))]);
    xlabel('')
    ylabel('PR [m] (m)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    legend(['PRN' num2str(rx_PRN(channel_cnt)) ' GNSS-SDR'],'Location','eastoutside')% bench
    plot(refSatData.GPS.SIM_time/1000, Pseudorange_m_sim(:,channel_cnt)','.','DisplayName','SPIRENT reference')
%     plot(navSolution.RX_time(1,:)-time_reference_spirent_obs, sat_dopp_hz_filt(1,:),'o','DisplayName','filtered VTL')
    hold off;grid minor
end
