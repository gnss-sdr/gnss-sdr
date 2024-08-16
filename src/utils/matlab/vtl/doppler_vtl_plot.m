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

    Carrier_Doppler_hz_sim=zeros(length(refSatData.GPS.SIM_time),channels);

    for i=1:length(refSatData.GPS.SIM_time)
        Carrier_Doppler_hz_sim(i,1)=refSatData.GPS.series(i).doppler_shift(4);%PRN 28
        Carrier_Doppler_hz_sim(i,2)=refSatData.GPS.series(i).doppler_shift(3);%PRN 20
        Carrier_Doppler_hz_sim(i,3)=refSatData.GPS.series(i).doppler_shift(8);%PRN 17
        Carrier_Doppler_hz_sim(i,4)=refSatData.GPS.series(i).doppler_shift(6);%PRN 12
        Carrier_Doppler_hz_sim(i,5)=refSatData.GPS.series(i).doppler_shift(5);%PRN 9
        Carrier_Doppler_hz_sim(i,6)=refSatData.GPS.series(i).doppler_shift(2);%PRN 5
        Carrier_Doppler_hz_sim(i,7)=refSatData.GPS.series(i).doppler_shift(1);%PRN 4
        Carrier_Doppler_hz_sim(i,8)=refSatData.GPS.series(i).doppler_shift(7);%PRN 2
    end

%%

    figure('Name','RX_Carrier_Doppler_hz');
for channel_cnt=1:channels
    subplot(3,3,channel_cnt)
    plot(linspace(0,tFinal,length(Carrier_Doppler_hz(channel_cnt,:)')), Carrier_Doppler_hz(channel_cnt,:)','s')
    xlim([0,tFinal]);
    ylim([min(Carrier_Doppler_hz_sim(:,channel_cnt+1)),max(Carrier_Doppler_hz_sim(:,channel_cnt+1))]);
    xlabel('')
    ylabel('Doppler (Hz)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    legend(['PRN' num2str(rx_PRN(channel_cnt)) ' GNSS-SDR'],'Location','eastoutside')% bench
    plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim(:,channel_cnt+1)','.','DisplayName','SPIRENT reference')
     plot(linspace(TTFF_sec+23,tFinal,length(sat_dopp_hz_filt(channel_cnt,:))),sat_dopp_hz_filt(channel_cnt,:),'o','DisplayName','filtered VTL')
    hold off;grid minor
end


