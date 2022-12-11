%%
% vtl_general_plot.m
%%
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
end
% -------------------------------------
% if(load_observables)
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
%     plot(refSatData.GPS.SIM_time/1000, Pseudorange_m_sim(:,1)','.','DisplayName','SPIRENT reference')
%     plot(navSolution.RX_time(1,:)-time_reference_spirent_obs, pr_m_filt(1,:),'o','DisplayName','filtered VTL')
%     hold off
%     grid on
% end
%---VTL VELOCITY: GNSS SDR plot --------------------------------------
VTL_VEL=figure('Name','velocities');
subplot(2,2,1);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.vX,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(4,:),'.');
plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.vX...
    ,'.','DisplayName','SPIRENT reference')
% plot(navSolution.RX_time-navSolution.RX_time(1),kf_xerr(4,:),'.');
ylabel('vX (m/s)')
xlabel('time U.A.')
ylim([-5 5])
title('Subplot 1: vX ')
legend ('raw navSolution','raw kf state','SPIRENT reference','Location','eastoutside')

subplot(2,2,2);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.vY,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(5,:),'.');
% plot(navSolution.RX_time-navSolution.RX_time(1),kf_xerr(5,:),'.');
plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.vY...
    ,'.','DisplayName','SPIRENT reference')
ylabel('vY (m/s)')
xlabel('time U.A.')
ylim([-5 5])
title('Subplot 1: vY ')
legend ('raw navSolution','raw kf state','SPIRENT reference','Location','eastoutside')

subplot(2,2,3);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.vZ,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(6,:),'.');
% plot(navSolution.RX_time-navSolution.RX_time(1),kf_xerr(6,:),'.');
plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.vZ...
    ,'.','DisplayName','SPIRENT reference')
ylabel('vZ (m/s)')
xlabel('time U.A.')
ylim([-5 5])
title('Subplot 1: vZ ')
legend ('raw navSolution','raw kf state','SPIRENT reference','Location','eastoutside')

sgtitle('velocities')

%% --- VTL UTM centered POSITION: GNSS SDR  plot --------------------------------------

VTL_POS=figure('Name','VTL UTM COORD CENTERED IN 1^{ST} POSITION');
subplot(2,2,1);
plot(navSolution.X-navSolution.X(1),'.');
hold on;grid on
plot(corr_kf_state(1,3:end)-corr_kf_state(1,3))
plot(kf_xerr(1,:),'.');
ylabel('X (m)')
xlabel('time U.A.')
ylim([-200 800])
title('Subplot 1: X ')
legend ('raw navSolution','raw kf state','kferr','Location','eastoutside')

subplot(2,2,2);
plot(navSolution.Y-navSolution.Y(1),'.');
hold on;grid on
plot(corr_kf_state(2,3:end)-corr_kf_state(2,3))
plot(kf_xerr(2,:),'.');
ylabel('Y (m)')
xlabel('time U.A.')
ylim([-200 50])
title('Subplot 1: Y ')
legend ('raw navSolution','raw kf state','kferr','Location','eastoutside')

subplot(2,2,3);
plot(navSolution.Z-navSolution.Z(1),'.');
hold on;grid on
plot(corr_kf_state(3,3:end)-corr_kf_state(3,3))
plot(kf_xerr(3,:),'.');
ylabel('Z (m)')
xlabel('time U.A.')
ylim([-350 50])
title('Subplot 1: Z ')
legend ('raw navSolution','raw kf state','kferr','Location','eastoutside')

sgtitle('VTL UTM COORD CENTERED IN 1^{ST} POSITION')
%%
if(load_observables)
    %     Rx_Dopp_all=figure('Name','RX_Carrier_Doppler_hz');plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz','s')
    %     xlim([0,140]);
    %     xlabel('')
    %     ylabel('Doppler (Hz)')
    %     xlabel('time from simulation init (seconds)')
    %     grid on
    %     hold on
    %     plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim','.')
    %     plot(navSolution.RX_time(1,:)-time_reference_spirent_obs, sat_dopp_hz_filt,'o')
    %     legend('PRN 28','PRN 4','PRN 17','PRN 15','PRN 27','PRN 9','Location','eastoutside')
    %     hold off
    %     grid on

    Rx_Dopp_28=figure('Name','RX_Carrier_Doppler_hz');
    subplot(2,2,1)
    plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz(1,:)','s')
    xlim([0,140]);
    ylim([-2340,-2220]);
    xlabel('')
    ylabel('Doppler (Hz)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    legend('PRN 28 GNSS-SDR','Location','eastoutside')
    plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim(:,1)','.','DisplayName','SPIRENT reference')
    plot(navSolution.RX_time(1,:)-time_reference_spirent_obs, sat_dopp_hz_filt(1,:),'o','DisplayName','filtered VTL')
    hold off;grid minor

    %     Rx_Dopp_4=figure('Name','RX_Carrier_Doppler_hz');
    subplot(2,2,2)
    plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz(2,:)','s')
    xlim([0,140]);
    ylim([2540,2640]);
    xlabel('')
    ylabel('Doppler (Hz)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    legend('PRN 4 GNSS-SDR','Location','eastoutside')
    plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim(:,2)','.','DisplayName','SPIRENT reference')
    plot(navSolution.RX_time(1,:)-time_reference_spirent_obs, sat_dopp_hz_filt(2,:),'o','DisplayName','filtered VTL')
    hold off;grid minor

    %     Rx_Dopp_17=figure('Name','RX_Carrier_Doppler_hz');
    subplot(2,2,3)
    plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz(3,:)','s')
    xlim([0,140]);
    ylim([-1800,-1730]);
    xlabel('')
    ylabel('Doppler (Hz)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    legend('PRN 17 GNSS-SDR','Location','eastoutside')
    plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim(:,3)','.','DisplayName','SPIRENT reference')
    plot(navSolution.RX_time(1,:)-time_reference_spirent_obs, sat_dopp_hz_filt(3,:),'o','DisplayName','filtered VTL')
    hold off;grid minor

    %     Rx_Dopp_15=figure('Name','RX_Carrier_Doppler_hz');
    subplot(2,2,4)
    plot(RX_time(1,:)-time_reference_spirent_obs, Carrier_Doppler_hz(4,:)','s')
    xlim([0,140]);
    ylim([-2680,-2620]);
    xlabel('')
    ylabel('Doppler (Hz)')
    xlabel('time from simulation init (seconds)')
    grid on
    hold on
    legend('PRN 15 GNSS-SDR','Location','eastoutside')
    plot(refSatData.GPS.SIM_time/1000, Carrier_Doppler_hz_sim(:,4)','.','DisplayName','SPIRENT reference')
    plot(navSolution.RX_time(1,:)-time_reference_spirent_obs, sat_dopp_hz_filt(4,:),'o','DisplayName','filtered VTL')
    hold off;grid minor
end
%% STATE PLOT
VTL_STATE=figure('Name','VTL STATE');
subplot(2,2,1);
plot(navSolution.RX_time-navSolution.RX_time(1),[navSolution.X-navSolution.X(1);...
    navSolution.Y-navSolution.Y(1) ;navSolution.Z-navSolution.Z(1)],...
    'b.','DisplayName','RTKLIB solution');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(1:3,:)'-corr_kf_state(1:3,3)',...
    'k.','DisplayName','filt VTL');
plot(refSolution.SIM_time/1000-TTFF_sec,[refSolution.X-refSolution.X(1)...
    refSolution.Y-refSolution.Y(1) refSolution.Z-refSolution.Z(1)],...
    'r.','DisplayName','SPIRENT reference');
legend('Location','eastoutside');
ylim([-200,200])
xlim([0,120])
ylabel('X Y Z (m)')
xlabel('time [s]')
title('Subplot 1: POSITION [m]')

subplot(2,2,2);
plot(navSolution.RX_time-navSolution.RX_time(1),[navSolution.vX;...
    navSolution.vY; navSolution.vZ],...
    'b.','DisplayName','RTKLIB solution');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(4:6,:)',...
    'k.','DisplayName','filt VTL');
plot(refSolution.SIM_time/1000-TTFF_sec,[refSolution.vX...
    refSolution.vY refSolution.vZ],...
    'r.','DisplayName','SPIRENT reference');

xlim([0,120])
ylabel('vX vY vZ (m/s)')
xlabel('time [s]')
title('Subplot 1: VELOCITIES [m/s]')

subplot(2,2,3);
plot(navSolution.RX_time-navSolution.RX_time(1),clk_bias_s*SPEED_OF_LIGHT_M_S,...
    'b.','DisplayName','RTKLIB solution');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(7,:),...
    'k.','DisplayName','filt VTL');
ylim([3019190, 3019700])
xlim([0,120])
ylabel('clk bias (m)')
xlabel('time [s]')
title('Subplot 1: clk bias [m]')

subplot(2,2,4);
plot(navSolution.RX_time-navSolution.RX_time(1),clk_drift*SPEED_OF_LIGHT_M_S,...
    'b.','DisplayName','RTKLIB solution');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(8,:),...
    'k.','DisplayName','filt VTL');
xlim([0,120])
ylabel('clk drift (m/s)')
xlabel('time [s]')
title('Subplot 1: clk drift [m/s]')

sgtitle('VTL STATE')