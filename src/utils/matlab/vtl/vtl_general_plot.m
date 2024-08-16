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
%%
% vtl_general_plot.m
%%
% 
% if(load_observables)
%      for i=1:length(refSatData.GPS.SIM_time)
%         Pseudorange_m_sim(i,1)=refSatData.GPS.series(i).pr_m(9);%PRN 30
%         Pseudorange_m_sim(i,2)=refSatData.GPS.series(i).pr_m(3);%PRN 29
%         Pseudorange_m_sim(i,3)=refSatData.GPS.series(i).pr_m(5);%PRN 24
%         Pseudorange_m_sim(i,4)=refSatData.GPS.series(i).pr_m(2);%PRN 12
%         Pseudorange_m_sim(i,5)=refSatData.GPS.series(i).pr_m(8);%PRN 10
%         Pseudorange_m_sim(i,6)=refSatData.GPS.series(i).pr_m(6);%PRN 5
%     end
% end
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
%% ---VTL VELOCITY: GNSS SDR plot --------------------------------------
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
% ylim([-5 5])
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
% ylim([-5 5])
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
% ylim([-5 5])
title('Subplot 1: vZ ')
legend ('raw navSolution','raw kf state','SPIRENT reference','Location','eastoutside')

sgtitle('velocities')

%% --- VTL UTM centered POSITION: GNSS SDR  plot --------------------------------------

VTL_POS=figure('Name','VTL UTM COORD ');
subplot(2,2,1);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.X,'.');
hold on;grid on
plot(navSolution.RX_time(3:end)-navSolution.RX_time(1),corr_kf_state(1,3:end))
plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.X,'.')
ylabel('X (m)')
xlabel('time U.A.')
% ylim([-200 800])
title('Subplot 1: X ')
legend ('raw navSolution','raw kf state','kferr','Location','eastoutside')

subplot(2,2,2);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.Y,'.');
hold on;grid on
plot(navSolution.RX_time(3:end)-navSolution.RX_time(1),corr_kf_state(2,3:end))
plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.Y,'.')
ylabel('Y (m)')
xlabel('time U.A.')
% ylim([-200 50])
title('Subplot 1: Y ')
legend ('raw navSolution','raw kf state','kferr','Location','eastoutside')

subplot(2,2,3);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.Z,'.');
hold on;grid on
plot(navSolution.RX_time(3:end)-navSolution.RX_time(1),corr_kf_state(3,3:end))
plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.Z,'.')
ylabel('Z (m)')
xlabel('time U.A.')
% ylim([-350 50])
title('Subplot 1: Z ')
legend ('raw navSolution','raw kf state','SPIRENT ref','Location','eastoutside')

sgtitle('VTL UTM COORD')

%% STATE PLOT
VTL_STATE=figure('Name','VTL STATE');
subplot(2,3,1);
plot(navSolution.RX_time-navSolution.RX_time(1),[navSolution.X-navSolution.X(1);...
    navSolution.Y-navSolution.Y(1) ;navSolution.Z-navSolution.Z(1)],...
    'b.','DisplayName','RTKLIB solution');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(1:3,:)'-corr_kf_state(1:3,3)',...
    'k.','DisplayName','filt VTL');
plot(refSolution.SIM_time/1000-TTFF_sec,[refSolution.X-refSolution.X(spirent_index_TTFF)...
    refSolution.Y-refSolution.Y(spirent_index_TTFF) refSolution.Z-refSolution.Z(spirent_index_TTFF)],...
    'r.','DisplayName','SPIRENT reference');
legend('Location','eastoutside');
% ylim([-200,200])
xlim([0,tFinal])
ylabel('X Y Z (m)')
xlabel('time [s]')
title('Subplot 1: POSITION [m]')

subplot(2,3,2);
plot(navSolution.RX_time-navSolution.RX_time(1),[navSolution.vX;...
    navSolution.vY; navSolution.vZ],...
    'b.','DisplayName','RTKLIB solution');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(4:6,:)',...
    'k.','DisplayName','filt VTL');
plot(refSolution.SIM_time/1000-TTFF_sec,[refSolution.vX...
    refSolution.vY refSolution.vZ],...
    'r.','DisplayName','SPIRENT reference');

xlim([0,tFinal])
ylabel('vX vY vZ (m/s)')
xlabel('time [s]')
title('Subplot 1: VELOCITIES [m/s]')

subplot(2,3,3);
plot(navSolution.RX_time-navSolution.RX_time(1),clk_bias_s*SPEED_OF_LIGHT_M_S,...
    'b.','DisplayName','RTKLIB solution');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(7,:),...
    'k.','DisplayName','filt VTL');
% ylim([3633390, 3634580])
xlim([0,tFinal])
ylabel('clk bias (m)')
xlabel('time [s]')
title('Subplot 1: clk bias [m]')

subplot(2,3,4);
plot(navSolution.RX_time-navSolution.RX_time(1),clk_drift*SPEED_OF_LIGHT_M_S,...
    'b.','DisplayName','RTKLIB solution');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(8,:),...
    'k.','DisplayName','filt VTL');
xlim([0,tFinal])
ylabel('clk drift (m/s)')
xlabel('time [s]')
title('Subplot 1: clk drift [m/s]')

subplot(2,3,5);
plot(navSolution.RX_time(1:end-1)-navSolution.RX_time(1),diff(clk_drift)/kf_dt*SPEED_OF_LIGHT_M_S,...
    'b.','DisplayName','RTKLIB solution');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(9,:),...
    'k.','DisplayName','filt VTL');
xlim([0,tFinal])
ylabel('clk drift (m/s)')
xlabel('time [s]')
title('Subplot 1: clk drift [m/s]')

sgtitle('VTL STATE')
