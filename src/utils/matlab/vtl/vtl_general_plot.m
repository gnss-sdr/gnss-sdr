%%
% vtl_general_plot.m
%%
%---VTL VELOCITY: GNSS SDR plot --------------------------------------
VTL_VEL=figure('Name','velocities');
subplot(2,2,1);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.vX,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(4,:),'.');
plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.vX...
    ,'.','DisplayName','reference')
% plot(navSolution.RX_time-navSolution.RX_time(1),kf_xerr(4,:),'.');
ylabel('vX (m/s)')
xlabel('time U.A.')
ylim([-5 5])
title('Subplot 1: vX ')
legend ('raw navSolution','raw kf state','reference','Location','east')

subplot(2,2,2);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.vY,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(5,:),'.');
% plot(navSolution.RX_time-navSolution.RX_time(1),kf_xerr(5,:),'.');
plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.vY...
    ,'.','DisplayName','reference')
ylabel('vY (m/s)')
xlabel('time U.A.')
ylim([-5 5])
title('Subplot 1: vY ')
legend ('raw navSolution','raw kf state','reference','Location','east')

subplot(2,2,3);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.vZ,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(6,:),'.');
% plot(navSolution.RX_time-navSolution.RX_time(1),kf_xerr(6,:),'.');
plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.vZ...
    ,'.','DisplayName','reference')
ylabel('vZ (m/s)')
xlabel('time U.A.')
ylim([-5 5])
title('Subplot 1: vZ ')
legend ('raw navSolution','raw kf state','reference','Location','east')

sgtitle('velocities') 

%% --- VTL UTM centered POSITION: GNSS SDR  plot --------------------------------------

VTL_POS=figure('Name','VTL UTM COORD CENTERED IN 1^{ST} POSITION');
subplot(2,2,1);
plot(navSolution.X-navSolution.X(1),'.');
hold on;grid on
plot(corr_kf_state(1,:)-corr_kf_state(1,1),'.');
plot(kf_xerr(1,:),'.');
ylabel('X (m)')
xlabel('time U.A.')
ylim([-200 800])
title('Subplot 1: X ')
legend ('raw navSolution','raw kf state','kferr','Location','east')

subplot(2,2,2);
plot(navSolution.Y-navSolution.Y(1),'.');
hold on;grid on
plot(corr_kf_state(2,:)-corr_kf_state(2,1),'.');
plot(kf_xerr(2,:),'.');
ylabel('Y (m)')
xlabel('time U.A.')
ylim([-200 50])
title('Subplot 1: Y ')
legend ('raw navSolution','raw kf state','kferr','Location','east')

subplot(2,2,3);
plot(navSolution.Z-navSolution.Z(1),'.');
hold on;grid on
plot(corr_kf_state(3,:)-corr_kf_state(3,1),'.');
plot(kf_xerr(3,:),'.');
ylabel('Z (m)')
xlabel('time U.A.')
ylim([-350 50])
title('Subplot 1: Z ')
legend ('raw navSolution','raw kf state','kferr','Location','east')

sgtitle('VTL UTM COORD CENTERED IN 1^{ST} POSITION') 
%%
%% STATE PLOT
VTL_STATE=figure('Name','VTL STATE');
subplot(2,2,1);
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(1:3,:)'-corr_kf_state(1:3,3)','.');
hold on;grid on
plot(refSolution.SIM_time/1000-TTFF_sec,[refSolution.X-refSolution.X(1)...
    refSolution.Y-refSolution.Y(1) refSolution.Z-refSolution.Z(1)],...
    '.','DisplayName','reference');
ylim([-200,200])
xlim([0,120])
ylabel('X Y Z (m)')
xlabel('time U.A.')
title('Subplot 1: POSITION ')

subplot(2,2,2);
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(4:6,:)','.');
hold on;grid on
plot(refSolution.SIM_time/1000-TTFF_sec,[refSolution.vX...
    refSolution.vY refSolution.vZ],...
    '.','DisplayName','reference');
xlim([0,120])
ylabel('vX vY vZ (m/s)')
xlabel('time U.A.')
title('Subplot 1: VELOCITIES ')

subplot(2,2,3);
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(7,:),'.');
ylim([3019190, 3019700])
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),clk_bias_s*SPEED_OF_LIGHT_M_S,'.');
xlim([0,120])
ylabel('clk bias (m)')
xlabel('time U.A.')
title('Subplot 1: clk bias')
legend('vtl','rtklib')

subplot(2,2,4);
plot(navSolution.RX_time-navSolution.RX_time(1),corr_kf_state(8,:),'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),clk_drift*SPEED_OF_LIGHT_M_S,'.');
xlim([0,120])
ylabel('clk drift (m/s)')
legend('vtl','rtklib')
xlabel('time U.A.')
title('Subplot 1: clk drift ')

sgtitle('VTL STATE') 