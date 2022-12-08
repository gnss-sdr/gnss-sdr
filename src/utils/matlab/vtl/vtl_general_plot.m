%%
% vtl_general_plot.m
%%
%---VTL VELOCITY: GNSS SDR plot --------------------------------------
VTL_VEL=figure('Name','velocities');
subplot(2,2,1);
plot(navSolution.vX,'.');
hold on;grid on
plot(kf_x(4,:),'.');
plot(kf_xerr(4,:),'.');
ylabel('vX (m/s)')
xlabel('time U.A.')
ylim([-5 5])
title('Subplot 1: vX ')
legend ('raw navSolution','raw kf state','kferr','Location','east')

subplot(2,2,2);
plot(navSolution.vY,'.');
hold on;grid on
plot(kf_x(5,:),'.');
plot(kf_xerr(5,:),'.');
ylabel('vY (m/s)')
xlabel('time U.A.')
ylim([-5 5])
title('Subplot 1: vY ')
legend ('raw navSolution','raw kf state','kferr','Location','east')

subplot(2,2,3);
plot(navSolution.vZ,'.');
hold on;grid on
plot(kf_x(6,:),'.');
plot(kf_xerr(6,:),'.');
ylabel('vZ (m/s)')
xlabel('time U.A.')
ylim([-5 5])
title('Subplot 1: vZ ')
legend ('raw navSolution','raw kf state','kferr','Location','east')

sgtitle('velocities') 

%% --- VTL UTM centered POSITION: GNSS SDR  plot --------------------------------------

VTL_POS=figure('Name','VTL UTM COORD CENTERED IN 1^{ST} POSITION');
subplot(2,2,1);
plot(navSolution.X-navSolution.X(1),'.');
hold on;grid on
plot(kf_x(1,:)-kf_x(1,1),'.');
plot(kf_xerr(1,:),'.');
ylabel('X (m)')
xlabel('time U.A.')
ylim([-200 800])
title('Subplot 1: X ')
legend ('raw navSolution','raw kf state','kferr','Location','east')

subplot(2,2,2);
plot(navSolution.Y-navSolution.Y(1),'.');
hold on;grid on
plot(kf_x(2,:)-kf_x(2,1),'.');
plot(kf_xerr(2,:),'.');
ylabel('Y (m)')
xlabel('time U.A.')
ylim([-200 50])
title('Subplot 1: Y ')
legend ('raw navSolution','raw kf state','kferr','Location','east')

subplot(2,2,3);
plot(navSolution.Z-navSolution.Z(1),'.');
hold on;grid on
plot(kf_x(3,:)-kf_x(3,1),'.');
plot(kf_xerr(3,:),'.');
ylabel('Z (m)')
xlabel('time U.A.')
ylim([-350 50])
title('Subplot 1: Z ')
legend ('raw navSolution','raw kf state','kferr','Location','east')

sgtitle('VTL UTM COORD CENTERED IN 1^{ST} POSITION') 
%%
% % --- 'VTL errPV correction --------------------------------------
% 
% VTL_errPV=figure('Name','VTL errPV correction');
% subplot(2,3,1);
% plot(navSolution.X-navSolution.X(1),'.');
% hold on;grid on
% plot(vtlSolution.kfpvt.X-vtlSolution.kfpvt.X(1),'.');
% plot(vtlSolution.kferr.X,'.');
% plot(vtlSolution.kfpvt.X-navSolution.X(1)+vtlSolution.kferr.X,'.');
% ylabel('X (m)')
% xlabel('time U.A.')
% ylim([-200 800])
% title('Subplot 1: X ')
% legend ('raw RTKlib','raw kf','kferr','added err+raw','Location','eastoutside')
% 
% subplot(2,3,2);
% plot(navSolution.Y-navSolution.Y(1),'.');
% hold on;grid on
% plot(vtlSolution.kfpvt.Y-vtlSolution.kfpvt.Y(1),'.');
% plot(vtlSolution.kferr.Y,'.');
% plot(vtlSolution.kfpvt.Y-navSolution.Y(1)+vtlSolution.kferr.Y,'.');
% ylabel('Y (m)')
% xlabel('time U.A.')
% ylim([-200 50])
% title('Subplot 1: Y ')
% legend ('raw RTKlib','raw kf','kferr','added err+raw','Location','eastoutside')
% 
% subplot(2,3,3);
% plot(navSolution.Z-navSolution.Z(1),'.');
% hold on;grid on
% plot(vtlSolution.kfpvt.Z-vtlSolution.kfpvt.Z(1),'.');
% plot(vtlSolution.kferr.Z,'.');
% plot(vtlSolution.kfpvt.Z-navSolution.Z(1)+vtlSolution.kferr.Z,'.');
% ylabel('Z (m)')
% xlabel('time U.A.')
% ylim([-350 50])
% title('Subplot 1: Z ')
% legend ('raw RTKlib','raw kf','kferr','added err+raw','Location','eastoutside')
% 
% subplot(2,3,4);
% plot(navSolution.vX,'.');
% hold on;grid on
% plot(vtlSolution.kfpvt.vX,'.');
% plot(vtlSolution.kferr.vX,'.');
% plot(vtlSolution.kfpvt.vX+vtlSolution.kferr.vX,'.');
% ylabel('vX (m/s)')
% xlabel('time U.A.')
% ylim([-5 5])
% title('Subplot 1: vX ')
% legend ('raw RTKlib','raw kf','kferr','added err+raw','Location','eastoutside')
% 
% subplot(2,3,5);
% plot(navSolution.vY,'.');
% hold on;grid on
% plot(vtlSolution.kfpvt.vY,'.');
% plot(vtlSolution.kferr.vY,'.');
% plot(vtlSolution.kfpvt.vY+vtlSolution.kferr.vY,'.');
% ylabel('vY (m/s)')
% xlabel('time U.A.')
% ylim([-5 5])
% title('Subplot 1: vY ')
% legend ('raw RTKlib','raw kf','kferr','added err+raw','Location','eastoutside')
% 
% subplot(2,3,6);
% plot(navSolution.vZ,'.');
% hold on;grid on
% plot(vtlSolution.kfpvt.vZ,'.');
% plot(vtlSolution.kferr.vZ,'.');
% plot(vtlSolution.kfpvt.vZ+vtlSolution.kferr.vZ,'.');
% ylabel('vZ (m/s)')
% xlabel('time U.A.')
% ylim([-5 5])
% title('Subplot 1: vZ ')
% legend ('raw RTKlib','raw kf','kferr','added err+raw','Location','eastoutside')
% 
% 
% sgtitle('VTL errPV correction') 