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
%general_raw_plot.m

% navSolution.X(navSolution.X==0) = [];
%% ====== GNSS-SDR Plot all figures =======================================================
close all
%--- LLH POSITION: GNSS SDR plot  on map --------------------------------------
LLH=figure('Name','LLH system  on map');
geoplot([navSolution.latitude],[navSolution.longitude],'.')
geobasemap satellite
hold on
geoplot(LAT_FILT,LON_FILT,'r.')
title('Position in LLH on map ')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','southeast')
% geobasemap streets
% geobasemap topographic
% geobasemap streets-dark

%---VELOCITY: GNSS SDR plot --------------------------------------
VEL=figure('Name','velocities and heigh');
subplot(2,2,1);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.vX,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),vX_FILT,'r.');
ylabel('vX (m/s)')
xlabel('time from First FIX in (seconds)')
title('Subplot 1: vX GOR')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','southeast')

subplot(2,2,2);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.vY,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),vY_FILT,'r.');
ylabel('vY (m/s)')
xlabel('time from First FIX in (seconds)')
title('Subplot 2: vY')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','southeast')

subplot(2,2,3);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.vZ,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),vZ_FILT,'r.');
ylabel('vZ (m/s)')
xlabel('time from First FIX in (seconds)')
title('Subplot 3: vZ')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','southeast')

subplot(2,2,4);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.height,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),HEIGH_FILT,'r.');
ylabel('HEIGH (m)')
xlabel('time from First FIX in (seconds)')
title('Subplot 4: HEIGH')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','southeast')

sgtitle('velocities and heigh') 

% --- UTM centered POSITION: GNSS SDR  plot --------------------------------------

POS=figure('Name','UTM COORD CENTERED IN 1^{ST} POSITION');
subplot(2,2,1);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.X-refSolution.X(1),'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),X_FILT-refSolution.X(1),'r.');
ylabel('X (m)')
xlabel('time from First FIX in (seconds)')
% ylim([-55 100])
title('Subplot 1: X GOR')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','east')

subplot(2,2,2);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.Y-refSolution.Y(1),'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),Y_FILT-refSolution.Y(1),'r.');
ylabel('Y (m)')
xlabel('time from First FIX in (seconds)')
% ylim([-140 -20])
title('Subplot 2: Y')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','east')

subplot(2,2,3);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.Z-refSolution.Z(1),'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),Z_FILT-refSolution.Z(1),'r.');
ylabel('Z (m)')
xlabel('time from First FIX in (seconds)')
% ylim([-150 20])
title('Subplot 3: Z')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','east')

sgtitle('UTM COORD CENTERED IN 1^{ST} POSITION') 

% --- UTM full POSITION: GNSS SDR  plot --------------------------------------

POS_utm=figure('Name','UTM COORD IN 1^{ST} POSITION');
subplot(2,2,1);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.X,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),X_FILT,'r.');
ylabel('X (m)')
xlabel('time from First FIX in (seconds)')
% ylim([-55 100])
title('Subplot 1: X GOR')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','east')

subplot(2,2,2);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.Y,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),Y_FILT,'r.');
ylabel('Y (m)')
xlabel('time from First FIX in (seconds)')
% ylim([-140 -20])
title('Subplot 2: Y')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','east')

subplot(2,2,3);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.Z,'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),Z_FILT,'r.');
ylabel('Z (m)')
xlabel('time from First FIX in (seconds)')
% ylim([-150 20])
title('Subplot 3: Z')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','east')

sgtitle('UTM COORD FULL IN 1^{ST} POSITION') 

%% ====== SPIRENT Plot all figures =======================================================
if(plot_reference)
    %--- LLH POSITION: SPIRENT plot  on map --------------------------------------
    figure(LLH)
    geoplot([refSolution.latitude],[refSolution.longitude],'.','DisplayName','reference')
    geobasemap satellite
    hold off
    % geobasemap streets
    % geobasemap topographic
    % geobasemap streets-dark

    %---VELOCITY: SPIRENT plot --------------------------------------
    figure(VEL)
    subplot(2,2,1);
    plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.vX,'.','DisplayName','reference');
    hold off;grid on

    subplot(2,2,2);
    plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.vY,'.','DisplayName','reference');
    hold on;grid on

    subplot(2,2,3);
    plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.vZ,'.','DisplayName','reference');
    hold on;grid on

    subplot(2,2,4);
    plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.height,'.','DisplayName','reference');
    hold on;grid on

    %---UTM centered POSITION: SPIRENT plot --------------------------------------
    figure(POS)
    subplot(2,2,1);
    plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.X-refSolution.X(1),'.','DisplayName','reference');
    hold off;grid on

    subplot(2,2,2);
    plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.Y-refSolution.Y(1),'.','DisplayName','reference');
    hold on;grid on

    subplot(2,2,3);
    plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.Z-refSolution.Z(1),'.','DisplayName','reference');
    hold on;grid on
    
     %---UTM POSITION: SPIRENT plot --------------------------------------
    figure(POS_utm)
    subplot(2,2,1);
    plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.X,'.','DisplayName','reference');
    hold off;grid on

    subplot(2,2,2);
    plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.Y,'.','DisplayName','reference');
    hold on;grid on

    subplot(2,2,3);
    plot(refSolution.SIM_time/1000-TTFF_sec,refSolution.Z,'.','DisplayName','reference');
    hold on;grid on
end
