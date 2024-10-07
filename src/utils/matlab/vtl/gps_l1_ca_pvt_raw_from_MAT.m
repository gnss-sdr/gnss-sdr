
% Read PVG raw dump
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
clc
close all;
clear all;

% if ~exist('gps_l1_ca_read_pvt_raw_dump.m', 'file')
%     addpath('./libs')
% end
% 
% if ~exist('cat2geo.m', 'file')
%     addpath('./libs/geoFunctions')
% end

%%
samplingFreq=25000000;
channels=6;
TTFF_sec=41.48;
path='';
pvt_raw_log_path=[path 'PVT_raw.dat'];
% GNSS_PVT_raw= gps_l1_ca_read_pvt_raw_dump(channels,pvt_raw_log_path);
GnssSDR2struct('PVT_raw.mat')

%% ===== Import data from text file motion_V1.csv 2 ARRAY ============================
% Script for importing data from the following text file:
%
%    filename: D:\virtualBOX_VM\ubuntu20\ubuntu20\shareFolder\myWork\results\log_spirent\motion_V1.csv
%
% Auto-generated by MATLAB on 20-Nov-2022 12:31:17

% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 38);

% Specify range and delimiter
opts.DataLines = [3, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Time_ms", "Pos_X", "Pos_Y", "Pos_Z", "Vel_X", "Vel_Y", "Vel_Z", "Acc_X", "Acc_Y", "Acc_Z", "Jerk_X", "Jerk_Y", "Jerk_Z", "Lat", "Long", "Height", "Heading", "Elevation", "Bank", "Angvel_X", "Angvel_Y", "Angvel_Z", "Angacc_X", "Angacc_Y", "Angacc_Z", "Ant1_Pos_X", "Ant1_Pos_Y", "Ant1_Pos_Z", "Ant1_Vel_X", "Ant1_Vel_Y", "Ant1_Vel_Z", "Ant1_Acc_X", "Ant1_Acc_Y", "Ant1_Acc_Z", "Ant1_Lat", "Ant1_Long", "Ant1_Height", "Ant1_DOP"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
motionV1 = readtable("..\log_spirent\motion_V1_SPF_LD_05.csv", opts);

% Convert to output type
motionV1 = table2array(motionV1);

% Clear temporary variables
clear opts

%% ============================ PARSER TO STRUCT ============================
plot_skyplot=1;
%% GNSS SDR SOLUTION
% navSolution.samplingFreq=25000000;
% navSolution.channels=6;

navSolution.solution_status=solution_status;
navSolution.solution_type=solution_type;
navSolution.valid_sats=valid_sats;

navSolution.RX_time=RX_time;
navSolution.TOW_at_current_symbol_ms=TOW_at_current_symbol_ms;

navSolution.X=pos_x;
navSolution.Y=pos_y;
navSolution.Z=pos_z;

navSolution.latitude=latitude;
navSolution.longitude=longitude;
navSolution.height=height;

navSolution.pdop=pdop;
navSolution.gdop=gdop;
navSolution.hdop=hdop;

navSolution.vX=vel_x;
navSolution.vY=vel_y;
navSolution.vZ=vel_z;

navSolution.vdop=vdop;

navSolution.week=week;

%% SPIRENT REFERENCE SOLUTION

refSolution.SIM_time=motionV1(:,1);

refSolution.X=motionV1(:,2);
refSolution.Y=motionV1(:,3);
refSolution.Z=motionV1(:,4);

refSolution.vX=motionV1(:,5);
refSolution.vY=motionV1(:,6);
refSolution.vZ=motionV1(:,7);

refSolution.aX=motionV1(:,8);
refSolution.aY=motionV1(:,9);
refSolution.aZ=motionV1(:,10);

refSolution.jX=motionV1(:,11);
refSolution.jY=motionV1(:,12);
refSolution.jZ=motionV1(:,13);

refSolution.latitude=rad2deg(motionV1(:,14));
refSolution.longitude=rad2deg(motionV1(:,15));
refSolution.height=motionV1(:,16);

refSolution.dop=motionV1(:,38);

% Clear temporary variables
clear motionV1
%% === Convert to UTM coordinate system =============================

% Scenario latitude  is xx.xxxxxxx  N37 49 9.98
% Scenario longitude is -xx.xxxxxxx  W122 28 42.58
% Scenario elevation is 35 meters.
    % lat=[37 49 9.98];
    % long=[-122 -28 -42.58];
    % lat_deg=dms2deg(lat);
    % long_deg=dms2deg(long);
    % h=35;
    
    lat_deg=navSolution.latitude(1);
    lon_deg=navSolution.longitude(1);
    lat=degrees2dms(lat_deg);
    lon=degrees2dms(lon_deg);
    h=navSolution.height(1);


utmstruct = defaultm('utm');
utmstruct.zone =  utmzone(lat_deg, lon_deg);
utmstruct = defaultm(utmstruct);
[utmstruct.latlim,utmstruct.lonlim] = utmzone(utmstruct.zone );
%Choices i of Reference Ellipsoid
%   1. International Ellipsoid 1924
%   2. International Ellipsoid 1967
%   3. World Geodetic System 1972
%   4. Geodetic Reference System 1980
%   5. World Geodetic System 1984

utmstruct.geoid = wgs84Ellipsoid;

% [X, Y] = projfwd(utmstruct,lat_deg,lon_deg);
% Z=h; % geographical to cartesian conversion


% for k=1:1:length(navSolution.X)
%     [navSolution.E(k), ...
%         navSolution.N(k), ...
%         navSolution.U(k)]=cart2utm(navSolution.X(k), navSolution.Y(k), navSolution.Z(k), utmZone);
% end

%% ====== FILTERING =======================================================
moving_avg_factor= 500;
LAT_FILT = movmean(navSolution.latitude,moving_avg_factor);
LON_FILT = movmean(navSolution.longitude,moving_avg_factor);
HEIGH_FILT = movmean(navSolution.height,moving_avg_factor);

X_FILT = movmean(navSolution.X,moving_avg_factor);
Y_FILT = movmean(navSolution.Y,moving_avg_factor);
Z_FILT = movmean(navSolution.Z,moving_avg_factor);

vX_FILT = movmean(navSolution.vX,moving_avg_factor);
vY_FILT = movmean(navSolution.vY,moving_avg_factor);
vZ_FILT = movmean(navSolution.vZ,moving_avg_factor);

%% ====== GNSS-SDR Plot all figures =======================================================
close all
%--- LLH POSITION: GNSS SDR plot  on map --------------------------------------
LLH=figure('Name','LLH system  on map');
geoplot([navSolution.latitude],[navSolution.longitude],'.')
geobasemap satellite
hold on
geoplot([LAT_FILT],[LON_FILT],'r.')
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
title('Subplot 1: X GOR')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','east')

subplot(2,2,2);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.Y-refSolution.Y(1),'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),Y_FILT-refSolution.Y(1),'r.');
ylabel('Y (m)')
xlabel('time from First FIX in (seconds)')
title('Subplot 2: Y')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','east')

subplot(2,2,3);
plot(navSolution.RX_time-navSolution.RX_time(1),navSolution.Z-refSolution.Z(1),'.');
hold on;grid on
plot(navSolution.RX_time-navSolution.RX_time(1),Z_FILT-refSolution.Z(1),'r.');
ylabel('Z (m)')
xlabel('time from First FIX in (seconds)')
title('Subplot 3: Z')
legend ('raw',['moving avg:' num2str(moving_avg_factor)],'Location','east')

sgtitle('UTM COORD CENTERED IN 1^{ST} POSITION') 
%% ====== SPIRENT Plot all figures =======================================================

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

%% ==== ERRORS Plots  Navigation errors =======================================================

% figureNumber = 300;
% % The 300 is chosen for more convenient handling of the open
% % figure windows, when many figures are closed and reopened. Figures
% % drawn or opened by the user, will not be "overwritten" by this
% % function if the auto numbering is not used.
% 
% %=== Select (or create) and clear the figure ==========================
% figure(figureNumber);
% clf   (figureNumber);
% set   (figureNumber, 'Name', 'Navigation errors');
% 
% %--- Draw axes --------------------------------------------------------
% handles(1, 1) = subplot(4, 2, 1 : 4);
% handles(3, 1) = subplot(4, 2, [5, 7]);
% handles(3, 2) = subplot(4, 2, [6, 8]);
% 
% 
% % 
% idx_syn=max(find(TTFF_sec*1000>refSolution.SIM_time));
%     %--- VELOCITY differences -----------------------------
%     plot([(navSolution.vX - refSolution.vX(idx_syn:end))'; ...
%         (navSolution.vY - refSolution.vY)';...
%         (navSolution.vZ - refSolution.vZ)']);
% 
%     title (handles(1, 1), 'VELOCITY variations');
%     legend(handles(1, 1), 'vX', 'vY', 'vZ');
%     xlabel(handles(1, 1), ['Measurement period: ', ...
%         num2str(424242), 'ms']);
%     ylabel(handles(1, 1), 'Variations (m/s)');
%     grid  (handles(1, 1));
%     axis  (handles(1, 1), 'tight');

%%
% plotNavigation(navSolutions,settings,plot_skyplot);
% 
% %% Plot results in the necessary data exists ==============================
% if (~isempty(navSolutions))
% 
%     %% If reference position is not provided, then set reference position
%     %% to the average position
%     if isnan(settings.truePosition.E) || isnan(settings.truePosition.N) ...
%             || isnan(settings.truePosition.U)
% 
%         %=== Compute mean values ==========================================
%         % Remove NaN-s or the output of the function MEAN will be NaN.
%         refCoord.E = mean(navSolutions.E(~isnan(navSolutions.E)));
%         refCoord.N = mean(navSolutions.N(~isnan(navSolutions.N)));
%         refCoord.U = mean(navSolutions.U(~isnan(navSolutions.U)));
% 
%         %Also convert geodetic coordinates to deg:min:sec vector format
%         meanLongitude = dms2mat(deg2dms(...
%             mean(navSolutions.longitude(~isnan(navSolutions.longitude)))), -5);
%         meanLatitude  = dms2mat(deg2dms(...
%             mean(navSolutions.latitude(~isnan(navSolutions.latitude)))), -5);
% 
%         LatLong_str=[num2str(meanLatitude(1)), '??', ...
%             num2str(meanLatitude(2)), '''', ...
%             num2str(meanLatitude(3)), '''''', ...
%             ',', ...
%             num2str(meanLongitude(1)), '??', ...
%             num2str(meanLongitude(2)), '''', ...
%             num2str(meanLongitude(3)), '''''']
% 
% 
% 
%         refPointLgText = ['Mean Position\newline  Lat: ', ...
%             num2str(meanLatitude(1)), '{\circ}', ...
%             num2str(meanLatitude(2)), '{\prime}', ...
%             num2str(meanLatitude(3)), '{\prime}{\prime}', ...
%             '\newline Lng: ', ...
%             num2str(meanLongitude(1)), '{\circ}', ...
%             num2str(meanLongitude(2)), '{\prime}', ...
%             num2str(meanLongitude(3)), '{\prime}{\prime}', ...
%             '\newline Hgt: ', ...
%             num2str(mean(navSolutions.height(~isnan(navSolutions.height))), '%+6.1f')];
% 
%     else
%         % compute the mean error for static receiver
%         mean_position.E = mean(navSolutions.E(~isnan(navSolutions.E)));
%         mean_position.N = mean(navSolutions.N(~isnan(navSolutions.N)));
%         mean_position.U = mean(navSolutions.U(~isnan(navSolutions.U)));
%         refCoord.E = settings.truePosition.E;
%         refCoord.N = settings.truePosition.N;
%         refCoord.U = settings.truePosition.U;
% 
%         error_meters=sqrt((mean_position.E-refCoord.E)^2+(mean_position.N-refCoord.N)^2+(mean_position.U-refCoord.U)^2);
% 
%         refPointLgText = ['Reference Position, Mean 3D error = ' num2str(error_meters) ' [m]'];
%     end
% 
%     figureNumber = 300;
%     % The 300 is chosen for more convenient handling of the open
%     % figure windows, when many figures are closed and reopened. Figures
%     % drawn or opened by the user, will not be "overwritten" by this
%     % function if the auto numbering is not used.
% 
%     %=== Select (or create) and clear the figure ==========================
%     figure(figureNumber);
%     clf   (figureNumber);
%     set   (figureNumber, 'Name', 'Navigation solutions');
% 
%     %--- Draw axes --------------------------------------------------------
%     handles(1, 1) = subplot(4, 2, 1 : 4);
%     handles(3, 1) = subplot(4, 2, [5, 7]);
%     handles(3, 2) = subplot(4, 2, [6, 8]);

    %% Plot all figures =======================================================
% 
%     %--- Coordinate differences in UTM system -----------------------------
%     plot(handles(1, 1), [(navSolutions.E - refCoord.E)', ...
%         (navSolutions.N - refCoord.N)',...
%         (navSolutions.U - refCoord.U)']);
% 
%     title (handles(1, 1), 'Coordinates variations in UTM system');
%     legend(handles(1, 1), 'E', 'N', 'U');
%     xlabel(handles(1, 1), ['Measurement period: ', ...
%         num2str(settings.navSolPeriod), 'ms']);
%     ylabel(handles(1, 1), 'Variations (m)');
%     grid  (handles(1, 1));
%     axis  (handles(1, 1), 'tight');
% 
%     %--- Position plot in UTM system --------------------------------------
%     plot3 (handles(3, 1), navSolutions.E - refCoord.E, ...
%         navSolutions.N - refCoord.N, ...
%         navSolutions.U - refCoord.U, '+');
%     hold  (handles(3, 1), 'on');
% 
%     %Plot the reference point
%     plot3 (handles(3, 1), 0, 0, 0, 'r+', 'LineWidth', 1.5, 'MarkerSize', 10);
%     hold  (handles(3, 1), 'off');
% 
%     view  (handles(3, 1), 0, 90);
%     axis  (handles(3, 1), 'equal');
%     grid  (handles(3, 1), 'minor');
% 
%     legend(handles(3, 1), 'Measurements', refPointLgText);
% 
%     title (handles(3, 1), 'Positions in UTM system (3D plot)');
%     xlabel(handles(3, 1), 'East (m)');
%     ylabel(handles(3, 1), 'North (m)');
%     zlabel(handles(3, 1), 'Upping (m)');
% 
%     if (plot_skyplot==1)
%         %--- Satellite sky plot -----------------------------------------------
%         skyPlot(handles(3, 2), ...
%             navSolution.channel.az, ...
%             navSolution.channel.el, ...
%             navSolution.channel.PRN(:, 1));
% 
% %         title (handles(3, 2), ['Sky plot (mean PDOP: ', ...
% %             num2str(mean(navSolution.DOP(2,:))), ')']);
%     end
% 
% else
%     disp('plotNavigation: No navigation data to plot.');
% end % if (~isempty(navSolutions))
