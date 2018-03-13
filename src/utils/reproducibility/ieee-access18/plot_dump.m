% /*!
%  * \file plot_dump.m
%  * \brief Read GNSS-SDR Tracking dump binary file and plot some internal
%     variables
%  * \author Antonio Ramos, 2018. antonio.ramos(at)cttc.es
%  * -------------------------------------------------------------------------
%  *
%  * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
%  *
%  * GNSS-SDR is a software defined Global Navigation
%  *          Satellite Systems receiver
%  *
%  * This file is part of GNSS-SDR.
%  *
%  * GNSS-SDR is free software: you can redistribute it and/or modify
%  * it under the terms of the GNU General Public License as published by
%  * the Free Software Foundation, either version 3 of the License, or
%  * at your option) any later version.
%  *
%  * GNSS-SDR is distributed in the hope that it will be useful,
%  * but WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  * GNU General Public License for more details.
%  *
%  * You should have received a copy of the GNU General Public License
%  * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
%  *
%  * -------------------------------------------------------------------------
%  */

clear all;
clc;

n_channel     = 0;
symbol_period = 20e-3;
filename      = 'track_ch_';

fontsize = 12;

addpath('./data') % Path to gnss-sdr dump files (Tracking and PVT)
addpath('./geoFunctions')

load([filename int2str(n_channel) '.mat']);
t = (0 : length(abs_P) - 1) * symbol_period;
hf = figure('visible', 'off');
set(hf, 'paperorientation', 'landscape');
subplot(3, 3, [1,3])
plot(t, abs_E, t, abs_P, t, abs_L)
xlabel('Time [s]','fontname','Times','fontsize', fontsize)
ylabel('Correlation result','fontname','Times','fontsize', fontsize)
legend('Early', 'Prompt', 'Late')
grid on


subplot(3, 3, 7)
plot(Prompt_I./1000, Prompt_Q./1000, 'linestyle', 'none', 'marker', '.')
xlabel('I','fontname','Times','fontsize', fontsize)
ylabel('Q','fontname','Times','fontsize', fontsize)
axis equal
grid on

subplot(3, 3, [4,6])
plot(t, Prompt_I)
xlabel('Time [s]','fontname','Times','fontsize', fontsize)
ylabel('Navigation data bits','fontname','Times','fontsize', fontsize)
grid on


fileID   = fopen('data/PVT_ls_pvt.dat', 'r');
dinfo    = dir('data/PVT_ls_pvt.dat');
filesize = dinfo.bytes;
aux = 1;
while ne(ftell(fileID), filesize)
    navsol.RX_time(aux)    = fread(fileID, 1, 'double');
    navsol.X(aux)          = fread(fileID, 1, 'double');
    navsol.Y(aux)          = fread(fileID, 1, 'double');
    navsol.Z(aux)          = fread(fileID, 1, 'double');
    navsol.user_clock(aux) = fread(fileID, 1, 'double');
    navsol.lat(aux)        = fread(fileID, 1, 'double');
    navsol.long(aux)       = fread(fileID, 1, 'double');
    navsol.height(aux)     = fread(fileID, 1, 'double');
    aux = aux + 1;
end
fclose(fileID);


mean_Latitude=mean(navsol.lat);
mean_Longitude=mean(navsol.long);
mean_h=mean(navsol.height);
utmZone = findUtmZone(mean_Latitude,mean_Longitude);
[ref_X_cart,ref_Y_cart,ref_Z_cart]=geo2cart(dms2mat(deg2dms(mean_Latitude)), dms2mat(deg2dms(mean_Longitude)), mean_h, 5);
[mean_utm_X,mean_utm_Y,mean_utm_Z]=cart2utm(ref_X_cart,ref_Y_cart,ref_Z_cart,utmZone);


numPoints=length(navsol.X);
aux=0;
for n=1:numPoints
    aux=aux+1;
    [E(aux),N(aux),U(aux)]=cart2utm(navsol.X(n), navsol.Y(n), navsol.Z(n), utmZone);
end

v_2d=[E;N].'; %2D East Nort position vectors
v_3d=[E;N;U].'; %2D East Nort position vectors


%% ACCURACY

% 2D -------------------

sigma_E_accuracy=sqrt((1/(numPoints-1))*sum((v_2d(:,1)-mean_utm_X).^2));
sigma_N_accuracy=sqrt((1/(numPoints-1))*sum((v_2d(:,2)-mean_utm_Y).^2));

sigma_ratio_2d_accuracy=sigma_N_accuracy/sigma_E_accuracy

% if sigma_ratio=1 -> Prob in circle with r=DRMS -> 65%
DRMS_accuracy=sqrt(sigma_E_accuracy^2+sigma_N_accuracy^2)
% if sigma_ratio=1 -> Prob in circle with r=2DRMS -> 95%
TWO_DRMS_accuracy=2*DRMS_accuracy
% if sigma_ratio>0.3 -> Prob in circle with r=CEP -> 50%
CEP_accuracy=0.62*sigma_E_accuracy+0.56*sigma_N_accuracy

% 3D -------------------

sigma_U_accuracy=sqrt((1/(numPoints-1))*sum((v_3d(:,3)-mean_utm_Z).^2));

% if sigma_ratio=1 -> Prob in circle with r=DRMS -> 50%
SEP_accuracy=0.51*sqrt(sigma_E_accuracy^2+sigma_N_accuracy^2+sigma_U_accuracy^2)

% if sigma_ratio=1 -> Prob in circle with r=DRMS -> 61%
MRSE_accuracy=sqrt(sigma_E_accuracy^2+sigma_N_accuracy^2+sigma_U_accuracy^2)
% if sigma_ratio=1 -> Prob in circle with r=2DRMS -> 95%
TWO_MRSE_accuracy=2*MRSE_accuracy



%% PRECISION

% 2D analysis
% Simulated X,Y measurements
%v1=randn(1000,2);

% 2D Mean and Variance
mean_2d  = [mean(v_2d(:,1)) ; mean(v_2d(:,2))];
sigma_2d = [sqrt(var(v_2d(:,1))) ; sqrt(var(v_2d(:,2)))];

sigma_ratio_2d=sigma_2d(2)/sigma_2d(1)

% if sigma_ratio=1 -> Prob in circle with r=DRMS -> 65%
DRMS=sqrt(sigma_2d(1)^2+sigma_2d(2)^2)
% if sigma_ratio=1 -> Prob in circle with r=2DRMS -> 95%
TWO_DRMS=2*DRMS
% if sigma_ratio>0.3 -> Prob in circle with r=CEP -> 50%
CEP=0.62*sigma_2d(1)+0.56*sigma_2d(2)


% Mean and Variance
mean_3d=[mean(v_3d(:,1)) ; mean(v_3d(:,2)) ; mean(v_3d(:,3))];
sigma_3d=[sqrt(var(v_3d(:,1))) ; sqrt(var(v_3d(:,2))) ; sqrt(var(v_3d(:,3)))];

% absolute mean error
% 2D

error_2D_vec=[mean_utm_X-mean_2d(1) mean_utm_Y-mean_2d(2)];
error_2D_m=norm(error_2D_vec)

error_3D_vec=[mean_utm_X-mean_3d(1) mean_utm_Y-mean_3d(2) mean_utm_Z-mean_3d(3)];
error_3D_m=norm(error_3D_vec)

% RMSE 2D

RMSE_X=sqrt(mean((v_3d(:,1)-mean_utm_X).^2))
RMSE_Y=sqrt(mean((v_3d(:,2)-mean_utm_Y).^2))
RMSE_Z=sqrt(mean((v_3d(:,3)-mean_utm_Z).^2))


RMSE_2D=sqrt(mean((v_2d(:,1)-mean_utm_X).^2+(v_2d(:,2)-mean_utm_Y).^2))

RMSE_3D=sqrt(mean((v_3d(:,1)-mean_utm_X).^2+(v_3d(:,2)-mean_utm_Y).^2+(v_3d(:,3)-mean_utm_Z).^2))

% SCATTER PLOT
subplot(3,3,8)
scatter(v_2d(:,1)-mean_2d(1),v_2d(:,2)-mean_2d(2));
hold on;

plot(0,0,'k*');


[x,y,z] = cylinder([TWO_DRMS TWO_DRMS],200);
plot(x(1,:),y(1,:),'Color',[0 0.6 0]);
str = strcat('2DRMS=',num2str(TWO_DRMS), ' m');
text(cosd(65)*TWO_DRMS,sind(65)*TWO_DRMS,str,'Color',[0 0.6 0]);


[x,y,z] = cylinder([CEP CEP],200);

plot(x(1,:),y(1,:),'r--');
str = strcat('CEP=',num2str(CEP), ' m');
text(cosd(80)*CEP,sind(80)*CEP,str,'Color','r');

grid on
axis equal;
xlabel('North [m]','fontname','Times','fontsize', fontsize)
ylabel('East [m]','fontname','Times','fontsize', fontsize)

% 3D analysis
% Simulated X,Y,Z measurements

% if sigma_ratio=1 -> Prob in circle with r=DRMS -> 50%
SEP=0.51*sqrt(sigma_3d(1)^2+sigma_3d(2)^2+sigma_3d(3)^2)

% if sigma_ratio=1 -> Prob in circle with r=DRMS -> 61%
MRSE=sqrt(sigma_3d(1)^2+sigma_3d(2)^2+sigma_3d(3)^2)
% if sigma_ratio=1 -> Prob in circle with r=2DRMS -> 95%
TWO_MRSE=2*MRSE



% SCATTER PLOT
subplot(3,3,9)
scatter3(v_3d(:,1)-mean_3d(1),v_3d(:,2)-mean_3d(2), v_3d(:,3)-mean_3d(3));

hold on;

[x,y,z] = sphere();
hSurface=surf(MRSE*x,MRSE*y,MRSE*z);  % sphere centered at origin

set(hSurface,'facecolor','none','edgecolor',[0 0.6 0],'edgealpha',1,'facealpha',1);

%axis equal;
xlabel('North [m]','fontname','Times','fontsize', fontsize-2)
ylabel('East [m]','fontname','Times','fontsize', fontsize-2)
zlabel('Up [m]','fontname','Times','fontsize', fontsize-2)
str = strcat('MRSE=',num2str(MRSE), ' m')
text(cosd(45)*MRSE,sind(45)*MRSE,20,str,'Color',[0 0.6 0]);
a=gca;
set(a,'fontsize',fontsize-6)

hh=findall(hf,'-property','FontName');
set(hh,'FontName','Times');
print(hf, 'Figure2.eps', '-depsc')
close(hf);
