% Reads GNSS-SDR Tracking dump binary file using the provided
%  function and plots some internal variables
% Javier Arribas, 2011. jarribas(at)cttc.es
% -------------------------------------------------------------------------
%
% Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
%
% GNSS-SDR is a software defined Global Navigation
%           Satellite Systems receiver
%
% This file is part of GNSS-SDR.
% 
% GNSS-SDR is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% at your option) any later version.
% 
% GNSS-SDR is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
%
% -------------------------------------------------------------------------
%

close all;clear;
samplingFreq       = 10000000;     %[Hz]
channels=[0:17];
path='/home/dmiralles/Documents/gnss-sdr/build/';
addpath('libs/');
clear PRN_absolute_sample_start;
for N=1:1:length(channels)
    telemetry_log_path=[path 'telemetry' num2str(channels(N)) '.dat'];
    GNSS_telemetry(N)= gps_l1_ca_read_telemetry_dump(telemetry_log_path);
end

%% Plotting values
%--- Configurations
chn_num_a = 11;
chn_num_b = 3;
%--- Plot results
figure;
plot(GNSS_telemetry(chn_num_a).tracking_sample_counter, ...
     GNSS_telemetry(chn_num_a).tow_current_symbol_ms/1000, 'b+');
hold on;
grid on;
plot(GNSS_telemetry(chn_num_b).tracking_sample_counter, ...
     GNSS_telemetry(chn_num_b).tow_current_symbol_ms, 'ro');
xlabel('TRK Sampling Counter');
ylabel('Current Symbol TOW');
legend(['CHN-',num2str(chn_num_a-1)], ['CHN-',num2str(chn_num_b-1)]);

figure;
plot(GNSS_telemetry(chn_num_a).tracking_sample_counter, ...
     GNSS_telemetry(chn_num_a).tow, 'b+');
hold on;
grid on;
plot(GNSS_telemetry(chn_num_b).tracking_sample_counter, ...
     GNSS_telemetry(chn_num_b).tow, 'ro');
xlabel('TRK Sampling Counter');
ylabel('Decoded Nav TOW');
legend(['CHN-',num2str(chn_num_a-1)], ['CHN-',num2str(chn_num_b-1)]);
