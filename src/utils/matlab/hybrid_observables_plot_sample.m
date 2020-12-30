% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------
%

%% Read Hibrid Observables Dump
clearvars;
close all;
addpath('./libs');
samplingFreq       = 25000000;     %[Hz]
channels=10;
path='/home/dmiralles/Documents/gnss-sdr/';
observables_log_path=[path 'observables.dat'];
GNSS_observables= read_hybrid_observables_dump(channels,observables_log_path);

%% Plo data
%--- optional: search all channels having good satellite simultaneously
min_tow_idx=1;
obs_idx=1;
for n=1:1:channels
    idx=find(GNSS_observables.valid(n,:)>0,1,'first');
    if min_tow_idx<idx
        min_tow_idx=idx;
        obs_idx = n;
    end
end

%--- plot observables from that index
figure;
plot(GNSS_observables.RX_time(obs_idx,min_tow_idx+1:end),GNSS_observables.Pseudorange_m(:,min_tow_idx+1:end)');
grid on;
xlabel('TOW [s]')
ylabel('Pseudorange [m]');

figure;
plot(GNSS_observables.RX_time(obs_idx,min_tow_idx+1:end),GNSS_observables.Carrier_phase_hz(:,min_tow_idx+1:end)');
xlabel('TOW [s]')
ylabel('Accumulated Carrier Phase [cycles]');
grid on;

figure;
plot(GNSS_observables.RX_time(obs_idx,min_tow_idx+1:end),GNSS_observables.Carrier_Doppler_hz(:,min_tow_idx+1:end)');
xlabel('TOW [s]');
ylabel('Doppler Frequency [Hz]');
grid on;

%% Deprecated Code
% %read true obs from simulator (optional)
% GPS_STARTOFFSET_s = 68.802e-3;
%
% true_observables_log_path='/home/javier/git/gnss-sdr/build/obs_out.bin';
% GNSS_true_observables= read_true_sim_observables_dump(true_observables_log_path);
%
% %correct the clock error using true values (it is not possible for a receiver to correct
% %the receiver clock offset error at the observables level because it is required the
% %decoding of the ephemeris data and solve the PVT equations)
%
% SPEED_OF_LIGHT_M_S = 299792458.0;
%
% %find the reference satellite
% [~,ref_sat_ch]=min(GNSS_observables.Pseudorange_m(:,min_idx+1));
% shift_time_s=GNSS_true_observables.Pseudorange_m(ref_sat_ch,:)/SPEED_OF_LIGHT_M_S-GPS_STARTOFFSET_s;
%
%
% %Compute deltas if required and interpolate to measurement time
% delta_true_psudorange_m=GNSS_true_observables.Pseudorange_m(1,:)-GNSS_true_observables.Pseudorange_m(2,:);
% delta_true_interp_psudorange_m=interp1(GNSS_true_observables.RX_time(1,:)-shift_time_s, ...
%     delta_true_psudorange_m,GNSS_observables.RX_time(1,min_idx+1:end),'lineal','extrap');
% true_interp_acc_carrier_phase_ch1_hz=interp1(GNSS_true_observables.RX_time(1,:)-shift_time_s, ...
%     GNSS_true_observables.Carrier_phase_hz(1,:),GNSS_observables.RX_time(1,min_idx+1:end),'lineal','extrap');
% true_interp_acc_carrier_phase_ch2_hz=interp1(GNSS_true_observables.RX_time(1,:)-shift_time_s, ...
%     GNSS_true_observables.Carrier_phase_hz(2,:),GNSS_observables.RX_time(2,min_idx+1:end),'lineal','extrap');
%
% %Compute measurement errors
%
% delta_measured_psudorange_m=GNSS_observables.Pseudorange_m(1,min_idx+1:end)-GNSS_observables.Pseudorange_m(2,min_idx+1:end);
% psudorange_error_m=delta_measured_psudorange_m-delta_true_interp_psudorange_m;
% psudorange_rms_m=sqrt(sum(psudorange_error_m.^2)/length(psudorange_error_m))
%
% acc_carrier_error_ch1_hz=GNSS_observables.Carrier_phase_hz(1,min_idx+1:end)-true_interp_acc_carrier_phase_ch1_hz...
%     -GNSS_observables.Carrier_phase_hz(1,min_idx+1)+true_interp_acc_carrier_phase_ch1_hz(1);
%
% acc_phase_rms_ch1_hz=sqrt(sum(acc_carrier_error_ch1_hz.^2)/length(acc_carrier_error_ch1_hz))
%
% acc_carrier_error_ch2_hz=GNSS_observables.Carrier_phase_hz(2,min_idx+1:end)-true_interp_acc_carrier_phase_ch2_hz...
%     -GNSS_observables.Carrier_phase_hz(2,min_idx+1)+true_interp_acc_carrier_phase_ch2_hz(1);
% acc_phase_rms_ch2_hz=sqrt(sum(acc_carrier_error_ch2_hz.^2)/length(acc_carrier_error_ch2_hz))
%
%
% %plot results
% figure;
% plot(GNSS_true_observables.RX_time(1,:),delta_true_psudorange_m,'g');
% hold on;
% plot(GNSS_observables.RX_time(1,min_idx+1:end),delta_measured_psudorange_m,'b');
% title('TRUE vs. measured Pseudoranges [m]')
% xlabel('TOW [s]')
% ylabel('[m]');
%
% figure;
% plot(GNSS_observables.RX_time(1,min_idx+1:end),psudorange_error_m)
% title('Pseudoranges error [m]')
% xlabel('TOW [s]')
% ylabel('[m]');
%
% figure;
% plot(GNSS_observables.RX_time(1,min_idx+1:end),acc_carrier_error_ch1_hz)
% title('Accumulated carrier phase error CH1 [hz]')
% xlabel('TOW [s]')
% ylabel('[hz]');
%
% figure;
% plot(GNSS_observables.RX_time(1,min_idx+1:end),acc_carrier_error_ch2_hz)
% title('Accumulated carrier phase error CH2 [hz]')
% xlabel('TOW [s]')
% ylabel('[hz]');
%
%
%
%
