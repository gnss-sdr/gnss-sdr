% Read observables dump

clear all;
close all;

%IFEN NSR Sampler Fs=20480000
% GNSS-SDR decimation factor 8
samplingFreq       = 20480000/8;     %[Hz]
channels=8;
path='/home/gnss/workspace/gnss-sdr/trunk/install/';
observables_log_path=[path 'observables.dat'];
GNSS_observables= gps_l1_ca_read_observables_dump(channels,observables_log_path);   


skip=10000;
ref_channel=1;
plot(GNSS_observables.d_TOW_at_current_symbol(ref_channel,skip:end),GNSS_observables.Pseudorange_m(1:6,skip:end).')
