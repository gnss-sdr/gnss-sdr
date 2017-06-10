% Read observables dump

%clear all;

samplingFreq       = 64e6/16;     %[Hz]
channels=4;
path='/home/gnss/workspace/gnss-sdr/trunk/install/';
observables_log_path=[path 'observables.dat'];
GNSS_observables= gps_l1_ca_read_observables_dump(channels,observables_log_path);   
