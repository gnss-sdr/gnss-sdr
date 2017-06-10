% Read PVG raw dump

%clear all;

samplingFreq       = 64e6/16;     %[Hz]
channels=4;
path='/home/javier/workspace/gnss-sdr-ref/trunk/install/';
pvt_raw_log_path=[path 'PVT_raw.dat'];
GNSS_PVT_raw= gps_l1_ca_read_pvt_raw_dump(channels,pvt_raw_log_path);   
