function GNSS_tracking = load_gps_l1_dll_fll_pll_dir( dumpDir, fs, count )

%
% Load a directory containing gps l1 dll fll pll data:

filePattern = 'epl_fll_tracking_ch*.dat';

allFiles = dir( [ dumpDir filesep filePattern ] );

channels = size( allFiles, 1 );

if nargin < 3
    count = inf;
end

for N=1:1:channels
    GNSS_tracking(N)= gps_l1_ca_dll_fll_pll_read_tracking_dump([ dumpDir filesep allFiles( N,: ).name ], fs, count);
end

