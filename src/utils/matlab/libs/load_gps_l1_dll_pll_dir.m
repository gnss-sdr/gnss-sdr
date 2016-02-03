function GNSS_tracking = load_gps_l1_dll_pll_dir( dumpDir, count )

%
% Load a directory containing gps l1 dll pll data:

filePattern = 'epl_tracking_ch*.dat';

allFiles = dir( [ dumpDir filesep filePattern ] );

channels = size( allFiles, 1 );

if nargin < 2
    count = inf;
end

for N=1:1:channels
    GNSS_tracking(N)= gps_l1_ca_dll_pll_read_tracking_dump([ dumpDir filesep allFiles( N).name], count);
end


