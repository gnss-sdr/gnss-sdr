function GNSS_tracking = load_galileo_e1_prs_codeless_dir( dumpDir, count )

%
% Load a directory containing e1 prs codeless data:

filePattern = 'prs_codeless_tracking_ch_*.dat';

allFiles = dir( [ dumpDir filesep filePattern ] );

channels = size( allFiles, 1 );

if nargin < 2
    count = inf;
end

for N=1:1:channels
    GNSS_tracking(N)= galileo_e1_prs_codeless_read_tracking_dump([ dumpDir filesep allFiles( N,: ).name ], count);
end

