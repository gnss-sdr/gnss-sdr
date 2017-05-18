function GNSS_tracking = load_gps_mcode_codeless_channel( dumpDir, channel, count )

%
% Load a directory containing l1 mcode  data:

fileName = [ dumpDir filesep 'mcode_codeless_tracking_ch_' num2str( channel ) '.dat' ];


if nargin < 3
    count = inf;
end

GNSS_tracking(1)= gps_mcode_codeless_read_tracking_dump(fileName, count);




