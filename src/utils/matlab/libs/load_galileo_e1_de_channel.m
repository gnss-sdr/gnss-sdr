function GNSS_tracking = load_galileo_e1_de_channel( dumpDir, channel, count )

%
% Load a directory containing e1 de data:

fileName = [ dumpDir filesep 'de_tracking_ch_' num2str( channel ) '.dat' ];


if nargin < 3
    count = inf;
end

GNSS_tracking(1)= galileo_e1_de_read_tracking_dump(fileName, count);


