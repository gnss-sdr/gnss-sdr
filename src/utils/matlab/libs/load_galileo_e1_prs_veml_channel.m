function GNSS_tracking = load_galileo_e1_prs_veml_channel( dumpDir, channel, count )

%
% Load a directory containing e1 prs de data:

fileName = [ dumpDir filesep 'prs_veml_tracking_ch_' num2str( channel ) '.dat' ];


if nargin < 3
    count = inf;
end

GNSS_tracking(1)= galileo_e1_prs_veml_read_tracking_dump(fileName, count);


