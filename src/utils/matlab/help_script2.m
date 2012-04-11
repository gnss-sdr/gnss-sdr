% compare pseudoranges 

close all;
% GNSS SDR
plot(GNSS_PVT_raw.tx_time(1,1:300).'-200/settings.samplingFreq,GNSS_PVT_raw.Pseudorange_m(1,1:300).')

% MATLAB
hold on;
plot(navSolutions.transmitTime,navSolutions.channel.rawP(1,:),'g')
