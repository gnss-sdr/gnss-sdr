% -------------------------------------------------------------------------
%
% Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
%
% GNSS-SDR is a software defined Global Navigation
%           Satellite Systems receiver
%
% This file is part of GNSS-SDR.
% 
% GNSS-SDR is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% at your option) any later version.
% 
% GNSS-SDR is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
%
% -------------------------------------------------------------------------
%

%help script to compare GNSS-SDR Preambles starts
channel=3;
% From GNSS_SDR telemetry decoder

% 1 find preambles indexes
preambles_index=find(GNSS_telemetry(channel).Preamble_symbol_counter==0);

% 2 Get associated timestamp ms

preambles_timestamp_sdr_ms=GNSS_telemetry(channel).prn_delay_ms(preambles_index);


% From Matlab receiver

[firstSubFrame, activeChnList, javi_subFrameStart_sample] = findPreambles(trackResults_sdr,settings);

preambles_timestamp_matlab_ms=trackResults_sdr(channel).prn_delay_ms(javi_subFrameStart_sample(channel,1:6));


%Compare

common_start_index=max(find(abs(preambles_timestamp_sdr_ms-preambles_timestamp_matlab_ms(1))<2000));

error_ms=preambles_timestamp_sdr_ms(common_start_index:(common_start_index+length(preambles_timestamp_matlab_ms)-1))-preambles_timestamp_matlab_ms.'

% figure
% stem(tracking_loop_start+javi_subFrameStart_sample(channel,:),1000*trackResults_sdr(channel).absoluteSample(javi_subFrameStart_sample(channel,:))/settings.samplingFreq);
%
% hold on;
%
% plot(GNSS_observables.preamble_delay_ms(channel,:));
%
% plot(GNSS_observables.prn_delay_ms(channel,:),'r')
