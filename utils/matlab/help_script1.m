% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
% SPDX-License-Identifier: GPL-3.0-or-later
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
