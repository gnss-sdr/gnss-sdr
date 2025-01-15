% plot tracking quality indicators

% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Javier Arribas <jarribas@cttc.es>
% SPDX-License-Identifier: GPL-3.0-or-later

figure;
hold on;
title('Carrier lock test output for all the channels');
for n=1:1:length(GNSS_tracking)
    plot(GNSS_tracking(n).carrier_lock_test)
    plotnames{n}=['SV ' num2str(round(mean(GNSS_tracking(n).PRN)))];
end
legend(plotnames);

figure;
hold on;
title('Carrier CN0 output for all the channels');
for n=1:1:length(GNSS_tracking)
    plot(GNSS_tracking(n).CN0_SNV_dB_Hz)
    plotnames{n}=['SV ' num2str(round(mean(GNSS_tracking(n).PRN)))];
end
legend(plotnames);
