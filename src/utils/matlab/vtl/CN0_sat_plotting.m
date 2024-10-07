% Miguel Angel Gomez, 2024. gomezlma(at)inta.es
% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------
%


time_reference_spirent_obs=129780;%s
time_vtl_dump_file=linspace(38,157,length(vtlSolution.CN0_sat));
%%
%--------------------------------------------------------
Rx_Dopp_30=figure('Name','RX_Carrier_Doppler_hz');
subplot(2,3,1);
xlim([0,160]);
xlabel('')
ylabel('CN0 sat (dB-Hz)')
xlabel('time from simulation init (seconds)')
grid on
hold on
title('PRN 30 GNSS-SDR')
plot(time_vtl_dump_file,vtlSolution.CN0_sat(1,:),'o','DisplayName','filtered VTL')
hold off;grid minor

%--------------------------------------------------------
subplot(2,3,2);
xlim([0,160]);
xlabel('')
ylabel('CN0 sat (dB-Hz)')
xlabel('time from simulation init (seconds)')
grid on
hold on
title('PRN 29 GNSS-SDR')
plot(time_vtl_dump_file,vtlSolution.CN0_sat(2,:),'o','DisplayName','filtered VTL')
hold off;grid minor
%--------------------------------------------------------
subplot(2,3,3);
xlim([0,160]);
xlabel('')
ylabel('CN0 sat (dB-Hz)')
xlabel('time from simulation init (seconds)')
grid on
hold on
% legend('PRN 17 GNSS-SDR','Location','eastoutside')
title('PRN 24 GNSS-SDR')
plot(time_vtl_dump_file,vtlSolution.CN0_sat(3,:),'o','DisplayName','filtered VTL')
hold off;grid minor
%--------------------------------------------------------
subplot(2,3,4);
xlim([0,160]);
xlabel('')
ylabel('CN0 sat (dB-Hz)')
xlabel('time from simulation init (seconds)')
grid on
hold on
title('PRN 12 GNSS-SDR')
plot(time_vtl_dump_file,vtlSolution.CN0_sat(4,:),'o','DisplayName','filtered VTL')
hold off;grid minor
%--------------------------------------------------------

subplot(2,3,5);
xlim([0,160]);
xlabel('')
ylabel('CN0 sat (dB-Hz)')
xlabel('time from simulation init (seconds)')
grid on
hold on
title('PRN 10 GNSS-SDR')
plot(time_vtl_dump_file,vtlSolution.CN0_sat(5,:),'o','DisplayName','filtered VTL')
hold off;grid minor
