% Reads GNSS-SDR Acquisition dump .mat file using the provided
%  function and plots acquisition grid of acquisition statistic of PRN sat
% Antonio Ramos, 2017. antonio.ramos(at)cttc.es
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

%% Configuration
path = '/home/dmiralles/Documents/gnss-sdr/';
file = 'bds_acq';
sat = 6;
channel = 0;
execution = 4;
% Signal:
%     1 GPS  L1
%     2 GPS  L2M
%     3 GPS  L5
%     4 Gal. E1B
%     5 Gal. E5
%     6 Glo. 1G
%     7 Glo. 2G
%     8 BDS. B1
%     9 BDS. B3
%    10 BDS. B2a

signal_type = 8;

%%% True for light grid representation
lite_view = true;

%%% If lite_view, it sets the number of samples per chip in the graphical representation
n_samples_per_chip = 3;
d_samples_per_code = 25000;

%% Load data

switch(signal_type)
    case 1
        n_chips = 1023;
        system = 'G';
        signal = '1C';
    case 2
        n_chips = 10230;
        system = 'G';
        signal = '2S';
    case 3
        n_chips = 10230;
        system = 'G';
        signal = 'L5';
    case 4
        n_chips = 4092;
        system = 'E';
        signal = '1B';
    case 5
        n_chips = 10230;
        system = 'E';
        signal = '5X';
    case 6
        n_chips = 511;
        system = 'R';
        signal = '1G';
    case 7
        n_chips = 511;
        system = 'R';
        signal = '2G';
    case 8
        n_chips = 2048;
        system = 'C';
        signal = 'B1';
    case 9
        n_chips = 10230;
        system = 'C';
        signal = 'B3';
    case 10
        n_chips = 10230;
        system = 'C';
        signal = '5C';
end
filename = [path file '_' system '_' signal '_ch_' num2str(channel) '_' num2str(execution) '_sat_' num2str(sat) '.mat'];
load(filename);
[n_fft, n_dop_bins] = size(acq_grid);
[d_max, f_max] = find(acq_grid == max(max(acq_grid)));
freq = (0 : n_dop_bins - 1) * double(doppler_step) - double(doppler_max);
delay = (0 : n_fft - 1) / n_fft * n_chips;


%% Plot data
%--- Acquisition grid (3D)
figure(1)
if(lite_view == false)
    surf(freq, delay, acq_grid, 'FaceColor', 'interp', 'LineStyle', 'none')
    ylim([min(delay) max(delay)])
else
    delay_interp = (0 : n_samples_per_chip * n_chips - 1) / n_samples_per_chip;
    grid_interp = spline(delay, acq_grid', delay_interp)';
    surf(freq, delay_interp, grid_interp, 'FaceColor', 'interp', 'LineStyle', 'none')
    ylim([min(delay_interp) max(delay_interp)])
end
xlabel('Doppler shift (Hz)')
xlim([min(freq) max(freq)])
ylabel('Code delay (chips)')
zlabel('Test Statistics')

%--- Acquisition grid (2D)
figure(2)
subplot(2,1,1)
plot(freq, acq_grid(d_max, :))
xlim([min(freq) max(freq)])
xlabel('Doppler shift (Hz)')
ylabel('Test statistics')
title(['Fixed code delay to ' num2str((d_max - 1) / n_fft * n_chips) ' chips'])
subplot(2,1,2)
normalization = (d_samples_per_code^4) * input_power;
plot(delay, acq_grid(:, f_max)./normalization)
xlim([min(delay) max(delay)])
xlabel('Code delay (chips)')
ylabel('Test statistics')
title(['Doppler wipe-off = ' num2str((f_max - 1) * doppler_step - doppler_max) ' Hz'])
