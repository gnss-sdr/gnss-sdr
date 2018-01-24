% /*!
%  * \file plot_acq_grid.m
%  * \brief Read GNSS-SDR Acquisition dump .mat file using the provided
%  function and plot acquisition grid of acquisition statistic of PRN sat
%
%  
%  * \author Antonio Ramos, 2017. antonio.ramos(at)cttc.es
%  * -------------------------------------------------------------------------
%  *
%  * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
%  *
%  * GNSS-SDR is a software defined Global Navigation
%  *          Satellite Systems receiver
%  *
%  * This file is part of GNSS-SDR.
%  *
%  * GNSS-SDR is free software: you can redistribute it and/or modify
%  * it under the terms of the GNU General Public License as published by
%  * the Free Software Foundation, either version 3 of the License, or
%  * at your option) any later version.
%  *
%  * GNSS-SDR is distributed in the hope that it will be useful,
%  * but WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  * GNU General Public License for more details.
%  *
%  * You should have received a copy of the GNU General Public License
%  * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
%  *
%  * -------------------------------------------------------------------------
%  */ 

%%%%%%%%% ¡¡¡ CONFIGURE !!! %%%%%%%%%%%%% 

path = '/home/aramos/signals/GNSS-IN-THE-SPACE/CAPTURES SPIRENT/acq/';
file = 'acq';

sat = 32;

% Signal:
%     1 GPS  L1
%     2 GPS  L2M
%     3 GPS  L5
%     4 Gal. E1B
%     5 Gal. E5

signal_type = 1;

%%% True for light grid representation
lite_view = true;

%%% If lite_view, it sets the number of samples per chip in the graphical representation
n_samples_per_chip = 4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

switch(signal_type)
    case 1
        n_chips = 1023;
        system = 'G';
    case 2
        n_chips = 10230;
        system = 'G';
    case 3
        n_chips = 10230;
        system = 'G';
    case 4
        n_chips = 4092;
        system = 'E';
    case 5
        n_chips = 10230;
        system = 'E';
end
filename = [path file '_' system '_sat_' num2str(sat) '.mat'];
load(filename);
[n_fft n_dop_bins] = size(grid);
[d_max f_max] = find(grid == max(max(grid)));
freq = (0 : n_dop_bins - 1) * doppler_step - doppler_max;
delay = (0 : n_fft - 1) / n_fft * n_chips;
figure(1)
if(lite_view == false)
    surf(freq, delay, grid)
    ylim([min(delay) max(delay)])
else
    delay_interp = (0 : n_samples_per_chip * n_chips - 1) / n_samples_per_chip;
    grid_interp = spline(delay, grid', delay_interp)';
    surf(freq, delay_interp, grid_interp)
    ylim([min(delay_interp) max(delay_interp)])
end
xlabel('Doppler shift / Hz')
xlim([min(freq) max(freq)])
ylabel('Code delay / chips')
zlabel('Test statistics')
figure(2)
subplot(2,1,1)
plot(freq, grid(d_max, :))
xlim([min(freq) max(freq)])
xlabel('Doppler shift / Hz')
ylabel('Test statistics')
title(['Fixed code delay to ' num2str((d_max - 1) / n_fft * n_chips) ' chips'])
subplot(2,1,2)
plot(delay, grid(:, f_max))
xlim([min(delay) max(delay)])
xlabel('Code delay / chips')
ylabel('Test statistics')
title(['Doppler wipe-off = ' num2str((f_max - 1) * doppler_step - doppler_max) ' Hz'])
