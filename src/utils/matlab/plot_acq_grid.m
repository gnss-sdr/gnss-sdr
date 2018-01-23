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
sat = 27;
n_chips = 1023;
system = 'G'; % GPS = 'G', Galileo = 'E' 
%%% True for light grid representation
lite_view = true;
%%% If lite_view, it sets the number of samples per chip in the graphical representation
n_samples_per_chip = 4;


path='/home/aramos/signals/';
file=['acq_' system '_sat_' num2str(sat) '.mat'];

load([path file]);
[n_fft n_dop_bins] = size(grid);
freq = (0 : n_dop_bins - 1) * doppler_step - doppler_max;
delay = (0 : n_fft - 1) / n_fft * n_chips;
figure(1)
if(lite_view == false)
    mesh(freq, delay, grid)
else
    delay_interp = (0 : n_samples_per_chip * n_chips - 1) / n_samples_per_chip;
    grid_interp = spline(delay, grid', delay_interp)';
    mesh(freq, delay_interp, grid_interp)
end
xlabel('Doppler shift / Hz')
ylabel('Code delay / chips')
zlabel('Test statistics')


