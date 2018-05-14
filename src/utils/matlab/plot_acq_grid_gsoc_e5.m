% Reads GNSS-SDR Acquisition dump binary file using the provided
% function and plot acquisition grid of acquisition statistic of PRN sat.
% CAF input must be 0 or 1 depending if the user desires to read the file
% that resolves doppler ambiguity or not.
%
% This function analyzes a experiment performed by Marc Sales in the framework
% of the Google Summer of Code (GSoC) 2014, with the collaboration of Luis Esteve, Javier Arribas
% and Carles Fernandez, related to the extension of GNSS-SDR to Galileo.
%
% Marc Sales marcsales92(at)gmail.com, 
% Luis Esteve, 2014. luis(at)epsilon-formacion.com
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
% along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
%
% -------------------------------------------------------------------------
%

function plot_acq_grid_gsoc_e5(sat,CAF)

path='/home/marc/git/gnss-sdr/data/';
file=[path 'test_statistics_E5a_sat_' num2str(sat) '_doppler_0.dat'];

sampling_freq_Hz=32E6
%Doppler_max_Hz = 14875
%Doppler_min_Hz = -15000
%Doppler_step_Hz = 125
Doppler_max_Hz = 10000
Doppler_min_Hz = -10000
Doppler_step_Hz = 250



% read files

%x=read_complex_binary (file);
%x=load_complex_data(file); % complex
%l_y=length(x);
myFile = java.io.File(file);
flen = length(myFile);
l_y=flen/4;% float


Doppler_axes=Doppler_min_Hz:Doppler_step_Hz:Doppler_max_Hz;

l_x=length(Doppler_axes);

acq_grid = zeros(l_x,l_y);

index=0;

for k=Doppler_min_Hz:Doppler_step_Hz:Doppler_max_Hz
    index=index+1;
    filename=[path 'test_statistics_E5a_sat_' num2str(sat) '_doppler_' num2str(k) '.dat'];
    fid=fopen(filename,'r');
    xx=fread(fid,'float');%floats from squared correlation
    %xx=load_complex_data (filename); %complex
    acq_grid(index,:)=abs(xx);
end

[fila,col]=find(acq_grid==max(max(acq_grid)));

if (CAF > 0)
    filename=[path 'test_statistics_E5a_sat_' num2str(sat) '_CAF.dat'];
    fid=fopen(filename,'r');
    xx=fread(fid,'float');%floats from squared correlation
    acq_grid(:,col(1))=abs(xx);
    Doppler_error_Hz = Doppler_axes(xx==max(xx))
    maximum_correlation_peak = max(xx)
else
    Doppler_error_Hz = Doppler_axes(fila)
    maximum_correlation_peak = max(max(acq_grid))
end

delay_error_sps = col -1



noise_grid=acq_grid;
delay_span=floor(3*sampling_freq_Hz/(1.023e7));
Doppler_span=floor(500/Doppler_step_Hz);
noise_grid(fila-Doppler_span:fila+Doppler_span,col-delay_span:col+delay_span)=0;

n=numel(noise_grid)-(2*delay_span+1)*(2*Doppler_span+1);

noise_floor= sum(sum(noise_grid))/n

Gain_dbs = 10*log10(maximum_correlation_peak/noise_floor)


%% Plot 3D FULL RESOLUTION


[X,Y] = meshgrid(Doppler_axes,1:1:l_y);
figure;
surf(X,Y,acq_grid');

xlabel('Doppler(Hz)');ylabel('Code Delay(samples)');title(['GLRT statistic of Galileo Parallel Code Phase Search Acquisition. PRN ' num2str(sat)]);


end

function x=load_complex_data(file)
fid = fopen(file,'r');
%fid = fopen('signal_source.dat','r');

myFile = java.io.File(file);
flen = length(myFile);
num_samples=flen/8; % 8 bytes (2 single floats) per complex sample

for k=1:num_samples
    a(1:2) = fread(fid, 2, 'float');
    x(k) = a(1) + a(2)*1i;
    k=k+1;
end

end

