function obs_results(obs)
% Reads GNSS-SDR Observables dump mat files and displays the corresponding
% results. Based on plotVEMLTraking.m written by Darius Plausinaitis.
%
% Inputs from .mat file: 
%   Carrier_Doppler_hz: Doppler estimation in each channel, in [Hz].
%   Carrier_phase_cycles: Carrier phase estimation in each channel, in [cycles].
%   Flag_valid_pseudorange: Pseudorange computation status in each channel.
%   PRN: Satellite ID processed in each channel.
%   Pseudorange_m: Pseudorange computation in each channel, in [m].
%   RX_time: Receiving time in each channel, in seconds after the start of the week.
%   TOW_at_current_symbol_s: Time of week of the current symbol, in [s].
%
% Juan Alfaro, 2024
% -------------------------------------------------------------------------
%

load(obs.file.file)
% Figure with the results (subplot)
figure('Position', [150, 100, 1300, 600]);

% Primer subplot
subplot(1, 2, 1);
plot(RX_time,Pseudorange_m');
grid on;
xlabel('TOW [s]')
ylabel('Pseudorange [m]');

% Segundo subplot
subplot(1, 2, 2);
plot(RX_time,Carrier_phase_hz');
xlabel('TOW [s]')
ylabel('Accumulated Carrier Phase [cycles]');
grid on;

% Tercer subplot
subplot(1, 2, 3);
plot(RX_time,Carrier_Doppler_hz');
xlabel('TOW [s]');
ylabel('Doppler Frequency [Hz]');
grid on;

end
