function acq_results(acq)
% Reads GNSS-SDR Acquisition dump mat file and displays the corresponding
% results.
%
% Inputs from .mat file: 
%   acq_delay_samples: Coarse estimation of time delay, in number of samples from the start of the pseudorandom code.
%   acq_doppler_hz: Coarse estimation of Doppler shift, in Hz.
%   acq_grid: Acquisition search grid.
%   d_positive_acq: 1 if there has been a positive acquisition, 0 for no detection.
%   doppler_max: Maximum Doppler shift in the search grid.
%   doppler_step: Doppler step in the search grid.
%   input_power: Input signal power.
%   num_dwells: Number of dwells performed in non-coherent acquisition.
%   PRN: Satellite ID.
%   sample_counter: Sample counter from receiver’s start.
%   test_statistic: Result of the test statistic.
%   threshold: Threshold above which a signal is declared present.
%
% Juan Alfaro, 2024
% -------------------------------------------------------------------------
%

%% 1.Create and load acquisition filename (.mat).

switch(acq.file.signal)
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

filename = [acq.file.path acq.file.file '_' system '_' signal...
    '_ch_' num2str(acq.file.dump_channel) '_' num2str(acq.file.execution)...
    '_sat_' num2str(acq.file.satellite) '.mat']; % Final acquisition filename

load(filename); % Load acquisition file

%% 2. Compute parameters of interest (freq, tau, freq_max, tau_max). 

[tau_max, freq_max] = find(acq_grid == max(max(acq_grid)));  % Find tau and freq max in acq_grid
freq = -doppler_max:doppler_step:(doppler_max-doppler_step); % Doppler shift [Hz]
tau = linspace(0, n_chips, size(acq_grid, 1));               % Code delay wrt reference [chips]

%% 3. Plot and print results

% Figure with the results (subplot)
figure('Position', [150, 100, 1300, 600]);

subplot(1, 2, 1);
surf(freq, tau, acq_grid, 'FaceColor', 'interp', 'LineStyle', 'none'); 
colormap(hsv);
xlabel('Doppler [Hz]'); 
ylabel('Code delay [chips]');
zlabel('Test Statistics');
xlim([min(freq) max(freq)]);
ylim([min(tau) max(tau)]);
title('GLRT statistic for Parallel Code Phase Search acquisition')

% Subplot en la segunda columna (plots 2D)
subplot(2, 2, 2);
plot(freq, acq_grid(tau_max, :),'b','LineWidth',1.4)
grid on
xlim([min(freq) max(freq)])
xlabel('Doppler shift [Hz]')
ylabel('Test statistics')
title(['Fixed code delay to ' num2str((tau_max - 1) / size(acq_grid,1) * n_chips) ' chips'])

subplot(2, 2, 4);
plot(tau, acq_grid(:, freq_max),'r')
grid on
xlim([min(tau) max(tau)])
xlabel('Code delay [chips]')
ylabel('Test statistics')
title(['Doppler wipe-off = ' num2str((freq_max - 1) * doppler_step - doppler_max) ' Hz'])

% Command window acquisition results print
disp('---------- Acquisition results ----------')
fprintf('Filename post-processed: %s \n',filename)
if d_positive_acq == 0; disp('No acquisition achieved!'); else; disp('Acquisition achieved!');end
fprintf('The error in acq delay samples is %.2f%% \n', 100*abs(acq_delay_samples-size(tau(1,:)))/size(tau(1,:)) )
fprintf('Doppler wipe-off = %.2f Hz \n',(freq_max - 1) * doppler_step - doppler_max)
fprintf('Result of the test statistc: %.2f \n', test_statistic)
disp('--------------------')

end

