function tel_results(tel)
% Reads GNSS-SDR Telemetry decoder dump mat files and displays the corresponding
% results. Based on plotVEMLTraking.m written by Darius Plausinaitis.
%
% Inputs from .mat file: 
%   TOW_at_current_symbol_ms: Time of Week associated with the current symbol for each epoch, in ms (different granularity depending on the message structure for each particular signal). Data type: double.
%   tracking_sample_counter: Sample counter associated with each epoch. Data type: uint64_t.
%   TOW_at_Preamble_ms: Time of Week associated with the preamble of the current symbol for each epoch, in ms (different granularity depending on the message structure for each particular signal). Data type: double.
%   nav_symbol: Navigation message symbol +-1, as obtained by the Tracking block, for each epoch. Data type: int32_t.
%   PRN: Satellite ID processed in each epoch. Data type: int32_t.
%
% Juan Alfaro, 2024
% -------------------------------------------------------------------------
%

load(tel.file.file)
% Figure with the results (subplot)
figure('Position', [150, 100, 1300, 600]);

% Primer subplot
subplot(1, 2, 1);
plot(tracking_sample_counter, TOW_at_current_symbol_ms, 'ro');
grid on
title('Title1');
xlabel('Sample counter');
ylabel('TOW at current symbol [ms]');

% Segundo subplot
subplot(1, 2, 2);
plot(tracking_sample_counter, TOW_at_Preamble_ms, 'b','LineWidth',1.2);
grid on
title('Title2');
xlabel('Sample counter');
ylabel('TOW at Preamble [ms]');

end

