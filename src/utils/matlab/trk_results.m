function trk_results(trk)
% Reads GNSS-SDR Traking dump mat files and displays the corresponding
% results. Based on plotVEMLTraking.m written by Darius Plausinaitis.
%
% Inputs from .mat file: 
%   abs_E: Magnitude of the Early correlator.
%   abs_L: Magnitude of the Late correlator.
%   abs_P: Magnitude of the Prompt correlator.
%   abs_VE: Magnitude of the Very Early correlator.
%   abs_VL: Magnitude of the Very Late correlator.
%   acc_carrier_phase_rad: Accumulated carrier phase, in rad.
%   aux1: not used.
%   aux2: not used.
%   carrier_error_filt_hz: Carrier error at the output of the PLL filter, in Hz.
%   carr_error_hz: Raw carrier error (unfiltered) at the PLL output, in Hz.
%   carrier_doppler_hz: Doppler shift, in Hz.
%   carrier_doppler_rate_hz: Doppler rate, in Hz/s.
%   carrier_lock_test: Output of the carrier lock test.
%   CN0_SNV_dB_Hz: estimation, in dB-Hz.
%   code_error_chips: Raw code error (unfiltered) at the DLL output, in chips.
%   code_error_filt_chips: Code error at the output of the DLL filter, in chips.
%   code_freq_chips: Code frequency, in chips/s.
%   code_freq_rate_chips: Code frequency rate, in chips/s.
%   PRN: Satellite ID.
%   PRN_start_sample_counter: Sample counter from tracking start.
%   Prompt_I: Value of the Prompt correlator in the In-phase component.
%   Prompt_Q: Value of the Prompt correlator in the Quadrature component.
%
% Juan Alfaro, 2024
% -------------------------------------------------------------------------
%

%=== For all listed channels ==============================================
for channelNr = 1:trk.Nchan
    
    filename = [trk.file.file num2str(channelNr-1) '.mat'] % Tracking filename for channel N
    load(filename); % Load tracking file

    %% Select (or create) and clear the figure ================================
    % The number 200 is added just for more convenient handling of the open
    % figure windows, when many figures are closed and reopened.
    % Figures drawn or opened by the user, will not be "overwritten" by
    % this function.
    
    figure(channelNr +200);
    clf(channelNr +200);
    set(channelNr +200, 'Name', ['Channel ', num2str(channelNr-1), ...
        ' (PRN ', ...
        num2str(PRN(end-1)), ...
        ') results'], 'Position', [150, 100, 1300, 600]);

    %% Draw axes ==============================================================
    % Row 1
    handles(1, 1) = subplot(3, 3, 1);
    handles(1, 2) = subplot(3, 3, [2 3]);
    % Row 2
    handles(2, 1) = subplot(3, 3, 4);
    handles(2, 2) = subplot(3, 3, [5 6]);
    % Row 3
    handles(3, 1) = subplot(3, 3, 7);
    handles(3, 2) = subplot(3, 3, 8);
    handles(3, 3) = subplot(3, 3, 9);

    %% Plot all figures =======================================================

    if exist('PRN_start_sample_count')
        t = PRN_start_sample_count/trk.samplingFreq;
        timeAxis=linspace(double(t(1)), double(t(end)), length(PRN_start_sample_count));
        time_label='RX Time (s)';
    else
        timeAxis = (1:length(PRN));
        time_label='Epoch';
    end

    %----- Discrete-Time Scatter Plot ---------------------------------
    plot(handles(1, 1), Prompt_I,...
        Prompt_Q, ...
        '.');

    grid  (handles(1, 1));
    axis  (handles(1, 1), 'equal');
    title (handles(1, 1), 'Discrete-Time Scatter Plot');
    xlabel(handles(1, 1), 'I prompt');
    ylabel(handles(1, 1), 'Q prompt');

    %----- Nav bits ---------------------------------------------------
    plot  (handles(1, 2), timeAxis, ...
        Prompt_I);

    grid  (handles(1, 2));
    title (handles(1, 2), 'Bits of the navigation message');
    xlabel(handles(1, 2), time_label);
    axis  (handles(1, 2), 'tight');

    %----- PLL discriminator unfiltered--------------------------------
    plot  (handles(2, 1), timeAxis, ...
        carr_error_hz, 'r');

    grid  (handles(2, 1));
    axis  (handles(2, 1), 'tight');
    xlabel(handles(2, 1), time_label);
    ylabel(handles(2, 1), 'Amplitude');
    title (handles(2, 1), 'Raw PLL discriminator');

    %----- Correlation ------------------------------------------------
    plot(handles(2, 2), timeAxis, ...
        [abs_VE', ...
        abs_E', ...
        abs_P', ...
        abs_L', ...
        abs_VL'], ...
        '-*');

    grid  (handles(2, 2));
    title (handles(2, 2), 'Correlation results');
    xlabel(handles(2, 2), time_label);
    axis  (handles(2, 2), 'tight');

    hLegend = legend(handles(2, 2), '$\sqrt{I_{VE}^2 + Q_{VE}^2}$', ...
        '$\sqrt{I_{E}^2 + Q_{E}^2}$', ...
        '$\sqrt{I_{P}^2 + Q_{P}^2}$', ...
        '$\sqrt{I_{L}^2 + Q_{L}^2}$', ...
        '$\sqrt{I_{VL}^2 + Q_{VL}^2}$');

    %set interpreter from tex to latex. This will draw \sqrt correctly
    set(hLegend, 'Interpreter', 'Latex');

    %----- PLL discriminator filtered----------------------------------
    plot  (handles(3, 1), timeAxis, ...
        carr_error_filt_hz, 'b');

    grid  (handles(3, 1));
    axis  (handles(3, 1), 'tight');
    xlabel(handles(3, 1), time_label);
    ylabel(handles(3, 1), 'Amplitude');
    title (handles(3, 1), 'Filtered PLL discriminator');

    %----- DLL discriminator unfiltered--------------------------------
    plot  (handles(3, 2), timeAxis, ...
        code_error_chips, 'r');

    grid  (handles(3, 2));
    axis  (handles(3, 2), 'tight');
    xlabel(handles(3, 2), time_label);
    ylabel(handles(3, 2), 'Amplitude');
    title (handles(3, 2), 'Raw DLL discriminator');

    %----- DLL discriminator filtered----------------------------------
    plot  (handles(3, 3), timeAxis, ...
        code_error_filt_chips, 'b');

    grid  (handles(3, 3));
    axis  (handles(3, 3), 'tight');
    xlabel(handles(3, 3), time_label);
    ylabel(handles(3, 3), 'Amplitude');
    title (handles(3, 3), 'Filtered DLL discriminator');
    
    % Delete all variables except from tracking struct
    actual_vars = who;
    trk_index = strcmp(actual_vars, 'trk');
    del_vars = actual_vars(~trk_index);
    clear(del_vars{:});

end % for channelNr = channelList

