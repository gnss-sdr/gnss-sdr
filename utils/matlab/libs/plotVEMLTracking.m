function plotVEMLTracking(channelList, trackResults, settings)
% This function plots the tracking results for the given channel list.
%
% plotTracking(channelList, trackResults, settings)
%
%   Inputs:
%       channelList     - list of channels to be plotted.
%       trackResults    - tracking results from the tracking function.
%       settings        - receiver settings.

%--------------------------------------------------------------------------
%                           SoftGNSS v3.0
%
% Written by Darius Plausinaitis
%--------------------------------------------------------------------------
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Darius Plausinaitis
% SPDX-License-Identifier: GPL-3.0-or-later
%--------------------------------------------------------------------------

% Protection - if the list contains incorrect channel numbers
channelList = intersect(channelList, 1:settings.numberOfChannels);

%=== For all listed channels ==============================================
for channelNr = channelList

    %% Select (or create) and clear the figure ================================
    % The number 200 is added just for more convenient handling of the open
    % figure windows, when many figures are closed and reopened.
    % Figures drawn or opened by the user, will not be "overwritten" by
    % this function.

    figure(channelNr +200);
    clf(channelNr +200);
    set(channelNr +200, 'Name', ['Channel ', num2str(channelNr), ...
        ' (PRN ', ...
        num2str(trackResults(channelNr).PRN(end-1)), ...
        ') results']);

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

    if isfield(trackResults(channelNr), 'prn_start_time_s')
        timeAxis=trackResults(channelNr).prn_start_time_s;
        time_label='RX Time (s)';
    else
        timeAxis = (1:length(trackResults(channelNr).PRN));
        time_label='Epoch';
    end

    %----- Discrete-Time Scatter Plot ---------------------------------
    plot(handles(1, 1), trackResults(channelNr).data_I,...
        trackResults(channelNr).data_Q, ...
        '.');

    grid  (handles(1, 1));
    axis  (handles(1, 1), 'equal');
    title (handles(1, 1), 'Discrete-Time Scatter Plot');
    xlabel(handles(1, 1), 'I prompt');
    ylabel(handles(1, 1), 'Q prompt');

    %----- Nav bits ---------------------------------------------------
    plot  (handles(1, 2), timeAxis, ...
        trackResults(channelNr).data_I);

    grid  (handles(1, 2));
    title (handles(1, 2), 'Bits of the navigation message');
    xlabel(handles(1, 2), time_label);
    axis  (handles(1, 2), 'tight');

    %----- PLL discriminator unfiltered--------------------------------
    plot  (handles(2, 1), timeAxis, ...
        trackResults(channelNr).pllDiscr, 'r');

    grid  (handles(2, 1));
    axis  (handles(2, 1), 'tight');
    xlabel(handles(2, 1), time_label);
    ylabel(handles(2, 1), 'Amplitude');
    title (handles(2, 1), 'Raw PLL discriminator');

    %----- Correlation ------------------------------------------------
    plot(handles(2, 2), timeAxis, ...
        [sqrt(trackResults(channelNr).I_VE.^2 + ...
        trackResults(channelNr).Q_VE.^2)', ...
        sqrt(trackResults(channelNr).I_E.^2 + ...
        trackResults(channelNr).Q_E.^2)', ...
        sqrt(trackResults(channelNr).I_P.^2 + ...
        trackResults(channelNr).Q_P.^2)', ...
        sqrt(trackResults(channelNr).I_L.^2 + ...
        trackResults(channelNr).Q_L.^2)', ...
        sqrt(trackResults(channelNr).I_VL.^2 + ...
        trackResults(channelNr).Q_VL.^2)'], ...
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
        trackResults(channelNr).pllDiscrFilt, 'b');

    grid  (handles(3, 1));
    axis  (handles(3, 1), 'tight');
    xlabel(handles(3, 1), time_label);
    ylabel(handles(3, 1), 'Amplitude');
    title (handles(3, 1), 'Filtered PLL discriminator');

    %----- DLL discriminator unfiltered--------------------------------
    plot  (handles(3, 2), timeAxis, ...
        trackResults(channelNr).dllDiscr, 'r');

    grid  (handles(3, 2));
    axis  (handles(3, 2), 'tight');
    xlabel(handles(3, 2), time_label);
    ylabel(handles(3, 2), 'Amplitude');
    title (handles(3, 2), 'Raw DLL discriminator');

    %----- DLL discriminator filtered----------------------------------
    plot  (handles(3, 3), timeAxis, ...
        trackResults(channelNr).dllDiscrFilt, 'b');

    grid  (handles(3, 3));
    axis  (handles(3, 3), 'tight');
    xlabel(handles(3, 3), time_label);
    ylabel(handles(3, 3), 'Amplitude');
    title (handles(3, 3), 'Filtered DLL discriminator');

end % for channelNr = channelList
