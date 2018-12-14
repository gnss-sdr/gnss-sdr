function plotTracking(channelList, trackResults, settings)
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
% Copyright (C) Darius Plausinaitis
% Written by Darius Plausinaitis
%--------------------------------------------------------------------------
%This program is free software; you can redistribute it and/or
%modify it under the terms of the GNU General Public License
%as published by the Free Software Foundation; either version 2
%of the License, or (at your option) any later version.
%
%This program is distributed in the hope that it will be useful,
%but WITHOUT ANY WARRANTY; without even the implied warranty of
%MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%GNU General Public License for more details.
%
%You should have received a copy of the GNU General Public License
%along with this program; if not, write to the Free Software
%Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
%USA.
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
    handles(1, 1) = subplot(4, 3, 1);
    handles(1, 2) = subplot(4, 3, [2 3]);
    % Row 2
    handles(2, 1) = subplot(4, 3, 4);
    handles(2, 2) = subplot(4, 3, [5 6]);
    % Row 3
    handles(3, 1) = subplot(4, 3, 7);
    handles(3, 2) = subplot(4, 3, 8);
    handles(3, 3) = subplot(4, 3, 9);
    % Row 4
    handles(4, 1) = subplot(4, 3, 10);
    handles(4, 2) = subplot(4, 3, 11);
    handles(4, 3) = subplot(4, 3, 12);
    
    
    %% Plot all figures =======================================================
    
    timeAxisInSeconds = (1:settings.msToProcess)/1000;
    
    %----- Discrete-Time Scatter Plot ---------------------------------
    plot(handles(1, 1), trackResults(channelNr).I_P,...
        trackResults(channelNr).Q_P, ...
        '.');
    
    grid  (handles(1, 1));
    axis  (handles(1, 1), 'equal');
    title (handles(1, 1), 'Discrete-Time Scatter Plot');
    xlabel(handles(1, 1), 'I prompt');
    ylabel(handles(1, 1), 'Q prompt');
    
    %----- Nav bits ---------------------------------------------------
    plot  (handles(1, 2), timeAxisInSeconds, ...
        trackResults(channelNr).I_P);
    
    grid  (handles(1, 2));
    title (handles(1, 2), 'Bits of the navigation message');
    xlabel(handles(1, 2), 'Time (s)');
    axis  (handles(1, 2), 'tight');
    
    %----- PLL discriminator unfiltered--------------------------------
    plot  (handles(2, 1), timeAxisInSeconds, ...
        trackResults(channelNr).pllDiscr, 'r');
    
    grid  (handles(2, 1));
    axis  (handles(2, 1), 'tight');
    xlabel(handles(2, 1), 'Time (s)');
    ylabel(handles(2, 1), 'Amplitude');
    title (handles(2, 1), 'Raw PLL discriminator');
    
    %----- Correlation ------------------------------------------------
    plot(handles(2, 2), timeAxisInSeconds, ...
        [sqrt(trackResults(channelNr).I_E.^2 + ...
        trackResults(channelNr).Q_E.^2)', ...
        sqrt(trackResults(channelNr).I_P.^2 + ...
        trackResults(channelNr).Q_P.^2)', ...
        sqrt(trackResults(channelNr).I_L.^2 + ...
        trackResults(channelNr).Q_L.^2)'], ...
        '-*');
    
    grid  (handles(2, 2));
    title (handles(2, 2), 'Correlation results');
    xlabel(handles(2, 2), 'Time (s)');
    axis  (handles(2, 2), 'tight');
    
    hLegend = legend(handles(2, 2), '$\sqrt{I_{E}^2 + Q_{E}^2}$', ...
        '$\sqrt{I_{P}^2 + Q_{P}^2}$', ...
        '$\sqrt{I_{L}^2 + Q_{L}^2}$');
    
    %set interpreter from tex to latex. This will draw \sqrt correctly
    set(hLegend, 'Interpreter', 'Latex');
    
    %----- PLL discriminator filtered----------------------------------
    plot  (handles(3, 1), timeAxisInSeconds, ...
        trackResults(channelNr).pllDiscrFilt(1:settings.msToProcess), 'b');
    
    grid  (handles(3, 1));
    axis  (handles(3, 1), 'tight');
    xlabel(handles(3, 1), 'Time (s)');
    ylabel(handles(3, 1), 'Amplitude');
    title (handles(3, 1), 'Filtered PLL discriminator');
    
    %----- DLL discriminator unfiltered--------------------------------
    plot  (handles(3, 2), timeAxisInSeconds, ...
        trackResults(channelNr).dllDiscr, 'r');
    
    grid  (handles(3, 2));
    axis  (handles(3, 2), 'tight');
    xlabel(handles(3, 2), 'Time (s)');
    ylabel(handles(3, 2), 'Amplitude');
    title (handles(3, 2), 'Raw DLL discriminator');
    
    %----- DLL discriminator filtered----------------------------------
    plot  (handles(3, 3), timeAxisInSeconds, ...
        trackResults(channelNr).dllDiscrFilt, 'b');
    
    grid  (handles(3, 3));
    axis  (handles(3, 3), 'tight');
    xlabel(handles(3, 3), 'Time (s)');
    ylabel(handles(3, 3), 'Amplitude');
    title (handles(3, 3), 'Filtered DLL discriminator');
    
    %----- CNo for signal----------------------------------
    plot  (handles(4, 1), timeAxisInSeconds, ...
        trackResults(channelNr).CNo(1:settings.msToProcess), 'b');
    
    grid  (handles(4, 1));
    axis  (handles(4, 1), 'tight');
    xlabel(handles(4, 1), 'Time (s)');
    ylabel(handles(4, 1), 'CNo (dB-Hz)');
    title (handles(4, 1), 'Carrier to Noise Ratio');
    
    %----- Carrier Frequency --------------------------------
    plot  (handles(4, 2), timeAxisInSeconds(2:end), ...
        trackResults(channelNr).carrFreq(2:settings.msToProcess), 'Color',[0.42 0.25 0.39]);
    
    grid  (handles(4, 2));
    axis  (handles(4, 2));
    xlabel(handles(4, 2), 'Time (s)');
    ylabel(handles(4, 2), 'Freq (hz)');
    title (handles(4, 2), 'Carrier Freq');
    
    %----- Code Frequency----------------------------------
    %--- Skip sample 0 to help with results display
    plot  (handles(4, 3), timeAxisInSeconds(2:end), ...
        trackResults(channelNr).codeFreq(2:settings.msToProcess), 'Color',[0.2 0.3 0.49]);
    
    grid  (handles(4, 3));
    axis  (handles(4, 3), 'tight');
    xlabel(handles(4, 3), 'Time (s)');
    ylabel(handles(4, 3), 'Freq (Hz)');
    title (handles(4, 3), 'Code Freq');
    
end % for channelNr = channelList
