function plotKalman(channelList, trackResults, settings)
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
    
    timeStart = settings.timeStartInSeconds;
    
    %% Draw axes ==============================================================
    % Row 1
    handles(1, 1) = subplot(4, 2, 1);
    handles(1, 2) = subplot(4, 2, 2);
    % Row 2
    handles(2, 1) = subplot(4, 2, 3);
    handles(2, 2) = subplot(4, 2, 4);
    % Row 3
    handles(3, 1) = subplot(4, 2, [5 6]);
    % Row 4
    handles(4, 1) = subplot(4, 2, [7 8]);
    
    %% Plot all figures =======================================================
    
    timeAxisInSeconds = (1:settings.msToProcess)/1000;
    
    %----- CNo for signal----------------------------------
    plot  (handles(1, 1), timeAxisInSeconds, ...
        trackResults(channelNr).CNo(1:settings.msToProcess), 'b');
    
    grid  (handles(1, 1));
    axis  (handles(1, 1), 'tight');
    xlabel(handles(1, 1), 'Time (s)');
    ylabel(handles(1, 1), 'CNo (dB-Hz)');
    title (handles(1, 1), 'Carrier to Noise Ratio');
    
    %----- PLL discriminator filtered----------------------------------
    plot  (handles(1, 2), timeAxisInSeconds, ...
        trackResults(channelNr).state1(1:settings.msToProcess), 'b');
    
    grid  (handles(1, 2));
    axis  (handles(1, 2), 'tight');
    xlim  (handles(1, 2), [timeStart, timeAxisInSeconds(end)]);
    xlabel(handles(1, 2), 'Time (s)');
    ylabel(handles(1, 2), 'Phase Amplitude');
    title (handles(1, 2), 'Filtered Carrier Phase');
    
    %----- Carrier Frequency --------------------------------
    plot  (handles(2, 1), timeAxisInSeconds(2:end), ...
        trackResults(channelNr).state2(2:settings.msToProcess), 'Color',[0.42 0.25 0.39]);
    
    grid  (handles(2, 1));
    axis  (handles(2, 1));
    xlim  (handles(2, 1), [timeStart, timeAxisInSeconds(end)]);
    xlabel(handles(2, 1), 'Time (s)');
    ylabel(handles(2, 1), 'Freq (hz)');
    title (handles(2, 1), 'Filtered Doppler Frequency');
    
    %----- Carrier Frequency Rate --------------------------------
    plot  (handles(2, 2), timeAxisInSeconds(2:end), ...
        trackResults(channelNr).state3(2:settings.msToProcess), 'Color',[0.42 0.25 0.39]);
    
    grid  (handles(2, 2));
    axis  (handles(2, 2));
    xlim  (handles(2, 2), [timeStart, timeAxisInSeconds(end)]);
    xlabel(handles(2, 2), 'Time (s)');
    ylabel(handles(2, 2), 'Freq (hz)');
    title (handles(2, 2), 'Filtered Doppler Frequency Rate');
    
    %----- PLL discriminator unfiltered--------------------------------
    plot  (handles(3, 1), timeAxisInSeconds, ...
        trackResults(channelNr).innovation, 'r');
    
    grid  (handles(3, 1));
    axis  (handles(3, 1), 'auto');
    xlim  (handles(3, 1), [timeStart, timeAxisInSeconds(end)]);
    xlabel(handles(3, 1), 'Time (s)');
    ylabel(handles(3, 1), 'Amplitude');
    title (handles(3, 1), 'Raw PLL discriminator (Innovation)');
    
    
    %----- PLL discriminator covariance --------------------------------
    plot  (handles(4, 1), timeAxisInSeconds, ...
        trackResults(channelNr).r_noise_cov, 'r');
    
    grid  (handles(4, 1));
    axis  (handles(4, 1), 'auto');
    xlim  (handles(4, 1), [timeStart, timeAxisInSeconds(end)]);
    xlabel(handles(4, 1), 'Time (s)');
    ylabel(handles(4, 1), 'Variance');
    title (handles(4, 1), 'Estimated Noise Variance');
end % for channelNr = channelList
