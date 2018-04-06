% Function plots variations of coordinates over time and a 3D position
% plot. It plots receiver coordinates in UTM system or coordinate offsets if
% the true UTM receiver coordinates are provided.
%
% plotNavigation(navSolutions, settings)
%
%   Inputs:
%       navSolutions    - Results from navigation solution function. It
%                       contains measured pseudoranges and receiver
%                       coordinates.
%       settings        - Receiver settings. The true receiver coordinates
%                       are contained in this structure.
%       plot_skyplot    - If ==1 then use satellite coordinates to plot the
%                       the satellite positions

% Darius Plausinaitis
% Modified by Javier Arribas, 2011. jarribas(at)cttc.es
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
% along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
%
% -------------------------------------------------------------------------
%

function plotNavigation(navSolutions, settings,plot_skyplot)


%% Plot results in the necessary data exists ==============================
if (~isempty(navSolutions))
    
    %% If reference position is not provided, then set reference position
    %% to the average position
    if isnan(settings.truePosition.E) || isnan(settings.truePosition.N) ...
            || isnan(settings.truePosition.U)
        
        %=== Compute mean values ==========================================
        % Remove NaN-s or the output of the function MEAN will be NaN.
        refCoord.E = mean(navSolutions.E(~isnan(navSolutions.E)));
        refCoord.N = mean(navSolutions.N(~isnan(navSolutions.N)));
        refCoord.U = mean(navSolutions.U(~isnan(navSolutions.U)));
        
        %Also convert geodetic coordinates to deg:min:sec vector format
        meanLongitude = dms2mat(deg2dms(...
            mean(navSolutions.longitude(~isnan(navSolutions.longitude)))), -5);
        meanLatitude  = dms2mat(deg2dms(...
            mean(navSolutions.latitude(~isnan(navSolutions.latitude)))), -5);
        
        LatLong_str=[num2str(meanLatitude(1)), '??', ...
            num2str(meanLatitude(2)), '''', ...
            num2str(meanLatitude(3)), '''''', ...
            ',', ...
            num2str(meanLongitude(1)), '??', ...
            num2str(meanLongitude(2)), '''', ...
            num2str(meanLongitude(3)), '''''']
        
        
        
        refPointLgText = ['Mean Position\newline  Lat: ', ...
            num2str(meanLatitude(1)), '{\circ}', ...
            num2str(meanLatitude(2)), '{\prime}', ...
            num2str(meanLatitude(3)), '{\prime}{\prime}', ...
            '\newline Lng: ', ...
            num2str(meanLongitude(1)), '{\circ}', ...
            num2str(meanLongitude(2)), '{\prime}', ...
            num2str(meanLongitude(3)), '{\prime}{\prime}', ...
            '\newline Hgt: ', ...
            num2str(mean(navSolutions.height(~isnan(navSolutions.height))), '%+6.1f')];
        
    else
        % compute the mean error for static receiver
        mean_position.E = mean(navSolutions.E(~isnan(navSolutions.E)));
        mean_position.N = mean(navSolutions.N(~isnan(navSolutions.N)));
        mean_position.U = mean(navSolutions.U(~isnan(navSolutions.U)));
        refCoord.E = settings.truePosition.E;
        refCoord.N = settings.truePosition.N;
        refCoord.U = settings.truePosition.U;
        
        error_meters=sqrt((mean_position.E-refCoord.E)^2+(mean_position.N-refCoord.N)^2+(mean_position.U-refCoord.U)^2);
        
        refPointLgText = ['Reference Position, Mean 3D error = ' num2str(error_meters) ' [m]'];
    end
    
    figureNumber = 300;
    % The 300 is chosen for more convenient handling of the open
    % figure windows, when many figures are closed and reopened. Figures
    % drawn or opened by the user, will not be "overwritten" by this
    % function if the auto numbering is not used.
    
    %=== Select (or create) and clear the figure ==========================
    figure(figureNumber);
    clf   (figureNumber);
    set   (figureNumber, 'Name', 'Navigation solutions');
    
    %--- Draw axes --------------------------------------------------------
    handles(1, 1) = subplot(4, 2, 1 : 4);
    handles(3, 1) = subplot(4, 2, [5, 7]);
    handles(3, 2) = subplot(4, 2, [6, 8]);
    
    %% Plot all figures =======================================================
    
    %--- Coordinate differences in UTM system -----------------------------
    plot(handles(1, 1), [(navSolutions.E - refCoord.E)', ...
        (navSolutions.N - refCoord.N)',...
        (navSolutions.U - refCoord.U)']);
    
    title (handles(1, 1), 'Coordinates variations in UTM system');
    legend(handles(1, 1), 'E', 'N', 'U');
    xlabel(handles(1, 1), ['Measurement period: ', ...
        num2str(settings.navSolPeriod), 'ms']);
    ylabel(handles(1, 1), 'Variations (m)');
    grid  (handles(1, 1));
    axis  (handles(1, 1), 'tight');
    
    %--- Position plot in UTM system --------------------------------------
    plot3 (handles(3, 1), navSolutions.E - refCoord.E, ...
        navSolutions.N - refCoord.N, ...
        navSolutions.U - refCoord.U, '+');
    hold  (handles(3, 1), 'on');
    
    %Plot the reference point
    plot3 (handles(3, 1), 0, 0, 0, 'r+', 'LineWidth', 1.5, 'MarkerSize', 10);
    hold  (handles(3, 1), 'off');
    
    view  (handles(3, 1), 0, 90);
    axis  (handles(3, 1), 'equal');
    grid  (handles(3, 1), 'minor');
    
    legend(handles(3, 1), 'Measurements', refPointLgText);
    
    title (handles(3, 1), 'Positions in UTM system (3D plot)');
    xlabel(handles(3, 1), 'East (m)');
    ylabel(handles(3, 1), 'North (m)');
    zlabel(handles(3, 1), 'Upping (m)');
    
    if (plot_skyplot==1)
        %--- Satellite sky plot -----------------------------------------------
        skyPlot(handles(3, 2), ...
            navSolutions.channel.az, ...
            navSolutions.channel.el, ...
            navSolutions.channel.PRN(:, 1));
        
        title (handles(3, 2), ['Sky plot (mean PDOP: ', ...
            num2str(mean(navSolutions.DOP(2,:))), ')']);
    end
    
else
    disp('plotNavigation: No navigation data to plot.');
end % if (~isempty(navSolutions))
