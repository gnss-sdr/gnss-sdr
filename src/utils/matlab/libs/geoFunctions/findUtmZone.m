function utmZone = findUtmZone(latitude, longitude)
% Function finds the UTM zone number for given longitude and latitude.
% The longitude value must be between -180 (180 degree West) and 180 (180
% degree East) degree. The latitude must be within -80 (80 degree South) and
% 84 (84 degree North).
%
% utmZone = findUtmZone(latitude, longitude);
%
% Latitude and longitude must be in decimal degrees (e.g. 15.5 degrees not
% 15 deg 30 min).

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
%==========================================================================

%% Check value bounds =====================================================

if ((longitude > 180) || (longitude < -180))
    error('Longitude value exceeds limits (-180:180).');
end

if ((latitude > 84) || (latitude < -80))
    error('Latitude value exceeds limits (-80:84).');
end

%% Find zone ==============================================================

% Start at 180 deg west = -180 deg

utmZone = fix((180 + longitude)/ 6) + 1;

%% Correct zone numbers for particular areas ==============================

if (latitude > 72)
    % Corrections for zones 31 33 35 37
    if ((longitude >= 0) && (longitude < 9))
        utmZone = 31;
    elseif ((longitude >= 9) && (longitude < 21))
        utmZone = 33;
    elseif ((longitude >= 21) && (longitude < 33))
        utmZone = 35;
    elseif ((longitude >= 33) && (longitude < 42))
        utmZone = 37;
    end
    
elseif ((latitude >= 56) && (latitude < 64))
    % Correction for zone 32
    if ((longitude >= 3) && (longitude < 12))
        utmZone = 32;
    end
end
