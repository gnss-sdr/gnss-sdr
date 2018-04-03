% Reads GNSS-SDR PVT dump binary file using the provided
%  function and plots some internal variables
% Javier Arribas, 2011. jarribas(at)cttc.es
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

close all;
clear all;

% True position of the antenna in UTM system (if known). Otherwise enter
% all NaN's and mean position will be used as a reference .
settings.truePosition.E     = nan;
settings.truePosition.N     = nan;
settings.truePosition.U     = nan;

settings.navSolPeriod=100; %[ms]

filename='/home/javier/workspace/gnss-sdr/trunk/install/PVT.dat';

navSolutions = gps_l1_ca_pvt_read_pvt_dump (filename);

% Reference position for Agilent cap2.dat (San Francisco static scenario)
% Scenario latitude  is 37.8194388888889  N37 49 9.98
% Scenario longitude is -122.4784944  W122 28 42.58
% Scenario elevation is 35 meters.
lat=[37 49 9.98];
long=[-122 -28 -42.58];

lat_deg=dms2deg(lat);
long_deg=dms2deg(long);

h=35;
%Choices i of Reference Ellipsoid
%   1. International Ellipsoid 1924
%   2. International Ellipsoid 1967
%   3. World Geodetic System 1972
%   4. Geodetic Reference System 1980
%   5. World Geodetic System 1984
[X, Y, Z]=geo2cart(lat, long, h, 5); % geographical to cartesian conversion

%=== Convert to UTM coordinate system =============================
utmZone = findUtmZone(lat_deg, long_deg);

[settings.truePosition.E, ...
    settings.truePosition.N, ...
    settings.truePosition.U] = cart2utm(X, Y, Z, utmZone);


for k=1:1:length(navSolutions.X)
    [navSolutions.E(k), ...
        navSolutions.N(k), ...
        navSolutions.U(k)]=cart2utm(navSolutions.X(k), navSolutions.Y(k), navSolutions.Z(k), utmZone);
end

plot_skyplot=0;
plotNavigation(navSolutions,settings,plot_skyplot);

