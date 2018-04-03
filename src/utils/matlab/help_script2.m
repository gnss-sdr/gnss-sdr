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

% compare pseudoranges

close all;
% GNSS SDR
plot(GNSS_PVT_raw.tx_time(1,1:300).'-200/settings.samplingFreq,GNSS_PVT_raw.Pseudorange_m(1,1:300).')

% MATLAB
hold on;
plot(navSolutions.transmitTime,navSolutions.channel.rawP(1,:),'g')
