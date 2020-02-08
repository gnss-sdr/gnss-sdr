function deg = dms2deg(dms)
% DMS2DEG  Conversion of  degrees, minutes, and seconds to degrees.

% Copyright (C) Javier Arribas 2011
% December 7, 2011
% GNSS-SDR is a software defined Global Navigation
%           Satellite Systems receiver
%
% This file is part of GNSS-SDR.
%
% SPDX-License-Identifier: GPL-3.0-or-later

%if (dms(1)>=0)
deg=dms(1)+dms(2)/60+dms(3)/3600;
%else
%deg=dms(1)-dms(2)/60-dms(3)/3600;
%end
