function deg = dms2deg(dms)
% DMS2DEG  Conversion of  degrees, minutes, and seconds to degrees.

% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Javier Arribas 2011
% SPDX-License-Identifier: GPL-3.0-or-later


%if (dms(1)>=0)
deg=dms(1)+dms(2)/60+dms(3)/3600;
%else
%deg=dms(1)-dms(2)/60-dms(3)/3600;
%end
