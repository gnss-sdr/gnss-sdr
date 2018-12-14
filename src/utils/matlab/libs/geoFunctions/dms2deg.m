
function deg = dms2deg(dms)
% DMS2DEG  Conversion of  degrees, minutes, and seconds to degrees.

% Written by Javier Arribas 2011
% December 7, 2011

%if (dms(1)>=0)
deg=dms(1)+dms(2)/60+dms(3)/3600;
%else
%deg=dms(1)-dms(2)/60-dms(3)/3600;
%end
