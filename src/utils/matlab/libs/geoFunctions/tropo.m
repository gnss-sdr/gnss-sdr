function ddr = tropo(sinel, hsta, p, tkel, hum, hp, htkel, hhum)
% TROPO  Calculation of tropospheric correction.
%       The range correction ddr in m is to be subtracted from
%       pseudo-ranges and carrier phases
%
% ddr = tropo(sinel, hsta, p, tkel, hum, hp, htkel, hhum);
%
%   Inputs:
%       sinel   - sin of elevation angle of satellite
%       hsta    - height of station in km
%       p       - atmospheric pressure in mb at height hp
%       tkel    - surface temperature in degrees Kelvin at height htkel
%       hum     - humidity in % at height hhum
%       hp      - height of pressure measurement in km
%       htkel   - height of temperature measurement in km
%       hhum    - height of humidity measurement in km
%
%   Outputs:
%       ddr     - range correction (meters)
%
% Reference
% Goad, C.C. & Goodman, L. (1974) A Modified Tropospheric
% Refraction Correction Model. Paper presented at the
% American Geophysical Union Annual Fall Meeting, San
% Francisco, December 12-17

% A Matlab reimplementation of a C code from driver.
% Kai Borre 06-28-95
%==========================================================================

a_e    = 6378.137;     % semi-major axis of earth ellipsoid
b0     = 7.839257e-5;
tlapse = -6.5;
tkhum  = tkel + tlapse*(hhum-htkel);
atkel  = 7.5*(tkhum-273.15) / (237.3+tkhum-273.15);
e0     = 0.0611 * hum * 10^atkel;
tksea  = tkel - tlapse*htkel;
em     = -978.77 / (2.8704e6*tlapse*1.0e-5);
tkelh  = tksea + tlapse*hhum;
e0sea  = e0 * (tksea/tkelh)^(4*em);
tkelp  = tksea + tlapse*hp;
psea   = p * (tksea/tkelp)^em;

if sinel < 0
    sinel = 0;
end

tropo   = 0;
done    = 'FALSE';
refsea  = 77.624e-6 / tksea;
htop    = 1.1385e-5 / refsea;
refsea  = refsea * psea;
ref     = refsea * ((htop-hsta)/htop)^4;

while 1
    rtop = (a_e+htop)^2 - (a_e+hsta)^2*(1-sinel^2);
    
    % check to see if geometry is crazy
    if rtop < 0
        rtop = 0;
    end
    
    rtop = sqrt(rtop) - (a_e+hsta)*sinel;
    a    = -sinel/(htop-hsta);
    b    = -b0*(1-sinel^2) / (htop-hsta);
    rn   = zeros(8,1);
    
    for i = 1:8
        rn(i) = rtop^(i+1);
    end
    
    alpha = [2*a, 2*a^2+4*b/3, a*(a^2+3*b),...
        a^4/5+2.4*a^2*b+1.2*b^2, 2*a*b*(a^2+3*b)/3,...
        b^2*(6*a^2+4*b)*1.428571e-1, 0, 0];
    
    if b^2 > 1.0e-35
        alpha(7) = a*b^3/2;
        alpha(8) = b^4/9;
    end
    
    dr = rtop;
    dr = dr + alpha*rn;
    tropo = tropo + dr*ref*1000;
    
    if done == 'TRUE '
        ddr = tropo;
        break;
    end
    
    done    = 'TRUE ';
    refsea  = (371900.0e-6/tksea-12.92e-6)/tksea;
    htop    = 1.1385e-5 * (1255/tksea+0.05)/refsea;
    ref     = refsea * e0sea * ((htop-hsta)/htop)^4;
end;
%%%%%%%%% end tropo.m  %%%%%%%%%%%%%%%%%%%
