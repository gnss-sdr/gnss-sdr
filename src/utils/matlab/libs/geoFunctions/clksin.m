function [re, im] = clksin(ar, degree, arg_real, arg_imag)
% Clenshaw summation of sinus with complex argument
% [re, im] = clksin(ar, degree, arg_real, arg_imag);

% Written by Kai Borre
% December 20, 1995
%
% See also WGS2UTM or CART2UTM
%
%==========================================================================

sin_arg_r   = sin(arg_real);
cos_arg_r   = cos(arg_real);
sinh_arg_i  = sinh(arg_imag);
cosh_arg_i  = cosh(arg_imag);

r           = 2 * cos_arg_r * cosh_arg_i;
i           =-2 * sin_arg_r * sinh_arg_i;

hr1 = 0; hr = 0; hi1 = 0; hi = 0;

for t = degree : -1 : 1
    hr2 = hr1;
    hr1 = hr;
    hi2 = hi1;
    hi1 = hi;
    z   = ar(t) + r*hr1 - i*hi - hr2;
    hi  =         i*hr1 + r*hi1 - hi2;
    hr  = z;
end

r   = sin_arg_r * cosh_arg_i;
i   = cos_arg_r * sinh_arg_i;

re  = r*hr - i*hi;
im  = r*hi + i*hr;
