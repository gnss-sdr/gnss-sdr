function  result = clsin(ar, degree, argument)
% Clenshaw summation of sinus of argument.
%
% result = clsin(ar, degree, argument);

% Written by Kai Borre
% December 20, 1995
%
% See also WGS2UTM or CART2UTM
%==========================================================================

cos_arg = 2 * cos(argument);
hr1     = 0;
hr      = 0;

for t = degree : -1 : 1
    hr2 = hr1;
    hr1 = hr;
    hr  = ar(t) + cos_arg*hr1 - hr2;
end

result = hr * sin(argument);

%%%%%%%%%%%%%%%%%%%%%%% end clsin.m  %%%%%%%%%%%%%%%%%%%%%
