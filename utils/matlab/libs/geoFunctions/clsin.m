function  result = clsin(ar, degree, argument)
% Clenshaw summation of sinus of argument.
%
% result = clsin(ar, degree, argument);

% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Kai Borre
% SPDX-License-Identifier: GPL-3.0-or-later
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
