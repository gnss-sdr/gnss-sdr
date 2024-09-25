function corrTime = check_t(time)
% CHECK_T accounting for beginning or end of week crossover.
%
% corrTime = check_t(time);
%
%   Inputs:
%       time        - time in seconds
%
%   Outputs:
%       corrTime    - corrected time (seconds)

% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Kai Borre
% SPDX-License-Identifier: GPL-3.0-or-later
%==========================================================================

half_week = 302400;     % seconds

corrTime = time;

if time > half_week
    corrTime = time - 2*half_week;
elseif time < -half_week
    corrTime = time + 2*half_week;
end

%%%%%%% end check_t.m  %%%%%%%%%%%%%%%%%
