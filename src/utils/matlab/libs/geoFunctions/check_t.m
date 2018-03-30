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

% Kai Borre 04-01-96
% Copyright (c) by Kai Borre
%==========================================================================

half_week = 302400;     % seconds

corrTime = time;

if time > half_week
    corrTime = time - 2*half_week;
elseif time < -half_week
    corrTime = time + 2*half_week;
end

%%%%%%% end check_t.m  %%%%%%%%%%%%%%%%%
