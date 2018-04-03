function X_sat_rot = e_r_corr(traveltime, X_sat)
% E_R_CORR  Returns rotated satellite ECEF coordinates due to Earth
% rotation during signal travel time
%
% X_sat_rot = e_r_corr(traveltime, X_sat);
%
%   Inputs:
%       travelTime  - signal travel time
%       X_sat       - satellite's ECEF coordinates
%
%   Outputs:
%       X_sat_rot   - rotated satellite's coordinates (ECEF)

% Written by Kai Borre
% Copyright (c) by Kai Borre
%==========================================================================

Omegae_dot = 7.292115147e-5;           %  rad/sec

%--- Find rotation angle --------------------------------------------------
omegatau   = Omegae_dot * traveltime;

%--- Make a rotation matrix -----------------------------------------------
R3 = [ cos(omegatau)    sin(omegatau)   0;
    -sin(omegatau)    cos(omegatau)   0;
    0                0               1];

%--- Do the rotation ------------------------------------------------------
X_sat_rot = R3 * X_sat;

%%%%%%%% end e_r_corr.m %%%%%%%%%%%%%%%%%%%%
