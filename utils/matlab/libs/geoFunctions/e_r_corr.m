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

% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Kai Borre
% SPDX-License-Identifier: GPL-3.0-or-later
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
