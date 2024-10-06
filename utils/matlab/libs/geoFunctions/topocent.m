function [Az, El, D] = topocent(X, dx)
% TOPOCENT  Transformation of vector dx into topocentric coordinate
%          system with origin at X.
%          Both parameters are 3 by 1 vectors.
%
% [Az, El, D] = topocent(X, dx);
%
%   Inputs:
%       X           - vector origin coordinates (in ECEF system [X; Y; Z;])
%       dx          - vector ([dX; dY; dZ;]).
%
%   Outputs:
%       D           - vector length. Units like units of the input
%       Az          - azimuth from north positive clockwise, degrees
%       El          - elevation angle, degrees

% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Kai Borre 11-24-96
% SPDX-License-Identifier: GPL-3.0-or-later

%==========================================================================

dtr = pi/180;

[phi, lambda, h] = togeod(6378137, 298.257223563, X(1), X(2), X(3));

cl  = cos(lambda * dtr);
sl  = sin(lambda * dtr);
cb  = cos(phi * dtr);
sb  = sin(phi * dtr);

F   = [-sl -sb*cl cb*cl;
    cl -sb*sl cb*sl;
    0    cb   sb];

local_vector = F' * dx;
E   = local_vector(1);
N   = local_vector(2);
U   = local_vector(3);

hor_dis = sqrt(E^2 + N^2);

if hor_dis < 1.e-20
    Az = 0;
    El = 90;
else
    Az = atan2(E, N)/dtr;
    El = atan2(U, hor_dis)/dtr;
end

if Az < 0
    Az = Az + 360;
end

D   = sqrt(dx(1)^2 + dx(2)^2 + dx(3)^2);
%%%%%%%%% end topocent.m %%%%%%%%%
