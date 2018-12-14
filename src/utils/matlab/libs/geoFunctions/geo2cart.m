function [X, Y, Z] = geo2cart(phi, lambda, h, i)
% GEO2CART Conversion of geographical coordinates (phi, lambda, h) to
% Cartesian coordinates (X, Y, Z).
%
% [X, Y, Z] = geo2cart(phi, lambda, h, i);
%
% Format for phi and lambda: [degrees minutes seconds].
% h, X, Y, and Z are in meters.
%
% Choices i of Reference Ellipsoid
%   1. International Ellipsoid 1924
%   2. International Ellipsoid 1967
%   3. World Geodetic System 1972
%   4. Geodetic Reference System 1980
%   5. World Geodetic System 1984
%
%   Inputs:
%       phi       - geocentric latitude (format [degrees minutes seconds])
%       lambda    - geocentric longitude (format [degrees minutes seconds])
%       h         - height
%       i         - reference ellipsoid type
%
%   Outputs:
%       X, Y, Z   - Cartesian coordinates (meters)

% Kai Borre 10-13-98
% Copyright (c) by Kai Borre
%==========================================================================

b   = phi(1) + phi(2)/60 + phi(3)/3600;
b   = b*pi / 180;
l   = lambda(1) + lambda(2)/60 + lambda(3)/3600;
l   = l*pi / 180;

a   = [6378388 6378160 6378135 6378137 6378137];
f   = [1/297 1/298.247 1/298.26 1/298.257222101 1/298.257223563];

ex2 = (2-f(i))*f(i) / ((1-f(i))^2);
c   = a(i) * sqrt(1+ex2);
N   = c / sqrt(1 + ex2*cos(b)^2);

X   = (N+h) * cos(b) * cos(l);
Y   = (N+h) * cos(b) * sin(l);
Z   = ((1-f(i))^2*N + h) * sin(b);

%%%%%%%%%%%%%% end geo2cart.m  %%%%%%%%%%%%%%%%%%%%%%%%
