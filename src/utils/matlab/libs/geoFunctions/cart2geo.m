function [phi, lambda, h] = cart2geo(X, Y, Z, i)
% CART2GEO Conversion of Cartesian coordinates (X,Y,Z) to geographical
% coordinates (phi, lambda, h) on a selected reference ellipsoid.
%
% [phi, lambda, h] = cart2geo(X, Y, Z, i);
%
%   Choices i of Reference Ellipsoid for Geographical Coordinates
%          	  1. International Ellipsoid 1924
%	          2. International Ellipsoid 1967
%	          3. World Geodetic System 1972
%	          4. Geodetic Reference System 1980
%	          5. World Geodetic System 1984

% Kai Borre 10-13-98
% Copyright (c) by Kai Borre
% Revision: 1.0   Date: 1998/10/23
%==========================================================================

a = [6378388 6378160 6378135 6378137 6378137];
f = [1/297 1/298.247 1/298.26 1/298.257222101 1/298.257223563];

lambda = atan2(Y,X);
ex2 = (2-f(i))*f(i)/((1-f(i))^2);
c = a(i)*sqrt(1+ex2);
phi = atan(Z/((sqrt(X^2+Y^2)*(1-(2-f(i)))*f(i))));

h = 0.1; oldh = 0;
iterations = 0;
while abs(h-oldh) > 1.e-12
    oldh = h;
    N = c/sqrt(1+ex2*cos(phi)^2);
    phi = atan(Z/((sqrt(X^2+Y^2)*(1-(2-f(i))*f(i)*N/(N+h)))));
    h = sqrt(X^2+Y^2)/cos(phi)-N;
    
    iterations = iterations + 1;
    if iterations > 100
        fprintf('Failed to approximate h with desired precision. h-oldh: %e.\n', h-oldh);
        break;
    end
end

phi = phi*180/pi;
% b = zeros(1,3);
% b(1,1) = fix(phi);
% b(2,1) = fix(rem(phi,b(1,1))*60);
% b(3,1) = (phi-b(1,1)-b(1,2)/60)*3600;

lambda = lambda*180/pi;
% l = zeros(1,3);
% l(1,1) = fix(lambda);
% l(2,1) = fix(rem(lambda,l(1,1))*60);
% l(3,1) = (lambda-l(1,1)-l(1,2)/60)*3600;

%fprintf('\n     phi =%3.0f %3.0f %8.5f',b(1),b(2),b(3))
%fprintf('\n  lambda =%3.0f %3.0f %8.5f',l(1),l(2),l(3))
%fprintf('\n       h =%14.3f\n',h)

%%%%%%%%%%%%%% end cart2geo.m %%%%%%%%%%%%%%%%%%%
