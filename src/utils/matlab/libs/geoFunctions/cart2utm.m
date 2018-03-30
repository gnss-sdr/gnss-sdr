function [E, N, U] = cart2utm(X, Y, Z, zone)
% CART2UTM  Transformation of (X,Y,Z) to (N,E,U) in UTM, zone 'zone'.
%
% [E, N, U] = cart2utm(X, Y, Z, zone);
%
%   Inputs:
%       X,Y,Z       - Cartesian coordinates. Coordinates are referenced
%                   with respect to the International Terrestrial Reference
%                   Frame 1996 (ITRF96)
%       zone        - UTM zone of the given position
%
%   Outputs:
%      E, N, U      - UTM coordinates (Easting, Northing, Uping)

% Kai Borre -11-1994
% Copyright (c) by Kai Borre

% This implementation is based upon
% O. Andersson & K. Poder (1981) Koordinattransformationer
%  ved Geod\ae{}tisk Institut. Landinspekt\oe{}ren
%  Vol. 30: 552--571 and Vol. 31: 76
%
% An excellent, general reference (KW) is
% R. Koenig & K.H. Weise (1951) Mathematische Grundlagen der
%  h\"oheren Geod\"asie und Kartographie.
%  Erster Band, Springer Verlag

% Explanation of variables used:
% f	   flattening of ellipsoid
% a	   semi major axis in m
% m0	   1 - scale at central meridian; for UTM 0.0004
% Q_n	   normalized meridian quadrant
% E0	   Easting of central meridian
% L0	   Longitude of central meridian
% bg	   constants for ellipsoidal geogr. to spherical geogr.
% gb	   constants for spherical geogr. to ellipsoidal geogr.
% gtu	   constants for ellipsoidal N, E to spherical N, E
% utg	   constants for spherical N, E to ellipoidal N, E
% tolutm	tolerance for utm, 1.2E-10*meridian quadrant
% tolgeo	tolerance for geographical, 0.00040 second of arc

% B, L refer to latitude and longitude. Southern latitude is negative
% International ellipsoid of 1924, valid for ED50

a     = 6378388;
f     = 1/297;
ex2   = (2-f)*f / ((1-f)^2);
c     = a * sqrt(1+ex2);
vec   = [X; Y; Z-4.5];
alpha = .756e-6;
R     = [ 1       -alpha  0;
    alpha	1       0;
    0       0       1];
trans = [89.5; 93.8; 127.6];
scale = 0.9999988;
v     = scale*R*vec + trans;	  % coordinate vector in ED50
L     = atan2(v(2), v(1));
N1    = 6395000;		          % preliminary value
B     = atan2(v(3)/((1-f)^2*N1), norm(v(1:2))/N1); % preliminary value
U     = 0.1;  oldU = 0;

iterations = 0;
while abs(U-oldU) > 1.e-4
    oldU = U;
    N1   = c/sqrt(1+ex2*(cos(B))^2);
    B    = atan2(v(3)/((1-f)^2*N1+U), norm(v(1:2))/(N1+U) );
    U    = norm(v(1:2))/cos(B)-N1;
    
    iterations = iterations + 1;
    if iterations > 100
        fprintf('Failed to approximate U with desired precision. U-oldU: %e.\n', U-oldU);
        break;
    end
end

% Normalized meridian quadrant, KW p. 50 (96), p. 19 (38b), p. 5 (21)
m0  = 0.0004;
n   = f / (2-f);
m   = n^2 * (1/4 + n*n/64);
w   = (a*(-n-m0+m*(1-m0))) / (1+n);
Q_n = a + w;

% Easting and longitude of central meridian
E0      = 500000;
L0      = (zone-30)*6 - 3;

% Check tolerance for reverse transformation
tolutm  = pi/2 * 1.2e-10 * Q_n;
tolgeo  = 0.000040;

% Coefficients of trigonometric series

% ellipsoidal to spherical geographical, KW p. 186--187, (51)-(52)
% bg[1] = n*(-2 + n*(2/3    + n*(4/3	  + n*(-82/45))));
% bg[2] = n^2*(5/3    + n*(-16/15 + n*(-13/9)));
% bg[3] = n^3*(-26/15 + n*34/21);
% bg[4] = n^4*1237/630;

% spherical to ellipsoidal geographical, KW p. 190--191, (61)-(62)
% gb[1] = n*(2	      + n*(-2/3    + n*(-2	 + n*116/45)));
% gb[2] = n^2*(7/3    + n*(-8/5 + n*(-227/45)));
% gb[3] = n^3*(56/15 + n*(-136/35));
% gb[4] = n^4*4279/630;

% spherical to ellipsoidal N, E, KW p. 196, (69)
%  gtu[1] = n*(1/2	  + n*(-2/3    + n*(5/16     + n*41/180)));
%  gtu[2] = n^2*(13/48	  + n*(-3/5 + n*557/1440));
%  gtu[3] = n^3*(61/240	 + n*(-103/140));
%  gtu[4] = n^4*49561/161280;

% ellipsoidal to spherical N, E, KW p. 194, (65)
%  utg[1] = n*(-1/2	   + n*(2/3    + n*(-37/96	+ n*1/360)));
%  utg[2] = n^2*(-1/48	  + n*(-1/15 + n*437/1440));
%  utg[3] = n^3*(-17/480 + n*37/840);
%  utg[4] = n^4*(-4397/161280);

% With f = 1/297 we get

bg = [-3.37077907e-3;
    4.73444769e-6;
    -8.29914570e-9;
    1.58785330e-11];

gb = [ 3.37077588e-3;
    6.62769080e-6;
    1.78718601e-8;
    5.49266312e-11];

gtu = [ 8.41275991e-4;
    7.67306686e-7;
    1.21291230e-9;
    2.48508228e-12];

utg = [-8.41276339e-4;
    -5.95619298e-8;
    -1.69485209e-10;
    -2.20473896e-13];

% Ellipsoidal latitude, longitude to spherical latitude, longitude
neg_geo = 'FALSE';

if B < 0
    neg_geo = 'TRUE ';
end

Bg_r    = abs(B);
[res_clensin] = clsin(bg, 4, 2*Bg_r);
Bg_r    = Bg_r + res_clensin;
L0      = L0*pi / 180;
Lg_r    = L - L0;

% Spherical latitude, longitude to complementary spherical latitude
%  i.e. spherical N, E
cos_BN  = cos(Bg_r);
Np      = atan2(sin(Bg_r), cos(Lg_r)*cos_BN);
Ep      = atanh(sin(Lg_r) * cos_BN);

%Spherical normalized N, E to ellipsoidal N, E
Np      = 2 * Np;
Ep      = 2 * Ep;
[dN, dE] = clksin(gtu, 4, Np, Ep);
Np      = Np/2;
Ep      = Ep/2;
Np      = Np + dN;
Ep      = Ep + dE;
N       = Q_n * Np;
E       = Q_n*Ep + E0;

if neg_geo == 'TRUE '
    N = -N + 20000000;
end;

%%%%%%%%%%%%%%%%%%%% end cart2utm.m %%%%%%%%%%%%%%%%%%%%