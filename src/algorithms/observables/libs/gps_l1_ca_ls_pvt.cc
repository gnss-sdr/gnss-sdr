
/**
 * Copyright notice
 */

/**
 * Author: Javier Arribas, 2011. jarribas(at)cttc.es
 */

#include <itpp/itbase.h>
#include <itpp/stat/misc_stat.h>
#include <itpp/base/matfunc.h>

using namespace itpp;

#include "gps_l1_ca_ls_pvt.h"

gps_l1_ca_ls_pvt::gps_l1_ca_ls_pvt(int nchannels)
{
    // init empty ephemerids for all the available GNSS channels
    d_nchannels=nchannels;
    d_ephemeris=new gps_navigation_message[nchannels];
}

gps_l1_ca_ls_pvt::~gps_l1_ca_ls_pvt()
{
    delete d_ephemeris;
}

vec gps_l1_ca_ls_pvt::e_r_corr(double traveltime, vec X_sat) {
    /*
     %E_R_CORR  Returns rotated satellite ECEF coordinates due to Earth
     %rotation during signal travel time
     %
     %X_sat_rot = e_r_corr(traveltime, X_sat);
     %
     %   Inputs:
     %       travelTime  - signal travel time
     %       X_sat       - satellite's ECEF coordinates
     %
     %   Outputs:
     %       X_sat_rot   - rotated satellite's coordinates (ECEF)
     */

    double Omegae_dot = 7.292115147e-5; //  rad/sec

    //--- Find rotation angle --------------------------------------------------
    double omegatau;
    omegatau = Omegae_dot * traveltime;

    //--- Make a rotation matrix -----------------------------------------------
    mat R3=zeros(3,3);
    R3(0, 0)= cos(omegatau);
    R3(0, 1)= sin(omegatau);
    R3(0, 2)= 0.0;
    R3(1, 0)=-sin(omegatau);
    R3(1, 1)= cos(omegatau);
    R3(1, 2)=0.0;
    R3(2, 0)= 0.0;
    R3(2, 1)= 0.0;
    R3(2, 2)= 1;
    //--- Do the rotation ------------------------------------------------------
    vec X_sat_rot;
    X_sat_rot = R3 * X_sat;
    return X_sat_rot;

}
vec gps_l1_ca_ls_pvt::leastSquarePos(mat satpos, vec obs, mat w) {
    //Function calculates the Least Square Solution.

    //pos= leastSquarePos(satpos, obs, w);

    //   Inputs:
    //       satpos      - Satellites positions (in ECEF system: [X; Y; Z;]
    //       obs         - Observations - the pseudorange measurements to each
    //                   satellite
    //       w           - weigths vector

    //   Outputs:
    //       pos         - receiver position and receiver clock error
    //                   (in ECEF system: [X, Y, Z, dt])

    //////////////////////////////////////////////////////////////////////////
    //       el          - Satellites elevation angles (degrees)
    //       az          - Satellites azimuth angles (degrees)
    //       dop         - Dilutions Of Precision ([GDOP PDOP HDOP VDOP TDOP])

    //=== Initialization =======================================================
    int nmbOfIterations = 10; // TODO: include in config

    //double dtr = GPS_PI / 180.0;

    int nmbOfSatellites;
    nmbOfSatellites = satpos.cols();
    vec pos = "0.0 0.0 0.0 0.0";
    mat A;
    mat omc;
    mat az;
    mat el;
    A=zeros(nmbOfSatellites,4);
    omc=zeros(nmbOfSatellites,1);
    az=zeros(1,nmbOfSatellites);
    el=zeros(1,nmbOfSatellites);
    for (int i = 0; i < nmbOfSatellites; i++) {
        for (int j = 0; j < 4; j++) {
            A.set(i, j, 0.0);
        }
        omc(i, 0)=0.0;
        az(0, i)=0.0;
    }
    el = az;
    mat X = satpos;

    vec Rot_X;
    double rho2;
    double traveltime;
    double trop;
    mat mat_tmp;
    vec x;
    //=== Iteratively find receiver position ===================================
    for (int iter = 0; iter < nmbOfIterations; iter++) {
        for (int i = 0; i < nmbOfSatellites; i++) {
            if (iter == 0) {
                //--- Initialize variables at the first iteration --------------
                Rot_X=X.get_col(i);
                trop = 0.0;
            } else {
                //--- Update equations -----------------------------------------
                rho2 = (X(0, i) - pos(0))*(X(0, i) - pos(0))  + (X(1, i) - pos(1))*(X(1, i) - pos(1))+ (X(2,i) - pos(2))*(X(2,i) - pos(2));
                traveltime = sqrt(rho2) / GPS_C_m_s;
                //--- Correct satellite position (do to earth rotation) --------
                Rot_X = e_r_corr(traveltime, X.get_col(i));

                //--- Find the elevation angel of the satellite ----------------
                //[az(i), el(i), dist] = topocent(pos(1:3, :), Rot_X - pos(1:3, :));

            }

            //--- Apply the corrections ----------------------------------------
            omc(i) = (obs(i) - norm(Rot_X - pos.left(3)) - pos(4) - trop);
            //--- Construct the A matrix ---------------------------------------
            A.set(i, 0, (-(Rot_X(0) - pos(0))) / obs(i));
            A.set(i, 1, (-(Rot_X(1) - pos(1))) / obs(i));
            A.set(i, 2, (-(Rot_X(2) - pos(2))) / obs(i));
            A.set(i, 3, 1.0);
        }


        // These lines allow the code to exit gracefully in case of any errors
        //if (rank(A) != 4) {
        //  pos.clear();
        //  return pos;
        //}

        //--- Find position update ---------------------------------------------
        x = ls_solve_od(w*A,w*omc);

        //--- Apply position update --------------------------------------------
        pos = pos + x;

    }
    return pos;
}

void gps_l1_ca_ls_pvt::get_PVT(std::map<int,float> pseudoranges,double GPS_current_time)
{
    std::map<int,float>::iterator pseudoranges_iter;
    mat satpos;
    mat W=eye(d_nchannels); //channels weights matrix
    vec obs=zeros(d_nchannels); // pseudoranges observation vector
    satpos=zeros(3,d_nchannels); //satellite positions matrix
    int valid_obs=0; //valid observations counter
    for (int i=0; i<d_nchannels; i++)
    {
        if (d_ephemeris[i].satellite_validation()==true)
        {
            pseudoranges_iter=pseudoranges.find(d_ephemeris[i].d_satellite_PRN);
            if (pseudoranges_iter!=pseudoranges.end())
            {
                W(i,i)=1; // TODO: Place here the satellite CN0 (power level, or weight factor)
                // compute the GPS master clock
                d_ephemeris[i].master_clock(GPS_current_time);
                // compute the satellite current ECEF position
                d_ephemeris[i].satpos();
                // compute the clock error including relativistic effects
                d_ephemeris[i].relativistic_clock_correction(GPS_current_time);
                satpos(0,i)=d_ephemeris[i].d_satpos_X;
                satpos(1,i)=d_ephemeris[i].d_satpos_Y;
                satpos(2,i)=d_ephemeris[i].d_satpos_Z;
                obs(i)=pseudoranges_iter->second+d_ephemeris[i].d_satClkCorr*GPS_C_m_s;
                valid_obs++;
            }else{
                // no valid pseudorange for the current channel
                W(i,i)=0; // channel de-activated
                obs(i)=1; // to avoid algorithm problems (divide by zero)
            }
        }else{
            // no valid ephemeris for the current channel
            W(i,i)=0; // channel de-activated
            obs(i)=1; // to avoid algorithm problems (divide by zero)
        }
    }
    std::cout<<"PVT: valid observations="<<valid_obs<<std::endl;
    if (valid_obs>=4)
    {
        vec mypos;
        mypos=leastSquarePos(satpos,obs,W);
        std::cout << "Position ("<<GPS_current_time<<") ECEF = " << mypos << std::endl;
        cart2geo(mypos(0), mypos(1), mypos(2), 4);
        std::cout << "Position ("<<GPS_current_time<<") Lat = " << d_latitude_d << " Long ="<< d_longitude_d <<" Height="<<d_height_m<< std::endl;
    }
}
void gps_l1_ca_ls_pvt::cart2geo(double X, double Y, double Z, int elipsoid_selection)
{
    //function [phi, lambda, h] = cart2geo(X, Y, Z, i)
    //CART2GEO Conversion of Cartesian coordinates (X,Y,Z) to geographical
    //coordinates (phi, lambda, h) on a selected reference ellipsoid.
    //
    //[phi, lambda, h] = cart2geo(X, Y, Z, i);
    //
    //   Choices i of Reference Ellipsoid for Geographical Coordinates
    //             0. International Ellipsoid 1924
    //             1. International Ellipsoid 1967
    //             2. World Geodetic System 1972
    //             3. Geodetic Reference System 1980
    //             4. World Geodetic System 1984

    double a[5] = {6378388, 6378160, 6378135, 6378137, 6378137};
    double f[5] = {1/297, 1/298.247, 1/298.26, 1/298.257222101, 1/298.257223563};

    double lambda;
    lambda = atan2(Y,X);
    double ex2;
    ex2 = (2-f[elipsoid_selection])*f[elipsoid_selection]/((1-f[elipsoid_selection])*(1-f[elipsoid_selection]));
    double c;
    c = a[elipsoid_selection]*sqrt(1+ex2);
    double phi;
    phi = atan(Z/((sqrt(X*X+Y*Y)*(1-(2-f[elipsoid_selection]))*f[elipsoid_selection])));

    double h = 0.1;
    double oldh = 0;
    double N;
    int iterations = 0;
    do{
       oldh = h;
       N = c/sqrt(1+ex2*(cos(phi)*cos(phi)));
       phi = atan(Z/((sqrt(X*X+Y*Y)*(1-(2-f[elipsoid_selection])*f[elipsoid_selection]*N/(N+h)))));
       h = sqrt(X*X+Y*Y)/cos(phi)-N;
       iterations = iterations + 1;
       if (iterations > 100)
       {
           std::cout<<"Failed to approximate h with desired precision. h-oldh= "<<h-oldh<<std::endl;
           break;
       }
    }while (abs(h-oldh) > 1.0e-12);
    d_latitude_d = phi*180.0/GPS_PI;
    d_longitude_d = lambda*180/pi;
    d_height_m = h;
}

//void gps_l1_ca_ls_pvt::topocent(traveltime, X_sat)
//{
    /*
%function [Az, El, D] = topocent(X, dx)
%TOPOCENT  Transformation of vector dx into topocentric coordinate
%          system with origin at X.
%          Both parameters are 3 by 1 vectors.
%
%[Az, El, D] = topocent(X, dx);
%
%   Inputs:
%       X           - vector origin corrdinates (in ECEF system [X; Y; Z;])
%       dx          - vector ([dX; dY; dZ;]).
%
%   Outputs:
%       D           - vector length. Units like units of the input
%       Az          - azimuth from north positive clockwise, degrees
%       El          - elevation angle, degrees


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
*/
//}
