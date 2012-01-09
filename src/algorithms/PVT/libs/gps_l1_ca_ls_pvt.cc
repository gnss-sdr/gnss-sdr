/*!
 * \file gps_l1_ca_ls_pvt.cc
 * \brief Implementation of a Least Squares Position, Velocity, and Time
 * (PVT) solver, based on K.Borre's Matlab receiver.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "armadillo"
#include "gps_l1_ca_ls_pvt.h"
#include "GPS_L1_CA.h"
#include <glog/log_severity.h>
#include <glog/logging.h>
#include "boost/date_time/posix_time/posix_time.hpp"

using google::LogMessage;

gps_l1_ca_ls_pvt::gps_l1_ca_ls_pvt(int nchannels,std::string dump_filename, bool flag_dump_to_file)
{
    // init empty ephemeris for all the available GNSS channels
    d_nchannels=nchannels;
    d_ephemeris=new gps_navigation_message[nchannels];
    d_dump_filename=dump_filename;
    d_flag_dump_enabled=flag_dump_to_file;
    d_averaging_depth=0;

    // ############# ENABLE DATA FILE LOG #################
    if (d_flag_dump_enabled==true)
        {
            if (d_dump_file.is_open()==false)
                {
                    try {
                            d_dump_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            std::cout<<"PVT lib dump enabled Log file: "<<d_dump_filename.c_str()<<std::endl;
                    }
                    catch (std::ifstream::failure e) {
                            std::cout << "Exception opening PVT lib dump file "<<e.what()<<"\r\n";
                    }
                }
        }


}

void gps_l1_ca_ls_pvt::set_averaging_depth(int depth)
{
    d_averaging_depth=depth;
}
gps_l1_ca_ls_pvt::~gps_l1_ca_ls_pvt()
{
    d_dump_file.close();
    delete[] d_ephemeris;
}

arma::vec gps_l1_ca_ls_pvt::e_r_corr(double traveltime, arma::vec X_sat) {
    /*
     *  Returns rotated satellite ECEF coordinates due to Earth
     * rotation during signal travel time
     *
     *   Inputs:
     *       travelTime  - signal travel time
     *       X_sat       - satellite's ECEF coordinates
     *
     *   Returns:
     *       X_sat_rot   - rotated satellite's coordinates (ECEF)
     */

    //const double Omegae_dot = 7.292115147e-5; //  rad/sec

    //--- Find rotation angle --------------------------------------------------
    double omegatau;
    omegatau = OMEGA_EARTH_DOT * traveltime;

    //--- Make a rotation matrix -----------------------------------------------
    arma::mat R3=arma::zeros(3,3);
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
    arma::vec X_sat_rot;
    X_sat_rot = R3 * X_sat;
    return X_sat_rot;

}
arma::vec gps_l1_ca_ls_pvt::leastSquarePos(arma::mat satpos, arma::vec obs, arma::mat w) {
    /*Function calculates the Least Square Solution.
     *   Inputs:
     *       satpos      - Satellites positions in ECEF system: [X; Y; Z;]
     *       obs         - Observations - the pseudorange measurements to each
     *                   satellite
     *       w           - weigths vector
     *
     *   Returns:
     *       pos         - receiver position and receiver clock error
     *                   (in ECEF system: [X, Y, Z, dt])
     */

    //=== Initialization =======================================================
    int nmbOfIterations = 10; // TODO: include in config

    //double dtr = GPS_PI / 180.0;

    int nmbOfSatellites;

    nmbOfSatellites = satpos.n_cols;	//Armadillo
    arma::vec pos = "0.0 0.0 0.0 0.0";
    arma::mat A;
    arma::mat omc;
    arma::mat az;
    arma::mat el;
    A=arma::zeros(nmbOfSatellites,4);
    omc=arma::zeros(nmbOfSatellites,1);
    az=arma::zeros(1,nmbOfSatellites);
    el=arma::zeros(1,nmbOfSatellites);
    for (int i = 0; i < nmbOfSatellites; i++) {
            for (int j = 0; j < 4; j++) {
                    A(i,j)=0.0; //Armadillo
            }
            omc(i, 0)=0.0;
            az(0, i)=0.0;
    }
    el = az;

    arma::mat X = satpos;
    arma::vec Rot_X;
    double rho2;
    double traveltime;
    double trop;
    arma::mat mat_tmp;
    arma::vec x;
    //=== Iteratively find receiver position ===================================
    for (int iter = 0; iter < nmbOfIterations; iter++) {
            for (int i = 0; i < nmbOfSatellites; i++) {
                    if (iter == 0) {
                            //--- Initialize variables at the first iteration --------------
                            Rot_X=X.col(i); //Armadillo
                            trop = 0.0;
                    } else {
                            //--- Update equations -----------------------------------------
                            rho2 = (X(0, i) - pos(0))*(X(0, i) - pos(0))  + (X(1, i) - pos(1))*(X(1, i) - pos(1))+ (X(2,i) - pos(2))*(X(2,i) - pos(2));
                            traveltime = sqrt(rho2) / GPS_C_m_s;
                            //--- Correct satellite position (do to earth rotation) --------

                            Rot_X = e_r_corr(traveltime, X.col(i)); //armadillo
                            //--- Find the elevation angel of the satellite ----------------
                            //[az(i), el(i), dist] = topocent(pos(1:3, :), Rot_X - pos(1:3, :));

                    }
                    //--- Apply the corrections ----------------------------------------
                    omc(i) = (obs(i) - norm(Rot_X - pos.subvec(0,2),2) - pos(3) - trop); // Armadillo

                    //--- Construct the A matrix ---------------------------------------
                    //Armadillo
                    A(i,0)=(-(Rot_X(0) - pos(0))) / obs(i);
                    A(i,1)=(-(Rot_X(1) - pos(1))) / obs(i);
                    A(i,2)=(-(Rot_X(2) - pos(2))) / obs(i);
                    A(i,3)=1.0;
            }


            // These lines allow the code to exit gracefully in case of any errors
            //if (rank(A) != 4) {
            //  pos.clear();
            //  return pos;
            //}

            //--- Find position update ---------------------------------------------
            x = arma::solve(w*A,w*omc); // Armadillo

            //--- Apply position update --------------------------------------------
            pos = pos + x;

    }
    return pos;
}

bool gps_l1_ca_ls_pvt::get_PVT(std::map<int,gnss_pseudorange> gnss_pseudoranges_map,double GPS_current_time,bool flag_averaging)
{
    std::map<int,gnss_pseudorange>::iterator gnss_pseudoranges_iter;

    arma::mat W=arma::eye(d_nchannels,d_nchannels); //channels weights matrix
    arma::vec obs=arma::zeros(d_nchannels); // pseudoranges observation vector
    arma::mat satpos=arma::zeros(3,d_nchannels); //satellite positions matrix

    int GPS_week = 0;
    double GPS_corrected_time = 0;
    double utc = 0;

    int valid_obs=0; //valid observations counter
    for (int i=0; i<d_nchannels; i++)
        {
            if (d_ephemeris[i].satellite_validation()==true)
                {
                    gnss_pseudoranges_iter=gnss_pseudoranges_map.find(d_ephemeris[i].d_satellite_PRN);
                    if (gnss_pseudoranges_iter!=gnss_pseudoranges_map.end())
                        {
                            /*!
                             * \todo Place here the satellite CN0 (power level, or weight factor)
                             */
                            W(i,i)=1;
                            // compute the GPS master clock
                            // d_ephemeris[i].master_clock(GPS_current_time); ?????

                             // compute the clock error including relativistic effects
                            GPS_corrected_time = d_ephemeris[i].sv_clock_correction(GPS_current_time);
                            GPS_week = d_ephemeris[i].i_GPS_week;

                            utc =d_ephemeris[i].utc_time(GPS_corrected_time);

                            // compute the satellite current ECEF position
                            d_ephemeris[i].satellitePosition(GPS_corrected_time);

                            satpos(0,i)=d_ephemeris[i].d_satpos_X;
                            satpos(1,i)=d_ephemeris[i].d_satpos_Y;
                            satpos(2,i)=d_ephemeris[i].d_satpos_Z;
                            LOG_AT_LEVEL(INFO)<<"ECEF satellite SV ID="<<d_ephemeris[i].d_satellite_PRN<<" X="<<d_ephemeris[i].d_satpos_X
                                    <<" [m] Y="<<d_ephemeris[i].d_satpos_Y<<" [m] Z="<<d_ephemeris[i].d_satpos_Z<<" [m]\r\n";
                            obs(i)=gnss_pseudoranges_iter->second.pseudorange_m+d_ephemeris[i].d_satClkCorr*GPS_C_m_s;
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
            arma::vec mypos;
            mypos=leastSquarePos(satpos,obs,W);
            LOG_AT_LEVEL(INFO) << "Position at TOW="<<GPS_current_time<<" in ECEF (X,Y,Z) = " << mypos << std::endl;
            cart2geo(mypos(0), mypos(1), mypos(2), 4);

            // Compute UTC time and print PVT solution
            boost::posix_time::time_duration t = boost::posix_time::seconds(utc + 604800*(double)GPS_week);
            boost::posix_time::ptime p_time(boost::gregorian::date(1999,8,22),t);
            std::cout << "Position at "<<boost::posix_time::to_simple_string(p_time)<<" is Lat = " << d_latitude_d << " [deg] Long = "<< d_longitude_d <<" [deg] Height= "<<d_height_m<<" [m]" <<std::endl;
            // ######## LOG FILE #########
            if(d_flag_dump_enabled==true) {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try {
                            double tmp_double;
                            //  PVT GPS time
                            tmp_double=GPS_current_time;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // ECEF User Position East [m]
                            tmp_double=mypos(0);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // ECEF User Position North [m]
                            tmp_double=mypos(1);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // ECEF User Position Up [m]
                            tmp_double=mypos(2);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // User clock offset [s]
                            tmp_double=mypos(3);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // GEO user position Latitude [deg]
                            tmp_double=d_latitude_d;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // GEO user position Longitude [deg]
                            tmp_double=d_longitude_d;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // GEO user position Height [m]
                            tmp_double=d_height_m;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                    }
                    catch (std::ifstream::failure e) {
                            std::cout << "Exception writing PVT LS dump file "<<e.what()<<"\r\n";
                    }
            }

            // MOVING AVERAGE PVT
            if (flag_averaging==true)
                {
                    if (d_hist_longitude_d.size()==(unsigned int)d_averaging_depth)
                        {
                            // Pop oldest value
                            d_hist_longitude_d.pop_back();
                            d_hist_latitude_d.pop_back();
                            d_hist_height_m.pop_back();
                            // Push new values
                            d_hist_longitude_d.push_front(d_longitude_d);
                            d_hist_latitude_d.push_front(d_latitude_d);
                            d_hist_height_m.push_front(d_height_m);

                            d_avg_latitude_d=0;
                            d_avg_longitude_d=0;
                            d_avg_height_m=0;
                            for (unsigned int i=0;i<d_hist_longitude_d.size();i++)
                                {
                                    d_avg_latitude_d=d_avg_latitude_d+d_hist_latitude_d.at(i);
                                    d_avg_longitude_d=d_avg_longitude_d+d_hist_longitude_d.at(i);
                                    d_avg_height_m=d_avg_height_m+d_hist_height_m.at(i);
                                }
                            d_avg_latitude_d=d_avg_latitude_d/(double)d_averaging_depth;
                            d_avg_longitude_d=d_avg_longitude_d/(double)d_averaging_depth;
                            d_avg_height_m=d_avg_height_m/(double)d_averaging_depth;
                            return true; //indicates that the returned position is valid
                        }else{
                                //int current_depth=d_hist_longitude_d.size();
                                // Push new values
                                d_hist_longitude_d.push_front(d_longitude_d);
                                d_hist_latitude_d.push_front(d_latitude_d);
                                d_hist_height_m.push_front(d_height_m);

                                d_avg_latitude_d=d_latitude_d;
                                d_avg_longitude_d=d_longitude_d;
                                d_avg_height_m=d_height_m;
                                return false;//indicates that the returned position is not valid yet
                                // output the average, although it will not have the full historic available
                                //    			d_avg_latitude_d=0;
                                //    			d_avg_longitude_d=0;
                                //    			d_avg_height_m=0;
                                //    		    for (unsigned int i=0;i<d_hist_longitude_d.size();i++)
                                //    		    {
                                //    		    	d_avg_latitude_d=d_avg_latitude_d+d_hist_latitude_d.at(i);
                                //    		        d_avg_longitude_d=d_avg_longitude_d+d_hist_longitude_d.at(i);
                                //    		    	d_avg_height_m=d_avg_height_m+d_hist_height_m.at(i);
                                //    		    }
                                //    		    d_avg_latitude_d=d_avg_latitude_d/(double)current_depth;
                                //    		    d_avg_longitude_d=d_avg_longitude_d/(double)current_depth;
                                //    		    d_avg_height_m=d_avg_height_m/(double)current_depth;
                        }
                }else{
                        return true;//indicates that the returned position is valid
                }
        }else{
                return false;
        }
}


void gps_l1_ca_ls_pvt::cart2geo(double X, double Y, double Z, int elipsoid_selection)
{
    // Conversion of Cartesian coordinates (X,Y,Z) to geographical
    // coordinates (latitude, longitude, h) on a selected reference ellipsoid.
    //
    //
    //   Choices of Reference Ellipsoid for Geographical Coordinates
    //             0. International Ellipsoid 1924
    //             1. International Ellipsoid 1967
    //             2. World Geodetic System 1972
    //             3. Geodetic Reference System 1980
    //             4. World Geodetic System 1984

    const double a[5] = {6378388, 6378160, 6378135, 6378137, 6378137};
    const double f[5] = {1/297, 1/298.247, 1/298.26, 1/298.257222101, 1/298.257223563};

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
    d_longitude_d = lambda*180/GPS_PI;
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
