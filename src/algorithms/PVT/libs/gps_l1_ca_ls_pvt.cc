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

#include "gnss_synchro.h"

using google::LogMessage;

gps_l1_ca_ls_pvt::gps_l1_ca_ls_pvt(int nchannels,std::string dump_filename, bool flag_dump_to_file)
{
    // init empty ephemeris for all the available GNSS channels
    d_nchannels = nchannels;
    d_ephemeris = new Gps_Navigation_Message[nchannels];
    d_dump_filename = dump_filename;
    d_flag_dump_enabled = flag_dump_to_file;
    d_averaging_depth = 0;
    d_GPS_current_time=0;;
    b_valid_position=false;
    // ############# ENABLE DATA FILE LOG #################
    if (d_flag_dump_enabled == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                    {
                            d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            std::cout << "PVT lib dump enabled Log file: "<< d_dump_filename.c_str() << std::endl;
                    }
                    catch (std::ifstream::failure e)
                    {
                            std::cout << "Exception opening PVT lib dump file "<< e.what() << std::endl;
                    }
                }
        }
}


void gps_l1_ca_ls_pvt::set_averaging_depth(int depth)
{
    d_averaging_depth = depth;
}


gps_l1_ca_ls_pvt::~gps_l1_ca_ls_pvt()
{
    d_dump_file.close();
    delete[] d_ephemeris;
}


arma::vec gps_l1_ca_ls_pvt::e_r_corr(double traveltime, arma::vec X_sat)
{
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
    arma::mat R3 = arma::zeros(3,3);
    R3(0, 0) = cos(omegatau);
    R3(0, 1) = sin(omegatau);
    R3(0, 2) = 0.0;
    R3(1, 0) = -sin(omegatau);
    R3(1, 1) = cos(omegatau);
    R3(1, 2) = 0.0;
    R3(2, 0) = 0.0;
    R3(2, 1) = 0.0;
    R3(2, 2) = 1;
    //--- Do the rotation ------------------------------------------------------
    arma::vec X_sat_rot;
    X_sat_rot = R3 * X_sat;
    return X_sat_rot;
}


arma::vec gps_l1_ca_ls_pvt::leastSquarePos(arma::mat satpos, arma::vec obs, arma::mat w)
{
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
    A=arma::zeros(nmbOfSatellites, 4);
    omc=arma::zeros(nmbOfSatellites, 1);
    az=arma::zeros(1, nmbOfSatellites);
    el=arma::zeros(1, nmbOfSatellites);
    for (int i = 0; i < nmbOfSatellites; i++)
        {
            for (int j = 0; j < 4; j++)
                {
                    A(i, j) = 0.0; //Armadillo
                }
            omc(i, 0) = 0.0;
            az(0, i)= 0.0;
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
    for (int iter = 0; iter < nmbOfIterations; iter++)
        {
            for (int i = 0; i < nmbOfSatellites; i++)
                {
                    if (iter == 0)
                        {
                            //--- Initialize variables at the first iteration --------------
                            Rot_X = X.col(i); //Armadillo
                            trop = 0.0;
                        }
                    else
                        {
                            //--- Update equations -----------------------------------------
                            rho2 = (X(0, i) - pos(0))*(X(0, i) - pos(0))  + (X(1, i) - pos(1))*(X(1, i) - pos(1))+ (X(2,i) - pos(2))*(X(2,i) - pos(2));
                            traveltime = sqrt(rho2) / GPS_C_m_s;
                            //--- Correct satellite position (do to earth rotation) --------

                            Rot_X = e_r_corr(traveltime, X.col(i)); //armadillo
                            //--- Find the elevation angle of the satellite ----------------

                            topocent(&d_visible_satellites_Az[i],&d_visible_satellites_El[i],&d_visible_satellites_Distance[i],pos.subvec(0,2), Rot_X - pos.subvec(0,2));

                            //[az(i), el(i), dist] = topocent(pos(1:3, :), Rot_X - pos(1:3, :));

                        }
                    //--- Apply the corrections ----------------------------------------
                    omc(i) = (obs(i) - norm(Rot_X - pos.subvec(0,2),2) - pos(3) - trop); // Armadillo

                    //--- Construct the A matrix ---------------------------------------
                    //Armadillo
                    A(i,0) = (-(Rot_X(0) - pos(0))) / obs(i);
                    A(i,1) = (-(Rot_X(1) - pos(1))) / obs(i);
                    A(i,2) = (-(Rot_X(2) - pos(2))) / obs(i);
                    A(i,3) = 1.0;
                }


            // These lines allow the code to exit gracefully in case of any errors
            //if (rank(A) != 4) {
            //  pos.clear();
            //  return pos;
            //}

            //--- Find position update ---------------------------------------------
            x = arma::solve(w*A, w*omc); // Armadillo

            //--- Apply position update --------------------------------------------
            pos = pos + x;
        }

    try{
    //-- compute the Dilution Of Precision values
		arma::mat Q;
		Q       = arma::inv(arma::htrans(A)*A);
		//std::cout<<Q<<std::endl;

		d_GDOP  = sqrt(arma::trace(Q));                 // GDOP
		d_PDOP  = sqrt(Q(0,0) + Q(1,1) + Q(2,2));       // PDOP
		d_HDOP  = sqrt(Q(0,0) + Q(1,1));                // HDOP
		d_VDOP  = sqrt(Q(2,2));                         // VDOP
		d_TDOP  = sqrt(Q(3,3));                         // TDOP
    }catch(std::exception e)
    {
    	d_GDOP  = -1;
		d_PDOP  = -1;
		d_HDOP  = -1;
		d_VDOP  = -1;
		d_TDOP	= -1;
    }
    return pos;
}


bool gps_l1_ca_ls_pvt::get_PVT(std::map<int,Gnss_Synchro> gnss_pseudoranges_map,double GPS_current_time,bool flag_averaging)
{
    std::map<int,Gnss_Synchro>::iterator gnss_pseudoranges_iter;

    arma::mat W = arma::eye(d_nchannels,d_nchannels); //channels weights matrix
    arma::vec obs = arma::zeros(d_nchannels); // pseudoranges observation vector
    arma::mat satpos = arma::zeros(3,d_nchannels); //satellite positions matrix

    int GPS_week = 0;
    double GPS_corrected_time = 0;
    double utc = 0;

    d_flag_averaging=flag_averaging;

    int valid_obs=0; //valid observations counter
    for (int i=0; i<d_nchannels; i++)
        {
            if (d_ephemeris[i].satellite_validation() == true)
                {
                    gnss_pseudoranges_iter = gnss_pseudoranges_map.find(d_ephemeris[i].i_satellite_PRN);
                    if (gnss_pseudoranges_iter != gnss_pseudoranges_map.end())
                        {
                            /*!
                             * \todo Place here the satellite CN0 (power level, or weight factor)
                             */
                            W(i,i) = 1;
                            // compute the GPS master clock
                            // d_ephemeris[i].master_clock(GPS_current_time); ?????

                            // compute the clock error including relativistic effects
                            GPS_corrected_time = d_ephemeris[i].sv_clock_correction(GPS_current_time);
                            GPS_week = d_ephemeris[i].i_GPS_week;

                            utc = d_ephemeris[i].utc_time(GPS_corrected_time);

                            // compute the satellite current ECEF position
                            d_ephemeris[i].satellitePosition(GPS_corrected_time);

                            satpos(0,i) = d_ephemeris[i].d_satpos_X;
                            satpos(1,i) = d_ephemeris[i].d_satpos_Y;
                            satpos(2,i) = d_ephemeris[i].d_satpos_Z;
                            DLOG(INFO) << "ECEF satellite SV ID=" << d_ephemeris[i].i_satellite_PRN <<" X=" << d_ephemeris[i].d_satpos_X
                                    << " [m] Y=" << d_ephemeris[i].d_satpos_Y << " [m] Z=" << d_ephemeris[i].d_satpos_Z << " [m]" << std::endl;
                            obs(i) = gnss_pseudoranges_iter->second.Pseudorange_m + d_ephemeris[i].d_satClkCorr*GPS_C_m_s;
                            d_visible_satellites_IDs[valid_obs]=d_ephemeris[i].i_satellite_PRN;
                            d_visible_satellites_CN0_dB[valid_obs]=gnss_pseudoranges_iter->second.CN0_dB_hz;
                            valid_obs++;
                        }
                    else
                        {
                            // no valid pseudorange for the current channel
                            W(i,i) = 0; // channel de-activated
                            obs(i) = 1; // to avoid algorithm problems (divide by zero)
                        }
                }
            else
                {
                    // no valid ephemeris for the current channel
                    W(i,i) = 0; // channel de-activated
                    obs(i) = 1; // to avoid algorithm problems (divide by zero)
                }
        }

    d_valid_observations = valid_obs;

    DLOG(INFO) <<"PVT: valid observations="<<valid_obs<<std::endl;

    if (valid_obs>=4)
        {
            arma::vec mypos;
            mypos=leastSquarePos(satpos,obs,W);
            DLOG(INFO) << "Position at TOW="<< GPS_current_time << " in ECEF (X,Y,Z) = " << mypos << std::endl;
            cart2geo(mypos(0), mypos(1), mypos(2), 4);

            // Compute UTC time and print PVT solution
            boost::posix_time::time_duration t = boost::posix_time::seconds(utc + 604800.0*(double)GPS_week);
            boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
            d_position_UTC_time = p_time;
            GPS_current_time=GPS_corrected_time;

            DLOG(INFO)<< "Position at " << boost::posix_time::to_simple_string(p_time)
            << " is Lat = " << d_latitude_d << " [deg], Long = " << d_longitude_d
            << " [deg], Height= " << d_height_m << " [m]" << std::endl;

            // ######## LOG FILE #########
            if(d_flag_dump_enabled == true)
                {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try
                    {
                            double tmp_double;
                            //  PVT GPS time
                            tmp_double = GPS_current_time;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // ECEF User Position East [m]
                            tmp_double = mypos(0);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // ECEF User Position North [m]
                            tmp_double = mypos(1);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // ECEF User Position Up [m]
                            tmp_double = mypos(2);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // User clock offset [s]
                            tmp_double = mypos(3);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // GEO user position Latitude [deg]
                            tmp_double = d_latitude_d;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // GEO user position Longitude [deg]
                            tmp_double = d_longitude_d;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            // GEO user position Height [m]
                            tmp_double = d_height_m;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                    }
                    catch (std::ifstream::failure e)
                    {
                            std::cout << "Exception writing PVT LS dump file "<<e.what()<<"\r\n";
                    }
                }

            // MOVING AVERAGE PVT
            if (flag_averaging == true)
                {
                    if (d_hist_longitude_d.size() == (unsigned int)d_averaging_depth)
                        {
                            // Pop oldest value
                            d_hist_longitude_d.pop_back();
                            d_hist_latitude_d.pop_back();
                            d_hist_height_m.pop_back();
                            // Push new values
                            d_hist_longitude_d.push_front(d_longitude_d);
                            d_hist_latitude_d.push_front(d_latitude_d);
                            d_hist_height_m.push_front(d_height_m);

                            d_avg_latitude_d = 0;
                            d_avg_longitude_d = 0;
                            d_avg_height_m = 0;
                            for (unsigned int i=0; i<d_hist_longitude_d.size(); i++)
                                {
                                    d_avg_latitude_d = d_avg_latitude_d + d_hist_latitude_d.at(i);
                                    d_avg_longitude_d = d_avg_longitude_d + d_hist_longitude_d.at(i);
                                    d_avg_height_m  = d_avg_height_m + d_hist_height_m.at(i);
                                }
                            d_avg_latitude_d = d_avg_latitude_d / (double)d_averaging_depth;
                            d_avg_longitude_d = d_avg_longitude_d / (double)d_averaging_depth;
                            d_avg_height_m = d_avg_height_m / (double)d_averaging_depth;
                            b_valid_position=true;
                            return true; //indicates that the returned position is valid
                        }
                    else
                        {
                            //int current_depth=d_hist_longitude_d.size();
                            // Push new values
                            d_hist_longitude_d.push_front(d_longitude_d);
                            d_hist_latitude_d.push_front(d_latitude_d);
                            d_hist_height_m.push_front(d_height_m);

                            d_avg_latitude_d = d_latitude_d;
                            d_avg_longitude_d = d_longitude_d;
                            d_avg_height_m = d_height_m;
                            b_valid_position=false;
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
                }
            else
                {
            	b_valid_position=true;
                    return true;//indicates that the returned position is valid
                }
        }
    else
        {
    	b_valid_position=false;
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
    ex2 = (2 - f[elipsoid_selection]) * f[elipsoid_selection] / ((1 - f[elipsoid_selection])*(1 -f[elipsoid_selection]));
    double c;
    c = a[elipsoid_selection] * sqrt(1+ex2);
    double phi;
    phi = atan(Z / ((sqrt(X*X + Y*Y)*(1 - (2 - f[elipsoid_selection])) * f[elipsoid_selection])));

    double h = 0.1;
    double oldh = 0;
    double N;
    int iterations = 0;
    do
        {
            oldh = h;
            N = c / sqrt(1 + ex2 * (cos(phi) * cos(phi)));
            phi = atan(Z / ((sqrt(X*X + Y*Y) * (1 - (2 -f[elipsoid_selection]) * f[elipsoid_selection] *N / (N + h) ))));
            h = sqrt(X*X + Y*Y) / cos(phi) - N;
            iterations = iterations + 1;
            if (iterations > 100)
                {
                    std::cout << "Failed to approximate h with desired precision. h-oldh= " << h-oldh << std::endl;
                    break;
                }
        }
    while (abs(h - oldh) > 1.0e-12);
    d_latitude_d = phi * 180.0 / GPS_PI;
    d_longitude_d = lambda * 180 / GPS_PI;
    d_height_m = h;
}

void gps_l1_ca_ls_pvt::togeod(double *dphi, double *dlambda, double *h, double a, double finv, double X, double Y, double Z)
{
	//function [dphi, dlambda, h] = togeod(a, finv, X, Y, Z)
	//%TOGEOD   Subroutine to calculate geodetic coordinates latitude, longitude,
	//%         height given Cartesian coordinates X,Y,Z, and reference ellipsoid
	//%         values semi-major axis (a) and the inverse of flattening (finv).
	//%
	//%[dphi, dlambda, h] = togeod(a, finv, X, Y, Z);
	//%
	//%  The units of linear parameters X,Y,Z,a must all agree (m,km,mi,ft,..etc)
	//%  The output units of angular quantities will be in decimal degrees
	//%  (15.5 degrees not 15 deg 30 min). The output units of h will be the
	//%  same as the units of X,Y,Z,a.
	//%
	//%   Inputs:
	//%       a           - semi-major axis of the reference ellipsoid
	//%       finv        - inverse of flattening of the reference ellipsoid
	//%       X,Y,Z       - Cartesian coordinates
	//%
	//%   Outputs:
	//%       dphi        - latitude
	//%       dlambda     - longitude
	//%       h           - height above reference ellipsoid
	//
	//%  Copyright (C) 1987 C. Goad, Columbus, Ohio
	//%  Reprinted with permission of author, 1996
	//%  Fortran code translated into MATLAB
	//%  Kai Borre 03-30-96
	//%
	//% CVS record:
	//% $Id: togeod.m,v 1.1.1.1.2.4 2006/08/22 13:45:59 dpl Exp $
	//%==========================================================================
	//
	*h       = 0;
	double tolsq = 1.e-10;
	int maxit=10;
	// compute radians-to-degree factor
	double rtd;
	rtd     = 180/GPS_PI;

	// compute square of eccentricity
	double esq;
	if (finv < 1.0E-20)
	{
		esq = 0;
	}else
	{
		esq = (2 - 1/finv) / finv;
	}

	double oneesq;

	oneesq  = 1 - esq;

	// first guess
	// P is distance from spin axis
	double P;
	P = sqrt(X*X+Y*Y);
	//direct calculation of longitude
	//
	if (P > 1.0E-20)
	{
		*dlambda = atan2(Y,X) * rtd;
	}else
	{
		*dlambda = 0;
	}

	if (*dlambda < 0)
	{
		*dlambda = *dlambda + 360.0;
	}

	// r is distance from origin (0,0,0)
	double r;
	r = sqrt(P*P + Z*Z);

	double sinphi;

	if (r > 1.0E-20)
	{
		sinphi = Z/r;
	} else
	{
		sinphi = 0;
	}

	*dphi = asin(sinphi);

	// initial value of height  =  distance from origin minus
	// approximate distance from origin to surface of ellipsoid
	if (r < 1.0E-20)
	{
		*h = 0;
		return;
	}

	*h = r - a*(1-sinphi*sinphi/finv);


	// iterate
	double cosphi;
	double N_phi;
	double dP;
	double dZ;
	for (int i=0; i<maxit; i++)
	{
		sinphi  = sin(*dphi);
		cosphi  = cos(*dphi);

		//     compute radius of curvature in prime vertical direction
		N_phi   = a/sqrt(1-esq*sinphi*sinphi);

		//    compute residuals in P and Z
		dP      = P - (N_phi + (*h)) * cosphi;
		dZ      = Z - (N_phi*oneesq + (*h)) * sinphi;
		//
		//    update height and latitude
		*h       = *h + (sinphi*dZ + cosphi*dP);
		*dphi    = *dphi + (cosphi*dZ - sinphi*dP)/(N_phi + (*h));

		//     test for convergence
		if ((dP*dP + dZ*dZ) < tolsq)
		{
			break;
		}

		//    Not Converged--Warn user
		//    if (i == (maxit-1))
		//        fprintf([' Problem in TOGEOD, did not converge in %2.0f',...
		//            ' iterations\n'], i);
		//    end
	}
	//
	*dphi = (*dphi) * rtd;

}
void gps_l1_ca_ls_pvt::topocent(double *Az, double *El, double *D, arma::vec x, arma::vec x_sat)
{

	//%function [Az, El, D] = topocent(X, dx)
	//%TOPOCENT  Transformation of vector dx into topocentric coordinate
	//%          system with origin at X.
	//%          Both parameters are 3 by 1 vectors.
	//%
	//%[Az, El, D] = topocent(X, dx);
	//%
	//%   Inputs:
	//%       X           - vector origin corrdinates (in ECEF system [X; Y; Z;])
	//%       dx          - vector ([dX; dY; dZ;]).
	//%
	//%   Outputs:
	//%       D           - vector length. Units like units of the input
	//%       Az          - azimuth from north positive clockwise, degrees
	//%       El          - elevation angle, degrees

	double dtr;
	double lambda;
	double phi;
	double cl;
	double sl;
	double cb;
	double sb;

	double h;

	dtr = GPS_PI/180.0;

	//[phi, lambda, h] = togeod(6378137, 298.257223563, X(1), X(2), X(3));

	togeod(&phi, &lambda, &h,6378137.0, 298.257223563, x(0), x(1), x(2));

	cl  = cos(lambda * dtr);
	sl  = sin(lambda * dtr);
	cb  = cos(phi * dtr);
	sb  = sin(phi * dtr);

	arma::mat F=arma::zeros(3,3);

	F(0,0)=-sl;
	F(0,1)=-sb*cl;
	F(0,2)=cb*cl;

	F(1,0)=cl;
	F(1,1)=-sb*sl;
	F(1,2)=cb*sl;

	F(2,0)=0;
	F(2,1)=cb;
	F(2,2)=sb;

	arma::vec local_vector;

	local_vector = arma::htrans(F) * x_sat;

	double E;
	double N;
	double U;

	E   = local_vector(0);
	N   = local_vector(1);
	U   = local_vector(2);

	double hor_dis;

	hor_dis = sqrt(E*E + N*N);

	if (hor_dis < 1.0E-20)
	{
		*Az = 0;
		*El = 90;
	}
	else
	{
		*Az = atan2(E, N)/dtr;
		*El = atan2(U, hor_dis)/dtr;
	}

	if (*Az < 0)
	{
		*Az = *Az + 360.0;
	}


	*D   = sqrt(x_sat(0)*x_sat(0) + x_sat(1)*x_sat(1) + x_sat(2)*x_sat(2));

}
