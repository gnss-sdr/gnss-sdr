/*!
 * \file gps_l1_ca_ls_pvt.cc
 * \brief Implementation of a Least Squares Position, Velocity, and Time
 * (PVT) solver, based on K.Borre's Matlab receiver.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
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


#include "gps_l1_ca_ls_pvt.h"
#include <gflags/gflags.h>
#include <glog/logging.h>


using google::LogMessage;

DEFINE_bool(tropo, true, "Apply tropospheric correction");

gps_l1_ca_ls_pvt::gps_l1_ca_ls_pvt(int nchannels, std::string dump_filename, bool flag_dump_to_file)
{
    // init empty ephemeris for all the available GNSS channels
    d_nchannels = nchannels;
    d_ephemeris = new Gps_Navigation_Message[nchannels];
    d_dump_filename = dump_filename;
    d_flag_dump_enabled = flag_dump_to_file;
    d_averaging_depth = 0;
    d_GPS_current_time = 0;
    b_valid_position = false;

    d_valid_observations = 0;
    d_latitude_d = 0.0;
    d_longitude_d = 0.0;
    d_height_m = 0.0;
    d_avg_latitude_d = 0.0;
    d_avg_longitude_d = 0.0;
    d_avg_height_m = 0.0;
    d_x_m = 0.0;
    d_y_m = 0.0;
    d_z_m = 0.0;
    d_GDOP = 0.0;
    d_PDOP = 0.0;
    d_HDOP = 0.0;
    d_VDOP = 0.0;
    d_TDOP = 0.0;
    d_flag_averaging = false;

    // ############# ENABLE DATA FILE LOG #################
    if (d_flag_dump_enabled == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                    {
                            d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "PVT lib dump enabled Log file: " << d_dump_filename.c_str();
                    }
                    catch (std::ifstream::failure e)
                    {
                            LOG(WARNING) << "Exception opening PVT lib dump file " << e.what();
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


arma::vec gps_l1_ca_ls_pvt::rotateSatellite(double traveltime, const arma::vec & X_sat)
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

    //--- Find rotation angle --------------------------------------------------
    double omegatau;
    omegatau = OMEGA_EARTH_DOT * traveltime;

    //--- Build a rotation matrix ----------------------------------------------
    arma::mat R3 = arma::zeros(3,3);
    R3(0, 0) = cos(omegatau);
    R3(0, 1) = sin(omegatau);
    R3(0, 2) = 0.0;
    R3(1, 0) = -sin(omegatau);
    R3(1, 1) = cos(omegatau);
    R3(1, 2) = 0.0;
    R3(2, 0) = 0.0;
    R3(2, 1) = 0.0;
    R3(2, 2) = 1.0;

    //--- Do the rotation ------------------------------------------------------
    arma::vec X_sat_rot;
    X_sat_rot = R3 * X_sat;
    return X_sat_rot;
}


arma::vec gps_l1_ca_ls_pvt::leastSquarePos(const arma::mat & satpos, const arma::vec & obs, const arma::mat & w)
{
    /* Computes the Least Squares Solution.
     *   Inputs:
     *       satpos      - Satellites positions in ECEF system: [X; Y; Z;]
     *       obs         - Observations - the pseudorange measurements to each satellite
     *       w           - weigths vector
     *
     *   Returns:
     *       pos         - receiver position and receiver clock error
     *                   (in ECEF system: [X, Y, Z, dt])
     */

    //=== Initialization =======================================================
    int nmbOfIterations = 10; // TODO: include in config
    int nmbOfSatellites;
    nmbOfSatellites = satpos.n_cols;	//Armadillo
    arma::vec pos = "0.0 0.0 0.0 0.0";
    arma::mat A;
    arma::mat omc;
    arma::mat az;
    arma::mat el;
    A = arma::zeros(nmbOfSatellites, 4);
    omc = arma::zeros(nmbOfSatellites, 1);
    az = arma::zeros(1, nmbOfSatellites);
    el = arma::zeros(1, nmbOfSatellites);
    arma::mat X = satpos;
    arma::vec Rot_X;
    double rho2;
    double traveltime;
    double trop = 0.0;
    double dlambda;
    double dphi;
    double h;
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
                            rho2 = (X(0, i) - pos(0)) * (X(0, i) - pos(0))
                                 + (X(1, i) - pos(1)) * (X(1, i) - pos(1))
                                 + (X(2, i) - pos(2)) * (X(2, i) - pos(2));
                            traveltime = sqrt(rho2) / GPS_C_m_s;

                            //--- Correct satellite position (do to earth rotation) --------
                            Rot_X = rotateSatellite(traveltime, X.col(i)); //armadillo

                            //--- Find satellites' DOA
                            topocent(&d_visible_satellites_Az[i], &d_visible_satellites_El[i],
                                    &d_visible_satellites_Distance[i], pos.subvec(0,2), Rot_X - pos.subvec(0,2));

                            if(FLAGS_tropo)
                                {                         
                                    if(traveltime < 0.1 && nmbOfSatellites > 3)
                                        {
                                            //--- Find receiver's height
                                            togeod(&dphi, &dlambda, &h, 6378137.0, 298.257223563, pos(0), pos(1), pos(2));

                                            //--- Find delay due to troposphere (in meters)
                                            tropo(&trop, sin(d_visible_satellites_El[i] * GPS_PI / 180.0), h / 1000, 1013.0, 293.0, 50.0, 0.0, 0.0, 0.0);
                                            if(trop > 50.0 ) trop = 0.0;
                                        }
                                }
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

            //--- Find position update ---------------------------------------------
            x = arma::solve(w*A, w*omc); // Armadillo

            //--- Apply position update --------------------------------------------
            pos = pos + x;
            if (arma::norm(x, 2) < 1e-4)
                {
                    break; // exit the loop because we assume that the LS algorithm has converged (err < 0.1 cm)
                }
        }

    try
    {
            //-- compute the Dilution Of Precision values
            d_Q = arma::inv(arma::htrans(A)*A);
    }
    catch(std::exception& e)
    {
            d_Q = arma::zeros(4, 4);
    }
    return pos;
}


bool gps_l1_ca_ls_pvt::get_PVT(std::map<int,Gnss_Synchro> gnss_pseudoranges_map, double GPS_current_time, bool flag_averaging)
{
    std::map<int,Gnss_Synchro>::iterator gnss_pseudoranges_iter;
    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
    int valid_pseudoranges = gnss_pseudoranges_map.size();

    arma::mat W = arma::eye(valid_pseudoranges, valid_pseudoranges); //channels weights matrix
    arma::vec obs = arma::zeros(valid_pseudoranges);                 // pseudoranges observation vector
    arma::mat satpos = arma::zeros(3, valid_pseudoranges);           //satellite positions matrix

    int GPS_week = 0;
    double utc = 0;
    double TX_time_corrected_s;
    double SV_clock_bias_s = 0;

    d_flag_averaging = flag_averaging;

    // ********************************************************************************
    // ****** PREPARE THE LEAST SQUARES DATA (SV POSITIONS MATRIX AND OBS VECTORS) ****
    // ********************************************************************************
    int valid_obs = 0; //valid observations counter
    int obs_counter = 0;
    for(gnss_pseudoranges_iter = gnss_pseudoranges_map.begin();
            gnss_pseudoranges_iter != gnss_pseudoranges_map.end();
            gnss_pseudoranges_iter++)
        {
            // 1- find the ephemeris for the current SV observation. The SV PRN ID is the map key
            gps_ephemeris_iter = gps_ephemeris_map.find(gnss_pseudoranges_iter->first);
            if (gps_ephemeris_iter != gps_ephemeris_map.end())
                {
                    /*!
                     * \todo Place here the satellite CN0 (power level, or weight factor)
                     */
                    W(obs_counter, obs_counter) = 1;

                    // COMMON RX TIME PVT ALGORITHM MODIFICATION (Like RINEX files)
                    // first estimate of transmit time
                    double Rx_time = GPS_current_time;
                    double Tx_time = Rx_time - gnss_pseudoranges_iter->second.Pseudorange_m / GPS_C_m_s;

                    // 2- compute the clock drift using the clock model (broadcast) for this SV, including relativistic effect
                    SV_clock_bias_s = gps_ephemeris_iter->second.sv_clock_drift(Tx_time); //- gps_ephemeris_iter->second.d_TGD;

                    // 3- compute the current ECEF position for this SV using corrected TX time
                    TX_time_corrected_s = Tx_time - SV_clock_bias_s;
                    gps_ephemeris_iter->second.satellitePosition(TX_time_corrected_s);

                    satpos(0, obs_counter) = gps_ephemeris_iter->second.d_satpos_X;
                    satpos(1, obs_counter) = gps_ephemeris_iter->second.d_satpos_Y;
                    satpos(2, obs_counter) = gps_ephemeris_iter->second.d_satpos_Z;

                    // 4- fill the observations vector with the corrected pseudoranges
                    obs(obs_counter) = gnss_pseudoranges_iter->second.Pseudorange_m + SV_clock_bias_s * GPS_C_m_s;
                    d_visible_satellites_IDs[valid_obs] = gps_ephemeris_iter->second.i_satellite_PRN;
                    d_visible_satellites_CN0_dB[valid_obs] = gnss_pseudoranges_iter->second.CN0_dB_hz;
                    valid_obs++;

                    // SV ECEF DEBUG OUTPUT
                    DLOG(INFO) << "(new)ECEF satellite SV ID=" << gps_ephemeris_iter->second.i_satellite_PRN
                            << " X=" << gps_ephemeris_iter->second.d_satpos_X
                            << " [m] Y=" << gps_ephemeris_iter->second.d_satpos_Y
                            << " [m] Z=" << gps_ephemeris_iter->second.d_satpos_Z
                            << " [m] PR_obs=" << obs(obs_counter) << " [m]";

                    // compute the UTC time for this SV (just to print the associated UTC timestamp)
                    GPS_week = gps_ephemeris_iter->second.i_GPS_week;
                    utc = gps_utc_model.utc_time(TX_time_corrected_s, GPS_week);
                }
            else // the ephemeris are not available for this SV
                {
                    // no valid pseudorange for the current SV
                    W(obs_counter, obs_counter) = 0; // SV de-activated
                    obs(obs_counter) = 1;            // to avoid algorithm problems (divide by zero)
                    DLOG(INFO) << "No ephemeris data for SV " << gnss_pseudoranges_iter->first;
                }
            obs_counter++;
        }

    // ********************************************************************************
    // ****** SOLVE LEAST SQUARES******************************************************
    // ********************************************************************************
    d_valid_observations = valid_obs;
    LOG(INFO) << "(new)PVT: valid observations=" << valid_obs;

    if (valid_obs >= 4)
        {
            arma::vec mypos;
            DLOG(INFO) << "satpos=" << satpos;
            DLOG(INFO) << "obs=" << obs;
            DLOG(INFO) << "W=" << W;
            mypos = leastSquarePos(satpos, obs, W);
            DLOG(INFO) << "(new)Position at TOW=" << GPS_current_time << " in ECEF (X,Y,Z) = " << mypos;
            gps_l1_ca_ls_pvt::cart2geo(mypos(0), mypos(1), mypos(2), 4);
            //ToDo: Find an Observables/PVT random bug with some satellite configurations that gives an erratic PVT solution (i.e. height>50 km)
            if (d_height_m > 50000)
            {
            	b_valid_position = false;
            	return false;
            }
            // Compute UTC time and print PVT solution
            double secondsperweek = 604800.0; // number of seconds in one week (7*24*60*60)
            boost::posix_time::time_duration t = boost::posix_time::seconds(utc + secondsperweek * static_cast<double>(GPS_week));
            // 22 August 1999 last GPS time roll over
            boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
            d_position_UTC_time = p_time;

            LOG(INFO) << "(new)Position at " << boost::posix_time::to_simple_string(p_time)
                      << " is Lat = " << d_latitude_d << " [deg], Long = " << d_longitude_d
                      << " [deg], Height= " << d_height_m << " [m]";

            // ###### Compute DOPs ########

            // 1- Rotation matrix from ECEF coordinates to ENU coordinates
            // ref: http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates
            arma::mat F=arma::zeros(3,3);
            F(0,0) = -sin(GPS_TWO_PI*(d_longitude_d/360.0));
            F(0,1) = -sin(GPS_TWO_PI*(d_latitude_d/360.0))*cos(GPS_TWO_PI*(d_longitude_d/360.0));
            F(0,2) = cos(GPS_TWO_PI*(d_latitude_d/360.0))*cos(GPS_TWO_PI*(d_longitude_d/360.0));

            F(1,0) = cos((GPS_TWO_PI*d_longitude_d)/360.0);
            F(1,1) = -sin((GPS_TWO_PI*d_latitude_d)/360.0)*sin((GPS_TWO_PI*d_longitude_d)/360.0);
            F(1,2) = cos((GPS_TWO_PI*d_latitude_d/360.0))*sin((GPS_TWO_PI*d_longitude_d)/360.0);

            F(2,0) = 0;
            F(2,1) = cos((GPS_TWO_PI*d_latitude_d)/360.0);
            F(2,2) = sin((GPS_TWO_PI*d_latitude_d/360.0));

            // 2- Apply the rotation to the latest covariance matrix (available in ECEF from LS)
            arma::mat Q_ECEF = d_Q.submat(0, 0, 2, 2);
            arma::mat DOP_ENU = arma::zeros(3, 3);

            try
            {
                    DOP_ENU = arma::htrans(F)*Q_ECEF*F;
                    d_GDOP = sqrt(arma::trace(DOP_ENU));                         // Geometric DOP
                    d_PDOP = sqrt(DOP_ENU(0, 0) + DOP_ENU(1, 1) + DOP_ENU(2, 2));// PDOP
                    d_HDOP = sqrt(DOP_ENU(0, 0) + DOP_ENU(1, 1));                // HDOP
                    d_VDOP = sqrt(DOP_ENU(2, 2));                                // VDOP
                    d_TDOP = sqrt(d_Q(3, 3));	                                 // TDOP
            }
            catch(std::exception& ex)
            {
                    d_GDOP = -1; // Geometric DOP
                    d_PDOP = -1; // PDOP
                    d_HDOP = -1; // HDOP
                    d_VDOP = -1; // VDOP
                    d_TDOP = -1; // TDOP
            }

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
                            LOG(WARNING) << "Exception writing PVT LS dump file " << e.what();
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
                            for (unsigned int i = 0; i < d_hist_longitude_d.size(); i++)
                                {
                                    d_avg_latitude_d = d_avg_latitude_d + d_hist_latitude_d.at(i);
                                    d_avg_longitude_d = d_avg_longitude_d + d_hist_longitude_d.at(i);
                                    d_avg_height_m  = d_avg_height_m + d_hist_height_m.at(i);
                                }
                            d_avg_latitude_d = d_avg_latitude_d / static_cast<double>(d_averaging_depth);
                            d_avg_longitude_d = d_avg_longitude_d / static_cast<double>(d_averaging_depth);
                            d_avg_height_m = d_avg_height_m / static_cast<double>(d_averaging_depth);
                            b_valid_position = true;
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
                            b_valid_position = false;
                            return false; //indicates that the returned position is not valid yet
                        }
                }
            else
                {
                    b_valid_position = true;
                    return true; //indicates that the returned position is valid
                }
        }
    else
        {
            b_valid_position = false;
            return false;
        }
}


void gps_l1_ca_ls_pvt::cart2geo(double X, double Y, double Z, int elipsoid_selection)
{
    /* Conversion of Cartesian coordinates (X,Y,Z) to geographical
     coordinates (latitude, longitude, h) on a selected reference ellipsoid.

       Choices of Reference Ellipsoid for Geographical Coordinates
                 0. International Ellipsoid 1924
                 1. International Ellipsoid 1967
                 2. World Geodetic System 1972
                 3. Geodetic Reference System 1980
                 4. World Geodetic System 1984
     */

    const double a[5] = {6378388.0, 6378160.0, 6378135.0, 6378137.0, 6378137.0};
    const double f[5] = {1.0 / 297.0, 1.0 / 298.247, 1.0 / 298.26, 1.0 / 298.257222101, 1.0 / 298.257223563};

    double lambda  = atan2(Y,X);
    double ex2 = (2.0 - f[elipsoid_selection]) * f[elipsoid_selection] / ((1.0 - f[elipsoid_selection]) * (1.0 - f[elipsoid_selection]));
    double c = a[elipsoid_selection] * sqrt(1.0 + ex2);
    double phi = atan(Z / ((sqrt(X * X + Y * Y) * (1.0 - (2.0 - f[elipsoid_selection])) * f[elipsoid_selection])));

    double h = 0.1;
    double oldh = 0.0;
    double N;
    int iterations = 0;
    do
        {
            oldh = h;
            N = c / sqrt(1 + ex2 * (cos(phi) * cos(phi)));
            phi = atan(Z / ((sqrt(X*X + Y*Y) * (1.0 - (2.0 -f[elipsoid_selection]) * f[elipsoid_selection] * N / (N + h) ))));
            h = sqrt(X * X + Y * Y) / cos(phi) - N;
            iterations = iterations + 1;
            if (iterations > 100)
                {
                    LOG(WARNING) << "Failed to approximate h with desired precision. h-oldh= " << h - oldh;
                    break;
                }
        }
    while (std::abs(h - oldh) > 1.0e-12);
    d_latitude_d = phi * 180.0 / GPS_PI;
    d_longitude_d = lambda * 180.0 / GPS_PI;
    d_height_m = h;
}


void gps_l1_ca_ls_pvt::togeod(double *dphi, double *dlambda, double *h, double a, double finv, double X, double Y, double Z)
{
    /* Subroutine to calculate geodetic coordinates latitude, longitude,
	height given Cartesian coordinates X,Y,Z, and reference ellipsoid
	values semi-major axis (a) and the inverse of flattening (finv).

	 The output units of angular quantities will be in decimal degrees
	  (15.5 degrees not 15 deg 30 min). The output units of h will be the
	  same as the units of X,Y,Z,a.

	   Inputs:
	       a           - semi-major axis of the reference ellipsoid
	       finv        - inverse of flattening of the reference ellipsoid
	       X,Y,Z       - Cartesian coordinates

	   Outputs:
	       dphi        - latitude
	       dlambda     - longitude
	       h           - height above reference ellipsoid

	       Based in a Matlab function by Kai Borre
     */

    *h = 0;
    double tolsq = 1.e-10;  // tolerance to accept convergence
    int maxit = 10;         // max number of iterations
    double rtd = 180/GPS_PI;

    // compute square of eccentricity
    double esq;
    if (finv < 1.0E-20)
        {
            esq = 0;
        }
    else
        {
            esq = (2 - 1/finv) / finv;
        }

    // first guess

    double P = sqrt(X*X + Y*Y); // P is distance from spin axis
    //direct calculation of longitude
    if (P > 1.0E-20)
        {
            *dlambda = atan2(Y,X) * rtd;
        }
    else
        {
            *dlambda = 0;
        }
    // correct longitude bound
    if (*dlambda < 0)
        {
            *dlambda = *dlambda + 360.0;
        }
    double r = sqrt(P * P + Z * Z); // r is distance from origin (0,0,0)

    double sinphi;
    if (r > 1.0E-20)
        {
            sinphi = Z/r;
        }
    else
        {
            sinphi = 0.0;
        }
    *dphi = asin(sinphi);

    // initial value of height  =  distance from origin minus
    // approximate distance from origin to surface of ellipsoid
    if (r < 1.0E-20)
        {
            *h = 0;
            return;
        }

    *h = r - a * (1 - sinphi * sinphi / finv);

    // iterate
    double cosphi;
    double N_phi;
    double dP;
    double dZ;
    double oneesq = 1.0 - esq;

    for (int i = 0; i < maxit; i++)
        {
            sinphi = sin(*dphi);
            cosphi = cos(*dphi);

            // compute radius of curvature in prime vertical direction
            N_phi = a / sqrt(1 - esq * sinphi * sinphi);

            // compute residuals in P and Z
            dP = P - (N_phi + (*h)) * cosphi;
            dZ = Z - (N_phi*oneesq + (*h)) * sinphi;

            // update height and latitude
            *h = *h + (sinphi * dZ + cosphi * dP);
            *dphi = *dphi + (cosphi * dZ - sinphi * dP)/(N_phi + (*h));

            //     test for convergence
            if ((dP * dP + dZ * dZ) < tolsq)
                {
                    break;
                }
            if (i == (maxit - 1))
                {
                    LOG(WARNING) << "The computation of geodetic coordinates did not converge";
                }
        }
    *dphi = (*dphi) * rtd;
}


void gps_l1_ca_ls_pvt::topocent(double *Az, double *El, double *D, const arma::vec & x, const arma::vec & dx)
{
    /*  Transformation of vector dx into topocentric coordinate
	system with origin at x
	   Inputs:
	      x           - vector origin coordinates (in ECEF system [X; Y; Z;])
	      dx          - vector ([dX; dY; dZ;]).

	   Outputs:
	      D           - vector length. Units like the input
	      Az          - azimuth from north positive clockwise, degrees
	      El          - elevation angle, degrees

	      Based on a Matlab function by Kai Borre
     */

    double lambda;
    double phi;
    double h;
    double dtr = GPS_PI / 180.0;
    double a = 6378137.0;          // semi-major axis of the reference ellipsoid WGS-84
    double finv = 298.257223563;   // inverse of flattening of the reference ellipsoid WGS-84

    // Transform x into geodetic coordinates
    togeod(&phi, &lambda, &h, a, finv, x(0), x(1), x(2));

    double cl = cos(lambda * dtr);
    double sl = sin(lambda * dtr);
    double cb = cos(phi * dtr);
    double sb = sin(phi * dtr);

    arma::mat F = arma::zeros(3,3);

    F(0,0) = -sl;
    F(0,1) = -sb * cl;
    F(0,2) = cb * cl;

    F(1,0) = cl;
    F(1,1) = -sb * sl;
    F(1,2) = cb * sl;

    F(2,0) = 0;
    F(2,1) = cb;
    F(2,2) = sb;

    arma::vec local_vector;

    local_vector = arma::htrans(F) * dx;

    double E = local_vector(0);
    double N = local_vector(1);
    double U = local_vector(2);

    double hor_dis;
    hor_dis = sqrt(E * E + N * N);

    if (hor_dis < 1.0E-20)
        {
            *Az = 0;
            *El = 90;
        }
    else
        {
            *Az = atan2(E, N) / dtr;
            *El = atan2(U, hor_dis) / dtr;
        }

    if (*Az < 0)
        {
            *Az = *Az + 360.0;
        }

    *D = sqrt(dx(0) * dx(0) + dx(1) * dx(1) + dx(2) * dx(2));
}


void gps_l1_ca_ls_pvt::tropo(double *ddr_m, double sinel, double hsta_km, double p_mb, double t_kel, double hum, double hp_km, double htkel_km, double hhum_km)
{
    /*   Inputs:
           sinel     - sin of elevation angle of satellite
           hsta_km   - height of station in km
           p_mb      - atmospheric pressure in mb at height hp_km
           t_kel     - surface temperature in degrees Kelvin at height htkel_km
           hum       - humidity in % at height hhum_km
           hp_km     - height of pressure measurement in km
           htkel_km  - height of temperature measurement in km
           hhum_km   - height of humidity measurement in km

       Outputs:
           ddr_m     - range correction (meters)

     Reference
     Goad, C.C. & Goodman, L. (1974) A Modified Hopfield Tropospheric
     Refraction Correction Model. Paper presented at the
     American Geophysical Union Annual Fall Meeting, San
     Francisco, December 12-17

     Translated to C++ by Carles Fernandez from a Matlab implementation by Kai Borre
     */

    const double a_e    = 6378.137;    // semi-major axis of earth ellipsoid
    const double b0     = 7.839257e-5;
    const double tlapse = -6.5;
    const double em     = -978.77 / (2.8704e6 * tlapse * 1.0e-5);

    double tkhum  = t_kel + tlapse * (hhum_km - htkel_km);
    double atkel  = 7.5 * (tkhum - 273.15) / (237.3 + tkhum - 273.15);
    double e0     = 0.0611 * hum * pow(10, atkel);
    double tksea  = t_kel - tlapse * htkel_km;
    double tkelh  = tksea + tlapse * hhum_km;
    double e0sea  = e0 * pow((tksea / tkelh), (4 * em));
    double tkelp  = tksea + tlapse * hp_km;
    double psea   = p_mb * pow((tksea / tkelp), em);

    if(sinel < 0) { sinel = 0.0; }

    double tropo_delay   = 0.0;
    bool done      = false;
    double refsea  = 77.624e-6 / tksea;
    double htop    = 1.1385e-5 / refsea;
    refsea         = refsea * psea;
    double ref     = refsea * pow(((htop - hsta_km) / htop), 4);

    double a;
    double b;
    double rtop;

    while(1)
        {
            rtop = pow((a_e + htop), 2) - pow((a_e + hsta_km), 2) * (1 - pow(sinel, 2));

            // check to see if geometry is crazy
            if(rtop < 0) { rtop = 0; }

            rtop = sqrt(rtop) - (a_e + hsta_km) * sinel;

            a    = -sinel / (htop - hsta_km);
            b    = -b0 * (1 - pow(sinel,2)) / (htop - hsta_km);

            arma::vec rn = arma::vec(8);
            rn.zeros();

            for(int i = 0; i<8; i++)
                {
                    rn(i) = pow(rtop, (i+1+1));

                }

            arma::rowvec alpha = {2 * a, 2 * pow(a, 2) + 4 * b /3, a * (pow(a, 2) + 3 * b),
                    pow(a, 4)/5 + 2.4 * pow(a, 2) * b + 1.2 * pow(b, 2), 2 * a * b * (pow(a, 2) + 3 * b)/3,
                    pow(b, 2) * (6 * pow(a, 2) + 4 * b) * 1.428571e-1, 0, 0};

            if(pow(b, 2) > 1.0e-35)
                {
                    alpha(6) = a * pow(b, 3) /2;
                    alpha(7) = pow(b, 4) / 9;
                }

            double dr = rtop;
            arma::mat aux_ = alpha * rn;
            dr = dr + aux_(0, 0);
            tropo_delay = tropo_delay + dr * ref * 1000;

            if(done == true)
                {
                    *ddr_m = tropo_delay;
                    break;
                }

            done    = true;
            refsea  = (371900.0e-6 / tksea - 12.92e-6) / tksea;
            htop    = 1.1385e-5 * (1255 / tksea + 0.05) / refsea;
            ref     = refsea * e0sea * pow(((htop - hsta_km) / htop), 4);
        }
}
