/*!
 * \file sbas_ionospheric_correction.h
 * \brief Interface of the SBAS ionosphere correction set based on SBAS RTKLIB functions
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
 *
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

#ifndef SBAS_IONOSPHERIC_CORRECTION_H_
#define SBAS_IONOSPHERIC_CORRECTION_H_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>


/*!
 * \brief Struct that represents a Ionospheric Grid Point (IGP)
 */
struct Igp
{
public:
    //bool d_valid; // valid==true indicates that the IGP can be used for corrections. it is set to false when a new IGP mask (MT18) has been received but no corresponding delays (MT26)
    double t0; // time of reception, time of correction
    int d_latitude;
    int d_longitude;
    int d_give;
    double d_delay;
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & t0;
        ar & d_latitude;
        ar & d_longitude;
        ar & d_give;
        ar & d_delay;
    }
};


/*!
 * \brief Struct that represents the band of a Ionospheric Grid Point (IGP)
 */
struct Igp_Band
{
    //int d_iodi;
    //int d_nigp;       // number if IGPs in this band (defined by IGP mask from MT18)
    std::vector<Igp> d_igps;
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & d_igps;
    }
};



/*!
 * \brief Class that handles valid SBAS ionospheric correction for GPS
 */
class Sbas_Ionosphere_Correction
{
private:
    /* Inner product of vectors
     * params : double *a,*b     I   vector a,b (n x 1)
     *          int    n         I   size of vector a,b
     * return : a'*b
     */
    double dot(const double *a, const double *b, int n);

    /* Multiply matrix  */
    void matmul(const char *tr, int n, int k, int m, double alpha,
            const double *A, const double *B, double beta, double *C);

    /* EFEC to local coordinate transfomartion matrix
     * Compute ecef to local coordinate transfomartion matrix
     * params : double *pos      I   geodetic position {lat,lon} (rad)
     *          double *E        O   ecef to local coord transformation matrix (3x3)
     * return : none
     * notes  : matrix stored by column-major order (fortran convention)
     */
    void xyz2enu(const double *pos, double *E);

    /* Transforms ECEF vector into local tangential coordinates
     * params : double *pos      I   geodetic position {lat,lon} (rad)
     *          double *r        I   vector in ecef coordinate {x,y,z}
     *          double *e        O   vector in local tangental coordinate {e,n,u}
     * return : none
     */
    void ecef2enu(const double *pos, const double *r, double *e);

    /* Compute satellite azimuth/elevation angle
     * params : double *pos      I   geodetic position {lat,lon,h} (rad,m)
     *          double *e        I   receiver-to-satellilte unit vevtor (ecef)
     *          double *azel     IO  azimuth/elevation {az,el} (rad) (NULL: no output)
     *                               (0.0 <= azel[0] < 2*pi, -pi/2 <= azel[1] <= pi/2)
     * return : elevation angle (rad)
     */
    double satazel(const double *pos, const double *e, double *azel);

    /* Debug trace functions */
    void trace(int level, const char *format, ...);

    /* time difference -------------------------------------------------------------
     * difference between gtime_t structs
     * args   : gtime_t t1,t2    I   gtime_t structs
     * return : time difference (t1-t2) (s)
     *-----------------------------------------------------------------------------*/
    //double timediff(gtime_t t1, gtime_t t2);

    /* Compute Ionospheric Pierce Point (IPP) position and slant factor
     * params : double *pos      I   receiver position {lat,lon,h} (rad,m)
     *          double *azel     I   azimuth/elevation angle {az,el} (rad)
     *          double re        I   earth radius (km)
     *          double hion      I   altitude of ionosphere (km)
     *          double *posp     O   pierce point position {lat,lon,h} (rad,m)
     * return : slant factor
     * notes  : see ref [2], only valid on the earth surface
     *          fixing bug on ref [2] A.4.4.10.1 A-22,23
     *-----------------------------------------------------------------------------*/
    double ionppp(const double *pos, const double *azel, double re,
            double hion, double *posp);

    /* Variance of ionosphere correction (give = GIVEI + 1) */
    double varicorr(int give);

    /* Search igps */
    void searchigp(const double *pos, const Igp **igp, double *x, double *y);

    /* Compute sbas ionospheric delay correction
     * params : long     sample_stamp    I   sample stamp of observable on which the correction will be applied
     *          sbsion_t *ion    I   ionospheric correction data (implicit)
     *          double   *pos    I   receiver position {lat,lon,height} (rad/m)
     *          double   *azel   I   satellite azimuth/elavation angle (rad)
     *          double   *delay  O   slant ionospheric delay (L1) (m)
     *          double   *var    O   variance of ionospheric delay (m^2)
     * return : status (1:ok, 0:no correction)
     * notes  : before calling the function, sbas ionosphere correction parameters
     *          in navigation data (nav->sbsion) must be set by callig
     *          sbsupdatecorr()
     */
    int sbsioncorr(const double sample_stamp, const double *pos,
            const double *azel, double *delay, double *var);

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version){ar & d_bands;}

public:
    std::vector<Igp_Band> d_bands;
    void print(std::ostream &out);

    /*!
     * \brief Computes SBAS ionospheric delay correction.
     *
     * \param[out] delay        Slant ionospheric delay (L1) (m)
     * \param[out] var          Variance of ionospheric delay (m^2)
     * \param[in]  sample_stamp Sample stamp of observable on which the correction will be applied
     * \param[in]  longitude_d  Receiver's longitude in terms of WGS84 (degree)
     * \param[in]  latitude_d   Receiver's latitude in terms of WGS84 (degree)
     * \param[in]  azimuth_d    Satellite azimuth/elavation angle (rad). Azimuth is the angle of
     *                             the satellite from the user's location measured clockwise from north
     * \param[in]  elevation_d  Elevation is the angle of the satellite from the user's location measured
     *                             with respect to the local-tangent-plane
     */
    bool apply(double sample_stamp, double latitude_d, double longitude_d,
            double azimut_d, double evaluation_d, double &delay, double &var);

};


#endif /* SBAS_IONOSPHERIC_CORRECTION_H_ */
