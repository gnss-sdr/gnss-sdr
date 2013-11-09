/*!
 * \file sbas_ionospheric_correction.h
 * \brief Interface of the SBAS ionosphere correction set based on SBAS RTKLIB functions
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2013  (see AUTHORS file for a list of contributors)
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

#ifndef SBAS_IONOSPHERIC_CORRECTION_H_
#define SBAS_IONOSPHERIC_CORRECTION_H_

#include <iostream>
#include <vector>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <string>
#include <fstream>


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

// valid ionosphere correction for GPS
class Sbas_Ionosphere_Correction
{
private:
//    /* type definitions ----------------------------------------------------------*/
//#define MAXBAND     10                  /* max SBAS band of IGP */
//#define MAXNIGP     201                 /* max number of IGP in SBAS band */
//
//    typedef struct {        /* time struct */
//        time_t time;        /* time (s) expressed by standard time_t */
//        double sec;         /* fraction of second under 1 s */
//    } gtime_t;
//
//    typedef struct {        /* SBAS ionospheric correction type */
//            gtime_t t0;         /* correction time */
//            short lat,lon;      /* latitude/longitude (deg) */
//            short give;         /* GIVI+1 */
//            float delay;        /* vertical delay estimate (m) */
//        } sbsigp_t;
//
//    typedef struct {        /* SBAS ionospheric corrections type */
//         int iodi;           /* IODI (issue of date ionos corr) */
//         int nigp;           /* number of igps */
//         sbsigp_t igp[MAXNIGP]; /* ionospheric correction */
//     } sbsion_t;

	/* inner product ---------------------------------------------------------------
	* inner product of vectors
	* args   : double *a,*b     I   vector a,b (n x 1)
	*          int    n         I   size of vector a,b
	* return : a'*b
	*-----------------------------------------------------------------------------*/
	double dot(const double *a, const double *b, int n);
	/* multiply matrix -----------------------------------------------------------*/
	void matmul(const char *tr, int n, int k, int m, double alpha,
	                   const double *A, const double *B, double beta, double *C);
	/* ecef to local coordinate transfromation matrix ------------------------------
	* compute ecef to local coordinate transfromation matrix
	* args   : double *pos      I   geodetic position {lat,lon} (rad)
	*          double *E        O   ecef to local coord transformation matrix (3x3)
	* return : none
	* notes  : matirix stored by column-major order (fortran convention)
	*-----------------------------------------------------------------------------*/
	void xyz2enu(const double *pos, double *E);
	/* transform ecef vector to local tangental coordinate -------------------------
	* transform ecef vector to local tangental coordinate
	* args   : double *pos      I   geodetic position {lat,lon} (rad)
	*          double *r        I   vector in ecef coordinate {x,y,z}
	*          double *e        O   vector in local tangental coordinate {e,n,u}
	* return : none
	*-----------------------------------------------------------------------------*/
	void ecef2enu(const double *pos, const double *r, double *e);
	/* satellite azimuth/elevation angle -------------------------------------------
	* compute satellite azimuth/elevation angle
	* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
	*          double *e        I   receiver-to-satellilte unit vevtor (ecef)
	*          double *azel     IO  azimuth/elevation {az,el} (rad) (NULL: no output)
	*                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
	* return : elevation angle (rad)
	*-----------------------------------------------------------------------------*/
	double satazel(const double *pos, const double *e, double *azel);

    /* debug trace functions -----------------------------------------------------*/
    void trace(int level, const char *format, ...);
    /* time difference -------------------------------------------------------------
        * difference between gtime_t structs
        * args   : gtime_t t1,t2    I   gtime_t structs
        * return : time difference (t1-t2) (s)
        *-----------------------------------------------------------------------------*/
    //double timediff(gtime_t t1, gtime_t t2);
    /* ionospheric pierce point position -------------------------------------------
    * compute ionospheric pierce point (ipp) position and slant factor
    * args   : double *pos      I   receiver position {lat,lon,h} (rad,m)
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
    /* variance of ionosphere correction (give=GIVEI+1) --------------------------*/
    double varicorr(int give);
    /* search igps ---------------------------------------------------------------*/
    void searchigp(const double *pos, const Igp **igp, double *x, double *y);
    /* sbas ionospheric delay correction -------------------------------------------
    * compute sbas ionosphric delay correction
    * args   : long     sample_stamp    I   sample stamp of observable on which the correction will be applied
    *          sbsion_t *ion    I   ionospheric correction data (implicit)
    *          double   *pos    I   receiver position {lat,lon,height} (rad/m)
    *          double   *azel   I   satellite azimuth/elavation angle (rad)
    *          double   *delay  O   slant ionospheric delay (L1) (m)
    *          double   *var    O   variance of ionospheric delay (m^2)
    * return : status (1:ok, 0:no correction)
    * notes  : before calling the function, sbas ionosphere correction parameters
    *          in navigation data (nav->sbsion) must be set by callig
    *          sbsupdatecorr()
    *-----------------------------------------------------------------------------*/
    int sbsioncorr(const double sample_stamp, const double *pos,
            const double *azel, double *delay, double *var);



public:
    std::vector<Igp_Band> d_bands;

    void print(std::ostream &out);
    bool apply(double sample_stamp, double latitude_d, double longitude_d,
    		double azimut_d, double evaluation_d, double &delay, double &var);

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version){ar & d_bands;}
};


#endif /* SBAS_IONOSPHERIC_CORRECTION_H_ */
