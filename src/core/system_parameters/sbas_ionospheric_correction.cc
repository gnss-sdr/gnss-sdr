/*!
 * \file sbas_ionospheric_correction.cc
 * \brief Implementation of the SBAS ionosphere correction set based on SBAS RTKLIB functions
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

#include "sbas_ionospheric_correction.h"
#include <cmath>
#include <iostream>
#include <glog/logging.h>


enum V_Log_Level {EVENT = 2, // logs important events which don't occur every update() call
                  FLOW = 3,  // logs the function calls of block processing functions
                  MORE = 4};  // very detailed stuff


void Sbas_Ionosphere_Correction::print(std::ostream &out)
{
    for(std::vector<Igp_Band>::const_iterator it_band = d_bands.begin(); it_band != d_bands.end(); ++it_band)
        {
            int band = it_band - d_bands.begin();
            out << "<<I>> Band" <<  band << ":" << std::endl;
            for(std::vector<Igp>::const_iterator it_igp = it_band->d_igps.begin(); it_igp != it_band->d_igps.end(); ++it_igp)
                {
                    int igp = it_igp-it_band->d_igps.begin();
                    out << "<<I>> -IGP" <<  igp << ":";
                    //std::cout << "  valid=" <<  it_igp->d_valid;
                    out << "  t0=" <<  it_igp->t0;
                    out << "  lat=" <<  it_igp->d_latitude;
                    out << "  lon=" <<  it_igp->d_longitude;
                    out << "  give=" <<  it_igp->d_give;
                    out << "  delay=" <<  it_igp->d_delay;
                    out << std::endl;
                }
        }
}


/* Applies SBAS ionosphric delay correction
 * \param[out] delay        Slant ionospheric delay (L1) (m)
 * \param[out] var          Variance of ionospheric delay (m^2)
 * \param[in]  sample_stamp Sample stamp of observable on which the correction will be applied
 * \param[in]  longitude_d  Receiver's longitude in terms of WGS84 (degree)
 * \param[in]  latitude_d   Receiver's latitude in terms of WGS84 (degree)
 * \param[in]  azimuth_d    Satellite azimuth/elavation angle (rad). Azimuth is the angle of
 *                             the satellite from the user�s location measured clockwise from north
 * \param[in]  elevation_d  Elevation is the angle of the satellite from the user's location measured
 *                             with respect to the local-tangent-plane
 */
bool Sbas_Ionosphere_Correction::apply(double sample_stamp,
                                       double latitude_d,
                                       double longitude_d,
                                       double azimut_d,
                                       double elevation_d,
                                       double &delay,
                                       double &var)
{
    const double GPS_PI = 3.1415926535898;  //!< Pi as defined in IS-GPS-200E
    int result;
    double pos[3];
    double azel[2];

    // convert receiver position from degrees to rad
    pos[0] = latitude_d * GPS_PI / 180.0;
    pos[1] = longitude_d * GPS_PI / 180.0;
    pos[2] = 0; // is not used by sbsioncorr, for ionocorrection is a fixed earth radius assumed

    // convert satellite azimut and elevation from degrees to rad , use topocent to obtain it in pvt block
    azel[0] = azimut_d * GPS_PI / 180.0;
    azel[1] = elevation_d * GPS_PI / 180.0;

    result = sbsioncorr(sample_stamp, pos, azel, &delay, &var);
    return (bool)result;
}



/* geometric distance ----------------------------------------------------------
* compute geometric distance and receiver-to-satellite unit vector
* args   : double *rs       I   satellilte position (ecef at transmission) (m)
*          double *rr       I   receiver position (ecef at reception) (m)
*          double *e        O   line-of-sight vector (ecef)
* return : geometric distance (m) (0>:error/no satellite position)
* notes  : distance includes sagnac effect correction
*-----------------------------------------------------------------------------*/
//extern double geodist(const double *rs, const double *rr, double *e)
//{
//    double r;
//    int i;
//
//    if (norm(rs,3)<RE_WGS84) return -1.0;
//    for (i=0;i<3;i++) e[i]=rs[i]-rr[i];
//    r=norm(e,3);
//    for (i=0;i<3;i++) e[i]/=r;
//    return r+OMGE*(rs[0]*rr[1]-rs[1]*rr[0])/CLIGHT;
//}

/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
double Sbas_Ionosphere_Correction::dot(const double *a, const double *b, int n)
{
    double c = 0.0;
    while (--n >= 0) c += a[n]*b[n];
    return c;
}




/* multiply matrix -----------------------------------------------------------*/
void Sbas_Ionosphere_Correction::matmul(const char *tr, int n, int k, int m, double alpha,
        const double *A, const double *B, double beta, double *C)
{
    double d;
    int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

    for (i = 0; i < n; i++) for (j = 0; j < k; j++)
        {
            d = 0.0;
            switch (f)
            {
            case 1: for (x = 0; x < m; x++) d += A[i + x*n]*B[x + j*m]; break;
            case 2: for (x = 0; x < m; x++) d += A[i + x*n]*B[j + x*k]; break;
            case 3: for (x = 0; x < m; x++) d += A[x + i*m]*B[x + j*m]; break;
            case 4: for (x = 0; x < m; x++) d += A[x + i*m]*B[j + x*k]; break;
            }
            if (beta == 0.0)
                {
                    C[i + j*n] = alpha*d;
                }
            else
                {
                    C[i + j*n] = alpha*d + beta*C[i + j*n];
                }
        }
}


/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matrix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
void Sbas_Ionosphere_Correction::xyz2enu(const double *pos, double *E)
{
    double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);
    E[0] = -sinl;      E[3] = cosl;       E[6] = 0.0;
    E[1] = -sinp*cosl; E[4] = -sinp*sinl; E[7] = cosp;
    E[2] = cosp*cosl;  E[5] = cosp*sinl;  E[8] = sinp;
}


/* transform ecef vector to local tangential coordinate -------------------------
* transform ecef vector to local tangential coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
void Sbas_Ionosphere_Correction::ecef2enu(const double *pos, const double *r, double *e)
{
    double E[9];
    xyz2enu(pos, E);
    matmul("NN", 3, 1, 3, 1.0, E, r, 0.0, e);
}



const double PI = 3.1415926535897932; /* pi */



/* satellite azimuth/elevation angle -------------------------------------------
* compute satellite azimuth/elevation angle
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *e        I   receiver-to-satellilte unit vevtor (ecef)
*          double *azel     IO  azimuth/elevation {az,el} (rad) (NULL: no output)
*                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
* return : elevation angle (rad)
*-----------------------------------------------------------------------------*/
double Sbas_Ionosphere_Correction::satazel(const double *pos, const double *e, double *azel)
{
    const double RE_WGS84 = 6378137.0;    /* earth semimajor axis (WGS84) (m) */

    double az = 0.0, el = PI/2.0, enu[3];

    if (pos[2] > -RE_WGS84)
        {
            ecef2enu(pos, e, enu);
            az = dot(enu, enu, 2) < 1E-12 ? 0.0 : atan2(enu[0], enu[1]);
            if (az < 0.0) az += 2*PI;
            el = asin(enu[2]);
        }
    if (azel)
        {
            azel[0] = az;
            azel[1] = el;
        }
    return el;
}

/* debug trace function -----------------------------------------------------*/
void Sbas_Ionosphere_Correction::trace(int level, const char *format, ...)
{
    va_list ap;
    char str[1000];

    va_start(ap,format); vsprintf(str,format,ap); va_end(ap);
    VLOG(FLOW) << "<<I>> " << std::string(str);
}

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
double Sbas_Ionosphere_Correction::ionppp(const double *pos, const double *azel,
        double re, double hion, double *posp)
{
    double cosaz, rp, ap, sinap, tanap;
    const double D2R = (PI/180.0);        /* deg to rad */

    rp = re/(re + hion)*cos(azel[1]);
    ap = PI/2.0 - azel[1] - asin(rp);
    sinap = sin(ap);
    tanap = tan(ap);
    cosaz = cos(azel[0]);
    posp[0] = asin(sin(pos[0])*cos(ap) + cos(pos[0])*sinap*cosaz);

    if ((pos[0] > 70.0*D2R && tanap*cosaz > tan(PI/2.0 - pos[0])) ||
            (pos[0] < -70.0*D2R && - tanap*cosaz > tan(PI/2.0 + pos[0])))
        {
            posp[1] = pos[1] + PI - asin(sinap*sin(azel[0])/cos(posp[0]));
        }
    else
        {
            posp[1] = pos[1] + asin(sinap*sin(azel[0])/cos(posp[0]));
        }
    return 1.0 / sqrt(1.0 - rp*rp);
}


/* variance of ionosphere correction (give=GIVEI) --------------------------*/
double Sbas_Ionosphere_Correction::varicorr(int give)
{
    const double var[15]={
        0.0084, 0.0333, 0.0749, 0.1331, 0.2079, 0.2994, 0.4075, 0.5322, 0.6735, 0.8315,
        1.1974, 1.8709, 3.326, 20.787, 187.0826
    };
    return 0 <= give && give < 15 ? var[give]:0.0;
}


/* search igps ---------------------------------------------------------------*/
void Sbas_Ionosphere_Correction::searchigp(const double *pos, const Igp **igp, double *x, double *y)
{
    int i;
    int latp[2];
    int lonp[4];
    const double R2D = (180.0/PI);   /* rad to deg */

    double lat = pos[0]*R2D;
    double lon = pos[1]*R2D;

    trace(4,"searchigp: pos=%.3f %.3f",pos[0]*R2D, pos[1]*R2D);

    // round the pierce point position to the next IGP grid point
    if (lon >= 180.0) lon -= 360.0;
    if (-55.0 <= lat && lat < 55.0)
        {
            latp[0] = (int)floor(lat/5.0)*5;
            latp[1] = latp[0] + 5;
            lonp[0] = lonp[1] = (int)floor(lon/5.0)*5;
            lonp[2] = lonp[3] = lonp[0] + 5;
            *x = (lon - lonp[0])/5.0;
            *y = (lat - latp[0])/5.0;
        }
    else
        {
            latp[0] = (int)floor((lat-5.0)/10.0)*10+5;
            latp[1] = latp[0] + 10;
            lonp[0] = lonp[1] = (int)floor(lon/10.0)*10;
            lonp[2] = lonp[3] = lonp[0] + 10;
            *x = (lon - lonp[0])/10.0;
            *y = (lat - latp[0])/10.0;
            if (75.0 <= lat && lat < 85.0)
                {
                    lonp[1] = (int)floor(lon/90.0)*90;
                    lonp[3] = lonp[1] + 90;
                }
            else if (-85.0 <= lat && lat < -75.0)
                {
                    lonp[0] = (int)floor((lon - 50.0)/90.0)*90 + 40;
                    lonp[2] = lonp[0] + 90;
                }
            else if (lat >= 85.0)
                {
                    for (i = 0; i < 4; i++) lonp[i] = (int)floor(lon/90.0)*90;
                }
            else if (lat <- 85.0)
                {
                    for (i = 0; i < 4; i++) lonp[i] = (int)floor((lon - 50.0)/90.0)*90 + 40;
                }
        }

    for (i = 0; i < 4; i++) if (lonp[i] == 180) lonp[i] = -180;

    // find the correction data for the grid points in latp[] and lonp[]
    // iterate over bands
    for (std::vector<Igp_Band>::const_iterator band_it = this->d_bands.begin(); band_it != d_bands.end(); ++band_it)
        {
            //VLOG(MORE) << "band=" << band_it-d_bands.begin() << std::endl;
            // iterate over IGPs in band_it
            for (std::vector<Igp>::const_iterator igp_it = band_it->d_igps.begin(); igp_it != band_it->d_igps.end(); ++igp_it)
                {
                    std::stringstream ss;
                    int give = igp_it->d_give;
                    ss << "IGP: give=" << give;
                    if(give < 15) // test if valid correction data is sent for current IGP
                        {
                            int lat = igp_it->d_latitude;
                            int lon = igp_it->d_longitude;
                            ss << " lat=" << lat << " lon=" << lon;
                            if (lat == latp[0] && lon == lonp[0]) igp[0] = igp_it.base();
                            else if (lat == latp[1] && lon == lonp[1]) igp[1] = igp_it.base();
                            else if (lat == latp[0] && lon == lonp[2]) igp[2] = igp_it.base();
                            else if (lat == latp[1] && lon == lonp[3]) igp[3] = igp_it.base();
                        }
                    //VLOG(MORE) << ss.str();
                }
        }
    //VLOG(MORE) << "igp[0:3]={" << igp[0] << "," << igp[1] << "," << igp[2] << "," << igp[3] << "}";
}



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
*          in navigation data (nav->sbsion) must be set by calling
*          sbsupdatecorr()
*-----------------------------------------------------------------------------*/
int Sbas_Ionosphere_Correction::sbsioncorr(const double sample_stamp, const double *pos,
        const double *azel, double *delay, double *var)
{
    const double re = 6378.1363;
    const double hion = 350.0;
    int err = 0;
    double fp;
    double posp[2];
    double x = 0.0;
    double y = 0.0;
    double t;
    double w[4] = {0};
    const Igp *igp[4] = {0}; /* {ws,wn,es,en} */
    const double R2D = (180.0/PI);        /* rad to deg */

    trace(4, "sbsioncorr: pos=%.3f %.3f azel=%.3f %.3f", pos[0]*R2D, pos[1]*R2D, azel[0]*R2D, azel[1]*R2D);

    *delay = *var = 0.0;
    if (pos[2] < -100.0 || azel[1] <= 0) return 1;

    /* ipp (ionospheric pierce point) position */
    fp = ionppp(pos, azel, re, hion, posp);

    /* search igps around ipp */
    searchigp(posp, igp, &x, &y);

    VLOG(FLOW) << "<<I>> SBAS iono correction:" << " igp[0]=" << igp[0] << " igp[1]=" << igp[1]
               << " igp[2]=" << igp[2] << " igp[3]=" << igp[3] << " x=" << x << " y=" << y;

    /* weight of igps */
    if (igp[0] && igp[1] && igp[2] && igp[3])
        {
            w[0] = (1.0 - x)*(1.0 - y);
            w[1] = (1.0 - x)*y;
            w[2] = x*(1.0 - y);
            w[3] = x*y;
        }
    else if (igp[0] && igp[1] && igp[2])
        {
            w[1] = y;
            w[2] = x;
            if ((w[0] = 1.0 - w[1] - w[2]) < 0.0) err = 1;
        }
    else if (igp[0] && igp[2] && igp[3])
        {
            w[0] = 1.0 - x;
            w[3] = y;
            if ((w[2] = 1.0 - w[0] -w[3]) < 0.0) err = 1;
        }
    else if (igp[0] && igp[1] && igp[3])
        {
            w[0] = 1.0 - y;
            w[3] = x;
            if ((w[1] = 1.0 - w[0] - w[3]) < 0.0) err = 1;
        }
    else if (igp[1]&&igp[2]&&igp[3])
        {
            w[1] = 1.0 - x;
            w[2] = 1.0 - y;
            if ((w[3] = 1.0 - w[1] - w[2]) < 0.0) err = 1;
        }
    else err = 1;

    if (err)
        {
            trace(2, "no sbas iono correction: lat=%3.0f lon=%4.0f", posp[0]*R2D, posp[1]*R2D);
            return 0;
        }
    for (int i = 0; i <4 ; i++)
        {
            if (!igp[i]) continue;
            t = (sample_stamp - igp[i]->t0); // time diff between now and reception of the igp data in seconds
            *delay += w[i]*igp[i]->d_delay;
            *var += w[i] * varicorr(igp[i]->d_give) * 9E-8 * fabs(t);
        }
    *delay *= fp;
    *var *= fp*fp;

    trace(5, "sbsioncorr: dion=%7.2f sig=%7.2f", *delay, sqrt(*var));
    return 1;
}
