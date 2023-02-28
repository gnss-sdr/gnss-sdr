/*!
 * \file rtklib_ephemeris.cc
 * \brief satellite ephemeris and clock functions
 * \authors <ul>
 *          <li> 2007-2013, T. Takasu
 *          <li> 2017, Javier Arribas
 *          <li> 2017-2023, Carles Fernandez
 *          </ul>
 *
 * This is a derived work from RTKLIB http://www.rtklib.com/
 * The original source code at https://github.com/tomojitakasu/RTKLIB is
 * released under the BSD 2-clause license with an additional exclusive clause
 * that does not apply here. This additional clause is reproduced below:
 *
 * " The software package includes some companion executive binaries or shared
 * libraries necessary to execute APs on Windows. These licenses succeed to the
 * original ones of these software. "
 *
 * Neither the executive binaries nor the shared libraries are required by, used
 * or included in GNSS-SDR.
 *
 * -----------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017-2023, Carles Fernandez
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * -----------------------------------------------------------------------------
 */

#include "rtklib_ephemeris.h"
#include "rtklib_preceph.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_sbas.h"
#include <vector>

/* constants -----------------------------------------------------------------*/

const double RE_GLO = 6378136.0;      /* radius of earth (m)            ref [2] */
const double MU_GPS = 3.9860050e14;   /* gravitational constant         ref [1] */
const double MU_GLO = 3.9860044e14;   /* gravitational constant         ref [2] */
const double MU_GAL = 3.986004418e14; /* earth gravitational constant   ref [7] */
const double MU_BDS = 3.986004418e14; /* earth gravitational constant   ref [9] */
const double J2_GLO = 1.0826257e-3;   /* 2nd zonal harmonic of geopot   ref [2] */

const double OMGE_GLO = GLONASS_OMEGA_EARTH_DOT; /* earth angular velocity (rad/s) ref [2] */
const double OMGE_GAL = GNSS_OMEGA_EARTH_DOT;    /* earth angular velocity (rad/s) ref [7] */
const double OMGE_BDS = BEIDOU_OMEGA_EARTH_DOT;  /* earth angular velocity (rad/s) ref [9] */

const double SIN_5 = -0.0871557427476582; /* sin(-5.0 deg) */
const double COS_5 = 0.9961946980917456;  /* cos(-5.0 deg) */

const double ERREPH_GLO = 5.0;    /* error of glonass ephemeris (m) */
const double TSTEP = 60.0;        /* integration step glonass ephemeris (s) */
const double RTOL_KEPLER = 1e-13; /* relative tolerance for Kepler equation */

const double DEFURASSR = 0.15;                       /* default accuracy of ssr corr (m) */
const double MAXECORSSR = 10.0;                      /* max orbit correction of ssr (m) */
const double MAXCCORSSR = 1e-6 * SPEED_OF_LIGHT_M_S; /* max clock correction of ssr (m) */
const double MAXAGESSR = 90.0;                       /* max age of ssr orbit and clock (s) */
const double MAXAGESSR_HRCLK = 10.0;                 /* max age of ssr high-rate clock (s) */
const double STD_BRDCCLK = 30.0;                     /* error of broadcast clock (m) */

const int MAX_ITER_KEPLER = 30; /* max number of iteration of Kelpler */


/* variance by ura ephemeris (ref [1] 20.3.3.3.1.1) --------------------------*/
double var_uraeph(int ura)
{
    const double ura_value[] = {
        2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0,
        3072.0, 6144.0};
    return ura < 0 || 14 < ura ? std::pow(6144.0, 2.0) : std::pow(ura_value[ura], 2.0);
}


/* variance by ura ssr (ref [4]) ---------------------------------------------*/
double var_urassr(int ura)
{
    double std_;
    if (ura <= 0)
        {
            return std::pow(DEFURASSR, 2.0);
        }
    if (ura >= 63)
        {
            return std::pow(5.4665, 2.0);
        }
    std_ = (std::pow((ura >> 3) & 7, 2.0) * (1.0 + (ura & 7) / 4.0) - 1.0) * 1e-3;
    return std::pow(std_, 2.0);
}


/* almanac to satellite position and clock bias --------------------------------
 * compute satellite position and clock bias with almanac (gps, galileo, qzss)
 * args   : gtime_t time     I   time (gpst)
 *          alm_t *alm       I   almanac
 *          double *rs       O   satellite position (ecef) {x,y,z} (m)
 *          double *dts      O   satellite clock bias (s)
 * return : none
 * notes  : see ref [1],[7],[8]
 *-----------------------------------------------------------------------------*/
void alm2pos(gtime_t time, const alm_t *alm, double *rs, double *dts)
{
    double tk;
    double M;
    double E;
    double Ek;
    double sinE;
    double cosE;
    double u;
    double r;
    double i;
    double O;
    double x;
    double y;
    double sinO;
    double cosO;
    double cosi;
    double mu;
    int n;

    trace(4, "alm2pos : time=%s sat=%2d\n", time_str(time, 3), alm->sat);

    tk = timediffweekcrossover(time, alm->toa);

    if (alm->A <= 0.0)
        {
            rs[0] = rs[1] = rs[2] = *dts = 0.0;
            return;
        }
    mu = satsys(alm->sat, nullptr) == SYS_GAL ? MU_GAL : MU_GPS;

    M = alm->M0 + sqrt(mu / (alm->A * alm->A * alm->A)) * tk;
    for (n = 0, E = M, Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER && n < MAX_ITER_KEPLER; n++)
        {
            Ek = E;
            E -= (E - alm->e * sin(E) - M) / (1.0 - alm->e * cos(E));
        }
    if (n >= MAX_ITER_KEPLER)
        {
            trace(2, "alm2pos: kepler iteration overflow sat=%2d\n", alm->sat);
            return;
        }
    sinE = sin(E);
    cosE = cos(E);
    u = atan2(sqrt(1.0 - alm->e * alm->e) * sinE, cosE - alm->e) + alm->omg;
    r = alm->A * (1.0 - alm->e * cosE);
    i = alm->i0;
    O = alm->OMG0 + (alm->OMGd - GNSS_OMEGA_EARTH_DOT) * tk - GNSS_OMEGA_EARTH_DOT * alm->toas;
    x = r * cos(u);
    y = r * sin(u);
    sinO = sin(O);
    cosO = cos(O);
    cosi = cos(i);
    rs[0] = x * cosO - y * cosi * sinO;
    rs[1] = x * sinO + y * cosi * cosO;
    rs[2] = y * sin(i);
    *dts = alm->f0 + alm->f1 * tk;
}


/* broadcast ephemeris to satellite clock bias ---------------------------------
 * compute satellite clock bias with broadcast ephemeris (gps, galileo, qzss)
 * args   : gtime_t time     I   time by satellite clock (gpst)
 *          eph_t *eph       I   broadcast ephemeris
 * return : satellite clock bias (s) without relativeity correction
 * notes  : see ref [1],[7],[8]
 *          satellite clock does not include relativity correction and tdg
 *-----------------------------------------------------------------------------*/
double eph2clk(gtime_t time, const eph_t *eph)
{
    double t;
    int i;

    trace(4, "eph2clk : time=%s sat=%2d\n", time_str(time, 3), eph->sat);

    t = timediffweekcrossover(time, eph->toc);

    for (i = 0; i < 2; i++)
        {
            t -= eph->f0 + eph->f1 * t + eph->f2 * t * t;
        }
    return eph->f0 + eph->f1 * t + eph->f2 * t * t;
}


/* broadcast ephemeris to satellite position and clock bias --------------------
 * compute satellite position and clock bias with broadcast ephemeris (gps,
 * galileo, qzss)
 * args   : gtime_t time     I   time (gpst)
 *          eph_t *eph       I   broadcast ephemeris
 *          double *rs       O   satellite position (ecef) {x,y,z} (m)
 *          double *dts      O   satellite clock bias (s)
 *          double *var      O   satellite position and clock variance (m^2)
 * return : none
 * notes  : see ref [1],[7],[8]
 *          satellite clock includes relativity correction without code bias
 *          (tgd or bgd)
 *-----------------------------------------------------------------------------*/
void eph2pos(gtime_t time, const eph_t *eph, double *rs, double *dts,
    double *var)
{
    double tk;
    double M;
    double E;
    double Ek;
    double sinE;
    double cosE;
    double u;
    double r;
    double i;
    double O;
    double sin2u;
    double cos2u;
    double x;
    double y;
    double sinO;
    double cosO;
    double cosi;
    double mu;
    double omge;
    double xg;
    double yg;
    double zg;
    double sino;
    double coso;
    int n;
    int sys;
    int prn;

    double has_relativistic_correction = 0.0;
    trace(4, "eph2pos : time=%s sat=%2d\n", time_str(time, 3), eph->sat);

    if (eph->A <= 0.0)
        {
            rs[0] = rs[1] = rs[2] = *dts = *var = 0.0;
            return;
        }
    tk = timediffweekcrossover(time, eph->toe);

    switch ((sys = satsys(eph->sat, &prn)))
        {
        case SYS_GAL:
            mu = MU_GAL;
            omge = OMGE_GAL;
            break;
        case SYS_BDS:
            mu = MU_BDS;
            omge = OMGE_BDS;
            break;
        default:
            mu = MU_GPS;
            omge = GNSS_OMEGA_EARTH_DOT;
            break;
        }
    M = eph->M0 + (sqrt(mu / (eph->A * eph->A * eph->A)) + eph->deln) * tk;

    for (n = 0, E = M, Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER && n < MAX_ITER_KEPLER; n++)
        {
            Ek = E;
            E -= (E - eph->e * sin(E) - M) / (1.0 - eph->e * cos(E));
        }
    if (n >= MAX_ITER_KEPLER)
        {
            trace(2, "eph2pos: kepler iteration overflow sat=%2d\n", eph->sat);
            return;
        }
    sinE = sin(E);
    cosE = cos(E);

    trace(4, "kepler: sat=%2d e=%8.5f n=%2d del=%10.3e\n", eph->sat, eph->e, n, E - Ek);

    u = atan2(sqrt(1.0 - eph->e * eph->e) * sinE, cosE - eph->e) + eph->omg;
    r = eph->A * (1.0 - eph->e * cosE);
    i = eph->i0 + eph->idot * tk;
    sin2u = sin(2.0 * u);
    cos2u = cos(2.0 * u);
    u += eph->cus * sin2u + eph->cuc * cos2u;
    r += eph->crs * sin2u + eph->crc * cos2u;
    i += eph->cis * sin2u + eph->cic * cos2u;
    x = r * cos(u);
    y = r * sin(u);
    cosi = cos(i);

    /* beidou geo satellite (ref [9]) */
    if (sys == SYS_BDS && (prn <= 5 || prn > 58))
        {
            O = eph->OMG0 + eph->OMGd * tk - omge * eph->toes;
            sinO = sin(O);
            cosO = cos(O);
            xg = x * cosO - y * cosi * sinO;
            yg = x * sinO + y * cosi * cosO;
            zg = y * sin(i);
            sino = sin(omge * tk);
            coso = cos(omge * tk);
            rs[0] = xg * coso + yg * sino * COS_5 + zg * sino * SIN_5;
            rs[1] = -xg * sino + yg * coso * COS_5 + zg * coso * SIN_5;
            rs[2] = -yg * SIN_5 + zg * COS_5;
        }
    else
        {
            O = eph->OMG0 + (eph->OMGd - omge) * tk - omge * eph->toes;
            sinO = sin(O);
            cosO = cos(O);
            rs[0] = x * cosO - y * cosi * sinO;
            rs[1] = x * sinO + y * cosi * cosO;
            rs[2] = y * sin(i);
            // Apply HAS orbit correction if available
            if (eph->apply_has_corrections)
                {
                    // HAS SIS ICD, Issue 1.0, Section 7.2
                    double vel_sat[3]{};
                    double cross_pos_vel[3]{};
                    double et[3]{};
                    double ew[3]{};
                    double en[3]{};
                    double R[3][3]{};
                    double corrections[3]{};
                    double rotated_corrections[3]{};
                    // Compute satellite velocity
                    const double OneMinusecosE = 1.0 - (eph->e * cosE);
                    const double ekdot = (sqrt(mu / (eph->A * eph->A * eph->A)) + eph->deln) / OneMinusecosE;
                    const double pkdot = sqrt(1.0 - eph->e * eph->e) * ekdot / OneMinusecosE;
                    const double ukdot = pkdot * (1.0 + 2.0 * (eph->cus * cos2u - eph->cuc * sin2u));
                    const double ikdot = eph->idot + 2.0 * pkdot * (eph->cis * cos2u - eph->cic * sin2u);
                    const double rkdot = eph->A * eph->e * sinE * ekdot + 2.0 * pkdot * (eph->crs * cos2u - eph->crc * sin2u);
                    const double xpkdot = rkdot * cos(u) - y * ukdot;
                    const double ypkdot = rkdot * sin(u) + x * ukdot;
                    const double tmp = ypkdot * cosi - rs[2] * ikdot;

                    vel_sat[0] = -(eph->OMGd - omge) * rs[1] + xpkdot * cosO - tmp * sinO;
                    vel_sat[1] = (eph->OMGd - omge) * rs[0] + xpkdot * sinO + tmp * cosO;
                    vel_sat[2] = y * cosi * ikdot + ypkdot * sin(i);

                    // Compute HAS relativistic clock correction (HAS SIS ICD, Issue 1.0, Section 7.3)
                    const double pos_by_vel = rs[0] * vel_sat[0] + rs[1] * vel_sat[1] + rs[2] * vel_sat[2];
                    has_relativistic_correction = -(2.0 * pos_by_vel) / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S);

                    // Compute rotation matrix
                    const double norm_velocity = sqrt(vel_sat[0] * vel_sat[0] + vel_sat[1] * vel_sat[1] + vel_sat[2] * vel_sat[2]);
                    et[0] = vel_sat[0] / norm_velocity;
                    et[1] = vel_sat[1] / norm_velocity;
                    et[2] = vel_sat[2] / norm_velocity;

                    cross_pos_vel[0] = rs[1] * vel_sat[2] - rs[2] * vel_sat[1];
                    cross_pos_vel[1] = rs[2] * vel_sat[0] - rs[0] * vel_sat[2];
                    cross_pos_vel[2] = rs[0] * vel_sat[1] - rs[1] * vel_sat[0];
                    const double norm_cross_pos_vel = sqrt(cross_pos_vel[0] * cross_pos_vel[0] + cross_pos_vel[1] * cross_pos_vel[1] + cross_pos_vel[2] * cross_pos_vel[2]);

                    ew[0] = cross_pos_vel[0] / norm_cross_pos_vel;
                    ew[1] = cross_pos_vel[1] / norm_cross_pos_vel;
                    ew[2] = cross_pos_vel[2] / norm_cross_pos_vel;

                    en[0] = et[1] * ew[2] - et[2] * ew[1];
                    en[1] = et[2] * ew[0] - et[0] * ew[2];
                    en[2] = et[0] * ew[1] - et[1] * ew[0];

                    R[0][0] = en[0];
                    R[0][1] = et[0];
                    R[0][2] = ew[0];

                    R[1][0] = en[1];
                    R[1][1] = et[1];
                    R[1][2] = ew[1];

                    R[2][0] = en[2];
                    R[2][1] = et[2];
                    R[2][2] = ew[2];

                    // Compute rotated corrections
                    corrections[0] = eph->has_orbit_radial_correction_m;
                    corrections[1] = eph->has_orbit_in_track_correction_m;
                    corrections[2] = eph->has_orbit_cross_track_correction_m;

                    for (int row = 0; row < 3; row++)
                        {
                            for (int col = 0; col < 3; col++)
                                {
                                    rotated_corrections[row] = R[row][col] * corrections[col];
                                }
                        }

                    // Apply HAS orbit corrections
                    rs[0] += rotated_corrections[0];
                    rs[1] += rotated_corrections[1];
                    rs[2] += rotated_corrections[2];
                }
        }
    tk = timediffweekcrossover(time, eph->toc);
    *dts = eph->f0 + eph->f1 * tk + eph->f2 * tk * tk;

    /* relativity correction */
    if (eph->apply_has_corrections)
        {
            // Apply HAS clock correction (HAS SIS ICD, Issue 1.0, Section 7.3)
            *dts += (has_relativistic_correction + (eph->has_clock_correction_m / SPEED_OF_LIGHT_M_S));
            // Note: This is referred to the GST for Galileo satellites. The user must account for
            // a possible common offset in the broadcast HAS GPS clock corrections
        }
    else
        {
            *dts -= 2.0 * sqrt(mu * eph->A) * eph->e * sinE / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S);
        }

    /* position and clock error variance */
    *var = var_uraeph(eph->sva);
}


/* glonass orbit differential equations --------------------------------------*/
void deq(const double *x, double *xdot, const double *acc)
{
    double a;
    double b;
    double c;
    double r2 = dot(x, x, 3);
    double r3 = r2 * sqrt(r2);
    double omg2 = std::pow(OMGE_GLO, 2.0);

    if (r2 <= 0.0)
        {
            xdot[0] = xdot[1] = xdot[2] = xdot[3] = xdot[4] = xdot[5] = 0.0;
            return;
        }
    /* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
    a = 1.5 * J2_GLO * MU_GLO * std::pow(RE_GLO, 2.0) / r2 / r3; /* 3/2*J2*mu*Ae^2/r^5 */
    b = 5.0 * x[2] * x[2] / r2;                                  /* 5*z^2/r^2 */
    c = -MU_GLO / r3 - a * (1.0 - b);                            /* -mu/r^3-a(1-b) */
    xdot[0] = x[3];
    xdot[1] = x[4];
    xdot[2] = x[5];
    xdot[3] = (c + omg2) * x[0] + 2.0 * OMGE_GLO * x[4] + acc[0];
    xdot[4] = (c + omg2) * x[1] - 2.0 * OMGE_GLO * x[3] + acc[1];
    xdot[5] = (c - 2.0 * a) * x[2] + acc[2];
}


/* glonass position and velocity by numerical integration --------------------*/
void glorbit(double t, double *x, const double *acc)
{
    double k1[6];
    double k2[6];
    double k3[6];
    double k4[6];
    double w[6];
    int i;

    deq(x, k1, acc);
    for (i = 0; i < 6; i++)
        {
            w[i] = x[i] + k1[i] * t / 2.0;
        }
    deq(w, k2, acc);
    for (i = 0; i < 6; i++)
        {
            w[i] = x[i] + k2[i] * t / 2.0;
        }
    deq(w, k3, acc);
    for (i = 0; i < 6; i++)
        {
            w[i] = x[i] + k3[i] * t;
        }
    deq(w, k4, acc);
    for (i = 0; i < 6; i++)
        {
            x[i] += (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) * t / 6.0;
        }
}


/* glonass ephemeris to satellite clock bias -----------------------------------
 * compute satellite clock bias with glonass ephemeris
 * args   : gtime_t time     I   time by satellite clock (gpst)
 *          geph_t *geph     I   glonass ephemeris
 * return : satellite clock bias (s)
 * notes  : see ref [2]
 *-----------------------------------------------------------------------------*/
double geph2clk(gtime_t time, const geph_t *geph)
{
    double t;
    int i;

    trace(4, "geph2clk: time=%s sat=%2d\n", time_str(time, 3), geph->sat);

    t = timediff(time, geph->toe);

    for (i = 0; i < 2; i++)
        {
            t -= -geph->taun + geph->gamn * t;
        }
    return -geph->taun + geph->gamn * t;
}


/* glonass ephemeris to satellite position and clock bias ----------------------
 * compute satellite position and clock bias with glonass ephemeris
 * args   : gtime_t time     I   time (gpst)
 *          geph_t *geph     I   glonass ephemeris
 *          double *rs       O   satellite position {x,y,z} (ecef) (m)
 *          double *dts      O   satellite clock bias (s)
 *          double *var      O   satellite position and clock variance (m^2)
 * return : none
 * notes  : see ref [2]
 *-----------------------------------------------------------------------------*/
void geph2pos(gtime_t time, const geph_t *geph, double *rs, double *dts,
    double *var)
{
    double t;
    double tt;
    double x[6];
    int i;

    trace(4, "geph2pos: time=%s sat=%2d\n", time_str(time, 3), geph->sat);

    t = timediff(time, geph->toe);

    *dts = -geph->taun + geph->gamn * t;

    for (i = 0; i < 3; i++)
        {
            x[i] = geph->pos[i];
            x[i + 3] = geph->vel[i];
        }
    tt = t < 0.0 ? -TSTEP : TSTEP;
    while (fabs(t) > 1e-9)
        {
            if (fabs(t) < TSTEP)
                {
                    tt = t;
                }
            glorbit(tt, x, geph->acc);
            t -= tt;
        }
    for (i = 0; i < 3; i++)
        {
            rs[i] = x[i];
        }

    *var = std::pow(ERREPH_GLO, 2.0);
}


/* sbas ephemeris to satellite clock bias --------------------------------------
 * compute satellite clock bias with sbas ephemeris
 * args   : gtime_t time     I   time by satellite clock (gpst)
 *          seph_t *seph     I   sbas ephemeris
 * return : satellite clock bias (s)
 * notes  : see ref [3]
 *-----------------------------------------------------------------------------*/
double seph2clk(gtime_t time, const seph_t *seph)
{
    double t;
    int i;

    trace(4, "seph2clk: time=%s sat=%2d\n", time_str(time, 3), seph->sat);

    t = timediffweekcrossover(time, seph->t0);

    for (i = 0; i < 2; i++)
        {
            t -= seph->af0 + seph->af1 * t;
        }
    return seph->af0 + seph->af1 * t;
}


/* sbas ephemeris to satellite position and clock bias -------------------------
 * compute satellite position and clock bias with sbas ephemeris
 * args   : gtime_t time     I   time (gpst)
 *          seph_t  *seph    I   sbas ephemeris
 *          double  *rs      O   satellite position {x,y,z} (ecef) (m)
 *          double  *dts     O   satellite clock bias (s)
 *          double  *var     O   satellite position and clock variance (m^2)
 * return : none
 * notes  : see ref [3]
 *-----------------------------------------------------------------------------*/
void seph2pos(gtime_t time, const seph_t *seph, double *rs, double *dts,
    double *var)
{
    double t;
    int i;

    trace(4, "seph2pos: time=%s sat=%2d\n", time_str(time, 3), seph->sat);

    t = timediffweekcrossover(time, seph->t0);

    for (i = 0; i < 3; i++)
        {
            rs[i] = seph->pos[i] + seph->vel[i] * t + seph->acc[i] * t * t / 2.0;
        }
    *dts = seph->af0 + seph->af1 * t;

    *var = var_uraeph(seph->sva);
}


/* select ephemeris --------------------------------------------------------*/
eph_t *seleph(gtime_t time, int sat, int iode, const nav_t *nav)
{
    double t;
    double tmax;
    double tmin;
    int i;
    int j = -1;

    trace(4, "seleph  : time=%s sat=%2d iode=%d\n", time_str(time, 3), sat, iode);

    switch (satsys(sat, nullptr))
        {
        case SYS_QZS:
            tmax = MAXDTOE_QZS + 1.0;
            break;
        case SYS_GAL:
            tmax = MAXDTOE_GAL + 1.0;
            break;
        case SYS_BDS:
            tmax = MAXDTOE_BDS + 1.0;
            break;
        default:
            tmax = MAXDTOE + 1.0;
            break;
        }
    tmin = tmax + 1.0;

    for (i = 0; i < nav->n; i++)
        {
            if (nav->eph[i].sat != sat)
                {
                    continue;
                }
            if (iode >= 0 && nav->eph[i].iode != iode)
                {
                    continue;
                }
            if ((t = fabs(timediffweekcrossover(nav->eph[i].toe, time))) > tmax)
                {
                    continue;
                }
            if (iode >= 0)
                {
                    return nav->eph + i;
                }
            if (t <= tmin)
                {
                    j = i;
                    tmin = t;
                } /* toe closest to time */
        }
    if (iode >= 0 || j < 0)
        {
            trace(3, "no broadcast ephemeris: %s sat=%2d iode=%3d\n", time_str(time, 0),
                sat, iode);
            return nullptr;
        }
    return nav->eph + j;
}


/* select glonass ephemeris ------------------------------------------------*/
geph_t *selgeph(gtime_t time, int sat, int iode, const nav_t *nav)
{
    double t;
    double tmax = MAXDTOE_GLO;
    double tmin = tmax + 1.0;
    int i;
    int j = -1;

    trace(4, "selgeph : time=%s sat=%2d iode=%2d\n", time_str(time, 3), sat, iode);

    for (i = 0; i < nav->ng; i++)
        {
            if (nav->geph[i].sat != sat)
                {
                    continue;
                }
            if (iode >= 0 && nav->geph[i].iode != iode)
                {
                    continue;
                }
            if ((t = fabs(timediff(nav->geph[i].toe, time))) > tmax)
                {
                    continue;
                }
            if (iode >= 0)
                {
                    return nav->geph + i;
                }
            if (t <= tmin)
                {
                    j = i;
                    tmin = t;
                } /* toe closest to time */
        }
    if (iode >= 0 || j < 0)
        {
            trace(3, "no glonass ephemeris  : %s sat=%2d iode=%2d\n", time_str(time, 0),
                sat, iode);
            return nullptr;
        }
    return nav->geph + j;
}


/* select sbas ephemeris ---------------------------------------------------*/
seph_t *selseph(gtime_t time, int sat, const nav_t *nav)
{
    double t;
    double tmax = MAXDTOE_SBS;
    double tmin = tmax + 1.0;
    int i;
    int j = -1;

    trace(4, "selseph : time=%s sat=%2d\n", time_str(time, 3), sat);

    for (i = 0; i < nav->ns; i++)
        {
            if (nav->seph[i].sat != sat)
                {
                    continue;
                }
            if ((t = fabs(timediffweekcrossover(nav->seph[i].t0, time))) > tmax)
                {
                    continue;
                }
            if (t <= tmin)
                {
                    j = i;
                    tmin = t;
                } /* toe closest to time */
        }
    if (j < 0)
        {
            trace(3, "no sbas ephemeris     : %s sat=%2d\n", time_str(time, 0), sat);
            return nullptr;
        }
    return nav->seph + j;
}


/* satellite clock with broadcast ephemeris ----------------------------------*/
int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
    double *dts)
{
    eph_t *eph;
    geph_t *geph;
    seph_t *seph;
    int sys;

    trace(4, "ephclk  : time=%s sat=%2d\n", time_str(time, 3), sat);

    sys = satsys(sat, nullptr);

    if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_BDS)
        {
            if (!(eph = seleph(teph, sat, -1, nav)))
                {
                    return 0;
                }
            *dts = eph2clk(time, eph);
        }
    else if (sys == SYS_GLO)
        {
            if (!(geph = selgeph(teph, sat, -1, nav)))
                {
                    return 0;
                }
            *dts = geph2clk(time, geph);
        }
    else if (sys == SYS_SBS)
        {
            if (!(seph = selseph(teph, sat, nav)))
                {
                    return 0;
                }
            *dts = seph2clk(time, seph);
        }
    else
        {
            return 0;
        }

    return 1;
}


/* satellite position and clock by broadcast ephemeris -----------------------*/
int ephpos(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
    int iode, double *rs, double *dts, double *var, int *svh)
{
    eph_t *eph;
    geph_t *geph;
    seph_t *seph;
    double rst[3];
    double dtst[1];
    double tt = 1e-3;
    int i;
    int sys;

    trace(4, "ephpos  : time=%s sat=%2d iode=%d\n", time_str(time, 3), sat, iode);

    sys = satsys(sat, nullptr);

    *svh = -1;

    if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_BDS)
        {
            if (!(eph = seleph(teph, sat, iode, nav)))
                {
                    return 0;
                }

            eph2pos(time, eph, rs, dts, var);
            time = timeadd(time, tt);
            eph2pos(time, eph, rst, dtst, var);
            *svh = eph->svh;
        }
    else if (sys == SYS_GLO)
        {
            if (!(geph = selgeph(teph, sat, iode, nav)))
                {
                    return 0;
                }
            geph2pos(time, geph, rs, dts, var);
            time = timeadd(time, tt);
            geph2pos(time, geph, rst, dtst, var);
            *svh = geph->svh;
        }
    else if (sys == SYS_SBS)
        {
            if (!(seph = selseph(teph, sat, nav)))
                {
                    return 0;
                }

            seph2pos(time, seph, rs, dts, var);
            time = timeadd(time, tt);
            seph2pos(time, seph, rst, dtst, var);
            *svh = seph->svh;
        }
    else
        {
            return 0;
        }

    /* satellite velocity and clock drift by differential approx */
    for (i = 0; i < 3; i++)
        {
            rs[i + 3] = (rst[i] - rs[i]) / tt;
        }
    dts[1] = (dtst[0] - dts[0]) / tt;

    return 1;
}


/* satellite position and clock with sbas correction -------------------------*/
int satpos_sbas(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
    double *rs, double *dts, double *var, int *svh)
{
    const sbssatp_t *sbs;
    int i;

    trace(4, "satpos_sbas: time=%s sat=%2d\n", time_str(time, 3), sat);

    /* search sbas satellite correciton */
    for (i = 0; i < nav->sbssat.nsat; i++)
        {
            sbs = nav->sbssat.sat + i;
            if (sbs->sat == sat)
                {
                    break;
                }
        }
    if (i >= nav->sbssat.nsat)
        {
            trace(2, "no sbas correction for orbit: %s sat=%2d\n", time_str(time, 0), sat);
            ephpos(time, teph, sat, nav, -1, rs, dts, var, svh);
            *svh = -1;
            return 0;
        }
    /* satellite position and clock by broadcast ephemeris */
    if (!ephpos(time, teph, sat, nav, sbs->lcorr.iode, rs, dts, var, svh))
        {
            return 0;
        }

    /* sbas satellite correction (long term and fast) */
    if (sbssatcorr(time, sat, nav, rs, dts, var))
        {
            return 1;
        }
    *svh = -1;
    return 0;
}


/* satellite position and clock with ssr correction --------------------------*/
int satpos_ssr(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
    int opt, double *rs, double *dts, double *var, int *svh)
{
    const ssr_t *ssr;
    eph_t *eph;
    double t1;
    double t2;
    double t3;
    double er[3];
    double ea[3];
    double ec[3];
    double rc[3];
    double deph[3];
    double dclk;
    double dant[3] = {0};
    double tk;
    int i;
    int sys;

    trace(4, "satpos_ssr: time=%s sat=%2d\n", time_str(time, 3), sat);

    ssr = nav->ssr + sat - 1;

    if (!ssr->t0[0].time)
        {
            trace(2, "no ssr orbit correction: %s sat=%2d\n", time_str(time, 0), sat);
            return 0;
        }
    if (!ssr->t0[1].time)
        {
            trace(2, "no ssr clock correction: %s sat=%2d\n", time_str(time, 0), sat);
            return 0;
        }
    /* inconsistency between orbit and clock correction */
    if (ssr->iod[0] != ssr->iod[1])
        {
            trace(2, "inconsist ssr correction: %s sat=%2d iod=%d %d\n",
                time_str(time, 0), sat, ssr->iod[0], ssr->iod[1]);
            *svh = -1;
            return 0;
        }
    t1 = timediffweekcrossover(time, ssr->t0[0]);
    t2 = timediffweekcrossover(time, ssr->t0[1]);
    t3 = timediffweekcrossover(time, ssr->t0[2]);

    /* ssr orbit and clock correction (ref [4]) */
    if (fabs(t1) > MAXAGESSR || fabs(t2) > MAXAGESSR)
        {
            trace(2, "age of ssr error: %s sat=%2d t=%.0f %.0f\n", time_str(time, 0),
                sat, t1, t2);
            *svh = -1;
            return 0;
        }
    if (ssr->udi[0] >= 1.0)
        {
            t1 -= ssr->udi[0] / 2.0;
        }
    if (ssr->udi[1] >= 1.0)
        {
            t2 -= ssr->udi[0] / 2.0;
        }

    for (i = 0; i < 3; i++)
        {
            deph[i] = ssr->deph[i] + ssr->ddeph[i] * t1;
        }
    dclk = ssr->dclk[0] + ssr->dclk[1] * t2 + ssr->dclk[2] * t2 * t2;

    /* ssr highrate clock correction (ref [4]) */
    if (ssr->iod[0] == ssr->iod[2] && ssr->t0[2].time && fabs(t3) < MAXAGESSR_HRCLK)
        {
            dclk += ssr->hrclk;
        }
    if (norm_rtk(deph, 3) > MAXECORSSR || fabs(dclk) > MAXCCORSSR)
        {
            trace(3, "invalid ssr correction: %s deph=%.1f dclk=%.1f\n",
                time_str(time, 0), norm_rtk(deph, 3), dclk);
            *svh = -1;
            return 0;
        }
    /* satellite position and clock by broadcast ephemeris */
    if (!ephpos(time, teph, sat, nav, ssr->iode, rs, dts, var, svh))
        {
            return 0;
        }

    /* satellite clock for gps, galileo and qzss */
    sys = satsys(sat, nullptr);
    if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_BDS)
        {
            if (!(eph = seleph(teph, sat, ssr->iode, nav)))
                {
                    return 0;
                }

            /* satellite clock by clock parameters */
            tk = timediffweekcrossover(time, eph->toc);
            dts[0] = eph->f0 + eph->f1 * tk + eph->f2 * tk * tk;
            dts[1] = eph->f1 + 2.0 * eph->f2 * tk;

            /* relativity correction */
            dts[0] -= 2.0 * dot(rs, rs + 3, 3) / SPEED_OF_LIGHT_M_S / SPEED_OF_LIGHT_M_S;
        }
    /* radial-along-cross directions in ecef */
    if (!normv3(rs + 3, ea))
        {
            return 0;
        }
    cross3(rs, rs + 3, rc);
    if (!normv3(rc, ec))
        {
            *svh = -1;
            return 0;
        }
    cross3(ea, ec, er);

    /* satellite antenna offset correction */
    if (opt)
        {
            satantoff(time, rs, sat, nav, dant);
        }
    for (i = 0; i < 3; i++)
        {
            rs[i] += -(er[i] * deph[0] + ea[i] * deph[1] + ec[i] * deph[2]) + dant[i];
        }
    /* t_corr = t_sv - (dts(brdc) + dclk(ssr) / SPEED_OF_LIGHT_M_S) (ref [10] eq.3.12-7) */
    dts[0] += dclk / SPEED_OF_LIGHT_M_S;

    /* variance by ssr ura */
    *var = var_urassr(ssr->ura);

    trace(5, "satpos_ssr: %s sat=%2d deph=%6.3f %6.3f %6.3f er=%6.3f %6.3f %6.3f dclk=%6.3f var=%6.3f\n",
        time_str(time, 2), sat, deph[0], deph[1], deph[2], er[0], er[1], er[2], dclk, *var);

    return 1;
}


/* satellite position and clock ------------------------------------------------
 * compute satellite position, velocity and clock
 * args   : gtime_t time     I   time (gpst)
 *          gtime_t teph     I   time to select ephemeris (gpst)
 *          int    sat       I   satellite number
 *          nav_t  *nav      I   navigation data
 *          int    ephopt    I   ephemeris option (EPHOPT_???)
 *          double *rs       O   sat position and velocity (ecef)
 *                               {x,y,z,vx,vy,vz} (m|m/s)
 *          double *dts      O   sat clock {bias,drift} (s|s/s)
 *          double *var      O   sat position and clock error variance (m^2)
 *          int    *svh      O   sat health flag (-1:correction not available)
 * return : status (1:ok,0:error)
 * notes  : satellite position is referenced to antenna phase center
 *          satellite clock does not include code bias correction (tgd or bgd)
 *-----------------------------------------------------------------------------*/
int satpos(gtime_t time, gtime_t teph, int sat, int ephopt,
    const nav_t *nav, double *rs, double *dts, double *var,
    int *svh)
{
    trace(4, "satpos  : time=%s sat=%2d ephopt=%d\n", time_str(time, 3), sat, ephopt);

    *svh = 0;

    switch (ephopt)
        {
        case EPHOPT_BRDC:
            return ephpos(time, teph, sat, nav, -1, rs, dts, var, svh);
        case EPHOPT_SBAS:
            return satpos_sbas(time, teph, sat, nav, rs, dts, var, svh);
        case EPHOPT_SSRAPC:
            return satpos_ssr(time, teph, sat, nav, 0, rs, dts, var, svh);
        case EPHOPT_SSRCOM:
            return satpos_ssr(time, teph, sat, nav, 1, rs, dts, var, svh);
        case EPHOPT_PREC:
            if (!peph2pos(time, sat, nav, 1, rs, dts, var))
                {
                    break;
                }
            else
                {
                    return 1;
                }
            // TODO: enable lex
            // case EPHOPT_LEX   :
            //    if (!lexeph2pos(time, sat, nav, rs, dts, var)) break; else return 1;
        }
    *svh = -1;
    return 0;
}


/* satellite positions and clocks ----------------------------------------------
 * compute satellite positions, velocities and clocks
 * args   : gtime_t teph     I   time to select ephemeris (gpst)
 *          obsd_t *obs      I   observation data
 *          int    n         I   number of observation data
 *          nav_t  *nav      I   navigation data
 *          int    ephopt    I   ephemeris option (EPHOPT_???)
 *          double *rs       O   satellite positions and velocities (ecef)
 *          double *dts      O   satellite clocks
 *          double *var      O   sat position and clock error variances (m^2)
 *          int    *svh      O   sat health flag (-1:correction not available)
 * return : none
 * notes  : rs [(0:2)+i*6]= obs[i] sat position {x,y,z} (m)
 *          rs [(3:5)+i*6]= obs[i] sat velocity {vx,vy,vz} (m/s)
 *          dts[(0:1)+i*2]= obs[i] sat clock {bias,drift} (s|s/s)
 *          var[i]        = obs[i] sat position and clock error variance (m^2)
 *          svh[i]        = obs[i] sat health flag
 *          if no navigation data, set 0 to rs[], dts[], var[] and svh[]
 *          satellite position and clock are values at signal transmission time
 *          satellite position is referenced to antenna phase center
 *          satellite clock does not include code bias correction (tgd or bgd)
 *          any pseudorange and broadcast ephemeris are always needed to get
 *          signal transmission time
 *-----------------------------------------------------------------------------*/
void satposs(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav,
    int ephopt, double *rs, double *dts, double *var, int *svh)
{
    std::vector<gtime_t> time(MAXOBS);
    double dt;
    double pr;
    int i;
    int j;

    trace(3, "satposs : teph=%s n=%d ephopt=%d\n", time_str(teph, 3), n, ephopt);

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            for (j = 0; j < 6; j++)
                {
                    rs[j + i * 6] = 0.0;
                }
            for (j = 0; j < 2; j++)
                {
                    dts[j + i * 2] = 0.0;
                }
            var[i] = 0.0;
            svh[i] = 0;

            /* search any pseudorange */
            for (j = 0, pr = 0.0; j < NFREQ; j++)
                {
                    if ((pr = obs[i].P[j]) != 0.0)
                        {
                            break;
                        }
                }

            if (j >= NFREQ)
                {
                    trace(2, "no pseudorange %s sat=%2d\n", time_str(obs[i].time, 3), obs[i].sat);
                    continue;
                }
            /* transmission time by satellite clock */
            time[i] = timeadd(obs[i].time, -pr / SPEED_OF_LIGHT_M_S);

            /* satellite clock bias by broadcast ephemeris */
            if (!ephclk(time[i], teph, obs[i].sat, nav, &dt))
                {
                    trace(3, "no broadcast clock %s sat=%2d\n", time_str(time[i], 3), obs[i].sat);
                    continue;
                }
            time[i] = timeadd(time[i], -dt);

            /* satellite position and clock at transmission time */
            if (!satpos(time[i], teph, obs[i].sat, ephopt, nav, rs + i * 6, dts + i * 2, var + i,
                    svh + i))
                {
                    trace(3, "no ephemeris %s sat=%2d\n", time_str(time[i], 3), obs[i].sat);
                    continue;
                }
            /* if no precise clock available, use broadcast clock instead */
            if (dts[i * 2] == 0.0)
                {
                    if (!ephclk(time[i], teph, obs[i].sat, nav, dts + i * 2))
                        {
                            continue;
                        }
                    dts[1 + i * 2] = 0.0;
                    *var = std::pow(STD_BRDCCLK, 2.0);
                }
        }
    for (i = 0; i < n && i < MAXOBS; i++)
        {
            trace(4, "%s sat=%2d rs=%13.3f %13.3f %13.3f dts=%12.3f var=%7.3f svh=%02X\n",
                time_str(time[i], 6), obs[i].sat, rs[i * 6], rs[1 + i * 6], rs[2 + i * 6],
                dts[i * 2] * 1e9, var[i], svh[i]);
        }
}
