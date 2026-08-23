/*!
 * \file rtklib_pntpos.h
 * \brief standard code-based positioning
 * \authors <ul>
 *          <li> 2007-2013, T. Takasu
 *          <li> 2017, Javier Arribas
 *          <li> 2017, Carles Fernandez
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
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RTKLIB_PNTPOS_H
#define GNSS_SDR_RTKLIB_PNTPOS_H

#include "rtklib.h"
#include "rtklib_rtkcmn.h"

/* constants -----------------------------------------------------------------*/
const int NX = 4 + 4;                    //!< # of estimated parameters: pos (3), GPS clock, GLO/GAL/BDS/QZS-GPS offsets
const int MAXITR = 10;                   //!< max number of iteration for point pos
const double VARERR_MIN_EL = 5.0 * D2R;  //!< minimum elevation for measurement error variance (rad)
const double ERR_ION = 5.0;              //!< ionospheric delay std (m)
const double ERR_TROP = 3.0;             //!< tropspheric delay std (m)


/* pseudorange measurement error variance ------------------------------------*/
double varerr(const prcopt_t *opt, const obsd_t *obs, double el, int sys);

/* get tgd parameter (m) -----------------------------------------------------*/
double gettgd(int sat, const nav_t *nav);
double gettgd(int sat, const nav_t *nav, int tgd_index);
/* BDS DNAV/CNAV1 TGD selection by observation code (CODE_L1D/L1P/L1X/...) */
double gettgd_bds_by_obs_code(int sat, const nav_t *nav, unsigned char obs_code);

/* select Galileo BGD from observation and ephemeris provenance -------------*/
int galileo_bgd_index(unsigned char observation_code, int sat, const nav_t *nav);

/* get isc parameter (m) -----------------------------------------------------*/
double getiscl1(int sat, const nav_t *nav);
double getiscl2(int sat, const nav_t *nav);
double getiscl5i(int sat, const nav_t *nav);
double getiscl5q(int sat, const nav_t *nav);

/* psendorange with code bias correction -------------------------------------
 * iono_scale (O) is the multiplier the caller must apply to the modeled L1
 * ionospheric delay so that it is consistent with the returned pseudorange:
 * 1.0 for L1-referenced measurements, (f_L1/f_band)^2 for single-band
 * measurements on another band, 0.0 for ionosphere-free combinations  */
double prange(const obsd_t *obs, const nav_t *nav, const double *azel,
    int iter, const prcopt_t *opt, double *var, double *iono_scale);

/* ionospheric correction ------------------------------------------------------
 * compute ionospheric correction
 * args   : gtime_t time     I   time
 *          nav_t  *nav      I   navigation data
 *          int    sat       I   satellite number
 *          double *pos      I   receiver position {lat,lon,h} (rad|m)
 *          double *azel     I   azimuth/elevation angle {az,el} (rad)
 *          int    ionoopt   I   ionospheric correction option (IONOOPT_???)
 *          double *ion      O   ionospheric delay (L1) (m)
 *          double *var      O   ionospheric delay (L1) variance (m^2)
 *          unsigned char obs_code I optional observation code (CODE_???); for BDS
 *                                   CODE_L1D/L1P/L1X selects BDGIM at FREQ1 (B1C)
 * return : status(1:ok,0:error)
 *-----------------------------------------------------------------------------*/
int ionocorr(gtime_t time, const nav_t *nav, int sat, const double *pos,
    const double *azel, int ionoopt, double *ion, double *var, unsigned char obs_code = 0);
/* tropospheric correction -----------------------------------------------------
 * compute tropospheric correction
 * args   : gtime_t time     I   time
 *          nav_t  *nav      I   navigation data
 *          double *pos      I   receiver position {lat,lon,h} (rad|m)
 *          double *azel     I   azimuth/elevation angle {az,el} (rad)
 *          int    tropopt   I   tropospheric correction option (TROPOPT_???)
 *          double *trp      O   tropospheric delay (m)
 *          double *var      O   tropospheric delay variance (m^2)
 * return : status(1:ok,0:error)
 *-----------------------------------------------------------------------------*/
int tropcorr(gtime_t time, const nav_t *nav, const double *pos,
    const double *azel, int tropopt, double *trp, double *var);

/* pseudorange residuals -----------------------------------------------------*/
int rescode(int iter, const obsd_t *obs, int n, const double *rs,
    const double *dts, const double *vare, const int *svh,
    const nav_t *nav, const double *x, const prcopt_t *opt,
    double *v, double *H, double *var, double *azel, int *vsat,
    double *resp, int *ns);

/* validate solution ---------------------------------------------------------*/
int valsol(const double *azel, const int *vsat, int n,
    const prcopt_t *opt, const double *v, int nv, int nx,
    char *msg);

/* estimate receiver position ------------------------------------------------*/
int estpos(const obsd_t *obs, int n, const double *rs, const double *dts,
    const double *vare, const int *svh, const nav_t *nav,
    const prcopt_t *opt, sol_t *sol, double *azel, int *vsat,
    double *resp, char *msg);

/* raim fde (failure detection and exclution) -------------------------------*/
int raim_fde(const obsd_t *obs, int n, const double *rs,
    const double *dts, const double *vare, const int *svh,
    const nav_t *nav, const prcopt_t *opt, sol_t *sol,
    double *azel, int *vsat, double *resp, char *msg);

/* doppler residuals ---------------------------------------------------------*/
int resdop(const obsd_t *obs, int n, const double *rs, const double *dts,
    const nav_t *nav, const double *rr, const double *x,
    const double *azel, const int *vsat, double *v, double *H);

/* estimate receiver velocity ------------------------------------------------*/
void estvel(const obsd_t *obs, int n, const double *rs, const double *dts,
    const nav_t *nav, const prcopt_t *opt, sol_t *sol,
    const double *azel, const int *vsat);

/*!
 * \brief single-point positioning
 * compute receiver position, velocity, clock bias by single-point positioning
 * with pseudorange and doppler observables
 * args   : obsd_t *obs      I   observation data
 *          int    n         I   number of observation data
 *          nav_t  *nav      I   navigation data
 *          prcopt_t *opt    I   processing options
 *          sol_t  *sol      IO  solution
 *          double *azel     IO  azimuth/elevation angle (rad) (NULL: no output)
 *          ssat_t *ssat     IO  satellite status              (NULL: no output)
 *          char   *msg      O   error message for error exit
 * return : status(1:ok,0:error)
 * notes  : assuming sbas-gps, galileo-gps, qzss-gps, compass-gps time offset and
 *          receiver bias are negligible (only involving glonass-gps time offset
 *          and receiver bias)
 */
int pntpos(const obsd_t *obs, int n, const nav_t *nav,
    const prcopt_t *opt, sol_t *sol, double *azel, ssat_t *ssat,
    char *msg);

#endif  // GNSS_SDR_RTKLIB_PNTPOS_H
