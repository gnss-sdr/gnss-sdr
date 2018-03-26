/*!
 * \file rtklib_ppp.h
 * \brief Precise Point Positioning
 * \authors <ul>
 *          <li> 2007-2008, T. Takasu
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
 * -------------------------------------------------------------------------
 * Copyright (C) 2007-2008, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------*/


#ifndef GNSS_SDR_RTKLIB_PPP_H_
#define GNSS_SDR_RTKLIB_PPP_H_

#include "rtklib.h"


#define MIN_PPP(x, y) ((x) <= (y) ? (x) : (y))
#define ROUND_PPP(x) (int)floor((x) + 0.5)

#define SWAP_I(x, y)    \
    do                  \
        {               \
            int _z = x; \
            x = y;      \
            y = _z;     \
        }               \
    while (0)
#define SWAP_D(x, y)       \
    do                     \
        {                  \
            double _z = x; \
            x = y;         \
            y = _z;        \
        }                  \
    while (0)

const double MIN_ARC_GAP = 300.0;          /* min arc gap (s) */
const double CONST_AMB = 0.001;            /* constraint to fixed ambiguity */
const double THRES_RES = 0.3;              /* threshold of residuals test (m) */
const double LOG_PI = 1.14472988584940017; /* log(pi) */
const double SQRT2 = 1.41421356237309510;  /* sqrt(2) */

const double VAR_POS_PPP = std::pow(100.0, 2.0); /* init variance receiver position (m^2) */
const double VAR_CLK = std::pow(100.0, 2.0);     /* init variance receiver clock (m^2) */
const double VAR_ZTD = std::pow(0.3, 2.0);       /* init variance ztd (m^2) */
const double VAR_GRA_PPP = std::pow(0.001, 2.0); /* init variance gradient (m^2) */
const double VAR_BIAS = std::pow(100.0, 2.0);    /* init variance phase-bias (m^2) */

const double VAR_IONO_OFF = std::pow(10.0, 2.0); /* variance of iono-model-off */


/* functions originally included in RTKLIB/src/ppp_ar.c v2.4.2*/
double lam_LC(int i, int j, int k);

double L_LC(int i, int j, int k, const double *L);

double P_LC(int i, int j, int k, const double *P);

double var_LC(int i, int j, int k, double sig);

double q_gamma(double a, double x, double log_gamma_a);

double p_gamma(double a, double x, double log_gamma_a);

double f_erfc(double x);

double conffunc(int N, double B, double sig);

void average_LC(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav, const double *azel);

int fix_amb_WL(rtk_t *rtk, const nav_t *nav, int sat1, int sat2, int *NW);

int is_depend(int sat1, int sat2, int *flgs, int *max_flg);

int sel_amb(int *sat1, int *sat2, double *N, double *var, int n);

int fix_sol(rtk_t *rtk, const int *sat1, const int *sat2, const double *NC, int n);

int fix_amb_ROUND(rtk_t *rtk, int *sat1, int *sat2, const int *NW, int n);

int fix_amb_ILS(rtk_t *rtk, int *sat1, int *sat2, int *NW, int n);

int pppamb(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav, const double *azel);


/* functions originally included in RTKLIB/src/ppp.c v2.4.2 */
void pppoutsolstat(rtk_t *rtk, int level, FILE *fp);

void tide_pl(const double *eu, const double *rp, double GMp, const double *pos, double *dr);

void tide_solid(const double *rsun, const double *rmoon,
    const double *pos, const double *E, double gmst, int opt,
    double *dr);

void tide_oload(gtime_t tut, const double *odisp, double *denu);

void iers_mean_pole(gtime_t tut, double *xp_bar, double *yp_bar);

void tide_pole(gtime_t tut, const double *pos, const double *erpv, double *denu);

void tidedisp(gtime_t tutc, const double *rr, int opt, const erp_t *erp,
    const double *odisp, double *dr);

void testeclipse(const obsd_t *obs, int n, const nav_t *nav, double *rs);

double varerr(int sat, int sys, double el, int type, const prcopt_t *opt);

void initx(rtk_t *rtk, double xi, double var, int i);

int ifmeas(const obsd_t *obs, const nav_t *nav, const double *azel,
    const prcopt_t *opt, const double *dantr, const double *dants,
    double phw, double *meas, double *var);

double gettgd_ppp(int sat, const nav_t *nav);

int corr_ion(gtime_t time, const nav_t *nav, int sat, const double *pos,
    const double *azel, int ionoopt, double *ion, double *var,
    int *brk);

int corrmeas(const obsd_t *obs, const nav_t *nav, const double *pos,
    const double *azel, const prcopt_t *opt,
    const double *dantr, const double *dants, double phw,
    double *meas, double *var, int *brk);

double gfmeas(const obsd_t *obs, const nav_t *nav);

void udpos_ppp(rtk_t *rtk);

void udclk_ppp(rtk_t *rtk);

void udtrop_ppp(rtk_t *rtk);

void detslp_ll(rtk_t *rtk, const obsd_t *obs, int n);

void detslp_gf(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);

void udbias_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);

void udstate_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);

void satantpcv(const double *rs, const double *rr, const pcv_t *pcv, double *dant);

double prectrop(gtime_t time, const double *pos, const double *azel,
    const prcopt_t *opt, const double *x, double *dtdx,
    double *var);

int res_ppp(int iter, const obsd_t *obs, int n, const double *rs,
    const double *dts, const double *vare, const int *svh,
    const nav_t *nav, const double *x, rtk_t *rtk, double *v,
    double *H, double *R, double *azel);

int pppnx(const prcopt_t *opt);

void pppos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);

#endif
