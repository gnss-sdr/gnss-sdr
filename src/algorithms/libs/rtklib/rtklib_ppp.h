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

#define MAX_PPP(x,y)    ((x)>(y)?(x):(y))
#define ROUND_PPP(x)    (int)floor((x)+0.5)

const int MAX_ITER =  8;               /* max number of iterations */
const double MAX_STD_FIX = 0.15;            /* max std-dev (3d) to fix solution */
const int MIN_NSAT_SOL = 4;              /* min satellite number for solution */
const double THRES_REJECT = 4.0;            /* reject threshold of posfit-res (sigma) */

const double THRES_MW_JUMP = 10.0;

const double VAR_POS_PPP =  std::pow(60.0, 2.0);       /* init variance receiver position (m^2) */
const double VAR_CLK = std::pow(60.0, 2.0);       /* init variance receiver clock (m^2) */
const double VAR_ZTD = std::pow( 0.6, 2.0);      /* init variance ztd (m^2) */
const double VAR_GRA_PPP = std::pow(0.01, 2.0);       /* init variance gradient (m^2) */
const double VAR_DCB = std::pow(30.0, 2.0);       /* init variance dcb (m^2) */
const double VAR_BIAS = std::pow(60.0, 2.0);       /* init variance phase-bias (m^2) */
const double VAR_IONO = std::pow(60.0, 2.0);       /* init variance iono-delay */
const double VAR_GLO_IFB = std::pow( 0.6, 2.0);       /* variance of glonass ifb */

const double EFACT_GPS_L5 = 10.0;           /* error factor of GPS/QZS L5 */

const double MUDOT_GPS = (0.00836*D2R);   /* average angular velocity GPS (rad/s) */
const double MUDOT_GLO = (0.00888*D2R);   /* average angular velocity GLO (rad/s) */
const double EPS0_GPS = (13.5*D2R);      /* max shadow crossing angle GPS (rad) */
const double EPS0_GLO = (14.2*D2R);      /* max shadow crossing angle GLO (rad) */
const double T_POSTSHADOW = 1800.0;         /* post-shadow recovery time (s) */
const double QZS_EC_BETA = 20.0;            /* max beta angle for qzss Ec (deg) */

/* number and index of states */
#define NF_PPP(opt)     ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)
#define NP_PPP(opt)     ((opt)->dynamics?9:3)
#define NC_PPP(opt)     (NSYS)
#define NT_PPP(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt==TROPOPT_EST?1:3))
#define NI_PPP(opt)     ((opt)->ionoopt==IONOOPT_EST?MAXSAT:0)
#define ND_PPP(opt)     ((opt)->nf>=3?1:0)
#define NR_PPP(opt)     (NP_PPP(opt)+NC_PPP(opt)+NT_PPP(opt)+NI_PPP(opt)+ND_PPP(opt))
#define NB_PPP(opt)     (NF_PPP(opt)*MAXSAT)
#define NX_PPP(opt)     (NR_PPP(opt)+NB_PPP(opt))
#define IC_PPP(s,opt)   (NP_PPP(opt)+(s))
#define IT_PPP(opt)     (NP_PPP(opt)+NC_PPP(opt))
#define II_PPP(s,opt)   (NP_PPP(opt)+NC_PPP(opt)+NT_PPP(opt)+(s)-1)
#define ID_PPP(opt)     (NP_PPP(opt)+NC_PPP(opt)+NT_PPP(opt)+NI_PPP(opt))
#define IB_PPP(s,f,opt) (NR_PPP(opt)+MAXSAT*(f)+(s)-1)


int pppcorr_read(pppcorr_t *corr, const char *file);
void pppcorr_free(pppcorr_t *corr);
int pppcorr_trop(const pppcorr_t *corr, gtime_t time, const double *pos,
        double *trp, double *std);
int pppcorr_stec(const pppcorr_t *corr, gtime_t time, const double *pos,
        double *ion, double *std);
int ppp_ar(rtk_t *rtk, const obsd_t *obs, int n, int *exc,
        const nav_t *nav, const double *azel, double *x, double *P);

double STD(rtk_t *rtk, int i);
int pppoutstat(rtk_t *rtk, char *buff);
void testeclipse(const obsd_t *obs, int n, const nav_t *nav, double *rs);
double yaw_nominal(double beta, double mu);
double yaw_angle(int sat, const char *type, int opt, double beta, double mu,
        double *yaw);
int sat_yaw(gtime_t time, int sat, const char *type, int opt,
        const double *rs, double *exs, double *eys);


int model_phw(gtime_t time, int sat, const char *type, int opt,
        const double *rs, const double *rr, double *phw);

double varerr(int sat, int sys, double el, int freq, int type,
        const prcopt_t *opt);
void initx(rtk_t *rtk, double xi, double var, int i);

double gfmeas(const obsd_t *obs, const nav_t *nav);

double mwmeas(const obsd_t *obs, const nav_t *nav);

void corr_meas(const obsd_t *obs, const nav_t *nav, const double *azel,
        const prcopt_t *opt, const double *dantr,
        const double *dants, double phw, double *L, double *P,
        double *Lc, double *Pc);


void detslp_ll(rtk_t *rtk, const obsd_t *obs, int n);

void detslp_gf(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);

void detslp_mw(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);

void udpos_ppp(rtk_t *rtk);

void udclk_ppp(rtk_t *rtk);

void udtrop_ppp(rtk_t *rtk);

void udiono_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);

void uddcb_ppp(rtk_t *rtk);

void udbias_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);


void udstate_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);

void satantpcv(const double *rs, const double *rr, const pcv_t *pcv,
        double *dant);

double trop_model_prec(gtime_t time, const double *pos,
        const double *azel, const double *x, double *dtdx,
        double *var);

int model_trop(gtime_t time, const double *pos, const double *azel,
        const prcopt_t *opt, const double *x, double *dtdx,
        const nav_t *nav, double *dtrp, double *var);

int model_iono(gtime_t time, const double *pos, const double *azel,
        const prcopt_t *opt, int sat, const double *x,
        const nav_t *nav, double *dion, double *var);


int const_corr(const obsd_t *obs, int n, const int *exc,
        const nav_t *nav, const double *x, const double *pos,
        const double *azel, rtk_t *rtk, double *v, double *H,
        double *var);

int ppp_res(int post, const obsd_t *obs, int n, const double *rs,
        const double *dts, const double *var_rs, const int *svh,
        const double *dr, int *exc, const nav_t *nav,
        const double *x, rtk_t *rtk, double *v, double *H, double *R,
        double *azel);

int pppnx(const prcopt_t *opt);

void update_stat(rtk_t *rtk, const obsd_t *obs, int n, int stat);

int test_hold_amb(rtk_t *rtk);


void pppos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);



#endif
