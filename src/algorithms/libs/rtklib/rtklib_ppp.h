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


#define SQR(x)      ((x)*(x))
#define MIN(x,y)    ((x)<=(y)?(x):(y))
#define ROUND(x)        (int)floor((x)+0.5)

#define SWAP_I(x,y)     do {int _z=x; x=y; y=_z;} while (0)
#define SWAP_D(x,y)     do {double _z=x; x=y; y=_z;} while (0)

#define MIN_ARC_GAP     300.0       /* min arc gap (s) */
#define CONST_AMB       0.001       /* constraint to fixed ambiguity */
#define THRES_RES       0.3         /* threashold of residuals test (m) */
#define LOG_PI          1.14472988584940017 /* log(pi) */
#define SQRT2           1.41421356237309510 /* sqrt(2) */

#define AS2R        (D2R/3600.0)    /* arc sec to radian */
//#define GME         3.986004415E+14 /* earth gravitational constant */
//const double GMS = 1.327124E+20;    /* sun gravitational constant */
//#define GMM         4.902801E+12    /* moon gravitational constant */

                                    /* initial variances */
#define VAR_POS     SQR(100.0)      /*   receiver position (m^2) */
#define VAR_CLK     SQR(100.0)      /*   receiver clock (m^2) */
#define VAR_ZTD     SQR(  0.3)      /*   ztd (m^2) */
#define VAR_GRA     SQR(0.001)      /*   gradient (m^2) */
#define VAR_BIAS    SQR(100.0)      /*   phase-bias (m^2) */

#define VAR_IONO_OFF SQR(10.0)      /* variance of iono-model-off */

#define ERR_SAAS    0.3             /* saastamoinen model error std (m) */
#define ERR_BRDCI   0.5             /* broadcast iono model error factor */
#define ERR_CBIAS   0.3             /* code bias error std (m) */
#define REL_HUMI    0.7             /* relative humidity for saastamoinen model */

#define NP(opt)     ((opt)->dynamics?9:3) /* number of pos solution */
#define IC(s,opt)   (NP(opt)+(s))      /* state index of clocks (s=0:gps,1:glo) */
#define IT(opt)     (IC(0,opt)+NSYS)   /* state index of tropos */
#define NR(opt)     (IT(opt)+((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt==TROPOPT_EST?1:3)))
                                       /* number of solutions */
#define IB(s,opt)   (NR(opt)+(s)-1)    /* state index of phase bias */
#define NX(opt)     (IB(MAXSAT,opt)+1) /* number of estimated states */



double lam_LC(int i, int j, int k);

double L_LC(int i, int j, int k, const double *L);

double P_LC(int i, int j, int k, const double *P);

double var_LC(int i, int j, int k, double sig);

double q_gamma(double a, double x, double log_gamma_a);

double p_gamma(double a, double x, double log_gamma_a);

double f_erfc(double x);

double conffunc(int N, double B, double sig);

void average_LC(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav,
                       const double *azel);

int fix_amb_WL(rtk_t *rtk, const nav_t *nav, int sat1, int sat2, int *NW);

int is_depend(int sat1, int sat2, int *flgs, int *max_flg);

int sel_amb(int *sat1, int *sat2, double *N, double *var, int n);

int fix_sol(rtk_t *rtk, const int *sat1, const int *sat2,
                   const double *NC, int n);

int fix_amb_ROUND(rtk_t *rtk, int *sat1, int *sat2, const int *NW, int n);

int fix_amb_ILS(rtk_t *rtk, int *sat1, int *sat2, int *NW, int n);


int pppamb(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav,
                  const double *azel);





void pppoutsolstat(rtk_t *rtk, int level, FILE *fp);

void tide_pl(const double *eu, const double *rp, double GMp,
                    const double *pos, double *dr);

void tide_solid(const double *rsun, const double *rmoon,
                const double *pos, const double *E, double gmst, int opt,
                double *dr);

void tide_oload(gtime_t tut, const double *odisp, double *denu);

void iers_mean_pole(gtime_t tut, double *xp_bar, double *yp_bar);

void tide_pole(gtime_t tut, const double *pos, const double *erpv,
                     double *denu);

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

void satantpcv(const double *rs, const double *rr, const pcv_t *pcv,
                      double *dant);

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
