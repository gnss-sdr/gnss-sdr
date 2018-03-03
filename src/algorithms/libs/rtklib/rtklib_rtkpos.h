/*!
 * \file rtklib_rtkpos.h
 * \brief rtklib ppp-related functions
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
 * -------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
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
 *
 *----------------------------------------------------------------------------*/

#ifndef GNSS_SDR_RTKLIB_RKTPOS_H_
#define GNSS_SDR_RTKLIB_RKTPOS_H_

#include "rtklib.h"
#include "rtklib_rtkcmn.h"

/* constants/macros ----------------------------------------------------------*/
const double VAR_POS = std::pow(30.0, 2.0);   /* initial variance of receiver pos (m^2) */
const double VAR_VEL = std::pow(10.0, 2.0);   /* initial variance of receiver vel ((m/s)^2) */
const double VAR_ACC = std::pow(10.0, 2.0);   /* initial variance of receiver acc ((m/ss)^2) */
const double VAR_HWBIAS = std::pow(1.0, 2.0); /* initial variance of h/w bias ((m/MHz)^2) */
const double VAR_GRA = std::pow(0.001, 2.0);  /* initial variance of gradient (m^2) */
const double INIT_ZWD = 0.15;                 /* initial zwd (m) */

const double PRN_HWBIA = 1E-6; /* process noise of h/w bias (m/MHz/sqrt(s)) */
const double MAXAC = 30.0;     /* max accel for doppler slip detection (m/s^2) */

const double VAR_HOLDAMB = 0.001; /* constraint to hold ambiguity (cycle^2) */

const double TTOL_MOVEB = (1.0 + 2 * DTTOL);
/* time sync tolerance for moving-baseline (s) */

/* number of parameters (pos,ionos,tropos,hw-bias,phase-bias,real,estimated) */


/* state variable index */
#define II_RTK(s, opt) (NP_RTK(opt) + (s)-1)                               /* ionos (s:satellite no) */
#define IT_RTK(r, opt) (NP_RTK(opt) + NI_RTK(opt) + NT_RTK(opt) / 2 * (r)) /* tropos (r:0=rov,1:ref) */
#define IL_RTK(f, opt) (NP_RTK(opt) + NI_RTK(opt) + NT_RTK(opt) + (f))     /* receiver h/w bias */
#define IB_RTK(s, f, opt) (NR_RTK(opt) + MAXSAT * (f) + (s)-1)             /* phase bias (s:satno,f:freq) */

int rtkopenstat(const char *file, int level);

void rtkclosestat(void);

void rtkoutstat(rtk_t *rtk);

void swapsolstat(void);

void outsolstat(rtk_t *rtk);

void errmsg(rtk_t *rtk, const char *format, ...);

double sdobs(const obsd_t *obs, int i, int j, int f);

double gfobs_L1L2(const obsd_t *obs, int i, int j, const double *lam);

double gfobs_L1L5(const obsd_t *obs, int i, int j, const double *lam);

double varerr(int sat, int sys, double el, double bl, double dt, int f,
    const prcopt_t *opt);


double baseline(const double *ru, const double *rb, double *dr);

void initx_rtk(rtk_t *rtk, double xi, double var, int i);

int selsat(const obsd_t *obs, double *azel, int nu, int nr,
    const prcopt_t *opt, int *sat, int *iu, int *ir);

void udpos(rtk_t *rtk, double tt);

void udion(rtk_t *rtk, double tt, double bl, const int *sat, int ns);

void udtrop(rtk_t *rtk, double tt, double bl);

void udrcvbias(rtk_t *rtk, double tt);

void detslp_ll(rtk_t *rtk, const obsd_t *obs, int i, int rcv);
void detslp_gf_L1L2(rtk_t *rtk, const obsd_t *obs, int i, int j,
    const nav_t *nav);

void detslp_gf_L1L5(rtk_t *rtk, const obsd_t *obs, int i, int j,
    const nav_t *nav);

void detslp_dop(rtk_t *rtk, const obsd_t *obs, int i, int rcv,
    const nav_t *nav);

void udbias(rtk_t *rtk, double tt, const obsd_t *obs, const int *sat,
    const int *iu, const int *ir, int ns, const nav_t *nav);

void udstate(rtk_t *rtk, const obsd_t *obs, const int *sat,
    const int *iu, const int *ir, int ns, const nav_t *nav);

void zdres_sat(int base, double r, const obsd_t *obs, const nav_t *nav,
    const double *azel, const double *dant,
    const prcopt_t *opt, double *y);

int zdres(int base, const obsd_t *obs, int n, const double *rs,
    const double *dts, const int *svh, const nav_t *nav,
    const double *rr, const prcopt_t *opt, int index, double *y,
    double *e, double *azel);

int validobs(int i, int j, int f, int nf, double *y);

void ddcov(const int *nb, int n, const double *Ri, const double *Rj,
    int nv, double *R);

int constbl(rtk_t *rtk, const double *x, const double *P, double *v,
    double *H, double *Ri, double *Rj, int index);

double prectrop(gtime_t time, const double *pos, int r,
    const double *azel, const prcopt_t *opt, const double *x,
    double *dtdx);

double gloicbcorr(int sat1, int sat2, const prcopt_t *opt, double lam1,
    double lam2, int f);

int test_sys(int sys, int m);

int ddres(rtk_t *rtk, const nav_t *nav, double dt, const double *x,
    const double *P, const int *sat, double *y, double *e,
    double *azel, const int *iu, const int *ir, int ns, double *v,
    double *H, double *R, int *vflg);

double intpres(gtime_t time, const obsd_t *obs, int n, const nav_t *nav,
    rtk_t *rtk, double *y);


int ddmat(rtk_t *rtk, double *D);

void restamb(rtk_t *rtk, const double *bias, int nb, double *xa);

void holdamb(rtk_t *rtk, const double *xa);

int resamb_LAMBDA(rtk_t *rtk, double *bias, double *xa);

int valpos(rtk_t *rtk, const double *v, const double *R, const int *vflg,
    int nv, double thres);

int relpos(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
    const nav_t *nav);

void rtkinit(rtk_t *rtk, const prcopt_t *opt);

void rtkfree(rtk_t *rtk);

int rtkpos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);


#endif
