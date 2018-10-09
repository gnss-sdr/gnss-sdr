/*!
 * \file rtklib_ppp.cc
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

#include "rtklib_ppp.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_sbas.h"
#include "rtklib_ephemeris.h"
#include "rtklib_ionex.h"
#include "rtklib_tides.h"
#include "rtklib_lambda.h"

/* wave length of LC (m) -----------------------------------------------------*/
double lam_LC(int i, int j, int k)
{
    const double f1 = FREQ1, f2 = FREQ2, f5 = FREQ5;

    return SPEED_OF_LIGHT / (i * f1 + j * f2 + k * f5);
}


/* carrier-phase LC (m) ------------------------------------------------------*/
double L_LC(int i, int j, int k, const double *L)
{
    const double f1 = FREQ1, f2 = FREQ2, f5 = FREQ5;
    double L1, L2, L5;

    if ((i && !L[0]) || (j && !L[1]) || (k && !L[2]))
        {
            return 0.0;
        }
    L1 = SPEED_OF_LIGHT / f1 * L[0];
    L2 = SPEED_OF_LIGHT / f2 * L[1];
    L5 = SPEED_OF_LIGHT / f5 * L[2];
    return (i * f1 * L1 + j * f2 * L2 + k * f5 * L5) / (i * f1 + j * f2 + k * f5);
}


/* pseudorange LC (m) --------------------------------------------------------*/
double P_LC(int i, int j, int k, const double *P)
{
    const double f1 = FREQ1, f2 = FREQ2, f5 = FREQ5;
    double P1, P2, P5;

    if ((i && !P[0]) || (j && !P[1]) || (k && !P[2]))
        {
            return 0.0;
        }
    P1 = P[0];
    P2 = P[1];
    P5 = P[2];
    return (i * f1 * P1 + j * f2 * P2 + k * f5 * P5) / (i * f1 + j * f2 + k * f5);
}


/* noise variance of LC (m) --------------------------------------------------*/
double var_LC(int i, int j, int k, double sig)
{
    const double f1 = FREQ1, f2 = FREQ2, f5 = FREQ5;

    return (std::pow(i * f1, 2.0) + std::pow(j * f2, 2.0) + std::pow(k * f5, 2.0)) / std::pow(i * f1 + j * f2 + k * f5, 2.0) * std::pow(sig, 2.0);
}


/* complementaty error function (ref [1] p.227-229) --------------------------*/
double p_gamma(double a, double x, double log_gamma_a)
{
    double y, w;
    int i;

    if (x == 0.0) return 0.0;
    if (x >= a + 1.0) return 1.0 - q_gamma(a, x, log_gamma_a);

    y = w = exp(a * log(x) - x - log_gamma_a) / a;

    for (i = 1; i < 100; i++)
        {
            w *= x / (a + i);
            y += w;
            if (fabs(w) < 1E-15) break;
        }
    return y;
}


double q_gamma(double a, double x, double log_gamma_a)
{
    double y, w, la = 1.0, lb = x + 1.0 - a, lc;
    int i;

    if (x < a + 1.0) return 1.0 - p_gamma(a, x, log_gamma_a);
    w = exp(-x + a * log(x) - log_gamma_a);
    y = w / lb;
    for (i = 2; i < 100; i++)
        {
            lc = ((i - 1 - a) * (lb - la) + (i + x) * lb) / i;
            la = lb;
            lb = lc;
            w *= (i - 1 - a) / i;
            y += w / la / lb;
            if (fabs(w / la / lb) < 1E-15) break;
        }
    return y;
}


double f_erfc(double x)
{
    return x >= 0.0 ? q_gamma(0.5, x * x, LOG_PI / 2.0) : 1.0 + p_gamma(0.5, x * x, LOG_PI / 2.0);
}


/* confidence function of integer ambiguity ----------------------------------*/
double conffunc(int N, double B, double sig)
{
    double x, p = 1.0;
    int i;

    x = fabs(B - N);
    for (i = 1; i < 8; i++)
        {
            p -= f_erfc((i - x) / (SQRT2 * sig)) - f_erfc((i + x) / (SQRT2 * sig));
        }
    return p;
}


/* average LC ----------------------------------------------------------------*/
void average_LC(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav __attribute__((unused)),
    const double *azel)
{
    ambc_t *amb;
    double LC1, LC2, LC3, var1, var2, var3, sig;
    int i, j, sat;

    for (i = 0; i < n; i++)
        {
            sat = obs[i].sat;

            if (azel[1 + 2 * i] < rtk->opt.elmin) continue;

            if (satsys(sat, NULL) != SYS_GPS) continue;

            /* triple-freq carrier and code LC (m) */
            LC1 = L_LC(1, -1, 0, obs[i].L) - P_LC(1, 1, 0, obs[i].P);
            LC2 = L_LC(0, 1, -1, obs[i].L) - P_LC(0, 1, 1, obs[i].P);
            LC3 = L_LC(1, -6, 5, obs[i].L) - P_LC(1, 1, 0, obs[i].P);

            sig = std::sqrt(std::pow(rtk->opt.err[1], 2.0) + std::pow(rtk->opt.err[2] / sin(azel[1 + 2 * i]), 2.0));

            /* measurement noise variance (m) */
            var1 = var_LC(1, 1, 0, sig * rtk->opt.eratio[0]);
            var2 = var_LC(0, 1, 1, sig * rtk->opt.eratio[0]);
            var3 = var_LC(1, 1, 0, sig * rtk->opt.eratio[0]);

            amb = rtk->ambc + sat - 1;

            if (rtk->ssat[sat - 1].slip[0] || rtk->ssat[sat - 1].slip[1] ||
                rtk->ssat[sat - 1].slip[2] || amb->n[0] == 0.0 ||
                fabs(timediff(amb->epoch[0], obs[0].time)) > MIN_ARC_GAP)
                {
                    amb->n[0] = amb->n[1] = amb->n[2] = 0.0;
                    amb->LC[0] = amb->LC[1] = amb->LC[2] = 0.0;
                    amb->LCv[0] = amb->LCv[1] = amb->LCv[2] = 0.0;
                    amb->fixcnt = 0;
                    for (j = 0; j < MAXSAT; j++) amb->flags[j] = 0;
                }
            /* averaging */
            if (LC1)
                {
                    amb->n[0] += 1.0;
                    amb->LC[0] += (LC1 - amb->LC[0]) / amb->n[0];
                    amb->LCv[0] += (var1 - amb->LCv[0]) / amb->n[0];
                }
            if (LC2)
                {
                    amb->n[1] += 1.0;
                    amb->LC[1] += (LC2 - amb->LC[1]) / amb->n[1];
                    amb->LCv[1] += (var2 - amb->LCv[1]) / amb->n[1];
                }
            if (LC3)
                {
                    amb->n[2] += 1.0;
                    amb->LC[2] += (LC3 - amb->LC[2]) / amb->n[2];
                    amb->LCv[2] += (var3 - amb->LCv[2]) / amb->n[2];
                }
            amb->epoch[0] = obs[0].time;
        }
}


/* fix wide-lane ambiguity ---------------------------------------------------*/
int fix_amb_WL(rtk_t *rtk, const nav_t *nav, int sat1, int sat2, int *NW)
{
    ambc_t *amb1, *amb2;
    double BW, vW, lam_WL = lam_LC(1, -1, 0);

    amb1 = rtk->ambc + sat1 - 1;
    amb2 = rtk->ambc + sat2 - 1;
    if (!amb1->n[0] || !amb2->n[0]) return 0;

        /* wide-lane ambiguity */
#ifndef REV_WL_FCB
    BW = (amb1->LC[0] - amb2->LC[0]) / lam_WL + nav->wlbias[sat1 - 1] - nav->wlbias[sat2 - 1];
#else
    BW = (amb1->LC[0] - amb2->LC[0]) / lam_WL - nav->wlbias[sat1 - 1] + nav->wlbias[sat2 - 1];
#endif
    *NW = ROUND_PPP(BW);

    /* variance of wide-lane ambiguity */
    vW = (amb1->LCv[0] / amb1->n[0] + amb2->LCv[0] / amb2->n[0]) / std::pow(lam_WL, 2.0);

    /* validation of integer wide-lane ambigyity */
    return fabs(*NW - BW) <= rtk->opt.thresar[2] &&
           conffunc(*NW, BW, sqrt(vW)) >= rtk->opt.thresar[1];
}


/* linear dependency check ---------------------------------------------------*/
int is_depend(int sat1, int sat2, int *flgs, int *max_flg)
{
    int i;

    if (flgs[sat1 - 1] == 0 && flgs[sat2 - 1] == 0)
        {
            flgs[sat1 - 1] = flgs[sat2 - 1] = ++(*max_flg);
        }
    else if (flgs[sat1 - 1] == 0 && flgs[sat2 - 1] != 0)
        {
            flgs[sat1 - 1] = flgs[sat2 - 1];
        }
    else if (flgs[sat1 - 1] != 0 && flgs[sat2 - 1] == 0)
        {
            flgs[sat2 - 1] = flgs[sat1 - 1];
        }
    else if (flgs[sat1 - 1] > flgs[sat2 - 1])
        {
            for (i = 0; i < MAXSAT; i++)
                if (flgs[i] == flgs[sat2 - 1]) flgs[i] = flgs[sat1 - 1];
        }
    else if (flgs[sat1 - 1] < flgs[sat2 - 1])
        {
            for (i = 0; i < MAXSAT; i++)
                if (flgs[i] == flgs[sat1 - 1]) flgs[i] = flgs[sat2 - 1];
        }
    else
        return 0; /* linear depenent */
    return 1;
}


/* select fixed ambiguities --------------------------------------------------*/
int sel_amb(int *sat1, int *sat2, double *N, double *var, int n)
{
    int i, j, flgs[MAXSAT] = {0}, max_flg = 0;

    /* sort by variance */
    for (i = 0; i < n; i++)
        for (j = 1; j < n - i; j++)
            {
                if (var[j] >= var[j - 1]) continue;
                SWAP_I(sat1[j], sat1[j - 1]);
                SWAP_I(sat2[j], sat2[j - 1]);
                SWAP_D(N[j], N[j - 1]);
                SWAP_D(var[j], var[j - 1]);
            }
    /* select linearly independent satellite pair */
    for (i = j = 0; i < n; i++)
        {
            if (!is_depend(sat1[i], sat2[i], flgs, &max_flg)) continue;
            sat1[j] = sat1[i];
            sat2[j] = sat2[i];
            N[j] = N[i];
            var[j++] = var[i];
        }
    return j;
}


/* fixed solution ------------------------------------------------------------*/
int fix_sol(rtk_t *rtk, const int *sat1, const int *sat2,
    const double *NC, int n)
{
    double *v, *H, *R;
    int i, j, k, info;

    if (n <= 0) return 0;

    v = zeros(n, 1);
    H = zeros(rtk->nx, n);
    R = zeros(n, n);

    /* constraints to fixed ambiguities */
    for (i = 0; i < n; i++)
        {
            j = IB_PPP(sat1[i], &rtk->opt);
            k = IB_PPP(sat2[i], &rtk->opt);
            v[i] = NC[i] - (rtk->x[j] - rtk->x[k]);
            H[j + i * rtk->nx] = 1.0;
            H[k + i * rtk->nx] = -1.0;
            R[i + i * n] = std::pow(CONST_AMB, 2.0);
        }
    /* update states with constraints */
    if ((info = filter(rtk->x, rtk->P, H, v, R, rtk->nx, n)))
        {
            trace(1, "filter error (info=%d)\n", info);
            free(v);
            free(H);
            free(R);
            return 0;
        }
    /* set solution */
    for (i = 0; i < rtk->na; i++)
        {
            rtk->xa[i] = rtk->x[i];
            for (j = 0; j < rtk->na; j++)
                {
                    rtk->Pa[i + j * rtk->na] = rtk->Pa[j + i * rtk->na] = rtk->P[i + j * rtk->nx];
                }
        }
    /* set flags */
    for (i = 0; i < n; i++)
        {
            rtk->ambc[sat1[i] - 1].flags[sat2[i] - 1] = 1;
            rtk->ambc[sat2[i] - 1].flags[sat1[i] - 1] = 1;
        }
    free(v);
    free(H);
    free(R);
    return 1;
}


/* fix narrow-lane ambiguity by rounding -------------------------------------*/
int fix_amb_ROUND(rtk_t *rtk, int *sat1, int *sat2, const int *NW, int n)
{
    double C1, C2, B1, v1, BC, v, vc, *NC, *var, lam_NL = lam_LC(1, 1, 0), lam1, lam2;
    int i, j, k, m = 0, N1, stat;

    lam1 = lam_carr[0];
    lam2 = lam_carr[1];

    C1 = std::pow(lam2, 2.0) / (std::pow(lam2, 2.0) - std::pow(lam1, 2.0));
    C2 = -std::pow(lam1, 2.0) / (std::pow(lam2, 2.0) - std::pow(lam1, 2.0));

    NC = zeros(n, 1);
    var = zeros(n, 1);

    for (i = 0; i < n; i++)
        {
            j = IB_PPP(sat1[i], &rtk->opt);
            k = IB_PPP(sat2[i], &rtk->opt);

            /* narrow-lane ambiguity */
            B1 = (rtk->x[j] - rtk->x[k] + C2 * lam2 * NW[i]) / lam_NL;
            N1 = ROUND_PPP(B1);

            /* variance of narrow-lane ambiguity */
            var[m] = rtk->P[j + j * rtk->nx] + rtk->P[k + k * rtk->nx] - 2.0 * rtk->P[j + k * rtk->nx];
            v1 = var[m] / std::pow(lam_NL, 2.0);

            /* validation of narrow-lane ambiguity */
            if (fabs(N1 - B1) > rtk->opt.thresar[2] ||
                conffunc(N1, B1, sqrt(v1)) < rtk->opt.thresar[1])
                {
                    continue;
                }
            /* iono-free ambiguity (m) */
            BC = C1 * lam1 * N1 + C2 * lam2 * (N1 - NW[i]);

            /* check residuals */
            v = rtk->ssat[sat1[i] - 1].resc[0] - rtk->ssat[sat2[i] - 1].resc[0];
            vc = v + (BC - (rtk->x[j] - rtk->x[k]));
            if (fabs(vc) > THRES_RES) continue;

            sat1[m] = sat1[i];
            sat2[m] = sat2[i];
            NC[m++] = BC;
        }
    /* select fixed ambiguities by dependency check */
    m = sel_amb(sat1, sat2, NC, var, m);

    /* fixed solution */
    stat = fix_sol(rtk, sat1, sat2, NC, m);

    free(NC);
    free(var);

    return stat && m >= 3;
}


/* fix narrow-lane ambiguity by ILS ------------------------------------------*/
int fix_amb_ILS(rtk_t *rtk, int *sat1, int *sat2, int *NW, int n)
{
    double C1, C2, *B1, *N1, *NC, *D, *E, *Q, s[2], lam_NL = lam_LC(1, 1, 0), lam1, lam2;
    int i, j, k, m = 0, info, stat, flgs[MAXSAT] = {0}, max_flg = 0;

    lam1 = lam_carr[0];
    lam2 = lam_carr[1];

    C1 = std::pow(lam2, 2.0) / (std::pow(lam2, 2.0) - std::pow(lam1, 2.0));
    C2 = -std::pow(lam1, 2.0) / (std::pow(lam2, 2.0) - std::pow(lam1, 2.0));

    B1 = zeros(n, 1);
    N1 = zeros(n, 2);
    D = zeros(rtk->nx, n);
    E = mat(n, rtk->nx);
    Q = mat(n, n);
    NC = mat(n, 1);

    for (i = 0; i < n; i++)
        {
            /* check linear independency */
            if (!is_depend(sat1[i], sat2[i], flgs, &max_flg)) continue;

            j = IB_PPP(sat1[i], &rtk->opt);
            k = IB_PPP(sat2[i], &rtk->opt);

            /* float narrow-lane ambiguity (cycle) */
            B1[m] = (rtk->x[j] - rtk->x[k] + C2 * lam2 * NW[i]) / lam_NL;
            N1[m] = ROUND_PPP(B1[m]);

            /* validation of narrow-lane ambiguity */
            if (fabs(N1[m] - B1[m]) > rtk->opt.thresar[2]) continue;

            /* narrow-lane ambiguity transformation matrix */
            D[j + m * rtk->nx] = 1.0 / lam_NL;
            D[k + m * rtk->nx] = -1.0 / lam_NL;

            sat1[m] = sat1[i];
            sat2[m] = sat2[i];
            NW[m++] = NW[i];
        }
    if (m < 3)
        {
            free(B1);
            free(N1);
            free(D);
            free(E);
            free(Q);
            free(NC);
            return 0;
        }

    /* covariance of narrow-lane ambiguities */
    matmul("TN", m, rtk->nx, rtk->nx, 1.0, D, rtk->P, 0.0, E);
    matmul("NN", m, m, rtk->nx, 1.0, E, D, 0.0, Q);

    /* integer least square */
    if ((info = lambda(m, 2, B1, Q, N1, s)))
        {
            trace(2, "lambda error: info=%d\n", info);
            free(B1);
            free(N1);
            free(D);
            free(E);
            free(Q);
            free(NC);
            return 0;
        }
    if (s[0] <= 0.0)
        {
            free(B1);
            free(N1);
            free(D);
            free(E);
            free(Q);
            free(NC);
            return 0;
        }

    rtk->sol.ratio = (float)(MIN_PPP(s[1] / s[0], 999.9));

    /* varidation by ratio-test */
    if (rtk->opt.thresar[0] > 0.0 && rtk->sol.ratio < rtk->opt.thresar[0])
        {
            trace(2, "varidation error: n=%2d ratio=%8.3f\n", m, rtk->sol.ratio);
            free(B1);
            free(N1);
            free(D);
            free(E);
            free(Q);
            free(NC);
            return 0;
        }
    trace(2, "varidation ok: %s n=%2d ratio=%8.3f\n", time_str(rtk->sol.time, 0), m,
        rtk->sol.ratio);

    /* narrow-lane to iono-free ambiguity */
    for (i = 0; i < m; i++)
        {
            NC[i] = C1 * lam1 * N1[i] + C2 * lam2 * (N1[i] - NW[i]);
        }
    /* fixed solution */
    stat = fix_sol(rtk, sat1, sat2, NC, m);

    free(B1);
    free(N1);
    free(D);
    free(E);
    free(Q);
    free(NC);

    return stat;
}


/* resolve integer ambiguity for ppp -----------------------------------------*/
int pppamb(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav,
    const double *azel)
{
    double elmask;
    int i, j, m = 0, stat = 0, *NW, *sat1, *sat2;

    if (n <= 0 || rtk->opt.ionoopt != IONOOPT_IFLC || rtk->opt.nf < 2) return 0;

    trace(3, "pppamb: time=%s n=%d\n", time_str(obs[0].time, 0), n);

    elmask = rtk->opt.elmaskar > 0.0 ? rtk->opt.elmaskar : rtk->opt.elmin;

    sat1 = imat(n * n, 1);
    sat2 = imat(n * n, 1);
    NW = imat(n * n, 1);

    /* average LC */
    average_LC(rtk, obs, n, nav, azel);

    /* fix wide-lane ambiguity */
    for (i = 0; i < n - 1; i++)
        for (j = i + 1; j < n; j++)
            {
                if (!rtk->ssat[obs[i].sat - 1].vsat[0] ||
                    !rtk->ssat[obs[j].sat - 1].vsat[0] ||
                    azel[1 + i * 2] < elmask || azel[1 + j * 2] < elmask) continue;
#if 0
            /* test already fixed */
            if (rtk->ambc[obs[i].sat-1].flags[obs[j].sat-1] &&
                    rtk->ambc[obs[j].sat-1].flags[obs[i].sat-1]) continue;
#endif
                sat1[m] = obs[i].sat;
                sat2[m] = obs[j].sat;
                if (fix_amb_WL(rtk, nav, sat1[m], sat2[m], NW + m)) m++;
            }
    /* fix narrow-lane ambiguity */
    if (rtk->opt.modear == ARMODE_PPPAR)
        {
            stat = fix_amb_ROUND(rtk, sat1, sat2, NW, m);
        }
    else if (rtk->opt.modear == ARMODE_PPPAR_ILS)
        {
            stat = fix_amb_ILS(rtk, sat1, sat2, NW, m);
        }
    free(sat1);
    free(sat2);
    free(NW);

    return stat;
}


void pppoutsolstat(rtk_t *rtk, int level, FILE *fp)
{
    ssat_t *ssat;
    double tow, pos[3], vel[3], acc[3];
    int i, j, week, nfreq = 1;
    char id[32];

    if (level <= 0 || !fp) return;

    trace(3, "pppoutsolstat:\n");

    tow = time2gpst(rtk->sol.time, &week);

    /* receiver position */
    fprintf(fp, "$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", week, tow,
        rtk->sol.stat, rtk->x[0], rtk->x[1], rtk->x[2], 0.0, 0.0, 0.0);

    /* receiver velocity and acceleration */
    if (rtk->opt.dynamics)
        {
            ecef2pos(rtk->sol.rr, pos);
            ecef2enu(pos, rtk->x + 3, vel);
            ecef2enu(pos, rtk->x + 6, acc);
            fprintf(fp, "$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n",
                week, tow, rtk->sol.stat, vel[0], vel[1], vel[2], acc[0], acc[1], acc[2],
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
    /* receiver clocks */
    i = IC_PPP(0, &rtk->opt);
    fprintf(fp, "$CLK,%d,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f\n",
        week, tow, rtk->sol.stat, 1, rtk->x[i] * 1E9 / SPEED_OF_LIGHT, rtk->x[i + 1] * 1E9 / SPEED_OF_LIGHT,
        0.0, 0.0);

    /* tropospheric parameters */
    if (rtk->opt.tropopt == TROPOPT_EST || rtk->opt.tropopt == TROPOPT_ESTG)
        {
            i = IT_PPP(&rtk->opt);
            fprintf(fp, "$TROP,%d,%.3f,%d,%d,%.4f,%.4f\n", week, tow, rtk->sol.stat,
                1, rtk->x[i], 0.0);
        }
    if (rtk->opt.tropopt == TROPOPT_ESTG)
        {
            i = IT_PPP(&rtk->opt);
            fprintf(fp, "$TRPG,%d,%.3f,%d,%d,%.5f,%.5f,%.5f,%.5f\n", week, tow,
                rtk->sol.stat, 1, rtk->x[i + 1], rtk->x[i + 2], 0.0, 0.0);
        }
    if (rtk->sol.stat == SOLQ_NONE || level <= 1) return;

    /* residuals and status */
    for (i = 0; i < MAXSAT; i++)
        {
            ssat = rtk->ssat + i;
            if (!ssat->vs) continue;
            satno2id(i + 1, id);
            for (j = 0; j < nfreq; j++)
                {
                    fprintf(fp, "$SAT,%d,%.3f,%s,%d,%.1f,%.1f,%.4f,%.4f,%d,%.0f,%d,%d,%d,%d,%d,%d\n",
                        week, tow, id, j + 1, ssat->azel[0] * R2D, ssat->azel[1] * R2D,
                        ssat->resp[j], ssat->resc[j], ssat->vsat[j], ssat->snr[j] * 0.25,
                        ssat->fix[j], ssat->slip[j] & 3, ssat->lock[j], ssat->outc[j],
                        ssat->slipc[j], ssat->rejc[j]);
                }
        }
}


/* solar/lunar tides (ref [2] 7) ---------------------------------------------*/
void tide_pl(const double *eu, const double *rp, double GMp,
    const double *pos, double *dr)
{
    const double H3 = 0.292, L3 = 0.015;
    double r, ep[3], latp, lonp, p, K2, K3, a, H2, L2, dp, du, cosp, sinl, cosl;
    int i;

    trace(4, "tide_pl : pos=%.3f %.3f\n", pos[0] * R2D, pos[1] * R2D);

    if ((r = norm_rtk(rp, 3)) <= 0.0) return;

    for (i = 0; i < 3; i++) ep[i] = rp[i] / r;

    K2 = GMp / GME * std::pow(RE_WGS84, 2.0) * std::pow(RE_WGS84, 2.0) / (r * r * r);
    K3 = K2 * RE_WGS84 / r;
    latp = asin(ep[2]);
    lonp = atan2(ep[1], ep[0]);
    cosp = cos(latp);
    sinl = sin(pos[0]);
    cosl = cos(pos[0]);

    /* step1 in phase (degree 2) */
    p = (3.0 * sinl * sinl - 1.0) / 2.0;
    H2 = 0.6078 - 0.0006 * p;
    L2 = 0.0847 + 0.0002 * p;
    a = dot(ep, eu, 3);
    dp = K2 * 3.0 * L2 * a;
    du = K2 * (H2 * (1.5 * a * a - 0.5) - 3.0 * L2 * a * a);

    /* step1 in phase (degree 3) */
    dp += K3 * L3 * (7.5 * a * a - 1.5);
    du += K3 * (H3 * (2.5 * a * a * a - 1.5 * a) - L3 * (7.5 * a * a - 1.5) * a);

    /* step1 out-of-phase (only radial) */
    du += 3.0 / 4.0 * 0.0025 * K2 * sin(2.0 * latp) * sin(2.0 * pos[0]) * sin(pos[1] - lonp);
    du += 3.0 / 4.0 * 0.0022 * K2 * cosp * cosp * cosl * cosl * sin(2.0 * (pos[1] - lonp));

    dr[0] = dp * ep[0] + du * eu[0];
    dr[1] = dp * ep[1] + du * eu[1];
    dr[2] = dp * ep[2] + du * eu[2];

    trace(5, "tide_pl : dr=%.3f %.3f %.3f\n", dr[0], dr[1], dr[2]);
}


/* displacement by solid earth tide (ref [2] 7) ------------------------------*/
void tide_solid(const double *rsun, const double *rmoon,
    const double *pos, const double *E, double gmst, int opt,
    double *dr)
{
    double dr1[3], dr2[3], eu[3], du, dn, sinl, sin2l;

    trace(3, "tide_solid: pos=%.3f %.3f opt=%d\n", pos[0] * R2D, pos[1] * R2D, opt);

    /* step1: time domain */
    eu[0] = E[2];
    eu[1] = E[5];
    eu[2] = E[8];
    tide_pl(eu, rsun, GMS, pos, dr1);
    tide_pl(eu, rmoon, GMM, pos, dr2);

    /* step2: frequency domain, only K1 radial */
    sin2l = sin(2.0 * pos[0]);
    du = -0.012 * sin2l * sin(gmst + pos[1]);

    dr[0] = dr1[0] + dr2[0] + du * E[2];
    dr[1] = dr1[1] + dr2[1] + du * E[5];
    dr[2] = dr1[2] + dr2[2] + du * E[8];

    /* eliminate permanent deformation */
    if (opt & 8)
        {
            sinl = sin(pos[0]);
            du = 0.1196 * (1.5 * sinl * sinl - 0.5);
            dn = 0.0247 * sin2l;
            dr[0] += du * E[2] + dn * E[1];
            dr[1] += du * E[5] + dn * E[4];
            dr[2] += du * E[8] + dn * E[7];
        }
    trace(5, "tide_solid: dr=%.3f %.3f %.3f\n", dr[0], dr[1], dr[2]);
}


/* displacement by ocean tide loading (ref [2] 7) ----------------------------*/
void tide_oload(gtime_t tut, const double *odisp, double *denu)
{
    const double args[][5] = {
        {1.40519E-4, 2.0, -2.0, 0.0, 0.00},  /* M2 */
        {1.45444E-4, 0.0, 0.0, 0.0, 0.00},   /* S2 */
        {1.37880E-4, 2.0, -3.0, 1.0, 0.00},  /* N2 */
        {1.45842E-4, 2.0, 0.0, 0.0, 0.00},   /* K2 */
        {0.72921E-4, 1.0, 0.0, 0.0, 0.25},   /* K1 */
        {0.67598E-4, 1.0, -2.0, 0.0, -0.25}, /* O1 */
        {0.72523E-4, -1.0, 0.0, 0.0, -0.25}, /* P1 */
        {0.64959E-4, 1.0, -3.0, 1.0, -0.25}, /* Q1 */
        {0.53234E-5, 0.0, 2.0, 0.0, 0.00},   /* Mf */
        {0.26392E-5, 0.0, 1.0, -1.0, 0.00},  /* Mm */
        {0.03982E-5, 2.0, 0.0, 0.0, 0.00}    /* Ssa */
    };
    const double ep1975[] = {1975, 1, 1, 0, 0, 0};
    double ep[6], fday, days, t, t2, t3, a[5], ang, dp[3] = {0};
    int i, j;

    trace(3, "tide_oload:\n");

    /* angular argument: see subroutine arg.f for reference [1] */
    time2epoch(tut, ep);
    fday = ep[3] * 3600.0 + ep[4] * 60.0 + ep[5];
    ep[3] = ep[4] = ep[5] = 0.0;
    days = timediff(epoch2time(ep), epoch2time(ep1975)) / 86400.0;
    t = (27392.500528 + 1.000000035 * days) / 36525.0;
    t2 = t * t;
    t3 = t2 * t;

    a[0] = fday;
    a[1] = (279.69668 + 36000.768930485 * t + 3.03E-4 * t2) * D2R;                 /* H0 */
    a[2] = (270.434358 + 481267.88314137 * t - 0.001133 * t2 + 1.9E-6 * t3) * D2R; /* S0 */
    a[3] = (334.329653 + 4069.0340329577 * t - 0.010325 * t2 - 1.2E-5 * t3) * D2R; /* P0 */
    a[4] = 2.0 * PI;

    /* displacements by 11 constituents */
    for (i = 0; i < 11; i++)
        {
            ang = 0.0;
            for (j = 0; j < 5; j++) ang += a[j] * args[i][j];
            for (j = 0; j < 3; j++) dp[j] += odisp[j + i * 6] * cos(ang - odisp[j + 3 + i * 6] * D2R);
        }
    denu[0] = -dp[1];
    denu[1] = -dp[2];
    denu[2] = dp[0];

    trace(5, "tide_oload: denu=%.3f %.3f %.3f\n", denu[0], denu[1], denu[2]);
}


/* iers mean pole (ref [7] eq.7.25) ------------------------------------------*/
void iers_mean_pole(gtime_t tut, double *xp_bar, double *yp_bar)
{
    const double ep2000[] = {2000, 1, 1, 0, 0, 0};
    double y, y2, y3;

    y = timediff(tut, epoch2time(ep2000)) / 86400.0 / 365.25;

    if (y < 3653.0 / 365.25)
        { /* until 2010.0 */
            y2 = y * y;
            y3 = y2 * y;
            *xp_bar = 55.974 + 1.8243 * y + 0.18413 * y2 + 0.007024 * y3; /* (mas) */
            *yp_bar = 346.346 + 1.7896 * y - 0.10729 * y2 - 0.000908 * y3;
        }
    else
        {                                  /* after 2010.0 */
            *xp_bar = 23.513 + 7.6141 * y; /* (mas) */
            *yp_bar = 358.891 - 0.6287 * y;
        }
}


/* displacement by pole tide (ref [7] eq.7.26) --------------------------------*/
void tide_pole(gtime_t tut, const double *pos, const double *erpv,
    double *denu)
{
    double xp_bar, yp_bar, m1, m2, cosl, sinl;

    trace(3, "tide_pole: pos=%.3f %.3f\n", pos[0] * R2D, pos[1] * R2D);

    /* iers mean pole (mas) */
    iers_mean_pole(tut, &xp_bar, &yp_bar);

    /* ref [7] eq.7.24 */
    m1 = erpv[0] / AS2R - xp_bar * 1E-3; /* (as) */
    m2 = -erpv[1] / AS2R + yp_bar * 1E-3;

    /* sin(2*theta) = sin(2*phi), cos(2*theta)=-cos(2*phi) */
    cosl = cos(pos[1]);
    sinl = sin(pos[1]);
    denu[0] = 9E-3 * sin(pos[0]) * (m1 * sinl - m2 * cosl);         /* de= Slambda (m) */
    denu[1] = -9E-3 * cos(2.0 * pos[0]) * (m1 * cosl + m2 * sinl);  /* dn=-Stheta  (m) */
    denu[2] = -33E-3 * sin(2.0 * pos[0]) * (m1 * cosl + m2 * sinl); /* du= Sr      (m) */

    trace(5, "tide_pole : denu=%.3f %.3f %.3f\n", denu[0], denu[1], denu[2]);
}


/* tidal displacement ----------------------------------------------------------
 * displacements by earth tides
 * args   : gtime_t tutc     I   time in utc
 *          double *rr       I   site position (ecef) (m)
 *          int    opt       I   options (or of the followings)
 *                                 1: solid earth tide
 *                                 2: ocean tide loading
 *                                 4: pole tide
 *                                 8: elimate permanent deformation
 *          double *erp      I   earth rotation parameters (NULL: not used)
 *          double *odisp    I   ocean loading parameters  (NULL: not used)
 *                                 odisp[0+i*6]: consituent i amplitude radial(m)
 *                                 odisp[1+i*6]: consituent i amplitude west  (m)
 *                                 odisp[2+i*6]: consituent i amplitude south (m)
 *                                 odisp[3+i*6]: consituent i phase radial  (deg)
 *                                 odisp[4+i*6]: consituent i phase west    (deg)
 *                                 odisp[5+i*6]: consituent i phase south   (deg)
 *                                (i=0:M2,1:S2,2:N2,3:K2,4:K1,5:O1,6:P1,7:Q1,
 *                                   8:Mf,9:Mm,10:Ssa)
 *          double *dr       O   displacement by earth tides (ecef) (m)
 * return : none
 * notes  : see ref [1], [2] chap 7
 *          see ref [4] 5.2.1, 5.2.2, 5.2.3
 *          ver.2.4.0 does not use ocean loading and pole tide corrections
 *-----------------------------------------------------------------------------*/
void tidedisp(gtime_t tutc, const double *rr, int opt, const erp_t *erp,
    const double *odisp, double *dr)
{
    gtime_t tut;
    double pos[2], E[9], drt[3], denu[3], rs[3], rm[3], gmst, erpv[5] = {0};
    int i;
#ifdef IERS_MODEL
    double ep[6], fhr;
    int year, mon, day;
#endif

    trace(3, "tidedisp: tutc=%s\n", time_str(tutc, 0));

    if (erp) geterp(erp, tutc, erpv);

    tut = timeadd(tutc, erpv[2]);

    dr[0] = dr[1] = dr[2] = 0.0;

    if (norm_rtk(rr, 3) <= 0.0) return;

    pos[0] = asin(rr[2] / norm_rtk(rr, 3));
    pos[1] = atan2(rr[1], rr[0]);
    xyz2enu(pos, E);

    if (opt & 1)
        { /* solid earth tides */

            /* sun and moon position in ecef */
            sunmoonpos(tutc, erpv, rs, rm, &gmst);

#ifdef IERS_MODEL
            time2epoch(tutc, ep);
            year = (int)ep[0];
            mon = (int)ep[1];
            day = (int)ep[2];
            fhr = ep[3] + ep[4] / 60.0 + ep[5] / 3600.0;

            /* call DEHANTTIDEINEL */
            dehanttideinel_((double *)rr, &year, &mon, &day, &fhr, rs, rm, drt);
#else
            tide_solid(rs, rm, pos, E, gmst, opt, drt);
#endif
            for (i = 0; i < 3; i++) dr[i] += drt[i];
        }
    if ((opt & 2) && odisp)
        { /* ocean tide loading */
            tide_oload(tut, odisp, denu);
            matmul("TN", 3, 1, 3, 1.0, E, denu, 0.0, drt);
            for (i = 0; i < 3; i++) dr[i] += drt[i];
        }
    if ((opt & 4) && erp)
        { /* pole tide */
            tide_pole(tut, pos, erpv, denu);
            matmul("TN", 3, 1, 3, 1.0, E, denu, 0.0, drt);
            for (i = 0; i < 3; i++) dr[i] += drt[i];
        }
    trace(5, "tidedisp: dr=%.3f %.3f %.3f\n", dr[0], dr[1], dr[2]);
}


/* exclude meas of eclipsing satellite (block IIA) ---------------------------*/
void testeclipse(const obsd_t *obs, int n, const nav_t *nav, double *rs)
{
    double rsun[3], esun[3], r, ang, erpv[5] = {0}, cosa;
    int i, j;
    const char *type;

    trace(3, "testeclipse:\n");

    /* unit vector of sun direction (ecef) */
    sunmoonpos(gpst2utc(obs[0].time), erpv, rsun, NULL, NULL);
    if (normv3(rsun, esun) == 0) trace(1, "Error computing the norm");

    for (i = 0; i < n; i++)
        {
            type = nav->pcvs[obs[i].sat - 1].type;

            if ((r = norm_rtk(rs + i * 6, 3)) <= 0.0) continue;
#if 1
            /* only block IIA */
            if (*type && !strstr(type, "BLOCK IIA")) continue;
#endif
            /* sun-earth-satellite angle */
            cosa = dot(rs + i * 6, esun, 3) / r;
            cosa = cosa < -1.0 ? -1.0 : (cosa > 1.0 ? 1.0 : cosa);
            ang = acos(cosa);

            /* test eclipse */
            if (ang < PI / 2.0 || r * sin(ang) > RE_WGS84) continue;

            trace(2, "eclipsing sat excluded %s sat=%2d\n", time_str(obs[0].time, 0),
                obs[i].sat);

            for (j = 0; j < 3; j++) rs[j + i * 6] = 0.0;
        }
}


/* measurement error variance ------------------------------------------------*/
double varerr(int sat __attribute__((unused)), int sys, double el, int type, const prcopt_t *opt)
{
    double a, b, a2, b2, fact = 1.0;
    double sinel = sin(el);
    int i = sys == SYS_GLO ? 1 : (sys == SYS_GAL ? 2 : 0);

    /* extended error model */
    if (type == 1 && opt->exterr.ena[0])
        { /* code */
            a = opt->exterr.cerr[i][0];
            b = opt->exterr.cerr[i][1];
            if (opt->ionoopt == IONOOPT_IFLC)
                {
                    a2 = opt->exterr.cerr[i][2];
                    b2 = opt->exterr.cerr[i][3];
                    a = std::sqrt(std::pow(2.55, 2.0) * a * a + std::pow(1.55, 2.0) * a2 * a2);
                    b = std::sqrt(std::pow(2.55, 2.0) * b * b + std::pow(1.55, 2.0) * b2 * b2);
                }
        }
    else if (type == 0 && opt->exterr.ena[1])
        { /* phase */
            a = opt->exterr.perr[i][0];
            b = opt->exterr.perr[i][1];
            if (opt->ionoopt == IONOOPT_IFLC)
                {
                    a2 = opt->exterr.perr[i][2];
                    b2 = opt->exterr.perr[i][3];
                    a = std::sqrt(std::pow(2.55, 2.0) * a * a + std::pow(1.55, 2.0) * a2 * a2);
                    b = std::sqrt(std::pow(2.55, 2.0) * b * b + std::pow(1.55, 2.0) * b2 * b2);
                }
        }
    else
        { /* normal error model */
            if (type == 1) fact *= opt->eratio[0];
            fact *= sys == SYS_GLO ? EFACT_GLO : (sys == SYS_SBS ? EFACT_SBS : EFACT_GPS);
            if (opt->ionoopt == IONOOPT_IFLC) fact *= 3.0;
            a = fact * opt->err[1];
            b = fact * opt->err[2];
        }
    return a * a + b * b / sinel / sinel;
}


/* initialize state and covariance -------------------------------------------*/
void initx(rtk_t *rtk, double xi, double var, int i)
{
    int j;
    rtk->x[i] = xi;
    for (j = 0; j < rtk->nx; j++)
        {
            rtk->P[i + j * rtk->nx] = rtk->P[j + i * rtk->nx] = i == j ? var : 0.0;
        }
}


/* dual-frequency iono-free measurements -------------------------------------*/
int ifmeas(const obsd_t *obs, const nav_t *nav, const double *azel,
    const prcopt_t *opt, const double *dantr, const double *dants,
    double phw, double *meas, double *var)
{
    const double *lam = nav->lam[obs->sat - 1];
    double c1, c2, L1, L2, P1, P2, P1_C1, P2_C2, gamma;
    int i = 0, j = 1, k;

    trace(4, "ifmeas  :\n");

    /* L1-L2 for GPS/GLO/QZS, L1-L5 for GAL/SBS */
    if (NFREQ >= 3 && (satsys(obs->sat, NULL) & (SYS_GAL | SYS_SBS))) j = 2;

    if (NFREQ < 2 || lam[i] == 0.0 || lam[j] == 0.0) return 0;

    /* test snr mask */
    if (testsnr(0, i, azel[1], obs->SNR[i] * 0.25, &opt->snrmask) ||
        testsnr(0, j, azel[1], obs->SNR[j] * 0.25, &opt->snrmask))
        {
            return 0;
        }
    gamma = std::pow(lam[j], 2.0) / std::pow(lam[i], 2.0); /* f1^2/f2^2 */
    c1 = gamma / (gamma - 1.0);                            /*  f1^2/(f1^2-f2^2) */
    c2 = -1.0 / (gamma - 1.0);                             /* -f2^2/(f1^2-f2^2) */

    L1 = obs->L[i] * lam[i]; /* cycle -> m */
    L2 = obs->L[j] * lam[j];
    P1 = obs->P[i];
    P2 = obs->P[j];
    P1_C1 = nav->cbias[obs->sat - 1][1];
    P2_C2 = nav->cbias[obs->sat - 1][2];
    if (opt->sateph == EPHOPT_LEX)
        {
            P1_C1 = nav->lexeph[obs->sat - 1].isc[0] * SPEED_OF_LIGHT; /* ISC_L1C/A */
        }
    if (L1 == 0.0 || L2 == 0.0 || P1 == 0.0 || P2 == 0.0) return 0;

    /* iono-free phase with windup correction */
    meas[0] = c1 * L1 + c2 * L2 - (c1 * lam[i] + c2 * lam[j]) * phw;

    /* iono-free code with dcb correction */
    if (obs->code[i] == CODE_L1C) P1 += P1_C1; /* C1->P1 */
    if (obs->code[j] == CODE_L2C) P2 += P2_C2; /* C2->P2 */
    meas[1] = c1 * P1 + c2 * P2;
    var[1] = std::pow(ERR_CBIAS, 2.0);

    if (opt->sateph == EPHOPT_SBAS) meas[1] -= P1_C1; /* sbas clock based C1 */

    /* gps-glonass h/w bias correction for code */
    if (opt->exterr.ena[3] && satsys(obs->sat, NULL) == SYS_GLO)
        {
            meas[1] += c1 * opt->exterr.gpsglob[0] + c2 * opt->exterr.gpsglob[1];
        }
    /* antenna phase center variation correction */
    for (k = 0; k < 2; k++)
        {
            if (dants) meas[k] -= c1 * dants[i] + c2 * dants[j];
            if (dantr) meas[k] -= c1 * dantr[i] + c2 * dantr[j];
        }
    return 1;
}


/* get tgd parameter (m) -----------------------------------------------------*/
double gettgd_ppp(int sat, const nav_t *nav)
{
    int i;
    for (i = 0; i < nav->n; i++)
        {
            if (nav->eph[i].sat != sat) continue;
            return SPEED_OF_LIGHT * nav->eph[i].tgd[0];
        }
    return 0.0;
}


/* slant ionospheric delay ---------------------------------------------------*/
int corr_ion(gtime_t time, const nav_t *nav, int sat __attribute__((unused)), const double *pos,
    const double *azel, int ionoopt, double *ion, double *var,
    int *brk __attribute__((unused)))
{
#ifdef EXTSTEC
    double rate;
#endif
    /* sbas ionosphere model */
    if (ionoopt == IONOOPT_SBAS)
        {
            return sbsioncorr(time, nav, pos, azel, ion, var);
        }
    /* ionex tec model */
    if (ionoopt == IONOOPT_TEC)
        {
            return iontec(time, nav, pos, azel, 1, ion, var);
        }
#ifdef EXTSTEC
    /* slant tec model */
    if (ionoopt == IONOOPT_STEC)
        {
            return stec_ion(time, nav, sat, pos, azel, ion, &rate, var, brk);
        }
#endif
    /* broadcast model */
    if (ionoopt == IONOOPT_BRDC)
        {
            *ion = ionmodel(time, nav->ion_gps, pos, azel);
            *var = std::pow(*ion * ERR_BRDCI, 2.0);
            return 1;
        }
    /* ionosphere model off */
    *ion = 0.0;
    *var = VAR_IONO_OFF;
    return 1;
}


/* ionosphere and antenna corrected measurements -----------------------------*/
int corrmeas(const obsd_t *obs, const nav_t *nav, const double *pos,
    const double *azel, const prcopt_t *opt,
    const double *dantr, const double *dants, double phw,
    double *meas, double *var, int *brk)
{
    const double *lam = nav->lam[obs->sat - 1];
    double ion = 0.0, L1, P1, PC, P1_P2, P1_C1, vari, gamma;
    int i;

    trace(4, "corrmeas:\n");

    meas[0] = meas[1] = var[0] = var[1] = 0.0;

    /* iono-free LC */
    if (opt->ionoopt == IONOOPT_IFLC)
        {
            return ifmeas(obs, nav, azel, opt, dantr, dants, phw, meas, var);
        }
    if (lam[0] == 0.0 || obs->L[0] == 0.0 || obs->P[0] == 0.0) return 0;

    if (testsnr(0, 0, azel[1], obs->SNR[0] * 0.25, &opt->snrmask)) return 0;

    L1 = obs->L[0] * lam[0];
    P1 = obs->P[0];

    /* dcb correction */
    gamma = std::pow(lam[1] / lam[0], 2.0); /* f1^2/f2^2 */
    P1_P2 = nav->cbias[obs->sat - 1][0];
    P1_C1 = nav->cbias[obs->sat - 1][1];
    if (P1_P2 == 0.0 && (satsys(obs->sat, NULL) & (SYS_GPS | SYS_GAL | SYS_QZS)))
        {
            P1_P2 = (1.0 - gamma) * gettgd_ppp(obs->sat, nav);
        }
    if (obs->code[0] == CODE_L1C) P1 += P1_C1; /* C1->P1 */
    PC = P1 - P1_P2 / (1.0 - gamma);           /* P1->PC */

    /* slant ionospheric delay L1 (m) */
    if (!corr_ion(obs->time, nav, obs->sat, pos, azel, opt->ionoopt, &ion, &vari, brk))
        {
            trace(2, "iono correction error: time=%s sat=%2d ionoopt=%d\n",
                time_str(obs->time, 2), obs->sat, opt->ionoopt);
            return 0;
        }
    /* ionosphere and windup corrected phase and code */
    meas[0] = L1 + ion - lam[0] * phw;
    meas[1] = PC - ion;

    var[0] += vari;
    var[1] += vari + std::pow(ERR_CBIAS, 2.0);

    /* antenna phase center variation correction */
    for (i = 0; i < 2; i++)
        {
            if (dants) meas[i] -= dants[0];
            if (dantr) meas[i] -= dantr[0];
        }
    return 1;
}


/* L1/L2 geometry-free phase measurement -------------------------------------*/
double gfmeas(const obsd_t *obs, const nav_t *nav)
{
    const double *lam = nav->lam[obs->sat - 1];

    if (lam[0] == 0.0 || lam[1] == 0.0 || obs->L[0] == 0.0 || obs->L[1] == 0.0) return 0.0;

    return lam[0] * obs->L[0] - lam[1] * obs->L[1];
}


/* temporal update of position -----------------------------------------------*/
void udpos_ppp(rtk_t *rtk)
{
    int i;

    trace(3, "udpos_ppp:\n");

    /* fixed mode */
    if (rtk->opt.mode == PMODE_PPP_FIXED)
        {
            for (i = 0; i < 3; i++) initx(rtk, rtk->opt.ru[i], 1E-8, i);
            return;
        }
    /* initialize position for first epoch */
    if (norm_rtk(rtk->x, 3) <= 0.0)
        {
            for (i = 0; i < 3; i++) initx(rtk, rtk->sol.rr[i], VAR_POS_PPP, i);
        }
    /* static ppp mode */
    if (rtk->opt.mode == PMODE_PPP_STATIC) return;

    /* kinmatic mode without dynamics */
    for (i = 0; i < 3; i++)
        {
            initx(rtk, rtk->sol.rr[i], VAR_POS_PPP, i);
        }
}


/* temporal update of clock --------------------------------------------------*/
void udclk_ppp(rtk_t *rtk)
{
    double dtr;
    int i;

    trace(3, "udclk_ppp:\n");

    /* initialize every epoch for clock (white noise) */
    for (i = 0; i < NSYS; i++)
        {
            if (rtk->opt.sateph == EPHOPT_PREC)
                {
                    /* time of prec ephemeris is based gpst */
                    /* negelect receiver inter-system bias  */
                    dtr = rtk->sol.dtr[0];
                }
            else
                {
                    dtr = i == 0 ? rtk->sol.dtr[0] : rtk->sol.dtr[0] + rtk->sol.dtr[i];
                }
            initx(rtk, SPEED_OF_LIGHT * dtr, VAR_CLK, IC_PPP(i, &rtk->opt));
        }
}


/* temporal update of tropospheric parameters --------------------------------*/
void udtrop_ppp(rtk_t *rtk)
{
    double pos[3], azel[] = {0.0, PI / 2.0}, ztd, var;
    int i = IT_PPP(&rtk->opt), j;

    trace(3, "udtrop_ppp:\n");

    if (rtk->x[i] == 0.0)
        {
            ecef2pos(rtk->sol.rr, pos);
            ztd = sbstropcorr(rtk->sol.time, pos, azel, &var);
            initx(rtk, ztd, var, i);

            if (rtk->opt.tropopt >= TROPOPT_ESTG)
                {
                    for (j = 0; j < 2; j++) initx(rtk, 1E-6, VAR_GRA_PPP, ++i);
                }
        }
    else
        {
            rtk->P[i * (1 + rtk->nx)] += std::pow(rtk->opt.prn[2], 2.0) * fabs(rtk->tt);

            if (rtk->opt.tropopt >= TROPOPT_ESTG)
                {
                    for (j = 0; j < 2; j++)
                        {
                            rtk->P[++i * (1 + rtk->nx)] += std::pow(rtk->opt.prn[2] * 0.1, 2.0) * fabs(rtk->tt);
                        }
                }
        }
}


/* detect cycle slip by LLI --------------------------------------------------*/
void detslp_ll(rtk_t *rtk, const obsd_t *obs, int n)
{
    int i, j;

    trace(3, "detslp_ll: n=%d\n", n);

    for (i = 0; i < n && i < MAXOBS; i++)
        for (j = 0; j < rtk->opt.nf; j++)
            {
                if (obs[i].L[j] == 0.0 || !(obs[i].LLI[j] & 3)) continue;

                trace(3, "detslp_ll: slip detected sat=%2d f=%d\n", obs[i].sat, j + 1);

                rtk->ssat[obs[i].sat - 1].slip[j] = 1;
            }
}


/* detect cycle slip by geometry free phase jump -----------------------------*/
void detslp_gf(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    double g0, g1;
    int i, j;

    trace(3, "detslp_gf: n=%d\n", n);

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            if ((g1 = gfmeas(obs + i, nav)) == 0.0) continue;

            g0 = rtk->ssat[obs[i].sat - 1].gf;
            rtk->ssat[obs[i].sat - 1].gf = g1;

            trace(4, "detslip_gf: sat=%2d gf0=%8.3f gf1=%8.3f\n", obs[i].sat, g0, g1);

            if (g0 != 0.0 && fabs(g1 - g0) > rtk->opt.thresslip)
                {
                    trace(3, "detslip_gf: slip detected sat=%2d gf=%8.3f->%8.3f\n",
                        obs[i].sat, g0, g1);

                    for (j = 0; j < rtk->opt.nf; j++) rtk->ssat[obs[i].sat - 1].slip[j] |= 1;
                }
        }
}


/* temporal update of phase biases -------------------------------------------*/
void udbias_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    double meas[2], var[2], bias[MAXOBS] = {0}, offset = 0.0, pos[3] = {0};
    int i, j, k, sat, brk = 0;

    trace(3, "udbias  : n=%d\n", n);

    for (i = 0; i < MAXSAT; i++)
        for (j = 0; j < rtk->opt.nf; j++)
            {
                rtk->ssat[i].slip[j] = 0;
            }
    /* detect cycle slip by LLI */
    detslp_ll(rtk, obs, n);

    /* detect cycle slip by geometry-free phase jump */
    detslp_gf(rtk, obs, n, nav);

    /* reset phase-bias if expire obs outage counter */
    for (i = 0; i < MAXSAT; i++)
        {
            if (++rtk->ssat[i].outc[0] > (unsigned int)rtk->opt.maxout)
                {
                    initx(rtk, 0.0, 0.0, IB_PPP(i + 1, &rtk->opt));
                }
        }
    ecef2pos(rtk->sol.rr, pos);

    for (i = k = 0; i < n && i < MAXOBS; i++)
        {
            sat = obs[i].sat;
            j = IB_PPP(sat, &rtk->opt);
            if (!corrmeas(obs + i, nav, pos, rtk->ssat[sat - 1].azel, &rtk->opt, NULL, NULL,
                    0.0, meas, var, &brk)) continue;

            if (brk)
                {
                    rtk->ssat[sat - 1].slip[0] = 1;
                    trace(2, "%s: sat=%2d correction break\n", time_str(obs[i].time, 0), sat);
                }
            bias[i] = meas[0] - meas[1];
            if (rtk->x[j] == 0.0 ||
                rtk->ssat[sat - 1].slip[0] || rtk->ssat[sat - 1].slip[1]) continue;
            offset += bias[i] - rtk->x[j];
            k++;
        }
    /* correct phase-code jump to enssure phase-code coherency */
    if (k >= 2 && fabs(offset / k) > 0.0005 * SPEED_OF_LIGHT)
        {
            for (i = 0; i < MAXSAT; i++)
                {
                    j = IB_PPP(i + 1, &rtk->opt);
                    if (rtk->x[j] != 0.0) rtk->x[j] += offset / k;
                }
            trace(2, "phase-code jump corrected: %s n=%2d dt=%12.9fs\n",
                time_str(rtk->sol.time, 0), k, offset / k / SPEED_OF_LIGHT);
        }
    for (i = 0; i < n && i < MAXOBS; i++)
        {
            sat = obs[i].sat;
            j = IB_PPP(sat, &rtk->opt);

            rtk->P[j + j * rtk->nx] += std::pow(rtk->opt.prn[0], 2.0) * fabs(rtk->tt);

            if (rtk->x[j] != 0.0 &&
                !rtk->ssat[sat - 1].slip[0] && !rtk->ssat[sat - 1].slip[1]) continue;

            if (bias[i] == 0.0) continue;

            /* reinitialize phase-bias if detecting cycle slip */
            initx(rtk, bias[i], VAR_BIAS, IB_PPP(sat, &rtk->opt));

            trace(5, "udbias_ppp: sat=%2d bias=%.3f\n", sat, meas[0] - meas[1]);
        }
}


/* temporal update of states --------------------------------------------------*/
void udstate_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    trace(3, "udstate_ppp: n=%d\n", n);

    /* temporal update of position */
    udpos_ppp(rtk);

    /* temporal update of clock */
    udclk_ppp(rtk);

    /* temporal update of tropospheric parameters */
    if (rtk->opt.tropopt >= TROPOPT_EST)
        {
            udtrop_ppp(rtk);
        }
    /* temporal update of phase-bias */
    udbias_ppp(rtk, obs, n, nav);
}


/* satellite antenna phase center variation ----------------------------------*/
void satantpcv(const double *rs, const double *rr, const pcv_t *pcv,
    double *dant)
{
    double ru[3], rz[3], eu[3], ez[3], nadir, cosa;
    int i;

    for (i = 0; i < 3; i++)
        {
            ru[i] = rr[i] - rs[i];
            rz[i] = -rs[i];
        }
    if (!normv3(ru, eu) || !normv3(rz, ez)) return;

    cosa = dot(eu, ez, 3);
    cosa = cosa < -1.0 ? -1.0 : (cosa > 1.0 ? 1.0 : cosa);
    nadir = acos(cosa);

    antmodel_s(pcv, nadir, dant);
}


/* precise tropospheric model ------------------------------------------------*/
double prectrop(gtime_t time, const double *pos, const double *azel,
    const prcopt_t *opt, const double *x, double *dtdx,
    double *var)
{
    const double zazel[] = {0.0, PI / 2.0};
    double zhd, m_h, m_w, cotz, grad_n, grad_e;

    /* zenith hydrostatic delay */
    zhd = tropmodel(time, pos, zazel, 0.0);

    /* mapping function */
    m_h = tropmapf(time, pos, azel, &m_w);

    if ((opt->tropopt == TROPOPT_ESTG || opt->tropopt == TROPOPT_CORG) && azel[1] > 0.0)
        {
            /* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
            cotz = 1.0 / tan(azel[1]);
            grad_n = m_w * cotz * cos(azel[0]);
            grad_e = m_w * cotz * sin(azel[0]);
            m_w += grad_n * x[1] + grad_e * x[2];
            dtdx[1] = grad_n * (x[0] - zhd);
            dtdx[2] = grad_e * (x[0] - zhd);
        }
    dtdx[0] = m_w;
    *var = std::pow(0.01, 2.0);
    return m_h * zhd + m_w * (x[0] - zhd);
}


/* phase and code residuals --------------------------------------------------*/
int res_ppp(int iter __attribute__((unused)), const obsd_t *obs, int n, const double *rs,
    const double *dts, const double *vare, const int *svh,
    const nav_t *nav, const double *x, rtk_t *rtk, double *v,
    double *H, double *R, double *azel)
{
    prcopt_t *opt = &rtk->opt;
    double r, rr[3], disp[3], pos[3], e[3], meas[2], dtdx[3], dantr[NFREQ] = {0};
    double dants[NFREQ] = {0}, var[MAXOBS * 2], dtrp = 0.0, vart = 0.0, varm[2] = {0};
    int i, j, k, sat, sys, nv = 0, nx = rtk->nx, brk, tideopt, sva = -1;
    eph_t *eph;

    trace(3, "res_ppp : n=%d nx=%d\n", n, nx);

    for (i = 0; i < MAXSAT; i++) rtk->ssat[i].vsat[0] = 0;

    for (i = 0; i < 3; i++) rr[i] = x[i];

    /* earth tides correction */
    if (opt->tidecorr)
        {
            tideopt = opt->tidecorr == 1 ? 1 : 7; /* 1:solid, 2:solid+otl+pole */

            tidedisp(gpst2utc(obs[0].time), rr, tideopt, &nav->erp, opt->odisp[0],
                disp);
            for (i = 0; i < 3; i++) rr[i] += disp[i];
        }
    ecef2pos(rr, pos);

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            sat = obs[i].sat;
            if (!(sys = satsys(sat, NULL)) || !rtk->ssat[sat - 1].vs) continue;

            /* geometric distance/azimuth/elevation angle */
            if ((r = geodist(rs + i * 6, rr, e)) <= 0.0 ||
                satazel(pos, e, azel + i * 2) < opt->elmin) continue;

            /* if Gal satellite, check SISA is not NAPA*/
            if (sys == SYS_GAL)
                {
                    if (!(eph = seleph(obs[i].time, obs[i].sat, -1, nav))) continue;
                    sva = eph->sva;
                }
            /* excluded satellite? */
            if (satexclude(obs[i].sat, svh[i], sva, opt)) continue;

            /* tropospheric delay correction */
            if (opt->tropopt == TROPOPT_SAAS)
                {
                    dtrp = tropmodel(obs[i].time, pos, azel + i * 2, REL_HUMI);
                    vart = std::pow(ERR_SAAS, 2.0);
                }
            else if (opt->tropopt == TROPOPT_SBAS)
                {
                    dtrp = sbstropcorr(obs[i].time, pos, azel + i * 2, &vart);
                }
            else if (opt->tropopt == TROPOPT_EST || opt->tropopt == TROPOPT_ESTG)
                {
                    dtrp = prectrop(obs[i].time, pos, azel + i * 2, opt, x + IT_PPP(opt), dtdx, &vart);
                }
            else if (opt->tropopt == TROPOPT_COR || opt->tropopt == TROPOPT_CORG)
                {
                    dtrp = prectrop(obs[i].time, pos, azel + i * 2, opt, x, dtdx, &vart);
                }
            /* satellite antenna model */
            if (opt->posopt[0])
                {
                    satantpcv(rs + i * 6, rr, nav->pcvs + sat - 1, dants);
                }
            /* receiver antenna model */
            antmodel(opt->pcvr, opt->antdel[0], azel + i * 2, opt->posopt[1], dantr);

            /* phase windup correction */
            if (opt->posopt[2])
                {
                    windupcorr(rtk->sol.time, rs + i * 6, rr, &rtk->ssat[sat - 1].phw);
                }
            /* ionosphere and antenna phase corrected measurements */
            if (!corrmeas(obs + i, nav, pos, azel + i * 2, &rtk->opt, dantr, dants,
                    rtk->ssat[sat - 1].phw, meas, varm, &brk))
                {
                    continue;
                }
            /* satellite clock and tropospheric delay */
            r += -SPEED_OF_LIGHT * dts[i * 2] + dtrp;

            trace(5, "sat=%2d azel=%6.1f %5.1f dtrp=%.3f dantr=%6.3f %6.3f dants=%6.3f %6.3f phw=%6.3f\n",
                sat, azel[i * 2] * R2D, azel[1 + i * 2] * R2D, dtrp, dantr[0], dantr[1], dants[0],
                dants[1], rtk->ssat[sat - 1].phw);

            for (j = 0; j < 2; j++)
                { /* for phase and code */

                    if (meas[j] == 0.0) continue;

                    for (k = 0; k < nx; k++) H[k + nx * nv] = 0.0;

                    v[nv] = meas[j] - r;

                    for (k = 0; k < 3; k++) H[k + nx * nv] = -e[k];

                    if (sys != SYS_GLO)
                        {
                            v[nv] -= x[IC_PPP(0, opt)];
                            H[IC_PPP(0, opt) + nx * nv] = 1.0;
                        }
                    else
                        {
                            v[nv] -= x[IC_PPP(1, opt)];
                            H[IC_PPP(1, opt) + nx * nv] = 1.0;
                        }
                    if (opt->tropopt >= TROPOPT_EST)
                        {
                            for (k = 0; k < (opt->tropopt >= TROPOPT_ESTG ? 3 : 1); k++)
                                {
                                    H[IT_PPP(opt) + k + nx * nv] = dtdx[k];
                                }
                        }
                    if (j == 0)
                        {
                            v[nv] -= x[IB_PPP(obs[i].sat, opt)];
                            H[IB_PPP(obs[i].sat, opt) + nx * nv] = 1.0;
                        }
                    var[nv] = varerr(obs[i].sat, sys, azel[1 + i * 2], j, opt) + varm[j] + vare[i] + vart;

                    if (j == 0)
                        rtk->ssat[sat - 1].resc[0] = v[nv];
                    else
                        rtk->ssat[sat - 1].resp[0] = v[nv];

                        /* test innovation */
#if 0
                    if (opt->maxinno>0.0 && fabs(v[nv])>opt->maxinno)
                        {
#else
                    if (opt->maxinno > 0.0 && fabs(v[nv]) > opt->maxinno && sys != SYS_GLO)
                        {
#endif
                    trace(2, "ppp outlier rejected %s sat=%2d type=%d v=%.3f\n",
                        time_str(obs[i].time, 0), sat, j, v[nv]);
                    rtk->ssat[sat - 1].rejc[0]++;
                    continue;
                }
            if (j == 0) rtk->ssat[sat - 1].vsat[0] = 1;
            nv++;
        }
}
for (i = 0; i < nv; i++)
    for (j = 0; j < nv; j++)
        {
            R[i + j * nv] = i == j ? var[i] : 0.0;
        }
trace(5, "x=\n");
tracemat(5, x, 1, nx, 8, 3);
trace(5, "v=\n");
tracemat(5, v, 1, nv, 8, 3);
trace(5, "H=\n");
tracemat(5, H, nx, nv, 8, 3);
trace(5, "R=\n");
tracemat(5, R, nv, nv, 8, 5);
return nv;
}


/* number of estimated states ------------------------------------------------*/
int pppnx(const prcopt_t *opt)
{
    return NX_PPP(opt);
}


/* precise point positioning -------------------------------------------------*/
void pppos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    const prcopt_t *opt = &rtk->opt;
    double *rs, *dts, *var, *v, *H, *R, *azel, *xp, *Pp;
    int i, nv, info, svh[MAXOBS], stat = SOLQ_SINGLE;

    trace(3, "pppos   : nx=%d n=%d\n", rtk->nx, n);

    rs = mat(6, n);
    dts = mat(2, n);
    var = mat(1, n);
    azel = zeros(2, n);

    for (i = 0; i < MAXSAT; i++) rtk->ssat[i].fix[0] = 0;

    /* temporal update of states */
    udstate_ppp(rtk, obs, n, nav);

    trace(4, "x(0)=");
    tracemat(4, rtk->x, 1, NR_PPP(opt), 13, 4);

    /* satellite positions and clocks */
    satposs(obs[0].time, obs, n, nav, rtk->opt.sateph, rs, dts, var, svh);

    /* exclude measurements of eclipsing satellite */
    if (rtk->opt.posopt[3])
        {
            testeclipse(obs, n, nav, rs);
        }
    xp = mat(rtk->nx, 1);
    Pp = zeros(rtk->nx, rtk->nx);
    matcpy(xp, rtk->x, rtk->nx, 1);
    nv = n * rtk->opt.nf * 2;
    v = mat(nv, 1);
    H = mat(rtk->nx, nv);
    R = mat(nv, nv);

    for (i = 0; i < rtk->opt.niter; i++)
        {
            /* phase and code residuals */
            if ((nv = res_ppp(i, obs, n, rs, dts, var, svh, nav, xp, rtk, v, H, R, azel)) <= 0) break;

            /* measurement update */
            matcpy(Pp, rtk->P, rtk->nx, rtk->nx);

            if ((info = filter(xp, Pp, H, v, R, rtk->nx, nv)))
                {
                    trace(2, "ppp filter error %s info=%d\n", time_str(rtk->sol.time, 0), info);
                    break;
                }
            trace(4, "x(%d)=", i + 1);
            tracemat(4, xp, 1, NR_PPP(opt), 13, 4);

            stat = SOLQ_PPP;
        }
    if (stat == SOLQ_PPP)
        {
            /* postfit residuals */
            res_ppp(1, obs, n, rs, dts, var, svh, nav, xp, rtk, v, H, R, azel);

            /* update state and covariance matrix */
            matcpy(rtk->x, xp, rtk->nx, 1);
            matcpy(rtk->P, Pp, rtk->nx, rtk->nx);

            /* ambiguity resolution in ppp */
            if (opt->modear == ARMODE_PPPAR || opt->modear == ARMODE_PPPAR_ILS)
                {
                    if (pppamb(rtk, obs, n, nav, azel)) stat = SOLQ_FIX;
                }
            /* update solution status */
            rtk->sol.ns = 0;
            for (i = 0; i < n && i < MAXOBS; i++)
                {
                    if (!rtk->ssat[obs[i].sat - 1].vsat[0]) continue;
                    rtk->ssat[obs[i].sat - 1].lock[0]++;
                    rtk->ssat[obs[i].sat - 1].outc[0] = 0;
                    rtk->ssat[obs[i].sat - 1].fix[0] = 4;
                    rtk->sol.ns++;
                }
            rtk->sol.stat = stat;

            for (i = 0; i < 3; i++)
                {
                    rtk->sol.rr[i] = rtk->x[i];
                    rtk->sol.qr[i] = (float)rtk->P[i + i * rtk->nx];
                }
            rtk->sol.qr[3] = (float)rtk->P[1];
            rtk->sol.qr[4] = (float)rtk->P[2 + rtk->nx];
            rtk->sol.qr[5] = (float)rtk->P[2];
            rtk->sol.dtr[0] = rtk->x[IC_PPP(0, opt)];
            rtk->sol.dtr[1] = rtk->x[IC_PPP(1, opt)] - rtk->x[IC_PPP(0, opt)];
            for (i = 0; i < n && i < MAXOBS; i++)
                {
                    rtk->ssat[obs[i].sat - 1].snr[0] = MIN_PPP(obs[i].SNR[0], obs[i].SNR[1]);
                }
            for (i = 0; i < MAXSAT; i++)
                {
                    if (rtk->ssat[i].slip[0] & 3) rtk->ssat[i].slipc[0]++;
                }
        }
    free(rs);
    free(dts);
    free(var);
    free(azel);
    free(xp);
    free(Pp);
    free(v);
    free(H);
    free(R);
}
