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
 * -----------------------------------------------------------------------------
 * Copyright (C) 2007-2008, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * -----------------------------------------------------------------------------
 */

#include "rtklib_ppp.h"
#include "rtklib_ephemeris.h"
#include "rtklib_ionex.h"
#include "rtklib_lambda.h"
#include "rtklib_pntpos.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_sbas.h"
#include "rtklib_tides.h"
#include <algorithm>
#include <cstring>
#include <vector>

/* wave length of LC (m) -----------------------------------------------------*/
double lam_LC(int i, int j, int k)
{
    const double f1 = FREQ1;
    const double f2 = FREQ2;
    const double f5 = FREQ5;

    return SPEED_OF_LIGHT_M_S / (i * f1 + j * f2 + k * f5);
}


/* carrier-phase LC (m) ------------------------------------------------------*/
double L_LC(int i, int j, int k, const double *L)
{
    const double f1 = FREQ1;
    const double f2 = FREQ2;
    const double f5 = FREQ5;
    double L1;
    double L2;
    double L5;

    if ((i && !L[0]) || (j && !L[1]) || (k && !L[2]))
        {
            return 0.0;
        }
    L1 = SPEED_OF_LIGHT_M_S / f1 * L[0];
    L2 = SPEED_OF_LIGHT_M_S / f2 * L[1];
    L5 = SPEED_OF_LIGHT_M_S / f5 * L[2];
    return (i * f1 * L1 + j * f2 * L2 + k * f5 * L5) / (i * f1 + j * f2 + k * f5);
}


/* pseudorange LC (m) --------------------------------------------------------*/
double P_LC(int i, int j, int k, const double *P)
{
    const double f1 = FREQ1;
    const double f2 = FREQ2;
    const double f5 = FREQ5;
    double P1;
    double P2;
    double P5;

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
    const double f1 = FREQ1;
    const double f2 = FREQ2;
    const double f5 = FREQ5;

    return (std::pow(i * f1, 2.0) + std::pow(j * f2, 2.0) + std::pow(k * f5, 2.0)) / std::pow(i * f1 + j * f2 + k * f5, 2.0) * std::pow(sig, 2.0);
}


/* complementaty error function (ref [1] p.227-229) --------------------------*/
double p_gamma(double a, double x, double log_gamma_a)
{
    double y;
    double w;
    int i;

    if (x == 0.0)
        {
            return 0.0;
        }
    if (x >= a + 1.0)
        {
            return 1.0 - q_gamma(a, x, log_gamma_a);
        }

    y = w = exp(a * log(x) - x - log_gamma_a) / a;

    for (i = 1; i < 100; i++)
        {
            w *= x / (a + i);
            y += w;
            if (fabs(w) < 1E-15)
                {
                    break;
                }
        }
    return y;
}


double q_gamma(double a, double x, double log_gamma_a)
{
    double y;
    double w;
    double la = 1.0;
    double lb = x + 1.0 - a;
    double lc;
    int i;

    if (x < a + 1.0)
        {
            return 1.0 - p_gamma(a, x, log_gamma_a);
        }
    w = exp(-x + a * log(x) - log_gamma_a);
    y = w / lb;
    for (i = 2; i < 100; i++)
        {
            lc = ((i - 1 - a) * (lb - la) + (i + x) * lb) / i;
            la = lb;
            lb = lc;
            w *= (i - 1 - a) / i;
            y += w / la / lb;
            if (fabs(w / la / lb) < 1E-15)
                {
                    break;
                }
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
    double x;
    double p = 1.0;
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
    double LC1;
    double LC2;
    double LC3;
    double var1;
    double var2;
    double var3;
    double sig;
    int i;
    int j;
    int sat;

    for (i = 0; i < n; i++)
        {
            sat = obs[i].sat;

            if (azel[1 + 2 * i] < rtk->opt.elmin)
                {
                    continue;
                }

            if (satsys(sat, nullptr) != SYS_GPS)
                {
                    continue;
                }

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
                    for (j = 0; j < MAXSAT; j++)
                        {
                            amb->flags[j] = 0;
                        }
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
    ambc_t *amb1;
    ambc_t *amb2;
    double BW;
    double vW;
    double lam_WL = lam_LC(1, -1, 0);

    amb1 = rtk->ambc + sat1 - 1;
    amb2 = rtk->ambc + sat2 - 1;
    if (!amb1->n[0] || !amb2->n[0])
        {
            return 0;
        }

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
                {
                    if (flgs[i] == flgs[sat2 - 1])
                        {
                            flgs[i] = flgs[sat1 - 1];
                        }
                }
        }
    else if (flgs[sat1 - 1] < flgs[sat2 - 1])
        {
            for (i = 0; i < MAXSAT; i++)
                {
                    if (flgs[i] == flgs[sat1 - 1])
                        {
                            flgs[i] = flgs[sat2 - 1];
                        }
                }
        }
    else
        {
            return 0; /* linear dependent */
        }
    return 1;
}


/* select fixed ambiguities --------------------------------------------------*/
int sel_amb(int *sat1, int *sat2, double *N, double *var, int n)
{
    int i;
    int j;
    std::vector<int> flgs(MAXSAT, 0);
    int max_flg = 0;

    /* sort by variance */
    for (i = 0; i < n; i++)
        {
            for (j = 1; j < n - i; j++)
                {
                    if (var[j] >= var[j - 1])
                        {
                            continue;
                        }
                    SWAP_I(sat1[j], sat1[j - 1]);
                    SWAP_I(sat2[j], sat2[j - 1]);
                    SWAP_D(N[j], N[j - 1]);
                    SWAP_D(var[j], var[j - 1]);
                }
        }
    /* select linearly independent satellite pair */
    for (i = j = 0; i < n; i++)
        {
            if (!is_depend(sat1[i], sat2[i], flgs.data(), &max_flg))
                {
                    continue;
                }
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
    double *v;
    double *H;
    double *R;
    int i;
    int j;
    int k;
    int info;

    if (n <= 0)
        {
            return 0;
        }

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
    double C1;
    double C2;
    double B1;
    double v1;
    double BC;
    double v;
    double vc;
    double *NC;
    double *var;
    double lam_NL = lam_LC(1, 1, 0);
    double lam1;
    double lam2;
    int i;
    int j;
    int k;
    int m = 0;
    int N1;
    int stat;

    lam1 = LAM_CARR[0];
    lam2 = LAM_CARR[1];

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
            if (fabs(vc) > THRES_RES)
                {
                    continue;
                }

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
    double C1;
    double C2;
    double *B1;
    double *N1;
    double *NC;
    double *D;
    double *E;
    double *Q;
    double s[2];
    double lam_NL = lam_LC(1, 1, 0);
    double lam1;
    double lam2;
    int i;
    int j;
    int k;
    int m = 0;
    int info;
    int stat;
    std::vector<int> flgs(MAXSAT, 0);
    int max_flg = 0;

    lam1 = LAM_CARR[0];
    lam2 = LAM_CARR[1];

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
            if (!is_depend(sat1[i], sat2[i], flgs.data(), &max_flg))
                {
                    continue;
                }

            j = IB_PPP(sat1[i], &rtk->opt);
            k = IB_PPP(sat2[i], &rtk->opt);

            /* float narrow-lane ambiguity (cycle) */
            B1[m] = (rtk->x[j] - rtk->x[k] + C2 * lam2 * NW[i]) / lam_NL;
            N1[m] = ROUND_PPP(B1[m]);

            /* validation of narrow-lane ambiguity */
            if (fabs(N1[m] - B1[m]) > rtk->opt.thresar[2])
                {
                    continue;
                }

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

    rtk->sol.ratio = static_cast<float>(MIN_PPP(s[1] / s[0], 999.9));

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
    int i;
    int j;
    int m = 0;
    int stat = 0;
    int *NW;
    int *sat1;
    int *sat2;

    if (n <= 0 || rtk->opt.ionoopt != IONOOPT_IFLC || rtk->opt.nf < 2)
        {
            return 0;
        }

    trace(3, "pppamb: time=%s n=%d\n", time_str(obs[0].time, 0), n);

    elmask = rtk->opt.elmaskar > 0.0 ? rtk->opt.elmaskar : rtk->opt.elmin;

    sat1 = imat(n * n, 1);
    sat2 = imat(n * n, 1);
    NW = imat(n * n, 1);

    /* average LC */
    average_LC(rtk, obs, n, nav, azel);

    /* fix wide-lane ambiguity */
    for (i = 0; i < n - 1; i++)
        {
            for (j = i + 1; j < n; j++)
                {
                    if (!rtk->ssat[obs[i].sat - 1].vsat[0] ||
                        !rtk->ssat[obs[j].sat - 1].vsat[0] ||
                        azel[1 + i * 2] < elmask || azel[1 + j * 2] < elmask)
                        {
                            continue;
                        }
#if 0
            /* test already fixed */
            if (rtk->ambc[obs[i].sat-1].flags[obs[j].sat-1] &&
                    rtk->ambc[obs[j].sat-1].flags[obs[i].sat-1]) continue;
#endif
                    sat1[m] = obs[i].sat;
                    sat2[m] = obs[j].sat;
                    if (fix_amb_WL(rtk, nav, sat1[m], sat2[m], NW + m))
                        {
                            m++;
                        }
                }
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
    double tow;
    double pos[3];
    double vel[3];
    double acc[3];
    int i;
    int j;
    int week;
    int nfreq = 1;

    if (level <= 0 || !fp)
        {
            return;
        }

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
        week, tow, rtk->sol.stat, 1, rtk->x[i] * 1E9 / SPEED_OF_LIGHT_M_S, rtk->x[i + 1] * 1E9 / SPEED_OF_LIGHT_M_S,
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
    if (rtk->sol.stat == SOLQ_NONE || level <= 1)
        {
            return;
        }

    /* residuals and status */
    for (i = 0; i < MAXSAT; i++)
        {
            ssat = rtk->ssat + i;
            if (!ssat->vs)
                {
                    continue;
                }
            auto id = satno2id(i + 1);
            for (j = 0; j < nfreq; j++)
                {
                    fprintf(fp, "$SAT,%d,%.3f,%s,%d,%.1f,%.1f,%.4f,%.4f,%d,%.0f,%d,%d,%d,%d,%d,%d\n",
                        week, tow, id.data(), j + 1, ssat->azel[0] * R2D, ssat->azel[1] * R2D,
                        ssat->resp[j], ssat->resc[j], ssat->vsat[j], ssat->snr[j] * 0.25,
                        ssat->fix[j], ssat->slip[j] & 3, ssat->lock[j], ssat->outc[j],
                        ssat->slipc[j], ssat->rejc[j]);
                }
        }
}


/* exclude meas of eclipsing satellite (block IIA) ---------------------------*/
void testeclipse(const obsd_t *obs, int n, const nav_t *nav, double *rs)
{
    double rsun[3];
    double esun[3];
    double r;
    double ang;
    double erpv[5] = {0};
    double cosa;
    int i;
    int j;
    const char *type;

    trace(3, "testeclipse:\n");

    /* unit vector of sun direction (ecef) */
    sunmoonpos(gpst2utc(obs[0].time), erpv, rsun, nullptr, nullptr);
    if (normv3(rsun, esun) == 0)
        {
            trace(1, "Error computing the norm");
        }

    for (i = 0; i < n; i++)
        {
            type = nav->pcvs[obs[i].sat - 1].type;

            if ((r = norm_rtk(rs + i * 6, 3)) <= 0.0)
                {
                    continue;
                }
#if 1
            /* only block IIA */
            if (*type && !strstr(type, "BLOCK IIA"))
                {
                    continue;
                }
#endif
            /* sun-earth-satellite angle */
            cosa = dot(rs + i * 6, esun, 3) / r;
            cosa = cosa < -1.0 ? -1.0 : (cosa > 1.0 ? 1.0 : cosa);
            ang = acos(cosa);

            /* test eclipse */
            if (ang < GNSS_PI / 2.0 || r * sin(ang) > RE_WGS84)
                {
                    continue;
                }

            trace(2, "eclipsing sat excluded %s sat=%2d\n", time_str(obs[0].time, 0),
                obs[i].sat);

            for (j = 0; j < 3; j++)
                {
                    rs[j + i * 6] = 0.0;
                }
        }
}


/* measurement error variance ------------------------------------------------*/
double varerr(int sat __attribute__((unused)), int sys, double el, int type, const prcopt_t *opt)
{
    double a;
    double b;
    double a2;
    double b2;
    double fact = 1.0;
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
            if (type == 1)
                {
                    fact *= opt->eratio[0];
                }
            fact *= sysefact(sys);
            if (opt->ionoopt == IONOOPT_IFLC)
                {
                    fact *= 3.0;
                }
            a = fact * opt->err[1];
            b = fact * opt->err[2];
        }
    return a * a + b * b / sinel / sinel;
}


/* measurement error variance of a frequency slot -----------------------------*/
double varerr_slot(int sys, double el, int type, int f, const prcopt_t *opt)
{
    double fact = sysefact(sys);
    const double sinel = sin(el);

    if (type == 1)
        { /* code */
            fact *= opt->eratio[f] > 0.0 ? opt->eratio[f] : opt->eratio[0];
        }
    const double a = fact * opt->err[1];
    const double b = fact * opt->err[2];
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


/* frequency slots of the iono-free measurements of a satellite ---------------*/
void ifmeas_slots(const obsd_t *obs, int *i, int *j)
{
    *i = 0;
    *j = 1;

    /* L1-L2 for GPS/GLO/QZS, L1-L5 for GAL/SBS */
    if (NFREQ >= 3 && (satsys(obs->sat, nullptr) & (SYS_GAL | SYS_SBS)))
        {
            *j = 2;
        }

    if (satsys(obs->sat, nullptr) == SYS_BDS)
        {
            bds_observation_slots(obs, i, j);
        }
}


/* receiver phase clock state of the phase of a satellite, in the models with one
 * phase for each satellite (see udphaseclock_ppp()): for the iono-free
 * measurements, one for each system and pair of frequency slots; otherwise, the
 * one of the first frequency slot of the system
 *-----------------------------------------------------------------------------*/
int ppp_phase_clock(const obsd_t *obs, const prcopt_t *opt)
{
    int i = 0;
    int j = 0;
    const int system_index = ppp_system_index(satsys(obs->sat, nullptr));
    if (opt->ionoopt != IONOOPT_IFLC)
        {
            return IH_PPP(system_index, 0, opt);
        }
    ifmeas_slots(obs, &i, &j);
    return IH_PPP(system_index, (i + j + NFREQ - 1) % NFREQ, opt);
}


/* dual-frequency iono-free measurements -------------------------------------*/
int ifmeas(const obsd_t *obs, const nav_t *nav, const double *azel,
    const prcopt_t *opt, const double *dantr, const double *dants,
    double phw, double *meas, double *var)
{
    const double *lam = nav->lam[obs->sat - 1];
    double c1;
    double c2;
    double L1;
    double L2;
    double P1;
    double P2;
    double P1_C1;
    double P2_C2;
    double gamma;
    int i;
    int j;
    int k;

    trace(4, "ifmeas  :\n");

    ifmeas_slots(obs, &i, &j);

    if (NFREQ < 2 || i == j || lam[i] == 0.0 || lam[j] == 0.0 || lam[i] == lam[j])
        {
            return 0;
        }

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
            P1_C1 = nav->lexeph[obs->sat - 1].isc[0] * SPEED_OF_LIGHT_M_S; /* ISC_L1C/A */
        }
    if (L1 == 0.0 || L2 == 0.0 || P1 == 0.0 || P2 == 0.0)
        {
            return 0;
        }

    /* iono-free phase with windup correction */
    meas[0] = c1 * L1 + c2 * L2 - (c1 * lam[i] + c2 * lam[j]) * phw;

    /* iono-free code with dcb correction */
    if (obs->code[i] == CODE_L1C)
        {
            P1 += P1_C1; /* C1->P1 */
        }
    if (obs->code[j] == CODE_L2C)
        {
            P2 += P2_C2; /* C2->P2 */
        }
    if (satsys(obs->sat, nullptr) == SYS_BDS)
        {
            P1 -= gettgd_bds_by_obs_code(obs->sat, nav, obs->code[i]);
            P2 -= gettgd_bds_by_obs_code(obs->sat, nav, obs->code[j]);
        }
    meas[1] = c1 * P1 + c2 * P2;
    var[1] = std::pow(ERR_CBIAS, 2.0);

    if (opt->sateph == EPHOPT_SBAS)
        {
            meas[1] -= P1_C1; /* sbas clock based C1 */
        }

    /* gps-glonass h/w bias correction for code */
    if (opt->exterr.ena[3] && satsys(obs->sat, nullptr) == SYS_GLO)
        {
            meas[1] += c1 * opt->exterr.gpsglob[0] + c2 * opt->exterr.gpsglob[1];
        }
    /* antenna phase center variation correction */
    for (k = 0; k < 2; k++)
        {
            if (dants)
                {
                    meas[k] -= c1 * dants[i] + c2 * dants[j];
                }
            if (dantr)
                {
                    meas[k] -= c1 * dantr[i] + c2 * dantr[j];
                }
        }
    return 1;
}


/* get tgd parameter (m) -----------------------------------------------------*/
double gettgd_ppp(int sat, const nav_t *nav, int tgd_index)
{
    if (tgd_index < 0 || tgd_index >= 4)
        {
            return 0.0;
        }

    int i;
    for (i = 0; i < nav->n; i++)
        {
            if (nav->eph[i].sat != sat)
                {
                    continue;
                }
            return SPEED_OF_LIGHT_M_S * nav->eph[i].tgd[tgd_index];
        }
    return 0.0;
}


double gettgd_ppp(int sat, const nav_t *nav)
{
    return gettgd_ppp(sat, nav, 0);
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
    double ion = 0.0;
    double L1;
    double P1;
    double PC;
    double P1_P2;
    double P1_C1;
    double vari;
    double gamma;
    int i;

    trace(4, "corrmeas:\n");

    meas[0] = meas[1] = var[0] = var[1] = 0.0;

    /* iono-free LC */
    if (opt->ionoopt == IONOOPT_IFLC)
        {
            return ifmeas(obs, nav, azel, opt, dantr, dants, phw, meas, var);
        }
    if (lam[0] == 0.0 || obs->L[0] == 0.0 || obs->P[0] == 0.0)
        {
            return 0;
        }

    if (testsnr(0, 0, azel[1], obs->SNR[0] * 0.25, &opt->snrmask))
        {
            return 0;
        }

    L1 = obs->L[0] * lam[0];
    P1 = obs->P[0];

    /* dcb correction */
    gamma = std::pow(lam[1] / lam[0], 2.0); /* f1^2/f2^2 */
    P1_P2 = nav->cbias[obs->sat - 1][0];
    P1_C1 = nav->cbias[obs->sat - 1][1];
    const int sys = satsys(obs->sat, nullptr);
    if (P1_P2 == 0.0 && (sys & (SYS_GPS | SYS_GAL | SYS_QZS)))
        {
            int tgd_index = 0;
            if (sys == SYS_GAL)
                {
                    tgd_index = galileo_bgd_index(obs->code[0], obs->sat, nav);
                }
            P1_P2 = (1.0 - gamma) * gettgd_ppp(obs->sat, nav, tgd_index);
        }
    if (obs->code[0] == CODE_L1C)
        {
            P1 += P1_C1; /* C1->P1 */
        }
    PC = P1 - P1_P2 / (1.0 - gamma); /* P1->PC */

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
            if (dants)
                {
                    meas[i] -= dants[0];
                }
            if (dantr)
                {
                    meas[i] -= dantr[0];
                }
        }
    return 1;
}


/* geometry-free phase measurement of two frequency slots --------------------*/
double gfmeas(const obsd_t *obs, const nav_t *nav, int i, int j)
{
    const double *lam = nav->lam[obs->sat - 1];

    if (i == j || lam[i] == 0.0 || lam[j] == 0.0 || obs->L[i] == 0.0 || obs->L[j] == 0.0)
        {
            return 0.0;
        }

    return lam[i] * obs->L[i] - lam[j] * obs->L[j];
}


/* temporal update of position -----------------------------------------------*/
void udpos_ppp(rtk_t *rtk)
{
    int i;

    trace(3, "udpos_ppp:\n");

    /* fixed mode */
    if (rtk->opt.mode == PMODE_PPP_FIXED)
        {
            for (i = 0; i < 3; i++)
                {
                    initx(rtk, rtk->opt.ru[i], 1E-8, i);
                }
            return;
        }
    /* initialize position for first epoch */
    if (norm_rtk(rtk->x, 3) <= 0.0)
        {
            for (i = 0; i < 3; i++)
                {
                    initx(rtk, rtk->sol.rr[i], VAR_POS_PPP, i);
                }
        }
    /* static ppp mode */
    if (rtk->opt.mode == PMODE_PPP_STATIC)
        {
            return;
        }

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
            initx(rtk, SPEED_OF_LIGHT_M_S * dtr, VAR_CLK, IC_PPP(i, &rtk->opt));
        }
}


/* temporal update of tropospheric parameters --------------------------------*/
void udtrop_ppp(rtk_t *rtk)
{
    double pos[3];
    double azel[] = {0.0, GNSS_PI / 2.0};
    double ztd;
    double var;
    int i = IT_PPP(&rtk->opt);
    int j;

    trace(3, "udtrop_ppp:\n");

    if (rtk->x[i] == 0.0)
        {
            ecef2pos(rtk->sol.rr, pos);
            ztd = sbstropcorr(rtk->sol.time, pos, azel, &var);
            initx(rtk, ztd, var, i);

            if (rtk->opt.tropopt >= TROPOPT_ESTG)
                {
                    for (j = 0; j < 2; j++)
                        {
                            initx(rtk, 1E-6, VAR_GRA_PPP, ++i);
                        }
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
    int i;
    int j;

    trace(3, "detslp_ll: n=%d\n", n);

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            // Dual-frequency measurements can occupy slots 0 and 2.
            for (j = 0; j < NFREQ; j++)
                {
                    if (obs[i].L[j] == 0.0 || !(obs[i].LLI[j] & 3))
                        {
                            continue;
                        }

                    trace(3, "detslp_ll: slip detected sat=%2d f=%d\n", obs[i].sat, j + 1);

                    rtk->ssat[obs[i].sat - 1].slip[j] = 1;
                }
        }
}


/* median of a set of values ---------------------------------------------------*/
double ppp_median(std::vector<double> values)
{
    if (values.empty())
        {
            return 0.0;
        }
    std::sort(values.begin(), values.end());
    const size_t middle = values.size() / 2;
    return values.size() % 2 ? values[middle] : 0.5 * (values[middle - 1] + values[middle]);
}


/* phase-code jump of the receiver ---------------------------------------------
 * A step of the receiver clock moves the codes and not the phases, by the same
 * amount for all the satellites. The argument holds, for the satellites with a
 * phase bias and no cycle slip, the phase bias as measured now less that of the
 * filter. The jump is the mean of the values that agree with their median, so
 * a single satellite is enough to tell it, and a phase bias that an earlier
 * jump left behind does not count. It returns 0.0 if there is no jump
 *-----------------------------------------------------------------------------*/
double ppp_phase_code_jump(const std::vector<double> &differences)
{
    const double median = ppp_median(differences);
    double sum = 0.0;
    int k = 0;
    for (double difference : differences)
        {
            if (fabs(difference - median) <= THRES_PHASE_CODE_JUMP)
                {
                    sum += difference;
                    k++;
                }
        }
    if (k == 0 || fabs(sum / k) <= THRES_PHASE_CODE_JUMP)
        {
            return 0.0;
        }
    return sum / k;
}


/* detect cycle slip by geometry free phase jump -------------------------------
 * with the two frequency slots of the iono-free measurements, or with the first
 * two otherwise. The phases of two bands may drift with respect to each other by
 * the same amount for all the satellites (see udphaseclock_ppp()), so a jump is
 * what departs from the median of the changes over the satellites of the system
 * with that pair of slots. A satellite alone with its pair is tested by its
 * change
 *-----------------------------------------------------------------------------*/
void detslp_gf(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    std::vector<double> change(MAXOBS, 0.0);
    std::vector<int> group(MAXOBS, -1);
    double g0;
    double g1;
    int i;
    int j;

    trace(3, "detslp_gf: n=%d\n", n);

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            int first = 0;
            int second = 1;
            if (rtk->opt.ionoopt == IONOOPT_IFLC)
                {
                    ifmeas_slots(obs + i, &first, &second);
                }
            else if (satsys(obs[i].sat, nullptr) == SYS_BDS)
                {
                    bds_observation_slots(obs + i, &first, &second);
                }
            if ((g1 = gfmeas(obs + i, nav, first, second)) == 0.0)
                {
                    continue;
                }

            g0 = rtk->ssat[obs[i].sat - 1].gf;
            rtk->ssat[obs[i].sat - 1].gf = g1;

            trace(4, "detslip_gf: sat=%2d gf0=%8.3f gf1=%8.3f\n", obs[i].sat, g0, g1);

            if (g0 != 0.0)
                {
                    change[i] = g1 - g0;
                    group[i] = ppp_system_index(satsys(obs[i].sat, nullptr)) * NFREQ + (first + second + NFREQ - 1) % NFREQ;
                }
        }

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            if (group[i] < 0)
                {
                    continue;
                }
            std::vector<double> changes;
            for (j = 0; j < n && j < MAXOBS; j++)
                {
                    if (group[j] == group[i])
                        {
                            changes.push_back(change[j]);
                        }
                }
            const double common = changes.size() > 1 ? ppp_median(changes) : 0.0;
            if (fabs(change[i] - common) > rtk->opt.thresslip)
                {
                    trace(3, "detslip_gf: slip detected sat=%2d change=%8.3f common=%8.3f\n",
                        obs[i].sat, change[i], common);

                    for (j = 0; j < NFREQ; j++)
                        {
                            rtk->ssat[obs[i].sat - 1].slip[j] |= 1;
                        }
                }
        }
}


/* temporal update of phase biases -------------------------------------------*/
void udbias_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    double meas[2];
    double var[2];
    std::vector<double> bias(MAXOBS, 0.0);
    std::vector<bool> slip(MAXOBS, false);
    std::vector<double> differences;
    double pos[3] = {0};
    int i;
    int j;
    int sat;
    int brk = 0;

    trace(3, "udbias  : n=%d\n", n);

    for (i = 0; i < MAXSAT; i++)
        {
            for (j = 0; j < NFREQ; j++)
                {
                    rtk->ssat[i].slip[j] = 0;
                }
        }
    /* detect cycle slip by LLI */
    detslp_ll(rtk, obs, n);

    /* detect cycle slip by geometry-free phase jump */
    detslp_gf(rtk, obs, n, nav);

    /* reset phase-bias if expire obs outage counter */
    for (i = 0; i < MAXSAT; i++)
        {
            if (++rtk->ssat[i].outc[0] > static_cast<unsigned int>(rtk->opt.maxout))
                {
                    initx(rtk, 0.0, 0.0, IB_PPP(i + 1, &rtk->opt));
                }
        }
    ecef2pos(rtk->sol.rr, pos);

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            sat = obs[i].sat;
            j = IB_PPP(sat, &rtk->opt);
            if (!corrmeas(obs + i, nav, pos, rtk->ssat[sat - 1].azel, &rtk->opt, nullptr, nullptr,
                    0.0, meas, var, &brk))
                {
                    continue;
                }

            if (brk)
                {
                    rtk->ssat[sat - 1].slip[0] = 1;
                    trace(2, "%s: sat=%2d correction break\n", time_str(obs[i].time, 0), sat);
                }
            bias[i] = meas[0] - meas[1];
            int primary = 0;
            int secondary = 1;
            /* the phase bias is that of the phase with its receiver clock */
            bias[i] -= rtk->x[ppp_phase_clock(obs + i, &rtk->opt)];
            if (rtk->opt.ionoopt == IONOOPT_IFLC)
                {
                    /* frequency slots of the iono-free measurements */
                    ifmeas_slots(obs + i, &primary, &secondary);
                }
            slip[i] = brk || rtk->ssat[sat - 1].slip[primary] || rtk->ssat[sat - 1].slip[secondary];
            if (rtk->x[j] == 0.0 || slip[i])
                {
                    continue;
                }
            differences.push_back(bias[i] - rtk->x[j]);
        }
    /* correct phase-code jump to ensure phase-code coherency */
    const double jump = ppp_phase_code_jump(differences);
    if (jump != 0.0)
        {
            for (i = 0; i < MAXSAT; i++)
                {
                    j = IB_PPP(i + 1, &rtk->opt);
                    if (rtk->x[j] != 0.0)
                        {
                            rtk->x[j] += jump;
                        }
                }
            trace(2, "phase-code jump corrected: %s n=%2d dt=%12.9fs\n",
                time_str(rtk->sol.time, 0), static_cast<int>(differences.size()), jump / SPEED_OF_LIGHT_M_S);
        }
    for (i = 0; i < n && i < MAXOBS; i++)
        {
            sat = obs[i].sat;
            j = IB_PPP(sat, &rtk->opt);

            rtk->P[j + j * rtk->nx] += std::pow(rtk->opt.prn[0], 2.0) * fabs(rtk->tt);

            /* a phase bias that a jump of the receiver clock left behind starts again */
            const bool left_behind = bias[i] != 0.0 && rtk->x[j] != 0.0 && fabs(bias[i] - rtk->x[j]) > THRES_PHASE_CODE_JUMP;
            if (rtk->x[j] != 0.0 && !slip[i] && !left_behind)
                {
                    continue;
                }

            if (bias[i] == 0.0)
                {
                    continue;
                }

            /* reinitialize phase-bias if detecting cycle slip */
            initx(rtk, bias[i], VAR_BIAS, IB_PPP(sat, &rtk->opt));

            trace(5, "udbias_ppp: sat=%2d bias=%.3f\n", sat, meas[0] - meas[1]);
        }
}


/* factor of the ionospheric delay in a frequency slot --------------------------
 * the slant ionospheric delays are estimated at the L1 frequency: the delay of
 * the signal of a frequency slot is that one times (f_L1 / f)^2
 *-----------------------------------------------------------------------------*/
double ppp_iono_factor(const nav_t *nav, int sat, int f)
{
    const double lam = nav->lam[sat - 1][f];
    return (lam / LAM_CARR[0]) * (lam / LAM_CARR[0]);
}


/* detect cycle slips by geometry free phase jumps, per frequency slot ----------
 * with the geometry-free phases of the first frequency slot with the second and
 * with the third: a jump of both is a slip in the first slot, and a jump of one
 * is a slip in the other slot of that pair. The phases of a band may drift with
 * respect to those of another by the same amount for all the satellites (see
 * udphaseclock_ppp()), so a jump is what departs from the median of the changes
 * over the satellites of the system. A satellite alone with its pair is tested
 * by its change
 *-----------------------------------------------------------------------------*/
void detslp_gf_est(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    std::vector<double> change(MAXOBS * 2, 0.0);
    std::vector<bool> compared(MAXOBS * 2, false);
    int i;
    int f;

    trace(3, "detslp_gf_est: n=%d\n", n);

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            const double *lam = nav->lam[obs[i].sat - 1];
            ssat_t *ssat = rtk->ssat + obs[i].sat - 1;
            double *previous[2] = {&ssat->gf, &ssat->gf2};

            for (f = 1; f < NFREQ && f < 3; f++)
                {
                    double g1 = 0.0;
                    if (lam[0] != 0.0 && lam[f] != 0.0 && obs[i].L[0] != 0.0 && obs[i].L[f] != 0.0)
                        {
                            g1 = lam[0] * obs[i].L[0] - lam[f] * obs[i].L[f];
                        }
                    const double g0 = *previous[f - 1];
                    *previous[f - 1] = g1;
                    compared[i * 2 + f - 1] = g0 != 0.0 && g1 != 0.0;
                    change[i * 2 + f - 1] = g1 - g0;
                }
        }

    /* change common to the satellites of each system, for each pair of slots */
    std::vector<double> common(NSYS * 2, 0.0);
    for (int s = 0; s < NSYS; s++)
        {
            for (int pair = 0; pair < 2; pair++)
                {
                    std::vector<double> changes;
                    for (i = 0; i < n && i < MAXOBS; i++)
                        {
                            const int sys = satsys(obs[i].sat, nullptr);
                            if (sys && ppp_system_index(sys) == s && compared[i * 2 + pair])
                                {
                                    changes.push_back(change[i * 2 + pair]);
                                }
                        }
                    common[s * 2 + pair] = changes.size() > 1 ? ppp_median(changes) : 0.0;
                }
        }

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            const int sys = satsys(obs[i].sat, nullptr);
            if (!sys)
                {
                    continue;
                }
            ssat_t *ssat = rtk->ssat + obs[i].sat - 1;
            bool jump[2] = {false, false};
            for (int pair = 0; pair < 2; pair++)
                {
                    jump[pair] = compared[i * 2 + pair] &&
                                 fabs(change[i * 2 + pair] - common[ppp_system_index(sys) * 2 + pair]) > rtk->opt.thresslip;
                }
            if (!jump[0] && !jump[1])
                {
                    continue;
                }
            trace(3, "detslp_gf_est: slip detected sat=%2d jumps=%d %d\n", obs[i].sat, jump[0] ? 1 : 0, jump[1] ? 1 : 0);

            if (jump[0] && jump[1])
                {
                    for (f = 0; f < NFREQ; f++)
                        {
                            ssat->slip[f] |= 1;
                        }
                }
            else
                {
                    /* the other slot of the pair, or both of them if the other
                       pair has no comparison across epochs to tell which */
                    const int pair = jump[0] ? 0 : 1;
                    ssat->slip[pair + 1] |= 1;
                    if (!compared[i * 2 + 1 - pair])
                        {
                            ssat->slip[0] |= 1;
                        }
                }
        }
}


/* temporal update of slant ionospheric delays ---------------------------------
 * a new state starts at the delay of the broadcast model, and then follows a
 * random walk. The phase biases of a satellite are reset here after an outage
 * of their observations, and its ionospheric delay when none of them is left
 *-----------------------------------------------------------------------------*/
void udiono_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    const prcopt_t *opt = &rtk->opt;
    double pos[3] = {0};
    int i;
    int j;
    int f;

    trace(3, "udiono_ppp: n=%d\n", n);

    for (i = 0; i < MAXSAT; i++)
        {
            bool any_phase_bias = false;
            for (f = 0; f < NFREQ; f++)
                {
                    j = IBF_PPP(i + 1, f, opt);
                    if (++rtk->ssat[i].outc[f] > static_cast<unsigned int>(opt->maxout) && rtk->x[j] != 0.0)
                        {
                            initx(rtk, 0.0, 0.0, j);
                        }
                    any_phase_bias = any_phase_bias || rtk->x[j] != 0.0;
                }
            j = II_PPP(i + 1, opt);
            if (!any_phase_bias && rtk->x[j] != 0.0)
                {
                    initx(rtk, 0.0, 0.0, j);
                }
        }

    ecef2pos(rtk->sol.rr, pos);

    /* The phases of two frequencies follow the changes of the delay of a
       satellite. With one, the measurements do not tell the delay from the
       range unless other satellites give the position: the errors of the
       phases in time would pass to the position through the slow change of
       the geometry. A satellite with one frequency takes part once four or
       more satellites with two give the position */
    std::vector<int> phases(MAXOBS, 0);
    int satellites_with_two_phases = 0;
    for (i = 0; i < n && i < MAXOBS; i++)
        {
            for (f = 0; f < NFREQ; f++)
                {
                    phases[i] += (obs[i].L[f] != 0.0 && nav->lam[obs[i].sat - 1][f] != 0.0) ? 1 : 0;
                }
            satellites_with_two_phases += (rtk->ssat[obs[i].sat - 1].vs && phases[i] >= 2) ? 1 : 0;
        }

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            const int sat = obs[i].sat;
            const double *azel = rtk->ssat[sat - 1].azel;
            if (!rtk->ssat[sat - 1].vs || azel[1] <= 0.0)
                {
                    continue;
                }
            j = II_PPP(sat, opt);
            if (phases[i] < 2 && satellites_with_two_phases < 4)
                {
                    /* out of the solution: its states start again later */
                    if (rtk->x[j] != 0.0)
                        {
                            initx(rtk, 0.0, 0.0, j);
                        }
                    for (f = 0; f < NFREQ; f++)
                        {
                            const int k = IBF_PPP(sat, f, opt);
                            if (rtk->x[k] != 0.0)
                                {
                                    initx(rtk, 0.0, 0.0, k);
                                }
                        }
                    continue;
                }
            if (rtk->x[j] == 0.0)
                {
                    double ion = ionmodel(obs[i].time, nav->ion_gps, pos, azel);
                    const double var = std::pow(ion * ERR_BRDCI, 2.0) + VAR_IONO_MODEL;
                    if (ion == 0.0)
                        {
                            ion = 1E-6; /* a state at zero is not estimated */
                        }
                    initx(rtk, ion, var, j);
                }
            else
                {
                    rtk->P[j + j * rtk->nx] += std::pow(opt->prn[1], 2.0) * fabs(rtk->tt);
                }
        }
}


/* temporal update of receiver code biases -------------------------------------
 * code bias of the receiver in the second and in the third frequency slot of
 * each system, with respect to the first one. A new state starts at the median
 * over the satellites of the difference of the pseudoranges, less the
 * ionospheric delay
 *-----------------------------------------------------------------------------*/
void udifb_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    const prcopt_t *opt = &rtk->opt;
    int i;
    int f;
    int s;

    trace(3, "udifb_ppp: n=%d\n", n);

    for (s = 0; s < NSYS; s++)
        {
            for (f = 1; f < NFREQ; f++)
                {
                    const int j = ID_PPP(s, f, opt);
                    if (rtk->x[j] != 0.0)
                        {
                            rtk->P[j + j * rtk->nx] += std::pow(PRN_IFB, 2.0) * fabs(rtk->tt);
                            continue;
                        }
                    std::vector<double> bias;
                    for (i = 0; i < n && i < MAXOBS; i++)
                        {
                            const int sat = obs[i].sat;
                            const int sys = satsys(sat, nullptr);
                            if (!sys || ppp_system_index(sys) != s)
                                {
                                    continue;
                                }
                            const double ion = rtk->x[II_PPP(sat, opt)];
                            const double code_first = prange_slot(obs + i, 0, nav, opt);
                            const double code = prange_slot(obs + i, f, nav, opt);
                            if (ion == 0.0 || code_first == 0.0 || code == 0.0)
                                {
                                    continue;
                                }
                            bias.push_back(code - code_first - (ppp_iono_factor(nav, sat, f) - ppp_iono_factor(nav, sat, 0)) * ion);
                        }
                    if (bias.empty())
                        {
                            continue;
                        }
                    double median = ppp_median(bias);
                    if (median == 0.0)
                        {
                            median = 1E-6; /* a state at zero is not estimated */
                        }
                    initx(rtk, median, VAR_IFB, j);
                    trace(3, "udifb_ppp: system=%d slot=%d code bias=%.3f\n", s, f, median);
                }
        }
}


/* temporal update of receiver phase clocks ------------------------------------
 * The phases of a frequency band have their own receiver clock, with respect
 * to the one of the codes of the system: the oscillators of the bands of a
 * front end are not at their nominal frequencies by the same fraction, so the
 * phases of a band drift with respect to the codes and to the phases of the
 * other bands, by the same amount for all the satellites. One state for each
 * system and frequency slot with a phase in the model (with the iono-free
 * measurements, for each system and pair of frequency slots), as a random walk
 * loose enough to follow it. It
 * takes the part common to the satellites and leaves the phases with what
 * differs between them, as in a model of differences between satellites
 *-----------------------------------------------------------------------------*/
void udphaseclock_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    const prcopt_t *opt = &rtk->opt;
    int i;
    int f;

    trace(3, "udphaseclock_ppp: n=%d\n", n);

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            const int sat = obs[i].sat;
            const int sys = satsys(sat, nullptr);
            if (!sys || !rtk->ssat[sat - 1].vs)
                {
                    continue;
                }
            if (opt->ionoopt == IONOOPT_IFLC)
                {
                    /* one for each pair of frequency slots of the iono-free measurements */
                    int first;
                    int second;
                    ifmeas_slots(obs + i, &first, &second);
                    if (first == second || obs[i].L[first] == 0.0 || obs[i].L[second] == 0.0 ||
                        nav->lam[sat - 1][first] == 0.0 || nav->lam[sat - 1][second] == 0.0)
                        {
                            continue;
                        }
                    const int j = ppp_phase_clock(obs + i, opt);
                    if (rtk->x[j] == 0.0)
                        {
                            initx(rtk, 1E-6, VAR_PHASE_CLOCK, j);
                        }
                    continue;
                }
            if (opt->ionoopt != IONOOPT_EST)
                {
                    /* the phase of the first frequency slot */
                    if (obs[i].L[0] == 0.0 || nav->lam[sat - 1][0] == 0.0)
                        {
                            continue;
                        }
                    const int j = ppp_phase_clock(obs + i, opt);
                    if (rtk->x[j] == 0.0)
                        {
                            initx(rtk, 1E-6, VAR_PHASE_CLOCK, j);
                        }
                    continue;
                }
            for (f = 0; f < NFREQ; f++)
                {
                    if (obs[i].L[f] == 0.0 || nav->lam[sat - 1][f] == 0.0)
                        {
                            continue;
                        }
                    const int j = IH_PPP(ppp_system_index(sys), f, opt);
                    if (rtk->x[j] == 0.0)
                        {
                            initx(rtk, 1E-6, VAR_PHASE_CLOCK, j); /* a state at zero is not estimated */
                        }
                }
        }
    for (i = 0; i < NSYS; i++)
        {
            for (f = 0; f < NFREQ; f++)
                {
                    const int j = IH_PPP(i, f, opt);
                    if (rtk->x[j] != 0.0)
                        {
                            rtk->P[j + j * rtk->nx] += std::pow(PRN_PHASE_CLOCK, 2.0) * fabs(rtk->tt);
                        }
                }
        }
}


/* temporal update of phase biases, per frequency slot --------------------------
 * a phase bias starts, and starts again after a cycle slip, at the phase less
 * the code, plus twice the ionospheric delay, less the receiver phase clock
 *-----------------------------------------------------------------------------*/
void udbias_est_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    const prcopt_t *opt = &rtk->opt;
    std::vector<double> bias(MAXOBS, 0.0);
    std::vector<bool> slip(MAXOBS, false);
    int i;
    int j;
    int f;

    trace(3, "udbias_est_ppp: n=%d\n", n);

    for (i = 0; i < MAXSAT; i++)
        {
            for (f = 0; f < NFREQ; f++)
                {
                    rtk->ssat[i].slip[f] = 0;
                }
        }
    /* detect cycle slip by LLI */
    detslp_ll(rtk, obs, n);

    /* detect cycle slip by geometry-free phase jump */
    detslp_gf_est(rtk, obs, n, nav);

    for (f = 0; f < NFREQ; f++)
        {
            std::vector<double> differences;
            for (i = 0; i < n && i < MAXOBS; i++)
                {
                    const int sat = obs[i].sat;
                    const int sys = satsys(sat, nullptr);
                    const double lam = nav->lam[sat - 1][f];
                    const double ion = rtk->x[II_PPP(sat, opt)];
                    j = IBF_PPP(sat, f, opt);
                    bias[i] = 0.0;
                    slip[i] = false;
                    if (!sys || lam == 0.0 || obs[i].L[f] == 0.0 || ion == 0.0)
                        {
                            continue;
                        }
                    double code = prange_slot(obs + i, f, nav, opt);
                    if (code == 0.0)
                        {
                            continue;
                        }
                    if (f > 0)
                        {
                            code -= rtk->x[ID_PPP(ppp_system_index(sys), f, opt)];
                        }
                    bias[i] = obs[i].L[f] * lam - code + 2.0 * ppp_iono_factor(nav, sat, f) * ion -
                              rtk->x[IH_PPP(ppp_system_index(sys), f, opt)];
                    slip[i] = rtk->ssat[sat - 1].slip[f] != 0;
                    if (rtk->x[j] == 0.0 || slip[i])
                        {
                            continue;
                        }
                    differences.push_back(bias[i] - rtk->x[j]);
                }
            /* correct phase-code jump to ensure phase-code coherency */
            const double jump = ppp_phase_code_jump(differences);
            if (jump != 0.0)
                {
                    for (i = 0; i < MAXSAT; i++)
                        {
                            j = IBF_PPP(i + 1, f, opt);
                            if (rtk->x[j] != 0.0)
                                {
                                    rtk->x[j] += jump;
                                }
                        }
                    trace(2, "phase-code jump corrected: %s f=%d n=%2d dt=%12.9fs\n",
                        time_str(rtk->sol.time, 0), f + 1, static_cast<int>(differences.size()), jump / SPEED_OF_LIGHT_M_S);
                }
            for (i = 0; i < n && i < MAXOBS; i++)
                {
                    j = IBF_PPP(obs[i].sat, f, opt);
                    if (rtk->x[j] != 0.0)
                        {
                            rtk->P[j + j * rtk->nx] += std::pow(opt->prn[0], 2.0) * fabs(rtk->tt);
                        }
                    /* a phase bias that a jump of the receiver clock left behind starts again */
                    const bool left_behind = bias[i] != 0.0 && rtk->x[j] != 0.0 && fabs(bias[i] - rtk->x[j]) > THRES_PHASE_CODE_JUMP;
                    if (bias[i] == 0.0 || (rtk->x[j] != 0.0 && !slip[i] && !left_behind))
                        {
                            continue;
                        }
                    /* (re)initialize the phase bias */
                    initx(rtk, bias[i], VAR_BIAS, j);

                    trace(5, "udbias_est_ppp: sat=%2d f=%d bias=%.3f\n", obs[i].sat, f + 1, bias[i]);
                }
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
    if (rtk->opt.ionoopt == IONOOPT_EST)
        {
            /* temporal update of slant ionospheric delays, receiver code biases
               and phase clocks, and the phase biases of each frequency slot */
            udiono_ppp(rtk, obs, n, nav);
            udifb_ppp(rtk, obs, n, nav);
            udphaseclock_ppp(rtk, obs, n, nav);
            udbias_est_ppp(rtk, obs, n, nav);
            return;
        }
    /* temporal update of receiver phase clocks */
    udphaseclock_ppp(rtk, obs, n, nav);

    /* temporal update of phase-bias */
    udbias_ppp(rtk, obs, n, nav);
}


/* satellite antenna phase center variation ----------------------------------*/
void satantpcv(const double *rs, const double *rr, const pcv_t *pcv,
    double *dant)
{
    double ru[3];
    double rz[3];
    double eu[3];
    double ez[3];
    double nadir;
    double cosa;
    int i;

    for (i = 0; i < 3; i++)
        {
            ru[i] = rr[i] - rs[i];
            rz[i] = -rs[i];
        }
    if (!normv3(ru, eu) || !normv3(rz, ez))
        {
            return;
        }

    cosa = dot(eu, ez, 3);
    cosa = cosa < -1.0 ? -1.0 : (cosa > 1.0 ? 1.0 : cosa);
    nadir = acos(cosa);

    antmodel_s(pcv, nadir, dant);
}


/* satellite antenna phase center offset ---------------------------------------
 * adds to a range correction the offset of the phase center of each frequency
 * with respect to the reference point of the satellite position
 * args   : double *rs       I   satellite position (ecef) (m)
 *          double *e        I   line-of-sight vector, receiver to satellite (ecef)
 *          double *rsun     I   sun position (ecef) (m)
 *          pcv_t  *pcv      I   satellite antenna offsets, x/y/z (m)
 *          double *ref      I   offset of the reference point of the satellite
 *                               position, x/y/z (m)
 *          double *dant     IO  range correction of each frequency (m)
 * return : none
 * notes  : nominal attitude, as in satantoff(): z-axis to the center of the
 *          Earth, y-axis along the solar panels and normal to the direction of
 *          the Sun (Galileo HAS Reference User Algorithm TN, Issue 1.0,
 *          Appendix C)
 *-----------------------------------------------------------------------------*/
void satantpco(const double *rs, const double *e, const double *rsun, const pcv_t *pcv,
    const double *ref, double *dant)
{
    double ex[3];
    double ey[3];
    double ez[3];
    double es[3];
    double r[3];
    double los[3];
    int i;
    int j;

    /* unit vectors of satellite fixed coordinates */
    for (i = 0; i < 3; i++)
        {
            r[i] = -rs[i];
        }
    if (!normv3(r, ez))
        {
            return;
        }
    for (i = 0; i < 3; i++)
        {
            r[i] = rsun[i] - rs[i];
        }
    if (!normv3(r, es))
        {
            return;
        }
    cross3(ez, es, r);
    if (!normv3(r, ey))
        {
            return;
        }
    cross3(ey, ez, ex);

    /* line of sight in satellite fixed coordinates */
    los[0] = dot(e, ex, 3);
    los[1] = dot(e, ey, 3);
    los[2] = dot(e, ez, 3);

    for (i = 0; i < NFREQ; i++)
        {
            for (j = 0; j < 3; j++)
                {
                    dant[i] += los[j] * (pcv->off[i][j] - ref[j]);
                }
        }
}


/* precise tropospheric model ------------------------------------------------*/
double prectrop(gtime_t time, const double *pos, const double *azel,
    const prcopt_t *opt, const double *x, double *dtdx,
    double *var)
{
    const double zazel[] = {0.0, GNSS_PI / 2.0};
    double zhd;
    double m_h;
    double m_w;
    double cotz;
    double grad_n;
    double grad_e;

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


/* receiver clock state of a system -------------------------------------------
 * Each system has the receiver clock state that udclk_ppp() initializes from
 * the single point solution, where the receiver clock of a system other than
 * GPS is the GPS one plus the offset of the system (see rescode()). Without
 * GPS satellites, the whole receiver clock offset is in that term.
 *-----------------------------------------------------------------------------*/
int ppp_clock_index(int sys, const prcopt_t *opt)
{
    int index = 0;
    switch (sys)
        {
        case SYS_GLO:
            index = 1;
            break;
        case SYS_GAL:
            index = 2;
            break;
        case SYS_BDS:
            index = 3;
            break;
        case SYS_QZS:
            /* QZS shares the GPS clock unless its offset is estimated */
            index = opt->estqzsisb ? 4 : 0;
            break;
        default:
            break;
        }
    return index < NSYS ? index : 0;
}


/* index of the system of a satellite, for the tables with an entry per system */
int ppp_system_index(int sys)
{
    int index = 0;
    switch (sys)
        {
        case SYS_GLO:
            index = 1;
            break;
        case SYS_GAL:
            index = 2;
            break;
        case SYS_BDS:
            index = 3;
            break;
        case SYS_QZS:
            index = 4;
            break;
        default:
            break;
        }
    return index < NSYS ? index : 0;
}


/* offset common to the innovations of each group of rows (see res_ppp()) --------
 * the median of the code innovations of each receiver clock, and of the phase
 * innovations of each receiver phase clock. The phase of a satellite alone in
 * its group cannot be told from that clock, so it takes the offset of the codes
 *-----------------------------------------------------------------------------*/
void ppp_common_innovations(const std::vector<double> &innovations, const std::vector<int> &row_group,
    const std::vector<int> &row_type, const std::vector<int> &row_clock, std::vector<double> &common)
{
    for (size_t k = 0; k < common.size(); k++)
        {
            const bool phases = k >= static_cast<size_t>(NSYS);
            std::vector<double> values;
            int clock = 0;
            for (size_t i = 0; i < innovations.size(); i++)
                {
                    if (row_group[i] == static_cast<int>(k) && (phases || row_type[i] == 1))
                        {
                            values.push_back(innovations[i]);
                            clock = row_clock[i];
                        }
                }
            if (values.empty())
                {
                    continue;
                }
            common[k] = (phases && values.size() == 1) ? common[clock] : ppp_median(values);
        }
}


/* phase and code residuals --------------------------------------------------*/
int res_ppp(int iter __attribute__((unused)), const obsd_t *obs, int n, const double *rs,
    const double *dts, const double *vare, const int *svh,
    const nav_t *nav, const double *x, rtk_t *rtk, double *v,
    double *H, double *R, double *azel)
{
    prcopt_t *opt = &rtk->opt;
    double r;
    double rr[3];
    double disp[3];
    double pos[3];
    double e[3];
    double meas[2];
    double dtdx[3];
    double dantr[NFREQ] = {0};
    double dants[NFREQ] = {0};
    double rsun[3] = {0};
    const int max_rows = MAXOBS * 2 * NF_PPP(opt);
    std::vector<double> var(max_rows, 0.0);
    std::vector<int> row_obs(max_rows, 0);               /* observation of each row */
    std::vector<int> row_type(max_rows, 0);              /* 0: phase, 1: code */
    std::vector<int> row_clock(max_rows, 0);             /* receiver clock of each row */
    std::vector<int> row_freq(max_rows, 0);              /* frequency slot of each row */
    std::vector<int> row_group(max_rows, 0);             /* rows with a common receiver term: codes of a clock, or phases of a clock and frequency slot */
    std::vector<double> row_common_var(max_rows, 0.0);   /* variance of the errors common to the rows of a satellite (m^2) */
    std::vector<double> common(NSYS * (1 + NFREQ), 0.0); /* innovation common to the rows of a group (m) */
    double dtrp = 0.0;
    double vart = 0.0;
    double varm[2] = {0};
    int i;
    int j;
    int k;
    int sat;
    int sys;
    int nv = 0;
    int nrows;
    int nx = rtk->nx;
    int brk;
    int tideopt;

    trace(3, "res_ppp : n=%d nx=%d\n", n, nx);

    for (i = 0; i < MAXSAT; i++)
        {
            rtk->ssat[i].vsat[0] = 0;
            if (opt->ionoopt == IONOOPT_EST)
                {
                    /* phases of each frequency slot in the solution */
                    for (j = 0; j < NFREQ; j++)
                        {
                            rtk->ssat[i].fix[j] = 0;
                        }
                }
        }

    for (i = 0; i < 3; i++)
        {
            rr[i] = x[i];
        }

    /* earth tides correction */
    if (opt->tidecorr)
        {
            tideopt = opt->tidecorr == 1 ? 1 : 7; /* 1:solid, 2:solid+otl+pole */

            tidedisp(gpst2utc(obs[0].time), rr, tideopt, &nav->erp, opt->odisp[0],
                disp);
            for (i = 0; i < 3; i++)
                {
                    rr[i] += disp[i];
                }
        }
    ecef2pos(rr, pos);

    /* sun position, for the attitude of the satellites */
    if (opt->posopt[0] && n > 0)
        {
            const double erpv[5] = {0};
            double gmst;
            sunmoonpos(gpst2utc(obs[0].time), erpv, rsun, nullptr, &gmst);
        }

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            sat = obs[i].sat;
            if (!(sys = satsys(sat, nullptr)) || !rtk->ssat[sat - 1].vs)
                {
                    continue;
                }

            /* geometric distance/azimuth/elevation angle */
            if ((r = geodist(rs + i * 6, rr, e)) <= 0.0 ||
                satazel(pos, e, azel + i * 2) < opt->elmin)
                {
                    continue;
                }

            /* excluded satellite? */
            if (satexclude(obs[i].sat, svh[i], opt))
                {
                    continue;
                }

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
            /* satellite antenna model: variation of the phase center, and its
               offset from the reference point of the satellite position */
            if (opt->posopt[0])
                {
                    satantpcv(rs + i * 6, rr, nav->pcvs + sat - 1, dants);
                    satantpco(rs + i * 6, e, rsun, nav->pcvs + sat - 1, nav->pcoref[sat - 1], dants);
                }
            /* receiver antenna model, for the signals of the system */
            antmodel(nav->pcvrcv + ppp_system_index(sys), opt->antdel[0], azel + i * 2, opt->posopt[1], dantr);

            /* phase windup correction */
            if (opt->posopt[2])
                {
                    windupcorr(rtk->sol.time, rs + i * 6, rr, &rtk->ssat[sat - 1].phw);
                }
            if (opt->ionoopt == IONOOPT_EST)
                {
                    /* phase and code of each frequency slot, with the slant
                       ionospheric delay of the satellite as a state */
                    const double ion = x[II_PPP(sat, opt)];
                    if (ion == 0.0)
                        {
                            continue;
                        }
                    const int est_clock_index = ppp_clock_index(sys, opt);
                    const int est_clock_state = IC_PPP(est_clock_index, opt);
                    const int system_index = ppp_system_index(sys);

                    /* satellite clock and tropospheric delay */
                    r += -SPEED_OF_LIGHT_M_S * dts[i * 2] + dtrp;

                    for (int f = 0; f < NFREQ; f++)
                        {
                            const double lam = nav->lam[sat - 1][f];
                            if (lam == 0.0 || obs[i].code[f] == CODE_NONE ||
                                testsnr(0, f, azel[1 + i * 2], obs[i].SNR[f] * 0.25, &opt->snrmask))
                                {
                                    continue;
                                }
                            const double factor = ppp_iono_factor(nav, sat, f);
                            const int iono_state = II_PPP(sat, opt);
                            const int bias_state = IBF_PPP(sat, f, opt);
                            const int code_bias_state = f > 0 ? ID_PPP(system_index, f, opt) : -1;
                            const int phase_clock_state = IH_PPP(system_index, f, opt);

                            for (j = 0; j < 2; j++)
                                { /* for phase and code */
                                    double measurement;
                                    if (j == 0)
                                        {
                                            /* a phase needs its phase bias */
                                            if (obs[i].L[f] == 0.0 || x[bias_state] == 0.0)
                                                {
                                                    continue;
                                                }
                                            measurement = obs[i].L[f] * lam - lam * rtk->ssat[sat - 1].phw;
                                        }
                                    else
                                        {
                                            measurement = prange_slot(obs + i, f, nav, opt);
                                            if (measurement == 0.0)
                                                {
                                                    continue;
                                                }
                                        }
                                    measurement -= dants[f] + dantr[f];

                                    for (k = 0; k < nx; k++)
                                        {
                                            H[k + nx * nv] = 0.0;
                                        }
                                    v[nv] = measurement - r - x[est_clock_state];
                                    for (k = 0; k < 3; k++)
                                        {
                                            H[k + nx * nv] = -e[k];
                                        }
                                    H[est_clock_state + nx * nv] = 1.0;
                                    if (opt->tropopt >= TROPOPT_EST)
                                        {
                                            for (k = 0; k < (opt->tropopt >= TROPOPT_ESTG ? 3 : 1); k++)
                                                {
                                                    H[IT_PPP(opt) + k + nx * nv] = dtdx[k];
                                                }
                                        }
                                    /* the ionosphere delays the code and advances the phase */
                                    const double sign = j == 0 ? -1.0 : 1.0;
                                    v[nv] -= sign * factor * ion;
                                    H[iono_state + nx * nv] = sign * factor;
                                    if (j == 0)
                                        {
                                            v[nv] -= x[bias_state] + x[phase_clock_state];
                                            H[bias_state + nx * nv] = 1.0;
                                            H[phase_clock_state + nx * nv] = 1.0;
                                        }
                                    else if (code_bias_state >= 0 && x[code_bias_state] != 0.0)
                                        {
                                            v[nv] -= x[code_bias_state];
                                            H[code_bias_state + nx * nv] = 1.0;
                                        }
                                    /* the errors of the orbit, the clock and the troposphere
                                       are the same in all the rows of the satellite */
                                    row_common_var[nv] = vare[i] + vart;
                                    var[nv] = varerr_slot(sys, azel[1 + i * 2], j, f, opt) + (j == 1 ? std::pow(ERR_CBIAS, 2.0) : 0.0) + row_common_var[nv];

                                    if (j == 0)
                                        {
                                            rtk->ssat[sat - 1].resc[f] = v[nv];
                                        }
                                    else
                                        {
                                            rtk->ssat[sat - 1].resp[f] = v[nv];
                                        }
                                    row_obs[nv] = i;
                                    row_type[nv] = j;
                                    row_clock[nv] = est_clock_index;
                                    row_freq[nv] = f;
                                    row_group[nv] = j == 1 ? est_clock_index : NSYS + system_index * NFREQ + f;
                                    nv++;
                                }
                        }
                    continue;
                }

            /* ionosphere and antenna phase corrected measurements */
            if (!corrmeas(obs + i, nav, pos, azel + i * 2, &rtk->opt, dantr, dants,
                    rtk->ssat[sat - 1].phw, meas, varm, &brk))
                {
                    continue;
                }
            /* satellite clock and tropospheric delay */
            r += -SPEED_OF_LIGHT_M_S * dts[i * 2] + dtrp;

            /* receiver clock of the system */
            const int clock_index = ppp_clock_index(sys, opt);
            const int clock_state = IC_PPP(clock_index, opt);

            /* receiver clock of the phase */
            const int phase_clock_state = ppp_phase_clock(obs + i, opt);

            trace(5, "sat=%2d azel=%6.1f %5.1f dtrp=%.3f dantr=%6.3f %6.3f dants=%6.3f %6.3f phw=%6.3f\n",
                sat, azel[i * 2] * R2D, azel[1 + i * 2] * R2D, dtrp, dantr[0], dantr[1], dants[0],
                dants[1], rtk->ssat[sat - 1].phw);

            for (j = 0; j < 2; j++)
                { /* for phase and code */
                    if (meas[j] == 0.0)
                        {
                            continue;
                        }

                    for (k = 0; k < nx; k++)
                        {
                            H[k + nx * nv] = 0.0;
                        }

                    v[nv] = meas[j] - r;

                    for (k = 0; k < 3; k++)
                        {
                            H[k + nx * nv] = -e[k];
                        }

                    v[nv] -= x[clock_state];
                    H[clock_state + nx * nv] = 1.0;
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
                            v[nv] -= x[phase_clock_state];
                            H[phase_clock_state + nx * nv] = 1.0;
                        }
                    var[nv] = varerr(obs[i].sat, sys, azel[1 + i * 2], j, opt) + varm[j] + vare[i] + vart;

                    if (j == 0)
                        {
                            rtk->ssat[sat - 1].resc[0] = v[nv];
                        }
                    else
                        {
                            rtk->ssat[sat - 1].resp[0] = v[nv];
                        }

                    row_obs[nv] = i;
                    row_type[nv] = j;
                    row_clock[nv] = clock_index;
                    row_group[nv] = j == 0 ? NSYS + phase_clock_state - IH_PPP(0, 0, opt) : clock_index;
                    nv++;
                }
        }

    /* The receiver clocks are estimated anew at each epoch, from the single
       point solution. An offset common to the innovations of the satellites of
       a receiver clock is then not a fault of their observations: it shows that
       the position of the filter and that solution differ, as after a start
       with poor geometry, and testing the innovations as they are rejects the
       satellites that would correct the position. The offset is taken as the
       median of the code innovations. The phases have a receiver clock of
       their own for each system and frequency slot, or pair of them with the
       iono-free measurements, and take the median of their innovations.
       For the same reason, the innovations are also computed as they would be
       at the position of the single point solution: a satellite that agrees
       with it is not an outlier, even if most satellites agree with a wrong
       position of the filter */
    std::vector<double> innovations(v, v + nv);
    std::vector<double> innovations_spp(nv, 0.0);
    for (i = 0; i < nv; i++)
        {
            innovations_spp[i] = v[i];
            for (k = 0; k < 3; k++)
                {
                    innovations_spp[i] += H[k + nx * i] * (x[k] - rtk->sol.rr[k]);
                }
        }
    std::vector<double> common_spp(common.size(), 0.0);
    ppp_common_innovations(innovations, row_group, row_type, row_clock, common);
    ppp_common_innovations(innovations_spp, row_group, row_type, row_clock, common_spp);

    /* test innovation, with separate thresholds for phase (type 0) and code (type 1) */
    nrows = nv;
    nv = 0;
    for (i = 0; i < nrows; i++)
        {
            const int type = row_type[i];
            sat = obs[row_obs[i]].sat;
            if (opt->maxinno[type] > 0.0 && fabs(innovations[i] - common[row_group[i]]) > opt->maxinno[type] &&
                fabs(innovations_spp[i] - common_spp[row_group[i]]) > opt->maxinno[type] &&
                satsys(sat, nullptr) != SYS_GLO)
                {
                    trace(2, "ppp outlier rejected %s sat=%2d type=%d v=%.3f common=%.3f\n",
                        time_str(obs[row_obs[i]].time, 0), sat, type, v[i], common[row_group[i]]);
                    rtk->ssat[sat - 1].rejc[row_freq[i]]++;
                    continue;
                }
            if (type == 0)
                {
                    rtk->ssat[sat - 1].vsat[0] = 1;
                    if (opt->ionoopt == IONOOPT_EST)
                        {
                            rtk->ssat[sat - 1].fix[row_freq[i]] = 4;
                        }
                }
            if (nv != i)
                {
                    v[nv] = v[i];
                    var[nv] = var[i];
                    row_obs[nv] = row_obs[i];
                    row_common_var[nv] = row_common_var[i];
                    for (k = 0; k < nx; k++)
                        {
                            H[k + nx * nv] = H[k + nx * i];
                        }
                }
            nv++;
        }
    const bool correlated_rows = opt->ionoopt == IONOOPT_EST;
    for (i = 0; i < nv; i++)
        {
            for (j = 0; j < nv; j++)
                {
                    R[i + j * nv] = i == j ? var[i] : ((correlated_rows && row_obs[i] == row_obs[j]) ? row_common_var[i] : 0.0);
                }
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
    double *rs;
    double *dts;
    double *var;
    double *v;
    double *H;
    double *R;
    double *azel;
    double *xp;
    double *Pp;
    int i;
    int nv;
    int info;
    std::vector<int> svh(MAXOBS);
    int stat = SOLQ_SINGLE;

    trace(3, "pppos   : nx=%d n=%d\n", rtk->nx, n);

    rs = mat(6, n);
    dts = mat(2, n);
    var = mat(1, n);
    azel = zeros(2, n);

    for (i = 0; i < MAXSAT; i++)
        {
            rtk->ssat[i].fix[0] = 0;
        }

    /* temporal update of states */
    udstate_ppp(rtk, obs, n, nav);

    trace(4, "x(0)=");
    tracemat(4, rtk->x, 1, NR_PPP(opt), 13, 4);

    /* satellite positions and clocks */
    satposs(obs[0].time, obs, n, nav, rtk->opt.sateph, rs, dts, var, svh.data());

    /* The broadcast accuracy would weight the carrier phases of a satellite
       with Galileo HAS corrections at the meter level. The single point
       solution keeps it, as a margin for its validation of residuals */
    if (rtk->opt.sateph == EPHOPT_BRDC)
        {
            for (i = 0; i < n && i < MAXOBS; i++)
                {
                    var[i] = var_has(obs[0].time, obs[i].sat, nav, var[i]);
                }
        }

    /* exclude measurements of eclipsing satellite */
    if (rtk->opt.posopt[3])
        {
            testeclipse(obs, n, nav, rs);
        }
    xp = mat(rtk->nx, 1);
    Pp = zeros(rtk->nx, rtk->nx);
    matcpy(xp, rtk->x, rtk->nx, 1);
    nv = n * std::max(rtk->opt.nf, NF_PPP(opt)) * 2;
    v = mat(nv, 1);
    H = mat(rtk->nx, nv);
    R = mat(nv, nv);

    for (i = 0; i < rtk->opt.niter; i++)
        {
            /* phase and code residuals */
            if ((nv = res_ppp(i, obs, n, rs, dts, var, svh.data(), nav, xp, rtk, v, H, R, azel)) <= 0)
                {
                    break;
                }

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
            res_ppp(1, obs, n, rs, dts, var, svh.data(), nav, xp, rtk, v, H, R, azel);

            /* update state and covariance matrix */
            matcpy(rtk->x, xp, rtk->nx, 1);
            matcpy(rtk->P, Pp, rtk->nx, rtk->nx);

            /* ambiguity resolution in ppp */
            if (opt->modear == ARMODE_PPPAR || opt->modear == ARMODE_PPPAR_ILS)
                {
                    if (pppamb(rtk, obs, n, nav, azel))
                        {
                            stat = SOLQ_FIX;
                        }
                }
            /* update solution status */
            rtk->sol.ns = 0;
            for (i = 0; i < n && i < MAXOBS; i++)
                {
                    if (!rtk->ssat[obs[i].sat - 1].vsat[0])
                        {
                            continue;
                        }
                    if (opt->ionoopt == IONOOPT_EST)
                        {
                            /* res_ppp() flags the phases of each frequency slot */
                            for (int f = 0; f < NFREQ; f++)
                                {
                                    if (rtk->ssat[obs[i].sat - 1].fix[f])
                                        {
                                            rtk->ssat[obs[i].sat - 1].lock[f]++;
                                            rtk->ssat[obs[i].sat - 1].outc[f] = 0;
                                        }
                                }
                            rtk->sol.ns++;
                            continue;
                        }
                    rtk->ssat[obs[i].sat - 1].lock[0]++;
                    rtk->ssat[obs[i].sat - 1].outc[0] = 0;
                    rtk->ssat[obs[i].sat - 1].fix[0] = 4;
                    rtk->sol.ns++;
                }
            rtk->sol.stat = stat;

            for (i = 0; i < 3; i++)
                {
                    rtk->sol.rr[i] = rtk->x[i];
                    rtk->sol.qr[i] = static_cast<float>(rtk->P[i + i * rtk->nx]);
                }
            rtk->sol.qr[3] = static_cast<float>(rtk->P[1]);
            rtk->sol.qr[4] = static_cast<float>(rtk->P[2 + rtk->nx]);
            rtk->sol.qr[5] = static_cast<float>(rtk->P[2]);
            rtk->sol.dtr[0] = rtk->x[IC_PPP(0, opt)];
            rtk->sol.dtr[1] = rtk->x[IC_PPP(1, opt)] - rtk->x[IC_PPP(0, opt)];
            for (i = 0; i < n && i < MAXOBS; i++)
                {
                    const int ppp_snr_index = obs[i].SNR[0] <= obs[i].SNR[1] ? 0 : 1;
                    rtk->ssat[obs[i].sat - 1].snr[0] = obs[i].SNR[ppp_snr_index];
                    rtk->ssat[obs[i].sat - 1].code[0] = obs[i].code[ppp_snr_index];
                }
            for (i = 0; i < MAXSAT; i++)
                {
                    if (rtk->ssat[i].slip[0] & 3)
                        {
                            rtk->ssat[i].slipc[0]++;
                        }
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
