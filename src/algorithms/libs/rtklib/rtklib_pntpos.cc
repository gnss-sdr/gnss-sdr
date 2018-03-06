/*!
 * \file rtklib_pntpos.cc
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
 *----------------------------------------------------------------------------*/

#include "rtklib_pntpos.h"
#include "rtklib_ephemeris.h"
#include "rtklib_ionex.h"
#include "rtklib_sbas.h"
#include <fstream>
#include <string>

/* pseudorange measurement error variance ------------------------------------*/
double varerr(const prcopt_t *opt, double el, int sys)
{
    double fact, varr;
    fact = sys == SYS_GLO ? EFACT_GLO : (sys == SYS_SBS ? EFACT_SBS : EFACT_GPS);
    varr = std::pow(opt->err[0], 2.0) * (std::pow(opt->err[1], 2.0) + std::pow(opt->err[2], 2.0) / sin(el));
    if (opt->ionoopt == IONOOPT_IFLC) varr *= std::pow(2, 3.0); /* iono-free */
    return std::pow(fact, 2.0) * varr;
}


/* get tgd parameter (m) -----------------------------------------------------*/
double gettgd(int sat, const nav_t *nav)
{
    int i;
    for (i = 0; i < nav->n; i++)
        {
            if (nav->eph[i].sat != sat) continue;
            return SPEED_OF_LIGHT * nav->eph[i].tgd[0];
        }
    return 0.0;
}

/* get isc parameter (m) -----------------------------------------------------*/
double getiscl1(int sat, const nav_t *nav)
{
    for (int i = 0; i < nav->n; i++)
        {
            if (nav->eph[i].sat != sat) continue;
            return SPEED_OF_LIGHT * nav->eph[i].isc[0];
        }
    return 0.0;
}

double getiscl2(int sat, const nav_t *nav)
{
    for (int i = 0; i < nav->n; i++)
        {
            if (nav->eph[i].sat != sat) continue;
            return SPEED_OF_LIGHT * nav->eph[i].isc[1];
        }
    return 0.0;
}

double getiscl5i(int sat, const nav_t *nav)
{
    for (int i = 0; i < nav->n; i++)
        {
            if (nav->eph[i].sat != sat) continue;
            return SPEED_OF_LIGHT * nav->eph[i].isc[2];
        }
    return 0.0;
}

double getiscl5q(int sat, const nav_t *nav)
{
    for (int i = 0; i < nav->n; i++)
        {
            if (nav->eph[i].sat != sat) continue;
            return SPEED_OF_LIGHT * nav->eph[i].isc[3];
        }
    return 0.0;
}

/* psendorange with code bias correction -------------------------------------*/
double prange(const obsd_t *obs, const nav_t *nav, const double *azel,
    int iter, const prcopt_t *opt, double *var)
{
    const double* lam = nav->lam[obs->sat - 1];
    double PC = 0.0;
    double P1 = 0.0;
    double P2 = 0.0;
    double P1_P2 = 0.0;
    double P1_C1 = 0.0;
    double P2_C2 = 0.0;
    double ISCl1  = 0.0;
    double ISCl2  = 0.0;
    double ISCl5i = 0.0;
    double ISCl5q = 0.0;
    double gamma_ = 0.0;
    int i = 0;
    int j = 1;
    int sys = satsys(obs->sat, NULL);
    *var = 0.0;

    if(sys == SYS_NONE)
    {
        trace(4, "prange: satsys NULL\n");
        return 0.0;
    }


    /* L1-L2 for GPS/GLO/QZS, L1-L5 for GAL/SBS */
    if(sys == SYS_GAL or sys == SYS_SBS) {j = 2;}

    if(sys == SYS_GPS)
    {
        if(obs->code[1] != CODE_NONE) {j = 1;}
        else if(obs->code[2] != CODE_NONE) {j = 2;}
    }

    if(lam[i] == 0.0 or lam[j] == 0.0)
    {
        trace(4, "prange: NFREQ<2||lam[i]==0.0||lam[j]==0.0\n");
        printf("i: %d j:%d, lam[i]: %f lam[j] %f\n", i, j, lam[i], lam[j]);
        return 0.0;
    }

    /* test snr mask */
    if(iter > 0)
    {
        if (testsnr(0, i, azel[1], obs->SNR[i] * 0.25, &opt->snrmask))
        {
            trace(4, "snr mask: %s sat=%2d el=%.1f snr=%.1f\n",
                    time_str(obs->time, 0), obs->sat, azel[1] * R2D, obs->SNR[i] * 0.25);
            return 0.0;
        }
        if (opt->ionoopt == IONOOPT_IFLC)
        {
            if (testsnr(0, j, azel[1], obs->SNR[j] * 0.25, &opt->snrmask))
            {
                trace(4, "prange: testsnr error\n");
                return 0.0;
            }
        }
    }
    /* fL1^2 / fL2(orL5)^2 . See IS-GPS-200, p. 103 and Galileo ICD p. 48 */
    if(sys == SYS_GPS or sys == SYS_GAL)
    {
        gamma_ = std::pow(lam[j], 2.0) / std::pow(lam[i], 2.0);
    }
    P1 = obs->P[i];
    P2 = obs->P[j];
    P1_P2 = nav->cbias[obs->sat - 1][0];
    P1_C1 = nav->cbias[obs->sat - 1][1];
    P2_C2 = nav->cbias[obs->sat - 1][2];

    std::string d_dump_filename = "/home/aramos/dump_prange.dat";
    std::ofstream d_file;
    d_file.exceptions (std::ifstream::failbit | std::ifstream::badbit );
    d_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary | std::ios::app);
    double tmp_double = static_cast<double>(obs->sat);
    d_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
    d_file.write(reinterpret_cast<char*>(&P1), sizeof(double));
    d_file.write(reinterpret_cast<char*>(&P2), sizeof(double));
    d_file.write(reinterpret_cast<char*>(&P1_P2), sizeof(double));
    d_file.write(reinterpret_cast<char*>(&P1_C1), sizeof(double));
    d_file.write(reinterpret_cast<char*>(&P2_C2), sizeof(double));
    /* if no P1-P2 DCB, use TGD instead */
    if(P1_P2 == 0.0 and sys == SYS_GPS)
    {
        P1_P2 = gettgd(obs->sat, nav);
    }
    else if(P1_P2 == 0.0 and sys == SYS_GAL)
    {
        //TODO
    }

    if(sys == SYS_GPS)
    {
        ISCl1  = getiscl1(obs->sat, nav);
        ISCl2  = getiscl2(obs->sat, nav);
        ISCl5i = getiscl5i(obs->sat, nav);
        ISCl5q = getiscl5q(obs->sat, nav);
        d_file.write(reinterpret_cast<char*>(&ISCl1), sizeof(double));
        d_file.write(reinterpret_cast<char*>(&ISCl2), sizeof(double));
        d_file.write(reinterpret_cast<char*>(&ISCl5i), sizeof(double));
        d_file.write(reinterpret_cast<char*>(&ISCl5q), sizeof(double));
    }
    d_file.write(reinterpret_cast<char*>(&P1_P2), sizeof(double));

    //CHECK IF IT IS STILL NEEDED
    if(opt->ionoopt == IONOOPT_IFLC)
    { /* dual-frequency */

        if (P1 == 0.0 || P2 == 0.0) { return 0.0; }
        if (obs->code[i] == CODE_L1C) { P1 += P1_C1; } /* C1->P1 */
        if (obs->code[j] == CODE_L2C) { P2 += P2_C2; } /* C2->P2 */

        /* iono-free combination */
        PC = (gamma_ * P1 - P2) / (gamma_ - 1.0);
    }
    ////////////////////////////////////////////
    else
    { /* single-frequency */
        if(obs->code[i] == CODE_NONE and obs->code[j] == CODE_NONE) { return 0.0; }

        else if(obs->code[i] != CODE_NONE and obs->code[j] == CODE_NONE)
        {//CHECK!!
            P1 += P1_C1; /* C1->P1 */
            PC = P1 + P1_P2;
        }
        else if(obs->code[i] == CODE_NONE and obs->code[j] != CODE_NONE)
        {
            if(sys == SYS_GPS)
            {//CHECK!!
                P2 += P2_C2; /* C2->P2 */
                //PC = P2 - gamma_ * P1_P2 / (1.0 - gamma_);
                PC = P2 + P1_P2 - ISCl2;
            }
            else if(sys == SYS_GAL)
            {
                //TODO
            }
        }
        /* dual-frequency */
        else if(sys == SYS_GPS)
        {
            if(obs->code[j] == CODE_L2S) /* L1 + L2 */
            {
                PC = (P2 + ISCl2 - gamma_ * (P1 + ISCl1)) / (1.0 - gamma_) - P1_P2;
            }
            if(obs->code[j] == CODE_L5X) /* L1 + L5 */
            {

            }
        }
        else if(sys == SYS_GAL) /* E1 + E5a */
        {

        }
    }
    d_file.write(reinterpret_cast<char*>(&PC), sizeof(double));
    d_file.close();
    if(opt->sateph == EPHOPT_SBAS) { PC -= P1_C1; } /* sbas clock based C1 */
    *var = std::pow(ERR_CBIAS, 2.0);
    return PC;
}


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
 * return : status(1:ok,0:error)
 *-----------------------------------------------------------------------------*/
int ionocorr(gtime_t time, const nav_t *nav, int sat, const double *pos,
    const double *azel, int ionoopt, double *ion, double *var)
{
    trace(4, "ionocorr: time=%s opt=%d sat=%2d pos=%.3f %.3f azel=%.3f %.3f\n",
        time_str(time, 3), ionoopt, sat, pos[0] * R2D, pos[1] * R2D, azel[0] * R2D,
        azel[1] * R2D);

    /* broadcast model */
    if (ionoopt == IONOOPT_BRDC)
        {
            *ion = ionmodel(time, nav->ion_gps, pos, azel);
            *var = std::pow(*ion * ERR_BRDCI, 2.0);
            return 1;
        }
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
    /* qzss broadcast model */
    if (ionoopt == IONOOPT_QZS && norm_rtk(nav->ion_qzs, 8) > 0.0)
        {
            *ion = ionmodel(time, nav->ion_qzs, pos, azel);
            *var = std::pow(*ion * ERR_BRDCI, 2.0);
            return 1;
        }
    /* lex ionosphere model */
    //if (ionoopt == IONOOPT_LEX) {
    //    return lexioncorr(time, nav, pos, azel, ion, var);
    //}
    *ion = 0.0;
    *var = ionoopt == IONOOPT_OFF ? std::pow(ERR_ION, 2.0) : 0.0;
    return 1;
}


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
int tropcorr(gtime_t time, const nav_t *nav __attribute__((unused)), const double *pos,
    const double *azel, int tropopt, double *trp, double *var)
{
    trace(4, "tropcorr: time=%s opt=%d pos=%.3f %.3f azel=%.3f %.3f\n",
        time_str(time, 3), tropopt, pos[0] * R2D, pos[1] * R2D, azel[0] * R2D,
        azel[1] * R2D);

    /* saastamoinen model */
    if (tropopt == TROPOPT_SAAS || tropopt == TROPOPT_EST || tropopt == TROPOPT_ESTG)
        {
            *trp = tropmodel(time, pos, azel, REL_HUMI);
            *var = std::pow(ERR_SAAS / (sin(azel[1]) + 0.1), 2.0);
            return 1;
        }
    /* sbas troposphere model */
    if (tropopt == TROPOPT_SBAS)
        {
            *trp = sbstropcorr(time, pos, azel, var);
            return 1;
        }
    /* no correction */
    *trp = 0.0;
    *var = tropopt == TROPOPT_OFF ? std::pow(ERR_TROP, 2.0) : 0.0;
    return 1;
}


/* pseudorange residuals -----------------------------------------------------*/
int rescode(int iter, const obsd_t *obs, int n, const double *rs,
    const double *dts, const double *vare, const int *svh,
    const nav_t *nav, const double *x, const prcopt_t *opt,
    double *v, double *H, double *var, double *azel, int *vsat,
    double *resp, int *ns)
{
    double r, dion, dtrp, vmeas, vion, vtrp, rr[3], pos[3], dtr, e[3], P, lam_L1;
    int i, j, nv = 0, sys, mask[4] = {0};

    trace(3, "resprng : n=%d\n", n);

    for (i = 0; i < 3; i++) rr[i] = x[i];
    dtr = x[3];

    ecef2pos(rr, pos);

    for (i = *ns = 0; i < n && i < MAXOBS; i++)
        {
            vsat[i] = 0;
            azel[i * 2] = azel[1 + i * 2] = resp[i] = 0.0;

            if (!(sys = satsys(obs[i].sat, NULL))) continue;

            /* reject duplicated observation data */
            if (i < n - 1 && i < MAXOBS - 1 && obs[i].sat == obs[i + 1].sat)
                {
                    trace(2, "duplicated observation data %s sat=%2d\n",
                        time_str(obs[i].time, 3), obs[i].sat);
                    i++;
                    continue;
                }
            /* geometric distance/azimuth/elevation angle */
            if ((r = geodist(rs + i * 6, rr, e)) <= 0.0)
                {
                    trace(4, "geodist error\n");
                    continue;
                }
            double elaux = satazel(pos, e, azel + i * 2);
            if (elaux < opt->elmin)
                {
                    trace(4, "satazel error. el = %lf , elmin = %lf\n", elaux, opt->elmin);
                    continue;
                }
            /* psudorange with code bias correction */
            if ((P = prange(obs + i, nav, azel + i * 2, iter, opt, &vmeas)) == 0.0)
                {
                    trace(4, "prange error\n");
                    continue;
                }

            /* excluded satellite? */
            if (satexclude(obs[i].sat, svh[i], opt))
                {
                    trace(4, "satexclude error\n");
                    continue;
                }

            /* ionospheric corrections */
            if (!ionocorr(obs[i].time, nav, obs[i].sat, pos, azel + i * 2,
                    iter > 0 ? opt->ionoopt : IONOOPT_BRDC, &dion, &vion))
                {
                    trace(4, "ionocorr error\n");
                    continue;
                }

            /* GPS-L1 -> L1/B1 */
            if ((lam_L1 = nav->lam[obs[i].sat - 1][0]) > 0.0)
                {
                    dion *= std::pow(lam_L1 / lam_carr[0], 2.0);
                }
            /* tropospheric corrections */
            if (!tropcorr(obs[i].time, nav, pos, azel + i * 2,
                    iter > 0 ? opt->tropopt : TROPOPT_SAAS, &dtrp, &vtrp))
                {
                    trace(4, "tropocorr error\n");
                    continue;
                }
            /* pseudorange residual */
            v[nv] = P - (r + dtr - SPEED_OF_LIGHT * dts[i * 2] + dion + dtrp);

            /* design matrix */
            for (j = 0; j < NX; j++) H[j + nv * NX] = j < 3 ? -e[j] : (j == 3 ? 1.0 : 0.0);

            /* time system and receiver bias offset correction */
            if (sys == SYS_GLO)
                {
                    v[nv] -= x[4];
                    H[4 + nv * NX] = 1.0;
                    mask[1] = 1;
                }
            else if (sys == SYS_GAL)
                {
                    v[nv] -= x[5];
                    H[5 + nv * NX] = 1.0;
                    mask[2] = 1;
                }
            else if (sys == SYS_BDS)
                {
                    v[nv] -= x[6];
                    H[6 + nv * NX] = 1.0;
                    mask[3] = 1;
                }
            else
                mask[0] = 1;

            vsat[i] = 1;
            resp[i] = v[nv];
            (*ns)++;

            /* error variance */
            var[nv++] = varerr(opt, azel[1 + i * 2], sys) + vare[i] + vmeas + vion + vtrp;

            trace(4, "sat=%2d azel=%5.1f %4.1f res=%7.3f sig=%5.3f\n", obs[i].sat,
                azel[i * 2] * R2D, azel[1 + i * 2] * R2D, resp[i], sqrt(var[nv - 1]));
        }
    /* constraint to avoid rank-deficient */
    for (i = 0; i < 4; i++)
        {
            if (mask[i]) continue;
            v[nv] = 0.0;
            for (j = 0; j < NX; j++) H[j + nv * NX] = j == i + 3 ? 1.0 : 0.0;
            var[nv++] = 0.01;
        }
    return nv;
}


/* validate solution ---------------------------------------------------------*/
int valsol(const double *azel, const int *vsat, int n,
    const prcopt_t *opt, const double *v, int nv, int nx,
    char *msg)
{
    double azels[MAXOBS * 2] = {0};
    double dop[4], vv;
    int i, ns;

    trace(3, "valsol  : n=%d nv=%d\n", n, nv);

    /* chi-square validation of residuals */
    vv = dot(v, v, nv);
    if (nv > nx && vv > chisqr[nv - nx - 1])
        {
            sprintf(msg, "chi-square error nv=%d vv=%.1f cs=%.1f", nv, vv, chisqr[nv - nx - 1]);
            return 0;
        }
    /* large gdop check */
    for (i = ns = 0; i < n; i++)
        {
            if (!vsat[i]) continue;
            azels[ns * 2] = azel[i * 2];
            azels[1 + ns * 2] = azel[1 + i * 2];
            ns++;
        }
    dops(ns, azels, opt->elmin, dop);
    if (dop[0] <= 0.0 || dop[0] > opt->maxgdop)
        {
            sprintf(msg, "gdop error nv=%d gdop=%.1f", nv, dop[0]);
            return 0;
        }
    return 1;
}


/* estimate receiver position ------------------------------------------------*/
int estpos(const obsd_t *obs, int n, const double *rs, const double *dts,
    const double *vare, const int *svh, const nav_t *nav,
    const prcopt_t *opt, sol_t *sol, double *azel, int *vsat,
    double *resp, char *msg)
{
    double x[NX] = {0}, dx[NX], Q[NX * NX], *v, *H, *var, sig;
    int i, j, k, info, stat, nv, ns;

    trace(3, "estpos  : n=%d\n", n);

    v = mat(n + 4, 1);
    H = mat(NX, n + 4);
    var = mat(n + 4, 1);

    for (i = 0; i < 3; i++) x[i] = sol->rr[i];

    for (i = 0; i < MAXITR; i++)
        {
            /* pseudorange residuals */
            nv = rescode(i, obs, n, rs, dts, vare, svh, nav, x, opt, v, H, var, azel, vsat, resp, &ns);

            if (nv < NX)
                {
                    sprintf(msg, "lack of valid sats ns=%d", nv);
                    break;
                }
            /* weight by variance */
            for (j = 0; j < nv; j++)
                {
                    sig = sqrt(var[j]);
                    v[j] /= sig;
                    for (k = 0; k < NX; k++) H[k + j * NX] /= sig;
                }
            /* least square estimation */
            if ((info = lsq(H, v, NX, nv, dx, Q)))
                {
                    sprintf(msg, "lsq error info=%d", info);
                    break;
                }
            for (j = 0; j < NX; j++) x[j] += dx[j];

            if (norm_rtk(dx, NX) < 1e-4)
                {
                    sol->type = 0;
                    sol->time = timeadd(obs[0].time, -x[3] / SPEED_OF_LIGHT);
                    sol->dtr[0] = x[3] / SPEED_OF_LIGHT; /* receiver clock bias (s) */
                    sol->dtr[1] = x[4] / SPEED_OF_LIGHT; /* glo-gps time offset (s) */
                    sol->dtr[2] = x[5] / SPEED_OF_LIGHT; /* gal-gps time offset (s) */
                    sol->dtr[3] = x[6] / SPEED_OF_LIGHT; /* bds-gps time offset (s) */
                    for (j = 0; j < 6; j++) sol->rr[j] = j < 3 ? x[j] : 0.0;
                    for (j = 0; j < 3; j++) sol->qr[j] = (float)Q[j + j * NX];
                    sol->qr[3] = (float)Q[1];      /* cov xy */
                    sol->qr[4] = (float)Q[2 + NX]; /* cov yz */
                    sol->qr[5] = (float)Q[2];      /* cov zx */
                    sol->ns = (unsigned char)ns;
                    sol->age = sol->ratio = 0.0;

                    /* validate solution */
                    if ((stat = valsol(azel, vsat, n, opt, v, nv, NX, msg)))
                        {
                            sol->stat = opt->sateph == EPHOPT_SBAS ? SOLQ_SBAS : SOLQ_SINGLE;
                        }
                    free(v);
                    free(H);
                    free(var);

                    return stat;
                }
        }
    if (i >= MAXITR) sprintf(msg, "iteration divergent i=%d", i);

    free(v);
    free(H);
    free(var);

    return 0;
}


/* raim fde (failure detection and exclution) -------------------------------*/
int raim_fde(const obsd_t *obs, int n, const double *rs,
    const double *dts, const double *vare, const int *svh,
    const nav_t *nav, const prcopt_t *opt, sol_t *sol,
    double *azel, int *vsat, double *resp, char *msg)
{
    obsd_t *obs_e;
    sol_t sol_e = {{0, 0}, {}, {}, {}, '0', '0', '0', 0.0, 0.0, 0.0};
    char tstr[32], name[16], msg_e[128];
    double *rs_e, *dts_e, *vare_e, *azel_e, *resp_e, rms_e, rms = 100.0;
    int i, j, k, nvsat, stat = 0, *svh_e, *vsat_e, sat = 0;

    trace(3, "raim_fde: %s n=%2d\n", time_str(obs[0].time, 0), n);

    if (!(obs_e = (obsd_t *)malloc(sizeof(obsd_t) * n))) return 0;
    rs_e = mat(6, n);
    dts_e = mat(2, n);
    vare_e = mat(1, n);
    azel_e = zeros(2, n);
    svh_e = imat(1, n);
    vsat_e = imat(1, n);
    resp_e = mat(1, n);

    for (i = 0; i < n; i++)
        {
            /* satellite exclution */
            for (j = k = 0; j < n; j++)
                {
                    if (j == i) continue;
                    obs_e[k] = obs[j];
                    matcpy(rs_e + 6 * k, rs + 6 * j, 6, 1);
                    matcpy(dts_e + 2 * k, dts + 2 * j, 2, 1);
                    vare_e[k] = vare[j];
                    svh_e[k++] = svh[j];
                }
            /* estimate receiver position without a satellite */
            if (!estpos(obs_e, n - 1, rs_e, dts_e, vare_e, svh_e, nav, opt, &sol_e, azel_e,
                    vsat_e, resp_e, msg_e))
                {
                    trace(3, "raim_fde: exsat=%2d (%s)\n", obs[i].sat, msg);
                    continue;
                }
            for (j = nvsat = 0, rms_e = 0.0; j < n - 1; j++)
                {
                    if (!vsat_e[j]) continue;
                    rms_e += std::pow(resp_e[j], 2.0);
                    nvsat++;
                }
            if (nvsat < 5)
                {
                    trace(3, "raim_fde: exsat=%2d lack of satellites nvsat=%2d\n",
                        obs[i].sat, nvsat);
                    continue;
                }
            rms_e = sqrt(rms_e / nvsat);

            trace(3, "raim_fde: exsat=%2d rms=%8.3f\n", obs[i].sat, rms_e);

            if (rms_e > rms) continue;

            /* save result */
            for (j = k = 0; j < n; j++)
                {
                    if (j == i) continue;
                    matcpy(azel + 2 * j, azel_e + 2 * k, 2, 1);
                    vsat[j] = vsat_e[k];
                    resp[j] = resp_e[k++];
                }
            stat = 1;
            *sol = sol_e;
            sat = obs[i].sat;
            rms = rms_e;
            vsat[i] = 0;
            strcpy(msg, msg_e);
        }
    if (stat)
        {
            time2str(obs[0].time, tstr, 2);
            satno2id(sat, name);
            trace(2, "%s: %s excluded by raim\n", tstr + 11, name);
        }
    free(obs_e);
    free(rs_e);
    free(dts_e);
    free(vare_e);
    free(azel_e);
    free(svh_e);
    free(vsat_e);
    free(resp_e);

    return stat;
}


/* doppler residuals ---------------------------------------------------------*/
int resdop(const obsd_t *obs, int n, const double *rs, const double *dts,
    const nav_t *nav, const double *rr, const double *x,
    const double *azel, const int *vsat, double *v, double *H)
{
    double lam, rate, pos[3], E[9], a[3], e[3], vs[3], cosel;
    int i, j, nv = 0;

    trace(3, "resdop  : n=%d\n", n);

    ecef2pos(rr, pos);
    xyz2enu(pos, E);

    for (i = 0; i < n && i < MAXOBS; i++)
        {
            lam = nav->lam[obs[i].sat - 1][0];

            if (obs[i].D[0] == 0.0 || lam == 0.0 || !vsat[i] || norm_rtk(rs + 3 + i * 6, 3) <= 0.0)
                {
                    continue;
                }
            /* line-of-sight vector in ecef */
            cosel = cos(azel[1 + i * 2]);
            a[0] = sin(azel[i * 2]) * cosel;
            a[1] = cos(azel[i * 2]) * cosel;
            a[2] = sin(azel[1 + i * 2]);
            matmul("TN", 3, 1, 3, 1.0, E, a, 0.0, e);

            /* satellite velocity relative to receiver in ecef */
            for (j = 0; j < 3; j++) vs[j] = rs[j + 3 + i * 6] - x[j];

            /* range rate with earth rotation correction */
            rate = dot(vs, e, 3) + DEFAULT_OMEGA_EARTH_DOT / SPEED_OF_LIGHT * (rs[4 + i * 6] * rr[0] + rs[1 + i * 6] * x[0] - rs[3 + i * 6] * rr[1] - rs[i * 6] * x[1]);

            /* doppler residual */
            v[nv] = -lam * obs[i].D[0] - (rate + x[3] - SPEED_OF_LIGHT * dts[1 + i * 2]);

            /* design matrix */
            for (j = 0; j < 4; j++) H[j + nv * 4] = j < 3 ? -e[j] : 1.0;

            nv++;
        }
    return nv;
}


/* estimate receiver velocity ------------------------------------------------*/
void estvel(const obsd_t *obs, int n, const double *rs, const double *dts,
    const nav_t *nav, const prcopt_t *opt __attribute__((unused)), sol_t *sol,
    const double *azel, const int *vsat)
{
    double x[4] = {0}, dx[4], Q[16], *v, *H;
    int i, j, nv;

    trace(3, "estvel  : n=%d\n", n);

    v = mat(n, 1);
    H = mat(4, n);

    for (i = 0; i < MAXITR; i++)
        {
            /* doppler residuals */
            if ((nv = resdop(obs, n, rs, dts, nav, sol->rr, x, azel, vsat, v, H)) < 4)
                {
                    break;
                }
            /* least square estimation */
            if (lsq(H, v, 4, nv, dx, Q)) break;

            for (j = 0; j < 4; j++) x[j] += dx[j];

            if (norm_rtk(dx, 4) < 1e-6)
                {
                    for (i = 0; i < 3; i++) sol->rr[i + 3] = x[i];
                    break;
                }
        }
    free(v);
    free(H);
}


/* single-point positioning ----------------------------------------------------
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
 *-----------------------------------------------------------------------------*/
int pntpos(const obsd_t *obs, int n, const nav_t *nav,
    const prcopt_t *opt, sol_t *sol, double *azel, ssat_t *ssat,
    char *msg)
{
    prcopt_t opt_ = *opt;
    double *rs, *dts, *var, *azel_, *resp;
    int i, stat, vsat[MAXOBS] = {0}, svh[MAXOBS];

    sol->stat = SOLQ_NONE;

    if (n <= 0)
        {
            strcpy(msg, "no observation data");
            return 0;
        }

    trace(3, "pntpos  : tobs=%s n=%d\n", time_str(obs[0].time, 3), n);

    sol->time = obs[0].time;
    msg[0] = '\0';

    rs = mat(6, n);
    dts = mat(2, n);
    var = mat(1, n);
    azel_ = zeros(2, n);
    resp = mat(1, n);

    if (opt_.mode != PMODE_SINGLE)
        { /* for precise positioning */
#if 0
            opt_.sateph  = EPHOPT_BRDC;
#endif
            opt_.ionoopt = IONOOPT_BRDC;
            opt_.tropopt = TROPOPT_SAAS;
        }
    /* satellite positons, velocities and clocks */
    satposs(sol->time, obs, n, nav, opt_.sateph, rs, dts, var, svh);

    /* estimate receiver position with pseudorange */
    stat = estpos(obs, n, rs, dts, var, svh, nav, &opt_, sol, azel_, vsat, resp, msg);

    /* raim fde */
    if (!stat && n >= 6 && opt->posopt[4])
        {
            stat = raim_fde(obs, n, rs, dts, var, svh, nav, &opt_, sol, azel_, vsat, resp, msg);
        }
    /* estimate receiver velocity with doppler */
    if (stat) estvel(obs, n, rs, dts, nav, &opt_, sol, azel_, vsat);

    if (azel)
        {
            for (i = 0; i < n * 2; i++) azel[i] = azel_[i];
        }
    if (ssat)
        {
            for (i = 0; i < MAXSAT; i++)
                {
                    ssat[i].vs = 0;
                    ssat[i].azel[0] = ssat[i].azel[1] = 0.0;
                    ssat[i].resp[0] = ssat[i].resc[0] = 0.0;
                    ssat[i].snr[0] = 0;
                }
            for (i = 0; i < n; i++)
                {
                    ssat[obs[i].sat - 1].azel[0] = azel_[i * 2];
                    ssat[obs[i].sat - 1].azel[1] = azel_[1 + i * 2];
                    ssat[obs[i].sat - 1].snr[0] = obs[i].SNR[0];
                    if (!vsat[i]) continue;
                    ssat[obs[i].sat - 1].vs = 1;
                    ssat[obs[i].sat - 1].resp[0] = resp[i];
                }
        }
    free(rs);
    free(dts);
    free(var);
    free(azel_);
    free(resp);
    return stat;
}
