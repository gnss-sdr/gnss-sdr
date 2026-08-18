/*!
 * \file rtklib_rtkpos.cc
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
 * -----------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * -----------------------------------------------------------------------------
 */

#include "rtklib_rtkpos.h"
#include "rtklib_ephemeris.h"
#include "rtklib_lambda.h"
#include "rtklib_pntpos.h"
#include "rtklib_ppp.h"
#include "rtklib_tides.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

/* poly coeffs used to adjust the AR ratio threshold by # of sat pairs, derived
   by the demo5 RTKLIB fork by fitting to the example from:
   https://www.tudelft.nl/citg/over-faculteit/afdelingen/geoscience-remote-sensing/research/lambda/lambda */
static const double AR_POLY_COEFFS[3][5] = {
    {-1.94058448e-01, -7.79023476e+00, 1.24231120e+02, -4.03126050e+02, 3.50413202e+02},
    {6.42237302e-01, -8.39813962e+00, 2.92107285e+01, -2.37577308e+01, -1.14307128e+00},
    {-2.22600390e-02, 3.23169103e-01, -1.39837429e+00, 2.19282996e+00, -5.34583971e-02}};

/* number of consecutive innovation rejections after which a phase bias is
   considered stale and reinitialized (demo5) */
const unsigned int MAX_REJECTIONS_BEFORE_RESET = 2;

/* factor applied to the innovation threshold when one of the two phase biases
   of a double difference has just been initialized (demo5) */
const double INNOVATION_THRESHOLD_INIT_FACTOR = 10.0;

/* variance added to a carrier phase measurement whose half-cycle ambiguity is
   not resolved (m^2), which brings its weight down to decimeter level (demo5) */
const double HALF_CYCLE_VARIANCE_M2 = 0.01;

static int resamb_WLNL(rtk_t *rtk __attribute((unused)), const obsd_t *obs __attribute((unused)), const int *sat __attribute((unused)),
    const int *iu __attribute((unused)), const int *ir __attribute((unused)), int ns __attribute__((unused)), const nav_t *nav __attribute((unused)),
    const double *azel __attribute((unused))) { return 0; }
static int resamb_TCAR(rtk_t *rtk __attribute((unused)), const obsd_t *obs __attribute((unused)), const int *sat __attribute((unused)),
    const int *iu __attribute((unused)), const int *ir __attribute((unused)), int ns __attribute((unused)), const nav_t *nav __attribute((unused)),
    const double *azel __attribute((unused))) { return 0; }

/* global variables ----------------------------------------------------------*/
static int statlevel = 0;          /* rtk status output level (0:off) */
static FILE *fp_stat = nullptr;    /* rtk status file pointer */
static char file_stat[1024] = "";  /* rtk status file original path */
static gtime_t time_stat = {0, 0}; /* rtk status file time */

/* open solution status file ---------------------------------------------------
 * open solution status file and set output level
 * args   : char     *file   I   rtk status file
 *          int      level   I   rtk status level (0: off)
 * return : status (1:ok,0:error)
 * notes  : file can contain time keywords (%Y,%y,%m...) defined in reppath().
 *          The time to replace keywords is based on UTC of CPU time.
 * output : solution status file record format
 *
 *   $POS,week,tow,stat,posx,posy,posz,posxf,posyf,poszf
 *          week/tow : gps week no/time of week (s)
 *          stat     : solution status
 *          posx/posy/posz    : position x/y/z ecef (m) float
 *          posxf/posyf/poszf : position x/y/z ecef (m) fixed
 *
 *   $VELACC,week,tow,stat,vele,veln,velu,acce,accn,accu,velef,velnf,veluf,accef,accnf,accuf
 *          week/tow : gps week no/time of week (s)
 *          stat     : solution status
 *          vele/veln/velu    : velocity e/n/u (m/s) float
 *          acce/accn/accu    : acceleration e/n/u (m/s^2) float
 *          velef/velnf/veluf : velocity e/n/u (m/s) fixed
 *          accef/accnf/accuf : acceleration e/n/u (m/s^2) fixed
 *
 *   $CLK,week,tow,stat,clk1,clk2,clk3,clk4
 *          week/tow : gps week no/time of week (s)
 *          stat     : solution status
 *          clk1     : receiver clock bias GPS (ns)
 *          clk2     : receiver clock bias GLO-GPS (ns)
 *          clk3     : receiver clock bias GAL-GPS (ns)
 *          clk4     : receiver clock bias BDS-GPS (ns)
 *
 *   $ION,week,tow,stat,sat,az,el,ion,ion-fixed
 *          week/tow : gps week no/time of week (s)
 *          stat     : solution status
 *          sat      : satellite id
 *          az/el    : azimuth/elevation angle(deg)
 *          ion      : vertical ionospheric delay L1 (m) float
 *          ion-fixed: vertical ionospheric delay L1 (m) fixed
 *
 *   $TROP,week,tow,stat,rcv,ztd,ztdf
 *          week/tow : gps week no/time of week (s)
 *          stat     : solution status
 *          rcv      : receiver (1:rover,2:base station)
 *          ztd      : zenith total delay (m) float
 *          ztdf     : zenith total delay (m) fixed
 *
 *   $HWBIAS,week,tow,stat,frq,bias,biasf
 *          week/tow : gps week no/time of week (s)
 *          stat     : solution status
 *          frq      : frequency (1:L1,2:L2,...)
 *          bias     : h/w bias coefficient (m/MHz) float
 *          biasf    : h/w bias coefficient (m/MHz) fixed
 *
 *   $SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc
 *          week/tow : gps week no/time of week (s)
 *          sat/frq  : satellite id/frequency (1:L1,2:L2,...)
 *          az/el    : azimuth/elevation angle (deg)
 *          resp     : pseudorange residual (m)
 *          resc     : carrier-phase residual (m)
 *          vsat     : valid data flag (0:invalid,1:valid)
 *          snr      : signal strength (dbHz)
 *          fix      : ambiguity flag  (0:no data,1:float,2:fixed,3:hold,4:ppp)
 *          slip     : cycle-slip flag (bit1:slip,bit2:parity unknown)
 *          lock     : carrier-lock count
 *          outc     : data outage count
 *          slipc    : cycle-slip count
 *          rejc     : data reject (outlier) count
 *
 *-----------------------------------------------------------------------------*/
int rtkopenstat(const char *file, int level)
{
    gtime_t time = utc2gpst(timeget());
    std::string path;

    trace(3, "rtkopenstat: file=%s level=%d\n", file, level);

    if (level <= 0)
        {
            return 0;
        }

    reppath(file, path, time, "", "");

    if (!(fp_stat = fopen(path.data(), "we")))
        {
            trace(1, "rtkopenstat: file open error path=%s\n", path.data());
            return 0;
        }
    if (strlen(file) < 1025)
        {
            std::strncpy(file_stat, file, 1024);
            file_stat[1023] = '\0';
        }
    else
        {
            trace(1, "File name is too long");
        }
    time_stat = time;
    statlevel = level;
    return 1;
}


/* close solution status file --------------------------------------------------
 * close solution status file
 * args   : none
 * return : none
 *-----------------------------------------------------------------------------*/
void rtkclosestat()
{
    trace(3, "rtkclosestat:\n");

    if (fp_stat)
        {
            fclose(fp_stat);
        }
    fp_stat = nullptr;
    file_stat[0] = '\0';
    statlevel = 0;
}


/* write solution status to buffer -------------------------------------------*/
void rtkoutstat(rtk_t *rtk, char *buff __attribute__((unused)))
{
    ssat_t *ssat;
    double tow;
    double pos[3];
    double vel[3];
    double acc[3];
    double vela[3] = {0};
    double acca[3] = {0};
    double xa[3];
    int i;
    int j;
    int week;
    int est;
    int nfreq;
    int nf = NF_RTK(&rtk->opt);

    if (statlevel <= 0 || !fp_stat)
        {
            return;
        }

    trace(3, "outsolstat:\n");

    /* swap solution status file */
    swapsolstat();

    est = rtk->opt.mode >= PMODE_DGPS;
    nfreq = est ? nf : 1;
    tow = time2gpst(rtk->sol.time, &week);

    /* receiver position */
    if (est)
        {
            for (i = 0; i < 3; i++)
                {
                    xa[i] = i < rtk->na ? rtk->xa[i] : 0.0;
                }
            fprintf(fp_stat, "$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", week, tow,
                rtk->sol.stat, rtk->x[0], rtk->x[1], rtk->x[2], xa[0], xa[1], xa[2]);
        }
    else
        {
            fprintf(fp_stat, "$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", week, tow,
                rtk->sol.stat, rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2],
                0.0, 0.0, 0.0);
        }
    /* receiver velocity and acceleration */
    if (est && rtk->opt.dynamics)
        {
            ecef2pos(rtk->sol.rr, pos);
            ecef2enu(pos, rtk->x + 3, vel);
            ecef2enu(pos, rtk->x + 6, acc);
            if (rtk->na >= 6)
                {
                    ecef2enu(pos, rtk->xa + 3, vela);
                }
            if (rtk->na >= 9)
                {
                    ecef2enu(pos, rtk->xa + 6, acca);
                }
            fprintf(fp_stat, "$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n",
                week, tow, rtk->sol.stat, vel[0], vel[1], vel[2], acc[0], acc[1], acc[2],
                vela[0], vela[1], vela[2], acca[0], acca[1], acca[2]);
        }
    else
        {
            ecef2pos(rtk->sol.rr, pos);
            ecef2enu(pos, rtk->sol.rr + 3, vel);
            fprintf(fp_stat, "$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n",
                week, tow, rtk->sol.stat, vel[0], vel[1], vel[2],
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
    /* receiver clocks */
    fprintf(fp_stat, "$CLK,%d,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f\n",
        week, tow, rtk->sol.stat, 1, rtk->sol.dtr[0] * 1E9, rtk->sol.dtr[1] * 1E9,
        rtk->sol.dtr[2] * 1E9, rtk->sol.dtr[3] * 1E9);

    /* ionospheric parameters */
    if (est && rtk->opt.ionoopt == IONOOPT_EST)
        {
            for (i = 0; i < MAXSAT; i++)
                {
                    ssat = rtk->ssat + i;
                    if (!ssat->vs)
                        {
                            continue;
                        }
                    auto id = satno2id(i + 1);
                    j = II_RTK(i + 1, &rtk->opt);
                    xa[0] = j < rtk->na ? rtk->xa[j] : 0.0;
                    fprintf(fp_stat, "$ION,%d,%.3f,%d,%s,%.1f,%.1f,%.4f,%.4f\n", week, tow, rtk->sol.stat,
                        id.data(), ssat->azel[0] * R2D, ssat->azel[1] * R2D, rtk->x[j], xa[0]);
                }
        }
    /* tropospheric parameters */
    if (est && (rtk->opt.tropopt == TROPOPT_EST || rtk->opt.tropopt == TROPOPT_ESTG))
        {
            for (i = 0; i < 2; i++)
                {
                    j = IT_RTK(i, &rtk->opt);
                    xa[0] = j < rtk->na ? rtk->xa[j] : 0.0;
                    fprintf(fp_stat, "$TROP,%d,%.3f,%d,%d,%.4f,%.4f\n", week, tow, rtk->sol.stat,
                        i + 1, rtk->x[j], xa[0]);
                }
        }
    /* receiver h/w bias */
    if (est && rtk->opt.glomodear == 2)
        {
            for (i = 0; i < nfreq; i++)
                {
                    j = IL_RTK(i, &rtk->opt);
                    xa[0] = j < rtk->na ? rtk->xa[j] : 0.0;
                    fprintf(fp_stat, "$HWBIAS,%d,%.3f,%d,%d,%.4f,%.4f\n", week, tow, rtk->sol.stat,
                        i + 1, rtk->x[j], xa[0]);
                }
        }
    if (rtk->sol.stat == SOLQ_NONE || statlevel <= 1)
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
                    fprintf(fp_stat, "$SAT,%d,%.3f,%s,%d,%.1f,%.1f,%.4f,%.4f,%d,%.0f,%d,%d,%d,%d,%d,%d\n",
                        week, tow, id.data(), j + 1, ssat->azel[0] * R2D, ssat->azel[1] * R2D,
                        ssat->resp[j], ssat->resc[j], ssat->vsat[j], ssat->snr[j] * 0.25,
                        ssat->fix[j], ssat->slip[j] & 3, ssat->lock[j], ssat->outc[j],
                        ssat->slipc[j], ssat->rejc[j]);
                }
        }
}


/* swap solution status file -------------------------------------------------*/
void swapsolstat()
{
    gtime_t time = utc2gpst(timeget());
    std::string path;

    if (static_cast<int>(time2gpst(time, nullptr) / INT_SWAP_STAT) ==
        static_cast<int>(time2gpst(time_stat, nullptr) / INT_SWAP_STAT))
        {
            return;
        }
    time_stat = time;

    if (!reppath(file_stat, path, time, "", ""))
        {
            return;
        }
    if (fp_stat)
        {
            fclose(fp_stat);
        }

    if (!(fp_stat = fopen(path.data(), "we")))
        {
            trace(2, "swapsolstat: file open error path=%s\n", path.data());
            return;
        }
    trace(3, "swapsolstat: path=%s\n", path.data());
}


/* output solution status ----------------------------------------------------*/
void outsolstat(rtk_t *rtk)
{
    ssat_t *ssat;
    double tow;
    double pos[3];
    double vel[3];
    double acc[3];
    double vela[3] = {0};
    double acca[3] = {0};
    double xa[3];
    int i;
    int j;
    int week;
    int est;
    int nfreq;
    int nf = NF_RTK(&rtk->opt);

    if (statlevel <= 0 || !fp_stat)
        {
            return;
        }

    trace(3, "outsolstat:\n");

    /* swap solution status file */
    swapsolstat();

    est = rtk->opt.mode >= PMODE_DGPS;
    nfreq = est ? nf : 1;
    tow = time2gpst(rtk->sol.time, &week);

    /* receiver position */
    if (est)
        {
            for (i = 0; i < 3; i++)
                {
                    xa[i] = i < rtk->na ? rtk->xa[i] : 0.0;
                }
            fprintf(fp_stat, "$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", week, tow,
                rtk->sol.stat, rtk->x[0], rtk->x[1], rtk->x[2], xa[0], xa[1], xa[2]);
        }
    else
        {
            fprintf(fp_stat, "$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", week, tow,
                rtk->sol.stat, rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2],
                0.0, 0.0, 0.0);
        }
    /* receiver velocity and acceleration */
    if (est && rtk->opt.dynamics)
        {
            ecef2pos(rtk->sol.rr, pos);
            ecef2enu(pos, rtk->x + 3, vel);
            ecef2enu(pos, rtk->x + 6, acc);
            if (rtk->na >= 6)
                {
                    ecef2enu(pos, rtk->xa + 3, vela);
                }
            if (rtk->na >= 9)
                {
                    ecef2enu(pos, rtk->xa + 6, acca);
                }
            fprintf(fp_stat, "$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n",
                week, tow, rtk->sol.stat, vel[0], vel[1], vel[2], acc[0], acc[1], acc[2],
                vela[0], vela[1], vela[2], acca[0], acca[1], acca[2]);
        }
    else
        {
            ecef2pos(rtk->sol.rr, pos);
            ecef2enu(pos, rtk->sol.rr + 3, vel);
            fprintf(fp_stat, "$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n",
                week, tow, rtk->sol.stat, vel[0], vel[1], vel[2],
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
    /* receiver clocks */
    fprintf(fp_stat, "$CLK,%d,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f\n",
        week, tow, rtk->sol.stat, 1, rtk->sol.dtr[0] * 1E9, rtk->sol.dtr[1] * 1E9,
        rtk->sol.dtr[2] * 1E9, rtk->sol.dtr[3] * 1E9);

    /* ionospheric parameters */
    if (est && rtk->opt.ionoopt == IONOOPT_EST)
        {
            for (i = 0; i < MAXSAT; i++)
                {
                    ssat = rtk->ssat + i;
                    if (!ssat->vs)
                        {
                            continue;
                        }
                    auto id = satno2id(i + 1);
                    j = II_RTK(i + 1, &rtk->opt);
                    xa[0] = j < rtk->na ? rtk->xa[j] : 0.0;
                    fprintf(fp_stat, "$ION,%d,%.3f,%d,%s,%.1f,%.1f,%.4f,%.4f\n", week, tow, rtk->sol.stat,
                        id.data(), ssat->azel[0] * R2D, ssat->azel[1] * R2D, rtk->x[j], xa[0]);
                }
        }
    /* tropospheric parameters */
    if (est && (rtk->opt.tropopt == TROPOPT_EST || rtk->opt.tropopt == TROPOPT_ESTG))
        {
            for (i = 0; i < 2; i++)
                {
                    j = IT_RTK(i, &rtk->opt);
                    xa[0] = j < rtk->na ? rtk->xa[j] : 0.0;
                    fprintf(fp_stat, "$TROP,%d,%.3f,%d,%d,%.4f,%.4f\n", week, tow, rtk->sol.stat,
                        i + 1, rtk->x[j], xa[0]);
                }
        }
    /* receiver h/w bias */
    if (est && rtk->opt.glomodear == 2)
        {
            for (i = 0; i < nfreq; i++)
                {
                    j = IL_RTK(i, &rtk->opt);
                    xa[0] = j < rtk->na ? rtk->xa[j] : 0.0;
                    fprintf(fp_stat, "$HWBIAS,%d,%.3f,%d,%d,%.4f,%.4f\n", week, tow, rtk->sol.stat,
                        i + 1, rtk->x[j], xa[0]);
                }
        }
    if (rtk->sol.stat == SOLQ_NONE || statlevel <= 1)
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
                    fprintf(fp_stat, "$SAT,%d,%.3f,%s,%d,%.1f,%.1f,%.4f,%.4f,%d,%.0f,%d,%d,%d,%d,%d,%d\n",
                        week, tow, id.data(), j + 1, ssat->azel[0] * R2D, ssat->azel[1] * R2D,
                        ssat->resp[j], ssat->resc[j], ssat->vsat[j], ssat->snr[j] * 0.25,
                        ssat->fix[j], ssat->slip[j] & 3, ssat->lock[j], ssat->outc[j],
                        ssat->slipc[j], ssat->rejc[j]);
                }
        }
}


/* save error message --------------------------------------------------------*/
void errmsg(rtk_t *rtk, const char *format, ...)
{
    char buff[256];
    char tstr[32];
    int n;
    va_list ap;
    time2str(rtk->sol.time, tstr, 2);
    n = std::snprintf(buff, sizeof(buff), "%s: ", tstr + 11);
    va_start(ap, format);
    n += vsnprintf(buff + n, sizeof(buff) - n, format, ap);
    va_end(ap);
    n = n < MAXERRMSG - rtk->neb ? n : MAXERRMSG - rtk->neb;
    memcpy(rtk->errbuf + rtk->neb, buff, n);
    rtk->neb += n;
    trace(2, "%s", buff);
}


/* single-differenced observable ---------------------------------------------*/
double sdobs(const obsd_t *obs, int i, int j, int f)
{
    double pi = f < NFREQ ? obs[i].L[f] : obs[i].P[f - NFREQ];
    double pj = f < NFREQ ? obs[j].L[f] : obs[j].P[f - NFREQ];
    return pi == 0.0 || pj == 0.0 ? 0.0 : pi - pj;
}


/* single-differenced geometry-free linear combination of phase --------------*/
double gfobs_L1L2(const obsd_t *obs, int i, int j, const double *lam)
{
    double pi = sdobs(obs, i, j, 0) * lam[0];
    double pj = sdobs(obs, i, j, 1) * lam[1];
    return pi == 0.0 || pj == 0.0 ? 0.0 : pi - pj;
}


double gfobs_L1L5(const obsd_t *obs, int i, int j, const double *lam)
{
    double pi = sdobs(obs, i, j, 0) * lam[0];
    double pj = sdobs(obs, i, j, 2) * lam[2];
    return pi == 0.0 || pj == 0.0 ? 0.0 : pi - pj;
}


/* single-differenced measurement error variance -----------------------------*/
double varerr(int sat __attribute((unused)), int sys, double el,
    double snr_rover, double snr_base, double bl, double dt, int f,
    const prcopt_t *opt, const obsd_t *obs)
{
    double a;
    double b;
    double c = opt->err[3] * bl / 1e4;
    double d = SPEED_OF_LIGHT_M_S * opt->sclkstab * dt;
    double e;
    double var;
    double snr_max = opt->err[5];
    double fact = 1.0;
    double sinel = sin(el);
    int i = sys == SYS_GLO ? 1 : (sys == SYS_GAL ? 2 : 0);
    int nf = NF_RTK(opt);
    int frq = f % nf;
    int code = f < nf ? 0 : 1;

    /* extended error model */
    if (code && opt->exterr.ena[0])
        { /* code */
            a = opt->exterr.cerr[i][frq * 2];
            b = opt->exterr.cerr[i][1 + frq * 2];
            if (sys == SYS_SBS)
                {
                    a *= EFACT_SBS;
                    b *= EFACT_SBS;
                }
        }
    else if (!code && opt->exterr.ena[1])
        { /* phase */
            a = opt->exterr.perr[i][frq * 2];
            b = opt->exterr.perr[i][1 + frq * 2];
            if (sys == SYS_SBS)
                {
                    a *= EFACT_SBS;
                    b *= EFACT_SBS;
                }
        }
    else
        { /* normal error model (demo5 form) */
            if (code)
                {
                    /* increase variance for pseudoranges */
                    fact = opt->eratio[frq];
                }
            else if (opt->eratio[0] > 0.0)
                {
                    /* adjust variance between freqs */
                    fact = opt->eratio[frq] / opt->eratio[0];
                }
            if (fact <= 0.0)
                {
                    fact = opt->eratio[0];
                }
            /* adjust variance for constellation */
            switch (sys)
                {
                case SYS_GPS:
                    fact *= EFACT_GPS;
                    break;
                case SYS_GLO:
                    fact *= EFACT_GLO;
                    break;
                case SYS_GAL:
                    fact *= EFACT_GAL;
                    break;
                case SYS_SBS:
                    fact *= EFACT_SBS;
                    break;
                case SYS_QZS:
                    fact *= EFACT_QZS;
                    break;
                case SYS_BDS:
                    fact *= EFACT_BDS;
                    break;
                default:
                    fact *= EFACT_GPS;
                    break;
                }
            a = fact * opt->err[1];
            b = fact * opt->err[2];
        }
    var = 2.0 * (a * a + b * b / sinel / sinel + c * c) + d * d;
    if (opt->err[6] > 0.0)
        { /* add SNR-dependent term */
            e = fact * opt->err[6];
            var += e * e * (std::pow(10.0, 0.1 * std::max(snr_max - snr_rover, 0.0)) + std::pow(10.0, 0.1 * std::max(snr_max - snr_base, 0.0)));
        }
    if (opt->err[7] > 0.0)
        { /* add receiver-reported stdev term */
            if (code)
                {
                    var += std::pow(opt->err[7] * obs->Pstd[frq], 2.0);
                }
            else
                {
                    var += std::pow(opt->err[7] * obs->Lstd[frq] * 0.2, 2.0);
                }
        }
    var *= opt->ionoopt == IONOOPT_IFLC ? 9.0 : 1.0;
    return var;
}


/* baseline length -----------------------------------------------------------*/
double baseline(const double *ru, const double *rb, double *dr)
{
    int i;
    for (i = 0; i < 3; i++)
        {
            dr[i] = ru[i] - rb[i];
        }
    return norm_rtk(dr, 3);
}


/* initialize state and covariance -------------------------------------------*/
void initx_rtk(rtk_t *rtk, double xi, double var, int i)
{
    int j;
    rtk->x[i] = xi;
    for (j = 0; j < rtk->nx; j++)
        {
            rtk->P[i + j * rtk->nx] = rtk->P[j + i * rtk->nx] = i == j ? var : 0.0;
        }
}


/* select common satellites between rover and reference station --------------*/
int selsat(const obsd_t *obs, const double *azel, int nu, int nr,
    const prcopt_t *opt, int *sat, int *iu, int *ir)
{
    int i;
    int j;
    int k = 0;

    trace(3, "selsat  : nu=%d nr=%d\n", nu, nr);

    for (i = 0, j = nu; i < nu && j < nu + nr; i++, j++)
        {
            if (obs[i].sat < obs[j].sat)
                {
                    j--;
                }
            else if (obs[i].sat > obs[j].sat)
                {
                    i--;
                }
            else if (azel[1 + j * 2] >= opt->elmin)
                { /* elevation at base station */
                    sat[k] = obs[i].sat;
                    iu[k] = i;
                    ir[k++] = j;
                    trace(4, "(%2d) sat=%3d iu=%2d ir=%2d\n", k - 1, obs[i].sat, i, j);
                }
        }
    return k;
}


/* temporal update of position/velocity/acceleration -------------------------*/
void udpos(rtk_t *rtk, double tt)
{
    double *F;
    double *FP;
    double *xp;
    double pos[3];
    double Q[9] = {0};
    double Qv[9];
    double var = 0.0;
    int i;
    int j;

    trace(3, "udpos   : tt=%.3f\n", tt);

    /* fixed mode */
    if (rtk->opt.mode == PMODE_FIXED)
        {
            for (i = 0; i < 3; i++)
                {
                    initx_rtk(rtk, rtk->opt.ru[i], 1E-8, i);
                }
            return;
        }
    /* initialize position for the first epoch, or whenever the state does not
       hold a plausible position on the Earth, which would otherwise keep the
       filter from converging (demo5) */
    if (norm_rtk(rtk->x, 3) <= RE_WGS84 / 2.0)
        {
            for (i = 0; i < 3; i++)
                {
                    initx_rtk(rtk, rtk->sol.rr[i], VAR_POS, i);
                }
            if (rtk->opt.dynamics)
                {
                    for (i = 3; i < 6; i++)
                        {
                            initx_rtk(rtk, rtk->sol.rr[i], VAR_VEL, i);
                        }
                    for (i = 6; i < 9; i++)
                        {
                            initx_rtk(rtk, 1E-6, VAR_ACC, i);
                        }
                }
        }
    /* static mode */
    if (rtk->opt.mode == PMODE_STATIC)
        {
            return;
        }

    /* kinmatic mode without dynamics */
    if (!rtk->opt.dynamics)
        {
            for (i = 0; i < 3; i++)
                {
                    initx_rtk(rtk, rtk->sol.rr[i], VAR_POS, i);
                }
            return;
        }
    /* check variance of estimated position */
    for (i = 0; i < 3; i++)
        {
            var += rtk->P[i + i * rtk->nx];
        }
    var /= 3.0;

    if (var > VAR_POS)
        {
            /* reset position with large variance */
            for (i = 0; i < 3; i++)
                {
                    initx_rtk(rtk, rtk->sol.rr[i], VAR_POS, i);
                }
            for (i = 3; i < 6; i++)
                {
                    initx_rtk(rtk, rtk->sol.rr[i], VAR_VEL, i);
                }
            for (i = 6; i < 9; i++)
                {
                    initx_rtk(rtk, 1E-6, VAR_ACC, i);
                }
            trace(2, "reset rtk position due to large variance: var=%.3f\n", var);
            return;
        }
    /* state transition of position/velocity/acceleration */
    F = eye(rtk->nx);
    FP = mat(rtk->nx, rtk->nx);
    xp = mat(rtk->nx, 1);

    for (i = 0; i < 6; i++)
        {
            F[i + (i + 3) * rtk->nx] = tt;
        }
    /* let the acceleration states drive the position only once the filter has
       converged: while the position variance is still large the acceleration
       estimates are meaningless and would inject noise into the position
       (demo5, which reuses its AR position-variance gate for this) */
    if (rtk->opt.armaxposvar <= 0.0 || var < rtk->opt.armaxposvar)
        {
            for (i = 0; i < 3; i++)
                {
                    F[i + (i + 6) * rtk->nx] = (tt >= 0.0 ? 1.0 : -1.0) * tt * tt / 2.0;
                }
        }
    else
        {
            trace(3, "udpos : position variance too high for the acceleration term: %.4f\n", var);
        }
    /* x=F*x, P=F*P*F+Q */
    matmul("NN", rtk->nx, 1, rtk->nx, 1.0, F, rtk->x, 0.0, xp);
    matcpy(rtk->x, xp, rtk->nx, 1);
    matmul("NN", rtk->nx, rtk->nx, rtk->nx, 1.0, F, rtk->P, 0.0, FP);
    matmul("NT", rtk->nx, rtk->nx, rtk->nx, 1.0, FP, F, 0.0, rtk->P);

    /* process noise added to only acceleration, scaled by the elapsed time */
    Q[0] = Q[4] = std::pow(rtk->opt.prn[3], 2.0) * fabs(tt);
    Q[8] = std::pow(rtk->opt.prn[4], 2.0) * fabs(tt);
    ecef2pos(rtk->x, pos);
    covecef(pos, Q, Qv);
    for (i = 0; i < 3; i++)
        {
            for (j = 0; j < 3; j++)
                {
                    rtk->P[i + 6 + (j + 6) * rtk->nx] += Qv[i + j * 3];
                }
        }
    free(F);
    free(FP);
    free(xp);
}


/* temporal update of ionospheric parameters ---------------------------------*/
void udion(rtk_t *rtk, double tt, double bl, const int *sat, int ns)
{
    double el;
    double fact;
    int i;
    int j;

    trace(3, "udion   : tt=%.1f bl=%.0f ns=%d\n", tt, bl, ns);

    for (i = 1; i <= MAXSAT; i++)
        {
            j = II_RTK(i, &rtk->opt);
            if (rtk->x[j] != 0.0 &&
                rtk->ssat[i - 1].outc[0] > GAP_RESION && rtk->ssat[i - 1].outc[1] > GAP_RESION)
                {
                    rtk->x[j] = 0.0;
                }
        }
    for (i = 0; i < ns; i++)
        {
            j = II_RTK(sat[i], &rtk->opt);

            if (rtk->x[j] == 0.0)
                {
                    initx_rtk(rtk, 1E-6, std::pow(rtk->opt.std[1] * bl / 1e4, 2.0), j);
                }
            else
                {
                    /* elevation dependent factor of process noise */
                    el = rtk->ssat[sat[i] - 1].azel[1];
                    fact = cos(el);
                    rtk->P[j + j * rtk->nx] += std::pow(rtk->opt.prn[1] * bl / 1e4 * fact, 2.0) * fabs(tt);
                }
        }
}


/* temporal update of tropospheric parameters --------------------------------*/
void udtrop(rtk_t *rtk, double tt, double bl __attribute((unused)))
{
    int i;
    int j;
    int k;

    trace(3, "udtrop  : tt=%.1f\n", tt);

    for (i = 0; i < 2; i++)
        {
            j = IT_RTK(i, &rtk->opt);

            if (rtk->x[j] == 0.0)
                {
                    initx_rtk(rtk, INIT_ZWD, std::pow(rtk->opt.std[2], 2.0), j); /* initial zwd */

                    if (rtk->opt.tropopt >= TROPOPT_ESTG)
                        {
                            for (k = 0; k < 2; k++)
                                {
                                    initx_rtk(rtk, 1e-6, VAR_GRA, ++j);
                                }
                        }
                }
            else
                {
                    rtk->P[j + j * rtk->nx] += std::pow(rtk->opt.prn[2], 2.0) * fabs(tt);

                    if (rtk->opt.tropopt >= TROPOPT_ESTG)
                        {
                            for (k = 0; k < 2; k++)
                                {
                                    rtk->P[++j * (1 + rtk->nx)] += std::pow(rtk->opt.prn[2] * 0.3, 2.0) * fabs(rtk->tt);
                                }
                        }
                }
        }
}


/* temporal update of receiver h/w biases ------------------------------------*/
void udrcvbias(rtk_t *rtk, double tt)
{
    int i;
    int j;

    trace(3, "udrcvbias: tt=%.1f\n", tt);

    for (i = 0; i < NFREQGLO; i++)
        {
            j = IL_RTK(i, &rtk->opt);

            if (rtk->x[j] == 0.0)
                {
                    initx_rtk(rtk, 1E-6, VAR_HWBIAS, j);
                }
            /* hold to fixed solution */
            else if (rtk->nfix >= rtk->opt.minfix && rtk->sol.ratio > rtk->opt.thresar[0])
                {
                    initx_rtk(rtk, rtk->xa[j], rtk->Pa[j + j * rtk->na], j);
                }
            else
                {
                    rtk->P[j + j * rtk->nx] += std::pow(PRN_HWBIAS, 2.0) * fabs(tt);
                }
        }
}


/* detect cycle slip by LLI --------------------------------------------------*/
void detslp_ll(rtk_t *rtk, const obsd_t *obs, int i, int rcv)
{
    unsigned int slip;
    unsigned int LLI;
    int f;
    int sat = obs[i].sat;

    trace(3, "detslp_ll: i=%d rcv=%d\n", i, rcv);

    for (f = 0; f < rtk->opt.nf; f++)
        {
            if (obs[i].L[f] == 0.0)
                {
                    continue;
                }

            /* restore previous LLI */
            if (rcv == 1)
                {
                    LLI = getbitu(&rtk->ssat[sat - 1].slip[f], 0, 2); /* rover */
                }
            else
                {
                    LLI = getbitu(&rtk->ssat[sat - 1].slip[f], 2, 2); /* base  */
                }

            /* detect slip by cycle slip flag in LLI */
            if (rtk->tt >= 0.0)
                { /* forward */
                    if (obs[i].LLI[f] & 1)
                        {
                            errmsg(rtk, "slip detected forward (sat=%2d rcv=%d F=%d LLI=%x)\n",
                                sat, rcv, f + 1, obs[i].LLI[f]);
                        }
                    slip = obs[i].LLI[f];
                }
            else
                { /* backward */
                    if (LLI & 1)
                        {
                            errmsg(rtk, "slip detected backward (sat=%2d rcv=%d F=%d LLI=%x)\n",
                                sat, rcv, f + 1, LLI);
                        }
                    slip = LLI;
                }
            /* detect slip by parity unknown flag transition in LLI */
            if (((LLI & 2) && !(obs[i].LLI[f] & 2)) || (!(LLI & 2) && (obs[i].LLI[f] & 2)))
                {
                    errmsg(rtk, "slip detected half-cyc (sat=%2d rcv=%d F=%d LLI=%x->%x)\n",
                        sat, rcv, f + 1, LLI, obs[i].LLI[f]);
                    slip |= 1;
                }
            /* save current LLI */
            if (rcv == 1)
                {
                    setbitu(&rtk->ssat[sat - 1].slip[f], 0, 2, obs[i].LLI[f]);
                }
            else
                {
                    setbitu(&rtk->ssat[sat - 1].slip[f], 2, 2, obs[i].LLI[f]);
                }

            /* save slip and half-cycle valid flag */
            rtk->ssat[sat - 1].slip[f] |= static_cast<unsigned char>(slip);
            rtk->ssat[sat - 1].half[f] = (obs[i].LLI[f] & 2) ? 0 : 1;
        }
}


/* detect cycle slip by L1-L2 geometry free phase jump -----------------------*/
void detslp_gf_L1L2(rtk_t *rtk, const obsd_t *obs, int i, int j,
    const nav_t *nav)
{
    int sat = obs[i].sat;
    double g0;
    double g1;

    trace(3, "detslp_gf_L1L2: i=%d j=%d\n", i, j);

    if (rtk->opt.nf <= 1 || (g1 = gfobs_L1L2(obs, i, j, nav->lam[sat - 1])) == 0.0)
        {
            return;
        }

    g0 = rtk->ssat[sat - 1].gf;
    rtk->ssat[sat - 1].gf = g1;

    if (g0 != 0.0 && fabs(g1 - g0) > rtk->opt.thresslip)
        {
            rtk->ssat[sat - 1].slip[0] |= 1;
            rtk->ssat[sat - 1].slip[1] |= 1;

            errmsg(rtk, "slip detected (sat=%2d GF_L1_L2=%.3f %.3f)\n", sat, g0, g1);
        }
}


/* detect cycle slip by L1-L5 geometry free phase jump -----------------------*/
void detslp_gf_L1L5(rtk_t *rtk, const obsd_t *obs, int i, int j,
    const nav_t *nav)
{
    int sat = obs[i].sat;
    double g0;
    double g1;

    trace(3, "detslp_gf_L1L5: i=%d j=%d\n", i, j);

    if (rtk->opt.nf <= 2 || (g1 = gfobs_L1L5(obs, i, j, nav->lam[sat - 1])) == 0.0)
        {
            return;
        }

    g0 = rtk->ssat[sat - 1].gf2;
    rtk->ssat[sat - 1].gf2 = g1;

    if (g0 != 0.0 && fabs(g1 - g0) > rtk->opt.thresslip)
        {
            rtk->ssat[sat - 1].slip[0] |= 1;
            rtk->ssat[sat - 1].slip[2] |= 1;

            errmsg(rtk, "slip detected (sat=%2d GF_L1_L5=%.3f %.3f)\n", sat, g0, g1);
        }
}


/* detect cycle slip by doppler and phase difference -------------------------*/
void detslp_dop(rtk_t *rtk, const obsd_t *obs, const int *ix, int ns, int rcv,
    const nav_t *nav)
{
    /* the common receiver clock error is removed as the median of the
       phase-doppler differences over all valid satellites/frequencies,
       so this detector is immune to the receiver clock jumps that led to
       its deactivation in vanilla RTKLIB (see demo5 fork) */
    double dopdif[MAXSAT][NFREQ];
    double dt[MAXSAT][NFREQ];
    double doplist[MAXSAT * NFREQ];
    double dph;
    double dpt;
    double lam;
    double med_dop;
    double tmp;
    int i;
    int j;
    int ii;
    int f;
    int sat;
    int ndop = 0;
    const int nf = rtk->opt.nf;

    trace(4, "detslp_dop: rcv=%d\n", rcv);

    if (rtk->opt.thresdop <= 0.0)
        {
            return; /* test disabled */
        }

    /* calculate phase-doppler differences in m/s for all sats and freqs */
    for (i = 0; i < ns; i++)
        {
            ii = ix[i];
            sat = obs[ii].sat;
            for (f = 0; f < nf; f++)
                {
                    dopdif[i][f] = 0.0;
                    dt[i][f] = 0.0;
                    if (obs[ii].L[f] == 0.0 || obs[ii].D[f] == 0.0 ||
                        rtk->ssat[sat - 1].ph[rcv - 1][f] == 0.0)
                        {
                            continue;
                        }
                    dt[i][f] = timediff(obs[ii].time, rtk->ssat[sat - 1].pt[rcv - 1][f]);
                    if (fabs(dt[i][f]) < DTTOL)
                        {
                            continue;
                        }
                    lam = nav->lam[sat - 1][f];
                    if (lam <= 0.0)
                        {
                            continue;
                        }

                    /* phase rate and doppler in cycles/s, converted to m/s */
                    dph = (obs[ii].L[f] - rtk->ssat[sat - 1].ph[rcv - 1][f]) / dt[i][f];
                    dpt = -obs[ii].D[f];
                    dopdif[i][f] = (dph - dpt) * lam;
                    doplist[ndop++] = dopdif[i][f];
                }
        }
    if (ndop == 0)
        {
            return;
        }

    /* insertion sort to extract the median common range-rate error,
       most likely due to the receiver clock error */
    for (i = 1; i < ndop; i++)
        {
            tmp = doplist[i];
            for (j = i; j > 0 && doplist[j - 1] > tmp; j--)
                {
                    doplist[j] = doplist[j - 1];
                }
            doplist[j] = tmp;
        }
    med_dop = (ndop % 2 != 0) ? doplist[ndop / 2] : (doplist[ndop / 2 - 1] + doplist[ndop / 2]) / 2.0;

    /* set slip if corrected phase-doppler rate exceeds threshold */
    for (i = 0; i < ns; i++)
        {
            sat = obs[ix[i]].sat;
            for (f = 0; f < nf; f++)
                {
                    if (dt[i][f] == 0.0)
                        {
                            continue;
                        }
                    if (fabs(dopdif[i][f] - med_dop) > rtk->opt.thresdop)
                        {
                            rtk->ssat[sat - 1].slip[f] |= 1;
                            errmsg(rtk, "slip detected doppler (sat=%2d rcv=%d dL%d=%.3f off=%.3f tt=%.2f)\n",
                                sat, rcv, f + 1, dopdif[i][f] - med_dop, med_dop, dt[i][f]);
                        }
                }
        }
}


/* temporal update of phase biases -------------------------------------------*/
void udbias(rtk_t *rtk, double tt, const obsd_t *obs, const int *sat,
    const int *iu, const int *ir, int ns, const nav_t *nav)
{
    double cp;
    double pr;
    double cp1;
    double cp2;
    double pr1;
    double pr2;
    double *bias;
    double offset;
    double lami;
    double lam1;
    double lam2;
    double C1;
    double C2;
    int i;
    int j;
    int f;
    int slip;
    unsigned int rejc;
    int reset;
    int nf = NF_RTK(&rtk->opt);

    trace(3, "udbias  : tt=%.1f ns=%d\n", tt, ns);

    /* clear cycle slips */
    for (i = 0; i < ns; i++)
        {
            for (f = 0; f < rtk->opt.nf; f++)
                {
                    rtk->ssat[sat[i] - 1].slip[f] &= 0xFC;
                }
        }

    /* detect cycle slip by doppler and phase difference */
    detslp_dop(rtk, obs, iu, ns, 1, nav);
    if (rtk->opt.mode == PMODE_MOVEB)
        { /* only check base if moving baseline */
            detslp_dop(rtk, obs, ir, ns, 2, nav);
        }

    for (i = 0; i < ns; i++)
        {
            /* detect cycle slip by LLI */
            detslp_ll(rtk, obs, iu[i], 1);
            detslp_ll(rtk, obs, ir[i], 2);

            /* detect cycle slip by geometry-free phase jump */
            detslp_gf_L1L2(rtk, obs, iu[i], ir[i], nav);
            detslp_gf_L1L5(rtk, obs, iu[i], ir[i], nav);

            /* update half-cycle valid flag */
            for (f = 0; f < nf; f++)
                {
                    rtk->ssat[sat[i] - 1].half[f] =
                        !((obs[iu[i]].LLI[f] & 2) || (obs[ir[i]].LLI[f] & 2));
                }
        }
    for (f = 0; f < nf; f++)
        {
            /* reset phase-bias if instantaneous AR or expire obs outage counter */
            for (i = 1; i <= MAXSAT; i++)
                {
                    reset = ++rtk->ssat[i - 1].outc[f] > static_cast<unsigned int>(rtk->opt.maxout);

                    if (rtk->opt.modear == ARMODE_INST && rtk->x[IB_RTK(i, f, &rtk->opt)] != 0.0)
                        {
                            initx_rtk(rtk, 0.0, 0.0, IB_RTK(i, f, &rtk->opt));
                        }
                    else if (reset && rtk->x[IB_RTK(i, f, &rtk->opt)] != 0.0)
                        {
                            initx_rtk(rtk, 0.0, 0.0, IB_RTK(i, f, &rtk->opt));
                            trace(3, "udbias : obs outage counter overflow (sat=%3d L%d n=%d)\n",
                                i, f + 1, rtk->ssat[i - 1].outc[f]);
                        }
                    if (rtk->opt.modear != ARMODE_INST && reset)
                        {
                            rtk->ssat[i - 1].lock[f] = -rtk->opt.minlock;
                        }
                }
            /* reset phase-bias if detecting cycle slip or repeated outliers */
            for (i = 0; i < ns; i++)
                {
                    j = IB_RTK(sat[i], f, &rtk->opt);
                    rtk->P[j + j * rtk->nx] += rtk->opt.prn[0] * rtk->opt.prn[0] * fabs(tt);
                    slip = rtk->ssat[sat[i] - 1].slip[f];
                    rejc = rtk->ssat[sat[i] - 1].rejc[f];
                    if (rtk->opt.ionoopt == IONOOPT_IFLC)
                        {
                            slip |= rtk->ssat[sat[i] - 1].slip[1];
                        }
                    /* a bias that keeps producing outliers is stale: reinitialize it
                       instead of rejecting its measurements forever (demo5) */
                    if (rtk->opt.modear == ARMODE_INST ||
                        (!(slip & 1) && rejc < MAX_REJECTIONS_BEFORE_RESET))
                        {
                            continue;
                        }
                    rtk->x[j] = 0.0;
                    rtk->ssat[sat[i] - 1].rejc[f] = 0;
                    rtk->ssat[sat[i] - 1].lock[f] = -rtk->opt.minlock;
                }
            bias = zeros(ns, 1);

            /* estimate approximate phase-bias by phase - code */
            for (i = j = 0, offset = 0.0; i < ns; i++)
                {
                    if (rtk->opt.ionoopt != IONOOPT_IFLC)
                        {
                            cp = sdobs(obs, iu[i], ir[i], f); /* cycle */
                            pr = sdobs(obs, iu[i], ir[i], f + NFREQ);
                            lami = nav->lam[sat[i] - 1][f];
                            if (cp == 0.0 || pr == 0.0 || lami <= 0.0)
                                {
                                    continue;
                                }

                            bias[i] = cp - pr / lami;
                        }
                    else
                        {
                            cp1 = sdobs(obs, iu[i], ir[i], 0);
                            cp2 = sdobs(obs, iu[i], ir[i], 1);
                            pr1 = sdobs(obs, iu[i], ir[i], NFREQ);
                            pr2 = sdobs(obs, iu[i], ir[i], NFREQ + 1);
                            lam1 = nav->lam[sat[i] - 1][0];
                            lam2 = nav->lam[sat[i] - 1][1];
                            if (cp1 == 0.0 || cp2 == 0.0 || pr1 == 0.0 || pr2 == 0.0 || lam1 <= 0.0 || lam2 <= 0.0)
                                {
                                    continue;
                                }

                            C1 = std::pow(lam2, 2.0) / (std::pow(lam2, 2.0) - std::pow(lam1, 2.0));
                            C2 = -std::pow(lam1, 2.0) / (std::pow(lam2, 2.0) - std::pow(lam1, 2.0));
                            bias[i] = (C1 * lam1 * cp1 + C2 * lam2 * cp2) - (C1 * pr1 + C2 * pr2);
                        }
                    if (rtk->x[IB_RTK(sat[i], f, &rtk->opt)] != 0.0)
                        {
                            offset += bias[i] - rtk->x[IB_RTK(sat[i], f, &rtk->opt)];
                            j++;
                        }
                }
            /* correct phase-bias offset to enssure phase-code coherency */
            if (j > 0)
                {
                    for (i = 1; i <= MAXSAT; i++)
                        {
                            if (rtk->x[IB_RTK(i, f, &rtk->opt)] != 0.0)
                                {
                                    rtk->x[IB_RTK(i, f, &rtk->opt)] += offset / j;
                                }
                        }
                }
            /* set initial states of phase-bias */
            for (i = 0; i < ns; i++)
                {
                    if (bias[i] == 0.0 || rtk->x[IB_RTK(sat[i], f, &rtk->opt)] != 0.0)
                        {
                            continue;
                        }
                    initx_rtk(rtk, bias[i], std::pow(rtk->opt.std[0], 2.0), IB_RTK(sat[i], f, &rtk->opt));
                }
            free(bias);
        }
}


/* temporal update of states --------------------------------------------------*/
void udstate(rtk_t *rtk, const obsd_t *obs, const int *sat,
    const int *iu, const int *ir, int ns, const nav_t *nav)
{
    double tt = fabs(rtk->tt);
    double bl = 0.0;
    double dr[3];

    trace(3, "udstate : ns=%d\n", ns);

    /* temporal update of position/velocity/acceleration */
    udpos(rtk, tt);

    /* temporal update of ionospheric parameters */
    if (rtk->opt.ionoopt >= IONOOPT_EST)
        {
            bl = baseline(rtk->x, rtk->rb, dr);
            udion(rtk, tt, bl, sat, ns);
        }
    /* temporal update of tropospheric parameters */
    if (rtk->opt.tropopt >= TROPOPT_EST)
        {
            udtrop(rtk, tt, bl);
        }
    /* temporal update of eceiver h/w bias */
    if (rtk->opt.glomodear == 2 && (rtk->opt.navsys & SYS_GLO))
        {
            udrcvbias(rtk, tt);
        }
    /* temporal update of phase-bias */
    if (rtk->opt.mode > PMODE_DGPS)
        {
            udbias(rtk, tt, obs, sat, iu, ir, ns, nav);
        }
}


/* undifferenced phase/code residual for satellite ---------------------------*/
void zdres_sat(int base, double r, const obsd_t *obs, const nav_t *nav,
    const double *azel, const double *dant,
    const prcopt_t *opt, double *y)
{
    const double *lam = nav->lam[obs->sat - 1];
    double f1;
    double f2;
    double C1;
    double C2;
    double dant_if;
    int i;
    int nf = NF_RTK(opt);

    if (opt->ionoopt == IONOOPT_IFLC)
        { /* iono-free linear combination */
            if (lam[0] == 0.0 || lam[1] == 0.0)
                {
                    return;
                }

            if (testsnr(base, 0, azel[1], obs->SNR[0] * 0.25, &opt->snrmask) ||
                testsnr(base, 1, azel[1], obs->SNR[1] * 0.25, &opt->snrmask))
                {
                    return;
                }

            f1 = SPEED_OF_LIGHT_M_S / lam[0];
            f2 = SPEED_OF_LIGHT_M_S / lam[1];
            C1 = std::pow(f1, 2.0) / (std::pow(f1, 2.0) - std::pow(f2, 2.0));
            C2 = -std::pow(f2, 2.0) / (std::pow(f1, 2.0) - std::pow(f2, 2.0));
            dant_if = C1 * dant[0] + C2 * dant[1];

            if (obs->L[0] != 0.0 && obs->L[1] != 0.0)
                {
                    y[0] = C1 * obs->L[0] * lam[0] + C2 * obs->L[1] * lam[1] - r - dant_if;
                }
            if (obs->P[0] != 0.0 && obs->P[1] != 0.0)
                {
                    y[1] = C1 * obs->P[0] + C2 * obs->P[1] - r - dant_if;
                }
        }
    else
        {
            for (i = 0; i < nf; i++)
                {
                    if (lam[i] == 0.0)
                        {
                            continue;
                        }

                    /* check snr mask */
                    if (testsnr(base, i, azel[1], obs->SNR[i] * 0.25, &opt->snrmask))
                        {
                            continue;
                        }
                    /* residuals = observable - pseudorange */
                    if (obs->L[i] != 0.0)
                        {
                            y[i] = obs->L[i] * lam[i] - r - dant[i];
                        }
                    if (obs->P[i] != 0.0)
                        {
                            y[i + nf] = obs->P[i] - r - dant[i];
                        }
                }
        }
}


/* undifferenced phase/code residuals ----------------------------------------*/
int zdres(int base, const obsd_t *obs, int n, const double *rs,
    const double *dts, const int *svh, const nav_t *nav,
    const double *rr, const prcopt_t *opt, int index, double *y,
    double *e, double *azel)
{
    double r;
    double rr_[3];
    double pos[3];
    double dant[NFREQ] = {0};
    double disp[3];
    double zhd;
    double zazel[] = {0.0, 90.0 * D2R};
    int i;
    int nf = NF_RTK(opt);

    trace(3, "zdres   : n=%d\n", n);

    for (i = 0; i < n * nf * 2; i++)
        {
            y[i] = 0.0;
        }

    if (norm_rtk(rr, 3) <= 0.0)
        {
            return 0; /* no receiver position */
        }

    for (i = 0; i < 3; i++)
        {
            rr_[i] = rr[i];
        }

    /* earth tide correction */
    if (opt->tidecorr)
        {
            tidedisp(gpst2utc(obs[0].time), rr_, opt->tidecorr, &nav->erp,
                opt->odisp[base], disp);
            for (i = 0; i < 3; i++)
                {
                    rr_[i] += disp[i];
                }
        }
    ecef2pos(rr_, pos);

    for (i = 0; i < n; i++)
        {
            /* compute geometric-range and azimuth/elevation angle */
            if ((r = geodist(rs + i * 6, rr_, e + i * 3)) <= 0.0)
                {
                    continue;
                }
            if (satazel(pos, e + i * 3, azel + i * 2) < opt->elmin)
                {
                    continue;
                }

            /* excluded satellite? */
            if (satexclude(obs[i].sat, svh[i], opt))
                {
                    continue;
                }

            /* satellite clock-bias */
            r += -SPEED_OF_LIGHT_M_S * dts[i * 2];

            /* troposphere delay model (hydrostatic) */
            zhd = tropmodel(obs[0].time, pos, zazel, 0.0);
            r += tropmapf(obs[i].time, pos, azel + i * 2, nullptr) * zhd;

            /* receiver antenna phase center correction */
            antmodel(opt->pcvr + index, opt->antdel[index], azel + i * 2, opt->posopt[1],
                dant);

            /* undifferenced phase/code residual for satellite */
            zdres_sat(base, r, obs + i, nav, azel + i * 2, dant, opt, y + i * nf * 2);
        }
    trace(4, "rr_=%.3f %.3f %.3f\n", rr_[0], rr_[1], rr_[2]);
    trace(4, "pos=%.9f %.9f %.3f\n", pos[0] * R2D, pos[1] * R2D, pos[2]);
    for (i = 0; i < n; i++)
        {
            trace(4, "sat=%2d %13.3f %13.3f %13.3f %13.10f %6.1f %5.1f\n",
                obs[i].sat, rs[i * 6], rs[1 + i * 6], rs[2 + i * 6], dts[i * 2], azel[i * 2] * R2D,
                azel[1 + i * 2] * R2D);
        }
    trace(4, "y=\n");
    tracemat(4, y, nf * 2, n, 13, 3);

    return 1;
}


/* test valid observation data -----------------------------------------------*/
int validobs(int i, int j, int f, int nf, const double *y)
{
    /* if no phase observable, psudorange is also unusable */
    return y[f + i * nf * 2] != 0.0 && y[f + j * nf * 2] != 0.0 &&
           (f < nf || (y[f - nf + i * nf * 2] != 0.0 && y[f - nf + j * nf * 2] != 0.0));
}


/* double-differenced measurement error covariance ---------------------------*/
void ddcov(const int *nb, int n, const double *Ri, const double *Rj,
    int nv, double *R)
{
    int i;
    int j;
    int k = 0;
    int b;

    trace(3, "ddcov   : n=%d\n", n);

    for (i = 0; i < nv * nv; i++)
        {
            R[i] = 0.0;
        }
    for (b = 0; b < n; k += nb[b++])
        {
            for (i = 0; i < nb[b]; i++)
                {
                    for (j = 0; j < nb[b]; j++)
                        {
                            R[k + i + (k + j) * nv] = Ri[k + i] + (i == j ? Rj[k + i] : 0.0);
                        }
                }
        }
    trace(5, "R=\n");
    tracemat(5, R, nv, nv, 8, 6);
}


/* baseline length constraint ------------------------------------------------*/
int constbl(rtk_t *rtk, const double *x, const double *P, double *v,
    double *H, double *Ri, double *Rj, int index)
{
    const double thres = 0.1; /* threshold for nonliearity (v.2.3.0) */
    double xb[3];
    double b[3];
    double bb;
    double var = 0.0;
    int i;

    trace(3, "constbl : \n");

    /* no constraint */
    if (rtk->opt.baseline[0] <= 0.0)
        {
            return 0;
        }

    /* time-adjusted baseline vector and length */
    for (i = 0; i < 3; i++)
        {
            xb[i] = rtk->rb[i] + rtk->rb[i + 3] * rtk->sol.age;
            b[i] = x[i] - xb[i];
        }
    bb = norm_rtk(b, 3);

    /* approximate variance of solution */
    if (P)
        {
            for (i = 0; i < 3; i++)
                {
                    var += P[i + i * rtk->nx];
                }
            var /= 3.0;
        }
    /* check nonlinearity */
    if (var > thres * thres * bb * bb)
        {
            trace(3, "constbl : equation nonlinear (bb=%.3f var=%.3f)\n", bb, var);
            return 0;
        }
    /* constraint to baseline length */
    v[index] = rtk->opt.baseline[0] - bb;
    if (H)
        {
            for (i = 0; i < 3; i++)
                {
                    H[i + index * rtk->nx] = b[i] / bb;
                }
        }
    Ri[index] = 0.0;
    Rj[index] = std::pow(rtk->opt.baseline[1], 2.0);

    trace(4, "baseline len   v=%13.3f R=%8.6f %8.6f\n", v[index], Ri[index], Rj[index]);

    return 1;
}


/* precise tropspheric model -------------------------------------------------*/
double prectrop(gtime_t time, const double *pos, int r,
    const double *azel, const prcopt_t *opt, const double *x,
    double *dtdx)
{
    double m_w = 0.0;
    double cotz;
    double grad_n;
    double grad_e;
    int i = IT_RTK(r, opt);

    /* wet mapping function */
    tropmapf(time, pos, azel, &m_w);

    if (opt->tropopt >= TROPOPT_ESTG && azel[1] > 0.0)
        {
            /* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
            cotz = 1.0 / tan(azel[1]);
            grad_n = m_w * cotz * cos(azel[0]);
            grad_e = m_w * cotz * sin(azel[0]);
            m_w += grad_n * x[i + 1] + grad_e * x[i + 2];
            dtdx[1] = grad_n * x[i];
            dtdx[2] = grad_e * x[i];
        }
    else
        {
            dtdx[1] = dtdx[2] = 0.0;
        }
    dtdx[0] = m_w;
    return m_w * x[i];
}


/* glonass inter-channel bias correction -------------------------------------*/
double gloicbcorr(int sat1 __attribute((unused)), int sat2 __attribute((unused)), const prcopt_t *opt, double lam1,
    double lam2, int f)
{
    double dfreq;

    if (f >= NFREQGLO || f >= opt->nf || !opt->exterr.ena[2])
        {
            return 0.0;
        }

    dfreq = (SPEED_OF_LIGHT_M_S / lam1 - SPEED_OF_LIGHT_M_S / lam2) / (f == 0 ? DFRQ1_GLO : DFRQ2_GLO);

    return opt->exterr.gloicb[f] * 0.01 * dfreq; /* (m) */
}


/* test navi system (m=0:gps/sbs,1:glo,2:gal,3:bds,4:qzs) --------------------*/
int test_sys(int sys, int m)
{
    switch (sys)
        {
        case SYS_GPS:
            return m == 0;
        case SYS_QZS:
            return m == 4;
        case SYS_SBS:
            return m == 0;
        case SYS_GLO:
            return m == 1;
        case SYS_GAL:
            return m == 2;
        case SYS_BDS:
            return m == 3;
        }
    return 0;
}


/* double-differenced phase/code residuals -----------------------------------*/
int ddres(rtk_t *rtk, const obsd_t *obs, const nav_t *nav, double dt, const double *x,
    const double *P, const int *sat, double *y, const double *e,
    double *azel, const int *iu, const int *ir, int ns, double *v,
    double *H, double *R, int *vflg)
{
    prcopt_t *opt = &rtk->opt;
    double bl;
    double dr[3];
    double posu[3];
    double posr[3];
    double didxi = 0.0;
    double didxj = 0.0;
    double *im;
    double *tropr;
    double *tropu;
    double *dtdxr;
    double *dtdxu;
    double *Ri;
    double *Rj;
    double lami;
    double lamj;
    double fi;
    double fj;
    double df;
    double threshadj;
    double *Hi = nullptr;
    int i;
    int j;
    int k;
    int m;
    int f;
    int ff;
    int frq;
    int nv = 0;
    int nb[NFREQ * 5 * 2 + 2] = {0}; /* sized for the 5 constellation groups of test_sys() */
    int b = 0;
    int sysi;
    int sysj;
    int nf = NF_RTK(opt);
    double minvar;
    double refvar;

    trace(3, "ddres   : dt=%.1f nx=%d ns=%d\n", dt, rtk->nx, ns);

    bl = baseline(x, rtk->rb, dr);
    ecef2pos(x, posu);
    ecef2pos(rtk->rb, posr);

    Ri = mat(ns * nf * 2 + 2, 1);
    Rj = mat(ns * nf * 2 + 2, 1);
    im = mat(ns, 1);
    tropu = mat(ns, 1);
    tropr = mat(ns, 1);
    dtdxu = mat(ns, 3);
    dtdxr = mat(ns, 3);

    for (i = 0; i < MAXSAT; i++)
        {
            for (j = 0; j < NFREQ; j++)
                {
                    rtk->ssat[i].resp[j] = rtk->ssat[i].resc[j] = 0.0;
                }
        }
    /* compute factors of ionospheric and tropospheric delay */
    for (i = 0; i < ns; i++)
        {
            if (opt->ionoopt >= IONOOPT_EST)
                {
                    im[i] = (ionmapf(posu, azel + iu[i] * 2) + ionmapf(posr, azel + ir[i] * 2)) / 2.0;
                }
            if (opt->tropopt >= TROPOPT_EST)
                {
                    tropu[i] = prectrop(rtk->sol.time, posu, 0, azel + iu[i] * 2, opt, x, dtdxu + i * 3);
                    tropr[i] = prectrop(rtk->sol.time, posr, 1, azel + ir[i] * 2, opt, x, dtdxr + i * 3);
                }
        }
    for (m = 0; m < 5; m++)
        { /* m=0:gps/sbs, 1:glo, 2:gal, 3:bds, 4:qzs */
            for (f = opt->mode > PMODE_DGPS ? 0 : nf; f < nf * 2; f++)
                {
                    frq = f % nf;

                    /* choose the minimum-variance satellite without a slip as
                       the reference (demo5: better than highest elevation in
                       urban conditions, where SNR is the better quality proxy) */
                    minvar = 0.0;
                    for (i = -1, j = 0; j < ns; j++)
                        {
                            sysj = rtk->ssat[sat[j] - 1].sys;
                            if (!test_sys(sysj, m) || sysj == SYS_SBS)
                                {
                                    continue;
                                }
                            if (!validobs(iu[j], ir[j], f, nf, y))
                                {
                                    continue;
                                }
                            if (rtk->ssat[sat[j] - 1].slip[frq] & 1)
                                {
                                    continue;
                                }
                            if (rtk->ssat[sat[j] - 1].lock[frq] < 0)
                                {
                                    continue;
                                }
                            refvar = varerr(sat[j], sysj, azel[1 + iu[j] * 2],
                                rtk->ssat[sat[j] - 1].snr_rover[frq], rtk->ssat[sat[j] - 1].snr_base[frq],
                                bl, dt, f, opt, &obs[iu[j]]);
                            if (i < 0 || refvar < minvar)
                                {
                                    i = j;
                                    minvar = refvar;
                                }
                        }
                    /* fall back to a slipped satellite only if no clean reference exists */
                    if (i < 0)
                        {
                            minvar = 0.0;
                            for (j = 0; j < ns; j++)
                                {
                                    sysj = rtk->ssat[sat[j] - 1].sys;
                                    if (!test_sys(sysj, m) || sysj == SYS_SBS)
                                        {
                                            continue;
                                        }
                                    if (!validobs(iu[j], ir[j], f, nf, y))
                                        {
                                            continue;
                                        }
                                    refvar = varerr(sat[j], sysj, azel[1 + iu[j] * 2],
                                        rtk->ssat[sat[j] - 1].snr_rover[frq], rtk->ssat[sat[j] - 1].snr_base[frq],
                                        bl, dt, f, opt, &obs[iu[j]]);
                                    if (i < 0 || refvar < minvar)
                                        {
                                            i = j;
                                            minvar = refvar;
                                        }
                                }
                        }
                    if (i < 0)
                        {
                            continue;
                        }

                    /* make double difference */
                    for (j = 0; j < ns; j++)
                        {
                            if (i == j)
                                {
                                    continue;
                                }
                            sysi = rtk->ssat[sat[i] - 1].sys;
                            sysj = rtk->ssat[sat[j] - 1].sys;
                            if (!test_sys(sysj, m))
                                {
                                    continue;
                                }
                            if (!validobs(iu[j], ir[j], f, nf, y))
                                {
                                    continue;
                                }

                            ff = f % nf;
                            lami = nav->lam[sat[i] - 1][ff];
                            lamj = nav->lam[sat[j] - 1][ff];
                            if (lami <= 0.0 || lamj <= 0.0)
                                {
                                    continue;
                                }
                            if (H)
                                {
                                    Hi = H + nv * rtk->nx;
                                    for (k = 0; k < rtk->nx; k++)
                                        {
                                            Hi[k] = 0.0;
                                        }
                                }
                            /* double-differenced residual */
                            v[nv] = (y[f + iu[i] * nf * 2] - y[f + ir[i] * nf * 2]) -
                                    (y[f + iu[j] * nf * 2] - y[f + ir[j] * nf * 2]);

                            /* partial derivatives by rover position */
                            if (H)
                                {
                                    for (k = 0; k < 3; k++)
                                        {
                                            Hi[k] = -e[k + iu[i] * 3] + e[k + iu[j] * 3];
                                        }
                                }
                            /* double-differenced ionospheric delay term */
                            if (opt->ionoopt == IONOOPT_EST)
                                {
                                    fi = lami / LAM_CARR[0];
                                    fj = lamj / LAM_CARR[0];
                                    didxi = (f < nf ? -1.0 : 1.0) * fi * fi * im[i];
                                    didxj = (f < nf ? -1.0 : 1.0) * fj * fj * im[j];
                                    v[nv] -= didxi * x[II_RTK(sat[i], opt)] - didxj * x[II_RTK(sat[j], opt)];
                                    if (H)
                                        {
                                            Hi[II_RTK(sat[i], opt)] = didxi;
                                            Hi[II_RTK(sat[j], opt)] = -didxj;
                                        }
                                }
                            /* double-differenced tropospheric delay term */
                            if (opt->tropopt == TROPOPT_EST || opt->tropopt == TROPOPT_ESTG)
                                {
                                    v[nv] -= (tropu[i] - tropu[j]) - (tropr[i] - tropr[j]);
                                    for (k = 0; k < (opt->tropopt < TROPOPT_ESTG ? 1 : 3); k++)
                                        {
                                            if (!H)
                                                {
                                                    continue;
                                                }
                                            Hi[IT_RTK(0, opt) + k] = (dtdxu[k + i * 3] - dtdxu[k + j * 3]);
                                            Hi[IT_RTK(1, opt) + k] = -(dtdxr[k + i * 3] - dtdxr[k + j * 3]);
                                        }
                                }
                            /* double-differenced phase-bias term */
                            if (f < nf)
                                {
                                    if (opt->ionoopt != IONOOPT_IFLC)
                                        {
                                            v[nv] -= lami * x[IB_RTK(sat[i], f, opt)] - lamj * x[IB_RTK(sat[j], f, opt)];
                                            if (H)
                                                {
                                                    Hi[IB_RTK(sat[i], f, opt)] = lami;
                                                    Hi[IB_RTK(sat[j], f, opt)] = -lamj;
                                                }
                                        }
                                    else
                                        {
                                            v[nv] -= x[IB_RTK(sat[i], f, opt)] - x[IB_RTK(sat[j], f, opt)];
                                            if (H)
                                                {
                                                    Hi[IB_RTK(sat[i], f, opt)] = 1.0;
                                                    Hi[IB_RTK(sat[j], f, opt)] = -1.0;
                                                }
                                        }
                                }
                            /* glonass receiver h/w bias term */
                            if (rtk->opt.glomodear == 2 && sysi == SYS_GLO && sysj == SYS_GLO && ff < NFREQGLO)
                                {
                                    df = (SPEED_OF_LIGHT_M_S / lami - SPEED_OF_LIGHT_M_S / lamj) / 1E6; /* freq-difference (MHz) */
                                    v[nv] -= df * x[IL_RTK(ff, opt)];
                                    if (H)
                                        {
                                            Hi[IL_RTK(ff, opt)] = df;
                                        }
                                }
                            /* glonass interchannel bias correction */
                            else if (sysi == SYS_GLO && sysj == SYS_GLO)
                                {
                                    v[nv] -= gloicbcorr(sat[i], sat[j], &rtk->opt, lami, lamj, f);
                                }
                            if (f < nf)
                                {
                                    rtk->ssat[sat[j] - 1].resc[f] = v[nv];
                                }
                            else
                                {
                                    rtk->ssat[sat[j] - 1].resp[f - nf] = v[nv];
                                }

                            /* open up the innovation threshold if one of the two phase
                               biases has just been initialized, otherwise the freshly
                               reset bias is rejected again and never recovers (demo5) */
                            threshadj = 1.0;
                            if (P != nullptr && opt->mode > PMODE_DGPS)
                                {
                                    const int ii = IB_RTK(sat[i], frq, opt);
                                    const int jj = IB_RTK(sat[j], frq, opt);
                                    if (P[ii + rtk->nx * ii] == std::pow(opt->std[0], 2.0) ||
                                        P[jj + rtk->nx * jj] == std::pow(opt->std[0], 2.0))
                                        {
                                            threshadj = INNOVATION_THRESHOLD_INIT_FACTOR;
                                        }
                                }
                            /* test innovation, with separate thresholds for phase and code */
                            if (opt->maxinno[f < nf ? 0 : 1] > 0.0 &&
                                fabs(v[nv]) > opt->maxinno[f < nf ? 0 : 1] * threshadj)
                                {
                                    /* charge the outlier only to the non-reference
                                       satellite (demo5): the reference is part of every
                                       pair and would otherwise reach the rejc-based bias
                                       reset from rejections it did not cause */
                                    rtk->ssat[sat[j] - 1].vsat[frq] = 0;
                                    rtk->ssat[sat[j] - 1].rejc[frq]++;
                                    errmsg(rtk, "outlier rejected (sat=%3d-%3d %s%d v=%.3f)\n",
                                        sat[i], sat[j], f < nf ? "L" : "P", f % nf + 1, v[nv]);
                                    continue;
                                }
                            /* single-differenced measurement error variances */
                            Ri[nv] = varerr(sat[i], sysi, azel[1 + iu[i] * 2],
                                rtk->ssat[sat[i] - 1].snr_rover[frq], rtk->ssat[sat[i] - 1].snr_base[frq],
                                bl, dt, f, opt, &obs[iu[i]]);
                            Rj[nv] = varerr(sat[j], sysj, azel[1 + iu[j] * 2],
                                rtk->ssat[sat[j] - 1].snr_rover[frq], rtk->ssat[sat[j] - 1].snr_base[frq],
                                bl, dt, f, opt, &obs[iu[j]]);

                            /* a carrier phase with an unresolved half-cycle ambiguity is
                               only worth its half wavelength, so deweight it instead of
                               feeding it to the filter with full carrier phase accuracy
                               (demo5, extended here to the base observations, which carry
                               the flag too) */
                            if (f < nf)
                                {
                                    if ((obs[iu[i]].LLI[frq] & 2) || (obs[ir[i]].LLI[frq] & 2))
                                        {
                                            Ri[nv] += HALF_CYCLE_VARIANCE_M2;
                                        }
                                    if ((obs[iu[j]].LLI[frq] & 2) || (obs[ir[j]].LLI[frq] & 2))
                                        {
                                            Rj[nv] += HALF_CYCLE_VARIANCE_M2;
                                        }
                                }
                            /* set valid data flags */
                            if (opt->mode > PMODE_DGPS)
                                {
                                    if (f < nf)
                                        {
                                            rtk->ssat[sat[i] - 1].vsat[f] = rtk->ssat[sat[j] - 1].vsat[f] = 1;
                                        }
                                }
                            else
                                {
                                    rtk->ssat[sat[i] - 1].vsat[f - nf] = rtk->ssat[sat[j] - 1].vsat[f - nf] = 1;
                                }
                            trace(4, "sat=%3d-%3d %s%d v=%13.3f R=%8.6f %8.6f\n", sat[i],
                                sat[j], f < nf ? "L" : "P", f % nf + 1, v[nv], Ri[nv], Rj[nv]);

                            vflg[nv++] = (sat[i] << 16) | (sat[j] << 8) | ((f < nf ? 0 : 1) << 4) | (f % nf);
                            nb[b]++;
                        }
#if 0 /* residuals referenced to reference satellite (2.4.2 p11) */
                    /* restore single-differenced residuals assuming sum equal zero */
                    if (f < nf)
                        {
                            for (j = 0, s = 0.0; j < MAXSAT; j++) s += rtk->ssat[j].resc[f];
                            s /= nb[b] + 1;
                            for (j = 0; j < MAXSAT; j++)
                                {
                                    if (j == sat[i] - 1 || rtk->ssat[j].resc[f] != 0.0) rtk->ssat[j].resc[f] -= s;
                                }
                        }
                    else
                        {
                            for (j = 0, s = 0.0; j < MAXSAT; j++) s += rtk->ssat[j].resp[f - nf];
                            s /= nb[b] + 1;
                            for (j = 0; j < MAXSAT; j++)
                                {
                                    if (j == sat[i] - 1 || rtk->ssat[j].resp[f - nf] != 0.0)
                                        rtk->ssat[j].resp[f - nf] -= s;
                                }
                        }
#endif
                    b++;
                }
        }
    /* end of system loop */

    /* baseline length constraint for moving baseline */
    if (opt->mode == PMODE_MOVEB && constbl(rtk, x, P, v, H, Ri, Rj, nv))
        {
            vflg[nv++] = 3 << 4;
            nb[b++]++;
        }
    if (H)
        {
            trace(5, "H=\n");
            tracemat(5, H, rtk->nx, nv, 7, 4);
        }

    /* double-differenced measurement error covariance */
    ddcov(nb, b, Ri, Rj, nv, R);

    free(Ri);
    free(Rj);
    free(im);
    free(tropu);
    free(tropr);
    free(dtdxu);
    free(dtdxr);

    return nv;
}


/* time-interpolation of residuals (for post-mission) ------------------------*/
double intpres(gtime_t time, const obsd_t *obs, int n, const nav_t *nav,
    rtk_t *rtk, double *y)
{
    static obsd_t obsb[MAXOBS];
    static double yb[MAXOBS * NFREQ * 2];
    static double rs[MAXOBS * 6];
    static double dts[MAXOBS * 2];
    static double var[MAXOBS];
    static double e[MAXOBS * 3];
    static double azel[MAXOBS * 2];
    static int nb = 0;
    static int svh[MAXOBS * 2];
    prcopt_t *opt = &rtk->opt;
    double tt = timediff(time, obs[0].time);
    double ttb;
    double *p;
    double *q;
    int i;
    int j;
    int k;
    int nf = NF_RTK(opt);

    trace(3, "intpres : n=%d tt=%.1f\n", n, tt);

    if (nb == 0 || fabs(tt) < DTTOL)
        {
            nb = n;
            for (i = 0; i < n; i++)
                {
                    obsb[i] = obs[i];
                }
            return tt;
        }
    ttb = timediff(time, obsb[0].time);
    if (fabs(ttb) > opt->maxtdiff * 2.0 || ttb == tt)
        {
            return tt;
        }

    satposs(time, obsb, nb, nav, opt->sateph, rs, dts, var, svh);

    if (!zdres(1, obsb, nb, rs, dts, svh, nav, rtk->rb, opt, 1, yb, e, azel))
        {
            return tt;
        }
    for (i = 0; i < n; i++)
        {
            for (j = 0; j < nb; j++)
                {
                    if (obsb[j].sat == obs[i].sat)
                        {
                            break;
                        }
                }
            if (j >= nb)
                {
                    continue;
                }
            for (k = 0, p = y + i * nf * 2, q = yb + j * nf * 2; k < nf * 2; k++, p++, q++)
                {
                    if (*p == 0.0 || *q == 0.0)
                        {
                            *p = 0.0;
                        }
                    else
                        {
                            *p = (ttb * (*p) - tt * (*q)) / (ttb - tt);
                        }
                }
        }
    return fabs(ttb) > fabs(tt) ? ttb : tt;
}


/* single to double-difference transformation matrix (D') --------------------*/
/* index of single-to-double-difference transformation (demo5/RTKLIB 2.4.3 style):
   returns pairs of phase-bias state indices {ref, target} instead of a D matrix */
int ddidx(rtk_t *rtk, int *ix, int gps, int glo, int sbs)
{
    int i;
    int j;
    int k;
    int m;
    int f;
    int n;
    int nb = 0;
    int na = rtk->na;
    int nf = NF_RTK(&rtk->opt);
    int nofix;

    trace(3, "ddidx: gps=%d glo=%d/%d sbs=%d\n", gps, glo, rtk->opt.glomodear, sbs);

    /* clear fix flag for all sats (1=float, 2=fix) */
    for (i = 0; i < MAXSAT; i++)
        {
            for (j = 0; j < NFREQ; j++)
                {
                    rtk->ssat[i].fix[j] = 0;
                }
        }
    for (m = 0; m < 5; m++)
        { /* m=0:gps/sbs, 1:glo, 2:gal, 3:bds, 4:qzs */
            /* skip if ambiguity resolution turned off for this sys */
            nofix = (m == 0 && gps == 0) || (m == 1 && glo == 0) || (m == 3 && rtk->opt.bdsmodear == 0);

            /* step through freqs */
            for (f = 0, k = na; f < nf; f++, k += MAXSAT)
                {
                    /* look for first valid sat (i=state index, i-k=sat index) */
                    for (i = k; i < k + MAXSAT; i++)
                        {
                            if (rtk->x[i] == 0.0 || !test_sys(rtk->ssat[i - k].sys, m) ||
                                !rtk->ssat[i - k].vsat[f])
                                {
                                    continue;
                                }
                            /* set sat to use for fixing ambiguity if meets criteria */
                            if (rtk->ssat[i - k].lock[f] >= 0 && !(rtk->ssat[i - k].slip[f] & 2) &&
                                rtk->ssat[i - k].azel[1] >= rtk->opt.elmaskar && !nofix)
                                {
                                    rtk->ssat[i - k].fix[f] = 2; /* fix */
                                    break;
                                }
                            rtk->ssat[i - k].fix[f] = 1;
                        }
                    if (i >= k + MAXSAT || rtk->ssat[i - k].fix[f] != 2)
                        {
                            continue; /* no good ref sat found */
                        }
                    /* step through all sats (j=state index, j-k=sat index) */
                    for (n = 0, j = k; j < k + MAXSAT; j++)
                        {
                            if (i == j || rtk->x[j] == 0.0 || !test_sys(rtk->ssat[j - k].sys, m) ||
                                !rtk->ssat[j - k].vsat[f])
                                {
                                    continue;
                                }
                            if (sbs == 0 && satsys(j - k + 1, nullptr) == SYS_SBS)
                                {
                                    continue;
                                }
                            if (rtk->ssat[j - k].lock[f] >= 0 && !(rtk->ssat[j - k].slip[f] & 2) &&
                                rtk->ssat[j - k].vsat[f] &&
                                rtk->ssat[j - k].azel[1] >= rtk->opt.elmaskar && !nofix)
                                {
                                    ix[nb * 2] = i;     /* state index of ref bias */
                                    ix[nb * 2 + 1] = j; /* state index of target bias */
                                    nb++;
                                    rtk->ssat[j - k].fix[f] = 2; /* fix */
                                    n++;
                                }
                            else
                                {
                                    rtk->ssat[j - k].fix[f] = 1;
                                }
                        }
                    /* don't use ref sat if no sat pairs */
                    if (n == 0)
                        {
                            rtk->ssat[i - k].fix[f] = 1;
                        }
                }
        }
    return nb;
}


/* restore single-differenced ambiguity --------------------------------------*/
void restamb(rtk_t *rtk, const double *bias, int nb __attribute((unused)), double *xa)
{
    int i;
    int n;
    int m;
    int f;
    std::vector<int> index(MAXSAT);
    int nv = 0;
    int nf = NF_RTK(&rtk->opt);

    trace(3, "restamb :\n");

    for (i = 0; i < rtk->nx; i++)
        {
            xa[i] = rtk->x[i];
        }
    for (i = 0; i < rtk->na; i++)
        {
            xa[i] = rtk->xa[i];
        }

    for (m = 0; m < 5; m++)
        {
            for (f = 0; f < nf; f++)
                {
                    for (n = i = 0; i < MAXSAT; i++)
                        {
                            if (!test_sys(rtk->ssat[i].sys, m) || rtk->ssat[i].fix[f] != 2)
                                {
                                    continue;
                                }
                            index[n++] = IB_RTK(i + 1, f, &rtk->opt);
                        }
                    if (n < 2)
                        {
                            continue;
                        }

                    xa[index[0]] = rtk->x[index[0]];

                    for (i = 1; i < n; i++)
                        {
                            xa[index[i]] = xa[index[0]] - bias[nv++];
                        }
                }
        }
}


/* hold integer ambiguity ----------------------------------------------------*/
void holdamb(rtk_t *rtk, const double *xa)
{
    double *v;
    double *H;
    double *R;
    int i;
    int n;
    int m;
    int f;
    int info;
    std::vector<int> index(MAXSAT);
    std::vector<int> sat(MAXSAT);
    std::vector<int> used(MAXSAT, 0);
    int nb = rtk->nx - rtk->na;
    int nv = 0;
    int ns = 0;
    int nf = NF_RTK(&rtk->opt);

    trace(3, "holdamb :\n");

    v = mat(nb, 1);
    H = zeros(nb, rtk->nx);

    for (m = 0; m < 5; m++)
        {
            for (f = 0; f < nf; f++)
                {
                    for (n = i = 0; i < MAXSAT; i++)
                        {
                            if (!test_sys(rtk->ssat[i].sys, m) || rtk->ssat[i].fix[f] != 2 ||
                                rtk->ssat[i].azel[1] < rtk->opt.elmaskhold)
                                {
                                    continue;
                                }
                            index[n] = IB_RTK(i + 1, f, &rtk->opt);
                            sat[n++] = i;
                        }
                    if (n < 2)
                        {
                            continue; /* at least two sats needed to form a double difference */
                        }
                    /* mark and count unique sats actually used for hold */
                    for (i = 0; i < n; i++)
                        {
                            rtk->ssat[sat[i]].fix[f] = 3; /* hold */
                            if (!used[sat[i]])
                                {
                                    used[sat[i]] = 1;
                                    ns++;
                                }
                        }
                    /* constraint to fixed ambiguity */
                    for (i = 1; i < n; i++)
                        {
                            v[nv] = (xa[index[0]] - xa[index[i]]) - (rtk->x[index[0]] - rtk->x[index[i]]);

                            H[index[0] + nv * rtk->nx] = 1.0;
                            H[index[i] + nv * rtk->nx] = -1.0;
                            nv++;
                        }
                }
        }
    /* skip if less than the minimum sats for hold */
    if (rtk->opt.modear == ARMODE_FIXHOLD && ns < rtk->opt.minholdsats)
        {
            trace(3, "holdamb: not enough sats to hold ambiguity (ns=%d min=%d nv=%d)\n",
                ns, rtk->opt.minholdsats, nv);
            free(v);
            free(H);
            return;
        }
    if (nv > 0)
        {
            rtk->holdamb = 1; /* flag to indicate hold has occurred */
            R = zeros(nv, nv);
            for (i = 0; i < nv; i++)
                {
                    R[i + i * nv] = rtk->opt.varholdamb > 0.0 ? rtk->opt.varholdamb : VAR_HOLDAMB;
                }

            /* update states with constraints */
            if ((info = filter(rtk->x, rtk->P, H, v, R, rtk->nx, nv)))
                {
                    errmsg(rtk, "filter error (info=%d)\n", info);
                }
            free(R);
        }
    free(v);
    free(H);
}


/* resolve integer ambiguity by LAMBDA ---------------------------------------*/
int resamb_LAMBDA(rtk_t *rtk, double *bias, double *xa, int gps, int glo, int sbs)
{
    prcopt_t *opt = &rtk->opt;
    int i;
    int j;
    int f;
    int ns;
    int nb;
    int nb1;
    int info;
    int nx = rtk->nx;
    int na = rtk->na;
    int *ix;
    double *DP;
    double *y;
    double *b;
    double *db;
    double *Qb;
    double *Qab;
    double *QQ;
    double s[2];
    double coeff[3];

    trace(3, "resamb_LAMBDA : nx=%d\n", nx);

    rtk->sol.ratio = 0.0;
    rtk->nb_ar = 0;

    /* index of single to double-difference transformation matrix (D'),
       used to translate phase biases to double differences */
    ix = imat(nx, 2);
    nb = ddidx(rtk, ix, gps, glo, sbs);

    /* count unique sats used for AR on at least one frequency */
    ns = 0;
    for (i = 0; i < MAXSAT; i++)
        {
            for (f = 0; f < NF_RTK(opt); f++)
                {
                    if (rtk->ssat[i].fix[f] == 2)
                        {
                            ns++;
                            break;
                        }
                }
        }
    if (nb <= 0 || ns < opt->minfixsats)
        {
            errmsg(rtk, "not enough valid sats for AR: nb=%d ns=%d min=%d\n",
                nb, ns, opt->minfixsats);
            free(ix);
            return -1; /* flag abort */
        }
    rtk->nb_ar = nb;
    /* nx=# of float states, na=# of fixed states, nb=# of double-diff phase biases */
    y = mat(nb, 1);
    DP = mat(nb, nx - na);
    b = mat(nb, 2);
    db = mat(nb, 1);
    Qb = mat(nb, nb);
    Qab = mat(na, nb);
    QQ = mat(na, nb);

    /* phase-bias covariance (Qb) and real-parameters to bias covariance (Qab)
       computed directly from the index pairs: y=D*x, Qb=D*P*D', Qab=Qac*D' */
    for (i = 0; i < nb; i++)
        {
            y[i] = rtk->x[ix[i * 2]] - rtk->x[ix[i * 2 + 1]];
        }
    for (j = 0; j < nx - na; j++)
        {
            for (i = 0; i < nb; i++)
                {
                    DP[i + j * nb] = rtk->P[ix[i * 2] + (na + j) * nx] - rtk->P[ix[i * 2 + 1] + (na + j) * nx];
                }
        }
    for (j = 0; j < nb; j++)
        {
            for (i = 0; i < nb; i++)
                {
                    Qb[i + j * nb] = DP[i + (ix[j * 2] - na) * nb] - DP[i + (ix[j * 2 + 1] - na) * nb];
                }
        }
    for (j = 0; j < nb; j++)
        {
            for (i = 0; i < na; i++)
                {
                    Qab[i + j * na] = rtk->P[i + ix[j * 2] * nx] - rtk->P[i + ix[j * 2 + 1] * nx];
                }
        }

    trace(4, "N(0)=");
    tracemat(4, y, 1, nb, 10, 3);

    /* lambda/mlambda integer least-square estimation */
    if (!(info = lambda(nb, 2, y, Qb, b, s)))
        {
            trace(4, "N(1)=");
            tracemat(4, b, 1, nb, 10, 3);
            trace(4, "N(2)=");
            tracemat(4, b + nb, 1, nb, 10, 3);

            rtk->sol.ratio = s[0] > 0 ? static_cast<float>(s[1] / s[0]) : 0.0F;
            if (rtk->sol.ratio > 999.9)
                {
                    rtk->sol.ratio = 999.9F;
                }

            /* adjust AR ratio threshold based on # of sat pairs, unless min==max */
            if (opt->thresar[5] != opt->thresar[6])
                {
                    nb1 = nb < 50 ? nb : 50; /* poly only fitted for up to 50 sat pairs */
                    /* generate poly coeffs based on nominal AR ratio */
                    for (i = 0; i < 3; i++)
                        {
                            coeff[i] = AR_POLY_COEFFS[i][0];
                            for (j = 1; j < 5; j++)
                                {
                                    coeff[i] = coeff[i] * opt->thresar[0] + AR_POLY_COEFFS[i][j];
                                }
                        }
                    /* generate adjusted AR ratio based on # of sat pairs */
                    rtk->sol.thres = static_cast<float>(coeff[0]);
                    for (i = 1; i < 3; i++)
                        {
                            rtk->sol.thres = static_cast<float>(rtk->sol.thres * 1.0 / (nb1 + 1.0) + coeff[i]);
                        }
                    rtk->sol.thres = static_cast<float>(std::min(std::max(static_cast<double>(rtk->sol.thres), opt->thresar[5]), opt->thresar[6]));
                }
            else
                {
                    rtk->sol.thres = static_cast<float>(opt->thresar[0]);
                }
            /* validation by popular ratio-test */
            if (s[0] <= 0.0 || s[1] / s[0] >= rtk->sol.thres)
                {
                    /* transform float to fixed solution (xa=xa-Qab*Qb\(b0-b)) */
                    for (i = 0; i < na; i++)
                        {
                            rtk->xa[i] = rtk->x[i];
                            for (j = 0; j < na; j++)
                                {
                                    rtk->Pa[i + j * na] = rtk->P[i + j * nx];
                                }
                        }
                    for (i = 0; i < nb; i++)
                        {
                            bias[i] = b[i];
                            y[i] -= b[i];
                        }
                    if (!matinv(Qb, nb))
                        {
                            matmul("NN", nb, 1, nb, 1.0, Qb, y, 0.0, db);
                            matmul("NN", na, 1, nb, -1.0, Qab, db, 1.0, rtk->xa);

                            /* covariance of fixed solution (Qa=Qa-Qab*Qb^-1*Qab') */
                            matmul("NN", na, nb, nb, 1.0, Qab, Qb, 0.0, QQ);
                            matmul("NT", na, na, nb, -1.0, QQ, Qab, 1.0, rtk->Pa);

                            trace(3, "resamb : validation ok (nb=%d ratio=%.2f thresh=%.2f s=%.2f/%.2f)\n",
                                nb, s[0] == 0.0 ? 0.0 : s[1] / s[0], rtk->sol.thres, s[0], s[1]);

                            /* restore single-differenced ambiguity */
                            restamb(rtk, bias, nb, xa);
                        }
                    else
                        {
                            nb = 0;
                        }
                }
            else
                { /* validation failed */
                    errmsg(rtk, "ambiguity validation failed (nb=%d ratio=%.2f thresh=%.2f s=%.2f/%.2f)\n",
                        nb, s[1] / s[0], rtk->sol.thres, s[0], s[1]);
                    nb = 0;
                }
        }
    else
        {
            errmsg(rtk, "lambda error (info=%d)\n", info);
        }
    free(ix);
    free(y);
    free(DP);
    free(b);
    free(db);
    free(Qb);
    free(Qab);
    free(QQ);

    return nb; /* number of ambiguities */
}


/* resolve integer ambiguity by LAMBDA using partial fix techniques and
   multiple attempts (from the demo5 RTKLIB fork) ----------------------------*/
int manage_amb_LAMBDA(rtk_t *rtk, double *bias, double *xa, const int *sat,
    int nf, int ns)
{
    int i;
    int f;
    int lockc[NFREQ];
    int excsat = 0;
    int nbsat = 0;
    int gps1;
    int glo1;
    int sbas1;
    int glo2;
    int sbas2;
    int nb;
    int rerun;
    int dly;
    float ratio1;
    float posvar = 0.0F;

    /* calc position variance, will skip AR if too high to avoid false fix */
    for (i = 0; i < 3; i++)
        {
            posvar += static_cast<float>(rtk->P[i + i * rtk->nx]);
        }
    posvar /= 3.0F;

    trace(3, "posvar=%.6f\n", posvar);
    trace(3, "prevRatios= %.3f %.3f\n", rtk->sol.prev_ratio1, rtk->sol.prev_ratio2);
    trace(3, "num ambiguities used last AR: %d\n", rtk->nb_ar);

    /* skip AR if criteria not met */
    if (rtk->opt.mode <= PMODE_DGPS || rtk->opt.modear == ARMODE_OFF ||
        rtk->opt.thresar[0] < 1.0 ||
        (rtk->opt.armaxposvar > 0.0 && posvar > rtk->opt.armaxposvar))
        {
            trace(3, "Skip AR\n");
            rtk->sol.ratio = 0.0F;
            rtk->sol.prev_ratio1 = rtk->sol.prev_ratio2 = 0.0F;
            rtk->nb_ar = 0;
            return 0;
        }
    /* if no fix on previous sample and enough sats, cycle the excluded sat */
    for (i = 0; i < ns; i++)
        {
            for (f = 0; f < nf; f++)
                {
                    if (rtk->ssat[sat[i] - 1].fix[f] == 2)
                        {
                            nbsat++;
                            break;
                        }
                }
        }
    if (rtk->opt.mindropsats > 0 && rtk->sol.prev_ratio2 < rtk->sol.thres &&
        nbsat >= rtk->opt.mindropsats)
        {
            /* find the position of the last excluded sat */
            i = 0;
            if (rtk->excsat != 0)
                {
                    for (; i < ns; i++)
                        {
                            if (rtk->excsat == sat[i])
                                {
                                    i++;
                                    break;
                                }
                        }
                    if (i >= ns)
                        {
                            i = 0; /* if not found, restart from the first sat */
                        }
                }
            /* find the next sat used last time for AR */
            for (; i < ns; i++)
                {
                    for (f = 0; f < nf; f++)
                        {
                            if (rtk->ssat[sat[i] - 1].vsat[f] && rtk->ssat[sat[i] - 1].lock[f] >= 0 &&
                                rtk->ssat[sat[i] - 1].azel[1] >= rtk->opt.elmin)
                                {
                                    excsat = sat[i];
                                    break;
                                }
                        }
                    if (excsat)
                        {
                            break;
                        }
                }
            if (excsat)
                {
                    for (f = 0; f < nf; f++)
                        {
                            lockc[f] = rtk->ssat[excsat - 1].lock[f]; /* save lock count */
                            /* remove sat from AR long enough to enable hold if it stays fixed */
                            rtk->ssat[excsat - 1].lock[f] = -rtk->nb_ar;
                        }
                    trace(3, "AR: exclude sat %d\n", excsat);
                }
            rtk->excsat = excsat;
        }

    /* for the initial ambiguity resolution attempt, include all enabled sats */
    gps1 = 1;
    /* enable GLO AR only if configured and, for fix-and-hold, once hold has been acquired */
    glo1 = ((rtk->opt.navsys & SYS_GLO) != 0 && rtk->opt.glomodear != 0 &&
               (rtk->opt.glomodear != 3 || rtk->holdamb))
               ? 1
               : 0; /* glomodear 3: fix-and-hold */
    sbas1 = (rtk->opt.navsys & SYS_GLO) != 0 ? glo1 : (((rtk->opt.navsys & SYS_SBS) != 0) ? 1 : 0);

    /* first attempt to resolve ambiguities */
    nb = resamb_LAMBDA(rtk, bias, xa, gps1, glo1, sbas1);
    ratio1 = rtk->sol.ratio;

    /* reject bad satellites if AR filtering enabled */
    if (rtk->opt.arfilter)
        {
            rerun = 0;
            /* if results much poorer than previous epoch or below the AR ratio
               threshold, remove new sats */
            if (nb >= 0 && rtk->sol.prev_ratio2 >= rtk->sol.thres &&
                ((rtk->sol.ratio < rtk->sol.thres) ||
                    (rtk->sol.ratio < rtk->opt.thresar[0] * 1.1 && rtk->sol.ratio < rtk->sol.prev_ratio1 / 2.0)))
                {
                    trace(3, "low ratio: check for new sat\n");
                    dly = 2;
                    for (i = 0; i < ns; i++)
                        {
                            for (f = 0; f < nf; f++)
                                {
                                    if (rtk->ssat[sat[i] - 1].fix[f] != 2)
                                        {
                                            continue;
                                        }
                                    /* check for new sats */
                                    if (rtk->ssat[sat[i] - 1].lock[f] == 0)
                                        {
                                            trace(3, "remove sat %d:%d lock=%d\n", sat[i], f, rtk->ssat[sat[i] - 1].lock[f]);
                                            /* delay use of this sat with stagger */
                                            rtk->ssat[sat[i] - 1].lock[f] = -rtk->opt.minlock - dly;
                                            dly += 2;
                                            rerun = 1;
                                        }
                                }
                        }
                }
            /* rerun if filter removed any sats */
            if (rerun)
                {
                    trace(3, "rerun AR with new sats removed\n");
                    nb = resamb_LAMBDA(rtk, bias, xa, gps1, glo1, sbas1);
                }
        }

    /* if fix-and-hold glomodear enabled, re-run AR with GLO/SBS disabled if it differs */
    if ((rtk->opt.navsys & SYS_GLO) != 0 && rtk->opt.glomodear == 3 && rtk->sol.ratio < rtk->sol.thres)
        {
            glo2 = sbas2 = 0;
            if (glo1 != glo2)
                {
                    nb = resamb_LAMBDA(rtk, bias, xa, 1, glo2, sbas2);
                }
        }
    /* restore the excluded sat if still no fix or no significant increase in AR ratio */
    if (excsat != 0 && (rtk->sol.ratio < rtk->sol.thres) &&
        (rtk->sol.ratio < (1.5 * rtk->sol.prev_ratio2)))
        {
            for (f = 0; f < nf; f++)
                {
                    rtk->ssat[excsat - 1].lock[f] = lockc[f];
                }
            trace(3, "AR: restore sat %d\n", excsat);
        }

    rtk->sol.prev_ratio1 = ratio1 > 0.0F ? ratio1 : rtk->sol.ratio;
    rtk->sol.prev_ratio2 = rtk->sol.ratio;

    return nb;
}


/* validation of solution ----------------------------------------------------*/
int valpos(rtk_t *rtk, const double *v, const double *R, const int *vflg,
    int nv, double thres)
{
#if 0
    prcopt_t *opt = &rtk->opt;
    double vv = 0.0;
#endif
    double fact = thres * thres;
    int i;
    int stat = 1;
    int sat1;
    int sat2;
    int type;
    int freq;
    char stype;

    trace(3, "valpos  : nv=%d thres=%.1f\n", nv, thres);

    /* post-fit residual test */
    for (i = 0; i < nv; i++)
        {
            if (v[i] * v[i] <= fact * R[i + i * nv])
                {
                    continue;
                }
            sat1 = (vflg[i] >> 16) & 0xFF;
            sat2 = (vflg[i] >> 8) & 0xFF;
            type = (vflg[i] >> 4) & 0xF;
            freq = vflg[i] & 0xF;
            stype = type == 0 ? 'L' : (type == 1 ? 'L' : 'C');
            errmsg(rtk, "large residual (sat=%2d-%2d %c%d v=%6.3f sig=%.3f)\n",
                sat1, sat2, stype, freq + 1, v[i], std::sqrt(R[i + i * nv]));
        }
#if 0 /* omitted v.2.4.0 */
    if (stat && nv > NP(opt))
        {
            /* chi-square validation */
            for (i = 0; i < nv; i++) vv += v[i] * v[i] / R[i + i * nv];

            if (vv > chisqr[nv - NP(opt) - 1])
                {
                    errmsg(rtk, "residuals validation failed (nv=%d np=%d vv=%.2f cs=%.2f)\n",
                        nv, NP(opt), vv, chisqr[nv - NP(opt) - 1]);
                    stat = 0;
                }
            else
                {
                    trace(3, "valpos : validation ok (%s nv=%d np=%d vv=%.2f cs=%.2f)\n",
                        rtk->tstr, nv, NP(opt), vv, chisqr[nv - NP(opt) - 1]);
                }
        }
#endif
    return stat;
}


/* relative positioning ------------------------------------------------------*/
int relpos(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
    const nav_t *nav)
{
    prcopt_t *opt = &rtk->opt;
    gtime_t time = obs[0].time;
    double *rs;
    double *dts;
    double *var;
    double *y;
    double *e;
    double *azel;
    double *v;
    double *H;
    double *R;
    double *xp;
    double *Pp;
    double *xa;
    double *bias;
    double dt;
    int i;
    int j;
    int f;
    int n = nu + nr;
    int ns;
    int ny;
    int nv;
    std::vector<int> sat(MAXSAT);
    std::vector<int> iu(MAXSAT);
    std::vector<int> ir(MAXSAT);
    int niter;
    int info;
    std::vector<int> vflg(MAXOBS * NFREQ * 2 + 1);
    std::vector<int> svh(MAXOBS * 2);
    int stat = rtk->opt.mode <= PMODE_DGPS ? SOLQ_DGPS : SOLQ_FLOAT;
    int nf = opt->ionoopt == IONOOPT_IFLC ? 1 : opt->nf;

    trace(3, "relpos  : nx=%d nu=%d nr=%d\n", rtk->nx, nu, nr);

    dt = timediff(time, obs[nu].time);

    rs = mat(6, n);
    dts = mat(2, n);
    var = mat(1, n);
    y = mat(nf * 2, n);
    e = mat(3, n);
    azel = zeros(2, n);

    for (i = 0; i < MAXSAT; i++)
        {
            rtk->ssat[i].sys = satsys(i + 1, nullptr);
            for (j = 0; j < NFREQ; j++)
                {
                    rtk->ssat[i].vsat[j] = rtk->ssat[i].snr[j] = 0;
                    rtk->ssat[i].code[j] = 0;
                }
        }
    /* satellite positions/clocks */
    satposs(time, obs, n, nav, opt->sateph, rs, dts, var, svh.data());

    /* undifferenced residuals for base station */
    if (!zdres(1, obs + nu, nr, rs + nu * 6, dts + nu * 2, &svh[nu], nav, rtk->rb, opt, 1,
            y + nu * nf * 2, e + nu * 3, azel + nu * 2))
        {
            errmsg(rtk, "initial base station position error\n");

            free(rs);
            free(dts);
            free(var);
            free(y);
            free(e);
            free(azel);
            return 0;
        }
    /* time-interpolation of residuals (for post-processing) */
    if (opt->intpref)
        {
            dt = intpres(time, obs + nu, nr, nav, rtk, y + nu * nf * 2);
        }
    /* select common satellites between rover and base-station */
    if ((ns = selsat(obs, azel, nu, nr, opt, sat.data(), iu.data(), ir.data())) <= 0)
        {
            errmsg(rtk, "no common satellite\n");

            free(rs);
            free(dts);
            free(var);
            free(y);
            free(e);
            free(azel);
            return 0;
        }
    /* temporal update of states */
    udstate(rtk, obs, sat.data(), iu.data(), ir.data(), ns, nav);

    /* store rover and base signal strengths (dBHz) for the observation
       weighting model and the reference-satellite selection */
    for (i = 0; i < ns; i++)
        {
            for (j = 0; j < nf; j++)
                {
                    rtk->ssat[sat[i] - 1].snr_rover[j] = 0.25F * static_cast<float>(obs[iu[i]].SNR[j]);
                    rtk->ssat[sat[i] - 1].snr_base[j] = 0.25F * static_cast<float>(obs[ir[i]].SNR[j]);
                }
        }

    trace(4, "x(0)=");
    tracemat(4, rtk->x, 1, NR_RTK(opt), 13, 4);

    xp = mat(rtk->nx, 1);
    Pp = zeros(rtk->nx, rtk->nx);
    xa = mat(rtk->nx, 1);
    matcpy(xp, rtk->x, rtk->nx, 1);
    /* the first ddres call reads Pp to detect freshly initialized phase
       biases (innovation-threshold opening), so it must see the real
       covariance, not zeros (demo5) */
    matcpy(Pp, rtk->P, rtk->nx, rtk->nx);

    ny = ns * nf * 2 + 2;
    v = mat(ny, 1);
    H = zeros(rtk->nx, ny);
    R = mat(ny, ny);
    bias = mat(rtk->nx, 1);

    /* add 2 iterations for baseline-constraint moving-base */
    niter = opt->niter + (opt->mode == PMODE_MOVEB && opt->baseline[0] > 0.0 ? 2 : 0);

    for (i = 0; i < niter; i++)
        {
            /* undifferenced residuals for rover */
            if (!zdres(0, obs, nu, rs, dts, svh.data(), nav, xp, opt, 0, y, e, azel))
                {
                    errmsg(rtk, "rover initial position error\n");
                    stat = SOLQ_NONE;
                    break;
                }
            /* double-differenced residuals and partial derivatives */
            if ((nv = ddres(rtk, obs, nav, dt, xp, Pp, sat.data(), y, e, azel, iu.data(), ir.data(), ns, v, H, R, vflg.data())) < 1)
                {
                    errmsg(rtk, "no double-differenced residual\n");
                    stat = SOLQ_NONE;
                    break;
                }
            /* kalman filter measurement update */
            matcpy(Pp, rtk->P, rtk->nx, rtk->nx);
            if ((info = filter(xp, Pp, H, v, R, rtk->nx, nv)))
                {
                    errmsg(rtk, "filter error (info=%d)\n", info);
                    stat = SOLQ_NONE;
                    break;
                }
            trace(4, "x(%d)=", i + 1);
            tracemat(4, xp, 1, NR_RTK(opt), 13, 4);
        }
    if (stat != SOLQ_NONE && zdres(0, obs, nu, rs, dts, svh.data(), nav, xp, opt, 0, y, e, azel))
        {
            /* post-fit residuals for float solution */
            nv = ddres(rtk, obs, nav, dt, xp, Pp, sat.data(), y, e, azel, iu.data(), ir.data(), ns, v, nullptr, R, vflg.data());

            /* validation of float solution */
            if (valpos(rtk, v, R, vflg.data(), nv, 4.0))
                {
                    /* update state and covariance matrix */
                    matcpy(rtk->x, xp, rtk->nx, 1);
                    matcpy(rtk->P, Pp, rtk->nx, rtk->nx);

                    /* update ambiguity control struct */
                    rtk->sol.ns = 0;
                    for (i = 0; i < ns; i++)
                        {
                            for (f = 0; f < nf; f++)
                                {
                                    if (!rtk->ssat[sat[i] - 1].vsat[f])
                                        {
                                            continue;
                                        }
                                    /* inc lock count while it recovers from a reset (negative) or
                                       if the sat contributed to a good fix (demo5 policy) */
                                    if (rtk->ssat[sat[i] - 1].lock[f] < 0 ||
                                        (rtk->nfix > 0 && rtk->ssat[sat[i] - 1].fix[f] >= 2))
                                        {
                                            rtk->ssat[sat[i] - 1].lock[f]++;
                                        }
                                    rtk->ssat[sat[i] - 1].outc[f] = 0;
                                    if (f == 0)
                                        {
                                            rtk->sol.ns++; /* valid satellite count by L1 */
                                        }
                                }
                        }
                    /* too few valid phase measurements: fall back to the code-differenced
                       solution instead of dropping the epoch altogether (demo5) */
                    if (rtk->sol.ns < 4)
                        {
                            stat = SOLQ_DGPS;
                        }
                }
            else
                {
                    stat = SOLQ_NONE;
                }
        }
    /* resolve integer ambiguity by WL-NL */
    if (stat != SOLQ_NONE && rtk->opt.modear == ARMODE_WLNL)
        {
            if (resamb_WLNL(rtk, obs, sat.data(), iu.data(), ir.data(), ns, nav, azel))
                {
                    stat = SOLQ_FIX;
                }
        }
    /* resolve integer ambiguity by TCAR */
    else if (stat != SOLQ_NONE && rtk->opt.modear == ARMODE_TCAR)
        {
            if (resamb_TCAR(rtk, obs, sat.data(), iu.data(), ir.data(), ns, nav, azel))
                {
                    stat = SOLQ_FIX;
                }
        }
    /* resolve integer ambiguity by LAMBDA (only from a valid float solution: a
       code-differenced epoch has no phase biases to fix) */
    else if (stat == SOLQ_FLOAT && manage_amb_LAMBDA(rtk, bias, xa, sat.data(), nf, ns) > 1)
        {
            if (zdres(0, obs, nu, rs, dts, svh.data(), nav, xa, opt, 0, y, e, azel))
                {
                    /* post-fit reisiduals for fixed solution */
                    nv = ddres(rtk, obs, nav, dt, xa, nullptr, sat.data(), y, e, azel, iu.data(), ir.data(), ns, v, nullptr, R, vflg.data());

                    /* validation of fixed solution */
                    if (valpos(rtk, v, R, vflg.data(), nv, 4.0))
                        {
                            /* hold integer ambiguity */
                            if (++rtk->nfix >= rtk->opt.minfix &&
                                rtk->opt.modear == ARMODE_FIXHOLD)
                                {
                                    holdamb(rtk, xa);
                                }
                            stat = SOLQ_FIX;
                        }
                }
        }

    /* save solution status */
    if (stat == SOLQ_FIX)
        {
            for (i = 0; i < 3; i++)
                {
                    rtk->sol.rr[i] = rtk->xa[i];
                    rtk->sol.qr[i] = static_cast<float>(rtk->Pa[i + i * rtk->na]);
                }
            rtk->sol.qr[3] = static_cast<float>(rtk->Pa[1]);
            rtk->sol.qr[4] = static_cast<float>(rtk->Pa[1 + 2 * rtk->na]);
            rtk->sol.qr[5] = static_cast<float>(rtk->Pa[2]);
        }
    else
        {
            for (i = 0; i < 3; i++)
                {
                    rtk->sol.rr[i] = rtk->x[i];
                    rtk->sol.qr[i] = static_cast<float>(rtk->P[i + i * rtk->nx]);
                }
            rtk->sol.qr[3] = static_cast<float>(rtk->P[1]);
            rtk->sol.qr[4] = static_cast<float>(rtk->P[1 + 2 * rtk->nx]);
            rtk->sol.qr[5] = static_cast<float>(rtk->P[2]);
            rtk->nfix = 0;
        }
    for (i = 0; i < n; i++)
        {
            for (j = 0; j < nf; j++)
                {
                    if (obs[i].L[j] == 0.0)
                        {
                            continue;
                        }
                    rtk->ssat[obs[i].sat - 1].pt[obs[i].rcv - 1][j] = obs[i].time;
                    rtk->ssat[obs[i].sat - 1].ph[obs[i].rcv - 1][j] = obs[i].L[j];
                }
        }
    for (i = 0; i < ns; i++)
        {
            for (j = 0; j < nf; j++)
                {
                    /* output snr of rover receiver */
                    rtk->ssat[sat[i] - 1].snr[j] = obs[iu[i]].SNR[j];
                    rtk->ssat[sat[i] - 1].code[j] = obs[iu[i]].code[j];
                }
        }
    for (i = 0; i < MAXSAT; i++)
        {
            for (j = 0; j < nf; j++)
                {
                    /* keep fix[] == 2 across non-fixed epochs (demo5): the flags
                       record which satellites entered the last AR attempt, and
                       manage_amb_LAMBDA's mindropsats exclusion cycling counts
                       them after a failed epoch — demoting them here would make
                       that gate unreachable */
                    if (rtk->ssat[i].slip[j] & 1)
                        {
                            rtk->ssat[i].slipc[j]++;
                        }
                }
        }
    free(rs);
    free(dts);
    free(var);
    free(y);
    free(e);
    free(azel);
    free(xp);
    free(Pp);
    free(xa);
    free(v);
    free(H);
    free(R);
    free(bias);

    if (stat != SOLQ_NONE)
        {
            rtk->sol.stat = stat;
        }

    return stat != SOLQ_NONE;
}


/* initialize rtk control ------------------------------------------------------
 * initialize rtk control struct
 * args   : rtk_t    *rtk    IO  rtk control/result struct
 *          prcopt_t *opt    I   positioning options (see rtklib.h)
 * return : none
 *-----------------------------------------------------------------------------*/
void rtkinit(rtk_t *rtk, const prcopt_t *opt)
{
    sol_t sol0 = {{0, 0}, {}, {}, {}, '0', '0', '0', 0.0, 0.0, 0.0, 0.0, 0.0};
    ambc_t ambc0 = {{{0, 0}, {0, 0}, {0, 0}, {0, 0}}, {}, {}, {}, 0, {}};
    ssat_t ssat0 = {0, 0, {0.0}, {0.0}, {0.0}, {'0'}, {'0'}, {}, {'0'}, {'0'}, {'0'}, {}, {}, {}, {}, 0.0, 0.0, 0.0, 0.0, {{{0, 0}}, {{0, 0}}}, {{}, {}}, {}, {}};
    int i;

    trace(3, "rtkinit :\n");

    rtk->sol = sol0;
    for (i = 0; i < 6; i++)
        {
            rtk->rb[i] = 0.0;
        }
    rtk->nx = opt->mode <= PMODE_FIXED ? NX_RTK(opt) : pppnx(opt);
    rtk->na = opt->mode <= PMODE_FIXED ? NR_RTK(opt) : pppnx(opt);
    rtk->tt = 0.0;
    rtk->x = zeros(rtk->nx, 1);
    rtk->P = zeros(rtk->nx, rtk->nx);
    rtk->xa = zeros(rtk->na, 1);
    rtk->Pa = zeros(rtk->na, rtk->na);
    rtk->nfix = rtk->neb = 0;
    rtk->excsat = 0;
    rtk->nb_ar = 0;
    rtk->holdamb = 0;
    for (i = 0; i < MAXSAT; i++)
        {
            rtk->ambc[i] = ambc0;
            rtk->ssat[i] = ssat0;
        }
    for (i = 0; i < MAXERRMSG; i++)
        {
            rtk->errbuf[i] = 0;
        }
    rtk->opt = *opt;
}


/* free rtk control ------------------------------------------------------------
 * free memory for rtk control struct
 * args   : rtk_t    *rtk    IO  rtk control/result struct
 * return : none
 *-----------------------------------------------------------------------------*/
void rtkfree(rtk_t *rtk)
{
    trace(3, "rtkfree :\n");

    rtk->nx = rtk->na = 0;
    free(rtk->x);
    rtk->x = nullptr;
    free(rtk->P);
    rtk->P = nullptr;
    free(rtk->xa);
    rtk->xa = nullptr;
    free(rtk->Pa);
    rtk->Pa = nullptr;
}


/* precise positioning ---------------------------------------------------------
 * input observation data and navigation message, compute rover position by
 * precise positioning
 * args   : rtk_t *rtk       IO  rtk control/result struct
 *            rtk->sol       IO  solution
 *                .time      O   solution time
 *                .rr[]      IO  rover position/velocity
 *                               (I:fixed mode,O:single mode)
 *                .dtr[0]    O   receiver clock bias (s)
 *                .dtr[1]    O   receiver glonass-gps time offset (s)
 *                .Qr[]      O   rover position covarinace
 *                .stat      O   solution status (SOLQ_???)
 *                .ns        O   number of valid satellites
 *                .age       O   age of differential (s)
 *                .ratio     O   ratio factor for ambiguity validation
 *            rtk->rb[]      IO  base station position/velocity
 *                               (I:relative mode,O:moving-base mode)
 *            rtk->nx        I   number of all states
 *            rtk->na        I   number of integer states
 *            rtk->ns        O   number of valid satellite
 *            rtk->tt        O   time difference between current and previous (s)
 *            rtk->x[]       IO  float states pre-filter and post-filter
 *            rtk->P[]       IO  float covariance pre-filter and post-filter
 *            rtk->xa[]      O   fixed states after AR
 *            rtk->Pa[]      O   fixed covariance after AR
 *            rtk->ssat[s]   IO  sat(s+1) status
 *                .sys       O   system (SYS_???)
 *                .az   [r]  O   azimuth angle   (rad) (r=0:rover,1:base)
 *                .el   [r]  O   elevation angle (rad) (r=0:rover,1:base)
 *                .vs   [r]  O   data valid single     (r=0:rover,1:base)
 *                .resp [f]  O   freq(f+1) pseudorange residual (m)
 *                .resc [f]  O   freq(f+1) carrier-phase residual (m)
 *                .vsat [f]  O   freq(f+1) data valid (0:invalid,1:valid)
 *                .fix  [f]  O   freq(f+1) ambiguity flag
 *                               (0:nodata,1:float,2:fix,3:hold)
 *                .slip [f]  O   freq(f+1) slip flag
 *                               (bit8-7:rcv1 LLI, bit6-5:rcv2 LLI,
 *                                bit2:parity unknown, bit1:slip)
 *                .lock [f]  IO  freq(f+1) carrier lock count
 *                .outc [f]  IO  freq(f+1) carrier outage count
 *                .slipc[f]  IO  freq(f+1) cycle slip count
 *                .rejc [f]  IO  freq(f+1) data reject count
 *                .gf        IO  geometry-free phase (L1-L2) (m)
 *                .gf2       IO  geometry-free phase (L1-L5) (m)
 *            rtk->nfix      IO  number of continuous fixes of ambiguity
 *            rtk->neb       IO  bytes of error message buffer
 *            rtk->errbuf    IO  error message buffer
 *            rtk->tstr      O   time string for debug
 *            rtk->opt       I   processing options
 *          obsd_t *obs      I   observation data for an epoch
 *                               obs[i].rcv=1:rover,2:reference
 *                               sorted by receiver and satellte
 *          int    n         I   number of observation data
 *          nav_t  *nav      I   navigation messages
 * return : status (0:no solution,1:valid solution)
 * notes  : before calling function, base station position rtk->sol.rb[] should
 *          be properly set for relative mode except for moving-baseline
 *-----------------------------------------------------------------------------*/
int rtkpos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    prcopt_t *opt = &rtk->opt;
    sol_t solb = {{0, 0}, {}, {}, {}, '0', '0', '0', 0.0, 0.0, 0.0, 0.0, 0.0};
    gtime_t time;
    int i;
    int nu;
    int nr;
    char msg[128] = "";

    trace(3, "rtkpos  : time=%s n=%d\n", time_str(obs[0].time, 3), n);
    trace(4, "obs=\n");
    traceobs(4, obs, n);
    /*trace(5,"nav=\n"); tracenav(5,nav);*/

    /* set base station position */
    if (opt->refpos <= POSOPT_RINEX && opt->mode != PMODE_SINGLE &&
        opt->mode != PMODE_MOVEB)
        {
            for (i = 0; i < 6; i++)
                {
                    rtk->rb[i] = i < 3 ? opt->rb[i] : 0.0;
                }
        }
    /* count rover/base station observations */
    for (nu = 0; nu < n && obs[nu].rcv == 1; nu++)
        {
        }
    for (nr = 0; nu + nr < n && obs[nu + nr].rcv == 2; nr++)
        {
        }

    time = rtk->sol.time; /* previous epoch */

    /* rover position by single point positioning */
    if (!pntpos(obs, nu, nav, &rtk->opt, &rtk->sol, nullptr, rtk->ssat, msg))
        {
            errmsg(rtk, "point pos error (%s)\n", msg);
            if (!rtk->opt.dynamics)
                {
                    outsolstat(rtk);
                    return 0;
                }
        }
    if (time.time != 0)
        {
            rtk->tt = timediff(rtk->sol.time, time);
        }

    /* single point positioning */
    if (opt->mode == PMODE_SINGLE)
        {
            outsolstat(rtk);
            return 1;
        }
    /* suppress output of single solution */
    if (!opt->outsingle)
        {
            rtk->sol.stat = SOLQ_NONE;
        }
    /* precise point positioning */
    if (opt->mode >= PMODE_PPP_KINEMA)
        {
            pppos(rtk, obs, nu, nav);
            outsolstat(rtk);
            return 1;
        }
    /* check number of data of base station and age of differential */
    if (nr == 0)
        {
            errmsg(rtk, "no base station observation data for rtk\n");
            outsolstat(rtk);
            return 1;
        }
    if (opt->mode == PMODE_MOVEB)
        { /*  moving baseline */
            /* estimate position/velocity of base station */
            if (!pntpos(obs + nu, nr, nav, &rtk->opt, &solb, nullptr, nullptr, msg))
                {
                    errmsg(rtk, "base station position error (%s)\n", msg);
                    return 0;
                }
            rtk->sol.age = static_cast<float>(timediff(rtk->sol.time, solb.time));

            if (std::fabs(rtk->sol.age) > TTOL_MOVEB)
                {
                    errmsg(rtk, "time sync error for moving-base (age=%.1f)\n", rtk->sol.age);
                    return 0;
                }
            for (i = 0; i < 6; i++)
                {
                    rtk->rb[i] = solb.rr[i];
                }

            /* time-synchronized position of base station */
            for (i = 0; i < 3; i++)
                {
                    rtk->rb[i] += rtk->rb[i + 3] * rtk->sol.age;
                }
        }
    else
        {
            rtk->sol.age = static_cast<float>(timediff(obs[0].time, obs[nu].time));

            if (std::fabs(rtk->sol.age) > opt->maxtdiff)
                {
                    errmsg(rtk, "age of differential error (age=%.1f)\n", rtk->sol.age);
                    outsolstat(rtk);
                    return 1;
                }
        }
    /* relative potitioning */
    relpos(rtk, obs, nu, nr, nav);
    outsolstat(rtk);

    return 1;
}
