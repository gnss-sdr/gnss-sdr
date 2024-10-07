/*!
 * \file rtklib_rtcm3.cc
 * \brief RTCM functions for v3
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
 *
 * -----------------------------------------------------------------------------
 */


#include "rtklib_rtcm3.h"
#include "rtklib_rtkcmn.h"
#include <cstring>


/* msm signal id table -------------------------------------------------------*/
const char *msm_sig_gps[32] = {
    /* GPS: ref [13] table 3.5-87, ref [14][15] table 3.5-91 */
    "", "1C", "1P", "1W", "1Y", "1M", "", "2C", "2P", "2W", "2Y", "2M", /*  1-12 */
    "", "", "2S", "2L", "2X", "", "", "", "", "5I", "5Q", "5X",         /* 13-24 */
    "", "", "", "", "", "1S", "1L", "1X"                                /* 25-32 */
};


const char *msm_sig_glo[32] = {
    /* GLONASS: ref [13] table 3.5-93,  ref [14][15] table 3.5-97 */
    "", "1C", "1P", "", "", "", "", "2C", "2P", "", "3I", "3Q",
    "3X", "", "", "", "", "", "", "", "", "", "", "",
    "", "", "", "", "", "", "", ""};


const char *msm_sig_gal[32] = {
    /* Galileo: ref [15] table 3.5-100 */
    "", "1C", "1A", "1B", "1X", "1Z", "", "6C", "6A", "6B", "6X", "6Z",
    "", "7I", "7Q", "7X", "", "8I", "8Q", "8X", "", "5I", "5Q", "5X",
    "", "", "", "", "", "", "", ""};


const char *msm_sig_qzs[32] = {
    /* QZSS: ref [15] table 3.5-103 */
    "", "1C", "", "", "", "", "", "", "6S", "6L", "6X", "",
    "", "", "2S", "2L", "2X", "", "", "", "", "5I", "5Q", "5X",
    "", "", "", "", "", "1S", "1L", "1X"};


const char *msm_sig_sbs[32] = {
    /* SBAS: ref [13] table 3.5-T+005 */
    "", "1C", "", "", "", "", "", "", "", "", "", "",
    "", "", "", "", "", "", "", "", "", "5I", "5Q", "5X",
    "", "", "", "", "", "", "", ""};


const char *msm_sig_cmp[32] = {
    /* BeiDou: ref [15] table 3.5-106 */
    "", "1I", "1Q", "1X", "", "", "", "6I", "6Q", "6X", "", "",
    "", "7I", "7Q", "7X", "", "", "", "", "", "", "", "",
    "", "", "", "", "", "", "", ""};

/* get sign-magnitude bits ---------------------------------------------------*/
double getbitg(const unsigned char *buff, int pos, int len)
{
    double value = getbitu(buff, pos + 1, len - 1);
    return getbitu(buff, pos, 1) ? -value : value;
}


/* adjust weekly rollover of gps time ----------------------------------------*/
void adjweek(rtcm_t *rtcm, double tow)
{
    double tow_p;
    int week;

    /* if no time, get cpu time */
    if (rtcm->time.time == 0)
        {
            rtcm->time = utc2gpst(timeget());
        }
    tow_p = time2gpst(rtcm->time, &week);
    if (tow < tow_p - 302400.0)
        {
            tow += 604800.0;
        }
    else if (tow > tow_p + 302400.0)
        {
            tow -= 604800.0;
        }
    rtcm->time = gpst2time(week, tow);
}


/* adjust weekly rollover of bdt time ----------------------------------------*/
int adjbdtweek(int week)
{
    int w;
    (void)time2bdt(gpst2bdt(utc2gpst(timeget())), &w);
    if (w < 1)
        {
            w = 1; /* use 2006/1/1 if time is earlier than 2006/1/1 */
        }
    return week + (w - week + 512) / 1024 * 1024;
}


/* adjust daily rollover of glonass time -------------------------------------*/
void adjday_glot(rtcm_t *rtcm, double tod)
{
    gtime_t time;
    double tow;
    double tod_p;
    int week;

    if (rtcm->time.time == 0)
        {
            rtcm->time = utc2gpst(timeget());
        }
    time = timeadd(gpst2utc(rtcm->time), 10800.0); /* glonass time */
    tow = time2gpst(time, &week);
    tod_p = fmod(tow, 86400.0);
    tow -= tod_p;
    if (tod < tod_p - 43200.0)
        {
            tod += 86400.0;
        }
    else if (tod > tod_p + 43200.0)
        {
            tod -= 86400.0;
        }
    time = gpst2time(week, tow + tod);
    rtcm->time = utc2gpst(timeadd(time, -10800.0));
}


/* adjust carrier-phase rollover ---------------------------------------------*/
double adjcp(rtcm_t *rtcm, int sat, int freq, double cp)
{
    if (rtcm->cp[sat - 1][freq] == 0.0)
        {
        }
    else if (cp < rtcm->cp[sat - 1][freq] - 750.0)
        {
            cp += 1500.0;
        }
    else if (cp > rtcm->cp[sat - 1][freq] + 750.0)
        {
            cp -= 1500.0;
        }
    rtcm->cp[sat - 1][freq] = cp;
    return cp;
}


/* loss-of-lock indicator ----------------------------------------------------*/
int lossoflock(rtcm_t *rtcm, int sat, int freq, int lock)
{
    int lli = (!lock && !rtcm->lock[sat - 1][freq]) || lock < rtcm->lock[sat - 1][freq];
    rtcm->lock[sat - 1][freq] = static_cast<uint16_t>(lock);
    return lli;
}


/* s/n ratio -----------------------------------------------------------------*/
unsigned char snratio(double snr)
{
    return static_cast<unsigned char>(snr <= 0.0 || 255.5 <= snr ? 0.0 : snr * 4.0 + 0.5);
}


/* get observation data index ------------------------------------------------*/
int obsindex3(obs_t *obs, gtime_t time, int sat)
{
    int i;
    int j;

    for (i = 0; i < obs->n; i++)
        {
            if (obs->data[i].sat == sat)
                {
                    return i; /* field already exists */
                }
        }
    if (i >= MAXOBS)
        {
            return -1; /* overflow */
        }

    /* add new field */
    obs->data[i].time = time;
    obs->data[i].sat = sat;
    for (j = 0; j < NFREQ + NEXOBS; j++)
        {
            obs->data[i].L[j] = obs->data[i].P[j] = 0.0;
            obs->data[i].D[j] = 0.0;
            obs->data[i].SNR[j] = obs->data[i].LLI[j] = obs->data[i].code[j] = 0;
        }
    obs->n++;
    return i;
}


/* test station id consistency -----------------------------------------------*/
int test_staid(rtcm_t *rtcm, int staid)
{
    char *p;
    int type;
    int id;

    /* test station id option */
    if ((p = strstr(rtcm->opt, "-STA=")) && sscanf(p, "-STA=%d", &id) == 1)
        {
            if (staid != id)
                {
                    return 0;
                }
        }
    /* save station id */
    if (rtcm->staid == 0 || rtcm->obsflag)
        {
            rtcm->staid = staid;
        }
    else if (staid != rtcm->staid)
        {
            type = getbitu(rtcm->buff, 24, 12);
            trace(2, "rtcm3 %d staid invalid id=%d %d\n", type, staid, rtcm->staid);

            /* reset station id if station id error */
            rtcm->staid = 0;
            return 0;
        }
    return 1;
}


/* decode type 1001-1004 message header --------------------------------------*/
int decode_head1001(rtcm_t *rtcm, int *sync)
{
    double tow;
    char *msg;
    int i = 24;
    int staid;
    int nsat;
    int type;

    type = getbitu(rtcm->buff, i, 12);
    i += 12;

    if (i + 52 <= rtcm->len * 8)
        {
            staid = getbitu(rtcm->buff, i, 12);
            i += 12;
            tow = getbitu(rtcm->buff, i, 30) * 0.001;
            i += 30;
            *sync = getbitu(rtcm->buff, i, 1);
            i += 1;
            nsat = getbitu(rtcm->buff, i, 5);
        }
    else
        {
            trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
            return -1;
        }
    /* test station id */
    if (!test_staid(rtcm, staid))
        {
            return -1;
        }

    adjweek(rtcm, tow);

    trace(4, "decode_head1001: time=%s nsat=%d sync=%d\n", time_str(rtcm->time, 2),
        nsat, *sync);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " staid=%4d %s nsat=%2d sync=%d", staid,
                time_str(rtcm->time, 2), nsat, *sync);
        }
    return nsat;
}


/* decode type 1001: L1-only gps rtk observation -----------------------------*/
int decode_type1001(rtcm_t *rtcm)
{
    int sync;
    if (decode_head1001(rtcm, &sync) < 0)
        {
            return -1;
        }
    rtcm->obsflag = !sync;
    return sync ? 0 : 1;
}


/* decode type 1002: extended L1-only gps rtk observables --------------------*/
int decode_type1002(rtcm_t *rtcm)
{
    double pr1;
    double cnr1;
    double tt;
    double cp1;
    int i = 24 + 64;
    int j;
    int index;
    int nsat;
    int sync;
    int prn;
    int code;
    int sat;
    int ppr1;
    int lock1;
    int amb;
    int sys;

    if ((nsat = decode_head1001(rtcm, &sync)) < 0)
        {
            return -1;
        }

    for (j = 0; j < nsat && rtcm->obs.n < MAXOBS && i + 74 <= rtcm->len * 8; j++)
        {
            prn = getbitu(rtcm->buff, i, 6);
            i += 6;
            code = getbitu(rtcm->buff, i, 1);
            i += 1;
            pr1 = getbitu(rtcm->buff, i, 24);
            i += 24;
            ppr1 = getbits(rtcm->buff, i, 20);
            i += 20;
            lock1 = getbitu(rtcm->buff, i, 7);
            i += 7;
            amb = getbitu(rtcm->buff, i, 8);
            i += 8;
            cnr1 = getbitu(rtcm->buff, i, 8);
            i += 8;
            if (prn < 40)
                {
                    sys = SYS_GPS;
                }
            else
                {
                    sys = SYS_SBS;
                    prn += 80;
                }
            if (!(sat = satno(sys, prn)))
                {
                    trace(2, "rtcm3 1002 satellite number error: prn=%d\n", prn);
                    continue;
                }
            tt = timediff(rtcm->obs.data[0].time, rtcm->time);
            if (rtcm->obsflag || fabs(tt) > 1E-9)
                {
                    rtcm->obs.n = rtcm->obsflag = 0;
                }
            if ((index = obsindex3(&rtcm->obs, rtcm->time, sat)) < 0)
                {
                    continue;
                }
            pr1 = pr1 * 0.02 + amb * PRUNIT_GPS;
            if (ppr1 != static_cast<int>(0xFFF80000))
                {
                    rtcm->obs.data[index].P[0] = pr1;
                    cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 / LAM_CARR[0]);
                    rtcm->obs.data[index].L[0] = pr1 / LAM_CARR[0] + cp1;
                }
            rtcm->obs.data[index].LLI[0] = lossoflock(rtcm, sat, 0, lock1);
            rtcm->obs.data[index].SNR[0] = snratio(cnr1 * 0.25);
            rtcm->obs.data[index].code[0] = code ? CODE_L1P : CODE_L1C;
        }
    return sync ? 0 : 1;
}


/* decode type 1003: L1&L2 gps rtk observables -------------------------------*/
int decode_type1003(rtcm_t *rtcm)
{
    int sync;
    if (decode_head1001(rtcm, &sync) < 0)
        {
            return -1;
        }
    rtcm->obsflag = !sync;
    return sync ? 0 : 1;
}


/* decode type 1004: extended L1&L2 gps rtk observables ----------------------*/
int decode_type1004(rtcm_t *rtcm)
{
    const int L2codes[] = {CODE_L2X, CODE_L2P, CODE_L2D, CODE_L2W};
    double pr1;
    double cnr1;
    double cnr2;
    double tt;
    double cp1;
    double cp2;
    int i = 24 + 64;
    int j;
    int index;
    int nsat;
    int sync;
    int prn;
    int sat;
    int code1;
    int code2;
    int pr21;
    int ppr1;
    int ppr2;
    int lock1;
    int lock2;
    int amb;
    int sys;

    if ((nsat = decode_head1001(rtcm, &sync)) < 0)
        {
            return -1;
        }

    for (j = 0; j < nsat && rtcm->obs.n < MAXOBS && i + 125 <= rtcm->len * 8; j++)
        {
            prn = getbitu(rtcm->buff, i, 6);
            i += 6;
            code1 = getbitu(rtcm->buff, i, 1);
            i += 1;
            pr1 = getbitu(rtcm->buff, i, 24);
            i += 24;
            ppr1 = getbits(rtcm->buff, i, 20);
            i += 20;
            lock1 = getbitu(rtcm->buff, i, 7);
            i += 7;
            amb = getbitu(rtcm->buff, i, 8);
            i += 8;
            cnr1 = getbitu(rtcm->buff, i, 8);
            i += 8;
            code2 = getbitu(rtcm->buff, i, 2);
            i += 2;
            pr21 = getbits(rtcm->buff, i, 14);
            i += 14;
            ppr2 = getbits(rtcm->buff, i, 20);
            i += 20;
            lock2 = getbitu(rtcm->buff, i, 7);
            i += 7;
            cnr2 = getbitu(rtcm->buff, i, 8);
            i += 8;
            if (prn < 40)
                {
                    sys = SYS_GPS;
                }
            else
                {
                    sys = SYS_SBS;
                    prn += 80;
                }
            if (!(sat = satno(sys, prn)))
                {
                    trace(2, "rtcm3 1004 satellite number error: sys=%d prn=%d\n", sys, prn);
                    continue;
                }
            tt = timediff(rtcm->obs.data[0].time, rtcm->time);
            if (rtcm->obsflag || fabs(tt) > 1E-9)
                {
                    rtcm->obs.n = rtcm->obsflag = 0;
                }
            if ((index = obsindex3(&rtcm->obs, rtcm->time, sat)) < 0)
                {
                    continue;
                }
            pr1 = pr1 * 0.02 + amb * PRUNIT_GPS;
            if (ppr1 != static_cast<int>(0xFFF80000))
                {
                    rtcm->obs.data[index].P[0] = pr1;
                    cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 / LAM_CARR[0]);
                    rtcm->obs.data[index].L[0] = pr1 / LAM_CARR[0] + cp1;
                }
            rtcm->obs.data[index].LLI[0] = lossoflock(rtcm, sat, 0, lock1);
            rtcm->obs.data[index].SNR[0] = snratio(cnr1 * 0.25);
            rtcm->obs.data[index].code[0] = code1 ? CODE_L1P : CODE_L1C;

            if (pr21 != static_cast<int>(0xFFFFE000))
                {
                    rtcm->obs.data[index].P[1] = pr1 + pr21 * 0.02;
                }
            if (ppr2 != static_cast<int>(0xFFF80000))
                {
                    cp2 = adjcp(rtcm, sat, 1, ppr2 * 0.0005 / LAM_CARR[1]);
                    rtcm->obs.data[index].L[1] = pr1 / LAM_CARR[1] + cp2;
                }
            rtcm->obs.data[index].LLI[1] = lossoflock(rtcm, sat, 1, lock2);
            rtcm->obs.data[index].SNR[1] = snratio(cnr2 * 0.25);
            rtcm->obs.data[index].code[1] = L2codes[code2];
        }
    rtcm->obsflag = !sync;
    return sync ? 0 : 1;
}


/* get signed 38bit field ----------------------------------------------------*/
double getbits_38(const unsigned char *buff, int pos)
{
    return static_cast<double>(getbits(buff, pos, 32)) * 64.0 + getbitu(buff, pos + 32, 6);
}


/* decode type 1005: stationary rtk reference station arp --------------------*/
int decode_type1005(rtcm_t *rtcm)
{
    double rr[3];
    double re[3];
    double pos[3];
    char *msg;
    int i = 24 + 12;
    int j;
    int staid;
    int itrf;

    if (i + 140 == rtcm->len * 8)
        {
            staid = getbitu(rtcm->buff, i, 12);
            i += 12;
            itrf = getbitu(rtcm->buff, i, 6);
            i += 6 + 4;
            rr[0] = getbits_38(rtcm->buff, i);
            i += 38 + 2;
            rr[1] = getbits_38(rtcm->buff, i);
            i += 38 + 2;
            rr[2] = getbits_38(rtcm->buff, i);
        }
    else
        {
            trace(2, "rtcm3 1005 length error: len=%d\n", rtcm->len);
            return -1;
        }
    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            for (j = 0; j < 3; j++)
                {
                    re[j] = rr[j] * 0.0001;
                }
            ecef2pos(re, pos);
            std::snprintf(msg, sizeof(rtcm->msgtype), " staid=%4d pos=%.8f %.8f %.3f", staid, pos[0] * R2D, pos[1] * R2D,
                pos[2]);
        }
    /* test station id */
    if (!test_staid(rtcm, staid))
        {
            return -1;
        }

    rtcm->sta.deltype = 0; /* xyz */
    for (j = 0; j < 3; j++)
        {
            rtcm->sta.pos[j] = rr[j] * 0.0001;
            rtcm->sta.del[j] = 0.0;
        }
    rtcm->sta.hgt = 0.0;
    rtcm->sta.itrf = itrf;
    return 5;
}


/* decode type 1006: stationary rtk reference station arp with height --------*/
int decode_type1006(rtcm_t *rtcm)
{
    double rr[3];
    double re[3];
    double pos[3];
    double anth;
    char *msg;
    int i = 24 + 12;
    int j;
    int staid;
    int itrf;

    if (i + 156 <= rtcm->len * 8)
        {
            staid = getbitu(rtcm->buff, i, 12);
            i += 12;
            itrf = getbitu(rtcm->buff, i, 6);
            i += 6 + 4;
            rr[0] = getbits_38(rtcm->buff, i);
            i += 38 + 2;
            rr[1] = getbits_38(rtcm->buff, i);
            i += 38 + 2;
            rr[2] = getbits_38(rtcm->buff, i);
            i += 38;
            anth = getbitu(rtcm->buff, i, 16);
        }
    else
        {
            trace(2, "rtcm3 1006 length error: len=%d\n", rtcm->len);
            return -1;
        }
    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            for (j = 0; j < 3; j++)
                {
                    re[j] = rr[j] * 0.0001;
                }
            ecef2pos(re, pos);
            std::snprintf(msg, sizeof(rtcm->msgtype), " staid=%4d pos=%.8f %.8f %.3f anth=%.3f", staid, pos[0] * R2D,
                pos[1] * R2D, pos[2], anth);
        }
    /* test station id */
    if (!test_staid(rtcm, staid))
        {
            return -1;
        }

    rtcm->sta.deltype = 1; /* xyz */
    for (j = 0; j < 3; j++)
        {
            rtcm->sta.pos[j] = rr[j] * 0.0001;
            rtcm->sta.del[j] = 0.0;
        }
    rtcm->sta.hgt = anth * 0.0001;
    rtcm->sta.itrf = itrf;
    return 5;
}


/* decode type 1007: antenna descriptor --------------------------------------*/
int decode_type1007(rtcm_t *rtcm)
{
    char des[32] = "";
    char *msg;
    int i = 24 + 12;
    int j;
    int staid;
    int n;
    int setup;

    n = getbitu(rtcm->buff, i + 12, 8);

    if (i + 28 + 8 * n <= rtcm->len * 8)
        {
            staid = getbitu(rtcm->buff, i, 12);
            i += 12 + 8;
            for (j = 0; j < n && j < 31; j++)
                {
                    des[j] = static_cast<char>(getbitu(rtcm->buff, i, 8));
                    i += 8;
                }
            setup = getbitu(rtcm->buff, i, 8);
        }
    else
        {
            trace(2, "rtcm3 1007 length error: len=%d\n", rtcm->len);
            return -1;
        }
    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " staid=%4d", staid);
        }
    /* test station id */
    if (!test_staid(rtcm, staid))
        {
            return -1;
        }

    strncpy(rtcm->sta.antdes, des, n);
    rtcm->sta.antdes[n] = '\0';
    rtcm->sta.antsetup = setup;
    rtcm->sta.antsno[0] = '\0';
    return 5;
}


/* decode type 1008: antenna descriptor & serial number ----------------------*/
int decode_type1008(rtcm_t *rtcm)
{
    char des[32] = "";
    char sno[32] = "";
    char *msg;
    int i = 24 + 12;
    int j;
    int staid;
    int n;
    int m;
    int setup;

    n = getbitu(rtcm->buff, i + 12, 8);
    m = getbitu(rtcm->buff, i + 28 + 8 * n, 8);

    if (i + 36 + 8 * (n + m) <= rtcm->len * 8)
        {
            staid = getbitu(rtcm->buff, i, 12);
            i += 12 + 8;
            for (j = 0; j < n && j < 31; j++)
                {
                    des[j] = static_cast<char>(getbitu(rtcm->buff, i, 8));
                    i += 8;
                }
            setup = getbitu(rtcm->buff, i, 8);
            i += 8 + 8;
            for (j = 0; j < m && j < 31; j++)
                {
                    sno[j] = static_cast<char>(getbitu(rtcm->buff, i, 8));
                    i += 8;
                }
        }
    else
        {
            trace(2, "rtcm3 1008 length error: len=%d\n", rtcm->len);
            return -1;
        }
    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " staid=%4d", staid);
        }
    /* test station id */
    if (!test_staid(rtcm, staid))
        {
            return -1;
        }

    strncpy(rtcm->sta.antdes, des, n);
    rtcm->sta.antdes[n] = '\0';
    rtcm->sta.antsetup = setup;
    strncpy(rtcm->sta.antsno, sno, m);
    rtcm->sta.antsno[m] = '\0';
    return 5;
}


/* decode type 1009-1012 message header --------------------------------------*/
int decode_head1009(rtcm_t *rtcm, int *sync)
{
    double tod;
    char *msg;
    int i = 24;
    int staid;
    int nsat;
    int type;

    type = getbitu(rtcm->buff, i, 12);
    i += 12;

    if (i + 49 <= rtcm->len * 8)
        {
            staid = getbitu(rtcm->buff, i, 12);
            i += 12;
            tod = getbitu(rtcm->buff, i, 27) * 0.001;
            i += 27; /* sec in a day */
            *sync = getbitu(rtcm->buff, i, 1);
            i += 1;
            nsat = getbitu(rtcm->buff, i, 5);
        }
    else
        {
            trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
            return -1;
        }
    /* test station id */
    if (!test_staid(rtcm, staid))
        {
            return -1;
        }

    adjday_glot(rtcm, tod);

    trace(4, "decode_head1009: time=%s nsat=%d sync=%d\n", time_str(rtcm->time, 2),
        nsat, *sync);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " staid=%4d %s nsat=%2d sync=%d", staid,
                time_str(rtcm->time, 2), nsat, *sync);
        }
    return nsat;
}


/* decode type 1009: L1-only glonass rtk observables -------------------------*/
int decode_type1009(rtcm_t *rtcm)
{
    int sync;
    if (decode_head1009(rtcm, &sync) < 0)
        {
            return -1;
        }
    rtcm->obsflag = !sync;
    return sync ? 0 : 1;
}


/* decode type 1010: extended L1-only glonass rtk observables ----------------*/
int decode_type1010(rtcm_t *rtcm)
{
    double pr1;
    double cnr1;
    double tt;
    double cp1;
    double lam1;
    int i = 24 + 61;
    int j;
    int index;
    int nsat;
    int sync;
    int prn;
    int sat;
    int code;
    int freq;
    int ppr1;
    int lock1;
    int amb;
    int sys = SYS_GLO;

    if ((nsat = decode_head1009(rtcm, &sync)) < 0)
        {
            return -1;
        }

    for (j = 0; j < nsat && rtcm->obs.n < MAXOBS && i + 79 <= rtcm->len * 8; j++)
        {
            prn = getbitu(rtcm->buff, i, 6);
            i += 6;
            code = getbitu(rtcm->buff, i, 1);
            i += 1;
            freq = getbitu(rtcm->buff, i, 5);
            i += 5;
            pr1 = getbitu(rtcm->buff, i, 25);
            i += 25;
            ppr1 = getbits(rtcm->buff, i, 20);
            i += 20;
            lock1 = getbitu(rtcm->buff, i, 7);
            i += 7;
            amb = getbitu(rtcm->buff, i, 7);
            i += 7;
            cnr1 = getbitu(rtcm->buff, i, 8);
            i += 8;
            if (!(sat = satno(sys, prn)))
                {
                    trace(2, "rtcm3 1010 satellite number error: prn=%d\n", prn);
                    continue;
                }
            tt = timediff(rtcm->obs.data[0].time, rtcm->time);
            if (rtcm->obsflag || fabs(tt) > 1E-9)
                {
                    rtcm->obs.n = rtcm->obsflag = 0;
                }
            if ((index = obsindex3(&rtcm->obs, rtcm->time, sat)) < 0)
                {
                    continue;
                }
            pr1 = pr1 * 0.02 + amb * PRUNIT_GLO;
            if (ppr1 != static_cast<int>(0xFFF80000))
                {
                    rtcm->obs.data[index].P[0] = pr1;
                    lam1 = SPEED_OF_LIGHT_M_S / (FREQ1_GLO + DFRQ1_GLO * (freq - 7));
                    cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 / lam1);
                    rtcm->obs.data[index].L[0] = pr1 / lam1 + cp1;
                }
            rtcm->obs.data[index].LLI[0] = lossoflock(rtcm, sat, 0, lock1);
            rtcm->obs.data[index].SNR[0] = snratio(cnr1 * 0.25);
            rtcm->obs.data[index].code[0] = code ? CODE_L1P : CODE_L1C;
        }
    return sync ? 0 : 1;
}


/* decode type 1011: L1&L2 glonass rtk observables ---------------------------*/
int decode_type1011(rtcm_t *rtcm)
{
    int sync;
    if (decode_head1009(rtcm, &sync) < 0)
        {
            return -1;
        }
    rtcm->obsflag = !sync;
    return sync ? 0 : 1;
}


/* decode type 1012: extended L1&L2 glonass rtk observables ------------------*/
int decode_type1012(rtcm_t *rtcm)
{
    double pr1;
    double cnr1;
    double cnr2;
    double tt;
    double cp1;
    double cp2;
    double lam1;
    double lam2;
    int i = 24 + 61;
    int j;
    int index;
    int nsat;
    int sync;
    int prn;
    int sat;
    int freq;
    int code1;
    int code2;
    int pr21;
    int ppr1;
    int ppr2;
    int lock1;
    int lock2;
    int amb;
    int sys = SYS_GLO;

    if ((nsat = decode_head1009(rtcm, &sync)) < 0)
        {
            return -1;
        }

    for (j = 0; j < nsat && rtcm->obs.n < MAXOBS && i + 130 <= rtcm->len * 8; j++)
        {
            prn = getbitu(rtcm->buff, i, 6);
            i += 6;
            code1 = getbitu(rtcm->buff, i, 1);
            i += 1;
            freq = getbitu(rtcm->buff, i, 5);
            i += 5;
            pr1 = getbitu(rtcm->buff, i, 25);
            i += 25;
            ppr1 = getbits(rtcm->buff, i, 20);
            i += 20;
            lock1 = getbitu(rtcm->buff, i, 7);
            i += 7;
            amb = getbitu(rtcm->buff, i, 7);
            i += 7;
            cnr1 = getbitu(rtcm->buff, i, 8);
            i += 8;
            code2 = getbitu(rtcm->buff, i, 2);
            i += 2;
            pr21 = getbits(rtcm->buff, i, 14);
            i += 14;
            ppr2 = getbits(rtcm->buff, i, 20);
            i += 20;
            lock2 = getbitu(rtcm->buff, i, 7);
            i += 7;
            cnr2 = getbitu(rtcm->buff, i, 8);
            i += 8;
            if (!(sat = satno(sys, prn)))
                {
                    trace(2, "rtcm3 1012 satellite number error: sys=%d prn=%d\n", sys, prn);
                    continue;
                }
            tt = timediff(rtcm->obs.data[0].time, rtcm->time);
            if (rtcm->obsflag || fabs(tt) > 1E-9)
                {
                    rtcm->obs.n = rtcm->obsflag = 0;
                }
            if ((index = obsindex3(&rtcm->obs, rtcm->time, sat)) < 0)
                {
                    continue;
                }
            pr1 = pr1 * 0.02 + amb * PRUNIT_GLO;
            if (ppr1 != static_cast<int>(0xFFF80000))
                {
                    lam1 = SPEED_OF_LIGHT_M_S / (FREQ1_GLO + DFRQ1_GLO * (freq - 7));
                    rtcm->obs.data[index].P[0] = pr1;
                    cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 / lam1);
                    rtcm->obs.data[index].L[0] = pr1 / lam1 + cp1;
                }
            rtcm->obs.data[index].LLI[0] = lossoflock(rtcm, sat, 0, lock1);
            rtcm->obs.data[index].SNR[0] = snratio(cnr1 * 0.25);
            rtcm->obs.data[index].code[0] = code1 ? CODE_L1P : CODE_L1C;

            if (pr21 != static_cast<int>(0xFFFFE000))
                {
                    rtcm->obs.data[index].P[1] = pr1 + pr21 * 0.02;
                }
            if (ppr2 != static_cast<int>(0xFFF80000))
                {
                    lam2 = SPEED_OF_LIGHT_M_S / (FREQ2_GLO + DFRQ2_GLO * (freq - 7));
                    cp2 = adjcp(rtcm, sat, 1, ppr2 * 0.0005 / lam2);
                    rtcm->obs.data[index].L[1] = pr1 / lam2 + cp2;
                }
            rtcm->obs.data[index].LLI[1] = lossoflock(rtcm, sat, 1, lock2);
            rtcm->obs.data[index].SNR[1] = snratio(cnr2 * 0.25);
            rtcm->obs.data[index].code[1] = code2 ? CODE_L2P : CODE_L2C;
        }
    rtcm->obsflag = !sync;
    return sync ? 0 : 1;
}


/* decode type 1013: system parameters ---------------------------------------*/
int decode_type1013(rtcm_t *rtcm __attribute__((unused)))
{
    return 0;
}


/* decode type 1019: gps ephemerides -----------------------------------------*/
int decode_type1019(rtcm_t *rtcm, bool pre_2009_file)
{
    eph_t eph = {0, -1, -1, 0, 0, 0, 0, 0, {0, 0.0}, {0, 0.0}, {0, 0.0},
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {0.0}, {0.0}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
    double toc;
    double sqrtA;
    char *msg;
    int i = 24 + 12;
    int prn;
    int sat;
    int week;
    int sys = SYS_GPS;

    if (i + 476 <= rtcm->len * 8)
        {
            prn = getbitu(rtcm->buff, i, 6);
            i += 6;
            week = getbitu(rtcm->buff, i, 10);
            i += 10;
            eph.sva = getbitu(rtcm->buff, i, 4);
            i += 4;
            eph.code = getbitu(rtcm->buff, i, 2);
            i += 2;
            eph.idot = getbits(rtcm->buff, i, 14) * TWO_N43 * SC2RAD;
            i += 14;
            eph.iode = getbitu(rtcm->buff, i, 8);
            i += 8;
            toc = getbitu(rtcm->buff, i, 16) * 16.0;
            i += 16;
            eph.f2 = getbits(rtcm->buff, i, 8) * TWO_N55;
            i += 8;
            eph.f1 = getbits(rtcm->buff, i, 16) * TWO_N43;
            i += 16;
            eph.f0 = getbits(rtcm->buff, i, 22) * TWO_N31;
            i += 22;
            eph.iodc = getbitu(rtcm->buff, i, 10);
            i += 10;
            eph.crs = getbits(rtcm->buff, i, 16) * TWO_N5;
            i += 16;
            eph.deln = getbits(rtcm->buff, i, 16) * TWO_N43 * SC2RAD;
            i += 16;
            eph.M0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.cuc = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.e = getbitu(rtcm->buff, i, 32) * TWO_N33;
            i += 32;
            eph.cus = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            sqrtA = getbitu(rtcm->buff, i, 32) * TWO_N19;
            i += 32;
            eph.toes = getbitu(rtcm->buff, i, 16) * 16.0;
            i += 16;
            eph.cic = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.OMG0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.cis = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.i0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.crc = getbits(rtcm->buff, i, 16) * TWO_N5;
            i += 16;
            eph.omg = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.OMGd = getbits(rtcm->buff, i, 24) * TWO_N43 * SC2RAD;
            i += 24;
            eph.tgd[0] = getbits(rtcm->buff, i, 8) * TWO_N31;
            i += 8;
            eph.svh = getbitu(rtcm->buff, i, 6);
            i += 6;
            eph.flag = getbitu(rtcm->buff, i, 1);
            i += 1;
            eph.fit = getbitu(rtcm->buff, i, 1) ? 0.0 : 4.0; /* 0:4hr, 1:>4hr */
        }
    else
        {
            trace(2, "rtcm3 1019 length error: len=%d\n", rtcm->len);
            return -1;
        }
    if (prn >= 40)
        {
            sys = SYS_SBS;
            prn += 80;
        }
    trace(4, "decode_type1019: prn=%d iode=%d toe=%.0f\n", prn, eph.iode, eph.toes);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
                prn, eph.iode, eph.iodc, week, eph.toes, toc, eph.svh);
        }
    if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 1019 satellite number error: prn=%d\n", prn);
            return -1;
        }
    eph.sat = sat;
    eph.week = adjgpsweek(week, pre_2009_file);
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, toc);
    eph.ttr = rtcm->time;
    eph.A = sqrtA * sqrtA;
    if (!strstr(rtcm->opt, "-EPHALL"))
        {
            if (eph.iode == rtcm->nav.eph[sat - 1].iode)
                {
                    return 0; /* unchanged */
                }
        }
    rtcm->nav.eph[sat - 1] = eph;
    rtcm->ephsat = sat;
    return 2;
}


/* decode type 1020: glonass ephemerides -------------------------------------*/
int decode_type1020(rtcm_t *rtcm)
{
    geph_t geph = {0, -1, 0, 0, 0, 0, {0, 0.0}, {0, 0.0}, {0.0}, {0.0}, {0.0},
        0.0, 0.0, 0.0};
    double tk_h;
    double tk_m;
    double tk_s;
    double toe;
    double tow;
    double tod;
    double tof;
    char *msg;
    int i = 24 + 12;
    int prn;
    int sat;
    int week;
    int tb;
    int bn;
    int sys = SYS_GLO;

    if (i + 348 <= rtcm->len * 8)
        {
            prn = getbitu(rtcm->buff, i, 6);
            i += 6;
            geph.frq = getbitu(rtcm->buff, i, 5) - 7;
            i += 5 + 2 + 2;
            tk_h = getbitu(rtcm->buff, i, 5);
            i += 5;
            tk_m = getbitu(rtcm->buff, i, 6);
            i += 6;
            tk_s = getbitu(rtcm->buff, i, 1) * 30.0;
            i += 1;
            bn = getbitu(rtcm->buff, i, 1);
            i += 1 + 1;
            tb = getbitu(rtcm->buff, i, 7);
            i += 7;
            geph.vel[0] = getbitg(rtcm->buff, i, 24) * TWO_N20 * 1E3;
            i += 24;
            geph.pos[0] = getbitg(rtcm->buff, i, 27) * TWO_N11 * 1E3;
            i += 27;
            geph.acc[0] = getbitg(rtcm->buff, i, 5) * TWO_N30 * 1E3;
            i += 5;
            geph.vel[1] = getbitg(rtcm->buff, i, 24) * TWO_N20 * 1E3;
            i += 24;
            geph.pos[1] = getbitg(rtcm->buff, i, 27) * TWO_N11 * 1E3;
            i += 27;
            geph.acc[1] = getbitg(rtcm->buff, i, 5) * TWO_N30 * 1E3;
            i += 5;
            geph.vel[2] = getbitg(rtcm->buff, i, 24) * TWO_N20 * 1E3;
            i += 24;
            geph.pos[2] = getbitg(rtcm->buff, i, 27) * TWO_N11 * 1E3;
            i += 27;
            geph.acc[2] = getbitg(rtcm->buff, i, 5) * TWO_N30 * 1E3;
            i += 5 + 1;
            geph.gamn = getbitg(rtcm->buff, i, 11) * TWO_N40;
            i += 11 + 3;
            geph.taun = getbitg(rtcm->buff, i, 22) * TWO_N30;
        }
    else
        {
            trace(2, "rtcm3 1020 length error: len=%d\n", rtcm->len);
            return -1;
        }
    if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 1020 satellite number error: prn=%d\n", prn);
            return -1;
        }
    trace(4, "decode_type1020: prn=%d tk=%02.0f:%02.0f:%02.0f\n", prn, tk_h, tk_m, tk_s);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " prn=%2d tk=%02.0f:%02.0f:%02.0f frq=%2d bn=%d tb=%d",
                prn, tk_h, tk_m, tk_s, geph.frq, bn, tb);
        }
    geph.sat = sat;
    geph.svh = bn;
    geph.iode = tb & 0x7F;
    if (rtcm->time.time == 0)
        {
            rtcm->time = utc2gpst(timeget());
        }
    tow = time2gpst(gpst2utc(rtcm->time), &week);
    tod = fmod(tow, 86400.0);
    tow -= tod;
    tof = tk_h * 3600.0 + tk_m * 60.0 + tk_s - 10800.0; /* lt->utc */
    if (tof < tod - 43200.0)
        {
            tof += 86400.0;
        }
    else if (tof > tod + 43200.0)
        {
            tof -= 86400.0;
        }
    geph.tof = utc2gpst(gpst2time(week, tow + tof));
    toe = tb * 900.0 - 10800.0; /* lt->utc */
    if (toe < tod - 43200.0)
        {
            toe += 86400.0;
        }
    else if (toe > tod + 43200.0)
        {
            toe -= 86400.0;
        }
    geph.toe = utc2gpst(gpst2time(week, tow + toe)); /* utc->gpst */

    if (!strstr(rtcm->opt, "-EPHALL"))
        {
            if (fabs(timediff(geph.toe, rtcm->nav.geph[prn - 1].toe)) < 1.0 &&
                geph.svh == rtcm->nav.geph[prn - 1].svh)
                {
                    return 0; /* unchanged */
                }
        }
    rtcm->nav.geph[prn - 1] = geph;
    rtcm->ephsat = sat;
    return 2;
}


/* decode type 1021: helmert/abridged molodenski -----------------------------*/
int decode_type1021(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1021: not supported message\n");
    return 0;
}


/* decode type 1022: moledenski-badekas transformation -----------------------*/
int decode_type1022(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1022: not supported message\n");
    return 0;
}


/* decode type 1023: residual, ellipoidal grid representation ----------------*/
int decode_type1023(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1023: not supported message\n");
    return 0;
}


/* decode type 1024: residual, plane grid representation ---------------------*/
int decode_type1024(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1024: not supported message\n");
    return 0;
}


/* decode type 1025: projection (types except LCC2SP, OM) ---------------------*/
int decode_type1025(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1025: not supported message\n");
    return 0;
}


/* decode type 1026: projection (LCC2SP - lambert conic conformal (2sp)) -----*/
int decode_type1026(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1026: not supported message\n");
    return 0;
}


/* decode type 1027: projection (type OM - oblique mercator) -----------------*/
int decode_type1027(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1027: not supported message\n");
    return 0;
}


/* decode type 1029: unicode text string -------------------------------------*/
int decode_type1029(rtcm_t *rtcm)
{
    char *msg;
    int i = 24 + 12;
    int j;
    int staid;
    int nchar;  // mjd, tod, nchar, cunit;

    if (i + 60 <= rtcm->len * 8)
        {
            staid = getbitu(rtcm->buff, i, 12);
            i += 12;
            /* mjd   = getbitu(rtcm->buff, i, 16); */ i += 16;
            /* tod   = getbitu(rtcm->buff, i, 17); */ i += 17;
            nchar = getbitu(rtcm->buff, i, 7);
            i += 7;
            /* cunit = getbitu(rtcm->buff, i, 8); */ i += 8;
        }
    else
        {
            trace(2, "rtcm3 1029 length error: len=%d\n", rtcm->len);
            return -1;
        }
    if (i + nchar * 8 > rtcm->len * 8)
        {
            trace(2, "rtcm3 1029 length error: len=%d nchar=%d\n", rtcm->len, nchar);
            return -1;
        }
    for (j = 0; j < nchar && j < 126; j++)
        {
            rtcm->msg[j] = getbitu(rtcm->buff, i, 8);
            i += 8;
        }
    rtcm->msg[j] = '\0';

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " staid=%4d text=%s", staid, rtcm->msg);
        }
    return 0;
}


/* decode type 1030: network rtk residual ------------------------------------*/
int decode_type1030(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1030: not supported message\n");
    return 0;
}


/* decode type 1031: glonass network rtk residual ----------------------------*/
int decode_type1031(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1031: not supported message\n");
    return 0;
}


/* decode type 1032: physical reference station position information ---------*/
int decode_type1032(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1032: not supported message\n");
    return 0;
}


/* decode type 1033: receiver and antenna descriptor -------------------------*/
int decode_type1033(rtcm_t *rtcm)
{
    char des[32] = "";
    char sno[32] = "";
    char rec[32] = "";
    char ver[32] = "";
    char rsn[32] = "";
    char *msg;
    int i = 24 + 12;
    int j;
    int staid;
    int n;
    int m;
    int n1;
    int n2;
    int n3;
    int setup;

    n = getbitu(rtcm->buff, i + 12, 8);
    m = getbitu(rtcm->buff, i + 28 + 8 * n, 8);
    n1 = getbitu(rtcm->buff, i + 36 + 8 * (n + m), 8);
    n2 = getbitu(rtcm->buff, i + 44 + 8 * (n + m + n1), 8);
    n3 = getbitu(rtcm->buff, i + 52 + 8 * (n + m + n1 + n2), 8);

    if (i + 60 + 8 * (n + m + n1 + n2 + n3) <= rtcm->len * 8)
        {
            staid = getbitu(rtcm->buff, i, 12);
            i += 12 + 8;
            for (j = 0; j < n && j < 31; j++)
                {
                    des[j] = static_cast<char>(getbitu(rtcm->buff, i, 8));
                    i += 8;
                }
            setup = getbitu(rtcm->buff, i, 8);
            i += 8 + 8;
            for (j = 0; j < m && j < 31; j++)
                {
                    sno[j] = static_cast<char>(getbitu(rtcm->buff, i, 8));
                    i += 8;
                }
            i += 8;
            for (j = 0; j < n1 && j < 31; j++)
                {
                    rec[j] = static_cast<char>(getbitu(rtcm->buff, i, 8));
                    i += 8;
                }
            i += 8;
            for (j = 0; j < n2 && j < 31; j++)
                {
                    ver[j] = static_cast<char>(getbitu(rtcm->buff, i, 8));
                    i += 8;
                }
            i += 8;
            for (j = 0; j < n3 && j < 31; j++)
                {
                    rsn[j] = static_cast<char>(getbitu(rtcm->buff, i, 8));
                    i += 8;
                }
        }
    else
        {
            trace(2, "rtcm3 1033 length error: len=%d\n", rtcm->len);
            return -1;
        }
    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " staid=%4d", staid);
        }
    /* test station id */
    if (!test_staid(rtcm, staid))
        {
            return -1;
        }

    strncpy(rtcm->sta.antdes, des, n);
    rtcm->sta.antdes[n] = '\0';
    rtcm->sta.antsetup = setup;
    strncpy(rtcm->sta.antsno, sno, m);
    rtcm->sta.antsno[m] = '\0';
    strncpy(rtcm->sta.rectype, rec, n1);
    rtcm->sta.rectype[n1] = '\0';
    strncpy(rtcm->sta.recver, ver, n2);
    rtcm->sta.recver[n2] = '\0';
    strncpy(rtcm->sta.recsno, rsn, n3);
    rtcm->sta.recsno[n3] = '\0';

    trace(3, "rtcm3 1033: ant=%s:%s rec=%s:%s:%s\n", des, sno, rec, ver, rsn);
    return 5;
}


/* decode type 1034: gps network fkp gradient --------------------------------*/
int decode_type1034(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1034: not supported message\n");
    return 0;
}


/* decode type 1035: glonass network fkp gradient ----------------------------*/
int decode_type1035(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1035: not supported message\n");
    return 0;
}


/* decode type 1037: glonass network rtk ionospheric correction difference ---*/
int decode_type1037(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1037: not supported message\n");
    return 0;
}


/* decode type 1038: glonass network rtk geometic correction difference ------*/
int decode_type1038(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1038: not supported message\n");
    return 0;
}


/* decode type 1039: glonass network rtk combined correction difference ------*/
int decode_type1039(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1039: not supported message\n");
    return 0;
}


/* decode type 1044: qzss ephemerides (ref [15]) -----------------------------*/
int decode_type1044(rtcm_t *rtcm, bool pre_2009_file)
{
    eph_t eph = {0, -1, -1, 0, 0, 0, 0, 0, {0, 0.0}, {0, 0.0}, {0, 0.0},
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {0.0}, {0.0}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
    double toc;
    double sqrtA;
    char *msg;
    int i = 24 + 12;
    int prn;
    int sat;
    int week;
    int sys = SYS_QZS;

    if (i + 473 <= rtcm->len * 8)
        {
            prn = getbitu(rtcm->buff, i, 4) + 192;
            i += 4;
            toc = getbitu(rtcm->buff, i, 16) * 16.0;
            i += 16;
            eph.f2 = getbits(rtcm->buff, i, 8) * TWO_N55;
            i += 8;
            eph.f1 = getbits(rtcm->buff, i, 16) * TWO_N43;
            i += 16;
            eph.f0 = getbits(rtcm->buff, i, 22) * TWO_N31;
            i += 22;
            eph.iode = getbitu(rtcm->buff, i, 8);
            i += 8;
            eph.crs = getbits(rtcm->buff, i, 16) * TWO_N5;
            i += 16;
            eph.deln = getbits(rtcm->buff, i, 16) * TWO_N43 * SC2RAD;
            i += 16;
            eph.M0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.cuc = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.e = getbitu(rtcm->buff, i, 32) * TWO_N33;
            i += 32;
            eph.cus = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            sqrtA = getbitu(rtcm->buff, i, 32) * TWO_N19;
            i += 32;
            eph.toes = getbitu(rtcm->buff, i, 16) * 16.0;
            i += 16;
            eph.cic = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.OMG0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.cis = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.i0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.crc = getbits(rtcm->buff, i, 16) * TWO_N5;
            i += 16;
            eph.omg = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.OMGd = getbits(rtcm->buff, i, 24) * TWO_N43 * SC2RAD;
            i += 24;
            eph.idot = getbits(rtcm->buff, i, 14) * TWO_N43 * SC2RAD;
            i += 14;
            eph.code = getbitu(rtcm->buff, i, 2);
            i += 2;
            week = getbitu(rtcm->buff, i, 10);
            i += 10;
            eph.sva = getbitu(rtcm->buff, i, 4);
            i += 4;
            eph.svh = getbitu(rtcm->buff, i, 6);
            i += 6;
            eph.tgd[0] = getbits(rtcm->buff, i, 8) * TWO_N31;
            i += 8;
            eph.iodc = getbitu(rtcm->buff, i, 10);
            i += 10;
            eph.fit = getbitu(rtcm->buff, i, 1) ? 0.0 : 2.0; /* 0:2hr, 1:>2hr */
        }
    else
        {
            trace(2, "rtcm3 1044 length error: len=%d\n", rtcm->len);
            return -1;
        }
    trace(4, "decode_type1044: prn=%d iode=%d toe=%.0f\n", prn, eph.iode, eph.toes);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " prn=%3d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
                prn, eph.iode, eph.iodc, week, eph.toes, toc, eph.svh);
        }
    if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 1044 satellite number error: prn=%d\n", prn);
            return -1;
        }
    eph.sat = sat;
    eph.week = adjgpsweek(week, pre_2009_file);
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, toc);
    eph.ttr = rtcm->time;
    eph.A = sqrtA * sqrtA;
    if (!strstr(rtcm->opt, "-EPHALL"))
        {
            if (eph.iode == rtcm->nav.eph[sat - 1].iode &&
                eph.iodc == rtcm->nav.eph[sat - 1].iodc)
                {
                    return 0; /* unchanged */
                }
        }
    rtcm->nav.eph[sat - 1] = eph;
    rtcm->ephsat = sat;
    return 2;
}


/* decode type 1045: galileo satellite ephemerides (ref [15]) ----------------*/
int decode_type1045(rtcm_t *rtcm)
{
    eph_t eph = {0, -1, -1, 0, 0, 0, 0, 0, {0, 0.0}, {0, 0.0}, {0, 0.0},
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {0.0}, {0.0}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
    double toc;
    double sqrtA;
    char *msg;
    int i = 24 + 12;
    int prn;
    int sat;
    int week;
    int e5a_hs;
    int e5a_dvs;
    int sys = SYS_GAL;

    if (i + 484 <= rtcm->len * 8)
        {
            prn = getbitu(rtcm->buff, i, 6);
            i += 6;
            week = getbitu(rtcm->buff, i, 12);
            i += 12; /* gst-week */
            eph.iode = getbitu(rtcm->buff, i, 10);
            i += 10;
            eph.sva = getbitu(rtcm->buff, i, 8);
            i += 8;
            eph.idot = getbits(rtcm->buff, i, 14) * TWO_N43 * SC2RAD;
            i += 14;
            toc = getbitu(rtcm->buff, i, 14) * 60.0;
            i += 14;
            eph.f2 = getbits(rtcm->buff, i, 6) * TWO_N59;
            i += 6;
            eph.f1 = getbits(rtcm->buff, i, 21) * TWO_N46;
            i += 21;
            eph.f0 = getbits(rtcm->buff, i, 31) * TWO_N34;
            i += 31;
            eph.crs = getbits(rtcm->buff, i, 16) * TWO_N5;
            i += 16;
            eph.deln = getbits(rtcm->buff, i, 16) * TWO_N43 * SC2RAD;
            i += 16;
            eph.M0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.cuc = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.e = getbitu(rtcm->buff, i, 32) * TWO_N33;
            i += 32;
            eph.cus = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            sqrtA = getbitu(rtcm->buff, i, 32) * TWO_N19;
            i += 32;
            eph.toes = getbitu(rtcm->buff, i, 14) * 60.0;
            i += 14;
            eph.cic = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.OMG0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.cis = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.i0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.crc = getbits(rtcm->buff, i, 16) * TWO_N5;
            i += 16;
            eph.omg = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.OMGd = getbits(rtcm->buff, i, 24) * TWO_N43 * SC2RAD;
            i += 24;
            eph.tgd[0] = getbits(rtcm->buff, i, 10) * TWO_N32;
            i += 10; /* E5a/E1 */
            e5a_hs = getbitu(rtcm->buff, i, 2);
            i += 2; /* OSHS */
            e5a_dvs = getbitu(rtcm->buff, i, 1);
            i += 1; /* OSDVS */
            // rsv        = getbitu(rtcm->buff, i, 7);
        }
    else
        {
            trace(2, "rtcm3 1045 length error: len=%d\n", rtcm->len);
            return -1;
        }
    trace(4, "decode_type1045: prn=%d iode=%d toe=%.0f\n", prn, eph.iode, eph.toes);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f hs=%d dvs=%d",
                prn, eph.iode, week, eph.toes, toc, e5a_hs, e5a_dvs);
        }
    if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 1045 satellite number error: prn=%d\n", prn);
            return -1;
        }
    eph.sat = sat;
    eph.week = week + 1024; /* gal-week = gst-week + 1024 */
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, toc);
    eph.ttr = rtcm->time;
    eph.A = sqrtA * sqrtA;
    eph.svh = (e5a_hs << 4) + (e5a_dvs << 3);
    eph.code = 2; /* data source = f/nav e5a */
    if (!strstr(rtcm->opt, "-EPHALL"))
        {
            if (eph.iode == rtcm->nav.eph[sat - 1].iode)
                {
                    return 0; /* unchanged */
                }
        }
    rtcm->nav.eph[sat - 1] = eph;
    rtcm->ephsat = sat;
    return 2;
}


/* decode type 1046: galileo satellite ephemerides (extension for IGS MGEX) --*/
int decode_type1046(rtcm_t *rtcm)
{
    eph_t eph = {0, -1, -1, 0, 0, 0, 0, 0, {0, 0.0}, {0, 0.0}, {0, 0.0},
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {0.0}, {0.0}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
    double toc;
    double sqrtA;
    char *msg;
    int i = 24 + 12;
    int prn;
    int sat;
    int week;
    int e5a_hs;
    int e5a_dvs;
    int sys = SYS_GAL;

    if (i + 484 <= rtcm->len * 8)
        {
            prn = getbitu(rtcm->buff, i, 6);
            i += 6;
            week = getbitu(rtcm->buff, i, 12);
            i += 12; /* gst-week */
            eph.iode = getbitu(rtcm->buff, i, 10);
            i += 10;
            eph.sva = getbitu(rtcm->buff, i, 8);
            i += 8;
            eph.idot = getbits(rtcm->buff, i, 14) * TWO_N43 * SC2RAD;
            i += 14;
            toc = getbitu(rtcm->buff, i, 14) * 60.0;
            i += 14;
            eph.f2 = getbits(rtcm->buff, i, 6) * TWO_N59;
            i += 6;
            eph.f1 = getbits(rtcm->buff, i, 21) * TWO_N46;
            i += 21;
            eph.f0 = getbits(rtcm->buff, i, 31) * TWO_N34;
            i += 31;
            eph.crs = getbits(rtcm->buff, i, 16) * TWO_N5;
            i += 16;
            eph.deln = getbits(rtcm->buff, i, 16) * TWO_N43 * SC2RAD;
            i += 16;
            eph.M0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.cuc = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.e = getbitu(rtcm->buff, i, 32) * TWO_N33;
            i += 32;
            eph.cus = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            sqrtA = getbitu(rtcm->buff, i, 32) * TWO_N19;
            i += 32;
            eph.toes = getbitu(rtcm->buff, i, 14) * 60.0;
            i += 14;
            eph.cic = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.OMG0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.cis = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.i0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.crc = getbits(rtcm->buff, i, 16) * TWO_N5;
            i += 16;
            eph.omg = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.OMGd = getbits(rtcm->buff, i, 24) * TWO_N43 * SC2RAD;
            i += 24;
            eph.tgd[0] = getbits(rtcm->buff, i, 10) * TWO_N32;
            i += 10; /* E5a/E1 */
            e5a_hs = getbitu(rtcm->buff, i, 2);
            i += 2; /* OSHS */
            e5a_dvs = getbitu(rtcm->buff, i, 1);
            i += 1; /* OSDVS */
            // rsv        = getbitu(rtcm->buff, i, 7);
        }
    else
        {
            trace(2, "rtcm3 1046 length error: len=%d\n", rtcm->len);
            return -1;
        }
    trace(4, "decode_type1046: prn=%d iode=%d toe=%.0f\n", prn, eph.iode, eph.toes);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f hs=%d dvs=%d",
                prn, eph.iode, week, eph.toes, toc, e5a_hs, e5a_dvs);
        }
    if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 1046 satellite number error: prn=%d\n", prn);
            return -1;
        }
    eph.sat = sat;
    eph.week = week + 1024; /* gal-week = gst-week + 1024 */
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, toc);
    eph.ttr = rtcm->time;
    eph.A = sqrtA * sqrtA;
    eph.svh = (e5a_hs << 4) + (e5a_dvs << 3);
    eph.code = 2; /* data source = f/nav e5a */
    if (!strstr(rtcm->opt, "-EPHALL"))
        {
            if (eph.iode == rtcm->nav.eph[sat - 1].iode)
                {
                    return 0; /* unchanged */
                }
        }
    rtcm->nav.eph[sat - 1] = eph;
    rtcm->ephsat = sat;
    return 2;
}


/* decode type 1047: beidou ephemerides (tentative mt and format) ------------*/
int decode_type1047(rtcm_t *rtcm)
{
    eph_t eph = {0, -1, -1, 0, 0, 0, 0, 0, {0, 0.0}, {0, 0.0}, {0, 0.0},
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {0.0}, {0.0}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
    double toc;
    double sqrtA;
    char *msg;
    int i = 24 + 12;
    int prn;
    int sat;
    int week;
    int sys = SYS_BDS;

    if (i + 476 <= rtcm->len * 8)
        {
            prn = getbitu(rtcm->buff, i, 6);
            i += 6;
            week = getbitu(rtcm->buff, i, 10);
            i += 10;
            eph.sva = getbitu(rtcm->buff, i, 4);
            i += 4;
            eph.code = getbitu(rtcm->buff, i, 2);
            i += 2;
            eph.idot = getbits(rtcm->buff, i, 14) * TWO_N43 * SC2RAD;
            i += 14;
            eph.iode = getbitu(rtcm->buff, i, 8);
            i += 8;
            toc = getbitu(rtcm->buff, i, 16) * 16.0;
            i += 16;
            eph.f2 = getbits(rtcm->buff, i, 8) * TWO_N55;
            i += 8;
            eph.f1 = getbits(rtcm->buff, i, 16) * TWO_N43;
            i += 16;
            eph.f0 = getbits(rtcm->buff, i, 22) * TWO_N31;
            i += 22;
            eph.iodc = getbitu(rtcm->buff, i, 10);
            i += 10;
            eph.crs = getbits(rtcm->buff, i, 16) * TWO_N5;
            i += 16;
            eph.deln = getbits(rtcm->buff, i, 16) * TWO_N43 * SC2RAD;
            i += 16;
            eph.M0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.cuc = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.e = getbitu(rtcm->buff, i, 32) * TWO_N33;
            i += 32;
            eph.cus = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            sqrtA = getbitu(rtcm->buff, i, 32) * TWO_N19;
            i += 32;
            eph.toes = getbitu(rtcm->buff, i, 16) * 16.0;
            i += 16;
            eph.cic = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.OMG0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.cis = getbits(rtcm->buff, i, 16) * TWO_N29;
            i += 16;
            eph.i0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.crc = getbits(rtcm->buff, i, 16) * TWO_N5;
            i += 16;
            eph.omg = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.OMGd = getbits(rtcm->buff, i, 24) * TWO_N43 * SC2RAD;
            i += 24;
            eph.tgd[0] = getbits(rtcm->buff, i, 8) * TWO_N31;
            i += 8;
            eph.svh = getbitu(rtcm->buff, i, 6);
            i += 6;
            eph.flag = getbitu(rtcm->buff, i, 1);
            i += 1;
            eph.fit = getbitu(rtcm->buff, i, 1) ? 0.0 : 4.0; /* 0:4hr, 1:>4hr */
        }
    else
        {
            trace(2, "rtcm3 1047 length error: len=%d\n", rtcm->len);
            return -1;
        }
    trace(4, "decode_type1047: prn=%d iode=%d toe=%.0f\n", prn, eph.iode, eph.toes);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
                prn, eph.iode, eph.iodc, week, eph.toes, toc, eph.svh);
        }
    if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 1047 satellite number error: prn=%d\n", prn);
            return -1;
        }
    eph.sat = sat;
    eph.week = adjbdtweek(week);
    eph.toe = bdt2gpst(bdt2time(eph.week, eph.toes)); /* bdt -> gpst */
    eph.toc = bdt2gpst(bdt2time(eph.week, toc));      /* bdt -> gpst */
    eph.ttr = rtcm->time;
    eph.A = sqrtA * sqrtA;
    if (!strstr(rtcm->opt, "-EPHALL"))

        {
            if (timediff(eph.toe, rtcm->nav.eph[sat - 1].toe) == 0.0 &&
                eph.iode == rtcm->nav.eph[sat - 1].iode &&
                eph.iodc == rtcm->nav.eph[sat - 1].iodc)
                {
                    return 0; /* unchanged */
                }
        }
    rtcm->nav.eph[sat - 1] = eph;
    rtcm->ephsat = sat;
    return 2;
}


/* decode type 63: beidou ephemerides (rtcm draft) ---------------------------*/
int decode_type63(rtcm_t *rtcm)
{
    eph_t eph = {0, -1, -1, 0, 0, 0, 0, 0, {0, 0.0}, {0, 0.0}, {0, 0.0},
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {0.0}, {0.0}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
    double toc;
    double sqrtA;
    char *msg;
    int i = 24 + 12;
    int prn;
    int sat;
    int week;
    int sys = SYS_BDS;

    if (i + 499 <= rtcm->len * 8)
        {
            prn = getbitu(rtcm->buff, i, 6);
            i += 6;
            week = getbitu(rtcm->buff, i, 13);
            i += 13;
            eph.sva = getbitu(rtcm->buff, i, 4);
            i += 4;
            eph.idot = getbits(rtcm->buff, i, 14) * TWO_N43 * SC2RAD;
            i += 14;
            eph.iode = getbitu(rtcm->buff, i, 5);
            i += 5; /* AODE */
            toc = getbitu(rtcm->buff, i, 17) * 8.0;
            i += 17;
            eph.f2 = getbits(rtcm->buff, i, 11) * TWO_N55;
            i += 11;
            eph.f1 = getbits(rtcm->buff, i, 22) * TWO_N50;
            i += 22;
            eph.f0 = getbits(rtcm->buff, i, 24) * TWO_N33;
            i += 24;
            eph.iodc = getbitu(rtcm->buff, i, 5);
            i += 5; /* AODC */
            eph.crs = getbits(rtcm->buff, i, 18) * TWO_N6;
            i += 18;
            eph.deln = getbits(rtcm->buff, i, 16) * TWO_N43 * SC2RAD;
            i += 16;
            eph.M0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.cuc = getbits(rtcm->buff, i, 18) * TWO_N31;
            i += 18;
            eph.e = getbitu(rtcm->buff, i, 32) * TWO_N33;
            i += 32;
            eph.cus = getbits(rtcm->buff, i, 18) * TWO_N31;
            i += 18;
            sqrtA = getbitu(rtcm->buff, i, 32) * TWO_N19;
            i += 32;
            eph.toes = getbitu(rtcm->buff, i, 17) * 8.0;
            i += 17;
            eph.cic = getbits(rtcm->buff, i, 18) * TWO_N31;
            i += 18;
            eph.OMG0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.cis = getbits(rtcm->buff, i, 18) * TWO_N31;
            i += 18;
            eph.i0 = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.crc = getbits(rtcm->buff, i, 18) * TWO_N6;
            i += 18;
            eph.omg = getbits(rtcm->buff, i, 32) * TWO_N31 * SC2RAD;
            i += 32;
            eph.OMGd = getbits(rtcm->buff, i, 24) * TWO_N43 * SC2RAD;
            i += 24;
            eph.tgd[0] = getbits(rtcm->buff, i, 10) * 1E-10;
            i += 10;
            eph.tgd[1] = getbits(rtcm->buff, i, 10) * 1E-10;
            i += 10;
            eph.svh = getbitu(rtcm->buff, i, 1);
            i += 1;
        }
    else
        {
            trace(2, "rtcm3 63 length error: len=%d\n", rtcm->len);
            return -1;
        }
    trace(4, "decode_type63: prn=%d iode=%d toe=%.0f\n", prn, eph.iode, eph.toes);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
                prn, eph.iode, eph.iodc, week, eph.toes, toc, eph.svh);
        }
    if (!(sat = satno(sys, prn)))
        {
            trace(2, "rtcm3 63 satellite number error: prn=%d\n", prn);
            return -1;
        }
    eph.sat = sat;
    eph.week = adjbdtweek(week);
    eph.toe = bdt2gpst(bdt2time(eph.week, eph.toes)); /* bdt -> gpst */
    eph.toc = bdt2gpst(bdt2time(eph.week, toc));      /* bdt -> gpst */
    eph.ttr = rtcm->time;
    eph.A = sqrtA * sqrtA;
    if (!strstr(rtcm->opt, "-EPHALL"))
        {
            if (timediff(eph.toe, rtcm->nav.eph[sat - 1].toe) == 0.0 &&
                eph.iode == rtcm->nav.eph[sat - 1].iode &&
                eph.iodc == rtcm->nav.eph[sat - 1].iodc)
                {
                    return 0; /* unchanged */
                }
        }
    rtcm->nav.eph[sat - 1] = eph;
    rtcm->ephsat = sat;
    return 2;
}


/* decode ssr 1, 4 message header ---------------------------------------------*/
int decode_ssr1_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
    double *udint, int *refd, int *hsize)
{
    double tod;
    double tow;
    char *msg;
    int i = 24 + 12;
    int nsat;
    int udi;
    int provid = 0;
    int solid = 0;
    int ns = 6;

#ifndef SSR_QZSS_DRAFT_V05
    ns = sys == SYS_QZS ? 4 : 6;
#endif
    if (i + (sys == SYS_GLO ? 53 : 50 + ns) > rtcm->len * 8)
        {
            return -1;
        }

    if (sys == SYS_GLO)
        {
            tod = getbitu(rtcm->buff, i, 17);
            i += 17;
            adjday_glot(rtcm, tod);
        }
    else
        {
            tow = getbitu(rtcm->buff, i, 20);
            i += 20;
            adjweek(rtcm, tow);
        }
    udi = getbitu(rtcm->buff, i, 4);
    i += 4;
    *sync = getbitu(rtcm->buff, i, 1);
    i += 1;
    *refd = getbitu(rtcm->buff, i, 1);
    i += 1; /* satellite ref datum */
    *iod = getbitu(rtcm->buff, i, 4);
    i += 4; /* iod */
    provid = getbitu(rtcm->buff, i, 16);
    i += 16; /* provider id */
    solid = getbitu(rtcm->buff, i, 4);
    i += 4; /* solution id */
    nsat = getbitu(rtcm->buff, i, ns);
    i += ns;
    *udint = SSRUDINT[udi];

    trace(4, "decode_ssr1_head: time=%s sys=%d nsat=%d sync=%d iod=%d provid=%d solid=%d\n",
        time_str(rtcm->time, 2), sys, nsat, *sync, *iod, provid, solid);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " %s nsat=%2d iod=%2d udi=%2d sync=%d",
                time_str(rtcm->time, 2), nsat, *iod, udi, *sync);
        }
    *hsize = i;
    return nsat;
}


/* decode ssr 2, 3, 5, 6 message header -----------------------------------------*/
int decode_ssr2_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
    double *udint, int *hsize)
{
    double tod;
    double tow;
    char *msg;
    int i = 24 + 12;
    int nsat;
    int udi;
    int provid = 0;
    int solid = 0;
    int ns = 6;

#ifndef SSR_QZSS_DRAFT_V05
    ns = sys == SYS_QZS ? 4 : 6;
#endif
    if (i + (sys == SYS_GLO ? 52 : 49 + ns) > rtcm->len * 8)
        {
            return -1;
        }

    if (sys == SYS_GLO)
        {
            tod = getbitu(rtcm->buff, i, 17);
            i += 17;
            adjday_glot(rtcm, tod);
        }
    else
        {
            tow = getbitu(rtcm->buff, i, 20);
            i += 20;
            adjweek(rtcm, tow);
        }
    udi = getbitu(rtcm->buff, i, 4);
    i += 4;
    *sync = getbitu(rtcm->buff, i, 1);
    i += 1;
    *iod = getbitu(rtcm->buff, i, 4);
    i += 4;
    provid = getbitu(rtcm->buff, i, 16);
    i += 16; /* provider id */
    solid = getbitu(rtcm->buff, i, 4);
    i += 4; /* solution id */
    nsat = getbitu(rtcm->buff, i, ns);
    i += ns;
    *udint = SSRUDINT[udi];

    trace(4, "decode_ssr2_head: time=%s sys=%d nsat=%d sync=%d iod=%d provid=%d solid=%d\n",
        time_str(rtcm->time, 2), sys, nsat, *sync, *iod, provid, solid);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " %s nsat=%2d iod=%2d udi=%2d sync=%d",
                time_str(rtcm->time, 2), nsat, *iod, udi, *sync);
        }
    *hsize = i;
    return nsat;
}


/* decode ssr 7 message header -----------------------------------------------*/
int decode_ssr7_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
    double *udint, int *dispe, int *mw, int *hsize)
{
    double tod;
    double tow;
    char *msg;
    int i = 24 + 12;
    int nsat;
    int udi;
    int provid = 0;
    int solid = 0;
    int ns = 6;

#ifndef SSR_QZSS_DRAFT_V05
    ns = sys == SYS_QZS ? 4 : 6;
#endif
    if (i + (sys == SYS_GLO ? 54 : 51 + ns) > rtcm->len * 8)
        {
            return -1;
        }

    if (sys == SYS_GLO)
        {
            tod = getbitu(rtcm->buff, i, 17);
            i += 17;
            adjday_glot(rtcm, tod);
        }
    else
        {
            tow = getbitu(rtcm->buff, i, 20);
            i += 20;
            adjweek(rtcm, tow);
        }
    udi = getbitu(rtcm->buff, i, 4);
    i += 4;
    *sync = getbitu(rtcm->buff, i, 1);
    i += 1;
    *iod = getbitu(rtcm->buff, i, 4);
    i += 4;
    provid = getbitu(rtcm->buff, i, 16);
    i += 16; /* provider id */
    solid = getbitu(rtcm->buff, i, 4);
    i += 4; /* solution id */
    *dispe = getbitu(rtcm->buff, i, 1);
    i += 1; /* dispersive bias consistency ind */
    *mw = getbitu(rtcm->buff, i, 1);
    i += 1; /* MW consistency indicator */
    nsat = getbitu(rtcm->buff, i, ns);
    i += ns;
    *udint = SSRUDINT[udi];

    trace(4, "decode_ssr7_head: time=%s sys=%d nsat=%d sync=%d iod=%d provid=%d solid=%d\n",
        time_str(rtcm->time, 2), sys, nsat, *sync, *iod, provid, solid);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " %s nsat=%2d iod=%2d udi=%2d sync=%d",
                time_str(rtcm->time, 2), nsat, *iod, udi, *sync);
        }
    *hsize = i;
    return nsat;
}


/* decode ssr 1: orbit corrections -------------------------------------------*/
int decode_ssr1(rtcm_t *rtcm, int sys)
{
    double udint;
    double deph[3];
    double ddeph[3];
    int i;
    int j;
    int k;
    int type;
    int sync;
    int iod;
    int nsat;
    int prn;
    int sat;
    int iode;
    int iodcrc;
    int refd = 0;
    int np;
    int ni;
    int nj;
    int offp;

    type = getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr1_head(rtcm, sys, &sync, &iod, &udint, &refd, &i)) < 0)
        {
            trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
            return -1;
        }
    switch (sys)
        {
        case SYS_GPS:
            np = 6;
            ni = 8;
            nj = 0;
            offp = 0;
            break;
        case SYS_GLO:
            np = 5;
            ni = 8;
            nj = 0;
            offp = 0;
            break;
        case SYS_GAL:
            np = 6;
            ni = 10;
            nj = 0;
            offp = 0;
            break;
        case SYS_QZS:
            np = 4;
            ni = 8;
            nj = 0;
            offp = 192;
            break;
        case SYS_BDS:
            np = 6;
            ni = 10;
            nj = 24;
            offp = 1;
            break;
        case SYS_SBS:
            np = 6;
            ni = 9;
            nj = 24;
            offp = 120;
            break;
        default:
            return sync ? 0 : 10;
        }
    for (j = 0; j < nsat && i + 121 + np + ni + nj <= rtcm->len * 8; j++)
        {
            prn = getbitu(rtcm->buff, i, np) + offp;
            i += np;
            iode = getbitu(rtcm->buff, i, ni);
            i += ni;
            iodcrc = getbitu(rtcm->buff, i, nj);
            i += nj;
            deph[0] = getbits(rtcm->buff, i, 22) * 1E-4;
            i += 22;
            deph[1] = getbits(rtcm->buff, i, 20) * 4E-4;
            i += 20;
            deph[2] = getbits(rtcm->buff, i, 20) * 4E-4;
            i += 20;
            ddeph[0] = getbits(rtcm->buff, i, 21) * 1E-6;
            i += 21;
            ddeph[1] = getbits(rtcm->buff, i, 19) * 4E-6;
            i += 19;
            ddeph[2] = getbits(rtcm->buff, i, 19) * 4E-6;
            i += 19;

            if (!(sat = satno(sys, prn)))
                {
                    trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
                    continue;
                }
            rtcm->ssr[sat - 1].t0[0] = rtcm->time;
            rtcm->ssr[sat - 1].udi[0] = udint;
            rtcm->ssr[sat - 1].iod[0] = iod;
            rtcm->ssr[sat - 1].iode = iode;     /* sbas/bds: toe/t0 modulo */
            rtcm->ssr[sat - 1].iodcrc = iodcrc; /* sbas/bds: iod crc */
            rtcm->ssr[sat - 1].refd = refd;

            for (k = 0; k < 3; k++)
                {
                    rtcm->ssr[sat - 1].deph[k] = deph[k];
                    rtcm->ssr[sat - 1].ddeph[k] = ddeph[k];
                }
            rtcm->ssr[sat - 1].update = 1;
        }
    return sync ? 0 : 10;
}


/* decode ssr 2: clock corrections -------------------------------------------*/
int decode_ssr2(rtcm_t *rtcm, int sys)
{
    double udint;
    double dclk[3];
    int i;
    int j;
    int k;
    int type;
    int sync;
    int iod;
    int nsat;
    int prn;
    int sat;
    int np;
    int offp;

    type = getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr2_head(rtcm, sys, &sync, &iod, &udint, &i)) < 0)
        {
            trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
            return -1;
        }
    switch (sys)
        {
        case SYS_GPS:
            np = 6;
            offp = 0;
            break;
        case SYS_GLO:
            np = 5;
            offp = 0;
            break;
        case SYS_GAL:
            np = 6;
            offp = 0;
            break;
        case SYS_QZS:
            np = 4;
            offp = 192;
            break;
        case SYS_BDS:
            np = 6;
            offp = 1;
            break;
        case SYS_SBS:
            np = 6;
            offp = 120;
            break;
        default:
            return sync ? 0 : 10;
        }
    for (j = 0; j < nsat && i + 70 + np <= rtcm->len * 8; j++)
        {
            prn = getbitu(rtcm->buff, i, np) + offp;
            i += np;
            dclk[0] = getbits(rtcm->buff, i, 22) * 1E-4;
            i += 22;
            dclk[1] = getbits(rtcm->buff, i, 21) * 1E-6;
            i += 21;
            dclk[2] = getbits(rtcm->buff, i, 27) * 2E-8;
            i += 27;

            if (!(sat = satno(sys, prn)))
                {
                    trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
                    continue;
                }
            rtcm->ssr[sat - 1].t0[1] = rtcm->time;
            rtcm->ssr[sat - 1].udi[1] = udint;
            rtcm->ssr[sat - 1].iod[1] = iod;

            for (k = 0; k < 3; k++)
                {
                    rtcm->ssr[sat - 1].dclk[k] = dclk[k];
                }
            rtcm->ssr[sat - 1].update = 1;
        }
    return sync ? 0 : 10;
}


/* decode ssr 3: satellite code biases ---------------------------------------*/
int decode_ssr3(rtcm_t *rtcm, int sys)
{
    const int *codes;
    double udint;
    double bias;
    double cbias[MAXCODE];
    int i;
    int j;
    int k;
    int type;
    int mode;
    int sync;
    int iod;
    int nsat;
    int prn;
    int sat;
    int nbias;
    int np;
    int offp;
    int ncode;

    type = getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr2_head(rtcm, sys, &sync, &iod, &udint, &i)) < 0)
        {
            trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
            return -1;
        }
    switch (sys)
        {
        case SYS_GPS:
            np = 6;
            offp = 0;
            codes = CODES_GPS;
            ncode = 17;
            break;
        case SYS_GLO:
            np = 5;
            offp = 0;
            codes = CODES_GLO;
            ncode = 4;
            break;
        case SYS_GAL:
            np = 6;
            offp = 0;
            codes = CODES_GAL;
            ncode = 19;
            break;
        case SYS_QZS:
            np = 4;
            offp = 192;
            codes = CODES_QZS;
            ncode = 13;
            break;
        case SYS_BDS:
            np = 6;
            offp = 1;
            codes = CODES_BDS;
            ncode = 9;
            break;
        case SYS_SBS:
            np = 6;
            offp = 120;
            codes = CODES_SBS;
            ncode = 4;
            break;
        default:
            return sync ? 0 : 10;
        }
    for (j = 0; j < nsat && i + 5 + np <= rtcm->len * 8; j++)
        {
            prn = getbitu(rtcm->buff, i, np) + offp;
            i += np;
            nbias = getbitu(rtcm->buff, i, 5);
            i += 5;

            for (k = 0; k < MAXCODE; k++)
                {
                    cbias[k] = 0.0;
                }
            for (k = 0; k < nbias && i + 19 <= rtcm->len * 8; k++)
                {
                    mode = getbitu(rtcm->buff, i, 5);
                    i += 5;
                    bias = getbits(rtcm->buff, i, 14) * 0.01;
                    i += 14;
                    if (mode <= ncode)
                        {
                            cbias[codes[mode] - 1] = static_cast<float>(bias);
                        }
                    else
                        {
                            trace(2, "rtcm3 %d not supported mode: mode=%d\n", type, mode);
                        }
                }
            if (!(sat = satno(sys, prn)))
                {
                    trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
                    continue;
                }
            rtcm->ssr[sat - 1].t0[4] = rtcm->time;
            rtcm->ssr[sat - 1].udi[4] = udint;
            rtcm->ssr[sat - 1].iod[4] = iod;

            for (k = 0; k < MAXCODE; k++)
                {
                    rtcm->ssr[sat - 1].cbias[k] = static_cast<float>(cbias[k]);
                }
            rtcm->ssr[sat - 1].update = 1;
        }
    return sync ? 0 : 10;
}


/* decode ssr 4: combined orbit and clock corrections ------------------------*/
int decode_ssr4(rtcm_t *rtcm, int sys)
{
    double udint;
    double deph[3];
    double ddeph[3];
    double dclk[3];
    int i;
    int j;
    int k;
    int type;
    int nsat;
    int sync;
    int iod;
    int prn;
    int sat;
    int iode;
    int iodcrc;
    int refd = 0;
    int np;
    int ni;
    int nj;
    int offp;

    type = getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr1_head(rtcm, sys, &sync, &iod, &udint, &refd, &i)) < 0)
        {
            trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
            return -1;
        }
    switch (sys)
        {
        case SYS_GPS:
            np = 6;
            ni = 8;
            nj = 0;
            offp = 0;
            break;
        case SYS_GLO:
            np = 5;
            ni = 8;
            nj = 0;
            offp = 0;
            break;
        case SYS_GAL:
            np = 6;
            ni = 10;
            nj = 0;
            offp = 0;
            break;
        case SYS_QZS:
            np = 4;
            ni = 8;
            nj = 0;
            offp = 192;
            break;
        case SYS_BDS:
            np = 6;
            ni = 10;
            nj = 24;
            offp = 1;
            break;
        case SYS_SBS:
            np = 6;
            ni = 9;
            nj = 24;
            offp = 120;
            break;
        default:
            return sync ? 0 : 10;
        }
    for (j = 0; j < nsat && i + 191 + np + ni + nj <= rtcm->len * 8; j++)
        {
            prn = getbitu(rtcm->buff, i, np) + offp;
            i += np;
            iode = getbitu(rtcm->buff, i, ni);
            i += ni;
            iodcrc = getbitu(rtcm->buff, i, nj);
            i += nj;
            deph[0] = getbits(rtcm->buff, i, 22) * 1E-4;
            i += 22;
            deph[1] = getbits(rtcm->buff, i, 20) * 4E-4;
            i += 20;
            deph[2] = getbits(rtcm->buff, i, 20) * 4E-4;
            i += 20;
            ddeph[0] = getbits(rtcm->buff, i, 21) * 1E-6;
            i += 21;
            ddeph[1] = getbits(rtcm->buff, i, 19) * 4E-6;
            i += 19;
            ddeph[2] = getbits(rtcm->buff, i, 19) * 4E-6;
            i += 19;

            dclk[0] = getbits(rtcm->buff, i, 22) * 1E-4;
            i += 22;
            dclk[1] = getbits(rtcm->buff, i, 21) * 1E-6;
            i += 21;
            dclk[2] = getbits(rtcm->buff, i, 27) * 2E-8;
            i += 27;

            if (!(sat = satno(sys, prn)))
                {
                    trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
                    continue;
                }
            rtcm->ssr[sat - 1].t0[0] = rtcm->ssr[sat - 1].t0[1] = rtcm->time;
            rtcm->ssr[sat - 1].udi[0] = rtcm->ssr[sat - 1].udi[1] = udint;
            rtcm->ssr[sat - 1].iod[0] = rtcm->ssr[sat - 1].iod[1] = iod;
            rtcm->ssr[sat - 1].iode = iode;
            rtcm->ssr[sat - 1].iodcrc = iodcrc;
            rtcm->ssr[sat - 1].refd = refd;

            for (k = 0; k < 3; k++)
                {
                    rtcm->ssr[sat - 1].deph[k] = deph[k];
                    rtcm->ssr[sat - 1].ddeph[k] = ddeph[k];
                    rtcm->ssr[sat - 1].dclk[k] = dclk[k];
                }
            rtcm->ssr[sat - 1].update = 1;
        }
    return sync ? 0 : 10;
}


/* decode ssr 5: ura ---------------------------------------------------------*/
int decode_ssr5(rtcm_t *rtcm, int sys)
{
    double udint;
    int i;
    int j;
    int type;
    int nsat;
    int sync;
    int iod;
    int prn;
    int sat;
    int ura;
    int np;
    int offp;

    type = getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr2_head(rtcm, sys, &sync, &iod, &udint, &i)) < 0)
        {
            trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
            return -1;
        }
    switch (sys)
        {
        case SYS_GPS:
            np = 6;
            offp = 0;
            break;
        case SYS_GLO:
            np = 5;
            offp = 0;
            break;
        case SYS_GAL:
            np = 6;
            offp = 0;
            break;
        case SYS_QZS:
            np = 4;
            offp = 192;
            break;
        case SYS_BDS:
            np = 6;
            offp = 1;
            break;
        case SYS_SBS:
            np = 6;
            offp = 120;
            break;
        default:
            return sync ? 0 : 10;
        }
    for (j = 0; j < nsat && i + 6 + np <= rtcm->len * 8; j++)
        {
            prn = getbitu(rtcm->buff, i, np) + offp;
            i += np;
            ura = getbitu(rtcm->buff, i, 6);
            i += 6;

            if (!(sat = satno(sys, prn)))
                {
                    trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
                    continue;
                }
            rtcm->ssr[sat - 1].t0[3] = rtcm->time;
            rtcm->ssr[sat - 1].udi[3] = udint;
            rtcm->ssr[sat - 1].iod[3] = iod;
            rtcm->ssr[sat - 1].ura = ura;
            rtcm->ssr[sat - 1].update = 1;
        }
    return sync ? 0 : 10;
}


/* decode ssr 6: high rate clock correction ----------------------------------*/
int decode_ssr6(rtcm_t *rtcm, int sys)
{
    double udint;
    double hrclk;
    int i;
    int j;
    int type;
    int nsat;
    int sync;
    int iod;
    int prn;
    int sat;
    int np;
    int offp;

    type = getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr2_head(rtcm, sys, &sync, &iod, &udint, &i)) < 0)
        {
            trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
            return -1;
        }
    switch (sys)
        {
        case SYS_GPS:
            np = 6;
            offp = 0;
            break;
        case SYS_GLO:
            np = 5;
            offp = 0;
            break;
        case SYS_GAL:
            np = 6;
            offp = 0;
            break;
        case SYS_QZS:
            np = 4;
            offp = 192;
            break;
        case SYS_BDS:
            np = 6;
            offp = 1;
            break;
        case SYS_SBS:
            np = 6;
            offp = 120;
            break;
        default:
            return sync ? 0 : 10;
        }
    for (j = 0; j < nsat && i + 22 + np <= rtcm->len * 8; j++)
        {
            prn = getbitu(rtcm->buff, i, np) + offp;
            i += np;
            hrclk = getbits(rtcm->buff, i, 22) * 1E-4;
            i += 22;

            if (!(sat = satno(sys, prn)))
                {
                    trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
                    continue;
                }
            rtcm->ssr[sat - 1].t0[2] = rtcm->time;
            rtcm->ssr[sat - 1].udi[2] = udint;
            rtcm->ssr[sat - 1].iod[2] = iod;
            rtcm->ssr[sat - 1].hrclk = hrclk;
            rtcm->ssr[sat - 1].update = 1;
        }
    return sync ? 0 : 10;
}


/* decode ssr 7: phase bias --------------------------------------------------*/
int decode_ssr7(rtcm_t *rtcm, int sys)
{
    const int *codes;
    double udint;
    double bias;
    double std;
    double pbias[MAXCODE];
    double stdpb[MAXCODE];
    int i;
    int j;
    int k;
    int type;
    int mode;
    int sync;
    int iod;
    int nsat;
    int prn;
    int sat;
    int nbias;
    int ncode;
    int np;
    int mw;
    int offp;
    int dispe;
    int yaw_ang;
    int yaw_rate;

    type = getbitu(rtcm->buff, 24, 12);

    if ((nsat = decode_ssr7_head(rtcm, sys, &sync, &iod, &udint, &dispe, &mw, &i)) < 0)
        {
            trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
            return -1;
        }
    switch (sys)
        {
        case SYS_GPS:
            np = 6;
            offp = 0;
            codes = CODES_GPS;
            ncode = 17;
            break;
        case SYS_GLO:
            np = 5;
            offp = 0;
            codes = CODES_GLO;
            ncode = 4;
            break;
        case SYS_GAL:
            np = 6;
            offp = 0;
            codes = CODES_GAL;
            ncode = 19;
            break;
        case SYS_QZS:
            np = 4;
            offp = 192;
            codes = CODES_QZS;
            ncode = 13;
            break;
        case SYS_BDS:
            np = 6;
            offp = 1;
            codes = CODES_BDS;
            ncode = 9;
            break;
        default:
            return sync ? 0 : 10;
        }
    for (j = 0; j < nsat && i + 5 + 17 + np <= rtcm->len * 8; j++)
        {
            prn = getbitu(rtcm->buff, i, np) + offp;
            i += np;
            nbias = getbitu(rtcm->buff, i, 5);
            i += 5;
            yaw_ang = getbitu(rtcm->buff, i, 9);
            i += 9;
            yaw_rate = getbits(rtcm->buff, i, 8);
            i += 8;

            for (k = 0; k < MAXCODE; k++)
                {
                    pbias[k] = stdpb[k] = 0.0;
                }
            for (k = 0; k < nbias && i + 49 <= rtcm->len * 8; k++)
                {
                    mode = getbitu(rtcm->buff, i, 5);
                    i += 5;
                    /* sii  = getbitu(rtcm->buff, i, 1); */ i += 1; /* integer-indicator */
                    /* swl  = getbitu(rtcm->buff, i, 2); */ i += 2; /* WL integer-indicator */
                    /* sdc  = getbitu(rtcm->buff, i, 4); */ i += 4; /* discontinuity counter */
                    bias = getbits(rtcm->buff, i, 20);
                    i += 20; /* phase bias (m) */
                    std = getbitu(rtcm->buff, i, 17);
                    i += 17; /* phase bias std-dev (m) */
                    if (mode <= ncode)
                        {
                            pbias[codes[mode] - 1] = bias * 0.0001; /* (m) */
                            stdpb[codes[mode] - 1] = std * 0.0001;  /* (m) */
                        }
                    else
                        {
                            trace(2, "rtcm3 %d not supported mode: mode=%d\n", type, mode);
                        }
                }
            if (!(sat = satno(sys, prn)))
                {
                    trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
                    continue;
                }
            rtcm->ssr[sat - 1].t0[5] = rtcm->time;
            rtcm->ssr[sat - 1].udi[5] = udint;
            rtcm->ssr[sat - 1].iod[5] = iod;
            rtcm->ssr[sat - 1].yaw_ang = yaw_ang / 256.0 * 180.0;    /* (deg) */
            rtcm->ssr[sat - 1].yaw_rate = yaw_rate / 8192.0 * 180.0; /* (deg/s) */

            for (k = 0; k < MAXCODE; k++)
                {
                    rtcm->ssr[sat - 1].pbias[k] = pbias[k];
                    rtcm->ssr[sat - 1].stdpb[k] = static_cast<float>(stdpb[k]);
                }
        }
    return 20;
}


/* get signal index ----------------------------------------------------------*/
void sigindex(int sys, const unsigned char *code, const int *freq, int n,
    const char *opt, int *ind)
{
    int i;
    int nex;
    int pri;
    int pri_h[8] = {0};
    int index[8] = {0};
    int ex[32] = {0};

    /* test code priority */
    for (i = 0; i < n; i++)
        {
            if (!code[i])
                {
                    continue;
                }

            if (freq[i] > NFREQ)
                { /* save as extended signal if freq > NFREQ */
                    ex[i] = 1;
                    continue;
                }
            /* code priority */
            pri = getcodepri(sys, code[i], opt);

            /* select highest priority signal */
            if (pri > pri_h[freq[i] - 1])
                {
                    if (index[freq[i] - 1])
                        {
                            ex[index[freq[i] - 1] - 1] = 1;
                        }
                    pri_h[freq[i] - 1] = pri;
                    index[freq[i] - 1] = i + 1;
                }
            else
                {
                    ex[i] = 1;
                }
        }
    /* signal index in obs data */
    for (i = nex = 0; i < n; i++)
        {
            if (ex[i] == 0)
                {
                    ind[i] = freq[i] - 1;
                }
            else if (nex < NEXOBS)
                {
                    ind[i] = NFREQ + nex++;
                }
            else
                { /* no space in obs data */
                    trace(2, "rtcm msm: no space in obs data sys=%d code=%d\n", sys, code[i]);
                    ind[i] = -1;
                }
#if 0
            trace(2, "sig pos: sys=%d code=%d ex=%d ind=%d\n", sys, code[i], ex[i], ind[i]);
#endif
        }
}


/* save obs data in msm message ----------------------------------------------*/
void save_msm_obs(rtcm_t *rtcm, int sys, msm_h_t *h, const double *r,
    const double *pr, const double *cp, const double *rr,
    const double *rrf, const double *cnr, const int *lock,
    const int *ex, const int *half)
{
    const char *sig[32];
    double tt;
    double wl;
    unsigned char code[32];
    char *msm_type = const_cast<char *>("");
    char *q = nullptr;
    int i;
    int j;
    int k;
    int type;
    int prn;
    int sat;
    int fn;
    int index = 0;
    int freq[32];
    int ind[32];

    type = getbitu(rtcm->buff, 24, 12);

    switch (sys)
        {
        case SYS_GPS:
            msm_type = q = rtcm->msmtype[0];
            break;
        case SYS_GLO:
            msm_type = q = rtcm->msmtype[1];
            break;
        case SYS_GAL:
            msm_type = q = rtcm->msmtype[2];
            break;
        case SYS_QZS:
            msm_type = q = rtcm->msmtype[3];
            break;
        case SYS_SBS:
            msm_type = q = rtcm->msmtype[4];
            break;
        case SYS_BDS:
            msm_type = q = rtcm->msmtype[5];
            break;
        }
    /* id to signal */
    for (i = 0; i < h->nsig; i++)
        {
            switch (sys)
                {
                case SYS_GPS:
                    sig[i] = msm_sig_gps[h->sigs[i] - 1];
                    break;
                case SYS_GLO:
                    sig[i] = msm_sig_glo[h->sigs[i] - 1];
                    break;
                case SYS_GAL:
                    sig[i] = msm_sig_gal[h->sigs[i] - 1];
                    break;
                case SYS_QZS:
                    sig[i] = msm_sig_qzs[h->sigs[i] - 1];
                    break;
                case SYS_SBS:
                    sig[i] = msm_sig_sbs[h->sigs[i] - 1];
                    break;
                case SYS_BDS:
                    sig[i] = msm_sig_cmp[h->sigs[i] - 1];
                    break;
                default:
                    sig[i] = "";
                    break;
                }
            /* signal to rinex obs type */
            code[i] = obs2code(sig[i], freq + i);

            /* frequency index for beidou */
            if (sys == SYS_BDS)
                {
                    if (freq[i] == 5)
                        {
                            freq[i] = 2; /* B2 */
                        }
                    else if (freq[i] == 4)
                        {
                            freq[i] = 3; /* B3 */
                        }
                }
            if (code[i] != CODE_NONE)
                {
                    if (q)
                        {
                            q += std::snprintf(q, sizeof(rtcm->msmtype[0]), "L%s%s", sig[i], i < h->nsig - 1 ? ", " : "");
                        }
                }
            else
                {
                    if (q)
                        {
                            q += std::snprintf(q, sizeof(rtcm->msmtype[0]), "(%d)%s", h->sigs[i], i < h->nsig - 1 ? ", " : "");
                        }

                    trace(2, "rtcm3 %d: unknown signal id=%2d\n", type, h->sigs[i]);
                }
        }
    trace(3, "rtcm3 %d: signals=%s\n", type, msm_type);

    /* get signal index */
    sigindex(sys, code, freq, h->nsig, rtcm->opt, ind);

    for (i = j = 0; i < h->nsat; i++)
        {
            prn = h->sats[i];
            if (sys == SYS_QZS)
                {
                    prn += MINPRNQZS - 1;
                }
            else if (sys == SYS_SBS)
                {
                    prn += MINPRNSBS - 1;
                }

            if ((sat = satno(sys, prn)))
                {
                    tt = timediff(rtcm->obs.data[0].time, rtcm->time);
                    if (rtcm->obsflag || fabs(tt) > 1E-9)
                        {
                            rtcm->obs.n = rtcm->obsflag = 0;
                        }
                    index = obsindex3(&rtcm->obs, rtcm->time, sat);
                }
            else
                {
                    trace(2, "rtcm3 %d satellite error: prn=%d\n", type, prn);
                }
            for (k = 0; k < h->nsig; k++)
                {
                    if (!h->cellmask[k + i * h->nsig])
                        {
                            continue;
                        }

                    if (sat && index >= 0 && ind[k] >= 0)
                        {
                            /* satellite carrier wave length */
                            wl = satwavelen(sat, freq[k] - 1, &rtcm->nav);

                            /* glonass wave length by extended info */
                            if (sys == SYS_GLO && ex && ex[i] <= 13)
                                {
                                    fn = ex[i] - 7;
                                    wl = SPEED_OF_LIGHT_M_S / ((freq[k] == 2 ? FREQ2_GLO : FREQ1_GLO) +
                                                                  (freq[k] == 2 ? DFRQ2_GLO : DFRQ1_GLO) * fn);
                                }
                            /* pseudorange (m) */
                            if (r[i] != 0.0 && pr[j] > -1E12)
                                {
                                    rtcm->obs.data[index].P[ind[k]] = r[i] + pr[j];
                                }
                            /* carrier-phase (cycle) */
                            if (r[i] != 0.0 && cp[j] > -1E12 && wl > 0.0)
                                {
                                    rtcm->obs.data[index].L[ind[k]] = (r[i] + cp[j]) / wl;
                                }
                            /* doppler (hz) */
                            if (rr && rrf && rrf[j] > -1E12 && wl > 0.0)
                                {
                                    rtcm->obs.data[index].D[ind[k]] = static_cast<float>(-(rr[i] + rrf[j]) / wl);
                                }
                            rtcm->obs.data[index].LLI[ind[k]] =
                                lossoflock(rtcm, sat, ind[k], lock[j]) + (half[j] ? 3 : 0);
                            rtcm->obs.data[index].SNR[ind[k]] = static_cast<unsigned char>(cnr[j] * 4.0);
                            rtcm->obs.data[index].code[ind[k]] = code[k];
                        }
                    j++;
                }
        }
}


/* decode type msm message header --------------------------------------------*/
int decode_msm_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
    msm_h_t *h, int *hsize)
{
    msm_h_t h0 = {0, 0, 0, 0, 0, 0, 0, 0, {0}, {0}, {0}};
    double tow;
    double tod;
    char *msg;
    int i = 24;
    int j;
    int mask;
    int staid;
    int type;
    int ncell = 0;

    type = getbitu(rtcm->buff, i, 12);
    i += 12;

    *h = h0;
    if (i + 157 <= rtcm->len * 8)
        {
            staid = getbitu(rtcm->buff, i, 12);
            i += 12;

            if (sys == SYS_GLO)
                {
                    /* dow    = getbitu(rtcm->buff, i, 3);  */ i += 3;
                    tod = getbitu(rtcm->buff, i, 27) * 0.001;
                    i += 27;
                    adjday_glot(rtcm, tod);
                }
            else if (sys == SYS_BDS)
                {
                    tow = getbitu(rtcm->buff, i, 30) * 0.001;
                    i += 30;
                    tow += 14.0; /* BDT -> GPST */
                    adjweek(rtcm, tow);
                }
            else
                {
                    tow = getbitu(rtcm->buff, i, 30) * 0.001;
                    i += 30;
                    adjweek(rtcm, tow);
                }
            *sync = getbitu(rtcm->buff, i, 1);
            i += 1;
            *iod = getbitu(rtcm->buff, i, 3);
            i += 3;
            h->time_s = getbitu(rtcm->buff, i, 7);
            i += 7;
            h->clk_str = getbitu(rtcm->buff, i, 2);
            i += 2;
            h->clk_ext = getbitu(rtcm->buff, i, 2);
            i += 2;
            h->smooth = getbitu(rtcm->buff, i, 1);
            i += 1;
            h->tint_s = getbitu(rtcm->buff, i, 3);
            i += 3;
            for (j = 1; j <= 64; j++)
                {
                    mask = getbitu(rtcm->buff, i, 1);
                    i += 1;
                    if (mask)
                        {
                            h->sats[h->nsat++] = j;
                        }
                }
            for (j = 1; j <= 32; j++)
                {
                    mask = getbitu(rtcm->buff, i, 1);
                    i += 1;
                    if (mask)
                        {
                            h->sigs[h->nsig++] = j;
                        }
                }
        }
    else
        {
            trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
            return -1;
        }
    /* test station id */
    if (!test_staid(rtcm, staid))
        {
            return -1;
        }

    if (h->nsat * h->nsig > 64)
        {
            trace(2, "rtcm3 %d number of sats and sigs error: nsat=%d nsig=%d\n",
                type, h->nsat, h->nsig);
            return -1;
        }
    if (i + h->nsat * h->nsig > rtcm->len * 8)
        {
            trace(2, "rtcm3 %d length error: len=%d nsat=%d nsig=%d\n", type,
                rtcm->len, h->nsat, h->nsig);
            return -1;
        }
    for (j = 0; j < h->nsat * h->nsig; j++)
        {
            h->cellmask[j] = getbitu(rtcm->buff, i, 1);
            i += 1;
            if (h->cellmask[j])
                {
                    ncell++;
                }
        }
    *hsize = i;

    trace(4, "decode_head_msm: time=%s sys=%d staid=%d nsat=%d nsig=%d sync=%d iod=%d ncell=%d\n",
        time_str(rtcm->time, 2), sys, staid, h->nsat, h->nsig, *sync, *iod, ncell);

    if (rtcm->outtype)
        {
            msg = rtcm->msgtype + strlen(rtcm->msgtype);
            std::snprintf(msg, sizeof(rtcm->msgtype), " staid=%4d %s nsat=%2d nsig=%2d iod=%2d ncell=%2d sync=%d",
                staid, time_str(rtcm->time, 2), h->nsat, h->nsig, *iod, ncell, *sync);
        }
    return ncell;
}


/* decode unsupported msm message --------------------------------------------*/
int decode_msm0(rtcm_t *rtcm, int sys)
{
    msm_h_t h = {0, 0, 0, 0, 0, 0, 0, 0, {0}, {0}, {0}};
    int i;
    int sync;
    int iod;
    if (decode_msm_head(rtcm, sys, &sync, &iod, &h, &i) < 0)
        {
            return -1;
        }
    rtcm->obsflag = !sync;
    return sync ? 0 : 1;
}


/* decode msm 4: full pseudorange and phaserange plus cnr --------------------*/
int decode_msm4(rtcm_t *rtcm, int sys)
{
    msm_h_t h = {0, 0, 0, 0, 0, 0, 0, 0, {0}, {0}, {0}};
    double r[64];
    double pr[64];
    double cp[64];
    double cnr[64];
    int i;
    int j;
    int type;
    int sync;
    int iod;
    int ncell;
    int rng;
    int rng_m;
    int prv;
    int cpv;
    int lock[64];
    int half[64];

    type = getbitu(rtcm->buff, 24, 12);

    /* decode msm header */
    if ((ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i)) < 0)
        {
            return -1;
        }

    if (i + h.nsat * 18 + ncell * 48 > rtcm->len * 8)
        {
            trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat,
                ncell, rtcm->len);
            return -1;
        }
    for (j = 0; j < h.nsat; j++)
        {
            r[j] = 0.0;
        }
    for (j = 0; j < ncell; j++)
        {
            pr[j] = cp[j] = -1E16;
        }

    /* decode satellite data */
    for (j = 0; j < h.nsat; j++)
        { /* range */
            rng = getbitu(rtcm->buff, i, 8);
            i += 8;
            if (rng != 255)
                {
                    r[j] = rng * RANGE_MS;
                }
        }
    for (j = 0; j < h.nsat; j++)
        {
            rng_m = getbitu(rtcm->buff, i, 10);
            i += 10;
            if (r[j] != 0.0)
                {
                    r[j] += rng_m * TWO_N10 * RANGE_MS;
                }
        }
    /* decode signal data */
    for (j = 0; j < ncell; j++)
        { /* pseudorange */
            prv = getbits(rtcm->buff, i, 15);
            i += 15;
            if (prv != -16384)
                {
                    pr[j] = prv * TWO_N24 * RANGE_MS;
                }
        }
    for (j = 0; j < ncell; j++)
        { /* phaserange */
            cpv = getbits(rtcm->buff, i, 22);
            i += 22;
            if (cpv != -2097152)
                {
                    cp[j] = cpv * TWO_N29 * RANGE_MS;
                }
        }
    for (j = 0; j < ncell; j++)
        { /* lock time */
            lock[j] = getbitu(rtcm->buff, i, 4);
            i += 4;
        }
    for (j = 0; j < ncell; j++)
        { /* half-cycle ambiguity */
            half[j] = getbitu(rtcm->buff, i, 1);
            i += 1;
        }
    for (j = 0; j < ncell; j++)
        { /* cnr */
            cnr[j] = getbitu(rtcm->buff, i, 6) * 1.0;
            i += 6;
        }
    /* save obs data in msm message */
    save_msm_obs(rtcm, sys, &h, r, pr, cp, nullptr, nullptr, cnr, lock, nullptr, half);

    rtcm->obsflag = !sync;
    return sync ? 0 : 1;
}


/* decode msm 5: full pseudorange, phaserange, phaserangerate and cnr --------*/
int decode_msm5(rtcm_t *rtcm, int sys)
{
    msm_h_t h = {0, 0, 0, 0, 0, 0, 0, 0, {0}, {0}, {0}};
    double r[64];
    double rr[64];
    double pr[64];
    double cp[64];
    double rrf[64];
    double cnr[64];
    int i;
    int j;
    int type;
    int sync;
    int iod;
    int ncell;
    int rng;
    int rng_m;
    int rate;
    int prv;
    int cpv;
    int rrv;
    int lock[64];
    int ex[64];
    int half[64];

    type = getbitu(rtcm->buff, 24, 12);

    /* decode msm header */
    if ((ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i)) < 0)
        {
            return -1;
        }

    if (i + h.nsat * 36 + ncell * 63 > rtcm->len * 8)
        {
            trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat,
                ncell, rtcm->len);
            return -1;
        }
    for (j = 0; j < h.nsat; j++)
        {
            r[j] = rr[j] = 0.0;
            ex[j] = 15;
        }
    for (j = 0; j < ncell; j++)
        {
            pr[j] = cp[j] = rrf[j] = -1E16;
        }

    /* decode satellite data */
    for (j = 0; j < h.nsat; j++)
        { /* range */
            rng = getbitu(rtcm->buff, i, 8);
            i += 8;
            if (rng != 255)
                {
                    r[j] = rng * RANGE_MS;
                }
        }
    for (j = 0; j < h.nsat; j++)
        { /* extended info */
            ex[j] = getbitu(rtcm->buff, i, 4);
            i += 4;
        }
    for (j = 0; j < h.nsat; j++)
        {
            rng_m = getbitu(rtcm->buff, i, 10);
            i += 10;
            if (r[j] != 0.0)
                {
                    r[j] += rng_m * TWO_N10 * RANGE_MS;
                }
        }
    for (j = 0; j < h.nsat; j++)
        { /* phaserangerate */
            rate = getbits(rtcm->buff, i, 14);
            i += 14;
            if (rate != -8192)
                {
                    rr[j] = rate * 1.0;
                }
        }
    /* decode signal data */
    for (j = 0; j < ncell; j++)
        { /* pseudorange */
            prv = getbits(rtcm->buff, i, 15);
            i += 15;
            if (prv != -16384)
                {
                    pr[j] = prv * TWO_N24 * RANGE_MS;
                }
        }
    for (j = 0; j < ncell; j++)
        { /* phaserange */
            cpv = getbits(rtcm->buff, i, 22);
            i += 22;
            if (cpv != -2097152)
                {
                    cp[j] = cpv * TWO_N29 * RANGE_MS;
                }
        }
    for (j = 0; j < ncell; j++)
        { /* lock time */
            lock[j] = getbitu(rtcm->buff, i, 4);
            i += 4;
        }
    for (j = 0; j < ncell; j++)
        { /* half-cycle ambiguity */
            half[j] = getbitu(rtcm->buff, i, 1);
            i += 1;
        }
    for (j = 0; j < ncell; j++)
        { /* cnr */
            cnr[j] = getbitu(rtcm->buff, i, 6) * 1.0;
            i += 6;
        }
    for (j = 0; j < ncell; j++)
        { /* phaserangerate */
            rrv = getbits(rtcm->buff, i, 15);
            i += 15;
            if (rrv != -16384)
                {
                    rrf[j] = rrv * 0.0001;
                }
        }
    /* save obs data in msm message */
    save_msm_obs(rtcm, sys, &h, r, pr, cp, rr, rrf, cnr, lock, ex, half);

    rtcm->obsflag = !sync;
    return sync ? 0 : 1;
}


/* decode msm 6: full pseudorange and phaserange plus cnr (high-res) ---------*/
int decode_msm6(rtcm_t *rtcm, int sys)
{
    msm_h_t h = {0, 0, 0, 0, 0, 0, 0, 0, {0}, {0}, {0}};
    double r[64];
    double pr[64];
    double cp[64];
    double cnr[64];
    int i;
    int j;
    int type;
    int sync;
    int iod;
    int ncell;
    int rng;
    int rng_m;
    int prv;
    int cpv;
    int lock[64];
    int half[64];

    type = getbitu(rtcm->buff, 24, 12);

    /* decode msm header */
    if ((ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i)) < 0)
        {
            return -1;
        }

    if (i + h.nsat * 18 + ncell * 65 > rtcm->len * 8)
        {
            trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat,
                ncell, rtcm->len);
            return -1;
        }
    for (j = 0; j < h.nsat; j++)
        {
            r[j] = 0.0;
        }
    for (j = 0; j < ncell; j++)
        {
            pr[j] = cp[j] = -1E16;
        }

    /* decode satellite data */
    for (j = 0; j < h.nsat; j++)
        { /* range */
            rng = getbitu(rtcm->buff, i, 8);
            i += 8;
            if (rng != 255)
                {
                    r[j] = rng * RANGE_MS;
                }
        }
    for (j = 0; j < h.nsat; j++)
        {
            rng_m = getbitu(rtcm->buff, i, 10);
            i += 10;
            if (r[j] != 0.0)
                {
                    r[j] += rng_m * TWO_N10 * RANGE_MS;
                }
        }
    /* decode signal data */
    for (j = 0; j < ncell; j++)
        { /* pseudorange */
            prv = getbits(rtcm->buff, i, 20);
            i += 20;
            if (prv != -524288)
                {
                    pr[j] = prv * TWO_N29 * RANGE_MS;
                }
        }
    for (j = 0; j < ncell; j++)
        { /* phaserange */
            cpv = getbits(rtcm->buff, i, 24);
            i += 24;
            if (cpv != -8388608)
                {
                    cp[j] = cpv * TWO_N31 * RANGE_MS;
                }
        }
    for (j = 0; j < ncell; j++)
        { /* lock time */
            lock[j] = getbitu(rtcm->buff, i, 10);
            i += 10;
        }
    for (j = 0; j < ncell; j++)
        { /* half-cycle ambiguity */
            half[j] = getbitu(rtcm->buff, i, 1);
            i += 1;
        }
    for (j = 0; j < ncell; j++)
        { /* cnr */
            cnr[j] = getbitu(rtcm->buff, i, 10) * 0.0625;
            i += 10;
        }
    /* save obs data in msm message */
    save_msm_obs(rtcm, sys, &h, r, pr, cp, nullptr, nullptr, cnr, lock, nullptr, half);

    rtcm->obsflag = !sync;
    return sync ? 0 : 1;
}


/* decode msm 7: full pseudorange, phaserange, phaserangerate and cnr (h-res) */
int decode_msm7(rtcm_t *rtcm, int sys)
{
    msm_h_t h = {0, 0, 0, 0, 0, 0, 0, 0, {0}, {0}, {0}};
    double r[64];
    double rr[64];
    double pr[64];
    double cp[64];
    double rrf[64];
    double cnr[64];
    int i;
    int j;
    int type;
    int sync;
    int iod;
    int ncell;
    int rng;
    int rng_m;
    int rate;
    int prv;
    int cpv;
    int rrv;
    int lock[64];
    int ex[64];
    int half[64];

    type = getbitu(rtcm->buff, 24, 12);

    /* decode msm header */
    if ((ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i)) < 0)
        {
            return -1;
        }

    if (i + h.nsat * 36 + ncell * 80 > rtcm->len * 8)
        {
            trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat,
                ncell, rtcm->len);
            return -1;
        }
    for (j = 0; j < h.nsat; j++)
        {
            r[j] = rr[j] = 0.0;
            ex[j] = 15;
        }
    for (j = 0; j < ncell; j++)
        {
            pr[j] = cp[j] = rrf[j] = -1E16;
        }

    /* decode satellite data */
    for (j = 0; j < h.nsat; j++)
        { /* range */
            rng = getbitu(rtcm->buff, i, 8);
            i += 8;
            if (rng != 255)
                {
                    r[j] = rng * RANGE_MS;
                }
        }
    for (j = 0; j < h.nsat; j++)
        { /* extended info */
            ex[j] = getbitu(rtcm->buff, i, 4);
            i += 4;
        }
    for (j = 0; j < h.nsat; j++)
        {
            rng_m = getbitu(rtcm->buff, i, 10);
            i += 10;
            if (r[j] != 0.0)
                {
                    r[j] += rng_m * TWO_N10 * RANGE_MS;
                }
        }
    for (j = 0; j < h.nsat; j++)
        { /* phaserangerate */
            rate = getbits(rtcm->buff, i, 14);
            i += 14;
            if (rate != -8192)
                {
                    rr[j] = rate * 1.0;
                }
        }
    /* decode signal data */
    for (j = 0; j < ncell; j++)
        { /* pseudorange */
            prv = getbits(rtcm->buff, i, 20);
            i += 20;
            if (prv != -524288)
                {
                    pr[j] = prv * TWO_N29 * RANGE_MS;
                }
        }
    for (j = 0; j < ncell; j++)
        { /* phaserange */
            cpv = getbits(rtcm->buff, i, 24);
            i += 24;
            if (cpv != -8388608)
                {
                    cp[j] = cpv * TWO_N31 * RANGE_MS;
                }
        }
    for (j = 0; j < ncell; j++)
        { /* lock time */
            lock[j] = getbitu(rtcm->buff, i, 10);
            i += 10;
        }
    for (j = 0; j < ncell; j++)
        { /* half-cycle amiguity */
            half[j] = getbitu(rtcm->buff, i, 1);
            i += 1;
        }
    for (j = 0; j < ncell; j++)
        { /* cnr */
            cnr[j] = getbitu(rtcm->buff, i, 10) * 0.0625;
            i += 10;
        }
    for (j = 0; j < ncell; j++)
        { /* phaserangerate */
            rrv = getbits(rtcm->buff, i, 15);
            i += 15;
            if (rrv != -16384)
                {
                    rrf[j] = rrv * 0.0001;
                }
        }
    /* save obs data in msm message */
    save_msm_obs(rtcm, sys, &h, r, pr, cp, rr, rrf, cnr, lock, ex, half);

    rtcm->obsflag = !sync;
    return sync ? 0 : 1;
}


/* decode type 1230: glonass L1 and L2 code-phase biases ---------------------*/
int decode_type1230(rtcm_t *rtcm __attribute__((unused)))
{
    trace(2, "rtcm3 1230: not supported message\n");
    return 0;
}


/* decode rtcm ver.3 message -------------------------------------------------*/
int decode_rtcm3(rtcm_t *rtcm)
{
    double tow;
    int ret = 0;
    int type = getbitu(rtcm->buff, 24, 12);
    int week;

    trace(3, "decode_rtcm3: len=%3d type=%d\n", rtcm->len, type);

    if (rtcm->outtype)
        {
            std::snprintf(rtcm->msgtype, sizeof(rtcm->msgtype), "RTCM %4d (%4d):", type, rtcm->len);
        }
    /* real-time input option */
    if (strstr(rtcm->opt, "-RT_INP"))
        {
            tow = time2gpst(utc2gpst(timeget()), &week);
            rtcm->time = gpst2time(week, floor(tow));
        }
    switch (type)
        {
        case 1001:
            ret = decode_type1001(rtcm);
            break; /* not supported */
        case 1002:
            ret = decode_type1002(rtcm);
            break;
        case 1003:
            ret = decode_type1003(rtcm);
            break; /* not supported */
        case 1004:
            ret = decode_type1004(rtcm);
            break;
        case 1005:
            ret = decode_type1005(rtcm);
            break;
        case 1006:
            ret = decode_type1006(rtcm);
            break;
        case 1007:
            ret = decode_type1007(rtcm);
            break;
        case 1008:
            ret = decode_type1008(rtcm);
            break;
        case 1009:
            ret = decode_type1009(rtcm);
            break; /* not supported */
        case 1010:
            ret = decode_type1010(rtcm);
            break;
        case 1011:
            ret = decode_type1011(rtcm);
            break; /* not supported */
        case 1012:
            ret = decode_type1012(rtcm);
            break;
        case 1013:
            ret = decode_type1013(rtcm);
            break; /* not supported */
        case 1019:
            ret = decode_type1019(rtcm);
            break;
        case 1020:
            ret = decode_type1020(rtcm);
            break;
        case 1021:
            ret = decode_type1021(rtcm);
            break; /* not supported */
        case 1022:
            ret = decode_type1022(rtcm);
            break; /* not supported */
        case 1023:
            ret = decode_type1023(rtcm);
            break; /* not supported */
        case 1024:
            ret = decode_type1024(rtcm);
            break; /* not supported */
        case 1025:
            ret = decode_type1025(rtcm);
            break; /* not supported */
        case 1026:
            ret = decode_type1026(rtcm);
            break; /* not supported */
        case 1027:
            ret = decode_type1027(rtcm);
            break; /* not supported */
        case 1029:
            ret = decode_type1029(rtcm);
            break;
        case 1030:
            ret = decode_type1030(rtcm);
            break; /* not supported */
        case 1031:
            ret = decode_type1031(rtcm);
            break; /* not supported */
        case 1032:
            ret = decode_type1032(rtcm);
            break; /* not supported */
        case 1033:
            ret = decode_type1033(rtcm);
            break;
        case 1034:
            ret = decode_type1034(rtcm);
            break; /* not supported */
        case 1035:
            ret = decode_type1035(rtcm);
            break; /* not supported */
        case 1037:
            ret = decode_type1037(rtcm);
            break; /* not supported */
        case 1038:
            ret = decode_type1038(rtcm);
            break; /* not supported */
        case 1039:
            ret = decode_type1039(rtcm);
            break; /* not supported */
        case 1044:
            ret = decode_type1044(rtcm);
            break;
        case 1045:
            ret = decode_type1045(rtcm);
            break;
        case 1046:
            ret = decode_type1046(rtcm);
            break; /* extension for IGS MGEX */
        case 1047:
            ret = decode_type1047(rtcm);
            break; /* beidou ephemeris (tentative mt) */
        case 63:
            ret = decode_type63(rtcm);
            break; /* beidou ephemeris (rtcm draft) */
        case 1057:
            ret = decode_ssr1(rtcm, SYS_GPS);
            break;
        case 1058:
            ret = decode_ssr2(rtcm, SYS_GPS);
            break;
        case 1059:
            ret = decode_ssr3(rtcm, SYS_GPS);
            break;
        case 1060:
            ret = decode_ssr4(rtcm, SYS_GPS);
            break;
        case 1061:
            ret = decode_ssr5(rtcm, SYS_GPS);
            break;
        case 1062:
            ret = decode_ssr6(rtcm, SYS_GPS);
            break;
        case 1063:
            ret = decode_ssr1(rtcm, SYS_GLO);
            break;
        case 1064:
            ret = decode_ssr2(rtcm, SYS_GLO);
            break;
        case 1065:
            ret = decode_ssr3(rtcm, SYS_GLO);
            break;
        case 1066:
            ret = decode_ssr4(rtcm, SYS_GLO);
            break;
        case 1067:
            ret = decode_ssr5(rtcm, SYS_GLO);
            break;
        case 1068:
            ret = decode_ssr6(rtcm, SYS_GLO);
            break;
        case 1071:
            ret = decode_msm0(rtcm, SYS_GPS);
            break; /* not supported */
        case 1072:
            ret = decode_msm0(rtcm, SYS_GPS);
            break; /* not supported */
        case 1073:
            ret = decode_msm0(rtcm, SYS_GPS);
            break; /* not supported */
        case 1074:
            ret = decode_msm4(rtcm, SYS_GPS);
            break;
        case 1075:
            ret = decode_msm5(rtcm, SYS_GPS);
            break;
        case 1076:
            ret = decode_msm6(rtcm, SYS_GPS);
            break;
        case 1077:
            ret = decode_msm7(rtcm, SYS_GPS);
            break;
        case 1081:
            ret = decode_msm0(rtcm, SYS_GLO);
            break; /* not supported */
        case 1082:
            ret = decode_msm0(rtcm, SYS_GLO);
            break; /* not supported */
        case 1083:
            ret = decode_msm0(rtcm, SYS_GLO);
            break; /* not supported */
        case 1084:
            ret = decode_msm4(rtcm, SYS_GLO);
            break;
        case 1085:
            ret = decode_msm5(rtcm, SYS_GLO);
            break;
        case 1086:
            ret = decode_msm6(rtcm, SYS_GLO);
            break;
        case 1087:
            ret = decode_msm7(rtcm, SYS_GLO);
            break;
        case 1091:
            ret = decode_msm0(rtcm, SYS_GAL);
            break; /* not supported */
        case 1092:
            ret = decode_msm0(rtcm, SYS_GAL);
            break; /* not supported */
        case 1093:
            ret = decode_msm0(rtcm, SYS_GAL);
            break; /* not supported */
        case 1094:
            ret = decode_msm4(rtcm, SYS_GAL);
            break;
        case 1095:
            ret = decode_msm5(rtcm, SYS_GAL);
            break;
        case 1096:
            ret = decode_msm6(rtcm, SYS_GAL);
            break;
        case 1097:
            ret = decode_msm7(rtcm, SYS_GAL);
            break;
        case 1101:
            ret = decode_msm0(rtcm, SYS_SBS);
            break; /* not supported */
        case 1102:
            ret = decode_msm0(rtcm, SYS_SBS);
            break; /* not supported */
        case 1103:
            ret = decode_msm0(rtcm, SYS_SBS);
            break; /* not supported */
        case 1104:
            ret = decode_msm4(rtcm, SYS_SBS);
            break;
        case 1105:
            ret = decode_msm5(rtcm, SYS_SBS);
            break;
        case 1106:
            ret = decode_msm6(rtcm, SYS_SBS);
            break;
        case 1107:
            ret = decode_msm7(rtcm, SYS_SBS);
            break;
        case 1111:
            ret = decode_msm0(rtcm, SYS_QZS);
            break; /* not supported */
        case 1112:
            ret = decode_msm0(rtcm, SYS_QZS);
            break; /* not supported */
        case 1113:
            ret = decode_msm0(rtcm, SYS_QZS);
            break; /* not supported */
        case 1114:
            ret = decode_msm4(rtcm, SYS_QZS);
            break;
        case 1115:
            ret = decode_msm5(rtcm, SYS_QZS);
            break;
        case 1116:
            ret = decode_msm6(rtcm, SYS_QZS);
            break;
        case 1117:
            ret = decode_msm7(rtcm, SYS_QZS);
            break;
        case 1121:
            ret = decode_msm0(rtcm, SYS_BDS);
            break; /* not supported */
        case 1122:
            ret = decode_msm0(rtcm, SYS_BDS);
            break; /* not supported */
        case 1123:
            ret = decode_msm0(rtcm, SYS_BDS);
            break; /* not supported */
        case 1124:
            ret = decode_msm4(rtcm, SYS_BDS);
            break;
        case 1125:
            ret = decode_msm5(rtcm, SYS_BDS);
            break;
        case 1126:
            ret = decode_msm6(rtcm, SYS_BDS);
            break;
        case 1127:
            ret = decode_msm7(rtcm, SYS_BDS);
            break;
        case 1230:
            ret = decode_type1230(rtcm);
            break; /* not supported */
        case 1240:
            ret = decode_ssr1(rtcm, SYS_GAL);
            break;
        case 1241:
            ret = decode_ssr2(rtcm, SYS_GAL);
            break;
        case 1242:
            ret = decode_ssr3(rtcm, SYS_GAL);
            break;
        case 1243:
            ret = decode_ssr4(rtcm, SYS_GAL);
            break;
        case 1244:
            ret = decode_ssr5(rtcm, SYS_GAL);
            break;
        case 1245:
            ret = decode_ssr6(rtcm, SYS_GAL);
            break;
        case 1246:
            ret = decode_ssr1(rtcm, SYS_QZS);
            break;
        case 1247:
            ret = decode_ssr2(rtcm, SYS_QZS);
            break;
        case 1248:
            ret = decode_ssr3(rtcm, SYS_QZS);
            break;
        case 1249:
            ret = decode_ssr4(rtcm, SYS_QZS);
            break;
        case 1250:
            ret = decode_ssr5(rtcm, SYS_QZS);
            break;
        case 1251:
            ret = decode_ssr6(rtcm, SYS_QZS);
            break;
        case 1252:
            ret = decode_ssr1(rtcm, SYS_SBS);
            break;
        case 1253:
            ret = decode_ssr2(rtcm, SYS_SBS);
            break;
        case 1254:
            ret = decode_ssr3(rtcm, SYS_SBS);
            break;
        case 1255:
            ret = decode_ssr4(rtcm, SYS_SBS);
            break;
        case 1256:
            ret = decode_ssr5(rtcm, SYS_SBS);
            break;
        case 1257:
            ret = decode_ssr6(rtcm, SYS_SBS);
            break;
        case 1258:
            ret = decode_ssr1(rtcm, SYS_BDS);
            break;
        case 1259:
            ret = decode_ssr2(rtcm, SYS_BDS);
            break;
        case 1260:
            ret = decode_ssr3(rtcm, SYS_BDS);
            break;
        case 1261:
            ret = decode_ssr4(rtcm, SYS_BDS);
            break;
        case 1262:
            ret = decode_ssr5(rtcm, SYS_BDS);
            break;
        case 1263:
            ret = decode_ssr6(rtcm, SYS_BDS);
            break;
        case 2065:
            ret = decode_ssr7(rtcm, SYS_GPS);
            break; /* tentative */
        case 2066:
            ret = decode_ssr7(rtcm, SYS_GLO);
            break; /* tentative */
        case 2067:
            ret = decode_ssr7(rtcm, SYS_GAL);
            break; /* tentative */
        case 2068:
            ret = decode_ssr7(rtcm, SYS_QZS);
            break; /* tentative */
        case 2070:
            ret = decode_ssr7(rtcm, SYS_BDS);
            break; /* tentative */
        }
    if (ret >= 0)
        {
            type -= 1000;
            if (1 <= type && type <= 299)
                {
                    rtcm->nmsg3[type]++; /* 1001-1299 */
                }
            else if (1000 <= type && type <= 1099)
                {
                    rtcm->nmsg3[type - 700]++; /* 2000-2099 */
                }
            else
                {
                    rtcm->nmsg3[0]++;
                }
        }
    return ret;
}
