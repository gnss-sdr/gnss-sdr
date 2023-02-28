/*!
 * \file rtklib_rtksvr.cc
 * \brief rtk server functions
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

#include "rtklib_rtksvr.h"
#include "rtklib_preceph.h"
#include "rtklib_rtcm.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_rtkpos.h"
#include "rtklib_sbas.h"
#include "rtklib_solution.h"
#include "rtklib_stream.h"
#include <cstring>

/* write solution header to output stream ------------------------------------*/
void writesolhead(stream_t *stream, const solopt_t *solopt)
{
    unsigned char buff[MAXSOLMSG];
    int n;
    n = outsolheads(buff, solopt);
    strwrite(stream, buff, n);
}


/* save output buffer --------------------------------------------------------*/
void saveoutbuf(rtksvr_t *svr, unsigned char *buff, int n, int index)
{
    rtksvrlock(svr);

    n = n < svr->buffsize - svr->nsb[index] ? n : svr->buffsize - svr->nsb[index];
    memcpy(svr->sbuf[index] + svr->nsb[index], buff, n);
    svr->nsb[index] += n;

    rtksvrunlock(svr);
}


/* write solution to output stream -------------------------------------------*/
void writesol(rtksvr_t *svr, int index)
{
    solopt_t solopt = SOLOPT_DEFAULT;
    unsigned char buff[MAXSOLMSG];
    int i;
    int n;

    tracet(4, "writesol: index=%d\n", index);

    for (i = 0; i < 2; i++)
        {
            /* output solution */
            n = outsols(buff, &svr->rtk.sol, svr->rtk.rb, svr->solopt + i);
            strwrite(svr->stream + i + 3, buff, n);

            /* save output buffer */
            saveoutbuf(svr, buff, n, i);

            /* output extended solution */
            n = outsolexs(buff, &svr->rtk.sol, svr->rtk.ssat, svr->solopt + i);
            strwrite(svr->stream + i + 3, buff, n);

            /* save output buffer */
            saveoutbuf(svr, buff, n, i);
        }
    /* output solution to monitor port */
    if (svr->moni)
        {
            n = outsols(buff, &svr->rtk.sol, svr->rtk.rb, &solopt);
            strwrite(svr->moni, buff, n);
        }
    /* save solution buffer */
    if (svr->nsol < MAXSOLBUF)
        {
            rtksvrlock(svr);
            svr->solbuf[svr->nsol++] = svr->rtk.sol;
            rtksvrunlock(svr);
        }
}


/* update navigation data ----------------------------------------------------*/
void updatenav(nav_t *nav)
{
    int i;
    int j;
    for (i = 0; i < MAXSAT; i++)
        {
            for (j = 0; j < NFREQ; j++)
                {
                    nav->lam[i][j] = satwavelen(i + 1, j, nav);
                }
        }
}


/* update glonass frequency channel number in raw data struct ----------------*/
void updatefcn(rtksvr_t *svr)
{
    int i;
    int j;
    int sat;
    int frq;

    for (i = 0; i < MAXPRNGLO; i++)
        {
            sat = satno(SYS_GLO, i + 1);

            for (j = 0, frq = -999; j < 3; j++)
                {
                    if (svr->raw[j].nav.geph[i].sat != sat)
                        {
                            continue;
                        }
                    frq = svr->raw[j].nav.geph[i].frq;
                }
            if (frq < -7 || frq > 6)
                {
                    continue;
                }

            for (j = 0; j < 3; j++)
                {
                    if (svr->raw[j].nav.geph[i].sat == sat)
                        {
                            continue;
                        }
                    svr->raw[j].nav.geph[i].sat = sat;
                    svr->raw[j].nav.geph[i].frq = frq;
                }
        }
}


/* update rtk server struct --------------------------------------------------*/
void updatesvr(rtksvr_t *svr, int ret, obs_t *obs, nav_t *nav, int sat,
    sbsmsg_t *sbsmsg, int index, int iobs)
{
    eph_t *eph1;
    eph_t *eph2;
    eph_t *eph3;
    geph_t *geph1;
    geph_t *geph2;
    geph_t *geph3;
    // gtime_t tof;
    double pos[3];
    double del[3] = {0};
    double dr[3];
    int i;
    int n = 0;
    int prn;
    int sbssat = svr->rtk.opt.sbassatsel;
    int sys;
    int iode;

    tracet(4, "updatesvr: ret=%d sat=%2d index=%d\n", ret, sat, index);

    if (ret == 1)
        { /* observation data */
            if (iobs < MAXOBSBUF)
                {
                    for (i = 0; i < obs->n; i++)
                        {
                            if (svr->rtk.opt.exsats[obs->data[i].sat - 1] == 1 ||
                                !(satsys(obs->data[i].sat, nullptr) & svr->rtk.opt.navsys))
                                {
                                    continue;
                                }
                            svr->obs[index][iobs].data[n] = obs->data[i];
                            svr->obs[index][iobs].data[n++].rcv = index + 1;
                        }
                    svr->obs[index][iobs].n = n;
                    sortobs(&svr->obs[index][iobs]);
                }
            svr->nmsg[index][0]++;
        }
    else if (ret == 2)
        { /* ephemeris */
            if (satsys(sat, &prn) != SYS_GLO)
                {
                    if (!svr->navsel || svr->navsel == index + 1)
                        {
                            eph1 = nav->eph + sat - 1;
                            eph2 = svr->nav.eph + sat - 1;
                            eph3 = svr->nav.eph + sat - 1 + MAXSAT;
                            if (eph2->ttr.time == 0 ||
                                (eph1->iode != eph3->iode && eph1->iode != eph2->iode) ||
                                (timediff(eph1->toe, eph3->toe) != 0.0 &&
                                    timediff(eph1->toe, eph2->toe) != 0.0))
                                {
                                    *eph3 = *eph2;
                                    *eph2 = *eph1;
                                    updatenav(&svr->nav);
                                }
                        }
                    svr->nmsg[index][1]++;
                }
            else
                {
                    if (!svr->navsel || svr->navsel == index + 1)
                        {
                            geph1 = nav->geph + prn - 1;
                            geph2 = svr->nav.geph + prn - 1;
                            geph3 = svr->nav.geph + prn - 1 + MAXPRNGLO;
                            if (geph2->tof.time == 0 ||
                                (geph1->iode != geph3->iode && geph1->iode != geph2->iode))
                                {
                                    *geph3 = *geph2;
                                    *geph2 = *geph1;
                                    updatenav(&svr->nav);
                                    updatefcn(svr);
                                }
                        }
                    svr->nmsg[index][6]++;
                }
        }
    else if (ret == 3)
        { /* sbas message */
            if (sbsmsg && (sbssat == sbsmsg->prn || sbssat == 0))
                {
                    if (svr->nsbs < MAXSBSMSG)
                        {
                            svr->sbsmsg[svr->nsbs++] = *sbsmsg;
                        }
                    else
                        {
                            for (i = 0; i < MAXSBSMSG - 1; i++)
                                {
                                    svr->sbsmsg[i] = svr->sbsmsg[i + 1];
                                }
                            svr->sbsmsg[i] = *sbsmsg;
                        }
                    sbsupdatecorr(sbsmsg, &svr->nav);
                }
            svr->nmsg[index][3]++;
        }
    else if (ret == 9)
        { /* ion/utc parameters */
            if (svr->navsel == index || svr->navsel >= 3)
                {
                    for (i = 0; i < 8; i++)
                        {
                            svr->nav.ion_gps[i] = nav->ion_gps[i];
                        }
                    for (i = 0; i < 4; i++)
                        {
                            svr->nav.utc_gps[i] = nav->utc_gps[i];
                        }
                    for (i = 0; i < 4; i++)
                        {
                            svr->nav.ion_gal[i] = nav->ion_gal[i];
                        }
                    for (i = 0; i < 4; i++)
                        {
                            svr->nav.utc_gal[i] = nav->utc_gal[i];
                        }
                    for (i = 0; i < 8; i++)
                        {
                            svr->nav.ion_qzs[i] = nav->ion_qzs[i];
                        }
                    for (i = 0; i < 4; i++)
                        {
                            svr->nav.utc_qzs[i] = nav->utc_qzs[i];
                        }
                    svr->nav.leaps = nav->leaps;
                }
            svr->nmsg[index][2]++;
        }
    else if (ret == 5)
        { /* antenna position parameters */
            if (svr->rtk.opt.refpos == 4 && index == 1)
                {
                    for (i = 0; i < 3; i++)
                        {
                            svr->rtk.rb[i] = svr->rtcm[1].sta.pos[i];
                        }
                    /* antenna delta */
                    ecef2pos(svr->rtk.rb, pos);
                    if (svr->rtcm[1].sta.deltype)
                        { /* xyz */
                            del[2] = svr->rtcm[1].sta.hgt;
                            enu2ecef(pos, del, dr);
                            for (i = 0; i < 3; i++)
                                {
                                    svr->rtk.rb[i] += svr->rtcm[1].sta.del[i] + dr[i];
                                }
                        }
                    else
                        { /* enu */
                            enu2ecef(pos, svr->rtcm[1].sta.del, dr);
                            for (i = 0; i < 3; i++)
                                {
                                    svr->rtk.rb[i] += dr[i];
                                }
                        }
                }
            svr->nmsg[index][4]++;
        }
    else if (ret == 7)
        { /* dgps correction */
            svr->nmsg[index][5]++;
        }
    else if (ret == 10)
        { /* ssr message */
            for (i = 0; i < MAXSAT; i++)
                {
                    if (!svr->rtcm[index].ssr[i].update)
                        {
                            continue;
                        }
                    svr->rtcm[index].ssr[i].update = 0;

                    iode = svr->rtcm[index].ssr[i].iode;
                    sys = satsys(i + 1, &prn);

                    /* check corresponding ephemeris exists */
                    if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS)
                        {
                            if (svr->nav.eph[i].iode != iode &&
                                svr->nav.eph[i + MAXSAT].iode != iode)
                                {
                                    continue;
                                }
                        }
                    else if (sys == SYS_GLO)
                        {
                            if (svr->nav.geph[prn - 1].iode != iode &&
                                svr->nav.geph[prn - 1 + MAXPRNGLO].iode != iode)
                                {
                                    continue;
                                }
                        }
                    svr->nav.ssr[i] = svr->rtcm[index].ssr[i];
                }
            svr->nmsg[index][7]++;
        }
    else if (ret == 31)
        { /* lex message */
            // lexupdatecorr(&svr->raw[index].lexmsg, &svr->nav, &tof);
            svr->nmsg[index][8]++;
        }
    else if (ret == -1)
        { /* error */
            svr->nmsg[index][9]++;
        }
}


/* decode receiver raw/rtcm data ---------------------------------------------*/
int decoderaw(rtksvr_t *svr, int index)
{
    obs_t *obs;
    nav_t *nav;
    sbsmsg_t *sbsmsg = nullptr;
    int i;
    int ret = 0;
    int sat;
    int fobs = 0;

    tracet(4, "decoderaw: index=%d\n", index);

    rtksvrlock(svr);

    for (i = 0; i < svr->nb[index]; i++)
        {
            /* input rtcm/receiver raw data from stream */
            if (svr->format[index] == STRFMT_RTCM2)
                {
                    ret = input_rtcm2(svr->rtcm + index, svr->buff[index][i]);
                    obs = &svr->rtcm[index].obs;
                    nav = &svr->rtcm[index].nav;
                    sat = svr->rtcm[index].ephsat;
                }
            else if (svr->format[index] == STRFMT_RTCM3)
                {
                    ret = input_rtcm3(svr->rtcm + index, svr->buff[index][i]);
                    obs = &svr->rtcm[index].obs;
                    nav = &svr->rtcm[index].nav;
                    sat = svr->rtcm[index].ephsat;
                }
            else
                {
                    // Disabled !!
                    // ret = input_raw(svr->raw+index, svr->format[index], svr->buff[index][i]);
                    obs = &svr->raw[index].obs;
                    nav = &svr->raw[index].nav;
                    sat = svr->raw[index].ephsat;
                    sbsmsg = &svr->raw[index].sbsmsg;
                }
#if 0 /* record for receiving tick */
            if (ret == 1)
                {
                    trace(0, "%d %10d T=%s NS=%2d\n", index, tickget(),
                            time_str(obs->data[0].time, 0), obs->n);
                }
#endif
            /* update rtk server */
            if (ret > 0)
                {
                    updatesvr(svr, ret, obs, nav, sat, sbsmsg, index, fobs);
                }

            /* observation data received */
            if (ret == 1)
                {
                    if (fobs < MAXOBSBUF)
                        {
                            fobs++;
                        }
                    else
                        {
                            svr->prcout++;
                        }
                }
        }
    svr->nb[index] = 0;

    rtksvrunlock(svr);

    return fobs;
}


/* decode download file ------------------------------------------------------*/
void decodefile(rtksvr_t *svr, int index)
{
    nav_t nav{};
    char file[1024];
    int nb;

    tracet(4, "decodefile: index=%d\n", index);

    rtksvrlock(svr);

    /* check file path completed */
    if ((nb = svr->nb[index]) <= 2 ||
        svr->buff[index][nb - 2] != '\r' || svr->buff[index][nb - 1] != '\n')
        {
            rtksvrunlock(svr);
            return;
        }
    strncpy(file, reinterpret_cast<char *>(svr->buff[index]), nb - 2);
    file[nb - 2] = '\0';
    svr->nb[index] = 0;

    rtksvrunlock(svr);

    if (svr->format[index] == STRFMT_SP3)
        { /* precise ephemeris */
            /* read sp3 precise ephemeris */
            readsp3(file, &nav, 0);
            if (nav.ne <= 0)
                {
                    tracet(1, "sp3 file read error: %s\n", file);
                    return;
                }
            /* update precise ephemeris */
            rtksvrlock(svr);

            if (svr->nav.peph)
                {
                    free(svr->nav.peph);
                }
            svr->nav.ne = svr->nav.nemax = nav.ne;
            svr->nav.peph = nav.peph;
            svr->ftime[index] = utc2gpst(timeget());
            std::strncpy(svr->files[index], file, MAXSTRPATH);

            rtksvrunlock(svr);
        }
    else if (svr->format[index] == STRFMT_RNXCLK)
        {
            /* precise clock */
            /* read rinex clock */  // Disabled!!
            if (true /*readrnxc(file, &nav)<=0 */)
                {
                    tracet(1, "rinex clock file read error: %s\n", file);
                    return;
                }
            /* update precise clock */
            rtksvrlock(svr);

            if (svr->nav.pclk)
                {
                    free(svr->nav.pclk);
                }
            svr->nav.nc = svr->nav.ncmax = nav.nc;
            svr->nav.pclk = nav.pclk;
            svr->ftime[index] = utc2gpst(timeget());
            std::strncpy(svr->files[index], file, MAXSTRPATH);

            rtksvrunlock(svr);
        }
}


/* rtk server thread ---------------------------------------------------------*/
void *rtksvrthread(void *arg)
{
    auto *svr = static_cast<rtksvr_t *>(arg);
    obs_t obs;
    obsd_t data[MAXOBS * 2];
    double tt;
    unsigned int tick;
    unsigned int ticknmea;
    unsigned char *p;
    unsigned char *q;
    int i;
    int j;
    int n;
    int fobs[3] = {0};
    int cycle;
    int cputime;

    tracet(3, "rtksvrthread:\n");

    svr->state = 1;
    obs.data = data;
    svr->tick = tickget();
    ticknmea = svr->tick - 1000;

    for (cycle = 0; svr->state; cycle++)
        {
            tick = tickget();

            for (i = 0; i < 3; i++)
                {
                    p = svr->buff[i] + svr->nb[i];
                    q = svr->buff[i] + svr->buffsize;

                    /* read receiver raw/rtcm data from input stream */
                    if ((n = strread(svr->stream + i, p, static_cast<int>(q[0]) - static_cast<int>(p[0]))) <= 0)
                        {
                            continue;
                        }
                    /* write receiver raw/rtcm data to log stream */
                    strwrite(svr->stream + i + 5, p, n);
                    svr->nb[i] += n;

                    /* save peek buffer */
                    rtksvrlock(svr);
                    n = n < svr->buffsize - svr->npb[i] ? n : svr->buffsize - svr->npb[i];
                    memcpy(svr->pbuf[i] + svr->npb[i], p, n);
                    svr->npb[i] += n;
                    rtksvrunlock(svr);
                }
            for (i = 0; i < 3; i++)
                {
                    if (svr->format[i] == STRFMT_SP3 || svr->format[i] == STRFMT_RNXCLK)
                        {
                            /* decode download file */
                            decodefile(svr, i);
                        }
                    else
                        {
                            /* decode receiver raw/rtcm data */
                            fobs[i] = decoderaw(svr, i);
                        }
                }
            for (i = 0; i < fobs[0]; i++)
                { /* for each rover observation data */
                    obs.n = 0;
                    for (j = 0; j < svr->obs[0][i].n && obs.n < MAXOBS * 2; j++)
                        {
                            obs.data[obs.n++] = svr->obs[0][i].data[j];
                        }
                    for (j = 0; j < svr->obs[1][0].n && obs.n < MAXOBS * 2; j++)
                        {
                            obs.data[obs.n++] = svr->obs[1][0].data[j];
                        }
                    /* rtk positioning */
                    rtksvrlock(svr);
                    rtkpos(&svr->rtk, obs.data, obs.n, &svr->nav);
                    rtksvrunlock(svr);

                    if (svr->rtk.sol.stat != SOLQ_NONE)
                        {
                            /* adjust current time */
                            tt = static_cast<int>(tickget() - tick) / 1000.0 + DTTOL;
                            timeset(gpst2utc(timeadd(svr->rtk.sol.time, tt)));

                            /* write solution */
                            writesol(svr, i);
                        }
                    /* if cpu overload, inclement obs outage counter and break */
                    if (static_cast<int>(tickget() - tick) >= svr->cycle)
                        {
                            svr->prcout += fobs[0] - i - 1;
#if 0 /* omitted v.2.4.1 */
                            break;
#endif
                        }
                }
            /* send null solution if no solution (1hz) */
            if (svr->rtk.sol.stat == SOLQ_NONE && cycle % (1000 / svr->cycle) == 0)
                {
                    writesol(svr, 0);
                }
            /* send nmea request to base/nrtk input stream */
            if (svr->nmeacycle > 0 && static_cast<int>(tick - ticknmea) >= svr->nmeacycle)
                {
                    if (svr->stream[1].state == 1)
                        {
                            if (svr->nmeareq == 1)
                                {
                                    strsendnmea(svr->stream + 1, svr->nmeapos);
                                }
                            else if (svr->nmeareq == 2 && norm_rtk(svr->rtk.sol.rr, 3) > 0.0)
                                {
                                    strsendnmea(svr->stream + 1, svr->rtk.sol.rr);
                                }
                        }
                    ticknmea = tick;
                }
            if ((cputime = static_cast<int>(tickget() - tick)) > 0)
                {
                    svr->cputime = cputime;
                }

            /* sleep until next cycle */
            sleepms(svr->cycle - cputime);
        }
    for (i = 0; i < MAXSTRRTK; i++)
        {
            strclose(svr->stream + i);
        }
    for (i = 0; i < 3; i++)
        {
            svr->nb[i] = svr->npb[i] = 0;
            free(svr->buff[i]);
            svr->buff[i] = nullptr;
            free(svr->pbuf[i]);
            svr->pbuf[i] = nullptr;
            // free_raw (svr->raw +i);
            free_rtcm(svr->rtcm + i);
        }
    for (i = 0; i < 2; i++)
        {
            svr->nsb[i] = 0;
            free(svr->sbuf[i]);
            svr->sbuf[i] = nullptr;
        }
    return nullptr;
}


/* initialize rtk server -------------------------------------------------------
 * initialize rtk server
 * args   : rtksvr_t *svr    IO rtk server
 * return : status (0:error, 1:ok)
 *-----------------------------------------------------------------------------*/
int rtksvrinit(rtksvr_t *svr)
{
    gtime_t time0 = {0, 0.0};
    sol_t sol0 = {{0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
        '0', '0', '0', 0, 0, 0};
    eph_t eph0 = {0, -1, -1, 0, 0, 0, 0, 0, {0, 0.0}, {0, 0.0}, {0, 0.0},
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {0.0}, {0.0}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false};
    geph_t geph0 = {0, -1, 0, 0, 0, 0, {0, 0.0}, {0, 0.0}, {0.0}, {0.0}, {0.0},
        0.0, 0.0, 0.0};
    seph_t seph0 = {0, {0, 0.0}, {0, 0.0}, 0, 0, {0.0}, {0.0}, {0.0}, 0.0, 0.0};
    int i;
    int j;

    tracet(3, "rtksvrinit:\n");

    svr->state = svr->cycle = svr->nmeacycle = svr->nmeareq = 0;
    for (i = 0; i < 3; i++)
        {
            svr->nmeapos[i] = 0.0;
        }
    svr->buffsize = 0;
    for (i = 0; i < 3; i++)
        {
            svr->format[i] = 0;
        }
    for (i = 0; i < 2; i++)
        {
            svr->solopt[i] = SOLOPT_DEFAULT;
        }
    svr->navsel = svr->nsbs = svr->nsol = 0;
    rtkinit(&svr->rtk, &PRCOPT_DEFAULT);
    for (i = 0; i < 3; i++)
        {
            svr->nb[i] = 0;
        }
    for (i = 0; i < 2; i++)
        {
            svr->nsb[i] = 0;
        }
    for (i = 0; i < 3; i++)
        {
            svr->npb[i] = 0;
        }
    for (i = 0; i < 3; i++)
        {
            svr->buff[i] = nullptr;
        }
    for (i = 0; i < 2; i++)
        {
            svr->sbuf[i] = nullptr;
        }
    for (i = 0; i < 3; i++)
        {
            svr->pbuf[i] = nullptr;
        }
    for (i = 0; i < MAXSOLBUF; i++)
        {
            svr->solbuf[i] = sol0;
        }
    for (i = 0; i < 3; i++)
        {
            for (j = 0; j < 10; j++)
                {
                    svr->nmsg[i][j] = 0;
                }
        }
    for (i = 0; i < 3; i++)
        {
            svr->ftime[i] = time0;
        }
    for (i = 0; i < 3; i++)
        {
            svr->files[i][0] = '\0';
        }
    svr->moni = nullptr;
    svr->tick = 0;
    svr->thread = 0;  // NOLINT
    svr->cputime = svr->prcout = 0;

    if (!(svr->nav.eph = static_cast<eph_t *>(malloc(sizeof(eph_t) * MAXSAT * 2))) ||
        !(svr->nav.geph = static_cast<geph_t *>(malloc(sizeof(geph_t) * NSATGLO * 2))) ||
        !(svr->nav.seph = static_cast<seph_t *>(malloc(sizeof(seph_t) * NSATSBS * 2))))
        {
            tracet(1, "rtksvrinit: malloc error\n");
            return 0;
        }
    for (i = 0; i < MAXSAT * 2; i++)
        {
            svr->nav.eph[i] = eph0;
        }
    for (i = 0; i < NSATGLO * 2; i++)
        {
            svr->nav.geph[i] = geph0;
        }
    for (i = 0; i < NSATSBS * 2; i++)
        {
            svr->nav.seph[i] = seph0;
        }
    svr->nav.n = MAXSAT * 2;
    svr->nav.ng = NSATGLO * 2;
    svr->nav.ns = NSATSBS * 2;

    for (i = 0; i < 3; i++)
        {
            for (j = 0; j < MAXOBSBUF; j++)
                {
                    if (!(svr->obs[i][j].data = static_cast<obsd_t *>(malloc(sizeof(obsd_t) * MAXOBS))))
                        {
                            tracet(1, "rtksvrinit: malloc error\n");
                            return 0;
                        }
                }
        }
    for (i = 0; i < 3; i++)
        {
            memset(svr->raw + i, 0, sizeof(raw_t));
            memset(svr->rtcm + i, 0, sizeof(rtcm_t));
        }
    for (i = 0; i < MAXSTRRTK; i++)
        {
            strinit(svr->stream + i);
        }

    initlock(&svr->lock);

    return 1;
}


/* free rtk server -------------------------------------------------------------
 * free rtk server
 * args   : rtksvr_t *svr    IO rtk server
 * return : none
 *-----------------------------------------------------------------------------*/
void rtksvrfree(rtksvr_t *svr)
{
    int i;
    int j;

    free(svr->nav.eph);
    free(svr->nav.geph);
    free(svr->nav.seph);
    for (i = 0; i < 3; i++)
        {
            for (j = 0; j < MAXOBSBUF; j++)
                {
                    free(svr->obs[i][j].data);
                }
        }
}


/* lock/unlock rtk server ------------------------------------------------------
 * lock/unlock rtk server
 * args   : rtksvr_t *svr    IO rtk server
 * return : status (1:ok 0:error)
 *-----------------------------------------------------------------------------*/
void rtksvrlock(rtksvr_t *svr) { rtk_lock(&svr->lock); }


void rtksvrunlock(rtksvr_t *svr) { rtk_unlock(&svr->lock); }


/* start rtk server ------------------------------------------------------------
 * start rtk server thread
 * args   : rtksvr_t *svr    IO rtk server
 *          int     cycle    I  server cycle (ms)
 *          int     buffsize I  input buffer size (bytes)
 *          int     *strs    I  stream types (STR_???)
 *                              types[0]=input stream rover
 *                              types[1]=input stream base station
 *                              types[2]=input stream correction
 *                              types[3]=output stream solution 1
 *                              types[4]=output stream solution 2
 *                              types[5]=log stream rover
 *                              types[6]=log stream base station
 *                              types[7]=log stream correction
 *          char    *paths   I  input stream paths
 *          int     *format  I  input stream formats (STRFMT_???)
 *                              format[0]=input stream rover
 *                              format[1]=input stream base station
 *                              format[2]=input stream correction
 *          int     navsel   I  navigation message select
 *                              (0:rover, 1:base, 2:ephem, 3:all)
 *          char    **cmds   I  input stream start commands
 *                              cmds[0]=input stream rover (NULL: no command)
 *                              cmds[1]=input stream base (NULL: no command)
 *                              cmds[2]=input stream corr (NULL: no command)
 *          char    **rcvopts I receiver options
 *                              rcvopt[0]=receiver option rover
 *                              rcvopt[1]=receiver option base
 *                              rcvopt[2]=receiver option corr
 *          int     nmeacycle I nmea request cycle (ms) (0:no request)
 *          int     nmeareq  I  nmea request type (0:no, 1:base pos, 2:single sol)
 *          double *nmeapos  I  transmitted nmea position (ecef) (m)
 *          prcopt_t *prcopt I  rtk processing options
 *          solopt_t *solopt I  solution options
 *                              solopt[0]=solution 1 options
 *                              solopt[1]=solution 2 options
 *          stream_t *moni   I  monitor stream (NULL: not used)
 * return : status (1:ok 0:error)
 *-----------------------------------------------------------------------------*/
int rtksvrstart(rtksvr_t *svr, int cycle, int buffsize, int *strs,
    char **paths, const int *formats, int navsel, char **cmds,
    char **rcvopts, int nmeacycle, int nmeareq,
    const double *nmeapos, prcopt_t *prcopt,
    solopt_t *solopt, stream_t *moni)
{
    gtime_t time;
    gtime_t time0 = {0, 0.0};
    int i;
    int j;
    int rw;

    tracet(3, "rtksvrstart: cycle=%d buffsize=%d navsel=%d nmeacycle=%d nmeareq=%d\n",
        cycle, buffsize, navsel, nmeacycle, nmeareq);

    if (svr->state)
        {
            return 0;
        }

    strinitcom();
    svr->cycle = cycle > 1 ? cycle : 1;
    svr->nmeacycle = nmeacycle > 1000 ? nmeacycle : 1000;
    svr->nmeareq = nmeareq;
    for (i = 0; i < 3; i++)
        {
            svr->nmeapos[i] = nmeapos[i];
        }
    svr->buffsize = buffsize > 4096 ? buffsize : 4096;
    for (i = 0; i < 3; i++)
        {
            svr->format[i] = formats[i];
        }
    svr->navsel = navsel;
    svr->nsbs = 0;
    svr->nsol = 0;
    svr->prcout = 0;
    rtkfree(&svr->rtk);
    rtkinit(&svr->rtk, prcopt);

    for (i = 0; i < 3; i++)
        { /* input/log streams */
            svr->nb[i] = svr->npb[i] = 0;
            if (!(svr->buff[i] = static_cast<unsigned char *>(malloc(buffsize))) ||
                !(svr->pbuf[i] = static_cast<unsigned char *>(malloc(buffsize))))
                {
                    tracet(1, "rtksvrstart: malloc error\n");
                    return 0;
                }
            for (j = 0; j < 10; j++)
                {
                    svr->nmsg[i][j] = 0;
                }
            for (j = 0; j < MAXOBSBUF; j++)
                {
                    svr->obs[i][j].n = 0;
                }

            /* initialize receiver raw and rtcm control */
            // init_raw (svr->raw +i);
            init_rtcm(svr->rtcm + i);

            /* set receiver and rtcm option */
            if (strlen(rcvopts[i]) < 256)
                {
                    std::strncpy(svr->raw[i].opt, rcvopts[i], 256);
                    svr->raw[i].opt[255] = '\0';
                }
            if (strlen(rcvopts[i]) < 256)
                {
                    std::strncpy(svr->rtcm[i].opt, rcvopts[i], 256);
                    svr->rtcm[i].opt[255] = '\0';
                }

            /* connect dgps corrections */
            svr->rtcm[i].dgps = svr->nav.dgps;
        }
    for (i = 0; i < 2; i++)
        { /* output peek buffer */
            if (!(svr->sbuf[i] = static_cast<unsigned char *>(malloc(buffsize))))
                {
                    tracet(1, "rtksvrstart: malloc error\n");
                    return 0;
                }
        }
    /* set solution options */
    for (i = 0; i < 2; i++)
        {
            svr->solopt[i] = solopt[i];
        }
    /* set base station position */
    for (i = 0; i < 6; i++)
        {
            svr->rtk.rb[i] = i < 3 ? prcopt->rb[i] : 0.0;
        }
    /* update navigation data */
    for (i = 0; i < MAXSAT * 2; i++)
        {
            svr->nav.eph[i].ttr = time0;
        }
    for (i = 0; i < NSATGLO * 2; i++)
        {
            svr->nav.geph[i].tof = time0;
        }
    for (i = 0; i < NSATSBS * 2; i++)
        {
            svr->nav.seph[i].tof = time0;
        }
    updatenav(&svr->nav);

    /* set monitor stream */
    svr->moni = moni;

    /* open input streams */
    for (i = 0; i < 8; i++)
        {
            rw = i < 3 ? STR_MODE_R : STR_MODE_W;
            if (strs[i] != STR_FILE)
                {
                    rw |= STR_MODE_W;
                }
            if (!stropen(svr->stream + i, strs[i], rw, paths[i]))
                {
                    for (i--; i >= 0; i--)
                        {
                            strclose(svr->stream + i);
                        }
                    return 0;
                }
            /* set initial time for rtcm and raw */
            if (i < 3)
                {
                    time = utc2gpst(timeget());
                    svr->raw[i].time = strs[i] == STR_FILE ? strgettime(svr->stream + i) : time;
                    svr->rtcm[i].time = strs[i] == STR_FILE ? strgettime(svr->stream + i) : time;
                }
        }
    /* sync input streams */
    strsync(svr->stream, svr->stream + 1);
    strsync(svr->stream, svr->stream + 2);

    /* write start commands to input streams */
    for (i = 0; i < 3; i++)
        {
            if (cmds[i])
                {
                    strsendcmd(svr->stream + i, cmds[i]);
                }
        }
    /* write solution header to solution streams */
    for (i = 3; i < 5; i++)
        {
            writesolhead(svr->stream + i, svr->solopt + i - 3);
        }
    /* create rtk server thread */
    if (pthread_create(&svr->thread, nullptr, rtksvrthread, svr))
        {
            for (i = 0; i < MAXSTRRTK; i++)
                {
                    strclose(svr->stream + i);
                }
            return 0;
        }
    return 1;
}


/* stop rtk server -------------------------------------------------------------
 * start rtk server thread
 * args   : rtksvr_t *svr    IO rtk server
 *          char    **cmds   I  input stream stop commands
 *                              cmds[0]=input stream rover (NULL: no command)
 *                              cmds[1]=input stream base  (NULL: no command)
 *                              cmds[2]=input stream ephem (NULL: no command)
 * return : none
 *-----------------------------------------------------------------------------*/
void rtksvrstop(rtksvr_t *svr, char **cmds)
{
    int i;

    tracet(3, "rtksvrstop:\n");

    /* write stop commands to input streams */
    rtksvrlock(svr);
    for (i = 0; i < 3; i++)
        {
            if (cmds[i])
                {
                    strsendcmd(svr->stream + i, cmds[i]);
                }
        }
    rtksvrunlock(svr);

    /* stop rtk server */
    svr->state = 0;

    /* free rtk server thread */
    pthread_join(svr->thread, nullptr);
}


/* open output/log stream ------------------------------------------------------
 * open output/log stream
 * args   : rtksvr_t *svr    IO rtk server
 *          int     index    I  output/log stream index
 *                              (3:solution 1, 4:solution 2, 5:log rover,
 *                               6:log base station, 7:log correction)
 *          int     str      I  output/log stream types (STR_???)
 *          char    *path    I  output/log stream path
 *          solopt_t *solopt I  solution options
 * return : status (1:ok 0:error)
 *-----------------------------------------------------------------------------*/
int rtksvropenstr(rtksvr_t *svr, int index, int str, const char *path,
    const solopt_t *solopt)
{
    tracet(3, "rtksvropenstr: index=%d str=%d path=%s\n", index, str, path);

    if (index < 3 || index > 7 || !svr->state)
        {
            return 0;
        }

    rtksvrlock(svr);

    if (svr->stream[index].state > 0)
        {
            rtksvrunlock(svr);
            return 0;
        }
    if (!stropen(svr->stream + index, str, STR_MODE_W, path))
        {
            tracet(2, "stream open error: index=%d\n", index);
            rtksvrunlock(svr);
            return 0;
        }
    if (index <= 4)
        {
            svr->solopt[index - 3] = *solopt;

            /* write solution header to solution stream */
            writesolhead(svr->stream + index, svr->solopt + index - 3);
        }
    rtksvrunlock(svr);
    return 1;
}


/* close output/log stream -----------------------------------------------------
 * close output/log stream
 * args   : rtksvr_t *svr    IO rtk server
 *          int     index    I  output/log stream index
 *                              (3:solution 1, 4:solution 2, 5:log rover,
 *                               6:log base station, 7:log correction)
 * return : none
 *-----------------------------------------------------------------------------*/
void rtksvrclosestr(rtksvr_t *svr, int index)
{
    tracet(3, "rtksvrclosestr: index=%d\n", index);

    if (index < 3 || index > 7 || !svr->state)
        {
            return;
        }

    rtksvrlock(svr);

    strclose(svr->stream + index);

    rtksvrunlock(svr);
}


/* get observation data status -------------------------------------------------
 * get current observation data status
 * args   : rtksvr_t *svr    I  rtk server
 *          int     rcv      I  receiver (0:rover, 1:base, 2:ephem)
 *          gtime_t *time    O  time of observation data
 *          int     *sat     O  satellite prn numbers
 *          double  *az      O  satellite azimuth angles (rad)
 *          double  *el      O  satellite elevation angles (rad)
 *          int     **snr    O  satellite snr for each freq (dBHz)
 *                              snr[i][j] = sat i freq j snr
 *          int     *vsat    O  valid satellite flag
 * return : number of satellites
 *-----------------------------------------------------------------------------*/
int rtksvrostat(rtksvr_t *svr, int rcv, gtime_t *time, int *sat,
    double *az, double *el, int **snr, int *vsat)
{
    int i;
    int j;
    int ns;

    tracet(4, "rtksvrostat: rcv=%d\n", rcv);

    if (!svr->state)
        {
            return 0;
        }
    rtksvrlock(svr);
    ns = svr->obs[rcv][0].n;
    if (ns > 0)
        {
            *time = svr->obs[rcv][0].data[0].time;
        }
    for (i = 0; i < ns; i++)
        {
            sat[i] = svr->obs[rcv][0].data[i].sat;
            az[i] = svr->rtk.ssat[sat[i] - 1].azel[0];
            el[i] = svr->rtk.ssat[sat[i] - 1].azel[1];
            for (j = 0; j < NFREQ; j++)
                {
                    snr[i][j] = static_cast<int>(svr->obs[rcv][0].data[i].SNR[j] * 0.25);
                }
            if (svr->rtk.sol.stat == SOLQ_NONE || svr->rtk.sol.stat == SOLQ_SINGLE)
                {
                    vsat[i] = svr->rtk.ssat[sat[i] - 1].vs;
                }
            else
                {
                    vsat[i] = svr->rtk.ssat[sat[i] - 1].vsat[0];
                }
        }
    rtksvrunlock(svr);
    return ns;
}


/* get stream status -----------------------------------------------------------
 * get current stream status
 * args   : rtksvr_t *svr    I  rtk server
 *          int     *sstat   O  status of streams
 *          char    *msg     O  status messages
 * return : none
 *-----------------------------------------------------------------------------*/
void rtksvrsstat(rtksvr_t *svr, int *sstat, char *msg)
{
    int i;
    char s[MAXSTRMSG];
    char *p = msg;

    tracet(4, "rtksvrsstat:\n");

    rtksvrlock(svr);
    for (i = 0; i < MAXSTRRTK; i++)
        {
            sstat[i] = strstat(svr->stream + i, s);
            if (*s)
                {
                    p += std::snprintf(p, MAXSTRMSG, "(%d) %s ", i + 1, s);
                }
        }
    rtksvrunlock(svr);
}
