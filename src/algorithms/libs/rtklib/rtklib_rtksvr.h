/*!
 * \file rtklib_rtksvr.h
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

#ifndef GNSS_SDR_RTKLIB_RKTSVR_H_
#define GNSS_SDR_RTKLIB_RKTSVR_H_

#include "rtklib.h"
#include "rtklib_stream.h"

#define MAXRAWLEN   4096                /* max length of receiver raw message */
#define MAXSOLBUF   256                 /* max number of solution buffer */
#define MAXSBSMSG   32                  /* max number of SBAS msg in RTK server */
#define MAXOBSBUF   128                 /* max number of observation data buffer */

const solopt_t solopt_default = { /* defaults solution output options */
        SOLF_LLH, TIMES_GPST, 1, 3,     /* posf, times, timef, timeu */
        0, 1, 0, 0, 0, 0,                 /* degf, outhead, outopt, datum, height, geoid */
        0, 0, 0,                       /* solstatic, sstat, trace */
        {0.0, 0.0},                  /* nmeaintv */
        " ", "", 0                      /* separator/program name */
};

const prcopt_t prcopt_default = { /* defaults processing options */
        PMODE_SINGLE, 0, 2, SYS_GPS,    /* mode, soltype, nf, navsys */
        15.0*D2R, { {}, {{},{}} },            /* elmin, snrmask */
        0, 1, 1, 1,                     /* sateph, modear, glomodear, bdsmodear */
        5, 0, 10, 1,                    /* maxout, minlock, minfix, armaxiter */
        0, 0, 0, 0,                     /* estion, esttrop, dynamics, tidecorr */
        1, 0, 0, 0, 0,                   /* niter, codesmooth, intpref, sbascorr, sbassatsel */
        0, 0,                         /* rovpos, refpos */
        {100.0, 100.0, 100.0},               /* eratio[] */
        {100.0, 0.003, 0.003, 0.0, 1.0},  /* err[] */
        {30.0, 0.03, 0.3},             /* std[] */
        {1e-4, 1e-3, 1e-4, 1e-1, 1e-2, 0.0},  /* prn[] */
        5E-12,                       /* sclkstab */
        {3.0, 0.9999, 0.25, 0.1, 0.05, 0, 0, 0},  /* thresar */
        0.0, 0.0, 0.05,                /* elmaskar, almaskhold, thresslip */
        30.0, 30.0, 30.0,              /* maxtdif, maxinno, maxgdop */
        {}, {}, {},                 /* baseline, ru, rb */
        {"",""},                    /* anttype */
        {} , {}, {},              /* antdel, pcv, exsats */
        0, 0, 0, {"",""}, {}, 0, {{},{}}, { {}, {{},{}}, {{},{}}, {}, {} }, 0, {}
};

typedef struct {        /* receiver raw data control type */
    gtime_t time;       /* message time */
    gtime_t tobs;       /* observation data time */
    obs_t obs;          /* observation data */
    obs_t obuf;         /* observation data buffer */
    nav_t nav;          /* satellite ephemerides */
    sta_t sta;          /* station parameters */
    int ephsat;         /* sat number of update ephemeris (0:no satellite) */
    sbsmsg_t sbsmsg;    /* SBAS message */
    char msgtype[256];  /* last message type */
    unsigned char subfrm[MAXSAT][380];  /* subframe buffer */
    lexmsg_t lexmsg;    /* LEX message */
    double lockt[MAXSAT][NFREQ+NEXOBS]; /* lock time (s) */
    double icpp[MAXSAT],off[MAXSAT],icpc; /* carrier params for ss2 */
    double prCA[MAXSAT],dpCA[MAXSAT]; /* L1/CA pseudrange/doppler for javad */
    unsigned char halfc[MAXSAT][NFREQ+NEXOBS]; /* half-cycle add flag */
    char freqn[MAXOBS]; /* frequency number for javad */
    int nbyte;          /* number of bytes in message buffer */
    int len;            /* message length (bytes) */
    int iod;            /* issue of data */
    int tod;            /* time of day (ms) */
    int tbase;          /* time base (0:gpst,1:utc(usno),2:glonass,3:utc(su) */
    int flag;           /* general purpose flag */
    int outtype;        /* output message type */
    unsigned char buff[MAXRAWLEN]; /* message buffer */
    char opt[256];      /* receiver dependent options */
    double receive_time;/* RT17: Reiceve time of week for week rollover detection */
    unsigned int plen;  /* RT17: Total size of packet to be read */
    unsigned int pbyte; /* RT17: How many packet bytes have been read so far */
    unsigned int page;  /* RT17: Last page number */
    unsigned int reply; /* RT17: Current reply number */
    int week;           /* RT17: week number */
    unsigned char pbuff[255+4+2]; /* RT17: Packet buffer */
} raw_t;

typedef struct {        /* RTK server type */
    int state;          /* server state (0:stop,1:running) */
    int cycle;          /* processing cycle (ms) */
    int nmeacycle;      /* NMEA request cycle (ms) (0:no req) */
    int nmeareq;        /* NMEA request (0:no,1:nmeapos,2:single sol) */
    double nmeapos[3];  /* NMEA request position (ecef) (m) */
    int buffsize;       /* input buffer size (bytes) */
    int format[3];      /* input format {rov,base,corr} */
    solopt_t solopt[2]; /* output solution options {sol1,sol2} */
    int navsel;         /* ephemeris select (0:all,1:rover,2:base,3:corr) */
    int nsbs;           /* number of sbas message */
    int nsol;           /* number of solution buffer */
    rtk_t rtk;          /* RTK control/result struct */
    int nb [3];         /* bytes in input buffers {rov,base} */
    int nsb[2];         /* bytes in soulution buffers */
    int npb[3];         /* bytes in input peek buffers */
    unsigned char *buff[3]; /* input buffers {rov,base,corr} */
    unsigned char *sbuf[2]; /* output buffers {sol1,sol2} */
    unsigned char *pbuf[3]; /* peek buffers {rov,base,corr} */
    sol_t solbuf[MAXSOLBUF]; /* solution buffer */
    unsigned int nmsg[3][10]; /* input message counts */
    raw_t  raw [3];     /* receiver raw control {rov,base,corr} */
    rtcm_t rtcm[3];     /* RTCM control {rov,base,corr} */
    gtime_t ftime[3];   /* download time {rov,base,corr} */
    char files[3][MAXSTRPATH]; /* download paths {rov,base,corr} */
    obs_t obs[3][MAXOBSBUF]; /* observation data {rov,base,corr} */
    nav_t nav;          /* navigation data */
    sbsmsg_t sbsmsg[MAXSBSMSG]; /* SBAS message buffer */
    stream_t stream[8]; /* streams {rov,base,corr,sol1,sol2,logr,logb,logc} */
    stream_t *moni;     /* monitor stream */
    unsigned int tick;  /* start tick */
    thread_t thread;    /* server thread */
    int cputime;        /* CPU time (ms) for a processing cycle */
    int prcout;         /* missing observation data count */
    lock_t lock;        /* lock flag */
} rtksvr_t;



void writesolhead(stream_t *stream, const solopt_t *solopt);
void saveoutbuf(rtksvr_t *svr, unsigned char *buff, int n, int index);
void writesol(rtksvr_t *svr, int index);
void updatenav(nav_t *nav);
void updatefcn(rtksvr_t *svr);
void updatesvr(rtksvr_t *svr, int ret, obs_t *obs, nav_t *nav, int sat,
                      sbsmsg_t *sbsmsg, int index, int iobs);
int decoderaw(rtksvr_t *svr, int index);
void decodefile(rtksvr_t *svr, int index);
void *rtksvrthread(void *arg);
int rtksvrinit(rtksvr_t *svr);
void rtksvrfree(rtksvr_t *svr);
void rtksvrlock  (rtksvr_t *svr);
void rtksvrunlock(rtksvr_t *svr);
int rtksvrstart(rtksvr_t *svr, int cycle, int buffsize, int *strs,
                       char **paths, int *formats, int navsel, char **cmds,
                       char **rcvopts, int nmeacycle, int nmeareq,
                       const double *nmeapos, prcopt_t *prcopt,
                       solopt_t *solopt, stream_t *moni);
void rtksvrstop(rtksvr_t *svr, char **cmds);
int rtksvropenstr(rtksvr_t *svr, int index, int str, const char *path,
                         const solopt_t *solopt);
void rtksvrclosestr(rtksvr_t *svr, int index);
int rtksvrostat(rtksvr_t *svr, int rcv, gtime_t *time, int *sat,
                       double *az, double *el, int **snr, int *vsat);
void rtksvrsstat(rtksvr_t *svr, int *sstat, char *msg);



#endif
