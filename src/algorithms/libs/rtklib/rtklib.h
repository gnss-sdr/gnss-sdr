/*!
 * \file rtklib.h
 * \brief main header file for the rtklib library
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

#ifndef GNSS_SDR_RTKLIB_H_
#define GNSS_SDR_RTKLIB_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <cstring>
#include <cmath>
#include <time.h>
#include <ctype.h>
#include "MATH_CONSTANTS.h"
#include "gnss_frequencies.h"
#include "gnss_obs_codes.h"


const int FILEPATHSEP = '/';

const double RE_WGS84 = 6378137.0;             //!< earth semimajor axis (WGS84) (m)
const double FE_WGS84 = (1.0 / 298.257223563); //!< earth flattening (WGS84)

const double HION = 350000.0;                  //!<  ionosphere height (m)
const double PRN_HWBIAS = 1e-6;                //!<  process noise of h/w bias (m/MHz/sqrt(s))

const double INT_SWAP_STAT = 86400.0;          //!<  swap interval of solution status file (s)

const unsigned int POLYCRC32 = 0xEDB88320u;    //!<  CRC32 polynomial
const unsigned int POLYCRC24Q = 0x1864CFBu;    //!<  CRC24Q polynomial

const int PMODE_SINGLE = 0;           //!<  positioning mode: single
const int PMODE_DGPS = 1;             //!<  positioning mode: DGPS/DGNSS
const int PMODE_KINEMA = 2;           //!<  positioning mode: kinematic
const int PMODE_STATIC = 3;           //!<  positioning mode: static
const int PMODE_MOVEB = 4;            //!<  positioning mode: moving-base
const int PMODE_FIXED = 5;            //!<  positioning mode: fixed
const int PMODE_PPP_KINEMA = 6;       //!<  positioning mode: PPP-kinemaric
const int PMODE_PPP_STATIC = 7;       //!<  positioning mode: PPP-static
const int PMODE_PPP_FIXED = 8;        //!<  positioning mode: PPP-fixed

const unsigned int SOLF_LLH = 0;               //!<  solution format: lat/lon/height
const unsigned int SOLF_XYZ = 1;               //!<  solution format: x/y/z-ecef
const unsigned int SOLF_ENU = 2;               //!<  solution format: e/n/u-baseline
const unsigned int SOLF_NMEA = 3;              //!<  solution format: NMEA-183
const unsigned int SOLF_STAT = 4;              //!<  solution format: solution status
const unsigned int SOLF_GSIF = 5;              //!<  solution format: GSI F1/F2

const unsigned int SOLQ_NONE = 0;              //!<  solution status: no solution
const unsigned int SOLQ_FIX = 1;               //!<  solution status: fix
const unsigned int SOLQ_FLOAT = 2;             //!<  solution status: float
const unsigned int SOLQ_SBAS = 3;              //!<  solution status: SBAS
const unsigned int SOLQ_DGPS = 4;              //!<  solution status: DGPS/DGNSS
const unsigned int SOLQ_SINGLE = 5;            //!<  solution status: single
const unsigned int SOLQ_PPP = 6;               //!<  solution status: PPP
const unsigned int SOLQ_DR = 7;                //!<  solution status: dead reckoning
const unsigned int MAXSOLQ = 7;                //!<  max number of solution status

const unsigned int TIMES_GPST = 0;             //!<  time system: gps time
const unsigned int TIMES_UTC = 1;              //!<  time system: utc
const unsigned int TIMES_JST = 2;              //!<  time system: jst


const double ERR_SAAS = 0.3;                   //!<  saastamoinen model error std (m)
const double ERR_BRDCI = 0.5;                  //!<  broadcast iono model error factor
const double ERR_CBIAS = 0.3;                  //!<  code bias error std (m)
const double REL_HUMI = 0.7;                   //!<  relative humidity for saastamoinen model
const double GAP_RESION = 120;                 //!<  default gap to reset ionos parameters (ep)

const int MAXFREQ = 7;                //!<  max NFREQ

const int MAXLEAPS = 64;              //!<  max number of leap seconds table
const double DTTOL = 0.005;                    //!<  tolerance of time difference (s)

const int NFREQ = 3;                  //!<   number of carrier frequencies
const int NFREQGLO = 2;               //!<   number of carrier frequencies of GLONASS
const int NEXOBS = 0;                 //!<  number of extended obs codes
const int MAXANT = 64;                //!<  max length of station name/antenna type

const int MINPRNGPS = 1;              //!<   min satellite PRN number of GPS
const int MAXPRNGPS = 32;             //!<   max satellite PRN number of GPS
const int NSATGPS = (MAXPRNGPS - MINPRNGPS + 1); //!<   number of GPS satellites
const int NSYSGPS = 1;

const int SYS_NONE = 0x00;               //!<   navigation system: none
const int SYS_GPS = 0x01;                //!<   navigation system: GPS
const int SYS_SBS = 0x02;                //!<   navigation system: SBAS
const int SYS_GLO = 0x04;                //!<   navigation system: GLONASS
const int SYS_GAL = 0x08;                //!<   navigation system: Galileo
const int SYS_QZS = 0x10;                //!<   navigation system: QZSS
const int SYS_BDS = 0x20;                //!<   navigation system: BeiDou
const int SYS_IRN = 0x40;                //!<   navigation system: IRNS
const int SYS_LEO = 0x80;                //!<   navigation system: LEO
const int SYS_ALL = 0xFF;                //!<   navigation system: all



#ifdef ENAGLO
const int MINPRNGLO = 1;                   //!<   min satellite slot number of GLONASS
const int MAXPRNGLO = 27;                  //!<   max satellite slot number of GLONASS
const int NSATGLO = (MAXPRNGLO - MINPRNGLO + 1); //!<   number of GLONASS satellites
const int NSYSGLO = 1;
#else
const int MINPRNGLO = 0;
const int MAXPRNGLO = 0;
const int NSATGLO = 0;
const int NSYSGLO = 0;
#endif

#ifdef ENAGAL
const int MINPRNGAL = 1;                   //!<   min satellite PRN number of Galileo
const int MAXPRNGAL = 30;                  //!<   max satellite PRN number of Galileo
const int NSATGAL = (MAXPRNGAL - MINPRNGAL + 1); //!<   number of Galileo satellites
const int NSYSGAL = 1;
#else
const int MINPRNGAL = 0;
const int MAXPRNGAL = 0;
const int NSATGAL = 0;
const int NSYSGAL =  0;
#endif

#ifdef ENAQZS
const int MINPRNQZS = 193;                 //!<  min satellite PRN number of QZSS
const int MAXPRNQZS = 199;                 //!<   max satellite PRN number of QZSS
const int MINPRNQZS_S = 183;               //!<   min satellite PRN number of QZSS SAIF
const int MAXPRNQZS_S = 189;               //!<   max satellite PRN number of QZSS SAIF
const int NSATQZS  = (MAXPRNQZS - MINPRNQZS + 1); //!<   number of QZSS satellites
const int NSYSQZS = 1;
#else
const int MINPRNQZS = 0;
const int MAXPRNQZS = 0;
const int MINPRNQZS_S = 0;
const int MAXPRNQZS_S = 0;
const int NSATQZS = 0;
const int NSYSQZS = 0;
#endif

#ifdef ENABDS
const int MINPRNBDS = 1;                   //!<   min satellite sat number of BeiDou
const int MAXPRNBDS = 35;                  //!<   max satellite sat number of BeiDou
const int NSATBDS = (MAXPRNBDS - MINPRNCM + 1); //!<   number of BeiDou satellites
const int NSYSBDS = 1;
#else
const int MINPRNBDS = 0;
const int MAXPRNBDS = 0;
const int NSATBDS = 0;
const int NSYSBDS = 0;
#endif

#ifdef ENAIRN
const int MINPRNIRN = 1;                   //!<   min satellite sat number of IRNSS
const int MAXPRNIRN = 7;                   //!<  max satellite sat number of IRNSS
const int NSATIRN = (MAXPRNIRN - MINPRNIRN + 1); //!<   number of IRNSS satellites
const int NSYSIRN = 1;
#else
const int MINPRNIRN = 0;
const int MAXPRNIRN = 0;
const int NSATIRN = 0;
const int NSYSIRN = 0;
#endif

#ifdef ENALEO
const int MINPRNLEO = 1;                  //!<   min satellite sat number of LEO
const int NSATLEO = 10;                   //!<   max satellite sat number of LEO
const int NSATLEO = (MAXPRNLEO - MINPRNLEO + 1); //!<   number of LEO satellites
const int NSYSLEO = 1;
#else
const int MINPRNLEO = 0;
const int MAXPRNLEO = 0;
const int NSATLEO = 0;
const int NSYSLEO = 0;
#endif

const int NSYS = (NSYSGPS + NSYSGLO + NSYSGAL + NSYSQZS + NSYSBDS + NSYSIRN + NSYSLEO); //!< number of systems

const int MINPRNSBS = 120;                 //!<   min satellite PRN number of SBAS
const int MAXPRNSBS = 142;                 //!<   max satellite PRN number of SBAS
const int NSATSBS = (MAXPRNSBS - MINPRNSBS + 1); //!<   number of SBAS satellites

const int MAXSAT = (NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATBDS + NSATIRN + NSATSBS + NSATLEO);

const int MAXSTA = 255;

#ifndef MAXOBS
const int MAXOBS = 64;          //!<    max number of obs in an epoch
#endif

const int MAXRCV = 64;          //!<    max receiver number (1 to MAXRCV)
const int MAXOBSTYPE = 64;      //!<    max number of obs type in RINEX
const double MAXDTOE = 7200.0;           //!<    max time difference to GPS Toe (s)
const double MAXDTOE_QZS = 7200.0;       //!<    max time difference to QZSS Toe (s)
const double MAXDTOE_GAL = 10800.0;      //!<    max time difference to Galileo Toe (s)
const double MAXDTOE_BDS = 21600.0;      //!<    max time difference to BeiDou Toe (s)
const double MAXDTOE_GLO = 1800.0;       //!<    max time difference to GLONASS Toe (s)
const double MAXDTOE_SBS = 360.0;        //!<    max time difference to SBAS Toe (s)
const double MAXDTOE_S =  86400.0;       //!<    max time difference to ephem toe (s) for other
const double MAXGDOP = 300.0;            //!<    max GDOP

const int MAXSBSURA = 8;        //!<    max URA of SBAS satellite
const int MAXBAND = 10;         //!<    max SBAS band of IGP
const int MAXNIGP = 201;        //!<    max number of IGP in SBAS band
const int MAXNGEO = 4;          //!<    max number of GEO satellites

const int MAXSOLMSG = 8191;     //!<    max length of solution message
const int MAXERRMSG = 4096;     //!<    max length of error/warning message

const int IONOOPT_OFF = 0;      //!<    ionosphere option: correction off
const int IONOOPT_BRDC = 1;     //!<    ionosphere option: broadcast model
const int IONOOPT_SBAS = 2;     //!<    ionosphere option: SBAS model
const int IONOOPT_IFLC = 3;     //!<    ionosphere option: L1/L2 or L1/L5 iono-free LC
const int IONOOPT_EST = 4;      //!<    ionosphere option: estimation
const int IONOOPT_TEC = 5;      //!<    ionosphere option: IONEX TEC model
const int IONOOPT_QZS = 6;      //!<    ionosphere option: QZSS broadcast model
const int IONOOPT_LEX = 7;      //!<    ionosphere option: QZSS LEX ionospehre
const int IONOOPT_STEC = 8;     //!<    ionosphere option: SLANT TEC model

const int TROPOPT_OFF = 0;      //!<    troposphere option: correction off
const int TROPOPT_SAAS = 1;     //!<    troposphere option: Saastamoinen model
const int TROPOPT_SBAS = 2;     //!<    troposphere option: SBAS model
const int TROPOPT_EST = 3;      //!<    troposphere option: ZTD estimation
const int TROPOPT_ESTG = 4;     //!<    troposphere option: ZTD+grad estimation
const int TROPOPT_ZTD = 5;      //!<    troposphere option: ZTD correction

const int EPHOPT_BRDC = 0;      //!<    ephemeris option: broadcast ephemeris
const int EPHOPT_PREC = 1;      //!<    ephemeris option: precise ephemeris
const int EPHOPT_SBAS = 2;      //!<    ephemeris option: broadcast + SBAS
const int EPHOPT_SSRAPC = 3;    //!<    ephemeris option: broadcast + SSR_APC
const int EPHOPT_SSRCOM = 4;    //!<    ephemeris option: broadcast + SSR_COM
const int EPHOPT_LEX = 5;       //!<    ephemeris option: QZSS LEX ephemeris

const double EFACT_GPS = 1.0;            //!<    error factor: GPS
const double EFACT_GLO = 1.5;            //!<    error factor: GLONASS
const double EFACT_GAL = 1.0;            //!<    error factor: Galileo
const double EFACT_QZS = 1.0;            //!<    error factor: QZSS
const double EFACT_BDS = 1.0;            //!<    error factor: BeiDou
const double EFACT_IRN = 1.5;            //!<    error factor: IRNSS
const double EFACT_SBS = 3.0;            //!<    error factor: SBAS

const int MAXEXFILE = 1024;              //!<    max number of expanded files
const double MAXSBSAGEF = 30.0;          //!<    max age of SBAS fast correction (s)
const double MAXSBSAGEL = 1800.0;        //!<    max age of SBAS long term corr (s)

const int ARMODE_OFF = 0;       //!< AR mode: off
const int ARMODE_CONT = 1;      //!< AR mode: continuous
const int ARMODE_INST = 2;      //!< AR mode: instantaneous
const int ARMODE_FIXHOLD = 3;   //!< AR mode: fix and hold
const int ARMODE_WLNL = 4;      //!< AR mode: wide lane/narrow lane
const int ARMODE_TCAR = 5;      //!< AR mode: triple carrier ar

const int POSOPT_RINEX = 3;              //!< pos option: rinex header pos */

typedef void fatalfunc_t(const char *); //!<  fatal callback function type


typedef struct {        /* time struct */
    time_t time;        /* time (s) expressed by standard time_t */
    double sec;         /* fraction of second under 1 s */
} gtime_t;


typedef struct {        /* observation data record */
    gtime_t time;       /* receiver sampling time (GPST) */
    unsigned char sat,rcv; /* satellite/receiver number */
    unsigned char SNR [NFREQ+NEXOBS]; /* signal strength (0.25 dBHz) */
    unsigned char LLI [NFREQ+NEXOBS]; /* loss of lock indicator */
    unsigned char code[NFREQ+NEXOBS]; /* code indicator (CODE_???) */
    double L[NFREQ+NEXOBS]; /* observation data carrier-phase (cycle) */
    double P[NFREQ+NEXOBS]; /* observation data pseudorange (m) */
    float  D[NFREQ+NEXOBS]; /* observation data doppler frequency (Hz) */
} obsd_t;


typedef struct {        /* observation data */
    int n,nmax;         /* number of obervation data/allocated */
    obsd_t *data;       /* observation data records */
} obs_t;


typedef struct {        /* earth rotation parameter data type */
    double mjd;         /* mjd (days) */
    double xp,yp;       /* pole offset (rad) */
    double xpr,ypr;     /* pole offset rate (rad/day) */
    double ut1_utc;     /* ut1-utc (s) */
    double lod;         /* length of day (s/day) */
} erpd_t;


typedef struct {        /* earth rotation parameter type */
    int n,nmax;         /* number and max number of data */
    erpd_t *data;       /* earth rotation parameter data */
} erp_t;


typedef struct {        /* antenna parameter type */
    int sat;            /* satellite number (0:receiver) */
    char type[MAXANT];  /* antenna type */
    char code[MAXANT];  /* serial number or satellite code */
    gtime_t ts,te;      /* valid time start and end */
    double off[NFREQ][ 3]; /* phase center offset e/n/u or x/y/z (m) */
    double var[NFREQ][19]; /* phase center variation (m) */
                        /* el=90,85,...,0 or nadir=0,1,2,3,... (deg) */
} pcv_t;


typedef struct {        /* antenna parameters type */
    int n,nmax;         /* number of data/allocated */
    pcv_t *pcv;         /* antenna parameters data */
} pcvs_t;


typedef struct {        /* almanac type */
    int sat;            /* satellite number */
    int svh;            /* sv health (0:ok) */
    int svconf;         /* as and sv config */
    int week;           /* GPS/QZS: gps week, GAL: galileo week */
    gtime_t toa;        /* Toa */
                        /* SV orbit parameters */
    double A,e,i0,OMG0,omg,M0,OMGd;
    double toas;        /* Toa (s) in week */
    double f0,f1;       /* SV clock parameters (af0,af1) */
} alm_t;


typedef struct {        /* GPS/QZS/GAL broadcast ephemeris type */
    int sat;            /* satellite number */
    int iode,iodc;      /* IODE,IODC */
    int sva;            /* SV accuracy (URA index) */
    int svh;            /* SV health (0:ok) */
    int week;           /* GPS/QZS: gps week, GAL: galileo week */
    int code;           /* GPS/QZS: code on L2, GAL/BDS: data sources */
    int flag;           /* GPS/QZS: L2 P data flag, BDS: nav type */
    gtime_t toe,toc,ttr; /* Toe,Toc,T_trans */
                        /* SV orbit parameters */
    double A,e,i0,OMG0,omg,M0,deln,OMGd,idot;
    double crc,crs,cuc,cus,cic,cis;
    double toes;        /* Toe (s) in week */
    double fit;         /* fit interval (h) */
    double f0,f1,f2;    /* SV clock parameters (af0,af1,af2) */
    double tgd[4];      /* group delay parameters */
                        /* GPS/QZS:tgd[0]=TGD */
                        /* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
                        /* BDS    :tgd[0]=BGD1,tgd[1]=BGD2 */
    double Adot,ndot;   /* Adot,ndot for CNAV */
} eph_t;


typedef struct {        /* GLONASS broadcast ephemeris type */
    int sat;            /* satellite number */
    int iode;           /* IODE (0-6 bit of tb field) */
    int frq;            /* satellite frequency number */
    int svh,sva,age;    /* satellite health, accuracy, age of operation */
    gtime_t toe;        /* epoch of epherides (gpst) */
    gtime_t tof;        /* message frame time (gpst) */
    double pos[3];      /* satellite position (ecef) (m) */
    double vel[3];      /* satellite velocity (ecef) (m/s) */
    double acc[3];      /* satellite acceleration (ecef) (m/s^2) */
    double taun,gamn;   /* SV clock bias (s)/relative freq bias */
    double dtaun;       /* delay between L1 and L2 (s) */
} geph_t;


typedef struct {        /* precise ephemeris type */
    gtime_t time;       /* time (GPST) */
    int index;          /* ephemeris index for multiple files */
    double pos[MAXSAT][4]; /* satellite position/clock (ecef) (m|s) */
    float  std[MAXSAT][4]; /* satellite position/clock std (m|s) */
    double vel[MAXSAT][4]; /* satellite velocity/clk-rate (m/s|s/s) */
    float  vst[MAXSAT][4]; /* satellite velocity/clk-rate std (m/s|s/s) */
    float  cov[MAXSAT][3]; /* satellite position covariance (m^2) */
    float  vco[MAXSAT][3]; /* satellite velocity covariance (m^2) */
} peph_t;


typedef struct {        /* precise clock type */
    gtime_t time;       /* time (GPST) */
    int index;          /* clock index for multiple files */
    double clk[MAXSAT][1]; /* satellite clock (s) */
    float  std[MAXSAT][1]; /* satellite clock std (s) */
} pclk_t;


typedef struct {        /* SBAS ephemeris type */
    int sat;            /* satellite number */
    gtime_t t0;         /* reference epoch time (GPST) */
    gtime_t tof;        /* time of message frame (GPST) */
    int sva;            /* SV accuracy (URA index) */
    int svh;            /* SV health (0:ok) */
    double pos[3];      /* satellite position (m) (ecef) */
    double vel[3];      /* satellite velocity (m/s) (ecef) */
    double acc[3];      /* satellite acceleration (m/s^2) (ecef) */
    double af0,af1;     /* satellite clock-offset/drift (s,s/s) */
} seph_t;


typedef struct {        /* norad two line element data type */
    char name [32];     /* common name */
    char alias[32];     /* alias name */
    char satno[16];     /* satellite catalog number */
    char satclass;      /* classification */
    char desig[16];     /* international designator */
    gtime_t epoch;      /* element set epoch (UTC) */
    double ndot;        /* 1st derivative of mean motion */
    double nddot;       /* 2st derivative of mean motion */
    double bstar;       /* B* drag term */
    int etype;          /* element set type */
    int eleno;          /* element number */
    double inc;         /* orbit inclination (deg) */
    double OMG;         /* right ascension of ascending node (deg) */
    double ecc;         /* eccentricity */
    double omg;         /* argument of perigee (deg) */
    double M;           /* mean anomaly (deg) */
    double n;           /* mean motion (rev/day) */
    int rev;            /* revolution number at epoch */
} tled_t;


typedef struct {        /* norad two line element type */
    int n,nmax;         /* number/max number of two line element data */
    tled_t *data;       /* norad two line element data */
} tle_t;


typedef struct {        /* TEC grid type */
    gtime_t time;       /* epoch time (GPST) */
    int ndata[3];       /* TEC grid data size {nlat,nlon,nhgt} */
    double rb;          /* earth radius (km) */
    double lats[3];     /* latitude start/interval (deg) */
    double lons[3];     /* longitude start/interval (deg) */
    double hgts[3];     /* heights start/interval (km) */
    double *data;       /* TEC grid data (tecu) */
    float *rms;         /* RMS values (tecu) */
} tec_t;


typedef struct {        /* satellite fcb data type */
    gtime_t ts,te;      /* start/end time (GPST) */
    double bias[MAXSAT][3]; /* fcb value   (cyc) */
    double std [MAXSAT][3]; /* fcb std-dev (cyc) */
} fcbd_t;


typedef struct {        /* SBAS message type */
    int week,tow;       /* receiption time */
    int prn;            /* SBAS satellite PRN number */
    unsigned char msg[29]; /* SBAS message (226bit) padded by 0 */
} sbsmsg_t;


typedef struct {        /* SBAS messages type */
    int n,nmax;         /* number of SBAS messages/allocated */
    sbsmsg_t *msgs;     /* SBAS messages */
} sbs_t;


typedef struct {        /* SBAS fast correction type */
    gtime_t t0;         /* time of applicability (TOF) */
    double prc;         /* pseudorange correction (PRC) (m) */
    double rrc;         /* range-rate correction (RRC) (m/s) */
    double dt;          /* range-rate correction delta-time (s) */
    int iodf;           /* IODF (issue of date fast corr) */
    short udre;         /* UDRE+1 */
    short ai;           /* degradation factor indicator */
} sbsfcorr_t;


typedef struct {        /* SBAS long term satellite error correction type */
    gtime_t t0;         /* correction time */
    int iode;           /* IODE (issue of date ephemeris) */
    double dpos[3];     /* delta position (m) (ecef) */
    double dvel[3];     /* delta velocity (m/s) (ecef) */
    double daf0,daf1;   /* delta clock-offset/drift (s,s/s) */
} sbslcorr_t;


typedef struct {        /* SBAS satellite correction type */
    int sat;            /* satellite number */
    sbsfcorr_t fcorr;   /* fast correction */
    sbslcorr_t lcorr;   /* long term correction */
} sbssatp_t;


typedef struct {        /* SBAS satellite corrections type */
    int iodp;           /* IODP (issue of date mask) */
    int nsat;           /* number of satellites */
    int tlat;           /* system latency (s) */
    sbssatp_t sat[MAXSAT]; /* satellite correction */
} sbssat_t;


typedef struct {        /* SBAS ionospheric correction type */
    gtime_t t0;         /* correction time */
    short lat,lon;      /* latitude/longitude (deg) */
    short give;         /* GIVI+1 */
    float delay;        /* vertical delay estimate (m) */
} sbsigp_t;


typedef struct {        /* IGP band type */
    short x;            /* longitude/latitude (deg) */
    const short *y;     /* latitudes/longitudes (deg) */
    unsigned char bits; /* IGP mask start bit */
    unsigned char bite; /* IGP mask end bit */
} sbsigpband_t;


typedef struct {        /* SBAS ionospheric corrections type */
    int iodi;           /* IODI (issue of date ionos corr) */
    int nigp;           /* number of igps */
    sbsigp_t igp[MAXNIGP]; /* ionospheric correction */
} sbsion_t;


typedef struct {        /* DGPS/GNSS correction type */
    gtime_t t0;         /* correction time */
    double prc;         /* pseudorange correction (PRC) (m) */
    double rrc;         /* range rate correction (RRC) (m/s) */
    int iod;            /* issue of data (IOD) */
    double udre;        /* UDRE */
} dgps_t;


typedef struct {        /* SSR correction type */
    gtime_t t0[6];      /* epoch time (GPST) {eph,clk,hrclk,ura,bias,pbias} */
    double udi[6];      /* SSR update interval (s) */
    int iod[6];         /* iod ssr {eph,clk,hrclk,ura,bias,pbias} */
    int iode;           /* issue of data */
    int iodcrc;         /* issue of data crc for beidou/sbas */
    int ura;            /* URA indicator */
    int refd;           /* sat ref datum (0:ITRF,1:regional) */
    double deph [3];    /* delta orbit {radial,along,cross} (m) */
    double ddeph[3];    /* dot delta orbit {radial,along,cross} (m/s) */
    double dclk [3];    /* delta clock {c0,c1,c2} (m,m/s,m/s^2) */
    double hrclk;       /* high-rate clock corection (m) */
    float  cbias[MAXCODE]; /* code biases (m) */
    double pbias[MAXCODE]; /* phase biases (m) */
    float  stdpb[MAXCODE]; /* std-dev of phase biases (m) */
    double yaw_ang,yaw_rate; /* yaw angle and yaw rate (deg,deg/s) */
    unsigned char update; /* update flag (0:no update,1:update) */
} ssr_t;


typedef struct {        /* QZSS LEX message type */
    int prn;            /* satellite PRN number */
    int type;           /* message type */
    int alert;          /* alert flag */
    unsigned char stat; /* signal tracking status */
    unsigned char snr;  /* signal C/N0 (0.25 dBHz) */
    unsigned int ttt;   /* tracking time (ms) */
    unsigned char msg[212]; /* LEX message data part 1695 bits */
} lexmsg_t;


typedef struct {        /* QZSS LEX messages type */
    int n,nmax;         /* number of LEX messages and allocated */
    lexmsg_t *msgs;     /* LEX messages */
} lex_t;


typedef struct {        /* QZSS LEX ephemeris type */
    gtime_t toe;        /* epoch time (GPST) */
    gtime_t tof;        /* message frame time (GPST) */
    int sat;            /* satellite number */
    unsigned char health; /* signal health (L1,L2,L1C,L5,LEX) */
    unsigned char ura;  /* URA index */
    double pos[3];      /* satellite position (m) */
    double vel[3];      /* satellite velocity (m/s) */
    double acc[3];      /* satellite acceleration (m/s2) */
    double jerk[3];     /* satellite jerk (m/s3) */
    double af0,af1;     /* satellite clock bias and drift (s,s/s) */
    double tgd;         /* TGD */
    double isc[8];      /* ISC */
} lexeph_t;


typedef struct {        /* QZSS LEX ionosphere correction type */
    gtime_t t0;         /* epoch time (GPST) */
    double tspan;       /* valid time span (s) */
    double pos0[2];     /* reference position {lat,lon} (rad) */
    double coef[3][2];  /* coefficients lat x lon (3 x 2) */
} lexion_t;


typedef struct {        /* stec data type */
    gtime_t time;       /* time (GPST) */
    unsigned char sat;  /* satellite number */
    double ion;         /* slant ionos delay (m) */
    float std;          /* std-dev (m) */
    float azel[2];      /* azimuth/elevation (rad) */
    unsigned char flag; /* fix flag */
} stec_t;


typedef struct {        /* trop data type */
    gtime_t time;       /* time (GPST) */
    double trp[3];      /* zenith tropos delay/gradient (m) */
    float std[3];       /* std-dev (m) */
} trop_t;


typedef struct {        /* ppp corrections type */
    int nsta;           /* number of stations */
    char stas[MAXSTA][8]; /* station names */
    double rr[MAXSTA][3]; /* station ecef positions (m) */
    int ns[MAXSTA],nsmax[MAXSTA]; /* number of stec data */
    int nt[MAXSTA],ntmax[MAXSTA]; /* number of trop data */
    stec_t *stec[MAXSTA]; /* stec data */
    trop_t *trop[MAXSTA]; /* trop data */
} pppcorr_t;


typedef struct {        /* navigation data type */
    int n,nmax;         /* number of broadcast ephemeris */
    int ng,ngmax;       /* number of glonass ephemeris */
    int ns,nsmax;       /* number of sbas ephemeris */
    int ne,nemax;       /* number of precise ephemeris */
    int nc,ncmax;       /* number of precise clock */
    int na,namax;       /* number of almanac data */
    int nt,ntmax;       /* number of tec grid data */
    int nf,nfmax;       /* number of satellite fcb data */
    eph_t *eph;         /* GPS/QZS/GAL ephemeris */
    geph_t *geph;       /* GLONASS ephemeris */
    seph_t *seph;       /* SBAS ephemeris */
    peph_t *peph;       /* precise ephemeris */
    pclk_t *pclk;       /* precise clock */
    alm_t *alm;         /* almanac data */
    tec_t *tec;         /* tec grid data */
    fcbd_t *fcb;        /* satellite fcb data */
    erp_t  erp;         /* earth rotation parameters */
    double utc_gps[4];  /* GPS delta-UTC parameters {A0,A1,T,W} */
    double utc_glo[4];  /* GLONASS UTC GPS time parameters */
    double utc_gal[4];  /* Galileo UTC GPS time parameters */
    double utc_qzs[4];  /* QZS UTC GPS time parameters */
    double utc_cmp[4];  /* BeiDou UTC parameters */
    double utc_irn[4];  /* IRNSS UTC parameters */
    double utc_sbs[4];  /* SBAS UTC parameters */
    double ion_gps[8];  /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    double ion_gal[4];  /* Galileo iono model parameters {ai0,ai1,ai2,0} */
    double ion_qzs[8];  /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    double ion_cmp[8];  /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    double ion_irn[8];  /* IRNSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    int leaps;          /* leap seconds (s) */
    double lam[MAXSAT][NFREQ]; /* carrier wave lengths (m) */
    double cbias[MAXSAT][3]; /* satellite dcb (0:p1-p2,1:p1-c1,2:p2-c2) (m) */
    double rbias[MAXRCV][2][3]; /* receiver dcb (0:p1-p2,1:p1-c1,2:p2-c2) (m) */
    double wlbias[MAXSAT];   /* wide-lane bias (cycle) */
    double glo_cpbias[4];    /* glonass code-phase bias {1C,1P,2C,2P} (m) */
    char glo_fcn[MAXPRNGLO+1]; /* glonass frequency channel number + 8 */
    pcv_t pcvs[MAXSAT]; /* satellite antenna pcv */
    sbssat_t sbssat;    /* SBAS satellite corrections */
    sbsion_t sbsion[MAXBAND+1]; /* SBAS ionosphere corrections */
    dgps_t dgps[MAXSAT]; /* DGPS corrections */
    ssr_t ssr[MAXSAT];  /* SSR corrections */
    lexeph_t lexeph[MAXSAT]; /* LEX ephemeris */
    lexion_t lexion;    /* LEX ionosphere correction */
    pppcorr_t pppcorr;  /* ppp corrections */
} nav_t;


typedef struct {        /* station parameter type */
    char name   [MAXANT]; /* marker name */
    char marker [MAXANT]; /* marker number */
    char antdes [MAXANT]; /* antenna descriptor */
    char antsno [MAXANT]; /* antenna serial number */
    char rectype[MAXANT]; /* receiver type descriptor */
    char recver [MAXANT]; /* receiver firmware version */
    char recsno [MAXANT]; /* receiver serial number */
    int antsetup;       /* antenna setup id */
    int itrf;           /* ITRF realization year */
    int deltype;        /* antenna delta type (0:enu,1:xyz) */
    double pos[3];      /* station position (ecef) (m) */
    double del[3];      /* antenna position delta (e/n/u or x/y/z) (m) */
    double hgt;         /* antenna height (m) */
} sta_t;


typedef struct {        /* solution type */
    gtime_t time;       /* time (GPST) */
    double rr[6];       /* position/velocity (m|m/s) */
                        /* {x,y,z,vx,vy,vz} or {e,n,u,ve,vn,vu} */
    float  qr[6];       /* position variance/covariance (m^2) */
                        /* {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} or */
                        /* {c_ee,c_nn,c_uu,c_en,c_nu,c_ue} */
    double dtr[6];      /* receiver clock bias to time systems (s) */
    unsigned char type; /* type (0:xyz-ecef,1:enu-baseline) */
    unsigned char stat; /* solution status (SOLQ_???) */
    unsigned char ns;   /* number of valid satellites */
    float age;          /* age of differential (s) */
    float ratio;        /* AR ratio factor for validation */
    float thres;        /* AR ratio threshold for validation */
} sol_t;


typedef struct {        /* solution buffer type */
    int n,nmax;         /* number of solution/max number of buffer */
    int cyclic;         /* cyclic buffer flag */
    int start,end;      /* start/end index */
    gtime_t time;       /* current solution time */
    sol_t *data;        /* solution data */
    double rb[3];       /* reference position {x,y,z} (ecef) (m) */
    unsigned char buff[MAXSOLMSG+1]; /* message buffer */
    int nb;             /* number of byte in message buffer */
} solbuf_t;


typedef struct {        /* solution status type */
    gtime_t time;       /* time (GPST) */
    unsigned char sat;  /* satellite number */
    unsigned char frq;  /* frequency (1:L1,2:L2,...) */
    float az,el;        /* azimuth/elevation angle (rad) */
    float resp;         /* pseudorange residual (m) */
    float resc;         /* carrier-phase residual (m) */
    unsigned char flag; /* flags: (vsat<<5)+(slip<<3)+fix */
    unsigned char snr;  /* signal strength (0.25 dBHz) */
    unsigned short lock;  /* lock counter */
    unsigned short outc;  /* outage counter */
    unsigned short slipc; /* slip counter */
    unsigned short rejc;  /* reject counter */
} solstat_t;


typedef struct {        /* solution status buffer type */
    int n,nmax;         /* number of solution/max number of buffer */
    solstat_t *data;    /* solution status data */
} solstatbuf_t;


typedef struct {        /* RTCM control struct type */
    int staid;          /* station id */
    int stah;           /* station health */
    int seqno;          /* sequence number for rtcm 2 or iods msm */
    int outtype;        /* output message type */
    gtime_t time;       /* message time */
    gtime_t time_s;     /* message start time */
    obs_t obs;          /* observation data (uncorrected) */
    nav_t nav;          /* satellite ephemerides */
    sta_t sta;          /* station parameters */
    dgps_t *dgps;       /* output of dgps corrections */
    ssr_t ssr[MAXSAT];  /* output of ssr corrections */
    char msg[128];      /* special message */
    char msgtype[256];  /* last message type */
    char msmtype[6][128]; /* msm signal types */
    int obsflag;        /* obs data complete flag (1:ok,0:not complete) */
    int ephsat;         /* update satellite of ephemeris */
    double cp[MAXSAT][NFREQ+NEXOBS]; /* carrier-phase measurement */
    unsigned short lock[MAXSAT][NFREQ+NEXOBS]; /* lock time */
    unsigned short loss[MAXSAT][NFREQ+NEXOBS]; /* loss of lock count */
    gtime_t lltime[MAXSAT][NFREQ+NEXOBS]; /* last lock time */
    int nbyte;          /* number of bytes in message buffer */
    int nbit;           /* number of bits in word buffer */
    int len;            /* message length (bytes) */
    unsigned char buff[1200]; /* message buffer */
    unsigned int word;  /* word buffer for rtcm 2 */
    unsigned int nmsg2[100]; /* message count of RTCM 2 (1-99:1-99,0:other) */
    unsigned int nmsg3[400]; /* message count of RTCM 3 (1-299:1001-1299,300-399:2000-2099,0:ohter) */
    char opt[256];      /* RTCM dependent options */
} rtcm_t;


typedef struct {        /* download url type */
    char type[32];      /* data type */
    char path[1024];    /* url path */
    char dir [1024];    /* local directory */
    double tint;        /* time interval (s) */
} url_t;


typedef struct {        /* option type */
    const char *name;   /* option name */
    int format;         /* option format (0:int,1:double,2:string,3:enum) */
    void *var;          /* pointer to option variable */
    const char *comment; /* option comment/enum labels/unit */
} opt_t;


typedef struct {        /* extended receiver error model */
    int ena[4];         /* model enabled */
    double cerr[4][NFREQ*2]; /* code errors (m) */
    double perr[4][NFREQ*2]; /* carrier-phase errors (m) */
    double gpsglob[NFREQ]; /* gps-glonass h/w bias (m) */
    double gloicb [NFREQ]; /* glonass interchannel bias (m/fn) */
} exterr_t;


typedef struct {        /* SNR mask type */
    int ena[2];         /* enable flag {rover,base} */
    double mask[NFREQ][9]; /* mask (dBHz) at 5,10,...85 deg */
} snrmask_t;


typedef struct {        /* processing options type */
    int mode;           /* positioning mode (PMODE_???) */
    int soltype;        /* solution type (0:forward,1:backward,2:combined) */
    int nf;             /* number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
    int navsys;         /* navigation system */
    double elmin;       /* elevation mask angle (rad) */
    snrmask_t snrmask;  /* SNR mask */
    int sateph;         /* satellite ephemeris/clock (EPHOPT_???) */
    int modear;         /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
    int glomodear;      /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
    int bdsmodear;      /* BeiDou AR mode (0:off,1:on) */
    int maxout;         /* obs outage count to reset bias */
    int minlock;        /* min lock count to fix ambiguity */
    int minfix;         /* min fix count to hold ambiguity */
    int armaxiter;      /* max iteration to resolve ambiguity */
    int ionoopt;        /* ionosphere option (IONOOPT_???) */
    int tropopt;        /* troposphere option (TROPOPT_???) */
    int dynamics;       /* dynamics model (0:none,1:velociy,2:accel) */
    int tidecorr;       /* earth tide correction (0:off,1:solid,2:solid+otl+pole) */
    int niter;          /* number of filter iteration */
    int codesmooth;     /* code smoothing window size (0:none) */
    int intpref;        /* interpolate reference obs (for post mission) */
    int sbascorr;       /* SBAS correction options */
    int sbassatsel;     /* SBAS satellite selection (0:all) */
    int rovpos;         /* rover position for fixed mode */
    int refpos;         /* base position for relative mode */
                        /* (0:pos in prcopt,  1:average of single pos, */
                        /*  2:read from file, 3:rinex header, 4:rtcm pos) */
    double eratio[NFREQ]; /* code/phase error ratio */
    double err[5];      /* measurement error factor */
                        /* [0]:reserved */
                        /* [1-3]:error factor a/b/c of phase (m) */
                        /* [4]:doppler frequency (hz) */
    double std[3];      /* initial-state std [0]bias,[1]iono [2]trop */
    double prn[6];      /* process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos */
    double sclkstab;    /* satellite clock stability (sec/sec) */
    double thresar[8];  /* AR validation threshold */
    double elmaskar;    /* elevation mask of AR for rising satellite (deg) */
    double elmaskhold;  /* elevation mask to hold ambiguity (deg) */
    double thresslip;   /* slip threshold of geometry-free phase (m) */
    double maxtdiff;    /* max difference of time (sec) */
    double maxinno;     /* reject threshold of innovation (m) */
    double maxgdop;     /* reject threshold of gdop */
    double baseline[2]; /* baseline length constraint {const,sigma} (m) */
    double ru[3];       /* rover position for fixed mode {x,y,z} (ecef) (m) */
    double rb[3];       /* base position for relative mode {x,y,z} (ecef) (m) */
    char anttype[2][MAXANT]; /* antenna types {rover,base} */
    double antdel[2][3]; /* antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
    pcv_t pcvr[2];      /* receiver antenna parameters {rov,base} */
    unsigned char exsats[MAXSAT]; /* excluded satellites (1:excluded,2:included) */
    int  maxaveep;      /* max averaging epoches */
    int  initrst;       /* initialize by restart */
    int  outsingle;     /* output single by dgps/float/fix/ppp outage */
    char rnxopt[2][256]; /* rinex options {rover,base} */
    int  posopt[6];     /* positioning options */
    int  syncsol;       /* solution sync mode (0:off,1:on) */
    double odisp[2][6*11]; /* ocean tide loading parameters {rov,base} */
    exterr_t exterr;    /* extended receiver error model */
    int freqopt;        /* disable L2-AR */
    char pppopt[256];   /* ppp option */
} prcopt_t;


typedef struct {        /* solution options type */
    int posf;           /* solution format (SOLF_???) */
    int times;          /* time system (TIMES_???) */
    int timef;          /* time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s) */
    int timeu;          /* time digits under decimal point */
    int degf;           /* latitude/longitude format (0:ddd.ddd,1:ddd mm ss) */
    int outhead;        /* output header (0:no,1:yes) */
    int outopt;         /* output processing options (0:no,1:yes) */
    int datum;          /* datum (0:WGS84,1:Tokyo) */
    int height;         /* height (0:ellipsoidal,1:geodetic) */
    int geoid;          /* geoid model (0:EGM96,1:JGD2000) */
    int solstatic;      /* solution of static mode (0:all,1:single) */
    int sstat;          /* solution statistics level (0:off,1:states,2:residuals) */
    int trace;          /* debug trace level (0:off,1-5:debug) */
    double nmeaintv[2]; /* nmea output interval (s) (<0:no,0:all) */
                        /* nmeaintv[0]:gprmc,gpgga,nmeaintv[1]:gpgsv */
    char sep[64];       /* field separator */
    char prog[64];      /* program name */
    double maxsolstd;   /* max std-dev for solution output (m) (0:all) */
} solopt_t;


typedef struct {        /* satellite status type */
    unsigned char sys;  /* navigation system */
    unsigned char vs;   /* valid satellite flag single */
    double azel[2];     /* azimuth/elevation angles {az,el} (rad) */
    double resp[NFREQ]; /* residuals of pseudorange (m) */
    double resc[NFREQ]; /* residuals of carrier-phase (m) */
    unsigned char vsat[NFREQ]; /* valid satellite flag */
    unsigned char snr [NFREQ]; /* signal strength (0.25 dBHz) */
    unsigned char fix [NFREQ]; /* ambiguity fix flag (1:fix,2:float,3:hold) */
    unsigned char slip[NFREQ]; /* cycle-slip flag */
    unsigned char half[NFREQ]; /* half-cycle valid flag */
    int lock [NFREQ];   /* lock counter of phase */
    unsigned int outc [NFREQ]; /* obs outage counter of phase */
    unsigned int slipc[NFREQ]; /* cycle-slip counter */
    unsigned int rejc [NFREQ]; /* reject counter */
    double  gf;         /* geometry-free phase L1-L2 (m) */
    double  gf2;        /* geometry-free phase L1-L5 (m) */
    double  mw;         /* MW-LC (m) */
    double  phw;        /* phase windup (cycle) */
    gtime_t pt[2][NFREQ]; /* previous carrier-phase time */
    double  ph[2][NFREQ]; /* previous carrier-phase observable (cycle) */
} ssat_t;


typedef struct {        /* ambiguity control type */
    gtime_t epoch[4];   /* last epoch */
    int n[4];           /* number of epochs */
    double LC [4];      /* linear combination average */
    double LCv[4];      /* linear combination variance */
    int fixcnt;         /* fix count */
    char flags[MAXSAT]; /* fix flags */
} ambc_t;


typedef struct {        /* RTK control/result type */
    sol_t  sol;         /* RTK solution */
    double rb[6];       /* base position/velocity (ecef) (m|m/s) */
    int nx,na;          /* number of float states/fixed states */
    double tt;          /* time difference between current and previous (s) */
    double *x, *P;      /* float states and their covariance */
    double *xa,*Pa;     /* fixed states and their covariance */
    int nfix;           /* number of continuous fixes of ambiguity */
    ambc_t ambc[MAXSAT]; /* ambibuity control */
    ssat_t ssat[MAXSAT]; /* satellite status */
    int neb;            /* bytes in error message buffer */
    char errbuf[MAXERRMSG]; /* error message buffer */
    prcopt_t opt;       /* processing options */
} rtk_t;


typedef struct half_cyc_tag {  /* half-cycle correction list type */
    unsigned char sat;  /* satellite number */
    unsigned char freq; /* frequency number (0:L1,1:L2,2:L5) */
    unsigned char valid; /* half-cycle valid flag */
    char corr;          /* half-cycle corrected (x 0.5 cyc) */
    gtime_t ts,te;      /* time start, time end */
    struct half_cyc_tag *next; /* pointer to next correction */
} half_cyc_t;


const double chisqr[100] = {      /* chi-sqr(n) (alpha=0.001) */
    10.8, 13.8, 16.3, 18.5, 20.5, 22.5, 24.3, 26.1, 27.9, 29.6,
    31.3, 32.9, 34.5, 36.1, 37.7, 39.3, 40.8, 42.3, 43.8, 45.3,
    46.8, 48.3, 49.7, 51.2, 52.6, 54.1, 55.5, 56.9, 58.3, 59.7,
    61.1, 62.5, 63.9, 65.2, 66.6, 68.0, 69.3, 70.7, 72.1, 73.4,
    74.7, 76.0, 77.3, 78.6, 80.0, 81.3, 82.6, 84.0, 85.4, 86.7,
    88.0, 89.3, 90.6, 91.9, 93.3, 94.7, 96.0, 97.4, 98.7, 100 ,
    101 , 102 , 103 , 104 , 105 , 107 , 108 , 109 , 110 , 112 ,
    113 , 114 , 115 , 116 , 118 , 119 , 120 , 122 , 123 , 125 ,
    126 , 127 , 128 , 129 , 131 , 132 , 133 , 134 , 135 , 137 ,
    138 , 139 , 140 , 142 , 143 , 144 , 145 , 147 , 148 , 149
};


const double lam_carr[MAXFREQ] = { /* carrier wave length (m) */
    SPEED_OF_LIGHT / FREQ1, SPEED_OF_LIGHT / FREQ2, SPEED_OF_LIGHT / FREQ5, SPEED_OF_LIGHT / FREQ6, SPEED_OF_LIGHT / FREQ7,
    SPEED_OF_LIGHT / FREQ8, SPEED_OF_LIGHT / FREQ9
};

#endif
