/*!
 * \file sbas_telemetry_data.h
 * \brief Interface of the SBAS telemetry parser based on SBAS RTKLIB functions
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_SBAS_TELEMETRY_DATA_H_
#define GNSS_SDR_SBAS_TELEMETRY_DATA_H_

#include <algorithm>
#include <bitset>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "boost/assign.hpp"
#include "concurrent_queue.h"
#include "sbas_time.h"


class Sbas_Ionosphere_Correction;
class Sbas_Satellite_Correction;
struct Fast_Correction;
struct Long_Term_Correction;
class Sbas_Ephemeris;


/*!
 * \brief Represents a raw SBAS message of 250cbits + 6 bits padding
 *  (8b preamble + 6b message type + 212b data + 24b CRC + 6b zero padding)
 */
class Sbas_Raw_Msg
{
public:
    Sbas_Raw_Msg(){ rx_time = Sbas_Time(0); i_prn = -1; };
    Sbas_Raw_Msg(double sample_stamp, int prn, const std::vector<unsigned char> msg) : rx_time(sample_stamp), i_prn(prn), d_msg(msg) {}
    double get_sample_stamp() { return rx_time.get_time_stamp(); } //!< Time of reception sample stamp (first sample of preample)
    void relate(Sbas_Time_Relation time_relation)
    {
        rx_time.relate(time_relation);
    }
    Sbas_Time get_rx_time_obj() const { return rx_time; }
    int get_prn() const { return i_prn; }
    std::vector<unsigned char> get_msg() const { return d_msg; }
    int get_preamble()
    {
        return d_msg[0];
    }
    int get_msg_type() const
    {
        return d_msg[1] >> 2;
    }
    int get_crc()
    {
        unsigned char crc_last_byte = (d_msg[30] << 2) && (d_msg[31] >> 6);
        unsigned char crc_middle_byte = (d_msg[29] << 2) && (d_msg[30] >> 6);
        unsigned char crc_first_byte = (d_msg[28] << 2) && (d_msg[29] >> 6);
        return ((unsigned int)(crc_first_byte) << 16) && ((unsigned int)(crc_middle_byte) << 8) && crc_last_byte;
    }
private:
    Sbas_Time rx_time;
    int i_prn;                        /* SBAS satellite PRN number */
    std::vector<unsigned char> d_msg; /* SBAS message (226 bit) padded by 0 */
};




/*
 * \brief Holds an updated set of the telemetry data received from SBAS
 */
class Sbas_Telemetry_Data
{
public:
    int update(Sbas_Raw_Msg sbas_raw_msg);

    /*!
     * Default constructor
     */
    Sbas_Telemetry_Data();
    /*!
     * Default deconstructor
     */
    ~Sbas_Telemetry_Data();

private:
    std::map<int, Fast_Correction> emitted_fast_corrections;
    std::map<int, Long_Term_Correction> emitted_long_term_corrections;

    Sbas_Time_Relation mt12_time_ref;

    int decode_mt12(Sbas_Raw_Msg sbas_raw_msg);

    void updated_sbas_ephemeris(Sbas_Raw_Msg msg);
    void received_iono_correction();
    void updated_satellite_corrections();

    /////// rtklib.h

#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_SBS     0x02                /* navigation system: SBAS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_GAL     0x08                /* navigation system: Galileo */
#define SYS_QZS     0x10                /* navigation system: QZSS */
#define SYS_CMP     0x20                /* navigation system: BeiDou */
#define SYS_ALL     0xFF                /* navigation system: all */

#define TSYS_GPS    0                   /* time system: GPS time */
#define TSYS_UTC    1                   /* time system: UTC */
#define TSYS_GLO    2                   /* time system: GLONASS time */
#define TSYS_GAL    3                   /* time system: Galileo time */
#define TSYS_QZS    4                   /* time system: QZSS time */
#define TSYS_CMP    5                   /* time system: BeiDou time */

#ifndef NFREQ
#define NFREQ       3                   /* number of carrier frequencies */
#endif
#define NFREQGLO    2                   /* number of carrier frequencies of GLONASS */

#ifndef NEXOBS
#define NEXOBS      0                   /* number of extended obs codes */
#endif

#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1

#ifdef ENAGLO
#define MINPRNGLO   1                   /* min satellite slot number of GLONASS */
#define MAXPRNGLO   24                  /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define NSYSGLO     1
#else
#define MINPRNGLO   0
#define MAXPRNGLO   0
#define NSATGLO     0
#define NSYSGLO     0
#endif
#ifdef ENAGAL
#define MINPRNGAL   1                   /* min satellite PRN number of Galileo */
#define MAXPRNGAL   27                  /* max satellite PRN number of Galileo */
#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1) /* number of Galileo satellites */
#define NSYSGAL     1
#else
#define MINPRNGAL   0
#define MAXPRNGAL   0
#define NSATGAL     0
#define NSYSGAL     0
#endif
#ifdef ENAQZS
#define MINPRNQZS   193                 /* min satellite PRN number of QZSS */
#define MAXPRNQZS   195                 /* max satellite PRN number of QZSS */
#define MINPRNQZS_S 183                 /* min satellite PRN number of QZSS SAIF */
#define MAXPRNQZS_S 185                 /* max satellite PRN number of QZSS SAIF */
#define NSATQZS     (MAXPRNQZS-MINPRNQZS+1) /* number of QZSS satellites */
#define NSYSQZS     1
#else
#define MINPRNQZS   0
#define MAXPRNQZS   0
#define NSATQZS     0
#define NSYSQZS     0
#endif
#ifdef ENACMP
#define MINPRNCMP   1                   /* min satellite sat number of BeiDou */
#define MAXPRNCMP   35                  /* max satellite sat number of BeiDou */
#define NSATCMP     (MAXPRNCMP-MINPRNCMP+1) /* number of BeiDou satellites */
#define NSYSCMP     1
#else
#define MINPRNCMP   0
#define MAXPRNCMP   0
#define NSATCMP     0
#define NSYSCMP     0
#endif
#define NSYS        (NSYSGPS+NSYSGLO+NSYSGAL+NSYSQZS+NSYSCMP) /* number of systems */

#define MINPRNSBS   120                 /* min satellite PRN number of SBAS */
#define MAXPRNSBS   142                 /* max satellite PRN number of SBAS */
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) /* number of SBAS satellites */

#define MAXSAT      (NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATSBS)
    /* max satellite number (1 to MAXSAT) */
#ifndef MAXOBS
#define MAXOBS      64                  /* max number of obs in an epoch */
#endif
#define MAXRCV      64                  /* max receiver number (1 to MAXRCV) */
#define MAXOBSTYPE  64                  /* max number of obs type in RINEX */
#define DTTOL       0.005               /* tolerance of time difference (s) */
#if 0
#define MAXDTOE     10800.0             /* max time difference to ephem Toe (s) for GPS */
#else
#define MAXDTOE     7200.0              /* max time difference to ephem Toe (s) for GPS */
#endif
#define MAXDTOE_GLO 1800.0              /* max time difference to GLONASS Toe (s) */
#define MAXDTOE_SBS 360.0               /* max time difference to SBAS Toe (s) */
#define MAXDTOE_S   86400.0             /* max time difference to ephem toe (s) for other */
#define MAXGDOP     300.0               /* max GDOP */

    //#define MAXSBSAGEF  30.0                /* max age of SBAS fast correction (s) */
    //#define MAXSBSAGEL  1800.0              /* max age of SBAS long term corr (s) */
    //#define MAXSBSURA   8                   /* max URA of SBAS satellite */
#define MAXBAND     10                  /* max SBAS band of IGP */
#define MAXNIGP     201                 /* max number of IGP in SBAS band */
    //#define MAXNGEO     4                   /* max number of GEO satellites */


#define P2_11       4.882812500000000E-04 /* 2^-11 */
#define P2_31       4.656612873077393E-10 /* 2^-31 */
#define P2_39       1.818989403545856E-12 /* 2^-39 */

    /* type definitions ----------------------------------------------------------*/

    typedef struct {        /* time struct */
        time_t time;        /* time (s) expressed by standard time_t */
        double sec;         /* fraction of second under 1 s */
    } gtime_t;

    typedef struct {           /* SBAS message type */
        //int week,tow;        /* receiption time */
        double sample_stamp;
        int prn;               /* SBAS satellite PRN number */
        unsigned char msg[29]; /* SBAS message (226bit) padded by 0 */
    } sbsmsg_t;

    typedef struct {        /* SBAS messages type */
        int n,nmax;         /* number of SBAS messages/allocated */
        sbsmsg_t *msgs;     /* SBAS messages */
    } sbs_t;

    typedef struct {        /* SBAS fast correction type */
        //gtime_t t0;         /* time of applicability (TOF) */
        double t0;
        bool valid;
        double prc;         /* pseudorange correction (PRC) (m) */
        double rrc;         /* range-rate correction (RRC) (m/s) */
        double dt;          /* range-rate correction delta-time (s) */
        int iodf;           /* IODF (issue of date fast corr) */
        short udre;         /* UDRE+1 */
        short ai;           /* degradation factor indicator */
    } sbsfcorr_t;

    typedef struct {        /* SBAS long term satellite error correction type */
        //gtime_t t0;         /* correction time */
        double trx;        /* time when message was received */
        int tapp;           /* time of applicability (when vel=1 sent as t0) */
        int vel;            /* use velocity if vel=1 */
        bool valid;
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
        //gtime_t t0;         /* correction time */
        double t0;
        bool valid;
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

    /*
     *  indicators
     */

    typedef struct {        /* SBAS ephemeris type */
        int sat = 0;            /* satellite number */
        //gtime_t t0;         /* reference epoch time (GPST) */
        int t0 = 0;
        //gtime_t tof;        /* time of message frame (GPST) */
        double tof = 0;
        int sva = 0;            /* SV accuracy (URA index) */
        int svh = 0;            /* SV health (0:ok) */
        double pos[3] = {0, 0, 0};      /* satellite position (m) (ecef) */
        double vel[3] = {0, 0, 0};      /* satellite velocity (m/s) (ecef) */
        double acc[3] = {0, 0, 0};      /* satellite acceleration (m/s^2) (ecef) */
        double af0 = 0;
        double af1 = 0;     /* satellite clock-offset/drift (s,s/s) */
    } seph_t;

    typedef struct {        /* navigation data type */
        //int n,nmax;         /* number of broadcast ephemeris */
        //int ng,ngmax;       /* number of glonass ephemeris */
        //int ns,nsmax;       /* number of sbas ephemeris */
        //int ne,nemax;       /* number of precise ephemeris */
        //int nc,ncmax;       /* number of precise clock */
        //int na,namax;       /* number of almanac data */
        //int nt,ntmax;       /* number of tec grid data */
        //int nn,nnmax;       /* number of stec grid data */
        //eph_t *eph;         /* GPS/QZS/GAL ephemeris */
        //geph_t *geph;       /* GLONASS ephemeris */
        seph_t seph[2*NSATSBS];       /* SBAS ephemeris */
        //    peph_t *peph;       /* precise ephemeris */
        //    pclk_t *pclk;       /* precise clock */
        //    alm_t *alm;         /* almanac data */
        //    tec_t *tec;         /* tec grid data */
        //    stec_t *stec;       /* stec grid data */
        //    erp_t  erp;         /* earth rotation parameters */

        //double utc_gps[4];  /* GPS delta-UTC parameters {A0,A1,T,W} */
        //double utc_glo[4];  /* GLONASS UTC GPS time parameters */
        //double utc_gal[4];  /* Galileo UTC GPS time parameters */
        //double utc_qzs[4];  /* QZS UTC GPS time parameters */
        //double utc_cmp[4];  /* BeiDou UTC parameters */
        //double utc_sbs[4];  /* SBAS UTC parameters */
        //double ion_gps[8];  /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
        //double ion_gal[4];  /* Galileo iono model parameters {ai0,ai1,ai2,0} */
        //double ion_qzs[8];  /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
        //double ion_cmp[8];  /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
        //int leaps;          /* leap seconds (s) */
        //double lam[MAXSAT][NFREQ]; /* carrier wave lengths (m) */
        //double cbias[MAXSAT][3];   /* code bias (0:p1-p2,1:p1-c1,2:p2-c2) (m) */
        //double wlbias[MAXSAT];     /* wide-lane bias (cycle) */
        //double glo_cpbias[4];    /* glonass code-phase bias {1C,1P,2C,2P} (m) */
        //char glo_fcn[MAXPRNGLO+1]; /* glonass frequency channel number + 8 */
        //    pcv_t pcvs[MAXSAT]; /* satellite antenna pcv */
        sbssat_t sbssat;    /* SBAS satellite corrections */
        sbsion_t sbsion[MAXBAND+1]; /* SBAS ionosphere corrections */
        //    dgps_t dgps[MAXSAT]; /* DGPS corrections */
        //    ssr_t ssr[MAXSAT];  /* SSR corrections */
        //    lexeph_t lexeph[MAXSAT]; /* LEX ephemeris */
        //    lexion_t lexion;    /* LEX ionosphere correction */
    } nav_t;

    //// common

    static const double gpst0[]; /* gps time reference */

    /* debug trace functions -----------------------------------------------------*/

    FILE *fp_trace;     /* file pointer of trace */
    int level_trace;       /* level of trace */
    unsigned int tick_trace; /* tick time at traceopen (ms) */

    void trace(int level, const char *format, ...);

    /* satellite system+prn/slot number to satellite number ------------------------
     * convert satellite system+prn/slot number to satellite number
     * args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
     *          int    prn       I   satellite prn/slot number
     * return : satellite number (0:error)
     *-----------------------------------------------------------------------------*/
    int satno(int sys, int prn);

    /* extract unsigned/signed bits ------------------------------------------------
     * extract unsigned/signed bits from byte data
     * args   : unsigned char *buff I byte data
     *          int    pos    I      bit position from start of data (bits)
     *          int    len    I      bit length (bits) (len<=32)
     * return : extracted unsigned/signed bits
     *-----------------------------------------------------------------------------*/
    unsigned int getbitu(const unsigned char *buff, int pos, int len);

    int getbits(const unsigned char *buff, int pos, int len);

    /* convert calendar day/time to time -------------------------------------------
     * convert calendar day/time to gtime_t struct
     * args   : double *ep       I   day/time {year,month,day,hour,min,sec}
     * return : gtime_t struct
     * notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
     *-----------------------------------------------------------------------------*/
    gtime_t epoch2time(const double *ep);

    /* time difference -------------------------------------------------------------
     * difference between gtime_t structs
     * args   : gtime_t t1,t2    I   gtime_t structs
     * return : time difference (t1-t2) (s)
     *-----------------------------------------------------------------------------*/
    double timediff(gtime_t t1, gtime_t t2);

    /* gps time to time ------------------------------------------------------------
     * convert week and tow in gps time to gtime_t struct
     * args   : int    week      I   week number in gps time
     *          double sec       I   time of week in gps time (s)
     * return : gtime_t struct
     *-----------------------------------------------------------------------------*/
    gtime_t gpst2time(int week, double sec);


    ////// sbas.c

    /* sbas igp definition -------------------------------------------------------*/
    static const short
    x1[],
    x2[],
    x3[],
    x4[],
    x5[],
    x6[],
    x7[],
    x8[];

    static const sbsigpband_t igpband1[9][8]; /* band 0-8 */
    static const sbsigpband_t igpband2[2][5]; /* band 9-10 */

    /* decode type 1: prn masks --------------------------------------------------*/
    int decode_sbstype1(const sbsmsg_t *msg, sbssat_t *sbssat);
    /* decode type 2-5,0: fast corrections ---------------------------------------*/
    int decode_sbstype2(const sbsmsg_t *msg, sbssat_t *sbssat);
    /* decode type 6: integrity info ---------------------------------------------*/
    int decode_sbstype6(const sbsmsg_t *msg, sbssat_t *sbssat);
    /* decode type 7: fast correction degradation factor -------------------------*/
    int decode_sbstype7(const sbsmsg_t *msg, sbssat_t *sbssat);
    /* decode type 9: geo navigation message -------------------------------------*/
    int decode_sbstype9(const sbsmsg_t *msg, nav_t *nav);
    /* decode type 18: ionospheric grid point masks ------------------------------*/
    int decode_sbstype18(const sbsmsg_t *msg, sbsion_t *sbsion);
    /* decode half long term correction (vel code=0) -----------------------------*/
    int decode_longcorr0(const sbsmsg_t *msg, int p, sbssat_t *sbssat);
    /* decode half long term correction (vel code=1) -----------------------------*/
    int decode_longcorr1(const sbsmsg_t *msg, int p, sbssat_t *sbssat);
    /* decode half long term correction ------------------------------------------*/
    int decode_longcorrh(const sbsmsg_t *msg, int p, sbssat_t *sbssat);
    /* decode type 24: mixed fast/long term correction ---------------------------*/
    int decode_sbstype24(const sbsmsg_t *msg, sbssat_t *sbssat);
    /* decode type 25: long term satellite error correction ----------------------*/
    int decode_sbstype25(const sbsmsg_t *msg, sbssat_t *sbssat);
    /* decode type 26: ionospheric deley corrections -----------------------------*/
    int decode_sbstype26(const sbsmsg_t *msg, sbsion_t *sbsion);
    /* update sbas corrections -----------------------------------------------------
     * update sbas correction parameters in navigation data with a sbas message
     * args   : sbsmg_t  *msg    I   sbas message
     *          nav_t    *nav    IO  navigation data
     * return : message type (-1: error or not supported type)
     * notes  : nav->seph must point to seph[NSATSBS*2] (array of seph_t)
     *               seph[prn-MINPRNSBS+1]          : sat prn current epehmeris
     *               seph[prn-MINPRNSBS+1+MAXPRNSBS]: sat prn previous epehmeris
     *-----------------------------------------------------------------------------*/
    int sbsupdatecorr(const sbsmsg_t *msg, nav_t *nav);

    void prn_mask_changed();
    bool is_rtklib_sat_correction_valid(int sat);
    void igp_mask_changed(int band);

    // RTKLIB SBAS telemetry data representation
    nav_t d_nav;
};

#endif
