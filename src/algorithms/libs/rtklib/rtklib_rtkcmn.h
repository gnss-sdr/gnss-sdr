/*!
 * \file rtklib_rtkcmn.h
 * \brief rtklib common functions
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
* options : -DLAPACK   use LAPACK/BLAS
*           -DMKL      use Intel MKL
*           -DTRACE    enable debug trace
*           -DWIN32    use WIN32 API
*           -DNOCALLOC no use calloc for zero matrix
*           -DIERS_MODEL use GMF instead of NMF
*           -DDLL      built for shared library
*           -DCPUTIME_IN_GPST cputime operated in gpst
*
* references :
*     [1] IS-GPS-200D, Navstar GPS Space Segment/Navigation User Interfaces,
*         7 March, 2006
*     [2] RTCA/DO-229C, Minimum operational performanc standards for global
*         positioning system/wide area augmentation system airborne equipment,
*         RTCA inc, November 28, 2001
*     [3] M.Rothacher, R.Schmid, ANTEX: The Antenna Exchange Format Version 1.4,
*         15 September, 2010
*     [4] A.Gelb ed., Applied Optimal Estimation, The M.I.T Press, 1974
*     [5] A.E.Niell, Global mapping functions for the atmosphere delay at radio
*         wavelengths, Jounal of geophysical research, 1996
*     [6] W.Gurtner and L.Estey, RINEX The Receiver Independent Exchange Format
*         Version 3.00, November 28, 2007
*     [7] J.Kouba, A Guide to using International GNSS Service (IGS) products,
*         May 2009
*     [8] China Satellite Navigation Office, BeiDou navigation satellite system
*         signal in space interface control document, open service signal B1I
*         (version 1.0), Dec 2012
*     [9] J.Boehm, A.Niell, P.Tregoning and H.Shuh, Global Mapping Function
*         (GMF): A new empirical mapping function base on numerical weather
*         model data, Geophysical Research Letters, 33, L07304, 2006
*     [10] GLONASS/GPS/Galileo/Compass/SBAS NV08C receiver series BINR interface
*         protocol specification ver.1.3, August, 2012
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/01/12 1.0 new
*           2007/03/06 1.1 input initial rover pos of pntpos()
*                          update only effective states of filter()
*                          fix bug of atan2() domain error
*           2007/04/11 1.2 add function antmodel()
*                          add gdop mask for pntpos()
*                          change constant MAXDTOE value
*           2007/05/25 1.3 add function execcmd(),expandpath()
*           2008/06/21 1.4 add funciton sortobs(),uniqeph(),screent()
*                          replace geodist() by sagnac correction way
*           2008/10/29 1.5 fix bug of ionosphereic mapping function
*                          fix bug of seasonal variation term of tropmapf
*           2008/12/27 1.6 add function tickget(), sleepms(), tracenav(),
*                          xyz2enu(), satposv(), pntvel(), covecef()
*           2009/03/12 1.7 fix bug on error-stop when localtime() returns NULL
*           2009/03/13 1.8 fix bug on time adjustment for summer time
*           2009/04/10 1.9 add function adjgpsweek(),getbits(),getbitu()
*                          add function geph2pos()
*           2009/06/08 1.10 add function seph2pos()
*           2009/11/28 1.11 change function pntpos()
*                           add function tracegnav(),tracepeph()
*           2009/12/22 1.12 change default parameter of ionos std
*                           valid under second for timeget()
*           2010/07/28 1.13 fix bug in tropmapf()
*                           added api:
*                               obs2code(),code2obs(),cross3(),normv3(),
*                               gst2time(),time2gst(),time_str(),timeset(),
*                               deg2dms(),dms2deg(),searchpcv(),antmodel_s(),
*                               tracehnav(),tracepclk(),reppath(),reppaths(),
*                               createdir()
*                           changed api:
*                               readpcv(),
*                           deleted api:
*                               uniqeph()
*           2010/08/20 1.14 omit to include mkl header files
*                           fix bug on chi-sqr(n) table
*           2010/12/11 1.15 added api:
*                               freeobs(),freenav(),ionppp()
*           2011/05/28 1.16 fix bug on half-hour offset by time2epoch()
*                           added api:
*                               uniqnav()
*           2012/06/09 1.17 add a leap second after 2012-6-30
*           2012/07/15 1.18 add api setbits(),setbitu(),utc2gmst()
*                           fix bug on interpolation of antenna pcv
*                           fix bug on str2num() for string with over 256 char
*                           add api readblq(),satexclude(),setcodepri(),
*                           getcodepri()
*                           change api obs2code(),code2obs(),antmodel()
*           2012/12/25 1.19 fix bug on satwavelen(),code2obs(),obs2code()
*                           add api testsnr()
*           2013/01/04 1.20 add api gpst2bdt(),bdt2gpst(),bdt2time(),time2bdt()
*                           readblq(),readerp(),geterp(),crc16()
*                           change api eci2ecef(),sunmoonpos()
*           2013/03/26 1.21 tickget() uses clock_gettime() for linux
*           2013/05/08 1.22 fix bug on nutation coefficients for ast_args()
*           2013/06/02 1.23 add #ifdef for undefined CLOCK_MONOTONIC_RAW
*           2013/09/01 1.24 fix bug on interpolation of satellite antenna pcv
*           2013/09/06 1.25 fix bug on extrapolation of erp
*           2014/04/27 1.26 add SYS_LEO for satellite system
*                           add BDS L1 code for RINEX 3.02 and RTCM 3.2
*                           support BDS L1 in satwavelen()
*           2014/05/29 1.27 fix bug on obs2code() to search obs code table
*           2014/08/26 1.28 fix problem on output of uncompress() for tar file
*                           add function to swap trace file with keywords
*           2014/10/21 1.29 strtok() -> strtok_r() in expath() for thread-safe
*                           add bdsmodear in procopt_default
*           2015/03/19 1.30 fix bug on interpolation of erp values in geterp()
*                           add leap second insertion before 2015/07/01 00:00
*                           add api read_leaps()
*           2015/05/31 1.31 delte api windupcorr()
*           2015/08/08 1.32 add compile option CPUTIME_IN_GPST
*                           add api add_fatal()
*                           support usno leapsec.dat for api read_leaps()
*           2016/01/23 1.33 enable septentrio
*           2016/02/05 1.34 support GLONASS for savenav(), loadnav()
*           2016/06/11 1.35 delete trace() in reppath() to avoid deadlock
*           2016/07/01 1.36 support IRNSS
*                           add leap second before 2017/1/1 00:00:00
*           2016/07/29 1.37 rename api compress() -> rtk_uncompress()
*                           rename api crc16()    -> rtk_crc16()
*                           rename api crc24q()   -> rtk_crc24q()
*                           rename api crc32()    -> rtk_crc32()
*           2016/08/20 1.38 fix type incompatibility in win64 environment
*                           change constant _POSIX_C_SOURCE 199309 -> 199506
*           2016/08/21 1.39 fix bug on week overflow in time2gpst()/gpst2time()
*           2016/09/05 1.40 fix bug on invalid nav data read in readnav()
*           2016/09/17 1.41 suppress warnings
*           2016/09/19 1.42 modify api deg2dms() to consider numerical error
*-----------------------------------------------------------------------------*/
#ifndef RTKLIB_RTKCMN_H_
#define RTKLIB_RTKCMN_H_

#include "rtklib.h"


void fatalerr(const char *format, ...);
int satno(int sys, int prn);
int satsys(int sat, int *prn);
int satid2no(const char *id);
void satno2id(int sat, char *id);
int satexclude(int sat, int svh, const prcopt_t *opt);
int testsnr(int base, int freq, double el, double snr,const snrmask_t *mask);
unsigned char obs2code(const char *obs, int *freq);
char *code2obs(unsigned char code, int *freq);
void setcodepri(int sys, int freq, const char *pri);
int getcodepri(int sys, unsigned char code, const char *opt);
unsigned int getbitu(const unsigned char *buff, int pos, int len);
int getbits(const unsigned char *buff, int pos, int len);
void setbitu(unsigned char *buff, int pos, int len, unsigned int data);
void setbits(unsigned char *buff, int pos, int len, int data);
unsigned int rtk_crc32(const unsigned char *buff, int len);
unsigned int rtk_crc24q(const unsigned char *buff, int len);
unsigned short rtk_crc16(const unsigned char *buff, int len);
int decode_word(unsigned int word, unsigned char *data);
double *mat(int n, int m);
int *imat(int n, int m);
double *zeros(int n, int m);
double *eye(int n);
double dot(const double *a, const double *b, int n);
double norm(const double *a, int n);
void cross3(const double *a, const double *b, double *c);
int normv3(const double *a, double *b);
void matcpy(double *A, const double *B, int n, int m);
void matmul(const char *tr, int n, int k, int m, double alpha,
                   const double *A, const double *B, double beta, double *C);
int matinv(double *A, int n);
int solve(const char *tr, const double *A, const double *Y, int n,
                 int m, double *X);
int lsq(const double *A, const double *y, int n, int m, double *x,
               double *Q);
int filter_(const double *x, const double *P, const double *H,
                   const double *v, const double *R, int n, int m,
                   double *xp, double *Pp);
int filter(double *x, double *P, const double *H, const double *v,
                  const double *R, int n, int m);
int smoother(const double *xf, const double *Qf, const double *xb,
                    const double *Qb, int n, double *xs, double *Qs);
void matfprint(const double A[], int n, int m, int p, int q, FILE *fp);
void matprint(const double A[], int n, int m, int p, int q);
double str2num(const char *s, int i, int n);
int str2time(const char *s, int i, int n, gtime_t *t);
gtime_t epoch2time(const double *ep);
void time2epoch(gtime_t t, double *ep);
gtime_t gpst2time(int week, double sec);
double time2gpst(gtime_t t, int *week);
gtime_t gst2time(int week, double sec);
double time2gst(gtime_t t, int *week);
gtime_t bdt2time(int week, double sec);
double time2bdt(gtime_t t, int *week);
gtime_t timeadd(gtime_t t, double sec);
double timediff(gtime_t t1, gtime_t t2);
gtime_t timeget(void);
//void timeset(gtime_t t);
int read_leaps_text(FILE *fp);
int read_leaps_usno(FILE *fp);
int read_leaps(const char *file);
gtime_t gpst2utc(gtime_t t);
gtime_t utc2gpst(gtime_t t);
gtime_t gpst2bdt(gtime_t t);
gtime_t bdt2gpst(gtime_t t);
double time2sec(gtime_t time, gtime_t *day);
double utc2gmst(gtime_t t, double ut1_utc);
void time2str(gtime_t t, char *s, int n);
char *time_str(gtime_t t, int n);
double time2doy(gtime_t t);
int adjgpsweek(int week);
unsigned int tickget(void);
void sleepms(int ms);
void deg2dms(double deg, double *dms, int ndec);
double dms2deg(const double *dms);
void ecef2pos(const double *r, double *pos);
void pos2ecef(const double *pos, double *r);
void xyz2enu(const double *pos, double *E);
void ecef2enu(const double *pos, const double *r, double *e);
void enu2ecef(const double *pos, const double *e, double *r);
void covenu(const double *pos, const double *P, double *Q);
void covecef(const double *pos, const double *Q, double *P);

/* coordinate rotation matrix ------------------------------------------------*/
#define Rx(t,X) do { \
    (X)[0]=1.0; (X)[1]=(X)[2]=(X)[3]=(X)[6]=0.0; \
    (X)[4]=(X)[8]=cos(t); (X)[7]=sin(t); (X)[5]=-(X)[7]; \
} while (0)

#define Ry(t,X) do { \
    (X)[4]=1.0; (X)[1]=(X)[3]=(X)[5]=(X)[7]=0.0; \
    (X)[0]=(X)[8]=cos(t); (X)[2]=sin(t); (X)[6]=-(X)[2]; \
} while (0)

#define Rz(t,X) do { \
    (X)[8]=1.0; (X)[2]=(X)[5]=(X)[6]=(X)[7]=0.0; \
    (X)[0]=(X)[4]=cos(t); (X)[3]=sin(t); (X)[1]=-(X)[3]; \
} while (0)

void ast_args(double t, double *f);
void nut_iau1980(double t, const double *f, double *dpsi, double *deps);
void eci2ecef(gtime_t tutc, const double *erpv, double *U, double *gmst);
int decodef(char *p, int n, double *v);
void addpcv(const pcv_t *pcv, pcvs_t *pcvs);
int readngspcv(const char *file, pcvs_t *pcvs);
int readantex(const char *file, pcvs_t *pcvs);
int readpcv(const char *file, pcvs_t *pcvs);
pcv_t *searchpcv(int sat, const char *type, gtime_t time,
                        const pcvs_t *pcvs);
void readpos(const char *file, const char *rcv, double *pos);
int readblqrecord(FILE *fp, double *odisp);
int readblq(const char *file, const char *sta, double *odisp);
int readerp(const char *file, erp_t *erp);
int geterp(const erp_t *erp, gtime_t time, double *erpv);
int cmpeph(const void *p1, const void *p2);
void uniqeph(nav_t *nav);
int cmpgeph(const void *p1, const void *p2);
void uniqgeph(nav_t *nav);
int cmpseph(const void *p1, const void *p2);
void uniqseph(nav_t *nav);
void uniqnav(nav_t *nav);
int cmpobs(const void *p1, const void *p2);
int sortobs(obs_t *obs);
int screent(gtime_t time, gtime_t ts, gtime_t te, double tint);
int readnav(const char *file, nav_t *nav);
int savenav(const char *file, const nav_t *nav);
void freeobs(obs_t *obs);
void freenav(nav_t *nav, int opt);

void traceopen(const char *file);
void traceclose(void);
void tracelevel(int level);
void trace   (int level, const char *format, ...);
void tracet  (int level, const char *format, ...);
void tracemat(int level, const double *A, int n, int m, int p, int q);
void traceobs(int level, const obsd_t *obs, int n);
void tracenav(int level, const nav_t *nav);
void tracegnav(int level, const nav_t *nav);
void tracehnav(int level, const nav_t *nav);
void tracepeph(int level, const nav_t *nav);
void tracepclk(int level, const nav_t *nav);
void traceb  (int level, const unsigned char *p, int n);

int execcmd(const char *cmd);
void createdir(const char *path);
int repstr(char *str, const char *pat, const char *rep);
int reppath(const char *path, char *rpath, gtime_t time, const char *rov,
                   const char *base);
int reppaths(const char *path, char *rpath[], int nmax, gtime_t ts,
                    gtime_t te, const char *rov, const char *base);
double satwavelen(int sat, int frq, const nav_t *nav);
double geodist(const double *rs, const double *rr, double *e);
double satazel(const double *pos, const double *e, double *azel);

#define SQRT(x)     ((x)<0.0?0.0:sqrt(x));
void dops(int ns, const double *azel, double elmin, double *dop);
double ionmodel(gtime_t t, const double *ion, const double *pos,
                       const double *azel);
double ionmapf(const double *pos, const double *azel);
double ionppp(const double *pos, const double *azel, double re,
                     double hion, double *posp);
double tropmodel(gtime_t time, const double *pos, const double *azel,
                        double humi);
double interpc(const double coef[], double lat);
double mapf(double el, double a, double b, double c);
double nmf(gtime_t time, const double pos[], const double azel[],
                  double *mapfw);
double tropmapf(gtime_t time, const double pos[], const double azel[],
                       double *mapfw);
double interpvar(double ang, const double *var);

void antmodel(const pcv_t *pcv, const double *del, const double *azel,
                     int opt, double *dant);

void antmodel_s(const pcv_t *pcv, double nadir, double *dant);
void sunmoonpos_eci(gtime_t tut, double *rsun, double *rmoon);
void sunmoonpos(gtime_t tutc, const double *erpv, double *rsun,
                       double *rmoon, double *gmst);
void csmooth(obs_t *obs, int ns);
int rtk_uncompress(const char *file, char *uncfile);
int expath(const char *path, char *paths[], int nmax);

#endif /* RTKLIB_RTKCMN_H_ */
