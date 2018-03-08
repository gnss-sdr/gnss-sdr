/*!
 * \file rtklib_preceph.h
 * \brief precise ephemeris and clock functions
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
 * References :
 *     [1] S.Hilla, The Extended Standard Product 3 Orbit Format (SP3-c),
 *         12 February, 2007
 *     [2] J.Ray, W.Gurtner, RINEX Extensions to Handle Clock Information,
 *         27 August, 1998
 *     [3] D.D.McCarthy, IERS Technical Note 21, IERS Conventions 1996, July 1996
 *     [4] D.A.Vallado, Fundamentals of Astrodynamics and Applications 2nd ed,
 *         Space Technology Library, 2004
 *
 *----------------------------------------------------------------------------*/

#ifndef GNSS_SDR_RTKLIB_PRECEPH_H_
#define GNSS_SDR_RTKLIB_PRECEPH_H_

#include "rtklib.h"


const int NMAX = 10;            /* order of polynomial interpolation */
const double MAXDTE = 900.0;    /* max time difference to ephem time (s) */
const double EXTERR_CLK = 1e-3; /* extrapolation error for clock (m/s) */
const double EXTERR_EPH = 5e-7; /* extrapolation error for ephem (m/s^2) */

int code2sys(char code);
int readsp3h(FILE *fp, gtime_t *time, char *type, int *sats,
    double *bfact, char *tsys);
int addpeph(nav_t *nav, peph_t *peph);
void readsp3b(FILE *fp, char type, int *sats, int ns, double *bfact,
    char *tsys, int index, int opt, nav_t *nav);
int cmppeph(const void *p1, const void *p2);
void combpeph(nav_t *nav, int opt);
void readsp3(const char *file, nav_t *nav, int opt);
int readsap(const char *file, gtime_t time, nav_t *nav);
int readdcbf(const char *file, nav_t *nav, const sta_t *sta);
int readdcb(const char *file, nav_t *nav, const sta_t *sta);
int addfcb(nav_t *nav, gtime_t ts, gtime_t te, int sat,
    const double *bias, const double *std);
int readfcbf(const char *file, nav_t *nav);
int readdcb(const char *file, nav_t *nav, const sta_t *sta);
int addfcb(nav_t *nav, gtime_t ts, gtime_t te, int sat,
    const double *bias, const double *std);
int readfcbf(const char *file, nav_t *nav);
int cmpfcb(const void *p1, const void *p2);
int readfcb(const char *file, nav_t *nav);
double interppol(const double *x, double *y, int n);
int pephpos(gtime_t time, int sat, const nav_t *nav, double *rs,
    double *dts, double *vare, double *varc);

int pephclk(gtime_t time, int sat, const nav_t *nav, double *dts,
    double *varc);

void satantoff(gtime_t time, const double *rs, int sat, const nav_t *nav,
    double *dant);
int peph2pos(gtime_t time, int sat, const nav_t *nav, int opt,
    double *rs, double *dts, double *var);

#endif  // GNSS_SDR_RTKLIB_PRECEPH_H_
