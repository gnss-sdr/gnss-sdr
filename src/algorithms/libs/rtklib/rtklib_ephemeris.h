/*!
 * \file rtklib_ephemeris.h
 * \brief satellite ephemeris and clock functions
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


#ifndef GNSS_SDR_RTKLIB_EPHEMERIS_H_
#define GNSS_SDR_RTKLIB_EPHEMERIS_H_

#include "rtklib.h"


double var_uraeph(int ura);
double var_urassr(int ura);
void alm2pos(gtime_t time, const alm_t *alm, double *rs, double *dts);
double eph2clk(gtime_t time, const eph_t *eph);
void eph2pos(gtime_t time, const eph_t *eph, double *rs, double *dts,
    double *var);
void deq(const double *x, double *xdot, const double *acc);
void glorbit(double t, double *x, const double *acc);
double geph2clk(gtime_t time, const geph_t *geph);

void geph2pos(gtime_t time, const geph_t *geph, double *rs, double *dts,
    double *var);
double seph2clk(gtime_t time, const seph_t *seph);
void seph2pos(gtime_t time, const seph_t *seph, double *rs, double *dts,
    double *var);
eph_t *seleph(gtime_t time, int sat, int iode, const nav_t *nav);
geph_t *selgeph(gtime_t time, int sat, int iode, const nav_t *nav);
seph_t *selseph(gtime_t time, int sat, const nav_t *nav);
int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
    double *dts);
//satellite position and clock by broadcast ephemeris
int ephpos(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
    int iode, double *rs, double *dts, double *var, int *svh);
int satpos_sbas(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
    double *rs, double *dts, double *var, int *svh);
int satpos_ssr(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
    int opt, double *rs, double *dts, double *var, int *svh);

int satpos(gtime_t time, gtime_t teph, int sat, int ephopt,
    const nav_t *nav, double *rs, double *dts, double *var,
    int *svh);
void satposs(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav,
    int ephopt, double *rs, double *dts, double *var, int *svh);


#endif /* GNSS_SDR_RTKLIB_EPHEMERIS_H_ */
