/*!
 * \file rtklib_ionex.h
 * \brief ionex functions
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
 * References:
 *     [1] S.Schear, W.Gurtner and J.Feltens, IONEX: The IONosphere Map EXchange
 *         Format Version 1, February 25, 1998
 *     [2] S.Schaer, R.Markus, B.Gerhard and A.S.Timon, Daily Global Ionosphere
 *         Maps based on GPS Carrier Phase Data Routinely produced by CODE
 *         Analysis Center, Proceeding of the IGS Analysis Center Workshop, 1996
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RTKLIB_IONEX_H
#define GNSS_SDR_RTKLIB_IONEX_H

#include "rtklib.h"

const double VAR_NOTEC = 30.0 * 30.0; /* variance of no tec */
const double MIN_EL = 0.0;            /* min elevation angle (rad) */
const double MIN_HGT = -1000.0;       /* min user height (m) */

int getindex(double value, const double *range);

int nitem(const double *range);
int dataindex(int i, int j, int k, const int *ndata);
tec_t *addtec(const double *lats, const double *lons, const double *hgts,
    double rb, nav_t *nav);
void readionexdcb(FILE *fp, double *dcb, double *rms);
double readionexh(FILE *fp, double *lats, double *lons, double *hgts,
    double *rb, double *nexp, double *dcb, double *rms);
int readionexb(FILE *fp, const double *lats, const double *lons,
    const double *hgts, double rb, double nexp, nav_t *nav);
void combtec(nav_t *nav);
void readtec(const char *file, nav_t *nav, int opt);
int interptec(const tec_t *tec, int k, const double *posp, double *value,
    double *rms);

int iondelay(gtime_t time, const tec_t *tec, const double *pos,
    const double *azel, int opt, double *delay, double *var);
int iontec(gtime_t time, const nav_t *nav, const double *pos,
    const double *azel, int opt, double *delay, double *var);

#endif  // GNSS_SDR_RTKLIB_IONEX_H
