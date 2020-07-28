/*!
 * \file rtklib_tides.h
 * \brief Tidal displacement corrections
 * \authors <ul>
 *          <li> 2015, T. Takasu
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
 * Copyright (C) 2015, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * References:
 *     [1] D.D.McCarthy, IERS Technical Note 21, IERS Conventions 1996, July 1996
 *     [2] D.D.McCarthy and G.Petit, IERS Technical Note 32, IERS Conventions
 *         2003, November 2003
 *     [3] D.A.Vallado, Fundamentals of Astrodynamics and Applications 2nd ed,
 *         Space Technology Library, 2004
 *     [4] J.Kouba, A Guide to using International GNSS Service (IGS) products,
 *         May 2009
 *     [5] G.Petit and B.Luzum (eds), IERS Technical Note No. 36, IERS
 *         Conventions (2010), 2010
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_RTKLIB_TIDES_H
#define GNSS_SDR_RTKLIB_TIDES_H


#include "rtklib.h"


const double GME = 3.986004415E+14; /* earth gravitational constant */
const double GMS = 1.327124E+20;    /* sun gravitational constant */
const double GMM = 4.902801E+12;    /* moon gravitational constant */

void tide_pl(const double *eu, const double *rp, double GMp,
    const double *pos, double *dr);

void tide_solid(const double *rsun, const double *rmoon,
    const double *pos, const double *E, double gmst, int opt,
    double *dr);

void tide_oload(gtime_t tut, const double *odisp, double *denu);

void iers_mean_pole(gtime_t tut, double *xp_bar, double *yp_bar);


void tide_pole(gtime_t tut, const double *pos, const double *erpv,
    double *denu);

void tidedisp(gtime_t tutc, const double *rr, int opt, const erp_t *erp,
    const double *odisp, double *dr);
#endif
