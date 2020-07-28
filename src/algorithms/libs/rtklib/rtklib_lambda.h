/*!
 * \file rtklib_lambda.h
 * \brief Integer ambiguity resolution
 * \authors <ul>
 *          <li> 2007-2008, T. Takasu
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
 * Copyright (C) 2007-2008, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * References:
 *     [1] P.J.G.Teunissen, The least-square ambiguity decorrelation adjustment:
 *         a method for fast GPS ambiguity estimation, J.Geodesy, Vol.70, 65-82,
 *         1995
 *     [2] X.-W.Chang, X.Yang, T.Zhou, MLAMBDA: A modified LAMBDA method for
 *         integer least-squares estimation, J.Geodesy, Vol.79, 552-565, 2005
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RTKLIB_LAMBDA_H
#define GNSS_SDR_RTKLIB_LAMBDA_H


#include "rtklib.h"

/* constants/macros ----------------------------------------------------------*/
const int LOOPMAX = 10000; /* maximum count of search loop */
#define SGN_LAMBDA(x) ((x) <= 0.0 ? -1.0 : 1.0)
#define ROUND_LAMBDA(x) (floor((x) + 0.5))
#define SWAP_LAMBDA(x, y) \
    do                    \
        {                 \
            double tmp_;  \
            tmp_ = x;     \
            x = y;        \
            y = tmp_;     \
        }                 \
    while (0)

int LD(int n, const double *Q, double *L, double *D);
void gauss(int n, double *L, double *Z, int i, int j);
void perm(int n, double *L, double *D, int j, double del, double *Z);
void reduction(int n, double *L, double *D, double *Z);
int search(int n, int m, const double *L, const double *D,
    const double *zs, double *zn, double *s);

int lambda(int n, int m, const double *a, const double *Q, double *F, double *s);

int lambda_reduction(int n, const double *Q, double *Z);

int lambda_search(int n, int m, const double *a, const double *Q,
    double *F, double *s);


#endif
