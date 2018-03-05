/*!
 * \file rtklib_lambda.cc
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
 * -------------------------------------------------------------------------
 * Copyright (C) 2007-2008, T. Takasu
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

#include "rtklib_lambda.h"
#include "rtklib_rtkcmn.h"

/* LD factorization (Q=L'*diag(D)*L) -----------------------------------------*/
int LD(int n, const double *Q, double *L, double *D)
{
    int i, j, k, info = 0;
    double a, *A = mat(n, n);

    memcpy(A, Q, sizeof(double) * n * n);
    for (i = n - 1; i >= 0; i--)
        {
            if ((D[i] = A[i + i * n]) <= 0.0)
                {
                    info = -1;
                    break;
                }
            a = sqrt(D[i]);
            for (j = 0; j <= i; j++) L[i + j * n] = A[i + j * n] / a;
            for (j = 0; j <= i - 1; j++)
                for (k = 0; k <= j; k++) A[j + k * n] -= L[i + k * n] * L[i + j * n];
            for (j = 0; j <= i; j++) L[i + j * n] /= L[i + i * n];
        }
    free(A);
    if (info) fprintf(stderr, "%s : LD factorization error\n", __FILE__);
    return info;
}


/* integer gauss transformation ----------------------------------------------*/
void gauss(int n, double *L, double *Z, int i, int j)
{
    int k, mu;

    if ((mu = (int)ROUND_LAMBDA(L[i + j * n])) != 0)
        {
            for (k = i; k < n; k++) L[k + n * j] -= (double)mu * L[k + i * n];
            for (k = 0; k < n; k++) Z[k + n * j] -= (double)mu * Z[k + i * n];
        }
}


/* permutations --------------------------------------------------------------*/
void perm(int n, double *L, double *D, int j, double del, double *Z)
{
    int k;
    double eta, lam, a0, a1;

    eta = D[j] / del;
    lam = D[j + 1] * L[j + 1 + j * n] / del;
    D[j] = eta * D[j + 1];
    D[j + 1] = del;
    for (k = 0; k <= j - 1; k++)
        {
            a0 = L[j + k * n];
            a1 = L[j + 1 + k * n];
            L[j + k * n] = -L[j + 1 + j * n] * a0 + a1;
            L[j + 1 + k * n] = eta * a0 + lam * a1;
        }
    L[j + 1 + j * n] = lam;
    for (k = j + 2; k < n; k++) SWAP_LAMBDA(L[k + j * n], L[k + (j + 1) * n]);
    for (k = 0; k < n; k++) SWAP_LAMBDA(Z[k + j * n], Z[k + (j + 1) * n]);
}


/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
void reduction(int n, double *L, double *D, double *Z)
{
    int i, j, k;
    double del;

    j = n - 2;
    k = n - 2;
    while (j >= 0)
        {
            if (j <= k)
                for (i = j + 1; i < n; i++) gauss(n, L, Z, i, j);
            del = D[j] + L[j + 1 + j * n] * L[j + 1 + j * n] * D[j + 1];
            if (del + 1E-6 < D[j + 1])
                { /* compared considering numerical error */
                    perm(n, L, D, j, del, Z);
                    k = j;
                    j = n - 2;
                }
            else
                j--;
        }
}


/* modified lambda (mlambda) search (ref. [2]) -------------------------------*/
int search(int n, int m, const double *L, const double *D,
    const double *zs, double *zn, double *s)
{
    int i, j, k, c, nn = 0, imax = 0;
    double newdist, maxdist = 1E99, y;
    double *S = zeros(n, n), *dist = mat(n, 1), *zb = mat(n, 1), *z = mat(n, 1), *step = mat(n, 1);

    k = n - 1;
    dist[k] = 0.0;
    zb[k] = zs[k];
    z[k] = ROUND_LAMBDA(zb[k]);
    y = zb[k] - z[k];
    step[k] = SGN_LAMBDA(y);
    for (c = 0; c < LOOPMAX; c++)
        {
            newdist = dist[k] + y * y / D[k];
            if (newdist < maxdist)
                {
                    if (k != 0)
                        {
                            dist[--k] = newdist;
                            for (i = 0; i <= k; i++)
                                S[k + i * n] = S[k + 1 + i * n] + (z[k + 1] - zb[k + 1]) * L[k + 1 + i * n];
                            zb[k] = zs[k] + S[k + k * n];
                            z[k] = ROUND_LAMBDA(zb[k]);
                            y = zb[k] - z[k];
                            step[k] = SGN_LAMBDA(y);
                        }
                    else
                        {
                            if (nn < m)
                                {
                                    if (nn == 0 || newdist > s[imax]) imax = nn;
                                    for (i = 0; i < n; i++) zn[i + nn * n] = z[i];
                                    s[nn++] = newdist;
                                }
                            else
                                {
                                    if (newdist < s[imax])
                                        {
                                            for (i = 0; i < n; i++) zn[i + imax * n] = z[i];
                                            s[imax] = newdist;
                                            for (i = imax = 0; i < m; i++)
                                                if (s[imax] < s[i]) imax = i;
                                        }
                                    maxdist = s[imax];
                                }
                            z[0] += step[0];
                            y = zb[0] - z[0];
                            step[0] = -step[0] - SGN_LAMBDA(step[0]);
                        }
                }
            else
                {
                    if (k == n - 1)
                        break;
                    else
                        {
                            k++;
                            z[k] += step[k];
                            y = zb[k] - z[k];
                            step[k] = -step[k] - SGN_LAMBDA(step[k]);
                        }
                }
        }
    for (i = 0; i < m - 1; i++)
        { /* sort by s */
            for (j = i + 1; j < m; j++)
                {
                    if (s[i] < s[j]) continue;
                    SWAP_LAMBDA(s[i], s[j]);
                    for (k = 0; k < n; k++) SWAP_LAMBDA(zn[k + i * n], zn[k + j * n]);
                }
        }
    free(S);
    free(dist);
    free(zb);
    free(z);
    free(step);

    if (c >= LOOPMAX)
        {
            fprintf(stderr, "%s : search loop count overflow\n", __FILE__);
            return -1;
        }
    return 0;
}


/* lambda/mlambda integer least-square estimation ------------------------------
 * integer least-square estimation. reduction is performed by lambda (ref.[1]),
 * and search by mlambda (ref.[2]).
 * args   : int    n      I  number of float parameters
 *          int    m      I  number of fixed solutions
 *          double *a     I  float parameters (n x 1)
 *          double *Q     I  covariance matrix of float parameters (n x n)
 *          double *F     O  fixed solutions (n x m)
 *          double *s     O  sum of squared residulas of fixed solutions (1 x m)
 * return : status (0:ok,other:error)
 * notes  : matrix stored by column-major order (fortran convension)
 *-----------------------------------------------------------------------------*/
int lambda(int n, int m, const double *a, const double *Q, double *F,
    double *s)
{
    int info;
    double *L, *D, *Z, *z, *E;

    if (n <= 0 || m <= 0) return -1;
    L = zeros(n, n);
    D = mat(n, 1);
    Z = eye(n);
    z = mat(n, 1);
    E = mat(n, m);

    /* LD factorization */
    if (!(info = LD(n, Q, L, D)))
        {
            /* lambda reduction */
            reduction(n, L, D, Z);
            matmul("TN", n, 1, n, 1.0, Z, a, 0.0, z); /* z=Z'*a */

            /* mlambda search */
            if (!(info = search(n, m, L, D, z, E, s)))
                {
                    info = solve("T", Z, E, n, m, F); /* F=Z'\E */
                }
        }
    free(L);
    free(D);
    free(Z);
    free(z);
    free(E);
    return info;
}


/* lambda reduction ------------------------------------------------------------
 * reduction by lambda (ref [1]) for integer least square
 * args   : int    n      I  number of float parameters
 *          double *Q     I  covariance matrix of float parameters (n x n)
 *          double *Z     O  lambda reduction matrix (n x n)
 * return : status (0:ok,other:error)
 *-----------------------------------------------------------------------------*/
int lambda_reduction(int n, const double *Q, double *Z)
{
    double *L, *D;
    int i, j, info;

    if (n <= 0) return -1;

    L = zeros(n, n);
    D = mat(n, 1);

    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            {
                Z[i + j * n] = i == j ? 1.0 : 0.0;
            }
    /* LD factorization */
    if ((info = LD(n, Q, L, D)))
        {
            free(L);
            free(D);
            return info;
        }
    /* lambda reduction */
    reduction(n, L, D, Z);

    free(L);
    free(D);
    return 0;
}


/* mlambda search --------------------------------------------------------------
 * search by  mlambda (ref [2]) for integer least square
 * args   : int    n      I  number of float parameters
 *          int    m      I  number of fixed solutions
 *          double *a     I  float parameters (n x 1)
 *          double *Q     I  covariance matrix of float parameters (n x n)
 *          double *F     O  fixed solutions (n x m)
 *          double *s     O  sum of squared residulas of fixed solutions (1 x m)
 * return : status (0:ok,other:error)
 *-----------------------------------------------------------------------------*/
int lambda_search(int n, int m, const double *a, const double *Q,
    double *F, double *s)
{
    double *L, *D;
    int info;

    if (n <= 0 || m <= 0) return -1;

    L = zeros(n, n);
    D = mat(n, 1);

    /* LD factorization */
    if ((info = LD(n, Q, L, D)))
        {
            free(L);
            free(D);
            return info;
        }
    /* mlambda search */
    info = search(n, m, L, D, a, F, s);

    free(L);
    free(D);
    return info;
}
