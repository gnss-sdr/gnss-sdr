/*!
 * \file rtklib_solution.h
 * \brief solution functions headers
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
 *-----------------------------------------------------------------------------*/

#ifndef GNSS_SDR_RTKLIB_SOLUTION_H_
#define GNSS_SDR_RTKLIB_SOLUTION_H_

#include "rtklib.h"

#define COMMENTH "%"                   /* comment line indicator for solution */
#define MSG_DISCONN "$_DISCONNECT\r\n" /* disconnect message */

const char *opt2sep(const solopt_t *opt);

int tonum(char *buff, const char *sep, double *v);

double sqvar(double covar);

double dmm2deg(double dmm);

void septime(double t, double *t1, double *t2, double *t3);

void soltocov(const sol_t *sol, double *P);

void covtosol(const double *P, sol_t *sol);

int decode_nmearmc(char **val, int n, sol_t *sol);

int decode_nmeagga(char **val, int n, sol_t *sol);

int decode_nmea(char *buff, sol_t *sol);

char *decode_soltime(char *buff, const solopt_t *opt, gtime_t *time);

int decode_solxyz(char *buff, const solopt_t *opt, sol_t *sol);

int decode_solllh(char *buff, const solopt_t *opt, sol_t *sol);

int decode_solenu(char *buff, const solopt_t *opt, sol_t *sol);

int decode_solgsi(char *buff, const solopt_t *opt, sol_t *sol);

int decode_solpos(char *buff, const solopt_t *opt, sol_t *sol);

void decode_refpos(char *buff, const solopt_t *opt, double *rb);

int decode_sol(char *buff, const solopt_t *opt, sol_t *sol, double *rb);

void decode_solopt(char *buff, solopt_t *opt);

void readsolopt(FILE *fp, solopt_t *opt);

int inputsol(unsigned char data, gtime_t ts, gtime_t te, double tint,
    int qflag, const solopt_t *opt, solbuf_t *solbuf);

int readsoldata(FILE *fp, gtime_t ts, gtime_t te, double tint, int qflag,
    const solopt_t *opt, solbuf_t *solbuf);

int cmpsol(const void *p1, const void *p2);

int sort_solbuf(solbuf_t *solbuf);

int readsolt(char *files[], int nfile, gtime_t ts, gtime_t te,
    double tint, int qflag, solbuf_t *solbuf);

int readsol(char *files[], int nfile, solbuf_t *sol);

int addsol(solbuf_t *solbuf, const sol_t *sol);

sol_t *getsol(solbuf_t *solbuf, int index);

void initsolbuf(solbuf_t *solbuf, int cyclic, int nmax);

void freesolbuf(solbuf_t *solbuf);

void freesolstatbuf(solstatbuf_t *solstatbuf);

int cmpsolstat(const void *p1, const void *p2);

int sort_solstat(solstatbuf_t *statbuf);

int decode_solstat(char *buff, solstat_t *stat);

void addsolstat(solstatbuf_t *statbuf, const solstat_t *stat);

int readsolstatdata(FILE *fp, gtime_t ts, gtime_t te, double tint,
    solstatbuf_t *statbuf);

int readsolstatt(char *files[], int nfile, gtime_t ts, gtime_t te,
    double tint, solstatbuf_t *statbuf);

int readsolstat(char *files[], int nfile, solstatbuf_t *statbuf);

int outecef(unsigned char *buff, const char *s, const sol_t *sol,
    const solopt_t *opt);

int outpos(unsigned char *buff, const char *s, const sol_t *sol, const solopt_t *opt);

int outenu(unsigned char *buff, const char *s, const sol_t *sol,
    const double *rb, const solopt_t *opt);

int outnmea_rmc(unsigned char *buff, const sol_t *sol);

int outnmea_gga(unsigned char *buff, const sol_t *sol);

int outnmea_gsa(unsigned char *buff, const sol_t *sol,
    const ssat_t *ssat);

int outnmea_gsv(unsigned char *buff, const sol_t *sol,
    const ssat_t *ssat);

int outprcopts(unsigned char *buff, const prcopt_t *opt);

int outsolheads(unsigned char *buff, const solopt_t *opt);

int outsols(unsigned char *buff, const sol_t *sol, const double *rb,
    const solopt_t *opt);

int outsolexs(unsigned char *buff, const sol_t *sol, const ssat_t *ssat,
    const solopt_t *opt);

void outprcopt(FILE *fp, const prcopt_t *opt);

void outsolhead(FILE *fp, const solopt_t *opt);

void outsol(FILE *fp, const sol_t *sol, const double *rb,
    const solopt_t *opt);

void outsolex(FILE *fp, const sol_t *sol, const ssat_t *ssat,
    const solopt_t *opt);


#endif
