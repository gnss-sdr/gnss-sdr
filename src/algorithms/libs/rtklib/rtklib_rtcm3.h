/*!
 * \file rtklib_rtcm3.h
 * \brief RTCM v3 functions headers
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
 *----------------------------------------------------------------------------*/

#ifndef GNSS_SDR_RTKLIB_RTCM3_H_
#define GNSS_SDR_RTKLIB_RTCM3_H_

#include "rtklib.h"

/* constants -----------------------------------------------------------------*/

const double PRUNIT_GPS = 299792.458;           /* rtcm ver.3 unit of gps pseudorange (m) */
const double PRUNIT_GLO = 599584.916;           /* rtcm ver.3 unit of glonass pseudorange (m) */
const double RANGE_MS = SPEED_OF_LIGHT * 0.001; /* range in 1 ms */


/* ssr update intervals ------------------------------------------------------*/
const double ssrudint[16] = {
    1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600, 900, 1800, 3600, 7200, 10800};


/* ssr 3 and 7 signal and tracking mode ids ----------------------------------*/
const int codes_gps[] = {
    CODE_L1C, CODE_L1P, CODE_L1W, CODE_L1Y, CODE_L1M, CODE_L2C, CODE_L2D, CODE_L2S,
    CODE_L2L, CODE_L2X, CODE_L2P, CODE_L2W, CODE_L2Y, CODE_L2M, CODE_L5I, CODE_L5Q,
    CODE_L5X};


const int codes_glo[] = {
    CODE_L1C, CODE_L1P, CODE_L2C, CODE_L2P};


const int codes_gal[] = {
    CODE_L1A, CODE_L1B, CODE_L1C, CODE_L1X, CODE_L1Z, CODE_L5I, CODE_L5Q, CODE_L5X,
    CODE_L7I, CODE_L7Q, CODE_L7X, CODE_L8I, CODE_L8Q, CODE_L8X, CODE_L6A, CODE_L6B,
    CODE_L6C, CODE_L6X, CODE_L6Z};


const int codes_qzs[] = {
    CODE_L1C, CODE_L1S, CODE_L1L, CODE_L2S, CODE_L2L, CODE_L2X, CODE_L5I, CODE_L5Q,
    CODE_L5X, CODE_L6S, CODE_L6L, CODE_L6X, CODE_L1X};


const int codes_bds[] = {
    CODE_L1I, CODE_L1Q, CODE_L1X, CODE_L7I, CODE_L7Q, CODE_L7X, CODE_L6I, CODE_L6Q,
    CODE_L6X};


const int codes_sbs[] = {
    CODE_L1C, CODE_L5I, CODE_L5Q, CODE_L5X};


double getbitg(const unsigned char *buff, int pos, int len);

void adjweek(rtcm_t *rtcm, double tow);

int adjbdtweek(int week);

void adjday_glot(rtcm_t *rtcm, double tod);

double adjcp(rtcm_t *rtcm, int sat, int freq, double cp);

int lossoflock(rtcm_t *rtcm, int sat, int freq, int lock);

unsigned char snratio(double snr);

int obsindex3(obs_t *obs, gtime_t time, int sat);

int test_staid(rtcm_t *rtcm, int staid);

int decode_head1001(rtcm_t *rtcm, int *sync);

int decode_type1001(rtcm_t *rtcm);

int decode_type1002(rtcm_t *rtcm);

int decode_type1003(rtcm_t *rtcm);

int decode_type1004(rtcm_t *rtcm);

double getbits_38(const unsigned char *buff, int pos);

int decode_type1005(rtcm_t *rtcm);

int decode_type1006(rtcm_t *rtcm);

int decode_type1007(rtcm_t *rtcm);

int decode_type1008(rtcm_t *rtcm);

int decode_head1009(rtcm_t *rtcm, int *sync);

int decode_type1009(rtcm_t *rtcm);

int decode_type1010(rtcm_t *rtcm);

int decode_type1011(rtcm_t *rtcm);

int decode_type1012(rtcm_t *rtcm);

int decode_type1013(rtcm_t *rtcm);

int decode_type1019(rtcm_t *rtcm);

int decode_type1020(rtcm_t *rtcm);

int decode_type1021(rtcm_t *rtcm);

int decode_type1022(rtcm_t *rtcm);

int decode_type1023(rtcm_t *rtcm);

int decode_type1024(rtcm_t *rtcm);

int decode_type1025(rtcm_t *rtcm);

int decode_type1026(rtcm_t *rtcm);

int decode_type1027(rtcm_t *rtcm);

int decode_type1029(rtcm_t *rtcm);

int decode_type1030(rtcm_t *rtcm);

int decode_type1031(rtcm_t *rtcm);

int decode_type1032(rtcm_t *rtcm);

int decode_type1033(rtcm_t *rtcm);

int decode_type1034(rtcm_t *rtcm);

int decode_type1035(rtcm_t *rtcm);

int decode_type1037(rtcm_t *rtcm);

int decode_type1038(rtcm_t *rtcm);

int decode_type1039(rtcm_t *rtcm);

int decode_type1044(rtcm_t *rtcm);

int decode_type1045(rtcm_t *rtcm);

int decode_type1046(rtcm_t *rtcm);

int decode_type1047(rtcm_t *rtcm);

int decode_type1063(rtcm_t *rtcm);

int decode_ssr1_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
    double *udint, int *refd, int *hsize);

int decode_ssr2_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
    double *udint, int *hsize);

int decode_ssr7_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
    double *udint, int *dispe, int *mw, int *hsize);

int decode_ssr1(rtcm_t *rtcm, int sys);

int decode_ssr2(rtcm_t *rtcm, int sys);

int decode_ssr3(rtcm_t *rtcm, int sys);

int decode_ssr4(rtcm_t *rtcm, int sys);

int decode_ssr5(rtcm_t *rtcm, int sys);

int decode_ssr6(rtcm_t *rtcm, int sys);

int decode_ssr7(rtcm_t *rtcm, int sys);

void sigindex(int sys, const unsigned char *code, const int *freq, int n,
    const char *opt, int *ind);

void save_msm_obs(rtcm_t *rtcm, int sys, msm_h_t *h, const double *r,
    const double *pr, const double *cp, const double *rr,
    const double *rrf, const double *cnr, const int *lock,
    const int *ex, const int *half);

int decode_msm_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
    msm_h_t *h, int *hsize);

int decode_msm0(rtcm_t *rtcm, int sys);

int decode_msm4(rtcm_t *rtcm, int sys);

int decode_msm5(rtcm_t *rtcm, int sys);

int decode_msm6(rtcm_t *rtcm, int sys);

int decode_msm7(rtcm_t *rtcm, int sys);

int decode_type1230(rtcm_t *rtcm);

int decode_rtcm3(rtcm_t *rtcm);


#endif
