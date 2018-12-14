/*!
 * \file rtklib_sbas.h
 * \brief sbas functions
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
 *     [1] RTCA/DO-229C, Minimum operational performanc standards for global
 *         positioning system/wide area augmentation system airborne equipment,
 *         RTCA inc, November 28, 2001
 *     [2] IS-QZSS v.1.1, Quasi-Zenith Satellite System Navigation Service
 *         Interface Specification for QZSS, Japan Aerospace Exploration Agency,
 *         July 31, 2009
 *
 *----------------------------------------------------------------------------*/

#ifndef GNSS_SDR_RTKLIB_SBAS_H_
#define GNSS_SDR_RTKLIB_SBAS_H_

#include "rtklib.h"

/* constants -----------------------------------------------------------------*/

const int WEEKOFFSET = 1024; /* gps week offset for NovAtel OEM-3 */

/* sbas igp definition -------------------------------------------------------*/
static const short
    x1[] = {-75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20,
        25, 30, 35, 40, 45, 50, 55, 65, 75, 85},
    x2[] = {-55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30,
        35, 40, 45, 50, 55},
    x3[] = {-75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20,
        25, 30, 35, 40, 45, 50, 55, 65, 75},
    x4[] = {-85, -75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15,
        20, 25, 30, 35, 40, 45, 50, 55, 65, 75},
    x5[] = {-180, -175, -170, -165, -160, -155, -150, -145, -140, -135, -130, -125, -120, -115,
        -110, -105, -100, -95, -90, -85, -80, -75, -70, -65, -60, -55, -50, -45,
        -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25,
        30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95,
        100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165,
        170, 175},
    x6[] = {-180, -170, -160, -150, -140, -130, -120, -110, -100, -90, -80, -70, -60, -50,
        -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90,
        100, 110, 120, 130, 140, 150, 160, 170},
    x7[] = {-180, -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150},
    x8[] = {-170, -140, -110, -80, -50, -20, 10, 40, 70, 100, 130, 160};

const sbsigpband_t igpband1[9][8] = {/* band 0-8 */
    {{-180, x1, 1, 28}, {-175, x2, 29, 51}, {-170, x3, 52, 78}, {-165, x2, 79, 101},
        {-160, x3, 102, 128}, {-155, x2, 129, 151}, {-150, x3, 152, 178}, {-145, x2, 179, 201}},
    {{-140, x4, 1, 28}, {-135, x2, 29, 51}, {-130, x3, 52, 78}, {-125, x2, 79, 101},
        {-120, x3, 102, 128}, {-115, x2, 129, 151}, {-110, x3, 152, 178}, {-105, x2, 179, 201}},
    {{-100, x3, 1, 27}, {-95, x2, 28, 50}, {-90, x1, 51, 78}, {-85, x2, 79, 101},
        {-80, x3, 102, 128}, {-75, x2, 129, 151}, {-70, x3, 152, 178}, {-65, x2, 179, 201}},
    {{-60, x3, 1, 27}, {-55, x2, 28, 50}, {-50, x4, 51, 78}, {-45, x2, 79, 101},
        {-40, x3, 102, 128}, {-35, x2, 129, 151}, {-30, x3, 152, 178}, {-25, x2, 179, 201}},
    {{-20, x3, 1, 27}, {-15, x2, 28, 50}, {-10, x3, 51, 77}, {-5, x2, 78, 100},
        {0, x1, 101, 128}, {5, x2, 129, 151}, {10, x3, 152, 178}, {15, x2, 179, 201}},
    {{20, x3, 1, 27}, {25, x2, 28, 50}, {30, x3, 51, 77}, {35, x2, 78, 100},
        {40, x4, 101, 128}, {45, x2, 129, 151}, {50, x3, 152, 178}, {55, x2, 179, 201}},
    {{60, x3, 1, 27}, {65, x2, 28, 50}, {70, x3, 51, 77}, {75, x2, 78, 100},
        {80, x3, 101, 127}, {85, x2, 128, 150}, {90, x1, 151, 178}, {95, x2, 179, 201}},
    {{100, x3, 1, 27}, {105, x2, 28, 50}, {110, x3, 51, 77}, {115, x2, 78, 100},
        {120, x3, 101, 127}, {125, x2, 128, 150}, {130, x4, 151, 178}, {135, x2, 179, 201}},
    {{140, x3, 1, 27}, {145, x2, 28, 50}, {150, x3, 51, 77}, {155, x2, 78, 100},
        {160, x3, 101, 127}, {165, x2, 128, 150}, {170, x3, 151, 177}, {175, x2, 178, 200}}};
const sbsigpband_t igpband2[2][5] = {/* band 9-10 */
    {{60, x5, 1, 72}, {65, x6, 73, 108}, {70, x6, 109, 144}, {75, x6, 145, 180},
        {85, x7, 181, 192}},
    {{-60, x5, 1, 72}, {-65, x6, 73, 108}, {-70, x6, 109, 144}, {-75, x6, 145, 180},
        {-85, x8, 181, 192}}};


char *getfield(char *p, int pos);
double varfcorr(int udre);
double varicorr(int give);
double degfcorr(int ai);

int decode_sbstype1(const sbsmsg_t *msg, sbssat_t *sbssat);
int decode_sbstype2(const sbsmsg_t *msg, sbssat_t *sbssat);
int decode_sbstype6(const sbsmsg_t *msg, sbssat_t *sbssat);
int decode_sbstype7(const sbsmsg_t *msg, sbssat_t *sbssat);
int decode_sbstype9(const sbsmsg_t *msg, nav_t *nav);
int decode_sbstype18(const sbsmsg_t *msg, sbsion_t *sbsion);
int decode_longcorr0(const sbsmsg_t *msg, int p, sbssat_t *sbssat);
int decode_longcorr1(const sbsmsg_t *msg, int p, sbssat_t *sbssat);
int decode_longcorrh(const sbsmsg_t *msg, int p, sbssat_t *sbssat);
int decode_sbstype24(const sbsmsg_t *msg, sbssat_t *sbssat);
int decode_sbstype25(const sbsmsg_t *msg, sbssat_t *sbssat);
int decode_sbstype26(const sbsmsg_t *msg, sbsion_t *sbsion);

int sbsupdatecorr(const sbsmsg_t *msg, nav_t *nav);
void readmsgs(const char *file, int sel, gtime_t ts, gtime_t te, sbs_t *sbs);
int cmpmsgs(const void *p1, const void *p2);
int sbsreadmsgt(const char *file, int sel, gtime_t ts, gtime_t te,
    sbs_t *sbs);
int sbsreadmsg(const char *file, int sel, sbs_t *sbs);
void sbsoutmsg(FILE *fp, sbsmsg_t *sbsmsg);
void searchigp(gtime_t time, const double *pos, const sbsion_t *ion,
    const sbsigp_t **igp, double *x, double *y);
int sbsioncorr(gtime_t time, const nav_t *nav, const double *pos,
    const double *azel, double *delay, double *var);

void getmet(double lat, double *met);
double sbstropcorr(gtime_t time, const double *pos, const double *azel,
    double *var);
int sbslongcorr(gtime_t time, int sat, const sbssat_t *sbssat,
    double *drs, double *ddts);
int sbsfastcorr(gtime_t time, int sat, const sbssat_t *sbssat,
    double *prc, double *var);

int sbssatcorr(gtime_t time, int sat, const nav_t *nav, double *rs,
    double *dts, double *var);
int sbsdecodemsg(gtime_t time, int prn, const unsigned int *words,
    sbsmsg_t *sbsmsg);


#endif /* GNSS_SDR_RTKLIB_SBAS_H_ */
