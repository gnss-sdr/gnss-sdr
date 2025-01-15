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
 * -----------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RTKLIB_SBAS_H
#define GNSS_SDR_RTKLIB_SBAS_H

#include "rtklib.h"

/* constants -----------------------------------------------------------------*/

const int WEEKOFFSET = 1024; /* gps week offset for NovAtel OEM-3 */

/* sbas igp definition -------------------------------------------------------*/
static const short
    X1[] = {-75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20,
        25, 30, 35, 40, 45, 50, 55, 65, 75, 85},
    X2[] = {-55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30,
        35, 40, 45, 50, 55},
    X3[] = {-75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20,
        25, 30, 35, 40, 45, 50, 55, 65, 75},
    X4[] = {-85, -75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15,
        20, 25, 30, 35, 40, 45, 50, 55, 65, 75},
    X5[] = {-180, -175, -170, -165, -160, -155, -150, -145, -140, -135, -130, -125, -120, -115,
        -110, -105, -100, -95, -90, -85, -80, -75, -70, -65, -60, -55, -50, -45,
        -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25,
        30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95,
        100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165,
        170, 175},
    X6[] = {-180, -170, -160, -150, -140, -130, -120, -110, -100, -90, -80, -70, -60, -50,
        -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90,
        100, 110, 120, 130, 140, 150, 160, 170},
    X7[] = {-180, -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150},
    X8[] = {-170, -140, -110, -80, -50, -20, 10, 40, 70, 100, 130, 160};

const sbsigpband_t IGPBAND1[9][8] = {/* band 0-8 */
    {{-180, X1, 1, 28}, {-175, X2, 29, 51}, {-170, X3, 52, 78}, {-165, X2, 79, 101},
        {-160, X3, 102, 128}, {-155, X2, 129, 151}, {-150, X3, 152, 178}, {-145, X2, 179, 201}},
    {{-140, X4, 1, 28}, {-135, X2, 29, 51}, {-130, X3, 52, 78}, {-125, X2, 79, 101},
        {-120, X3, 102, 128}, {-115, X2, 129, 151}, {-110, X3, 152, 178}, {-105, X2, 179, 201}},
    {{-100, X3, 1, 27}, {-95, X2, 28, 50}, {-90, X1, 51, 78}, {-85, X2, 79, 101},
        {-80, X3, 102, 128}, {-75, X2, 129, 151}, {-70, X3, 152, 178}, {-65, X2, 179, 201}},
    {{-60, X3, 1, 27}, {-55, X2, 28, 50}, {-50, X4, 51, 78}, {-45, X2, 79, 101},
        {-40, X3, 102, 128}, {-35, X2, 129, 151}, {-30, X3, 152, 178}, {-25, X2, 179, 201}},
    {{-20, X3, 1, 27}, {-15, X2, 28, 50}, {-10, X3, 51, 77}, {-5, X2, 78, 100},
        {0, X1, 101, 128}, {5, X2, 129, 151}, {10, X3, 152, 178}, {15, X2, 179, 201}},
    {{20, X3, 1, 27}, {25, X2, 28, 50}, {30, X3, 51, 77}, {35, X2, 78, 100},
        {40, X4, 101, 128}, {45, X2, 129, 151}, {50, X3, 152, 178}, {55, X2, 179, 201}},
    {{60, X3, 1, 27}, {65, X2, 28, 50}, {70, X3, 51, 77}, {75, X2, 78, 100},
        {80, X3, 101, 127}, {85, X2, 128, 150}, {90, X1, 151, 178}, {95, X2, 179, 201}},
    {{100, X3, 1, 27}, {105, X2, 28, 50}, {110, X3, 51, 77}, {115, X2, 78, 100},
        {120, X3, 101, 127}, {125, X2, 128, 150}, {130, X4, 151, 178}, {135, X2, 179, 201}},
    {{140, X3, 1, 27}, {145, X2, 28, 50}, {150, X3, 51, 77}, {155, X2, 78, 100},
        {160, X3, 101, 127}, {165, X2, 128, 150}, {170, X3, 151, 177}, {175, X2, 178, 200}}};
const sbsigpband_t IGPBAND2[2][5] = {/* band 9-10 */
    {{60, X5, 1, 72}, {65, X6, 73, 108}, {70, X6, 109, 144}, {75, X6, 145, 180},
        {85, X7, 181, 192}},
    {{-60, X5, 1, 72}, {-65, X6, 73, 108}, {-70, X6, 109, 144}, {-75, X6, 145, 180},
        {-85, X8, 181, 192}}};


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


#endif  // GNSS_SDR_RTKLIB_SBAS_H
