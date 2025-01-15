/*!
 * \file rtklib_rtcm.h
 * \brief RTCM functions headers
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
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_RTKLIB_RTCM_H
#define GNSS_SDR_RTKLIB_RTCM_H

#include "rtklib.h"
#include "rtklib_rtcm2.h"
#include "rtklib_rtcm3.h"

#define RTCM2PREAMB 0x66 /* rtcm ver.2 frame preamble */
#define RTCM3PREAMB 0xD3 /* rtcm ver.3 frame preamble */


int init_rtcm(rtcm_t *rtcm);
void free_rtcm(rtcm_t *rtcm);
int input_rtcm2(rtcm_t *rtcm, unsigned char data);
int input_rtcm3(rtcm_t *rtcm, unsigned char data);
int input_rtcm2f(rtcm_t *rtcm, FILE *fp);
int input_rtcm3f(rtcm_t *rtcm, FILE *fp);
int gen_rtcm2(rtcm_t *rtcm, int type, int sync);
// int gen_rtcm3(rtcm_t *rtcm, int type, int sync);


#endif  // GNSS_SDR_RTKLIB_RTCM_H
