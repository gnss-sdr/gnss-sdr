/*!
 * \file rtklib_rtcm2.h
 * \brief RTCM v2 functions headers
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

#ifndef GNSS_SDR_RTKLIB_RTCM2_H
#define GNSS_SDR_RTKLIB_RTCM2_H

#include "rtklib.h"


void adjhour(rtcm_t *rtcm, double zcnt);
int obsindex(obs_t *obs, gtime_t time, int sat);
int decode_type1(rtcm_t *rtcm);
int decode_type3(rtcm_t *rtcm);
int decode_type14(rtcm_t *rtcm, bool pre_2009_file = false);
int decode_type16(rtcm_t *rtcm);
int decode_type17(rtcm_t *rtcm, bool pre_2009_file = false);
int decode_type18(rtcm_t *rtcm);
int decode_type19(rtcm_t *rtcm);
int decode_type22(rtcm_t *rtcm);
int decode_type23(rtcm_t *rtcm);
int decode_type24(rtcm_t *rtcm);
int decode_type31(rtcm_t *rtcm);
int decode_type32(rtcm_t *rtcm);
int decode_type34(rtcm_t *rtcm);
int decode_type36(rtcm_t *rtcm);
int decode_type37(rtcm_t *rtcm);
int decode_type59(rtcm_t *rtcm);
int decode_rtcm2(rtcm_t *rtcm);

#endif
