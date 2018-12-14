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

#ifndef GNSS_SDR_RTKLIB_RTCM2_H_
#define GNSS_SDR_RTKLIB_RTCM2_H_

#include "rtklib.h"


void adjhour(rtcm_t *rtcm, double zcnt);
int obsindex(obs_t *obs, gtime_t time, int sat);
int decode_type1(rtcm_t *rtcm);
int decode_type3(rtcm_t *rtcm);
int decode_type14(rtcm_t *rtcm);
int decode_type16(rtcm_t *rtcm);
int decode_type17(rtcm_t *rtcm);
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
