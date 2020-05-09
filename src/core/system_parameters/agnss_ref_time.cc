/*!
 * \file agnss_ref_time.cc
 * \brief  Interface of an Assisted GNSS REFERENCE TIME storage
 *
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#include "agnss_ref_time.h"

Agnss_Ref_Time::Agnss_Ref_Time()
{
    valid = false;
    d_TOW = 0.0;
    d_Week = 0.0;
    d_tv_sec = 0.0;
    d_tv_usec = 0.0;
}
