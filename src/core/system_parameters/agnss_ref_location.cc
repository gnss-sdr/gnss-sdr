/*!
 * \file agnss_ref_location.cc
 * \brief  Interface of an Assisted GNSS REFERENCE LOCATION storage
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

#include "agnss_ref_location.h"

Agnss_Ref_Location::Agnss_Ref_Location()
{
    valid = false;
    lat = 0.0;
    lon = 0.0;
    uncertainty = 0.0;
}
