/*!
 * \file beidou_dnav_utc_model.cc
 * \brief  Interface of a BeiDou UTC Model storage
 * \author Damian Miralles, 2018. dmiralles2009(at)gmail.com
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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

#include "beidou_dnav_utc_model.h"


Beidou_Dnav_Utc_Model::Beidou_Dnav_Utc_Model()
{
    valid = false;
    d_A1_UTC = 0;
    d_A0_UTC = 0;
    d_DeltaT_LS = 0;
    i_WN_LSF = 0;
    i_DN = 0;
    d_DeltaT_LSF = 0;

    d_A0_GPS = 0;
    d_A1_GPS = 0;
    d_A0_GAL = 0;
    d_A1_GAL = 0;
    d_A0_GLO = 0;
    d_A1_GLO = 0;
}
