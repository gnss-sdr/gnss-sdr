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
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
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
