/*!
 * \file beidou_cnav2_iono.cc
 * \brief  Interface of a BeiDou CNAV2 Ionospheric Model storage
 * \author Damian Miralles, 2018. dmiralles2009(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#include "beidou_cnav2_iono.h"

Beidou_Cnav2_Iono::Beidou_Cnav2_Iono()
{
    valid = false;
    alpha1 = 0.0;
    alpha2 = 0.0;
    alpha3 = 0.0;
    alpha4 = 0.0;
	alpha5 = 0.0;
	alpha6 = 0.0;
	alpha7 = 0.0;
	alpha8 = 0.0;
	alpha9 = 0.0;

    d_TOW_30 = 0.0;
    d_WN_30 = 0.0;
}