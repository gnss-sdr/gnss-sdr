/*
 * \file glonass_gnav_utc_model.h
 * \brief  Interface of a GLONASS GNAV UTC MODEL storage
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "glonass_gnav_utc_model.h"
#include <cmath>

Glonass_Gnav_Utc_Model::Glonass_Gnav_Utc_Model()
{
    valid = false;
    d_tau_c = 0.0;
    d_tau_gps = 0.0;
    d_N_4 = 0.0;
    d_N_A = 0.0;
    d_B1 = 0.0;
    d_B2 = 0.0;
}

double Glonass_Gnav_Utc_Model::utc_time(double glonass_time_corrected)
{
    double t_utc;

    // GLONASS Time is relative to UTC Moscow, so we simply add its time difference
    t_utc = glonass_time_corrected + 3*3600 + d_tau_c;

    return t_utc;
}
