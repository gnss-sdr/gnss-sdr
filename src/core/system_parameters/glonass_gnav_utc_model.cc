/*
 * \file glonass_gnav_utc_model.h
 * \brief  Interface of a GLONASS GNAV UTC MODEL storage
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "glonass_gnav_utc_model.h"

double Glonass_Gnav_Utc_Model::utc_time(double glonass_time_corrected) const
{
    // GLONASS Time is relative to UTC Moscow, so we simply add its time difference
    double t_utc = glonass_time_corrected + 3.0 * 3600.0 + d_tau_c;

    return t_utc;
}
