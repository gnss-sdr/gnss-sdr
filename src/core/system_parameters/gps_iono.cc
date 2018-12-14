/*!
 * \file gps_iono.cc
 * \brief  Interface of a GPS IONOSPHERIC MODEL storage
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#include "gps_iono.h"

Gps_Iono::Gps_Iono()
{
    valid = false;
    d_alpha0 = 0.0;
    d_alpha1 = 0.0;
    d_alpha2 = 0.0;
    d_alpha3 = 0.0;
    d_beta0 = 0.0;
    d_beta1 = 0.0;
    d_beta2 = 0.0;
    d_beta3 = 0.0;
}
