/*!
 * \file gps_acq_assist.cc
  * \brief  Interface of a GPS RRLL ACQUISITION ASSISTACE storage
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#include "gps_acq_assist.h"

Gps_Acq_Assist::Gps_Acq_Assist()
{
    i_satellite_PRN = 0;
    d_TOW = 0.0;
    d_Doppler0 = 0.0;
    d_Doppler1 = 0.0;
    dopplerUncertainty = 0.0;
    Code_Phase = 0.0;
    Code_Phase_int = 0.0;
    GPS_Bit_Number = 0.0;
    Code_Phase_window = 0.0;
    Azimuth = 0.0;
    Elevation = 0.0;
}
