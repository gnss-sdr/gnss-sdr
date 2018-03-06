/*
 * \file sbas_ephemeris.cc
 * \brief  Implementation of a SBAS REFERENCE LOCATION storage
 *
 * \author Daniel Fehr, 2013. daniel.co(at)bluewin.ch
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


#include "sbas_ephemeris.h"

void Sbas_Ephemeris::print(std::ostream &out)
{
    out << "<<E>> PRN" << i_prn << ":";
    out << "  d_t0=" << i_t0;
    out << "  d_tof=" << d_tof;
    out << "  i_sv_ura=" << i_sv_ura;
    out << "  b_sv_do_not_use=" << b_sv_do_not_use;
    out << "  d_pos=(x=" << d_pos[0] << ", y=" << d_pos[1] << ", z=" << d_pos[2] << ")";
    out << "  d_vel=(x=" << d_vel[0] << ", y=" << d_vel[1] << ", z=" << d_vel[2] << ")";
    out << "  d_acc=(x=" << d_acc[0] << ", y=" << d_acc[1] << ", z=" << d_acc[2] << ")";
    out << "  d_af0=" << d_af0;
    out << "  d_af1=" << d_af1;
}
