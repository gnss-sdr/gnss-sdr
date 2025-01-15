/*
 * \file sbas_ephemeris.cc
 * \brief  Implementation of a SBAS REFERENCE LOCATION storage
 *
 * \author Daniel Fehr, 2013. daniel.co(at)bluewin.ch
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
