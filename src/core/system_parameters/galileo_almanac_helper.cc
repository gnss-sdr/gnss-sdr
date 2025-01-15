/*!
 * \file galileo_almanac_helper.cc
 * \brief  Implementation of a Galileo ALMANAC storage helper
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#include "galileo_almanac_helper.h"

Galileo_Almanac Galileo_Almanac_Helper::get_almanac(int i) const
{
    Galileo_Almanac galileo_almanac;
    const double sqrtAnominal = 5440.588203494;  // square root of Galileo nominal orbit semi-major axis
    switch (i)
        {
        case 1:
            galileo_almanac.PRN = this->SVID1_7;
            galileo_almanac.toa = this->t0a_7;
            galileo_almanac.WNa = this->WN_a_7;
            galileo_almanac.IODa = this->IOD_a_7;
            galileo_almanac.delta_i = this->delta_i_7;
            galileo_almanac.M_0 = this->M0_7;
            galileo_almanac.ecc = this->e_7;
            galileo_almanac.sqrtA = sqrtAnominal + this->DELTA_A_7;
            galileo_almanac.OMEGA_0 = this->Omega0_7;
            galileo_almanac.omega = this->omega_7;
            galileo_almanac.OMEGAdot = this->Omega_dot_7;
            galileo_almanac.af0 = this->af0_8;
            galileo_almanac.af1 = this->af1_8;
            galileo_almanac.E5b_HS = this->E5b_HS_8;
            galileo_almanac.E1B_HS = this->E1B_HS_8;
            galileo_almanac.E5a_HS = this->E5a_HS_8;
            break;

        case 2:
            galileo_almanac.PRN = this->SVID2_8;
            galileo_almanac.toa = this->t0a_9;
            galileo_almanac.WNa = this->WN_a_9;
            galileo_almanac.IODa = this->IOD_a_9;
            galileo_almanac.delta_i = this->delta_i_8;
            galileo_almanac.M_0 = this->M0_9;
            galileo_almanac.ecc = this->e_8;
            galileo_almanac.sqrtA = sqrtAnominal + this->DELTA_A_8;
            galileo_almanac.OMEGA_0 = this->Omega0_8;
            galileo_almanac.omega = this->omega_8;
            galileo_almanac.OMEGAdot = this->Omega_dot_8;
            galileo_almanac.af0 = this->af0_9;
            galileo_almanac.af1 = this->af1_9;
            galileo_almanac.E1B_HS = this->E1B_HS_9;
            galileo_almanac.E5a_HS = this->E5a_HS_9;
            break;

        case 3:
            galileo_almanac.PRN = this->SVID3_9;
            galileo_almanac.toa = this->t0a_9;
            galileo_almanac.WNa = this->WN_a_9;
            galileo_almanac.IODa = this->IOD_a_10;
            galileo_almanac.delta_i = this->delta_i_9;
            galileo_almanac.M_0 = this->M0_10;
            galileo_almanac.ecc = this->e_9;
            galileo_almanac.sqrtA = sqrtAnominal + this->DELTA_A_9;
            galileo_almanac.OMEGA_0 = this->Omega0_10;
            galileo_almanac.omega = this->omega_9;
            galileo_almanac.OMEGAdot = this->Omega_dot_10;
            galileo_almanac.af0 = this->af0_10;
            galileo_almanac.af1 = this->af1_10;
            galileo_almanac.E5b_HS = this->E5b_HS_10;
            galileo_almanac.E1B_HS = this->E1B_HS_10;
            galileo_almanac.E5a_HS = this->E5a_HS_10;
            break;

        default:
            break;
        }
    return galileo_almanac;
}
