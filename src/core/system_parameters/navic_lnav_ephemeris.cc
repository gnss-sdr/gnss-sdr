/*!
 * \file navic_lnav_ephemeris.cc
 * \brief  Interface of a NavIC (IRNSS) LNAV EPHEMERIS storage and orbital model functions
 * \author Pradyumna Byppanahalli Suresha, 2025.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "navic_lnav_ephemeris.h"

Navic_Lnav_Ephemeris::Navic_Lnav_Ephemeris()
{
    this->System = 'I';
}
