/*!
 * \file sbas_l1_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  SBAS L1 signals
 * \author Miguel Gómez López, 2026. mgomezl(at)ing.uc3m.es
 * \author Víctor Castillo Agüero, 2026. victorcastilloaguero(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "sbas_l1_pcps_acquisition.h"


SbasL1PcpsAcquisition::SbasL1PcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : PcpsAcquisitionAdapter(configuration, role, "SBAS_L1_PCPS_Acquisition", in_streams, out_streams, SBAS_S1)
{
}
