/*!
 * \file sbas_l1_pcps_acquisition.h
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


#ifndef GNSS_SDR_SBAS_L1_PCPS_ACQUISITION_H
#define GNSS_SDR_SBAS_L1_PCPS_ACQUISITION_H

#include "pcps_acquisition_adapter.h"
#include <string>

class ConfigurationInterface;

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_adapters
 * \{ */

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 *  for SBAS L1 signals (PRN 120–138, same chip rate and code length as GPS L1 C/A).
 */
class SbasL1PcpsAcquisition : public PcpsAcquisitionAdapter
{
public:
    SbasL1PcpsAcquisition(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~SbasL1PcpsAcquisition() = default;
};


/** \} */
/** \} */

#endif  // GNSS_SDR_SBAS_L1_PCPS_ACQUISITION_H
