/*!
 * \file tlm_consistency_checks_conf.cc
 * \brief Class that contains all the configuration parameters for spoofing detection techniques that are a part of PVT block
 * \author Harshad Sathaye sathaye.h(at)northeastern.edu
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "spoofing_detector_conf.h"

TLMConsistencyChecksConf::TLMConsistencyChecksConf()
{
    // ####### Telemetry consistency check variables
    check_TOW = false;
    check_RX_clock = false;
}
