/*!
 * \file tlm_conf.h
 * \brief Class that contains all the configuration parameters for generic
 * telemetry decoder block.
 * \author Carles Fernandez, 2020. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_TLM_CONF_H
#define GNSS_SDR_TLM_CONF_H

#include "configuration_interface.h"
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_libs
 * \{ */


class Tlm_Conf
{
public:
    Tlm_Conf();

    void SetFromConfiguration(const ConfigurationInterface *configuration, const std::string &role);

    std::string dump_filename;
    bool dump;
    bool dump_mat;
    bool remove_dat;
    bool enable_reed_solomon;  // for INAV message in Galileo E1B
};


/** \} */
/** \} */
#endif  // GNSS_SDR_TLM_CONF_H
