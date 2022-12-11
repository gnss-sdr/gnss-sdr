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
    Tlm_Conf() = default;

    void SetFromConfiguration(const ConfigurationInterface *configuration, const std::string &role);

    std::string dump_filename;
    std::string dump_crc_stats_filename;
    bool dump{false};
    bool dump_mat{false};
    bool remove_dat{false};
    bool enable_reed_solomon{false};  // for INAV message in Galileo E1B
    bool dump_crc_stats{false};       // telemetry CRC statistics
    bool enable_navdata_monitor{false};
    bool there_are_e6_channels{false};
    int32_t ecc_errors_reject{1};
    int32_t ecc_errors_resync{6};
    uint32_t validator_thr{2};
    bool validator_accept_first{true};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_TLM_CONF_H
