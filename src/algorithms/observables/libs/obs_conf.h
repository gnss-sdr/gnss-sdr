/*!
 * \file obs_conf.h
 * \brief Class that contains all the configuration parameters for generic
 * observables block
 * \author Javier Arribas, 2020. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_OBS_CONF_H
#define GNSS_SDR_OBS_CONF_H

#include <cstdint>
#include <string>

/** \addtogroup Observables
 * \{ */
/** \addtogroup Observables_libs observables_libs
 * Utilities for GNSS observables configuration.
 * \{ */

class Obs_Conf
{
public:
    Obs_Conf();

    std::string dump_filename;
    int32_t smoothing_factor;
    uint32_t nchannels_in;
    uint32_t nchannels_out;
    uint32_t observable_interval_ms;
    bool enable_carrier_smoothing;
    bool dump;
    bool dump_mat;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_OBS_CONF_H
