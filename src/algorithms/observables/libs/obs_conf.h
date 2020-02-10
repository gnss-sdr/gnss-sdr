/*!
 * \file obs_conf.h
 * \brief Class that contains all the configuration parameters for generic
 * observables block
 * \author Javier Arribas, 2020. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_OBS_CONF_H
#define GNSS_SDR_OBS_CONF_H

#include <cstdint>
#include <string>

class Obs_Conf
{
public:
    bool enable_carrier_smoothing;
    double smoothing_factor;
    uint32_t nchannels_in;
    uint32_t nchannels_out;
    bool dump;
    bool dump_mat;
    std::string dump_filename;

    Obs_Conf();
};

#endif
