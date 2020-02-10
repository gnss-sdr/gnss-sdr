/*!
 * \file obs_conf.cc
 * \brief Class that contains all the configuration parameters for generic
 * observables block
 * \author Javier Arribas, 2020. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#include "obs_conf.h"

Obs_Conf::Obs_Conf()
{
    enable_carrier_smoothing = false;
    smoothing_factor = 200;
    nchannels_in = 0;
    nchannels_out = 0;
    dump = false;
    dump_mat = false;
    dump_filename = "obs_dump.dat";
}
