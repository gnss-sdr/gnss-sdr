/*!
 * \file obs_conf.cc
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

#include "obs_conf.h"
#include "gnss_sdr_flags.h"

#if USE_GLOG_AND_GFLAGS
Obs_Conf::Obs_Conf() : smoothing_factor(FLAGS_carrier_smoothing_factor)
{
}
#else
Obs_Conf::Obs_Conf() : smoothing_factor(absl::GetFlag(FLAGS_carrier_smoothing_factor))
{
}
#endif