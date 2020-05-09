/*!
 * \file hybrid_observables.cc
 * \brief Implementation of an adapter of a Galileo E1 observables block
 * to a ObservablesInterface
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#include "hybrid_observables.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "obs_conf.h"
#include <glog/logging.h>
#include <ostream>  // for operator<<

HybridObservables::HybridObservables(ConfigurationInterface* configuration,
    const std::string& role, unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    std::string default_dump_filename = "./observables.dat";
    DLOG(INFO) << "role " << role;
    dump_ = configuration->property(role + ".dump", false);
    dump_mat_ = configuration->property(role + ".dump_mat", true);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);

    Obs_Conf conf;

    conf.dump = dump_;
    conf.dump_mat = dump_mat_;
    conf.dump_filename = dump_filename_;
    conf.nchannels_in = in_streams_;
    conf.nchannels_out = out_streams_;
    conf.enable_carrier_smoothing = configuration->property(role + ".enable_carrier_smoothing", conf.enable_carrier_smoothing);

    if (FLAGS_carrier_smoothing_factor == DEFAULT_CARRIER_SMOOTHING_FACTOR)
        {
            conf.smoothing_factor = configuration->property(role + ".smoothing_factor", conf.smoothing_factor);
        }

    if (conf.enable_carrier_smoothing == true)
        {
            LOG(INFO) << "Observables carrier smoothing enabled with smoothing factor " << conf.smoothing_factor;
        }
    observables_ = hybrid_observables_gs_make(conf);
    DLOG(INFO) << "Observables block ID (" << observables_->unique_id() << ")";
}


void HybridObservables::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void HybridObservables::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr HybridObservables::get_left_block()
{
    return observables_;
}


gr::basic_block_sptr HybridObservables::get_right_block()
{
    return observables_;
}
