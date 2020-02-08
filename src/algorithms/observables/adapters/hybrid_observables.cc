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

    observables_ = hybrid_observables_gs_make(in_streams_, out_streams_, dump_, dump_mat_, dump_filename_);
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
