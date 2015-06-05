/*!
 * \file mixed_observables.h
 * \brief Implementation of an adapter of a mixed observables block (muti-frequency and multi-system)
 * to a ObservablesInterface
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_MIXED_OBSERVABLES_H_
#define GNSS_SDR_MIXED_OBSERVABLES_H_

#include <string>
#include <gnuradio/msg_queue.h>
#include "observables_interface.h"
#include "mixed_observables_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements an ObservablesInterface for mixed observables block (muti-frequency and multi-system)
 */
class MixedObservables : public ObservablesInterface
{
public:
    MixedObservables(ConfigurationInterface* configuration,
                       std::string role,
                       unsigned int in_streams,
                       unsigned int out_streams,
                       boost::shared_ptr<gr::msg_queue> queue);
    virtual ~MixedObservables();
    std::string role()
    {
        return role_;
    }

    //!  Returns "Mixed_Observables"
    std::string implementation()
    {
        return "Mixed_Observables";
    }
    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();
    void reset()
    {
        return;
    }

    //! All blocks must have an item_size() function implementation
    size_t item_size()
    {
        return sizeof(gr_complex);
    }

private:
    mixed_observables_cc_sptr observables_;
    bool dump_;
    //unsigned int fs_in_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif
