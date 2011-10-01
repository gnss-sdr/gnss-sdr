/*!
 * \file usrp1_signal_source.h
 * \brief This class represents a USRP signal source
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef USRP1_SIGNAL_SOURCE_H_
#define USRP1_SIGNAL_SOURCE_H_

#include <boost/shared_ptr.hpp>

#include <gnuradio/gr_hier_block2.h>
#include <gnuradio/gr_msg_queue.h>

#include "gnss_block_interface.h"

class ConfigurationInterface;
class usrp_source_base;
typedef boost::shared_ptr<usrp_source_base> usrp_source_base_sptr;

class Usrp1SignalSource: public GNSSBlockInterface
{

public:
    Usrp1SignalSource(ConfigurationInterface* configuration,
            std::string role, unsigned int in_stream,
            unsigned int out_stream, gr_msg_queue_sptr queue);

    virtual ~Usrp1SignalSource();

    std::string role()
    {
        return role_;
    }
    std::string implementation()
    {
        return "Usrp1SignalSource";
    }
    size_t item_size()
    {
        return item_size_;
    }

    void connect(gr_top_block_sptr top_block);
    void disconnect(gr_top_block_sptr top_block);
    gr_basic_block_sptr get_left_block();
    gr_basic_block_sptr get_right_block();

private:

    std::string role_;
    unsigned int in_stream_;
    unsigned int out_stream_;

    int which_board_;
    unsigned int decim_rate_;
    int nchan_;
    int mux_;
    int mode_;
    int fusb_block_size_;
    int fusb_nblocks_;
    std::string fpga_filename_;
    std::string firmware_filename_;
    unsigned int spec_side_;
    unsigned int spec_subdev_;
    double freq_;
    float gain_;
    std::string item_type_;
    size_t item_size_;
    long samples_;
    bool dump_;
    std::string dump_filename_;

    usrp_source_base_sptr usrp_source_;
    gr_block_sptr valve_;
    gr_block_sptr file_sink_;
    gr_msg_queue_sptr queue_;
};

#endif /*USRP1_SIGNAL_SOURCE_H_*/
