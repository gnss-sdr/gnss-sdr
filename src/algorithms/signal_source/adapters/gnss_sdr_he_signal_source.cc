/*!
 * \file gnss_sdr_he_signal_source.cc
 * \brief GNSS-SDR Hacker's Edition Signal Sampler Board Driver.
 * \author Ajith Peter, Google Summer of Code 2014-15, ajith.peter(at)gmail.com
 *         Javier Arribas, 2014-15 jarribas(at)cttc.es
 *         Carles Fernandez Prades, 2014 carles.fernandez (at) cttc.cat
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

#include "gnss_sdr_he_signal_source.h"
#include <gnss_sdr/gnss_sdr_source_b.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/msg_queue.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "configuration_interface.h"

using google::LogMessage;

GNSS_SDR_HE_SignalSource::GNSS_SDR_HE_SignalSource(
        ConfigurationInterface* configuration,
        std::string role, 
        unsigned int in_stream, 
        unsigned int out_stream, 
        gr::msg_queue::sptr queue) :
        role_(role), 
        in_stream_(in_stream), 
        out_stream_(out_stream), 
        queue_(queue)
  {
      std::string default_item_type = "byte";
      std::string default_dump_file = "../data/gnss_sdr_he_capture.dat";
      item_type_ = configuration->property(role + ".item_type", default_item_type);
      dump_ = configuration->property(role + ".dump", false);
      dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

      if (item_type_.compare("byte") == 0)
        {
            item_size_ = sizeof(char);
        }
      else
        {
            LOG(WARNING) << item_type_  << " unrecognized item type. Using byte.";
            item_size_ = sizeof(char);
        }

      try
        {
            gnss_sdr_source_b_ = gr::gnss_sdr::gnss_sdr_source_b::make();
            unpack_byte_ = make_unpack_byte_2bit_cpx_samples();
            inter_shorts_to_cpx_ =  gr::blocks::interleaved_short_to_complex::make(false,true); //I/Q swap enabled
        }
      catch (const std::exception &e)
        {
            std::cerr
            << "Error in configuring receiver flowgraph.  Please check receiver \
            connection before retrying...\n" << std::endl;

            LOG(WARNING) << "gnss_sdr_source_b could not start. exitting program.";

            throw(e);
        }

      if (dump_)
        {
            sink_ = gr::blocks::file_sink::make(sizeof(gr_complex), dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << sink_->unique_id() << ")";
        }
  }
GNSS_SDR_HE_SignalSource::~GNSS_SDR_HE_SignalSource()
{
}

void GNSS_SDR_HE_SignalSource::connect(gr::top_block_sptr top_block)
{
    top_block->connect(gnss_sdr_source_b_, 0, unpack_byte_, 0);
    top_block->connect(unpack_byte_, 0,inter_shorts_to_cpx_,0);
    if (dump_)
    {
      top_block->connect(inter_shorts_to_cpx_, 0, sink_, 0);
    }
    
    DLOG(INFO) << "connected gnss-sdr hackers edition";

}

void GNSS_SDR_HE_SignalSource::disconnect(gr::top_block_sptr top_block)
{
    top_block->disconnect(gnss_sdr_source_b_, 0, unpack_byte_, 0);
    top_block->disconnect(unpack_byte_, 0,inter_shorts_to_cpx_,0);
    if (dump_)
    {
      top_block->disconnect(inter_shorts_to_cpx_, 0, sink_, 0);
    }
    
    DLOG(INFO) << "disconnected gnss-sdr hackers edition";

}

gr::basic_block_sptr GNSS_SDR_HE_SignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::block_sptr();
}


gr::basic_block_sptr GNSS_SDR_HE_SignalSource::get_right_block()
{
    return unpack_byte_;
}
