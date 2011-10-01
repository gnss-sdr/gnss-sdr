/*!
 * \file usrp1_signal_source.cc
 * \brief Brief description of the file here
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

#include "usrp1_signal_source.h"

#include <gnuradio/usrp_source_s.h>
#include <gnuradio/usrp_source_c.h>
#include <gnuradio/gr_file_sink.h>

#include "configuration_interface.h"
#include "gnss_sdr_valve.h"

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

Usrp1SignalSource::Usrp1SignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream, unsigned int out_stream,
        gr_msg_queue_sptr queue) :
    role_(role), in_stream_(in_stream), out_stream_(out_stream),
            queue_(queue)
{

    std::string empty = "";
    std::string default_dump_file = "./data/signal_source.dat";
    std::string default_item_type = "short";

    which_board_ = configuration->property(role + ".which_board", 0);
    decim_rate_ = configuration->property(role + ".decim_rate", 16);
    nchan_ = configuration->property(role + ".nchan", 1);
    mux_ = configuration->property(role + ".mux", -1);
    mode_ = configuration->property(role + ".mode", 0);
    fusb_block_size_ = configuration->property(role + ".fusb_block_size", 0);
    fusb_nblocks_ = configuration->property(role + ".fusb_nblocks", 0);
    fpga_filename_ = configuration->property(role + ".fpga_filename", empty);
    firmware_filename_ = configuration->property(role + ".firmware_filename",
            empty);
    spec_side_ = configuration->property(role + ".spec_side", 0);
    spec_subdev_ = configuration->property(role + ".spec_subdev", 0);
    freq_ = configuration->property(role + ".freq", (double)1.57542e9);
    gain_ = configuration->property(role + ".gain", (float)40.0);
    item_type_ = configuration->property(role + ".item_type",
            default_item_type);
    samples_ = configuration->property(role + ".samples", 0);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename",
            default_dump_file);

    if (item_type_.compare("short") == 0)
    {
        item_size_ = sizeof(short);
        usrp_source_ = usrp_make_source_s(which_board_, decim_rate_, nchan_,
                mux_, mode_, fusb_block_size_, fusb_nblocks_, fpga_filename_,
                firmware_filename_);
    }
    else if (item_type_.compare("gr_complex") == 0)
    {
        item_size_ = sizeof(gr_complex);
        usrp_source_ = usrp_make_source_c(which_board_, decim_rate_, nchan_,
                mux_, mode_, fusb_block_size_, fusb_nblocks_, fpga_filename_,
                firmware_filename_);
    }
    else
    {
        LOG_AT_LEVEL(WARNING) << item_type_
                << " unrecognized item type. Using short.";
        item_size_ = sizeof(short);
        usrp_source_ = usrp_make_source_s(which_board_, decim_rate_, nchan_,
                mux_, mode_, fusb_block_size_, fusb_nblocks_, fpga_filename_,
                firmware_filename_);
    }

    DLOG(INFO) << "usrp_source(" << usrp_source_->unique_id() << ")";

    if (samples_ != 0)
    {
        DLOG(INFO) << "Send STOP signal after " << samples_ << " samples";
        valve_ = gnss_sdr_make_valve(item_size_, samples_, queue_);
        DLOG(INFO) << "valve(" << valve_->unique_id() << ")";
    }

    if (dump_)
    {
        DLOG(INFO) << "Dumping output into file " << dump_filename_;
        file_sink_ = gr_make_file_sink(item_size_, dump_filename_.c_str());
        DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
    }

    DLOG(INFO) << "Item size " << item_size_;

    db_base_sptr subdev = usrp_source_->selected_subdev(usrp_subdev_spec(
            spec_side_, spec_subdev_));

    DLOG(INFO) << "Subdevice name is " << subdev->side_and_name();
    DLOG(INFO) << "Subdevice frequency ranges from " << subdev->freq_min()
            << " to " << subdev->freq_max();

    mux_ = usrp_source_->determine_rx_mux_value(usrp_subdev_spec(spec_side_,
            spec_subdev_));

    DLOG(INFO) << "Mux is " << mux_;

    usrp_source_->set_mux(mux_);

    if (gain_ == -1)
    {
        gain_ = (subdev->gain_min() + subdev->gain_max()) / 2.0;
    }
    DLOG(INFO) << "Gain is " << gain_;
    subdev->set_gain(gain_);

    usrp_tune_result r;

    bool ok = usrp_source_->tune(0, subdev, freq_, &r); //DDC 0
    if (!ok)
    {
        LOG_AT_LEVEL(FATAL) << "Could not set frequency " << freq_
                << " for USRP";
    }
    DLOG(INFO) << "Frequency set to " << freq_;
    DLOG(INFO) << "Decimation set to " << decim_rate_;
}

Usrp1SignalSource::~Usrp1SignalSource()
{
}

void Usrp1SignalSource::connect(gr_top_block_sptr top_block)
{

    if (samples_ != 0)
    {
        top_block->connect(usrp_source_, 0, valve_, 0);
        DLOG(INFO) << "connected usrp source to valve";

        if (dump_)
        {
            top_block->connect(valve_, 0, file_sink_, 0);
            DLOG(INFO) << "connected valve to file sink";
        }
    }
    else
    {
        if (dump_)
        {
            top_block->connect(usrp_source_, 0, file_sink_, 0);
            DLOG(INFO) << "connected usrp source to file sink";
        }
    }
}

void Usrp1SignalSource::disconnect(gr_top_block_sptr top_block)
{

    if (samples_ != 0)
    {
        top_block->disconnect(usrp_source_, 0, valve_, 0);

        if (dump_)
        {
            top_block->disconnect(valve_, 0, file_sink_, 0);
        }
    }
    else
    {
        if (dump_)
        {
            top_block->disconnect(usrp_source_, 0, file_sink_, 0);
        }
    }
}

gr_basic_block_sptr Usrp1SignalSource::get_left_block()
{
    LOG_AT_LEVEL(WARNING) << "Trying to get signal source left block.";
    return gr_basic_block_sptr();
}

gr_basic_block_sptr Usrp1SignalSource::get_right_block()
{
    if (samples_ != 0)
    {
        return valve_;
    }
    else
    {
        return usrp_source_;
    }
}
