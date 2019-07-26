/*!
 * \file flexiband_signal_source.cc
 * \brief ignal Source adapter for the Teleorbit Flexiband front-end device.
 * This adapter requires a Flexiband GNU Radio driver
 * installed (not included with GNSS-SDR)
 * \author Javier Arribas, jarribas(at)cttc.es
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "flexiband_signal_source.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <gnuradio/blocks/file_sink.h>
#include <teleorbit/frontend.h>
#include <utility>


FlexibandSignalSource::FlexibandSignalSource(ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_stream,
    unsigned int out_stream,
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue) : role_(role),
                                                           in_stream_(in_stream),
                                                           out_stream_(out_stream),
                                                           queue_(std::move(queue))
{
    std::string default_item_type = "byte";
    item_type_ = configuration->property(role + ".item_type", default_item_type);

    std::string default_firmware_file = "flexiband_I-1b.bit";
    firmware_filename_ = configuration->property(role + ".firmware_file", default_firmware_file);

    gain1_ = configuration->property(role + ".gain1", 0);  // check gain DAC values for Flexiband frontend!
    gain2_ = configuration->property(role + ".gain2", 0);  // check gain DAC values for Flexiband frontend!
    gain3_ = configuration->property(role + ".gain3", 0);  // check gain DAC values for Flexiband frontend!

    AGC_ = configuration->property(role + ".AGC", true);                        // enabled AGC by default
    flag_read_file = configuration->property(role + ".flag_read_file", false);  //disable read samples from file by default
    std::string default_signal_file = "flexiband_frame_samples.bin";
    signal_file = configuration->property(role + ".signal_file", default_signal_file);

    usb_packet_buffer_size_ = configuration->property(role + ".usb_packet_buffer", 128);

    n_channels_ = configuration->property(role + ".total_channels", 0);
    if (n_channels_ == 0)
        {
            n_channels_ = configuration->property(role + ".RF_channels", 1);
        }
    sel_ch_ = configuration->property(role + ".sel_ch", 1);
    if (sel_ch_ > n_channels_)
        {
            LOG(WARNING) << "Invalid RF channel selection";
        }
    if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            flexiband_source_ = gr::teleorbit::frontend::make(firmware_filename_.c_str(), gain1_, gain2_, gain3_, AGC_, usb_packet_buffer_size_, signal_file.c_str(), flag_read_file);

            //create I, Q -> gr_complex type conversion blocks
            for (int n = 0; n < (n_channels_ * 2); n++)
                {
                    char_to_float.push_back(gr::blocks::char_to_float::make());
                }

            for (int n = 0; n < n_channels_; n++)
                {
                    float_to_complex_.push_back(gr::blocks::float_to_complex::make());
                    null_sinks_.push_back(gr::blocks::null_sink::make(sizeof(gr_complex)));
                }

            DLOG(INFO) << "Item size " << item_size_;
            DLOG(INFO) << "Firmware file " << firmware_filename_;
            DLOG(INFO) << "flexiband_source_(" << flexiband_source_->unique_id() << ")";
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type for flexiband_source_";
            item_size_ = sizeof(gr_complex);
        }
    if (in_stream_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void FlexibandSignalSource::connect(gr::top_block_sptr top_block)
{
    for (int n = 0; n < (n_channels_ * 2); n++)
        {
            top_block->connect(flexiband_source_, n, char_to_float.at(n), 0);
            DLOG(INFO) << "connected flexiband_source_ to char_to_float CH" << n;
        }
    for (int n = 0; n < n_channels_; n++)
        {
            top_block->connect(char_to_float.at(n * 2), 0, float_to_complex_.at(n), 0);
            top_block->connect(char_to_float.at(n * 2 + 1), 0, float_to_complex_.at(n), 1);
            top_block->connect(float_to_complex_.at(n), 0, null_sinks_.at(n), 0);
            DLOG(INFO) << "connected char_to_float to float_to_complex_ CH" << n;
        }
}


void FlexibandSignalSource::disconnect(gr::top_block_sptr top_block)
{
    for (int n = 0; n < (n_channels_ * 2); n++)
        {
            top_block->disconnect(flexiband_source_, n, char_to_float.at(n), 0);
            DLOG(INFO) << "disconnect flexiband_source_ to char_to_float CH" << n;
        }
    for (int n = 0; n < n_channels_; n++)
        {
            top_block->disconnect(char_to_float.at(n * 2), 0, float_to_complex_.at(n), 0);
            top_block->disconnect(char_to_float.at(n * 2 + 1), 0, float_to_complex_.at(n), 1);
            top_block->disconnect(float_to_complex_.at(n), 0, null_sinks_.at(n), 0);
            DLOG(INFO) << "disconnect char_to_float to float_to_complex_ CH" << n;
        }
}


gr::basic_block_sptr FlexibandSignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::block_sptr();
}


gr::basic_block_sptr FlexibandSignalSource::get_right_block()
{
    return get_right_block(0);
}

gr::basic_block_sptr FlexibandSignalSource::get_right_block(int RF_channel)
{
    if (RF_channel == 0)
        {
            //in the first RF channel, return the signalsource selected channel.
            //this trick enables the use of the second or the third frequency of a FlexiBand signal without a dual frequency configuration
            return float_to_complex_.at(sel_ch_ - 1);
        }
    else
        {
            return float_to_complex_.at(RF_channel);
        }
}
