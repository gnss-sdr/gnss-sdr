/*!
 * \file two_bit_packed_file_signal_source.cc
 * \brief Interface of a class that reads signals samples from a file. Each
 * sample is two bits, which are packed into bytes or shorts.
 *
 * \author Cillian O'Driscoll, 2015 cillian.odriscoll (at) gmail.com
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

#include "two_bit_packed_file_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_valve.h"
#include <glog/logging.h>
#include <gnuradio/blocks/char_to_float.h>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>


using google::LogMessage;


TwoBitPackedFileSignalSource::TwoBitPackedFileSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_streams, unsigned int out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
                        role_(role), in_streams_(in_streams), out_streams_(out_streams), queue_(queue)
{
    std::string default_filename = "../data/my_capture.dat";
    std::string default_item_type = "byte";
    std::string default_dump_filename = "../data/my_capture_dump.dat";
    std::string default_sample_type = "real";
    double default_seconds_to_skip = 0.0;

    samples_ = configuration->property(role + ".samples", 0L);
    sampling_frequency_ = configuration->property(role + ".sampling_frequency", 0);
    filename_ = configuration->property(role + ".filename", default_filename);

    // override value with commandline flag, if present
    if (FLAGS_signal_source.compare("-") != 0) filename_= FLAGS_signal_source;
    if (FLAGS_s.compare("-") != 0) filename_= FLAGS_s;

    item_type_ = configuration->property(role + ".item_type", default_item_type);
    big_endian_items_ = configuration->property(role + ".big_endian_items", true);
    big_endian_bytes_ = configuration->property(role + ".big_endian_bytes", false);
    sample_type_ = configuration->property(role + ".sample_type", default_sample_type ); // options: "real", "iq", "qi"
    repeat_ = configuration->property(role + ".repeat", false);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    enable_throttle_control_ = configuration->property(role + ".enable_throttle_control", false);
    double seconds_to_skip = configuration->property(role + ".seconds_to_skip", default_seconds_to_skip );
    long bytes_to_skip = 0;

    if (item_type_.compare("byte") == 0)
        {
            item_size_ = sizeof(char);
        }
    else if( item_type_.compare("short") == 0)
        {
            // If we have shorts stored in little endian format, might as
            // well read them in as bytes.
            if( big_endian_items_ )
                {
                    item_size_ = sizeof(short);
                }
            else
                {
                    item_size_ = sizeof(char);
                }
        }
    else
        {
            LOG(WARNING) << item_type_  << " unrecognized item type. Using byte.";
            item_size_ = sizeof(char);
        }

    if( sample_type_.compare("real") == 0 )
        {
            is_complex_ = false;
        }
    else if( sample_type_.compare("iq" ) == 0 )
        {
            is_complex_ = true;
            reverse_interleaving_ = false;
        }
    else if( sample_type_.compare("qi") == 0 )
        {
            is_complex_ = true;
            reverse_interleaving_ = true;
        }
    else
        {
            LOG(WARNING) << sample_type_ << " unrecognized sample type. Assuming: "
                    << ( is_complex_ ? ( reverse_interleaving_ ? "qi" : "iq" ) : "real" );
        }
    try
    {
            file_source_ = gr::blocks::file_source::make(item_size_, filename_.c_str(), repeat_);

            if( seconds_to_skip > 0 )
                {
                    bytes_to_skip = static_cast< long >(
                            seconds_to_skip * sampling_frequency_ / 4 );
                    if( is_complex_ )
                        {
                            bytes_to_skip <<= 1;
                        }
                    file_source_->seek( bytes_to_skip, SEEK_SET );
                }

            unpack_samples_ = make_unpack_2bit_samples( big_endian_bytes_,
                    item_size_, big_endian_items_, reverse_interleaving_);
            if( is_complex_ )
                {
                    char_to_float_ =
                            gr::blocks::interleaved_char_to_complex::make(false);
                }
            else
                {
                    char_to_float_ =
                            gr::blocks::char_to_float::make();
                }

    }
    catch (const std::exception &e)
    {
            std::cerr
            << "The receiver was configured to work with a file signal source "
            << std::endl
            << "but the specified file is unreachable by GNSS-SDR."
            << std::endl
            <<  "Please modify your configuration file"
            << std::endl
            <<  "and point SignalSource.filename to a valid raw data file. Then:"
            << std::endl
            << "$ gnss-sdr --config_file=/path/to/my_GNSS_SDR_configuration.conf"
            << std::endl
            << "Examples of configuration files available at:"
            << std::endl
            << GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/"
            << std::endl;

            LOG(WARNING) << "file_signal_source: Unable to open the samples file "
                    << filename_.c_str() << ", exiting the program.";
            throw(e);
    }

    DLOG(INFO) << "file_source(" << file_source_->unique_id() << ")";

    size_t output_item_size = ( is_complex_ ? sizeof( gr_complex ) : sizeof( float ) );

    if (samples_ == 0) // read all file
        {
            /*!
             * BUG workaround: The GNU Radio file source does not stop the receiver after reaching the End of File.
             * A possible solution is to compute the file length in samples using file size, excluding the last 2 milliseconds, and enable always the
             * valve block
             */
            std::ifstream file (filename_.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
            std::ifstream::pos_type size;

            if (file.is_open())
                {
                    size = file.tellg();
                    samples_ = floor(static_cast<double>(size) * ( is_complex_ ? 2.0 : 4.0 ) );
                    LOG(INFO) << "Total samples in the file= " << samples_; // 4 samples per byte
                    samples_ -= bytes_to_skip;

                    //Also skip the last two milliseconds:
                    samples_ -= ceil( 0.002 * sampling_frequency_ / (is_complex_ ? 2.0 : 4.0 ) );
                }
            else
                {
                    std::cout << "file_signal_source: Unable to open the samples file " << filename_.c_str() << std::endl;
                    LOG(ERROR) << "file_signal_source: Unable to open the samples file " << filename_.c_str();
                }
            std::streamsize ss = std::cout.precision();
            std::cout << std::setprecision(16);
            std::cout << "Processing file " << filename_ << ", which contains " << size << " [bytes]" << std::endl;
            std::cout.precision (ss);
        }

    CHECK(samples_ > 0) << "File does not contain enough samples to process.";
    double signal_duration_s;
    signal_duration_s = static_cast<double>(samples_) * ( 1 /static_cast<double>(sampling_frequency_));
    LOG(INFO) << "Total number samples to be processed= " << samples_ << " GNSS signal duration= " << signal_duration_s << " [s]";
    std::cout << "GNSS signal recorded time to be processed: " << signal_duration_s << " [s]" << std::endl;

    valve_ = gnss_sdr_make_valve(output_item_size, samples_, queue_);
    DLOG(INFO) << "valve(" << valve_->unique_id() << ")";

    if (dump_)
        {
            //sink_ = gr_make_file_sink(item_size_, dump_filename_.c_str());
            sink_ = gr::blocks::file_sink::make(output_item_size, dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << sink_->unique_id() << ")";
        }

    if (enable_throttle_control_)
        {
            throttle_ = gr::blocks::throttle::make(output_item_size, sampling_frequency_);
        }
    DLOG(INFO) << "File source filename " << filename_;
    DLOG(INFO) << "Samples " << samples_;
    DLOG(INFO) << "Sampling frequency " << sampling_frequency_;
    DLOG(INFO) << "Item type " << item_type_;
    DLOG(INFO) << "Item size " << item_size_;
    DLOG(INFO) << "Repeat " << repeat_;
    DLOG(INFO) << "Dump " << dump_;
    DLOG(INFO) << "Dump filename " << dump_filename_;
}


TwoBitPackedFileSignalSource::~TwoBitPackedFileSignalSource()
{}


void TwoBitPackedFileSignalSource::connect(gr::top_block_sptr top_block)
{
    gr::basic_block_sptr left_block = file_source_;
    gr::basic_block_sptr right_block = unpack_samples_;

    top_block->connect(file_source_, 0, unpack_samples_, 0);
    left_block = right_block;

    DLOG(INFO) << "connected file source to unpack samples";
    right_block = char_to_float_;
    top_block->connect( left_block, 0, right_block, 0 );
    left_block = right_block;
    DLOG(INFO) << "connected unpack samples to char to float";

    if( enable_throttle_control_ )
        {
            right_block = throttle_;
            top_block->connect( left_block, 0, right_block, 0 );
            left_block = right_block;
            DLOG(INFO) << " connected to throttle";
        }

    top_block->connect(left_block, 0, valve_, 0);
    DLOG(INFO) << "connected to valve";
    if (dump_)
        {
            top_block->connect(valve_, 0, sink_, 0);
            DLOG(INFO) << "connected valve to file sink";
        }
}


void TwoBitPackedFileSignalSource::disconnect(gr::top_block_sptr top_block)
{
    gr::basic_block_sptr left_block = file_source_;
    gr::basic_block_sptr right_block = unpack_samples_;

    top_block->disconnect(file_source_, 0, unpack_samples_, 0);
    left_block = right_block;


    DLOG(INFO) << "disconnected file source to unpack samples";
    right_block = char_to_float_;
    top_block->disconnect( left_block, 0, right_block, 0 );
    left_block = right_block;
    DLOG(INFO) << "disconnected unpack samples to char to float";

    if( enable_throttle_control_ )
        {
            right_block = throttle_;
            top_block->disconnect( left_block, 0, right_block, 0 );
            left_block = right_block;
            DLOG(INFO) << " disconnected to throttle";
        }

    top_block->disconnect(left_block, 0, valve_, 0);
    DLOG(INFO) << "disconnected to valve";
    if (dump_)
        {
            top_block->disconnect(valve_, 0, sink_, 0);
            DLOG(INFO) << "disconnected valve to file sink";
        }
}


gr::basic_block_sptr TwoBitPackedFileSignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    //return gr_block_sptr();
    return gr::blocks::file_source::sptr();
}


gr::basic_block_sptr TwoBitPackedFileSignalSource::get_right_block()
{
    return valve_;
}

