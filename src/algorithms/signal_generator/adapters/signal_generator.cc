/*!
 * \file signal_generator.cc
 * \brief Adapter of a class that generates synthesized GNSS signal.
 * \author Marc Molina, 2013. marc.molina.pena@gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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


#include "signal_generator.h"
#include "configuration_interface.h"
#include "Galileo_E1.h"
#include "GPS_L1_CA.h"
#include "Galileo_E5a.h"
#include "GLONASS_L1_L2_CA.h"
#include <glog/logging.h>


using google::LogMessage;

SignalGenerator::SignalGenerator(ConfigurationInterface* configuration,
    std::string role, unsigned int in_stream,
    unsigned int out_stream, boost::shared_ptr<gr::msg_queue> queue) : role_(role), in_stream_(in_stream), out_stream_(out_stream), queue_(queue)
{
    std::string default_item_type = "gr_complex";
    std::string default_dump_file = "./data/gen_source.dat";
    std::string default_system = "G";
    std::string default_signal = "1C";

    item_type_ = configuration->property(role + ".item_type", default_item_type);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

    unsigned int fs_in = configuration->property("SignalSource.fs_hz", 4e6);
    bool data_flag = configuration->property("SignalSource.data_flag", false);
    bool noise_flag = configuration->property("SignalSource.noise_flag", false);
    float BW_BB = configuration->property("SignalSource.BW_BB", 1.0);
    unsigned int num_satellites = configuration->property("SignalSource.num_satellites", 1);

    std::vector<std::string> signal1;
    std::vector<std::string> system;
    std::vector<unsigned int> PRN;
    std::vector<float> CN0_dB;
    std::vector<float> doppler_Hz;
    std::vector<unsigned int> delay_chips;
    std::vector<unsigned int> delay_sec;

    for (unsigned int sat_idx = 0; sat_idx < num_satellites; sat_idx++)
        {
            std::string sat = std::to_string(sat_idx);
            signal1.push_back(configuration->property("SignalSource.signal_" + sat, default_signal));
            system.push_back(configuration->property("SignalSource.system_" + sat, default_system));
            PRN.push_back(configuration->property("SignalSource.PRN_" + sat, 1));
            CN0_dB.push_back(configuration->property("SignalSource.CN0_dB_" + sat, 10));
            doppler_Hz.push_back(configuration->property("SignalSource.doppler_Hz_" + sat, 0));
            delay_chips.push_back(configuration->property("SignalSource.delay_chips_" + sat, 0));
            delay_sec.push_back(configuration->property("SignalSource.delay_sec_" + sat, 0));
        }

    // If Galileo signal is present -> vector duration = 100 ms (25 * 4 ms)
    // If there is only GPS signal (Galileo signal not present) -> vector duration = 1 ms
    unsigned int vector_length = 0;
    if (std::find(system.begin(), system.end(), "E") != system.end())
        {
            if (signal1[0].at(0) == '5')
                {
                    vector_length = round(static_cast<float>(fs_in) / (Galileo_E5a_CODE_CHIP_RATE_HZ / Galileo_E5a_CODE_LENGTH_CHIPS));
                }
            else
                {
                    vector_length = round(static_cast<float>(fs_in) / (Galileo_E1_CODE_CHIP_RATE_HZ / Galileo_E1_B_CODE_LENGTH_CHIPS)) * Galileo_E1_C_SECONDARY_CODE_LENGTH;
                }
        }
    else if (std::find(system.begin(), system.end(), "G") != system.end())
        {
            vector_length = round(static_cast<float>(fs_in) / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));
        }
    else if (std::find(system.begin(), system.end(), "R") != system.end())
        {
            if (signal1[0].at(0) == '1')
                {
                    vector_length = round((float)fs_in / (GLONASS_L1_CA_CODE_RATE_HZ / GLONASS_L1_CA_CODE_LENGTH_CHIPS));
                }
            else
                {
                    vector_length = round((float)fs_in / (GLONASS_L2_CA_CODE_RATE_HZ / GLONASS_L2_CA_CODE_LENGTH_CHIPS));
                }
        }

    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            DLOG(INFO) << "Item size " << item_size_;
            gen_source_ = signal_make_generator_c(signal1, system, PRN, CN0_dB, doppler_Hz, delay_chips, delay_sec,
                data_flag, noise_flag, fs_in, vector_length, BW_BB);

            vector_to_stream_ = gr::blocks::vector_to_stream::make(item_size_, vector_length);

            DLOG(INFO) << "vector_to_stream(" << vector_to_stream_->unique_id() << ")";
            DLOG(INFO) << "gen_source(" << gen_source_->unique_id() << ")";
        }

    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type for resampler";
            item_size_ = sizeof(short);
        }

    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
        }
    if (dump_)
        {
            DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }
}


SignalGenerator::~SignalGenerator()
{
}


void SignalGenerator::connect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->connect(gen_source_, 0, vector_to_stream_, 0);
            DLOG(INFO) << "connected gen_source to vector_to_stream";

            if (dump_)
                {
                    top_block->connect(vector_to_stream_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected vector_to_stream_ to file sink";
                }
        }
}


void SignalGenerator::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->disconnect(gen_source_, 0, vector_to_stream_, 0);
            if (dump_)
                {
                    top_block->disconnect(vector_to_stream_, 0, file_sink_, 0);
                }
        }
}


gr::basic_block_sptr SignalGenerator::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::block_sptr();
}


gr::basic_block_sptr SignalGenerator::get_right_block()
{
    return vector_to_stream_;
}
