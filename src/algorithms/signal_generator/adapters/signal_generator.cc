/*!
 * \file signal_generator.cc
 * \brief Adapter of a class that generates synthesized GNSS signal.
 * \author Marc Molina, 2013. marc.molina.pena@gmail.com
 *
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


#include "signal_generator.h"
#include "Beidou_B1I.h"
#include "GLONASS_L1_L2_CA.h"
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "Galileo_E5b.h"
#include "Galileo_E6.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <cstdint>
#include <utility>


SignalGenerator::SignalGenerator(const ConfigurationInterface* configuration,
    const std::string& role, unsigned int in_stream,
    unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue __attribute__((unused))) : role_(role),
                                                                   in_stream_(in_stream),
                                                                   out_stream_(out_stream),
                                                                   dump_(configuration->property(role + ".dump", false))
{
    const std::string default_item_type("gr_complex");
    const std::string default_dump_file("./data/gen_source.dat");
    const std::string default_system("G");
    const std::string default_signal("1C");

    item_type_ = configuration->property(role + ".item_type", default_item_type);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

    const unsigned int fs_in = configuration->property("SignalSource.fs_hz", static_cast<unsigned>(4e6));
    const bool data_flag = configuration->property("SignalSource.data_flag", false);
    const bool noise_flag = configuration->property("SignalSource.noise_flag", false);
    const float BW_BB = configuration->property("SignalSource.BW_BB", static_cast<float>(1.0));
    const unsigned int num_satellites = configuration->property("SignalSource.num_satellites", 1);

    std::vector<std::string> signal1;
    std::vector<std::string> system;
    std::vector<unsigned int> PRN;
    std::vector<float> CN0_dB;
    std::vector<float> doppler_Hz;
    std::vector<unsigned int> delay_chips;
    std::vector<unsigned int> delay_sec;

    signal1.reserve(num_satellites);
    system.reserve(num_satellites);
    PRN.reserve(num_satellites);
    CN0_dB.reserve(num_satellites);
    doppler_Hz.reserve(num_satellites);
    delay_chips.reserve(num_satellites);
    delay_sec.reserve(num_satellites);

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
                    vector_length = static_cast<unsigned int>(round(static_cast<float>(fs_in) / (GALILEO_E5A_CODE_CHIP_RATE_CPS / GALILEO_E5A_CODE_LENGTH_CHIPS)));
                }
            else if (signal1[0].at(0) == '7')
                {
                    vector_length = static_cast<unsigned int>(round(static_cast<float>(fs_in) / (GALILEO_E5B_CODE_CHIP_RATE_CPS / GALILEO_E5B_CODE_LENGTH_CHIPS)));
                }
            else if (signal1[0].at(1) == '6')
                {
                    vector_length = static_cast<unsigned int>(round(static_cast<float>(fs_in) / (GALILEO_E6_C_CODE_CHIP_RATE_CPS / GALILEO_E6_C_CODE_LENGTH_CHIPS)) * GALILEO_E6_C_SECONDARY_CODE_LENGTH_CHIPS);
                }
            else
                {
                    vector_length = static_cast<unsigned int>(round(static_cast<float>(fs_in) / (GALILEO_E1_CODE_CHIP_RATE_CPS / GALILEO_E1_B_CODE_LENGTH_CHIPS)) * GALILEO_E1_C_SECONDARY_CODE_LENGTH);
                }
        }
    else if (std::find(system.begin(), system.end(), "G") != system.end())
        {
            vector_length = static_cast<unsigned int>(round(static_cast<float>(fs_in) / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));
        }
    else if (std::find(system.begin(), system.end(), "R") != system.end())
        {
            if (signal1[0].at(0) == '1')
                {
                    vector_length = static_cast<unsigned int>(round(static_cast<float>(fs_in) / (GLONASS_L1_CA_CODE_RATE_CPS / GLONASS_L1_CA_CODE_LENGTH_CHIPS)));
                }
            else
                {
                    vector_length = static_cast<unsigned int>(round(static_cast<float>(fs_in) / (GLONASS_L2_CA_CODE_RATE_CPS / GLONASS_L2_CA_CODE_LENGTH_CHIPS)));
                }
        }

    else if (std::find(system.begin(), system.end(), "B") != system.end())
        {
            vector_length = static_cast<unsigned int>(round(static_cast<float>(fs_in) / (BEIDOU_B1I_CODE_RATE_CPS / BEIDOU_B1I_CODE_LENGTH_CHIPS)));
        }

    if (item_type_ == "gr_complex")
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
            item_size_ = sizeof(int16_t);
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
    if (in_stream_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void SignalGenerator::connect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
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
    if (item_type_ == "gr_complex")
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
