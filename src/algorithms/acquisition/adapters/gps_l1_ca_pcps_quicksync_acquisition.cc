/*!
 * \file gps_l1_ca_pcps_quicksync_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L1 C/A signals using the QuickSync Algorithm
 * \author Damian Miralles, 2014. dmiralles2009@gmail.com
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

#include "gps_l1_ca_pcps_quicksync_acquisition.h"
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include "gps_sdr_signal_processing.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"


using google::LogMessage;

GpsL1CaPcpsQuickSyncAcquisition::GpsL1CaPcpsQuickSyncAcquisition(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    configuration_ = configuration;
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./data/acquisition.dat";

    DLOG(INFO) << "role " << role;

    item_type_ = configuration_->property(role + ".item_type", default_item_type);
    long fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    if_ = configuration_->property(role + ".if", 0);
    dump_ = configuration_->property(role + ".dump", false);
    doppler_max_ = configuration->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0) doppler_max_ = FLAGS_doppler_max;
    sampled_ms_ = configuration_->property(role + ".coherent_integration_time_ms", 4);

    //--- Find number of samples per spreading code -------------------------
    code_length_ = round(fs_in_ / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));

    /*Calculate the folding factor value based on the calculations*/
    unsigned int temp = static_cast<unsigned int>(ceil(sqrt(log2(code_length_))));
    folding_factor_ = configuration_->property(role + ".folding_factor", temp);

    if (sampled_ms_ % folding_factor_ != 0)
        {
            LOG(WARNING) << "QuickSync Algorithm requires a coherent_integration_time"
                         << " multiple of " << folding_factor_ << "ms, Value entered "
                         << sampled_ms_ << " ms";
            if (sampled_ms_ < folding_factor_)
                {
                    sampled_ms_ = static_cast<int>(folding_factor_);
                }
            else
                {
                    sampled_ms_ = static_cast<int>(sampled_ms_ / folding_factor_) * folding_factor_;
                }

            LOG(WARNING) << " Coherent_integration_time of "
                         << sampled_ms_ << " ms will be used instead.";
        }

    vector_length_ = code_length_ * sampled_ms_;
    bit_transition_flag_ = configuration_->property(role + ".bit_transition_flag", false);

    if (!bit_transition_flag_)
        {
            max_dwells_ = configuration_->property(role + ".max_dwells", 1);
        }
    else
        {
            max_dwells_ = 2;
        }

    dump_filename_ = configuration_->property(role + ".dump_filename", default_dump_filename);

    int samples_per_ms = round(code_length_);
    code_ = new gr_complex[code_length_]();
    /*Object relevant information for debugging*/
    LOG(INFO) << "Implementation: " << this->implementation()
              << ", Vector Length: " << vector_length_
              << ", Samples per ms: " << samples_per_ms
              << ", Folding factor: " << folding_factor_
              << ", Sampled  ms: " << sampled_ms_
              << ", Code Length: " << code_length_;

    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            acquisition_cc_ = pcps_quicksync_make_acquisition_cc(folding_factor_,
                sampled_ms_, max_dwells_, doppler_max_, if_, fs_in_,
                samples_per_ms, code_length_, bit_transition_flag_,
                dump_, dump_filename_);

            stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_,
                code_length_ * folding_factor_);

            DLOG(INFO) << "stream_to_vector_quicksync(" << stream_to_vector_->unique_id() << ")";
            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id() << ")";
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }

    channel_ = 0;
    threshold_ = 0.0;
    doppler_step_ = 0;
    gnss_synchro_ = 0;
}


GpsL1CaPcpsQuickSyncAcquisition::~GpsL1CaPcpsQuickSyncAcquisition()
{
    delete[] code_;
}


void GpsL1CaPcpsQuickSyncAcquisition::set_channel(unsigned int channel)
{
    channel_ = channel;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_channel(channel_);
        }
}


void GpsL1CaPcpsQuickSyncAcquisition::set_threshold(float threshold)
{
    float pfa = configuration_->property(role_ +
                                             boost::lexical_cast<std::string>(channel_) + ".pfa",
        0.0);

    if (pfa == 0.0)
        {
            pfa = configuration_->property(role_ + ".pfa", 0.0);
        }
    if (pfa == 0.0)
        {
            threshold_ = threshold;
        }
    else
        {
            threshold_ = calculate_threshold(pfa);
        }

    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold_;

    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_threshold(threshold_);
        }
}


void GpsL1CaPcpsQuickSyncAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_doppler_max(doppler_max_);
        }
}


void GpsL1CaPcpsQuickSyncAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_doppler_step(doppler_step_);
        }
}


void GpsL1CaPcpsQuickSyncAcquisition::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_gnss_synchro(gnss_synchro_);
        }
}


signed int GpsL1CaPcpsQuickSyncAcquisition::mag()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            return acquisition_cc_->mag();
        }
    else
        {
            return 0;
        }
}


void GpsL1CaPcpsQuickSyncAcquisition::init()
{
    acquisition_cc_->init();
    //set_local_code();
}


void GpsL1CaPcpsQuickSyncAcquisition::set_local_code()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            std::complex<float>* code = new std::complex<float>[code_length_]();

            gps_l1_ca_code_gen_complex_sampled(code, gnss_synchro_->PRN, fs_in_, 0);


            for (unsigned int i = 0; i < (sampled_ms_ / folding_factor_); i++)
                {
                    memcpy(&(code_[i * code_length_]), code,
                        sizeof(gr_complex) * code_length_);
                }

            //memcpy(code_, code,sizeof(gr_complex)*code_length_);
            acquisition_cc_->set_local_code(code_);

            delete[] code;
        }
}


void GpsL1CaPcpsQuickSyncAcquisition::reset()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_active(true);
        }
}


void GpsL1CaPcpsQuickSyncAcquisition::set_state(int state)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_state(state);
        }
}


float GpsL1CaPcpsQuickSyncAcquisition::calculate_threshold(float pfa)
{
    //Calculate the threshold
    unsigned int frequency_bins = 0;
    for (int doppler = static_cast<int>(-doppler_max_); doppler <= static_cast<int>(doppler_max_); doppler += doppler_step_)
        {
            frequency_bins++;
        }
    DLOG(INFO) << "Channel " << channel_ << "  Pfa = " << pfa;
    unsigned int ncells = (code_length_ / folding_factor_) * frequency_bins;
    double exponent = 1.0 / static_cast<double>(ncells);
    double val = pow(1.0 - pfa, exponent);
    double lambda = static_cast<double>(code_length_) / static_cast<double>(folding_factor_);
    boost::math::exponential_distribution<double> mydist(lambda);
    float threshold = static_cast<float>(quantile(mydist, val));

    return threshold;
}


void GpsL1CaPcpsQuickSyncAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


void GpsL1CaPcpsQuickSyncAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


gr::basic_block_sptr GpsL1CaPcpsQuickSyncAcquisition::get_left_block()
{
    return stream_to_vector_;
}


gr::basic_block_sptr GpsL1CaPcpsQuickSyncAcquisition::get_right_block()
{
    return acquisition_cc_;
}
