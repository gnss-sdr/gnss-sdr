/*!
 * \file galileo_e1_pcps_tong_ambiguous_acquisition.cc
 * \brief Adapts a PCPS Tong acquisition block to an Acq1uisitionInterface for
 *  Galileo E1 Signals
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
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

#include "galileo_e1_pcps_tong_ambiguous_acquisition.h"
#include <boost/lexical_cast.hpp>
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include "galileo_e1_signal_processing.h"
#include "Galileo_E1.h"
#include "configuration_interface.h"

using google::LogMessage;

GalileoE1PcpsTongAmbiguousAcquisition::GalileoE1PcpsTongAmbiguousAcquisition(
        ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams) :
        role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    configuration_ = configuration;
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "../data/acquisition.dat";

    DLOG(INFO) << "role " << role;

    item_type_ = configuration_->property(role + ".item_type",
            default_item_type);

    long fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 4000000);
    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    if_ = configuration_->property(role + ".if", 0);
    dump_ = configuration_->property(role + ".dump", false);
    doppler_max_ = configuration_->property(role + ".doppler_max", 5000);
    sampled_ms_ = configuration_->property(role + ".coherent_integration_time_ms", 4);

    if (sampled_ms_ % 4 != 0)
        {
            sampled_ms_ = static_cast<int>(sampled_ms_ / 4) * 4;
            LOG(WARNING) << "coherent_integration_time should be multiple of "
                    << "Galileo code length (4 ms). coherent_integration_time = "
                    << sampled_ms_ << " ms will be used.";
        }

    tong_init_val_ = configuration->property(role + ".tong_init_val", 1);
    tong_max_val_ = configuration->property(role + ".tong_max_val", 2);
    tong_max_dwells_ = configuration->property(role + ".tong_max_dwells", tong_max_val_ + 1);

    dump_filename_ = configuration_->property(role + ".dump_filename",
            default_dump_filename);

    //--- Find number of samples per spreading code (4 ms)  -----------------

    code_length_ = round(
            fs_in_
            / (Galileo_E1_CODE_CHIP_RATE_HZ
                    / Galileo_E1_B_CODE_LENGTH_CHIPS));

    vector_length_ = code_length_ * static_cast<int>(sampled_ms_ / 4);

    int samples_per_ms = code_length_ / 4;

    code_ = new gr_complex[vector_length_];

    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            acquisition_cc_ = pcps_tong_make_acquisition_cc(sampled_ms_, doppler_max_,
                    if_, fs_in_, samples_per_ms, code_length_, tong_init_val_,
                    tong_max_val_, tong_max_dwells_, dump_, dump_filename_);

            stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_, vector_length_);
            DLOG(INFO) << "stream_to_vector("
                    << stream_to_vector_->unique_id() << ")";
            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id()
                    << ")";
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


GalileoE1PcpsTongAmbiguousAcquisition::~GalileoE1PcpsTongAmbiguousAcquisition()
{
    delete[] code_;
}


void GalileoE1PcpsTongAmbiguousAcquisition::set_channel(unsigned int channel)
{
    channel_ = channel;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_channel(channel_);
        }
}


void GalileoE1PcpsTongAmbiguousAcquisition::set_threshold(float threshold)
{

    float pfa = configuration_->property(role_+ boost::lexical_cast<std::string>(channel_) + ".pfa", 0.0);

    if(pfa == 0.0) pfa = configuration_->property(role_+".pfa", 0.0);

    if(pfa == 0.0)
        {
            threshold_ = threshold;
        }
    else
        {
            threshold_ = calculate_threshold(pfa);
        }

    DLOG(INFO) <<"Channel "<<channel_<<" Threshold = " << threshold_;

    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_threshold(threshold_);
        }
}


void GalileoE1PcpsTongAmbiguousAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_doppler_max(doppler_max_);
        }
}


void GalileoE1PcpsTongAmbiguousAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_doppler_step(doppler_step_);
        }

}


void GalileoE1PcpsTongAmbiguousAcquisition::set_gnss_synchro(
        Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_gnss_synchro(gnss_synchro_);
        }
}


signed int GalileoE1PcpsTongAmbiguousAcquisition::mag()
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


void GalileoE1PcpsTongAmbiguousAcquisition::init()
{
    acquisition_cc_->init();
    //set_local_code();
}


void GalileoE1PcpsTongAmbiguousAcquisition::set_local_code()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            bool cboc = configuration_->property(
                    "Acquisition" + boost::lexical_cast<std::string>(channel_)
                            + ".cboc", false);

            std::complex<float> * code = new std::complex<float>[code_length_];

            galileo_e1_code_gen_complex_sampled(code, gnss_synchro_->Signal,
                    cboc, gnss_synchro_->PRN, fs_in_, 0, false);

            for (unsigned int i = 0; i < sampled_ms_/4; i++)
                {
                    memcpy(&(code_[i*code_length_]), code,
                           sizeof(gr_complex)*code_length_);
                }

            acquisition_cc_->set_local_code(code_);

            delete[] code;
        }
}


void GalileoE1PcpsTongAmbiguousAcquisition::reset()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_active(true);
        }
}

void GalileoE1PcpsTongAmbiguousAcquisition::set_state(int state)
{
    acquisition_cc_->set_state(state);
}


float GalileoE1PcpsTongAmbiguousAcquisition::calculate_threshold(float pfa)
{
    unsigned int frequency_bins = 0;
    for (int doppler = static_cast<int>(-doppler_max_); doppler <= static_cast<int>(doppler_max_); doppler += doppler_step_)
        {
            frequency_bins++;
        }

    DLOG(INFO) << "Channel " << channel_ << "  Pfa = " << pfa;

    unsigned int ncells = vector_length_ * frequency_bins;
    double exponent = 1 / static_cast<double>(ncells);
    double val = pow(1.0-pfa,exponent);
    double lambda = double(vector_length_);
    boost::math::exponential_distribution<double> mydist (lambda);
    float threshold = static_cast<float>(quantile(mydist,val));

    return threshold;
}


void GalileoE1PcpsTongAmbiguousAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


void GalileoE1PcpsTongAmbiguousAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


gr::basic_block_sptr GalileoE1PcpsTongAmbiguousAcquisition::get_left_block()
{
    return stream_to_vector_;
}


gr::basic_block_sptr GalileoE1PcpsTongAmbiguousAcquisition::get_right_block()
{
    return acquisition_cc_;
}

