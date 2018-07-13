/*!
 * \file galileo_e5a_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Antonio Ramos, 2018. antonio.ramos(at)cttc.es
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

#include "galileo_e5a_pcps_acquisition.h"
#include "configuration_interface.h"
#include "galileo_e5_signal_processing.h"
#include "Galileo_E5a.h"
#include "gnss_sdr_flags.h"
#include "acq_conf.h"
#include <boost/lexical_cast.hpp>
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>


using google::LogMessage;

GalileoE5aPcpsAcquisition::GalileoE5aPcpsAcquisition(ConfigurationInterface* configuration,
    std::string role, unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    Acq_Conf acq_parameters = Acq_Conf();
    configuration_ = configuration;
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "../data/acquisition.dat";

    DLOG(INFO) << "Role " << role;

    item_type_ = configuration_->property(role + ".item_type", default_item_type);

    long fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 32000000);
    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    acq_parameters.fs_in = fs_in_;
    acq_parameters.samples_per_chip = static_cast<unsigned int>(ceil((1.0 / Galileo_E5a_CODE_CHIP_RATE_HZ) * static_cast<float>(acq_parameters.fs_in)));
    acq_pilot_ = configuration_->property(role + ".acquire_pilot", false);
    acq_iq_ = configuration_->property(role + ".acquire_iq", false);
    if (acq_iq_)
        {
            acq_pilot_ = false;
        }
    dump_ = configuration_->property(role + ".dump", false);
    acq_parameters.dump = dump_;
    acq_parameters.dump_channel = configuration_->property(role + ".dump_channel", 0);
    doppler_max_ = configuration_->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0) doppler_max_ = FLAGS_doppler_max;
    acq_parameters.doppler_max = doppler_max_;
    sampled_ms_ = 1;
    max_dwells_ = configuration_->property(role + ".max_dwells", 1);
    acq_parameters.max_dwells = max_dwells_;
    dump_filename_ = configuration_->property(role + ".dump_filename", default_dump_filename);
    acq_parameters.dump_filename = dump_filename_;
    bit_transition_flag_ = configuration_->property(role + ".bit_transition_flag", false);
    acq_parameters.bit_transition_flag = bit_transition_flag_;
    use_CFAR_ = configuration_->property(role + ".use_CFAR_algorithm", false);
    acq_parameters.use_CFAR_algorithm_flag = use_CFAR_;
    blocking_ = configuration_->property(role + ".blocking", true);
    acq_parameters.blocking = blocking_;
    //--- Find number of samples per spreading code (1ms)-------------------------
    code_length_ = static_cast<unsigned int>(std::round(static_cast<double>(fs_in_) / Galileo_E5a_CODE_CHIP_RATE_HZ * static_cast<double>(Galileo_E5a_CODE_LENGTH_CHIPS)));
    vector_length_ = code_length_ * sampled_ms_;

    code_ = new gr_complex[vector_length_];

    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
        }
    else if (item_type_.compare("cshort") == 0)
        {
            item_size_ = sizeof(lv_16sc_t);
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
    acq_parameters.it_size = item_size_;
    acq_parameters.samples_per_ms = static_cast<float>(fs_in_) * 0.001;
    acq_parameters.sampled_ms = sampled_ms_;
    acq_parameters.samples_per_code = acq_parameters.samples_per_ms * static_cast<float>(GALILEO_E5a_CODE_PERIOD_MS);
    acq_parameters.num_doppler_bins_step2 = configuration_->property(role + ".second_nbins", 4);
    acq_parameters.doppler_step2 = configuration_->property(role + ".second_doppler_step", 125.0);
    acq_parameters.make_2_steps = configuration_->property(role + ".make_two_steps", false);
    acq_parameters.blocking_on_standby = configuration_->property(role + ".blocking_on_standby", false);
    acquisition_ = pcps_make_acquisition(acq_parameters);

    stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_, vector_length_);
    channel_ = 0;
    threshold_ = 0.0;
    doppler_step_ = 0;
    gnss_synchro_ = 0;
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


GalileoE5aPcpsAcquisition::~GalileoE5aPcpsAcquisition()
{
    delete[] code_;
}


void GalileoE5aPcpsAcquisition::set_channel(unsigned int channel)
{
    channel_ = channel;
    acquisition_->set_channel(channel_);
}


void GalileoE5aPcpsAcquisition::set_threshold(float threshold)
{
    float pfa = configuration_->property(role_ + boost::lexical_cast<std::string>(channel_) + ".pfa", 0.0);

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

    acquisition_->set_threshold(threshold_);
}


void GalileoE5aPcpsAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;
    acquisition_->set_doppler_max(doppler_max_);
}


void GalileoE5aPcpsAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    acquisition_->set_doppler_step(doppler_step_);
}


void GalileoE5aPcpsAcquisition::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    acquisition_->set_gnss_synchro(gnss_synchro_);
}


signed int GalileoE5aPcpsAcquisition::mag()
{
    return acquisition_->mag();
}


void GalileoE5aPcpsAcquisition::init()
{
    acquisition_->init();
}


void GalileoE5aPcpsAcquisition::set_local_code()
{
    gr_complex* code = new gr_complex[code_length_];
    char signal_[3];

    if (acq_iq_)
        {
            strcpy(signal_, "5X");
        }
    else if (acq_pilot_)
        {
            strcpy(signal_, "5Q");
        }
    else
        {
            strcpy(signal_, "5I");
        }

    galileo_e5_a_code_gen_complex_sampled(code, signal_, gnss_synchro_->PRN, fs_in_, 0);

    for (unsigned int i = 0; i < sampled_ms_; i++)
        {
            memcpy(code_ + (i * code_length_), code, sizeof(gr_complex) * code_length_);
        }

    acquisition_->set_local_code(code_);
    delete[] code;
}


void GalileoE5aPcpsAcquisition::reset()
{
    acquisition_->set_active(true);
}


float GalileoE5aPcpsAcquisition::calculate_threshold(float pfa)
{
    unsigned int frequency_bins = 0;
    for (int doppler = static_cast<int>(-doppler_max_); doppler <= static_cast<int>(doppler_max_); doppler += doppler_step_)
        {
            frequency_bins++;
        }
    DLOG(INFO) << "Channel " << channel_ << "  Pfa = " << pfa;
    unsigned int ncells = vector_length_ * frequency_bins;
    double exponent = 1 / static_cast<double>(ncells);
    double val = pow(1.0 - pfa, exponent);
    double lambda = double(vector_length_);
    boost::math::exponential_distribution<double> mydist(lambda);
    float threshold = static_cast<float>(quantile(mydist, val));

    return threshold;
}


void GalileoE5aPcpsAcquisition::set_state(int state)
{
    acquisition_->set_state(state);
}


void GalileoE5aPcpsAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->connect(stream_to_vector_, 0, acquisition_, 0);
        }
    else if (item_type_.compare("cshort") == 0)
        {
            top_block->connect(stream_to_vector_, 0, acquisition_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


void GalileoE5aPcpsAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_, 0);
        }
    else if (item_type_.compare("cshort") == 0)
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


gr::basic_block_sptr GalileoE5aPcpsAcquisition::get_left_block()
{
    return stream_to_vector_;
}


gr::basic_block_sptr GalileoE5aPcpsAcquisition::get_right_block()
{
    return acquisition_;
}
