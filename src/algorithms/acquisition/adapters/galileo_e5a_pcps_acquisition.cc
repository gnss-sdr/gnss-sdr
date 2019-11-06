/*!
 * \file galileo_e5a_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Antonio Ramos, 2018. antonio.ramos(at)cttc.es
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

#include "Galileo_E5a.h"
#include "acq_conf.h"
#include "configuration_interface.h"
#include "galileo_e5_signal_processing.h"
#include "galileo_e5a_pcps_acquisition.h"
#include "gnss_sdr_flags.h"
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <algorithm>


GalileoE5aPcpsAcquisition::GalileoE5aPcpsAcquisition(ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    configuration_ = configuration;
    acq_parameters_.ms_per_code = 1;
    acq_parameters_.SetFromConfiguration(configuration_, role, GALILEO_E5A_CODE_CHIP_RATE_CPS, GALILEO_E5A_OPT_ACQ_FS_SPS);

    DLOG(INFO) << "Role " << role;

    if (FLAGS_doppler_max != 0) acq_parameters_.doppler_max = FLAGS_doppler_max;
    doppler_max_ = acq_parameters_.doppler_max;
    doppler_step_ = acq_parameters_.doppler_step;
    item_type_ = acq_parameters_.item_type;
    fs_in_ = acq_parameters_.fs_in;

    acq_pilot_ = configuration_->property(role + ".acquire_pilot", false);
    acq_iq_ = configuration_->property(role + ".acquire_iq", false);
    if (acq_iq_)
        {
            acq_pilot_ = false;
        }

    code_length_ = static_cast<unsigned int>(std::floor(static_cast<double>(acq_parameters_.resampled_fs) / (GALILEO_E5A_CODE_CHIP_RATE_CPS / GALILEO_E5A_CODE_LENGTH_CHIPS)));
    vector_length_ = std::floor(acq_parameters_.sampled_ms * acq_parameters_.samples_per_ms) * (acq_parameters_.bit_transition_flag ? 2 : 1);
    code_ = std::vector<std::complex<float>>(vector_length_);

    sampled_ms_ = acq_parameters_.sampled_ms;

    acquisition_ = pcps_make_acquisition(acq_parameters_);
    DLOG(INFO) << "acquisition(" << acquisition_->unique_id() << ")";


    channel_ = 0;
    threshold_ = 0.0;
    doppler_center_ = 0;
    gnss_synchro_ = nullptr;

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


void GalileoE5aPcpsAcquisition::stop_acquisition()
{
}


void GalileoE5aPcpsAcquisition::set_threshold(float threshold)
{
    float pfa = configuration_->property(role_ + std::to_string(channel_) + ".pfa", 0.0);

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


void GalileoE5aPcpsAcquisition::set_doppler_center(int doppler_center)
{
    doppler_center_ = doppler_center;

    acquisition_->set_doppler_center(doppler_center_);
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
    std::vector<std::complex<float>> code(code_length_);
    std::array<char, 3> signal_{};
    signal_[0] = '5';
    signal_[2] = '\0';

    if (acq_iq_)
        {
            signal_[1] = 'X';
        }
    else if (acq_pilot_)
        {
            signal_[1] = 'Q';
        }
    else
        {
            signal_[1] = 'I';
        }

    if (acq_parameters_.use_automatic_resampler)
        {
            galileo_e5_a_code_gen_complex_sampled(code, signal_, gnss_synchro_->PRN, acq_parameters_.resampled_fs, 0);
        }
    else
        {
            galileo_e5_a_code_gen_complex_sampled(code, signal_, gnss_synchro_->PRN, fs_in_, 0);
        }
    gsl::span<gr_complex> code_span(code_.data(), vector_length_);
    for (unsigned int i = 0; i < sampled_ms_; i++)
        {
            std::copy_n(code.data(), code_length_, code_span.subspan(i * code_length_, code_length_).data());
        }

    acquisition_->set_local_code(code_.data());
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
    auto lambda = static_cast<double>(vector_length_);
    boost::math::exponential_distribution<double> mydist(lambda);
    auto threshold = static_cast<float>(quantile(mydist, val));

    return threshold;
}


void GalileoE5aPcpsAcquisition::set_state(int state)
{
    acquisition_->set_state(state);
}


void GalileoE5aPcpsAcquisition::connect(gr::top_block_sptr top_block __attribute__((unused)))
{
    if (item_type_ == "fc32")
        {
            // nothing to connect
        }
    else if (item_type_ == "ic16")
        {
            // nothing to connect
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


void GalileoE5aPcpsAcquisition::disconnect(gr::top_block_sptr top_block __attribute__((unused)))
{
    if (item_type_ == "fc32")
        {
            // nothing to disconnect
        }
    else if (item_type_ == "ic16")
        {
            // nothing to disconnect
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


gr::basic_block_sptr GalileoE5aPcpsAcquisition::get_left_block()
{
    return acquisition_;
}


gr::basic_block_sptr GalileoE5aPcpsAcquisition::get_right_block()
{
    return acquisition_;
}


void GalileoE5aPcpsAcquisition::set_resampler_latency(uint32_t latency_samples)
{
    acquisition_->set_resampler_latency(latency_samples);
}
