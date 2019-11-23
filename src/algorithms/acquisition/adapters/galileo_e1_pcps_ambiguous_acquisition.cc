/*!
 * \file galileo_e1_pcps_ambiguous_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E1 Signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#include "galileo_e1_pcps_ambiguous_acquisition.h"
#include "Galileo_E1.h"
#include "acq_conf.h"
#include "configuration_interface.h"
#include "galileo_e1_signal_processing.h"
#include "gnss_sdr_flags.h"
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include <algorithm>


GalileoE1PcpsAmbiguousAcquisition::GalileoE1PcpsAmbiguousAcquisition(
    ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    configuration_ = configuration;
    acq_parameters_.ms_per_code = 4;
    acq_parameters_.SetFromConfiguration(configuration_, role, GALILEO_E1_CODE_CHIP_RATE_CPS, GALILEO_E1_OPT_ACQ_FS_SPS);

    DLOG(INFO) << "role " << role;

    if (FLAGS_doppler_max != 0)
        {
            acq_parameters_.doppler_max = FLAGS_doppler_max;
        }
    doppler_max_ = acq_parameters_.doppler_max;
    doppler_step_ = acq_parameters_.doppler_step;
    item_type_ = acq_parameters_.item_type;
    item_size_ = acq_parameters_.it_size;
    fs_in_ = acq_parameters_.fs_in;
    acquire_pilot_ = configuration->property(role + ".acquire_pilot", "false");

    code_length_ = static_cast<unsigned int>(std::floor(static_cast<double>(acq_parameters_.resampled_fs) / (GALILEO_E1_CODE_CHIP_RATE_CPS / GALILEO_E1_B_CODE_LENGTH_CHIPS)));
    vector_length_ = std::floor(acq_parameters_.sampled_ms * acq_parameters_.samples_per_ms) * (acq_parameters_.bit_transition_flag ? 2 : 1);
    code_ = std::vector<std::complex<float>>(vector_length_);

    sampled_ms_ = acq_parameters_.sampled_ms;

    acquisition_ = pcps_make_acquisition(acq_parameters_);
    DLOG(INFO) << "acquisition(" << acquisition_->unique_id() << ")";

    if (item_type_ == "cbyte")
        {
            cbyte_to_float_x2_ = make_complex_byte_to_float_x2();
            float_to_complex_ = gr::blocks::float_to_complex::make();
        }

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


void GalileoE1PcpsAmbiguousAcquisition::stop_acquisition()
{
}


void GalileoE1PcpsAmbiguousAcquisition::set_threshold(float threshold)
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


void GalileoE1PcpsAmbiguousAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    acquisition_->set_doppler_max(doppler_max_);
}


void GalileoE1PcpsAmbiguousAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;

    acquisition_->set_doppler_step(doppler_step_);
}


void GalileoE1PcpsAmbiguousAcquisition::set_doppler_center(int doppler_center)
{
    doppler_center_ = doppler_center;

    acquisition_->set_doppler_center(doppler_center_);
}


void GalileoE1PcpsAmbiguousAcquisition::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;

    acquisition_->set_gnss_synchro(gnss_synchro_);
}


signed int GalileoE1PcpsAmbiguousAcquisition::mag()
{
    return acquisition_->mag();
}


void GalileoE1PcpsAmbiguousAcquisition::init()
{
    acquisition_->init();
}


void GalileoE1PcpsAmbiguousAcquisition::set_local_code()
{
    bool cboc = configuration_->property(
        "Acquisition" + std::to_string(channel_) + ".cboc", false);

    std::vector<std::complex<float>> code(code_length_);

    if (acquire_pilot_ == true)
        {
            // set local signal generator to Galileo E1 pilot component (1C)
            std::array<char, 3> pilot_signal = {{'1', 'C', '\0'}};
            if (acq_parameters_.use_automatic_resampler)
                {
                    galileo_e1_code_gen_complex_sampled(code, pilot_signal,
                        cboc, gnss_synchro_->PRN, acq_parameters_.resampled_fs, 0, false);
                }
            else
                {
                    galileo_e1_code_gen_complex_sampled(code, pilot_signal,
                        cboc, gnss_synchro_->PRN, fs_in_, 0, false);
                }
        }
    else
        {
            std::array<char, 3> Signal_{};
            Signal_[0] = gnss_synchro_->Signal[0];
            Signal_[1] = gnss_synchro_->Signal[1];
            Signal_[2] = '\0';
            if (acq_parameters_.use_automatic_resampler)
                {
                    galileo_e1_code_gen_complex_sampled(code, Signal_,
                        cboc, gnss_synchro_->PRN, acq_parameters_.resampled_fs, 0, false);
                }
            else
                {
                    galileo_e1_code_gen_complex_sampled(code, Signal_,
                        cboc, gnss_synchro_->PRN, fs_in_, 0, false);
                }
        }

    gsl::span<gr_complex> code__span(code_.data(), vector_length_);
    for (unsigned int i = 0; i < sampled_ms_ / 4; i++)
        {
            std::copy_n(code.data(), code_length_, code__span.subspan(i * code_length_, code_length_).data());
        }

    acquisition_->set_local_code(code_.data());
}


void GalileoE1PcpsAmbiguousAcquisition::reset()
{
    acquisition_->set_active(true);
}


void GalileoE1PcpsAmbiguousAcquisition::set_state(int state)
{
    acquisition_->set_state(state);
}


float GalileoE1PcpsAmbiguousAcquisition::calculate_threshold(float pfa)
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


void GalileoE1PcpsAmbiguousAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
        {
            // nothing to connect
        }
    else if (item_type_ == "cshort")
        {
            // nothing to connect
        }
    else if (item_type_ == "cbyte")
        {
            // Since a byte-based acq implementation is not available,
            // we just convert cshorts to gr_complex
            top_block->connect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->connect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->connect(float_to_complex_, 0, acquisition_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


void GalileoE1PcpsAmbiguousAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
        {
            // nothing to disconnect
        }
    else if (item_type_ == "cshort")
        {
            // nothing to disconnect
        }
    else if (item_type_ == "cbyte")
        {
            top_block->disconnect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->disconnect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->disconnect(float_to_complex_, 0, acquisition_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


gr::basic_block_sptr GalileoE1PcpsAmbiguousAcquisition::get_left_block()
{
    if (item_type_ == "gr_complex")
        {
            return acquisition_;
        }
    if (item_type_ == "cshort")
        {
            return acquisition_;
        }
    if (item_type_ == "cbyte")
        {
            return cbyte_to_float_x2_;
        }

    LOG(WARNING) << item_type_ << " unknown acquisition item type";
    return nullptr;
}


gr::basic_block_sptr GalileoE1PcpsAmbiguousAcquisition::get_right_block()
{
    return acquisition_;
}


void GalileoE1PcpsAmbiguousAcquisition::set_resampler_latency(uint32_t latency_samples)
{
    acquisition_->set_resampler_latency(latency_samples);
}
