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

#include "galileo_e5a_pcps_acquisition.h"
#include "Galileo_E5a.h"
#include "acq_conf.h"
#include "configuration_interface.h"
#include "galileo_e5_signal_processing.h"
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
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./acquisition.mat";

    DLOG(INFO) << "Role " << role;

    item_type_ = configuration_->property(role + ".item_type", default_item_type);

    int64_t fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 32000000);
    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    acq_parameters_.fs_in = fs_in_;
    acq_pilot_ = configuration_->property(role + ".acquire_pilot", false);
    acq_iq_ = configuration_->property(role + ".acquire_iq", false);
    if (acq_iq_)
        {
            acq_pilot_ = false;
        }
    dump_ = configuration_->property(role + ".dump", false);
    acq_parameters_.dump = dump_;
    acq_parameters_.dump_channel = configuration_->property(role + ".dump_channel", 0);
    doppler_max_ = configuration_->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0)
        {
            doppler_max_ = FLAGS_doppler_max;
        }
    acq_parameters_.doppler_max = doppler_max_;
    sampled_ms_ = 1;
    max_dwells_ = configuration_->property(role + ".max_dwells", 1);
    acq_parameters_.max_dwells = max_dwells_;
    dump_filename_ = configuration_->property(role + ".dump_filename", default_dump_filename);
    acq_parameters_.dump_filename = dump_filename_;
    bit_transition_flag_ = configuration_->property(role + ".bit_transition_flag", false);
    acq_parameters_.bit_transition_flag = bit_transition_flag_;
    use_CFAR_ = configuration_->property(role + ".use_CFAR_algorithm", false);
    acq_parameters_.use_CFAR_algorithm_flag = use_CFAR_;
    blocking_ = configuration_->property(role + ".blocking", true);
    acq_parameters_.blocking = blocking_;

    acq_parameters_.use_automatic_resampler = configuration_->property("GNSS-SDR.use_acquisition_resampler", false);
    if (acq_parameters_.use_automatic_resampler == true and item_type_ != "gr_complex")
        {
            LOG(WARNING) << "Galileo E5a acquisition disabled the automatic resampler feature because its item_type is not set to gr_complex";
            acq_parameters_.use_automatic_resampler = false;
        }
    if (acq_parameters_.use_automatic_resampler)
        {
            if (acq_parameters_.fs_in > GALILEO_E5A_OPT_ACQ_FS_HZ)
                {
                    acq_parameters_.resampler_ratio = floor(static_cast<float>(acq_parameters_.fs_in) / GALILEO_E5A_OPT_ACQ_FS_HZ);
                    uint32_t decimation = acq_parameters_.fs_in / GALILEO_E5A_OPT_ACQ_FS_HZ;
                    while (acq_parameters_.fs_in % decimation > 0)
                        {
                            decimation--;
                        };
                    acq_parameters_.resampler_ratio = decimation;
                    acq_parameters_.resampled_fs = acq_parameters_.fs_in / static_cast<int>(acq_parameters_.resampler_ratio);
                }

            //--- Find number of samples per spreading code -------------------------
            code_length_ = static_cast<unsigned int>(std::floor(static_cast<double>(acq_parameters_.resampled_fs) / (GALILEO_E5A_CODE_CHIP_RATE_HZ / GALILEO_E5A_CODE_LENGTH_CHIPS)));
            acq_parameters_.samples_per_ms = static_cast<float>(acq_parameters_.resampled_fs) * 0.001;
            acq_parameters_.samples_per_chip = static_cast<unsigned int>(ceil((1.0 / GALILEO_E5A_CODE_CHIP_RATE_HZ) * static_cast<float>(acq_parameters_.resampled_fs)));
        }
    else
        {
            acq_parameters_.resampled_fs = fs_in_;
            //--- Find number of samples per spreading code -------------------------
            code_length_ = static_cast<unsigned int>(std::floor(static_cast<double>(fs_in_) / (GALILEO_E5A_CODE_CHIP_RATE_HZ / GALILEO_E5A_CODE_LENGTH_CHIPS)));
            acq_parameters_.samples_per_ms = static_cast<float>(fs_in_) * 0.001;
            acq_parameters_.samples_per_chip = static_cast<unsigned int>(ceil((1.0 / GALILEO_E5A_CODE_CHIP_RATE_HZ) * static_cast<float>(acq_parameters_.fs_in)));
        }

    //--- Find number of samples per spreading code (1ms)-------------------------
    code_length_ = static_cast<unsigned int>(std::round(static_cast<double>(fs_in_) / GALILEO_E5A_CODE_CHIP_RATE_HZ * static_cast<double>(GALILEO_E5A_CODE_LENGTH_CHIPS)));
    vector_length_ = code_length_ * sampled_ms_;

    code_ = std::vector<std::complex<float>>(vector_length_);

    if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
        }
    else if (item_type_ == "cshort")
        {
            item_size_ = sizeof(lv_16sc_t);
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
    acq_parameters_.it_size = item_size_;
    acq_parameters_.sampled_ms = sampled_ms_;
    acq_parameters_.ms_per_code = 1;
    acq_parameters_.samples_per_code = acq_parameters_.samples_per_ms * static_cast<float>(GALILEO_E5A_CODE_PERIOD_MS);
    acq_parameters_.num_doppler_bins_step2 = configuration_->property(role + ".second_nbins", 4);
    acq_parameters_.doppler_step2 = configuration_->property(role + ".second_doppler_step", 125.0);
    acq_parameters_.make_2_steps = configuration_->property(role + ".make_two_steps", false);
    acq_parameters_.blocking_on_standby = configuration_->property(role + ".blocking_on_standby", false);
    acquisition_ = pcps_make_acquisition(acq_parameters_);

    channel_ = 0;
    threshold_ = 0.0;
    doppler_step_ = 0;
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
    auto lambda = double(vector_length_);
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
    if (item_type_ == "gr_complex")
        {
            // nothing to connect
        }
    else if (item_type_ == "cshort")
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
    if (item_type_ == "gr_complex")
        {
            // nothing to disconnect
        }
    else if (item_type_ == "cshort")
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
