/*!
 * \file galileo_e5a_noncoherent_iq_acquisition_caf.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          </ul>
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

#include "galileo_e5a_noncoherent_iq_acquisition_caf.h"
#include <boost/lexical_cast.hpp>
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include "galileo_e5_signal_processing.h"
#include "Galileo_E5a.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"

using google::LogMessage;

void GalileoE5aNoncoherentIQAcquisitionCaf::stop_acquisition()
{
}

GalileoE5aNoncoherentIQAcquisitionCaf::GalileoE5aNoncoherentIQAcquisitionCaf(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    configuration_ = configuration;
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "../data/acquisition.dat";

    DLOG(INFO) << "role " << role;

    item_type_ = configuration_->property(role + ".item_type", default_item_type);

    long fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 32000000);
    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    dump_ = configuration_->property(role + ".dump", false);
    doppler_max_ = configuration_->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0) doppler_max_ = FLAGS_doppler_max;
    CAF_window_hz_ = configuration_->property(role + ".CAF_window_hz", 0);
    Zero_padding = configuration_->property(role + ".Zero_padding", 0);
    sampled_ms_ = configuration_->property(role + ".coherent_integration_time_ms", 1);
    if (sampled_ms_ > 3)
        {
            sampled_ms_ = 3;
            DLOG(INFO) << "Coherent integration time should be 3 ms or less. Changing to 3ms ";
            std::cout << "Too high coherent integration time. Changing to 3ms" << std::endl;
        }
    if (Zero_padding > 0)
        {
            sampled_ms_ = 2;
            DLOG(INFO) << "Zero padding activated. Changing to 1ms code + 1ms zero padding ";
            std::cout << "Zero padding activated. Changing to 1ms code + 1ms zero padding" << std::endl;
        }

    max_dwells_ = configuration_->property(role + ".max_dwells", 1);
    dump_filename_ = configuration_->property(role + ".dump_filename", default_dump_filename);
    bit_transition_flag_ = configuration_->property(role + ".bit_transition_flag", false);

    //--- Find number of samples per spreading code (1ms)-------------------------
    code_length_ = round(static_cast<double>(fs_in_) / Galileo_E5a_CODE_CHIP_RATE_HZ * static_cast<double>(Galileo_E5a_CODE_LENGTH_CHIPS));

    vector_length_ = code_length_ * sampled_ms_;

    codeI_ = new gr_complex[vector_length_];
    codeQ_ = new gr_complex[vector_length_];
    both_signal_components = false;

    std::string sig_ = configuration_->property("Channel.signal", std::string("5X"));
    if (sig_.at(0) == '5' && sig_.at(1) == 'X')
        {
            both_signal_components = true;
        }
    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            acquisition_cc_ = galileo_e5a_noncoherentIQ_make_acquisition_caf_cc(sampled_ms_, max_dwells_,
                doppler_max_, fs_in_, code_length_, code_length_, bit_transition_flag_,
                dump_, dump_filename_, both_signal_components, CAF_window_hz_, Zero_padding);
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
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


GalileoE5aNoncoherentIQAcquisitionCaf::~GalileoE5aNoncoherentIQAcquisitionCaf()
{
    delete[] codeI_;
    delete[] codeQ_;
}


void GalileoE5aNoncoherentIQAcquisitionCaf::set_channel(unsigned int channel)
{
    channel_ = channel;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_channel(channel_);
        }
}


void GalileoE5aNoncoherentIQAcquisitionCaf::set_threshold(float threshold)
{
    float pfa = configuration_->property(role_ + boost::lexical_cast<std::string>(channel_) + ".pfa", 0.0);

    if (pfa == 0.0) pfa = configuration_->property(role_ + ".pfa", 0.0);

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


void GalileoE5aNoncoherentIQAcquisitionCaf::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_doppler_max(doppler_max_);
        }
}


void GalileoE5aNoncoherentIQAcquisitionCaf::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_doppler_step(doppler_step_);
        }
}


void GalileoE5aNoncoherentIQAcquisitionCaf::set_gnss_synchro(
    Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_gnss_synchro(gnss_synchro_);
        }
}


signed int GalileoE5aNoncoherentIQAcquisitionCaf::mag()
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


void GalileoE5aNoncoherentIQAcquisitionCaf::init()
{
    acquisition_cc_->init();
    //set_local_code();
}


void GalileoE5aNoncoherentIQAcquisitionCaf::set_local_code()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            std::complex<float>* codeI = new std::complex<float>[code_length_];
            std::complex<float>* codeQ = new std::complex<float>[code_length_];

            if (gnss_synchro_->Signal[0] == '5' && gnss_synchro_->Signal[1] == 'X')
                {
                    char a[3];
                    strcpy(a, "5I");
                    galileo_e5_a_code_gen_complex_sampled(codeI, a,
                        gnss_synchro_->PRN, fs_in_, 0);

                    strcpy(a, "5Q");
                    galileo_e5_a_code_gen_complex_sampled(codeQ, a,
                        gnss_synchro_->PRN, fs_in_, 0);
                }
            else
                {
                    galileo_e5_a_code_gen_complex_sampled(codeI, gnss_synchro_->Signal,
                        gnss_synchro_->PRN, fs_in_, 0);
                }
            // WARNING: 3ms are coherently integrated. Secondary sequence (1,1,1)
            // is generated, and modulated in the 'block'.
            if (Zero_padding == 0)  // if no zero_padding
                {
                    for (unsigned int i = 0; i < sampled_ms_; i++)
                        {
                            memcpy(&(codeI_[i * code_length_]), codeI,
                                sizeof(gr_complex) * code_length_);
                            if (gnss_synchro_->Signal[0] == '5' && gnss_synchro_->Signal[1] == 'X')
                                {
                                    memcpy(&(codeQ_[i * code_length_]), codeQ,
                                        sizeof(gr_complex) * code_length_);
                                }
                        }
                }
            else
                {
                    // 1ms code + 1ms zero padding
                    memcpy(&(codeI_[0]), codeI,
                        sizeof(gr_complex) * code_length_);
                    if (gnss_synchro_->Signal[0] == '5' && gnss_synchro_->Signal[1] == 'X')
                        {
                            memcpy(&(codeQ_[0]), codeQ,
                                sizeof(gr_complex) * code_length_);
                        }
                }

            acquisition_cc_->set_local_code(codeI_, codeQ_);
            delete[] codeI;
            delete[] codeQ;
        }
}


void GalileoE5aNoncoherentIQAcquisitionCaf::reset()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_active(true);
        }
}


float GalileoE5aNoncoherentIQAcquisitionCaf::calculate_threshold(float pfa)
{
    //Calculate the threshold
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


void GalileoE5aNoncoherentIQAcquisitionCaf::set_state(int state)
{
    acquisition_cc_->set_state(state);
}


void GalileoE5aNoncoherentIQAcquisitionCaf::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect internally
}


void GalileoE5aNoncoherentIQAcquisitionCaf::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect internally
}


gr::basic_block_sptr GalileoE5aNoncoherentIQAcquisitionCaf::get_left_block()
{
    return acquisition_cc_;
}


gr::basic_block_sptr GalileoE5aNoncoherentIQAcquisitionCaf::get_right_block()
{
    return acquisition_cc_;
}
