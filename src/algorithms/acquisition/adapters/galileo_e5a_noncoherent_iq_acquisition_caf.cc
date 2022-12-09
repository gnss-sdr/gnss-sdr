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

#include "galileo_e5a_noncoherent_iq_acquisition_caf.h"
#include "Galileo_E5a.h"
#include "configuration_interface.h"
#include "galileo_e5_signal_replica.h"
#include "gnss_sdr_flags.h"
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include <algorithm>

#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl/gsl-lite.hpp>
namespace own = gsl;
#endif

GalileoE5aNoncoherentIQAcquisitionCaf::GalileoE5aNoncoherentIQAcquisitionCaf(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : configuration_(configuration),
      role_(role),
      gnss_synchro_(nullptr),
      item_size_(sizeof(gr_complex)),
      threshold_(0.0),
      Zero_padding(configuration_->property(role + ".Zero_padding", 0)),
      CAF_window_hz_(configuration_->property(role + ".CAF_window_hz", 0)),
      channel_(0),
      doppler_max_(configuration_->property(role + ".doppler_max", 5000)),
      doppler_step_(0),
      sampled_ms_(configuration_->property(role + ".coherent_integration_time_ms", 1)),
      max_dwells_(configuration_->property(role + ".max_dwells", 1)),
      in_streams_(in_streams),
      out_streams_(out_streams),
      bit_transition_flag_(configuration_->property(role + ".bit_transition_flag", false)),
      dump_(configuration_->property(role + ".dump", false))
{
    const std::string default_item_type("gr_complex");
    const std::string default_dump_filename("../data/acquisition.dat");
    item_type_ = configuration_->property(role_ + ".item_type", default_item_type);
    dump_filename_ = configuration_->property(role_ + ".dump_filename", default_dump_filename);
    int64_t fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 32000000);
    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);

    if (FLAGS_doppler_max != 0)
        {
            doppler_max_ = FLAGS_doppler_max;
        }

    DLOG(INFO) << "role " << role_;
    if (sampled_ms_ > 3)
        {
            sampled_ms_ = 3;
            DLOG(INFO) << "Coherent integration time should be 3 ms or less. Changing to 3ms ";
            std::cout << "Too high coherent integration time. Changing to 3ms\n";
        }
    if (Zero_padding > 0)
        {
            sampled_ms_ = 2;
            DLOG(INFO) << "Zero padding activated. Changing to 1ms code + 1ms zero padding ";
            std::cout << "Zero padding activated. Changing to 1ms code + 1ms zero padding\n";
        }

    // -- Find number of samples per spreading code (1ms)-------------------------
    code_length_ = static_cast<int>(round(static_cast<double>(fs_in_) / GALILEO_E5A_CODE_CHIP_RATE_CPS * static_cast<double>(GALILEO_E5A_CODE_LENGTH_CHIPS)));

    vector_length_ = code_length_ * sampled_ms_;

    codeI_ = std::vector<std::complex<float>>(vector_length_);
    codeQ_ = std::vector<std::complex<float>>(vector_length_);
    both_signal_components = false;

    bool enable_monitor_output = configuration->property("AcquisitionMonitor.enable_monitor", false);

    std::string sig_ = configuration_->property("Channel.signal", std::string("5X"));
    if (sig_.at(0) == '5' && sig_.at(1) == 'X')
        {
            both_signal_components = true;
        }
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_ = galileo_e5a_noncoherentIQ_make_acquisition_caf_cc(sampled_ms_, max_dwells_,
                doppler_max_, fs_in_, code_length_, code_length_, bit_transition_flag_,
                dump_, dump_filename_, both_signal_components, CAF_window_hz_, Zero_padding, enable_monitor_output);
        }
    else
        {
            item_size_ = 0;
            acquisition_cc_ = nullptr;
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


void GalileoE5aNoncoherentIQAcquisitionCaf::stop_acquisition()
{
    acquisition_cc_->set_state(0);
    acquisition_cc_->set_active(false);
}


void GalileoE5aNoncoherentIQAcquisitionCaf::set_threshold(float threshold)
{
    float pfa = configuration_->property(role_ + std::to_string(channel_) + ".pfa", static_cast<float>(0.0));

    if (pfa == 0.0)
        {
            pfa = configuration_->property(role_ + ".pfa", static_cast<float>(0.0));
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

    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_threshold(threshold_);
        }
}


void GalileoE5aNoncoherentIQAcquisitionCaf::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_doppler_max(doppler_max_);
        }
}


void GalileoE5aNoncoherentIQAcquisitionCaf::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_doppler_step(doppler_step_);
        }
}


void GalileoE5aNoncoherentIQAcquisitionCaf::set_gnss_synchro(
    Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_gnss_synchro(gnss_synchro_);
        }
}


signed int GalileoE5aNoncoherentIQAcquisitionCaf::mag()
{
    if (item_type_ == "gr_complex")
        {
            return static_cast<signed int>(acquisition_cc_->mag());
        }
    return 0;
}


void GalileoE5aNoncoherentIQAcquisitionCaf::init()
{
    acquisition_cc_->init();
}


void GalileoE5aNoncoherentIQAcquisitionCaf::set_local_code()
{
    if (item_type_ == "gr_complex")
        {
            std::vector<std::complex<float>> codeI(code_length_);
            std::vector<std::complex<float>> codeQ(code_length_);

            if (gnss_synchro_->Signal[0] == '5' && gnss_synchro_->Signal[1] == 'X')
                {
                    std::array<char, 3> a = {{'5', 'I', '\0'}};
                    galileo_e5_a_code_gen_complex_sampled(codeI,
                        gnss_synchro_->PRN, a, fs_in_, 0);

                    std::array<char, 3> b = {{'5', 'Q', '\0'}};
                    galileo_e5_a_code_gen_complex_sampled(codeQ,
                        gnss_synchro_->PRN, b, fs_in_, 0);
                }
            else
                {
                    std::array<char, 3> signal_type_ = {{'5', 'X', '\0'}};
                    galileo_e5_a_code_gen_complex_sampled(codeI,
                        gnss_synchro_->PRN, signal_type_, fs_in_, 0);
                }
            // WARNING: 3ms are coherently integrated. Secondary sequence (1,1,1)
            // is generated, and modulated in the 'block'.
            own::span<gr_complex> codeQ_span(codeQ_.data(), vector_length_);
            own::span<gr_complex> codeI_span(codeI_.data(), vector_length_);
            if (Zero_padding == 0)  // if no zero_padding
                {
                    for (unsigned int i = 0; i < sampled_ms_; i++)
                        {
                            std::copy_n(codeI.data(), code_length_, codeI_span.subspan(i * code_length_, code_length_).data());
                            if (gnss_synchro_->Signal[0] == '5' && gnss_synchro_->Signal[1] == 'X')
                                {
                                    std::copy_n(codeQ.data(), code_length_, codeQ_span.subspan(i * code_length_, code_length_).data());
                                }
                        }
                }
            else
                {
                    // 1ms code + 1ms zero padding
                    std::copy_n(codeI.data(), code_length_, codeI_.data());
                    if (gnss_synchro_->Signal[0] == '5' && gnss_synchro_->Signal[1] == 'X')
                        {
                            std::copy_n(codeQ.data(), code_length_, codeQ_.data());
                        }
                }

            acquisition_cc_->set_local_code(codeI_.data(), codeQ_.data());
        }
}


void GalileoE5aNoncoherentIQAcquisitionCaf::reset()
{
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_active(true);
        }
}


float GalileoE5aNoncoherentIQAcquisitionCaf::calculate_threshold(float pfa) const
{
    // Calculate the threshold
    unsigned int frequency_bins = 0;
    for (int doppler = static_cast<int>(-doppler_max_); doppler <= static_cast<int>(doppler_max_); doppler += static_cast<int>(doppler_step_))
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
