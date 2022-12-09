/*!
 * \file galileo_e1_pcps_8ms_ambiguous_acquisition.cc
 * \brief Adapts a Galileo PCPS 8ms acquisition block to an
 * AcquisitionInterface for Galileo E1 Signals
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
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

#include "galileo_e1_pcps_8ms_ambiguous_acquisition.h"
#include "Galileo_E1.h"
#include "configuration_interface.h"
#include "galileo_e1_signal_replica.h"
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

GalileoE1Pcps8msAmbiguousAcquisition::GalileoE1Pcps8msAmbiguousAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : configuration_(configuration),
      gnss_synchro_(nullptr),
      role_(role),
      item_size_(sizeof(gr_complex)),
      threshold_(0.0),
      channel_(0),
      doppler_max_(configuration_->property(role + ".doppler_max", 5000)),
      doppler_step_(0),
      sampled_ms_(configuration_->property(role + ".coherent_integration_time_ms", 4)),
      max_dwells_(configuration_->property(role + ".max_dwells", 1)),
      in_streams_(in_streams),
      out_streams_(out_streams),
      dump_(configuration_->property(role + ".dump", false))
{
    const std::string default_item_type("gr_complex");
    const std::string default_dump_filename("../data/acquisition.dat");
    item_type_ = configuration_->property(role_ + ".item_type", default_item_type);
    int64_t fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 4000000);
    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    dump_filename_ = configuration_->property(role_ + ".dump_filename", default_dump_filename);

    if (FLAGS_doppler_max != 0)
        {
            doppler_max_ = FLAGS_doppler_max;
        }

    if (sampled_ms_ % 4 != 0)
        {
            sampled_ms_ = static_cast<int>(sampled_ms_ / 4) * 4;
            LOG(WARNING) << "coherent_integration_time should be multiple of "
                         << "Galileo code length (4 ms). coherent_integration_time = "
                         << sampled_ms_ << " ms will be used.";
        }

    // -- Find number of samples per spreading code (4 ms)  -----------------
    code_length_ = static_cast<unsigned int>(round(
        fs_in_ / (GALILEO_E1_CODE_CHIP_RATE_CPS / GALILEO_E1_B_CODE_LENGTH_CHIPS)));

    vector_length_ = code_length_ * static_cast<int>(sampled_ms_ / 4);

    auto samples_per_ms = static_cast<int>(code_length_) / 4;

    code_ = std::vector<std::complex<float>>(vector_length_);

    bool enable_monitor_output = configuration->property("AcquisitionMonitor.enable_monitor", false);

    DLOG(INFO) << "role " << role_;
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_ = galileo_pcps_8ms_make_acquisition_cc(sampled_ms_, max_dwells_,
                doppler_max_, fs_in_, samples_per_ms, code_length_,
                dump_, dump_filename_, enable_monitor_output);
            stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_, vector_length_);
            DLOG(INFO) << "stream_to_vector("
                       << stream_to_vector_->unique_id() << ")";
            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id()
                       << ")";
        }
    else
        {
            item_size_ = 0;
            acquisition_cc_ = nullptr;
            stream_to_vector_ = nullptr;
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


void GalileoE1Pcps8msAmbiguousAcquisition::stop_acquisition()
{
    acquisition_cc_->set_active(false);
}


void GalileoE1Pcps8msAmbiguousAcquisition::set_threshold(float threshold)
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


void GalileoE1Pcps8msAmbiguousAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_doppler_max(doppler_max_);
        }
}


void GalileoE1Pcps8msAmbiguousAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_doppler_step(doppler_step_);
        }
}


void GalileoE1Pcps8msAmbiguousAcquisition::set_gnss_synchro(
    Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_gnss_synchro(gnss_synchro_);
        }
}


signed int GalileoE1Pcps8msAmbiguousAcquisition::mag()
{
    if (item_type_ == "gr_complex")
        {
            return acquisition_cc_->mag();
        }
    return 0;
}


void GalileoE1Pcps8msAmbiguousAcquisition::init()
{
    acquisition_cc_->init();
}


void GalileoE1Pcps8msAmbiguousAcquisition::set_local_code()
{
    if (item_type_ == "gr_complex")
        {
            bool cboc = configuration_->property(
                "Acquisition" + std::to_string(channel_) + ".cboc", false);

            std::vector<std::complex<float>> code(code_length_);
            std::array<char, 3> Signal_{};
            Signal_[0] = gnss_synchro_->Signal[0];
            Signal_[1] = gnss_synchro_->Signal[1];
            Signal_[2] = '\0';

            galileo_e1_code_gen_complex_sampled(code, Signal_,
                cboc, gnss_synchro_->PRN, fs_in_, 0, false);

            own::span<gr_complex> code_span(code_.data(), vector_length_);
            for (unsigned int i = 0; i < sampled_ms_ / 4; i++)
                {
                    std::copy_n(code.data(), code_length_, code_span.subspan(i * code_length_, code_length_).data());
                }

            acquisition_cc_->set_local_code(code_.data());
        }
}


void GalileoE1Pcps8msAmbiguousAcquisition::reset()
{
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_active(true);
        }
}


float GalileoE1Pcps8msAmbiguousAcquisition::calculate_threshold(float pfa) const
{
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


void GalileoE1Pcps8msAmbiguousAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
        {
            top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


void GalileoE1Pcps8msAmbiguousAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


gr::basic_block_sptr GalileoE1Pcps8msAmbiguousAcquisition::get_left_block()
{
    return stream_to_vector_;
}


gr::basic_block_sptr GalileoE1Pcps8msAmbiguousAcquisition::get_right_block()
{
    return acquisition_cc_;
}
