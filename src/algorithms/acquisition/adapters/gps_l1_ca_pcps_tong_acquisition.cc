/*!
 * \file gps_l1_ca_pcps_tong_acquisition.cc
 * \brief Adapts a PCPS Tong acquisition block to an AcquisitionInterface for
 *  GPS L1 C/A signals
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

#include "gps_l1_ca_pcps_tong_acquisition.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gps_sdr_signal_replica.h"
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

GpsL1CaPcpsTongAcquisition::GpsL1CaPcpsTongAcquisition(
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
      doppler_max_(configuration->property(role + ".doppler_max", 5000)),
      doppler_step_(0),
      sampled_ms_(configuration_->property(role + ".coherent_integration_time_ms", 1)),
      tong_init_val_(configuration->property(role + ".tong_init_val", 1)),
      tong_max_val_(configuration->property(role + ".tong_max_val", 2)),
      tong_max_dwells_(configuration->property(role + ".tong_max_dwells", tong_max_val_ + 1)),
      in_streams_(in_streams),
      out_streams_(out_streams),
      dump_(configuration_->property(role + ".dump", false))
{
    const std::string default_item_type("gr_complex");
    std::string default_dump_filename = "./data/acquisition.dat";

    DLOG(INFO) << "role " << role_;

    item_type_ = configuration_->property(role_ + ".item_type", default_item_type);
    int64_t fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    dump_filename_ = configuration_->property(role_ + ".dump_filename", default_dump_filename);

    if (FLAGS_doppler_max != 0)
        {
            doppler_max_ = FLAGS_doppler_max;
        }
    bool enable_monitor_output = configuration_->property("AcquisitionMonitor.enable_monitor", false);

    // -- Find number of samples per spreading code -------------------------
    code_length_ = static_cast<unsigned int>(round(fs_in_ / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));

    vector_length_ = code_length_ * sampled_ms_;

    code_ = std::vector<std::complex<float>>(vector_length_);

    if (item_type_ == "gr_complex")
        {
            acquisition_cc_ = pcps_tong_make_acquisition_cc(sampled_ms_, doppler_max_, fs_in_,
                code_length_, code_length_, tong_init_val_, tong_max_val_, tong_max_dwells_,
                dump_, dump_filename_, enable_monitor_output);

            stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_, vector_length_);

            DLOG(INFO) << "stream_to_vector(" << stream_to_vector_->unique_id() << ")";
            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id() << ")";
        }
    else
        {
            acquisition_cc_ = nullptr;
            item_size_ = 0;
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


void GpsL1CaPcpsTongAcquisition::stop_acquisition()
{
    acquisition_cc_->set_state(0);
    acquisition_cc_->set_active(false);
}


void GpsL1CaPcpsTongAcquisition::set_threshold(float threshold)
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

    DLOG(INFO) << "Channel " << channel_ << "  Threshold = " << threshold_;

    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_threshold(threshold_);
        }
}


void GpsL1CaPcpsTongAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_doppler_max(doppler_max_);
        }
}


void GpsL1CaPcpsTongAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_doppler_step(doppler_step_);
        }
}


void GpsL1CaPcpsTongAcquisition::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_gnss_synchro(gnss_synchro_);
        }
}


signed int GpsL1CaPcpsTongAcquisition::mag()
{
    if (item_type_ == "gr_complex")
        {
            return acquisition_cc_->mag();
        }
    return 0;
}


void GpsL1CaPcpsTongAcquisition::init()
{
    acquisition_cc_->init();
}


void GpsL1CaPcpsTongAcquisition::set_local_code()
{
    if (item_type_ == "gr_complex")
        {
            std::vector<std::complex<float>> code(code_length_);

            gps_l1_ca_code_gen_complex_sampled(code, gnss_synchro_->PRN, fs_in_, 0);

            own::span<gr_complex> code_span(code_.data(), vector_length_);
            for (unsigned int i = 0; i < sampled_ms_; i++)
                {
                    std::copy_n(code.data(), code_length_, code_span.subspan(i * code_length_, code_length_).data());
                }

            acquisition_cc_->set_local_code(code_.data());
        }
}


void GpsL1CaPcpsTongAcquisition::reset()
{
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_active(true);
        }
}


void GpsL1CaPcpsTongAcquisition::set_state(int state)
{
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_state(state);
        }
}


float GpsL1CaPcpsTongAcquisition::calculate_threshold(float pfa) const
{
    // Calculate the threshold
    unsigned int frequency_bins = 0;
    for (int doppler = static_cast<int>(-doppler_max_); doppler <= static_cast<int>(doppler_max_); doppler += static_cast<int>(doppler_step_))
        {
            frequency_bins++;
        }

    DLOG(INFO) << "Channel " << channel_ << "   Pfa = " << pfa;

    unsigned int ncells = vector_length_ * frequency_bins;
    double exponent = 1 / static_cast<double>(ncells);
    double val = pow(1.0 - pfa, exponent);
    auto lambda = static_cast<double>(vector_length_);
    boost::math::exponential_distribution<double> mydist(lambda);
    auto threshold = static_cast<float>(quantile(mydist, val));

    return threshold;
}


void GpsL1CaPcpsTongAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
        {
            top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


void GpsL1CaPcpsTongAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


gr::basic_block_sptr GpsL1CaPcpsTongAcquisition::get_left_block()
{
    return stream_to_vector_;
}


gr::basic_block_sptr GpsL1CaPcpsTongAcquisition::get_right_block()
{
    return acquisition_cc_;
}
