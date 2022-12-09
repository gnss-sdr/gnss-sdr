/*!
 * \file galileo_e1_pcps_cccwsr_ambiguous_acquisition.cc
 * \brief Adapts a PCPS CCCWSR acquisition block to an AcquisitionInterface for
 *  Galileo E1 Signals
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

#include "galileo_e1_pcps_cccwsr_ambiguous_acquisition.h"
#include "Galileo_E1.h"
#include "configuration_interface.h"
#include "galileo_e1_signal_replica.h"
#include "gnss_sdr_flags.h"
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>


GalileoE1PcpsCccwsrAmbiguousAcquisition::GalileoE1PcpsCccwsrAmbiguousAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : configuration_(configuration),
      role_(role),
      gnss_synchro_(nullptr),
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

    code_data_ = std::vector<std::complex<float>>(vector_length_);
    code_pilot_ = std::vector<std::complex<float>>(vector_length_);

    bool enable_monitor_output = configuration_->property("AcquisitionMonitor.enable_monitor", false);

    DLOG(INFO) << "role " << role_;
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_ = pcps_cccwsr_make_acquisition_cc(sampled_ms_, max_dwells_,
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


void GalileoE1PcpsCccwsrAmbiguousAcquisition::stop_acquisition()
{
    acquisition_cc_->set_state(0);
    acquisition_cc_->set_active(false);
}


void GalileoE1PcpsCccwsrAmbiguousAcquisition::set_threshold(float threshold)
{
    threshold_ = threshold;

    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold_;

    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_threshold(threshold_);
        }
}


void GalileoE1PcpsCccwsrAmbiguousAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_doppler_max(doppler_max_);
        }
}


void GalileoE1PcpsCccwsrAmbiguousAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_doppler_step(doppler_step_);
        }
}


void GalileoE1PcpsCccwsrAmbiguousAcquisition::set_gnss_synchro(
    Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_gnss_synchro(gnss_synchro_);
        }
}


signed int GalileoE1PcpsCccwsrAmbiguousAcquisition::mag()
{
    if (item_type_ == "gr_complex")
        {
            return acquisition_cc_->mag();
        }
    return 0;
}


void GalileoE1PcpsCccwsrAmbiguousAcquisition::init()
{
    acquisition_cc_->init();
}


void GalileoE1PcpsCccwsrAmbiguousAcquisition::set_local_code()
{
    if (item_type_ == "gr_complex")
        {
            bool cboc = configuration_->property(
                "Acquisition" + std::to_string(channel_) + ".cboc", false);

            std::array<char, 3> signal = {{'1', 'B', '\0'}};

            galileo_e1_code_gen_complex_sampled(code_data_, signal,
                cboc, gnss_synchro_->PRN, fs_in_, 0, false);

            std::array<char, 3> signal_C = {{'1', 'C', '\0'}};

            galileo_e1_code_gen_complex_sampled(code_pilot_, signal_C,
                cboc, gnss_synchro_->PRN, fs_in_, 0, false);

            acquisition_cc_->set_local_code(code_data_.data(), code_pilot_.data());
        }
}


void GalileoE1PcpsCccwsrAmbiguousAcquisition::reset()
{
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_->set_active(true);
        }
}


void GalileoE1PcpsCccwsrAmbiguousAcquisition::set_state(int state)
{
    acquisition_cc_->set_state(state);
}


float GalileoE1PcpsCccwsrAmbiguousAcquisition::calculate_threshold(float pfa)
{
    if (pfa > 0.0)
        { /* Not implemented*/
        };
    return 0.0;
}


void GalileoE1PcpsCccwsrAmbiguousAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
        {
            top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


void GalileoE1PcpsCccwsrAmbiguousAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


gr::basic_block_sptr GalileoE1PcpsCccwsrAmbiguousAcquisition::get_left_block()
{
    return stream_to_vector_;
}


gr::basic_block_sptr GalileoE1PcpsCccwsrAmbiguousAcquisition::get_right_block()
{
    return acquisition_cc_;
}
