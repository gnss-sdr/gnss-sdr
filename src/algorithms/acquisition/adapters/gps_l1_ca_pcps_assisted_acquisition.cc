/*!
 * \file gps_l1_ca_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L1 C/A Signals
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#include "gps_l1_ca_pcps_assisted_acquisition.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gps_sdr_signal_replica.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GpsL1CaPcpsAssistedAcquisition::GpsL1CaPcpsAssistedAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : role_(role),
      gnss_synchro_(nullptr),
      item_size_(sizeof(gr_complex)),
      dump_(configuration->property(role + ".dump", false))
{
    const std::string default_item_type("gr_complex");
    const std::string default_dump_filename = "./data/acquisition.dat";
    const std::string dump_filename = configuration->property(role_ + ".dump_filename", default_dump_filename);
    const std::string item_type = configuration->property(role_ + ".item_type", default_item_type);
    const int64_t fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in_ = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);

    int doppler_max = configuration->property(role + ".doppler_max", 5000);

#if USE_GLOG_AND_GFLAGS
    if (FLAGS_doppler_max != 0)
        {
            doppler_max = FLAGS_doppler_max;
        }
#else
    if (absl::GetFlag(FLAGS_doppler_max) != 0)
        {
            doppler_max = absl::GetFlag(FLAGS_doppler_max);
        }
#endif
    const int doppler_min = configuration->property(role_ + ".doppler_min", -doppler_max);
    bool enable_monitor_output = configuration->property("AcquisitionMonitor.enable_monitor", false);

    // --- Find number of samples per spreading code -------------------------
    vector_length_ = static_cast<unsigned int>(round(fs_in_ / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));

    code_ = std::vector<std::complex<float>>(vector_length_);

    DLOG(INFO) << "role " << role_;
    if (item_type == "gr_complex")
        {
            const unsigned int max_dwells = configuration->property(role + ".max_dwells", 1);
            const unsigned int sampled_ms = configuration->property(role + ".coherent_integration_time_ms", 1);
            acquisition_cc_ = pcps_make_assisted_acquisition_cc(max_dwells, sampled_ms,
                doppler_max, doppler_min, fs_in_, vector_length_,
                dump_, dump_filename, enable_monitor_output);
        }
    else
        {
            item_size_ = 0;
            acquisition_cc_ = nullptr;
            LOG(WARNING) << item_type << " unknown acquisition item type";
        }

    if (in_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


void GpsL1CaPcpsAssistedAcquisition::stop_acquisition()
{
    acquisition_cc_->set_active(false);
    acquisition_cc_->set_state(0);
}


void GpsL1CaPcpsAssistedAcquisition::set_threshold(float threshold)
{
    acquisition_cc_->set_threshold(threshold);
}


void GpsL1CaPcpsAssistedAcquisition::set_doppler_step(unsigned int doppler_step)
{
    acquisition_cc_->set_doppler_step(doppler_step);
}


void GpsL1CaPcpsAssistedAcquisition::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    acquisition_cc_->set_gnss_synchro(gnss_synchro_);
}


signed int GpsL1CaPcpsAssistedAcquisition::mag()
{
    return acquisition_cc_->mag();
}


void GpsL1CaPcpsAssistedAcquisition::init()
{
    acquisition_cc_->init();
}


void GpsL1CaPcpsAssistedAcquisition::set_local_code()
{
    gps_l1_ca_code_gen_complex_sampled(code_, gnss_synchro_->PRN, fs_in_, 0);
    acquisition_cc_->set_local_code(code_.data());
}


void GpsL1CaPcpsAssistedAcquisition::reset()
{
    acquisition_cc_->set_active(true);
}


void GpsL1CaPcpsAssistedAcquisition::connect(gr::top_block_sptr /*top_block*/)
{
}


void GpsL1CaPcpsAssistedAcquisition::disconnect(gr::top_block_sptr /*top_block*/)
{
}


gr::basic_block_sptr GpsL1CaPcpsAssistedAcquisition::get_left_block()
{
    return acquisition_cc_;
}


gr::basic_block_sptr GpsL1CaPcpsAssistedAcquisition::get_right_block()
{
    return acquisition_cc_;
}
