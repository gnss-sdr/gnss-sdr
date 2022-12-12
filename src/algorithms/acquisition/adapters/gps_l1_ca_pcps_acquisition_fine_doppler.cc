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

#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "GPS_L1_CA.h"
#include "acq_conf.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gps_sdr_signal_replica.h"
#include <glog/logging.h>


GpsL1CaPcpsAcquisitionFineDoppler::GpsL1CaPcpsAcquisitionFineDoppler(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : role_(role),
      gnss_synchro_(nullptr),
      item_size_(sizeof(gr_complex)),
      threshold_(0.0),
      doppler_max_(configuration->property(role + ".doppler_max", 5000)),
      max_dwells_(configuration->property(role + ".max_dwells", 1)),
      channel_(0),
      doppler_step_(0),
      sampled_ms_(configuration->property(role + ".coherent_integration_time_ms", 1)),
      in_streams_(in_streams),
      out_streams_(out_streams),
      dump_(configuration->property(role + ".dump", false))
{
    const std::string default_item_type("gr_complex");
    std::string default_dump_filename = "./acquisition.mat";
    Acq_Conf acq_parameters = Acq_Conf();

    item_type_ = configuration->property(role_ + ".item_type", default_item_type);
    int64_t fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in_ = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    acq_parameters.fs_in = fs_in_;
    acq_parameters.samples_per_chip = static_cast<unsigned int>(ceil(GPS_L1_CA_CHIP_PERIOD_S * static_cast<float>(acq_parameters.fs_in)));
    acq_parameters.dump = dump_;
    dump_filename_ = configuration->property(role_ + ".dump_filename", default_dump_filename);
    acq_parameters.dump_filename = dump_filename_;
    if (FLAGS_doppler_max != 0)
        {
            doppler_max_ = FLAGS_doppler_max;
        }
    acq_parameters.doppler_max = doppler_max_;
    acq_parameters.sampled_ms = sampled_ms_;
    acq_parameters.max_dwells = max_dwells_;
    acq_parameters.blocking_on_standby = configuration->property(role + ".blocking_on_standby", false);

    // -- Find number of samples per spreading code -------------------------
    vector_length_ = static_cast<unsigned int>(round(fs_in_ / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));
    acq_parameters.samples_per_ms = static_cast<float>(vector_length_);
    code_ = std::vector<std::complex<float>>(vector_length_);

    DLOG(INFO) << "role " << role_;
    if (item_type_ == "gr_complex")
        {
            acquisition_cc_ = pcps_make_acquisition_fine_doppler_cc(acq_parameters);
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


void GpsL1CaPcpsAcquisitionFineDoppler::stop_acquisition()
{
    acquisition_cc_->set_state(0);
    acquisition_cc_->set_active(false);
}


void GpsL1CaPcpsAcquisitionFineDoppler::set_threshold(float threshold)
{
    threshold_ = threshold;
    acquisition_cc_->set_threshold(threshold_);
}


void GpsL1CaPcpsAcquisitionFineDoppler::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = static_cast<int>(doppler_max);
    acquisition_cc_->set_doppler_max(doppler_max_);
}


void GpsL1CaPcpsAcquisitionFineDoppler::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    acquisition_cc_->set_doppler_step(doppler_step_);
}


void GpsL1CaPcpsAcquisitionFineDoppler::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    acquisition_cc_->set_gnss_synchro(gnss_synchro_);
}


signed int GpsL1CaPcpsAcquisitionFineDoppler::mag()
{
    return static_cast<signed int>(acquisition_cc_->mag());
}


void GpsL1CaPcpsAcquisitionFineDoppler::init()
{
    acquisition_cc_->init();
}


void GpsL1CaPcpsAcquisitionFineDoppler::set_local_code()
{
    gps_l1_ca_code_gen_complex_sampled(code_, gnss_synchro_->PRN, fs_in_, 0);
    acquisition_cc_->set_local_code(code_.data());
}


void GpsL1CaPcpsAcquisitionFineDoppler::reset()
{
    acquisition_cc_->set_active(true);
}


void GpsL1CaPcpsAcquisitionFineDoppler::set_state(int state)
{
    acquisition_cc_->set_state(state);
}


void GpsL1CaPcpsAcquisitionFineDoppler::connect(gnss_shared_ptr<gr::top_block> top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


void GpsL1CaPcpsAcquisitionFineDoppler::disconnect(gnss_shared_ptr<gr::top_block> top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gnss_shared_ptr<gr::basic_block> GpsL1CaPcpsAcquisitionFineDoppler::get_left_block()
{
    return acquisition_cc_;
}


gnss_shared_ptr<gr::basic_block> GpsL1CaPcpsAcquisitionFineDoppler::get_right_block()
{
    return acquisition_cc_;
}
