/*!
 * \file gps_l1_ca_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L1 C/A signals
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena(at)gmail.com
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

#include "gps_l1_ca_pcps_acquisition.h"
#include "GPS_L1_CA.h"
#include "acq_conf.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gps_sdr_signal_replica.h"
#include <glog/logging.h>
#include <algorithm>

#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl/gsl-lite.hpp>
namespace own = gsl;
#endif

GpsL1CaPcpsAcquisition::GpsL1CaPcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : gnss_synchro_(nullptr),
                                role_(role),
                                threshold_(0.0),
                                doppler_center_(0),
                                channel_(0),
                                doppler_step_(0),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    acq_parameters_.ms_per_code = 1;
    acq_parameters_.SetFromConfiguration(configuration, role, GPS_L1_CA_CODE_RATE_CPS, GPS_L1_CA_OPT_ACQ_FS_SPS);

    DLOG(INFO) << "role " << role;

    if (FLAGS_doppler_max != 0)
        {
            acq_parameters_.doppler_max = FLAGS_doppler_max;
        }

    doppler_max_ = acq_parameters_.doppler_max;
    doppler_step_ = static_cast<unsigned int>(acq_parameters_.doppler_step);
    item_type_ = acq_parameters_.item_type;
    item_size_ = acq_parameters_.it_size;

    code_length_ = static_cast<unsigned int>(std::floor(static_cast<double>(acq_parameters_.resampled_fs) / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));
    vector_length_ = static_cast<unsigned int>(std::floor(acq_parameters_.sampled_ms * acq_parameters_.samples_per_ms) * (acq_parameters_.bit_transition_flag ? 2.0 : 1.0));
    code_ = volk_gnsssdr::vector<std::complex<float>>(vector_length_);

    sampled_ms_ = acq_parameters_.sampled_ms;

    acquisition_ = pcps_make_acquisition(acq_parameters_);
    DLOG(INFO) << "acquisition(" << acquisition_->unique_id() << ")";

    if (item_type_ == "cbyte")
        {
            cbyte_to_float_x2_ = make_complex_byte_to_float_x2();
            float_to_complex_ = gr::blocks::float_to_complex::make();
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


void GpsL1CaPcpsAcquisition::stop_acquisition()
{
    acquisition_->set_active(false);
}


void GpsL1CaPcpsAcquisition::set_threshold(float threshold)
{
    threshold_ = threshold;

    acquisition_->set_threshold(threshold_);
}


void GpsL1CaPcpsAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    acquisition_->set_doppler_max(doppler_max_);
}


void GpsL1CaPcpsAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;

    acquisition_->set_doppler_step(doppler_step_);
}


void GpsL1CaPcpsAcquisition::set_doppler_center(int doppler_center)
{
    doppler_center_ = doppler_center;

    acquisition_->set_doppler_center(doppler_center_);
}


void GpsL1CaPcpsAcquisition::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;

    acquisition_->set_gnss_synchro(gnss_synchro_);
}


signed int GpsL1CaPcpsAcquisition::mag()
{
    return acquisition_->mag();
}


void GpsL1CaPcpsAcquisition::init()
{
    acquisition_->init();
}


void GpsL1CaPcpsAcquisition::set_local_code()
{
    volk_gnsssdr::vector<std::complex<float>> code(code_length_);

    if (acq_parameters_.use_automatic_resampler)
        {
            gps_l1_ca_code_gen_complex_sampled(code, gnss_synchro_->PRN, acq_parameters_.resampled_fs, 0);
        }
    else
        {
            gps_l1_ca_code_gen_complex_sampled(code, gnss_synchro_->PRN, acq_parameters_.fs_in, 0);
        }
    own::span<gr_complex> code_span(code_.data(), vector_length_);
    for (unsigned int i = 0; i < sampled_ms_; i++)
        {
            std::copy_n(code.data(), code_length_, code_span.subspan(i * code_length_, code_length_).data());
        }

    acquisition_->set_local_code(code_.data());
}


void GpsL1CaPcpsAcquisition::reset()
{
    acquisition_->set_active(true);
}


void GpsL1CaPcpsAcquisition::set_state(int state)
{
    acquisition_->set_state(state);
}


void GpsL1CaPcpsAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex" || item_type_ == "cshort")
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
            LOG(WARNING) << item_type_ << " unknown acquisition item type: " << item_type_;
        }
}


void GpsL1CaPcpsAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex" || item_type_ == "cshort")
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
            LOG(WARNING) << item_type_ << " unknown acquisition item type" << item_type_;
        }
}


gr::basic_block_sptr GpsL1CaPcpsAcquisition::get_left_block()
{
    if (item_type_ == "gr_complex" || item_type_ == "cshort")
        {
            return acquisition_;
        }
    if (item_type_ == "cbyte")
        {
            return cbyte_to_float_x2_;
        }

    LOG(WARNING) << item_type_ << " unknown acquisition item type" << item_type_;
    return nullptr;
}


gr::basic_block_sptr GpsL1CaPcpsAcquisition::get_right_block()
{
    return acquisition_;
}


void GpsL1CaPcpsAcquisition::set_resampler_latency(uint32_t latency_samples)
{
    acquisition_->set_resampler_latency(latency_samples);
}
