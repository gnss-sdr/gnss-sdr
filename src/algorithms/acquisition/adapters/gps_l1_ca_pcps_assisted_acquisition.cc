/*!
 * \file gps_l1_ca_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L1 C/A Signals
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#include "gps_l1_ca_pcps_assisted_acquisition.h"
#include <glog/logging.h>
#include "gps_sdr_signal_processing.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"


using google::LogMessage;

GpsL1CaPcpsAssistedAcquisition::GpsL1CaPcpsAssistedAcquisition(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./data/acquisition.dat";

    DLOG(INFO) << "role " << role;

    item_type_ = configuration->property(role + ".item_type", default_item_type);
    long fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in_ = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    if_ = configuration->property(role + ".if", 0);
    dump_ = configuration->property(role + ".dump", false);
    doppler_max_ = configuration->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0) doppler_max_ = FLAGS_doppler_max;
    doppler_min_ = configuration->property(role + ".doppler_min", -doppler_max_);
    sampled_ms_ = configuration->property(role + ".coherent_integration_time_ms", 1);
    max_dwells_ = configuration->property(role + ".max_dwells", 1);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);

    //--- Find number of samples per spreading code -------------------------
    vector_length_ = round(fs_in_ / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));

    code_ = new gr_complex[vector_length_];

    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            acquisition_cc_ = pcps_make_assisted_acquisition_cc(max_dwells_, sampled_ms_,
                doppler_max_, doppler_min_, if_, fs_in_, vector_length_,
                dump_, dump_filename_);
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


GpsL1CaPcpsAssistedAcquisition::~GpsL1CaPcpsAssistedAcquisition()
{
    delete[] code_;
}


void GpsL1CaPcpsAssistedAcquisition::set_channel(unsigned int channel)
{
    channel_ = channel;
    acquisition_cc_->set_channel(channel_);
}


void GpsL1CaPcpsAssistedAcquisition::set_threshold(float threshold)
{
    threshold_ = threshold;
    acquisition_cc_->set_threshold(threshold_);
}


void GpsL1CaPcpsAssistedAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;
    acquisition_cc_->set_doppler_max(doppler_max_);
}


void GpsL1CaPcpsAssistedAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    acquisition_cc_->set_doppler_step(doppler_step_);
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
    //set_local_code();
}

void GpsL1CaPcpsAssistedAcquisition::set_local_code()
{
    gps_l1_ca_code_gen_complex_sampled(code_, gnss_synchro_->PRN, fs_in_, 0);
    acquisition_cc_->set_local_code(code_);
}

void GpsL1CaPcpsAssistedAcquisition::reset()
{
    acquisition_cc_->set_active(true);
}


void GpsL1CaPcpsAssistedAcquisition::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}


void GpsL1CaPcpsAssistedAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GpsL1CaPcpsAssistedAcquisition::get_left_block()
{
    return acquisition_cc_;
}


gr::basic_block_sptr GpsL1CaPcpsAssistedAcquisition::get_right_block()
{
    return acquisition_cc_;
}
