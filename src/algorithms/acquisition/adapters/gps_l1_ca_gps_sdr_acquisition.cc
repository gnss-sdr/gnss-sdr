/*!
 * \file gps_l1_ca_gps_sdr_acquisition.cc
 * \brief Brief description of the file here
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "gps_l1_ca_gps_sdr_acquisition.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"

#include <gnuradio/gr_io_signature.h>
#include <gnuradio/gr_stream_to_vector.h>
#include <gnuradio/gr_vector_to_stream.h>
#include <gnuradio/gr_complex_to_interleaved_short.h>
#include <gnuradio/gr_interleaved_short_to_complex.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

//! Constructor
GpsL1CaGpsSdrAcquisition::GpsL1CaGpsSdrAcquisition(
        ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams,
        gr_msg_queue_sptr queue) :
    role_(role), in_streams_(in_streams), out_streams_(out_streams), queue_(
            queue)
{

    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./data/acquisition_";

    DLOG(INFO) << "role " << role;

    item_type_ = configuration->property(role + ".item_type",
            default_item_type);
    //vector_length_ = configuration->property(role + ".vector_length", 2048);

    satellite_ = 0;
    fs_in_ = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    if_ = configuration->property(role + ".ifreq", 0);
    dump_ = configuration->property(role + ".dump", false);
    doppler_max_ = 0;
    acquisition_ms_ = configuration->property(role + ".sampled_ms", 1);
    dump_filename_ = configuration->property(role + ".dump_filename",
            default_dump_filename);

    //vector_length_=ceil((float)fs_in_*((float)acquisition_ms_/1000));

    //--- Find number of samples per spreading code ----------------------------
    vector_length_ = round(fs_in_ / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));

    printf("vector_length_ %i\n\r", vector_length_);

    if (item_type_.compare("short") == 0)
    {
        item_size_ = sizeof(short);
        acquisition_ss_ = gps_l1_ca_gps_sdr_make_acquisition_ss(
                acquisition_ms_, if_, fs_in_, vector_length_, queue_, dump_,
                dump_filename_);
        stream_to_vector_ = gr_make_stream_to_vector(item_size_, 2
                * vector_length_);

    }
    else if (item_type_.compare("gr_complex") == 0)
    {
        item_size_ = sizeof(gr_complex);
        acquisition_cc_ = gps_l1_ca_gps_sdr_make_acquisition_cc(
                acquisition_ms_, if_, fs_in_, vector_length_, queue_, dump_,
                dump_filename_);
        stream_to_vector_ = gr_make_stream_to_vector(item_size_,
                vector_length_);

    }
    else
    {
        LOG_AT_LEVEL(WARNING) << item_type_ << " unknown item type.";
    }

    DLOG(INFO) << "stream_to_vector(" << stream_to_vector_->unique_id()
            << ")";

}

//! Destructor
GpsL1CaGpsSdrAcquisition::~GpsL1CaGpsSdrAcquisition()
{}

//! Set satellite
void GpsL1CaGpsSdrAcquisition::set_satellite(unsigned int satellite)
{
    satellite_ = satellite;

    if (item_type_.compare("gr_complex") == 0)
    {
        acquisition_cc_->set_satellite(satellite_);
    }
    else
    {
        acquisition_ss_->set_satellite(satellite_);
    }
}


//! Set channel
void GpsL1CaGpsSdrAcquisition::set_channel(unsigned int channel)
{
    channel_ = channel;

    if (item_type_.compare("gr_complex") == 0)
    {
        acquisition_cc_->set_channel(channel_);
    }
    else
    {
        acquisition_ss_->set_channel(channel_);
    }
}

//! Set acquisition threshold
void GpsL1CaGpsSdrAcquisition::set_threshold(float threshold)
{
    threshold_ = threshold;

    if (item_type_.compare("gr_complex") == 0)
    {
        acquisition_cc_->set_threshold(threshold_);
    }
    else
    {
        acquisition_ss_->set_threshold(threshold_);
    }
}

//! Set maximum Doppler shift
void GpsL1CaGpsSdrAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    if (item_type_.compare("gr_complex") == 0)
    {
        acquisition_cc_->set_doppler_max(doppler_max_);
    }
    else
    {
        acquisition_ss_->set_doppler_max(doppler_max_);
    }
}

//! Set Channel Queue
void GpsL1CaGpsSdrAcquisition::set_channel_queue(
        concurrent_queue<int> *channel_internal_queue)
{
    channel_internal_queue_ = channel_internal_queue;

    if (item_type_.compare("gr_complex") == 0)
    {
        acquisition_cc_->set_channel_queue(channel_internal_queue_);
    }
    else
    {
        acquisition_ss_->set_channel_queue(channel_internal_queue_);
    }
}

signed int GpsL1CaGpsSdrAcquisition::prn_code_phase()
{

    if (item_type_.compare("gr_complex") == 0)
    {
        return acquisition_cc_->prn_code_phase();
    }
    else
    {
        return acquisition_ss_->prn_code_phase();
    }
}

float GpsL1CaGpsSdrAcquisition::doppler_freq_shift()
{

    if (item_type_.compare("gr_complex") == 0)
    {
        return acquisition_cc_->doppler_freq_phase();
    }
    else
    {
        return acquisition_ss_->doppler_freq_phase();
    }
}

signed int GpsL1CaGpsSdrAcquisition::mag()
{
    if (item_type_.compare("gr_complex") == 0)
    {
        return acquisition_cc_->mag();
    }
    else
    {
        return acquisition_ss_->mag();
    }
}

void GpsL1CaGpsSdrAcquisition::reset()
{
    if (item_type_.compare("gr_complex") == 0)
    {
        acquisition_cc_->set_active(true);
    }
    else
    {
        acquisition_ss_->set_active(true);
    }
}

unsigned long int GpsL1CaGpsSdrAcquisition::get_sample_stamp()
{

    if (item_type_.compare("gr_complex") == 0)
    {
        return acquisition_cc_->get_sample_stamp();
    }
    else
    {
        return acquisition_ss_->get_sample_stamp();
    }
}

void GpsL1CaGpsSdrAcquisition::connect(gr_top_block_sptr top_block)
{

    if (item_type_.compare("gr_complex") == 0)
    {
        //top_block->connect(stream_to_vector_, 0, vector_to_stream_,0);

        top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        //top_block->connect(acquisition_cc_, 0, vector_to_stream_, 0);
    }
    else
    {
        top_block->connect(stream_to_vector_, 0, acquisition_ss_, 0);
        //top_block->connect(acquisition_ss_, 0, vector_to_stream_, 0);
    }

}

void GpsL1CaGpsSdrAcquisition::disconnect(gr_top_block_sptr top_block)
{

    if (item_type_.compare("gr_complex") == 0)
    {
        top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        //top_block->disconnect(acquisition_cc_, 0, vector_to_stream_, 0);
    }
    else
    {
        top_block->disconnect(stream_to_vector_, 0, acquisition_ss_, 0);
        //top_block->disconnect(acquisition_ss_, 0, vector_to_stream_, 0);
    }
}

gr_basic_block_sptr GpsL1CaGpsSdrAcquisition::get_left_block()
{
    return stream_to_vector_;
}

gr_basic_block_sptr GpsL1CaGpsSdrAcquisition::get_right_block()
{
    if (item_type_.compare("gr_complex") == 0)
    {
        return acquisition_cc_;
    }
    else
    {
        return acquisition_ss_;
    }

}

