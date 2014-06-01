/*!
 * \file galileo_e5a_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
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

#include "galileo_e5a_pcps_acquisition.h"
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include "galileo_e5_signal_processing.h"
#include "Galileo_E5a.h"
#include "configuration_interface.h"

using google::LogMessage;

GalileoE5aPcpsAcquisition::GalileoE5aPcpsAcquisition(
        ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
        role_(role), in_streams_(in_streams), out_streams_(out_streams), queue_(queue)
{

	configuration_ = configuration;
	std::string default_item_type = "gr_complex";
	std::string default_dump_filename = "../data/acquisition.dat";

	DLOG(INFO) << "role " << role;

	item_type_ = configuration_->property(role + ".item_type",
	        default_item_type);
	//TODO descubrir que ponemos de default value
    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_hz", 4000000);
    //
    if_ = configuration_->property(role + ".ifreq", 0);
    dump_ = configuration_->property(role + ".dump", false);
    shift_resolution_ = configuration_->property(role + ".doppler_max", 15);
    sampled_ms_ = configuration_->property(role + ".coherent_integration_time_ms", 1);
    // sampled_ms is an integer, always multiple of primary code period
    bit_transition_flag_ = configuration_->property(role + ".bit_transition_flag", false);

    if (!bit_transition_flag_)
        {
            max_dwells_ = configuration_->property(role + ".max_dwells", 1);
        }
    else
        {
            max_dwells_ = 2;
        }

    dump_filename_ = configuration_->property(role + ".dump_filename",
            default_dump_filename);

    //--- Find number of samples per spreading code (1ms)-------------------------
    code_length_ = round(fs_in_
            / (Galileo_E5a_CODE_CHIP_RATE_HZ / Galileo_E5a_CODE_LENGTH_CHIPS));

    vector_length_=code_length_ * sampled_ms_;

    code_= new gr_complex[vector_length_];

    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            acquisition_cc_ = pcps_make_acquisition_cc(sampled_ms_, max_dwells_,
                    shift_resolution_, if_, fs_in_, code_length_, code_length_,
                    bit_transition_flag_, queue_, dump_, dump_filename_);

            stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_, vector_length_);

            DLOG(INFO) << "stream_to_vector(" << stream_to_vector_->unique_id()
                    << ")";
            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id()
                    << ")";
        }
        else
        {
            LOG(WARNING) << item_type_
                    << " unknown acquisition item type";
        }

}

GalileoE5aPcpsAcquisition::~GalileoE5aPcpsAcquisition()
{
	delete[] code_;
}

void GalileoE5aPcpsAcquisition::set_channel(unsigned int channel)
{
    channel_ = channel;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_channel(channel_);
        }
}

void GalileoE5aPcpsAcquisition::set_threshold(float threshold)
{

	float pfa = configuration_->property(role_+ boost::lexical_cast<std::string>(channel_) + ".pfa", 0.0);

	if(pfa==0.0) pfa = configuration_->property(role_+".pfa", 0.0);

	if(pfa==0.0)
        {
            threshold_ = threshold;
        }
	else
        {
            threshold_ = calculate_threshold(pfa);
        }

	DLOG(INFO) <<"Channel "<<channel_<<" Threshold = " << threshold_;

	if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_threshold(threshold_);
        }
}


void GalileoE5aPcpsAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_doppler_max(doppler_max_);
        }
}

void GalileoE5aPcpsAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_doppler_step(doppler_step_);
        }
}

void GalileoE5aPcpsAcquisition::set_channel_queue(
        concurrent_queue<int> *channel_internal_queue)
{
    channel_internal_queue_ = channel_internal_queue;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_channel_queue(channel_internal_queue_);
        }
}


void GalileoE5aPcpsAcquisition::set_gnss_synchro(
        Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_gnss_synchro(gnss_synchro_);
        }
}


signed int GalileoE5aPcpsAcquisition::mag()
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


void GalileoE5aPcpsAcquisition::init()
{
    acquisition_cc_->init();
    set_local_code();
}

void GalileoE5aPcpsAcquisition::set_local_code()
{
	if (item_type_.compare("gr_complex")==0)
	{
		bool pilot = configuration_->property(
                "Acquisition" + boost::lexical_cast<std::string>(channel_)
                        + ".pilot", true);

		std::complex<float>* code = new std::complex<float>[code_length_];

		galileo_e5_a_code_gen_complex_sampled(code, gnss_synchro_->Signal,
		    gnss_synchro_->PRN, fs_in_, 0, false);

		for (unsigned int i = 0; i < sampled_ms_/4; i++)
		    {
		        memcpy(&(code_[i*code_length_]), code,
		             sizeof(gr_complex)*code_length_);
		    }

		acquisition_cc_->set_local_code(code_);

		delete[] code;
	}

}

void GalileoE5aPcpsAcquisition::reset()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            acquisition_cc_->set_active(true);
        }
}


float GalileoE5aPcpsAcquisition::calculate_threshold(float pfa)
{
    //Calculate the threshold
    unsigned int frequency_bins = 0;
    for (int doppler = (int)(-doppler_max_); doppler <= (int)doppler_max_; doppler += doppler_step_)
        {
            frequency_bins++;
        }
    DLOG(INFO) << "Channel " << channel_<< "  Pfa = " << pfa;
    unsigned int ncells = vector_length_*frequency_bins;
    double exponent = 1/(double)ncells;
    double val = pow(1.0 - pfa, exponent);
    double lambda = double(vector_length_);
    boost::math::exponential_distribution<double> mydist (lambda);
    float threshold = (float)quantile(mydist,val);

    return threshold;
}


void GalileoE5aPcpsAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}


void GalileoE5aPcpsAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
}

gr::basic_block_sptr GalileoE5aPcpsAcquisition::get_left_block()
{
    return stream_to_vector_;
}


gr::basic_block_sptr GalileoE5aPcpsAcquisition::get_right_block()
{
    return acquisition_cc_;
}



