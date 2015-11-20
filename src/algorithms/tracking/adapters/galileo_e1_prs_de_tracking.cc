/*!
 * \file galileo_e1_prs_de_tracking.h
 * \brief  Adapts a double estimator tracking loop block
 *   to a TrackingInterface for Galileo E1 signals including PRS
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "galileo_e1_prs_de_tracking.h"
#include <glog/logging.h>
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"
#include "configuration_interface.h"
#include "spirent_prs_code_generator.h"


using google::LogMessage;

GalileoE1PrsDeTracking::GalileoE1PrsDeTracking(
        ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
        role_(role), in_streams_(in_streams), out_streams_(out_streams),
        queue_(queue)
{
    DLOG(INFO) << "role " << role;
    //################# CONFIGURATION PARAMETERS ########################
    int fs_in;
    int vector_length;
    int f_if;
    bool dump;
    std::string dump_filename;
    std::string item_type;
    std::string default_item_type = "gr_complex";
    float pll_initial_bw_hz;
    float pll_final_bw_hz;
    float dll_initial_bw_hz;
    float dll_final_bw_hz;
    float sll_initial_bw_hz;
    float sll_final_bw_hz;
    int   pll_loop_order;
    int   dll_loop_order;
    int   sll_loop_order;
    float initial_early_late_code_space_chips;
    float final_early_late_code_space_chips;
    float early_late_subcarrier_space_chips;
    bool aid_subcarrier_with_carrier;
    bool aid_code_with_subcarrier;
    bool model_prs_subcarrier_as_sinusoid;

    item_type = configuration->property(role + ".item_type", default_item_type);
    fs_in = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    f_if = configuration->property(role + ".if", 0);
    dump = configuration->property(role + ".dump", false);
    pll_initial_bw_hz = configuration->property(role + ".pll_initial_bw_hz", 15.0);
    pll_final_bw_hz = configuration->property(role + ".pll_final_bw_hz", 15.0);
    sll_initial_bw_hz = configuration->property(role + ".sll_initial_bw_hz", 2.0);
    sll_final_bw_hz = configuration->property(role + ".sll_final_bw_hz", 2.0);
    dll_initial_bw_hz = configuration->property(role + ".dll_initial_bw_hz", 0.5);
    dll_final_bw_hz = configuration->property(role + ".dll_final_bw_hz", 0.5);
    initial_early_late_code_space_chips = configuration->property(role + ".initial_early_late_code_space_chips", 0.5);
    final_early_late_code_space_chips = configuration->property(role + ".final_early_late_code_space_chips", 0.5);
    early_late_subcarrier_space_chips = configuration->property(role + ".early_late_subcarrier_space_cycles", 0.125);
    aid_subcarrier_with_carrier = configuration->property(role + ".aid_subcarrier_with_carrier", false );
    aid_code_with_subcarrier = configuration->property(role + ".aid_code_with_subcarrier", false );
    model_prs_subcarrier_as_sinusoid = configuration->property(role + ".model_prs_subcarrier_as_sinusoid", false );

    pll_loop_order = configuration->property(role + ".pll_loop_order", 3);
    sll_loop_order = configuration->property(role + ".sll_loop_order", 3);
    dll_loop_order = configuration->property(role + ".dll_loop_order", 1);

    std::string default_code_type = "Spirent";
    std::string code_type = configuration->property(role + ".prs_code_type", default_code_type );

    std::string default_dump_filename = "./track_de_";
    dump_filename = configuration->property(role + ".dump_filename",
            default_dump_filename); //unused!
    vector_length = std::round(fs_in / (Galileo_E1_CODE_CHIP_RATE_HZ / Galileo_E1_B_CODE_LENGTH_CHIPS));

    boost::shared_ptr< LongCodeInterface > code_gen;

    if( not code_type.compare("Spirent") )// anything other than zero means not a match
    {
        code_gen = boost::shared_ptr< LongCodeInterface >(
                new SpirentPrsCodeGenerator( 1, true ) );

        if( code_gen == 0 )
        {
            LOG(ERROR) << "Unable to create a SpirentPrsCodeGenerator";
        }
    }
    else
    {
        LOG(ERROR) << code_type << " unknown PRS code type";
    }

    //################# MAKE TRACKING GNURadio object ###################
    if (item_type.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            tracking_ = galileo_e1_prs_de_make_tracking_cc(
                    f_if,
                    fs_in,
                    vector_length,
                    queue_,
                    dump,
                    dump_filename,
                    pll_loop_order, pll_initial_bw_hz, pll_final_bw_hz,
                    dll_loop_order, dll_initial_bw_hz, dll_final_bw_hz,
                    sll_loop_order, sll_initial_bw_hz, sll_final_bw_hz,
                    initial_early_late_code_space_chips, final_early_late_code_space_chips,
                    early_late_subcarrier_space_chips,
                    aid_subcarrier_with_carrier, aid_code_with_subcarrier,
                    model_prs_subcarrier_as_sinusoid, code_gen);
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << item_type << " unknown tracking item type.";
        }

    channel_ = 0;
    channel_internal_queue_ = 0;

    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
}

GalileoE1PrsDeTracking::~GalileoE1PrsDeTracking()
{}

void GalileoE1PrsDeTracking::start_tracking()
{
    tracking_->start_tracking();
}

/*
 * Set tracking channel unique ID
 */
void GalileoE1PrsDeTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}

/*
 * Set tracking channel internal queue
 */
void GalileoE1PrsDeTracking::set_channel_queue(
        concurrent_queue<int> *channel_internal_queue)
{
    channel_internal_queue_ = channel_internal_queue;

    tracking_->set_channel_queue(channel_internal_queue_);

}

void GalileoE1PrsDeTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_->set_gnss_synchro(p_gnss_synchro);
}

void GalileoE1PrsDeTracking::connect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	//nothing to connect, now the tracking uses gr_sync_decimator
}

void GalileoE1PrsDeTracking::disconnect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	//nothing to disconnect, now the tracking uses gr_sync_decimator
}

gr::basic_block_sptr GalileoE1PrsDeTracking::get_left_block()
{
    return tracking_;
}

gr::basic_block_sptr GalileoE1PrsDeTracking::get_right_block()
{
    return tracking_;
}


