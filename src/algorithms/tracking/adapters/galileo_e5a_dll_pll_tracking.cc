/*
 * galileo_e5a_dll_pll_tracking.cc
 *
 *  Created on: Jun 19, 2014
 *      Author: marc
 */

#include "galileo_e5a_dll_pll_tracking.h"
#include <glog/logging.h>
#include "Galileo_E5a.h"
#include "configuration_interface.h"


using google::LogMessage;

GalileoE5aDllPllTracking::GalileoE5aDllPllTracking(
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
    float pll_bw_hz;
    float dll_bw_hz;
    float pll_bw_init_hz;
    float dll_bw_init_hz;
    int ti_ms;
    float early_late_space_chips;
    item_type = configuration->property(role + ".item_type", default_item_type);
    //vector_length = configuration->property(role + ".vector_length", 2048);
    fs_in = configuration->property("GNSS-SDR.internal_fs_hz", 12000000);
    f_if = configuration->property(role + ".if", 0);
    dump = configuration->property(role + ".dump", false);
    pll_bw_hz = configuration->property(role + ".pll_bw_hz", 5.0);
    dll_bw_hz = configuration->property(role + ".dll_bw_hz", 2.0);
    pll_bw_init_hz = configuration->property(role + ".pll_bw_init_hz", 20.0);
    dll_bw_init_hz = configuration->property(role + ".dll_bw_init_hz", 20.0);
    ti_ms = configuration->property(role + ".ti_ms", 3);

    early_late_space_chips = configuration->property(role + ".early_late_space_chips", 0.5);
    std::string default_dump_filename = "./track_ch";
    dump_filename = configuration->property(role + ".dump_filename",
            default_dump_filename); //unused!
    vector_length = std::round(fs_in / (Galileo_E5a_CODE_CHIP_RATE_HZ / Galileo_E5a_CODE_LENGTH_CHIPS));

    //################# MAKE TRACKING GNURadio object ###################
    if (item_type.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            tracking_ = galileo_e5a_dll_pll_make_tracking_cc(
                    f_if,
                    fs_in,
                    vector_length,
                    queue_,
                    dump,
                    dump_filename,
                    pll_bw_hz,
                    dll_bw_hz,
                    pll_bw_init_hz,
                    dll_bw_init_hz,
                    ti_ms,
                    early_late_space_chips);
        }
    else
        {
            LOG(WARNING) << item_type << " unknown tracking item type.";
        }
    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
}


GalileoE5aDllPllTracking::~GalileoE5aDllPllTracking()
{}


void GalileoE5aDllPllTracking::start_tracking()
{
    tracking_->start_tracking();
}

/*
 * Set tracking channel unique ID
 */
void GalileoE5aDllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}

/*
 * Set tracking channel internal queue
 */
void GalileoE5aDllPllTracking::set_channel_queue(
        concurrent_queue<int> *channel_internal_queue)
{
    channel_internal_queue_ = channel_internal_queue;
    tracking_->set_channel_queue(channel_internal_queue_);
}

void GalileoE5aDllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_->set_gnss_synchro(p_gnss_synchro);
}

void GalileoE5aDllPllTracking::connect(gr::top_block_sptr top_block)
{
    //nothing to connect, now the tracking uses gr_sync_decimator
}

void GalileoE5aDllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}

gr::basic_block_sptr GalileoE5aDllPllTracking::get_left_block()
{
    return tracking_;
}

gr::basic_block_sptr GalileoE5aDllPllTracking::get_right_block()
{
    return tracking_;
}


