/*!
 * \file uhd_signal_source.cc
 * \brief Universal Hardware Driver signal source
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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

#include "uhd_signal_source.h"
//#include <uhd/usrp/multi_usrp.hpp>
#include <gnuradio/gr_uhd_usrp_source.h>
#include <uhd/types/device_addr.hpp>
#include <uhd/exception.hpp>
//#include <boost/program_options.hpp>
#include <gnuradio/gr_file_sink.h>
#include "configuration_interface.h"
#include "gnss_sdr_valve.h"
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <iostream>
#include "GPS_L1_CA.h"

using google::LogMessage;

UhdSignalSource::UhdSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream, unsigned int out_stream,
        gr_msg_queue_sptr queue) :
        role_(role), in_stream_(in_stream), out_stream_(out_stream),
        queue_(queue)
{

    // DUMP PARAMETERS
    std::string empty = "";
    std::string default_dump_file = "./data/signal_source.dat";
    std::string default_item_type = "short";
    samples_ = configuration->property(role + ".samples", 0);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename",
            default_dump_file);

    // UHD PARAMETERS
    uhd::device_addr_t dev_addr;
    device_address_= configuration->property(role + ".device_address",empty);
    if (empty.compare(empty)!=0)
        {
            dev_addr["addr0"]=device_address_;
        }

    subdevice_=configuration->property(role + ".subdevice",empty);

    freq_ = configuration->property(role + ".freq", GPS_L1_FREQ_HZ);
    gain_ = configuration->property(role + ".gain", (double)50.0);
    sample_rate_ = configuration->property(role + ".sampling_frequency", (double)4.0e6);

    IF_bandwidth_hz_ = configuration->property(role + ".IF_bandwidth_hz", sample_rate_/2);

    item_type_ = configuration->property(role + ".item_type",
            default_item_type);
    if (item_type_.compare("short") == 0)
        {
            item_size_ = sizeof(short);
        }
    else if (item_type_.compare("gr_complex") == 0)
        {
    	    item_size_ = sizeof(gr_complex);
            // 1. Make the uhd driver instance
            //uhd_source_= uhd::usrp::multi_usrp::make(dev_addr);

            // single source
            // * \param device_addr the address to identify the hardware
            // * \param io_type the desired output data type
            uhd_source_ = uhd_make_usrp_source(dev_addr, uhd::stream_args_t("fc32"));


            // 2.1 set sampling clock reference
            //Lock mboard clocks internal, external, or mimo
            std::string clk_reference = "internal";
            uhd_source_->set_clock_source(clk_reference);

            // 2.2 set sampling rate
            uhd_source_->set_samp_rate(sample_rate_);
            std::cout << boost::format("Actual RX Rate: %f [SPS]...") % (uhd_source_->get_samp_rate()) << std::endl << std::endl;
            DLOG(INFO) << boost::format("Actual RX Rate: %f [SPS]...") % (uhd_source_->get_samp_rate()) << std::endl << std::endl;

            // 3. set rx frequency
            uhd_source_->set_center_freq(freq_);
            std::cout << boost::format("Actual RX Freq: %f [Hz]...") % (uhd_source_->get_center_freq()) << std::endl << std::endl;
            DLOG(INFO) << boost::format("Actual RX Freq: %f [Hz]...") % (uhd_source_->get_center_freq()) << std::endl << std::endl;

            // TODO: Asign the remanent IF from the PLL tune error
            std::cout << boost::format("PLL Frequency tune error %f [Hz]...") % (uhd_source_->get_center_freq()-freq_) << std::endl;
            DLOG(INFO) <<  boost::format("PLL Frequency tune error %f [Hz]...") % (uhd_source_->get_center_freq()-freq_) << std::endl;

            // 4. set rx gain
            uhd_source_->set_gain(gain_);
            std::cout << boost::format("Actual RX Gain: %f dB...") % uhd_source_->get_gain() << std::endl << std::endl;
            DLOG(INFO) << boost::format("Actual RX Gain: %f dB...") % uhd_source_->get_gain() << std::endl << std::endl;

            //5. set the IF filter bandwidth

            std::cout << boost::format("Setting RX Bandwidth: %f [Hz]...") % IF_bandwidth_hz_ << std::endl;
            uhd_source_->set_bandwidth(IF_bandwidth_hz_);
            ///std::cout << boost::format("Actual RX Bandwidth: %f MHz...") % uhd_source_->get_rx_bandwidth() << std::endl << std::endl;
            //DLOG(INFO) << boost::format("Actual RX Bandwidth: %f MHz...") % uhd_source_->get_rx_bandwidth() << std::endl << std::endl;

            //set the antenna (optional)
            //uhd_source_->set_antenna(ant);

            //LO lock status
            //Check Ref and LO Lock detect
            std::vector<std::string> sensor_names;
            sensor_names = uhd_source_->get_sensor_names(0);
            if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
                uhd::sensor_value_t lo_locked = uhd_source_->get_sensor("lo_locked",0);
                std::cout << boost::format("Check for front-end %s ...") % lo_locked.to_pp_string() << " is ";
                if (lo_locked.to_bool()==true)
                {
                	std::cout<<"Locked"<<std::endl;
                }else{
                	std::cout<<"UNLOCKED!!!!"<<std::endl;
                }
            //UHD_ASSERT_THROW(lo_locked.to_bool());
            }

            uhd_source_->set_subdev_spec(subdevice_,0);
        }
    else
        {
            LOG_AT_LEVEL(WARNING) << item_type_
                    << " unrecognized item type. Using short.";
            item_size_ = sizeof(short);
        }

    if (samples_ != 0)
        {
            DLOG(INFO) << "Send STOP signal after " << samples_ << " samples";
            valve_ = gnss_sdr_make_valve(item_size_, samples_, queue_);
            DLOG(INFO) << "valve(" << valve_->unique_id() << ")";
        }

    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr_make_file_sink(item_size_, dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }

}

UhdSignalSource::~UhdSignalSource()
{
}

void UhdSignalSource::connect(gr_top_block_sptr top_block)
{

    if (samples_ != 0)
        {
            top_block->connect(uhd_source_, 0, valve_, 0);
            DLOG(INFO) << "connected usrp source to valve";

            if (dump_)
                {
                    top_block->connect(valve_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected valve to file sink";
                }
        }
    else
        {
            if (dump_)
                {
                    top_block->connect(uhd_source_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected usrp source to file sink";
                }
        }
}

void UhdSignalSource::disconnect(gr_top_block_sptr top_block)
{

    if (samples_ != 0)
        {
            top_block->disconnect(uhd_source_, 0, valve_, 0);

            if (dump_)
                {
                    top_block->disconnect(valve_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (dump_)
                {
                    top_block->disconnect(uhd_source_, 0, file_sink_, 0);
                }
        }
}

gr_basic_block_sptr UhdSignalSource::get_left_block()
{
    LOG_AT_LEVEL(WARNING) << "Trying to get signal source left block.";
    return gr_basic_block_sptr();
}

gr_basic_block_sptr UhdSignalSource::get_right_block()
{
    if (samples_ != 0)
        {
            return valve_;
        }
    else
        {
            return uhd_source_;
        }
}
