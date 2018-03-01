/*!
 * \file uhd_signal_source.cc
 * \brief Universal Hardware Driver signal source
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
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

#include "uhd_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_valve.h"
#include "GPS_L1_CA.h"
#include <glog/logging.h>
#include <uhd/types/device_addr.hpp>
#include <uhd/exception.hpp>
#include <volk/volk.h>
#include <iostream>


using google::LogMessage;

UhdSignalSource::UhdSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream, unsigned int out_stream,
        boost::shared_ptr<gr::msg_queue> queue) :
                role_(role), in_stream_(in_stream), out_stream_(out_stream),
                queue_(queue)
{
    // DUMP PARAMETERS
    std::string empty = "";
    std::string default_dump_file = "./data/signal_source.dat";
    std::string default_item_type = "cshort";

    // UHD COMMON PARAMETERS
    uhd::device_addr_t dev_addr;
    device_address_ = configuration->property(role + ".device_address", empty);
    // When left empty, the device discovery routines will search all
    // available transports on the system (ethernet, usb...).
    // To narrow down the discovery process to a particular device,
    // specify a transport key/value pair specific to your device.
    if (empty.compare(device_address_) != 0) // if not empty
        {
            dev_addr["addr"] = device_address_;
        }
    //filter the device by serial number if required (useful for USB devices)
    std::string device_serial = configuration->property(role + ".device_serial", empty);
    if (empty.compare(device_serial) != 0) // if not empty
        {
            dev_addr["serial"] = device_serial;
        }
    subdevice_ = configuration->property(role + ".subdevice", empty);
    clock_source_ = configuration->property(role + ".clock_source", std::string("internal"));
    RF_channels_ = configuration->property(role + ".RF_channels", 1);
    sample_rate_ = configuration->property(role + ".sampling_frequency", 4.0e6);
    item_type_ = configuration->property(role + ".item_type", default_item_type);

    if (RF_channels_ == 1)
        {
            // Single RF channel UHD operation (backward compatible config file format)
            samples_.push_back(configuration->property(role + ".samples", 0));
            dump_.push_back(configuration->property(role + ".dump", false));
            dump_filename_.push_back(configuration->property(role + ".dump_filename", default_dump_file));

            freq_.push_back(configuration->property(role + ".freq", GPS_L1_FREQ_HZ));
            gain_.push_back(configuration->property(role + ".gain", 50.0));

            IF_bandwidth_hz_.push_back(configuration->property(role + ".IF_bandwidth_hz", sample_rate_/2));

        }
    else
        {
            // multiple RF channels selected
            for (int i = 0; i < RF_channels_; i++)
                {
                    // Single RF channel UHD operation (backward compatible config file format)
                    samples_.push_back(configuration->property(role + ".samples" + boost::lexical_cast<std::string>(i), 0));
                    dump_.push_back(configuration->property(role + ".dump" + boost::lexical_cast<std::string>(i), false));
                    dump_filename_.push_back(configuration->property(role + ".dump_filename" + boost::lexical_cast<std::string>(i), default_dump_file));

                    freq_.push_back(configuration->property(role + ".freq" + boost::lexical_cast<std::string>(i), GPS_L1_FREQ_HZ));
                    gain_.push_back(configuration->property(role + ".gain" + boost::lexical_cast<std::string>(i), 50.0));

                    IF_bandwidth_hz_.push_back(configuration->property(role + ".IF_bandwidth_hz" + boost::lexical_cast<std::string>(i), sample_rate_/2));
                }
        }
    // 1. Make the uhd driver instance
    //uhd_source_= uhd::usrp::multi_usrp::make(dev_addr);

    // single source
    // param: device_addr the address to identify the hardware
    // param: io_type the desired output data type
    //    fc64: Complex floating point (64-bit floats) range [-1.0, +1.0].
    //    fc32: Complex floating point (32-bit floats) range [-1.0, +1.0].
    //    sc16: Complex signed integer (16-bit integers) range [-32768, +32767].
    //     sc8: Complex signed integer (8-bit integers) range [-128, 127].
    if (item_type_.compare("cbyte") == 0)
        {
            item_size_ = sizeof(lv_8sc_t);
            uhd_stream_args_ = uhd::stream_args_t("sc8");
        }
    else if (item_type_.compare("cshort") == 0)
        {
            item_size_ = sizeof(lv_16sc_t);
            uhd_stream_args_ = uhd::stream_args_t("sc16");
        }
    else if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            uhd_stream_args_ = uhd::stream_args_t("fc32");
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type. Using cshort.";
            item_size_ = sizeof(lv_16sc_t);
            uhd_stream_args_ = uhd::stream_args_t("sc16");
        }

    // select the number of channels and the subdevice specifications
    for (int i = 0; i < RF_channels_; i++)
        {
            uhd_stream_args_.channels.push_back(i);
        }

    // 1.2 Make the UHD source object
    uhd_source_ = gr::uhd::usrp_source::make(dev_addr, uhd_stream_args_);

    // Set subdevice specification string for USRP family devices. It is composed of:
    // <motherboard slot name>:<daughterboard frontend name>
    // For motherboards: All USRP family motherboards have a first slot named A:.
    //       The USRP1 has two daughterboard subdevice slots, known as A: and B:.
    // For daughterboards, see http://files.ettus.com/uhd_docs/manual/html/dboards.html
    // "0" is valid for DBSRX, DBSRX2, WBX Series
    // Dual channel example: "A:0 B:0"
    // TODO: Add support for multiple motherboards (i.e. four channels "A:0 B:0 A:1 B1")

    uhd_source_->set_subdev_spec(subdevice_, 0);

    // 2.1 set sampling clock reference
    // Set the clock source for the usrp device.
    // Options: internal, external, or MIMO
    uhd_source_->set_clock_source(clock_source_);

    // 2.2 set the sample rate for the usrp device
    uhd_source_->set_samp_rate(sample_rate_);
    // the actual sample rate may differ from the rate set
    std::cout << boost::format("Sampling Rate for the USRP device: %f [sps]...") % (uhd_source_->get_samp_rate()) << std::endl;
    LOG(INFO) << boost::format("Sampling Rate for the USRP device: %f [sps]...") % (uhd_source_->get_samp_rate());

    std::vector<std::string> sensor_names;

    for (int i = 0; i < RF_channels_; i++)
        {
            std::cout << "UHD RF CHANNEL #" << i << " SETTINGS" << std::endl;
            // 3. Tune the usrp device to the desired center frequency
            uhd_source_->set_center_freq(freq_.at(i), i);
            std::cout << boost::format("Actual USRP center freq.: %f [Hz]...") % (uhd_source_->get_center_freq(i)) << std::endl;
            LOG(INFO) << boost::format("Actual USRP center freq. set to: %f [Hz]...") % (uhd_source_->get_center_freq(i));

            // TODO: Assign the remnant IF from the PLL tune error
            std::cout << boost::format("PLL Frequency tune error %f [Hz]...") % (uhd_source_->get_center_freq(i) - freq_.at(i)) << std::endl;
            LOG(INFO) << boost::format("PLL Frequency tune error %f [Hz]...") % (uhd_source_->get_center_freq(i) - freq_.at(i));

            // 4. set the gain for the daughterboard
            uhd_source_->set_gain(gain_.at(i), i);
            std::cout << boost::format("Actual daughterboard gain set to: %f dB...") % uhd_source_->get_gain(i) << std::endl;
            LOG(INFO) << boost::format("Actual daughterboard gain set to: %f dB...") % uhd_source_->get_gain(i);

            //5.  Set the bandpass filter on the RF frontend
            std::cout << boost::format("Setting RF bandpass filter bandwidth to: %f [Hz]...") % IF_bandwidth_hz_.at(i) << std::endl;
            uhd_source_->set_bandwidth(IF_bandwidth_hz_.at(i), i);

            //set the antenna (optional)
            //uhd_source_->set_antenna(ant);

            // We should wait? #include <boost/thread.hpp>
            // boost::this_thread::sleep(boost::posix_time::seconds(1));

            // Check out the status of the lo_locked sensor (boolean for LO lock state)
            sensor_names = uhd_source_->get_sensor_names(i);
            if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end())
                {
                    uhd::sensor_value_t lo_locked = uhd_source_->get_sensor("lo_locked", i);
                    std::cout << boost::format("Check for front-end %s ...") % lo_locked.to_pp_string() << " is ";
                    if (lo_locked.to_bool() == true)
                        {
                            std::cout << "Locked" << std::endl;
                        }
                    else
                        {
                            std::cout << "UNLOCKED!" << std::endl;
                        }
                    //UHD_ASSERT_THROW(lo_locked.to_bool());
                }
        }


    for (int i = 0; i < RF_channels_; i++)
        {
            if (samples_.at(i) != 0)
                {
                    LOG(INFO) << "RF_channel "<< i << " Send STOP signal after " << samples_.at(i) << " samples";
                    valve_.push_back(gnss_sdr_make_valve(item_size_, samples_.at(i), queue_));
                    DLOG(INFO) << "valve(" << valve_.at(i)->unique_id() << ")";
                }

            if (dump_.at(i))
                {
                    LOG(INFO) << "RF_channel "<< i << "Dumping output into file " << dump_filename_.at(i);
                    file_sink_.push_back(gr::blocks::file_sink::make(item_size_, dump_filename_.at(i).c_str()));
                    DLOG(INFO) << "file_sink(" << file_sink_.at(i)->unique_id() << ")";
                }
        }
}



UhdSignalSource::~UhdSignalSource()
{}


void UhdSignalSource::connect(gr::top_block_sptr top_block)
{
    for (int i = 0; i < RF_channels_; i++)
        {
            if (samples_.at(i) != 0)
                {
                    top_block->connect(uhd_source_, i, valve_.at(i), 0);
                    DLOG(INFO) << "connected usrp source to valve RF Channel " << i;
                    if (dump_.at(i))
                        {
                            top_block->connect(valve_.at(i), 0, file_sink_.at(i), 0);
                            DLOG(INFO) << "connected valve to file sink RF Channel " << i;
                        }
                }
            else
                {
                    if (dump_.at(i))
                        {
                            top_block->connect(uhd_source_, i, file_sink_.at(i), 0);
                            DLOG(INFO) << "connected usrp source to file sink RF Channel " << i;
                        }
                }
        }
}



void UhdSignalSource::disconnect(gr::top_block_sptr top_block)
{
    for (int i = 0; i < RF_channels_; i++)
        {
            if (samples_.at(i) != 0)
                {
                    top_block->disconnect(uhd_source_, i, valve_.at(i), 0);
                    LOG(INFO) << "UHD source disconnected";
                    if (dump_.at(i))
                        {
                            top_block->disconnect(valve_.at(i), 0, file_sink_.at(i), 0);
                        }
                }
            else
                {
                    if (dump_.at(i))
                        {
                            top_block->disconnect(uhd_source_, i, file_sink_.at(i), 0);
                        }
                }
        }
}



gr::basic_block_sptr UhdSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    //return gr_basic_block_sptr();
    return gr::uhd::usrp_source::sptr();
}


gr::basic_block_sptr UhdSignalSource::get_right_block()
{
    return get_right_block(0);
}


gr::basic_block_sptr UhdSignalSource::get_right_block(int RF_channel)
{
    //TODO: There is a incoherence here: Multichannel UHD is a single block with multiple outputs, but if the sample limit is enabled, the output is a multiple block!
    if (samples_.at(RF_channel) != 0)
        {
            return valve_.at(RF_channel);
        }
    else
        {
            return uhd_source_;
        }
}
