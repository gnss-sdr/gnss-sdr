/*!
 * \file ad9361_fpga_signal_source.cc
 * \brief signal source for Analog Devices front-end AD9361 connected directly to FPGA accelerators.
 * This source implements only the AD9361 control. It is NOT compatible with conventional SDR acquisition and tracking blocks.
 * Please use the fmcomms2 source if conventional SDR acquisition and tracking is selected in the configuration file.
 * \author Javier Arribas, jarribas(at)cttc.es
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

#include "ad9361_fpga_signal_source.h"
#include "configuration_interface.h"
#include "ad9361_manager.h"
#include "GPS_L1_CA.h"
#include "GPS_L2C.h"
#include <glog/logging.h>
#include <iostream>  // for cout, endl

#ifdef __APPLE__
#include <iio/iio.h>
#else
#include <iio.h>
#endif

Ad9361FpgaSignalSource::Ad9361FpgaSignalSource(ConfigurationInterface* configuration,
    std::string role, unsigned int in_stream, unsigned int out_stream,
    boost::shared_ptr<gr::msg_queue> queue) : role_(role), in_stream_(in_stream), out_stream_(out_stream), queue_(queue)
{
    std::string default_item_type = "gr_complex";
    std::string default_dump_file = "./data/signal_source.dat";
    freq_ = configuration->property(role + ".freq", GPS_L1_FREQ_HZ);
    sample_rate_ = configuration->property(role + ".sampling_frequency", 2600000);
    bandwidth_ = configuration->property(role + ".bandwidth", 2000000);
    rx1_en_ = configuration->property(role + ".rx1_enable", true);
    rx2_en_ = configuration->property(role + ".rx2_enable", false);
    buffer_size_ = configuration->property(role + ".buffer_size", 0xA0000);
    quadrature_ = configuration->property(role + ".quadrature", true);
    rf_dc_ = configuration->property(role + ".rf_dc", true);
    bb_dc_ = configuration->property(role + ".bb_dc", true);
    gain_mode_rx1_ = configuration->property(role + ".gain_mode_rx1", std::string("manual"));
    gain_mode_rx2_ = configuration->property(role + ".gain_mode_rx2", std::string("manual"));
    rf_gain_rx1_ = configuration->property(role + ".gain_rx1", 64.0);
    rf_gain_rx2_ = configuration->property(role + ".gain_rx2", 64.0);
    rf_port_select_ = configuration->property(role + ".rf_port_select", std::string("A_BALANCED"));
    filter_file_ = configuration->property(role + ".filter_file", std::string(""));
    filter_auto_ = configuration->property(role + ".filter_auto", true);
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    samples_ = configuration->property(role + ".samples", 0);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

    enable_dds_lo_ = configuration->property(role + ".enable_dds_lo", false);
    freq_rf_tx_hz_ = configuration->property(role + ".freq_rf_tx_hz", GPS_L1_FREQ_HZ - GPS_L2_FREQ_HZ - 1000);
    freq_dds_tx_hz_ = configuration->property(role + ".freq_dds_tx_hz", 1000);
    scale_dds_dbfs_ = configuration->property(role + ".scale_dds_dbfs", -3.0);
    phase_dds_deg_ = configuration->property(role + ".phase_dds_deg", 0.0);
    tx_attenuation_db_ = configuration->property(role + ".tx_attenuation_db", 0.0);

    item_size_ = sizeof(gr_complex);

    std::cout << "device address: " << uri_ << std::endl;
    std::cout << "LO frequency : " << freq_ << " Hz" << std::endl;
    std::cout << "sample rate: " << sample_rate_ << " Hz" << std::endl;

    config_ad9361_rx_local(bandwidth_,
        sample_rate_,
        freq_,
        rf_port_select_,
        gain_mode_rx1_,
        gain_mode_rx2_,
        rf_gain_rx1_,
        rf_gain_rx2_);

    //LOCAL OSCILLATOR DDS GENERATOR FOR DUAL FREQUENCY OPERATION
    if (enable_dds_lo_ == true)
        {
            config_ad9361_lo_local(bandwidth_,
                sample_rate_,
                freq_rf_tx_hz_,
                tx_attenuation_db_,
                freq_dds_tx_hz_,
                scale_dds_dbfs_);
        }

    // turn switch to A/D position
    std::string default_device_name = "/dev/uio13";
    std::string device_name = configuration->property(role + ".devicename", default_device_name);
    int switch_position = configuration->property(role + ".switch_position", 0);
    switch_fpga = std::make_shared<fpga_switch>(device_name);
    switch_fpga->set_switch_position(switch_position);
}


Ad9361FpgaSignalSource::~Ad9361FpgaSignalSource()
{
    /* cleanup and exit */
    //std::cout<<"* AD9361 Disabling streaming channels\n";
    //if (rx0_i) { iio_channel_disable(rx0_i); }
    //if (rx0_q) { iio_channel_disable(rx0_q); }

    if (enable_dds_lo_)
        {
            ad9361_disable_lo_local();
        }

    // std::cout<<"* AD9361 Destroying context\n";
    //if (ctx) { iio_context_destroy(ctx); }
}


void Ad9361FpgaSignalSource::connect(gr::top_block_sptr top_block)
{
    DLOG(INFO) << "AD9361 FPGA source nothing to connect";
}


void Ad9361FpgaSignalSource::disconnect(gr::top_block_sptr top_block)
{
    DLOG(INFO) << "AD9361 FPGA source nothing to disconnect";
}


gr::basic_block_sptr Ad9361FpgaSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return gr::basic_block_sptr();
}


gr::basic_block_sptr Ad9361FpgaSignalSource::get_right_block()
{
    LOG(WARNING) << "Trying to get AD9361 FPGA signal source right block.";
    return gr::basic_block_sptr();
}
