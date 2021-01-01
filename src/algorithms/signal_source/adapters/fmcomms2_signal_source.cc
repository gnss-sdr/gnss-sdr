/*!
 * \file fmcomms2_signal_source.cc
 * \brief Signal source for SDR hardware from Analog Devices based on
 * fmcomms2 evaluation board.
 * \author Rodrigo Muñoz, 2017, rmunozl(at)inacap.cl, rodrigo.munoz(at)proteinlab.cl
 *
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

#include "fmcomms2_signal_source.h"
#include "GPS_L1_CA.h"
#include "GPS_L2C.h"
#include "ad9361_manager.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_valve.h"
#include <glog/logging.h>
#include <algorithm>  // for max
#include <exception>
#include <iostream>
#include <utility>


Fmcomms2SignalSource::Fmcomms2SignalSource(const ConfigurationInterface *configuration,
    const std::string &role, unsigned int in_stream, unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t> *queue) : role_(role), in_stream_(in_stream), out_stream_(out_stream)
{
    const std::string default_item_type("gr_complex");
    const std::string default_dump_file("./data/signal_source.dat");
    const std::string default_gain_mode("slow_attack");
    const double default_tx_attenuation_db = -10.0;
    uri_ = configuration->property(role + ".device_address", std::string("192.168.2.1"));
    freq_ = configuration->property(role + ".freq", static_cast<uint64_t>(GPS_L1_FREQ_HZ));
    sample_rate_ = configuration->property(role + ".sampling_frequency", static_cast<uint64_t>(2600000));
    bandwidth_ = configuration->property(role + ".bandwidth", static_cast<uint64_t>(2000000));
    rx1_en_ = configuration->property(role + ".rx1_enable", true);
    rx2_en_ = configuration->property(role + ".rx2_enable", false);
    buffer_size_ = configuration->property(role + ".buffer_size", 0xA0000);
    quadrature_ = configuration->property(role + ".quadrature", true);
    rf_dc_ = configuration->property(role + ".rf_dc", true);
    bb_dc_ = configuration->property(role + ".bb_dc", true);
    RF_channels_ = configuration->property(role + ".RF_channels", 1);
    gain_mode_rx1_ = configuration->property(role + ".gain_mode_rx1", default_gain_mode);
    gain_mode_rx2_ = configuration->property(role + ".gain_mode_rx2", default_gain_mode);
    rf_gain_rx1_ = configuration->property(role + ".gain_rx1", 64.0);
    rf_gain_rx2_ = configuration->property(role + ".gain_rx2", 64.0);
    rf_port_select_ = configuration->property(role + ".rf_port_select", std::string("A_BALANCED"));
    filter_file_ = configuration->property(role + ".filter_file", std::string(""));
    filter_auto_ = configuration->property(role + ".filter_auto", false);
    if (filter_auto_)
        {
            filter_source_ = configuration->property(role + ".filter_source", std::string("Auto"));
        }
    else
        {
            filter_source_ = configuration->property(role + ".filter_source", std::string("Off"));
        }
    filter_filename_ = configuration->property(role + ".filter_filename", filter_file_);
    Fpass_ = configuration->property(role + ".Fpass", 0.0);
    Fstop_ = configuration->property(role + ".Fstop", 0.0);

    item_type_ = configuration->property(role + ".item_type", default_item_type);
    samples_ = configuration->property(role + ".samples", static_cast<int64_t>(0));
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

    // AD9361 Local Oscillator generation for dual band operation
    enable_dds_lo_ = configuration->property(role + ".enable_dds_lo", false);
    freq_dds_tx_hz_ = configuration->property(role + ".freq_dds_tx_hz", static_cast<uint64_t>(10000));
    freq_rf_tx_hz_ = configuration->property(role + ".freq_rf_tx_hz", static_cast<uint64_t>(GPS_L1_FREQ_HZ - GPS_L2_FREQ_HZ - freq_dds_tx_hz_));
    scale_dds_dbfs_ = configuration->property(role + ".scale_dds_dbfs", 0.0);
    phase_dds_deg_ = configuration->property(role + ".phase_dds_deg", 0.0);
    tx_attenuation_db_ = configuration->property(role + ".tx_attenuation_db", default_tx_attenuation_db);
    tx_bandwidth_ = configuration->property(role + ".tx_bandwidth", static_cast<uint64_t>(500000));

    rf_shutdown_ = configuration->property(role + ".rf_shutdown", FLAGS_rf_shutdown);

    item_size_ = sizeof(gr_complex);

    // some basic checks
    if ((rf_port_select_ != "A_BALANCED") and (rf_port_select_ != "B_BALANCED") and (rf_port_select_ != "A_N") and (rf_port_select_ != "B_N") and (rf_port_select_ != "B_P") and (rf_port_select_ != "C_N") and (rf_port_select_ != "C_P") and (rf_port_select_ != "TX_MONITOR1") and (rf_port_select_ != "TX_MONITOR2") and (rf_port_select_ != "TX_MONITOR1_2"))
        {
            std::cout << "Configuration parameter rf_port_select should take one of these values:\n";
            std::cout << " A_BALANCED, B_BALANCED, A_N, B_N, B_P, C_N, C_P, TX_MONITOR1, TX_MONITOR2, TX_MONITOR1_2\n";
            std::cout << "Error: provided value rf_port_select=" << rf_port_select_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value rf_port_select=A_BALANCED\n";
            rf_port_select_ = std::string("A_BALANCED");
            LOG(WARNING) << "Invalid configuration value for rf_port_select parameter. Set to rf_port_select=A_BALANCED";
        }

    if ((gain_mode_rx1_ != "manual") and (gain_mode_rx1_ != "slow_attack") and (gain_mode_rx1_ != "fast_attack") and (gain_mode_rx1_ != "hybrid"))
        {
            std::cout << "Configuration parameter gain_mode_rx1 should take one of these values:\n";
            std::cout << " manual, slow_attack, fast_attack, hybrid\n";
            std::cout << "Error: provided value gain_mode_rx1=" << gain_mode_rx1_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value gain_mode_rx1=" << default_gain_mode << '\n';
            gain_mode_rx1_ = default_gain_mode;
            LOG(WARNING) << "Invalid configuration value for gain_mode_rx1 parameter. Set to gain_mode_rx1=" << default_gain_mode;
        }

    if ((gain_mode_rx2_ != "manual") and (gain_mode_rx2_ != "slow_attack") and (gain_mode_rx2_ != "fast_attack") and (gain_mode_rx2_ != "hybrid"))
        {
            std::cout << "Configuration parameter gain_mode_rx2 should take one of these values:\n";
            std::cout << " manual, slow_attack, fast_attack, hybrid\n";
            std::cout << "Error: provided value gain_mode_rx2=" << gain_mode_rx2_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value gain_mode_rx2=" << default_gain_mode << '\n';
            gain_mode_rx2_ = default_gain_mode;
            LOG(WARNING) << "Invalid configuration value for gain_mode_rx1 parameter. Set to gain_mode_rx2=" << default_gain_mode;
        }

    if ((filter_source_ != "Off") and (filter_source_ != "Auto") and (filter_source_ != "File") and (filter_source_ != "Design"))
        {
            std::cout << "Configuration parameter filter_source should take one of these values:\n";
            std::cout << "  Off: Disable filter\n";
            std::cout << "  Auto: Use auto-generated filters\n";
            std::cout << "  File: User-provided filter in filter_filename parameter\n";
#if LIBAD9361_VERSION_GREATER_THAN_01
            std::cout << "  Design: Create filter from Fpass, Fstop, sampling_frequency and bandwidth parameters\n";
#endif
            std::cout << "Error: provided value filter_source=" << filter_source_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value filter_source=Off\n";
            filter_source_ = std::string("Off");
            LOG(WARNING) << "Invalid configuration value for filter_source parameter. Set to filter_source=Off";
        }

    if (gain_mode_rx1_ == "manual")
        {
            if (rf_gain_rx1_ > 73.0 or rf_gain_rx1_ < -1.0)
                {
                    std::cout << "Configuration parameter rf_gain_rx1 should take values between -1.0 and 73 dB\n";
                    std::cout << "Error: provided value rf_gain_rx1=" << rf_gain_rx1_ << " is not among valid values\n";
                    std::cout << " This parameter has been set to its default value rf_gain_rx1=64.0\n";
                    rf_gain_rx1_ = 64.0;
                    LOG(WARNING) << "Invalid configuration value for rf_gain_rx1 parameter. Set to rf_gain_rx1=64.0";
                }
        }

    if (gain_mode_rx2_ == "manual")
        {
            if (rf_gain_rx2_ > 73.0 or rf_gain_rx2_ < -1.0)
                {
                    std::cout << "Configuration parameter rf_gain_rx2 should take values between -1.0 and 73 dB\n";
                    std::cout << "Error: provided value rf_gain_rx2=" << rf_gain_rx2_ << " is not among valid values\n";
                    std::cout << " This parameter has been set to its default value rf_gain_rx2=64.0\n";
                    rf_gain_rx2_ = 64.0;
                    LOG(WARNING) << "Invalid configuration value for rf_gain_rx2 parameter. Set to rf_gain_rx2=64.0";
                }
        }

    if (bandwidth_ < 200000 or bandwidth_ > 56000000)
        {
            std::cout << "Configuration parameter bandwidth should take values between 200000 and 56000000 Hz\n";
            std::cout << "Error: provided value bandwidth=" << bandwidth_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value bandwidth=2000000\n";
            bandwidth_ = 2000000;
            LOG(WARNING) << "Invalid configuration value for bandwidth parameter. Set to bandwidth=2000000";
        }

    std::cout << "device address: " << uri_ << '\n';
    std::cout << "LO frequency : " << freq_ << " Hz\n";
    std::cout << "sample rate: " << sample_rate_ << " Hz\n";

    if (item_type_ == "gr_complex")
        {
            if (RF_channels_ == 1)
                {
                    if (rx1_en_ and rx2_en_)
                        {
                            LOG(FATAL) << "Configuration error: both rx1 and rx2 are enabled but RF_channels=1 !";
                        }
                    else
                        {
#if GNURADIO_API_IIO
                            fmcomms2_source_f32c_ = gr::iio::fmcomms2_source_f32c::make(
                                uri_.c_str(), freq_, sample_rate_,
                                bandwidth_,
                                rx1_en_, rx2_en_,
                                buffer_size_, quadrature_, rf_dc_,
                                bb_dc_, gain_mode_rx1_.c_str(), rf_gain_rx1_,
                                gain_mode_rx2_.c_str(), rf_gain_rx2_,
                                rf_port_select_.c_str(), filter_source_.c_str(),
                                filter_filename_.c_str(), Fpass_, Fstop_);
#else
                            fmcomms2_source_f32c_ = gr::iio::fmcomms2_source_f32c::make(
                                uri_.c_str(), freq_, sample_rate_,
                                bandwidth_,
                                rx1_en_, rx2_en_,
                                buffer_size_, quadrature_, rf_dc_,
                                bb_dc_, gain_mode_rx1_.c_str(), rf_gain_rx1_,
                                gain_mode_rx2_.c_str(), rf_gain_rx2_,
                                rf_port_select_.c_str(), filter_file_.c_str(),
                                filter_auto_);
#endif
                            // configure LO
                            if (enable_dds_lo_ == true)
                                {
                                    if (tx_bandwidth_ < static_cast<uint64_t>(std::floor(static_cast<float>(freq_dds_tx_hz_) * 1.1)) or (tx_bandwidth_ < 200000) or (tx_bandwidth_ > 1000000))
                                        {
                                            std::cout << "Configuration parameter tx_bandwidth value should be between " << std::max(static_cast<float>(freq_dds_tx_hz_) * 1.1, 200000.0) << " and 1000000 Hz\n";
                                            std::cout << "Error: provided value tx_bandwidth=" << tx_bandwidth_ << " is not among valid values\n";
                                            std::cout << " This parameter has been set to its default value tx_bandwidth=500000\n";
                                            tx_bandwidth_ = 500000;
                                            LOG(WARNING) << "Invalid configuration value for tx_bandwidth parameter. Set to tx_bandwidth=500000";
                                        }
                                    if (tx_attenuation_db_ > 0.0 or tx_attenuation_db_ < -89.75)
                                        {
                                            std::cout << "Configuration parameter tx_attenuation_db should take values between 0.0 and -89.95 in 0.25 dB steps\n";
                                            std::cout << "Error: provided value tx_attenuation_db=" << tx_attenuation_db_ << " is not among valid values\n";
                                            std::cout << " This parameter has been set to its default value tx_attenuation_db=" << default_tx_attenuation_db << '\n';
                                            tx_attenuation_db_ = default_tx_attenuation_db;
                                            LOG(WARNING) << "Invalid configuration value for tx_attenuation_db parameter. Set to tx_attenuation_db=" << default_tx_attenuation_db;
                                        }
                                    std::cout << "Enabling Local Oscillator generator in FMCOMMS2\n";
                                    try
                                        {
                                            config_ad9361_lo_remote(uri_,
                                                tx_bandwidth_,
                                                sample_rate_,
                                                freq_rf_tx_hz_,
                                                tx_attenuation_db_,
                                                freq_dds_tx_hz_,
                                                scale_dds_dbfs_,
                                                phase_dds_deg_);
                                        }
                                    catch (const std::runtime_error &e)
                                        {
                                            std::cout << "Exception cached when configuring the TX carrier: " << e.what() << '\n';
                                        }
                                }
                        }
                }
            else if (RF_channels_ == 2)
                {
                    if (!(rx1_en_ and rx2_en_))
                        {
                            LOG(FATAL) << "Configuration error: RF_channels=2 but are not enabled both receivers in FMCOMMS2 !";
                        }
                    else
                        {
#if GNURADIO_API_IIO
                            fmcomms2_source_f32c_ = gr::iio::fmcomms2_source_f32c::make(
                                uri_.c_str(), freq_, sample_rate_,
                                bandwidth_,
                                rx1_en_, rx2_en_,
                                buffer_size_, quadrature_, rf_dc_,
                                bb_dc_, gain_mode_rx1_.c_str(), rf_gain_rx1_,
                                gain_mode_rx2_.c_str(), rf_gain_rx2_,
                                rf_port_select_.c_str(), filter_source_.c_str(),
                                filter_filename_.c_str(), Fpass_, Fstop_);
#else
                            fmcomms2_source_f32c_ = gr::iio::fmcomms2_source_f32c::make(
                                uri_.c_str(), freq_, sample_rate_,
                                bandwidth_,
                                rx1_en_, rx2_en_,
                                buffer_size_, quadrature_, rf_dc_,
                                bb_dc_, gain_mode_rx1_.c_str(), rf_gain_rx1_,
                                gain_mode_rx2_.c_str(), rf_gain_rx2_,
                                rf_port_select_.c_str(), filter_file_.c_str(),
                                filter_auto_);
#endif
                            // configure LO
                            if (enable_dds_lo_ == true)
                                {
                                    if (tx_bandwidth_ < static_cast<uint64_t>(std::floor(static_cast<float>(freq_dds_tx_hz_) * 1.1)) or (tx_bandwidth_ < 200000) or (tx_bandwidth_ > 1000000))
                                        {
                                            std::cout << "Configuration parameter tx_bandwidth value should be between " << std::max(static_cast<float>(freq_dds_tx_hz_) * 1.1, 200000.0) << " and 1000000 Hz\n";
                                            std::cout << "Error: provided value tx_bandwidth=" << tx_bandwidth_ << " is not among valid values\n";
                                            std::cout << " This parameter has been set to its default value tx_bandwidth=500000\n";
                                            tx_bandwidth_ = 500000;
                                            LOG(WARNING) << "Invalid configuration value for tx_bandwidth parameter. Set to tx_bandwidth=500000";
                                        }
                                    if (tx_attenuation_db_ > 0.0 or tx_attenuation_db_ < -89.75)
                                        {
                                            std::cout << "Configuration parameter tx_attenuation_db should take values between 0.0 and -89.95 in 0.25 dB steps\n";
                                            std::cout << "Error: provided value tx_attenuation_db=" << tx_attenuation_db_ << " is not among valid values\n";
                                            std::cout << " This parameter has been set to its default value tx_attenuation_db=" << default_tx_attenuation_db << '\n';
                                            tx_attenuation_db_ = default_tx_attenuation_db;
                                            LOG(WARNING) << "Invalid configuration value for tx_attenuation_db parameter. Set to tx_attenuation_db=" << default_tx_attenuation_db;
                                        }
                                    std::cout << "Enabling Local Oscillator generator in FMCOMMS2\n";
                                    try
                                        {
                                            config_ad9361_lo_remote(uri_,
                                                tx_bandwidth_,
                                                sample_rate_,
                                                freq_rf_tx_hz_,
                                                tx_attenuation_db_,
                                                freq_dds_tx_hz_,
                                                scale_dds_dbfs_,
                                                phase_dds_deg_);
                                        }
                                    catch (const std::runtime_error &e)
                                        {
                                            std::cout << "Exception cached when configuring the TX carrier: " << e.what() << '\n';
                                        }
                                }
                        }
                }
            else
                {
                    LOG(FATAL) << "Configuration error: Unsupported number of RF_channels !";
                }
        }
    else
        {
            LOG(FATAL) << "Configuration error: item type " << item_type_ << " not supported!";
        }

    if (samples_ != 0)
        {
            DLOG(INFO) << "Send STOP signal after " << samples_ << " samples";
            valve_ = gnss_sdr_make_valve(item_size_, samples_, queue);
            DLOG(INFO) << "valve(" << valve_->unique_id() << ")";
        }

    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }
}


Fmcomms2SignalSource::~Fmcomms2SignalSource()
{
    if (rf_shutdown_)
        {
            if (!disable_ad9361_rx_remote(uri_))
                {
                    LOG(WARNING) << "Problem shutting down the AD9361 RX channels";
                }
            if (enable_dds_lo_ == true)
                {
                    try
                        {
                            ad9361_disable_lo_remote(uri_);
                        }
                    catch (const std::exception &e)
                        {
                            LOG(WARNING) << "Problem shutting down the AD9361 TX channel: " << e.what();
                        }
                }
        }
}


void Fmcomms2SignalSource::connect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->connect(fmcomms2_source_f32c_, 0, valve_, 0);
            DLOG(INFO) << "connected fmcomms2 source to valve";
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
                    top_block->connect(fmcomms2_source_f32c_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected fmcomms2 source to file sink";
                }
        }
}


void Fmcomms2SignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->disconnect(fmcomms2_source_f32c_, 0, valve_, 0);
            if (dump_)
                {
                    top_block->disconnect(valve_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (dump_)
                {
                    top_block->disconnect(fmcomms2_source_f32c_, 0, file_sink_, 0);
                }
        }
}


gr::basic_block_sptr Fmcomms2SignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return gr::basic_block_sptr();
}


gr::basic_block_sptr Fmcomms2SignalSource::get_right_block()
{
    if (samples_ != 0)
        {
            return valve_;
        }
    else
        {
            return (fmcomms2_source_f32c_);
        }
}
