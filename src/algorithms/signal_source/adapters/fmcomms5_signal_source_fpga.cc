/*!
 * \file fmcomms5_signal_source_fpga.cc
 * \brief signal source for the Analog Devices FMCOMMS5 directly connected
 * to the FPGA accelerators.
 * This source implements only the AD9361 control. It is NOT compatible with
 * conventional SDR acquisition and tracking blocks.
 * Please use the fmcomms2 source if conventional SDR acquisition and tracking
 * is selected in the configuration file.
 * \authors <ul>
 *          <li> Javier Arribas, jarribas(at)cttc.es
 *          <li> Marc Majoral, mmajoral(at)cttc.es
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "fmcomms5_signal_source_fpga.h"
#include "GPS_L1_CA.h"
#include "GPS_L5.h"
#include "ad9361_manager.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_string_literals.h"
#include <algorithm>  // for std::max
#include <chrono>     // for std::chrono
#include <cmath>      // for std::floor
#include <exception>  // for std::exception
#include <iostream>   // for std::cout

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/check.h>
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

Fmcomms5SignalSourceFPGA::Fmcomms5SignalSourceFPGA(const ConfigurationInterface *configuration,
    const std::string &role, unsigned int in_stream, unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t> *queue __attribute__((unused)))
    : SignalSourceBase(configuration, role, "FMCOMMS5_Signal_Source_FPGA"s),
      gain_mode_rx1_(configuration->property(role + ".gain_mode_rx1", default_gain_mode)),
      gain_mode_rx2_(configuration->property(role + ".gain_mode_rx2", default_gain_mode)),
      rf_port_select_(configuration->property(role + ".rf_port_select", default_rf_port_select)),
      filter_filename_(configuration->property(role + ".filter_filename", filter_file_)),
      rf_gain_rx1_(configuration->property(role + ".gain_rx1", default_manual_gain_rx1)),
      rf_gain_rx2_(configuration->property(role + ".gain_rx2", default_manual_gain_rx2)),
      freq0_(configuration->property(role + ".freq0", static_cast<uint64_t>(GPS_L1_FREQ_HZ))),
      freq1_(configuration->property(role + ".freq1", static_cast<uint64_t>(GPS_L5_FREQ_HZ))),
      sample_rate_(configuration->property(role + ".sampling_frequency", default_bandwidth)),
      bandwidth_(configuration->property(role + ".bandwidth", default_bandwidth)),
      Fpass_(configuration->property(role + ".Fpass", static_cast<float>(0.0))),
      Fstop_(configuration->property(role + ".Fstop", static_cast<float>(0.0))),
      in_stream_(in_stream),
      out_stream_(out_stream),
      item_size_(sizeof(int8_t)),
      filter_auto_(configuration->property(role + ".filter_auto", false)),
      quadrature_(configuration->property(role + ".quadrature", true)),
      rf_dc_(configuration->property(role + ".rf_dc", true)),
      bb_dc_(configuration->property(role + ".bb_dc", true)),
      rx1_enable_(configuration->property(role + ".rx1_enable", true)),
      rx2_enable_(configuration->property(role + ".rx2_enable", true)),
      enable_dynamic_bit_selection_(configuration->property(role + ".enable_dynamic_bit_selection", true)),
      enable_ovf_check_buffer_monitor_active_(true),
      dump_(configuration->property(role + ".dump", false)),
#if USE_GLOG_AND_GFLAGS
      rf_shutdown_(configuration->property(role + ".rf_shutdown", FLAGS_rf_shutdown))
#else
      rf_shutdown_(configuration->property(role + ".rf_shutdown", absl::GetFlag(FLAGS_rf_shutdown)))
#endif
{
    const bool enable_rx1_band((configuration->property("Channels_1C.count", 0) > 0) ||
                               (configuration->property("Channels_1B.count", 0) > 0));
    const bool enable_rx2_band((configuration->property("Channels_L2.count", 0) > 0) ||
                               (configuration->property("Channels_L5.count", 0) > 0) ||
                               (configuration->property("Channels_5X.count", 0) > 0));

    const uint32_t num_freq_bands = ((enable_rx1_band == true) and (enable_rx2_band == true)) ? 2 : 1;

    if (filter_auto_)
        {
            filter_source_ = configuration->property(role + ".filter_source", std::string("Auto"));
        }
    else
        {
            filter_source_ = configuration->property(role + ".filter_source", std::string("Off"));
        }

    switch_fpga = std::make_shared<Fpga_Switch>();
    switch_fpga->set_switch_position(switch_to_real_time_mode);

    std::cout << "Sample rate: " << sample_rate_ << " Sps\n";

    // some basic checks
    if ((rf_port_select_ != "A_BALANCED") && (rf_port_select_ != "B_BALANCED") && (rf_port_select_ != "A_N") && (rf_port_select_ != "B_N") && (rf_port_select_ != "B_P") && (rf_port_select_ != "C_N") && (rf_port_select_ != "C_P") && (rf_port_select_ != "TX_MONITOR1") && (rf_port_select_ != "TX_MONITOR2") && (rf_port_select_ != "TX_MONITOR1_2"))
        {
            std::cout << "Configuration parameter rf_port_select should take one of these values:\n";
            std::cout << " A_BALANCED, B_BALANCED, A_N, B_N, B_P, C_N, C_P, TX_MONITOR1, TX_MONITOR2, TX_MONITOR1_2\n";
            std::cout << "Error: provided value rf_port_select=" << rf_port_select_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value rf_port_select=" << default_rf_port_select << '\n';
            rf_port_select_ = default_rf_port_select;
            LOG(WARNING) << "Invalid configuration value for rf_port_select parameter. Set to rf_port_select=" << default_rf_port_select;
        }

    if ((gain_mode_rx1_ != "manual") && (gain_mode_rx1_ != "slow_attack") && (gain_mode_rx1_ != "fast_attack") && (gain_mode_rx1_ != "hybrid"))
        {
            std::cout << "Configuration parameter gain_mode_rx1 should take one of these values:\n";
            std::cout << " manual, slow_attack, fast_attack, hybrid\n";
            std::cout << "Error: provided value gain_mode_rx1=" << gain_mode_rx1_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value gain_mode_rx1=" << default_gain_mode << '\n';
            gain_mode_rx1_ = default_gain_mode;
            LOG(WARNING) << "Invalid configuration value for gain_mode_rx1 parameter. Set to gain_mode_rx1=" << default_gain_mode;
        }

    if ((gain_mode_rx2_ != "manual") && (gain_mode_rx2_ != "slow_attack") && (gain_mode_rx2_ != "fast_attack") && (gain_mode_rx2_ != "hybrid"))
        {
            std::cout << "Configuration parameter gain_mode_rx2 should take one of these values:\n";
            std::cout << " manual, slow_attack, fast_attack, hybrid\n";
            std::cout << "Error: provided value gain_mode_rx2=" << gain_mode_rx2_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value gain_mode_rx2=" << default_gain_mode << '\n';
            gain_mode_rx2_ = default_gain_mode;
            LOG(WARNING) << "Invalid configuration value for gain_mode_rx2 parameter. Set to gain_mode_rx2=" << default_gain_mode;
        }

    if (gain_mode_rx1_ == "manual")
        {
            if (rf_gain_rx1_ > 73.0 || rf_gain_rx1_ < -1.0)
                {
                    std::cout << "Configuration parameter rf_gain_rx1 should take values between -1.0 and 73 dB\n";
                    std::cout << "Error: provided value rf_gain_rx1=" << rf_gain_rx1_ << " is not among valid values\n";
                    std::cout << " This parameter has been set to its default value rf_gain_rx1=" << default_manual_gain_rx1 << '\n';
                    rf_gain_rx1_ = default_manual_gain_rx1;
                    LOG(WARNING) << "Invalid configuration value for rf_gain_rx1 parameter. Set to rf_gain_rx1=" << default_manual_gain_rx1;
                }
        }

    if (gain_mode_rx2_ == "manual")
        {
            if (rf_gain_rx2_ > 73.0 || rf_gain_rx2_ < -1.0)
                {
                    std::cout << "Configuration parameter rf_gain_rx2 should take values between -1.0 and 73 dB\n";
                    std::cout << "Error: provided value rf_gain_rx2=" << rf_gain_rx2_ << " is not among valid values\n";
                    std::cout << " This parameter has been set to its default value rf_gain_rx2=" << default_manual_gain_rx2 << '\n';
                    rf_gain_rx2_ = default_manual_gain_rx2;
                    LOG(WARNING) << "Invalid configuration value for rf_gain_rx2 parameter. Set to rf_gain_rx2=" << default_manual_gain_rx2;
                }
        }

    if ((filter_source_ != "Off") && (filter_source_ != "Auto") && (filter_source_ != "File") && (filter_source_ != "Design"))
        {
            std::cout << "Configuration parameter filter_source should take one of these values:\n";
            std::cout << "  Off: Disable filter\n";
            std::cout << "  Auto: Use auto-generated filters\n";
            std::cout << "  File: User-provided filter in filter_filename parameter\n";
            std::cout << "  Design: Create filter from Fpass, Fstop, sampling_frequency and bandwidth parameters\n";
            std::cout << "Error: provided value filter_source=" << filter_source_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value filter_source=Off\n";
            filter_source_ = std::string("Off");
            LOG(WARNING) << "Invalid configuration value for filter_source parameter. Set to filter_source=Off";
        }

    if (bandwidth_ < 200000 || bandwidth_ > 56000000)
        {
            std::cout << "Configuration parameter bandwidth should take values between 200000 and 56000000 Hz\n";
            std::cout << "Error: provided value bandwidth=" << bandwidth_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value bandwidth=" << default_bandwidth << '\n';
            bandwidth_ = default_bandwidth;
            LOG(WARNING) << "Invalid configuration value for bandwidth parameter. Set to bandwidth=" << default_bandwidth;
        }

    if (enable_rx1_band)
        {
            std::cout << "LO 0 frequency : " << freq0_ << " Hz\n";
        }
    if (enable_rx2_band)
        {
            std::cout << "LO 1 frequency : " << freq1_ << " Hz\n";
        }
    try
        {
            config_ad9361_rx_local(bandwidth_,
                sample_rate_,
                freq0_,
                freq1_,
                rf_port_select_,
                rx1_enable_,
                rx2_enable_,
                gain_mode_rx1_,
                gain_mode_rx2_,
                rf_gain_rx1_,
                rf_gain_rx2_,
                quadrature_,
                rf_dc_,
                bb_dc_,
                filter_source_,
                filter_filename_,
                Fpass_,
                Fstop_);
        }
    catch (const std::runtime_error &e)
        {
            std::cerr << "Exception cached when configuring the RX chain: " << e.what() << '\n';
            return;
        }

    std::string dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);

    buffer_monitor_fpga = std::make_shared<Fpga_buffer_monitor>(num_freq_bands, dump_, dump_filename);
    thread_buffer_monitor = std::thread([&] { run_buffer_monitor_process(); });


    // dynamic bits selection
    if (enable_dynamic_bit_selection_)
        {
            dynamic_bit_selection_fpga = std::make_shared<Fpga_dynamic_bit_selection>(enable_rx1_band, enable_rx2_band);
            thread_dynamic_bit_selection = std::thread([&] { run_dynamic_bit_selection_process(); });
        }

    if (in_stream_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


Fmcomms5SignalSourceFPGA::~Fmcomms5SignalSourceFPGA()
{
    /* cleanup and exit */

    if (rf_shutdown_)
        {
            std::cout << "* Disabling RX streaming channels\n";
            if (!disable_ad9361_rx_local())
                {
                    LOG(WARNING) << "Problem shutting down the AD9361 RX channels";
                }
        }

    // disable buffer overflow checking and buffer monitoring
    std::unique_lock<std::mutex> lock_buffer_monitor(buffer_monitor_mutex);
    enable_ovf_check_buffer_monitor_active_ = false;
    lock_buffer_monitor.unlock();

    if (thread_buffer_monitor.joinable())
        {
            thread_buffer_monitor.join();
        }

    std::unique_lock<std::mutex> lock_dyn_bit_sel(dynamic_bit_selection_mutex);
    bool bit_selection_enabled = enable_dynamic_bit_selection_;
    lock_dyn_bit_sel.unlock();

    if (bit_selection_enabled == true)
        {
            std::unique_lock<std::mutex> lock(dynamic_bit_selection_mutex);
            enable_dynamic_bit_selection_ = false;
            lock.unlock();

            if (thread_dynamic_bit_selection.joinable())
                {
                    thread_dynamic_bit_selection.join();
                }
        }
}


void Fmcomms5SignalSourceFPGA::run_dynamic_bit_selection_process()
{
    bool dynamic_bit_selection_active = true;

    while (dynamic_bit_selection_active)
        {
            // setting the bit selection to the top bits
            dynamic_bit_selection_fpga->bit_selection();
            std::this_thread::sleep_for(std::chrono::milliseconds(Gain_control_period_ms));
            std::unique_lock<std::mutex> lock(dynamic_bit_selection_mutex);
            if (enable_dynamic_bit_selection_ == false)
                {
                    dynamic_bit_selection_active = false;
                }
            lock.unlock();
        }
}


void Fmcomms5SignalSourceFPGA::run_buffer_monitor_process()
{
    bool enable_ovf_check_buffer_monitor_active = true;

    std::this_thread::sleep_for(std::chrono::milliseconds(buffer_monitoring_initial_delay_ms));

    while (enable_ovf_check_buffer_monitor_active)
        {
            buffer_monitor_fpga->check_buffer_overflow_and_monitor_buffer_status();
            std::this_thread::sleep_for(std::chrono::milliseconds(buffer_monitor_period_ms));
            std::unique_lock<std::mutex> lock(buffer_monitor_mutex);
            if (enable_ovf_check_buffer_monitor_active_ == false)
                {
                    enable_ovf_check_buffer_monitor_active = false;
                }
            lock.unlock();
        }
}


void Fmcomms5SignalSourceFPGA::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "AD9361 FPGA source nothing to connect";
}


void Fmcomms5SignalSourceFPGA::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "AD9361 FPGA source nothing to disconnect";
}


gr::basic_block_sptr Fmcomms5SignalSourceFPGA::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return {};
}


gr::basic_block_sptr Fmcomms5SignalSourceFPGA::get_right_block()
{
    return {};
}
