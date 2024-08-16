/*!
 * \file max2771_evkit_signal_source_fpga.cc
 * \brief Signal source for the MAX2771EVKIT evaluation board connected directly
 * to FPGA accelerators.
 * This source implements only the MAX2771 control. It is NOT compatible with
 * conventional SDR acquisition and tracking blocks.
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

#include "max2771_evkit_signal_source_fpga.h"
#include "GPS_L1_CA.h"
#include "GPS_L2C.h"
#include "GPS_L5.h"
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

MAX2771EVKITSignalSourceFPGA::MAX2771EVKITSignalSourceFPGA(const ConfigurationInterface *configuration,
    const std::string &role, unsigned int in_stream, unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t> *queue __attribute__((unused)))
    : SignalSourceBase(configuration, role, "MAX2771_EVKIT_Signal_Source_FPGA"s),
      freq_(configuration->property(role + ".freq", static_cast<uint64_t>(GPS_L1_FREQ_HZ))),
      sample_rate_(configuration->property(role + ".sampling_frequency", default_sampling_rate)),
      in_stream_(in_stream),
      out_stream_(out_stream),
      bandwidth_(configuration->property(role + ".bandwidth", default_bandwidth)),
      filter_order_(configuration->property(role + ".filter_order", default_filter_order)),
      gain_in_(configuration->property(role + ".PGA_gain", default_PGA_gain_value)),
      item_size_(sizeof(int8_t)),
      chipen_(true),
      if_filter_gain_(configuration->property(role + ".enable_IF_filter_gain", true)),
      LNA_active_(configuration->property(role + ".LNA_active", false)),
      enable_agc_(configuration->property(role + ".enable_AGC", true)),
      enable_ovf_check_buffer_monitor_active_(true),
      dump_(configuration->property(role + ".dump", false)),
#if USE_GLOG_AND_GFLAGS
      rf_shutdown_(configuration->property(role + ".rf_shutdown", FLAGS_rf_shutdown))
#else
      rf_shutdown_(configuration->property(role + ".rf_shutdown", absl::GetFlag(FLAGS_rf_shutdown)))
#endif
{
    // some basic checks
    if (freq_ != GPS_L1_FREQ_HZ and freq_ != GPS_L2_FREQ_HZ and freq_ != GPS_L5_FREQ_HZ)
        {
            std::cout << "Configuration parameter freq should take values " << GPS_L1_FREQ_HZ << ", " << GPS_L2_FREQ_HZ << ", or " << GPS_L5_FREQ_HZ << "\n";
            std::cout << "Error: provided value freq = " << freq_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value freq = " << GPS_L1_FREQ_HZ << '\n';
            LOG(WARNING) << "Invalid configuration value for freq parameter. Set to freq = " << GPS_L1_FREQ_HZ;
            freq_ = GPS_L1_FREQ_HZ;
        }
    if (sample_rate_ != 4092000 and sample_rate_ != 8184000 and sample_rate_ != 16368000 and sample_rate_ != 32736000)
        {
            std::cout << "Configuration parameter sampling_frequency should take values 4092000, 8184000, 16368000, or 32736000\n";
            std::cout << "Error: provided value sampling_frequency = " << sample_rate_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value sampling_frequency = " << default_sampling_rate << '\n';
            LOG(WARNING) << "Invalid configuration value for sampling_frequency parameter. Set to sampling_frequency = " << default_sampling_rate;
            sample_rate_ = default_sampling_rate;
        }
    if (bandwidth_ != 2500000 and bandwidth_ != 4200000 and bandwidth_ != 8700000 and bandwidth_ != 16400000 and bandwidth_ != 23400000 and bandwidth_ != 36000000)
        {
            std::cout << "Configuration parameter bandwidth can only take the following values: 2500000, 4200000, 8700000, 16400000, 23400000, and 36000000 Hz\n";
            std::cout << "Error: provided value bandwidth = " << bandwidth_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value bandwidth = " << default_bandwidth << '\n';
            LOG(WARNING) << "Invalid configuration value for bandwidth parameter. Set to bandwidth = " << default_bandwidth;
            bandwidth_ = default_bandwidth;
        }
    if (filter_order_ != 3 and filter_order_ != 5)
        {
            std::cout << "Configuration parameter filter_order should take values 3 or 5\n";
            std::cout << "Error: provided value filter_order = " << filter_order_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value filter_order = " << default_filter_order << '\n';
            LOG(WARNING) << "Invalid configuration value for filter_order parameter. Set to filter_order = " << default_filter_order;
            filter_order_ = default_filter_order;
        }
    if (gain_in_ > max_PGA_gain_value)
        {
            std::cout << "Configuration parameter PGA_gain should be up to " << max_PGA_gain_value << "\n";
            std::cout << "Error: provided value PGA_gain = " << gain_in_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value PGA_gain = " << default_PGA_gain_value << '\n';
            LOG(WARNING) << "Invalid configuration value for PGA_gain parameter. Set to PGA_gain = " << default_PGA_gain_value;
            gain_in_ = default_PGA_gain_value;
        }

    std::vector<uint32_t> register_values = setup_regs();

    spidev_fpga = std::make_shared<Fpga_spidev>();

    if (spidev_fpga->SPI_open())
        {
            std::cerr << "Cannot open SPI device\n";
            // stop the receiver
            queue->push(pmt::make_any(command_event_make(200, 0)));
            return;
        }

    if (configure(register_values))
        {
            std::cerr << "Error configuring the MAX2771 device " << '\n';
        }

    if (spidev_fpga->SPI_close())
        {
            std::cerr << "Error closing SPI device " << '\n';
        }

    std::string dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);

    buffer_monitor_fpga = std::make_shared<Fpga_buffer_monitor>(NUM_FREQ_BANDS, dump_, dump_filename);
    thread_buffer_monitor = std::thread([&] { run_buffer_monitor_process(); });

    if (in_stream_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}

std::vector<uint32_t> MAX2771EVKITSignalSourceFPGA::setup_regs(void)
{
    std::vector<uint32_t> register_values = std::vector<uint32_t>(MAX2771_NUM_REGS);


    uint32_t LNA_mode = (LNA_active_) ? 0x0 : 0x2;

    uint32_t Filter_Bandwidth;

    switch (bandwidth_)
        {
        case 2500000:
            Filter_Bandwidth = 0x0;
            break;
        case 4200000:
            Filter_Bandwidth = 0x2;
            break;
        case 8700000:
            Filter_Bandwidth = 0x1;
            break;
        case 16400000:
            Filter_Bandwidth = 0x7;
            break;
        case 23400000:
            Filter_Bandwidth = 0x3;
            break;
        case 36000000:
            Filter_Bandwidth = 0x4;
            break;
        default:
            Filter_Bandwidth = 0x0;  // default bandwidth
        }

    uint32_t chipen_select = (chipen_) ? 0x1 : 0x0;
    uint32_t Filter_order_sel = (filter_order_ == 5) ? 0x0 : 0x1;
    uint32_t IF_filter_gain_sel = (if_filter_gain_) ? 0x1 : 0x0;

    register_values[0] =  // configuration 1 register
        (chipen_select << 31) +
        (IDLE << 30) +
        (0x8 << 26) +  // reserved
        (0x8 << 22) +  // reserved
        (0x2 << 20) +  // reserved
        (0x1 << 18) +  // reserved
        (MIXPOLE << 17) +
        (LNA_mode << 15) +
        (MIXERMODE << 13) +
        (FCEN << 6) +
        (Filter_Bandwidth << 3) +
        (Filter_order_sel << 2) +
        (FCENX << 1) +
        IF_filter_gain_sel;

    uint32_t AGC_mode = (enable_agc_) ? 0x0 : 0x2;

    register_values[1] =  // configuration 2 register
        (0x0 << 31) +     // reserved
        (0x1 << 29) +     // reserved
        (ANAIMON << 28) +
        (IQEN << 27) +
        (GAINREF << 15) +
        (SPI_SDIO_CONFIG << 13) +
        (AGC_mode << 11) +
        (FORMAT << 9) +
        (BITS << 6) +
        (DRVCFG << 4) +
        (0x1 << 3) +  // reserved
        (0x0 << 2) +  // reserved
        DIEID;

    register_values[2] =  // configuration 3 register
        (0x0 << 28) +     // reserved
        (gain_in_ << 22) +
        (0x1 << 21) +  // reserved
        (HILOADEN << 20) +
        (0x1 << 19) +  // reserved
        (0x1 << 18) +  // reserved
        (0x1 << 17) +  // reserved
        (0x1 << 16) +  // reserved
        (FHIPEN << 15) +
        (0x0 << 14) +  // reserved
        (PGAIEN << 13) +
        (PGAQEN << 12) +
        (STRMEN << 11) +
        (STRMSTART << 10) +
        (STRMSTOP << 9) +
        (0x7 << 6) +  // reserved
        (STRMBITS << 4) +
        (STAMPEN << 3) +
        (TIMESYNCEN << 2) +
        (DATASYNCEN << 1) +
        STRMRST;

    uint32_t clock_out_div_ratio;

    switch (sample_rate_)
        {
        case 4092000:
            clock_out_div_ratio = 0x1;  // XTAL frequency /4
            break;
        case 8184000:
            clock_out_div_ratio = 0x2;  // XTAL frequency /2
            break;
        case 16368000:
            clock_out_div_ratio = 0x3;  // XTAL frequency
            break;
        case 32736000:
            clock_out_div_ratio = 0x0;  // XTAL Frequency x2
            break;
        default:
            clock_out_div_ratio = 0x1;  // default XTAL frequency
        }

    register_values[3] =  // PLL configuration register
        (clock_out_div_ratio << 29) +
        (LOBAND << 28) +
        (0x1 << 27) +  // reserved
        (0x0 << 26) +  // reserved
        (0x0 << 25) +  // reserved
        (REFOUTEN << 24) +
        (0x1 << 23) +  // reserved
        (0x0 << 21) +  // reserved
        (IXTAL << 19) +
        (0x10 << 14) +  // reserved
        (0x0 << 13) +   // reserved
        (0x0 << 10) +   // reserved
        (ICP << 9) +
        (0x0 << 8) +  // reserved
        (0x0 << 7) +  // reserved
        (0x0 << 4) +  // reserved
        (INT_PLL << 3) +
        (PWRSAV << 2) +
        (0x0 << 1) +  // reserved
        0x0;          // reserved

    uint32_t freq_sel;
    switch (freq_)
        {
        case static_cast<uint64_t>(GPS_L1_FREQ_HZ):
            freq_sel = 0x604;
            break;
        case static_cast<uint64_t>(GPS_L2_FREQ_HZ):
            freq_sel = 0x4B0;
            break;
        case static_cast<uint64_t>(GPS_L5_FREQ_HZ):
            freq_sel = 0x47E;
            break;
        default:
            freq_sel = 0x604;
        }

    register_values[4] =  // PLL integer division register
        (0x0 << 28) +     // reserved
        (freq_sel << 13) +
        (RDIV << 3) +
        0x0;  // reserved

    register_values[5] =  // PLL fractional division register
        (0x0 << 28) +     // reserved
        (FDIV << 8) +
        (0x7 << 4) +  // reserved
        (0x0 << 3) +  // reserved
        (0x0 << 2) +  // reserved
        (0x0 << 1) +  // reserved
        0x0;          // reserved

    register_values[6] =  // DSP interface register
        (0x0 << 28) +     // reserved
        0x8000000;        // reserved

    register_values[7] =  // clock configuration 1 register
        (0x0 << 29) +     // reserved
        (EXTADCCLK << 28) +
        (REFCLK_L_CNT << 16) +
        (REFCLK_M_CNT << 4) +
        (FCLKIN << 3) +
        (ADCCLK << 2) +
        (0x1 << 1) +  // reserved
        MODE;

    register_values[8] = TEST_MODE_1_REG_VAL;  // test mode 1 register

    register_values[9] = TEST_MODE_2_REG_VAL;  // test mode 2 register

    register_values[10] =  // clock configuration 2 register
        (0x0 << 29) +      // reserved
        (0x0 << 28) +      // reserved
        (ADCCLK_L_CNT << 16) +
        (ADCCLK_M_CNT << 4) +
        (PRE_FRACDIV_SEL << 3) +
        (CLKOUT_SEL << 2) +
        0x0;  // reserved

    return register_values;
}


bool MAX2771EVKITSignalSourceFPGA::configure(std::vector<uint32_t> register_values)
{
    // write the registers
    std::cerr << "Configuring MAX2771 registers" << std::endl;
    uint32_t status = 0;
    for (uint32_t k = 0; k < register_values.size(); k++)
        {
            status = spidev_fpga->write_reg32(k, register_values[k]);
            if (status)
                {
                    std::cerr << "Error writing the MAX2771 registers" << std::endl;
                    break;
                }
        }

    // Read the registers and verify that the values are correctly written
    std::vector<uint32_t> reg_read = std::vector<uint32_t>(register_values.size());

    for (uint8_t n = 0; n < register_values.size(); ++n)
        {
            status = spidev_fpga->read_reg32(n, &reg_read[n]);
            if (status)
                {
                    std::cerr << "Error reading the MAX2771 registers" << std::endl;
                    return status;
                }
            else
                {
                    if (reg_read[n] != register_values[n])
                        {
                            std::cerr << "Error: Failed to verify the MAX2771 registers " << std::endl;
                            return -1;
                        }
                }
        }

    return 0;
}

MAX2771EVKITSignalSourceFPGA::~MAX2771EVKITSignalSourceFPGA()
{
    /* cleanup and exit */

    if (rf_shutdown_)
        {
            chipen_ = false;
            std::cout << "* MAX2771 Disabling RX streaming channels\n";
            std::vector<uint32_t> register_values = setup_regs();

            if (spidev_fpga->SPI_open())
                {
                    std::cerr << "Cannot open SPI device\n";
                    return;
                }


            if (configure(register_values))
                {
                    std::cerr << "Error disabling the MAX2771 device " << '\n';
                }

            if (spidev_fpga->SPI_close())
                {
                    std::cerr << "Error closing SPI device " << '\n';
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
}


void MAX2771EVKITSignalSourceFPGA::run_buffer_monitor_process()
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


void MAX2771EVKITSignalSourceFPGA::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "AD9361 FPGA source nothing to connect";
}


void MAX2771EVKITSignalSourceFPGA::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "AD9361 FPGA source nothing to disconnect";
}


gr::basic_block_sptr MAX2771EVKITSignalSourceFPGA::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return {};
}


gr::basic_block_sptr MAX2771EVKITSignalSourceFPGA::get_right_block()
{
    return {};
}
