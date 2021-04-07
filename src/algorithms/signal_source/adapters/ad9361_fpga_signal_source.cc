/*!
 * \file ad9361_fpga_signal_source.cc
 * \brief signal source for Analog Devices front-end AD9361 connected directly to FPGA accelerators.
 * This source implements only the AD9361 control. It is NOT compatible with conventional SDR acquisition and tracking blocks.
 * Please use the fmcomms2 source if conventional SDR acquisition and tracking is selected in the configuration file.
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
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "ad9361_fpga_signal_source.h"
#include "GPS_L1_CA.h"
#include "GPS_L5.h"
#include "ad9361_manager.h"
#include "command_event.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_string_literals.h"
#include "uio_fpga.h"
#include <glog/logging.h>
#include <iio.h>
#include <algorithm>  // for max
#include <chrono>     // for std::this_thread
#include <cmath>      // for abs
#include <exception>  // for exceptions
#include <fcntl.h>    // for open, O_WRONLY
#include <fstream>    // for std::ifstream
#include <iomanip>    // for std::setprecision
#include <iostream>   // for cout
#include <string>     // for string manipulation
#include <thread>     // for std::chrono
#include <unistd.h>   // for write
#include <utility>
#include <vector>


using namespace std::string_literals;

Ad9361FpgaSignalSource::Ad9361FpgaSignalSource(const ConfigurationInterface *configuration,
    const std::string &role, unsigned int in_stream, unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t> *queue __attribute__((unused)))
    : SignalSourceBase(configuration, role, "Ad9361_Fpga_Signal_Source"s), in_stream_(in_stream), out_stream_(out_stream), queue_(queue)
{
    // initialize the variables that are used in real-time mode

    const std::string default_gain_mode("slow_attack");
    const double default_tx_attenuation_db = -10.0;
    const double default_manual_gain_rx1 = 64.0;
    const double default_manual_gain_rx2 = 64.0;
    const uint64_t default_bandwidth = 12500000;
    const std::string default_rf_port_select("A_BALANCED");
    freq_ = configuration->property(role + ".freq", static_cast<uint64_t>(GPS_L1_FREQ_HZ));
    sample_rate_ = configuration->property(role + ".sampling_frequency", static_cast<uint64_t>(12500000));
    bandwidth_ = configuration->property(role + ".bandwidth", default_bandwidth);
    quadrature_ = configuration->property(role + ".quadrature", true);
    rf_dc_ = configuration->property(role + ".rf_dc", true);
    bb_dc_ = configuration->property(role + ".bb_dc", true);
    rx1_enable_ = configuration->property(role + ".rx1_enable", true);
    rx2_enable_ = configuration->property(role + ".rx2_enable", true);
    gain_mode_rx1_ = configuration->property(role + ".gain_mode_rx1", default_gain_mode);
    gain_mode_rx2_ = configuration->property(role + ".gain_mode_rx2", default_gain_mode);
    rf_gain_rx1_ = configuration->property(role + ".gain_rx1", default_manual_gain_rx1);
    rf_gain_rx2_ = configuration->property(role + ".gain_rx2", default_manual_gain_rx2);
    rf_port_select_ = configuration->property(role + ".rf_port_select", default_rf_port_select);
    filter_file_ = configuration->property(role + ".filter_file", std::string(""));
    filter_filename_ = configuration->property(role + ".filter_filename", filter_file_);
    filter_auto_ = configuration->property(role + ".filter_auto", false);
    if (filter_auto_)
        {
            filter_source_ = configuration->property(role + ".filter_source", std::string("Auto"));
        }
    else
        {
            filter_source_ = configuration->property(role + ".filter_source", std::string("Off"));
        }
    Fpass_ = configuration->property(role + ".Fpass", static_cast<float>(0.0));
    Fstop_ = configuration->property(role + ".Fstop", static_cast<float>(0.0));
    enable_dds_lo_ = configuration->property(role + ".enable_dds_lo", false);
    freq_dds_tx_hz_ = configuration->property(role + ".freq_dds_tx_hz", static_cast<uint64_t>(10000));
    freq_rf_tx_hz_ = configuration->property(role + ".freq_rf_tx_hz", static_cast<uint64_t>(GPS_L1_FREQ_HZ - GPS_L5_FREQ_HZ - freq_dds_tx_hz_));
    scale_dds_dbfs_ = configuration->property(role + ".scale_dds_dbfs", -3.0);
    tx_attenuation_db_ = configuration->property(role + ".tx_attenuation_db", default_tx_attenuation_db);
    tx_bandwidth_ = configuration->property(role + ".tx_bandwidth", static_cast<uint64_t>(500000));
    phase_dds_deg_ = configuration->property(role + ".phase_dds_deg", 0.0);

    rf_shutdown_ = configuration->property(role + ".rf_shutdown", FLAGS_rf_shutdown);

    // initialize the variables that are used in post-processing mode

    enable_DMA_ = false;

    const int l1_band = configuration->property("Channels_1C.count", 0) +
                        configuration->property("Channels_1B.count", 0);

    const double default_seconds_to_skip = 0.0;

    const std::string empty_string;
    filename0 = configuration->property(role + ".filename", empty_string);

    // override value with commandline flag, if present
    if (FLAGS_signal_source != "-")
        {
            filename0 = FLAGS_signal_source;
        }
    if (FLAGS_s != "-")
        {
            filename0 = FLAGS_s;
        }

    if (filename0.empty())
        {
            filename0 = configuration->property(role + ".filename0", empty_string);
            filename1 = configuration->property(role + ".filename1", empty_string);
        }

    // by default the DMA transfers samples corresponding to two frequency bands to the FPGA
    num_freq_bands_ = 2;
    dma_buff_offset_pos_ = 0;

    // if only one input file is specified in the configuration file then:
    // if there is at least one channel assigned to frequency band 1 then the DMA transfers the samples to the L1 frequency band channels
    // otherwise the DMA transfers the samples to the L2/L5 frequency band channels
    // if more than one input file are specified then the DMA transfer the samples to both the L1 and the L2/L5 frequency channels.
    if (filename1.empty())
        {
            num_freq_bands_ = 1;
            if (l1_band != 0)
                {
                    dma_buff_offset_pos_ = 2;
                }
        }
    else
        {
            dma_buff_offset_pos_ = 2;
        }

    samples_ = configuration->property(role + ".samples", static_cast<int64_t>(0));

    const double seconds_to_skip = configuration->property(role + ".seconds_to_skip", default_seconds_to_skip);
    const size_t header_size = configuration->property(role + ".header_size", 0);
    std::string item_type = "ibyte";  // for now only the ibyte format is supported
    item_size_ = sizeof(int8_t);
    repeat_ = configuration->property(role + ".repeat", false);

    samples_to_skip_ = 0;
    if (seconds_to_skip > 0)
        {
            samples_to_skip_ = static_cast<uint64_t>(seconds_to_skip * sample_rate_) * 2;
        }
    if (header_size > 0)
        {
            samples_to_skip_ += header_size;
        }

    // check the switch status (determines real-time mode or post-processing mode)

    std::string device_io_name;  // Switch UIO device file

    // find the uio device file corresponding to the switch.
    if (find_uio_dev_file_name(device_io_name, switch_device_name, 0) < 0)
        {
            std::cerr << "Cannot find the FPGA uio device file corresponding to device name " << switch_device_name << '\n';
            item_size_ = 0;
            return;
        }

    switch_position_ = configuration->property(role + ".switch_position", 0);
    if (switch_position_ != 0 && switch_position_ != 2)
        {
            std::cout << "SignalSource.switch_position configuration parameter must be either 0: read from file(s) via DMA, or 2: read from AD9361\n";
            std::cout << "SignalSource.switch_position configuration parameter set to its default value switch_position=0 - read from file(s)\n";
            switch_position_ = 0;
        }

    switch_fpga = std::make_shared<Fpga_Switch>(device_io_name);
    switch_fpga->set_switch_position(switch_position_);

    if (switch_position_ == 0)  // Inject file(s) via DMA
        {
            enable_DMA_ = true;

            if (samples_ == 0)  // read all file
                {
                    /*!
                     * BUG workaround: The GNU Radio file source does not stop the receiver after reaching the End of File.
                     * A possible solution is to compute the file length in samples using file size, excluding the last 100 milliseconds, and enable always the
                     * valve block
                     */
                    std::ifstream file(filename0.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
                    std::ifstream::pos_type size;

                    if (file.is_open())
                        {
                            size = file.tellg();
                            DLOG(INFO) << "Total samples in the file= " << floor(static_cast<double>(size) / static_cast<double>(item_size_));
                        }
                    else
                        {
                            std::cerr << "SignalSource: Unable to open the samples file " << filename0.c_str() << '\n';
                            item_size_ = 0;
                            return;
                        }
                    std::streamsize ss = std::cout.precision();
                    std::cout << std::setprecision(16);
                    std::cout << "Processing file " << filename0 << ", which contains " << static_cast<double>(size) << " [bytes]\n";
                    std::cout.precision(ss);

                    if (size > 0)
                        {
                            const uint64_t bytes_to_skip = samples_to_skip_ * item_size_;
                            const uint64_t bytes_to_process = static_cast<uint64_t>(size) - bytes_to_skip;
                            samples_ = floor(static_cast<double>(bytes_to_process) / static_cast<double>(item_size_) - ceil(0.002 * static_cast<double>(sample_rate_)));  // process all the samples available in the file excluding at least the last 1 ms
                        }

                    if (!filename1.empty())
                        {
                            std::ifstream file(filename1.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
                            std::ifstream::pos_type size;

                            if (file.is_open())
                                {
                                    size = file.tellg();
                                    DLOG(INFO) << "Total samples in the file= " << floor(static_cast<double>(size) / static_cast<double>(item_size_));
                                }
                            else
                                {
                                    std::cerr << "SignalSource: Unable to open the samples file " << filename1.c_str() << '\n';
                                    item_size_ = 0;
                                    return;
                                }
                            std::streamsize ss = std::cout.precision();
                            std::cout << std::setprecision(16);
                            std::cout << "Processing file " << filename1 << ", which contains " << static_cast<double>(size) << " [bytes]\n";
                            std::cout.precision(ss);

                            int64_t samples_rx2 = 0;
                            if (size > 0)
                                {
                                    const uint64_t bytes_to_skip = samples_to_skip_ * item_size_;
                                    const uint64_t bytes_to_process = static_cast<uint64_t>(size) - bytes_to_skip;
                                    samples_rx2 = floor(static_cast<double>(bytes_to_process) / static_cast<double>(item_size_) - ceil(0.002 * static_cast<double>(sample_rate_)));  // process all the samples available in the file excluding at least the last 1 ms
                                }
                            samples_ = std::min(samples_, samples_rx2);
                        }
                }

            CHECK(samples_ > 0) << "File does not contain enough samples to process.";
            double signal_duration_s = (static_cast<double>(samples_) * (1 / static_cast<double>(sample_rate_))) / 2.0;

            DLOG(INFO) << "Total number samples to be processed= " << samples_ << " GNSS signal duration= " << signal_duration_s << " [s]";
            std::cout << "GNSS signal recorded time to be processed: " << signal_duration_s << " [s]\n";

            if (filename1.empty())
                {
                    DLOG(INFO) << "File source filename " << filename0;
                }
            else
                {
                    DLOG(INFO) << "File source filename rx1 " << filename0;
                    DLOG(INFO) << "File source filename rx2 " << filename1;
                }
            DLOG(INFO) << "Samples " << samples_;
            DLOG(INFO) << "Sampling frequency " << sample_rate_;
            DLOG(INFO) << "Item type " << item_type;
            DLOG(INFO) << "Item size " << item_size_;
            DLOG(INFO) << "Repeat " << repeat_;
        }
    if (switch_position_ == 2)  // Real-time via AD9361
        {
            std::cout << "Sample rate: " << sample_rate_ << " Sps\n";

            enable_ovf_check_buffer_monitor_active_ = false;  // check buffer overflow and buffer monitor disabled by default

            // some basic checks
            if ((rf_port_select_ != "A_BALANCED") and (rf_port_select_ != "B_BALANCED") and (rf_port_select_ != "A_N") and (rf_port_select_ != "B_N") and (rf_port_select_ != "B_P") and (rf_port_select_ != "C_N") and (rf_port_select_ != "C_P") and (rf_port_select_ != "TX_MONITOR1") and (rf_port_select_ != "TX_MONITOR2") and (rf_port_select_ != "TX_MONITOR1_2"))
                {
                    std::cout << "Configuration parameter rf_port_select should take one of these values:\n";
                    std::cout << " A_BALANCED, B_BALANCED, A_N, B_N, B_P, C_N, C_P, TX_MONITOR1, TX_MONITOR2, TX_MONITOR1_2\n";
                    std::cout << "Error: provided value rf_port_select=" << rf_port_select_ << " is not among valid values\n";
                    std::cout << " This parameter has been set to its default value rf_port_select=" << default_rf_port_select << '\n';
                    rf_port_select_ = default_rf_port_select;
                    LOG(WARNING) << "Invalid configuration value for rf_port_select parameter. Set to rf_port_select=" << default_rf_port_select;
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
                    LOG(WARNING) << "Invalid configuration value for gain_mode_rx2 parameter. Set to gain_mode_rx2=" << default_gain_mode;
                }

            if (gain_mode_rx1_ == "manual")
                {
                    if (rf_gain_rx1_ > 73.0 or rf_gain_rx1_ < -1.0)
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
                    if (rf_gain_rx2_ > 73.0 or rf_gain_rx2_ < -1.0)
                        {
                            std::cout << "Configuration parameter rf_gain_rx2 should take values between -1.0 and 73 dB\n";
                            std::cout << "Error: provided value rf_gain_rx2=" << rf_gain_rx2_ << " is not among valid values\n";
                            std::cout << " This parameter has been set to its default value rf_gain_rx2=" << default_manual_gain_rx2 << '\n';
                            rf_gain_rx2_ = default_manual_gain_rx2;
                            LOG(WARNING) << "Invalid configuration value for rf_gain_rx2 parameter. Set to rf_gain_rx2=" << default_manual_gain_rx2;
                        }
                }

            if ((filter_source_ != "Off") and (filter_source_ != "Auto") and (filter_source_ != "File") and (filter_source_ != "Design"))
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

            if (bandwidth_ < 200000 or bandwidth_ > 56000000)
                {
                    std::cout << "Configuration parameter bandwidth should take values between 200000 and 56000000 Hz\n";
                    std::cout << "Error: provided value bandwidth=" << bandwidth_ << " is not among valid values\n";
                    std::cout << " This parameter has been set to its default value bandwidth=" << default_bandwidth << '\n';
                    bandwidth_ = default_bandwidth;
                    LOG(WARNING) << "Invalid configuration value for bandwidth parameter. Set to bandwidth=" << default_bandwidth;
                }

            std::cout << "LO frequency : " << freq_ << " Hz\n";
            try
                {
                    config_ad9361_rx_local(bandwidth_,
                        sample_rate_,
                        freq_,
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
                    item_size_ = 0;
                    return;
                }
            // LOCAL OSCILLATOR DDS GENERATOR FOR DUAL FREQUENCY OPERATION
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
                    try
                        {
                            config_ad9361_lo_local(tx_bandwidth_,
                                sample_rate_,
                                freq_rf_tx_hz_,
                                tx_attenuation_db_,
                                freq_dds_tx_hz_,
                                scale_dds_dbfs_,
                                phase_dds_deg_);
                        }
                    catch (const std::runtime_error &e)
                        {
                            std::cerr << "Exception cached when configuring the TX carrier: " << e.what() << '\n';
                            item_size_ = 0;
                            return;
                        }
                }

            // when the receiver is working in real-time mode via AD9361 perform buffer overflow checking,
            // and if dump is enabled perform buffer monitoring
            enable_ovf_check_buffer_monitor_active_ = true;

            std::string device_io_name_buffer_monitor;

            dump_ = configuration->property(role + ".dump", false);
            std::string dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);

            // find the uio device file corresponding to the buffer monitor
            if (find_uio_dev_file_name(device_io_name_buffer_monitor, buffer_monitor_device_name, 0) < 0)
                {
                    std::cerr << "Cannot find the FPGA uio device file corresponding to device name " << buffer_monitor_device_name << '\n';
                    item_size_ = 0;
                    return;
                }

            buffer_monitor_fpga = std::make_shared<Fpga_buffer_monitor>(device_io_name_buffer_monitor, num_freq_bands_, dump_, dump_filename);
            thread_buffer_monitor = std::thread([&] { run_buffer_monitor_process(); });
        }

    // dynamic bits selection
    enable_dynamic_bit_selection_ = configuration->property(role + ".enable_dynamic_bit_selection", true);
    if (enable_dynamic_bit_selection_)
        {
            std::string device_io_name_dyn_bit_sel_0;
            std::string device_io_name_dyn_bit_sel_1;

            // find the uio device file corresponding to the dynamic bit selector 0 module.
            if (find_uio_dev_file_name(device_io_name_dyn_bit_sel_0, dyn_bit_sel_device_name, 0) < 0)
                {
                    std::cerr << "Cannot find the FPGA uio device file corresponding to device name " << dyn_bit_sel_device_name << '\n';
                    item_size_ = 0;
                    return;
                }

            // find the uio device file corresponding to the dynamic bit selector 1 module.
            if (find_uio_dev_file_name(device_io_name_dyn_bit_sel_1, dyn_bit_sel_device_name, 1) < 0)
                {
                    std::cerr << "Cannot find the FPGA uio device file corresponding to device name " << dyn_bit_sel_device_name << '\n';
                    item_size_ = 0;
                    return;
                }
            dynamic_bit_selection_fpga = std::make_shared<Fpga_dynamic_bit_selection>(device_io_name_dyn_bit_sel_0, device_io_name_dyn_bit_sel_1);
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


Ad9361FpgaSignalSource::~Ad9361FpgaSignalSource()
{
    /* cleanup and exit */
    if (switch_position_ == 0)  // read samples from a file via DMA
        {
            std::unique_lock<std::mutex> lock(dma_mutex);
            enable_DMA_ = false;  // disable the DMA
            lock.unlock();
            if (thread_file_to_dma.joinable())
                {
                    thread_file_to_dma.join();
                }
        }

    if (switch_position_ == 2)  // Real-time via AD9361
        {
            if (rf_shutdown_)
                {
                    std::cout << "* AD9361 Disabling RX streaming channels\n";
                    if (!disable_ad9361_rx_local())
                        {
                            LOG(WARNING) << "Problem shutting down the AD9361 RX channels";
                        }
                    if (enable_dds_lo_)
                        {
                            try
                                {
                                    ad9361_disable_lo_local();
                                }
                            catch (const std::exception &e)
                                {
                                    LOG(WARNING) << "Problem shutting down the AD9361 TX stream: " << e.what();
                                }
                        }
                }

            // disable buffer overflow checking and buffer monitoring
            std::unique_lock<std::mutex> lock(buffer_monitor_mutex);
            enable_ovf_check_buffer_monitor_active_ = false;
            lock.unlock();

            if (thread_buffer_monitor.joinable())
                {
                    thread_buffer_monitor.join();
                }
        }

    std::unique_lock<std::mutex> lock(dynamic_bit_selection_mutex);
    bool bit_selection_enabled = enable_dynamic_bit_selection_;
    lock.unlock();

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


void Ad9361FpgaSignalSource::start()
{
    thread_file_to_dma = std::thread([&] { run_DMA_process(filename0, filename1, samples_to_skip_, item_size_, samples_, repeat_, dma_buff_offset_pos_, queue_); });
}


void Ad9361FpgaSignalSource::run_DMA_process(const std::string &filename0, const std::string &filename1, uint64_t &samples_to_skip, size_t &item_size, int64_t &samples, bool &repeat, uint32_t &dma_buff_offset_pos, Concurrent_Queue<pmt::pmt_t> *queue)
{
    std::ifstream infile1;
    infile1.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    // open the files
    try
        {
            infile1.open(filename0, std::ios::binary);
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Exception opening file " << filename0 << '\n';
            // stop the receiver
            queue->push(pmt::make_any(command_event_make(200, 0)));
            return;
        }

    std::ifstream infile2;
    if (!filename1.empty())
        {
            infile2.exceptions(std::ifstream::failbit | std::ifstream::badbit);
            try
                {
                    infile2.open(filename1, std::ios::binary);
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cerr << "Exception opening file " << filename1 << '\n';
                    // stop the receiver
                    queue->push(pmt::make_any(command_event_make(200, 0)));
                    return;
                }
        }

    // skip the initial samples if needed
    uint64_t bytes_to_skeep = samples_to_skip * item_size;
    try
        {
            infile1.ignore(bytes_to_skeep);
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Exception skipping initial samples file " << filename0 << '\n';
            // stop the receiver
            queue->push(pmt::make_any(command_event_make(200, 0)));
            return;
        }

    if (!filename1.empty())
        {
            try
                {
                    infile2.ignore(bytes_to_skeep);
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cerr << "Exception skipping initial samples file " << filename1 << '\n';
                    // stop the receiver
                    queue->push(pmt::make_any(command_event_make(200, 0)));
                    return;
                }
        }


    // rx signal vectors
    std::vector<int8_t> input_samples(sample_block_size * 2);      // complex samples
    std::vector<int8_t> input_samples_dma(sample_block_size * 4);  // complex samples, two frequency bands

    int nread_elements = 0;  // num bytes read from the file corresponding to frequency band 1
    bool run_DMA = true;
    int num_transferred_bytes;

    // Open DMA device
    const int tx_fd = open("/dev/loop_tx", O_WRONLY);
    if (tx_fd < 0)
        {
            std::cerr << "Cannot open loop device\n";
            // stop the receiver
            queue->push(pmt::make_any(command_event_make(200, 0)));
            return;
        }

    // if only one frequency band is used then clear the samples corresponding to the unused frequency band
    uint32_t dma_index = 0;
    if (num_freq_bands_ == 1)
        {
            // if only one file is enabled then clear the samples corresponding to the frequency band that is not used.
            for (int index0 = 0; index0 < (nread_elements); index0 += 2)
                {
                    input_samples_dma[dma_index + (2 - dma_buff_offset_pos)] = 0;
                    input_samples_dma[dma_index + 1 + (2 - dma_buff_offset_pos)] = 0;
                    dma_index += 4;
                }
        }

    uint64_t nbytes_remaining = samples * item_size;
    uint32_t read_buffer_size = sample_block_size * 2;  // complex samples

    // run the DMA
    while (run_DMA)
        {
            dma_index = 0;
            if (nbytes_remaining < read_buffer_size)
                {
                    read_buffer_size = nbytes_remaining;
                }
            nbytes_remaining = nbytes_remaining - read_buffer_size;

            // read filename 0
            try
                {
                    infile1.read(reinterpret_cast<char *>(input_samples.data()), read_buffer_size);
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cerr << "Exception reading file " << filename0 << '\n';
                    break;
                }
            if (infile1)
                {
                    nread_elements = read_buffer_size;
                }
            else
                {
                    // FLAG AS ERROR !! IT SHOULD NEVER HAPPEN
                    nread_elements = infile1.gcount();
                }

            for (int index0 = 0; index0 < (nread_elements); index0 += 2)
                {
                    // dma_buff_offset_pos is 1 for the L1 band and 0 for the other bands
                    input_samples_dma[dma_index + dma_buff_offset_pos] = input_samples[index0];
                    input_samples_dma[dma_index + 1 + dma_buff_offset_pos] = input_samples[index0 + 1];
                    dma_index += 4;
                }

            // read filename 1 (if enabled)
            if (num_freq_bands_ > 1)
                {
                    dma_index = 0;
                    try
                        {
                            infile2.read(reinterpret_cast<char *>(input_samples.data()), read_buffer_size);
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            std::cerr << "Exception reading file " << filename1 << '\n';
                            break;
                        }
                    if (infile2)
                        {
                            nread_elements = read_buffer_size;
                        }
                    else
                        {
                            // FLAG AS ERROR !! IT SHOULD NEVER HAPPEN
                            nread_elements = infile2.gcount();
                        }

                    for (int index0 = 0; index0 < (nread_elements); index0 += 2)
                        {
                            // filename2 is never the L1 band
                            input_samples_dma[dma_index] = input_samples[index0];
                            input_samples_dma[dma_index + 1] = input_samples[index0 + 1];
                            dma_index += 4;
                        }
                }

            if (nread_elements > 0)
                {
                    num_transferred_bytes = nread_elements * 2;
                    const int num_bytes_sent = write(tx_fd, input_samples_dma.data(), nread_elements * 2);
                    if (num_bytes_sent != num_transferred_bytes)
                        {
                            std::cerr << "Error: DMA could not send all the required samples\n";
                            break;
                        }

                    // Throttle the DMA
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }

            if (nbytes_remaining == 0)
                {
                    if (repeat)
                        {
                            // read the file again
                            nbytes_remaining = samples * item_size;
                            read_buffer_size = sample_block_size * 2;
                            try
                                {
                                    infile1.seekg(0);
                                }
                            catch (const std::ifstream::failure &e)
                                {
                                    std::cerr << "Exception resetting the position of the next byte to be extracted to zero " << filename0 << '\n';
                                    break;
                                }

                            // skip the initial samples if needed
                            uint64_t bytes_to_skeep = samples_to_skip * item_size;
                            try
                                {
                                    infile1.ignore(bytes_to_skeep);
                                }
                            catch (const std::ifstream::failure &e)
                                {
                                    std::cerr << "Exception skipping initial samples file " << filename0 << '\n';
                                    break;
                                }

                            if (!filename1.empty())
                                {
                                    try
                                        {
                                            infile2.seekg(0);
                                        }
                                    catch (const std::ifstream::failure &e)
                                        {
                                            std::cerr << "Exception setting the position of the next byte to be extracted to zero " << filename1 << '\n';
                                            break;
                                        }

                                    try
                                        {
                                            infile2.ignore(bytes_to_skeep);
                                        }
                                    catch (const std::ifstream::failure &e)
                                        {
                                            std::cerr << "Exception skipping initial samples file " << filename1 << '\n';
                                            break;
                                        }
                                }
                        }
                    else
                        {
                            // the input file is completely processed. Stop the receiver.
                            run_DMA = false;
                        }
                }
            std::unique_lock<std::mutex> lock(dma_mutex);
            if (enable_DMA_ == false)
                {
                    run_DMA = false;
                }
            lock.unlock();
        }

    if (close(tx_fd) < 0)
        {
            std::cerr << "Error closing loop device " << '\n';
        }

    try
        {
            infile1.close();
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Exception closing file " << filename0 << '\n';
        }

    if (num_freq_bands_ > 1)
        {
            try
                {
                    infile2.close();
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cerr << "Exception closing file " << filename1 << '\n';
                }
        }

    // Stop the receiver
    queue->push(pmt::make_any(command_event_make(200, 0)));
}


void Ad9361FpgaSignalSource::run_dynamic_bit_selection_process()
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


void Ad9361FpgaSignalSource::run_buffer_monitor_process()
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


void Ad9361FpgaSignalSource::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "AD9361 FPGA source nothing to connect";
}


void Ad9361FpgaSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "AD9361 FPGA source nothing to disconnect";
}


gr::basic_block_sptr Ad9361FpgaSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return gr::basic_block_sptr();
}


gr::basic_block_sptr Ad9361FpgaSignalSource::get_right_block()
{
    return gr::basic_block_sptr();
}
