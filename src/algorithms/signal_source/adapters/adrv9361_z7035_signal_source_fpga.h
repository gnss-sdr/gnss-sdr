/*!
 * \file adrv9361_z7035_signal_source_fpga.h
 * \brief signal source for the Analog Devices ADRV9361-Z7035 evaluation board
 * directly connected to the FPGA accelerators.
 * This source implements only the AD9361 control. It is NOT compatible with
 * conventional SDR acquisition and tracking blocks.
 * Please use the fmcomms2 source if conventional SDR acquisition and tracking
 * is selected in the configuration file.
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

#ifndef GNSS_SDR_ADRV9361_Z7035_SIGNAL_SOURCE_FPGA_H
#define GNSS_SDR_ADRV9361_Z7035_SIGNAL_SOURCE_FPGA_H

#include "concurrent_queue.h"
#include "fpga_buffer_monitor.h"
#include "fpga_dma-proxy.h"
#include "fpga_dynamic_bit_selection.h"
#include "fpga_switch.h"
#include "gnss_block_interface.h"
#include "signal_source_base.h"
#include <pmt/pmt.h>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>


/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

class Adrv9361z7035SignalSourceFPGA : public SignalSourceBase
{
public:
    Adrv9361z7035SignalSourceFPGA(const ConfigurationInterface *configuration,
        const std::string &role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t> *queue);

    ~Adrv9361z7035SignalSourceFPGA();

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    const std::string default_dump_filename = std::string("FPGA_buffer_monitor_dump.dat");
    const std::string default_rf_port_select = std::string("A_BALANCED");
    const std::string default_gain_mode = std::string("slow_attack");
    const double default_tx_attenuation_db = -10.0;
    const double default_manual_gain_rx1 = 64.0;
    const double default_manual_gain_rx2 = 64.0;
    const uint64_t default_bandwidth = 12500000;

    // perform dynamic bit selection every 500 ms by default
    const uint32_t Gain_control_period_ms = 500;
    // check buffer overflow and perform buffer monitoring every 1s by default
    const uint32_t buffer_monitor_period_ms = 1000;
    // buffer overflow and buffer monitoring initial delay
    const uint32_t buffer_monitoring_initial_delay_ms = 2000;
    // sample block size when running in post-processing mode
    const int sample_block_size = 16384;
    const int32_t switch_to_real_time_mode = 2;

    void run_dynamic_bit_selection_process();
    void run_buffer_monitor_process();

    std::thread thread_dynamic_bit_selection;
    std::thread thread_buffer_monitor;

    std::shared_ptr<Fpga_Switch> switch_fpga;
    std::shared_ptr<Fpga_dynamic_bit_selection> dynamic_bit_selection_fpga;
    std::shared_ptr<Fpga_buffer_monitor> buffer_monitor_fpga;

    std::mutex dynamic_bit_selection_mutex;
    std::mutex buffer_monitor_mutex;

    std::string gain_mode_rx1_;
    std::string gain_mode_rx2_;
    std::string rf_port_select_;
    std::string filter_file_;
    std::string filter_source_;
    std::string filter_filename_;

    double rf_gain_rx1_;
    double rf_gain_rx2_;
    double scale_dds_dbfs_;
    double phase_dds_deg_;
    double tx_attenuation_db_;

    uint64_t freq0_;  // frequency of local oscillator for ADRV9361-A 0
    uint64_t freq1_;  // frequency of local oscillator for ADRV9361-B (if present)
    uint64_t sample_rate_;
    uint64_t bandwidth_;
    uint64_t freq_dds_tx_hz_;
    uint64_t freq_rf_tx_hz_;
    uint64_t tx_bandwidth_;

    float Fpass_;
    float Fstop_;
    uint32_t in_stream_;
    uint32_t out_stream_;

    size_t item_size_;

    bool enable_dds_lo_;
    bool filter_auto_;
    bool quadrature_;
    bool rf_dc_;
    bool bb_dc_;
    bool rx1_enable_;
    bool rx2_enable_;
    bool enable_dynamic_bit_selection_;
    bool enable_ovf_check_buffer_monitor_active_;
    bool dump_;
    bool rf_shutdown_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_ADRV9361_Z7035_SIGNAL_SOURCE_FPGA_H
