/*!
 * \file ad9361_fpga_signal_source.h
 * \brief signal source for Analog Devices front-end AD9361 connected directly
 * to FPGA accelerators.
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
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_AD9361_FPGA_SIGNAL_SOURCE_H
#define GNSS_SDR_AD9361_FPGA_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "fpga_buffer_monitor.h"
#if INTPTR_MAX == INT64_MAX  // 64-bit processor architecture
#include "fpga_dma-proxy.h"
#else
#include "fpga_ezdma.h"
#endif
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

class Ad9361FpgaSignalSource : public SignalSourceBase
{
public:
    Ad9361FpgaSignalSource(const ConfigurationInterface *configuration,
        const std::string &role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t> *queue);

    ~Ad9361FpgaSignalSource();

    void start() override;

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    const std::string switch_device_name = std::string("AXIS_Switch_v1_0_0");          // Switch UIO device name
    const std::string dyn_bit_sel_device_name = std::string("dynamic_bits_selector");  // Switch dhnamic bit selector device name
    const std::string buffer_monitor_device_name = std::string("buffer_monitor");      // buffer monitor device name
    const std::string default_dump_filename = std::string("FPGA_buffer_monitor_dump.dat");
    const std::string default_rf_port_select = std::string("A_BALANCED");
    const std::string default_gain_mode = std::string("slow_attack");
    const std::string empty_string;
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

    void run_DMA_process(const std::string &filename0,
        const std::string &filename1,
        uint64_t &samples_to_skip,
        size_t &item_size,
        int64_t &samples,
        bool &repeat,
        uint32_t &dma_buff_offset_pos,
        Concurrent_Queue<pmt::pmt_t> *queue);

    void run_dynamic_bit_selection_process();
    void run_buffer_monitor_process();

    std::thread thread_file_to_dma;
    std::thread thread_dynamic_bit_selection;
    std::thread thread_buffer_monitor;

    std::shared_ptr<Fpga_Switch> switch_fpga;
    std::shared_ptr<Fpga_dynamic_bit_selection> dynamic_bit_selection_fpga;
    std::shared_ptr<Fpga_buffer_monitor> buffer_monitor_fpga;
    std::shared_ptr<Fpga_DMA> dma_fpga;

    std::mutex dma_mutex;
    std::mutex dynamic_bit_selection_mutex;
    std::mutex buffer_monitor_mutex;

    Concurrent_Queue<pmt::pmt_t> *queue_;

    // Front-end settings
    std::string gain_mode_rx1_;
    std::string gain_mode_rx2_;
    std::string rf_port_select_;
    std::string filter_file_;
    std::string filter_source_;
    std::string filter_filename_;
    std::string filename0_;
    std::string filename1_;

    double rf_gain_rx1_;
    double rf_gain_rx2_;
    uint64_t freq0_;  // frequency of local oscillator for ADRV9361-A 0
    uint64_t freq1_;  // frequency of local oscillator for ADRV9361-B (if present)
    uint64_t sample_rate_;
    uint64_t bandwidth_;
    uint64_t samples_to_skip_;
    int64_t samples_;
    float Fpass_;
    float Fstop_;
    uint32_t num_freq_bands_;
    uint32_t dma_buff_offset_pos_;

    // DDS configuration for LO generation for external mixer
    double scale_dds_dbfs_;
    double phase_dds_deg_;
    double tx_attenuation_db_;
    uint64_t freq_dds_tx_hz_;
    uint64_t freq_rf_tx_hz_;
    uint64_t tx_bandwidth_;
    size_t item_size_;
    uint32_t in_stream_;
    uint32_t out_stream_;
    int32_t switch_position_;
    bool enable_dds_lo_;

    bool filter_auto_;
    bool quadrature_;
    bool rf_dc_;
    bool bb_dc_;
    bool rx1_enable_;
    bool rx2_enable_;
    bool enable_DMA_;
    bool enable_dynamic_bit_selection_;
    bool enable_ovf_check_buffer_monitor_active_;
    bool dump_;
    bool rf_shutdown_;
    bool repeat_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_AD9361_FPGA_SIGNAL_SOURCE_H
