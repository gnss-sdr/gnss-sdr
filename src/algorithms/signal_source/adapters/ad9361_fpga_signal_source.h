/*!
 * \file ad9361_fpga_signal_source.h
 * \brief signal source for Analog Devices front-end AD9361 connected directly to FPGA accelerators.
 * This source implements only the AD9361 control. It is NOT compatible with conventional SDR acquisition and tracking blocks.
 * Please use the fmcomms2 source if conventional SDR acquisition and tracking is selected in the configuration file.
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
#include "fpga_dynamic_bit_selection.h"
#include "fpga_switch.h"
#include "gnss_block_interface.h"
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

class Ad9361FpgaSignalSource : public GNSSBlockInterface
{
public:
    Ad9361FpgaSignalSource(const ConfigurationInterface *configuration,
        const std::string &role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t> *queue);

    ~Ad9361FpgaSignalSource();

    void start() override;

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Ad9361_Fpga_Signal_Source"
     */
    inline std::string implementation() override
    {
        return "Ad9361_Fpga_Signal_Source";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    const std::string switch_device_name = "AXIS_Switch_v1_0_0";          // Switch UIO device name
    const std::string dyn_bit_sel_device_name = "dynamic_bits_selector";  // Switch UIO device name

    // perform dynamic bit selection every 500 ms by default
    static const uint32_t Gain_control_period_ms = 500;

    void run_DMA_process(const std::string &FreqBand,
        const std::string &Filename1,
        const std::string &Filename2);

    void run_dynamic_bit_selection_process();

    std::thread thread_file_to_dma;
    std::thread thread_dynamic_bit_selection;

    std::shared_ptr<Fpga_Switch> switch_fpga;
    std::shared_ptr<Fpga_dynamic_bit_selection> dynamic_bit_selection_fpga;

    std::string role_;

    // Front-end settings
    std::string gain_mode_rx1_;
    std::string gain_mode_rx2_;
    std::string rf_port_select_;
    std::string filter_file_;
    std::string filter_source_;
    std::string filter_filename_;
    std::string filename_rx1;
    std::string filename_rx2;
    std::string freq_band;

    std::mutex dma_mutex;
    std::mutex dynamic_bit_selection_mutex;

    double rf_gain_rx1_;
    double rf_gain_rx2_;
    uint64_t freq_;  // frequency of local oscillator
    uint64_t sample_rate_;
    uint64_t bandwidth_;
    float Fpass_;
    float Fstop_;

    // DDS configuration for LO generation for external mixer
    double scale_dds_dbfs_;
    double phase_dds_deg_;
    double tx_attenuation_db_;
    uint64_t freq_rf_tx_hz_;
    uint64_t freq_dds_tx_hz_;
    uint64_t tx_bandwidth_;
    size_t item_size_;
    uint32_t in_stream_;
    uint32_t out_stream_;
    int32_t switch_position;
    bool enable_dds_lo_;

    bool filter_auto_;
    bool quadrature_;
    bool rf_dc_;
    bool bb_dc_;
    bool rx1_enable_;
    bool rx2_enable_;
    bool enable_DMA_;
    bool enable_dynamic_bit_selection_;
    bool rf_shutdown_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_AD9361_FPGA_SIGNAL_SOURCE_H
