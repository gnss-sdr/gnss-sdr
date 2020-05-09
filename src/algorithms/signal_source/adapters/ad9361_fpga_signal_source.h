/*!
 * \file ad9361_fpga_signal_source.h
 * \brief signal source for Analog Devices front-end AD9361 connected directly to FPGA accelerators.
 * This source implements only the AD9361 control. It is NOT compatible with conventional SDR acquisition and tracking blocks.
 * Please use the fmcomms2 source if conventional SDR acquisition and tracking is selected in the configuration file.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_AD9361_FPGA_SIGNAL_SOURCE_H
#define GNSS_SDR_AD9361_FPGA_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "fpga_switch.h"
#include "gnss_block_interface.h"
#include <boost/shared_ptr.hpp>
#include <pmt/pmt.h>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

class ConfigurationInterface;

class Ad9361FpgaSignalSource : public GNSSBlockInterface
{
public:
    Ad9361FpgaSignalSource(ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    ~Ad9361FpgaSignalSource();

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
    std::string role_;

    // Front-end settings
    uint64_t freq_;  // frequency of local oscillator
    uint64_t sample_rate_;
    uint64_t bandwidth_;
    bool quadrature_;
    bool rf_dc_;
    bool bb_dc_;
    bool rx1_enable_;
    bool rx2_enable_;
    std::string gain_mode_rx1_;
    std::string gain_mode_rx2_;
    double rf_gain_rx1_;
    double rf_gain_rx2_;
    std::string rf_port_select_;
    std::string filter_file_;
    bool filter_auto_;
    std::string filter_source_;
    std::string filter_filename_;
    float Fpass_;
    float Fstop_;

    // DDS configuration for LO generation for external mixer
    bool enable_dds_lo_;
    uint64_t freq_rf_tx_hz_;
    uint64_t freq_dds_tx_hz_;
    uint64_t tx_bandwidth_;
    double scale_dds_dbfs_;
    double phase_dds_deg_;
    double tx_attenuation_db_;

    uint32_t in_stream_;
    uint32_t out_stream_;

    size_t item_size_;

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue_;

    std::shared_ptr<Fpga_Switch> switch_fpga;
    int32_t switch_position;

    std::thread thread_file_to_dma;
    std::string filename_rx1;
    std::string filename_rx2;
    std::string freq_band;

    bool enable_DMA_;
    bool rf_shutdown_;
};

#endif  // GNSS_SDR_AD9361_FPGA_SIGNAL_SOURCE_H
