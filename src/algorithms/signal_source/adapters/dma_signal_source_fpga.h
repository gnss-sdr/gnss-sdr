/*!
 * \file dma_signal_source_fpga.h
 * \brief signal source for a DMA connected directly to FPGA accelerators.
 * This source implements only the DMA control. It is NOT compatible with
 * conventional SDR acquisition and tracking blocks.
 * \author Marc Majoral, mmajoral(at)cttc.es
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

#ifndef GNSS_SDR_DMA_SIGNAL_SOURCE_FPGA_H
#define GNSS_SDR_DMA_SIGNAL_SOURCE_FPGA_H

#include "concurrent_queue.h"
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

class DMASignalSourceFPGA : public SignalSourceBase
{
public:
    DMASignalSourceFPGA(const ConfigurationInterface *configuration,
        const std::string &role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t> *queue);

    ~DMASignalSourceFPGA();

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
    const std::string dyn_bit_sel_device_name = std::string("dynamic_bits_selector");  // Switch dhnamic bit selector device name
    const std::string empty_string;
    const uint64_t default_bandwidth = 12500000;
    // perform dynamic bit selection every 500 ms by default
    const uint32_t Gain_control_period_ms = 500;
    // sample block size when running in post-processing mode
    const int sample_block_size = 16384;
    const int32_t switch_to_DMA = 0;

    void run_DMA_process(const std::string &filename0,
        const std::string &filename1,
        uint64_t &samples_to_skip,
        size_t &item_size,
        int64_t &samples,
        bool &repeat,
        uint32_t &dma_buff_offset_pos,
        Concurrent_Queue<pmt::pmt_t> *queue);

    void run_dynamic_bit_selection_process();

    std::thread thread_file_to_dma;
    std::thread thread_dynamic_bit_selection;

    std::shared_ptr<Fpga_Switch> switch_fpga;
    std::shared_ptr<Fpga_dynamic_bit_selection> dynamic_bit_selection_fpga;
    std::shared_ptr<Fpga_DMA> dma_fpga;

    std::mutex dma_mutex;
    std::mutex dynamic_bit_selection_mutex;

    Concurrent_Queue<pmt::pmt_t> *queue_;

    std::string filename0_;
    std::string filename1_;

    uint64_t sample_rate_;
    uint64_t samples_to_skip_;
    int64_t samples_;
    uint32_t num_input_files_;
    uint32_t dma_buff_offset_pos_;
    uint32_t in_stream_;
    uint32_t out_stream_;
    size_t item_size_;

    bool enable_DMA_;
    bool enable_dynamic_bit_selection_;
    bool repeat_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_DMA_SIGNAL_SOURCE_FPGA_H
