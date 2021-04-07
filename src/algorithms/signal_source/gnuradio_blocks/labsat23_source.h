/*!
 * \file labsat23_source.h
 *
 * \brief Unpacks capture files in the LabSat 2 (ls2), LabSat 3 (ls3), or LabSat
 * 3 Wideband (LS3W) formats.
 * \author Javier Arribas jarribas (at) cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_LABSAT23_SOURCE_H
#define GNSS_SDR_LABSAT23_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class labsat23_source;

using labsat23_source_sptr = gnss_shared_ptr<labsat23_source>;

labsat23_source_sptr labsat23_make_source_sptr(
    const char *signal_file_basename,
    const std::vector<int> &channel_selector,
    Concurrent_Queue<pmt::pmt_t> *queue,
    bool digital_io_enabled);

/*!
 * \brief This class implements conversion between Labsat 2, 3 and 3 Wideband
 * formats to gr_complex
 */
class labsat23_source : public gr::block
{
public:
    ~labsat23_source();

    int general_work(int noutput_items,
        gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend labsat23_source_sptr labsat23_make_source_sptr(
        const char *signal_file_basename,
        const std::vector<int> &channel_selector,
        Concurrent_Queue<pmt::pmt_t> *queue,
        bool digital_io_enabled);

    labsat23_source(const char *signal_file_basename,
        const std::vector<int> &channel_selector,
        Concurrent_Queue<pmt::pmt_t> *queue,
        bool digital_io_enabled);

    std::string generate_filename();

    int parse_header();
    int getBit(uint8_t byte, int position);
    int read_ls3w_ini(const std::string &filename);
    int number_of_samples_per_ls3w_register() const;

    void decode_samples_one_channel(int16_t input_short, gr_complex *out, int type);
    void decode_ls3w_register(uint64_t input, std::vector<gr_complex *> &out, std::size_t output_pointer) const;

    std::ifstream binary_input_file;
    std::string d_signal_file_basename;
    Concurrent_Queue<pmt::pmt_t> *d_queue;
    std::vector<int> d_channel_selector_config;
    int d_current_file_number;
    uint8_t d_labsat_version;
    uint8_t d_channel_selector;
    uint8_t d_ref_clock;
    uint8_t d_bits_per_sample;
    bool d_header_parsed;

    // Data members for Labsat 3 Wideband
    std::string d_ls3w_OSC;
    std::vector<int> d_ls3w_selected_channel_offset;
    int64_t d_ls3w_SMP{};
    int32_t d_ls3w_QUA{};
    int32_t d_ls3w_CHN{};
    int32_t d_ls3w_SFT{};
    int32_t d_ls3w_CFA{};
    int32_t d_ls3w_CFB{};
    int32_t d_ls3w_CFC{};
    int32_t d_ls3w_BWA{};
    int32_t d_ls3w_BWB{};
    int32_t d_ls3w_BWC{};
    int d_ls3w_spare_bits{};
    int d_ls3w_samples_per_register{};
    bool d_is_ls3w = false;
    bool d_ls3w_digital_io_enabled = false;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_LABSAT23_SOURCE_H
