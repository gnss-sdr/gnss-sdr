/*!
 * \file spir_gss6450_file_signal_source.h
 * \brief Implementation of a class that reads signals samples from a SPIR file
 * and adapts it to a SignalSourceInterface.
 * \author Antonio Ramos, 2017 antonio.ramos(at)cttc.es
 * \author Carles Fernandez, 2026 carles.fernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_SPIR_GSS6450_FILE_SIGNAL_SOURCE_H
#define GNSS_SDR_SPIR_GSS6450_FILE_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_sdr_valve.h"
#include "signal_source_base.h"
#include "unpack_spir_gss6450_samples.h"
#include <gnuradio/blocks/endian_swap.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/stream_to_streams.h>
#include <gnuradio/blocks/throttle.h>
#include <gnuradio/hier_block2.h>
#include <pmt/pmt.h>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>


/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief Class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 */
class SpirGSS6450FileSignalSource : public SignalSourceBase
{
public:
    SpirGSS6450FileSignalSource(const ConfigurationInterface* configuration, const std::string& role,
        uint32_t in_streams, uint32_t out_streams, Concurrent_Queue<pmt::pmt_t>* queue);

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block(int RF_channel) override;
    gr::basic_block_sptr get_right_block() override;
    size_t getRfChannels() const override;

    inline std::string filename() const
    {
        return filename_;
    }

    inline std::string item_type() const
    {
        return item_type_;
    }

    inline bool repeat() const
    {
        return repeat_;
    }

    inline int64_t sampling_frequency() const
    {
        return sampling_frequency_;
    }

    inline uint64_t samples() const
    {
        return samples_;
    }

private:
    struct Gss6450FileLayout
    {
        int64_t data_offset = -1;
        int32_t channels = 0;
        uint32_t adc_bits = 0;
        bool gss6425_compatible = false;
    };

    static uint32_t read_le_word(const std::vector<uint8_t>& bytes, size_t offset);
    static bool matches_sync_channel(uint32_t word, uint8_t channel_pattern);
    static uint32_t bits_from_header(const std::string& header, const std::string& filename);
    static bool is_gss6425_compatibility_header(const std::string& header);
    static Gss6450FileLayout detect_gss6450_layout(const std::string& filename);
    static bool is_valid_gss6450_bits(uint32_t bits);
    static bool is_ascii_digit(char value);
    static bool is_ascii_space(char value);
    static char to_lower_ascii(char value);
    static size_t find_case_insensitive(const std::string& text, const std::string& pattern, size_t start_pos = 0);

    gr::blocks::file_source::sptr file_source_;
    gr::blocks::stream_to_streams::sptr stream_to_streams_;
    std::vector<gnss_shared_ptr<gr::block>> valve_vec_;
    std::vector<gr::blocks::endian_swap::sptr> endian_vec_;
    std::vector<gr::blocks::null_sink::sptr> null_sinks_;
    std::vector<unpack_spir_gss6450_samples_sptr> unpack_spir_vec_;
    std::vector<gr::blocks::file_sink::sptr> sink_vec_;
    std::vector<gr::blocks::throttle::sptr> throttle_vec_;
    std::string filename_;
    std::string dump_filename_;
    std::string item_type_;
    uint64_t samples_;
    int64_t bytes_to_skip_;
    int64_t sampling_frequency_;
    size_t item_size_;
    uint32_t in_streams_;
    uint32_t out_streams_;
    uint32_t adc_bits_;
    int32_t n_channels_;
    int32_t sel_ch_;
    size_t rf_channels_;
    bool gss6425_compatible_;
    bool repeat_;
    bool dump_;  // Enables dumping the gr_complex sample output
    bool enable_throttle_control_;
    bool endian_swap_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SPIR_GSS6450_FILE_SIGNAL_SOURCE_H
