/*!
 * \file two_bit_packed_file_signal_source.h
 * \brief Interface of a class that reads signals samples from a file. Each
 * sample is two bits, which are packed into bytes or shorts.
 *
 * \author Cillian O'Driscoll, 2015 cillian.odriscoll (at) gmail.com
 *
 * This class represents a file signal source.
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

#ifndef GNSS_SDR_TWO_BIT_PACKED_FILE_SIGNAL_SOURCE_H
#define GNSS_SDR_TWO_BIT_PACKED_FILE_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include "unpack_2bit_samples.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/throttle.h>
#include <gnuradio/hier_block2.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <memory>
#include <string>
#if GNURADIO_USES_STD_POINTERS
#else
#include <boost/shared_ptr.hpp>
#endif


class ConfigurationInterface;

/*!
 * \brief Class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 */
class TwoBitPackedFileSignalSource : public GNSSBlockInterface
{
public:
    TwoBitPackedFileSignalSource(ConfigurationInterface* configuration, const std::string& role,
        unsigned int in_streams, unsigned int out_streams,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~TwoBitPackedFileSignalSource() = default;
    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Two_Bit_Packed_File_Signal_Source".
     */
    inline std::string implementation() override
    {
        return "Two_Bit_Packed_File_Signal_Source";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

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

    inline bool big_endian_items() const
    {
        return big_endian_items_;
    }

    inline bool big_endian_bytes() const
    {
        return big_endian_bytes_;
    }

    inline bool is_complex() const
    {
        return is_complex_;
    }

    inline bool reverse_interleaving() const
    {
        return reverse_interleaving_;
    }

private:
    uint64_t samples_;
    int64_t sampling_frequency_;
    std::string filename_;
    std::string item_type_;
    bool repeat_;
    bool dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    gr::blocks::file_source::sptr file_source_;
    unpack_2bit_samples_sptr unpack_samples_;
    gr::basic_block_sptr char_to_float_;
#if GNURADIO_USES_STD_POINTERS
    std::shared_ptr<gr::block> valve_;
#else
    boost::shared_ptr<gr::block> valve_;
#endif
    gr::blocks::file_sink::sptr sink_;
    gr::blocks::throttle::sptr throttle_;
    size_t item_size_;
    bool big_endian_items_;
    bool big_endian_bytes_;
    bool is_complex_;
    bool reverse_interleaving_;
    std::string sample_type_;
    // Throttle control
    bool enable_throttle_control_;
};

#endif  // GNSS_SDR_TWO_BIT_CPX_FILE_SIGNAL_SOURCE_H
