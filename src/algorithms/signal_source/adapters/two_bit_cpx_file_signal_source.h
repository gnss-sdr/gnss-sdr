/*!
 * \file two_bit_cpx_file_signal_source.h
 * \brief Interface of a class that reads signals samples from a 2 bit complex sampler front-end file
 * and adapts it to a SignalSourceInterface.
 * \author Javier Arribas, 2015 jarribas(at)cttc.es
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

#ifndef GNSS_SDR_TWO_BIT_CPX_FILE_SIGNAL_SOURCE_H
#define GNSS_SDR_TWO_BIT_CPX_FILE_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include "unpack_byte_2bit_cpx_samples.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/interleaved_short_to_complex.h>
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
class TwoBitCpxFileSignalSource : public GNSSBlockInterface
{
public:
    TwoBitCpxFileSignalSource(ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams,
        const std::shared_ptr<Concurrent_Queue<pmt::pmt_t>>& queue);

    ~TwoBitCpxFileSignalSource() = default;
    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Two_Bit_Cpx_File_Signal_Source".
     */
    inline std::string implementation() override
    {
        return "Two_Bit_Cpx_File_Signal_Source";
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
    unpack_byte_2bit_cpx_samples_sptr unpack_byte_;
    gr::blocks::interleaved_short_to_complex::sptr inter_shorts_to_cpx_;
#if GNURADIO_USES_STD_POINTERS
    std::shared_ptr<gr::block> valve_;
#else
    boost::shared_ptr<gr::block> valve_;
#endif
    gr::blocks::file_sink::sptr sink_;
    gr::blocks::throttle::sptr throttle_;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue_;
    size_t item_size_;
    // Throttle control
    bool enable_throttle_control_;
};

#endif  // GNSS_SDR_TWO_BIT_CPX_FILE_SIGNAL_SOURCE_H
