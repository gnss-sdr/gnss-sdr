/*!
 * \file file_signal_source.h
 * \brief Interface of a class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * This class represents a file signal source. Internally it uses a GNU Radio's
 * gr_file_source as a connector to the data.
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

#ifndef GNSS_SDR_FILE_SIGNAL_SOURCE_H
#define GNSS_SDR_FILE_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/throttle.h>
#include <gnuradio/hier_block2.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <string>

/** \addtogroup Signal_Source Signal Source
 * Classes for Signal Source management.
 * \{ */
/** \addtogroup Signal_Source_adapters signal_source_adapters
 * Classes that wrap GNU Radio signal sources with a GNSSBlockInterface
 * \{ */


class ConfigurationInterface;

/*!
 * \brief Class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 */
class FileSignalSource : public GNSSBlockInterface
{
public:
    FileSignalSource(const ConfigurationInterface* configuration, const std::string& role,
        unsigned int in_streams, unsigned int out_streams,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~FileSignalSource() = default;

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "File_Signal_Source".
     */
    inline std::string implementation() override
    {
        return "File_Signal_Source";
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
    gr::blocks::file_source::sptr file_source_;
    gnss_shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr sink_;
    gr::blocks::throttle::sptr throttle_;

    std::string role_;
    std::string item_type_;
    std::string filename_;
    std::string dump_filename_;

    uint64_t samples_;
    int64_t sampling_frequency_;
    size_t item_size_;

    uint32_t in_streams_;
    uint32_t out_streams_;

    bool enable_throttle_control_;
    bool repeat_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FILE_SIGNAL_SOURCE_H
