/*!
 * \file multichannel_file_signal_source.h
 * \brief Implementation of a class that reads signals samples from files at
 * different frequency band and adapts it to a SignalSourceInterface
 * \author Javier Arribas, 2019 jarribas(at)cttc.es
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

#ifndef GNSS_SDR_MULTICHANNEL_FILE_SIGNAL_SOURCE_H
#define GNSS_SDR_MULTICHANNEL_FILE_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include "signal_source_base.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/throttle.h>
#include <gnuradio/hier_block2.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <string>
#include <vector>


/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief Class that reads signals samples from files at different frequency bands
 * and adapts it to a SignalSourceInterface
 */
class MultichannelFileSignalSource : public SignalSourceBase
{
public:
    MultichannelFileSignalSource(const ConfigurationInterface* configuration, const std::string& role,
        unsigned int in_streams, unsigned int out_streams,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~MultichannelFileSignalSource() = default;

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
        return filename_vec_.at(0);
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
    std::vector<gr::blocks::file_source::sptr> file_source_vec_;
    gnss_shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr sink_;
    std::vector<gr::blocks::throttle::sptr> throttle_vec_;
    std::vector<std::string> filename_vec_;
    std::string item_type_;
    uint64_t samples_;
    int64_t sampling_frequency_;
    size_t item_size_;
    int32_t n_channels_;
    uint32_t in_streams_;
    uint32_t out_streams_;
    bool repeat_;
    // Throttle control
    bool enable_throttle_control_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_MULTICHANNEL_FILE_SIGNAL_SOURCE_H
