/*!
 * \file multichannel_file_signal_source.h
 * \brief Implementation of a class that reads signals samples from files at
 * different frequency band and adapts it to a SignalSourceInterface
 * \author Javier Arribas, 2019 jarribas(at)cttc.es
 *
 * This class represents a file signal source. Internally it uses a GNU Radio's
 * gr_file_source as a connector to the data.
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
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_MULTICHANNEL_FILE_SIGNAL_SOURCE_H_
#define GNSS_SDR_MULTICHANNEL_FILE_SIGNAL_SOURCE_H_

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/throttle.h>
#include <gnuradio/hier_block2.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <string>
#include <vector>

class ConfigurationInterface;

/*!
 * \brief Class that reads signals samples from files at different frequency bands
 * and adapts it to a SignalSourceInterface
 */
class MultichannelFileSignalSource : public GNSSBlockInterface
{
public:
    MultichannelFileSignalSource(ConfigurationInterface* configuration, const std::string& role,
        unsigned int in_streams, unsigned int out_streams,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    ~MultichannelFileSignalSource() = default;

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Multichannel_File_Signal_Source".
     */
    inline std::string implementation() override
    {
        return "Multichannel_File_Signal_Source";
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
    uint64_t samples_;
    int64_t sampling_frequency_;
    uint32_t n_channels_;
    std::vector<std::string> filename_vec_;
    std::string item_type_;
    bool repeat_;
    std::string role_;
    uint32_t in_streams_;
    uint32_t out_streams_;
    std::vector<gr::blocks::file_source::sptr> file_source_vec_;
    boost::shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr sink_;
    std::vector<gr::blocks::throttle::sptr> throttle_vec_;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue_;
    size_t item_size_;
    // Throttle control
    bool enable_throttle_control_;
};

#endif /* GNSS_SDR_MULTICHANNEL_FILE_SIGNAL_SOURCE_H_ */
