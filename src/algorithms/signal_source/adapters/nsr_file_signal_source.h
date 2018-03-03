/*!
 * \file nsr_file_signal_source.h
 * \brief Implementation of a class that reads signals samples from a NSR 2 bits sampler front-end file
 * and adapts it to a SignalSourceInterface. More information about the front-end here
 * http://www.ifen.com/products/sx-scientific-gnss-solutions/nsr-software-receiver.html
 * \author Javier Arribas, 2013 jarribas(at)cttc.es
 *
 * This class represents a file signal source.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_NSR_FILE_SIGNAL_SOURCE_H_
#define GNSS_SDR_NSR_FILE_SIGNAL_SOURCE_H_

#include "gnss_block_interface.h"
#include "unpack_byte_2bit_samples.h"
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/throttle.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/msg_queue.h>
#include <string>

class ConfigurationInterface;

/*!
 * \brief Class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 */
class NsrFileSignalSource : public GNSSBlockInterface
{
public:
    NsrFileSignalSource(ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams,
        boost::shared_ptr<gr::msg_queue> queue);

    virtual ~NsrFileSignalSource();
    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Nsr_File_Signal_Source".
     */
    inline std::string implementation() override
    {
        return "Nsr_File_Signal_Source";
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

    inline long sampling_frequency() const
    {
        return sampling_frequency_;
    }

    inline long samples() const
    {
        return samples_;
    }

private:
    unsigned long long samples_;
    long sampling_frequency_;
    std::string filename_;
    std::string item_type_;
    bool repeat_;
    bool dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    gr::blocks::file_source::sptr file_source_;
    unpack_byte_2bit_samples_sptr unpack_byte_;
    boost::shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr sink_;
    gr::blocks::throttle::sptr throttle_;
    boost::shared_ptr<gr::msg_queue> queue_;
    size_t item_size_;
    // Throttle control
    bool enable_throttle_control_;
};

#endif /*GNSS_SDR_NSR_FILE_SIGNAL_SOURCE_H_*/
