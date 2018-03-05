/*!
 * \file raw_array_signal_source.h
 * \brief CTTC Experimental GNSS 8 channels array signal source
 * \author Javier Arribas, jarribas(at)cttc.es
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


#ifndef RAW_ARRAY_SIGNAL_SOURCE_H_
#define RAW_ARRAY_SIGNAL_SOURCE_H_

#include "gnss_block_interface.h"
#include <gnuradio/hier_block2.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/file_sink.h>
#include <string>

class ConfigurationInterface;

/*!
 * \brief This class reads samples from a GN3S USB dongle, a RF front-end signal sampler
 */
class RawArraySignalSource : public GNSSBlockInterface
{
public:
    RawArraySignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream,
        unsigned int out_stream, gr::msg_queue::sptr queue);

    virtual ~RawArraySignalSource();

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "RawArraySignalSource".
     */
    inline std::string implementation() override
    {
        return "Raw_Array_Signal_Source";
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
    unsigned int in_stream_;
    unsigned int out_stream_;
    std::string item_type_;
    size_t item_size_;
    long samples_;
    bool dump_;
    std::string dump_filename_;
    std::string eth_device_;
    gr::block_sptr raw_array_source_;
    gr::blocks::file_sink::sptr file_sink_;
    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif /*RAW_ARRAY_SIGNAL_SOURCE_H_*/
