/*!
 * \file udp_signal_source.h
 *
 * \brief Receives ip frames containing samples in UDP frame encapsulation
 * using a high performance packet capture library (libpcap)
 * \author Javier Arribas jarribas (at) cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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


#ifndef GNSS_SDR_CUSTOM_UDP_SIGNAL_SOURCE_H
#define GNSS_SDR_CUSTOM_UDP_SIGNAL_SOURCE_H

#include "gnss_block_interface.h"
#include "gr_complex_ip_packet_source.h"
#include <boost/shared_ptr.hpp>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/null_sink.h>
#include <stdexcept>
#include <string>
#include <vector>


class ConfigurationInterface;

/*!
 * \brief This class reads from UDP packets, which streams interleaved
 * I/Q samples over a network.
 */
class CustomUDPSignalSource : public GNSSBlockInterface
{
public:
    CustomUDPSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream,
        unsigned int out_stream, boost::shared_ptr<gr::msg_queue> queue);

    virtual ~CustomUDPSignalSource();

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "UDP_Signal_Source"
     */
    inline std::string implementation() override
    {
        return "Custom_UDP_Signal_Source";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;
    gr::basic_block_sptr get_right_block(int RF_channel) override;

private:
    std::string role_;

    bool IQ_swap_;
    int RF_channels_;
    int channels_in_udp_;
    unsigned int in_stream_;
    unsigned int out_stream_;

    std::string item_type_;
    size_t item_size_;
    bool dump_;
    std::string dump_filename_;
    std::vector<boost::shared_ptr<gr::block>> null_sinks_;
    gr_complex_ip_packet_source::sptr udp_gnss_rx_source_;
    std::vector<boost::shared_ptr<gr::block>> file_sink_;
    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif /*GNSS_SDR_CUSTOM_UDP_SIGNAL_SOURCE_H */
