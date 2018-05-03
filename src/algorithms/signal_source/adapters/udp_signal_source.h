/*!
 * \file rtl_tcp_signal_source.h
 * \brief Signal source which reads from rtl_tcp.
 * (see http://sdr.osmocom.org/trac/wiki/rtl-sdr for more information)
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
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

#ifndef GNSS_SDR_UDP_SIGNAL_SOURCE_H
#define GNSS_SDR_UDP_SIGNAL_SOURCE_H

#include "gnss_block_interface.h"
#include "udp_gnss_rx_source.h"
#include <boost/shared_ptr.hpp>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/char_to_float.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/deinterleave.h>
#include <gnuradio/blocks/float_to_complex.h>
#include <stdexcept>
#include <string>
#include <vector>


class ConfigurationInterface;

/*!
 * \brief This class reads from UDP packets, which streams interleaved
 * I/Q samples over a network.
 */
class UDPSignalSource : public GNSSBlockInterface
{
public:
    UDPSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream,
        unsigned int out_stream, boost::shared_ptr<gr::msg_queue> queue);

    virtual ~UDPSignalSource();

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "UDP_Signal_Source"
     */
    inline std::string implementation() override
    {
        return "UDP_Signal_Source";
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
    int select_single_channel_;
    int channels_in_udp_;
    unsigned int in_stream_;
    unsigned int out_stream_;

    std::string item_type_;
    size_t item_size_;
    bool dump_;
    std::string dump_filename_;
    std::vector<boost::shared_ptr<gr::block>> char_to_float_;
    std::vector<boost::shared_ptr<gr::block>> float_to_complex_;
    std::vector<boost::shared_ptr<gr::block>> null_sinks_;

    udp_gnss_rx_source_sptr udp_gnss_rx_source_;
    gr::blocks::deinterleave::sptr demux_;
    std::vector<boost::shared_ptr<gr::block>> file_sink_;
    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif /*GNSS_SDR_UDP_SIGNAL_SOURCE_H */
