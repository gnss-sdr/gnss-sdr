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

#ifndef GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_H
#define GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_H

#include "rtl_tcp_signal_source_c.h"
#include "gnss_block_interface.h"
#include <boost/shared_ptr.hpp>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/deinterleave.h>
#include <gnuradio/blocks/float_to_complex.h>
#include <stdexcept>
#include <string>


class ConfigurationInterface;

/*!
 * \brief This class reads from rtl_tcp, which streams interleaved
 * I/Q samples over TCP.
 * (see http://sdr.osmocom.org/trac/wiki/rtl-sdr)
 */
class RtlTcpSignalSource : public GNSSBlockInterface
{
public:
    RtlTcpSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream,
        unsigned int out_stream, boost::shared_ptr<gr::msg_queue> queue);

    virtual ~RtlTcpSignalSource();

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "RtlTcp_Signal_Source"
     */
    inline std::string implementation() override
    {
        return "RtlTcp_Signal_Source";
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
    void MakeBlock();
    std::string role_;

    // rtl_tcp settings
    std::string address_;
    short port_;
    bool AGC_enabled_;
    double sample_rate_;
    bool flip_iq_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    double freq_;
    double gain_;
    double if_gain_;
    double rf_gain_;

    std::string item_type_;
    size_t item_size_;
    long samples_;
    bool dump_;
    std::string dump_filename_;

    rtl_tcp_signal_source_c_sptr signal_source_;

    boost::shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr file_sink_;
    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif /*GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_H */
