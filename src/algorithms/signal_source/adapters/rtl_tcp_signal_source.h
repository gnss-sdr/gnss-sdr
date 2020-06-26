/*!
 * \file rtl_tcp_signal_source.h
 * \brief Signal source which reads from rtl_tcp.
 * (see https://osmocom.org/projects/rtl-sdr/wiki for more information)
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
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

#ifndef GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_H
#define GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include "rtl_tcp_signal_source_c.h"
#include <gnuradio/blocks/deinterleave.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/float_to_complex.h>
#include <pmt/pmt.h>
#include <memory>
#include <stdexcept>
#include <string>
#if GNURADIO_USES_STD_POINTERS
#else
#include <boost/shared_ptr.hpp>
#endif


class ConfigurationInterface;

/*!
 * \brief This class reads from rtl_tcp, which streams interleaved
 * I/Q samples over TCP.
 * (see https://osmocom.org/projects/rtl-sdr/wiki)
 */
class RtlTcpSignalSource : public GNSSBlockInterface
{
public:
    RtlTcpSignalSource(ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_stream,
        unsigned int out_stream,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~RtlTcpSignalSource() = default;

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

    rtl_tcp_signal_source_c_sptr signal_source_;

#if GNURADIO_USES_STD_POINTERS
    std::shared_ptr<gr::block> valve_;
#else
    boost::shared_ptr<gr::block> valve_;
#endif
    gr::blocks::file_sink::sptr file_sink_;

    std::string role_;
    std::string item_type_;
    std::string dump_filename_;

    // rtl_tcp settings
    std::string address_;
    size_t item_size_;
    uint64_t samples_;
    double sample_rate_;
    double freq_;
    double gain_;
    double if_gain_;
    double rf_gain_;
    unsigned int in_stream_;
    unsigned int out_stream_;
    int16_t port_;
    bool AGC_enabled_;
    bool flip_iq_;
    bool dump_;
};

#endif  // GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_H
