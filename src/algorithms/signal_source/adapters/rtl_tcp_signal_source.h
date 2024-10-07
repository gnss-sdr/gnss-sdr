/*!
 * \file rtl_tcp_signal_source.h
 * \brief Signal source which reads from rtl_tcp.
 * (see https://osmocom.org/projects/rtl-sdr/wiki for more information)
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
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

#ifndef GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_H
#define GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "rtl_tcp_signal_source_c.h"
#include "signal_source_base.h"
#include <gnuradio/blocks/deinterleave.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/float_to_complex.h>
#include <pmt/pmt.h>
#include <stdexcept>
#include <string>


/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class reads from rtl_tcp, which streams interleaved
 * I/Q samples over TCP.
 * (see https://osmocom.org/projects/rtl-sdr/wiki)
 */
class RtlTcpSignalSource : public SignalSourceBase
{
public:
    RtlTcpSignalSource(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_stream,
        unsigned int out_stream,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~RtlTcpSignalSource() = default;

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

    gnss_shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr file_sink_;

    std::string item_type_;
    std::string dump_filename_;

    // rtl_tcp settings
    std::string address_;
    size_t item_size_;
    uint64_t samples_;
    double rf_gain_;
    int sample_rate_;
    int freq_;
    int gain_;
    int if_gain_;
    unsigned int in_stream_;
    unsigned int out_stream_;
    int16_t port_;
    bool AGC_enabled_;
    bool flip_iq_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_H
