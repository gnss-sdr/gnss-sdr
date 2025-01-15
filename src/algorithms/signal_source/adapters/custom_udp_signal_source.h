/*!
 * \file custom_udp_signal_source.h
 * \brief Receives ip frames containing samples in UDP frame encapsulation
 * using a high performance packet capture library (libpcap)
 * \author Javier Arribas jarribas (at) cttc.es
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


#ifndef GNSS_SDR_CUSTOM_UDP_SIGNAL_SOURCE_H
#define GNSS_SDR_CUSTOM_UDP_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "gr_complex_ip_packet_source.h"
#include "signal_source_base.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/null_sink.h>
#include <pmt/pmt.h>
#include <stdexcept>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class reads from UDP packets, which streams interleaved
 * I/Q samples over a network.
 */
class CustomUDPSignalSource : public SignalSourceBase
{
public:
    CustomUDPSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~CustomUDPSignalSource() = default;

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
    Gr_Complex_Ip_Packet_Source::sptr udp_gnss_rx_source_;
    std::vector<gnss_shared_ptr<gr::block>> null_sinks_;
    std::vector<gnss_shared_ptr<gr::block>> file_sink_;

    std::string item_type_;
    std::string dump_filename_;

    size_t item_size_;

    int RF_channels_;
    int channels_in_udp_;
    unsigned int in_stream_;
    unsigned int out_stream_;
    bool IQ_swap_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CUSTOM_UDP_SIGNAL_SOURCE_H
