/*!
 * \file rtl_tcp_signal_source_c.h
 * \brief Interface of an rtl_tcp signal source reader.
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
 *
 * The implementation of this block is a combination of various helpful
 * sources. The data format and command structure is taken from the
 * original Osmocom rtl_tcp_source_f (https://git.osmocom.org/gr-osmosdr).
 * The aynchronous reading code comes from the examples provides
 * by Boost.Asio and the bounded buffer producer-consumer solution is
 * taken from the Boost.CircularBuffer examples (https://www.boost.org/).
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

#ifndef GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_C_H
#define GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_C_H

#include "gnss_block_interface.h"
#include "rtl_tcp_dongle_info.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <gnuradio/sync_block.h>
#include <cstdint>
#include <string>
#include <vector>


/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class rtl_tcp_signal_source_c;

using rtl_tcp_signal_source_c_sptr = gnss_shared_ptr<rtl_tcp_signal_source_c>;

#if USE_BOOST_ASIO_IO_CONTEXT
using b_io_context = boost::asio::io_context;
#else
using b_io_context = boost::asio::io_service;
#endif

rtl_tcp_signal_source_c_sptr
rtl_tcp_make_signal_source_c(const std::string &address,
    int16_t port,
    bool flip_iq = false);

/*!
 * \brief This class reads interleaved I/Q samples
 * from an rtl_tcp server and outputs complex types.
 */
class rtl_tcp_signal_source_c : public gr::sync_block
{
public:
    ~rtl_tcp_signal_source_c();

    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

    void set_frequency(int frequency);
    void set_sample_rate(int sample_rate);
    void set_agc_mode(bool agc);
    void set_gain(int gain);
    void set_if_gain(int gain);

private:
    friend rtl_tcp_signal_source_c_sptr
    rtl_tcp_make_signal_source_c(const std::string &address,
        int16_t port,
        bool flip_iq);

    rtl_tcp_signal_source_c(const std::string &address,
        int16_t port,
        bool flip_iq);

    // async read callback
    void handle_read(const boost::system::error_code &ec,
        size_t bytes_transferred);

    inline bool not_full() const
    {
        return unread_ < buffer_.capacity();
    }

    inline bool not_empty() const
    {
        return unread_ > 0 || io_context_.stopped();
    }

    boost::circular_buffer_space_optimized<float> buffer_;
    // producer-consumer helpers
    boost::mutex mutex_;
    boost::condition not_full_;
    boost::condition not_empty_;

    // lookup for scaling data
    boost::array<float, 0xff> lookup_{};

    // IO members
    b_io_context io_context_;
    boost::asio::ip::tcp::socket socket_;
    std::vector<unsigned char> data_;

    Rtl_Tcp_Dongle_Info info_;
    size_t unread_;
    bool flip_iq_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_C_H
