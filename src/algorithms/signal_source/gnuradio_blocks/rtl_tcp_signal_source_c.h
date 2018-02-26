/*!
 * \file rtl_tcp_signal_source_c.h
 * \brief Interface of an rtl_tcp signal source reader.
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
 *
 * The implementation of this block is a combination of various helpful
 * sources. The data format and command structure is taken from the
 * original Osmocom rtl_tcp_source_f (http://git.osmocom.org/gr-osmosdr).
 * The aynchronous reading code comes from the examples provides
 * by Boost.Asio and the bounded buffer producer-consumer solution is
 * taken from the Boost.CircularBuffer examples (http://boost.org/).
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

#ifndef GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_C_H
#define GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_C_H

#include "rtl_tcp_dongle_info.h"
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/array.hpp>
#include <boost/circular_buffer.hpp>
#include <gnuradio/sync_block.h>
#include <string>

class rtl_tcp_signal_source_c;

typedef boost::shared_ptr<rtl_tcp_signal_source_c>
        rtl_tcp_signal_source_c_sptr;

rtl_tcp_signal_source_c_sptr
rtl_tcp_make_signal_source_c(const std::string &address,
                             short port,
                             bool flip_iq = false);

/*!
 * \brief This class reads interleaved I/Q samples
 * from an rtl_tcp server and outputs complex types.
 */
class rtl_tcp_signal_source_c : public gr::sync_block
{
public:
    ~rtl_tcp_signal_source_c();

    int work (int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);

    void set_frequency (int frequency);
    void set_sample_rate (int sample_rate);
    void set_agc_mode (bool agc);
    void set_gain (int gain);
    void set_if_gain (int gain);

private:
    typedef boost::circular_buffer_space_optimized<float> buffer_type;

    friend rtl_tcp_signal_source_c_sptr
    rtl_tcp_make_signal_source_c(const std::string &address,
            short port,
            bool flip_iq);

    rtl_tcp_signal_source_c(const std::string &address,
            short port,
            bool flip_iq);

    rtl_tcp_dongle_info info_;

    // IO members
    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::socket socket_;
    std::vector<unsigned char> data_;
    bool flip_iq_;

    // producer-consumer helpers
    boost::mutex mutex_;
    boost::condition not_full_;
    boost::condition not_empty_;
    buffer_type buffer_;
    size_t unread_;

    // lookup for scaling data
    boost::array<float, 0xff> lookup_;

    // async read callback
    void handle_read (const boost::system::error_code &ec,
            size_t bytes_transferred);

    inline bool not_full ( ) const
    {
        return unread_ < buffer_.capacity( );
    }

    inline bool not_empty ( ) const
    {
        return unread_ > 0 || io_service_.stopped ();
    }
};

#endif // GNSS_SDR_RTL_TCP_SIGNAL_SOURCE_C_H
