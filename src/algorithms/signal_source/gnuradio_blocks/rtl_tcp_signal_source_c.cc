/*!
 * \file rtl_tcp_signal_source_c.cc
 * \brief An rtl_tcp signal source reader.
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
 *
 * This module contains logic taken from gr-omsosdr
 * <http://git.osmocom.org/gr-osmosdr>
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

#include "rtl_tcp_signal_source_c.h"
#include "rtl_tcp_commands.h"
#include <glog/logging.h>
#include <boost/thread/thread.hpp>
#include <map>

using google::LogMessage;

namespace ip = boost::asio::ip;
using boost::asio::ip::tcp;

// Buffer constants
// TODO: Make these configurable
enum
{
    RTL_TCP_BUFFER_SIZE = 1024 * 16,  // 16 KB
    RTL_TCP_PAYLOAD_SIZE = 1024 * 4   //  4 KB
};

rtl_tcp_signal_source_c_sptr
rtl_tcp_make_signal_source_c(const std::string &address,
    short port,
    bool flip_iq)
{
    return gnuradio::get_initial_sptr(new rtl_tcp_signal_source_c(address,
        port,
        flip_iq));
}


rtl_tcp_signal_source_c::rtl_tcp_signal_source_c(const std::string &address,
    short port,
    bool flip_iq)
    : gr::sync_block("rtl_tcp_signal_source_c",
          gr::io_signature::make(0, 0, 0),
          gr::io_signature::make(1, 1, sizeof(gr_complex))),
      socket_(io_service_),
      data_(RTL_TCP_PAYLOAD_SIZE),
      flip_iq_(flip_iq),
      buffer_(RTL_TCP_BUFFER_SIZE),
      unread_(0)
{
    boost::system::error_code ec;

    // 1. Setup lookup table
    for (unsigned i = 0; i < 0xff; i++)
        {
            lookup_[i] = (static_cast<float>(i & 0xff) - 127.4f) * (1.0f / 128.0f);
        }

    // 2. Set socket options
    ip::address addr = ip::address::from_string(address, ec);
    if (ec)
        {
            std::cout << address << " is not an IP address" << std::endl;
            LOG(ERROR) << address << " is not an IP address";
            return;
        }
    ip::tcp::endpoint ep(addr, port);
    socket_.open(ep.protocol(), ec);
    if (ec)
        {
            std::cout << "Failed to open socket." << std::endl;
            LOG(ERROR) << "Failed to open socket.";
        }

    socket_.set_option(boost::asio::socket_base::reuse_address(true), ec);
    if (ec)
        {
            std::cout << "Failed to set reuse address option: " << ec << std::endl;
            LOG(WARNING) << "Failed to set reuse address option";
        }
    socket_.set_option(boost::asio::socket_base::linger(true, 0), ec);
    if (ec)
        {
            std::cout << "Failed to set linger option: " << ec << std::endl;
            LOG(WARNING) << "Failed to set linger option";
        }

    // 3. Connect socket

    socket_.connect(ep, ec);
    if (ec)
        {
            std::cout << "Failed to connect to " << addr << ":" << port
                      << "(" << ec << ")" << std::endl;
            LOG(ERROR) << "Failed to connect to " << addr << ":" << port
                       << "(" << ec << ")";
            return;
        }
    std::cout << "Connected to " << addr << ":" << port << std::endl;
    LOG(INFO) << "Connected to " << addr << ":" << port;

    // 4. Set nodelay
    socket_.set_option(tcp::no_delay(true), ec);
    if (ec)
        {
            std::cout << "Failed to set no delay option." << std::endl;
            LOG(WARNING) << "Failed to set no delay option";
        }

    // 5. Receive dongle info
    ec = info_.read(socket_);
    if (ec)
        {
            std::cout << "Failed to read dongle info." << std::endl;
            LOG(WARNING) << "Failed to read dongle info";
        }
    else if (info_.is_valid())
        {
            std::cout << "Found " << info_.get_type_name() << " tuner." << std::endl;
            LOG(INFO) << "Found " << info_.get_type_name() << " tuner.";
        }

    // 6. Start reading
    boost::asio::async_read(socket_, boost::asio::buffer(data_),
        boost::bind(&rtl_tcp_signal_source_c::handle_read,
            this, _1, _2));
    boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
}


rtl_tcp_signal_source_c::~rtl_tcp_signal_source_c()
{
    boost::mutex::scoped_lock lock(mutex_);
    io_service_.stop();
    not_empty_.notify_one();
}


void rtl_tcp_signal_source_c::set_frequency(int frequency)
{
    boost::system::error_code ec =
        rtl_tcp_command(RTL_TCP_SET_FREQUENCY, frequency, socket_);
    if (ec)
        {
            std::cout << "Failed to set frequency" << std::endl;
            LOG(WARNING) << "Failed to set frequency";
        }
}


void rtl_tcp_signal_source_c::set_sample_rate(int sample_rate)
{
    boost::system::error_code ec =
        rtl_tcp_command(RTL_TCP_SET_SAMPLE_RATE, sample_rate, socket_);
    if (ec)
        {
            std::cout << "Failed to set sample rate" << std::endl;
            LOG(WARNING) << "Failed to set sample rate";
        }
}


void rtl_tcp_signal_source_c::set_agc_mode(bool agc)
{
    boost::system::error_code ec =
        rtl_tcp_command(RTL_TCP_SET_GAIN_MODE, !agc, socket_);
    if (ec)
        {
            std::cout << "Failed to set gain mode" << std::endl;
            LOG(WARNING) << "Failed to set gain mode";
        }
    ec = rtl_tcp_command(RTL_TCP_SET_AGC_MODE, agc, socket_);
    if (ec)
        {
            std::cout << "Failed to set gain mode" << std::endl;
            LOG(WARNING) << "Failed to set gain mode";
        }
}


void rtl_tcp_signal_source_c::set_gain(int gain)
{
    unsigned clipped = static_cast<unsigned>(info_.clip_gain(gain) * 10.0);
    boost::system::error_code ec = rtl_tcp_command(RTL_TCP_SET_GAIN, clipped, socket_);
    if (ec)
        {
            std::cout << "Failed to set gain" << std::endl;
            LOG(WARNING) << "Failed to set gain";
        }
}


void rtl_tcp_signal_source_c::set_if_gain(int gain)
{
    // from gr-osmosdr
    struct range
    {
        double start, stop, step;
    };
    if (info_.get_tuner_type() != rtl_tcp_dongle_info::TUNER_E4000)
        {
            return;
        }

    std::vector<range> ranges = {
        {-3, 6, 9},
        {0, 9, 3},
        {0, 9, 3},
        {0, 2, 1},
        {3, 15, 3},
        {3, 15, 3}};

    std::map<int, double> gains;
    for (int i = 0; i < static_cast<int>(ranges.size()); i++)
        {
            gains[i + 1] = ranges[i].start;
        }

    for (int i = ranges.size() - 1; i >= 0; i--)
        {
            const range &r = ranges[i];
            double error = gain;

            for (double g = r.start; g < r.stop; g += r.step)
                {
                    double sum = 0;
                    for (int j = 0; j < static_cast<int>(gains.size()); j++)
                        {
                            if (i == j)
                                {
                                    sum += g;
                                }
                            else
                                {
                                    sum += gains[j + 1];
                                }
                        }
                    double err = std::abs(gain - sum);
                    if (err < error)
                        {
                            error = err;
                            gains[i + 1] = g;
                        }
                }
        }
    for (unsigned stage = 1; stage <= gains.size(); stage++)
        {
            int stage_gain = static_cast<int>(gains[stage] * 10);
            unsigned param = (stage << 16) | (stage_gain & 0xffff);
            boost::system::error_code ec = rtl_tcp_command(RTL_TCP_SET_IF_GAIN, param, socket_);
            if (ec)
                {
                    std::cout << "Failed to set if gain" << std::endl;
                    LOG(WARNING) << "Failed to set if gain";
                }
        }
}


void rtl_tcp_signal_source_c::handle_read(const boost::system::error_code &ec,
    size_t bytes_transferred)
{
    if (ec)
        {
            std::cout << "Error during read: " << ec << std::endl;
            LOG(WARNING) << "Error during read: " << ec;
            boost::mutex::scoped_lock lock(mutex_);
            io_service_.stop();
            not_empty_.notify_one();
        }
    else
        {
            {
                // Unpack read data
                boost::mutex::scoped_lock lock(mutex_);
                not_full_.wait(lock,
                    boost::bind(&rtl_tcp_signal_source_c::not_full,
                        this));

                for (size_t i = 0; i < bytes_transferred; i++)
                    {
                        while (!not_full())
                            {
                                // uh-oh, buffer overflow
                                // wait until there's space for more
                                not_empty_.notify_one();  // needed?
                                not_full_.wait(lock,
                                    boost::bind(&rtl_tcp_signal_source_c::not_full,
                                        this));
                            }

                        buffer_.push_front(lookup_[data_[i]]);
                        unread_++;
                    }
            }
            // let woker know that more data is available
            not_empty_.notify_one();
            // Read some more
            boost::asio::async_read(socket_,
                boost::asio::buffer(data_),
                boost::bind(&rtl_tcp_signal_source_c::handle_read,
                    this, _1, _2));
        }
}


int rtl_tcp_signal_source_c::work(int noutput_items,
    gr_vector_const_void_star & /*input_items*/,
    gr_vector_void_star &output_items)
{
    gr_complex *out = reinterpret_cast<gr_complex *>(output_items[0]);
    int i = 0;
    if (io_service_.stopped())
        {
            return -1;
        }

    {
        boost::mutex::scoped_lock lock(mutex_);
        not_empty_.wait(lock, boost::bind(&rtl_tcp_signal_source_c::not_empty,
                                  this));

        for (; i < noutput_items && unread_ > 1; i++)
            {
                float re = buffer_[--unread_];
                float im = buffer_[--unread_];
                if (flip_iq_)
                    {
                        out[i] = gr_complex(im, re);
                    }
                else
                    {
                        out[i] = gr_complex(re, im);
                    }
            }
    }
    not_full_.notify_one();
    return i == 0 ? -1 : i;
}
