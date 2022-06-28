/*!
 * \file rtl_tcp_signal_source_c.cc
 * \brief An rtl_tcp signal source reader.
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
 *
 * This module contains logic taken from gr-omsosdr
 * <https://git.osmocom.org/gr-osmosdr>
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

#include "rtl_tcp_signal_source_c.h"
#include "rtl_tcp_commands.h"
#include <boost/bind/bind.hpp>
#include <boost/thread/thread.hpp>
#include <glog/logging.h>
#include <map>


namespace ip = boost::asio::ip;

// Buffer constants
// TODO: Make these configurable
enum
{
    RTL_TCP_BUFFER_SIZE = 1024 * 16,  // 16 KB
    RTL_TCP_PAYLOAD_SIZE = 1024 * 4   //  4 KB
};

rtl_tcp_signal_source_c_sptr rtl_tcp_make_signal_source_c(
    const std::string &address,
    int16_t port,
    bool flip_iq)
{
    return rtl_tcp_signal_source_c_sptr(new rtl_tcp_signal_source_c(address,
        port,
        flip_iq));
}


rtl_tcp_signal_source_c::rtl_tcp_signal_source_c(const std::string &address,
    int16_t port,
    bool flip_iq)
    : gr::sync_block("rtl_tcp_signal_source_c",
          gr::io_signature::make(0, 0, 0),
          gr::io_signature::make(1, 1, sizeof(gr_complex))),
      buffer_(RTL_TCP_BUFFER_SIZE),
      socket_(io_context_),
      data_(RTL_TCP_PAYLOAD_SIZE),
      unread_(0),
      flip_iq_(flip_iq)
{
    boost::system::error_code ec;

    // 1. Setup lookup table
    for (unsigned i = 0; i < 0xff; i++)
        {
            lookup_[i] = (static_cast<float>(i & 0xff) - 127.4F) * (1.0F / 128.0F);
        }

    // 2. Set socket options
    ip::address addr = ip::address::from_string(address, ec);
    if (ec)
        {
            std::cout << address << " is not an IP address\n";
            LOG(ERROR) << address << " is not an IP address";
            return;
        }
    ip::tcp::endpoint ep(addr, port);
    socket_.open(ep.protocol(), ec);
    if (ec)
        {
            std::cout << "Failed to open socket.\n";
            LOG(ERROR) << "Failed to open socket.";
        }

    socket_.set_option(boost::asio::socket_base::reuse_address(true), ec);
    if (ec)
        {
            std::cout << "Failed to set reuse address option: " << ec << '\n';
            LOG(WARNING) << "Failed to set reuse address option";
        }
    socket_.set_option(boost::asio::socket_base::linger(true, 0), ec);
    if (ec)
        {
            std::cout << "Failed to set linger option: " << ec << '\n';
            LOG(WARNING) << "Failed to set linger option";
        }

    // 3. Connect socket

    socket_.connect(ep, ec);
    if (ec)
        {
            std::cout << "Failed to connect to " << addr << ":" << port
                      << "(" << ec << ")\n";
            LOG(ERROR) << "Failed to connect to " << addr << ":" << port
                       << "(" << ec << ")";
            return;
        }
    std::cout << "Connected to " << addr << ":" << port << '\n';
    LOG(INFO) << "Connected to " << addr << ":" << port;

    // 4. Set nodelay
    socket_.set_option(ip::tcp::no_delay(true), ec);
    if (ec)
        {
            std::cout << "Failed to set no delay option.\n";
            LOG(WARNING) << "Failed to set no delay option";
        }

    // 5. Receive dongle info
    ec = info_.read(socket_);
    if (ec)
        {
            std::cout << "Failed to read dongle info.\n";
            LOG(WARNING) << "Failed to read dongle info";
        }
    else if (info_.is_valid())
        {
            std::cout << "Found " << info_.get_type_name() << " tuner.\n";
            LOG(INFO) << "Found " << info_.get_type_name() << " tuner.";
        }

// 6. Start reading
#if USE_BOOST_BIND_PLACEHOLDERS
    boost::asio::async_read(socket_, boost::asio::buffer(data_),
        boost::bind(&rtl_tcp_signal_source_c::handle_read, this, boost::placeholders::_1, boost::placeholders::_2));  // NOLINT(modernize-avoid-bind)
#else
    boost::asio::async_read(socket_, boost::asio::buffer(data_),
        boost::bind(&rtl_tcp_signal_source_c::handle_read, this, _1, _2));  // NOLINT(modernize-avoid-bind)
#endif

    boost::thread(
#if HAS_GENERIC_LAMBDA
        [ObjectPtr = &io_context_] { ObjectPtr->run(); });
#else
        boost::bind(&b_io_context::run, &io_context_));
#endif
}


rtl_tcp_signal_source_c::~rtl_tcp_signal_source_c()  // NOLINT(modernize-use-equals-default)
{
    mutex_.unlock();
    io_context_.stop();
    not_empty_.notify_one();
    not_full_.notify_one();
}


void rtl_tcp_signal_source_c::set_frequency(int frequency)
{
    boost::system::error_code ec =
        rtl_tcp_command(RTL_TCP_SET_FREQUENCY, frequency, socket_);
    if (ec)
        {
            std::cout << "Failed to set frequency\n";
            LOG(WARNING) << "Failed to set frequency";
        }
}


void rtl_tcp_signal_source_c::set_sample_rate(int sample_rate)
{
    boost::system::error_code ec =
        rtl_tcp_command(RTL_TCP_SET_SAMPLE_RATE, sample_rate, socket_);
    if (ec)
        {
            std::cout << "Failed to set sample rate\n";
            LOG(WARNING) << "Failed to set sample rate";
        }
}


void rtl_tcp_signal_source_c::set_agc_mode(bool agc)
{
    boost::system::error_code ec =
        rtl_tcp_command(RTL_TCP_SET_GAIN_MODE, !agc, socket_);
    if (ec)
        {
            std::cout << "Failed to set gain mode\n";
            LOG(WARNING) << "Failed to set gain mode";
        }
    ec = rtl_tcp_command(RTL_TCP_SET_AGC_MODE, agc, socket_);
    if (ec)
        {
            std::cout << "Failed to set gain mode\n";
            LOG(WARNING) << "Failed to set gain mode";
        }
}


void rtl_tcp_signal_source_c::set_gain(int gain)
{
    auto clipped = static_cast<unsigned>(info_.clip_gain(gain) * 10.0);
    boost::system::error_code ec = rtl_tcp_command(RTL_TCP_SET_GAIN, clipped, socket_);
    if (ec)
        {
            std::cout << "Failed to set gain\n";
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
    if (info_.get_tuner_type() != Rtl_Tcp_Dongle_Info::TUNER_E4000)
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
    for (size_t i = 0; i < ranges.size(); i++)
        {
            gains[i + 1] = ranges[i].start;
        }

    for (size_t i = ranges.size() - 1; i > 0; i--)
        {
            const range &r = ranges[i];
            double error = gain;
            double g = r.start;
            while (g < r.stop)
                {
                    double sum = 0;
                    for (size_t j = 0; j < gains.size(); j++)
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
                    g += r.step;
                }
        }
    for (size_t stage = 1; stage <= gains.size(); stage++)
        {
            int stage_gain = static_cast<int>(gains[stage] * 10);
            size_t param = (stage << 16) | (stage_gain & 0xffff);
            boost::system::error_code ec = rtl_tcp_command(RTL_TCP_SET_IF_GAIN, param, socket_);
            if (ec)
                {
                    std::cout << "Failed to set if gain\n";
                    LOG(WARNING) << "Failed to set if gain";
                }
        }
}


void rtl_tcp_signal_source_c::handle_read(const boost::system::error_code &ec,
    size_t bytes_transferred)
{
    if (ec)
        {
            std::cout << "Error during read: " << ec << '\n';
            LOG(WARNING) << "Error during read: " << ec;
            boost::mutex::scoped_lock lock(mutex_);
            io_context_.stop();
            not_empty_.notify_one();
        }
    else
        {
            {
                // Unpack read data
                boost::mutex::scoped_lock lock(mutex_);
                not_full_.wait(lock,
                    boost::bind(&rtl_tcp_signal_source_c::not_full, this));  // NOLINT(modernize-avoid-bind)

                for (size_t i = 0; i < bytes_transferred; i++)
                    {
                        while (!not_full())
                            {
                                // uh-oh, buffer overflow
                                // wait until there's space for more
                                not_empty_.notify_one();  // needed?
                                not_full_.wait(lock,
                                    boost::bind(&rtl_tcp_signal_source_c::not_full, this));  // NOLINT(modernize-avoid-bind)
                            }

                        buffer_.push_front(lookup_[data_[i]]);
                        unread_++;
                    }
            }
            // let woker know that more data is available
            not_empty_.notify_one();
// Read some more
#if USE_BOOST_BIND_PLACEHOLDERS
            boost::asio::async_read(socket_,
                boost::asio::buffer(data_),
                boost::bind(&rtl_tcp_signal_source_c::handle_read, this, boost::placeholders::_1, boost::placeholders::_2));  // NOLINT(modernize-avoid-bind)
#else
            boost::asio::async_read(socket_,
                boost::asio::buffer(data_),
                boost::bind(&rtl_tcp_signal_source_c::handle_read, this, _1, _2));  // NOLINT(modernize-avoid-bind)
#endif
        }
}


int rtl_tcp_signal_source_c::work(int noutput_items,
    gr_vector_const_void_star & /*input_items*/,
    gr_vector_void_star &output_items)
{
    auto *out = reinterpret_cast<gr_complex *>(output_items[0]);
    int i = 0;
    if (io_context_.stopped())
        {
            return -1;
        }

    {
        boost::mutex::scoped_lock lock(mutex_);
        not_empty_.wait(lock,
            boost::bind(&rtl_tcp_signal_source_c::not_empty, this));  // NOLINT(modernize-avoid-bind)

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
