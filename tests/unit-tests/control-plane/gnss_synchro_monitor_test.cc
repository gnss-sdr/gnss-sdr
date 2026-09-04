/*!
 * \file gnss_synchro_monitor_test.cc
 * \brief This file implements tests for gnss_synchro_monitor
 * \author Vladislav P, 2026. vladisslav2011(at)gmail.com
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


#include "gnss_block_interface.h"
#include "gnss_synchro_monitor.h"
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <gnuradio/blocks/api.h>
#include <gnuradio/runtime_types.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/top_block.h>
#include <set>
#include <thread>
#include <vector>

class gnss_synchro_emitter;

using gnss_synchro_emitter_sptr = gnss_shared_ptr<gnss_synchro_emitter>;

class gnss_synchro_emitter : public gr::sync_block
{
public:
    virtual ~gnss_synchro_emitter() = default;

    // Emit syncros one by one with small pause to match actual "Monitor.enable_monitor=true" GNSS-SDR configuration behavior
    int work(int /*noutput_items*/, gr_vector_const_void_star& /*input_items*/, gr_vector_void_star& output_items)
    {
        auto** out = reinterpret_cast<Gnss_Synchro**>(&output_items[0]);
        if (counter >= d_nitems)
            {
                d_finished = true;
                return WORK_DONE;
            }
        for (int i = 0; i < d_nchannels; i++)
            {
                out[i][0] = vgs[i];
                vgs[i].interp_TOW_ms++;
            }
        counter++;
        usleep(d_interval_us);
        return 1;
    }

    void reset()
    {
        d_finished = false;
    }

    bool finished()
    {
        return d_finished;
    }

private:
    friend gnss_synchro_emitter_sptr gnss_synchro_make_emitter(int n_channels, int nitems, int interval_us);

    gnss_synchro_emitter(int n_channels, int nitems, int interval_us) : sync_block(
                                                                            "gnss_synchro_emitter", gr::io_signature::make(0, 0, 0), gr::io_signature::make(n_channels, n_channels, sizeof(Gnss_Synchro))),
                                                                        d_nitems(nitems),
                                                                        d_nchannels(n_channels),
                                                                        d_interval_us(interval_us)
    {
        for (int i = 0; i < d_nchannels; i++)
            {
                Gnss_Synchro gs{};
                // unique key: Channel_ID, interp_TOW_ms
                gs.Channel_ID = i;
                gs.interp_TOW_ms = 0;
                // PRN is filled with a known value to validate deserialization result
                gs.PRN = i + 1;
                vgs.push_back(gs);
            }
    }

    const int d_nitems;
    const int d_nchannels;
    const int d_interval_us;
    std::vector<Gnss_Synchro> vgs;
    int counter{0};
    bool d_finished{false};
};

gnss_synchro_emitter_sptr gnss_synchro_make_emitter(int n_channels, int nitems, int interval_us)
{
    return gnss_synchro_emitter_sptr(new gnss_synchro_emitter(n_channels, nitems, interval_us));
}


TEST(Monitor, decimation)
{
    // Minimum number of channels to confirm, that decimation by 2 is not performed just by dropping all synchros from one channel
    constexpr uint32_t NCHANNELS = 2;
    constexpr uint32_t NSYNCHRO = 16;
    constexpr uint32_t NDEC_MAX = 8;
    // Limit receive the test execution time
    constexpr uint32_t RX_LOOP_MAX_ITER = 32;
    constexpr uint32_t GNSS_SYNCHRO_INTERVAL_US = 1000;
    b_io_context io_context;
    boost::asio::ip::udp::socket socket(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 1112));
    std::vector<char> recv_buf(8192);
    boost::asio::ip::udp::endpoint remote_endpoint;
    for (uint32_t ndec = 1; ndec <= NDEC_MAX; ndec <<= 1)
        {
            std::cout << "Testing decimation factor = " << ndec << "\n";
            const uint32_t nexpected = NSYNCHRO / ndec;
            std::set<uint32_t> expected{};
            uint32_t counter{0};
            for (uint32_t i = 0; i < NSYNCHRO; i++)
                {
                    counter++;
                    if (counter >= ndec)
                        {
                            expected.insert(i);
                            counter = 0;
                        }
                }
            std::vector<std::set<uint32_t>> received(NCHANNELS);
            gnss_sdr::Observables obs{};
            Serdes_Gnss_Synchro serdes = Serdes_Gnss_Synchro();
            auto top_block = gr::make_top_block("monitor_test");
            auto emitter = gnss_synchro_make_emitter(NCHANNELS, NSYNCHRO, GNSS_SYNCHRO_INTERVAL_US);
            auto monitor = gnss_synchro_make_monitor(NCHANNELS, ndec, {"1112"}, {"127.0.0.1"}, true);
            for (uint32_t i = 0; i < NCHANNELS; i++)
                {
                    top_block->connect(emitter, i, monitor, i);
                }
            top_block->start();
            for (uint32_t rx_iter = 0; rx_iter < RX_LOOP_MAX_ITER; rx_iter++)
                {
                    usleep(GNSS_SYNCHRO_INTERVAL_US);
                    if (socket.available() == 0)
                        {
                            continue;
                        }
                    size_t nbytes = socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint);
                    EXPECT_GT(nbytes, 0);
                    obs.ParseFromArray(recv_buf.data(), nbytes);
                    const std::vector<Gnss_Synchro> vgs_read = serdes.readProtobuffer(obs);
                    // validate deserialization result and add it's unique index to set
                    for (auto& gs : vgs_read)
                        {
                            EXPECT_EQ(gs.PRN, gs.Channel_ID + 1);
                            received[gs.Channel_ID].insert(gs.interp_TOW_ms);
                        }
                    // Everything received, exit loop
                    if (received[0].size() >= nexpected)
                        {
                            break;
                        }
                }
            top_block->stop();
            top_block->wait();
            // Check the number of unique syncros per channel against the expected number
            for (auto& gs_set : received)
                {
                    EXPECT_EQ(gs_set.size(), nexpected);
                    EXPECT_EQ(gs_set, expected);
                }
        }
}
