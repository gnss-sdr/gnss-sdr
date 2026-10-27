/*!
 * \file deterministic_telemetry_fault_test.cc
 * \brief Checks deterministic telemetry fault handoff and progress ordering
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "deterministic_clock.h"
#include "deterministic_tracking.h"
#include "galileo_telemetry_decoder_gs.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_telemetry_decoder_gs.h"
#include "gps_l2c_telemetry_decoder_gs.h"
#include "gps_l5_telemetry_decoder_gs.h"
#include "horizon_gate.h"
#include "telemetry_message_stamp.h"
#include "tlm_conf.h"
#include <gnuradio/block_detail.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/buffer.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#ifdef GR_GREATER_38
#include <gnuradio/blocks/vector_source.h>
#else
#include <gnuradio/blocks/vector_source_b.h>
#endif


class DeterministicTelemetryFaultTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        DeterministicClock::instance().enable(1000.0, 100, 100);
    }

    void TearDown() override
    {
        DeterministicClock::instance().disable();
    }

    void check_fault_order(const gr::block_sptr& decoder)
    {
        // Let GNU Radio allocate the stream buffers, then drive work directly
        // with no scheduler threads so that the publication boundary is observable.
        auto source = gr::blocks::vector_source_b::make(std::vector<uint8_t>(sizeof(Gnss_Synchro), 0), false, sizeof(Gnss_Synchro));
        auto sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        auto top_block = gr::make_top_block("deterministic_telemetry_fault_test");
        top_block->connect(source, 0, decoder, 0);
        top_block->connect(decoder, 0, sink, 0);
        top_block->run();

        // No scheduler dispatches telemetry messages during these work calls.
        // An invalid destination also catches any accidental asynchronous fault.
        const auto port = pmt::mp("telemetry_to_trk");
        decoder->message_port_sub(port, pmt::cons(sink->alias_pmt(), pmt::mp("faults")));

        auto& clock = DeterministicClock::instance();
        DeterministicTracking tracking(source.get());
        tracking.start(0);
        Gnss_Synchro symbol{};
        symbol.Prompt_I = 1.0;
        symbol.fs = 1000;
        Gnss_Synchro output{};
        gr_vector_const_void_star inputs{&symbol};
        gr_vector_void_star outputs{&output};
        gr_vector_int available{1};
        bool fault_applied = false;
        for (uint64_t i = 1; i <= 20000; ++i)
            {
                symbol.Tracking_sample_counter = i * 20;
                const uint64_t consumed = decoder->nitems_read(0);
                tracking.record_output(consumed, symbol.Tracking_sample_counter);
                tracking.publish(symbol.Tracking_sample_counter + clock.lookahead());
                ASSERT_EQ(clock.min_progress(), symbol.Tracking_sample_counter);
                ASSERT_EQ(decoder->general_work(1, available, inputs, outputs), 1);
                EXPECT_EQ(decoder->nitems_read(0), consumed + 1);
                EXPECT_GT(clock.min_progress(), symbol.Tracking_sample_counter);
                const uint64_t effective = clock.effective_sample(symbol.Tracking_sample_counter);
                EXPECT_FALSE(tracking.fault_due(effective - 1, 0));
                if (tracking.fault_due(effective, 0))
                    {
                        EXPECT_FALSE(tracking.fault_due(effective, 0));
                        fault_applied = true;
                        break;
                    }
            }
        EXPECT_TRUE(fault_applied);
    }
};


TEST_F(DeterministicTelemetryFaultTest, GpsL1CaFaultWithoutMessageDispatch)
{
    auto decoder = gps_l1_ca_make_telemetry_decoder_gs(Tlm_Conf{});
    decoder->set_satellite(Gnss_Satellite("GPS", 1));
    check_fault_order(decoder);
}


TEST_F(DeterministicTelemetryFaultTest, GalileoInavFaultWithoutMessageDispatch)
{
    auto decoder = galileo_make_telemetry_decoder_gs(Tlm_Conf{}, 1);
    decoder->set_satellite(Gnss_Satellite("Galileo", 1));
    check_fault_order(decoder);
}


TEST_F(DeterministicTelemetryFaultTest, GpsL2cFaultWithoutMessageDispatch)
{
    auto decoder = gps_l2c_make_telemetry_decoder_gs(Tlm_Conf{});
    decoder->set_satellite(Gnss_Satellite("GPS", 1));
    check_fault_order(decoder);
}


TEST_F(DeterministicTelemetryFaultTest, GpsL5FaultWithoutMessageDispatch)
{
    auto decoder = gps_l5_make_telemetry_decoder_gs(Tlm_Conf{});
    decoder->set_satellite(Gnss_Satellite("GPS", 1));
    check_fault_order(decoder);
}


TEST_F(DeterministicTelemetryFaultTest, FaultsAreIsolatedByChannelAndTrackingSession)
{
    auto source = gr::blocks::vector_source_b::make(std::vector<uint8_t>{0});
    DeterministicTracking tracking(source.get());
    tracking.start(100);  // This session starts at sample 200.
    auto& clock = DeterministicClock::instance();
    clock.report_telemetry_fault(0, 199);
    clock.report_telemetry_fault(1, 200);
    clock.report_telemetry_fault(0, 220);
    clock.report_telemetry_fault(0, 200);
    EXPECT_FALSE(tracking.fault_due(299, 0));
    EXPECT_TRUE(tracking.fault_due(300, 0));
    EXPECT_FALSE(tracking.fault_due(320, 0));
    EXPECT_EQ(clock.take_telemetry_fault(1, 200), 200U);

    clock.report_telemetry_fault(0, 250);
    tracking.start(200);  // A restarted session must ignore the old fault.
    EXPECT_FALSE(tracking.fault_due(400, 0));
    clock.report_telemetry_fault(0, 300);
    EXPECT_TRUE(tracking.fault_due(400, 0));
}


TEST_F(DeterministicTelemetryFaultTest, NewRunClearsQueuedFaults)
{
    auto& clock = DeterministicClock::instance();
    clock.report_telemetry_fault(0, 100);
    clock.enable(1000.0, 100, 100);
    EXPECT_EQ(clock.take_telemetry_fault(0, 0), NO_SAMPLE_STAMP);
    clock.report_telemetry_fault(0, 100);
    clock.disable();
    EXPECT_EQ(clock.take_telemetry_fault(0, 0), NO_SAMPLE_STAMP);
}


TEST_F(DeterministicTelemetryFaultTest, ProgressWaitsForWorkScopeToFinish)
{
    auto source = gr::blocks::vector_source_b::make(std::vector<uint8_t>{0});
    auto sink = gr::blocks::null_sink::make(1);
    auto top_block = gr::make_top_block("telemetry_scope_test");
    top_block->connect(source, 0, sink, 0);
    top_block->run();
    auto& clock = DeterministicClock::instance();
    const auto consumed = sink->nitems_read(0);
    DeterministicTracking tracking(source.get());
    tracking.record_output(consumed, 100);
    tracking.publish(200);
    {
        const TelemetryWorkScope scope(sink.get(), 100);
        sink->consume_each(1);
        EXPECT_EQ(clock.min_progress(), 100U);
        clock.report_telemetry_fault(0, 100);
        EXPECT_EQ(clock.min_progress(), 100U);
    }
    EXPECT_EQ(clock.min_progress(), 200U);
    EXPECT_TRUE(tracking.fault_due(200, 0));

    const auto finished = sink->nitems_read(0);
    tracking.record_output(finished, 200);
    tracking.publish(300);
    EXPECT_THROW(
        {
            const TelemetryWorkScope scope(sink.get(), 200);
            sink->consume_each(1);
            throw std::runtime_error("decoder failed before handing off its fault");
        },
        std::runtime_error);
    EXPECT_EQ(clock.consumed(static_cast<gr::block*>(sink.get()), sink->nitems_read(0)), finished);
    EXPECT_EQ(clock.min_progress(), 200U);
}


struct DeterministicFaultSchedulerState
{
    std::mutex mutex;
    std::condition_variable condition;
    bool paused = false;
    bool published = false;
    bool resume = false;
    std::atomic<uint64_t> applied_at{NO_SAMPLE_STAMP};
};


class DeterministicFaultTrackingBlock : public gr::block
{
public:
    explicit DeterministicFaultTrackingBlock(DeterministicFaultSchedulerState& state)
        : gr::block("fault_tracking_test", gr::io_signature::make(1, 1, 1), gr::io_signature::make(1, 1, 1)),
          d_state(state),
          d_tracking(this)
    {
        auto fault_port = pmt::mp("telemetry_to_trk");
        d_pause_port = pmt::mp("pause_after_fault_queue");
        // GNU Radio drains the message ports in this comparator's order.
        // Pause only after it has checked the fault port for this iteration.
        if (pmt::comparator{}(d_pause_port, fault_port))
            {
                std::swap(d_pause_port, fault_port);
            }
        message_port_register_in(fault_port);
        message_port_register_in(d_pause_port);
        set_msg_handler(fault_port, [this](const pmt::pmt_t& msg) {
            d_tracking.take_fault(msg, nitems_read(0), true, 0);
        });
        set_msg_handler(d_pause_port, [this](const pmt::pmt_t&) {
            std::unique_lock<std::mutex> lock(d_state.mutex);
            d_state.paused = true;
            d_state.condition.notify_all();
            d_state.condition.wait_for(lock, std::chrono::seconds(5), [this] { return d_state.resume; });
        });
        d_tracking.start(0);
    }

    void forecast(int, gr_vector_int& required) override { required[0] = 10; }

    int general_work(int, gr_vector_int&, gr_vector_const_void_star&, gr_vector_void_star& output) override
    {
        if (d_tracking.fault_due(nitems_read(0), 0))
            {
                d_state.applied_at.store(nitems_read(0));
            }
        consume_each(10);
        d_tracking.publish(nitems_read(0));
        if (nitems_read(0) == 200)
            {
                _post(d_pause_port, pmt::PMT_T);
            }
        // One symbol per call, stamped with the sample it reaches. A call
        // without output would wait, in GNU Radio >= 3.10, until the decoder
        // that holds a symbol leaves its work call (output blocked callback).
        d_tracking.record_output(nitems_written(0), nitems_read(0));
        static_cast<uint8_t*>(output[0])[0] = 0;
        return 1;
    }

private:
    DeterministicFaultSchedulerState& d_state;
    DeterministicTracking d_tracking;
    pmt::pmt_t d_pause_port;
};


class DeterministicFaultTelemetryBlock : public gr::block
{
public:
    explicit DeterministicFaultTelemetryBlock(DeterministicFaultSchedulerState& state)
        : gr::block("fault_telemetry_test", gr::io_signature::make(1, 1, 1), gr::io_signature::make(0, 0, 0)),
          d_state(state)
    {
    }

    int general_work(int, gr_vector_int&, gr_vector_const_void_star&, gr_vector_void_star&) override
    {
        // One symbol per call. The tracking block stamps symbol k with
        // sample 10 * (k + 1); the fault is raised by the one at sample 100.
        const uint64_t stamp = 10 * (nitems_read(0) + 1);
        {
            const TelemetryWorkScope scope(this, stamp);
            consume_each(1);
            if (stamp != 100)
                {
                    return 0;
                }
            std::unique_lock<std::mutex> lock(d_state.mutex);
            d_state.condition.wait_for(lock, std::chrono::seconds(5), [this] { return d_state.paused; });
            DeterministicClock::instance().report_telemetry_fault(0, 100);
        }
        std::lock_guard<std::mutex> lock(d_state.mutex);
        d_state.published = true;
        return 0;
    }

private:
    DeterministicFaultSchedulerState& d_state;
};


TEST_F(DeterministicTelemetryFaultTest, FaultReportedAfterSchedulerChecksMessages)
{
    auto& clock = DeterministicClock::instance();
    DeterministicFaultSchedulerState state;
    auto source = gr::blocks::vector_source_b::make(std::vector<uint8_t>(400, 0));
    auto gate = make_horizon_gate(1, 1);
    auto tracking = gnss_shared_ptr<DeterministicFaultTrackingBlock>(new DeterministicFaultTrackingBlock(state));
    auto telemetry = gnss_shared_ptr<DeterministicFaultTelemetryBlock>(new DeterministicFaultTelemetryBlock(state));
    auto top_block = gr::make_top_block("deterministic_fault_scheduler_test");
    top_block->connect(source, 0, gate, 0);
    top_block->connect(gate, 0, tracking, 0);
    top_block->connect(tracking, 0, telemetry, 0);
    top_block->start();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (state.applied_at.load() == NO_SAMPLE_STAMP && std::chrono::steady_clock::now() < deadline)
        {
            clock.advance_horizon(clock.min_progress() + clock.lookahead());
            uint64_t released;
            {
                const auto output = gate->detail()->output(0);
                gr::thread::scoped_lock lock(*output->mutex());
                released = gate->nitems_written(0);
            }
            {
                std::lock_guard<std::mutex> lock(state.mutex);
                // Give tracking enough input to cross the effective sample
                // before it gets another chance to dispatch queued messages.
                if (state.published && released >= 300)
                    {
                        state.resume = true;
                        state.condition.notify_all();
                    }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.resume = true;
        state.condition.notify_all();
    }
    clock.open_horizon();
    top_block->stop();
    top_block->wait();
    EXPECT_TRUE(state.paused);
    EXPECT_TRUE(state.published);
    EXPECT_EQ(state.applied_at.load(), 200U);
}
