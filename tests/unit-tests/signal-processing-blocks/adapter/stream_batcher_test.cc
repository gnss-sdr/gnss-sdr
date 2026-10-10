/*!
 * \file stream_batcher_test.cc
 * \brief Sample conservation, tags and scheduler liveness at the fan-out boundary
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

#include "gnss_block_factory.h"
#include "in_memory_configuration.h"
#include "signal_conditioner.h"
#include "stream_batcher.h"
#include "stream_batcher_deadline.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <limits>
#include <numeric>
#include <thread>

#ifdef GR_GREATER_38
#include <gnuradio/blocks/vector_sink.h>
#include <gnuradio/blocks/vector_source.h>
#else
#include <gnuradio/blocks/vector_sink_b.h>
#include <gnuradio/blocks/vector_sink_c.h>
#include <gnuradio/blocks/vector_sink_i.h>
#include <gnuradio/blocks/vector_source_b.h>
#include <gnuradio/blocks/vector_source_c.h>
#include <gnuradio/blocks/vector_source_s.h>
#endif

namespace
{
class ChunkedTestSource : public gr::sync_block
{
public:
    ChunkedTestSource(int length, int chunk, bool paced)
        : gr::sync_block("chunked_test_source", gr::io_signature::make(0, 0, 0), gr::io_signature::make(1, 1, sizeof(int))),
          length_(length),
          chunk_(chunk),
          paced_(paced)
    {
    }
    int work(int noutput, gr_vector_const_void_star& /*input_items*/, gr_vector_void_star& output) override
    {
        const int offset = static_cast<int>(nitems_written(0));
        if (offset == length_)
            {
                return WORK_DONE;
            }
        if (paced_)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        const int count = std::min({noutput, chunk_, length_ - offset});
        auto* out = static_cast<int*>(output[0]);
        for (int i = 0; i < count; ++i)
            {
                out[i] = offset + i;
                if (out[i] == 0 || out[i] == length_ / 2 || out[i] == length_ - 1)
                    {
                        add_item_tag(0, out[i], pmt::intern("sample"), pmt::from_long(out[i]));
                    }
            }
        return count;
    }

private:
    int length_;
    int chunk_;
    bool paced_;
};

// Deliberately supplies neither more input nor EOF after its first delivery.
class StalledTestSource : public gr::sync_block
{
public:
    StalledTestSource()
        : gr::sync_block("stalled_source", gr::io_signature::make(0, 0, 0), gr::io_signature::make(1, 1, sizeof(int))) {}
    bool start() override
    {
        sent_ = 0;
        return true;
    }
    int work(int count, gr_vector_const_void_star& /*input_items*/, gr_vector_void_star& output) override
    {
        const auto offset = nitems_written(0);
        if (sent_ >= 17)
            {
                return 0;
            }
        count = std::min(count, 17 - sent_);
        std::iota(static_cast<int*>(output[0]), static_cast<int*>(output[0]) + count, sent_);
        add_item_tag(0, offset + count - 1, pmt::mp("tail"), pmt::from_long(42), pmt::mp("source"));
        sent_ += count;
        return count;
    }

private:
    int sent_ = 0;
};

class ObservedTestSink : public gr::sync_block
{
public:
    ObservedTestSink()
        : gr::sync_block("observed_sink", gr::io_signature::make(1, 1, sizeof(int)), gr::io_signature::make(0, 0, 0)) {}
    int work(int count, gr_vector_const_void_star& input, gr_vector_void_star& /*output_items*/) override
    {
        const auto* items = static_cast<const int*>(input[0]);
        samples.insert(samples.end(), items, items + count);
        std::vector<gr::tag_t> incoming;
        get_tags_in_range(incoming, 0, nitems_read(0), nitems_read(0) + count);
        tags.insert(tags.end(), incoming.begin(), incoming.end());
        received.fetch_add(count);
        return count;
    }
    std::atomic<int> received{0};
    std::vector<int> samples;
    std::vector<gr::tag_t> tags;
};

class WindowTestSink : public gr::block
{
public:
    explicit WindowTestSink(int window)
        : gr::block("window_test_sink", gr::io_signature::make(1, 1, sizeof(int)), gr::io_signature::make(0, 0, 0)), window_(window)
    {
    }
    void forecast(int /*noutput_items*/, gr_vector_int& required) override { required[0] = window_; }
    int general_work(int /*noutput_items*/, gr_vector_int& available, gr_vector_const_void_star& input, gr_vector_void_star& /*output_items*/) override
    {
        const auto* in = static_cast<const int*>(input[0]);
        const int count = std::min(window_, available[0]);
        for (int i = 0; i < count; ++i)
            {
                valid_ = valid_ && in[i] == received_ + i;
            }
        received_ += count;
        consume_each(count);
        return 0;
    }
    int received_ = 0;
    bool valid_ = true;

private:
    int window_;
};
}  // namespace

class StreamBatcherTest : public ::testing::Test
{
protected:
    bool run(const gr::top_block_sptr& top)
    {
        top->start();
        auto done = std::async(std::launch::async, [top]() { top->wait(); });
        const bool completed = done.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
        if (!completed)
            {
                top->stop();
            }
        done.get();
        return completed;
    }
    void check(int length, int chunk, int batch, bool paced = false)
    {
        auto top = gr::make_top_block("batch_test");
        auto source = gnss_make_shared<ChunkedTestSource>(length, chunk, paced);
        auto batcher = make_stream_batcher(sizeof(int), batch, 0);
        auto sink = gr::blocks::vector_sink_i::make();
        top->connect(source, 0, batcher, 0);
        top->connect(batcher, 0, sink, 0);
        ASSERT_TRUE(run(top));
        std::vector<int> expected(length);
        std::iota(expected.begin(), expected.end(), 0);
        EXPECT_EQ(expected, sink->data());
        auto tags = sink->tags();
        const size_t expected_tags = length == 0 ? 0 : (length == 1 ? 1 : (length == 2 ? 2 : 3));
        ASSERT_EQ(expected_tags, tags.size());
        for (const auto& tag : tags)
            {
                EXPECT_EQ(tag.offset, static_cast<uint64_t>(pmt::to_long(tag.value)));
                EXPECT_TRUE(pmt::eq(tag.key, pmt::intern("sample")));
            }
    }
};


TEST_F(StreamBatcherTest, FiniteTailsAndTags)
{
    for (int length : {0, 1, 2, 1023, 1024, 1025, 4097, 65537})
        {
            for (int chunk : {1, 73, 8192})
                {
                    SCOPED_TRACE(::testing::Message() << "length=" << length << " chunk=" << chunk);
                    check(length, chunk, 1024);
                }
        }
}


TEST_F(StreamBatcherTest, StagedTailAfterUpstreamFinishes)
{
    for (int i = 0; i < 10; ++i)
        {
            check(73, 1, 1024, true);
        }
}


TEST_F(StreamBatcherTest, ConsumerWindowLargerThanBatch)
{
    auto top = gr::make_top_block("batch_backpressure");
    auto source = gnss_make_shared<ChunkedTestSource>(3500 * 8, 79, false);
    auto batcher = make_stream_batcher(sizeof(int), 1024, 4096);
    auto sink = gnss_make_shared<WindowTestSink>(3500);
    top->connect(source, 0, batcher, 0);
    top->connect(batcher, 0, sink, 0);
    ASSERT_TRUE(run(top));
    EXPECT_EQ(3500 * 8, sink->received_);
    EXPECT_TRUE(sink->valid_);
}


TEST_F(StreamBatcherTest, ManyReadersAndSmallWorkAllowance)
{
    auto top = gr::make_top_block("batch_fanout");
    auto source = gnss_make_shared<ChunkedTestSource>(65537, 113, false);
    auto batcher = make_stream_batcher(sizeof(int), 1024, 4096);
    batcher->set_max_noutput_items(97);
    top->connect(source, 0, batcher, 0);
    std::vector<gr::blocks::vector_sink_i::sptr> sinks;
    for (int i = 0; i < 40; ++i)
        {
            auto sink = gr::blocks::vector_sink_i::make();
            top->connect(batcher, 0, sink, 0);
            sinks.push_back(sink);
        }
    ASSERT_TRUE(run(top));
    std::vector<int> expected(65537);
    std::iota(expected.begin(), expected.end(), 0);
    for (const auto& sink : sinks)
        {
            EXPECT_EQ(expected, sink->data());
            EXPECT_EQ(3U, sink->tags().size());
        }
}


TEST_F(StreamBatcherTest, ComplexSamples)
{
    std::vector<gr_complex> samples(4097, gr_complex(0.125F, -0.5F));
    auto top = gr::make_top_block("batch_complex");
    auto source = gr::blocks::vector_source_c::make(samples, false);
    auto sink = gr::blocks::vector_sink_c::make();
    auto batcher = make_stream_batcher(sizeof(gr_complex), 1024, 0);
    top->connect(source, 0, batcher, 0);
    top->connect(batcher, 0, sink, 0);
    ASSERT_TRUE(run(top));
    EXPECT_EQ(samples, sink->data());
}


TEST_F(StreamBatcherTest, RejectsInvalidSizes)
{
    EXPECT_THROW(make_stream_batcher(0, 1024, 0), std::invalid_argument);
    EXPECT_THROW(make_stream_batcher(std::numeric_limits<size_t>::max(), 1024, 0), std::invalid_argument);
    EXPECT_THROW(make_stream_batcher(sizeof(int), 0, 0), std::invalid_argument);
    EXPECT_THROW(make_stream_batcher(sizeof(int), std::numeric_limits<uint64_t>::max(), 0), std::invalid_argument);
    EXPECT_THROW(make_stream_batcher(sizeof(int), 1024, 1), std::invalid_argument);
    EXPECT_THROW(make_stream_batcher(sizeof(int), 1024, std::numeric_limits<uint64_t>::max()), std::invalid_argument);
    auto batcher = make_stream_batcher(sizeof(int), 1024, 512);
    EXPECT_EQ(256U, batcher->batch_items());
    EXPECT_EQ(1, batcher->output_multiple());
}


TEST_F(StreamBatcherTest, StopDuringAccumulation)
{
    auto top = gr::make_top_block("batch_cancel");
    auto source = gnss_make_shared<ChunkedTestSource>(1000000, 1, true);
    auto batcher = make_stream_batcher(sizeof(int), 100000, 0);
    auto sink = gr::blocks::vector_sink_i::make();
    top->connect(source, 0, batcher, 0);
    top->connect(batcher, 0, sink, 0);
    top->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    top->stop();
    auto done = std::async(std::launch::async, [top]() { top->wait(); });
    EXPECT_EQ(std::future_status::ready, done.wait_for(std::chrono::seconds(2)));
    done.get();
}


TEST_F(StreamBatcherTest, SmallBuffers)
{
    for (uint64_t capacity : {2, 128, 512, 4096})
        {
            auto top = gr::make_top_block("batch_small_buffer");
            auto source = gnss_make_shared<ChunkedTestSource>(4097, 13, false);
            auto batcher = make_stream_batcher(sizeof(int), 1024, capacity);
            auto sink = gr::blocks::vector_sink_i::make();
            top->connect(source, 0, batcher, 0);
            top->connect(batcher, 0, sink, 0);
            ASSERT_TRUE(run(top));
            std::vector<int> expected(4097);
            std::iota(expected.begin(), expected.end(), 0);
            EXPECT_EQ(expected, sink->data());
        }
}


TEST_F(StreamBatcherTest, ByteSamplesAndDuplicateTags)
{
    std::vector<uint8_t> samples(1025);
    std::iota(samples.begin(), samples.end(), uint8_t(0));
    std::vector<gr::tag_t> tags(2);
    for (auto& tag : tags)
        {
            tag.offset = 1024;
            tag.key = pmt::intern("tail");
            tag.value = pmt::from_long(1024);
            tag.srcid = pmt::intern("test_source");
        }
    auto top = gr::make_top_block("batch_bytes");
    auto source = gr::blocks::vector_source_b::make(samples, false, 1, tags);
    auto sink = gr::blocks::vector_sink_b::make();
    auto batcher = make_stream_batcher(1, 1024, 0);
    top->connect(source, 0, batcher, 0);
    top->connect(batcher, 0, sink, 0);
    ASSERT_TRUE(run(top));
    EXPECT_EQ(samples, sink->data());
    ASSERT_EQ(2U, sink->tags().size());
    for (const auto& tag : sink->tags())
        {
            EXPECT_EQ(1024U, tag.offset);
            EXPECT_TRUE(pmt::eq(tag.srcid, tags[0].srcid));
        }
}


TEST_F(StreamBatcherTest, AllConditionerStageCombinationsPreserveInversion)
{
    for (int mask = 0; mask < 8; ++mask)
        {
            InMemoryConfiguration config;
            config.set_property("SignalConditioner.implementation", "Signal_Conditioner");
            int inversions = 0;
            int bit = 0;
            for (const auto& role : {"DataTypeAdapter", "InputFilter", "Resampler"})
                {
                    const bool invert = (mask & (1 << bit++)) != 0;
                    inversions += invert ? 1 : 0;
                    config.set_property(std::string(role) + ".implementation", "Pass_Through");
                    config.set_property(std::string(role) + ".inverted_spectrum", invert ? "true" : "false");
                }
            auto conditioner = block_factory::GetSignalConditioner(&config);
            auto* adapter = dynamic_cast<SignalConditioner*>(conditioner.get());
            ASSERT_NE(nullptr, adapter);
            adapter->set_remove_identity(true);
            auto top = gr::make_top_block("conditioner_chain_test");
            std::vector<gr_complex> samples(1025, gr_complex(0.25F, -0.75F));
            auto source = gr::blocks::vector_source_c::make(samples, false);
            auto sink = gr::blocks::vector_sink_c::make();
            conditioner->connect(top);
            conditioner->connect(top);
            if (conditioner->is_identity())
                {
                    EXPECT_EQ(0, mask);
                    top->connect(source, 0, sink, 0);
                }
            else
                {
                    top->connect(source, 0, conditioner->get_left_block(), 0);
                    top->connect(conditioner->get_right_block(), 0, sink, 0);
                }
            ASSERT_TRUE(run(top));
            if (inversions % 2 != 0)
                {
                    std::fill(samples.begin(), samples.end(), gr_complex(0.25F, 0.75F));
                }
            EXPECT_EQ(samples, sink->data()) << mask;
            conditioner->disconnect(top);
            conditioner->disconnect(top);
        }
}


TEST_F(StreamBatcherTest, RestartAfterPartialBatch)
{
    auto top = gr::make_top_block("batch_restart");
    auto source = gnss_make_shared<ChunkedTestSource>(2000, 1, true);
    // The deadline must outlast the first, interrupted run.
    auto batcher = make_stream_batcher(sizeof(int), 1024, 0, 20.0);
    auto sink = gr::blocks::vector_sink_i::make();
    top->connect(source, 0, batcher, 0);
    top->connect(batcher, 0, sink, 0);
    top->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    top->stop();
    top->wait();
    ASSERT_TRUE(run(top));
    const auto data = sink->data();
    ASSERT_FALSE(data.empty());
    for (size_t i = 1; i < data.size(); ++i)
        {
            EXPECT_EQ(data[i - 1] + 1, data[i]);
        }
    EXPECT_EQ(0, data.front());
    EXPECT_EQ(1999, data.back());
    EXPECT_EQ(2000U, data.size());
}


TEST_F(StreamBatcherTest, ConversionRemainsWhenCopiesAreRemoved)
{
    InMemoryConfiguration config;
    config.set_property("SignalConditioner.implementation", "Signal_Conditioner");
    config.set_property("DataTypeAdapter.implementation", "Ishort_To_Complex");
    config.set_property("DataTypeAdapter.inverted_spectrum", "true");
    auto conditioner = block_factory::GetSignalConditioner(&config);
    auto* adapter = dynamic_cast<SignalConditioner*>(conditioner.get());
    ASSERT_NE(nullptr, adapter);
    adapter->set_remove_identity(true);
    ASSERT_FALSE(conditioner->is_identity());
    std::vector<int16_t> input(2050);
    for (size_t i = 0; i < input.size(); i += 2)
        {
            input[i] = 100;
            input[i + 1] = -200;
        }
    auto top = gr::make_top_block("conditioner_conversion");
    auto source = gr::blocks::vector_source_s::make(input, false);
    auto sink = gr::blocks::vector_sink_c::make();
    auto batcher = make_stream_batcher(sizeof(gr_complex), 1024, 0);
    conditioner->connect(top);
    top->connect(source, 0, conditioner->get_left_block(), 0);
    top->connect(conditioner->get_right_block(), 0, batcher, 0);
    top->connect(batcher, 0, sink, 0);
    ASSERT_TRUE(run(top));
    EXPECT_EQ(std::vector<gr_complex>(1025, gr_complex(100.0F, 200.0F)), sink->data());
}


TEST_F(StreamBatcherTest, DeadlinePublishesWithoutFurtherInputOrEof)
{
    auto service = std::make_shared<StreamBatcherDeadline>();
    auto top = gr::make_top_block("deadline_streams");
    std::vector<gnss_shared_ptr<ObservedTestSink>> sinks;
    for (int i = 0; i < 8; ++i)
        {
            auto source = gnss_make_shared<StalledTestSource>();
            auto batcher = make_stream_batcher(sizeof(int), 1024, 0, 5.0, service);
            batcher->set_max_noutput_items(3);
            auto sink = gnss_make_shared<ObservedTestSink>();
            top->connect(source, 0, batcher, 0);
            top->connect(batcher, 0, sink, 0);
            sinks.push_back(sink);
        }
    top->start();
    const auto limit = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < limit &&
           std::any_of(sinks.begin(), sinks.end(), [](const gnss_shared_ptr<ObservedTestSink>& sink) { return sink->received.load() != 17; }))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    top->stop();
    top->wait();
    std::vector<int> expected(17);
    std::iota(expected.begin(), expected.end(), 0);
    for (const auto& sink : sinks)
        {
            EXPECT_EQ(expected, sink->samples);
            ASSERT_EQ(1U, sink->tags.size());
            EXPECT_EQ(16U, sink->tags[0].offset);
            EXPECT_EQ(42, pmt::to_long(sink->tags[0].value));
            EXPECT_TRUE(pmt::eq(pmt::mp("source"), sink->tags[0].srcid));
        }
}


TEST_F(StreamBatcherTest, DeadlineCancellationAndDestruction)
{
    auto service = std::make_shared<StreamBatcherDeadline>();
    for (int i = 0; i < 25; ++i)
        {
            auto top = gr::make_top_block("deadline_cancel");
            auto source = gnss_make_shared<StalledTestSource>();
            auto batcher = make_stream_batcher(sizeof(int), 1024, 0, 1.0, service);
            auto sink = gnss_make_shared<ObservedTestSink>();
            top->connect(source, 0, batcher, 0);
            top->connect(batcher, 0, sink, 0);
            top->start();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            top->stop();
            top->wait();
        }
    // The shared timer remains alive after all registrations are removed.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}


TEST_F(StreamBatcherTest, RejectsInvalidDeadline)
{
    EXPECT_NO_THROW(make_stream_batcher(sizeof(int), 1024, 0, 20.0));
    for (double delay : {0.0, -1.0, 20.001, 40.0, 511.0, 60000.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        {
            EXPECT_THROW(make_stream_batcher(sizeof(int), 1024, 0, delay), std::invalid_argument);
        }
}


TEST_F(StreamBatcherTest, DeadlineSurvivesRepeatedSchedulerRestarts)
{
    auto top = gr::make_top_block("deadline_restart");
    auto source = gnss_make_shared<StalledTestSource>();
    auto batcher = make_stream_batcher(sizeof(int), 1024, 0, 2.0);
    auto sink = gnss_make_shared<ObservedTestSink>();
    top->connect(source, 0, batcher, 0);
    top->connect(batcher, 0, sink, 0);
    for (int run = 1; run <= 10; ++run)
        {
            top->start();
            const auto limit = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (sink->received.load() < run * 17 && std::chrono::steady_clock::now() < limit)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
            top->stop();
            top->wait();
            ASSERT_EQ(run * 17, sink->received.load());
        }
    for (size_t i = 0; i < sink->samples.size(); ++i)
        {
            EXPECT_EQ(static_cast<int>(i % 17), sink->samples[i]);
        }
}
