
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * This class implements unit tests for a the valve custom block.
 *
 */

#include <gr_top_block.h>
#include <gr_sig_source_f.h>
#include <gr_null_sink.h>
#include <gr_msg_queue.h>

#include <gtest/gtest.h>

#include "gnss_sdr_valve.h"

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

TEST(GNSS_SDR_VALVE, CheckEventSentAfter100Samples) {

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	gr_top_block_sptr top_block = gr_make_top_block("gnss_sdr_valve_test");
	gr_block_sptr valve = gnss_sdr_make_valve(sizeof(float), 100, queue);
	gr_sig_source_f_sptr source = gr_make_sig_source_f(100, GR_CONST_WAVE, 100, 1, 0);
	gr_block_sptr sink = gr_make_null_sink(sizeof(float));

	LOG_AT_LEVEL(INFO)  << "Queue count is " << queue->count();
	EXPECT_EQ(0, queue->count());

	top_block->connect(source, 0, valve, 0);
	top_block->connect(valve, 0, sink, 0);

	top_block->run();
	top_block->stop();

	LOG_AT_LEVEL(INFO)  << "Queue count is " << queue->count();
	EXPECT_EQ(1, queue->count());

}