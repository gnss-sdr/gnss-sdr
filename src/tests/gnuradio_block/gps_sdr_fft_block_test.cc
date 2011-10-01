
/**
 * Copyright notice
 */

/**
 * Author: Luis Esteve, 2010.
 */

/**
 * Executes the Tong algorithm for a  gps sdr acquisition based on some input parameters.
 *
 */

#include <iostream>
#include <sys/time.h>

#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

#include <gr_top_block.h>
#include <gr_sig_source_c.h>
#include <gr_file_sink.h>
#include <gr_stream_to_vector.h>
#include <gr_complex_to_interleaved_short.h>
#include <gr_head.h>

#include "gps_sdr_fft_block.h"
#include "gps_sdr_signal_processing.h"

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

    gr_top_block_sptr top_block = gr_make_top_block("gps sdr fft block test");
    gr_block_sptr source = gr_make_sig_source_c(2048, GR_SIN_WAVE, 1, 32767, 0);
    gr_block_sptr complex2short = gr_make_complex_to_interleaved_short();
    gr_block_sptr fft = gps_sdr_make_fft();
    gr_block_sptr sink = gr_make_file_sink(sizeof(short)*2*2048, "./data/gps_sdr_fft_block_test.dat");
    gr_block_sptr str2vec = gr_make_stream_to_vector(sizeof(short), 2048*2);
    gr_block_sptr head = gr_make_head(sizeof(gr_complex), 2048);

    top_block->connect(source, 0, head, 0);
    top_block->connect(head, 0, complex2short, 0);
    top_block->connect(complex2short, 0, str2vec, 0);
    top_block->connect(str2vec, 0, fft, 0);
    top_block->connect(fft, 0, sink, 0);

	LOG_AT_LEVEL(INFO) << "Run";
	struct timeval tv;
	gettimeofday(&tv, NULL);
	long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;
	top_block->run(); // Start threads and wait
	gettimeofday(&tv, NULL);
	long long int end = tv.tv_sec *1000000 + tv.tv_usec;
	LOG_AT_LEVEL(INFO) << "Finished in " << (end - begin) << " microseconds";
	std::cout << (end - begin) << std::endl;
	top_block->stop();
}
