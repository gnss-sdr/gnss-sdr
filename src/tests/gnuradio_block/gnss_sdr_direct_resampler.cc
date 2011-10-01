
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * Executes a resampler based on some input parameters.
 * This is intended to check performance of resampling implementations.
 *
 */

#include <iostream>
#include <sys/time.h>

#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

#include <gr_top_block.h>
#include <gr_file_source.h>
#include <gr_null_sink.h>
#include <gr_rational_resampler_base_ccf.h>
#include <gr_firdes.h>

#include "gnss_sdr_direct_resampler_ccf.h"

DEFINE_string(signal_file, "signal_samples/signal.dat",
		"Path to the file containing the signal samples");

DEFINE_int64(fs_in, 8000000, "FS of the signal in Hz");
DEFINE_int64(fs_out, 2048000, "FS of the resampled signal in Hz");

int mcd(int a, int b){
	if (a==0) return b;
	return mcd(b%a,a);
}

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);

	LOG_AT_LEVEL(INFO) << "Frequency IN " << FLAGS_fs_in;
	LOG_AT_LEVEL(INFO) << "Frequency OUT " << FLAGS_fs_out;

    gr_top_block_sptr top_block = gr_make_top_block("gnss_sdr_direct_resampler_test");
    gr_file_source_sptr source = gr_make_file_source(sizeof(gr_complex),FLAGS_signal_file.c_str());

    gnss_sdr_direct_resampler_ccf_sptr resampler = gnss_sdr_make_direct_resampler_ccf(FLAGS_fs_in, FLAGS_fs_out);
    gr_block_sptr sink = gr_make_null_sink(sizeof(gr_complex));

    top_block->connect(source, 0, resampler, 0);
    top_block->connect(resampler, 0, sink, 0);

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
