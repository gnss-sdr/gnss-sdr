
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
#include <gr_firdes.h>

#include "gr_pfb_arb_resampler_ccf.h"

DEFINE_string(signal_file, "signal_samples/signal.dat",
		"Path to the file containing the signal samples");

DEFINE_int64(fs_in, 8000000, "FS of the signal in Hz");
DEFINE_int64(fs_out, 2048000, "FS of the resampled signal in Hz");
DEFINE_int64(trans_band, 100000, "Transition band of the filter");

int mcd(int a, int b){
	if (a==0) return b;
	return mcd(b%a,a);
}

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);

	double cutoff_freq;
	int div = mcd(FLAGS_fs_in,FLAGS_fs_out);
	unsigned int interpolation = FLAGS_fs_out/div;
	unsigned int decimation = FLAGS_fs_in/div;
	double sampling_freq = FLAGS_fs_in*interpolation;
	float rate = FLAGS_fs_in/FLAGS_fs_out;
	double gain = (double) interpolation;

	if(interpolation > decimation) {
		cutoff_freq = FLAGS_fs_in/2;
	} else {
		cutoff_freq = (interpolation*FLAGS_fs_in)/(decimation*2);
	}

	LOG_AT_LEVEL(INFO) << "fs_in " << FLAGS_fs_in;
	LOG_AT_LEVEL(INFO) << "fs_out " << FLAGS_fs_out;
	LOG_AT_LEVEL(INFO) << "signal file " << FLAGS_signal_file;
	LOG_AT_LEVEL(INFO) << "transition band " << FLAGS_trans_band;
	LOG_AT_LEVEL(INFO) << "Interpolation " << interpolation;
	LOG_AT_LEVEL(INFO) << "Decimation " << decimation;
	LOG_AT_LEVEL(INFO) << "LPF cut-off frequency " << cutoff_freq;
	LOG_AT_LEVEL(INFO) << "Sampling frequency " << sampling_freq;

	gr_top_block_sptr top_block = gr_make_top_block("gr_pfb_arb_resampler_test");

	gr_file_source_sptr source = gr_make_file_source(sizeof(gr_complex),FLAGS_signal_file.c_str());

	std::vector<float> low_pass_taps =
		gr_firdes::low_pass(gain, sampling_freq, cutoff_freq, FLAGS_trans_band, (gr_firdes::win_type)1);

	LOG_AT_LEVEL(INFO) << "Taps count " << low_pass_taps.size();

	gr_pfb_arb_resampler_ccf_sptr resampler =
		gr_make_pfb_arb_resampler_ccf(rate, low_pass_taps, interpolation);
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
