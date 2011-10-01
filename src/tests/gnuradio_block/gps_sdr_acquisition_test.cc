
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * Executes a gps sdr acquisition based on some input parameters.
 *
 */

#include <iostream>
#include <sys/time.h>

#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

#include <gnuradio/gr_top_block.h>
#include <gr_file_source.h>
#include <gr_null_sink.h>
#include <gps_sdr_acquisition_ss.h>
#include <gr_stream_to_vector.h>
#include <gr_msg_queue.h>
#include <gr_complex_to_interleaved_short.h>

#include "gnss_sdr_direct_resampler_ccf.h"

DEFINE_string(signal_file, "signal_samples/signal.dat",
		"Path to the file containing the signal samples");

DEFINE_int64(fs_in, 2048000, "FS of the signal in Hz");
DEFINE_int64(if_freq, 0, "Intermediate frequency");
DEFINE_int32(satellite, 0, "Satellite number");
DEFINE_bool(dump, false, "If true, acquisition result will be dumped in a file");
DEFINE_int32(ms, 1, "ms of signal to be used in the acquisition process");
DEFINE_int32(shift_resolution, 15, "shift resolution for acquisition");

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	int samples_per_ms = ceil(FLAGS_fs_in/1000);

	LOG_AT_LEVEL(INFO) << "fs_in " << FLAGS_fs_in;
	LOG_AT_LEVEL(INFO) << "if_freq " << FLAGS_if_freq;
	LOG_AT_LEVEL(INFO) << "satellite " << FLAGS_satellite;

    gr_top_block_sptr top_block = gr_make_top_block("gps_sdr_acquisition_test");
    gr_block_sptr source = gr_make_file_source(sizeof(gr_complex),FLAGS_signal_file.c_str());
    gr_block_sptr complex_to_interleaved_short = gr_make_complex_to_interleaved_short();

    gr_block_sptr stream_to_vector = gr_make_stream_to_vector(sizeof(short), samples_per_ms);
    gr_block_sptr acquisition = gps_sdr_make_acquisition_ss(FLAGS_satellite, FLAGS_ms, FLAGS_shift_resolution, FLAGS_if_freq, FLAGS_fs_in, samples_per_ms, queue, FLAGS_dump);
    gr_block_sptr null_sink = gr_make_null_sink(sizeof(short)*samples_per_ms);

    top_block->connect(source, 0, complex_to_interleaved_short, 0);
    top_block->connect(complex_to_interleaved_short, 0, stream_to_vector, 0);
    top_block->connect(stream_to_vector, 0 , acquisition, 0);
    top_block->connect(acquisition, 0, null_sink, 0);

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
