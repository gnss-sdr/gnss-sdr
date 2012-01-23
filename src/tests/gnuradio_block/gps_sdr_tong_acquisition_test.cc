
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
#include <gr_file_source.h>
#include <gr_null_sink.h>
#include <gps_sdr_tong_acquisition_ss.h>
#include <gr_stream_to_vector.h>
#include <gr_msg_queue.h>
#include <gr_complex_to_interleaved_short.h>
#include <gr_multiply_const_cc.h>

#include "gnss_sdr_direct_resampler_ccf.h"

DEFINE_string(signal_file, "signal_samples/signal.dat",
		"Path to the file containing the signal samples");

DEFINE_int64(fs_in, 2048000, "FS of the signal in Hz");
DEFINE_int64(if_freq, 0, "Intermediate frequency");
DEFINE_int32(satellite, 0, "Satellite number");
DEFINE_bool(dump, false, "If true, acquisition result will be dumped in a file");
DEFINE_int32(ms, 1, "ms of signal to be used in the acquisition process");
DEFINE_int32(shift_resolution, 15, "shift resolution for acquisition");
DEFINE_int32(A_value, 8, "Value of the k variable when the acquisition_done is true");
DEFINE_int64(acquisition_threshold, 2000000, "threshold of the acquisition algorithm");
//DEFINE_int32(max_dwells, 30, "number of dwells in Tong acquisition algorithm");
DEFINE_int32(B_value, 2, "Value of the initial K variable in Tong algorithm");

int main(int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    gr_msg_queue_sptr queue = gr_msg_queue_sptr();

    int samples_per_ms = ceil(FLAGS_fs_in/1000);
    gr_complex div = 0.001;

    LOG_AT_LEVEL(INFO) << "fs_in " << FLAGS_fs_in;
    LOG_AT_LEVEL(INFO) << "if_freq " << FLAGS_if_freq;
    LOG_AT_LEVEL(INFO) << "satellite " << FLAGS_satellite;
    //	LOG_AT_LEVEL(INFO) << "max_dwells " << FLAGS_max_dwells;
    LOG_AT_LEVEL(INFO) << "threshold " << FLAGS_acquisition_threshold;
    LOG_AT_LEVEL(INFO) << "A_value " << FLAGS_A_value;
    LOG_AT_LEVEL(INFO) << "B_value " << FLAGS_B_value;


    gr_top_block_sptr top_block = gr_make_top_block("gps_sdr_tong_acquisition_test");
    gr_block_sptr source = gr_make_file_source(sizeof(gr_complex),FLAGS_signal_file.c_str());
    gr_multiply_const_cc_sptr divisor = gr_make_multiply_const_cc(div);
    gr_block_sptr complex_to_interleaved_short = gr_make_complex_to_interleaved_short();

    gr_block_sptr stream_to_vector = gr_make_stream_to_vector(sizeof(short), samples_per_ms);
    gr_block_sptr acquisition = gps_sdr_make_tong_acquisition_ss(FLAGS_satellite, FLAGS_ms, FLAGS_shift_resolution, FLAGS_if_freq, FLAGS_fs_in, samples_per_ms, queue, FLAGS_dump, FLAGS_B_value, FLAGS_acquisition_threshold, FLAGS_A_value);
    gr_block_sptr null_sink = gr_make_null_sink(sizeof(short)*samples_per_ms);

    top_block->connect(source, 0, divisor, 0);
    top_block->connect(divisor, 0, complex_to_interleaved_short, 0);
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
