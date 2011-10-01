
#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

#include <gr_sig_source_c.h>
#include <gr_head.h>
#include <gr_file_sink.h>
#include <gr_top_block.h>

using google::LogMessage;

DEFINE_string(filename, "./signal_samples/saw_signal.dat", "Path to the file where the prn code will be stored.");
DEFINE_int32(fs, 4000000, "FS of the resulting code.");
DEFINE_int32(samples, 16000, "Number of samples to be generated");
DEFINE_int32(f, 250, "Frequency of the signal");
DEFINE_int32(a, 16000, "Amplitude of the signal");

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

	DLOG(INFO) << "fs " << FLAGS_fs;
	DLOG(INFO) << "filename " << FLAGS_filename;
	DLOG(INFO) << "samples " << FLAGS_samples;

	gr_block_sptr source = gr_make_sig_source_c(FLAGS_fs, GR_SAW_WAVE, FLAGS_f, FLAGS_a, 0);
	gr_block_sptr file_sink = gr_make_file_sink(sizeof(gr_complex), FLAGS_filename.c_str());
	gr_block_sptr valve = gr_make_head(sizeof(gr_complex), FLAGS_samples);

	gr_top_block_sptr top_block = gr_make_top_block("saw signal generator");

	top_block->connect(source, 0, valve, 0);
	top_block->connect(valve, 0, file_sink, 0);

	top_block->run();

	DLOG(INFO) << "Finished";
}
