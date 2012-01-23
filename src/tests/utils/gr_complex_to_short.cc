
#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>
#include "control_message_factory.h"
#include <gr_complex_to_interleaved_short.h>
#include <gnuradio/gr_file_source.h>
#include <gnuradio/gr_file_sink.h>
#include <gr_top_block.h>
#include <gnuradio/gr_io_signature.h>
#include <gnuradio/gr_sync_block.h>


using google::LogMessage;

DEFINE_string(in_file, "", "Path to the file containing the samples in gr_complex");
DEFINE_string(out_file, "", "Path to the file containing the samples in short");

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

	DLOG(INFO) << "in_file " << FLAGS_in_file;
	DLOG(INFO) << "out_file " << FLAGS_out_file;

	gr_block_sptr file_source = gr_make_file_source(sizeof(gr_complex), FLAGS_in_file.c_str(), false);
	gr_block_sptr file_sink = gr_make_file_sink(sizeof(short), FLAGS_out_file.c_str());
	gr_block_sptr converter = gr_make_complex_to_interleaved_short();

	gr_top_block_sptr top_block = gr_make_top_block("complex to short");

	top_block->connect(file_source, 0, converter, 0);
	top_block->connect(converter, 0, file_sink, 0);

	top_block->run();

	DLOG(INFO) << "Finished";
}
