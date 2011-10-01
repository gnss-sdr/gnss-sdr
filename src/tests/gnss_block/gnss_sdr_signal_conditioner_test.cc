
#include <gflags/gflags.h>

#include <iostream>
#include <sys/time.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

#include <gr_sig_source_c.h>
#include <gr_head.h>
#include <gr_null_sink.h>
#include <gr_top_block.h>
#include <gr_msg_queue.h>
#include <gr_file_source.h>

#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"

using google::LogMessage;

DEFINE_string(fs_in, "4000000", "FS of the input signal");
DEFINE_string(fs_out, "2048000", "FS of the output signal");
DEFINE_string(dump_filename, "./data/signal_conditioner_test.dat", "Dump filename");
DEFINE_string(signal_filename, "./signal_samples/usrp_30.dat", "Name of the file containing the signal samples");
DEFINE_string(item_type, "short", "Type of data for samples");

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	DLOG(INFO) << "fs_in " << FLAGS_fs_in;
	DLOG(INFO) << "fs_out " << FLAGS_fs_out;

	gr_top_block_sptr top_block = gr_make_top_block("signal conditioner test");

	GNSSBlockFactory* factory = new GNSSBlockFactory();
	InMemoryConfiguration* config = new InMemoryConfiguration();

	config->set_property("SignalConditioner.implementation", "DirectResampler");
	config->set_property("SignalConditioner.sample_freq_in", FLAGS_fs_in.c_str());
	config->set_property("SignalConditioner.sample_freq_out", FLAGS_fs_out.c_str());
	config->set_property("SignalConditioner.dump", "true");
	config->set_property("SignalConditioner.dump_filename", FLAGS_dump_filename);
	config->set_property("SignalConditioner.item_type", FLAGS_item_type);

	GNSSBlockInterface* gnss_block = factory->GetSignalConditioner(config, queue);
	gnss_block->connect(top_block);

	size_t item_size;

	if(FLAGS_item_type.compare("short") == 0) {
		item_size = sizeof(short);
	} else if(FLAGS_item_type.compare("gr_complex") == 0) {
		item_size = sizeof(gr_complex);
	} else {
		item_size = sizeof(short);
	}

	gr_block_sptr null_sink = gr_make_null_sink(item_size);
	gr_block_sptr file_source = gr_make_file_source(item_size, FLAGS_signal_filename.c_str());

	top_block->connect(file_source, 0, gnss_block->get_left_block(), 0);
	top_block->connect(gnss_block->get_right_block(), 0, null_sink, 0);

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
