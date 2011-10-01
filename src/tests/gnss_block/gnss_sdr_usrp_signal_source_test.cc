
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

#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"

using google::LogMessage;

DEFINE_string(d, "16", "Decimation");
DEFINE_string(f, "1.57542e9", "Carrier frequency of the signal");
DEFINE_string(g, "40", "Gain in dB");
DEFINE_string(s, "4000000", "Samples to be recorded");
DEFINE_string(dump_filename, "./data/usrp_test.dat", "Dump filename");
DEFINE_string(item_type, "short", "Data type for samples");

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	DLOG(INFO) << "samples " << FLAGS_s;
	DLOG(INFO) << "f " << FLAGS_f;
	DLOG(INFO) << "g " << FLAGS_g;
	DLOG(INFO) << "item_type " << FLAGS_item_type;

	gr_top_block_sptr top_block = gr_make_top_block("usrp signal source test");

	GNSSBlockFactory* factory = new GNSSBlockFactory();
	InMemoryConfiguration* config = new InMemoryConfiguration();

	config->set_property("SignalSource.implementation", "USRPSignalSource");
	config->set_property("SignalSource.freq", FLAGS_f.c_str());
	config->set_property("SignalSource.gain", FLAGS_g.c_str());
	config->set_property("SignalSource.samples", FLAGS_s.c_str());
	config->set_property("SignalSource.item_type", FLAGS_item_type.c_str());
	config->set_property("SignalSource.dump", "true");
	config->set_property("SignalSource.dump_filename", FLAGS_dump_filename);

	GNSSBlockInterface* gnss_block = factory->GetSignalSource(config, queue);
	gnss_block->connect(top_block);

	size_t item_size;

	if(FLAGS_item_type.compare("short") == 0) {
		item_size = sizeof(short);
	} else {
		item_size = sizeof(gr_complex);
	}

	gr_block_sptr null_sink = gr_make_null_sink(item_size);

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
