
#include <gflags/gflags.h>

#include <iostream>
#include <sys/time.h>
#include <string>

#include <glog/log_severity.h>
#include <glog/logging.h>

#include <gnuradio/gr_sig_source_s.h>
#include <gnuradio/gr_sig_source_c.h>
#include <gnuradio/gr_head.h>
#include <gnuradio/gr_null_sink.h>
#include <gnuradio/gr_top_block.h>
#include <gnuradio/gr_msg_queue.h>
#include <gnuradio/gr_file_source.h>
#include <gnuradio/gr_file_sink.h>
#include <gnuradio/gr_multiply_const_ss.h>
#include <gnuradio/gr_complex_to_xxx.h>
#include <gnuradio/gr_complex_to_interleaved_short.h>

#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "acquisition_interface.h"

using google::LogMessage;

DEFINE_bool(usrp, false, "If true, the USRPSignalSource will be used");
DEFINE_string(usrp_gain, "40", "Gain");
DEFINE_int32(satellite, 0, "Satellite to be acquired");
DEFINE_string(fs_in, "2048000", "FS of the input signal");
DEFINE_string(ifreq, "0", "Intermediate frequency");
DEFINE_string(doppler_max, "15", "Doppler max");
DEFINE_string(sampled_ms, "1", "ms used for acquisition");
DEFINE_string(signal_filename, "./signal_samples/conditioner_30.dat", "Name of the file with the signal samples");
DEFINE_string(item_type, "short", "Type of data for samples");
DEFINE_string(dump, "true", "If true, magnitudes are written to a file");

int main(int argc, char** argv) {

	//printf("Runlevel 1 \r\n");
	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

	DLOG(INFO) << "fs_in " << FLAGS_fs_in;
	DLOG(INFO) << "ifreq " << FLAGS_ifreq;
	DLOG(INFO) << "doppler_max " << FLAGS_doppler_max;
	DLOG(INFO) << "sampled ms " << FLAGS_sampled_ms;
	DLOG(INFO) << "signal filename " << FLAGS_signal_filename;
	DLOG(INFO) << "satellite " << FLAGS_satellite;
	DLOG(INFO) << "dump " << FLAGS_dump;

	gr_top_block_sptr top_block = gr_make_top_block("acquisition test");

	size_t item_size;

	if(FLAGS_item_type.compare("short") == 0) {
		item_size = sizeof(short);
	} else if(FLAGS_item_type.compare("gr_complex") == 0) {
		item_size = sizeof(gr_complex);
	} else {
		item_size = sizeof(short);
	}

	// Build Acquisition GNSS block
	GNSSBlockFactory* factory = new GNSSBlockFactory();
	InMemoryConfiguration* config = new InMemoryConfiguration();

	//config->set_property("Acquisition.implementation", "GpsSdrAcquisition");
	config->set_property("Acquisition.implementation", "gnss_acquisition_a");
	config->set_property("Acquisition.fs_in", FLAGS_fs_in.c_str());
	config->set_property("Acquisition.ifreq", FLAGS_ifreq.c_str());
	config->set_property("Acquisition.doppler_max", FLAGS_doppler_max);
	config->set_property("Acquisition.sampled_ms", FLAGS_sampled_ms);
	config->set_property("Acquisition.dump", FLAGS_dump);
	config->set_property("Acquisition.item_type", FLAGS_item_type);

	//GNSSBlockInterface* acquisition_gnss_block = factory->GetBlock(config, "Acquisition", "GpsSdrAcquisition", 1, 1, gr_msg_queue_sptr());
	GNSSBlockInterface* acquisition_gnss_block = factory->GetBlock(config, "Acquisition", "gnss_acquisition_a", 1, 1, gr_msg_queue_sptr());
	acquisition_gnss_block->connect(top_block);


	if(FLAGS_usrp) {

		// Build USRP GNSS block
		config->set_property("SignalSource.implementation", "USRPSignalSource");
		config->set_property("SignalSource.item_type", FLAGS_item_type);
		config->set_property("SignalSource.decim_rate", "16");
		config->set_property("SignalSource.gain", FLAGS_usrp_gain);
		config->set_property("SignalSource.dump", "false");

		GNSSBlockInterface* usrp_source_gnss_block = factory->GetBlock(config, "SignalSource", "USRPSignalSource", 0, 1, gr_msg_queue_sptr());
		usrp_source_gnss_block->connect(top_block);

		// Build conditioner GNSS block
		config->set_property("SignalConditioner.implementation", "DirectResampler");
		config->set_property("SignalConditioner.item_type", FLAGS_item_type);
		config->set_property("SignalConditioner.sample_freq_in", "4000000");
		config->set_property("SignalConditioner.sample_freq_out", "2048000");
		config->set_property("SignalConditioner.dump", "false");

		GNSSBlockInterface* resampler_gnss_block = factory->GetBlock(config, "SignalConditioner", "DirectResampler", 1, 1, gr_msg_queue_sptr());
		resampler_gnss_block->connect(top_block);

		top_block->connect(usrp_source_gnss_block->get_right_block(), 0, resampler_gnss_block->get_left_block(), 0);
		top_block->connect(resampler_gnss_block->get_right_block(), 0, acquisition_gnss_block->get_left_block(), 0);
	} else {
		// file_source->gnss_acquisition
		gr_block_sptr file_source = gr_make_file_source(item_size, FLAGS_signal_filename.c_str());
		top_block->connect(file_source, 0, acquisition_gnss_block->get_left_block(), 0);
	}

	//printf("runlevel 2 \r \n");
	gr_block_sptr null_sink = gr_make_null_sink(item_size);
	top_block->connect(acquisition_gnss_block->get_right_block(), 0, null_sink, 0); // gnss_acquisition->null_sink

	LOG_AT_LEVEL(INFO) << "testing satellite " << FLAGS_satellite;
	((AcquisitionInterface*)acquisition_gnss_block)->set_satellite(FLAGS_satellite);

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
