
#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

#include <gnuradio/gr_float_to_complex.h>
#include <gnuradio/gr_head.h>
#include <gnuradio/gr_file_sink.h>
#include <gnuradio/gr_top_block.h>

#include "dbfcttc_gpsprn.h"
#include "gnss_sdr_direct_resampler_ccf.h"

using google::LogMessage;

DEFINE_string(filename, "./data/prn_code.dat", "Path to the file where the prn code will be stored.");
DEFINE_int32(fs, 2048000, "FS of the resulting code.");
DEFINE_int32(satellite, 0, "ID of the satellite");
DEFINE_int32(time,0.001,"Signal duration"); // javi

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

	// unsigned int number_of_codes = 1023; // not needed (javi)
	unsigned int phase = 0;

	DLOG(INFO) << "satellite " << FLAGS_satellite;
	DLOG(INFO) << "fs " << FLAGS_fs;
	DLOG(INFO) << "filename " << FLAGS_filename;

	dbfcttc_gpsprn_sptr prn_source = dbfcttc_make_gpsprn(FLAGS_satellite, phase);

	gr_float_to_complex_sptr float2complex = gr_make_float_to_complex(1);
	// resample prn_codes from 1,023 Msps to 2,048 Msps
	gnss_sdr_direct_resampler_ccf_sptr resampler = gnss_sdr_make_direct_resampler_ccf(1023000, FLAGS_fs);
	// Valve is used to stop the flow after an amount of samples went through the block
	gr_block_sptr valve = gr_make_head(sizeof(gr_complex), FLAGS_fs*FLAGS_time);
	gr_file_sink_sptr file = gr_make_file_sink(sizeof(gr_complex), FLAGS_filename.c_str());

	gr_top_block_sptr top_block = gr_make_top_block("prn codes generator");

	top_block->connect(prn_source, 0, float2complex, 0);
	top_block->connect(float2complex, 0, resampler , 0);
	top_block->connect(resampler, 0, valve, 0);
	top_block->connect(valve, 0, file, 0);

	top_block->run();

	DLOG(INFO) << "Finished";

}
