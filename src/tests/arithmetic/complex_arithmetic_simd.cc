
#include <sys/time.h>
#include <cstdlib>

#include "gps_sdr_simd.h"

#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

DEFINE_int32(N, 2046, "Size of the arrays used for calculations");
DEFINE_int32(M, 1000, "Iterations");

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

	LOG_AT_LEVEL(INFO) << "Using standard SIMD implementation (GPS-SDR) to perform complex arithmetic";

	std::srand((int)time(0));
	CPX* input = new CPX[FLAGS_N];

	for(int i=0;i<FLAGS_N;i++) {
		input[i].i = std::rand() % 10000;
		input[i].q = std::rand() % 10000;
	}

	LOG_AT_LEVEL(INFO) << "Begin calculations";

	struct timeval tv;
	gettimeofday(&tv, NULL);
	long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

	for(int i=0;i<FLAGS_M;i++) {
		sse_cmul(input, input, FLAGS_N);
	}

	gettimeofday(&tv, NULL);
	long long int end = tv.tv_sec *1000000 + tv.tv_usec;
	LOG_AT_LEVEL(INFO) << "Finished in " << (end - begin) << " microseconds";
}
