
#include <complex>
#include <sys/time.h>

#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

DEFINE_int32(size, 100000, "Size of the arrays used for calculations");

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

	LOG_AT_LEVEL(INFO) << "Using standard C++ library implementation to perform complex arithmetic";

	std::complex<float>* input = new std::complex<float>[FLAGS_size];
	std::complex<float>* output = new std::complex<float>[FLAGS_size];

	memset(input, 0, sizeof(std::complex<float>)*FLAGS_size);

	LOG_AT_LEVEL(INFO) << "Allocated two vectors containing " << FLAGS_size << " complex numbers";

	LOG_AT_LEVEL(INFO) << "Begin multiplications";

	struct timeval tv;
	gettimeofday(&tv, NULL);
	long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

	for(int i=0;i<FLAGS_size;i++) {
		output[i] = input[i] * input[i];
	}

	gettimeofday(&tv, NULL);
	long long int end = tv.tv_sec *1000000 + tv.tv_usec;
	LOG_AT_LEVEL(INFO) << "Finished in " << (end - begin) << " microseconds";
}
