
#include <complex>
#include <sys/time.h>
#include <cstdlib>
#include <ctime>

#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

DEFINE_int32(N, 2046, "Samples per milisecond of signal");
DEFINE_int32(M, 6000, "Number of correlations per GNSS-SDR channel");
DEFINE_int32(C, 12, "Number of channels to simulate");
DEFINE_string(data_type, "complex", "Data type for samples");

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

	int correlations = FLAGS_M * FLAGS_C;

	LOG_AT_LEVEL(INFO) << "Simulating " << FLAGS_C << " channels";
	LOG_AT_LEVEL(INFO) << FLAGS_M << " correlations per channel";
	LOG_AT_LEVEL(INFO) << "Performing " << correlations << " correlations";

	LOG_AT_LEVEL(INFO) << "Testing standard C++ library using complex numbers";

	std::complex<float>* input = new std::complex<float>[FLAGS_N];
	std::complex<float> accum;

	std::srand((unsigned)time(0));

	for(int i=0;i<FLAGS_N;i++) {
		input[i] = std::complex<float>(std::rand() % 10000, std::rand() % 10000);
	}

	LOG_AT_LEVEL(INFO) << "Begin Calculations";

	struct timeval tv;
	gettimeofday(&tv, NULL);
	long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

	for(int i=0;i<correlations; i++) {
		for(int j=0;j<FLAGS_N;j++) {
			input[j] = input[j]*input[j];
			input[j] = input[j]*input[j];
			accum += input[j];
		}
	}

	gettimeofday(&tv, NULL);
	long long int end = tv.tv_sec *1000000 + tv.tv_usec;
	LOG_AT_LEVEL(INFO) << "Finished in " << (end - begin) << " microseconds";
}
