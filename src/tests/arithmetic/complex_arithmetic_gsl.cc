
#include <sys/time.h>
#include <cstdlib>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_vector.h>

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

	LOG_AT_LEVEL(INFO) << "Using standard GNU GSL library to perform complex arithmetic";

	std::srand((int)time(0));

	gsl_vector_complex* input = gsl_vector_complex_alloc(FLAGS_N);
	gsl_complex zero;
	GSL_SET_COMPLEX(&zero, std::rand() % 10000, std::rand() % 10000);

	gsl_vector_complex_set_all(input, zero);

	LOG_AT_LEVEL(INFO) << "Begin calculations";

	struct timeval tv;
	gettimeofday(&tv, NULL);
	long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

	for(int i=0;i<FLAGS_M;i++) {
		gsl_vector_complex_mul(input, input);
	}

	gettimeofday(&tv, NULL);
	long long int end = tv.tv_sec *1000000 + tv.tv_usec;
	LOG_AT_LEVEL(INFO) << "Finished in " << (end - begin) << " microseconds";
}
