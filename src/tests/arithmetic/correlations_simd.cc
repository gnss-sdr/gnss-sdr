#include <sys/time.h>
#include <cstdlib>
#include <ctime>

#include "gps_sdr_simd.h"

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

	LOG_AT_LEVEL(INFO) << "Testing GPS SDR SIMD library using CPX struct";

	std::srand((unsigned)time(0));

	CPX* input = new CPX[FLAGS_N];

	for (int i=0;i<FLAGS_N;i++) {
		input[i].i = std::rand() % 10000;
		input[i].q = std::rand() % 10000;
	}

	LOG_AT_LEVEL(INFO) << "Begin Calculations";

	struct timeval tv;
	gettimeofday(&tv, NULL);
	long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

	for(int i=0;i<correlations; i++) {
		sse_cmul(input, input, FLAGS_N);
		sse_cmul(input, input, FLAGS_N);
	}

	gettimeofday(&tv, NULL);
	long long int end = tv.tv_sec *1000000 + tv.tv_usec;
	LOG_AT_LEVEL(INFO) << "Finished in " << (end - begin) << " microseconds";
}


//void test_gsl_complex() {

//	LOG_AT_LEVEL(INFO) << "Testing GNU GSL library usign gsl_complex";
//
//	gsl_vector_complex* input = gsl_vector_complex_alloc(FLAGS_N);
//	gsl_vector_complex* ones = gsl_vector_complex_alloc(FLAGS_N);
//	gsl_complex one;
//	gsl_complex sum;
//	GSL_SET_COMPLEX(&one, 1, 1);
//	GSL_SET_COMPLEX(&sum, 1, 1);
//
//	std::srand((unsigned)time(0));
//
//	for(int i=0;i<FLAGS_N;i++) {
//		gsl_vector_complex_set(input, i, std::rand() % 10000);
//	}
//
//	gsl_vector_complex_set_all(ones, one);
//
//	LOG_AT_LEVEL(INFO) << "Begin Calculations";
//
//	struct timeval tv;
//	gettimeofday(&tv, NULL);
//	long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;
//
//	for(int i=0;i<FLAGS_M;i++) {
//		gsl_vector_complex_mul(input, input);
//		gsl_vector_complex_mul(input, input);
//	}
//
//	gettimeofday(&tv, NULL);
//	long long int end = tv.tv_sec *1000000 + tv.tv_usec;
//	LOG_AT_LEVEL(INFO) << "Finished in " << (end - begin) << " microseconds";
//}

//void test_gps_sdr_simd() {

//	LOG_AT_LEVEL(INFO) << "Testing GPS SDR SIMD library";
//
//	CPX* input = new CPX[FLAGS_N];
//	memset(input, 0, FLAGS_N);
//
//	LOG_AT_LEVEL(INFO) << "Begin Calculations";
//
//	struct timeval tv;
//	gettimeofday(&tv, NULL);
//	long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;
//
//	for(int i=0;i<FLAGS_M;i++) {
//		sse_cmul(input, input, FLAGS_N);
//	}
//
//	gettimeofday(&tv, NULL);
//	long long int end = tv.tv_sec *1000000 + tv.tv_usec;
//	LOG_AT_LEVEL(INFO) << "Finished in " << (end - begin) << " microseconds";
//}

