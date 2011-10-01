
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * Executes a gps sdr acquisition based on some input parameters.
 *
 */

#include <iostream>
#include <sys/time.h>

#include <gflags/gflags.h>

#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

#include <gr_top_block.h>
#include <gr_file_source.h>
#include <gr_null_sink.h>
#include <gps_sdr_acquisition_ss.h>
#include <gr_stream_to_vector.h>

#include "gps_sdr_simd.h"

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);

	CPX* A = new CPX[100];
	CPX* B = new CPX[100];
	CPX* C = new CPX[100];

	for(int i=0;i<100;i++) {
		A[i].i = 1;
		A[i].q = 1;
		B[i].i = 0;
		B[i].q = 0;
		C[i].i = 2;
		C[i].q = 2;
	}

	LOG_AT_LEVEL(INFO) << "Run";
	struct timeval tv;
	gettimeofday(&tv, NULL);
	long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;
	sse_cmulsc(A, B, C, 10, 10);
	gettimeofday(&tv, NULL);
	long long int end = tv.tv_sec *1000000 + tv.tv_usec;
	LOG_AT_LEVEL(INFO) << "Finished in " << (end - begin) << " microseconds";
	std::cout << (end - begin) << std::endl;

	std::cout << "A=[";
	for(int i=0;i<100;i++) {
		std::cout << A[i].i << "," << A[i].q << ":";
	}
	std::cout << "]" << std::endl;

	std::cout << "B=[";
	for(int i=0;i<100;i++) {
		std::cout << B[i].i << "," << B[i].q << ":";
	}
	std::cout << "]" << std::endl;

	std::cout << "C=[";
	for(int i=0;i<100;i++) {
		std::cout << C[i].i << "," << C[i].q << ":";
	}
	std::cout << "]" << std::endl;;
}
