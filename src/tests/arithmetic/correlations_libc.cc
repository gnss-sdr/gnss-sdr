/*!
 * \file correlations_libc.cc
 * \brief  This file implements a unit test for correlation
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */



#include <gtest/gtest.h>
#include <complex>
#include <sys/time.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <gflags/gflags.h>



using google::LogMessage;

DEFINE_int32(N, 2046, "Samples per millisecond of signal");
DEFINE_int32(M, 3, "Number of correlations per GNSS-SDR channel");
DEFINE_int32(C, 12, "Number of channels to simulate");
DEFINE_string(data_type, "complex", "Data type for samples");

TEST(Correlation_Test, StandardCImplementation)
{
    int correlations = FLAGS_M * FLAGS_C;

    //LOG_AT_LEVEL(INFO) << "Simulating " << FLAGS_C << " channels";
    //LOG_AT_LEVEL(INFO) << FLAGS_M << " correlations per channel";
    //LOG_AT_LEVEL(INFO) << "Performing " << correlations << " correlations";
    //LOG_AT_LEVEL(INFO) << "Testing standard C++ library using complex numbers";

    std::complex<float>* input = new std::complex<float>[FLAGS_N];
    std::complex<float> accum;

    std::srand((unsigned)time(0));

    for(int i=0; i < FLAGS_N; i++)
        {
            input[i] = std::complex<float>(std::rand() % 10000, std::rand() % 10000);
        }

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i=0; i<correlations; i++)
        {
            for(int j=0; j < FLAGS_N; j++)
                {
                    input[j] = input[j] * input[j];
                    input[j] = input[j] * input[j];
                    accum += input[j];
                }
        }

    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout << correlations << " correlations of " << FLAGS_N
            << "-length vectors computed in " << (end - begin)
            << " microseconds" << std::endl;
}
