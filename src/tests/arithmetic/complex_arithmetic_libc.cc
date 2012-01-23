/*!
 * \file complex_arithmetic_libc.cc
 * \brief  This file implements a unit test for multiplication of long arrays.
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
#include <gflags/gflags.h>
#include <complex>
#include <sys/time.h>
#include <iostream>
#include <glog/log_severity.h>
#include <glog/logging.h>


using google::LogMessage;

DEFINE_int32(size_multiply_test, 100000, "Size of the arrays used for calculations");

TEST(Multiply_Test, StandardCImplementation)
{
    //LOG_AT_LEVEL(INFO) << "Using standard C++ library implementation to perform complex arithmetic";
    std::complex<float>* input = new std::complex<float>[FLAGS_size_multiply_test];
    std::complex<float>* output = new std::complex<float>[FLAGS_size_multiply_test];

    memset(input, 0, sizeof(std::complex<float>) * FLAGS_size_multiply_test);

    //LOG_AT_LEVEL(INFO) << "Allocated two vectors containing " << FLAGS_size << " complex numbers";
    //LOG_AT_LEVEL(INFO) << "Begin multiplications";

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i=0; i<FLAGS_size_multiply_test; i++)
        {
            output[i] = input[i] * input[i];
        }

    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec *1000000 + tv.tv_usec;
   // LOG_AT_LEVEL(INFO) << "Finished in " << (end - begin) << " microseconds";
    std::cout <<  "Multiplication of "<< FLAGS_size_multiply_test << " complex<float> finished in " << (end - begin) << " microseconds" << std::endl;
}
