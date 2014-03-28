/*!
 * \file complex_carrier_test.cc
 * \brief  This file implements tests for the generation of complex exponentials.
 * \author Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
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


#include <complex>
#include <ctime>
#include <armadillo>
#include <volk/volk.h>
#include "gnss_signal_processing.h"

DEFINE_int32(size_carrier_test, 100000, "Size of the arrays used for complex carrier testing");


TEST(ComplexCarrier_Test, StandardComplexImplementation)
{
    std::complex<float>* input = new std::complex<float>[FLAGS_size_carrier_test];
    std::complex<float>* output = new std::complex<float>[FLAGS_size_carrier_test];
    memset(input, 0, sizeof(std::complex<float>) * FLAGS_size_carrier_test);
    double _f = 2000;
    double _fs = 2000000;
    double phase_step = (double)((GPS_TWO_PI * _f) / _fs);
    double phase = 0;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < FLAGS_size_carrier_test; i++)
         {
             output[i] = std::complex<float>(cos(phase), sin(phase));
             phase += phase_step;
         }

    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout << "A " << FLAGS_size_carrier_test
              << "-length complex carrier generated in " << (end - begin)
              << " microseconds" << std::endl;
    ASSERT_LE(0, end - begin);

    delete input;
    delete output;
}




TEST(ComplexCarrier_Test, OwnComplexImplementation)
{
    std::complex<float>* input = new std::complex<float>[FLAGS_size_carrier_test];
    memset(input, 0, sizeof(std::complex<float>) * FLAGS_size_carrier_test);
    std::complex<float>* output = new std::complex<float>[FLAGS_size_carrier_test];
    double _f = 2000;
    double _fs = 2000000;
    double phase_step = (double)((GPS_TWO_PI * _f) / _fs);
    double phase = 0;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    complex_exp_gen(output, _f, _fs, (unsigned int)FLAGS_size_carrier_test);

    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout << "A " << FLAGS_size_carrier_test
              << "-length complex carrier using fixed point generated in " << (end - begin)
              << " microseconds" << std::endl;
    ASSERT_LE(0, end - begin);
    delete input;
    delete output;
}
