/*!
 * \file code_generation_test.cc
 * \brief  This file implements tests for the generation of complex exponentials.
 * \author Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
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
#include "gps_sdr_signal_processing.h"



TEST(CodeGenGPSL1_Test, CodeGeneration)
{
    std::complex<float>* _dest = new std::complex<float>[1023];
    signed int _prn = 1;
    unsigned int _chip_shift = 4;

    int iterations = 100;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < iterations; i++)
        {
            gps_l1_ca_code_gen_complex( _dest,  _prn,  _chip_shift);
        }

    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    ASSERT_LE(0, end - begin);
    std::cout << "Generation completed in " << (end - begin) << " microseconds" << std::endl;
    delete[] _dest;


    /* std::complex<float>* _dest2 = new std::complex<float>[1023];gettimeofday(&tv, NULL);
    long long int begin2 = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < iterations; i++)
        {
            gps_l1_ca_code_gen_complex2( _dest2,  _prn,  _chip_shift);
        }

    gettimeofday(&tv, NULL);
    long long int end2 = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout << "Generation 2 completed in " << (end2 - begin2) << " microseconds" << std::endl;

    for (int j=0; j<1023;j++)
        {
            if(_dest[j] != _dest2[j]) std::cout << "Error!" << std::endl;
        }
    delete _dest2; */
}
