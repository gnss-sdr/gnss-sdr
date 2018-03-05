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

#include <chrono>
#include <complex>
#include "gps_sdr_signal_processing.h"
#include "gnss_signal_processing.h"


TEST(CodeGenerationTest, CodeGenGPSL1Test)
{
    std::complex<float>* _dest = new std::complex<float>[1023];
    signed int _prn = 1;
    unsigned int _chip_shift = 4;

    int iterations = 1000;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for (int i = 0; i < iterations; i++)
        {
            gps_l1_ca_code_gen_complex(_dest, _prn, _chip_shift);
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

    delete[] _dest;
    ASSERT_LE(0, elapsed_seconds.count());
    std::cout << "Generation completed in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}


TEST(CodeGenerationTest, CodeGenGPSL1SampledTest)
{
    signed int _prn = 1;
    unsigned int _chip_shift = 4;
    double _fs = 8000000.0;
    const signed int _codeFreqBasis = 1023000;  //Hz
    const signed int _codeLength = 1023;
    int _samplesPerCode = round(_fs / static_cast<double>(_codeFreqBasis / _codeLength));
    std::complex<float>* _dest = new std::complex<float>[_samplesPerCode];

    int iterations = 1000;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for (int i = 0; i < iterations; i++)
        {
            gps_l1_ca_code_gen_complex_sampled(_dest, _prn, _fs, _chip_shift);
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

    delete[] _dest;
    ASSERT_LE(0, elapsed_seconds.count());
    std::cout << "Generation completed in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}


TEST(CodeGenerationTest, ComplexConjugateTest)
{
    double _fs = 8000000.0;
    double _f = 4000.0;
    const signed int _codeFreqBasis = 1023000;  //Hz
    const signed int _codeLength = 1023;
    int _samplesPerCode = round(_fs / static_cast<double>(_codeFreqBasis / _codeLength));
    std::complex<float>* _dest = new std::complex<float>[_samplesPerCode];

    int iterations = 1000;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for (int i = 0; i < iterations; i++)
        {
            complex_exp_gen_conj(_dest, _f, _fs, _samplesPerCode);
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

    delete[] _dest;
    ASSERT_LE(0, elapsed_seconds.count());
    std::cout << "Generation completed in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}
