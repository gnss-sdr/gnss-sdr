/*!
 * \file complex_carrier_test.cc
 * \brief  This file implements tests for the generation of complex exponentials.
 * \author Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#include "GPS_L1_CA.h"
#include "gnss_signal_processing.h"
#include <armadillo>
#include <chrono>
#include <complex>

#if HAS_STD_SPAN
#include <span>
namespace gsl = std;
#else
#include <gsl/gsl>
#endif

DEFINE_int32(size_carrier_test, 100000, "Size of the arrays used for complex carrier testing");


TEST(ComplexCarrierTest, StandardComplexImplementation)
{
    // Dynamic allocation creates new usable space on the program STACK
    // (an area of RAM specifically allocated to the program)
    auto* output = new std::complex<float>[FLAGS_size_carrier_test];
    const double _f = 2000.0;
    const double _fs = 2000000.0;
    const auto phase_step = static_cast<double>((GPS_TWO_PI * _f) / _fs);
    double phase = 0.0;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for (int i = 0; i < FLAGS_size_carrier_test; i++)
        {
            output[i] = std::complex<float>(cos(phase), sin(phase));
            phase += phase_step;
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "A " << FLAGS_size_carrier_test
              << "-length complex carrier in standard C++ (dynamic allocation) generated in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;

    std::complex<float> expected(1, 0);
    std::vector<std::complex<float>> mag(FLAGS_size_carrier_test);
    for (int i = 0; i < FLAGS_size_carrier_test; i++)
        {
            mag[i] = output[i] * std::conj(output[i]);
        }
    delete[] output;
    for (int i = 0; i < FLAGS_size_carrier_test; i++)
        {
            ASSERT_FLOAT_EQ(std::norm(expected), std::norm(mag[i]));
        }

    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}


TEST(ComplexCarrierTest, C11ComplexImplementation)
{
    // declaration: load data onto the program data segment
    std::vector<std::complex<float>> output(FLAGS_size_carrier_test);
    const double _f = 2000.0;
    const double _fs = 2000000.0;
    const auto phase_step = static_cast<double>((GPS_TWO_PI * _f) / _fs);
    double phase = 0.0;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for (int i = 0; i < FLAGS_size_carrier_test; i++)
        {
            output[i] = std::complex<float>(cos(phase), sin(phase));
            phase += phase_step;
        }
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "A " << FLAGS_size_carrier_test
              << "-length complex carrier in standard C++ (declaration) generated in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
    std::complex<float> expected(1, 0);
    std::vector<std::complex<float>> mag(FLAGS_size_carrier_test);
    for (int i = 0; i < FLAGS_size_carrier_test; i++)
        {
            mag[i] = output[i] * std::conj(output[i]);
            ASSERT_FLOAT_EQ(std::norm(expected), std::norm(mag[i]));
        }
}


TEST(ComplexCarrierTest, OwnComplexImplementation)
{
    auto* output = new std::complex<float>[FLAGS_size_carrier_test];
    double _f = 2000.0;
    double _fs = 2000000.0;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    complex_exp_gen(own::span<std::complex<float>>(output, static_cast<unsigned int>(FLAGS_size_carrier_test)), _f, _fs);

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "A " << FLAGS_size_carrier_test
              << "-length complex carrier using fixed point generated in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;

    std::complex<float> expected(1, 0);
    std::vector<std::complex<float>> mag(FLAGS_size_carrier_test);
    for (int i = 0; i < FLAGS_size_carrier_test; i++)
        {
            mag[i] = output[i] * std::conj(output[i]);
        }
    delete[] output;
    for (int i = 0; i < FLAGS_size_carrier_test; i++)
        {
            ASSERT_NEAR(std::norm(expected), std::norm(mag[i]), 0.0001);
        }
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}
