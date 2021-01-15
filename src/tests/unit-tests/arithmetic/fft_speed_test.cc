/*!
 * \file fft_speed_test.cc
 * \brief  This file implements timing tests for the Armadillo
 *         and GNU Radio FFT implementations
 * \author Antonio Ramos, 2017. antonio.ramos(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "gnss_sdr_fft.h"
#include <armadillo>
#include <chrono>
#include <memory>

DEFINE_int32(fft_speed_iterations_test, 100, "Number of averaged iterations in FFT length timing test");

TEST(FFTSpeedTest, ArmadilloVSGNURadioExecutionTime)
{
    unsigned int d_fft_size;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds;

    unsigned int fft_sizes[19] = {16, 25, 32, 45, 64, 95, 128, 195, 256, 325, 512, 785, 1024, 1503, 2048, 3127, 4096, 6349, 8192};
    double d_execution_time;
    EXPECT_NO_THROW(
        for (unsigned int fft_size
             : fft_sizes) {
            d_fft_size = fft_size;
            auto d_gr_fft = gnss_fft_fwd_make_unique(d_fft_size);
            arma::arma_rng::set_seed_random();
            arma::cx_fvec d_arma_fft = arma::cx_fvec(d_fft_size).randn() + gr_complex(0.0, 1.0) * arma::cx_fvec(d_fft_size).randn();
            arma::cx_fvec d_arma_fft_result(d_fft_size);
            memcpy(d_gr_fft->get_inbuf(), d_arma_fft.memptr(), sizeof(gr_complex) * d_fft_size);

            start = std::chrono::system_clock::now();
            for (int k = 0; k < FLAGS_fft_speed_iterations_test; k++)
                {
                    d_gr_fft->execute();
                }
            end = std::chrono::system_clock::now();
            elapsed_seconds = end - start;
            d_execution_time = elapsed_seconds.count() / static_cast<double>(FLAGS_fft_speed_iterations_test);
            std::cout << "GNU Radio FFT execution time for length = " << d_fft_size << " : " << d_execution_time * 1e6 << " [us]\n";

            start = std::chrono::system_clock::now();
            for (int k = 0; k < FLAGS_fft_speed_iterations_test; k++)
                {
                    d_arma_fft_result = arma::fft(d_arma_fft);
                }
            end = std::chrono::system_clock::now();
            elapsed_seconds = end - start;
            d_execution_time = elapsed_seconds.count() / static_cast<double>(FLAGS_fft_speed_iterations_test);
            std::cout << "Armadillo FFT execution time for length = " << d_fft_size << " : " << d_execution_time * 1e6 << " [us]\n";
        });
}
