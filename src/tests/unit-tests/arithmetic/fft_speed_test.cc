/*!
 * \file fft_speed_test.cc
 * \brief  This file implements timing tests for the Armadillo
 *         and GNU Radio FFT implementations
 * \author Antonio Ramos, 2017. antonio.ramos(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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
#include <memory>
#include <gnuradio/fft/fft.h>
#include <armadillo>


DEFINE_int32(fft_speed_iterations_test, 1000, "Number of averaged iterations in FFT length timing test");

TEST(FFTSpeedTest, ArmadilloVSGNURadioExecutionTime)
{
    unsigned int d_fft_size;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds;

    unsigned int fft_sizes [13] = { 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536 };
    double d_execution_time;
    EXPECT_NO_THROW(
            for(int i = 0; i < 13; i++)
                {
                    d_fft_size = fft_sizes[i];    
                    
                    gr::fft::fft_complex* d_gr_fft;
                    d_gr_fft = new gr::fft::fft_complex(d_fft_size, true);
                    
                    arma::cx_vec d_arma_fft(d_fft_size);
                    d_arma_fft = arma::cx_vec(d_fft_size).randn() + gr_complex(0.0, 1.0) * arma::cx_vec(d_fft_size).randn();
                    
                    memcpy(d_gr_fft->get_inbuf(), d_arma_fft.memptr(), sizeof(gr_complex) * d_fft_size);
                    
                    start = std::chrono::system_clock::now();
                    for(int k = 0; k < FLAGS_fft_speed_iterations_test; k++)
                        {
                            d_gr_fft->execute();
                        }
                    end = std::chrono::system_clock::now();
                    elapsed_seconds = end - start;
                    d_execution_time = elapsed_seconds.count() / static_cast<double>(FLAGS_fft_speed_iterations_test);
                    std::cout << "GNU Radio FFT execution time for length = " << d_fft_size << " : " << d_execution_time << " [s]" << std::endl;
                    delete d_gr_fft;
                    
                    start = std::chrono::system_clock::now();
                    for(int k = 0; k < FLAGS_fft_speed_iterations_test; k++)
                        {
                            arma::fft(d_arma_fft);
                        }
                    end = std::chrono::system_clock::now();
                    elapsed_seconds = end - start;
                    d_execution_time = elapsed_seconds.count() / static_cast<double>(FLAGS_fft_speed_iterations_test);
                    std::cout << "Armadillo FFT execution time for length = " << d_fft_size << " : " << d_execution_time << " [s]" << std::endl;
                }
    );
}
