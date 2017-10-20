/*!
 * \file fft_length_test.cc
 * \brief  This file implements timing tests for the FFT.
 * \author Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2016  (see AUTHORS file for a list of contributors)
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

#include <algorithm>
#include <chrono>
#include <functional>
#include <random>
#include <gnuradio/fft/fft.h>


DEFINE_int32(fft_iterations_test, 1000, "Number of averaged iterations in FFT length timing test");

TEST(FFTLengthTest, MeasureExecutionTime)
{
    unsigned int fft_sizes [] = { 1000, 1024, 1960, 2000, 2048, 4000, 4096, 4725, 5000, 8000, 8192, 10368, 12000, 16000, 16384, 27000, 32768, 50000, 65536 };

    std::chrono::time_point<std::chrono::system_clock> start, end;

    std::random_device r;
    std::default_random_engine e1(r());
    std::default_random_engine e2(r());
    std::uniform_real_distribution<float> uniform_dist(-1, 1);
    auto func = [] (float a, float b) { return gr_complex(a, b); }; // Helper lambda function that returns a gr_complex
    auto random_number1 = std::bind(uniform_dist, e1);
    auto random_number2 = std::bind(uniform_dist, e2);
    auto gen = std::bind(func, random_number1, random_number2); // Function that returns a random gr_complex

    std::vector<unsigned int> fft_sizes_v(fft_sizes, fft_sizes + sizeof(fft_sizes) / sizeof(unsigned int) );
    std::vector<unsigned int>::const_iterator it;
    unsigned int d_fft_size;

    EXPECT_NO_THROW(
            for(it = fft_sizes_v.cbegin(); it != fft_sizes_v.cend(); ++it)
                {
                    gr::fft::fft_complex* d_fft;
                    d_fft_size = *it;
                    d_fft = new gr::fft::fft_complex(d_fft_size, true);

                    std::generate_n( d_fft->get_inbuf(), d_fft_size, gen );

                    start = std::chrono::system_clock::now();
                    for(int k = 0; k < FLAGS_fft_iterations_test; k++)
                        {
                            d_fft->execute();
                        }
                    end = std::chrono::system_clock::now();
                    std::chrono::duration<double> elapsed_seconds = end - start;
                    double execution_time = elapsed_seconds.count() / static_cast<double>(FLAGS_fft_iterations_test);
                    std::cout << "FFT execution time for length=" << d_fft_size << " : " << execution_time << " [s]" << std::endl;
                    delete d_fft;
                }
    );
}
