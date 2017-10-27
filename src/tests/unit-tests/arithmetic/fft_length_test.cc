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

#include <chrono>
#include <gnuradio/fft/fft.h>


DEFINE_int32(fft_iterations_test, 1000, "Number of averaged iterations in FFT length timing test");

TEST(FFTLengthTest, MeasureExecutionTime)
{
    unsigned int d_fft_size;
    std::chrono::time_point<std::chrono::system_clock> start, end;

    unsigned int fft_sizes [18] = { 1000, 1024, 1960, 2000, 2048, 4000, 4096, 4725, 8000, 8192, 10368, 12000, 16000, 16384, 27000, 32768, 50000, 65536 };
    double execution_times [18];
    EXPECT_NO_THROW(
            for(int i = 0; i < 18; i++)
                {
                    gr::fft::fft_complex* d_fft;
                    d_fft_size = fft_sizes[i];
                    d_fft = new gr::fft::fft_complex(d_fft_size, true);
                    std::fill_n( d_fft->get_inbuf(), d_fft_size, gr_complex( 0.0, 0.0 ) );

                    start = std::chrono::system_clock::now();
                    for(int k = 0; k < FLAGS_fft_iterations_test; k++)
                        {
                            d_fft->execute();
                        }
                    end = std::chrono::system_clock::now();
                    std::chrono::duration<double> elapsed_seconds = end - start;
                    execution_times[i] = elapsed_seconds.count() / static_cast<double>(FLAGS_fft_iterations_test);
                    std::cout << "FFT execution time for length=" << d_fft_size << " : " << execution_times[i] * 1e6 << " [us]" << std::endl;
                    delete d_fft;
                }
    );
}
