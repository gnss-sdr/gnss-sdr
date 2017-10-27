/*!
 * \file fft_buffer_test.cc
 * \brief  This file implements a input-buffer invariability test for the GNU Radio FFT.
 *         Checks if the elements located in the input buffer of the gr::fft::fft_complex object
 *         are not corrupted after the FFT computation.
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
#include <gnuradio/fft/fft.h>
#include <armadillo>

TEST(FFTInvariabilityTest, CheckInvariability)
{
    unsigned int d_fft_size;
    unsigned int fft_sizes [18] = { 1000, 1024, 1960, 2000, 2048, 4000, 4096, 4725, 8000, 8192, 10368, 12000, 16000, 16384, 27000, 32768, 50000, 65536 };
    EXPECT_NO_THROW(
            for(int i = 0; i < 18; i++)
                {
                    gr::fft::fft_complex* d_fft;
                    d_fft_size = fft_sizes[i];
                    d_fft = new gr::fft::fft_complex(d_fft_size, true);
                    arma::arma_rng::set_seed_random();
                    arma::cx_fvec d_arma_fft = arma::cx_fvec(d_fft_size).randn() + gr_complex(0.0, 1.0) * arma::cx_fvec(d_fft_size).randn();
                    memcpy(d_fft->get_inbuf(), d_arma_fft.memptr(), sizeof(gr_complex) * d_fft_size);
                    d_fft->execute();
                    EXPECT_PRED_FORMAT2(::testing::DoubleLE, 0.5, arma::norm(arma::cx_fvec(d_fft->get_outbuf(), d_fft_size)));
                    EXPECT_PRED_FORMAT2(::testing::DoubleLE, 0.5, arma::norm(d_arma_fft));
                    arma::cx_fvec d_diff = d_arma_fft - arma::cx_fvec(d_fft->get_inbuf(), d_fft_size);
                    EXPECT_DOUBLE_EQ(0.0, arma::norm(d_diff));
                    delete d_fft;
                }
    );
}
