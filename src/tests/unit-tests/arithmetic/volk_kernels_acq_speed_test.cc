/*!
 * \file volk_kernels_acq_speed_test.cc
 * \brief  This file implements tests for measuring speed performances
 *         of volk kernels involved in acquisition blocks.
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
#include <iostream>
#include <complex>
#include <armadillo>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>

DEFINE_int32(repetitions_test_volk, 1000, "Averaging number in VolkSpeedTest");

TEST(VolkSpeedTest, MagnitudeSquared)
{
	std::chrono::time_point<std::chrono::system_clock> start, end;
	int sizes[14] = {1000, 1024, 2000, 2048, 4000, 4096, 8000, 8192, 16000, 16384, 32000, 32768, 64000, 65536};
	for (int aux = 0; aux < 14; aux++)
	{
		int d_size = sizes[aux];
		float* magnitude = static_cast<float*>(volk_gnsssdr_malloc(sizeof(float) * d_size, volk_gnsssdr_get_alignment()));
		gr_complex* in = static_cast<gr_complex*>(volk_gnsssdr_malloc(sizeof(gr_complex) * d_size, volk_gnsssdr_get_alignment()));
        arma::arma_rng::set_seed_random();
        arma::cx_fvec d_vec = arma::cx_fvec(d_size).randn() + gr_complex(0.0, 1.0) * arma::cx_fvec(d_size).randn();
        memcpy(in, d_vec.memptr(), sizeof(gr_complex) * d_size);
        start = std::chrono::system_clock::now();
        for (int k = 0; k < FLAGS_repetitions_test_volk; k++)
        {
        	volk_32fc_magnitude_squared_32f(magnitude, in, d_size);
        }
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        volk_gnsssdr_free(magnitude);
        volk_gnsssdr_free(in);
        std::cout << "Magnitude squared. Vector size: " << d_size << ". Averaged processing time: " << elapsed_seconds.count() * 1e6 / static_cast<double>(FLAGS_repetitions_test_volk) << " [us]" << std::endl;
	}
}

TEST(VolkSpeedTest, Accumulator)
{
	std::chrono::time_point<std::chrono::system_clock> start, end;
	int sizes[14] = {1000, 1024, 2000, 2048, 4000, 4096, 8000, 8192, 16000, 16384, 32000, 32768, 64000, 65536};
	for (int aux = 0; aux < 14; aux++)
	{
		int d_size = sizes[aux];
		float* in = static_cast<float*>(volk_gnsssdr_malloc(sizeof(float) * d_size, volk_gnsssdr_get_alignment()));
		float res = 0.0;
        arma::arma_rng::set_seed_random();
        arma::fvec d_vec = arma::fvec(d_size).randn();
        d_vec = arma::abs(d_vec);
        memcpy(in, d_vec.memptr(), sizeof(float) * d_size);
        start = std::chrono::system_clock::now();
        for (int k = 0; k < FLAGS_repetitions_test_volk; k++)
        {
        	volk_32f_accumulator_s32f(&res, in, d_size);
        }
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        volk_gnsssdr_free(in);
        std::cout << "Accumulator. Vector size: " << d_size << ". Averaged processing time: " << elapsed_seconds.count() * 1e6 / static_cast<double>(FLAGS_repetitions_test_volk) << " [us]" << std::endl;
	}
}

TEST(VolkSpeedTest, Multiply)
{
	std::chrono::time_point<std::chrono::system_clock> start, end;
	int sizes[14] = {1000, 1024, 2000, 2048, 4000, 4096, 8000, 8192, 16000, 16384, 32000, 32768, 64000, 65536};
	for (int aux = 0; aux < 14; aux++)
	{
		int d_size = sizes[aux];
		gr_complex* in1 = static_cast<gr_complex*>(volk_gnsssdr_malloc(sizeof(gr_complex) * d_size, volk_gnsssdr_get_alignment()));
		gr_complex* in2 = static_cast<gr_complex*>(volk_gnsssdr_malloc(sizeof(gr_complex) * d_size, volk_gnsssdr_get_alignment()));
		gr_complex* res = static_cast<gr_complex*>(volk_gnsssdr_malloc(sizeof(gr_complex) * d_size, volk_gnsssdr_get_alignment()));
		arma::arma_rng::set_seed_random();
        arma::cx_fvec d_vec = arma::cx_fvec(d_size).randn() + gr_complex(0.0, 1.0) * arma::cx_fvec(d_size).randn();
        memcpy(in1, d_vec.memptr(), sizeof(gr_complex) * d_size);
        arma::arma_rng::set_seed_random();
        d_vec = arma::cx_fvec(d_size).randn() + gr_complex(0.0, 1.0) * arma::cx_fvec(d_size).randn();
        memcpy(in2, d_vec.memptr(), sizeof(gr_complex) * d_size);
        start = std::chrono::system_clock::now();
        for (int k = 0; k < FLAGS_repetitions_test_volk; k++)
        {
        	volk_32fc_x2_multiply_32fc(res, in1, in2, d_size);
        }
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        volk_gnsssdr_free(in1);
        volk_gnsssdr_free(in2);
        volk_gnsssdr_free(res);
        std::cout << "Multiply. Vector size: " << d_size << ". Averaged processing time: " << elapsed_seconds.count() * 1e6 / static_cast<double>(FLAGS_repetitions_test_volk) << " [us]" << std::endl;
	}
}

TEST(VolkSpeedTest, IndexMax)
{
	std::chrono::time_point<std::chrono::system_clock> start, end;
	int sizes[14] = {1000, 1024, 2000, 2048, 4000, 4096, 8000, 8192, 16000, 16384, 32000, 32768, 64000, 65536};
	for (int aux = 0; aux < 14; aux++)
	{
		int d_size = sizes[aux];
		float* in = static_cast<float*>(volk_gnsssdr_malloc(sizeof(float) * d_size, volk_gnsssdr_get_alignment()));
		uint32_t indext = 0;
		arma::arma_rng::set_seed_random();
        arma::fvec d_vec = arma::fvec(d_size).randn();
        d_vec = arma::abs(d_vec);
        memcpy(in, d_vec.memptr(), sizeof(float) * d_size);
        start = std::chrono::system_clock::now();
        for (int k = 0; k < FLAGS_repetitions_test_volk; k++)
        {
        	volk_gnsssdr_32f_index_max_32u(&indext, in, d_size);
        }
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        volk_gnsssdr_free(in);
        std::cout << "Max index search. Vector size: " << d_size << ". Averaged processing time: " << elapsed_seconds.count() * 1e6 / static_cast<double>(FLAGS_repetitions_test_volk) << " [us]" << std::endl;
	}
}

TEST(VolkSpeedTest, Memcpy)
{
	std::chrono::time_point<std::chrono::system_clock> start, end;
	int sizes[14] = {1000, 1024, 2000, 2048, 4000, 4096, 8000, 8192, 16000, 16384, 32000, 32768, 64000, 65536};
	for (int aux = 0; aux < 14; aux++)
	{
		int d_size = sizes[aux];
		gr_complex* in = static_cast<gr_complex*>(volk_gnsssdr_malloc(sizeof(gr_complex) * d_size, volk_gnsssdr_get_alignment()));
		gr_complex* out = static_cast<gr_complex*>(volk_gnsssdr_malloc(sizeof(gr_complex) * d_size, volk_gnsssdr_get_alignment()));
		arma::arma_rng::set_seed_random();
        arma::cx_fvec d_vec = arma::cx_fvec(d_size).randn() + gr_complex(0.0, 1.0) * arma::cx_fvec(d_size).randn();
        memcpy(in, d_vec.memptr(), sizeof(gr_complex) * d_size);
        start = std::chrono::system_clock::now();
        for (int k = 0; k < FLAGS_repetitions_test_volk; k++)
        {
        	memcpy(out, in, sizeof(gr_complex) * d_size);
        }
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        volk_gnsssdr_free(in);
        volk_gnsssdr_free(out);
        std::cout << "Memcpy. Vector size: " << d_size << ". Averaged processing time: " << elapsed_seconds.count() * 1e6 / static_cast<double>(FLAGS_repetitions_test_volk) << " [us]" << std::endl;
	}
}

