/*!
 * \file cordic_test.cc
 * \brief Test of the CORDIC (COordinate Rotation DIgital Computer) algorithm
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 * 		   Javier Arribas, 2012. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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


#include "GPS_L1_CA.h"
#include <iostream>
#include <cmath>
#include <gtest/gtest.h>
#include "cordic.h"
#include <sys/time.h>
#include <algorithm>
#include <cstdlib> // for RAND_MAX
#include <gnuradio/gr_fxpt_nco.h>
#include "nco_lib.h"

TEST(Cordic_Test, StandardCIsFasterThanCordic)
{
    int largest_k = 10;
    Cordic* cordicPtr;
    cordicPtr = new Cordic(largest_k);

    std::complex<float> *d_carr_sign;
    float* d_carr_sign_I;
    float* d_carr_sign_Q;
    // carrier parameters
    int d_vector_length=4000;
    float phase_rad;
    float phase_step_rad;
    float carrier_freq=2000;
    float d_fs_in=4000000;
    phase_step_rad = (float)GPS_TWO_PI*carrier_freq / (float)d_fs_in;

    // space for carrier wipeoff and signal baseband vectors
    if (posix_memalign((void**)&d_carr_sign, 16, d_vector_length * sizeof(std::complex<float>) * 2) == 0){};

    if (posix_memalign((void**)&d_carr_sign_I, 16, d_vector_length * sizeof(float) * 2) == 0){};
    if (posix_memalign((void**)&d_carr_sign_Q, 16, d_vector_length * sizeof(float) * 2) == 0){};

    double sin_d,cos_d;
    double sin_f,cos_f;


    double niter = 10000;
    struct timeval tv;

    //*** NON-OPTIMIZED CORDIC *****
    gettimeofday(&tv, NULL);
    long long int begin1 = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i=0; i<niter; i++)
        {
    	phase_rad=0;
            for(int j=0;j<d_vector_length;j++)
            {

            	cordicPtr->cordic_get_cos_sin(phase_rad, cos_d, sin_d);
            	d_carr_sign[j]=std::complex<float>(cos_d,-sin_d);
            	phase_rad=phase_rad+phase_step_rad;
            }

        }
    gettimeofday(&tv, NULL);
    long long int end1 = tv.tv_sec *1000000 + tv.tv_usec;


    //*** STD COS, SIN standalone *****
    gettimeofday(&tv, NULL);
    long long int begin2 = tv.tv_sec * 1000000 + tv.tv_usec;
    for(int i=0; i<niter; i++)
        {
			for(int j=0;j<d_vector_length;j++)
			{
				cos_f = std::cos(phase_rad);
				sin_f = std::sin(phase_rad);
				d_carr_sign[j]=std::complex<float>(cos_f,-sin_f);
				phase_rad=phase_rad+phase_step_rad;
			}
        }
    gettimeofday(&tv, NULL);
    long long int end2 = tv.tv_sec *1000000 + tv.tv_usec;

    //*** GNU RADIO FIXED POINT ARITHMETIC ********
    gettimeofday(&tv, NULL);
    long long int begin3 = tv.tv_sec * 1000000 + tv.tv_usec;
    for(int i=0; i<niter; i++)
        {
        	fxp_nco(d_carr_sign, d_vector_length,0, phase_step_rad);
        }
    gettimeofday(&tv, NULL);
    long long int end3 = tv.tv_sec *1000000 + tv.tv_usec;

    //*** SSE2 NCO ****************
     gettimeofday(&tv, NULL);
    long long int begin4 = tv.tv_sec * 1000000 + tv.tv_usec;
    for(int i=0; i<niter; i++)
        {
        	sse_nco(d_carr_sign, d_vector_length,0, phase_step_rad);
        }
    gettimeofday(&tv, NULL);
    long long int end4 = tv.tv_sec *1000000 + tv.tv_usec;


    //*** GNU RADIO FIXED POINT ARITHMETIC COPY BY REFERENCE********
    gettimeofday(&tv, NULL);
    long long int begin5 = tv.tv_sec * 1000000 + tv.tv_usec;
    for(int i=0; i<niter; i++)
        {
        	fxp_nco_cpyref(d_carr_sign, d_vector_length,0, phase_step_rad);
        }
    gettimeofday(&tv, NULL);
    long long int end5 = tv.tv_sec *1000000 + tv.tv_usec;


    //*** GNU RADIO FIXED POINT ARITHMETIC COPY BY REFERENCE SPLIT IQ********
    gettimeofday(&tv, NULL);
    long long int begin6 = tv.tv_sec * 1000000 + tv.tv_usec;
    for(int i=0; i<niter; i++)
        {
        	fxp_nco_IQ_split(d_carr_sign_I, d_carr_sign_Q, d_vector_length,0, phase_step_rad);
        }
    gettimeofday(&tv, NULL);
    long long int end6 = tv.tv_sec *1000000 + tv.tv_usec;

    delete cordicPtr;


    std::cout << "NON-OPTIMIZED CORDIC computed " << niter << " times in " << (end1-begin1) << " microseconds" << std::endl;
    std::cout << "STD LIB ARITHM computed " << niter << " times in " << (end2-begin2) << " microseconds" << std::endl;
    std::cout << "FXPT CORDIC computed " << niter << " times in " << (end3-begin3) << " microseconds" << std::endl;
    std::cout << "SSE CORDIC computed " << niter << " times in " << (end4-begin4) << " microseconds" << std::endl;
    std::cout << "FXPT CORDIC CPY REF computed " << niter << " times in " << (end5-begin5) << " microseconds" << std::endl;
    std::cout << "FXPT CORDIC CPY REF SPLIT computed " << niter << " times in " << (end6-begin6) << " microseconds" << std::endl;
    EXPECT_TRUE((end2-begin2) < (end1-begin1)); // if true, standard C++ is faster than the cordic implementation
}
