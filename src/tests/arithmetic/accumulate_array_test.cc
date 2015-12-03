/*!
 * \file accumulate_array_test.cc
 * \brief  This file implements tests for accumulate_array 
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
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


#include "accumulate_array.h"
#include "code_resampler.h"
#include <complex>
#include <ctime>
#include <sys/time.h>

#include <armadillo>

#include <gtest/gtest.h>

typedef std::vector< float > real_vec;
typedef std::vector< std::complex< float > > cplx_vec;

TEST(AccumulateArrayTest, FunctionalTest )
{


    real_vec input_vec( 13 );
    std::iota( input_vec.begin(), input_vec.end(), 0 );

    std::vector< int > index_vec = { 0, 0, 0, 1, 1, 1, 1,  2, 2, 3, 3, 3, 3 };

    real_vec output_vec( 5, 0.0 );

    accumulate_array( &input_vec[0], &index_vec[0], &output_vec[0], 13 );

    real_vec expected_output = { 3, 18, 15, 42, 0 };

    for( unsigned int ii = 0; ii < expected_output.size(); ++ii )
    {
        ASSERT_FLOAT_EQ( output_vec[ii], expected_output[ii] );
    }


}

TEST(AccumulateArrayTest, ComplexTest )
{

    double fs = 6e6;
    double fc = 1.023e6;
    double T = 0.001;

    double code_phase_step = fc/fs;

    int _num_samples = static_cast< int >(
            std::ceil( T * fs )
            );

    cplx_vec output_vec( 1023, 0.0 );

    std::vector< int > code_phases( 1023 );
    std::iota( code_phases.begin(), code_phases.end(), 0 );

    CodeResamplerGeneric< int > the_resampler;

    std::vector< int > index_vec( _num_samples );



    std::vector< int* > _resampled_code_phases;
    _resampled_code_phases.resize(1);
    _resampled_code_phases[0] = &index_vec[0];

    std::vector< double > _desired_code_phases;
    _desired_code_phases.resize(1);

    _desired_code_phases[0] = 0;

    the_resampler.resample_code( &code_phases[0], 1023, _desired_code_phases,
            code_phase_step, _num_samples, _resampled_code_phases );


    int iterations = 1000;

    arma::cx_mat rv = arma::randn< arma::cx_mat >( _num_samples, iterations );
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;


    for(int i = 0; i < iterations; i++)
        {

            cplx_vec input_vec = arma::conv_to< cplx_vec >::from( rv.col( i ) );

            accumulate_array( &input_vec[0], &index_vec[0], &output_vec[0], _num_samples );

        }
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    ASSERT_LE(0, end - begin);
    std::cout << "Accumulate_array completed in " << (end - begin) << " microseconds" << std::endl;


}

