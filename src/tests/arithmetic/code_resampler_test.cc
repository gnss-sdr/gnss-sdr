/*!
 * \file code_generation_test.cc
 * \brief  This file implements tests for resampling codes
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


#include "code_resampler.h"
#include <complex>
#include <ctime>
#include <sys/time.h>
#include "gps_sdr_signal_processing.h"
#include "gnss_signal_processing.h"

#include <gtest/gtest.h>

void allocate_replicas( std::vector< std::complex< float > * >& replicas, int num_samples )
{

    for( int i = 0;  i < replicas.size(); ++i )
    {
        replicas[i] = static_cast< std::complex<float> *>(
                volk_malloc( num_samples*sizeof( std::complex<float> ),
                    volk_get_alignment() )
                );
    }
}

void deallocate_replicas( std::vector< std::complex< float > * > &replicas )
{
    for( int i = 0; i < replicas.size(); ++i )
    {
        volk_free( replicas[i] );
    }
}

TEST(CodeResamplerTest, CodeResamplerGeneric )
{
    std::complex<float>* _orig_code = new std::complex<float>[1023];
    signed int _prn = 1;
    unsigned int _chip_shift = 4;
    unsigned int _num_replicas = 3;

    double fs = 6e6;
    double fc = 1.023e6;

    double code_phase_step = fc/fs;
    double code_spacing_chips = 0.5;

    double T = 0.001;

    int _num_samples = static_cast< int >(
            std::ceil( T * fs )
            );

    std::vector< std::complex<float>* > _resampled_codes;
    _resampled_codes.resize(_num_replicas);
    std::vector< double > _desired_code_phases;
    _desired_code_phases.resize(_num_replicas);

    gps_l1_ca_code_gen_complex( _orig_code,  _prn,  0);

    allocate_replicas( _resampled_codes, _num_samples );

    CodeResamplerGeneric< std::complex< float > > the_resampler;

    int iterations = 1000;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < iterations; i++)
        {
            double current_code_phase = static_cast< double >( i ) - code_spacing_chips;
            for( int j = 0; j < _num_replicas; ++j )
            {
                _desired_code_phases[j] = current_code_phase;
                current_code_phase += code_spacing_chips;
            }

            the_resampler.resample_code( _orig_code, 1023, _desired_code_phases,
                    code_phase_step, _num_samples, _resampled_codes );


        }
    delete[] _orig_code;
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    ASSERT_LE(0, end - begin);
    std::cout << "Resampling completed in " << (end - begin) << " microseconds" << std::endl;

    deallocate_replicas( _resampled_codes );

}

TEST(CodeResamplerTest, CodeResamplerIntegerChipSpacing )
{
    std::complex<float>* _orig_code = new std::complex<float>[1023];
    signed int _prn = 1;
    unsigned int _chip_shift = 4;
    unsigned int _num_replicas = 3;

    double fs = 6e6;
    double fc = 1.023e6;

    double code_phase_step = fc/fs;
    double code_spacing_chips = 0.5;

    double T = 0.001;

    int _num_samples = static_cast< int >(
            std::ceil( T * fs )
            );

    std::vector< std::complex<float>* > _resampled_codes;
    _resampled_codes.resize(_num_replicas);
    std::vector< double > _desired_code_phases;
    _desired_code_phases.resize(_num_replicas);

    gps_l1_ca_code_gen_complex( _orig_code,  _prn,  0);

    allocate_replicas( _resampled_codes, _num_samples );

    CodeResamplerIntegerChipSpacing< std::complex< float > >  the_resampler(
            boost::shared_ptr< CodeResamplerInterface< std::complex<float> > >(
                    new CodeResamplerGeneric< std::complex<float> >
                ));

    int iterations = 1000;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < iterations; i++)
        {
            double current_code_phase = static_cast< double >( i ) - code_spacing_chips;
            for( int j = 0; j < _num_replicas; ++j )
            {
                _desired_code_phases[j] = current_code_phase;
                current_code_phase += code_spacing_chips;
            }

            the_resampler.resample_code( _orig_code, 1023, _desired_code_phases,
                    code_phase_step, _num_samples, _resampled_codes );


        }
    delete[] _orig_code;
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    ASSERT_LE(0, end - begin);
    std::cout << "Resampling completed in " << (end - begin) << " microseconds" << std::endl;

    deallocate_replicas( _resampled_codes );

}

TEST(CodeResamplerTest, CodeResamplerFxpt64 )
{
    std::complex<float>* _orig_code = new std::complex<float>[1023];
    signed int _prn = 1;
    unsigned int _chip_shift = 4;
    unsigned int _num_replicas = 3;

    double fs = 6e6;
    double fc = 1.023e6;

    double code_phase_step = fc/fs;
    double code_spacing_chips = 0.5;

    double T = 0.001;

    int _num_samples = static_cast< int >(
            std::ceil( T * fs )
            );

    std::vector< std::complex<float>* > _resampled_codes;
    _resampled_codes.resize(_num_replicas);
    std::vector< double > _desired_code_phases;
    _desired_code_phases.resize(_num_replicas);

    gps_l1_ca_code_gen_complex( _orig_code,  _prn,  0);

    allocate_replicas( _resampled_codes, _num_samples );

    CodeResamplerFxpt64< std::complex< float > >  the_resampler;

    int iterations = 1000;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < iterations; i++)
        {
            double current_code_phase = static_cast< double >( i ) - code_spacing_chips;
            for( int j = 0; j < _num_replicas; ++j )
            {
                _desired_code_phases[j] = current_code_phase;
                current_code_phase += code_spacing_chips;
            }

            the_resampler.resample_code( _orig_code, 1023, _desired_code_phases,
                    code_phase_step, _num_samples, _resampled_codes );


        }
    delete[] _orig_code;
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    ASSERT_LE(0, end - begin);
    std::cout << "Resampling completed in " << (end - begin) << " microseconds" << std::endl;

    deallocate_replicas( _resampled_codes );

}

TEST(CodeResamplerTest, CodeResamplerIntegerChipSpacingFxpt64 )
{
    std::complex<float>* _orig_code = new std::complex<float>[1023];
    signed int _prn = 1;
    unsigned int _chip_shift = 4;
    unsigned int _num_replicas = 3;

    double fs = 6e6;
    double fc = 1.023e6;

    double code_phase_step = fc/fs;
    double code_spacing_chips = 0.5;

    double T = 0.001;

    int _num_samples = static_cast< int >(
            std::ceil( T * fs )
            );

    std::vector< std::complex<float>* > _resampled_codes;
    _resampled_codes.resize(_num_replicas);
    std::vector< double > _desired_code_phases;
    _desired_code_phases.resize(_num_replicas);

    gps_l1_ca_code_gen_complex( _orig_code,  _prn,  0);

    allocate_replicas( _resampled_codes, _num_samples );

    CodeResamplerIntegerChipSpacing< std::complex< float > >  the_resampler(
            boost::shared_ptr< CodeResamplerInterface< std::complex<float> > >(
                    new CodeResamplerFxpt64< std::complex<float> >
                ));

    int iterations = 1000;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < iterations; i++)
        {
            double current_code_phase = static_cast< double >( i ) - code_spacing_chips;
            for( int j = 0; j < _num_replicas; ++j )
            {
                _desired_code_phases[j] = current_code_phase;
                current_code_phase += code_spacing_chips;
            }

            the_resampler.resample_code( _orig_code, 1023, _desired_code_phases,
                    code_phase_step, _num_samples, _resampled_codes );


        }
    delete[] _orig_code;
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    ASSERT_LE(0, end - begin);
    std::cout << "Resampling completed in " << (end - begin) << " microseconds" << std::endl;

    deallocate_replicas( _resampled_codes );

}

TEST(CodeResamplerTest, CodeResamplerMemoryStore )
{
    std::complex<float>* _orig_code = new std::complex<float>[1023];
    signed int _prn = 1;
    unsigned int _chip_shift = 4;
    unsigned int _num_replicas = 3;

    double fs = 6e6;
    double fc = 1.023e6;

    double code_phase_step = fc/fs;
    double code_spacing_chips = 0.5;

    double T = 0.001;

    int _num_samples = static_cast< int >(
            std::ceil( T * fs )
            );

    std::vector< std::complex<float>* > _resampled_codes;
    _resampled_codes.resize(_num_replicas);
    std::vector< double > _desired_code_phases;
    _desired_code_phases.resize(_num_replicas);

    gps_l1_ca_code_gen_complex( _orig_code,  _prn,  0);

    allocate_replicas( _resampled_codes, _num_samples );

    CodeResamplerMemoryStore< std::complex< float > >  the_resampler(
            _orig_code, 1023, code_phase_step, _num_samples, 0.1, 1.0 );

    int iterations = 1000;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < iterations; i++)
        {
            double current_code_phase = static_cast< double >( i ) - code_spacing_chips;
            for( int j = 0; j < _num_replicas; ++j )
            {
                _desired_code_phases[j] = current_code_phase;
                current_code_phase += code_spacing_chips;
            }

            the_resampler.resample_code( _orig_code, 1023, _desired_code_phases,
                    code_phase_step, _num_samples, _resampled_codes );


        }
    delete[] _orig_code;
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    ASSERT_LE(0, end - begin);
    std::cout << "Resampling completed in " << (end - begin) << " microseconds" << std::endl;

    deallocate_replicas( _resampled_codes );

}

