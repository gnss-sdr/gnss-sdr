/*!
 * \file subcarrier_resampler_test.cc
 * \brief  This file implements tests for resampling subcarriers
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


#include "subcarrier_resampler.h"
#include <complex>
#include <ctime>
#include <sys/time.h>
#include "gps_sdr_signal_processing.h"
#include "gnss_signal_processing.h"

#include <gtest/gtest.h>

void allocate_subcarrier_replicas( std::vector< std::complex< float > * >& replicas, int num_samples )
{

    for( int i = 0;  i < replicas.size(); ++i )
    {
        replicas[i] = static_cast< std::complex<float> *>(
                volk_malloc( num_samples*sizeof( std::complex<float> ),
                    volk_get_alignment() )
                );
    }
}

void deallocate_subcarrier_replicas( std::vector< std::complex< float > * > &replicas )
{
    for( int i = 0; i < replicas.size(); ++i )
    {
        volk_free( replicas[i] );
    }
}

TEST(SubcarrierResamplerTest, SubcarrierResamplerGeneric )
{
    unsigned int _num_replicas = 3;

    double fs = 12e6;
    double fsub = 6*1.023e6;

    double subcarrier_phase_step = fsub/fs;
    double subcarrier_spacing_cycles = 0.125;

    double T = 0.001;

    int _num_samples = static_cast< int >(
            std::ceil( T * fs )
            );

    std::vector< std::complex<float>* > _resampled_subcarriers;
    _resampled_subcarriers.resize(_num_replicas);
    std::vector< double > _desired_subcarrier_phases;
    _desired_subcarrier_phases.resize(_num_replicas);

    allocate_subcarrier_replicas( _resampled_subcarriers, _num_samples );

    SubcarrierResamplerGeneric< std::complex< float > > the_resampler;

    int iterations = 1000;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < iterations; i++)
        {
            double current_subcarrier_phase = static_cast< double >( i ) - subcarrier_spacing_cycles;
            for( int j = 0; j < _num_replicas; ++j )
            {
                _desired_subcarrier_phases[j] = current_subcarrier_phase;
                current_subcarrier_phase += subcarrier_spacing_cycles;
            }

            the_resampler.resample_subcarrier( _desired_subcarrier_phases,
                    subcarrier_phase_step, _num_samples, _resampled_subcarriers,
                    i % 2 == 1 );


        }
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    ASSERT_LE(0, end - begin);
    std::cout << "Resampling completed in " << (end - begin) << " microseconds" << std::endl;

    deallocate_subcarrier_replicas( _resampled_subcarriers );

}

TEST(SubcarrierResamplerTest, SubcarrierResamplerIntegerSampleSpacing )
{
    std::complex<float>* _orig_subcarrier = new std::complex<float>[1023];
    signed int _prn = 1;
    unsigned int _chip_shift = 4;
    unsigned int _num_replicas = 3;

    double fs = 6e6;
    double fc = 1.023e6;

    double subcarrier_phase_step = fc/fs;
    double subcarrier_spacing_cycles = 0.5;

    double T = 0.001;

    int _num_samples = static_cast< int >(
            std::ceil( T * fs )
            );

    std::vector< std::complex<float>* > _resampled_subcarriers;
    _resampled_subcarriers.resize(_num_replicas);
    std::vector< double > _desired_subcarrier_phases;
    _desired_subcarrier_phases.resize(_num_replicas);

    allocate_subcarrier_replicas( _resampled_subcarriers, _num_samples );

    SubcarrierResamplerIntegerSampleSpacing< std::complex< float > >  the_resampler(
            boost::shared_ptr< SubcarrierResamplerInterface< std::complex<float> > >(
                    new SubcarrierResamplerGeneric< std::complex<float> >
                ));

    int iterations = 1000;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < iterations; i++)
        {
            double current_subcarrier_phase = static_cast< double >( i ) - subcarrier_spacing_cycles;
            for( int j = 0; j < _num_replicas; ++j )
            {
                _desired_subcarrier_phases[j] = current_subcarrier_phase;
                current_subcarrier_phase += subcarrier_spacing_cycles;
            }

            the_resampler.resample_subcarrier( _desired_subcarrier_phases,
                    subcarrier_phase_step, _num_samples, _resampled_subcarriers,
                    i % 2 == 1 );


        }
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    ASSERT_LE(0, end - begin);
    std::cout << "Resampling completed in " << (end - begin) << " microseconds" << std::endl;

    deallocate_subcarrier_replicas( _resampled_subcarriers );

}

TEST(SubcarrierResamplerTest, SubcarrierResamplerFxpt64 )
{
    std::complex<float>* _orig_subcarrier = new std::complex<float>[1023];
    signed int _prn = 1;
    unsigned int _chip_shift = 4;
    unsigned int _num_replicas = 3;

    double fs = 6e6;
    double fc = 1.023e6;

    double subcarrier_phase_step = fc/fs;
    double subcarrier_spacing_cycles = 0.5;

    double T = 0.001;

    int _num_samples = static_cast< int >(
            std::ceil( T * fs )
            );

    std::vector< std::complex<float>* > _resampled_subcarriers;
    _resampled_subcarriers.resize(_num_replicas);
    std::vector< double > _desired_subcarrier_phases;
    _desired_subcarrier_phases.resize(_num_replicas);

    allocate_subcarrier_replicas( _resampled_subcarriers, _num_samples );

    SubcarrierResamplerFxpt64< std::complex< float > >  the_resampler;

    int iterations = 1000;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < iterations; i++)
        {
            double current_subcarrier_phase = static_cast< double >( i ) - subcarrier_spacing_cycles;
            for( int j = 0; j < _num_replicas; ++j )
            {
                _desired_subcarrier_phases[j] = current_subcarrier_phase;
                current_subcarrier_phase += subcarrier_spacing_cycles;
            }

            the_resampler.resample_subcarrier( _desired_subcarrier_phases,
                    subcarrier_phase_step, _num_samples, _resampled_subcarriers,
                    i % 2 == 1 );


        }
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    ASSERT_LE(0, end - begin);
    std::cout << "Resampling completed in " << (end - begin) << " microseconds" << std::endl;

    deallocate_subcarrier_replicas( _resampled_subcarriers );

}

TEST(SubcarrierResamplerTest, SubcarrierResamplerIntegerSampleSpacingFxpt64 )
{
    std::complex<float>* _orig_subcarrier = new std::complex<float>[1023];
    signed int _prn = 1;
    unsigned int _chip_shift = 4;
    unsigned int _num_replicas = 3;

    double fs = 6e6;
    double fc = 1.023e6;

    double subcarrier_phase_step = fc/fs;
    double subcarrier_spacing_cycles = 0.5;

    double T = 0.001;

    int _num_samples = static_cast< int >(
            std::ceil( T * fs )
            );

    std::vector< std::complex<float>* > _resampled_subcarriers;
    _resampled_subcarriers.resize(_num_replicas);
    std::vector< double > _desired_subcarrier_phases;
    _desired_subcarrier_phases.resize(_num_replicas);

    allocate_subcarrier_replicas( _resampled_subcarriers, _num_samples );

    SubcarrierResamplerIntegerSampleSpacing< std::complex< float > >  the_resampler(
            boost::shared_ptr< SubcarrierResamplerInterface< std::complex<float> > >(
                    new SubcarrierResamplerFxpt64< std::complex<float> >
                ));

    int iterations = 1000;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < iterations; i++)
        {
            double current_subcarrier_phase = static_cast< double >( i ) - subcarrier_spacing_cycles;
            for( int j = 0; j < _num_replicas; ++j )
            {
                _desired_subcarrier_phases[j] = current_subcarrier_phase;
                current_subcarrier_phase += subcarrier_spacing_cycles;
            }

            the_resampler.resample_subcarrier( _desired_subcarrier_phases,
                    subcarrier_phase_step, _num_samples, _resampled_subcarriers,
                    i % 2 == 1 );


        }
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    ASSERT_LE(0, end - begin);
    std::cout << "Resampling completed in " << (end - begin) << " microseconds" << std::endl;

    deallocate_subcarrier_replicas( _resampled_subcarriers );

}

TEST(SubcarrierResamplerTest, SubcarrierResamplerMemoryStore )
{
    std::complex<float>* _orig_subcarrier = new std::complex<float>[1023];
    signed int _prn = 1;
    unsigned int _chip_shift = 4;
    unsigned int _num_replicas = 3;

    double fs = 6e6;
    double fc = 1.023e6;

    double subcarrier_phase_step = fc/fs;
    double subcarrier_spacing_cycles = 0.5;

    double T = 0.001;

    int _num_samples = static_cast< int >(
            std::ceil( T * fs )
            );

    std::vector< std::complex<float>* > _resampled_subcarriers;
    _resampled_subcarriers.resize(_num_replicas);
    std::vector< double > _desired_subcarrier_phases;
    _desired_subcarrier_phases.resize(_num_replicas);

    allocate_subcarrier_replicas( _resampled_subcarriers, _num_samples );

    SubcarrierResamplerMemoryStore< std::complex< float > >  the_resampler(
            subcarrier_phase_step, _num_samples, 0.1  );

    int iterations = 1000;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < iterations; i++)
        {
            double current_subcarrier_phase = static_cast< double >( i ) - subcarrier_spacing_cycles;
            for( int j = 0; j < _num_replicas; ++j )
            {
                _desired_subcarrier_phases[j] = current_subcarrier_phase;
                current_subcarrier_phase += subcarrier_spacing_cycles;
            }

            the_resampler.resample_subcarrier( _desired_subcarrier_phases,
                    subcarrier_phase_step, _num_samples, _resampled_subcarriers,
                    i % 2 == 1 );


        }
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    ASSERT_LE(0, end - begin);
    std::cout << "Resampling completed in " << (end - begin) << " microseconds" << std::endl;

    deallocate_subcarrier_replicas( _resampled_subcarriers );

}


