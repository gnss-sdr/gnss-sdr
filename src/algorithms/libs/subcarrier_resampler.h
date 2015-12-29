/*!
 * \file subcarrier_resampler.h
 * \brief Class providing a number of subcarrier resampling options
 * \authors Cillian O'Driscoll 2015. cillian.odriscoll(at)gmail.com
 *
 * Class interface for subcarrier resampling.
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

#ifndef GNSS_SDR_SUBCARRIER_RESAMPLER_H_
#define GNSS_SDR_SUBCARRIER_RESAMPLER_H_

#include <volk/volk.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <limits>
#include "fxpt64.h"
#include <cassert>
#include <cstring>


/*!
 * \brief Interface for a generic subcarrier resampler
 *
 *
 */
template< typename T >
class SubcarrierResamplerInterface
{
public:

    /*!
     * Actually perform the code resampling.
     */
    virtual void resample_subcarrier( std::vector< double > &init_subcarrier_phase_cycles,
            double &subcarrier_phase_step,
            int num_samples,
            std::vector< T * > resampled_subcarriers,
            bool is_cosine_phased = false) = 0;

protected:

    static const std::vector< T > d_subcarrier_table;
};

template< typename T >
const std::vector< T > SubcarrierResamplerInterface< T >::d_subcarrier_table =
    { static_cast< T >( 1.0 ), static_cast<T >( -1.0 ) };

/*!
 * \brief Simple generic implementation of subcarrier resampling. Most accurate
 * and most expensive
 */
template< typename T >
class SubcarrierResamplerGeneric : public SubcarrierResamplerInterface<T>
{
public:
    /*!
     * Resample the subcarrier computing the subcarrier phases exactly for each desired
     * output.
     */
    virtual void resample_subcarrier( std::vector< double > &init_subcarrier_phase_cycles,
            double &subcarrier_phase_step,
            int num_samples,
            std::vector< T * > resampled_subcarriers,
            bool is_cosine_phased = false)
    {
        double subcarrier_phase_step_halfcycles = 2.0 * subcarrier_phase_step;

        // Loop over the desired outputs:
        for( int i = 0; i < init_subcarrier_phase_cycles.size(); ++i )
        {
            double tsubcarrier_halfcycles = std::fmod( 2.0*init_subcarrier_phase_cycles[i], 2.0);
            if( tsubcarrier_halfcycles < 0.0 )
            {
                tsubcarrier_halfcycles += 2.0;
            }

            if( is_cosine_phased )
            {
                tsubcarrier_halfcycles += 0.5; // add a 1/4 cycle for cosine phasing
            }

            T *curr_sample = resampled_subcarriers[i];

            int64_t int_phase;
            for( int j = 0; j < num_samples; ++j )
            {
                int_phase = static_cast< int64_t >( tsubcarrier_halfcycles );
                //*curr_sample = static_cast< T >( 1.0 - 2.0 *( int_phase & 0x01 ) );
                *curr_sample = this->d_subcarrier_table[ int_phase & 0x01 ];
                tsubcarrier_halfcycles += subcarrier_phase_step_halfcycles;
                ++curr_sample;
            }

        }
    }
};

/*!
 * \brief Code resampler that forces the code spacing to the nearest integer
 * number of samples
 */
template< typename T >
class SubcarrierResamplerIntegerSampleSpacing : public SubcarrierResamplerInterface<T>
{
public:

    SubcarrierResamplerIntegerSampleSpacing( boost::shared_ptr< SubcarrierResamplerInterface<T> > core_resampler )
        : d_core_resampler( core_resampler )
    {};

    /*!
     * Resample the subcarrier computing the subcarrier phases exactly for the latest output
     * and forcing all others to be integer shifts of the latest value*/
    virtual void resample_subcarrier( std::vector< double > &init_subcarrier_phase_cycles,
            double &subcarrier_phase_step,
            int num_samples,
            std::vector< T * > resampled_subcarriers,
            bool is_cosine_phased = false)
    {
        double earliest_subcarrier_phase = init_subcarrier_phase_cycles[0];
        std::vector< int > subcarrier_phase_offsets_samples( init_subcarrier_phase_cycles.size(), 0 );

        for( int i = 1; i < init_subcarrier_phase_cycles.size(); ++i )
        {
            subcarrier_phase_offsets_samples[i] = round(
                    ( init_subcarrier_phase_cycles[i] - earliest_subcarrier_phase ) / subcarrier_phase_step
                    );
        }

        int total_samples = num_samples + subcarrier_phase_offsets_samples.back();

        // Now we call the generic implementation for the earliest code phase
        // and copy for the later code phases:
        std::vector< T * > dummy_resampled_subcarriers(1, resampled_subcarriers[0] );
        std::vector< double > dummy_init_subcarrier_phase(1, init_subcarrier_phase_cycles[0] );

        d_core_resampler->resample_subcarrier(
                dummy_init_subcarrier_phase, subcarrier_phase_step,
                total_samples, dummy_resampled_subcarriers, is_cosine_phased );

        // Loop over the desired outputs:
        for( int i = 1; i < init_subcarrier_phase_cycles.size(); ++i )
        {
            // Copy the shifted 'early' code to the later codes:
            std::memcpy( resampled_subcarriers[i], resampled_subcarriers[0] +
                    subcarrier_phase_offsets_samples[i],
                    num_samples * sizeof( T ) );

            // Upate the init_subcarrier_phase parameter to reflect the true code phases
            init_subcarrier_phase_cycles[i] = init_subcarrier_phase_cycles[0] +
                subcarrier_phase_offsets_samples[i]*subcarrier_phase_step;
        }

    }

private:

    // A code resampler to do the basic resampling on the earliest code phase
    boost::shared_ptr< SubcarrierResamplerInterface<T> > d_core_resampler;
};

/*!
 * \brief Code resampler that uses 64 bit fixed point arithmetic */

template< typename T, unsigned FRAC_LEN = 32  >
class SubcarrierResamplerFxpt64 : public SubcarrierResamplerInterface<T>
{
public:
    /*!
     * Resample the code using 64-bit fixed point arithmetic
     */
    virtual void resample_subcarrier( std::vector< double > &init_subcarrier_phase_cycles,
            double &subcarrier_phase_step,
            int num_samples,
            std::vector< T * > resampled_subcarriers,
            bool is_cosine_phased = false)
    {
        int64_t subcarrier_phase_step_fxp = double_to_fxpt64( 2.0*subcarrier_phase_step );

        // Loop over the desired outputs:
        for( int i = 0; i < init_subcarrier_phase_cycles.size(); ++i )
        {
            double tsubcarrier_halfcycles = std::fmod( 2.0*init_subcarrier_phase_cycles[i], 2 );
            if( tsubcarrier_halfcycles < 0.0 )
            {
                tsubcarrier_halfcycles += 2.0;
            }

            if( is_cosine_phased )
            {
                tsubcarrier_halfcycles += 0.5;
            }

            int64_t subcarrier_phase_fxp = double_to_fxpt64( tsubcarrier_halfcycles, FRAC_LEN );
            T *curr_sample = resampled_subcarriers[i];
            init_subcarrier_phase_cycles[i] = fxpt64_to_double( subcarrier_phase_fxp )/2.0;


            for( int j = 0 ; j < num_samples; ++j )
            {
                //*curr_sample = static_cast< T >( 1 - 2*( (subcarrier_phase_fxp>>FRAC_LEN) & 0x01 ) );
                *curr_sample = this->d_subcarrier_table[ ( subcarrier_phase_fxp >> FRAC_LEN ) & 0x01 ];
                subcarrier_phase_fxp += subcarrier_phase_step_fxp;
                ++curr_sample;
            }

        }

        subcarrier_phase_step = fxpt64_to_double( subcarrier_phase_step_fxp ) / 2.0;
    }
};

/*!
 * \brief Code resampler that uses a fixed number of replicas in memory*/
template< typename T >
class SubcarrierResamplerMemoryStore : public SubcarrierResamplerInterface<T>
{
public:
    SubcarrierResamplerMemoryStore(
            double nominal_subcarrier_phase_step,
            unsigned int maximum_num_samples,
            double subcarrier_spacing_cycles )
        : d_nominal_subcarrier_phase_step( nominal_subcarrier_phase_step ),
        d_subcarrier_spacing_cycles( subcarrier_spacing_cycles )
    {
        int num_replicas = static_cast< int >(
                std::ceil( 1.0 /
                d_subcarrier_spacing_cycles ) );


        // Resize the storage:
        d_subcarrier_phase_offsets.resize( num_replicas );
        d_replica_store.resize( num_replicas );

        // Use a generic resampler to generate the local replica store:
        SubcarrierResamplerGeneric<T> baseResampler;

        double curr_offset = 0.0;

        for( int ii = 0; ii < num_replicas; ++ii )
        {
            d_subcarrier_phase_offsets[ii] = curr_offset;
            curr_offset += d_subcarrier_spacing_cycles;

            d_replica_store[ii] = static_cast< T *>( volk_malloc(
                        2*maximum_num_samples * sizeof( T ),
                        volk_get_alignment() ) );

        }

        // Now we generate the codes to be at least twice as long as the maximum possible
        baseResampler.resample_subcarrier(
                d_subcarrier_phase_offsets, d_nominal_subcarrier_phase_step, 2*maximum_num_samples,
                d_replica_store, false );


    };

    ~SubcarrierResamplerMemoryStore()
    {
        for( unsigned int ii = 0; ii < d_replica_store.size(); ++ii )
        {
            volk_free( d_replica_store[ii] );
        }
    }

    /*!
     * Resample the code by choosing the local replicas that most closely match
     * the code phase offsets required
     */
    virtual void resample_subcarrier( std::vector< double > &init_subcarrier_phase_cycles,
            double &subcarrier_phase_step,
            int num_samples,
            std::vector< T * > resampled_subcarriers,
            bool is_cosine_phased = false)
    {

        // Loop over the desired outputs:
        for( int ii = 0; ii < init_subcarrier_phase_cycles.size(); ++ii )
        {
            int offset_ind = 0;
            double delta_offset = std::numeric_limits<double>::infinity();

            double desired_subcarrier_phase = init_subcarrier_phase_cycles[ii];

            if( is_cosine_phased )
            {
                desired_subcarrier_phase += 0.25;
            }

            double desired_subcarrier_phase_mod_one_chip = std::fmod( desired_subcarrier_phase, 1.0 );

            if( desired_subcarrier_phase_mod_one_chip < 0.0 )
            {
                desired_subcarrier_phase_mod_one_chip += 1.0;
            }

            double achieved_subcarrier_phase = 0.0;

            for( int jj = 0; jj < d_subcarrier_phase_offsets.size(); ++jj )
            {
                double abs_diff = std::abs( desired_subcarrier_phase_mod_one_chip - d_subcarrier_phase_offsets[jj] );
                if( abs_diff < delta_offset )
                {
                    delta_offset = abs_diff;
                    offset_ind = jj;
                    achieved_subcarrier_phase = desired_subcarrier_phase - desired_subcarrier_phase_mod_one_chip
                        + d_subcarrier_phase_offsets[jj];
                }
            }

            if( is_cosine_phased )
            {
                achieved_subcarrier_phase -= 0.25;
            }

            init_subcarrier_phase_cycles[ii] = achieved_subcarrier_phase;


            std::memcpy( resampled_subcarriers[ii], d_replica_store[offset_ind],
                    num_samples * sizeof( T ) );

        }

        subcarrier_phase_step = d_nominal_subcarrier_phase_step;
    }

private:

    double d_nominal_subcarrier_phase_step;


    double d_subcarrier_spacing_cycles;

    std::vector< double > d_subcarrier_phase_offsets;

    std::vector< T * > d_replica_store;

};

#endif


