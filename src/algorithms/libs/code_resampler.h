/*!
 * \file code_resampler.h
 * \brief Class providing a number of code resampling options
 * \authors Cillian O'Driscoll 2015. cillian.odriscoll(at)gmail.com
 *
 * Class interface for code resampling. Code resampling is characterised by
 * resampling an array that is constant over each element (called a chip).
 * The code frequency is called the chip rate, the code phase is a floating
 * point value indicating the current chip value between 0 and L -1 where
 * L is the code length
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

#ifndef GNSS_SDR_CODE_RESAMPLER_H_
#define GNSS_SDR_CODE_RESAMPLER_H_

#include <volk/volk.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <limits>
#include "fxpt64.h"
#include <cassert>

/*!
 * \brief Interface for a generic code resampler
 *
 *
 */
template< typename T >
class CodeResamplerInterface
{
public:

    /*!
     * Actually perform the code resampling.
     */
    virtual void resample_code( T const *orig_code, unsigned int code_length,
            std::vector< double > &init_code_phase, double &code_phase_step,
            int num_samples,
            std::vector< T * > resampled_codes ) = 0;
};

/*!
 * \brief Simple generic implementation of code resampling. Most accurate
 * and most expensive
 */
template< typename T >
class CodeResamplerGeneric : public CodeResamplerInterface<T>
{
public:
    /*!
     * Resample the code computing the code phases exactly for each desired
     * output.
     */
    virtual void resample_code( T const *orig_code, unsigned int code_length,
            std::vector< double > &init_code_phase, double &code_phase_step,
            int num_samples,
            std::vector< T * > resampled_codes )
    {
        // Loop over the desired outputs:
        for( int i = 0; i < init_code_phase.size(); ++i )
        {
            double tcode_chips = std::fmod( init_code_phase[i], code_length);
            if( tcode_chips < 0.0 )
            {
                tcode_chips += static_cast< double >( code_length );
            }

            int associated_chip_index;
            T *curr_sample = resampled_codes[i];

            int j = 0;

            // Here we lift the bounds checking out of the loop:
            // This gave a performance improvement of about a factor 3.3
            while( j < num_samples )
            {
                int num_samples_this_iter = num_samples;

                int num_samples_at_rollover = j + std::floor(
                        ( code_length - tcode_chips )/code_phase_step
                        )  + 1;

                if( num_samples_at_rollover < num_samples )
                {
                    num_samples_this_iter = num_samples_at_rollover;
                }


                for( ; j < num_samples_this_iter; ++j )
                {
                    associated_chip_index = static_cast< int >(tcode_chips);
                    *curr_sample = orig_code[associated_chip_index];
                    tcode_chips += code_phase_step;
                    ++curr_sample;
                }

                tcode_chips = std::fmod( tcode_chips, code_length );
            }
        }
    }
};

/*!
 * \brief Code resampler that forces the code spacing to the nearest integer
 * number of samples
 */
template< typename T >
class CodeResamplerIntegerChipSpacing : public CodeResamplerInterface<T>
{
public:

    CodeResamplerIntegerChipSpacing( boost::shared_ptr< CodeResamplerInterface<T> > core_resampler )
        : d_core_resampler( core_resampler )
    {};

    /*!
     * Resample the code computing the code phases exactly for the latest output
     * and forcing all others to be integer shifts of the latest value*/
    virtual void resample_code( T const *orig_code, unsigned int code_length,
            std::vector< double > &init_code_phase, double &code_phase_step,
            int num_samples,
            std::vector< T * > resampled_codes )
    {
        double earliest_code_phase = init_code_phase[0];
        std::vector< int > code_phase_offsets_samples( init_code_phase.size(), 0 );

        for( int i = 1; i < init_code_phase.size(); ++i )
        {
            code_phase_offsets_samples[i] = round(
                    ( init_code_phase[i] - earliest_code_phase ) / code_phase_step
                    );
        }

        int total_samples = num_samples + code_phase_offsets_samples.back();

        // Now we call the generic implementation for the earliest code phase
        // and copy for the later code phases:
        std::vector< T * > dummy_resampled_codes(1, resampled_codes[0] );
        std::vector< double > dummy_init_code_phase(1, init_code_phase[0] );

        d_core_resampler->resample_code( orig_code, code_length,
                dummy_init_code_phase, code_phase_step,
                total_samples, dummy_resampled_codes );

        // Loop over the desired outputs:
        for( int i = 1; i < init_code_phase.size(); ++i )
        {
            // Copy the shifted 'early' code to the later codes:
            memcpy( resampled_codes[i], resampled_codes[0] + code_phase_offsets_samples[i],
                    num_samples * sizeof( T ) );

            // Upate the init_code_phase parameter to reflect the true code phases
            init_code_phase[i] = init_code_phase[0] + code_phase_offsets_samples[i]*code_phase_step;
        }

    }

private:

    // A code resampler to do the basic resampling on the earliest code phase
    boost::shared_ptr< CodeResamplerInterface<T> > d_core_resampler;
};

/*!
 * \brief Code resampler that uses 64 bit fixed point arithmetic */

template< typename T, unsigned FRAC_LEN = 32  >
class CodeResamplerFxpt64 : public CodeResamplerInterface<T>
{
public:
    /*!
     * Resample the code using 64-bit fixed point arithmetic
     */
    virtual void resample_code( T const *orig_code, unsigned int code_length,
            std::vector< double > &init_code_phase, double &code_phase_step,
            int num_samples,
            std::vector< T * > resampled_codes )
    {
        int64_t code_phase_step_fxp = double_to_fxpt64( code_phase_step );
        int64_t code_length_fxp = static_cast< int64_t >( code_length ) << FRAC_LEN;

        // Loop over the desired outputs:
        for( int i = 0; i < init_code_phase.size(); ++i )
        {
            double tcode_chips = std::fmod( init_code_phase[i], code_length );
            if( tcode_chips < 0.0 )
            {
                tcode_chips += code_length;
            }

            int64_t code_phase_fxp = double_to_fxpt64( tcode_chips, FRAC_LEN );
            T *curr_sample = resampled_codes[i];
            init_code_phase[i] = fxpt64_to_double( code_phase_fxp );

            int j = 0;

            while( j < num_samples )
            {

                int num_samples_this_iter = num_samples;

                int num_samples_at_rollover = j +
                        ( code_length_fxp - code_phase_fxp ) / code_phase_step_fxp
                        + 1;

                if( num_samples_at_rollover < num_samples )
                {
                    num_samples_this_iter = num_samples_at_rollover;
                }


                for( ; j < num_samples_this_iter; ++j )
                {
                    *curr_sample = orig_code[ code_phase_fxp >> FRAC_LEN ];
                    code_phase_fxp += code_phase_step_fxp;
                    ++curr_sample;
                }

                assert( static_cast<int>(
                            fxpt64_to_double( code_phase_fxp - code_phase_step_fxp )
                            ) < code_length );

                code_phase_fxp -= ( static_cast< int64_t >( code_length ) << FRAC_LEN );
            }
        }

        code_phase_step = fxpt64_to_double( code_phase_step_fxp );
    }
};

/*!
 * \brief Code resampler that uses a fixed number of replicas in memory*/
template< typename T >
class CodeResamplerMemoryStore : public CodeResamplerInterface<T>
{
public:
    CodeResamplerMemoryStore( T const *orig_code,
            unsigned int code_length,
            double nominal_code_phase_step,
            unsigned int maximum_num_samples,
            double code_spacing_chips,
            double maximum_code_offset_chips )
        : d_nominal_code_phase_step( nominal_code_phase_step ),
        d_code_spacing_chips( code_spacing_chips ),
        d_maximum_code_offset_chips( maximum_code_offset_chips )
    {
        int num_replicas = static_cast< int >(
                std::ceil( d_maximum_code_offset_chips /
                d_code_spacing_chips ) );


        // Resize the storage:
        d_code_phase_offsets.resize( num_replicas );
        d_replica_store.resize( num_replicas );

        // Use a generic resampler to generate the local replica store:
        CodeResamplerGeneric<T> baseResampler;

        double curr_offset = 0.0;

        for( int ii = 0; ii < num_replicas; ++ii )
        {
            d_code_phase_offsets[ii] = curr_offset;
            curr_offset += d_code_spacing_chips;

            d_replica_store[ii] = static_cast< T *>( volk_malloc(
                        maximum_num_samples * sizeof( T ),
                        volk_get_alignment() ) );

        }

        // Now we generate the codes to be at least twice as long as the maximum possible
        baseResampler.resample_code( orig_code, code_length,
                d_code_phase_offsets, d_nominal_code_phase_step, 2*maximum_num_samples,
                d_replica_store );


    };

    ~CodeResamplerMemoryStore()
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
    virtual void resample_code( T const *orig_code, unsigned int code_length,
            std::vector< double > &init_code_phase, double &code_phase_step,
            int num_samples,
            std::vector< T * > resampled_codes )
    {

        // Loop over the desired outputs:
        for( int ii = 0; ii < init_code_phase.size(); ++ii )
        {
            int offset_ind = 0;
            double delta_offset = std::numeric_limits<double>::infinity();

            double desired_code_phase = init_code_phase[ii];
            double desired_code_phase_mod_one_chip = std::fmod( desired_code_phase, 1.0 );

            if( desired_code_phase_mod_one_chip < 0.0 )
            {
                desired_code_phase_mod_one_chip += 1.0;
            }

            double achieved_code_phase = 0.0;

            for( int jj = 0; jj < d_code_phase_offsets.size(); ++jj )
            {
                double abs_diff = std::abs( desired_code_phase_mod_one_chip - d_code_phase_offsets[jj] );
                if( abs_diff < delta_offset )
                {
                    delta_offset = abs_diff;
                    offset_ind = jj;
                    achieved_code_phase = desired_code_phase - desired_code_phase_mod_one_chip
                        + d_code_phase_offsets[jj];
                }
            }

            init_code_phase[ii] = achieved_code_phase;

            int integer_offset = static_cast< int >( desired_code_phase - desired_code_phase_mod_one_chip );

            memcpy( resampled_codes[ii], d_replica_store[offset_ind] + integer_offset*sizeof(T),
                    num_samples * sizeof( T ) );

        }

        code_phase_step = d_nominal_code_phase_step;
    }

private:

    double d_nominal_code_phase_step;

    double d_maximum_code_offset_chips;

    double d_code_spacing_chips;

    std::vector< double > d_code_phase_offsets;

    std::vector< T * > d_replica_store;



};

#endif

