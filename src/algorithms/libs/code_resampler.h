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

#include "fxpt64.h"
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
            std::vector< double > &init_code_phase, double code_phase_step,
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
            std::vector< double > &init_code_phase, double code_phase_step,
            int num_samples,
            std::vector< T * > resampled_codes )
    {
        // Loop over the desired outputs:
        for( int i = 0; i < init_code_phase.size(); ++i )
        {
            double tcode_chips = init_code_phase[i];
            int associated_chip_index;
            T *curr_sample = resampled_codes[i];
            for( int j = 0; j < num_samples; ++j )
            {
                associated_chip_index = std::floor(std::fmod(tcode_chips , code_length_chips));
                *curr_sample = orig_code[associated_chip_index];
                tcode_chips = tcode_chips + code_phase_step_chips;
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
            std::vector< double > &init_code_phase, double code_phase_step,
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

        core_resampler->resample_code( orig_code, code_length,
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
            std::vector< double > &init_code_phase, double code_phase_step,
            int num_samples,
            std::vector< T * > resampled_codes )
    {
        int64_t code_phase_step_fxp = double_to_fxpt64( code_phase_step );

        // Loop over the desired outputs:
        for( int i = 0; i < init_code_phase.size(); ++i )
        {
            int64_t code_phase_fxp = double_to_fxpt64( init_code_phase[i], FRAC_LEN );
            T *curr_sample = resampled_codes[i];

            init_code_phase[i] = fxpt64_to_double( code_phase_fxp );

            for( int j = 0; j < num_samples; ++j )
            {
                *curr_sample = orig_code[ code_phase_fxp >> FRAC_LEN ];
                code_phase_fxp += code_phase_step_fxp;
                ++curr_sample;
            }
        }
    }
};
#endif


