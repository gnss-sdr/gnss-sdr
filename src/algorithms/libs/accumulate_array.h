/*!
 * \file accumulate_array.h
 * \brief Class providing functionality similar to Matlab's accumarray
 * \authors Cillian O'Driscoll 2015. cillian.odriscoll(at)gmail.com
 *
 * This function accumulates data from an input vector, into an output
 * vector, according to the indices provided by an index vector.
 *
 * e.g.
 *
 *  Input:
 *      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12
 *
 *  Index Vec:
 *      0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 3, 3
 *
 *  Output:
 *    Before:
 *      0,  0,  0,  0, 0
 *    After:
 *      3, 18, 15, 42, 0
 *
 * Note accumulate_array does not zero the output vector
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

#ifndef GNSS_SDR_ACCUMULATE_ARRAY_H_
#define GNSS_SDR_ACCUMULATE_ARRAY_H_

template< typename T >
void accumulate_array( T *input_vec, int *index_vec, T *output_vec, int num_samples )
{
    int *curr_index = index_vec;
    T *curr_input = input_vec;

    for( unsigned int ii = 0; ii < num_samples; ++ii )
    {
        output_vec[ *curr_index++ ] += *curr_input++;
    }
}

#endif

