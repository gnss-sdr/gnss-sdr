/*!
 * \file galileo_e5_signal_processing.cc
 * \brief This library implements various functions for Galileo E5 signals such
 * as replica code generation
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_E5_SIGNAL_PROCESSING_H_
#define GNSS_SDR_GALILEO_E5_SIGNAL_PROCESSING_H_

#include <complex>
#include <cstdint>


/*!
 * \brief Generates Galileo E5a code at 1 sample/chip
 * bool _pilot generates E5aQ code if true and E5aI (data signal) if false.
 */
void galileo_e5_a_code_gen_complex_primary(std::complex<float>* _dest, int32_t _prn, char _Signal[3]);

void galileo_e5_a_code_gen_tiered(std::complex<float>* _dest, std::complex<float>* _primary, uint32_t _prn, char _Signal[3]);

/*!
 * \brief Generates Galileo E5a complex code, shifted to the desired chip and sampled at a frequency fs
 * bool _pilot generates E5aQ code if true and E5aI (data signal) if false.
 */
void galileo_e5_a_code_gen_complex_sampled(std::complex<float>* _dest,
    char _Signal[3], uint32_t _prn, int32_t _fs, uint32_t _chip_shift);


#endif /* GNSS_SDR_GALILEO_E5_SIGNAL_PROCESSING_H_ */
