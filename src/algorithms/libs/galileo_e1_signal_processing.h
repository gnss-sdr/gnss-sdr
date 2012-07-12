/*!
 * \file galileo_e1_signal_processing.h
 * \brief This library implements various functions for Galileo E1 signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
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

#ifndef GALILEO_E1_SIGNAL_PROCESSING_H_
#define GALILEO_E1_SIGNAL_PROCESSING_H_

#include <complex>
#include <iostream>
#include "Galileo_E1.h"
#include "gnss_signal_processing.h"

/*!
 * \brief This function generates Galileo E1 code (one sample per chip).
 *
 */
void galileo_e1_code_gen_int(int* _dest, char _Signal[3], signed int _prn,
        unsigned int _chip_shift);
/*!
 * \brief This function generates Galileo E1 sinboc(1,1) code (minimum 2 samples per chip),
 * the _codeLength variable must be a multiple of 2*4092.
 *
 */
void galileo_e1_sinboc_11_gen(std::complex<float>* _dest, int* _prn,
        unsigned int _codeLength);
/*!
 * \brief This function generates Galileo E1 sinboc(6,1) code (minimum 12 samples per chip),
 * the _codeLength variable must be a multiple of 12*4092.
 *
 */
void galileo_e1_sinboc_61_gen(std::complex<float>* _dest, int* _prn,
        unsigned int _codeLength);
/*!
 * \brief This function generates Galileo E1 cboc code (12 samples per chip).
 *
 */
void galileo_e1_cboc_gen(std::complex<float>* _dest, int* _prn, char _Signal[3]);
/*!
 * \brief This function generates Galileo E1 code (can select E1B or E1C, cboc or sinboc
 * and the sample frequency _fs).
 *
 */
void galileo_e1_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
        bool _cboc, unsigned int _prn, signed int _fs,
        unsigned int _chip_shift);

#endif /* GALILEO_E1_SIGNAL_PROCESSING_H_ */
