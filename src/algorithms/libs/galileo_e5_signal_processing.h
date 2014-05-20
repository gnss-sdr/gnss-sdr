/*
 * galileo_e5_signal_processing.h
 *
 *  Created on: May 20, 2014
 *      Author: marc
 */

#ifndef GALILEO_E5_SIGNAL_PROCESSING_H_
#define GALILEO_E5_SIGNAL_PROCESSING_H_

#include <complex>
#include <iostream>
#include <gnuradio/math.h>
#include "Galileo_E5a.h"
#include "gnss_signal_processing.h"

/*!
 * \brief Generates Galileo E5a code at 1 sample/chip
 * bool _pilot generates E5aQ code if true and E5aI (data signal) if false.
 */
void galileo_e5_a_code_gen_complex(std::complex<float>* _dest, signed int _prn, bool _pilot);

/*!
 * \brief Generates Galileo E5a complex code, shifted to the desired chip and sampled at a frequency fs
 * bool _pilot generates E5aQ code if true and E5aI (data signal) if false.
 */
void galileo_e5_a_code_gen_complex_sampled(std::complex<float>* _dest,
        bool _pilot, unsigned int _prn, signed int _fs, unsigned int _chip_shift,
        bool _secondary_flag);


#endif /* GALILEO_E5_SIGNAL_PROCESSING_H_ */
