/*
 * beidou_sdr_signal_processing.h
 *
 *  Created on: Jul 15, 2015
 *      Author: giorgio
 */

#ifndef SRC_ALGORITHMS_LIBS_BEIDOU_SDR_SIGNAL_PROCESSING_H_
#define SRC_ALGORITHMS_LIBS_BEIDOU_SDR_SIGNAL_PROCESSING_H_

#include <complex>
#include <iostream>
#include "BEIDOU_B1I.h"

//!Generates complex GPS L1 C/A code for the desired SV ID and code shift, and sampled to specific sampling frequency
void beidou_b1i_code_gen_complex(std::complex<float>* _dest, signed int _prn, unsigned int _chip_shift);

//! Generates N complex GPS L1 C/A codes for the desired SV ID and code shift
void beidou_b1i_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs, unsigned int _chip_shift, unsigned int _ncodes);

//! Generates complex GPS L1 C/A code for the desired SV ID and code shift
void beidou_b1i_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs, unsigned int _chip_shift);



#endif /* SRC_ALGORITHMS_LIBS_BEIDOU_SDR_SIGNAL_PROCESSING_H_ */
