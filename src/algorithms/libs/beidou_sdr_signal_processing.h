/*
 * beidou_sdr_signal_processing.h
 *
 * \author Giorgio Savastano, 2015. giorgio.savastano(at)uniroma1.it
 *
 */

#ifndef GNSS_SDR_BEIDOU_SDR_SIGNAL_PROCESSING_H_
#define GNSS_SDR_BEIDOU_SDR_SIGNAL_PROCESSING_H_

#include <complex>
#include <iostream>

//!Generates complex GPS L1 C/A code for the desired SV ID and code shift, and sampled to specific sampling frequency
void beidou_b1i_code_gen_complex(std::complex<float>* _dest, signed int _prn, unsigned int _chip_shift);

//! Generates N complex GPS L1 C/A codes for the desired SV ID and code shift
void beidou_b1i_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs, unsigned int _chip_shift, unsigned int _ncodes);

//! Generates complex GPS L1 C/A code for the desired SV ID and code shift
void beidou_b1i_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs, unsigned int _chip_shift);


#endif /* GNSS_SDR_BEIDOU_SDR_SIGNAL_PROCESSING_H_ */
