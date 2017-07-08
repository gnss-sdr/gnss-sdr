/*
 * beidou_sdr_signal_processing.h
 *
 * \author Giorgio Savastano, 2015. giorgio.savastano(at)uniroma1.it
 *
 */
#ifndef BEIDOU_SDR_SIGNAL_PROCESSING_TEST_2MS_H
#define BEIDOU_SDR_SIGNAL_PROCESSING_TEST_2MS_H


#include <complex>
#include <iostream>
#include "BEIDOU_B1I.h"

//!Generates complex GPS L1 C/A code for the desired SV ID and code shift, and sampled to specific sampling frequency
void beidou_b1i_code_gen_complex_2ms(std::complex<float> *_dest, signed int _prn, unsigned int _chip_shift);

//! Generates N complex GPS L1 C/A codes for the desired SV ID and code shift
//void beidou_b1i_code_gen_complex_sampled_2ms(std::complex<float>* _dest, unsigned int _prn, signed int _fs, unsigned int _chip_shift, unsigned int _ncodes);

//! Generates complex GPS L1 C/A code for the desired SV ID, code shift and _secondary_flag
void
beidou_b1i_code_gen_complex_sampled_2ms(std::complex<float> *_dest, char _Signal[3], unsigned int _prn, signed int _fs,
                                        unsigned int _chip_shift, bool _secondary_flag);


#endif // BEIDOU_SDR_SIGNAL_PROCESSING_TEST_2MS_H