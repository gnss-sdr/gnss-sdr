#ifndef GPS_SDR_SIGNAL_PROCESSING_H_
#define GPS_SDR_SIGNAL_PROCESSING_H_

#include "gps_sdr_defines.h"

#include <complex>
#include <iostream>

/*
 * Signal processing functions from gps-sdr.
 */

/*----------------------------------------------------------------------------------------------*/
/*! @ingroup STRUCTS
 *  @brief Define the CPX structure, used in the Fine_Acquisition FFT */
typedef struct CPX
{
	int16 i;	//!< Real value
	int16 q;	//!< Imaginary value
} CPX;

/*! \ingroup STRUCTS
 *	@brief Format of complex accumulations */
typedef struct CPX_ACCUM {

	int32 i;	//!< Inphase (real)
	int32 q;	//!< Quadrature (imaginary)

} CPX_ACCUM;


/*! \ingroup STRUCTS
 *
 */
typedef struct MIX {

	int16 i;	//!< Inphase (real)
	int16 nq;	//!< Quadrature (imaginary)
	int16 q;	//!< Quadrature (imaginary)
	int16 ni;	//!< Inphase (real)

} MIX;
/*----------------------------------------------------------------------------------------------*/

void code_gen_conplex(std::complex<float>* _dest, int32 _prn, unsigned int _chip_shift);
void code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, int32 _fs, unsigned int _chip_shift);
int32 code_gen(CPX *_dest, int32 _prn);
void sine_gen(CPX *_dest, double _f, double _fs, int32 _samps);
void sine_gen_complex(std::complex<float>* _dest, double _f, double _fs, unsigned int _samps);
void sine_gen(CPX *_dest, double _f, double _fs, int32 _samps, double _p);
void wipeoff_gen(MIX *_dest, double _f, double _fs, int32 _samps);
void downsample(CPX *_dest, CPX *_source, double _fdest, double _fsource, int32 _samps);
void init_agc(CPX *_buff, int32 _samps, int32 bits, int32 *scale);
int32 run_agc(CPX *_buff, int32 _samps, int32 bits, int32 scale);

#endif /* GPS_SDR_SIGNAL_PROCESSING_H_ */
