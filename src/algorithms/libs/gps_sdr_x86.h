#include "gps_sdr_defines.h"
#include "gps_sdr_signal_processing.h"
#include "gr_complex.h"

void  x86_add(int16 *A, int16 *B, int32 cnt);	//!< Pointwise vector addition
void  x86_sub(int16 *A, int16 *B, int32 cnt);	//!< Pointwise vector difference
void  x86_mul(int16 *A, int16 *B, int32 cnt);	//!< Pointwise vector multiply
void  x86_muls(int16 *A, int16 *B, int32 cnt, int32 shift);	//!< Pointwise vector multiply
int32 x86_dot(int16 *A, int16 *B, int32 cnt);	//!< Compute vector dot product

void  x86_conj(CPX *A, int32 cnt);											//!< Pointwise vector conjugate
void  x86_cacc(CPX *A, MIX *B, int32 cnt, int32 *iaccum, int32 *baccum);	//!< Compute dot product of cpx and a mix vector
void  x86_cmul(CPX *A, CPX *B, int32 cnt);									//!< Pointwise vector multiply
void  x86_cmuls(CPX *A, CPX *B, int32 cnt, int32 shift);					//!< Pointwise complex multiply with shift
void  x86_cmulsc(CPX *A, CPX *B, CPX *C, int32 cnt, int32 shift);			//!< Pointwise vector multiply with shift, dump results into C
void  x86_cmag(CPX *A, int32 cnt);											//!< Convert from complex to a power
void  x86_prn_accum(CPX *A, CPX *E, CPX *P, CPX *L, int32 cnt, CPX *accum);  //!< This is a long story
void  x86_prn_accum_new(CPX *A, MIX *E, MIX *P, MIX *L, int32 cnt, CPX_ACCUM *accum);  //!< This is a long story
void  x86_max(unsigned int *_A, unsigned int *_index, unsigned int *_magt, unsigned int _cnt);
void  x86_float_max(float* _A, unsigned int* _index, float* _magt, unsigned int _cnt);
void  x86_gr_complex_mag(gr_complex* _A, int32 _cnt);
