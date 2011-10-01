/*! \file x86.cpp
	Emulate SIMD functionality with plain x86 crap, for older processors
*/
/************************************************************************************************
Copyright 2008 Gregory W Heckler

This file is part of the GPS Software Defined Radio (GPS-SDR)

The GPS-SDR is free software; you can redistribute it and/or modify it under the terms of the
GNU General Public License as published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The GPS-SDR is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along with GPS-SDR; if not,
write to the:

Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
************************************************************************************************/

#include "gps_sdr_x86.h"
#include "gr_complex.h"
#include <complex>

/*----------------------------------------------------------------------------------------------*/
void x86_add(int16 *_A, int16 *_B, int32 _cnt)
{

	int16 *a = _A;
	int16 *b = _B;
	int32 lcv;

	for(lcv = 0; lcv < _cnt; lcv++)
		a[lcv] += b[lcv];

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void x86_sub(int16 *_A, int16 *_B, int32 _cnt)
{

	int16 *a = _A;
	int16 *b = _B;
	int32 lcv;

	for(lcv = 0; lcv < _cnt; lcv++)
		a[lcv] -= b[lcv];
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void x86_mul(int16 *_A, int16 *_B, int32 _cnt)
{

	int16 *a = _A;
	int16 *b = _B;
	int32 lcv;

	for(lcv = 0; lcv < _cnt; lcv++)
		a[lcv] *= b[lcv];
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void x86_muls(int16 *_A, int16 *_B, int32 _cnt, int32 _shift)
{

	int32 lcv;

	for(lcv = 0; lcv < _cnt; lcv++)
	{
		_A[lcv] *= *_B;
		_A[lcv] >>= _shift;
	}
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
int32 x86_dot(int16 *_A, int16 *_B, int32 _cnt)
{

	int32 accum = 0;
	int16 *a = _A;
	int16 *b = _B;
	int32 lcv;

	for(lcv = 0; lcv < _cnt; lcv++)
		accum += (int32)a[lcv]*(int32)b[lcv];

	return(accum);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void x86_conj(CPX *_A, int32 _cnt)
{
	int32 lcv;

	for(lcv = 0; lcv < _cnt; lcv++)
		_A[lcv].q = -_A[lcv].q;

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void x86_cmul(CPX *_A, CPX *_B, int32 _cnt)
{

	int32 lcv;
	int32 ai, aq;
	int32 bi, bq;
	int32 ti, tq;

	for(lcv = 0; lcv < _cnt; lcv++)
	{

		ai = _A[lcv].i;
		aq = _A[lcv].q;
		bi = _B[lcv].i;
		bq = _B[lcv].q;

		ti = ai*bi-aq*bq;
		tq = ai*bq+aq*bi;

		_A[lcv].i = (int16)(ti);
		_A[lcv].q = (int16)(tq);
	}

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void x86_cmuls(CPX *_A, CPX *_B, int32 _cnt, int32 _shift)
{

	int32 lcv;
	int32 ai, aq;
	int32 bi, bq;
	int32 ti, tq;
	int32 shift;
	int32 round;

	shift = _shift;
	round = 1 << (shift-1);

	for(lcv = 0; lcv < _cnt; lcv++)
	{

		ai = _A[lcv].i;
		aq = _A[lcv].q;
		bi = _B[lcv].i;
		bq = _B[lcv].q;

		ti = ai*bi-aq*bq;
		tq = ai*bq+aq*bi;

		ti += round;
		tq += round;

		ti >>= shift;
		tq >>= shift;

		_A[lcv].i = (int16)ti;
		_A[lcv].q = (int16)tq;
	}

}
/*----------------------------------------------------------------------------------------------*/

void x86_cmulsc_complex(std::complex<float> *_A, std::complex<float> *_B, std::complex<float> *_C, int32 _cnt) {

	for(int i = 0; i < _cnt; i++) {

		_C[i] = _A[i] * _B[i];
	}

}

/*----------------------------------------------------------------------------------------------*/
void x86_cmulsc(CPX *_A, CPX *_B, CPX *_C, int32 _cnt, int32 _shift)
{

	int32 lcv;
	int32 ai, aq;
	int32 bi, bq;
	int32 ti, tq;
	int32 shift;
	int32 round;

	shift = _shift;
	round = 1 << (shift-1);

	for(lcv = 0; lcv < _cnt; lcv++)
	{

		ai = _A[lcv].i;
		aq = _A[lcv].q;
		bi = _B[lcv].i;
		bq = _B[lcv].q;

		ti = ai*bi-aq*bq;
		tq = ai*bq+aq*bi;

		ti += round;
		tq += round;

		ti >>= shift;
		tq >>= shift;

		_C[lcv].i = (int16)ti;
		_C[lcv].q = (int16)tq;
	}

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void x86_cacc(CPX *_A, MIX *_B, int32 _cnt, int32 *_iaccum, int32 *_qaccum)
{
	int32 lcv;
	int32 ai, aq;
	int32 ti, tq;
	int32 iaccum;
	int32 qaccum;

	iaccum = qaccum = 0;

	for(lcv = 0; lcv < _cnt; lcv++)
	{

		ai = _A[lcv].i;
		aq = _A[lcv].q;

		ti = ai*_B[lcv].i+aq*_B[lcv].nq;
		tq = ai*_B[lcv].q+aq*_B[lcv].ni;

		iaccum += ti;
		qaccum += tq;

	}

	*_iaccum = iaccum;
	*_qaccum = qaccum;

}
/*----------------------------------------------------------------------------------------------*/
// _A=|abs(·)|^2 overwrite the input array
void x86_gr_complex_mag(gr_complex* _A, int32 _cnt) {
	float* p = (float*)_A;

	for(int i=0;i<_cnt;i++) {
		*p = _A[i].real()*_A[i].real() + _A[i].imag()*_A[i].imag();
		p++;
	}
}

/*----------------------------------------------------------------------------------------------*/
void x86_cmag(CPX *_A, int32 _cnt)
{

	int32 lcv, *p;

	p = (int32 *)_A;

	for(lcv = 0; lcv < _cnt; lcv++)
	{
		*p = _A[lcv].i*_A[lcv].i + _A[lcv].q*_A[lcv].q;
		p++;
	}


}
/*----------------------------------------------------------------------------------------------*/
// Find the maximum float in _A[·]
void x86_float_max(float* _A, unsigned int* _index, float* _magt, unsigned int _cnt) {

	unsigned int index;
	float mag;

	mag = index = 0;

	for(int i=0; i<_cnt; i++) {
		if(_A[i] > mag) {
			index = i;
			mag = _A[i];
		}
	}

	*_index = index;
	*_magt = mag;
}

/*----------------------------------------------------------------------------------------------*/
void x86_max(unsigned int *_A, unsigned int *_index, unsigned int *_magt, unsigned int _cnt)
{

	unsigned int lcv, mag, index;

	mag = index = 0;

	for(lcv = 0; lcv < _cnt; lcv++)
	{
		if(_A[lcv] > mag)
		{
			index = lcv;
			mag = _A[lcv];
		}
	}

	*_index = index;
	*_magt = mag;

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void x86_prn_accum(CPX *A, CPX *E, CPX *P, CPX *L, int32 cnt, CPX *accum)  //!< This is a long story
{

	CPX Ea, Pa, La;
	int32 lcv;

	Ea.i = 0;
	Ea.q = 0;
	Pa.i = 0;
	Pa.q = 0;
	La.i = 0;
	La.q = 0;

	for(lcv = 0; lcv < cnt; lcv++)
	{
		if(E[lcv].i < 0)
		{
			Ea.i -= A[lcv].i;
			Ea.q -= A[lcv].q;
		}
		else
		{
			Ea.i += A[lcv].i;
			Ea.q += A[lcv].q;
		}

		if(P[lcv].i < 0)
		{
			Pa.i -= A[lcv].i;
			Pa.q -= A[lcv].q;
		}
		else
		{
			Pa.i += A[lcv].i;
			Pa.q += A[lcv].q;
		}

		if(L[lcv].i < 0)
		{
			La.i -= A[lcv].i;
			La.q -= A[lcv].q;
		}
		else
		{
			La.i += A[lcv].i;
			La.q += A[lcv].q;
		}
	}

	accum[0].i = Ea.i;
	accum[0].q = Ea.q;
	accum[1].i = Pa.i;
	accum[1].q = Pa.q;
	accum[2].i = La.i;
	accum[2].q = La.q;

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void x86_prn_accum_new(CPX *A, MIX *E, MIX *P, MIX *L, int32 cnt, CPX_ACCUM *accum)
{

	CPX_ACCUM Ea, Pa, La;
	int32 lcv;

	Ea.i = 0;	Ea.q = 0;
	Pa.i = 0;	Pa.q = 0;
	La.i = 0;	La.q = 0;

	for(lcv = 0; lcv < cnt; lcv++)
	{
		Ea.i += A[lcv].i*E[lcv].i;
		Ea.q += A[lcv].q*E[lcv].ni;
		Pa.i += A[lcv].i*P[lcv].i;
		Pa.q += A[lcv].q*P[lcv].ni;
		La.i += A[lcv].i*L[lcv].i;
		La.q += A[lcv].q*L[lcv].ni;
	}

	accum[0].i = Ea.i;
	accum[0].q = Ea.q;
	accum[1].i = Pa.i;
	accum[1].q = Pa.q;
	accum[2].i = La.i;
	accum[2].q = La.q;

}
/*----------------------------------------------------------------------------------------------*/


//int32 x86_acc(int16 *_A, int32 _cnt)
//{
//
//	int32 accum = 0;
//	int16 *a = _A;
//	int32 lcv;
//
//	for(lcv = 0; lcv < _cnt; lcv++)
//		accum += a[lcv];
//
//	return(accum);
//}
//void x86_crot(CPX *_A, CPX *_B, int32 _cnt)
//{
//
//	int32 lcv;
//	int32 ai, aq;
//	int32 bi, bq;
//	int32 ti, tq;
//
//	bi = _B->i;
//	bq = _B->q;
//
//	for(lcv = 0; lcv < _cnt; lcv++)
//	{
//
//		ai = _A[lcv].i;
//		aq = _A[lcv].q;
//
//		ti = ai*bi-aq*bq;
//		tq = ai*bq+aq*bi;
//
//		_A[lcv].i = (int16)ti;
//		_A[lcv].q = (int16)tq;
//	}
//}
//
//void x86_crot(CPX *_A, CPX *_B, int32 _shift, int32 _cnt)
//{
//
//	int32 lcv;
//	int32 ai, aq;
//	int32 bi, bq;
//	int32 ti, tq;
//	int32 shift;
//	int32 round;
//
//	shift = _shift;
//	round = 1 << (shift-1);
//
//	bi = _B->i;
//	bq = _B->q;
//
//	for(lcv = 0; lcv < _cnt; lcv++)
//	{
//
//		ai = _A[lcv].i;
//		aq = _A[lcv].q;
//
//		ti = ai*bi-aq*bq;
//		tq = ai*bq+aq*bi;
//
//		ti += round;
//		tq += round;
//
//		ti >>= shift;
//		tq >>= shift;
//
//		_A[lcv].i = (int16)ti;
//		_A[lcv].q = (int16)tq;
//	}
//
//}
