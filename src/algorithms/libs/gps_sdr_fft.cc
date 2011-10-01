/*----------------------------------------------------------------------------------------------*/
/*! \file fft.cpp
//
// FILENAME: fft.cpp
//
// DESCRIPTION: Implements member functions of the FFT class.
//
// DEVELOPERS: Gregory W. Heckler (2003-2009)
//
// LICENSE TERMS: Copyright (c) Gregory W. Heckler 2009
//
// This file is part of the GPS Software Defined Radio (GPS-SDR)
//
// The GPS-SDR is free software; you can redistribute it and/or modify it under the terms of the
// GNU General Public License as published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version. The GPS-SDR is distributed in the hope that
// it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// Note:  Comments within this file follow a syntax that is compatible with
//        DOXYGEN and are utilized for automated document extraction
//
// Reference:
*/
/*----------------------------------------------------------------------------------------------*/

#include "gps_sdr_fft.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>


//#define NO_SIMD

#ifdef NO_SIMD
	void rank(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize);
	void rankdf(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize);
	void rank_noscale(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize);
	void rankdf_noscale(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize);
	void bfly(CPX *_A, CPX *_B, MIX *_W);
	void bflydf(CPX *_A, CPX *_B, MIX *_W);
	void bfly_noscale(CPX *_A, CPX *_B, MIX *_W);
	void bflydf_noscale(CPX *_A, CPX *_B, MIX *_W);
#else
	void rank(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)  __attribute__ ((noinline));
	void rankdf(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)  __attribute__ ((noinline));
	void rank_noscale(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)  __attribute__ ((noinline));
	void rankdf_noscale(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)  __attribute__ ((noinline));
#endif

FFT::FFT()
{

	N = 0;

}

FFT::FFT(int32 _N)
{

	int32 lcv;

	for(lcv = 0; lcv < MAX_RANKS; lcv++)
		R[lcv] = 1;

	N = _N;
	M = 0;

	/* Get the number of ranks */
	while(_N > 1)
	{
		M++;
		_N >>= 1;
	}

	W = (MIX *)malloc(N/2*sizeof(MIX));  	// Forward twiddle lookup
	iW = (MIX *)malloc(N/2*sizeof(MIX)); 	// Inverse twiddle lookup
	BR  = (int32 *)malloc(N*sizeof(int32)); 	// Bit reverse lookup
	BRX  = (int32 *)malloc(N*sizeof(CPX)); 	// Shuffle temp array

	initW();
	initBR();

}


FFT::FFT(int32 _N, int32 _R[MAX_RANKS])
{

	int32 lcv;

	for(lcv = 0; lcv < MAX_RANKS; lcv++)
		R[lcv] = _R[lcv];

	N = _N;
	M = 0;

	/* Get the number of ranks */
	while(_N > 1)
	{
		M++;
		_N >>= 1;
	}

	W = (MIX *)malloc(N/2*sizeof(MIX));  	// Forward twiddle lookup
	iW = (MIX *)malloc(N/2*sizeof(MIX)); 	// Inverse twiddle lookup
	BR  = (int32 *)malloc(N*sizeof(int32)); 	// Bit reverse lookup
	BRX  = (int32 *)malloc(N*sizeof(CPX)); 	// Shuffle temp array

	initW();
	initBR();

}


FFT::~FFT()
{
	free(BRX);
	free(BR);
	free(W);
	free(iW);
}

void FFT::initW()
{

	int32 lcv;
	double s, c, phase;
    const double pi = 3.14159265358979323846264338327;

	for(lcv = 0; lcv < N/2; lcv++)
	{
		//Forward twiddles
		phase = (-2*pi*lcv)/N;
		c =		floor(16384*cos(phase));
		s =		floor(16384*sin(phase));
		W[lcv].i = (short)(c);
		W[lcv].q = (short)(s);
		W[lcv].nq = (short)(-s);
		W[lcv].ni = (short)(c);

//		fprintf(stdout,"W:%d,%d\n",W[lcv].i,W[lcv].q);

		//Inverse twiddles
		iW[lcv].i = (short)(c);
		iW[lcv].q = (short)(-s);
		iW[lcv].nq = (short)(s);
		iW[lcv].ni = (short)(c);
	}

}



void FFT::initBR()
{
	int lcv, lcv2, index;

	for(lcv = 0; lcv < N; lcv++)
	{
		index = 0;
		for(lcv2 = 0; lcv2 < M; lcv2++)
		{
			index += ((lcv >> lcv2) & 0x1);
			index <<= 1;
		}
		index >>= 1;

		BR[lcv] = index;

//		fprintf(stdout,"BR:%d\n",BR[lcv]);
	}

}

void FFT::doFFT(CPX *_x, bool _shuf)
{
	int32 lcv, nblocks, bsize;
	CPX *a, *b;
	MIX *w;

	if(_shuf) {
		doShuffle(_x);	//bit reverse the array
	}

	bsize = 1;
	nblocks = N >> 1;

	for(lcv = 0; lcv < M; lcv++)					//Loop over M ranks
	{
		a = _x;
		b = _x + bsize;
		w = W;

		if(R[lcv])
			rank(a, b, w, nblocks, bsize);
		else
			rank_noscale(a, b, w, nblocks, bsize);

		bsize <<= 1;
		nblocks >>= 1;
	}
}


void FFT::doiFFT(CPX *_x, bool _shuf)
{

	int32 lcv, nblocks, bsize;
	CPX *a, *b;
	MIX *w;

	if(_shuf)
		doShuffle(_x);	//bit reverse the array

	bsize = 1;
	nblocks = N >> 1;

	for(lcv = 0; lcv < M; lcv++)					//Loop over M ranks
	{
		a = _x;
		b = _x + bsize;
		w = iW;

		if(R[lcv])
			rank(a, b, w, nblocks, bsize);
		else
			rank_noscale(a, b, w, nblocks, bsize);


		bsize <<= 1;
		nblocks >>= 1;
	}

}


void FFT::doFFTdf(CPX *_x, bool _shuf)
{

	int32 lcv;
	int32 bsize;
	int32 nblocks;
	CPX *a, *b;
	MIX *w;

	bsize = N >> 1;
	nblocks = 1;

	for(lcv = 0; lcv < M; lcv++)					//Loop over M ranks
	{
		a = _x;
		b = _x + bsize;
		w = W;

		if(R[lcv])
			rankdf(a, b, w, nblocks, bsize);
		else
			rankdf_noscale(a, b, w, nblocks, bsize);

		bsize >>= 1;
		nblocks <<= 1;
	}

	if(_shuf)
		doShuffle(_x);	//bit reverse the array

}


void FFT::doiFFTdf(CPX *_x, bool _shuf)
{

	int32 lcv;
	int32 bsize, nblocks;
	CPX *a, *b;
	MIX *w;

	bsize = N >> 1;
	nblocks = 1;

	for(lcv = 0; lcv < M; lcv++)					//Loop over M ranks
	{
		a = _x;
		b = _x + bsize;
		w = iW;

		if(R[lcv])
			rankdf(a, b, w, nblocks, bsize);
		else
			rankdf_noscale(a, b, w, nblocks, bsize);

		bsize >>= 1;
		nblocks <<= 1;
	}

	if(_shuf)
		doShuffle(_x);	//bit reverse the array

}

void FFT::doShuffle(CPX *_x)
{

	int32 lcv;
	int32 *p = (int32 *)_x;

	memcpy(BRX, p, N*sizeof(CPX));

	for(lcv = 0; lcv < N; lcv++)
		p[lcv] = BRX[BR[lcv]];

}


#ifdef NO_SIMD  /* Include the cPP FFT Functions */

void rank(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)
{

	int32 lcv, lcv2;

	MIX *wbase = _W;

	for(lcv = 0; lcv < _nblocks; lcv++)
	{
		for(lcv2 = 0; lcv2 < _bsize; lcv2++)
		{
			bfly(_A, _B, _W);
			_A++; _B++; _W+=_nblocks;
		}

		_A += _bsize;
		_B += _bsize;
		_W  =  wbase;
	}

}

void rank_noscale(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)
{

	int32 lcv, lcv2;

	MIX *wbase = _W;

	for(lcv = 0; lcv < _nblocks; lcv++)
	{
		for(lcv2 = 0; lcv2 < _bsize; lcv2++)
		{
			bfly_noscale(_A, _B, _W);
			_A++; _B++; _W+=_nblocks;
		}

		_A += _bsize;
		_B += _bsize;
		_W  =  wbase;
	}

}

void rankdf(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)
{

	int32 lcv, lcv2;

	MIX *wbase = _W;

	for(lcv = 0; lcv < _nblocks; lcv++)
	{
		for(lcv2 = 0; lcv2 < _bsize; lcv2++)
		{
			bflydf(_A, _B, _W);
			_A++; _B++; _W+=_nblocks;
		}

		_A += _bsize;
		_B += _bsize;
		_W  =  wbase;
	}

}


void rankdf_noscale(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)
{

	int32 lcv, lcv2;

	MIX *wbase = _W;

	for(lcv = 0; lcv < _nblocks; lcv++)
	{
		for(lcv2 = 0; lcv2 < _bsize; lcv2++)
		{
			bflydf_noscale(_A, _B, _W);
			_A++; _B++; _W+=_nblocks;
		}

		_A += _bsize;
		_B += _bsize;
		_W  =  wbase;
	}

}

void bfly(CPX *_A, CPX *_B, MIX *_W)
{
	int32 bi, bq;

	_A->i >>= 1;
	_A->q >>= 1;
	_B->i >>= 1;
	_B->q >>= 1;

	bi = _B->i*_W->i - _B->q*_W->q;
	bq = _B->i*_W->q + _B->q*_W->i;

	bi = (bi + 8192) >> 14;
	bq = (bq + 8192) >> 14;

	_B->i = _A->i - (int16)bi;
	_B->q = _A->q - (int16)bq;

	_A->i += (int16)bi;
	_A->q += (int16)bq;
}

void bfly_noscale(CPX *_A, CPX *_B, MIX *_W)
{
	int32 bi, bq;

	bi = _B->i*_W->i - _B->q*_W->q;
	bq = _B->i*_W->q + _B->q*_W->i;

	bi = (bi + 8192) >> 14;
	bq = (bq + 8192) >> 14;

	_B->i = _A->i - (int16)bi;
	_B->q = _A->q - (int16)bq;

	_A->i += (int16)bi;
	_A->q += (int16)bq;
}

void bflydf(CPX *_A, CPX *_B, MIX *_W)
{
	int32 bi, bq;

	_A->i >>= 1;
	_A->q >>= 1;
	_B->i >>= 1;
	_B->q >>= 1;

	bi = _B->i;
	bq = _B->q;

	_B->i = _A->i - bi;
	_B->q = _A->q - bq;
	_A->i = _A->i + bi;
	_A->q = _A->q + bq;

	bi = _A->i*_W->i - _A->q*_W->q;
	bq = _A->i*_W->q + _A->q*_W->i;

	bi = (bi + 8192) >> 14;
	bq = (bq + 8192) >> 14;

	_A->i += (int16)bi;
	_A->q += (int16)bq;
}

void bflydf_noscale(CPX *_A, CPX *_B, MIX *_W)
{
	int32 bi, bq;

	bi = _B->i;
	bq = _B->q;

	_B->i = _A->i - bi;
	_B->q = _A->q - bq;
	_A->i = _A->i + bi;
	_A->q = _A->q + bq;

	bi = _A->i*_W->i - _A->q*_W->q;
	bq = _A->i*_W->q + _A->q*_W->i;

	bi = (bi + 8192) >> 14;
	bq = (bq + 8192) >> 14;

	_A->i += (int16)bi;
	_A->q += (int16)bq;
}

#else /* Include the SIMD FFT Functions */

void rank(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)
{

	__asm
	(
		".intel_syntax noprefix		\n" //Use intel syntax
                "push ebx\n\t"
		"mov		ebx, [ebp+8]	\n"	//A
		"mov		edi, [ebp+12]	\n" //B
		"mov		edx, [ebp+20]	\n" //nblocks
		"mov		eax, 0x00002000 \n"
		"movd		mm4, eax		\n"
		"punpckldq	mm4, mm4		\n"	//Low 32 bits to high 32 bits
		"B%=:						\n"
		"mov		esi, [ebp+16]	\n"	//W
		"mov		ecx, 0x0		\n"
		"mov		eax, [ebp+20]	\n"	//nblocks
		"sal		eax, 3			\n" //make into bytes
		"S%=:						\n"
		"movd		mm0, [ebx+ecx*4]\n"	//Copy A to mm0
		"movd		mm1, [edi+ecx*4]\n"	//Copy B to mm1
		"movq		mm2, [esi]		\n"	//Copy W to mm2
		"psraw		mm0, 1			\n"	//Divide A by 2
		"psraw		mm1, 1			\n"	//Divide B by 2
		"movq		mm3, mm0		\n" //Copy A to mm3
		"punpckldq	mm1, mm1		\n"	//Low 32 bits to high 32 bits
		"pmaddwd	mm1, mm2		\n" //Multiply and add
		"padddw		mm1, mm4		\n"
		"psrad		mm1, 0xe		\n" //Right shift by 14 bits
		"packssdw	mm1, mm1		\n" //Pack back into 16 bit interleaved
		"paddw		mm0, mm1		\n" //A+Bw
		"psubw		mm3, mm1		\n" //A-Bw
		"movd		[ebx+ecx*4], mm0\n" //Copy back to A
		"movd		[edi+ecx*4], mm3\n" //Copy back to B
		"add		esi, eax		\n"
		"add		ecx, 0x1		\n"
		"cmp		ecx, [ebp+24]	\n"
		"jne		S%=				\n"
		"sal		ecx, 3			\n"
		"add		ebx, ecx		\n"
		"add		edi, ecx		\n"
		"sub		edx, 0x1		\n"
		"jnz		B%=				\n"
                "pop ebx\n\t"
		"EMMS						\n" //Done with MMX
		".att_syntax				\n" //Back to ATT syntax
		:
		: "m" (_A), "m" (_B), "m" (_W), "m" (_nblocks), "m" (_bsize)
		: "%eax", "%ecx", "%edx", "%edi", "%esi"
	);

}


void rank_noscale(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)
{

	__asm
	(
		".intel_syntax noprefix		\n" //Use intel syntax
                "push ebx\n\t"
		"mov		ebx, [ebp+8]	\n"	//A
		"mov		edi, [ebp+12]	\n" //B
		"mov		edx, [ebp+20]	\n" //nblocks
		"mov		eax, 0x00002000 \n"
		"movd		mm4, eax		\n"
		"punpckldq	mm4, mm4		\n"	//Low 32 bits to high 32 bits
		"B%=:						\n"
		"mov		esi, [ebp+16]	\n"	//W
		"mov		ecx, 0x0		\n"
		"mov		eax, [ebp+20]	\n"	//nblocks
		"sal		eax, 3			\n" //make into bytes
		"S%=:						\n"
		"movd		mm0, [ebx+ecx*4]\n"	//Copy A to mm0
		"movd		mm1, [edi+ecx*4]\n"	//Copy B to mm1
		"movq		mm2, [esi]		\n"	//Copy W to mm2
		"movq		mm3, mm0		\n" //Copy A to mm3
		"punpckldq	mm1, mm1		\n"	//Low 32 bits to high 32 bits
		"pmaddwd	mm1, mm2		\n" //Multiply and add
		"padddw		mm1, mm4		\n"
		"psrad		mm1, 0xe		\n" //Right shift by 14 bits
		"packssdw	mm1, mm1		\n" //Pack back into 16 bit interleaved
		"paddw		mm0, mm1		\n" //A+Bw
		"psubw		mm3, mm1		\n" //A-Bw
		"movd		[ebx+ecx*4], mm0\n" //Copy back to A
		"movd		[edi+ecx*4], mm3\n" //Copy back to B
		"add		esi, eax		\n"
		"add		ecx, 0x1		\n"
		"cmp		ecx, [ebp+24]	\n"
		"jne		S%=				\n"
		"sal		ecx, 3			\n"
		"add		ebx, ecx		\n"
		"add		edi, ecx		\n"
		"sub		edx, 0x1		\n"
		"jnz		B%=				\n"
                "pop ebx\n\t"
		"EMMS						\n" //Done with MMX
		".att_syntax				\n" //Back to ATT syntax
		:
		: "m" (_A), "m" (_B), "m" (_W), "m" (_nblocks), "m" (_bsize)
		: "%eax", "%ecx", "%edx", "%edi", "%esi"
	);

}

//static void bflydf(void *_A, void *_B, void *_W)
//{
//
//	__asm
//	{
//		mov		ebx, _A;
//		mov		edi, _B;
//		mov		esi, _W;
//
//		movd		mm0, [ebx];	/*move 32 bits from A to bottom 32 bits of mm0*/
//		movd		mm1, [edi];	/*move 32 bits from B to bottom 32 bits of mm1*/
//		movq		mm2, [esi];	/*move 64 bits from W to mm2*/
//		psraw		mm0, 0x1;
//		psraw		mm1, 0x1;
//		movq		mm3, mm0;	/*copy A to mm3*/
//		paddw		mm0, mm1;	/*A+B*/
//		psubw		mm3, mm1;	/*A-B*/
//		punpckldq	mm3, mm3;	/*copy bottom 32 bits of B data into high 32 bits*/
//		pmaddwd		mm3, mm2;	/*complex multiply, real now 0..31 of mm1, imag 32..63 of mm1*/
//		psrad		mm3, 0xf;	/*right shift 0..31 by 16, 32..63 by 16*/
//		packssdw	mm3, mm3;	/*pack bits 0..31 to 0..16, bits 32..63 to  16..31*/
//		movd		[ebx], mm0;
//		movd		[edi], mm3;
//
//		EMMS;
//	}
//}


void rankdf(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)
{

	__asm
	(
		".intel_syntax noprefix		\n" //Use intel syntax
                "push ebx\n\t"
		"mov		ebx, [ebp+8]	\n"	//A
		"mov		edi, [ebp+12]	\n" //B
		"mov		edx, [ebp+20]	\n" //nblocks
		"mov		eax, 0x00002000 \n"
		"movd		mm4, eax		\n"
		"punpckldq	mm4, mm4		\n"	//Low 32 bits to high 32 bits
		"B%=:						\n"
		"mov		esi, [ebp+16]	\n"	//W
		"mov		ecx, [ebp+24]	\n" //bsize
		"mov		eax, [ebp+20]	\n"	//nblocks
		"sal		eax, 3			\n" //make into bytes
		"L%=:						\n"
		"movd		mm0, [ebx]		\n"	//Copy A to mm0
		"movd		mm1, [edi]		\n"	//Copy B to mm1
		"movq		mm2, [esi]		\n"	//Copy W to mm2
		"psraw		mm0, 1			\n"	//Divide A by 2
		"psraw		mm1, 1			\n"	//Divide B by 2
		"movq		mm3, mm0		\n"	//Copy A to mm3
		"paddw		mm0, mm1		\n" //A+B
		"psubw		mm3, mm1		\n" //A-B
		"punpckldq	mm3, mm3		\n"	//Copy bottom 32 bits of B data into high 32 bits*/
		"pmaddwd	mm3, mm2		\n" //Complex multiply, real now 0..31 of mm1, imag 32..63 of mm1*/
		"padddw		mm3, mm4		\n"
		"psrad		mm3, 0xe		\n" //Right shift 0..31 by 14, 32..63 by 14*/
		"packssdw	mm3, mm3		\n" //Pack bits 0..31 to 0..16, bits 32..63 to  16..31*/
		"movd		[ebx], mm0		\n" //Copy back to A
		"movd		[edi], mm3		\n" //Copy back to B
		"add		ebx, 0x4		\n"
		"add		edi, 0x4		\n"
		"add		esi, eax		\n"
		"loop		L%=				\n"
		"mov		ecx, [ebp+24]	\n" //bsize
		"sal		ecx, 2			\n"
		"add		ebx, ecx		\n"
		"add		edi, ecx		\n"
		"sub		edx, 0x1		\n"
		"jnz		B%=				\n"
                "pop ebx\n\t"
		"EMMS						\n" //Done with MMX
		".att_syntax				\n" //Back to ATT syntax
		:
		: "m" (_A), "m" (_B), "m" (_W), "m" (_nblocks), "m" (_bsize)
		: "%eax", "%ecx", "%edx", "%edi", "%esi"
	);


}


void rankdf_noscale(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)
{

	__asm
	(
		".intel_syntax noprefix		\n" //Use intel syntax
                "push ebx\n\t"
		"mov		ebx, [ebp+8]	\n"	//A
		"mov		edi, [ebp+12]	\n" //B
		"mov		edx, [ebp+20]	\n" //nblocks
		"mov		eax, 0x00002000 \n"
		"movd		mm4, eax		\n"
		"punpckldq	mm4, mm4		\n"	//Low 32 bits to high 32 bits
		"B%=:						\n"
		"mov		esi, [ebp+16]	\n"	//W
		"mov		ecx, [ebp+24]	\n" //bsize
		"mov		eax, [ebp+20]	\n"	//nblocks
		"sal		eax, 3			\n" //make into bytes
		"L%=:						\n"
		"movd		mm0, [ebx]		\n"	//Copy A to mm0
		"movd		mm1, [edi]		\n"	//Copy B to mm1
		"movq		mm2, [esi]		\n"	//Copy W to mm2
		"movq		mm3, mm0		\n"	//Copy A to mm3
		"paddw		mm0, mm1		\n" //A+B
		"psubw		mm3, mm1		\n" //A-B
		"punpckldq	mm3, mm3		\n"	//Copy bottom 32 bits of B data into high 32 bits*/
		"pmaddwd	mm3, mm2		\n" //Complex multiply, real now 0..31 of mm1, imag 32..63 of mm1*/
		"padddw		mm3, mm4		\n"
		"psrad		mm3, 0xe		\n" //Right shift 0..31 by 14, 32..63 by 14*/
		"packssdw	mm3, mm3		\n" //Pack bits 0..31 to 0..16, bits 32..63 to  16..31*/
		"movd		[ebx], mm0		\n" //Copy back to A
		"movd		[edi], mm3		\n" //Copy back to B
		"add		ebx, 0x4		\n"
		"add		edi, 0x4		\n"
		"add		esi, eax		\n"
		"loop		L%=				\n"
		"mov		ecx, [ebp+24]	\n" //bsize
		"sal		ecx, 2			\n"
		"add		ebx, ecx		\n"
		"add		edi, ecx		\n"
		"sub		edx, 0x1		\n"
		"jnz		B%=				\n"
                "pop ebx\n\t"
		"EMMS						\n" //Done with MMX
		".att_syntax				\n" //Back to ATT syntax
		:
		: "m" (_A), "m" (_B), "m" (_W), "m" (_nblocks), "m" (_bsize)
		: "%eax", "%ecx", "%edx", "%edi", "%esi"
	);


}

//void bfly(CPX *_A, CPX *_B, MIX *_W)
//{
//	__asm
//	(
//		".intel_syntax noprefix		\n" //Use intel syntax
//		"mov		ebx, [ebp+8]	\n"	//A
//		"mov		edi, [ebp+12]	\n" //B
//		"mov		esi, [ebp+16]	\n"	//W
//		"movd		mm0, [ebx]		\n"	//Copy A to mm0
//		"movd		mm1, [edi]		\n"	//Copy B to mm1
//		"movq		mm2, [esi]		\n"	//Copy W to mm2
//		"psraw		mm0, 1			\n"	//Divide A by 2
//		"psraw		mm1, 1			\n"	//Divide B by 2
//		"movq		mm3, mm0		\n" //Copy A to mm3
//		"punpckldq	mm1, mm1		\n"	//Low 32 bits to high 32 bits
//		"pmaddwd	mm1, mm2		\n" //Multiply and add
//		"psrad		mm1, 0xe		\n" //Right shift by 14 bits
//		"packssdw	mm1, mm1		\n" //Pack back into 16 bit interleaved
//		"paddw		mm0, mm1		\n" //A+Bw
//		"psubw		mm3, mm1		\n" //A-Bw
//		"movd		[ebx], mm0		\n" //Copy back to A
//		"movd		[edi], mm3		\n" //Copy back to B
//		"EMMS						\n" //Done with MMX
//		".att_syntax				\n" //Back to ATT syntax
//		:
//		:
//		: "%eax", "%ebx", "%ecx", "%edx", "%edi", "%esi"
//	);
//}
//
//
//void block(CPX *_A, CPX *_B, MIX *_W, int32 _nblocks, int32 _bsize)
//{
//
//	__asm
//	(
//		".intel_syntax noprefix		\n" //Use intel syntax
//		"mov		ebx, [ebp+8]	\n"	//A
//		"mov		edi, [ebp+12]	\n" //B
//		"mov		esi, [ebp+16]	\n"	//W
//		"mov		eax, [ebp+20]	\n"	//nblocks
//		"sal		eax, 3			\n" //make into bytes
//		"mov		ecx, [ebp+24]	\n" //bsize
//		"Lbsize:					\n"
//		"movd		mm0, [ebx]		\n"	//Copy A to mm0
//		"movd		mm1, [edi]		\n"	//Copy B to mm1
//		"movq		mm2, [esi]		\n"	//Copy W to mm2
//		"psraw		mm0, 1			\n"	//Divide A by 2
//		"psraw		mm1, 1			\n"	//Divide B by 2
//		"movq		mm3, mm0		\n" //Copy A to mm3
//		"punpckldq	mm1, mm1		\n"	//Low 32 bits to high 32 bits
//		"pmaddwd	mm1, mm2		\n" //Multiply and addint64
//		"psrad		mm1, 0xe		\n" //Right shift by 14 bits
//		"packssdw	mm1, mm1		\n" //Pack back into 16 bit interleaved
//		"paddw		mm0, mm1		\n" //A+Bw
//		"psubw		mm3, mm1		\n" //A-Bw
//		"movd		[ebx], mm0		\n" //Copy back to A
//		"movd		[edi], mm3		\n" //Copy back to B
//		"add		ebx, 0x4		\n"
//		"add		edi, 0x4		\n"
//		"add		esi, eax		\n"
//		"loop		Lbsize			\n"
//		"EMMS						\n" //Done with MMX
//		".att_syntax				\n" //Back to ATT syntax
//		:
//		:
//		: "%eax", "%ebx", "%ecx", "%edx", "%edi", "%esi"
//	);
//
//}


#endif


