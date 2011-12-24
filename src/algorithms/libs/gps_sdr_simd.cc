/*! \file SSE.cpp
	SIMD functionality, mainly for 32 bit interleaved complex integer type (CPX)
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
#ifdef USE_SIMD
#include "gps_sdr_simd.h"

//	__asm
//	(
//		"mov eax, 0x1			\n\t"
//
//		"mov eax, edx			\n\t"
//		"leave					\n\t"
//		"ret					\n\t"
//		".att_syntax			\n\t"
//	);


void sse_add(int16 *A, int16 *B, int32 cnt)
{

	int32 cnt1;
	int32 cnt2;

	cnt1 = cnt / 8;
	cnt2 = (cnt - (8*cnt1));

	if(((int)A%16) || ((int)B%16)) // unaligned version
	{

		__asm
		(
			".intel_syntax noprefix		\n\t" //Set up for loop
			"mov edi, [ebp+8]			\n\t" //Address of A
			"mov esi, [ebp+12]			\n\t" //Address of B
			"mov ecx, [ebp-12]			\n\t" //Counter 1
			"jecxz Z%=					\n\t"
			"L%=:						\n\t"
				"movupd xmm0,  [edi]	\n\t" //Load from A
				"movupd xmm1,  [esi]	\n\t" //Load from B
				"paddw 	xmm0,  xmm1		\n\t" //Multiply A*B
				"movupd [edi], xmm0		\n\t" //Move into A
				"add 	edi, 16			\n\t"
				"add 	esi, 16			\n\t"
			"loop L%=					\n\t" //Loop if not done
			"Z%=:						\n\t"
			"mov ecx, [ebp-16]			\n\t" //Counter 2
			"jecxz ZZ%=					\n\t"
			"mov eax, 0					\n\t"
			"LL%=:						\n\t" //Really finish off loop with non SIMD instructions
				"mov ax, [edi]			\n\t"
				"add ax, [esi]			\n\t"
				"mov [edi], ax			\n\t"
				"add esi, 2				\n\t"
				"add edi, 2				\n\t"
			"loop LL%=					\n\t"
			"ZZ%=:						\n\t"
			"EMMS						\n\t"
			".att_syntax				\n\t"
			:
			: "m" (A), "m" (B), "m" (cnt), "m" (cnt1), "m" (cnt2)
			: "%eax", "%ecx", "%edi", "%esi"
		);

	}
	else
	{
		__asm
		(
			".intel_syntax noprefix		\n\t" //Set up for loop
			"mov edi, [ebp+8]			\n\t" //Address of A
			"mov esi, [ebp+12]			\n\t" //Address of B
			"mov ecx, [ebp-12]			\n\t" //Counter 1
			"jecxz Z%=					\n\t"
			"L%=:						\n\t"
				"movapd xmm0,  [edi]	\n\t" //Load from A
				"paddw 	xmm0,  [esi]	\n\t" //Multiply A*B
				"movapd [edi], xmm0		\n\t" //Move into A
				"add 	edi, 16			\n\t"
				"add 	esi, 16			\n\t"
			"loop L%=					\n\t" //Loop if not done
			"Z%=:						\n\t"
			"mov ecx, [ebp-16]			\n\t" //Counter 2
			"jecxz ZZ%=					\n\t"
			"mov eax, 0					\n\t"
			"LL%=:						\n\t" //Really finish off loop with non SIMD instructions
				"mov ax, [edi]			\n\t"
				"add ax, [esi]			\n\t"
				"mov [edi], ax			\n\t"
				"add esi, 2				\n\t"
				"add edi, 2				\n\t"
			"loop LL%=					\n\t"
			"ZZ%=:						\n\t"
			"EMMS						\n\t"
			".att_syntax				\n\t"
			:
			: "m" (A), "m" (B), "m" (cnt), "m" (cnt1), "m" (cnt2)
			: "%eax", "%ecx", "%edi", "%esi"
		);//end __asm

	}//end if

}


void sse_sub(int16 *A, int16 *B, int32 cnt)
{

	int32 cnt1;
	int32 cnt2;

	cnt1 = cnt / 8;
	cnt2 = (cnt - (8*cnt1));

	if(((int)A%16) || ((int)B%16)) // unaligned version
	{

		__asm
		(
			".intel_syntax noprefix		\n\t" //Set up for loop
			"mov edi, [ebp+8]			\n\t" //Address of A
			"mov esi, [ebp+12]			\n\t" //Address of B
			"mov ecx, [ebp-12]			\n\t" //Counter 1
			"jecxz Z%=					\n\t"
			"L%=:						\n\t"
				"movupd xmm0,  [edi]	\n\t" //Load from A
				"movupd xmm1,  [esi]	\n\t" //Load from B
				"psubw 	xmm0,  xmm1		\n\t" //Multiply A*B
				"movupd [edi], xmm0		\n\t" //Move into A
				"add 	edi, 16			\n\t"
				"add 	esi, 16			\n\t"
			"loop L%=					\n\t" //Loop if not done
			"Z%=:						\n\t"
			"mov ecx, [ebp-16]			\n\t" //Counter 2
			"jecxz ZZ%=					\n\t"
			"mov eax, 0					\n\t"
			"LL%=:						\n\t" //Really finish off loop with non SIMD instructions
				"mov ax, [edi]			\n\t"
				"sub ax, [esi]			\n\t"
				"mov [edi], ax			\n\t"
				"add esi, 2				\n\t"
				"add edi, 2				\n\t"
			"loop LL%=					\n\t"
			"ZZ%=:						\n\t"
			"EMMS						\n\t"
			".att_syntax				\n\t"
			:
			: "m" (A), "m" (B), "m" (cnt), "m" (cnt1), "m" (cnt2)
			: "%eax", "%ecx", "%edi", "%esi"
		);

	}
	else
	{
		__asm
		(
			".intel_syntax noprefix		\n\t" //Set up for loop
			"mov edi, [ebp+8]			\n\t" //Address of A
			"mov esi, [ebp+12]			\n\t" //Address of B
			"mov ecx, [ebp-12]			\n\t" //Counter 1
			"jecxz Z%=					\n\t"
			"L%=:						\n\t"
				"movapd xmm0,  [edi]	\n\t" //Load from A
				"psubw 	xmm0,  [esi]	\n\t" //Multiply A*B
				"movapd [edi], xmm0		\n\t" //Move into A
				"add 	edi, 16			\n\t"
				"add 	esi, 16			\n\t"
			"loop L%=					\n\t" //Loop if not done
			"Z%=:						\n\t"
			"mov ecx, [ebp-16]			\n\t" //Counter 2
			"jecxz ZZ%=					\n\t"
			"mov eax, 0					\n\t"
			"LL%=:						\n\t" //Really finish off loop with non SIMD instructions
				"mov ax, [edi]			\n\t"
				"sub ax, [esi]			\n\t"
				"mov [edi], ax			\n\t"
				"add esi, 2				\n\t"
				"add edi, 2				\n\t"
			"loop LL%=					\n\t"
			"ZZ%=:						\n\t"
			"EMMS						\n\t"
			".att_syntax				\n\t"
			:
			: "m" (A), "m" (B), "m" (cnt), "m" (cnt1), "m" (cnt2)
			: "%eax", "%ecx", "%edi", "%esi"
		);//end __asm

	}//end if

}

void sse_mul(int16 *A, int16 *B, int32 cnt)
{

	int32 cnt1;
	int32 cnt2;

	cnt1 = cnt / 8;
	cnt2 = (cnt - (8*cnt1));

	if(((int)A%16) || ((int)B%16)) // unaligned version
	{

		__asm
		(
			".intel_syntax noprefix		\n\t" //Set up for loop
			"mov edi, [ebp+8]			\n\t" //Address of A
			"mov esi, [ebp+12]			\n\t" //Address of B
			"mov ecx, [ebp-12]			\n\t" //Counter 1
			"jecxz Z%=					\n\t"
			"L%=:						\n\t"
				"movupd xmm0,  [edi]	\n\t" //Load from A
				"movupd xmm1,  [esi]	\n\t" //Load from B
				"pmullw	xmm0,  xmm1		\n\t" //Multiply A*B
				"movupd [edi], xmm0		\n\t" //Move into A
				"add 	edi, 16			\n\t"
				"add 	esi, 16			\n\t"
			"loop L%=					\n\t" //Loop if not done
			"Z%=:						\n\t"
			"mov ecx, [ebp-16]			\n\t" //Counter 2
			"jecxz ZZ%=					\n\t"
			"mov eax, 0					\n\t"
			"LL%=:						\n\t" //Really finish off loop with non SIMD instructions
				"mov ax, [edi]			\n\t"
				"imul ax, [esi]			\n\t"
				"mov [edi], ax			\n\t"
				"add esi, 2				\n\t"
				"add edi, 2				\n\t"
			"loop LL%=					\n\t"
			"ZZ%=:						\n\t"
			"EMMS						\n\t"
			".att_syntax				\n\t"
			:
			: "m" (A), "m" (B), "m" (cnt), "m" (cnt1), "m" (cnt2)
			: "%eax", "%ecx", "%edi", "%esi"
		);

	}
	else
	{
		__asm
		(
			".intel_syntax noprefix		\n\t"	//Set up for loop
			"mov edi, [ebp+8]			\n\t"	//Address of A
			"mov esi, [ebp+12]			\n\t"	//Address of B
			"mov ecx, [ebp-12]			\n\t"	//Counter 1
			"jecxz Z%=					\n\t"
			"L%=:						\n\t"
				"movapd xmm0,  [edi]	\n\t" //Load from A
				"pmullw xmm0,  [esi]	\n\t"	//Multiply A*B
				"movapd [edi], xmm0		\n\t"	//Move into A
				"add 	edi, 16			\n\t"
				"add 	esi, 16			\n\t"
			"loop L%=					\n\t"	//Loop if not done
			"Z%=:						\n\t"
			"mov ecx, [ebp-16]			\n\t"	//Counter 2
			"jecxz ZZ%=					\n\t"
			"mov eax, 0					\n\t"
			"LL%=:						\n\t"	//Really finish off loop with non SIMD instructions
				"mov ax, [edi]			\n\t"
				"imul ax, [esi]			\n\t"
				"mov [edi], ax			\n\t"
				"add esi, 2				\n\t"
				"add edi, 2				\n\t"
			"loop LL%=					\n\t"
			"ZZ%=:						\n\t"
			"EMMS						\n\t"
			".att_syntax				\n\t"
			:
			: "m" (A), "m" (B), "m" (cnt), "m" (cnt1), "m" (cnt2)
			: "%eax", "%ecx", "%edi", "%esi"
		);//end __asm

	}//end if

}


int32 sse_dot(int16 *A, int16 *B, int32 cnt)
{

	int32 cnt1;
	int32 cnt2;
	int32 temp;

	if(((int32)A%16) || ((int32)B%16))	//If the memory locations are not 16 byte aligned use slower movupd instruction
	{

		cnt1 = cnt / 24;
		cnt2 = (cnt - (24*cnt1));

		__asm
		(
			".intel_syntax noprefix		\n\t" //Set up for loop
                        "push ebx\n\t"
			"mov edi, [ebp+8]			\n\t" //Address of A
			"mov esi, [ebp+12]			\n\t" //Address of B
			"mov ecx, [ebp-16]			\n\t" //Counter 1
			"pxor xmm0, xmm0			\n\t" //Clear the running sum (accumulator)
			"jecxz Z%=					\n\t"
			"L%=:						\n\t"
				"movupd xmm1, [esi]		\n\t" //Load from A
				"movupd xmm2, [esi+16]	\n\t" //Load from A
				"movupd xmm3, [esi+32]	\n\t" //Load from A
				"movupd xmm4, [edi]		\n\t" //Load from B
				"movupd xmm5, [edi+16]	\n\t" //Load from B
				"movupd xmm6, [edi+32]	\n\t" //Load from B
				"pmaddwd xmm1, xmm4		\n\t" //Multiply and accumulate
				"pmaddwd xmm2, xmm5		\n\t" //Multiply and accumulate
				"pmaddwd xmm3, xmm6		\n\t" //Multiply and accumulate
				"paddd	xmm1, xmm3		\n\t" //Add into accumulator (efficiently)
				"paddd	xmm0, xmm2		\n\t"
				"paddd	xmm0, xmm1		\n\t"
				"add esi, 48			\n\t"
				"add edi, 48			\n\t"
			"loop L%=					\n\t" //Loop if not done
			"Z%=:						\n\t"
			"movd ebx, xmm0				\n\t" //right-hand word to ebx
			"psrldq xmm0, 4				\n\t" //left-hand word to right side of xmm0
			"movd eax, xmm0				\n\t" //left-hand word into eax
			"add eax, ebx				\n\t" //running sum now in eax
			"mov edx, eax				\n\t" //move into edx
			"psrldq xmm0, 4				\n\t" //left-hand word to right side of xmm0
			"movd ebx, xmm0				\n\t" //right-hand word to ebx
			"psrldq xmm0, 4				\n\t" //left-hand word to right side of xmm0
			"movd eax, xmm0				\n\t" //left-hand word into eax
			"add eax, ebx				\n\t" //running sum now in eax
			"add edx, eax				\n\t" //add to edx
			"mov ecx, [ebp-20]			\n\t"
			"jecxz ZZ%=					\n\t"
			"LL%=:						\n\t" //Really finish off loop with non SIMD instructions
				"mov bx, [edi]			\n\t" //Move 16 bits into bx
				"movsx ebx, bx			\n\t" //Sign extend to 32 bits
				"mov ax, [esi]			\n\t" //Move 16 bits into ax
				"movsx eax, ax			\n\t" //Sign extend to 32 bits
				"imul ebx, eax			\n\t" //Multiply
				"add edx, ebx			\n\t" //Add into accumulator
				"add esi, 2				\n\t"
				"add edi, 2				\n\t"
			"loop LL%=					\n\t"
			"ZZ%=:						\n\t"
			"EMMS						\n\t"
			"mov [ebp-24], edx			\n\t"
                        "pop ebx\n\t"
			".att_syntax				\n\t"
			: "=m"(temp)
			: "m" (A), "m" (B), "m" (cnt), "m" (cnt1), "m" (cnt2)
			: "%eax", "%ecx", "%edx", "%edi", "%esi"
		);//end __asm
	}
	else	//use faster movapd instruction
	{

		cnt1 = cnt / 56;
		cnt2 = (cnt - (56*cnt1));

		__asm
		(
				//Set up for loop
			".intel_syntax noprefix		\n\t"	//Set up for loop
                        "push ebx\n\t"
			"mov edi, [ebp+8]			\n\t"	//Address of A
			"mov esi, [ebp+12]			\n\t"	//Address of B
			"mov ecx, [ebp-16]			\n\t"	//Counter 1
			"pxor xmm0, xmm0			\n\t"	//Clear the running sum (accumulator)
			"jecxz Z%=					\n\t"
			"L%=:						\n\t"
				"movapd xmm1, [esi]		\n\t"	//Load from A
				"movapd xmm2, [esi+16]	\n\t"	//Load from A
				"movapd xmm3, [esi+32]	\n\t" 	//Load from A
				"movapd xmm4, [esi+48]	\n\t"	//Load from A
				"movapd xmm5, [esi+64]	\n\t"	//Load from A
				"movapd xmm6, [esi+80]	\n\t"	//Load from A
				"movapd xmm7, [esi+96]	\n\t"	//Load from A
				"pmaddwd xmm1, [edi]	\n\t"
				"pmaddwd xmm2, [edi+16]	\n\t"
				"pmaddwd xmm3, [edi+32]	\n\t"
				"pmaddwd xmm4, [edi+48]	\n\t"
				"pmaddwd xmm5, [edi+64]	\n\t"
				"pmaddwd xmm6, [edi+80]	\n\t"
				"pmaddwd xmm7, [edi+96]	\n\t"
				"paddd	xmm0, xmm7		\n\t"
				"paddd	xmm1, xmm2		\n\t"
				"paddd	xmm3, xmm4		\n\t"
				"paddd	xmm5, xmm6		\n\t"
				"paddd	xmm1, xmm3		\n\t"
				"paddd	xmm0, xmm5		\n\t"
				"paddd	xmm0, xmm1		\n\t"
				"add esi, 112			\n\t"
				"add edi, 112			\n\t"
			"loop L%=					\n\t"	// Loop if not done
			"Z%=:						\n\t"
			"movd ebx, xmm0				\n\t"	// right-hand word to ebx
			"psrldq xmm0, 4				\n\t"	// left-hand word to right side of xmm0
			"movd eax, xmm0				\n\t" 	// left-hand word into eax
			"add eax, ebx				\n\t"	// running sum now in eax
			"mov edx, eax				\n\t"	// move into temp
			"psrldq xmm0, 4				\n\t"
			"movd ebx, xmm0				\n\t"	// right-hand word to ebx
			"psrldq xmm0, 4				\n\t"	// left-hand word to right side of xmm0
			"movd eax, xmm0				\n\t"	// left-hand word into eax
			"add eax, ebx				\n\t"	// running sum now in eax
			"add edx, eax				\n\t" 	// add into temp
			"mov ecx, [ebp-20]			\n\t"
			"jecxz ZZ%=					\n\t"
			"LL%=:						\n\t"	//Really finish off loop with non SIMD instructions
				"mov bx, [edi]			\n\t"	//Move 16 bits into bx
				"movsx ebx, bx			\n\t"	//Sign extend to 32 bits
				"mov ax, [esi]			\n\t"	//Move 16 bits into ax
				"movsx eax, ax			\n\t"	//Sign extend to 32 bits
				"imul ebx, eax			\n\t"	//Multiply
				"add edx, ebx			\n\t"	//Add into accumulator
				"add esi, 2				\n\t"
				"add edi, 2				\n\t"
			"loop LL%=					\n\t"
			"ZZ%=:						\n\t"
			"EMMS						\n\t"
			"mov [ebp-24], edx			\n\t"
                        "pop ebx\n\t"
			".att_syntax				\n\t"
			: "=m"(temp)
			: "m" (A), "m" (B), "m" (cnt), "m" (cnt1), "m" (cnt2)
			: "%eax", "%ecx", "%edx", "%edi", "%esi"
		);
	}

	return(temp);

}

void sse_conj(CPX *A, int32 cnt)
{

	int32 cnt1;
	int32 cnt2;
	int32 temp = 0xffff0001; //[1, -1]

	cnt1 = cnt/4;
	cnt2 = cnt-4*cnt1;

	__asm
	(
		".intel_syntax noprefix			\n\t" //Set up for loop
		"mov edi, [ebp+8]				\n\t" //Address of A	source1
		"mov ecx, [ebp-8]				\n\t" //Counter
		"movd mm7, [ebp-16]				\n\t"
		"punpckldq mm7, mm7				\n\t"
		"movd xmm7, [ebp-16]			\n\t"
		"punpckldq xmm7, xmm7			\n\t"
		"punpckldq xmm7, xmm7			\n\t"
		"jecxz Z%=						\n\t"
		"L%=:							\n\t"
		"	movupd	xmm0, [edi]			\n\t" //Load from A
		"	pmullw	xmm0, xmm7			\n\t" //Multiply to get [Re -Im Re -Im]
		"	movupd	[edi], xmm0			\n\t" //Move into A
		"	add		edi, 16				\n\t" //Move in array
		"loop L%=						\n\t" //Loop if not done
		"Z%=:							\n\t"
		"mov ecx, [ebp-12]				\n\t"
		"jecxz ZZ%=						\n\t"
		"LL%=:							\n\t"
		"	movd	mm0, [edi]			\n\t" //Load from A
		"	pmullw	mm0, mm7			\n\t" //Multiply to get [Re -Im Re -Im]
		"	movd	[edi], mm0			\n\t" //Move into A
		"	add		edi, 4				\n\t"
		"loop LL%=						\n\t"
		"ZZ%=:							\n\t"
		"EMMS							\n\t"
		".att_syntax					\n\t"
		:
		: "m" (A), "m" (cnt), "m" (cnt1), "m" (cnt2), "m" (temp)
		: "%ecx", "%edx", "%edi"
	);

}

void sse_cmul(CPX *A, CPX *B, int32 cnt)
{

	int32 cnt1;
	int32 cnt2;

	//volatile int32 M[4] = {0xffff0001, 0x00010001, 0xffff0001, 0x00010001}; //{1,-1,1,1,1,-1,1,1};

	cnt1 = cnt/4;
	cnt2 = cnt-4*cnt1;

	cnt1 = 0;
	cnt2 = cnt;

	__asm
	(
		".intel_syntax noprefix			\n\t" //Set up for loop
		"mov edi, [ebp+8]				\n\t" //Address of A
		"mov esi, [ebp+12]				\n\t" //Address of B
		"mov ecx, [ebp-12]				\n\t" //Counter 1
		"movupd xmm7,[ebp-32]			\n\t" // Move the multiply thingie
		"jecxz Z%=						\n\t"
		"L%=:							\n\t"
			"movlpd xmm0, [edi]			\n\t" //Copy from A
			"movlpd xmm1, [edi+8]		\n\t" //Copy from A
			"movlpd xmm3, [esi]			\n\t" //Copy from B
			"movlpd xmm4, [esi+8]		\n\t" //Copy from B
			"punpckldq xmm0, xmm0		\n\t" //Copy low 32 bits to high 32 bits
			"punpckldq xmm1, xmm1		\n\t" //Copy low 32 bits to high 32 bits
			"punpckldq xmm3, xmm3		\n\t" //Copy low 32 bits to high 32 bits
			"punpckldq xmm4, xmm4		\n\t" //Copy low 32 bits to high 32 bits
			"pshuflw xmm3, xmm3, 0x14	\n\t" //Shuffle Low 64 bits to get [Re Im Im Re]
			"pshuflw xmm4, xmm4, 0x14	\n\t" //Shuffle Low 64 bits to get [Re Im Im Re]
			"pshufhw xmm3, xmm3, 0x14	\n\t" //Shuffle High 64 bits to get [Re Im Im Re]
			"pshufhw xmm4, xmm4, 0x14	\n\t" //Shuffle High 64 bits to get [Re Im Im Re]
			"pmullw xmm3, xmm7			\n\t" //Multiply to get [Re Im -Im Re]
			"pmullw xmm4, xmm7			\n\t" //Multiply to get [Re Im -Im Re]
			"pmaddwd xmm0, xmm3			\n\t" //Complex multiply and add
			"pmaddwd xmm1, xmm4			\n\t" //Complex multiply and add
			"packssdw xmm0, xmm0		\n\t" //Get into low 64 bits
			"packssdw xmm1, xmm1		\n\t" //Get into low 64 bits
			"movsd [edi],   xmm0		\n\t" //Move into A
			"movsd [edi+8], xmm1		\n\t" //Move into A
			"add edi, 16				\n\t" //Move in array
			"add esi, 16				\n\t" //Move in array
		"loop L%=						\n\t" // Loop if not done
		"Z%=:							\n\t"
		"mov ecx, [ebp-16]				\n\t"
		"jecxz ZZ%=						\n\t"
		"LL%=:							\n\t"
			"movlpd		xmm0, [edi]		\n\t" //Copy from A
			"movlpd		xmm1, [esi]		\n\t" //Copy from B
			"punpckldq	xmm0, xmm0		\n\t" //Copy low 32 bits to high 32 bits
			"punpckldq	xmm1, xmm1		\n\t" //Copy low 32 bits to high 32 bits
			"pshuflw	xmm1, xmm1, 0x14\n\t" //Shuffle Low 64 bits to get [Re Im Im Re]
			"pmullw		xmm1, xmm7		\n\t" //Multiply to get [Re Im -Im Re]
			"pmaddwd	xmm0, xmm1		\n\t" //Complex multiply and add
			"packssdw	xmm0, xmm0		\n\t" //Get into low 32 bits
			"movd		[edi], xmm0		\n\t" //Move into A
			"add edi, 4					\n\t"
			"add esi, 4					\n\t"
			"loop LL%=					\n\t"
		"ZZ%=:							\n\t"
		"EMMS							\n\t"
		".att_syntax					\n\t"
		:
		: "m" (A), "m" (B), "m" (cnt), "m" (cnt1), "m" (cnt2)
		: "%ecx", "%edi", "%esi"
	);

}


void sse_cmuls(CPX *A, CPX *B, int32 cnt, int32 shift)
{

	int32 cnt1;
	int32 cnt2;
	int32 round;

	//volatile int32 M[4] = {0xffff0001, 0x00010001, 0xffff0001, 0x00010001}; //{1,-1,1,1,1,-1,1,1};

	cnt1 = cnt/4;
	cnt2 = cnt-4*cnt1;

	round = 1 << (shift-1);

	__asm
	(
		".intel_syntax noprefix			\n\t" //Set up for loop
		"mov edi, [ebp+8]				\n\t" //Address of A
		"mov esi, [ebp+12]				\n\t" //Address of B
		"mov ecx, [ebp-12]				\n\t" //Counter 1
		"movupd xmm7,[ebp-36]			\n\t" //Move the multiply thingie
		"movss  xmm6, [ebp+20]			\n\t" //Move the round thingie
		"movss  xmm5, [ebp-20]			\n\t" //Move the round thingie
		"punpckldq xmm5, xmm5			\n\t"
		"punpcklqdq xmm5, xmm5			\n\t"
		"jecxz Z%=						\n\t"
		"L%=:							\n\t"
			"movlpd xmm0, [edi]			\n\t" //Copy from A
			"movlpd xmm1, [edi+8]		\n\t" //Copy from A
			"movlpd xmm3, [esi]			\n\t" //Copy from B
			"movlpd xmm4, [esi+8]		\n\t" //Copy from B
			"punpckldq xmm0, xmm0		\n\t" //Copy low 32 bits to high 32 bits
			"punpckldq xmm1, xmm1		\n\t" //Copy low 32 bits to high 32 bits
			"punpckldq xmm3, xmm3		\n\t" //Copy low 32 bits to high 32 bits
			"punpckldq xmm4, xmm4		\n\t" //Copy low 32 bits to high 32 bits
			"pshuflw xmm3, xmm3, 0x14	\n\t" //Shuffle Low 64 bits to get [Re Im Im Re]
			"pshuflw xmm4, xmm4, 0x14	\n\t" //Shuffle Low 64 bits to get [Re Im Im Re]
			"pshufhw xmm3, xmm3, 0x14	\n\t" //Shuffle High 64 bits to get [Re Im Im Re]
			"pshufhw xmm4, xmm4, 0x14	\n\t" //Shuffle High 64 bits to get [Re Im Im Re]
			"pmullw xmm3, xmm7			\n\t" //Multiply to get [Re Im -Im Re]
			"pmullw xmm4, xmm7			\n\t" //Multiply to get [Re Im -Im Re]
			"pmaddwd xmm0, xmm3			\n\t" //Complex multiply and add
			"pmaddwd xmm1, xmm4			\n\t" //Complex multiply and add
			"paddd xmm0, xmm5			\n\t" //Add in 2^(shift-1)
			"paddd xmm1, xmm5			\n\t" //Add in 2^(shift-1)
			"psrad xmm0, xmm6			\n\t" //Shift by X bits
			"psrad xmm1, xmm6			\n\t" //Shift by X bits
			"packssdw xmm0, xmm0		\n\t" //Get into low 64 bits
			"packssdw xmm1, xmm1		\n\t" //Get into low 64 bits
			"movlpd [edi],   xmm0		\n\t" //Move into A
			"movlpd [edi+8], xmm1		\n\t" //Move into A
			"add edi, 16				\n\t" //Move in array
			"add esi, 16				\n\t" //Move in array
		"loop L%=						\n\t" //Loop if not done
		"Z%=:							\n\t"
		"mov ecx, [ebp-16]				\n\t"
		"jecxz ZZ%=						\n\t"
		"LL%=:							\n\t"
			"movlpd		xmm0, [edi]		\n\t" //Copy from A
			"movlpd		xmm1, [esi]		\n\t" //Copy from B
			"punpckldq	xmm0, xmm0		\n\t" //Copy low 32 bits to high 32 bits
			"punpckldq	xmm1, xmm1		\n\t" //Copy low 32 bits to high 32 bits
			"pshuflw	xmm1, xmm1, 0x14\n\t" //Shuffle Low 64 bits to get [Re Im Im Re]
			"pmullw		xmm1, xmm7		\n\t" //Multiply to get [Re Im -Im Re]
			"pmaddwd	xmm0, xmm1		\n\t" //Complex multiply and add
			"paddd		xmm0, xmm5		\n\t" //Add in 2^(shift-1)
			"psrad		xmm0, xmm6		\n\t" //Shift by X bits
			"packssdw	xmm0, xmm0		\n\t" //Get into low 32 bits
			"movd		[edi], xmm0		\n\t" //Move into A
			"add edi, 4					\n\t"
			"add esi, 4					\n\t"
			"loop LL%=					\n\t"
		"ZZ%=:							\n\t"
		"EMMS							\n\t"
		".att_syntax					\n\t"
		:
		: "m" (A), "m" (B), "m" (cnt), "m" (cnt1), "m" (cnt2), "m" (shift), "m" (round)
		: "%ecx", "%edi", "%esi"
	);

}

void sse_cmulsc(CPX *A, CPX *B, CPX *C, int32 cnt, int32 shift)
{
	int32 cnt1;
	int32 cnt2;
	int32 round;

	//volatile int32 M[4] = {0xffff0001, 0x00010001, 0xffff0001, 0x00010001}; //{1,-1,1,1,1,-1,1,1};

	cnt1 = cnt/4;
	cnt2 = cnt-4*cnt1;

	round = 1 << (shift-1);

	__asm
	(
		".intel_syntax noprefix			\n\t" //Set up for loop
		"mov edi, [ebp+8]				\n\t" //Address of A
		"mov esi, [ebp+12]				\n\t" //Address of B
		"mov eax, [ebp+16]				\n\t" //Address of C
		"mov ecx, [ebp-12]				\n\t" //Counter 1
		"movupd xmm7,[ebp-36]			\n\t" //Move the multiply thingie
		"movss  xmm6, [ebp+24]			\n\t" //Move the round thingie
		"movss  xmm5, [ebp-20]			\n\t" //Move the round thingie
		"punpckldq xmm5, xmm5			\n\t"
		"punpcklqdq xmm5, xmm5			\n\t"
		"jecxz Z%=						\n\t"
		"L%=:							\n\t"
			"movlpd xmm0, [edi]			\n\t" //Copy from A
			"movlpd xmm1, [edi+8]		\n\t" //Copy from A
			"movlpd xmm3, [esi]			\n\t" //Copy from B
			"movlpd xmm4, [esi+8]		\n\t" //Copy from B
			"punpckldq xmm0, xmm0		\n\t" //Copy low 32 bits to high 32 bits
			"punpckldq xmm1, xmm1		\n\t" //Copy low 32 bits to high 32 bits
			"punpckldq xmm3, xmm3		\n\t" //Copy low 32 bits to high 32 bits
			"punpckldq xmm4, xmm4		\n\t" //Copy low 32 bits to high 32 bits
			"pshuflw xmm3, xmm3, 0x14	\n\t" //Shuffle Low 64 bits to get [Re Im Im Re]
			"pshuflw xmm4, xmm4, 0x14	\n\t" //Shuffle Low 64 bits to get [Re Im Im Re]
			"pshufhw xmm3, xmm3, 0x14	\n\t" //Shuffle High 64 bits to get [Re Im Im Re]
			"pshufhw xmm4, xmm4, 0x14	\n\t" //Shuffle High 64 bits to get [Re Im Im Re]
			"pmullw xmm3, xmm7			\n\t" //Multiply to get [Re Im -Im Re]
			"pmullw xmm4, xmm7			\n\t" //Multiply to get [Re Im -Im Re]
			"pmaddwd xmm0, xmm3			\n\t" //Complex multiply and add
			"pmaddwd xmm1, xmm4			\n\t" //Complex multiply and add
			"paddd xmm0, xmm5			\n\t" //Add in 2^(shift-1)
			"paddd xmm1, xmm5			\n\t" //Add in 2^(shift-1)
			"psrad xmm0, xmm6			\n\t" //Shift by X bits
			"psrad xmm1, xmm6			\n\t" //Shift by X bits
			"packssdw xmm0, xmm0		\n\t" //Get into low 64 bits
			"packssdw xmm1, xmm1		\n\t" //Get into low 64 bits
			"movlpd [eax],   xmm0		\n\t" //Move into A
			"movlpd [eax+8], xmm1		\n\t" //Move into A
			"add edi, 16				\n\t" //Move in array
			"add esi, 16				\n\t" //Move in array
			"add eax, 16				\n\t"
		"loop L%=						\n\t" //Loop if not done
		"Z%=:							\n\t"
		"mov ecx, [ebp-16]				\n\t"
		"jecxz ZZ%=						\n\t"
		"LL%=:							\n\t"
			"movlpd		xmm0, [edi]		\n\t" //Copy from A
			"movlpd		xmm1, [esi]		\n\t" //Copy from B
			"punpckldq	xmm0, xmm0		\n\t" //Copy low 32 bits to high 32 bits
			"punpckldq	xmm1, xmm1		\n\t" //Copy low 32 bits to high 32 bits
			"pshuflw	xmm1, xmm1, 0x14\n\t" //Shuffle Low 64 bits to get [Re Im Im Re]
			"pmullw		xmm1, xmm7		\n\t" //Multiply to get [Re Im -Im Re]
			"pmaddwd	xmm0, xmm1		\n\t" //Complex multiply and add
			"paddd		xmm0, xmm5		\n\t" //Add in 2^(shift-1)
			"psrad		xmm0, xmm6		\n\t" //Shift by X bits
			"packssdw	xmm0, xmm0		\n\t" //Get into low 32 bits
			"movd		[eax], xmm0		\n\t" //Move into A
			"add edi, 4					\n\t"
			"add esi, 4					\n\t"
			"add eax, 4					\n\t"
			"loop LL%=					\n\t"
		"ZZ%=:							\n\t"
		"EMMS							\n\t"
		".att_syntax					\n\t"
		:
		: "m" (A), "m" (B), "m" (C), "m" (cnt), "m" (cnt1), "m" (cnt2), "m" (shift), "m" (round)
		: "%eax", "%ecx", "%edi", "%esi"
	);

}


void sse_cacc(CPX *A, MIX *B, int32 cnt, int32 *iaccum, int32 *baccum)
{

	int32 cnt1;
	int32 cnt2;

	if(((int)A%16) || ((int)B%16))
	{

		cnt1 = cnt / 6;
		cnt2 = (cnt - (6*cnt1));

		__asm
		(
			".intel_syntax noprefix			\n\t" //Set up for loop
			"mov edi, [ebp+8]				\n\t" //Address of A
			"mov esi, [ebp+12]				\n\t" //Address of B
			"mov ecx, [ebp-12]				\n\t" //Counter 1
			"pxor xmm0, xmm0				\n\t" //Clear the running sum
			"pxor mm0, mm0					\n\t" //Clear the running sum
			"jecxz Z%=						\n\t"
			"L%=:							\n\t"
			"	movlpd xmm1, [edi]			\n\t" //load IF data
			"	movlpd xmm2, [edi+8]		\n\t" //load IF data
			"	movlpd xmm3, [edi+16]		\n\t" //load IF data
			"	movupd xmm4, [esi]			\n\t" //load Sine data
			"	movupd xmm5, [esi+16]		\n\t" //load Sine data
			"	movupd xmm6, [esi+32]		\n\t" //load Sine data
			"	punpckldq xmm1, xmm1		\n\t" //copies bits 0..31 to 32..63 and bits 32..63 to 64..95 and 65..127
			"	punpckldq xmm2, xmm2		\n\t" //copies bits 0..63 to 64..127
			"	punpckldq xmm3, xmm3		\n\t" //copies bits 0..63 to 64..127
			"	pmaddwd	xmm1, xmm4			\n\t" //multiply and add, result in xmm1
			"	pmaddwd xmm2, xmm5			\n\t" //multiply and add, result in xmm2
			"	pmaddwd	xmm3, xmm6			\n\t" //multiply and add, result in xmm3
			"	paddd xmm0, xmm3			\n\t" //Add into accumulator (efficiently)
			"	paddd xmm1, xmm2			\n\t"
			"	paddd xmm0, xmm1			\n\t"
			"	add edi, 24					\n\t" //move in complex sine by 24 bytes
			"	add esi, 48					\n\t" //move in IF array by 48 bytes
			"loop L%=						\n\t" //Loop if not done
			"Z%=:							\n\t"
			"mov ecx, [ebp-16]				\n\t"
			"jecxz ZZ%=						\n\t"
			"LL%=:							\n\t"
			"	movd		mm1, [edi]		\n\t" //load IF data
			"	movq		mm2, [esi]		\n\t"
			"	punpckldq	mm1, mm1		\n\t" //copy bottom 32 bits of IF data into high 32 bits
			"	pmaddwd		mm1, mm2		\n\t" //perform mmx complex multiply
			"	paddd		mm0, mm1		\n\t" //add into accumulator
			"	add edi, 4					\n\t" //move in complex sine by 4 bytes
			"	add esi, 8					\n\t" //move in IF array by 8 bytes
			"loop LL%=						\n\t"
			"ZZ%=:							\n\t"
			"movdq2q 	mm1, xmm0			\n\t"
			"punpckhqdq xmm0, xmm0			\n\t" //move bits 64..127 of xmm0 into 0..63 of xmm0
			"movdq2q 	mm2, xmm0			\n\t"
			"paddd 		mm0, mm1			\n\t" //add together
			"paddd		mm0, mm2			\n\t" //add"	punpckldq xmm1, xmm1		\n\t" //copies bits 0..31 to 32..63 and bits 32..63 to 64..95 and 65..127 together
			"mov		eax, [ebp+20]		\n\t"
			"movd		[eax], mm0			\n\t"
			"punpckhdq	mm0, mm0			\n\t"
			"mov		eax, [ebp+24]		\n\t"
			"movd		[eax], mm0			\n\t"
			"EMMS							\n\t"
			".att_syntax					\n\t"
			:
			: "m" (A), "m" (B), "m" (cnt), "m" (iaccum), "m" (baccum), "m" (cnt1), "m" (cnt2)
			: "%eax", "%ecx", "%edi", "%esi"
		);//end __asm
	}
	else
	{

		cnt1 = cnt / 12;
		cnt2 = (cnt - (12*cnt1));

		__asm
		(
			".intel_syntax noprefix			\n\t" //Set up for loop
			"mov edi, [ebp+8]				\n\t" //Address of A
			"mov esi, [ebp+12]				\n\t" //Address of B
			"mov ecx, [ebp-12]				\n\t" //Counter 1
			"pxor xmm0, xmm0				\n\t" //Clear the running sum
			"pxor mm0, mm0					\n\t" //Clear the running sum
			"jecxz AZ%=						\n\t"
			"AL%=:							\n\t"
			"	movlpd xmm1, [edi]			\n\t" //load IF data
			"	movlpd xmm2, [edi+8]		\n\t" //load IF data
			"	movlpd xmm3, [edi+16]		\n\t" //load IF data
			"	movlpd xmm4, [edi+24]		\n\t" //load IF data
			"	movlpd xmm5, [edi+32]		\n\t" //load IF data
			"	movlpd xmm6, [edi+40]		\n\t" //load IF data
			"	punpckldq xmm1, xmm1		\n\t" //copies bits 0..31 to 32..63 and bits 32..63 to 64..95 and 65..127
			"	punpckldq xmm2, xmm2		\n\t" //copies bits 0..63 to 64..127
			"	punpckldq xmm3, xmm3		\n\t" //copies bits 0..63 to 64..127
			"	punpckldq xmm4, xmm4		\n\t" //copies bits 0..63 to 64..127
			"	punpckldq xmm5, xmm5		\n\t" //copies bits 0..63 to 64..127
			"	punpckldq xmm6, xmm6		\n\t" //copies bits 0..63 to 64..127
			"	pmaddwd	xmm1, [esi]			\n\t" //multiply and add, result in xmm1
			"	pmaddwd xmm2, [esi+16]		\n\t" //multiply and add, result in xmm2
			"	pmaddwd	xmm3, [esi+32]		\n\t" //multiply and add, result in xmm3
			"	pmaddwd	xmm4, [esi+48]		\n\t" //multiply and add, result in xmm4
			"	pmaddwd	xmm5, [esi+64]		\n\t" //multiply and add, result in xmm5
			"	pmaddwd	xmm6, [esi+80]		\n\t" //multiply and add, result in xmm6
			"	paddd	xmm1, xmm2			\n\t" //Add into accumulator (efficiently)
			"	paddd	xmm3, xmm4			\n\t"
			"	paddd	xmm5, xmm6			\n\t"
			"	paddd	xmm1, xmm3			\n\t"
			"	paddd	xmm0, xmm5			\n\t"
			"	paddd	xmm0, xmm1			\n\t"
			"	add edi, 48					\n\t" //move in complex sine by 56 bytes
			"	add esi, 96					\n\t" //move in IF array by 112 bytes
			"loop AL%=						\n\t" // Loop if not done
			"AZ%=:							\n\t"
			"mov ecx, [ebp-16]				\n\t"
			"jecxz AZZ%=					\n\t"
			"ALL%=:							\n\t"
			"	movq		mm1, [edi]		\n\t" //load IF data
			"	punpckldq	mm1, mm1		\n\t" //copy bottom 32 bits of IF data into high 32 bits
			"	pmaddwd		mm1, [esi]		\n\t" //perform mmx complex multiply
			"	paddd		mm0, mm1		\n\t" //add into accumulator
			"	add edi, 4					\n\t" //move in complex sine by 4 bytes
			"	add esi, 8					\n\t" //move in IF array by 8 bytes
			"loop ALL%=						\n\t"
			"AZZ%=:							\n\t"
			"movdq2q	mm1, xmm0			\n\t"
			"punpckhqdq xmm0, xmm0			\n\t" //move bits 64..127 of xmm0 into 0..63 of xmm0
			"movdq2q	mm2, xmm0			\n\t"
			"paddd		mm0, mm1			\n\t" //add together
			"paddd		mm0, mm2			\n\t" //add together
			"mov		eax, [ebp+20]		\n\t"
			"movd		[eax], mm0			\n\t"
			"punpckhdq	mm0, mm0			\n\t"
			"mov		eax, [ebp+24]		\n\t"
			"movd		[eax], mm0			\n\t"
			"EMMS							\n\t"
			".att_syntax					\n\t"
			:
			: "m" (A), "m" (B), "m" (cnt), "m" (iaccum), "m" (baccum), "m" (cnt1), "m" (cnt2)
			: "%eax", "%ecx", "%edi", "%esi"
		);//end __asm

	}//end if


}

//!< A must hold baseband data, E,P,L must hold PRN data
void sse_prn_accum(CPX *A, CPX *E, CPX *P, CPX *L, int32 cnt, CPX *accum)
{

	int32 cnt1;
	int32 cnt2;

	cnt1 = cnt / 2;
	cnt2 = (cnt - (2*cnt1));
	cnt1 = 0;
	cnt2 = cnt;

	__asm
	(
		".intel_syntax noprefix			\n\t" //Set up for loop
                "push ebx\n\t"
		"mov esi, [ebp+8]				\n\t" //Address of A
		"mov eax, [ebp+12]				\n\t" //Address of E
		"mov ebx, [ebp+16]				\n\t" //Address of P
		"mov edx, [ebp+20]				\n\t" //Address of L
		"mov ecx, [ebp-12]				\n\t" //Counter 1
		"pxor mm5, mm5					\n\t" //Clear the running sum for E
		"pxor mm6, mm6					\n\t" //Clear the running sum for P
		"pxor mm7, mm7					\n\t" //Clear the running sum for L
		"jecxz Z%=						\n\t"
		"L%=:							\n\t"
		"	movq mm0, [esi]				\n\t" //load IF data
		"	movq mm1, [eax]				\n\t" //load E data
		"	movq mm2, [ebx]				\n\t" //load P data
		"	movq mm3, [edx]				\n\t" //load L data
		"	movq mm4, mm0 				\n\t" //load IF data
		"	pand mm0, mm1				\n\t" //mask it
		"	pandn mm1, mm4				\n\t" //mask it
		"	paddsw mm5, mm1				\n\t" //add
		"	psubsw mm5, mm0				\n\t" //subtract
		"	movq mm0, mm4 				\n\t" //load IF data
		"	pand mm0, mm2				\n\t" //mask it
		"	pandn mm2, mm4				\n\t" //mask it
		"	paddsw mm6, mm2				\n\t" //add
		"	psubsw mm6, mm0				\n\t" //subtract
		"	movq mm0, mm4 				\n\t" //load IF data
		"	pand mm0, mm3				\n\t" //mask it
		"	pandn mm3, mm4				\n\t" //mask it
		"	paddsw mm7, mm3				\n\t" //add
		"	psubsw mm7, mm0				\n\t" //subtract
		"	add esi, 8					\n\t" //move in complex sine by 24 bytes
		"	add eax, 8					\n\t" //move in IF array by 48 bytes
		"	add ebx, 8					\n\t" //move in IF array by 48 bytes
		"	add edx, 8					\n\t" //move in IF array by 48 bytes
		"loop L%=						\n\t" //Loop if not done
		"Z%=:							\n\t"
		"mov ecx, [ebp-16]				\n\t"
		"jecxz ZZ%=						\n\t"
		"LL%=:							\n\t"
		"	movd mm0, [esi]				\n\t" //load IF data
		"	movd mm1, [eax]				\n\t" //load E data
		"	movd mm2, [ebx]				\n\t" //load P data
		"	movd mm3, [edx]				\n\t" //load L data
		"	movq mm4, mm0 				\n\t" //load IF data
		"	pand mm0, mm1				\n\t" //mask it
		"	pandn mm1, mm4				\n\t" //mask it
		"	paddsw mm5, mm1				\n\t" //add
		"	psubsw mm5, mm0				\n\t" //subtract
		"	movq mm0, mm4 				\n\t" //load IF data
		"	pand mm0, mm2				\n\t" //mask it
		"	pandn mm2, mm4				\n\t" //mask it
		"	paddsw mm6, mm2				\n\t" //add
		"	psubsw mm6, mm0				\n\t" //subtract
		"	movq mm0, mm4 				\n\t" //load IF data
		"	pand mm0, mm3				\n\t" //mask it
		"	pandn mm3, mm4				\n\t" //mask it
		"	paddsw mm7, mm3				\n\t" //add
		"	psubsw mm7, mm0				\n\t" //subtract
		"	add esi, 4					\n\t" //move in complex sine by 24 bytes
		"	add eax, 4					\n\t" //move in IF array by 48 bytes
		"	add ebx, 4					\n\t" //move in IF array by 48 bytes
		"	add edx, 4					\n\t" //move in IF array by 48 bytes
		"loop LL%=						\n\t" //Loop if not done
		"ZZ%=:							\n\t"
		"mov 		esi, [ebp+28]		\n\t"
		"movq		mm0, mm5			\n\t"
		"punpckhdq 	mm0, mm0			\n\t"
		"paddsw 	mm5, mm0			\n\t"
		"movd		[esi], mm5			\n\t"
		"add esi, 4						\n\t"
		"movq		mm0, mm6			\n\t"
		"punpckhdq 	mm0, mm0			\n\t"
		"paddsw 	mm6, mm0			\n\t"
		"movd		[esi], mm6			\n\t"
		"add esi, 4						\n\t"
		"movq		mm0, mm7			\n\t"
		"punpckhdq 	mm0, mm0			\n\t"
		"paddsw 	mm7, mm0			\n\t"
		"movd		[esi], mm7			\n\t"
                "pop ebx\n\t"
		"EMMS							\n\t"
		".att_syntax					\n\t"
		:
		: "m" (A), "m" (E), "m" (P), "m" (L), "m" (cnt), "m" (accum), "m" (cnt1), "m" (cnt2)
		: "%eax", "%ecx", "%edx", "%esi"
	);//end __asm

}



//!< A must hold baseband data, E,P,L must hold PRN data
void sse_prn_accum_new(CPX *A, MIX *E, MIX *P, MIX *L, int32 cnt, CPX_ACCUM *accum)
{

	__asm
	(
		".intel_syntax noprefix			\n\t" //Set up for loop
                "push ebx\n\t"
		"mov esi, [ebp+8]				\n\t" //Address of A
		"mov eax, [ebp+12]				\n\t" //Address of E
		"mov ebx, [ebp+16]				\n\t" //Address of P
		"mov edx, [ebp+20]				\n\t" //Address of L
		"mov ecx, [ebp+24]				\n\t" //Value of cnt
		"pxor mm5, mm5					\n\t" //Clear the running sum for E
		"pxor mm6, mm6					\n\t" //Clear the running sum for P
		"pxor mm7, mm7					\n\t" //Clear the running sum for L
		"jecxz Z%=						\n\t"
		"L%=:							\n\t"
		"	movd mm0, [esi]				\n\t" //load IF data
		"	movq mm1, [eax]				\n\t" //load E data
		"	movq mm2, [ebx]				\n\t" //load P data
		"	movq mm3, [edx]				\n\t" //load L data
		"	punpckldq mm0, mm0			\n\t" //copy low 32 bits to high 32 pits
		"   pmaddwd mm1, mm0			\n\t" //complex multiply E by IF
		"   pmaddwd mm2, mm0			\n\t" //complex multiply P by IF
		"   pmaddwd mm3, mm0			\n\t" //complex multiply L by IF
		"	paddd mm5, mm1				\n\t" //add into E accumulator
		"	paddd mm6, mm2				\n\t" //add into E accumulator
		"	paddd mm7, mm3				\n\t" //add into E accumulator
		"	add esi, 4					\n\t" //move in baseband data by one sample (4 bytes)
		"	add eax, 8					\n\t" //move in PRN-E array by one sample (8 bytes)
		"	add ebx, 8					\n\t" //move in PRN-P array by one sample (8 bytes)
		"	add edx, 8					\n\t" //move in PRN-L array by one sample (8 bytes)
		"loop L%=						\n\t" //Loop if not done
		"Z%=:							\n\t"
		"mov 		esi, [ebp+28]		\n\t"
		"movq		[esi], mm5			\n\t"
		"add esi, 8						\n\t"
		"movq		[esi], mm6			\n\t"
		"add esi, 8						\n\t"
		"movq		[esi], mm7			\n\t"
                "pop ebx\n\t"
		"EMMS							\n\t"
		".att_syntax					\n\t"
		:
		: "m" (A), "m" (E), "m" (P), "m" (L), "m" (cnt), "m" (accum)
		: "%eax", "%ecx", "%edx", "%esi"
	);//end __asm

}

#endif



