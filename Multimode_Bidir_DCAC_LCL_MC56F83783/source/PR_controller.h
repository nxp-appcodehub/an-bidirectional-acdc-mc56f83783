/*
 * Copyright 2023-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "mlib.h"

#define CFD      27 /* Coefficient fractional digits */
#define IFD      27 /* Internal result fractional digits */

/* PR(s) = kp + (2*s*wc*ki)/(s^2+2*s*wc+w0^2) */
typedef struct
{
	acc32_t       kp;      
	acc32_t       ki;
	acc32_t       wc;      /* when the signal frequency deviates this value, the controller still maintains a high gain*/
	acc32_t       w0;      /* controlled signal frequency */
	acc32_t       fs;      /* sampling frequency */
}ACDCSTRUC_PR_PARAMS_T;

typedef struct
{
	int32_t i32UpperLim;
	int32_t i32LowerLim;
}FILTER_LIM_T;

typedef struct
{
	int32_t i32B0;         // +4 ,B0
	int32_t i32B1;         // +6 ,B1
	int32_t i32A1;         // +8 ,A1
	int32_t i32B2;         // +10,B2
	int32_t i32A2;         // +12,A2
}COEFF_2P2Z_TRANS_T;

typedef struct
{
	int32_t i32W1;
	int32_t i32W2;
	COEFF_2P2Z_TRANS_T sCoeff;
	FILTER_LIM_T sLim;
}FILTER_2P2Z_TRANS_T;

#define FRAC_Q31(x) x*2147483648
#define FRAC_Q30(x) x*1073741824
#define FRAC_Q29(x) x*536870912
#define FRAC_Q28(x) x*268435456
#define FRAC_Q27(x) x*134217728
#define FRAC_Q26(x) x*67108864
#define FRAC_Q25(x) x*33554432
#define FRAC_Q24(x) x*16777216
#define FRAC_Q23(x) x*8388608
#define FRAC_Q22(x) x*4194304
#define FRAC_Q21(x) x*2097152
#define FRAC_Q20(x) x*1048576
#define FRAC_Q19(x) x*524288
#define FRAC_Q18(x) x*262144
#define FRAC_Q17(x) x*131072
#define FRAC_Q16(x) x*65536
#define FRAC_Q15(x) x*32768
#define FRAC_Q14(x) x*16384
#define FRAC_Q13(x) x*8192
#define FRAC_Q12(x) x*4096
#define FRAC_Q11(x) x*2048
#define FRAC_Q10(x) x*1024
#define FRAC_Q9(x) x*512
#define FRAC_Q8(x) x*256
#define FRAC_Q7(x) x*128
#define FRAC_Q6(x) x*64
#define FRAC_Q5(x) x*32
#define FRAC_Q4(x) x*16
#define FRAC_Q3(x) x*8
#define FRAC_Q2(x) x*4
#define FRAC_Q1(x) x*2

#define FRAC_DYN(y,x) FRAC_DYN1(y,x)
#define FRAC_DYN1(y,x) FRAC_Q##y(x)

/******************************************************************************
* Global functions
******************************************************************************/
extern void ComputePRCoeff(ACDCSTRUC_PR_PARAMS_T *concoef, FILTER_2P2Z_TRANS_T *discoef);

/* Transposed Direct form II - Fractional in Assembly language, with MAC and parallel move - Inline version. 65 cycles when there's no saturation. */
inline frac32_t IIR_2P2Z_II_TRANS_LIM_mac_asm_inline(register frac16_t f16In, register FILTER_2P2Z_TRANS_T *ptr)
{
	// a -> f32In, r2 -> ptr
	/* 
	 *  typedef struct
		{
			int32_t i32B0;         // +4 ,B0 8EA
			int32_t i32B1;         // +6 ,B1 8EC
			int32_t i32A1;         // +8 ,A1 8EE
			int32_t i32B2;         // +10,B2 8F0
			int32_t i32A2;         // +12,A2 8F2
		}COEFF_2P2Z_TRANS_T;
		
		typedef struct
		{
			int32_t i32W1;         // +0 ,W1 8E6
			int32_t i32W2;         // +2 ,W2 8E8
			COEFF_2P2Z_T sCoeff;
		}FILTER_2P2Z_TRANS_T;
		
		step 1: out = w2 + in*b0
		step 2: w2 = a1*out + b1*in + w1
		step 3: w1 = a2*out + b2*in
		
	 * */
	/*     y is the input in Q(IFD) format,
	 *     b is the updated output in Q(IFD) format
	 * 
	 * */

		register int32_t *ptr_alt;
		register int32_t *ptr_lim;
		register int32_t i32OutReg;
	
	asm{
		.optimize_iasm on
		move.w f16In,a
		asrr.l #(31-IFD),a       // a = IN, Q(IFD)
		tfr    a,y               // y = IN		
		adda   #14,ptr,ptr_lim   // ptr_lim->UpperLim
		adda   #2,ptr,ptr_alt    // ptr_alt->W2, ptr->W1
		move.l x:(ptr_alt)+,a    // a = W2, ptr_alt->B0
		move.l x:(ptr_alt)+,c    // c = B0, ptr_alt->B1
		mpy32  c,y,c             // c = B0*IN
		asll.l #(31-CFD),c
		add    c,a               // a = B0*IN + W2, now a is updated output
		
		move.l x:(ptr_lim)+,c    // d = UpperLim, ptr_lim->LowerLim
		cmp    c,a
		bge    LIM_OUTPUT
		move.l x:(ptr_lim)-,c    // d = LowerLim, ptr_lim->UpperLim
		cmp    c,a
		blt    LIM_OUTPUT
		tfr    a,c
		
LIM_OUTPUT:
		tfr    c,b               // b is updated output
		
		move.l x:(ptr_alt)+,c    // c = B1, ptr_alt->A1
		mpy32  c,y,a x:(ptr_alt)+,c       // a = B1*IN, c = A1, ptr_alt->B2
		mac32  b,c,a             // a = B1*IN + A1*OUT
		asll.l #(31-CFD),a
		move.l x:(ptr)+,c        // c = W1, ptr->W2
		add    c,a               // a = W1 + B1*IN + A1*OUT
		
		move.l x:(ptr_lim)+,c    // d = UpperLim, ptr_lim->LowerLim
		cmp    c,a
		bge    LIM_INTEGRAL
		move.l x:(ptr_lim)-,c    // d = LowerLim, ptr_lim->UpperLim
		cmp    c,a
		blt    LIM_INTEGRAL
		tfr    a,c

LIM_INTEGRAL:		
		move.l c10, x:(ptr)-     // Store W2, ptr->W1
		
		move.l x:(ptr_alt)+,c    // c = B2, ptr_alt->A2
		mpy32  c,y,a x:(ptr_alt)+,c       // a = B2*IN, c = A2
		mac32  c,b,a             // a = B2*IN + A2*OUT
		asll.l #(31-CFD),a
		move.l a10, x:(ptr)      // Store W1
		
		asll.l #(31-IFD),b       // Q(IFD)->Q1.31
		tfr    b,i32OutReg
		.optimize_iasm off
	}
	
	return i32OutReg;
}

inline void PRCtrlInit(FILTER_2P2Z_TRANS_T *psParam)
{
	psParam->i32W1 = (frac32_t)0;
	psParam->i32W2 = (frac32_t)0;
}

#endif /* CONTROLLER_H_ */
