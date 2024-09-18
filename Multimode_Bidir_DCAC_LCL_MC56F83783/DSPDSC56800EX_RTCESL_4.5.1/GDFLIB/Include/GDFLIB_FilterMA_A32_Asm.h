/*******************************************************************************
*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*
****************************************************************************//*!
*
* @brief  Moving Average Filter in assembler
* 
*******************************************************************************/
#ifndef _GDFLIB_FILTERMA_A32_ASM_H_
#define _GDFLIB_FILTERMA_A32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"

/******************************************************************************
* Macros 
******************************************************************************/
#define GDFLIB_FilterMAInit_F16_Asmi(f16InitVal, psParam) GDFLIB_FilterMAInit_F16_FAsmi(f16InitVal, psParam)

#define GDFLIB_FilterMA_F16_Asmi(f16InX, psParam) GDFLIB_FilterMA_F16_FAsmi(f16InX, psParam)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	acc32_t		a32Acc;
	uint16_t	u16Sh;
} GDFLIB_FILTER_MA_T_A32;

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function initializes the moving average filter with the predefined value.
*
* @param  ptr   		GDFLIB_FILTER_MA_T_F16 *psParam
*						  	- Pointer to filter structure
*							
*							GDFLIB_FILTER_MA_T_F16 structure:
*						
*							- a32Acc - accumulator of the filter
*							- u16Sh - 2 ^ u16Sh is the number of values
*
* @param  in    		frac16_t f16InitVal
* 							- Initial value
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern inline void GDFLIB_FilterMAInit_F16_FAsmi(register frac16_t f16InitVal, register GDFLIB_FILTER_MA_T_A32 *psParam)
{
	register uint16_t u16Sh, u16Clb;
	register acc32_t a32Acc, a32Temp, a32Temp1;

	asm(.optimize_iasm on);	
	asm(move.w	X:(psParam+2),u16Sh);		/* u16Sh = psParam -> u16Sh */
	asm(asr16	f16InitVal,a32Acc);			/* a32Acc = f16InitVal >> 16 */
	asm(tfr		a32Acc,a32Temp);			/* a32Temp = a32Acc */
	asm(clb		a32Acc,u16Clb);				/* u16Clb is number of leading bits ofa32Acc */
	asm(asll.l 	u16Sh,a32Acc);				/* A = A << psParam -> u16Sh (arithmetically) */
	asm(asr16   f16InitVal,a32Temp1);		/* a32Temp = f16Sub >> 16 */	
	asm(sub     a32Temp1,a32Acc);			/* a32Accum = a32Accum - f16Sub */	
	asm(bfchg	#0x8000,a32Temp.1);			/* changes the MSB of a32Temp */
	asm(cmp.w	u16Sh,u16Clb);				/* if psParam -> u16Sh < u16Clb */
	asm(tlt		a32Temp,a32Acc);			/* if no space to shift, uses a32Temp */
	asm(sat		a32Acc);					/* saturates a32Acc */
	asm(move.l	a32Acc,X:(psParam));		/* psParam -> a32Acc = a32Acc */ 
	asm(.optimize_iasm off);
}

/***************************************************************************//*!
*
* @brief  The function calculates recursive form of moving average filter
*
* @param  ptr   		GDFLIB_FILTER_MA_T_A32 *psParam
*						  	- Pointer to filter structure
*							
*							GDFLIB_FILTER_MA_T_A32 structure:
*						
*							- a32Acc - accumulator of the filter
*							- u16Sh - 2 ^ u16Sh is the number of values
*
* @param  in    		f16InX - input signal
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 
*
* Filter Equation:
* 
* sum(k) 	= sum(k-1) + input
* out(k)	= sum(k)/N
* sum(k-1)	= sum(k) - out(k)
*
*			THE SATURATION MUST BE TURNED OFF.
*
****************************************************************************/
extern inline frac16_t GDFLIB_FilterMA_F16_FAsmi(register frac16_t f16InX, register GDFLIB_FILTER_MA_T_A32 *psParam)
{
	register frac16_t f16Out;
	register acc32_t a32Acc, a32In, a32Temp;
	register uint16_t u16Sh, u16Clb;
	register uint16_t * pu16Sh; 

	asm(.optimize_iasm on);
	asm(adda	#2,psParam,pu16Sh);					/* pu16Sh points to psParam -> u16Sh */
	asm(move.l  X:(psParam),a32Acc);				/* a32Acc = psParam -> f32Acc */
	asm(asr16   f16InX,a32In);						/* a32In = f16InX >> 16 */
	asm(add     a32In,a32Acc	X:(pu16Sh)+,u16Sh); /* a32Acc = psParam -> f32Acc + (f16InX >> 16) */
													/* u16Sh = psParam -> u16Sh */
	asm(tfr		a32Acc,a32Temp);					/* a32Temp = a32Acc */
	asm(sat		a32Temp);							/* saturates a32Temp */
	asm(asrr.l	u16Sh,a32Temp);						/* a32Temp = a32Temp >> u16Sh */
	asm(sub		a32Temp,a32Acc);					/* a32Acc = a32Acc - a32Temp */
	asm(sat		a32Acc);							/* saturates a32Acc */
	asm(move.l	a32Acc,X:(psParam));				/* psParam -> f32Acc = a32Acc */

	asm(tfr		a32Temp,a32Acc);					/* a32Acc = a32Temp */
    asm(clb 	a32Temp,u16Clb);                	/* acquire number of leading ones/zeros minus one of a32Temp */
    asm(asl16 	a32Acc);							/* a32Acc = a32Acc << 16 (arithmetically) */
	asm(bfchg 	#0x8000,a32Temp.1);					/* changes the MSB of a32Acc */
    asm(cmp.w 	#16,u16Clb);               			/* u16Clb - 16 -> comparing the number of leading ones of the result */
	asm(tlt		a32Temp,a32Acc);					/* if no space to shift, uses a32Temp */
	asm(sat		a32Acc);							/* saturates a32Acc */
	asm(rnd 	a32Acc);							/* rounds a32Acc to 16 bits */
	asm(move.w	a32Acc,f16Out);						/* result conversion to 16 bits with saturation */
	asm(.optimize_iasm off);

	return(f16Out);
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GDFLIB_FILTERMA_A32_ASM_H_ */
