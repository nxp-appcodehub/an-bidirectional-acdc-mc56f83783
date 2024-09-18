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
* @brief  Single quadrant reciprocal functions with 32-bit accumulator output 
* 		  in assembler
* 
*******************************************************************************/
#ifndef _MLIB_RCP1Q_A32_ASM_H_
#define _MLIB_RCP1Q_A32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
/* Reciprocal */
#define MLIB_Rcp1Q1_A32s_Asmi(f16Denom) MLIB_Rcp1Q1_A32s_FAsmi(f16Denom)
#define MLIB_Rcp1Q_A32s_Asmi(f16Denom) MLIB_Rcp1Q_A32s_FAsmi(f16Denom)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  16-bit input 32-output 16-bit precision single quadrant reciprocal function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Denom
*                         - Denominator in [0;1] in frac16_t
*                       
*
* @return This function returns
*     - acc32_t value [0;65536.0 - (2^-15)]
*		
* @remarks 	This function calculates the multiplicative inverse value of
* 			the non-negative fractional input:
* 			result = FRAC16(1) / f16Denom. The function calculates
* 			the result with 16-bit division precision.
* 			The function normalizes the inputs to get higher precision of
* 			division.
* 			If the denominator is 0, the output is 0x7FFF FFFF FFFF FFFF.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline acc32_t MLIB_Rcp1Q1_A32s_FAsmi(register frac16_t f16Denom)
{
	register frac32_t f32Result;
	register int16_t w16ClbDenom;
	register frac32_t f32Num;	
		
	asm(.optimize_iasm on);
		
	asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
	asm(move.w #0x4000,f32Num);			/* f32Num = FRAC16(0.5) */
	asm(asll.l w16ClbDenom,f16Denom);	/* normalization of f16Denom to 0.5 to 1.0 */
	asm(sub.w #15,w16ClbDenom);			/* Add 1 because the numerator is 0.5 (not 1) */ 
	
	asm(tst f32Num);					/* Clears the C flag */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32Num);			/* f32Num = f32Num / f16Denom */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32Num);			/* f32Num = f32Num / f16Denom */
		
	asm(move.w f32Num.0,f32Num);		/* f32Result = f32Result << 16 */
	asm(tfr f32Num,f32Result);			/* Copy of the result */			

	asm(asll.l w16ClbDenom,f32Result);	/* f32Result = f32Result << w16ClbDenom (arithmetically) */

	asm(bfchg #0x8000,f32Num.1);		/* changes the MSB of the result */		

	asm(cmp.w #15,w16ClbDenom);			/* Leading bits comparison */

	asm(tgt f32Num,f32Result);			/* In case of result's overflow, uses the maximum output */
		
	asm(sat f32Result);					/* saturation */
		
	asm(.optimize_iasm off);
		
	return f32Result;
}

/***************************************************************************//*!
*
* @brief  16-bit input 32-output 32-bit precision quadrant reciprocal function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Denom
*                         - Denominator in [0;1] in frac16_t
*                       
*
* @return This function returns
*     - acc32_t value [0;65536.0 - (2^-15)]
*		
* @remarks 	This function calculates the multiplicative inverse value of
* 			the non-negative fractional input:
* 			result = FRAC16(1) / f16Denom. The function calculates
* 			the result with 32-bit division precision.
* 			The function normalizes the inputs to get higher precision of
* 			division.
* 			If the denominator is 0, the output is 0x7FFF FFFF.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline acc32_t MLIB_Rcp1Q_A32s_FAsmi(register frac16_t f16Denom)
{
	register frac32_t f32Result;
	register int16_t w16ClbDenom;
	register frac32_t f32Num, f32Copy;	
		
	asm(.optimize_iasm on);
		
	asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
	asm(move.w #0x4000,f32Num);			/* f32Num = FRAC16(0.5) */
	asm(asll.l w16ClbDenom,f16Denom);	/* normalization of f16Denom to 0.5 to 1.0 */
	asm(sub.w #15,w16ClbDenom);			/* Sub 15 because the numerator is 1 << 15 */ 
	
	asm(tst f32Num);					/* Clears the C flag */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32Num);			/* f32Num = f32Num / f16Denom */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32Num);			/* f32Num = f32Num / f16Denom */

	asm(move.w f32Num.0,f32Copy);		/* Upper 16 bits of the result */ 

	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32Num);			/* f32Num = f32Num / f16Denom */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32Num);			/* f32Num = f32Num / f16Denom */

	asm(move.w f32Num.0,f32Copy.0);		/* Lower 16 bits of the result */
		
	asm(tfr f32Copy,f32Result);			/* Copy of the result */

	asm(asll.l w16ClbDenom,f32Result);	/* f32Result = f32Result << w16ClbDenom (arithmetically) */
	
	asm(bfchg #0x8000,f32Copy.1);		/* changes the MSB of the result */		

	asm(cmp.w #15,w16ClbDenom);			/* Leading bits comparison */

	asm(tgt f32Copy,f32Result);			/* In case of result's overflow, uses the maximum output */

	asm(sat f32Result);					/* Saturation */
		
	asm(.optimize_iasm off);
		
	return f32Result;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_RCP1Q_A32_ASM_H_ */
