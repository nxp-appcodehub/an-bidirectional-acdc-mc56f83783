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
* @brief  Division functions with 32-bit accumulator output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_DIV_A32_ASM_H_
#define _MLIB_DIV_A32_ASM_H_

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
/* Accumulator-output division */
#define MLIB_Div_A32ss_Asmi(f16Num, f16Denom) MLIB_Div_A32ss_FAsmi(f16Num, f16Denom)
#define MLIB_Div_A32ls_Asmi(f32Num, f16Denom) MLIB_Div_A32ls_FAsmi(f32Num, f16Denom)
#define MLIB_Div_A32ll_Asm(f32Num, f32Denom) MLIB_Div_A32ll_FAsm(f32Num, f32Denom)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
extern acc32_t MLIB_Div_A32ll_FAsm(frac32_t f32Num, frac32_t f32Denom);

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  16-bit numerator, 16-bit denominator inputs 32-bit accumulator output
* 		  4-quadrant division function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Num
*                         - Numerator in [-1;1] in frac16_t
*						frac16_t f16Denom
*                         - Denominator in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - acc32_t value [-65536.0;65536.0 - (2^-15)]
*		
* @remarks 	This function divides two negative fractional inputs:
* 			result = f16Num / f16Denom.
* 			The function normalises the inputs to get higher precision of
* 			division. The output is the accumulator type where the lower
* 			word is the fractional part.
* 			If the denominator is 0, the output is 0x7FFF FFFF or 0x8000 0000
* 			depending on the numerator's sign. For the both are 0, the output 
* 			is 0x7FFF FFFF.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline acc32_t MLIB_Div_A32ss_FAsmi(register frac16_t f16Num, register frac16_t f16Denom)
{
	register int16_t w16ClbNum, w16ClbDenom;
	register acc32_t a32Result;
	register frac32_t f32NumTemp, f32Copy;
		
	asm(.optimize_iasm on);

	asm(move.w f16Num,f32NumTemp);

	asm(clb f32NumTemp,w16ClbNum);		/* w16ClbNum = number of leading bits of f16Num */
	asm(sub.w #1,w16ClbNum);			/* w16ClbNum = w16ClbNum - 1 because we need to have f16Num 0.25 to 0.5 */
	asm(asll.l w16ClbNum,f32NumTemp);	/* Normalisation of f16Num to the range 0.25 to 0.5 */	
	asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
	
	asm(abs f32NumTemp);				/* Absolute value of the numerator */
		
	asm(asll.l w16ClbDenom,f16Denom);	/* Normalisation of f16Denom to 0.5 to 1.0 */
	asm(sub w16ClbNum,w16ClbDenom);		/* w16ClbDenom = shifts of f16Denom - shifts of w16ClbNum */
	
	asm(tst f32NumTemp);				/* Clears the C flag */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32NumTemp);		/* f32NumTemp = f32NumTemp / f16Denom */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32NumTemp);		/* f32NumTemp = f32NumTemp / f16Denom */
	
	asm(move.w f32NumTemp.0,f32NumTemp);/* f32NumTemp = f32NumTemp << 16 */	
	asm(tfr f32NumTemp,f32Copy);		/* Copy of the result */			

	asm(neg	f32NumTemp);				/* Negates the result */
	asm(eor.w f16Denom,f16Num);			/* Save sign bit of quotient in N bit of SR */
	asm(nop);
	asm(tlt	f32NumTemp,f32Copy);		/* f32Copy = -f32NumTemp if different signs */

	asm(tfr f32Copy,f32NumTemp);		/* Copy of the result */
	asm(asr16 f32Copy,a32Result);		/* a32Result = f32Copy >> 16 */ 
	asm(asll.l w16ClbDenom,a32Result);	/* a32Result = a32Result << w16ClbDenom (arithmetically) */
		
	asm(bfchg #0x8000,f32NumTemp.1);	/* Changes the MSB of the result */
	asm(sat f32NumTemp);
		
	asm(tst.w f16Denom);				/* Leading bits of denominator, comparison to 15 */
	asm(teq f32NumTemp,a32Result);		/* In case of 0, uses the maximum output */
		
	asm(.optimize_iasm off);
		
	return a32Result;
}

/***************************************************************************//*!
*
* @brief  32-bit numerator, 16-bit denominator inputs 32-bit accumulator output
* 		  4-quadrant division function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Num
*                         - Numerator in [-1;1] in frac32_t
*						frac16_t f16Denom
*                         - Denominator in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - acc32_t value [-65536.0;65536.0 - (2^-15)]
*		
* @remarks 	This function divides two negative fractional inputs:
* 			result = f32Num / f16Denom.
* 			The function normalises the inputs to get higher precision of
* 			division. The output is the accumulator type where the lower
* 			word is the fractional part.
* 			If the denominator is 0, the output is 0x7FFF FFFF or 0x8000 0000
* 			depending on the numerator's sign. For the both are 0, the output 
* 			is 0x7FFF FFFF.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline acc32_t MLIB_Div_A32ls_FAsmi(register frac32_t f32Num, register frac16_t f16Denom)
{
	register int16_t w16ClbNum, w16ClbDenom;
	register acc32_t a32Result;
	register frac32_t f32NumTemp, f32Copy;
	
	asm(.optimize_iasm on);

	asm(tfr f32Num,f32NumTemp);

	asm(clb f32NumTemp,w16ClbNum);		/* w16ClbNum = number of leading bits of f32Num */
	asm(sub.w #1,w16ClbNum);			/* w16ClbNum = w16ClbNum - 1 because we need to have f16Num 0.25 to 0.5 */
	asm(asll.l w16ClbNum,f32NumTemp);	/* Normalisation of f16Num to the range 0.25 to 0.5 */	
	asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
	
	asm(abs f32NumTemp);				/* Absolute value of the numerator */
		
	asm(asll.l w16ClbDenom,f16Denom);	/* Normalisation of f16Denom to 0.5 to 1.0 */
	asm(sub w16ClbNum,w16ClbDenom);		/* w16ClbDenom = shifts of f16Denom - shifts of w16ClbNum */
	
	asm(tst f32NumTemp);				/* Clears the C flag */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32NumTemp);		/* f32NumTemp = f32NumTemp / f16Denom */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32NumTemp);		/* f32NumTemp = f32NumTemp / f16Denom */
	
	asm(move.w f32NumTemp.0,f32NumTemp);/* f32NumTemp = f32NumTemp << 16 */	
	asm(tfr f32NumTemp,f32Copy);		/* Copy of the result */			

	asm(neg	f32NumTemp);				/* Negates the result */
	asm(eor.w f16Denom,f32Num);			/* Save sign bit of quotient in N bit of SR */
	asm(nop);
	asm(tlt	f32NumTemp,f32Copy);		/* f32Copy = -f32NumTemp if different signs */

	asm(tfr f32Copy,f32NumTemp);		/* Copy of the result */
	asm(asr16 f32Copy,a32Result);		/* a32Result = f32Copy >> 16 */ 
	asm(asll.l w16ClbDenom,a32Result);	/* a32Result = a32Result << w16ClbDenom (arithmetically) */
		
	asm(bfchg #0x8000,f32NumTemp.1);	/* Changes the MSB of the result */
	asm(sat f32NumTemp);
		
	asm(tst.w f16Denom);				/* Leading bits of denominator, comparison to 15 */
	asm(teq f32NumTemp,a32Result);		/* In case of 0, uses the maximum output */
		
	asm(.optimize_iasm off);
		
	return a32Result;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_DIV_A32_ASM_H_ */

