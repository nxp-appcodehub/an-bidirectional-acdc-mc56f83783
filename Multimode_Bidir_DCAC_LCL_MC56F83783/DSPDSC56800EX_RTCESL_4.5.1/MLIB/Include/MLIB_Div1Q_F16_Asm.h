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
* @brief  One-quadrant division functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_DIV1Q_F16_ASM_H_
#define _MLIB_DIV1Q_F16_ASM_H_

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
/* Non-saturated division */
#define MLIB_Div1Q_F16_Asmi(f16Num, f16Denom) MLIB_Div1Q_F16_FAsmi(f16Num, f16Denom)

/* Saturated division */
#define MLIB_Div1QSat_F16_Asmi(f16Num, f16Denom) MLIB_Div1QSat_F16_FAsmi(f16Num, f16Denom)

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
* @brief  16-bit inputs 16-output single quadrant division function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Num
*                         - Numerator in [0;1] in frac16_t
*						frac16_t f16Denom
*                         - Denominator in [0;1] in frac16_t
*                       
*
* @return This function returns
*     - frac16_t value [0;1]
*		
* @remarks 	This function divides two non-negative fractional inputs:
* 			result = f16Num / f16Denom.
* 			The function normalises the inputs to get higher precision of
* 			division.
* 			The function does not saturate the output.
* 			If the denominator is 0, the output is 0x7FFF. 			
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_Div1Q_F16_FAsmi(register frac16_t f16Num, register frac16_t f16Denom)
{
	register int16_t w16ClbNum, w16ClbDenom;
	register frac16_t f16Out;
	register frac32_t f32NumTemp, f32Copy;
		
	asm(.optimize_iasm on);
	
	asm(.iasm_sideeffects off;	.iasm_reg2regsetcopyflag off;
		move.w f16Num,f32NumTemp;	
		.iasm_sideeffects on;	.iasm_reg2regsetcopyflag on);
		
	asm(clb f32NumTemp,w16ClbNum);		/* w16ClbNum = number of leading bits of f16Num */
		
	asm(sub.w #1,w16ClbNum);			/* w16ClbNum = w16ClbNum - 1 because we need to have f16Num 0.25 to 0.5 */
	
	asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
		
	asm(asll.l w16ClbNum,f32NumTemp);	/* Normalisation of f16Num to the range 0.25 to 0.5 */	
		
	asm(asll.l w16ClbDenom,f16Denom);	/* Normalisation of f16Denom to 0.5 to 1.0 */
	
	asm(sub w16ClbNum,w16ClbDenom);		/* w16ClbDenom = shifts of f16Denom - shifts of w16ClbNum */
	
	asm(tst f32NumTemp);				/* Clears the C flag */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32NumTemp);		/* f32Num = f32NumTemp / f16Denom */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32NumTemp);		/* f32Num = f32NumTemp / f16Denom */
		
	asm(move.w f32NumTemp.0,f32NumTemp);/* f32NumTemp = f32NumTemp << 16 */	
	asm(tfr f32NumTemp,f32Copy);		/* Copy of the result */			

	asm(asll.l w16ClbDenom,f32Copy);	/* f32Copy = f32Copy << w16ClbDenom (arithmetically) */
	
	asm(bfchg #0x8000,f32NumTemp.1);	/* Changes the MSB of the result */		

	asm(tst.w f16Denom);				/* Leading bits of denominator, comparison to 15 */
		
	asm(teq f32NumTemp,f32Copy);		/* In case of 0, uses the maximum output */
				
	asm(move.w f32Copy,f16Out);			/* Saturation */
		
	asm(.optimize_iasm off);
		
	return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit inputs 16-output single quadrant division function with saturation
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Num
*                         - Numerator in [0;1] in frac16_t
*						frac16_t f16Denom
*                         - Denominator in [0;1] in frac16_t
*                       
*
* @return This function returns
*     - frac16_t value [0;1]
*		
* @remarks 	This function divides two non-negative fractional inputs:
* 			result = f16Num / f16Denom.
* 			The function normalises the inputs to get higher precision of
* 			division.
* 			The function saturates the output if f16Num > f16Denom
* 			to 0x7FFF.	
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_Div1QSat_F16_FAsmi(register frac16_t f16Num, register frac16_t f16Denom)
{
	register int16_t w16ClbNum, w16ClbDenom, w16ClbResult;
	register frac16_t f16Out;
	register frac32_t f32NumTemp, f32Copy;
		
	asm(.optimize_iasm on);
	
	asm(.iasm_sideeffects off;	.iasm_reg2regsetcopyflag off;
		move.w f16Num,f32NumTemp;	
		.iasm_sideeffects on;	.iasm_reg2regsetcopyflag on);
		
	asm(clb f32NumTemp,w16ClbNum);		/* w16ClbNum = number of leading bits of f16Num */
		
	asm(sub.w #1,w16ClbNum);			/* w16ClbNum = w16ClbNum - 1 because we need to have f16Num 0.25 to 0.5 */
	
	asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
		
	asm(asll.l w16ClbNum,f32NumTemp);	/* Normalisation of f16Num to the range 0.25 to 0.5 */	
		
	asm(asll.l w16ClbDenom,f16Denom);	/* Normalisation of f16Denom to 0.5 to 1.0 */
	
	asm(sub w16ClbNum,w16ClbDenom);		/* w16ClbDenom = shifts of f16Denom - shifts of w16ClbNum */
	
	asm(tst f32NumTemp);				/* Clears the C flag */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32NumTemp);		/* f32Num = f32NumTemp / f16Denom */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,f32NumTemp);		/* f32Num = f32NumTemp / f16Denom */
		
	asm(move.w f32NumTemp.0,f32NumTemp);/* f32NumTemp = f32NumTemp << 16 */	
	asm(tfr f32NumTemp,f32Copy);		/* Copy of the result */			

	asm(clb f32NumTemp,w16ClbResult);	/* Leading bits of the result */

	asm(asll.l w16ClbDenom,f32Copy);	/* f32Copy = f32Copy << w16ClbDenom (arithmetically) */
	
	asm(bfchg #0x8000,f32NumTemp.1);	/* Changes the MSB of the result */		

	asm(cmp.w w16ClbDenom,w16ClbResult);/* Leading bits comparison */
		
	asm(tlt f32NumTemp,f32Copy);		/* In case of result's overflow, uses the maximum output */
		
	asm(move.w f32Copy,f16Out);			/* Saturation */
		
	asm(.optimize_iasm off);
		
	return f16Out;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_DIV1Q_F16_ASM_H_ */

