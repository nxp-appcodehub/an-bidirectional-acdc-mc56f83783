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
* @brief  Saturation functions with 32-bit accumulator input in assembler
* 
*******************************************************************************/
#ifndef _MLIB_SAT_A32_ASM_H_
#define _MLIB_SAT_A32_ASM_H_

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
#define MLIB_Sat_F16a_Asmi(a32Accum) MLIB_Sat_F16a_FAsmi(a32Accum)

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
* @brief  32-bit accumulator saturation to 16-bit fractional value function
*
* @param  ptr			
* 
* @param  in    		acc32_t a32Accum
*                         - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the lower 16 bits of the input 32-bit
* 			accumulator. The function saturates the input if
* 			a32Accum > 0x0000 7FFF to 0x7FFF
* 			a32Accum < 0xFFFF 8000 to 0x8000
*
*			SATURATION mode independent!
*
****************************************************************************/
extern inline frac16_t MLIB_Sat_F16a_FAsmi(register acc32_t a32Accum)
{
	register acc32_t a32Value1;
	register int16_t i16Clb;
	register frac16_t f16Out;
	
	asm(.optimize_iasm on);

	asm(tfr a32Accum,a32Value1);	/* Makes a copy */

	asm(clb a32Accum,i16Clb);    	/* Acquire number of leading ones/zeros minus one of a32Value1 */ 

	asm(asl16 a32Accum);			/* Shifts the input 16x to the left */

	asm(bfchg #0x8000,a32Value1.1);	/* Changes the MSB of f32Value1 */

	asm(cmp.w #16,i16Clb);       	/* i16Clb - 16 -> comparing the number of leading ones of f32Value1 to 16 */

	asm(tlt	a32Value1,a32Accum);	/* if no space to shift, uses the limit value */
		
	asm(move.w a32Accum,f16Out);	/* Transfer upper 16 bits with saturation */
		
	asm(.optimize_iasm off);
		
	return f16Out;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_SAT_A32_ASM_H_ */
