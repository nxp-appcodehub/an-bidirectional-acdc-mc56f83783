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
* @brief  Subtraction of two products functions with 16-bit rounded fractional 
* 		  output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_MSU4RND_F16_ASM_H_
#define _MLIB_MSU4RND_F16_ASM_H_

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
#define MLIB_Msu4Rnd_F16_Asmi(f16MinMult1, f16MinMult2, f16SubMult1, f16SubMult2) \
	MLIB_Msu4Rnd_F16_FAsmi(f16MinMult1, f16MinMult2, f16SubMult1, f16SubMult2)
#define MLIB_Msu4RndSat_F16_Asmi(f16MinMult1, f16MinMult2, f16SubMult1, f16SubMult2) \
	MLIB_Msu4RndSat_F16_FAsmi(f16MinMult1, f16MinMult2, f16SubMult1, f16SubMult2)

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
* @brief  	16-bit inputs 16-output vector multiply and subtract function
*			with result rounding
* @param  ptr			
* 
* @param  in    		frac16_t f16MinMult1
*                         - Argument in [-1;1] in frac16_t
*                       frac16_t f16MinMult2
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16SubMult1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16SubMult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the vector multiply and subtract of input values. 
* 			The input values as well as output value is considered as 16-bit fractional values. 
* 			The output saturation is not implemented in this function. 
* 			The output of the function is defined by the following equation:
* 			f16MinMult1 * f16MinMult2 - f16SubMult1 * f16SubMult2
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac16_t MLIB_Msu4Rnd_F16_FAsmi(register frac16_t f16MinMult1, register frac16_t f16MinMult2, register frac16_t f16SubMult1, register frac16_t f16SubMult2)
{
	register frac32_t f32Acc;
		
	asm(.optimize_iasm on);
		
	asm(mpy f16MinMult1,f16MinMult2,f32Acc);	/* f16MinMult1 * f16MinMult2 */
				
	asm(macr -f16SubMult1,f16SubMult2,f32Acc);	/* f16MinMult1 * f16MinMult2 - f16SubMult1 * f16SubMult2 */
		
	asm(nop);
		
	asm(move.w f32Acc.1,f16MinMult1);
		
	asm(.optimize_iasm off);
		
	return f16MinMult1;
}

/***************************************************************************//*!
*
* @brief  	16-bit inputs 16-output vector multiply and subtract saturated function
*			with result rounding
* @param  ptr			
* 
* @param  in    		frac16_t f16MinMult1
*                         - Argument in [-1;1] in frac16_t
*                       frac16_t f16MinMult2
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16SubMult1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16SubMult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the vector multiply and subtract of input values. 
* 			The input values as well as output value is considered as 16-bit fractional values. 
* 			The function saturates the output. 
* 			The output of the function is defined by the following equation:
* 			f16MinMult1 * f16MinMult2 - f16SubMult1 * f16SubMult2
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_Msu4RndSat_F16_FAsmi(register frac16_t f16MinMult1, register frac16_t f16MinMult2, register frac16_t f16SubMult1, register frac16_t f16SubMult2)
{
	register frac32_t f32Acc;
		
	asm(.optimize_iasm on);
		
	asm(mpy f16MinMult1,f16MinMult2,f32Acc);	/* f16MinMult1 * f16MinMult2 */
		
	asm(macr -f16SubMult1,f16SubMult2,f32Acc);	/* f16MinMult1 * f16MinMult2 - f16SubMult1 * f16SubMult2 */
		
	asm(nop);
		
	asm(sat f32Acc,f16MinMult1);				/* Saturation */
		
	asm(.optimize_iasm off);
		
	return f16MinMult1;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MSU4RND_F16_ASM_H_ */
