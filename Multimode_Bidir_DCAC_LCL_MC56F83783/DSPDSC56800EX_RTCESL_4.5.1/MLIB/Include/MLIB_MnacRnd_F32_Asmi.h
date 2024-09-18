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
* @brief  Accumulator subtraction from product functions with 32-bit rounded fractional output 
* 		  in assembler for V3
* 
*******************************************************************************/
#ifndef _MLIB_MNACRND_F32_ASMI_H_
#define _MLIB_MNACRND_F32_ASMI_H_

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* Includes
*******************************************************************************/
#include "mlib_types.h"


/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 32x16-bit multipliers input, 32-output
* 		  accumulator subtraction from product function with result rounding
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Accum
*                         - Argument in [-1;1] in frac32_t, accumulator
*                       frac32_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x16) and then the
* 			accumulator is subtracted from their product and rounds the result
* 			to the upper 32 bits:
* 			result = f32Mult1 * f16Mult2 - f32Accum.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*		    The V3 core instructions used!	
*		    
****************************************************************************/
extern inline frac32_t MLIB_MnacRnd_F32lls_FAsmi(register frac32_t f32Accum, register frac32_t f32Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32In2;
		
	asm(.optimize_iasm on);
		
	asm(move.w f16Mult2,f32In2);
	
	asm(neg f32Accum);						/* f32Accum = -f32Accum */
		
	asm(mac32 f32Mult1,f32In2,f32Accum);	/* round(f32Accum + f32Mult1 * f16Mult2) */

	asm(nop);
		
	asm(.optimize_iasm off);
		
	return f32Accum;
}

/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 32x16-bit multipliers input, 32-output
* 		  accumulator subtraction from product saturated function with result rounding
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Accum
*                         - Argument in [-1;1] in frac32_t, accumulator
*                       frac16_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x16) and then the
* 			accumulator is subtracted from their product and rounds the result:
* 			result = f32Mult1 * f16Mult2 - f32Accum.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*		    The V3 core instructions used!			
*
****************************************************************************/
extern inline frac32_t MLIB_MnacRndSat_F32lls_FAsmi(register frac32_t f32Accum, register frac32_t f32Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32In2;
		
	asm(.optimize_iasm on);
		
	asm(move.w f16Mult2,f32In2);
	
	asm(neg f32Accum);						/* f32Accum = -f32Accum */	
		
	asm(mac32 f32Mult1,f32In2,f32Accum);	/* round(f32Accum + f32Mult1 * f16Mult2) */
		
	asm(nop);

	asm(sat f32Accum);						/* Saturation */
		
	asm(.optimize_iasm off);
		
	return f32Accum;
}

/***************************************************************************//*!
*
* @brief  32-bit inputs 32-output accumulator subtraction from product function with result rounding
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Accum
*                         - Argument in [-1;1] in frac32_t, accumulator
*                       frac32_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Mult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs and then the
* 			accumulator is subtracted from their product and rounds the result
* 			to the upper 32 bits:
* 			result = f32Mult1 * f32Mult2 - f32Accum.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*		    The V3 core instructions used!				
*
****************************************************************************/
extern inline frac32_t MLIB_MnacRnd_F32_FAsmi(register frac32_t f32Accum, register frac32_t f32Mult1, register frac32_t f32Mult2)
{
	asm(.optimize_iasm on);

	asm(neg f32Accum);						/* f32Accum = -f32Accum */	
	
	asm(mac32 f32Mult1,f32Mult2,f32Accum);	/* round(f32Accum + f32Mult1 * f32Mult2) */
	
	asm(nop);
	
	asm(.optimize_iasm off);
	
	return f32Accum;
}

/***************************************************************************//*!
*
* @brief  32-bit inputs 32-output accumulator subtraction from product saturated function
* 		  with result rounding
*
* @param  ptr			
* 
* @param  in    		frac16_t f32Accum
*                         - Argument in [-1;1] in frac32_t, accumulator
*                       frac16_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac16_t f32Mult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs and then the
* 			accumulator is subtracted from their product and rounds the result
* 			to the upper 32 bits:
* 			result = f32Mult1 * f32Mult2 - f32Accum.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*		    The V3 core instructions used!
*
****************************************************************************/
extern inline frac32_t MLIB_MnacRndSat_F32_FAsmi(register frac32_t f32Accum, register frac32_t f32Mult1, register frac32_t f32Mult2)
{
	asm(.optimize_iasm on);

	asm(neg f32Accum);						/* f32Accum = -f32Accum */	
	
	asm(mac32 f32Mult1,f32Mult2,f32Accum);	/* round(f32Accum + f32Mult1 * f32Mult2) */
		
	asm(nop);
		
	asm(sat f32Accum);						/* Saturation*/
		
	asm(.optimize_iasm off);
		
	return f32Accum;
}

#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_MNACRND_F32_ASM_H_ */
