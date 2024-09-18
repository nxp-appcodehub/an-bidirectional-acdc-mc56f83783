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
* @brief  Bidirectional shift functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_BISHIFT_F16_ASM_H_
#define _MLIB_BISHIFT_F16_ASM_H_

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
#define MLIB_ShLBi_F16_Asmi(f16Val, i16Sh) MLIB_ShLBi_F16_FAsmi(f16Val, i16Sh)
#define MLIB_ShLBiSat_F16_Asmi(f16Val, i16Sh) MLIB_ShLBiSat_F16_FAsmi(f16Val, i16Sh)
#define MLIB_ShRBi_F16_Asmi(f16Val, i16Sh) MLIB_ShRBi_F16_FAsmi(f16Val, i16Sh)
#define MLIB_ShRBiSat_F16_Asmi(f16Val, i16Sh) MLIB_ShRBiSat_F16_FAsmi(f16Val, i16Sh)

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
* @brief  16-bit arithmetic multi-bit shift left function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*                       int16_t i16Sh
*                         - Number of shifts to the left; negative: shifts to the right  
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the f16Val input shifted by the number of i16Sh to the left. 
* 			If the i16Sh is negative, the input is shifted to the right. The function
* 			does not saturate the output. 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_ShLBi_F16_FAsmi(register frac16_t f16Val, register int16_t i16Sh)
{
	register frac32_t f32Value;
	register frac16_t f16Out;
		
	asm(.optimize_iasm on);

	asm(move.w f16Val,f32Value);	/* Converts it to 32-bit, because 16-bit shift is uni-directional */

	asm(asll.l i16Sh,f32Value);
		
	asm(nop);
		
	asm(move.w f32Value.1,f16Out);	
		
	asm(.optimize_iasm off);
		
	return f16Out;		
}

/***************************************************************************//*!
*
* @brief  16-bit arithmetic multi-bit shift left function with saturation
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*                       int16_t i16Sh
*                         - Number of shifts to the left; negative: shifts to the right  
*                         
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks This function returns the f16Val input shifted by the number of i16Sh to the left. 
* 			If the i16Sh is negative, the input is shifted to the right. The function
* 			saturates if necessary.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_ShLBiSat_F16_FAsmi(register frac16_t f16Val, register int16_t i16Sh)
{
	register frac32_t	f32Value0, f32Value1;
	register int16_t i16Clb;
	register frac16_t f16Out;
	
	asm(.optimize_iasm on);

	asm(move.w	f16Val,f32Value0);	/* Converts it to 32-bit, because 16-bit shift is uni-directional */
	
	asm(tfr f32Value0,f32Value1);	/* Makes a copy */

	asm(clb f32Value0,i16Clb);    	/* Acquire number of leading ones/zeros minus one of f32Value0 */ 

	asm(asll.l i16Sh,f32Value0);	/* Shifts the input to the left */

	asm(bfchg #0x8000,f32Value1.1);	/* Changes the MSB of f32Value1 */

	asm(cmp i16Sh,i16Clb);         	/* i16Clb - i16Sh -> comparing the number of leading ones of f32Value and i16Sh  */
		
	asm(tlt	f32Value1,f32Value0);	/* If no space to shift, uses the limit value */

	asm(sat f32Value0,f16Out);		/* Saturates the result */
		
	asm(.optimize_iasm off);
		
	return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit arithmetic multi-bit shift right function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*                       int16_t i16Sh
*                         - Number of shifts to the left; negative: shifts to the left  
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the f16Val input shifted by the number of i16Sh to the right. 
* 			If the i16Sh is negative, the input is shifted to the left. The function
* 			does not saturate the output. 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_ShRBi_F16_FAsmi(register frac16_t f16Val, register int16_t i16Sh)
{
	register frac32_t	f32Value;
	register frac16_t f16Out;
		
	asm(.optimize_iasm on);
		
	asm(move.w f16Val,f32Value);	/* Converts it to 32-bit, because 16-bit shift is uni-directional */

	asm(asrr.l i16Sh,f32Value);

	asm(nop);		
		
	asm(move.w f32Value.1,f16Out);
		
	asm(.optimize_iasm off);
		
	return f16Out;		
}

/***************************************************************************//*!
*
* @brief  16-bit arithmetic multi-bit shift right function with saturation
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*                       int16_t i16Sh
*                         - Number of shifts to the right; negative: shifts to the left  
*                         
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks This function returns the f16Val input shifted by the number of i16Sh to the right. 
* 			If the i16Sh is negative, the input is shifted to the right. The function
* 			saturates if necessary.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_ShRBiSat_F16_FAsmi(register frac16_t f16Val, register int16_t i16Sh)
{
	register frac32_t	f32Value0, f32Value1;
	register int16_t i16Clb;
	register frac16_t f16Out;
	
	asm(.optimize_iasm on);

	asm(move.w	f16Val,f32Value0);	/* Converts it to 32-bit, because 16-bit shift is uni-directional */
	
	asm(tfr f32Value0,f32Value1);	/* Makes a copy */

	asm(clb f32Value0,i16Clb);    	/* Acquire number of leading ones/zeros minus one of f32Value0 */ 

	asm(asrr.l i16Sh,f32Value0);	/* Shifts the input to the right */

	asm(bfchg #0x8000,f32Value1.1);	/* Changes the MSB of f32Value */

	asm(add i16Sh,i16Clb);         	/* i16Clb + i16Sh -> comparing the number of leading ones of f32Value and i16Sh  */
		
	asm(tlt	f32Value1,f32Value0);	/* If no space to shift, uses the limit value */

	asm(sat f32Value0,f16Out);		/* Saturates the result */
		
	asm(.optimize_iasm off);
		
	return f16Out;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_BISHIFT_F16_ASM_H_ */
