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
* @brief  Convert functions without scale factor and with 32-bit fractional 
* 		  input or output in assembler
* 
*******************************************************************************/
#ifndef MLIB_CONV_F32_ASM_H_
#define MLIB_CONV_F32_ASM_H_

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
#define MLIB_Conv_F16l_Asmi(f32Val) MLIB_Conv_F16l_FAsmi(f32Val)
#define MLIB_Conv_F32s_Asmi(f16Val) MLIB_Conv_F32s_FAsmi(f16Val)

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
* @brief  32-bit fractional to 16-bit fractional convert function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function convert the lower 16-bits of the 32-bit input and
* 			returns the upper 16-bit. The function does not saturate the output 
* 			if the saturation mode is turned off.
*
*			SATURATION independent!
*
****************************************************************************/
extern inline frac16_t MLIB_Conv_F16l_FAsmi(register frac32_t f32Val)
{
		register frac16_t f16Out;	
		
		asm(.optimize_iasm on);	
		
		asm(move.w f32Val.1,f16Out);		/* f16Out = f32Val.1 */
		
		asm(.optimize_iasm off);	
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit fractional to 32-bit fractional convert function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function convert the lower 16-bits of the 32-bit input and
* 			returns the upper 16-bit. The function
* 			does not saturate the output if the saturation mode is turned off.
*
*			SATURATION independent!
*
****************************************************************************/
extern inline frac32_t MLIB_Conv_F32s_FAsmi(register frac16_t f16Val)
{
		register frac32_t f32Out;
	
		asm(.optimize_iasm on);
		
		asm(move.w f16Val,f32Out);		/* f32Val = f16Val << 16 */
		
		asm(.optimize_iasm off);
		
		return f32Out;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* MLIB_CONV_F32_ASM_H_ */
