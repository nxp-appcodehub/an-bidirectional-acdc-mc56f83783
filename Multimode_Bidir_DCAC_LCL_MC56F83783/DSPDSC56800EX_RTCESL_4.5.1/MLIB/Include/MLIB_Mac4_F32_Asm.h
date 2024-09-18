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
* @brief  Sum of two products functions with 32-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_MAC4_F32_ASM_H_
#define _MLIB_MAC4_F32_ASM_H_

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
#define MLIB_Mac4_F32ssss_Asmi(f16Add1Mult1, f16Add1Mult2, f16Add2Mult1, f16Add2Mult2) \
	MLIB_Mac4_F32ssss_FAsmi(f16Add1Mult1, f16Add1Mult2, f16Add2Mult1, f16Add2Mult2)
#define MLIB_Mac4Sat_F32ssss_Asmi(f16Add1Mult1, f16Add1Mult2, f16Add2Mult1, f16Add2Mult2) \
	MLIB_Mac4Sat_F32ssss_FAsmi(f16Add1Mult1, f16Add1Mult2, f16Add2Mult1, f16Add2Mult2)

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
* @brief  16-bit inputs 32-output vector multiply accumulate function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Add1Mult1
*                         - Argument in [-1;1] in frac16_t
*                       frac16_t f16Add1Mult2
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Add2Mult1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Add2Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the vector multiply accumulate of input values. 
* 			The output saturation is not implemented in this function. 
* 			
* 			The output of the function is defined by the following equation:
* 			f16Add1Mult1 * f16Add1Mult2 + f16Add2Mult1 * f16Add2Mult2
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac32_t MLIB_Mac4_F32ssss_FAsmi(register frac16_t f16Add1Mult1, register frac16_t f16Add1Mult2, register frac16_t f16Add2Mult1, register frac16_t f16Add2Mult2)
{
		register frac32_t f32Acc;
		
		asm(.optimize_iasm on);
		
		asm(mpy f16Add1Mult1,f16Add1Mult2,f32Acc);
				
		asm(mac f16Add2Mult1,f16Add2Mult2,f32Acc);
		
		asm(nop);
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}

/***************************************************************************//*!
*
* @brief  16-bit inputs 32-output vector multiply accumulate saturated function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Add1Mult1
*                         - Argument in [-1;1] in frac16_t
*                       frac16_t f16Add1Mult2
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Add2Mult1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Add2Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the vector multiply accumulate of input values. 
* 			The function saturates the output. 
* 			
* 			The output of the function is defined by the following equation:
* 			f16Add1Mult1 * f16Add1Mult2 + f16Add2Mult1 * f16Add2Mult2
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_Mac4Sat_F32ssss_FAsmi(register frac16_t f16Add1Mult1, register frac16_t f16Add1Mult2, register frac16_t f16Add2Mult1, register frac16_t f16Add2Mult2)
{
		register frac32_t f32Acc;
		
		asm(.optimize_iasm on);
		
		asm(mpy f16Add1Mult1,f16Add1Mult2,f32Acc);
		
		asm(mac f16Add2Mult1,f16Add2Mult2,f32Acc);
		
		asm(nop);
		
		asm(sat f32Acc);
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MAC4_F32_ASM_H_ */
