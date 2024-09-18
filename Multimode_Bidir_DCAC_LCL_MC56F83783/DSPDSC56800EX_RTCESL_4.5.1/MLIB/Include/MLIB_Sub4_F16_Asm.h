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
* @brief  Subtraction of 4 values functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_SUB4_F16_ASM_H_
#define _MLIB_SUB4_F16_ASM_H_

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
#define MLIB_Sub4_F16_Asmi(f16Min, f16Sub1, f16Sub2, f16Sub3) MLIB_Sub4_F16_FAsmi(f16Min, f16Sub1, f16Sub2, f16Sub3)
#define MLIB_Sub4Sat_F16_Asmi(f16Min, f16Sub1, f16Sub2, f16Sub3) MLIB_Sub4Sat_F16_FAsmi(f16Min, f16Sub1, f16Sub2, f16Sub3)

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
* @brief  16-bit subtraction function of four arguments
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Min
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Sub1
*                         - Argument in [-1;1] in frac16_t
*                       frac16_t f16Sub2
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Sub3
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the subtraction of four inputs. 
* 			The output of the function is defined by the following equation:
* 						f16Min - f16Sub1 - f16Sub2 - f16Sub3
* 			The function does not saturate the output if the saturation mode is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac16_t MLIB_Sub4_F16_FAsmi(register frac16_t f16Min, register frac16_t f16Sub1, register frac16_t f16Sub2, register frac16_t f16Sub3)
{
	asm(.optimize_iasm on);
	
	asm(sub f16Sub1,f16Min);
	
	asm(sub f16Sub2,f16Min);
	
	asm(sub f16Sub3,f16Min);
	
	asm(.optimize_iasm off);
	
	return f16Min;
}

/***************************************************************************//*!
*
* @brief  16-bit subtraction saturated function of four arguments
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Min
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Sub1
*                         - Argument in [-1;1] in frac16_t
*                       frac16_t f16Sub2
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Sub3
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the subtraction of four inputs. 
*  			The output of the function is defined by the following equation:
* 					f16Min - f16Sub1 - f16Sub2 - f16Sub3
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_Sub4Sat_F16_FAsmi(register frac16_t f16Min, register frac16_t f16Sub1, register frac16_t f16Sub2, register frac16_t f16Sub3)
{
	register frac32_t f32Val;
	
	asm(.optimize_iasm on);
	
	asm(move.w	f16Min,f32Val);
	
	asm(sub f16Sub1,f32Val);
	
	asm(sub f16Sub2,f32Val);
	
	asm(sub f16Sub3,f32Val);
	
	asm(sat f32Val, f16Min);
	
	asm(.optimize_iasm off);
	
	return f16Min;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_SUB4_F16_ASM_H_ */
