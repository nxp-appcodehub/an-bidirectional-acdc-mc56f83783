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
* @brief  Addition functions with 64-bit accumulator output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_ADD_A32_ASM_H_
#define _MLIB_ADD_A32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"

/*******************************************************************************
* Constants
*******************************************************************************/

/*******************************************************************************
* Macros 
*******************************************************************************/
#define MLIB_Add_A32ss_Asmi(f16Add1, f16Add2) MLIB_Add_A32ss_FAsmi(f16Add1, f16Add2)
#define MLIB_Add_A32as_Asmi(a32Accum, f16Add) MLIB_Add_A32as_FAsmi(a32Accum, f16Add)

/******************************************************************************
* Types
*******************************************************************************/

/*******************************************************************************
* Global variables
*******************************************************************************/

/*******************************************************************************
* Global functions
*******************************************************************************/

/*******************************************************************************
* Inline functions
*******************************************************************************/

/***************************************************************************//*!
*
* @brief  Addition of two 16-bit fractional arguments resulting a 32-bit accumulator
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Add1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Add2
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - acc32_t value [-65536.0;-65536.0 - (2^-15)]
*		
* @remarks 	This function returns the addition of two inputs. The result is stored
* 			in the lower word of the result with sing extension to the 32 bits.
* 			This function does not saturate the the output
*
*			SATURATION INDEPENDENT!
*
*******************************************************************************/
extern inline acc32_t MLIB_Add_A32ss_FAsmi(register frac16_t f16Add1, register frac16_t f16Add2)
{
	register acc32_t a32Acc, a32Temp;
	
	asm(.optimize_iasm on);
		
	asm(asr16 f16Add1,a32Acc);				/* a32Acc = f16Add1 >> 16 */
		
	asm(asr16 f16Add2,a32Temp);				/* a32Temp = f16Add2 >> 16 */
		
	asm(add a32Temp,a32Acc);				/* a32Acc = a32Acc + f16Add2 */
		
	asm(.optimize_iasm off);
		
	return a32Acc;
}

/***************************************************************************//*!
*
* @brief  Addition of a 16-bit fractional value to a 32-bit accumulator resulting
* 		  a 32-bit accumulator
*
* @param  ptr			
* 
* @param  in    		acc32_t a32Acc
*                         - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*						frac16_t f16Add
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - acc32_t value [-65536.0;65536.0 - (2^-15)]
*		
* @remarks 	This function returns the addition of a 32-bit accumulator and a
* 			16-bit fractional input. The f16Add value is added to the lower
* 			word of a32Acc.
* 			This function does not saturate the the output
*
*			SATURATION INDEPENDENT!
*
*******************************************************************************/
extern inline acc32_t MLIB_Add_A32as_FAsmi(register acc32_t a32Accum, register frac16_t f16Add)
{
	register acc32_t a32Temp;
	
	asm(.optimize_iasm on);
		
	asm(asr16 f16Add,a32Temp);				/* a32Temp = f16Add >> 16 */
		
	asm(add a32Temp,a32Accum);				/* a32Accum = f3a32Accum + f16Add */
		
	asm(.optimize_iasm off);
		
	return a32Accum;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_ADD_A32_ASM_H_ */
