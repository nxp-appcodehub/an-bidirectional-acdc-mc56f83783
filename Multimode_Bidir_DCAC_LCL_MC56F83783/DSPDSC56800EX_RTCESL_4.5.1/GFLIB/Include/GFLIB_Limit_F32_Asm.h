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
* @brief  Limit functions with 32-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _GFLIB_LIMIT_F32_ASM_H_
#define _GFLIB_LIMIT_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GFLIB_Limit_F32_Asmi(f32Val, f32LLim, f32ULim) GFLIB_Limit_F32_FAsmi(f32Val, f32LLim, f32ULim)
#define GFLIB_UpperLimit_F32_Asmi(f32Val, f32ULim) GFLIB_UpperLimit_F32_FAsmi(f32Val, f32ULim)
#define GFLIB_LowerLimit_F32_Asmi(f32Val, f32LLim) GFLIB_LowerLimit_F32_FAsmi(f32Val, f32LLim)

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
* @brief  Limit function 32-bit version
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32LLim  
*                         - Lower limit in [-1;1] in frac32_t
*                       frac32_t f32ULim
*						  - Upper limit in [-1;1] in frac32_t
*						  
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function trims the argument according to the upper f32ULim and 
* 			lower f32LLim limits. The upper limit must >= lower limit.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t GFLIB_Limit_F32_FAsmi(register frac32_t f32Val,  register frac32_t f32LLim, register frac32_t f32ULim)
{
	asm(.optimize_iasm on);
	
	asm(cmp.l	f32ULim,f32Val);				/* Compares f32Val to upper limit */
	asm(tgt		f32ULim,f32Val);				/* Upper limit to f32Val if f32Val > upper limit */
	asm(cmp.l	f32LLim,f32Val);				/* Compares f32Val to upper limit */
	asm(tlt		f32LLim,f32Val);				/* Lower limit to f32Val if f32Val < lower limit */
	
	asm(.optimize_iasm off);
	
	return f32Val;
}

/***************************************************************************//*!
*
* @brief  Upper limit function 32-bit version
*
* @param  ptr	
* 		
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32ULim
*						  - Max output trim in [-1;1] in frac16_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function trims the argument according to the upper 
*			limit in the f32ULim variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t GFLIB_UpperLimit_F32_FAsmi(register frac32_t f32Val, register frac32_t f32ULim)
{
	asm(.optimize_iasm on);
	
	asm(cmp.l 	f32ULim,f32Val);	/* Compares f32Val to upper limit */
	asm(tgt 	f32ULim,f32Val);	/* Upper limit to A if f32Val > upper limit */

	asm(.optimize_iasm off);
	
	return f32Val;
}

/***************************************************************************//*!
*
* @brief  Lower limit function 32-bit version
*
* @param  ptr		
* 	
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32LLim
*						  - Max output trim in [-1;1] in frac16_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function trims the argument according to the lower 
*			limits in the f32LLim variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t GFLIB_LowerLimit_F32_FAsmi(register frac32_t f32Val, register frac32_t f32LLim)
{
	asm(.optimize_iasm on);

	asm(cmp.l	f32LLim,f32Val);		/* Compares f32Val to upper limit */
	asm(tlt		f32LLim,f32Val);		/* Lower limit to f32Val if f32Val < lower limit */

	asm(.optimize_iasm off);
	
	return f32Val;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_LIMIT_F32_ASM_H_ */
