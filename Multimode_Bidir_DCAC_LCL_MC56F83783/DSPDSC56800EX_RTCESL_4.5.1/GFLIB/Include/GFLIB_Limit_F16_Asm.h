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
* @brief  Limit functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _GFLIB_LIMIT_F16_ASM_H_
#define _GFLIB_LIMIT_F16_ASM_H_

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
#define GFLIB_Limit_F16_Asmi(f16Val, f16LLim, f16ULim) GFLIB_Limit_F16_FAsmi(f16Val, f16LLim, f16ULim)
#define GFLIB_UpperLimit_F16_Asmi(f16Val, f16ULim) GFLIB_UpperLimit_F16_FAsmi(f16Val, f16ULim)
#define GFLIB_LowerLimit_F16_Asmi(f16Val, f16LLim) GFLIB_LowerLimit_F16_FAsmi(f16Val, f16LLim)

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
* @brief  Limit function
*
* @param  ptr			

* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*                       frac16_t f16LLim  
*                         - LowerLimit in [-1;1] in frac16_t
*                       frac16_t f16ULim
*						  - UpperLimit in [-1;1] in frac16_t
*						  
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function trims the argument according to the upper f16ULim and 
* 			lower f16LLim limits. The upper limit must >= lower limit.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t GFLIB_Limit_F16_FAsmi(register frac16_t f16Val, register frac16_t f16LLim, register frac16_t f16ULim)
{
	register frac32_t f32Val;
	
	asm(.optimize_iasm on);
	
	asm(move.w	f16Val,f32Val);				/* Copies argument to f32Val */
	asm(cmp.w	f16ULim,f32Val);			/* Compares f32Val to upper limit */
	asm(tgt		f16ULim,f32Val);			/* Upper limit to f32Val if f32Val > upper limit */
	asm(cmp.w	f16LLim,f32Val);			/* Compares new f32Val to lower limit */
	asm(tlt		f16LLim,f32Val);			/* Lower limit to f32Val if f32Val < lower limit */
	asm(move.w 	f32Val.1,f16Val);

	asm(.optimize_iasm off);
	
	return f16Val;
}

/***************************************************************************//*!
*
* @brief  Upper limit function
*
* @param  ptr	
* 		
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16ULim
*						  - Max output trim in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function trims the argument according to the upper 
*			limit in the f16ULim variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t GFLIB_UpperLimit_F16_FAsmi(register frac16_t f16Val, register frac16_t f16ULim)
{
	register frac32_t f32Val;

	asm(.optimize_iasm on);
	
	asm(move.w	f16Val,f32Val);			/* Copies argument to f32Out */
	asm(cmp.w	f16ULim,f16Val);		/* Compares f16Val to upper limit */
	asm(tgt		f16ULim,f32Val);		/* Upper limit to f32Out if f16Val > upper limit */
	asm(move.w	f32Val.1,f16Val);		/* Copies the result */
	
	asm(.optimize_iasm off);
	
	return f16Val;
}


/***************************************************************************//*!
*
* @brief  Lower limit function
*
* @param  ptr	
* 		
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*						frac16_t fwLowerLimit
*						  - Min output trim in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function trims the argument according to the lower 
*			limit in the fwLowerLimit variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t GFLIB_LowerLimit_F16_FAsmi(register frac16_t f16Val, register frac16_t f16LLim)
{
	register frac32_t f32Val;

	asm(.optimize_iasm on);
	
	asm(move.w	f16Val,f32Val);		/* Copies argument to f32Val */
	asm(cmp.w	f16LLim,f16Val);	/* Compares f16Val to upper limit */
	asm(tlt		f16LLim,f32Val);	/* Lower limit to f32Val if f32Val < lower limit */
	asm(move.w	f32Val.1,f16Val);	/* Copies the result */
	
	asm(.optimize_iasm off);
	
	return f16Val;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_LIMIT_F16_ASM_H_ */
