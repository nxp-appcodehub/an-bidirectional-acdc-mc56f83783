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
* @brief  Sum of 4 values functions with 32-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_ADD4_F32_ASM_H_
#define _MLIB_ADD4_F32_ASM_H_

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
#define MLIB_Add4_F32_Asmi(f32Add1, f32Add2, f32Add3, f32Add4) MLIB_Add4_F32_FAsmi(f32Add1, f32Add2, f32Add3, f32Add4)
#define MLIB_Add4Sat_F32_Asmi(f32Add1, f32Add2, f32Add3, f32Add4) MLIB_Add4Sat_F32_FAsmi(f32Add1, f32Add2, f32Add3, f32Add4)

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
* @brief  32-bit addition function of four arguments
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Add1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add2
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32Add3
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add4
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the addition of four inputs. The function
* 			does not saturate the output if the saturation mode is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac32_t MLIB_Add4_F32_FAsmi(register frac32_t f32Add1, register frac32_t f32Add2, register frac32_t f32Add3, register frac32_t f32Add4)
{
	asm(.optimize_iasm on);
	
	asm(add f32Add2,f32Add1);	/* f32Add1 + f32Add1 */
	
	asm(add f32Add3,f32Add1);	/* f32Add1 + f32Add2 + f32Add3 */
	
	asm(add f32Add4,f32Add1);	/* f32Add1 + f32Add2 + f32Add3 + f32Add4 */
	
	asm(.optimize_iasm off);
	
	return f32Add1;
}

/***************************************************************************//*!
*
* @brief  32-bit addition saturated function of four arguments
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Add1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add2
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32Add3
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add4
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the addition of four inputs. The function
* 			saturates the output.
*
*			SATURATION must be turned off!
*
****************************************************************************/
extern inline frac32_t MLIB_Add4Sat_F32_FAsmi(register frac32_t f32Add1, register frac32_t f32Add2, register frac32_t f32Add3, register frac32_t f32Add4)
{
	asm(.optimize_iasm on);
	
	asm(add f32Add2,f32Add1);	/* f32Add1 + f32Add2 */
	
	asm(add f32Add3,f32Add1);	/* f32Add1 + f32Add2 + f32Add3 */
	
	asm(add f32Add4,f32Add1);	/* f32Add1 + f32Add2 + f32Add3 + f32Add4 */
	
	asm(sat f32Add1);			/* Saturation */
	
	asm(.optimize_iasm off);
	
	return f32Add1;	
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_ADD4_F32_ASM_H_ */
