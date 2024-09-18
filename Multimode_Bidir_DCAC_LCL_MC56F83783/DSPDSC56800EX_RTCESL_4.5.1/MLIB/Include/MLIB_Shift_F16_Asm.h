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
* @brief  Shift functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_SHIFT_F16_ASM_H_
#define _MLIB_SHIFT_F16_ASM_H_

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
#define MLIB_Sh1L_F16_Asmi(f16Val) MLIB_Sh1L_F16_FAsmi(f16Val)
#define MLIB_Sh1LSat_F16_Asmi(f16Val) MLIB_Sh1LSat_F16_FAsmi(f16Val)
#define MLIB_Sh1R_F16_Asmi(f16Val) MLIB_Sh1R_F16_FAsmi(f16Val)

#define MLIB_ShL_F16_Asmi(f16Val, u16Sh) MLIB_ShL_F16_FAsmi(f16Val, u16Sh)
#define MLIB_ShLSat_F16_Asmi(f16Val, u16Sh) MLIB_ShLSat_F16_FAsmi(f16Val, u16Sh)
#define MLIB_ShR_F16_Asmi(f16Val, u16Sh) MLIB_ShR_F16_FAsmi(f16Val, u16Sh)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
extern frac64_t MLIB_Sh1L_F64_FAsm(frac64_t f64In);
extern frac64_t MLIB_Sh1LSat_F64_FAsm(frac64_t f64In);
extern frac64_t MLIB_Sh1R_F64_FAsm(frac64_t f64In);

extern frac64_t MLIB_ShL_F64_FAsm(frac64_t f64In, uint16_t u16Sh);
extern frac64_t MLIB_ShLSat_F64_FAsm(frac64_t f64In, uint16_t u16Sh);
extern frac64_t MLIB_ShR_F64_FAsm(frac64_t f64In, uint16_t u16Sh);

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  16-bit one arithmetic shift left function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the input shifted one bit to the left. The function
* 			does not saturate the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_Sh1L_F16_FAsmi(register frac16_t f16Val)
{
	asm(.optimize_iasm on);
		
	asm(asl.w f16Val);
		
	asm(.optimize_iasm off);
		
	return f16Val;
}

/***************************************************************************//*!
*
* @brief  16-bit one arithmetic shift left function with saturation
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the input shifted one bit to the left and
* 			saturates if necessary.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_Sh1LSat_F16_FAsmi(register frac16_t f16Val)
{
	register frac32_t f32Value;
	register frac16_t f16Out;
	
	asm(.optimize_iasm on);

	asm(move.w	f16Val,f32Value);	
		
	asm(asl f32Value);
	
	asm(sat f32Value,f16Out);
		
	asm(.optimize_iasm off);
		
	return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit one arithmetic shift right function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the input shifted one bit to the right. The function
* 			does not saturate the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_Sh1R_F16_FAsmi(register frac16_t f16Val)
{
	register frac32_t f32Value;
	register frac16_t f16Out;
		
	asm(.optimize_iasm on);
		
	asm(move.w	f16Val,f32Value);	
		
	asm(asr f32Value);
		
	asm(move.w f32Value.1,f16Out);		
		
	asm(.optimize_iasm off);
		
	return f16Out;		
}

/***************************************************************************//*!
*
* @brief  16-bit arithmetic multi-bit shift left function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*                       uint16_t u16Sh
*                         - Number of shifts to the left; negative: shifts to the right  
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the f16Val input shifted by the number of u16Sh to the left. 
* 			The function does not saturate the output. 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_ShL_F16_FAsmi(register frac16_t f16Val, register uint16_t u16Sh)
{
	asm(.optimize_iasm on);

	asm(asll.w u16Sh,f16Val);
		
	asm(.optimize_iasm off);
		
	return f16Val;		
}

/***************************************************************************//*!
*
* @brief  16-bit arithmetic multi-bit shift left function with saturation
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*                       uint16_t u16Sh
*                         - Number of shifts to the left; negative: shifts to the right  
*                         
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the f16Val input shifted by the number of u16Sh to the left. 
* 			The function saturates if necessary.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_ShLSat_F16_FAsmi(register frac16_t f16Val, register uint16_t u16Sh)
{
	register frac32_t f32Value0, f32Value1;
	register int16_t i16Clb;
	register frac16_t f16Out;
	
	asm(.optimize_iasm on);

	asm(move.w	f16Val,f32Value0);	/* Converts it to 32-bit, because 16-bit shift is uni-directional */
		
	asm(tfr f32Value0,f32Value1);	/* Makes a copy */

	asm(clb f32Value0,i16Clb);    	/* Acquire number of leading ones/zeros minus one of f32Value0 */ 

	asm(asll.l u16Sh,f32Value0);	/* Shifts the input to the left */

	asm(bfchg #0x8000,f32Value1.1);	/* Changes the MSB of f32Value1 */

	asm(cmp u16Sh,i16Clb);         	/* i16Clb - u16Sh -> comparing the number of leading ones of f32Value and u16Sh  */
	
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
*                       uint16_t u16Sh
*                         - Number of shifts to the left; negative: shifts to the left  
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the f16Val input shifted by the number of u16Sh to the right. 
* 			The function does not saturate the output. 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_ShR_F16_FAsmi(register frac16_t f16Val, register uint16_t u16Sh)
{
	asm(.optimize_iasm on);
		
	asm(asrr.w u16Sh,f16Val);

	asm(nop);		
		
	asm(.optimize_iasm off);
		
	return f16Val;		
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_SHIFT_F16_ASM_H_ */
