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
* @brief  Binary logarithm function in assembler
* 
*******************************************************************************/
#ifndef MLIB_LOG2ASM_H_
#define MLIB_LOG2ASM_H_

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
#define MLIB_Log2_U16_Asmi(u16Val) MLIB_Log2_U16_FAsmi(u16Val)

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
* @brief  Binary logarithm of 16-bit input
*
* @param  ptr			
* 
* @param  in    		uint16_t u16Val
*                         - Argument in uint16_t
*
* @return This function returns
*     - uint16_t value
*		
* @remarks 	This function returns the 16-bit integer part of binary logarithm of the input. 
* 			Returns 0 for input u16Val == 0.  
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline uint16_t MLIB_Log2_U16_FAsmi(register uint16_t u16Val)
{
	register uint32_t u32Temp;
	register uint16_t u16Shifts = 14;  
	
	asm(.optimize_iasm on);

	asm(move.w #-1,Y1);		
		
	asm(clb u16Val,u32Temp);	/* Number of leading bits of ui16In => u32Temp */
				
	asm(tst.w u16Val);			/* Compares ui16In to 0 */

	asm(tle	u16Shifts,u32Temp);	/* If (u16Val <= 0), then u16Shifts => u32Temp */
		
	asm(sub u32Temp,u16Shifts);	/* u16Shifts = u16Shifts - u32Temp */
				
	asm(move.w	u16Shifts, u16Val);			
		
	asm(.optimize_iasm off);
		
	return u16Val;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* MLIB_LOG2ASM_H_ */
