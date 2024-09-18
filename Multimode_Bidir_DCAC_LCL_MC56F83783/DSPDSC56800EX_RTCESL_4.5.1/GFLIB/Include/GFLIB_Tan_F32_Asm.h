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
* @brief  Tangent functions using piece-wise polynomial approximation 
* 		  in assembler
* 
*******************************************************************************/
#ifndef _GFLIB_TAN_F32_ASM_H_
#define _GFLIB_TAN_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"
#include "gflib_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/

/******************************************************************************
* Types
******************************************************************************/
typedef struct{
	frac32_t  f32A[4];
} GFLIB_TAN_COEF_T_F32;

typedef struct{
	GFLIB_TAN_COEF_T_F32  sTanSector[8];
} GFLIB_TAN_T_F32;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief    The MCLIB_Tan function computes the tan(pi*x) using    
*		    piece-wise polynomial approximation.
*
* @param  ptr   		*psParam
*                           Pointer to the table
*
* @param  in    		f16Angle
*                           The input data value is in the range of [-1,1), which
*                           corresponds to the angle in the range of [-pi,pi).   
*                       
*
* @return The function returns tan(pi*x).
*		
* @remarks 	The MCLIB_Tan function computes the tan(pi*x) using     
*		  	piece-wise polynomial approximation. All tangent values
*			falling beyond [-1, 1), are truncated to -1 and 1      
*			respectively. Calculates tan of the input fractional
*			argument                                               
*
*			The approximation of the tangent function over the entire defined
*			range of input angles is simplified to an approximation for angles in 
*			the range [0, pi/4), and then, depending on the input angle, the result 
*			will be negated. Interval [0, pi/4) is further divided into eight equally 
*			spaced sub intervals, and polynomial approximation is done for each 
*			interval respectively. 
*
*           Tan(x) = ((A1*x + A2)*x + A3)*x + A4
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm frac16_t GFLIB_Tan_F16_FAsm(frac16_t f16Angle, const GFLIB_TAN_T_F32 *psParam);

/******************************************************************************
* Inline functions
******************************************************************************/
#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_TAN_F32_ASM_H_ */
