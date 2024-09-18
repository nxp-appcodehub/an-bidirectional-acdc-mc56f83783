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
* @brief  Digital IIR Filter, 3rd order assembler implementation
* 
*******************************************************************************/
#ifndef _GDFLIB_FILTERIIR3_F32_ASM_H_
#define _GDFLIB_FILTERIIR3_F32_ASM_H_

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
#define GDFLIB_FilterIIR3Init_F16_Asmi(psParam) GDFLIB_FilterIIR3Init_F16_FAsmi(psParam)
#define GDFLIB_FilterIIR3_F16_Asm(f16InX, psParam) GDFLIB_FilterIIR3_F16_FAsm(f16InX, psParam) 

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	frac32_t			f32B0;
	frac32_t			f32B1;
	frac32_t			f32B2;
	frac32_t			f32B3;
	frac32_t			f32A1;
	frac32_t			f32A2;
	frac32_t			f32A3;
} GDFLIB_FILTER_IIR3_COEFF_T_F32;

typedef struct 
{
    GDFLIB_FILTER_IIR3_COEFF_T_F32	sFltCoeff;
    frac32_t 						f32FltBfrY[3];
    frac16_t 						f16FltBfrX[3];
} GDFLIB_FILTER_IIR3_T_F32;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
extern asm frac16_t GDFLIB_FilterIIR3_F16_FAsm(frac16_t f16In, GDFLIB_FILTER_IIR3_T_F32 *psParam);

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function clears the buffer of 4th order IIR filter
*
* @param  ptr			GDFLIB_FILTER_IIR3_T *psParam
* 						- pointer to the structure of the filter
*
*                         GDFLIB_FILTER_IIR3_T_F32 data structure:
*                           GDFLIB_FILTER_IIR3_COEFF_T_F32	sFltCoeff   
*                               - filter coefficients in frac32_t divided by 4
*                                 in the following range
*                                 -1.0 <= f16Gain < 1.0
*                               - B coefficients:
*                               	frac32_t			f32B0
*									frac32_t			f32B1
*									frac32_t			f32B2
*									frac32_t			f32B3
*								- -A coefficient, the sign is negated
*									frac32_t			f32A1;
*									frac32_t			f32A2;
*									frac32_t			f32A3;
*                           frac32_t f32FltBfrY[3];
*                               - y history buffer in frac32_t
*							frac16_t f16FltBfrX[3];
*								- x history buffer in frac16_t
*
* @param  in    		
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern inline void GDFLIB_FilterIIR3Init_F16_FAsmi(register GDFLIB_FILTER_IIR3_T_F32 *psParam)
{
	register frac32_t f32Temp = 0;
	register frac32_t *pf32Temp = psParam->f32FltBfrY;
	
	asm(move.l f32Temp,X:(pf32Temp)+);
	asm(move.l f32Temp,X:(pf32Temp)+);
	asm(move.l f32Temp,X:(pf32Temp)+);
	asm(move.w f32Temp,X:(pf32Temp)+);
	asm(move.w f32Temp,X:(pf32Temp)+);
	asm(move.w f32Temp,X:(pf32Temp));
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GDFLIB_FILTERIIR3_F32_ASM_H_ */
