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
* @brief  Integrator algorithms implemented in assembler for V3
* 
*******************************************************************************/
#ifndef _GFLIB_INTEGRATOR_A32_ASMI_H_
#define _GFLIB_INTEGRATOR_A32_ASMI_H_

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

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
   acc32_t a32Gain;
   frac32_t f32IAccK_1;
   frac16_t f16InValK_1;
} GFLIB_INTEGRATOR_T_A32;

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
* @brief Scaled integrator with the bilinear transformation.  
*		   
* @param  ptr   		GFLIB_INTEGRATOR_T_A32 *psParam
*						  	- Pointer to integrator structure
*
*						GFLIB_INTEGRATOR_T_A32 data structure:						  
*                           acc32 a32Gain
*                               - gain; input parameter
*                                 in the following range
*                                 0 <= f16IGain < 65536.0    
*                           frac32_t f32IAccK_1
*                               - state variable; fractional part (lower long)
*                                 of integral part at step k-1;
*                                 can be modified outside of the function;
*                                 input/output parameter
*                                 in the following range
*                                 -1.0 <= f32IAccK_1 < 1.0                               
*							frac16_t f16InValK_1
*								- input value at the step k-1;
*                                 input/output parameter
*                                 in the following range
*                                 -1.0 <= f16InVal_1 < 1.0   
*						  
* @param  in    		Frac16 f16InVal
*                         - input value
*
* @return out			This function returns the integrated value.
*     - frac16_t value [-1;1]
*		
* @remarks Integrated input value according to equation:	
* 
* 		  f32IAccK_1 = [(f16InVal + f16InValK_1) / 2]  * a32Gain) + f32IAccK_1
* 		  f16InValK_1 = f16InVal
* 		  output = round(f32IAccK_1)
* 
* 		  THE SATURATION MUST BE TURNED OFF!
*
****************************************************************************/
extern inline frac16_t GFLIB_Integrator_F16_FAsmi(register frac16_t f16InVal, register GFLIB_INTEGRATOR_T_A32 *psParam)
{
	register acc32_t a32InVal;
	register acc32_t a32InValK_1;
	register acc32_t a32Gain;
	register acc32_t a32ResultH;
	register frac32_t f32ResultL;
	register frac16_t f16Result;

	asm(move.l x:(psParam)+,a32Gain);			/* a32Gain = psParam->a32Gain */
	asm(move.w x:(psParam+2),a32InValK_1);		/* a32InValK_1 = psParam->a32InValK_1 */
	asm(move.w f16InVal,x:(psParam+2));			/* psParam->a32InValK_1 = f16InVal */
	asm(asr16 f16InVal,a32InVal);				/* a32InVal = f16InVal >> 16 */
	asm(asr16 a32InValK_1);						/* a32InValK_1 = a32InValK_1 >> 16 */
	asm(add a32InValK_1,a32InVal);				/* a32InVal = a32InVal + a32InValK_1 */
	asm(impy64 a32InVal,a32Gain,a32ResultH);	/* a32ResultH:Y = (a32InVal + a32InValK_1) / 2 * psParam->a32Gain; used integer mpy to do one shift left */ 
	asm(move.l x:(psParam),f32ResultL);			/* f32ResultL = psParam-> f32IAccK_1 */
	asm(add Y,f32ResultL);						/* f32ResultL = psParam-> f32IAccK_1 + Y (lower long from the multiplication) */
	asm(move.l f32ResultL,x:(psParam));			/* psParam-> f32IAccK_1 = f32ResultL */
	asm(rnd f32ResultL);						/* Rounds f32ResultL to 16 bits */
	asm(move.w f32ResultL.1,f16Result);			/* f16Result = upper 16 bits of f32ResultL */
	
	return (f16Result);
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_INTEGRATOR_A32_ASMI_H_ */
