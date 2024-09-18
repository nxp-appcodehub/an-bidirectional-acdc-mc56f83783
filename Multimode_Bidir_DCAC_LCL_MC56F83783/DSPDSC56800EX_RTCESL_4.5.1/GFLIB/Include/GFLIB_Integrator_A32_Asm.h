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
* @brief  Integrator algorithms implemented in assembler
* 
*******************************************************************************/
#ifndef _GFLIB_INTEGRATOR_A32_ASM_H_
#define _GFLIB_INTEGRATOR_A32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"
#include "GFLIB_Integrator_A32_Asmi.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GFLIB_IntegratorInit_F16_Asmi(f16InitVal, psParam) GFLIB_IntegratorInit_F16_FAsmi(f16InitVal, psParam)

#define GFLIB_Integrator_F16_Asmi(f16InVal, psParam) GFLIB_Integrator_F16_FAsmi(f16InVal, psParam)

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
* @brief  The function initializes the integral part of integrator.
*
* @param  ptr   		GFLIB_INTEGRATOR_T *psParam
*						  	- Pointer to integrator structure
*
*						GFLIB_INTEGRATOR_T data structure:						  
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
*								- error at the step k-1;
*                                 input/output parameter
*                                 in the following range
*                                 -1.0 <= f16InErrK_1 < 1.0   
						  	
*
* @param  in    		Frac16 f16InitVal
* 							- Initial value
*
* @return N/A
*		
* @remarks The function sets the integral portion state variable f32IAccK_1 to
* 		   the value f16InitVal in such a way that if the inputs are zero
* 		   the result of the controller will be equal to f16InitVal.
* 		   At the same time the f16InErrK_1 value is zeroed.
*
****************************************************************************/
extern inline void GFLIB_IntegratorInit_F16_FAsmi(register frac16_t f16InitVal, register GFLIB_INTEGRATOR_T_A32 *psParam)
{
	register frac32_t f32Integral;
	
	asm(move.w f16InitVal,f32Integral);			/* f32Integral = f16InitVal << 16 */
	asm(move.l f32Integral,x:(psParam + 2));	/* Stores the integral portion */

	psParam->f16InValK_1 = 0;					/* Resets the previous step error */
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_INTEGRATOR_A32_ASM_H_ */
