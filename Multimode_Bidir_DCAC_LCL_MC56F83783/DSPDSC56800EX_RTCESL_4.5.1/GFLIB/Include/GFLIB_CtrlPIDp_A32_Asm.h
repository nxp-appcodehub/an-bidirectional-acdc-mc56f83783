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
* @brief  The parallel form of Proportional-Integral-Derivative (PID) regulator 
* 		  algorithms implemented in assembler
* 
*******************************************************************************/
#ifndef _GFLIB_CTRLPIDP_A32_ASM_H_
#define _GFLIB_CTRLPIDP_A32_ASM_H_

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
#define GFLIB_CtrlPIDpAWInit_F16_Asmi(f16InitVal, psParam) GFLIB_CtrlPIDpAWInit_F16_FAsmi(f16InitVal, psParam)

#define GFLIB_CtrlPIDpAW_F16_Asm(f16InErr, f16InErrD, pbStopIntegFlag, psParam) GFLIB_CtrlPIDpAW_F16_FAsm(f16InErr, f16InErrD, pbStopIntegFlag, psParam)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
   acc32_t a32PGain;
   acc32_t a32IGain;
   acc32_t a32DGain;
   frac32_t f32IAccK_1;
   frac16_t f16InErrK_1;
   frac16_t f16UpperLim;  
   frac16_t f16LowerLim;    
   frac16_t f16InErrDK_1;
   bool_t  bLimFlag;
} GFLIB_CTRL_PID_P_AW_T_A32;


/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
extern asm frac16_t GFLIB_CtrlPIDpAW_F16_FAsm(frac16_t f16InErr, frac16_t f16InErrD, const bool_t *pbStopIntegFlag, GFLIB_CTRL_PID_P_AW_T_A32 *psParam);

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function initializes the integral part of the PID controller.
*
* @param  ptr			GFLIB_CTRL_PID_P_AW_T_A32 *psParam
* 						- pointer to the structure of the PI controller
*
*                         GFLIB_CTRL_PID_P_AW_T_A32 data structure:
*                           acc32_t a32PGain    
*                               - proportional gain; input parameter
*                                 in the following range
*                                 0 <= f16PGain < 65536.0  
*                           acc32 a32IGain
*                               - integral gain; input parameter
*                                 in the following range
*                                 0 <= f16IGain < 65536.0
*                           acc32 a33DGain
*                               - derivative gain; input parameter
*                                 in the following range:
*                                 0 <= f16DGain < 65536.0  
*                           frac32_t f32IAccK_1
*                               - state variable; fractional part (lower long)
*                                 of integral part at step k-1;
*                                 can be modified outside of the function;
*                                 input/output parameter
*                                 in the following range
*                                 -1.0 <= f16PropGain < 1.0                               
*							frac16_t f16InErrK_1
*								- error at the step k-1;
*                                 input/output parameter
*                                 in the following range
*                                 -1.0 <= f16PropGain < 1.0   
*                           frac16_t f16UpperLim
*                               - upper limit of the controller output;
*                                 input parameter
*                                 in the following range
*                                 -1.0 <= f16PropGain < 1.0                                    
*                                 f16UpperLim > f16LowerLim
*                           frac16_t f16LowerLim
*                               - lower limit of the controller output;
*                                 input parameter
*                                 in the following range
*                                 -1.0 <= f16PropGain < 1.0   
*                                 f16UpperLim > f16LowerLim
*							frac16_t f16InErrDK_1
*								- derivative input's error at the step k-1;
*                                 input/output parameter
*                                 in the following range
*                                 -1.0 <= f16InErrDK_1 < 1.0   
*                           bool_t bLimFlag
*                               - limitation flag; if set, the controller
*                                 output reached either f16UpperLim or f16LowerLim,
*                                 otherwise it is cleared
*                                 output parameter 
*
* @param  in    		frac16_t f16InitVal
* 							- Initial value
*
* @return N/A
*		
* @remarks The function sets the integral portion state variable f32IAccK_1 to
* 		   the value f16InitVal in such a way that if the inputs are zero
* 		   the result of the controller will be equal to f16InitVal.
* 		   At the same time the f16InErrK_1 and f16InErrDK_1 value is zeroed.
*
****************************************************************************/
extern inline void GFLIB_CtrlPIDpAWInit_F16_FAsmi(register frac16_t f16InitVal, register GFLIB_CTRL_PID_P_AW_T_A32 *psParam)
{
	register frac32_t f32Integral;
	
	asm(move.w f16InitVal,f32Integral);			/* f32Integral = f16InitVal << 16 */
	asm(move.l f32Integral,x:(psParam + 6));	/* Stores the integral portion */

	psParam->f16InErrK_1 = 0;					/* Resets the previous step error */
	psParam->f16InErrDK_1 = 0;					/* Resets the previous step D error */
}


#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_CTRLPIDP_A32_ASM_H_ */
