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
* @brief  Flex s-curve ramp functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _GFLIB_FLEXSRAMP_A32_ASM_H_
#define _GFLIB_FLEXSRAMP_A32_ASM_H_

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
#define GFLIB_FlexSRampInit_F16_Asm(f16InitVal, psParam) \
			GFLIB_FlexSRampInit_F16_FAsm(f16InitVal, psParam)

#define GFLIB_FlexSRampCalcIncr_F16_Asm(f16Target, a32Duration, psParam) \
			GFLIB_FlexSRampCalcIncr_F16_FAsm(f16Target, a32Duration, psParam)

#define GFLIB_FlexSRamp_F16_Asm(psParam) \
			GFLIB_FlexSRamp_F16_FAsm(psParam)


/******************************************************************************
* Types
******************************************************************************/
/* Flex dynamic ramp structure */
typedef struct
{
	 frac32_t f32State; 	/* State variable of x */
	 frac32_t f32Incr;		/* derivative of x (dX) - acceleration */
	 frac32_t f32AIncr;		/* derivative of a (dA) - der. of acceleration */
	 frac32_t f32ADes;		/* desired acceleration - must be <= f32Incrmax */
	 frac32_t f32Target; 	/* Target variable of x */
	 frac32_t f32Ts;		/* Sampling time */
	 frac32_t f32IncrMax;	/* Maximum increment of x */
	 frac32_t f32XT1;		/* X(T1) - x at the instant where the ramp got the desired acceleration (f32Incr = const) */
	 frac32_t f32XT2;		/* X(T2) - x at the instant where the ramp started to decelerate */
	 frac16_t f16DA;		/* Defined derivative of acceleration: dA = A / T */
	 uint16_t u16AccState;	/* State of the ramp process 0: f32Incr rising, 1: f32Incr = const, 2: f32Incr falling */
	 bool_t bReachFlag; 
} GFLIB_FLEXSRAMP_T_F32;


/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/


extern asm bool_t GFLIB_FlexSRampCalcIncr_F16_FAsm(frac16_t f16Target,
											acc32_t a32Duration,
									  		GFLIB_FLEXSRAMP_T_F32 *psParam);

extern asm frac16_t GFLIB_FlexSRamp_F16_FAsm(GFLIB_FLEXSRAMP_T_F32 *psParam);


/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Flex s-curve ramp initialization
*
* @param  in    		frac16_t f16InitVal
*                         - Measured instant value in [-1;1] in frac16_t
*
* @param  ptr			GFLIB_DFLEXRAMP_T_F32 *psParam
*						  - f32State: State variable keeping the last state x
*						  - f32Incr: derivative of x (dX) - acceleration
*						  - f32AIncr: derivative of a (dA) - der. of acceleration
*						  - f32ADes: desired acceleration - must be <= f32Incrmax
*						  - f32Target: Target value x
*						  - f32Ts: Sample time [s]
*						  - f32IncrMax: Maximum increment of x
*						  - f32XT1: X(T1) - x at the instant where the ramp got the desired acceleration (f32Incr = const)
*						  - f32XT2: X(T2) - x at the instant where the ramp started to decelerate
*						  - f16DA: Defined derivative of acceleration: dA = A / T
*						  - u16AccState: State of the ramp process 0: f32Incr rising, 1: f32Incr = const, 2: f32Incr falling
*						  - bReachFlag: Flag is set to TRUE if the desired value is achieved
*
* @return void
*		
* @remarks 	The init value is stored into the f32State variable. The f32AIncr is
* 			reset to 0 and the state variable u16AccState is set to 0. The
* 			bReachFlag flag is cleared.
*
*			SATURATION MODE INDEPENDENT!
*
****************************************************************************/
extern inline void GFLIB_FlexSRampInit_F16_FAsm(register frac16_t f16InitVal, register GFLIB_FLEXSRAMP_T_F32 *psParam)
{
	register frac32_t f32State;
	
	asm(move.w f16InitVal,f32State);	/* Extension to 32 bits */
	asm(move.l f32State,X:(psParam)+);	/* Saves the init value to the structure - f32State */
	
	asm(move.l #0,f32State);
	asm(move.l f32State,x:(psParam+4));/* Resets the A increment to 0 */
	
	asm(move.w #0,X:(psParam+17));		/* Sets u16AccState to 0 */
	asm(move.w #0,X:(psParam+18));		/* Clears bReachFlag */
	
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_FLEXSRAMP_A32_ASM_H_ */
