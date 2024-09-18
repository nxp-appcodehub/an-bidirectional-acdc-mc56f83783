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
* @brief  Dynamic ramp functions with 32-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _GFLIB_DRAMP_F32_ASM_H_
#define _GFLIB_DRAMP_F32_ASM_H_

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
#define GFLIB_DRampInit_F32_Asm(f32InitVal, psParam) \
			GFLIB_DRampInit_F32_FAsm(f32InitVal, psParam);

#define GFLIB_DRamp_F32_Asm(f32Desired, f32Instant, pbStopFlag, psParam) \
			GFLIB_DRamp_F32_FAsm(f32Desired, f32Instant, pbStopFlag, psParam)

/******************************************************************************
* Types
******************************************************************************/

/* Ramp structure */
typedef struct
{
    frac32_t f32RampUp;
    frac32_t f32RampDown;
	frac32_t f32RampUpSat;
	frac32_t f32RampDownSat;
	frac32_t f32State;
	bool_t bReachFlag;
} GFLIB_DRAMP_T_F32;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Ramp function 32-bit version
*
* @param  ptr			GFLIB_RAMP32 *psParam
*						  - f32RampUp: Ramp-up increment
*						  - f32RampDown: Ramp-down increment
*						  - f32RampUpSat: Ramp-up increment used in case of saturation
*						  - f32RampDownSat: Ramp-down increment used in case of saturation
*						  - f32State: Previous ramp value
*						  - bReachFlag: Flag is set to 1 if the desired value is achieved
* @param  in    		frac32_t f32Desired
*                         - Desired value in [-1;1] in frac32_t
*						frac32_t f32Instant
*						  - Instant value in [-1;1] in frac32_t
*						uint16_t pbStopFlag
*						  - determines the saturation mode: 0 non-saturation
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function ramps the value from the actual value up/down to
*			the f16Target value using the up/down increments defined in
*			the pParam structure. In case of saturation (pbStopFlag != 0)
*			the function uses the saturation up/down increments and ramps
*			the value toward the f16Instant value.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm frac32_t GFLIB_DRamp_F32_FAsm(frac32_t f32Desired,
									  frac32_t f32Instant,
									  const bool_t *pbStopFlag,
						   			  GFLIB_DRAMP_T_F32 *psParam);

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function initializes the actual value of DynRamp32.
*
* @param  ptr   		GFLIB_RAMP32 *psParam
*						  - f32RampUp: Ramp-up increment
*						  - f32RampDown: Ramp-down increment
*						  - f32RampUpSat: Ramp-up increment used in case of saturation
*						  - f32RampDownSat: Ramp-down increment used in case of saturation
*						  - f32State: Previous ramp value
*						  - bReachFlag: Flag of state, when f16State and f16Target are equal
*
* @param  in    		frac16_t f16InitVal
* 							- Initial value
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern inline void GFLIB_DRampInit_F32_FAsm(register frac32_t f32InitVal, register GFLIB_DRAMP_T_F32 *psParam)
{
	asm(move.l f32InitVal,X:(psParam+8));	/* Sets f32State */
	asm(move.w #0,X:(psParam+10));			/* Clears bReachFlag */
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_DRAMP_F32_ASM_H_ */
