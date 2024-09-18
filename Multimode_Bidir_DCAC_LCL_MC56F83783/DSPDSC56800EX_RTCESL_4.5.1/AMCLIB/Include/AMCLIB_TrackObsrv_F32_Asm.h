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
* @brief  Tracking observer for determination angular speed and position of 
* 		  input error functional signal
* 
*******************************************************************************/
#ifndef _AMCLIB_TRACK_OBSRV_F32_ASM_H_
#define _AMCLIB_TRACK_OBSRV_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"
#include "gmclib.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define AMCLIB_TrackObsrv_F16_Asm(f16Error, psCtrl) AMCLIB_TrackObsrv_F16_FAsm(f16Error, psCtrl)
#define AMCLIB_TrackObsrvInit_F16_Asmi(f16ThetaInit, psCtrl) AMCLIB_TrackObsrvInit_F16_FAsmi(f16ThetaInit, psCtrl)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{

	frac32_t	f32Theta;
	frac32_t	f32Speed;
	frac32_t	f32I_1;
	frac16_t	f16IGain;
	int16_t		i16IGainSh;
	frac16_t	f16PGain;
	int16_t		i16PGainSh;
	frac16_t	f16ThGain;
	int16_t		i16ThGainSh;
	
} AMCLIB_TRACK_OBSRV_T_F32;

/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/

extern asm frac16_t AMCLIB_TrackObsrv_F16_FAsm(frac16_t f16Error, 
											   AMCLIB_TRACK_OBSRV_T_F32 *psCtrl);

/******************************************************************************
* Inline functions
******************************************************************************/
/***************************************************************************//*!
*
* @brief  			Tracking observer initialization
*
* @param  ptr   		AMCLIB_TRACK_OBSRV_T_F32 *psCtrl
*                         frac32_t f32Theta
*                         	- Estimated position <-1;1) corresponds to <-pi;pi)
*                         frac32_t f32Speed
*                         	- Estimated speed <-1;1)
*                         frac16_t f32I_1
*                         	- Internal integrator <-1;1)
*                         frac16_t f16IGain
*                         	- Integ. constant to get speed from error <0;1)
*                         int16_t i16IGainSh
*                         	- Shift for f16IGain <-15;15>
*                         frac16_t f16PGain
*                         	- Prop. constant to get speed from angle <0;1)
*                         int16_t i16PGainSh
*                         	- Shift for f16PGain <-15;15>
*                         frac16_t f16ThGain
*                         	- Constant to get angle from speed <0;1)
*                         int16_t i16ThGainSh
*                         	- Shift for f16ThGain <-15;15>
*
* @param  in    	frac16_t f16ThetaInit
*                         - init angle <-1;1) corresponds to <-pi;pi)
*
* @return 			None
*                         
* @remarks	Initializes the structure of the tracking observer with an angle
* 			
* 			f32Theta = f16ThetaInit << 16
* 			
* 			f32Speed = 0
* 			f32I_1 = 0
* 			
*	 			THE SATURATION MUST BE TURNED OFF!
*
****************************************************************************/
extern inline void AMCLIB_TrackObsrvInit_F16_FAsmi(register frac16_t f16ThetaInit, 
											   register AMCLIB_TRACK_OBSRV_T_F32 *psCtrl)
{
	register frac32_t f32Theta;
	
	asm(move.w f16ThetaInit,f32Theta);			/* f32Theta = f16ThetaInit << 16 */
	asm(move.l f32Theta,x:(psCtrl)+);			/* Stores the initial value */
	asm(move.l #0,f32Theta);
	asm(move.l f32Theta,x:(psCtrl)+);			/* Clears f32Speed */
	asm(move.l f32Theta,x:(psCtrl)+);			/* Clears f32I_1 */	

}


#if defined(__cplusplus) 
} 
#endif 

#endif /* _AMCLIB_TRACK_OBSRV_F32_ASM_H_ */
