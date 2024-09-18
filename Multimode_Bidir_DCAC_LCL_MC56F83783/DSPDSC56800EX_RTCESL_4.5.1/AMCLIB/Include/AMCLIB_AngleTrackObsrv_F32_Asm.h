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
* @brief  Angle tracking observer
* 
*******************************************************************************/
#ifndef _AMCLIB_ANGLE_TRACK_OBSRV_F32_ASM_H_
#define _AMCLIB_ANGLE_TRACK_OBSRV_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"
#include "gflib.h"
#include "gmclib.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define AMCLIB_AngleTrackObsrvInit_F16_Asm(f16ThetaInit, psCtrl) AMCLIB_AngleTrackObsrvInit_F16_FAsm(f16ThetaInit, psCtrl)
#define AMCLIB_AngleTrackObsrv_F16_Asm(psSinCos, psCtrl) AMCLIB_AngleTrackObsrv_F16_FAsm(psSinCos, psCtrl)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	frac32_t	f32Speed;       	//0
	frac32_t	f32A2;          	//2
    frac16_t  	f16Theta;       	//4
	frac16_t	f16SinEstim;    	//5
	frac16_t	f16CosEstim;    	//6
	frac16_t	f16K1Gain;    		//7
	int16_t		i16K1GainSh;        //8
	frac16_t	f16K2Gain;    		//9
	int16_t		i16K2GainSh;        //10
	frac16_t	f16A2Gain;    		//11
	int16_t		i16A2GainSh;        //12
		
} AMCLIB_ANGLE_TRACK_OBSRV_T_F32;

/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Tracks the angle of the system and calculates the speed.
*
* @param  ptr   		GMCLIB_2COORD_SINCOS_T_F16 *psSinCos
*                         - angle's sine and cosine components
*
*                       AMCLIB_ANGLE_TRACK_OBSRV_T_F32 * psCtrl
*                         - parameters of the observer
*
* @param  in    		None
*
* @return This function returns the calculated angle
*		
* @remarks SATURATION MUST BE TURNED OFF! 	
*
****************************************************************************/
extern asm frac16_t AMCLIB_AngleTrackObsrv_F16_FAsm
(
    const GMCLIB_2COOR_SINCOS_T_F16 *psSinCos,
    AMCLIB_ANGLE_TRACK_OBSRV_T_F32 *psCtrl
);

extern asm void AMCLIB_AngleTrackObsrvInit_F16_FAsm(frac16_t f16ThetaInit, AMCLIB_ANGLE_TRACK_OBSRV_T_F32 *psCtrl);

/******************************************************************************
* Inline functions
******************************************************************************/

#if defined(__cplusplus) 
} 
#endif 

#endif /* _AMCLIB_ANGLE_TRACK_OBSRV_F32_ASM_H_ */
