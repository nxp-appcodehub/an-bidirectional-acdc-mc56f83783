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
* @brief  Park and Park Inverse Transformations
* 
*******************************************************************************/
#ifndef _GMCLIB_PARK_F16_ASM_H_
#define _GMCLIB_PARK_F16_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"
#include "gmclib_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GMCLIB_Park_F16_Asmi(psIn, psAnglePos, psOut) GMCLIB_Park_F16_FAsmi(psIn, psAnglePos, psOut)
#define GMCLIB_ParkInv_F16_Asmi(psIn, psAnglePos, psOut) GMCLIB_ParkInv_F16_FAsmi(psIn, psAnglePos, psOut)

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
* @brief  The function calculates Park Transformation which is used for 
*         transforming values (current, voltage, flux) from 
*         alpha-beta stationary orthogonal coordination system 
*         to d-q rotating orthogonal coordination system
*
* @param  ptr			GMCLIB_2COOR_ALBE_T_F16 *psIn
*                       IN  - 	pointer to structure containing data of two phase
*                           	stationary orthogonal system
*                       GMCLIB_2COORD_SINCOS_T_F16 *psAnglePos
*                       IN  - 	pointer to structure where the values 
*                           	of sine and cosine are stored
*                       GMCLIB_2_COOR_DQ_T_F16 *psOut
*                       OUT - 	pointer to structure containing data of 
*                           	DQ coordinate two-phase stationary 
*                           	orthogonal system
*
* @remarks  Modifies the structure pointed by pDQ pointer
*           according to the following equations
*           d = alpha * cos(theta) + beta * sin(theta)
*           q = beta * cos(theta) - alpha * sin(theta)
*
*			THE FUNCTION IS SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline void GMCLIB_Park_F16_FAsmi(register const GMCLIB_2COOR_ALBE_T_F16 *psIn,
									  	 register const GMCLIB_2COOR_SINCOS_T_F16 *psAnglePos,
									  	 register GMCLIB_2COOR_DQ_T_F16 *psOut)
{
	asm(.optimize_iasm on);
	
	asm(move.w 	X:(psAnglePos)+,Y0);
	
	asm(move.w 	X:(psIn)+,X0);						/* Y0 = sin; X0 = alpha */
	
	asm(move.w 	X:(psAnglePos)+,Y1);				/* Y1 = cos */
	
	asm(mpy 	-Y0,X0,B);							/* B = -alpha * sin */
	
	asm(mpy 	Y1,X0,A  	X:(psIn)+,X0);			/* A = alpha * cos; X0 = beta */
		
	asm(macr 	Y0,X0,A);			  				/* A = A [alpha * cos] + beta * sin	*/

	asm(macr 	Y1,X0,B  	A,X:(psOut)+);			/* B = B [-alpha * sin] + beta * cos */
	
	asm(nop);										/* psOut -> f16D = A (with saturation) */
	
	asm(move.w 	B,X:(psOut));						/* psOut -> f16Q = B (with saturation) */	
	
	asm(.optimize_iasm off);	
}

/***************************************************************************//*!
*
* @brief  The function calculates Inverse Park Transformation which is used 
*         for transforming values (current, voltage, flux) from 
*         d-q rotating orthogonal coordination system to alpha-beta 
*         stationary orthogonal coordination system.
*
* @param  ptr			GMCLIB_2COOR_DQ_T_F16 *psIn
*                       IN   - 	pointer to structure containing data of 
*                           	DQ coordinate two-phase stationary 
*                           	orthogonal system
*                       MCLIB_ANGLE_T *pudtSinCos
*                       IN   - 	pointer to structure where the values 
*                           	of sine and cosine are stored
*                       GMCLIB_2COOR_ALBE_T_F16 *psOut
*                       OUT  - 	pointer to structure containing data of two phase
*                           	stationary orthogonal system
*                       
* @remarks  Modifies the structure pointed by pAlphaBeta pointer 
*           according following equations:
*           alpha = d * cos(theta) - q * sin(theta)
*           beta  = d * sin(theta) + q * cos(theta)
*
*			THE FUNCTION IS SATURATION INDEPENDENT!
*
*******************************************************************************/
extern inline void GMCLIB_ParkInv_F16_FAsmi(register const GMCLIB_2COOR_DQ_T_F16 *psIn,
											register const GMCLIB_2COOR_SINCOS_T_F16 *psAnglePos,
											register GMCLIB_2COOR_ALBE_T_F16 *psOut)
{
	asm(.optimize_iasm on);
	
	asm(move.w X:(psAnglePos)+,Y0);
	
	asm(move.w X:(psIn)+,X0);						/* Y0 = sin; X0 = d */
	
	asm(move.w X:(psAnglePos)+,Y1);					/* Y1 = cos */
		
	asm(mpy Y0,X0,B);	 							/* B = sin * d */
	
	asm(mpy Y1,X0,A  X:(psIn)+,X0);					/* A = cos * d;  X0 = q */
	
	asm(macr -X0,Y0,A);								/* A = A [cos * d] - sin * q */

	asm(macr X0,Y1,B  A,X:(psOut)+);				/* B = B [sin * d] + cos * q */
	
	asm(nop);										/* psOut -> f16Alpha = A (with saturation) */
	
	asm(move.w B,X:(psOut));						/* psOut -> f16Beta = B (with saturation) */	
	
	asm(.optimize_iasm off);	
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GMCLIB_PARK_F16_ASM_H_ */
