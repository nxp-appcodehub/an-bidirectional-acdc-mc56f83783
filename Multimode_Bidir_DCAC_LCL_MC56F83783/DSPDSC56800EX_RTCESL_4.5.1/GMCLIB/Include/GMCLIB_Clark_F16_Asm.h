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
* @brief  Clarke and Clarke Inverse Transformations
* 
*******************************************************************************/
#ifndef _GMCLIB_CLARK_F16_ASM_H_
#define _GMCLIB_CLARK_F16_ASM_H_

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
#define GMCLIB_ONE_DIV_SQRT3_F16	18919
#define GMCLIB_SQRT3_DIV_2_F16		28378

#define	GMCLIB_Clark_F16_Asmi(psIn, psOut) GMCLIB_Clark_F16_FAsmi(psIn, psOut)
#define GMCLIB_ClarkInv_F16_Asmi(psIn, psOut) GMCLIB_ClarkInv_F16_FAsmi(psIn, psOut)

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
* @brief  The function calculates Clarke Transformation which is used 
*         for transforming values (current, voltage, flux) from the 
*         three phase stationary coordination system  to alpha-beta 
*         stationary orthogonal coordination system.
*
* @param  ptr			GMCLIB_3COOR_T_F16 *psIn
*						IN  - pointer to structure containing data of three phase
*                           stationary system
*                       GMCLIB_2COOR_ALBE_T_F16 *psOut
*                       OUT - pointer to structure containing data of two phase
*                           stationary orthogonal system
*                       
*
* @remarks 	Modifies the structure pointed by pudtAlphaBeta pointer 
*           according to the folloving equations:
*           alpha = a
*           beta  = b * 1 / sqrt(3)  - c * 1 / sqrt(3) 
*
*			THE FUNCTION IS SATURATION INDEPENDENT!
* 
****************************************************************************/
extern inline void GMCLIB_Clark_F16_FAsmi(register const GMCLIB_3COOR_T_F16 *psIn,
						    	   	   	  register GMCLIB_2COOR_ALBE_T_F16 *psOut)
{
	asm(.optimize_iasm on);
	
	asm(move.w X:(psIn)+,Y0);					/* Y0 = a */
	
	asm(move.w X:(psIn)+,B);					/* B1 = b */
	
	asm(move.w #GMCLIB_ONE_DIV_SQRT3_F16,Y1);	/* X0 = 1 / Sqrt(3) */
	
	asm(mpy B1,Y1,A  X:(psIn)+,X0);				/* A = b * 1 / Sqrt(3); X0 = c */
		
	asm(neg Y1);								/* Y1 = - 1 / sqrt(3) */
	
	asm(macr Y1,X0,A Y0,X:(psOut)+);			/* A = b * 1 / Sqrt(3) - c * 1 / Sqrt(3) */
	
	asm(nop);									
	
	asm(move.w A,X:(psOut)+);					/* pudtAlphaBeta -> f16Beta = A (with saturation) */

	asm(.optimize_iasm off);	
}

/***************************************************************************//*!
*
* @brief  	The function calculates Inverse Clarke Transformation which is used
*           for transforming values (current, voltage, flux) from alpha-beta 
*           stationary orthogonal coordination system to three phase 
*           stationary coordination system
*
* @param  ptr			GMCLIB_2COOR_ALBE_T_F16 *psIn
*                       IN  - pointer to structure containing data of two phase
*                           stationary orthogonal system
*                       GMCLIB_3COOR_T_F16 *psOut
*						OUT - pointer to structure containing data of three phase
*                           stationary system
*                       
*
* @remarks 	Modifies the structure pointed by p_abc pointer 
*           according to the folloving equations:
*           a = alpha
*           b = -0.5 * alpha + sgrt(3) / 2 * beta
*           c = -0.5 * alpha - sgrt(3) / 2 * beta
*
*			THE FUNCTION IS SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline void GMCLIB_ClarkInv_F16_FAsmi(register const GMCLIB_2COOR_ALBE_T_F16*psIn,
							   		  	  	 register GMCLIB_3COOR_T_F16 *psOut)
{
	asm(.optimize_iasm on);
	
	asm(move.w X:(psIn)+,A);				/* A = alpha */

	asm(move.w X:(psIn)-,Y1);				/* Y1 = beta */
	
	asm(move.w #GMCLIB_SQRT3_DIV_2_F16,X0);/* X0 = Sqrt(3) / 2 */
	
	asm(asr A A,X:(psOut)+);				/* pudtAbc -> f16A = alpha */
											/* A = 0.5 * alpha */
	asm(neg A);								/* A = -0.5 * alpha */
	
	asm(tfr A,B);							/* B = -0.5 * alpha */
	
	asm(macr Y1,X0,A);						/* A = -0.5 * alpha + sqrt(3) / 2 * beta */ 
	
	asm(macr -Y1,X0,B);						/* B = -0.5 * alpha - sqrt(3) / 2 * beta */
	
	asm(move.w A,X:(psOut)+);				/* pudtAbc -> f16B = -0.5 * alpha + sgrt(3) / 2 * beta */
	
	asm(move.w B,X:(psOut));				/* pudtAbc -> f16C = -0.5 * alpha - sgrt(3) / 2 * beta */	
	
	asm(.optimize_iasm off);	
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GMCLIB_CLARK_F16_ASM_H_ */
