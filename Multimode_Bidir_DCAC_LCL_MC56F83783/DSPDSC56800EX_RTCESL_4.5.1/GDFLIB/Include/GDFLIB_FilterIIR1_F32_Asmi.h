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
* @brief  Digital IIR Filter, 1st order assembler implementation for V3
* 
*******************************************************************************/
#ifndef _GDFLIB_FILTERIIR1_F32_ASMI_H_
#define _GDFLIB_FILTERIIR1_F32_ASMI_H_

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
	frac32_t			f32B0;
	frac32_t			f32B1;
	frac32_t			f32A1;
} GDFLIB_FILTER_IIR1_COEFF_T_F32;

typedef struct 
{
    GDFLIB_FILTER_IIR1_COEFF_T_F32	sFltCoeff;
    frac32_t 						f32FltBfrY[1];
    frac16_t 						f16FltBfrX[1];
} GDFLIB_FILTER_IIR1_T_F32;

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
* @brief  16-bit input and output 1st order IIR filter
*
* @param  ptr			GDFLIB_FILTER_IIR1_T *psParam
* 						- pointer to the structure of the filter
*
*                         GDFLIB_FILTER_IIR1_T_F32 data structure:
*                           GDFLIB_FILTER_IIR1_COEFF_T_F32	sFltCoeff   
*                               - filter coefficients in frac32_t divided by 2
*                                 in the following range
*                                 -1.0 <= f1PGain < 1.0
*                               - B coefficients:
*                               	frac32_t			f32B0
*									frac32_t			f32B1
*								- -A coefficient, the sign is negated
*									frac32_t			f32A1;
*                           frac32_t f32FltBfrY[1];
*                               - y history buffer in frac32_t
*							frac16_t f16FltBfrX[1];
*								- x history buffer in frac16_t
*                           
* @param  in    		frac16_t f16InX
*                         - Input signal at the step K step
*                           fractional value in the range <-1;1)
*                                                  
* @return This function filtered value
*     - frac16_t value [-1;1]

* @remarks 	    Filter algorithm:                                           
*                                                                       
*                        -----------                 
*                 x(k)  |           |  y(k)           
*               --------|    IIR4   |---------        
*                       |           |                 
*                        -----------                  
* 		                                                                
*       x(k) - input signal                    
*       y(k) - filtered output signal              
*                                                                       
*		Filter equation:
*		y(n) = b0*x(n) + b1*x(n-1) + a1*y(n-1)
*                                                                           
*		All coefficients must be divided by 2, i.e. if the desired b1 coefficient is 1.756 the value is then:
*		b1 = FRAC32(1.786 / 2.0);
*		
*		All A coefficients must be negated, i.e. if the desired a2 coefficient is 3.789, the value is then:
*		a2 = FRAC32(-3.789 / 2.0); 
*
*	|offset	|	0	|	2	|	4	|
*	|-------|-------|-------|-------|
*	|   	|	b0	|   b1  |  -a1  |
*
*	|offset	|	6	|
*	|-------|-------|
*	|   	| y(n-1)|
*
*	|offset	|	8	|
*	|-------|-------|
*	|   	| x(n-1)|
*
*
*	
*	
*	THE SATURATION MUST BE TURNED OFF!
*	The V3 core instructions used! 
*
****************************************************************************/
extern inline frac16_t GDFLIB_FilterIIR1_F16_FAsmi(register frac16_t f16InX, register GDFLIB_FILTER_IIR1_T_F32 * psParam)
{   
	register frac32_t *pf32Coef;
	register frac16_t *pf16XHist;
	register frac32_t f32Temp;
	register frac16_t f16TempX;
	register uint16_t u16SR;
	register frac16_t f16Result;

	asm(tfra psParam,pf32Coef);					/* pf32Coef -> b0 */
	asm(adda #0x8,psParam,pf16XHist);			/* pf16XHist -> x(n-1) */
			
	/* psParam -> x(n-1), pf32Coef -> b0 */
	asm(move.w	f16InX,B);						/* B = x(n) */
	asm(move.l	X:(pf32Coef)+,Y);				/* Y = b0 */
	asm(mpy32	B,Y,A 	X:(pf32Coef)+,Y);		/* A = b0 * x(n) */
														/* Y = b1 */
	asm(move.w	X:(pf16XHist),f16TempX);		/* f16TempX = x(n-1) */
	asm(move.w	B1,X:(pf16XHist));				/* x(n-1) = x(n) */
			
	/* psParam -> x(n-1), pf32Coef -> b2 */

	asm(move.w	f16TempX,B);					/* B = x(n-1) */
	asm(mac32	B,Y,A	X:(pf32Coef)+,Y);		/* A = A + b1 * x(n-1) */
														/* Y = -a1 */
	asm(move.l	X:(pf32Coef),B);				/* B = y(n-1) */
		
	/* pf32Coef -> y(n-1) */

	asm(mac32	B,Y,A);							/* A = A - a1 * y(n-1) */
	asm(nop);
			
	/* A = A << 1; final scale */
			
	asm(asl		A);								/* A = A << 1 */

	/* A = saturation of A to 32-bit frac */
	asm(move.w	SR,u16SR);						/* Stores the SR to X0 */
	asm(move.w	A2,f32Temp);					/*f32Temp = A2 */
	asm(bfchg 	#0x8000,f32Temp.1);				/* changes the MSB of the upper long */			
	asm(bftsth 	#0x20,u16SR);					/* Check the E bit of SR (extension in use) */
	asm(tcs 	f32Temp,A);						/* If E bit set then A = f32Temp */
	asm(sat 	A);								/* Saturates A */

	asm(move.l	A10,X:(pf32Coef));				/* A -> y(n-1) 32-bit	*/

	asm(rnd		A);								/* round to 16 bits */

	asm(move.w	A,f16Result);					/* y(n) 16-bit output with saturation */
			
	return (f16Result);
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GDFLIB_FILTERIIR1_F32_ASMI_H_ */
