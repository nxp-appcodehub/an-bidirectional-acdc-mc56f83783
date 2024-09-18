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
* @brief  Sine and cosine functions 
* 
*******************************************************************************/
#ifndef _GFLIB_SINCOS_F32_ASM_H_
#define _GFLIB_SINCOS_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"
#include "gflib_types.h"

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
	frac32_t  f32A[5];
} GFLIB_SIN_T_F32;

/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/

extern asm frac16_t GFLIB_Sin_F16_FAsm(frac16_t f16Angle, const GFLIB_SIN_T_F32 *psParam);

/***************************************************************************//*!
*
* @brief  Calculates the cosine of the given argument using Taylor 9th order
*         polynomial. Inline function which increments the argument by 0.5 and
*		  calls the Sin calculation.
*
* @param  ptr		frac16_t *psParam
*					- Pointer to the Taylor polynomial coefficients
* @param  in    	frac16_t f16Angle
*                   - Argument in range [-1; 1) in frac16_t corresponding
*					  to [-pi; pi)
*
* @return This function returns
*     				- frac16_t value [-1; 1)
*		
* @remarks 	
*	This function calculates cos(pi * x) using the Taylor 9th order
*	polynomial approximation of sine function.
*	At the beginning the input argument of cosine function
*	is transferred to argument of sine function:
*
*	sine_argument = pi/2 - ABS(cos_argument)
*
*	then the cosine function is calculated using the sine function.
*	The constant pi/2 can not be directly added to the cos_argument,
*	because the saturation is enabled, therefore the saturation
*	can occur during the addition.
*
*	The function calculates sin(pi * x) using the Taylor 9th order	polynomial:
*	x=[-pi; pi]:
*		sine(x) = x - x^3/3! + x^5/5! - x^7/7! + x^9/9!
*		sine(x) = x*(1 - x^2/6*(1 - x^2/20*(1 - x^2/42*(1 - x^2/72))))
*		sine(x) = x(d1 + x^2(d3 + x^2(d5 + x^2(d7 + x^2*d9))))
*
*		where:
*		d1 =  1
*		d3 = -1 / 3!
*		d5 =  1 / 5!
*		d7 = -1 / 7!
*		d9 =  1 / 9!
*
*	because the x value is in range [-1; 1) that corresponds to [-pi; pi],
*	the coefficients 'd' needs to be scaled:
*
*		x=[-1; 1):
*		sin(pi * x) = x(c1 + x^2(c3 + x^2(c5 + x^2(c7 + x^2*c9))))
*
*		c1 = d1 * pi^1 =  pi
*		c3 = d3 * pi^3 = -pi^3 / 3!
*		c5 = d5 * pi^5 =  pi^5 / 5!
*		c7 = d7 * pi^7 = -pi^7 / 7!
*		c9 = d9 * pi^9 =  pi^9 / 9!
*
*	The 9th order polynomial approximation of the sine function
*	has very good accuracy in range [-pi/2; pi/2] of argument, but in wider
*	range the calculation error is quickly growing up. Therefore the sine
*	function is	approximated only in range [-pi/2; pi/2] and it is used the
*	symmetry of the sine function [ sin(alpha) = sin(pi - alpha) ].
*	Therefore the alpha argument is in range [-pi/2; pi/2) that corresponds
*	to x [-0.5; 0.5).
*	To make calculations more precise (because we need to use the value
*	of x^2 rounded to 16-bit fractional number in our calculations),
*	the given argument value x (that is in range [-0.5; 0.5)) is shifted
*	by 1 bit to the left (multiplied by 2), then the calculated x^2 value
*	is in range [-1; 1) instead of [-0.25; 0.25).
*	Then the polynomial coefficients 'c' needs to be scaled
*	(shifted to the right):
*
*		b1 = c1 / 2^1 =  pi / 2
*		b3 = c3 / 2^3 = -pi^3 / 3! / 2^3
*		b5 = c5 / 2^5 =  pi^5 / 5! / 2^5
*		b7 = c7 / 2^7 = -pi^7 / 7! / 2^7
*		b9 = c9 / 2^9 =  pi^9 / 9! / 2^9
*
*	To avoid the saturation error during the polynomial calculation
*	the coefficients 'b' are divided by 2. After the polynomial calculation
*	the result is multiplied by 2 (shifted by 1 bit to the left)
*	to take the right result ( in range [-1; 1) ).
*
*		a1 = b1 / 2 =  pi / 2^2         =  0.785398163
*		a3 = b3 / 2 = -pi^3 / 3! / 2^4  = -0.322982049
*		a5 = b5 / 2 =  pi^5 / 5! / 2^6  =  0.039846313
*		a7 = b7 / 2 = -pi^7 / 7! / 2^8  = -0.002340877
*		a9 = b9 / 2 =  pi^9 / 9! / 2^10 =  0.000080220
*
*		In 32-bit signed fractional representation:
*		a1 = 0x6487ED51
*		a3 = 0xD6A88634
*		a5 = 0x0519AF1A
*		a7 = 0xFFB34B4D
*		a9 = 0x0002A0F0
*
*	sin(pi * x) = (x<<1)*(a1 + (x<<1)^2(a3 + (x<<1)^2(a5 + (x<<1)^2(a7 \
*				  + (x<<1)^2*a9)))) << 1
*
*	For better accuracy the (x<<1)^2*(a#...) calculations are performed
*	as the following:
*		(x<<1)^2 as 16-bit fractional value
*			  a# as 32-bit fractional value
*		[ (x<<1)^2 ] * [ (a#... ] = [16-bit]*[32-bit] = 32-bit number
*
* 			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t GFLIB_Cos_F16_FAsmi(register frac16_t f16Angle, register const GFLIB_SIN_T_F32 *psParam)
{
	asm(.optimize_iasm on);
	/* change argument of cosine function to sin(x+0.5pi); uses long because of saturation independence */
	asm(move.w f16Angle,Y0);
	asm(clr.w Y1);
	asm(add.l #16384,Y);
	asm(move.w Y0,f16Angle);
	return GFLIB_Sin_F16_FAsm(f16Angle, psParam);
	asm(.optimize_iasm off);
}

/******************************************************************************
* Inline functions
******************************************************************************/

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_SINCOS_F32_ASM_H_ */
