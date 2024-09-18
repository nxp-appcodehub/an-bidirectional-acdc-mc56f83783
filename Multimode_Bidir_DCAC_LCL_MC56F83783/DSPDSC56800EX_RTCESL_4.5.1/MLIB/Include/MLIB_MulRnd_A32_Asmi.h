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
* @brief  Multiply functions with 32-bit accumulator input and result rounding 
* 		  in assembler for V3
* 
*******************************************************************************/
#ifndef _MLIB_MULRND_A32_ASMI_H_
#define _MLIB_MULRND_A32_ASMI_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/

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
* @brief  Multiplication of 16-bit fractional argument and 32-bit accumulator 
* 		  with 16-bit fractional rounded result
*
* @param  ptr			
* 
* @param  in    		acc32_t a32Accum
*						  - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*                       frac16_t f16Mult
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns fractional part of the multiplication of two 
* 			inputs. The function does not saturate the output.
*
*			SATURATION mode must be turned off!
*		    The V3 core instructions used!	
*
****************************************************************************/
extern inline frac16_t MLIB_MulRnd_F16as_FAsmi(register acc32_t a32Accum, register frac16_t f16Mult)
{
	register frac32_t f32Val;
	
	asm(.optimize_iasm on);
		
	asm(move.w f16Mult, f32Val);
		
	asm(mpy32 a32Accum, f32Val, f32Val);	/*  a32Accum = a32Accum * f16Mult (with rounding) */
		
	asm(nop);
	
	asm(asl16 f32Val, f16Mult);				/* Takes fractional part of result */
		
	asm(.optimize_iasm off);
	
	return f16Mult;	
}

/***************************************************************************//*!
*
* @brief  Multiplication of 16-bit fractional argument and 32-bit accumulator 
* 		  with 16-bit fractional saturated rounded result
*
* @param  ptr			
* 
* @param  in    		acc32_t a32Accum
*						  - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*                       frac16_t f16Mult
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns saturated fractional result of multiplication 
* 			of two inputs. 
*
*			SATURATION mode must be turned off!
*		    The V3 core instructions used!	
*
****************************************************************************/
extern inline frac16_t MLIB_MulRndSat_F16as_FAsmi(register acc32_t a32Accum, register frac16_t f16Mult)
{
	register frac32_t f32Val, f32Val1;
	register int16_t i16Clb;
	
	asm(.optimize_iasm on);
		
	asm(move.w f16Mult, f32Val);
		
	asm(mpy32 a32Accum, f32Val, f32Val);	/*  a32Accum = a32Accum * f16Mult (with rounding) */
		
	asm(tfr f32Val,f32Val1);				/* Makes a copy */

	asm(clb f32Val,i16Clb);    				/* Acquire number of leading ones/zeros minus one of f32Val */ 

	asm(asl16 f32Val);						/* Shifts the upper 32 bits 16 times left */	
	
	asm(bfchg #0x8000,f32Val1.1);			/* Changes the MSB of f32Val1 */

	asm(cmp.w #16,i16Clb);      			/* i16Clb - 16 -> comparing the number of leading ones of f32Val and 16  */
		
	asm(tlt	f32Val1,f32Val);				/* If no space to shift, uses the limit value */
		
	asm(move.w f32Val,f16Mult);				/* transfer upper 16 bits with saturation */
		
	asm(.optimize_iasm off);
	
	return f16Mult;	
}

/***************************************************************************//*!
*
* @brief  Multiplication of 16-bit fractional argument and 32-bit accumulator 
* 		  with negative of 16-bit fractional rounded result
*
* @param  ptr			
* 
* @param  in    		acc32_t a32Accum
*						  - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*                       frac16_t f16Mult
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns negative fractional result of the multiplication 
* 			of two inputs. The function does not saturate the output.
*
*			SATURATION mode must be turned off!
*		    The V3 core instructions used!	
*
****************************************************************************/
extern inline frac16_t MLIB_MulNegRnd_F16as_FAsmi(register acc32_t a32Accum, register frac16_t f16Mult)
{
	register frac32_t f32Val;
	
	asm(.optimize_iasm on);
		
	asm(move.w f16Mult, f32Val);
		
	asm(mpy32 a32Accum, f32Val, f32Val);		/*  a32Accum = a32Accum * f16Mult (with rounding) */
		
	asm(nop);
		
	asm(neg f32Val);							/* Negates result */
	
	asm(asl16 f32Val, f16Mult);					/* Takes fractional part of result */
		
	asm(.optimize_iasm off);
	
	return f16Mult;		
}

/***************************************************************************//*!
*
* @brief  Multiplication of 16-bit fractional argument and 32-bit accumulator 
* 		  with negative of 16-bit fractional rounded saturated result
*
* @param  ptr			
* 
* @param  in    		acc32_t a32Accum
*						  - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*                       frac16_t f16Mult
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns negative fractional result of the multiplication 
* 			of two inputs. The function saturates the output.
*
*			SATURATION mode must be turned off!
*		    The V3 core instructions used!	
*
****************************************************************************/
extern inline frac16_t MLIB_MulNegRndSat_F16as_FAsmi(register acc32_t a32Accum, register frac16_t f16Mult)
{
	register frac32_t f32Val, f32Val1;
	register int16_t i16Clb;
	
	asm(.optimize_iasm on);
		
	asm(move.w f16Mult, f32Val);
		
	asm(mpy32 a32Accum, f32Val, f32Val);	/*  a32Accum = a32Accum * f16Mult (with rounding) */
		
	asm(nop);
	
	asm(neg f32Val);						/* Negates result */

	asm(tfr f32Val,f32Val1);				/* Makes a copy */

	asm(clb f32Val,i16Clb);    				/* Acquire number of leading ones/zeros minus one of f32Val */ 

	asm(asl16 f32Val);						/* Shifts the upper 32 bits 16 times left */	
	
	asm(bfchg #0x8000,f32Val1.1);			/* Changes the MSB of f32Val1 */

	asm(cmp.w #16,i16Clb);      			/* i16Clb - 16 -> comparing the number of leading ones of f32Val and 16  */
		
	asm(tlt	f32Val1,f32Val);				/* If no space to shift, uses the limit value */
		
	asm(move.w f32Val,f16Mult);				/* transfer upper 16 bits with saturation */
		
	asm(.optimize_iasm off);
	
	return f16Mult;	
}

/***************************************************************************//*!
*
* @brief  Multiplication of two 32-bit accumulator arguments with 32-bit
* 		  accumulator rounded result
*
* @param  ptr			
* 
* @param  in    		acc32_t a32Mult1
*						  - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*                       acc32_t a32Mult2
*						  - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*
* @return This function returns
*     - acc32_t value [-65536.0;65536.0 - (2^-15)]
*		
* @remarks 	This function returns rounded multiplication of two 32-bit accumulator
* 			inputs. The function does not saturate the output.
*
*			SATURATION mode must be turned off!
*		    The V3 core instructions used!	
*
****************************************************************************/
extern inline acc32_t MLIB_MulRnd_A32_FAsmi(register acc32_t a32Mult1, register acc32_t a32Mult2)
{
	register frac32_t a32Val;
	register frac32_t f32Temp;
	
	asm(.optimize_iasm on);
		
	asm(mpy64 a32Mult1, a32Mult2, a32Val);	/* a32Val = a32Mult1 * a32Mult2 (without rounding) */
		
	asm(nop);
	
	asm(tfr Y,f32Temp);						/* f32Temp = lower 32 bits of the result */
	asm(clr.w f32Temp.2);					/* clears f32Temp.2 */
	asm(rnd f32Temp);						/* rounds f32Temp */
	asm(move.w f32Temp.2,Y1);				/* Y1 = f32Temp.2 (overflow from rounding) */
	asm(move.w f32Temp.1,Y0);				/* Y0 = f32Temp.1 (rounded B) */
	
	asm(asl16 a32Val);						/* Shifts the upper 32 bits 16 times left */
	asm(add Y,a32Val);						/* upper 16 bits from the lower 32-bits are copied to a32Val.0 */
	
	asm(.optimize_iasm off);
	
	return a32Val;	

}

/***************************************************************************//*!
*
* @brief  Multiplication of two 32-bit accumulator arguments with 32-bit
* 		  accumulator rounded saturated result
*
* @param  ptr			
* 
* @param  in    		acc32_t a32Mult1
*						  - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*                       acc32_t a32Mult2
*						  - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*
* @return This function returns
*     - acc32_t value [-65536.0;65536.0 - (2^-15)]
*		
* @remarks 	This function returns rounded multiplication of two 32-bit accumulator
* 			inputs. The function saturates the output.
*
*			SATURATION mode must be turned off!
*		    The V3 core instructions used!	
*
****************************************************************************/
extern inline acc32_t MLIB_MulRndSat_A32_FAsmi(register acc32_t a32Mult1, register acc32_t a32Mult2)
{
	register frac32_t a32Val, a32Val1;
	register int16_t i16Clb;	
	register frac32_t f32Temp;
	
	asm(.optimize_iasm on);
		
	asm(mpy64 a32Mult1, a32Mult2, a32Val);	/* a32Val = a32Mult1 * a32Mult2 (without rounding) */

	asm(nop);

	asm(tfr Y,f32Temp);						/* f32Temp = lower 32 bits of the result */
	asm(clr.w f32Temp.2);					/* clears B2 */
	asm(rnd f32Temp);						/* rounds f32Temp */
	asm(move.w f32Temp.2,Y1);				/* Y1 = f32Temp.2 (overflow from rounding) */
	asm(move.w f32Temp.1,Y0);				/* Y0 = f32Temp.1 (rounded B) */

	asm(tfr a32Val,a32Val1);				/* Makes a copy */

	asm(clb a32Val,i16Clb);    				/* Acquire number of leading ones/zeros minus one of f32Val */ 

	asm(asl16 a32Val);						/* Shifts the upper 32 bits 16 times left */	
	asm(add Y,a32Val);						/* upper 16 bits from the lower 32-bits are copied to a32Val.0 */
	
	asm(bfchg #0x8000,a32Val1.1);			/* Changes the MSB of f32Val1 */

	asm(cmp.w #16,i16Clb);      			/* i16Clb - 16 -> comparing the number of leading ones of f32Val and 16  */
		
	asm(tlt	a32Val1,a32Val);				/* If no space to shift, uses the limit value */
	
	asm(sat a32Val);						/* Saturates the result */
	
	asm(.optimize_iasm off);
	
	return a32Val;	
}

/***************************************************************************//*!
*
* @brief  Multiplication of two 32-bit accumulator arguments with negative 32-bit
* 		  accumulator rounded result
*
* @param  ptr			
* 
* @param  in    		acc32_t a32Mult1
*						  - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*                       acc32_t a32Mult2
*						  - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*
* @return This function returns
*     - acc32_t value [-65536.0;65536.0 - (2^-15)]
*		
* @remarks 	This function returns negative rounded multiplication of two 32-bit
* 			accumulator	inputs. The function does not saturate the output.
*
*			SATURATION mode must be turned off!
*		    The V3 core instructions used!	
*
****************************************************************************/
extern inline acc32_t MLIB_MulNegRnd_A32_FAsmi(register acc32_t a32Mult1, register acc32_t a32Mult2)
{
	register frac32_t a32Val;
	register frac32_t f32Temp;
	
	asm(.optimize_iasm on);
		
	asm(mpy64 a32Mult1, a32Mult2, a32Val);	/* a32Val = a32Mult1 * a32Mult2 (without rounding) */
		
	asm(nop);
	asm(tfr Y,f32Temp);						/* f32Temp = lower 32 bits of the result */
	asm(clr.w f32Temp.2);					/* clears f32Temp.2 */
	asm(rnd f32Temp);						/* rounds f32Temp */
	asm(move.w f32Temp.2,Y1);				/* Y1 = f32Temp.2 (overflow from rounding) */
	asm(move.w f32Temp.1,Y0);				/* Y0 = f32Temp.1 (rounded B) */
	
	asm(asl16 a32Val);						/* Shifts the upper 32 bits 16 times left */

	asm(add Y,a32Val);						/* upper 16 bits from the lower 32-bits are copied to a32Val.0 */

	asm(neg a32Val);						/* Negates the result */
	
	asm(.optimize_iasm off);
	
	return a32Val;	

}

/***************************************************************************//*!
*
* @brief  Multiplication of two 32-bit accumulator arguments 
* 		  with negative 32-bit accumulator saturated result
*
* @param  ptr			
* 
* @param  in    		acc32_t a32Mult1
*						  - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*                       acc32_t a32Mult2
*						  - Argument in [-65536.0;65536.0 - (2^-15)] in acc32_t
*
* @return This function returns
*     - acc32_t value [-65536.0;65536.0 - (2^-15)]
*		
* @remarks 	This function returns negative rounded multiplication of two 32-bit
* 			accumulator	inputs. The function saturates the output.
*
*			SATURATION mode must be turned off!
*		    The V3 core instructions used!	
*
****************************************************************************/
extern inline acc32_t MLIB_MulNegRndSat_A32_FAsmi(register acc32_t a32Mult1, register acc32_t a32Mult2)
{
	register frac32_t a32Val, a32Val1;
	register int16_t i16Clb;	
	register frac32_t f32Temp;
	
	asm(.optimize_iasm on);
		
	asm(mpy64 a32Mult1, a32Mult2, a32Val);	/* a32Val = a32Mult1 * a32Mult2 (without rounding) */
		
	asm(nop);
	asm(tfr Y,f32Temp);						/* f32Temp = lower 32 bits of the result */
	asm(clr.w f32Temp.2);					/* clears f32Temp.2 */
	asm(rnd f32Temp);						/* rounds f32Temp */
	asm(move.w f32Temp.2,Y1);				/* Y1 = f32Temp.2 (overflow from rounding) */
	asm(move.w f32Temp.1,Y0);				/* Y0 = f32Temp.1 (rounded B) */

	asm(tfr a32Val,a32Val1);				/* Makes a copy */

	asm(clb a32Val,i16Clb);    				/* Acquire number of leading ones/zeros minus one of f32Val */ 

	asm(asl16 a32Val);						/* Shifts the upper 32 bits 16 times left */	
	asm(add Y,a32Val);						/* upper 16 bits from the lower 32-bits are copied to a32Val.0 */
	
	asm(bfchg #0x8000,a32Val1.1);			/* Changes the MSB of f32Val1 */

	asm(cmp.w #16,i16Clb);      			/* i16Clb - 16 -> comparing the number of leading ones of f32Val and 16  */
		
	asm(tlt	a32Val1,a32Val);				/* If no space to shift, uses the limit value */
	
	asm(neg a32Val);						/* Negates the result */
	
	asm(sat a32Val);						/* Saturates the result */
	
	asm(.optimize_iasm off);
	
	return a32Val;	
}


#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MULRND_A32_ASMI_H_ */
