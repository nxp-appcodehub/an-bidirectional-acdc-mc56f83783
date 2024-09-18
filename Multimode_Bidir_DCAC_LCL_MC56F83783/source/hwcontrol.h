/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef HWCONTROL_H_
#define HWCONTROL_H_

#include "fsl_device_registers.h"
#include "bidir_dcac_statemachine.h"
#include "pin_mux.h"
#include "fsl_gpio.h"

#pragma inline_max_total_size(30000)
#pragma inline_max_size(3000)
/* PWM submodule definition */
/*enable the counter of the PWM*/
#define PWMA_SM0_RUN()      PWMA->MCTRL |= PWM_MCTRL_RUN(0x1)
#define PWMA_SM0_STOP()     PWMA->MCTRL &= ~PWM_MCTRL_RUN(0x1)

#define PIT0_RUN()      	  PIT0->CTRL |= PIT_CTRL_CNT_EN_MASK
#define PIT0_STOP()      	  PIT0->CTRL &= ~PIT_CTRL_CNT_EN_MASK

/*hardware protection*/
#define ACDC_HW_OVERCUR()             	 (PWMA->FAULT[0].FSTS & (PWM_FSTS_FFLAG(3)|PWM_FSTS_FFPIN(3)))
#define ACDC_CLEAR_HWOVERCUR_FAULT()  	 PWMA->FAULT[0].FSTS = ((PWMA->FAULT[0].FSTS&0xFFF0)|PWM_FSTS_FFLAG(3))

#define ACDC_HW_OVERVOLT()             	 (PWMA->FAULT[0].FSTS & (PWM_FSTS_FFLAG(4)|PWM_FSTS_FFPIN(4))) 
#define ACDC_CLEAR_HWOVERVOLT_FAULT()  	 PWMA->FAULT[0].FSTS = ((PWMA->FAULT[0].FSTS&0xFFF0)|PWM_FSTS_FFLAG(4))
#define ACDC_CLEAR_HW_FAULT()            PWMA->FAULT[0].FSTS |= PWM_FSTS_FFLAG(7)

#define ACDC_FASTBRIDGE_PWM_DIS()     	 PWMA->OUTEN &= ~(PWM_OUTEN_PWMA_EN(1)|PWM_OUTEN_PWMB_EN(1))
#define ACDC_FASTBRIDGE_PWM_EN()      	 PWMA->OUTEN |= (PWM_OUTEN_PWMA_EN(1)|PWM_OUTEN_PWMB_EN(1))

#define ACDC_FASTHIGH_PWM_DIS()     	 PWMA->OUTEN &= ~PWM_OUTEN_PWMA_EN(1)
#define ACDC_FASTHIGH_PWM_EN()      	 PWMA->OUTEN |= PWM_OUTEN_PWMA_EN(1)
#define ACDC_FASTLOW_PWM_DIS()     	 	 PWMA->OUTEN &= ~PWM_OUTEN_PWMB_EN(1)
#define ACDC_FASTLOW_PWM_EN()      	 	 PWMA->OUTEN |= PWM_OUTEN_PWMB_EN(1)

#define ACDC_FASTBRIDGE_PWM_MASK()  	 PWMA->MASK |= PWM_MASK_UPDATE_MASK(1)|(PWM_MASK_MASKA(1)|PWM_MASK_MASKB(1))
#define ACDC_FASTBRIDGE_PWM_NOMASK()  	 PWMA->MASK = (PWMA->MASK  & 0xFEEF)|PWM_MASK_UPDATE_MASK(1)

#define ACDC_FASTHIGH_PWM_MASK()  	 	 PWMA->MASK |= PWM_MASK_MASKA(1)|PWM_MASK_UPDATE_MASK(1)
#define ACDC_FASTHIGH_PWM_NOMASK()  	 PWMA->MASK = (PWMA->MASK & 0xFEFF)|PWM_MASK_UPDATE_MASK(1)
#define ACDC_FASTLOW_PWM_MASK()  	 	 PWMA->MASK |= PWM_MASK_MASKB(1)|PWM_MASK_UPDATE_MASK(1)
#define ACDC_FASTLOW_PWM_NOMASK()  	 	 PWMA->MASK = (PWMA->MASK & 0xFFEF)|PWM_MASK_UPDATE_MASK(1)

#define INVERTOUTPUTPOLARITY     	     PWMA->SM[0].OCTRL |= (PWM_OCTRL_POLA_MASK | PWM_OCTRL_POLB_MASK)	/*Inverse polarity of PWM output in input negative cycle*/ 
#define RESETOUTPUTPOLARITY      	 	 PWMA->SM[0].OCTRL &= ~(PWM_OCTRL_POLA_MASK | PWM_OCTRL_POLB_MASK)	/*Not invert polarity of PWM output in input positive cycle*/

#define ACDC_THIRDBRIDGE_PWM5_DIS()   	 PWMA->OUTEN &= ~PWM_OUTEN_PWMA_EN(4)
#define ACDC_THIRDBRIDGE_PWM5_EN()    	 PWMA->OUTEN |= PWM_OUTEN_PWMA_EN(4)
#define ACDC_THIRDBRIDGE_PWM6_DIS()   	 PWMA->OUTEN &= ~PWM_OUTEN_PWMB_EN(4)
#define ACDC_THIRDBRIDGE_PWM6_EN()    	 PWMA->OUTEN |= PWM_OUTEN_PWMB_EN(4)

#define ACDC_SLOWBRIDGE_PWM_DIS()        GPIO_PinClear(BOARD_PWM3_GPIO,BOARD_PWM3_PIN_MASK); GPIO_PinClear(BOARD_PWM4_GPIO,BOARD_PWM4_PIN_MASK)
#define ACDC_SLOWBRIDGE_HIGHMOS_ON()     GPIO_PinSet(BOARD_PWM3_GPIO,BOARD_PWM3_PIN_MASK)	    	//GE3
#define ACDC_SLOWBRIDGE_HIGHMOS_OFF()    GPIO_PinClear(BOARD_PWM3_GPIO,BOARD_PWM3_PIN_MASK)
#define ACDC_SLOWBRIDGE_LOWMOS_ON()      GPIO_PinSet(BOARD_PWM4_GPIO,BOARD_PWM4_PIN_MASK)			//GE2
#define ACDC_SLOWBRIDGE_LOWMOS_OFF()     GPIO_PinClear(BOARD_PWM4_GPIO,BOARD_PWM4_PIN_MASK)
#define ACDC_SLOWBRIDGE_HIGHMOS_STATE()  GPIO_PinRead(BOARD_PWM3_GPIO,BOARD_PWM3_PIN_MASK)
#define ACDC_SLOWBRIDGE_LOWMOS_STATE()   GPIO_PinRead(BOARD_PWM4_GPIO,BOARD_PWM4_PIN_MASK)

/* switch */
#define CLOSE_SW_GRID()      GPIO_PinSet(BOARD_SW_GRID_GPIO,BOARD_SW_GRID_PIN_MASK)    /* GF3 */
#define OPEN_SW_GRID()       GPIO_PinClear(BOARD_SW_GRID_GPIO,BOARD_SW_GRID_PIN_MASK) 
#define CLOSE_SW_L()         GPIO_PinSet(BOARD_SW_L_GPIO,BOARD_SW_L_PIN_MASK)  		   /* GF6 */
#define OPEN_SW_L()          GPIO_PinClear(BOARD_SW_L_GPIO,BOARD_SW_L_PIN_MASK)
#define CLOSE_SW_N()         GPIO_PinSet(BOARD_SW_N_GPIO,BOARD_SW_N_PIN_MASK)  	       /* GF7 */
#define OPEN_SW_N()          GPIO_PinClear(BOARD_SW_N_GPIO,BOARD_SW_N_PIN_MASK) 

#define CLOSE_SW_LOAD()      GPIO_PinSet(BOARD_SW_L_GPIO,BOARD_SW_L_PIN_MASK);GPIO_PinSet(BOARD_SW_N_GPIO,BOARD_SW_N_PIN_MASK) 	/* GF6 & GF7 */
#define OPEN_SW_LOAD()       GPIO_PinClear(BOARD_SW_L_GPIO,BOARD_SW_L_PIN_MASK);GPIO_PinClear(BOARD_SW_N_GPIO,BOARD_SW_N_PIN_MASK)
#define CLOSE_SW_ALL()       GPIOF->DR |= 0xc8	/* GF3 & GF6 & GF7 */
#define OPEN_SW_ALL()        GPIOF->DR &= ~0xc8 

#define SW_GRID_STATE()      GPIO_PinRead(BOARD_SW_GRID_GPIO,BOARD_SW_GRID_PIN_MASK)

/* user LED */
#define USER_LED1_TOGGLE()   GPIO_PinToggle(BOARD_user_LED1_GPIO,BOARD_user_LED1_PIN_MASK)	/* GC0 */
#define USER_LED2_TOGGLE()   GPIO_PinToggle(BOARD_user_LED2_GPIO,BOARD_user_LED2_PIN_MASK) 	/* GF2 */
#define HVP_LED_TOGGLE()     GPIO_PinToggle(BOARD_LED_HVP_GPIO,BOARD_LED_HVP_PIN_MASK)		/* GC1 */

/* power board test point */
#define TP38_HIGH()      	 GPIO_PinSet(BOARD_TP38_GPIO,BOARD_TP38_PIN_MASK) 	    /* GC13 */
#define TP38_LOW()       	 GPIO_PinClear(BOARD_TP38_GPIO,BOARD_TP38_PIN_MASK)
#define TP40_HIGH()      	 GPIO_PinSet(BOARD_TP40_GPIO,BOARD_TP40_PIN_MASK)       /* GF0 */
#define TP40_LOW()       	 GPIO_PinClear(BOARD_TP40_GPIO,BOARD_TP40_PIN_MASK)	

/* CAN TRANSRECEIVER STB CONTROL */
#define CAN_STB_TOGGLE()   	 GPIO_PinToggle(BOARD_CAN_STB_GPIO,BOARD_CAN_STB_PIN_MASK) 		/* GC15 */
#define CAN_STB_ENABLE()   	 GPIO_PinSet(BOARD_CAN_STB_GPIO,BOARD_CAN_STB_PIN_MASK)	   		/* GC15 */
#define CAN_STB_DISABLE()  	 GPIO_PinClear(BOARD_CAN_STB_GPIO,BOARD_CAN_STB_PIN_MASK)  		/* GC15 */

/* phase sync test */
#define SYNC_TRANSFER()      GPIO_PinToggle(BOARD_phase_transfer_GPIO,BOARD_phase_transfer_PIN_MASK)	  /* GC5 */

inline void INV_PWM_Update()

{
	int16_t w16Val;
	
	gu16SlowBriDTCnt++;
	
	/* configuration for low-frequency bridge IGBT driver signal generation */
	if(gsACDC_Drive.f16Duty>0)  //f16Duty>0, corresponding to the positive half cycle, low side MOS on
	{
		if(ACDC_SLOWBRIDGE_HIGHMOS_STATE())
		{
			ACDC_SLOWBRIDGE_HIGHMOS_OFF();
			gsACDC_Drive.sIGridCtrl.sPIpAWParams.f32IAccK_1 = MLIB_Add_F32(gsACDC_Drive.sIGridCtrl.sPIpAWParams.f32IAccK_1,FRAC32(0.04));
			gsACDC_Drive.f16Duty = 0;
			gu16SlowBriDTCnt = 0;
		}
		else
		{
			if((gu16SlowBriDTCnt>=INV_SLOWBRIDGE_DT_CNT)&&(!ACDC_SLOWBRIDGE_LOWMOS_STATE())) //add a dead band for 2 interrupt cycles 
			{
				ACDC_SLOWBRIDGE_LOWMOS_ON(); 
				asm(nop);asm(nop);asm(nop);asm(nop);asm(nop);
				RESETOUTPUTPOLARITY;
			} 
		}
	}
	else //f16Duty<0, corresponding to the negative half cycle, high side MOS on
	{
		if(ACDC_SLOWBRIDGE_LOWMOS_STATE())
		{
			ACDC_SLOWBRIDGE_LOWMOS_OFF();
			gsACDC_Drive.sIGridCtrl.sPIpAWParams.f32IAccK_1 = MLIB_Sub_F32(gsACDC_Drive.sIGridCtrl.sPIpAWParams.f32IAccK_1,FRAC32(0.04));
			gsACDC_Drive.f16Duty = 0;
			gu16SlowBriDTCnt = 0;
		}
		else
		{
			if((gu16SlowBriDTCnt>=INV_SLOWBRIDGE_DT_CNT)&&(!ACDC_SLOWBRIDGE_HIGHMOS_STATE()))  
			{
				ACDC_SLOWBRIDGE_HIGHMOS_ON();
				asm(nop);asm(nop);asm(nop);asm(nop);asm(nop);
				INVERTOUTPUTPOLARITY;
			}
		}
	}	
	w16Val = MLIB_AbsSat_F16(MLIB_Mul_F16(ACDC_PWM_HALF_PERIOD, gsACDC_Drive.f16Duty));
	
	PWMA->MCTRL |= PWM_MCTRL_CLDOK(1);
	PWMA->SM[0].VAL2 = -w16Val;
	PWMA->SM[0].VAL3 = w16Val;
	PWMA->MCTRL |= PWM_MCTRL_LDOK(1);
}

inline void Sin_Gen()
{	
	gsACDC_Drive.sVacCtrl.sSinGen.f32Angle = MLIB_AddSat_F32(gsACDC_Drive.sVacCtrl.sSinGen.f32Angle, gsACDC_Drive.sVacCtrl.sSinGen.f32AngleStep);
    
	if(gsACDC_Drive.sVacCtrl.sSinGen.f32AnglePrev >= FRAC32(0.999))
	{
		gsACDC_Drive.sVacCtrl.sSinGen.f32Angle = MLIB_AddSat_F32(FRAC32(-1.0), gsACDC_Drive.sVacCtrl.sSinGen.f32AngleStep);
	}		
	
	if(gsACDC_Drive.sVacCtrl.sSinGen.f32Angle >= 0 && gsACDC_Drive.sVacCtrl.sSinGen.f32AnglePrev < 0) //from negative to positive half cycle
	{
		gsACDC_Drive.sVacCtrl.sSinGen.uw16SinCnt = 0;
		/* INV reference frequency update in off-grid to grid-connected sync phase */
		if(gsACDC_Drive.sFlag.GridCheckDone&&(gsACDC_Drive.sPowerMetering.a32VGridFreq<gsACDC_Drive.sFaultThresholds.a32VGridFreqOver)&&(gsACDC_Drive.sPowerMetering.a32VGridFreq>gsACDC_Drive.sFaultThresholds.a32VGridFreqUnder))
		{
			gsACDC_Drive.sVacCtrl.a32VInvRefFreq = MLIB_Add_A32as(gsACDC_Drive.sPowerMetering.a32VGridFreq,gsACDC_Drive.sVacCtrl.f16FreqAdjustment);
			gsACDC_Drive.sVacCtrl.sSinGen.f32AngleStep = MLIB_Mul_F32(gsACDC_Drive.sVacCtrl.a32VInvRefFreq<<10,INV_SINGEN_STEP_COFF); 
			OffGrid_CtrlparamTune();			
		}
	}
	else 
	{
		gsACDC_Drive.sVacCtrl.sSinGen.uw16SinCnt++;
	}
	
	gsACDC_Drive.sVacCtrl.sSinGen.f16Sin = GFLIB_Sin_F16((frac16_t)(gsACDC_Drive.sVacCtrl.sSinGen.f32Angle>>16));
	gsACDC_Drive.sVacCtrl.sSinGen.f32AnglePrev = gsACDC_Drive.sVacCtrl.sSinGen.f32Angle;
}
 
inline void PFC_PWM_UPDATE(uint16_t counter,frac16_t Dutycycle)
{
	frac16_t f16duty_temp;
	
	f16duty_temp = MLIB_Mul_F16(ACDC_PWM_HALF_PERIOD, Dutycycle);	
	
	if((counter < ZERO_CROSS_SOFT_START_PWM_CYCLES) && gsACDC_Drive.sFlag.Ensoftzero)
	{
		f16duty_temp = (counter << 7); /* duty cycle for zero cross soft start, max = 2*128= 256 < 2499 */
	}
	
	if(gsACDC_Drive.sFlag.VGridPol)
	{
		TP38_HIGH();
		/* change high frequency arm switches operation after zero crossing */
		if(counter == 1) ACDC_FASTHIGH_PWM_MASK();				
		else if(counter == 2)  ACDC_FASTLOW_PWM_NOMASK();
		
		PWMA->MCTRL |= PWM_MCTRL_CLDOK(1);
		PWMA->SM[0].VAL2 = f16duty_temp+1;
		PWMA->SM[0].VAL3 = -f16duty_temp;
		PWMA->MCTRL |= PWM_MCTRL_LDOK(1);
	}
	else
	{
		TP38_LOW();
		
		if(counter == 1) ACDC_FASTLOW_PWM_MASK();			
		else if(counter == 2) ACDC_FASTHIGH_PWM_NOMASK();		

		PWMA->MCTRL |= PWM_MCTRL_CLDOK(1);
		PWMA->SM[0].VAL2 = -f16duty_temp;
		PWMA->SM[0].VAL3 = f16duty_temp;
		PWMA->MCTRL |= PWM_MCTRL_LDOK(1);
	}
}

inline void Check_AC_drop(ACDCSTRUC_BI_DIR_T *ptr)
{
	if(!ptr->sFlag.AcDrop)
	{
	    if(ptr->sVacDetect.f16VGridFiltAbs < ptr->f16VacLackTh)
	    {
	    	ptr->u16VacFailCnt++;
	    	
	    	/* if the input voltage sampling value is less than the power-down threshold for several 
	    	 * consecutive times, the input voltage is considered power down. */ 
	        if(ptr->u16VacFailCnt == VAC_FAIL_CONFIRM_NUM)
	        {
	            /* Disable all switches, When Vdc<ACDC_VDC_UNDERVOLT_LIMIT, system will enter Fault mode, 
	        	 * no pre-charge is needed again, so the converter stays in RUN mode to ensure fast recovery. */
	        	ptr->bCloseLoopOnOff = 0;
	        	ACDC_FASTBRIDGE_PWM_MASK();
	    	    ptr->u16VacRecoveryOkCnt = 0;
	    	    
	    	    
	    	    /* disable polarity detection by clearing gu16VacCycleCnt every loop when ac drop,
	    	     * and clear polChangeConfirmCnt status, so no zero crossing will be detected before ac recovery */
	    	    gu16VGridCycleCnt = 0;
	    	    ptr->sVacDetect.u16VGridPolConfirmCnt = 0;
	    	    ptr->sFlag.VGridReadyforMetering = 0;
	    	    ptr->sFlag.ZeroCross = 0;
	    	    
	    	    ptr->sFlag.AcDrop = 1;
	    	    guw32StartCnt = gu32TimerCnt;
		    }
	    }
	    else 
	    {
	    	ptr->u16VacFailCnt = 0;
	    	guw32StartCnt = gu32TimerCnt;
	    }
	}
	else 
	{
		if(ptr->sVacDetect.f16VGridFiltAbs >= ptr->f16VacRecoveryTh)
		{
			ptr->u16VacRecoveryOkCnt++;
			/* if the input voltage sampling value is larger than the ac recovery threshold for several 
			 * consecutive times, the input voltage is considered power up again. */ 
			if(ptr->u16VacRecoveryOkCnt == VAC_RECOVERY_CONFIRM_NUM)
			{
				/* get current polarity for below polarity confirmation */
				ptr->sFlag.VGridPolTemp = (ptr->sVacDetect.f16VGridFilt >= 0)?1:0;
				
				ptr->u16VacFailCnt = 0;
				ptr->sFlag.AcDrop = 0;
			}
		}
		else  
		{
			ptr->u16VacRecoveryOkCnt = 0;
    	    gu16VGridCycleCnt = 0; /* ensure no polarity detection (zero-crossing) when ac drop */
		}
	}
	
	/* after the input voltage is restored, start PFC converter again */
	if(ptr->u16VacRecoveryOkCnt == VAC_RECOVERY_CONFIRM_NUM)
	{
		
		/* confirm current polarity before enabling zero-crossing detection and converter recovery, now the polarity detection in fast is not working. */
		if(ptr->sVacDetect.u16VGridPolConfirmCnt<VAC_POL_CHG_NUM)
		{
		    ptr->sFlag.VGridPol = ptr->sFlag.VGridPolTemp;
		    ptr->sFlag.VGridPolTemp = (ptr->sVacDetect.f16VGridFilt >= 0)?1:0;
		    
		    /* accept the detected polarity as current polarity when several consecutive polarity detection results are the same. */
		    if(ptr->sFlag.VGridPolTemp == ptr->sFlag.VGridPol) 
		    {		
			    ptr->sVacDetect.u16VGridPolConfirmCnt++;
			    if(ptr->sVacDetect.u16VGridPolConfirmCnt >= VAC_POL_CHG_NUM)   
			    {
			    	/* enable fast loop polarity (zero crossing) detection,
			    	 * because of uncertain recovery point, set pol_detect flag to discard the first zero-crossing. */
			    	ptr->sVacDetect.u16VGridPolConfirmCnt = 0;
			    	gsACDC_Drive.sVacDetect.u16ZeroCrossingCnt = 1;
			    	ptr->sFlag.AcFirstCycleDetect = 1;	
			    	gu16VGridCycleCnt = 0;/* reset gu16VacCycleCnt because the switch is enabled when counter==1 */ 
			    	ptr->u16VacRecoveryOkCnt = 0;
			    	
			    	/* converter restart */
			    	ptr->sVdcCtrl.bVacRise = ptr->sFlag.VGridPol?1:0; //avoid triac closing in wrong position during the first cycle
			    	ptr->sFlag.Ensoftzero = 0; /* disable zero-crossing soft-start to avoid current surge cause by high input voltage value when it is restored */ 
                    ptr->sVdcCtrl.sPIpAWParams.f32IAccK_1 = 0; /* reset PI integrator, PFC converter recovery from small duty cycle to avoid current surge */
			    	ptr->sIlCtrl.sPIpAWParams.f32IAccK_1 = 0;
			    	
			    	if(gsACDC_Ctrl.eState==RUN && gsPFC_Runsub!=LIGHTLOAD) ptr->bCloseLoopOnOff = 1;
			    	
			    }
		    }
		    else  ptr->sVacDetect.u16VGridPolConfirmCnt = 0;
		}
	}	
}



inline long Div_int_ll(long register l_numerator, long register l_denominator)
{
                long register w32ClbDenom;
                short register w16LeadBits; 
                // A is l_numerator, B is l_denominator
                
                asm(.optimize_iasm on);
                asm( tfr l_numerator, A);
                asm( tfr l_denominator, B);
                
                asm( tfr B, Y);                  // w32ClbDenom is for normalized denominator
                asm( clb Y, w16LeadBits);        // count leading bits of denominator
                asm( asll.l w16LeadBits, Y);     // normalize the denominator
                asm( sub Y, A);                  // l_numerator - w32ClbDenom
                asm( bftstl #$8, A2); // set Carry if positive
                // divide loop
                asm( rep w16LeadBits);
                asm( div Y, A);
                asm( rol.l A);      // now low "w16LeadBits+1" bits of A contains the quotient
                // mask
                asm( eor.l Y, Y); // clear w32ClbDenom
                asm( add.l #2, Y);
                asm( asll.l w16LeadBits,Y);
                asm( dec.l Y); // w32ClbDenom contains the mask
                asm( and.l Y, A); // apply the mask
                asm( tfr A, l_numerator);
                asm(.optimize_iasm off);
                return l_numerator;
}

#endif /* HWCONTROL_H_ */
