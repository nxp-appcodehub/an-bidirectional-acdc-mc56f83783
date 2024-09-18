
/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2023-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "peripherals.h"
#include "bidir_dcacctrl.h"
#include "hwcontrol.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
uint32_t gu32TimerCnt = 0,gu32IOTogTimeCnt = 0;
uint16_t gu16ExeTimeCnt;
uint16_t gu16VGridCycleCnt = 0,gu16VGridCycleCntSav = 0,gu16VInvCycleCnt = 0,gu16VInvCycleCntSav = 0,gu16SlowBriDTCnt = 0,gu16SoftStartStepCnt=0; 
frac32_t rmsmul;
/*temperature lookup table
 * temp(degree): 0     10   20   30   40   50  60  70  80  90 100 110 120
 *               |     |    |    |    |    |   |   |   |   |   |   |   |
 * ADC RSLT>>3: 3135 2726 2275 1827 1423 1084 815 609 456 342 259 197 152
 */
uint16_t u16TempTable[13] = {3135,2726,2275,1827,1423,1084,815,609,456,342,259,197,152};
 
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void ACDC_PowerMetering();
void Zerocross_reconfig();
void Temp_Sensing(uint16_t u16TempADValue);
void GridCon_CtrlparamTune();
void OffGrid_CtrlparamTune();
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Main function
 */
int main(void)
{
	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
     
    /* Add user initialization code */ 
    ACDC_CLEAR_HW_FAULT();  /* clear hardware fault flag */  
    /* initialize voltage/current offset MA filter */
    gsACDC_Drive.sVacDetect.sVGridOffsetFilter.u16Sh = ACDC_SAMP_OFFSET_MA_WINDOW;
    gsACDC_Drive.sVacDetect.sVGridOffsetFilter.a32Acc = 0;
    gsACDC_Drive.sVacCtrl.sVInvOffsetFilter.u16Sh = ACDC_SAMP_OFFSET_MA_WINDOW;
    gsACDC_Drive.sVacCtrl.sVInvOffsetFilter.a32Acc = 0;
    gsACDC_Drive.sIlCtrl.sIlOffsetFilter.u16Sh = ACDC_SAMP_OFFSET_MA_WINDOW;
    gsACDC_Drive.sIlCtrl.sIlOffsetFilter.a32Acc = 0;
    gsACDC_Drive.sIGridCtrl.sIGridOffsetFilter.u16Sh = ACDC_SAMP_OFFSET_MA_WINDOW;
    gsACDC_Drive.sIGridCtrl.sIGridOffsetFilter.a32Acc = 0;
    gsACDC_Drive.sCurDetect.sICapOffsetFilter.u16Sh = ACDC_SAMP_OFFSET_MA_WINDOW;
    gsACDC_Drive.sCurDetect.sICapOffsetFilter.a32Acc = 0;  
    guw32StartCnt = 0;  /* start counter for offset sampling */
    
    gsACDC_Drive.sFlag.VGridMeteringDone = 1; /* avoid unexpected power metering calculation at startup */
    gsACDC_Drive.sFlag.VInvMeteringDone = 1;
    gsACDC_Drive.f16TempSamp = FRAC16(0.2);  /* avoid false over-temp protection before sampling */
    
    __EI(0);
    /* Enable the counter of PWMA */
    PWMA_SM0_RUN();
     
    while(1)
    {   	
    	/*==================== work mode change, restart from INIT state =======================
    	* make sure the DC source is removed before the converter start to run in AC_TO_DC mode */
    	if((gsACDC_Drive.u16WorkModeCmd!=gsACDC_Drive.u16WorkModeUsed)&&(gsACDC_Drive.u16WorkModeUsed!=0)\
    	    &&((gsACDC_Drive.u16WorkModeCmd == AC_TO_DC)||(gsACDC_Drive.u16WorkModeCmd == DC_TO_AC)))
    	{    		
    	    gsACDC_Drive.bCloseLoopOnOff = 0;
    	    ACDC_FASTBRIDGE_PWM_DIS();
    	    ACDC_SLOWBRIDGE_PWM_DIS();
    	    ACDC_FASTBRIDGE_PWM_NOMASK();
    	    OPEN_SW_ALL();
    	    PIT0_STOP();
    	    gsACDC_Drive.sFlag.VarInitReady = 0;
    	    gsACDC_Ctrl.eState = INIT;
    	}
    	
    	Temp_Sensing(gsACDC_Drive.f16TempSamp);
    	
    	FMSTR_Poll();
    	
    	if(gsACDC_Drive.u16WorkModeUsed == DC_TO_AC)  /* make sure to check whether the grid is ok before grid-connected inverter work */
    	{
    		if((gsACDC_Drive.sVacDetect.u16ZeroCrossingCnt!=0)&&(gsACDC_Drive.u16CurrentInvMode != GRIDCONNECTED_INV)&&(gsACDC_Drive.u16InvModeCmd != GRIDCONNECTED_INV))
    		{
    			gsACDC_Drive.sFlag.GridCheckDone = 0;
    			gsACDC_Drive.sFlag.GridOK = 0;
    			gsACDC_Drive.sVacDetect.u16ZeroCrossingCnt = 0;
    			gsACDC_Drive.sFlag.AcFirstCycleDetect = 1;
    			gsACDC_Drive.sFlag.VGridReadyforMetering = 0;
    			gsACDC_Drive.sFlag.VGridMeteringDone = 1;
    		}
    	}
    	    	
    	/*========================= power metering calculation, once every half cycle ==========================*/
    	if(!gsACDC_Drive.sFlag.VInvMeteringDone) //power metering on off-grid inverter mode
    	{
    		gsACDC_Drive.sPowerMetering.f16VInvRms = GFLIB_Sqrt_F16((frac16_t)(Div_int_ll(gsACDC_Drive.sPowerMetering.a32VInvSqrSumSav,gu16VInvCycleCntSav)));
    		gsACDC_Drive.sPowerMetering.f16IacRms = GFLIB_Sqrt_F16((frac16_t)(Div_int_ll(gsACDC_Drive.sPowerMetering.a32IacSqrSumSav,gu16VInvCycleCntSav)));
    		gsACDC_Drive.sPowerMetering.a32VASum = MLIB_Add_A32as(gsACDC_Drive.sPowerMetering.a32VASum,MLIB_Mul_F16(gsACDC_Drive.sPowerMetering.f16VInvRms,gsACDC_Drive.sPowerMetering.f16IacRms));   
    		gsACDC_Drive.sPowerMetering.a32PSum = MLIB_Add_A32as(gsACDC_Drive.sPowerMetering.a32PSum,Div_int_ll(gsACDC_Drive.sPowerMetering.a32VIinstSumSav,gu16VInvCycleCntSav));    	   
    		gsACDC_Drive.sPowerMetering.a32VInvFreq = Div_int_ll(ACC32(INV_CTRL_FREQ),gu16VInvCycleCntSav)>>1;
    		gsACDC_Drive.sPowerMetering.u16PowerUpdateCnt++; 
    		gsACDC_Drive.sFlag.VInvMeteringDone = 1;
#if !BOARD_TEST
    		gsACDC_Drive.sFlag.VacFaultEn = 1; 
#endif
    	}
    	
    	if((!gsACDC_Drive.sFlag.VGridMeteringDone)&&gsACDC_Drive.sFlag.VGridReadyforMetering)
    	{
    		gsACDC_Drive.sPowerMetering.f16VGridRmsSqr = (frac16_t)(Div_int_ll(gsACDC_Drive.sPowerMetering.a32VGridSqrSumSav,gu16VGridCycleCntSav));
    		gsACDC_Drive.sPowerMetering.f16VGridRms = GFLIB_Sqrt_F16(gsACDC_Drive.sPowerMetering.f16VGridRmsSqr);
    		gsACDC_Drive.sPowerMetering.a32VGridFreq = Div_int_ll(ACC32(INV_CTRL_FREQ),gu16VGridCycleCntSav)>>1;
    		
    		if(((gsACDC_Drive.sPowerMetering.a32VGridFreq>ACDC_VAC_UNDERFREQ_LIMIT)&&(gsACDC_Drive.sPowerMetering.a32VGridFreq<ACDC_VAC_OVERFREQ_LIMIT)\
    		    &&gsACDC_Drive.sPowerMetering.f16VGridRms>ACDC_VAC_RMS_LOW_LIMIT)&&(gsACDC_Drive.sPowerMetering.f16VGridRms<ACDC_VAC_RMS_UP_LIMIT))
    		{
    		    gsACDC_Drive.sPowerMetering.a32VGridFreqSum = MLIB_Add_F32(gsACDC_Drive.sPowerMetering.a32VGridFreqSum,gsACDC_Drive.sPowerMetering.a32VGridFreq);
    		    gsACDC_Drive.sPowerMetering.u16GridFreqUpdateCnt++;
    		    if(gsACDC_Drive.sPowerMetering.u16GridFreqUpdateCnt>49) 
    		    {
    		    	gsACDC_Drive.sPowerMetering.a32VGridFreqAvg = MLIB_Mul_F32(gsACDC_Drive.sPowerMetering.a32VGridFreqSum,FRAC32(0.02));
    		    	gsACDC_Drive.sPowerMetering.a32VGridFreqSum = 0;
    		    	gsACDC_Drive.sPowerMetering.u16GridFreqUpdateCnt = 0;
    		    }
    		    	        
    		    if(!gsACDC_Drive.sFlag.GridOK)
    		    {
    		    	gsACDC_Drive.sVacDetect.u16ZeroCrossingCnt++;
    		    	if((gsACDC_Drive.sVacDetect.u16ZeroCrossingCnt >= INPUT_VRMS_DET_NUM)&&(gsACDC_Drive.sFlag.VGridPol))
    		    	{	
    		    	    SPLL_1PH_SOGI_PDCoeffConfig(&spll_obj,gsACDC_Drive.sPowerMetering.a32VGridFreq,ACC32(SPLL_CTRL_FREQ/2),ACC32(SOGI_K_COEFF));   		
    		    	    gsACDC_Drive.sFlag.GridOK = 1;
    		    	}
    		    }
    		    else if(!gsACDC_Drive.sFlag.GridCheckDone) /* automatic grid check, this must be done before AC_TO_DC and grid-connected inverter mode */
    		    {
    		    	if((gsACDC_Drive.sPowerMetering.a32VGridFreqAvg>ACDC_VAC_UNDERFREQ_LIMIT)&&(gsACDC_Drive.sPowerMetering.a32VGridFreqAvg<ACDC_VAC_OVERFREQ_LIMIT))
    		    	{    	        
    		    	    if(gsACDC_Drive.u16WorkModeUsed == DC_TO_AC)
    		    	    {
    		    		    if(gsACDC_Drive.sPowerMetering.f16VGridRms>FRAC16(160.0/VAC_SCALE))  		
    		    	        {
    		    	            gsACDC_Drive.sVacDetect.f16VacrmsNominal= FRAC16(220.0/VAC_SCALE);
    		    	            gsACDC_Drive.sVacDetect.a32VacFreqNominal = ACC32(50.0);
    		    	        }
    		    	        else 
    		    	        {
    		    	            gsACDC_Drive.sVacDetect.f16VacrmsNominal= FRAC16(110.0/VAC_SCALE);
    		    	            gsACDC_Drive.sVacDetect.a32VacFreqNominal = ACC32(60.0);
    		    	        }
    		    	       		    	    
    		    	        // update related value according to latest grid parameters 
    		    	        gsACDC_Drive.sVacCtrl.f16VacRefAmpReq = MLIB_Mul_F16as(VAC_MAX_COEFF,MLIB_Add_F16(gsACDC_Drive.sPowerMetering.f16VGridRms,FRAC16(6.0/808)));
    		    	        if(gsACDC_Ctrl.eState==STOP)  gsACDC_Drive.sVacCtrl.sSinGen.f32AngleStep = MLIB_Mul_F32(gsACDC_Drive.sPowerMetering.a32VGridFreqAvg<<10,INV_SINGEN_STEP_COFF);
    		    	        gsACDC_Drive.sFaultThresholds.f16VacRMSOver = MLIB_Add_F16(gsACDC_Drive.sVacDetect.f16VacrmsNominal, FRAC16(40.0/VAC_SCALE));
    		    	        gsACDC_Drive.sFaultThresholds.f16VacRMSUnder = MLIB_Sub_F16(gsACDC_Drive.sVacDetect.f16VacrmsNominal, FRAC16(30.0/VAC_SCALE));
    		    	        gsACDC_Drive.sFaultThresholds.a32VGridFreqOver = MLIB_Add_F32(gsACDC_Drive.sVacDetect.a32VacFreqNominal, ACC32(3.0));
    		    	        gsACDC_Drive.sFaultThresholds.a32VGridFreqUnder = MLIB_Sub_F32(gsACDC_Drive.sVacDetect.a32VacFreqNominal, ACC32(3.0));
    		    	    }
    		    	    GridCon_CtrlparamTune();    		    	    		    	   
    		    	    gsACDC_Drive.sFlag.GridCheckDone = 1;   	        	    	        
    		    	}
    		    }
    		    gsACDC_Drive.sVacDetect.u16VGridWrongCnt = 0;
    		}
    		else // restart vgrid check when consecutive tests fail in grid check phase, this won't happen when converter starts working because the fault protection logic will respond first    
    		{
    		    gsACDC_Drive.sVacDetect.u16VGridWrongCnt++;
    		    if(gsACDC_Drive.sVacDetect.u16VGridWrongCnt>=2) 
    		    {
    		    	gsACDC_Drive.sFlag.GridCheckDone = 0;
    		    	gsACDC_Drive.sFlag.GridOK = 0;
    		    	gsACDC_Drive.sVacDetect.u16ZeroCrossingCnt = 0;
    		    	gsACDC_Drive.sFlag.AcFirstCycleDetect = 1;
    		    	gsACDC_Drive.sFlag.VGridReadyforMetering = 0;
    		    }
    		}
    		
    		if((gsACDC_Drive.u16WorkModeUsed == AC_TO_DC) || (gsACDC_Drive.u16CurrentInvMode == GRIDCONNECTED_INV))
    		{
    		    gsACDC_Drive.sPowerMetering.f16IacRms = GFLIB_Sqrt_F16((frac16_t)(Div_int_ll(gsACDC_Drive.sPowerMetering.a32IacSqrSumSav,gu16VGridCycleCntSav)));
    		    gsACDC_Drive.sPowerMetering.a32VASum = MLIB_Add_A32as(gsACDC_Drive.sPowerMetering.a32VASum,MLIB_Mul_F16(gsACDC_Drive.sPowerMetering.f16VGridRms,gsACDC_Drive.sPowerMetering.f16IacRms));      
    		    gsACDC_Drive.sPowerMetering.a32PSum = MLIB_Add_A32as(gsACDC_Drive.sPowerMetering.a32PSum,Div_int_ll(gsACDC_Drive.sPowerMetering.a32VIinstSumSav,gu16VGridCycleCntSav));  
    		    gsACDC_Drive.sPowerMetering.u16PowerUpdateCnt++; 
    		} 
    		gsACDC_Drive.sFlag.VGridMeteringDone = 1; 
    	}
    	
    	if(gsACDC_Drive.sPowerMetering.u16PowerUpdateCnt>99) /* power data update */
    	{
    		gsACDC_Drive.sPowerMetering.f16ActivePower = MLIB_Mul_F16as(gsACDC_Drive.sPowerMetering.a32PSum,FRAC16(0.01));
    		gsACDC_Drive.sPowerMetering.f16ApparentPower = MLIB_Mul_F16as(gsACDC_Drive.sPowerMetering.a32VASum,FRAC16(0.01));   
    		gsACDC_Drive.sPowerMetering.f16PowerFactor = MLIB_DivSat_F16(gsACDC_Drive.sPowerMetering.f16ActivePower,gsACDC_Drive.sPowerMetering.f16ApparentPower); 
    		gsACDC_Drive.sPowerMetering.a32VASum = 0;
    		gsACDC_Drive.sPowerMetering.a32PSum = 0;
    		gsACDC_Drive.sPowerMetering.u16PowerUpdateCnt = 0;
    	}
		 
    	/* update coefficient according to the real-time grid frequency in grid-connected mode*/
		if(gsACDC_Drive.sFlag.GridCheckDone&&(MLIB_Abs_F32(MLIB_Sub_F32(gsACDC_Drive.sPowerMetering.a32VGridFreqAvg,spll_obj.a32NominalFreq))>ACC32(0.05)))
		{
		    GridCon_CtrlparamTune();
		}
		
		ACDC_FaultDetection();
    }   
}

#pragma section CODES_IN_RAM begin
void Zerocross_reconfig()
{		
	if(gsACDC_Drive.sFlag.AcFirstCycleDetect) 
	{
		/* discard the first 2 zero-crossing to avoid wrong RMS value */
		gsACDC_Drive.sVacDetect.u16ZeroCrossingCnt++;
		if(gsACDC_Drive.sVacDetect.u16ZeroCrossingCnt>=2)
		{
			gsACDC_Drive.sFlag.AcFirstCycleDetect = 0;
			gsACDC_Drive.sVacDetect.u16ZeroCrossingCnt=0;
		}	
	}
	
	if(gsACDC_Drive.sFlag.SampOffsetReady) /* start power metering after the offset detection is completed  */
	{
		if(!gsACDC_Drive.sFlag.VGridReadyforMetering) /* reset metering value for each time VGridReadyforMetering is set */
		{
		    gsACDC_Drive.sFlag.VGridReadyforMetering = 1;
			gsACDC_Drive.sFlag.VGridZeroCrossforMeter = 0;
			gu16VGridCycleCnt = 0;
			gsACDC_Drive.sFlag.AcFirstCycleDetect = 0;
			gsACDC_Drive.sPowerMetering.a32VGridSqrSum = 0;
			gsACDC_Drive.sPowerMetering.a32VGridFreqSum = 0;
			gsACDC_Drive.sPowerMetering.u16GridFreqUpdateCnt = 0;
			if(gsACDC_Drive.u16WorkModeUsed == AC_TO_DC)
			{
			    gsACDC_Drive.sPowerMetering.a32VIinstSum = 0;
			    gsACDC_Drive.sPowerMetering.a32IacSqrSum = 0;
			    /* zero-crossing soft-start is disabled at startup to avoid current pulse, because the voltage starting point is unknown  */
			    gsACDC_Drive.sFlag.Ensoftzero = 1;
			}
			else
			{
				gsACDC_Drive.sPowerMetering.a32VGridFreqAvg = 0;
			}
		}
		else if(gsACDC_Drive.u16WorkModeUsed == DC_TO_AC)
		{
			/* off-grid to grid-connected switch sync process */
			if((gsACDC_Drive.u16CurrentInvMode==OFFGRID_INV)&&(gsACDC_Drive.sFlag.GridCheckDone)&&(gsACDC_Drive.sFlag.VGridPol==1))
			{
				if(gsACDC_Drive.sVacCtrl.sSinGen.f32Angle>FRAC32(0.95)&&gsACDC_Drive.sVacCtrl.f16ViVgPhaseDif>0)  gsACDC_Drive.sVacCtrl.f16ViVgPhaseDif = FRAC16(1.0);
				else if(gsACDC_Drive.sVacCtrl.sSinGen.f32Angle<FRAC32(-0.95)&&gsACDC_Drive.sVacCtrl.f16ViVgPhaseDif<0)  gsACDC_Drive.sVacCtrl.f16ViVgPhaseDif = FRAC16(-1.0);
				else gsACDC_Drive.sVacCtrl.f16ViVgPhaseDif = MLIB_Neg_F16((frac16_t)(gsACDC_Drive.sVacCtrl.sSinGen.f32Angle>>16));
				gsACDC_Drive.sVacCtrl.f16FreqAdjustment = GFLIB_CtrlPIpAW_F16(gsACDC_Drive.sVacCtrl.f16ViVgPhaseDif, &gsACDC_Drive.sVacCtrl.bSyncPIStopIntegFlag ,&gsACDC_Drive.sVacCtrl.sSyncPIpAWParams);
					    
				if((MLIB_Abs_F16(gsACDC_Drive.sVacCtrl.sSinGen.f32Angle)<FRAC32(0.02))&&(MLIB_Abs_F16(gsACDC_Drive.sVacCtrl.f16VInvFilt)<FRAC16(0.018)))
				{
					CLOSE_SW_ALL();
					PRCtrlInit(&gsACDC_Drive.sVacCtrl.sVacFundamental2p2zParams);
					PRCtrlInit(&gsACDC_Drive.sVacCtrl.sVac3rdHarmonic2p2zParams);
					PRCtrlInit(&gsACDC_Drive.sVacCtrl.sVac5thHarmonic2p2zParams);
					PRCtrlInit(&gsACDC_Drive.sVacCtrl.sVac7thHarmonic2p2zParams);
					gsACDC_Drive.sIlCtrl.sPIpAWParams.f16InErrK_1 = 0;
					gsACDC_Drive.sIlCtrl.sPIpAWParams.f32IAccK_1 = 0;
					gsACDC_Drive.sFlag.VInvReadyforMetering = 0;
					PWMA->FAULT[0].FCTRL &= ~PWM_FCTRL_FAUTO(3);    /* disable automatic fault clearing in grid-connected mode */
					gsACDC_Drive.u16CurrentInvMode = GRIDCONNECTED_INV;
				}					    
			}			
		}
	}
	
	gu16TriacOnCnt = 0;
	gsACDC_Drive.sIlCtrl.sPIpAWParams.f32IAccK_1=0; /* suppress zero-crossing current spikes */	
}

/* keFlexPWMA_CMP1_VECTORn interrupt handler, 20kHz */
#pragma interrupt alignsp saveall
void Fast_Ctrl_ISR(void) 
{
	/* execution time measurement start*/
	TP40_HIGH();  
	TMRA->CHANNEL[0].CNTR = 0;	
	
	gu32TimerCnt++;     /* time base count */
	gu16VGridCycleCnt++;  /* VAC cycle count */
	gu16VInvCycleCnt++;
	gu32IOTogTimeCnt++;

	/*====================== Read input voltage and current sampling result ====================*/	
	/* read ctrl needed ADC results after the sampling ends */
	while(!(ADC->RDY & 0x4)) {;} 		
	gsACDC_Drive.sVacCtrl.f16VInv = ADC->RSLT[9];  /*read voltage related sample result */
	gsACDC_Drive.sVacDetect.f16VGrid = ADC->RSLT[2];//MLIB_Mul_F16(gsACDC_Drive.sVacCtrl.sSinGen.f16Sin,FRAC16(0.1925)); for test //
	gsACDC_Drive.sVdcCtrl.f16VdcBus = ADC->RSLT[10];
	
	gsACDC_Drive.sIlCtrl.f16Il = ADC->RSLT[0];  /*read current related sample result */
	gsACDC_Drive.sIGridCtrl.f16IGrid = ADC->RSLT[8];
	gsACDC_Drive.sCurDetect.f16ICap = ADC->RSLT[1];
			
	/*===================================== sampling result processing, polarity detection and phase generation =================================*/
	/*  begin to detect the zero-crossing point in last quarter cycle, 
	 * in AC-DC mode, start detecting immediately until the AC voltage first cycle flag is cleared to ensure the correctness of initial polarity and calculation */		
	if((gsACDC_Drive.u16WorkModeUsed == AC_TO_DC)||((gsACDC_Drive.u16WorkModeUsed == DC_TO_AC)&&((gsACDC_Drive.u16CurrentInvMode == GRIDCONNECTED_INV)||(gsACDC_Drive.u16InvModeCmd == GRIDCONNECTED_INV))))
	{
		gsACDC_Drive.sVacDetect.f16VGridFilt = GDFLIB_FilterIIR1_F16(gsACDC_Drive.sVacDetect.f16VGrid, &gsACDC_Drive.sVacDetect.sVGridFilter);	
		gsACDC_Drive.sVacDetect.f16VGridFiltAbs = MLIB_Abs_F16(gsACDC_Drive.sVacDetect.f16VGridFilt);
		
		if((gu16VGridCycleCnt > VAC_POL_START_DET_NUM)||(gsACDC_Drive.sFlag.AcFirstCycleDetect))    
		{
			if(gsACDC_Drive.sVacDetect.f16VGridFilt > 0) gsACDC_Drive.sFlag.VGridPolTemp = 1;
		    else if(gsACDC_Drive.sVacDetect.f16VGridFilt < 0) gsACDC_Drive.sFlag.VGridPolTemp = 0;
		    if(gsACDC_Drive.sFlag.VGridPolTemp != gsACDC_Drive.sFlag.VGridPol)
		    {
		    	gsACDC_Drive.sVacDetect.u16VGridPolConfirmCnt++;
		    	if(gsACDC_Drive.sVacDetect.u16VGridPolConfirmCnt >= VAC_POL_CHG_NUM)
		    	{
		    	    gsACDC_Drive.sFlag.VGridPol = gsACDC_Drive.sFlag.VGridPolTemp;
		    		gsACDC_Drive.sFlag.VGridZeroCrossforMeter = 1;  		    
		    		gsACDC_Drive.sFlag.ZeroCross = 1;
		    		gsACDC_Drive.sVacDetect.u16VGridPolConfirmCnt = 0;	
		    		Zerocross_reconfig();    
		    	}
		    }
		    else gsACDC_Drive.sVacDetect.u16VGridPolConfirmCnt = 0;
		}
		
		if(gsACDC_Drive.sFlag.GridOK)
		{		
			SPLL_1PH_SOGI_Run(&spll_obj, MLIB_Conv_F32s(gsACDC_Drive.sVacDetect.f16VGrid)); /* start spll immediately when vgrid is ok */
		}
	}
	
	if(gsACDC_Drive.u16CurrentInvMode == OFFGRID_INV)
	{
		gsACDC_Drive.sVacCtrl.f16VInvFilt = GDFLIB_FilterIIR1_F16(gsACDC_Drive.sVacCtrl.f16VInv, &gsACDC_Drive.sVacCtrl.sVInvFilter);
		
		if(gu16VInvCycleCnt > VAC_POL_START_DET_NUM)    
		{
			if(gsACDC_Drive.sVacCtrl.f16VInvFilt > 0) gsACDC_Drive.sFlag.VInvPolTemp = 1;
			else if(gsACDC_Drive.sVacCtrl.f16VInvFilt < 0) gsACDC_Drive.sFlag.VInvPolTemp = 0;
			if(gsACDC_Drive.sFlag.VInvPolTemp != gsACDC_Drive.sFlag.VInvPol)
			{
				gsACDC_Drive.sVacDetect.u16VInvPolConfirmCnt++;
				if(gsACDC_Drive.sVacDetect.u16VInvPolConfirmCnt >= VAC_POL_CHG_NUM)
				{
				    gsACDC_Drive.sFlag.VInvPol = gsACDC_Drive.sFlag.VInvPolTemp;
				    gsACDC_Drive.sFlag.VInvZeroCrossforMeter = 1;   		    
				    gsACDC_Drive.sVacDetect.u16VInvPolConfirmCnt = 0;	

				    /* just for test, non-linear loads without snubber resistors, gsACDC_Drive.sFlag.VacFaultEn should not be set to 1 in background function */
				    /* if((!gsACDC_Drive.sFlag.VacFaultEn)&&(gsACDC_Drive.sPowerMetering.f16IlRms>=INV_I_LOADON))
				    {
				    	if(gsACDC_Drive.sVacCtrl.uw16PeriodNumCnt>=5)
				    	{
				    		gsACDC_Drive.sFlag.VacFaultEn=1;
				    		gsACDC_Drive.sVacCtrl.uw16PeriodNumCnt=0;
				    	}
				    		else  gsACDC_Drive.sVacCtrl.uw16PeriodNumCnt++;
				    }*/
				}				
			}
			else gsACDC_Drive.sVacDetect.u16VInvPolConfirmCnt = 0;
		}
		
		Sin_Gen();
	}
	
	gsACDC_Drive.sIGridCtrl.f16IGridFilt = GDFLIB_FilterIIR1_F16(gsACDC_Drive.sIGridCtrl.f16IGrid, &gsACDC_Drive.sIGridCtrl.sIGridFilter);
	gsACDC_Drive.sIlCtrl.f16IlFilt = GDFLIB_FilterIIR1_F16(gsACDC_Drive.sIlCtrl.f16Il, &gsACDC_Drive.sIlCtrl.sIlFilter);
	gsACDC_Drive.sVdcCtrl.f16VdcBusFilt = GDFLIB_FilterIIR1_F16(gsACDC_Drive.sVdcCtrl.f16VdcBus, &gsACDC_Drive.sVdcCtrl.sVdcBusFilter);
	
	/*=========================================  get offset =======================================*/
	if(!gsACDC_Drive.sFlag.SampOffsetReady)
	{
	    gsACDC_Drive.sIGridCtrl.f16IGridOffsetDeviation = GDFLIB_FilterMA_F16(gsACDC_Drive.sIGridCtrl.f16IGrid, &gsACDC_Drive.sIGridCtrl.sIGridOffsetFilter);
	    gsACDC_Drive.sCurDetect.f16ICapOffsetDeviation = GDFLIB_FilterMA_F16(gsACDC_Drive.sCurDetect.f16ICap, &gsACDC_Drive.sCurDetect.sICapOffsetFilter);
	    gsACDC_Drive.sVacCtrl.f16VInvOffsetDeviation = GDFLIB_FilterMA_F16(gsACDC_Drive.sVacCtrl.f16VInv, &gsACDC_Drive.sVacCtrl.sVInvOffsetFilter);
	    gsACDC_Drive.sIlCtrl.f16IlOffsetDeviation = GDFLIB_FilterMA_F16(gsACDC_Drive.sIlCtrl.f16Il, &gsACDC_Drive.sIlCtrl.sIlOffsetFilter);
	        
	    if(ACDC_TimeDelay(guw32StartCnt, SAMPOFFSET_CALIB_DURATION))
	    {		
	      ADC->OFFST[0] = MLIB_Add_F16(ADC->OFFST[0],gsACDC_Drive.sIlCtrl.f16IlOffsetDeviation);
	      ADC->OFFST[1] = MLIB_Add_F16(ADC->OFFST[1],gsACDC_Drive.sCurDetect.f16ICapOffsetDeviation);
	      ADC->OFFST[8] = MLIB_Add_F16(ADC->OFFST[8],gsACDC_Drive.sIGridCtrl.f16IGridOffsetDeviation);
	      ADC->OFFST[9] = MLIB_Add_F16(ADC->OFFST[9],gsACDC_Drive.sVacCtrl.f16VInvOffsetDeviation);
	      gsACDC_Drive.sFlag.SampOffsetReady = 1;
	    }				
	}    
		
    /*===================================== DC to AC mode control =================================*/
	if(gsACDC_Drive.u16WorkModeUsed == DC_TO_AC)	   				 
	{		
		if(gsACDC_Drive.bCloseLoopOnOff) /* run these controller code in RUN state */
		{
			if(gsACDC_Drive.u16CurrentInvMode == GRIDCONNECTED_INV)
			{
				/* grid-connected to off-grid switch logic */
				if((gsACDC_Drive.u16InvModeCmd == OFFGRID_INV)&&(!SW_GRID_STATE()))
				{
					if(gu16VGridCycleCnt>190) gsACDC_Drive.sFlag.ReadyforINVModeChange=1;
					if((MLIB_Abs_F16(gsACDC_Drive.sIGridCtrl.f16IGridFilt)<FRAC16(0.002))&&gsACDC_Drive.sFlag.ReadyforINVModeChange==1)
					{
						OPEN_SW_ALL();  //because of hardware risk, all triacs are turned off, no load can be added through load port
						PRCtrlInit(&gsACDC_Drive.sIGridCtrl.sIGridFundamental2p2zParams); 
						PRCtrlInit(&gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonic2p2zParams);
						PRCtrlInit(&gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonic2p2zParams);
						PRCtrlInit(&gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonic2p2zParams);
						gsACDC_Drive.sIGridCtrl.sPIpAWParams.f16InErrK_1 = 0;
						gsACDC_Drive.sIGridCtrl.sPIpAWParams.f32IAccK_1 = 0;
						gsACDC_Drive.sVacDetect.u16ZeroCrossingCnt = 0;
						gsACDC_Drive.sFlag.GridOK = 0;
						gsACDC_Drive.sFlag.GridCheckDone =0;
						gsACDC_Drive.sFlag.AcFirstCycleDetect = 1;
						gsACDC_Drive.sFlag.VGridReadyforMetering = 0;
						
						/* Determine the starting point of the inverter output voltage and subsequent frequency according to the current grid voltage status */
						gsACDC_Drive.sVacCtrl.f16VacRefAmpReq = MLIB_Mul_F16as(VAC_MAX_COEFF,MLIB_Add_F16(gsACDC_Drive.sPowerMetering.f16VGridRms,FRAC16(6.0/808)));
						gsACDC_Drive.sVacCtrl.a32VInvRefFreq = gsACDC_Drive.sPowerMetering.a32VGridFreq;
						gsACDC_Drive.sVacCtrl.sSinGen.f32AngleStep = MLIB_Mul_F32(gsACDC_Drive.sVacCtrl.a32VInvRefFreq<<10,INV_SINGEN_STEP_COFF);
						gsACDC_Drive.sVacCtrl.sSinGen.f32Angle = (frac32_t)spll_obj.f16Angle<<16;
						gsACDC_Drive.sVacCtrl.sSinGen.f32AnglePrev = gsACDC_Drive.sVacCtrl.sSinGen.f32Angle;
						gsACDC_Drive.sPowerMetering.a32VInvSqrSum = gsACDC_Drive.sPowerMetering.a32VGridSqrSum;
						OffGrid_CtrlparamTune();
						gsACDC_Drive.sVacDetect.u16FirstCycle = 0;
						gu16VInvCycleCnt = gu16VGridCycleCnt;
						gsACDC_Drive.sVacCtrl.sVInvFilter.f32FltBfrY[1] = gsACDC_Drive.sVacDetect.f16VGridFilt;
						gsACDC_Drive.sVacCtrl.sVInvFilter.f16FltBfrX[1] = gsACDC_Drive.sVacDetect.f16VGrid;
						gsACDC_Drive.sPowerMetering.f16VInvRms = gsACDC_Drive.sPowerMetering.f16VGridRms;
						gsACDC_Drive.sPowerMetering.a32VInvFreq = gsACDC_Drive.sPowerMetering.a32VGridFreq;
						gsACDC_Drive.sFaultThresholds.a32VInvFreqOver = MLIB_Add_F32(gsACDC_Drive.sVacDetect.a32VacFreqNominal, ACC32(3.0));
						gsACDC_Drive.sFaultThresholds.a32VInvFreqUnder = MLIB_Sub_F32(gsACDC_Drive.sVacDetect.a32VacFreqNominal, ACC32(3.0));
						gsACDC_Drive.sFlag.VInvPol = gsACDC_Drive.sFlag.VGridPol;
						gsACDC_Drive.sFlag.VInvReadyforMetering = 1;
						gsACDC_Drive.sFlag.VInvZeroCrossforMeter = gsACDC_Drive.sFlag.VGridZeroCrossforMeter;
						PWMA->FAULT[0].FCTRL |= PWM_FCTRL_FAUTO(3);     /* enable automatic fault clearing in off-grid mode */
						gsACDC_Drive.sIlCtrl.uw16CurLimNumCnt = 0;		/* restart current protection */
						gsACDC_Drive.sIlCtrl.uw16CurLimTimerCnt = 0;
						gsACDC_Drive.u16CurrentInvMode = OFFGRID_INV;
						gsACDC_Drive.sFlag.ReadyforINVModeChange = 0;
						gsACDC_Drive.f16Duty = MLIB_Div_F16(gsACDC_Drive.sVacDetect.f16VGridFilt,MLIB_Mul_F16(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt, VDC_TO_VAC));
					}
				}
				/* grid current controller */
				if(gsACDC_Drive.u16CurrentInvMode == GRIDCONNECTED_INV)
				{
					gsACDC_Drive.sIGridCtrl.f16IGridRef = MLIB_Mul_F16(gsACDC_Drive.sIGridCtrl.f16IGridRefPeak,spll_obj.f16PLLoutsine);//gsINV_Drive.sSinGen.f16Sin);//
					gsACDC_Drive.sIGridCtrl.f16IGridErr = MLIB_Sub_F16(gsACDC_Drive.sIGridCtrl.f16IGridRef, gsACDC_Drive.sIGridCtrl.f16IGrid);
					gsACDC_Drive.sIGridCtrl.f16IGridErr = GFLIB_Limit_F16(gsACDC_Drive.sIGridCtrl.f16IGridErr,-GRIDCUR_ERR_LIMIT,GRIDCUR_ERR_LIMIT);
					gsACDC_Drive.sIGridCtrl.f16IGridCtrl1stOut = MLIB_Conv_F16l(IIR_2P2Z_II_TRANS_LIM_mac_asm_inline(gsACDC_Drive.sIGridCtrl.f16IGridErr, &gsACDC_Drive.sIGridCtrl.sIGridFundamental2p2zParams));			
					gsACDC_Drive.sIGridCtrl.f16IGridCtrl3rdOut = MLIB_Conv_F16l(IIR_2P2Z_II_TRANS_LIM_mac_asm_inline(gsACDC_Drive.sIGridCtrl.f16IGridErr, &gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonic2p2zParams));
					gsACDC_Drive.sIGridCtrl.f16IGridCtrl5thOut = MLIB_Conv_F16l(IIR_2P2Z_II_TRANS_LIM_mac_asm_inline(gsACDC_Drive.sIGridCtrl.f16IGridErr, &gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonic2p2zParams));
					gsACDC_Drive.sIGridCtrl.f16IGridCtrl7thOut = MLIB_Conv_F16l(IIR_2P2Z_II_TRANS_LIM_mac_asm_inline(gsACDC_Drive.sIGridCtrl.f16IGridErr, &gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonic2p2zParams));
				    gsACDC_Drive.sIGridCtrl.f16IGridPRCtrlOut = MLIB_AddSat_F16(MLIB_AddSat_F16(MLIB_AddSat_F16(gsACDC_Drive.sIGridCtrl.f16IGridCtrl1stOut,gsACDC_Drive.sIGridCtrl.f16IGridCtrl3rdOut),
										    		            gsACDC_Drive.sIGridCtrl.f16IGridCtrl5thOut),gsACDC_Drive.sIGridCtrl.f16IGridCtrl7thOut);	
					gsACDC_Drive.sIGridCtrl.f16IGridPICtrlOut = GFLIB_CtrlPIpAW_F16(gsACDC_Drive.sIGridCtrl.f16IGridErr, &gsACDC_Drive.sIGridCtrl.bStopIntegFlag ,&gsACDC_Drive.sIGridCtrl.sPIpAWParams);
					gsACDC_Drive.sIGridCtrl.f16IGridCtrlOut = MLIB_Add_F16(gsACDC_Drive.sIGridCtrl.f16IGridPRCtrlOut,gsACDC_Drive.sIGridCtrl.f16IGridPICtrlOut);
					gsACDC_Drive.sCurDetect.f16ICapFeedback = MLIB_Mul_F16(gsACDC_Drive.sCurDetect.f16ICapFeedbackcoef, gsACDC_Drive.sCurDetect.f16ICap);
					
					gsACDC_Drive.sVacDetect.f16VGriddiff1 = MLIB_Sub_F16(gsACDC_Drive.sVacDetect.f16VGrid,gsACDC_Drive.sVacDetect.f16VGridFiltlast1);
					gsACDC_Drive.sIGridCtrl.f16ProportinalComp = MLIB_Div_F16(gsACDC_Drive.sVacDetect.f16VGridFilt,MLIB_Mul_F16(FRAC16(380.0/VDC_SCALE), VDC_TO_VAC));
					gsACDC_Drive.sIGridCtrl.f321stdiffComp = MLIB_Mul_F32(INV_VacFeedForward_Kd1,MLIB_Conv_F32s(gsACDC_Drive.sVacDetect.f16VGriddiff1));
					gsACDC_Drive.sIGridCtrl.f322nddiffComp = (MLIB_Mul_F32(INV_VacFeedForward_Kd2,MLIB_Conv_F32s(MLIB_Add_F16(gsACDC_Drive.sVacDetect.f16VGriddiff1,gsACDC_Drive.sVacDetect.f16VGriddiff2))))<<4; 
					gsACDC_Drive.f16DutyComp = MLIB_AddSat_F16(gsACDC_Drive.sIGridCtrl.f16ProportinalComp,MLIB_Conv_F16l(MLIB_Add_F32(gsACDC_Drive.sIGridCtrl.f321stdiffComp, gsACDC_Drive.sIGridCtrl.f322nddiffComp)));
					gsACDC_Drive.f16Duty = MLIB_AddSat_F16(MLIB_SubSat_F16(gsACDC_Drive.f16DutyComp,gsACDC_Drive.sCurDetect.f16ICapFeedback),gsACDC_Drive.sIGridCtrl.f16IGridCtrlOut); 
				}
				
				gsACDC_Drive.sVacDetect.f16VGridFiltlast2 = gsACDC_Drive.sVacDetect.f16VGridFiltlast1;
				gsACDC_Drive.sVacDetect.f16VGridFiltlast1 = gsACDC_Drive.sVacDetect.f16VGridFilt;	
				gsACDC_Drive.sVacDetect.f16VGriddiff2 = MLIB_Sub_F16(gsACDC_Drive.sVacDetect.f16VGridFiltlast2,gsACDC_Drive.sVacDetect.f16VGridFiltlast1);						    				
			}
			else /* off-grid inverter controller */
			{
			    /* enable over-current protection when cycle by cycle current restriction exceeds INV_CUR_LIM_THRESHOLD times in time CUR_FAULT_CHECK_DURATION */
			    if(++gsACDC_Drive.sIlCtrl.uw16CurLimTimerCnt>=CUR_FAULT_CHECK_DURATION)
			    {
				    gsACDC_Drive.sIlCtrl.uw16CurLimTimerCnt = 0;
				    if(gsACDC_Drive.sIlCtrl.uw16CurLimNumCnt>INV_CUR_LIM_THRESHOLD)
				    {
					    gsACDC_Drive.sFaultIdPending.B.HW_IlOver = 1;
				    }
				    else gsACDC_Drive.sIlCtrl.uw16CurLimNumCnt = 0;
			    }
        #if BOARD_TEST
                gsACDC_Drive.f16Duty = gsACDC_Drive.sVacCtrl.sSinGen.f16Sin>>2;    
        #else
            #if MODE_OPTION == 0
		        gsACDC_Drive.f16Duty = MLIB_Mul_F16(gsACDC_Drive.sVacCtrl.sSinGen.f16Sin,FRAC16(0.4));
            #else
	            #if MODE_OPTION == 1
		        gsACDC_Drive.sCurCtrl.f16IlRef = MLIB_Mul_F16(FRAC16(MODE_OPTIONONE_CURRENT_REF/ISNS_SCALE),gsACDC_Drive.sVacCtrl.sSinGen.f16Sin);
	            #elif MODE_OPTION == 2
			    gsACDC_Drive.sVacCtrl.f16VacRef = MLIB_Mul_F16(gsACDC_Drive.sVacCtrl.f16VacRefAmpReq, gsACDC_Drive.sVacCtrl.sSinGen.f16Sin);
			    gsACDC_Drive.sVacCtrl.f16VacErr = MLIB_SubSat_F16(gsACDC_Drive.sVacCtrl.f16VacRef, gsACDC_Drive.sVacCtrl.f16VInv);
			    gsACDC_Drive.sVacCtrl.f16VacErr = GFLIB_Limit_F16(gsACDC_Drive.sVacCtrl.f16VacErr,-VAC_VOL_ERR_LIMIT,VAC_VOL_ERR_LIMIT);
			    gsACDC_Drive.sVacCtrl.f16VacCtrl1stOut = MLIB_Conv_F16l(IIR_2P2Z_II_TRANS_LIM_mac_asm_inline(gsACDC_Drive.sVacCtrl.f16VacErr, &gsACDC_Drive.sVacCtrl.sVacFundamental2p2zParams));						
			    gsACDC_Drive.sVacCtrl.f16VacCtrl3rdOut = MLIB_Conv_F16l(IIR_2P2Z_II_TRANS_LIM_mac_asm_inline(gsACDC_Drive.sVacCtrl.f16VacErr, &gsACDC_Drive.sVacCtrl.sVac3rdHarmonic2p2zParams));
			    gsACDC_Drive.sVacCtrl.f16VacCtrl5thOut = MLIB_Conv_F16l(IIR_2P2Z_II_TRANS_LIM_mac_asm_inline(gsACDC_Drive.sVacCtrl.f16VacErr, &gsACDC_Drive.sVacCtrl.sVac5thHarmonic2p2zParams));
			    gsACDC_Drive.sVacCtrl.f16VacCtrl7thOut = MLIB_Conv_F16l(IIR_2P2Z_II_TRANS_LIM_mac_asm_inline(gsACDC_Drive.sVacCtrl.f16VacErr, &gsACDC_Drive.sVacCtrl.sVac7thHarmonic2p2zParams));
			    gsACDC_Drive.sIlCtrl.f16IlRef = MLIB_AddSat_F16(MLIB_AddSat_F16(MLIB_AddSat_F16(gsACDC_Drive.sVacCtrl.f16VacCtrl1stOut,gsACDC_Drive.sVacCtrl.f16VacCtrl3rdOut),
					gsACDC_Drive.sVacCtrl.f16VacCtrl5thOut),gsACDC_Drive.sVacCtrl.f16VacCtrl7thOut);		
	            #endif	    
		        gsACDC_Drive.sIlCtrl.f16IlErr = MLIB_Sub_F16(gsACDC_Drive.sIlCtrl.f16IlRef, gsACDC_Drive.sIlCtrl.f16Il);		    
		        gsACDC_Drive.sIlCtrl.f16IlErr = GFLIB_Limit_F16(gsACDC_Drive.sIlCtrl.f16IlErr,-INV_IND_CUR_ERR_LIMIT,INV_IND_CUR_ERR_LIMIT);
		        gsACDC_Drive.f16Duty = GFLIB_CtrlPIpAW_F16(gsACDC_Drive.sIlCtrl.f16IlErr, &gsACDC_Drive.sIlCtrl.bStopIntegFlag ,&gsACDC_Drive.sIlCtrl.sPIpAWParams);
                #if INV_DECOUPLE_EN
		        gsACDC_Drive.f16DutyComp = MLIB_Div_F16(gsACDC_Drive.sVacCtrl.f16VacRef,MLIB_Mul_F16(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt, VDC_TO_VAC));
		        gsACDC_Drive.f16Duty = MLIB_AddSat_F16(gsACDC_Drive.f16DutyComp,gsACDC_Drive.f16Duty);		        
		        #endif
            #endif
        #endif
			}
			gsACDC_Drive.f16Duty = GFLIB_Limit_F16(gsACDC_Drive.f16Duty, -INV_DUTY_LIMIT, INV_DUTY_LIMIT);
		    INV_PWM_Update();			  		   
		}
	}
    /*===================================== AC to DC mode control =================================*/
	else if(gsACDC_Drive.u16WorkModeUsed == AC_TO_DC)	/* AC_TO_DC mode controller */			
	{		
	    /* adjust PI parameters according to input voltage */
		if(MLIB_Abs_F16(gsACDC_Drive.sVacDetect.f16VGridFilt) < gsACDC_Drive.sIlCtrl.f16VacLowTh) // different PI when vin is low for zero-crossing soft start
		{
			gsACDC_Drive.sIlCtrl.sPIpAWParams.a32PGain = PFC_CUR_P_GAIN_LVIN;
			gsACDC_Drive.sIlCtrl.sPIpAWParams.a32IGain = PFC_CUR_I_GAIN_LVIN;
		}
		else
		{
			gsACDC_Drive.sIlCtrl.sPIpAWParams.a32PGain = gsACDC_Drive.sIlCtrl.a32HVinkp;
			gsACDC_Drive.sIlCtrl.sPIpAWParams.a32IGain = gsACDC_Drive.sIlCtrl.a32HVinki;
		}
		
	    /* run current controller only in run state*/	
		if(gsACDC_Drive.bCloseLoopOnOff)
		{			
			if(gu16VGridCycleCnt < ZERO_CROSS_SOFT_START_PWM_CYCLES)
		   	{
		   		gsACDC_Drive.sIlCtrl.f16IlErr = 0; /* no regulation at zero-crossing soft-start interval */
		   	}
		   	else 
		   	{   			  			  			
		   		/* Iref = (vout_pi*sqrt(2)*sin)/vrms */ 
		   		gsACDC_Drive.sIlCtrl.f16IlRef = MLIB_Abs_F16(MLIB_MulSat_F16as(gsACDC_Drive.sIlCtrl.a32IrefReq, spll_obj.f16PLLoutsine)); 
		   	    /* a32IrefReq is limited with last input voltage RMS, adding extra limit to prevent over-current during voltage jump */
		   		gsACDC_Drive.sIlCtrl.f16IlRef = GFLIB_Limit_F16(gsACDC_Drive.sIlCtrl.f16IlRef, 0, gsACDC_Drive.sIlCtrl.f16RefHlim);
		   		gsACDC_Drive.sIlCtrl.f16IlRec = gsACDC_Drive.sFlag.VGridPol?(-gsACDC_Drive.sIGridCtrl.f16IGrid):gsACDC_Drive.sIGridCtrl.f16IGrid;
		   		gsACDC_Drive.sIlCtrl.f16IlErr = MLIB_Sub_F16(gsACDC_Drive.sIlCtrl.f16IlRef, gsACDC_Drive.sIlCtrl.f16IlRec);
		      	gsACDC_Drive.sIlCtrl.f16IlErr = GFLIB_Limit_F16(gsACDC_Drive.sIlCtrl.f16IlErr,-PFC_CUR_ERR_LIMIT,PFC_CUR_ERR_LIMIT);
		   	}
			
#if PFC_DUTY_FEEDFORWARD_EN		    
		    /* duty feed-forward */
		    //CCM mode  
		    gsACDC_Drive.sIlCtrl.f16DutyCCM = MLIB_Div1Q_F16(MLIB_SubSat_F16(gsACDC_Drive.sVdcCtrl.f16VdcRef, MLIB_MulSat_F16as(VAC_TO_VDC,gsACDC_Drive.sVacDetect.f16VGridFiltAbs)),gsACDC_Drive.sVdcCtrl.f16VdcRef);
			//DCM mode
		    gsACDC_Drive.sIlCtrl.f16PFCDCMRatioNum = MLIB_MulSat_F16as(PFC_DCM_DUTY_COEFF,gsACDC_Drive.sVdcCtrl.f16VdcCtrlOut);
		    gsACDC_Drive.sIlCtrl.f16PFCDCMRatioNum = MLIB_Mul_F16(gsACDC_Drive.sIlCtrl.f16PFCDCMRatioNum,gsACDC_Drive.sIlCtrl.f16DutyCCM);
		    gsACDC_Drive.sIlCtrl.f16DutyDCM = GFLIB_Sqrt_F16(MLIB_DivSat_F16(gsACDC_Drive.sIlCtrl.f16PFCDCMRatioNum,gsACDC_Drive.sPowerMetering.f16VGridRmsSqr));
		    
		    if(gsACDC_Drive.sIlCtrl.f16DutyCCM > gsACDC_Drive.sIlCtrl.f16DutyDCM)
		    {
		    	gsACDC_Drive.f16DutyComp = gsACDC_Drive.sIlCtrl.f16DutyDCM;
		    }
		    else
		    {
		    	gsACDC_Drive.f16DutyComp = gsACDC_Drive.sIlCtrl.f16DutyCCM;
		    }
#endif
		    gsACDC_Drive.sCurDetect.f16ICapFeedback = MLIB_Mul_F16(gsACDC_Drive.sCurDetect.f16ICapFeedbackcoef, gsACDC_Drive.sCurDetect.f16ICap);
			gsACDC_Drive.sIlCtrl.f16CurCtrlOut = GFLIB_CtrlPIpAW_F16(gsACDC_Drive.sIlCtrl.f16IlErr, &gsACDC_Drive.sIlCtrl.bStopIntegFlag, &gsACDC_Drive.sIlCtrl.sPIpAWParams);
			
#if PFC_DUTY_FEEDFORWARD_EN
		    gsACDC_Drive.f16Duty = MLIB_AddSat_F16(MLIB_SubSat_F16(gsACDC_Drive.f16DutyComp,gsACDC_Drive.sCurDetect.f16ICapFeedback),gsACDC_Drive.sIlCtrl.f16CurCtrlOut);//todo
		   	gsACDC_Drive.f16Duty = GFLIB_Limit_F16(gsACDC_Drive.f16Duty, PFC_DUTY_LLIMIT, PFC_DUTY_HLIMIT);
#else
		    gsACDC_Drive.f16Duty = gsACDC_Drive.sCurCtrl.f16CurCtrlOut;
#endif
		    
		   	PFC_PWM_UPDATE(gu16VGridCycleCnt,gsACDC_Drive.f16Duty);
		}
		
		if(gsACDC_Drive.sVacDetect.f16VGridFiltAbs>gsACDC_Drive.sVacDetect.f16VGridPeak)  
		{
			gsACDC_Drive.sVacDetect.f16VGridPeak = gsACDC_Drive.sVacDetect.f16VGridFiltAbs;
		}
	}

#if BOARD_TEST
	SM_StateMachine(&gsACDC_Ctrl);  /* state machine */
#else
	if((gsACDC_Drive.u16WorkModeCmd == AC_TO_DC)||(gsACDC_Drive.u16WorkModeUsed == AC_TO_DC)
	    ||(((gsACDC_Drive.u16WorkModeCmd == DC_TO_AC)||(gsACDC_Drive.u16WorkModeUsed == DC_TO_AC))&&
	    ((gsACDC_Drive.u16InvModeCmd == GRIDCONNECTED_INV)||(gsACDC_Drive.u16InvModeCmd == OFFGRID_INV)
	    ||(gsACDC_Drive.u16CurrentInvMode == GRIDCONNECTED_INV)||(gsACDC_Drive.u16CurrentInvMode == OFFGRID_INV))))
	{
	    SM_StateMachine(&gsACDC_Ctrl);  /* run state machine only in acceptable work mode */
	}
	if(gsACDC_Drive.sFlag.VGridReadyforMetering||gsACDC_Drive.sFlag.VInvReadyforMetering)  ACDC_PowerMetering();
#endif
	
	gsACDC_Drive.sVacDetect.f16VacLoad = ADC->RSLT[4]; /* not used in control now, just for observing and debugging */
    gsACDC_Drive.sVacDetect.f16ACVoltBias = ADC->RSLT[11];	
    gsACDC_Drive.sCurDetect.f16IBus = ADC->RSLT[3];
		
    gsACDC_Drive.f16TempSamp = ADC->RSLT[12];  /* read IGBT temperature sampling result */
	
	FMSTR_Recorder(0);		
	
	PWMA->SM[0].STS = PWM_STS_CMPF(0x20); // clear interrupt flag of Val5 compare
	
	if(gu32IOTogTimeCnt>=20000) /* slow LED flash indicate the converter is running */
	{
		HVP_LED_TOGGLE();
		gu32IOTogTimeCnt=0;
	}
		
	/* execution time measurement end*/
	TP40_LOW();
	gu16ExeTimeCnt = TMRA->CHANNEL[0].CNTR;
}
#pragma interrupt off

/* keFlexPWMA_FAULT_VECTORn interrupt handler */
#pragma interrupt alignsp saveall
void CUR_CBCPro_Cnt_ISR(void) 
{
	if(gsACDC_Drive.u16CurrentInvMode == OFFGRID_INV) gsACDC_Drive.sIlCtrl.uw16CurLimNumCnt++;
	else  gsACDC_Drive.sFaultIdPending.B.HW_IlOver = 1;
	
	/* clear fault status interrupt flags */
	ACDC_CLEAR_HWOVERCUR_FAULT();
}
#pragma interrupt off

/* kPIT0_ROLLOVR_VECTORn interrupt handler, 2kHz */
#pragma interrupt alignsp saveall
void PFC_VolCtrl_ISR(void) 
{	
	/* update voltage controller output limit, that is, the current reference limit, according to last RMS */
	gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16LowerLim = MLIB_Mul_F16(FRAC16(PFC_LOW_CURRENT/ISNS_SCALE), gsACDC_Drive.sPowerMetering.f16VGridRms);
	gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16UpperLim = MLIB_Mul_F16(FRAC16(PFC_HIGH_CURRENT/ISNS_SCALE), gsACDC_Drive.sPowerMetering.f16VGridRms);
	
	/* update current controller parameters, according to last RMS */
	if(gsACDC_Drive.sPowerMetering.f16VGridRms < PFC_VRMS_LOW_TH)
	{
		gsACDC_Drive.sIlCtrl.f16VacLowTh = FRAC16(PFC_VAC_LOWRMS_LOWTH/VAC_SCALE);
		gsACDC_Drive.sIlCtrl.a32HVinkp = PFC_CUR_P_GAIN_LVRMS;
		gsACDC_Drive.sIlCtrl.a32HVinki = PFC_CUR_I_GAIN_LVRMS;	  
	}
	else if(gsACDC_Drive.sPowerMetering.f16VGridRms > PFC_VRMS_HIGH_TH)
	{
		gsACDC_Drive.sIlCtrl.f16VacLowTh = FRAC16(PFC_VAC_HIGHRMS_LOWTH/VAC_SCALE);
		gsACDC_Drive.sIlCtrl.a32HVinkp = PFC_CUR_P_GAIN_HVRMS;
		gsACDC_Drive.sIlCtrl.a32HVinki = PFC_CUR_I_GAIN_HVRMS;		  
	}
	
	/*====================================== run sub-state =======================================*/
	if(gsACDC_Ctrl.eState == RUN && gsPFC_Runsub == SOFTSTART)
	{
		/* when voltage ref is less than the target, ramp up ref
		 * when voltage ref or output voltage is grater than the target, jump to normal mode
		 *  */
		if(gsACDC_Drive.sVdcCtrl.f16VdcRef < gsACDC_Drive.sVdcCtrl.sVdcRamp.f16Target)
		{
			gu16SoftStartStepCnt++;
			if (gu16SoftStartStepCnt >= RAMPUP_PERIOD)
			{
				gsACDC_Drive.sVdcCtrl.f16VdcRef = GFLIB_Ramp_F16(gsACDC_Drive.sVdcCtrl.sVdcRamp.f16Target, &gsACDC_Drive.sVdcCtrl.sVdcRamp.sRamp);
		        gu16SoftStartStepCnt = 0;
		       if(gsACDC_Drive.sVdcCtrl.f16VdcBusFilt >= gsACDC_Drive.sVdcCtrl.sVdcRamp.f16Target)
		       {
		           gsPFC_Runsub = NORMAL;
		           gsACDC_Drive.sFaultThresholds.f16VdcUnder = FRAC16(ACDC_VDC_UNDERVOLT_LIMIT/VDC_SCALE); 
		           gsACDC_Drive.sVdcCtrl.f16VdcRef = gsACDC_Drive.sVdcCtrl.sVdcRamp.f16Target;		/* make sure final voltage ref equals to the target */
		       }  
		    }
		}
		else
		{
			gsPFC_Runsub = NORMAL;
			gsACDC_Drive.sFaultThresholds.f16VdcUnder = FRAC16(ACDC_VDC_UNDERVOLT_LIMIT/VDC_SCALE);
		}
	}
	/*===================================== voltage controller ==================================*/
	if(gsACDC_Drive.bCloseLoopOnOff)
	{
		if(gsPFC_Runsub == LIGHTLOAD) /* fixed current reference is used in light load mode */
		{
		    gsACDC_Drive.sVdcCtrl.f16VdcCtrlOut = MLIB_Mul_F16(FRAC16(PFC_BURSTON_CURRENT/ISNS_SCALE), gsACDC_Drive.sPowerMetering.f16VGridRms);//gsACDC_Drive.sVdcCtrl.sPIpAWParams.f16LowerLim;
		}
		else
		{
			gsACDC_Drive.sVdcCtrl.f16VdcErr = MLIB_Sub_F16(gsACDC_Drive.sVdcCtrl.f16VdcRef, gsACDC_Drive.sVdcCtrl.f16VdcBusFilt); 
			gsACDC_Drive.sVdcCtrl.f16VdcErr = GFLIB_Limit_F16(gsACDC_Drive.sVdcCtrl.f16VdcErr, -VDC_VOL_ERR_LIMIT, VDC_VOL_ERR_LIMIT);
			gsACDC_Drive.sVdcCtrl.f16VdcCtrlOut = GFLIB_CtrlPIpAW_F16(gsACDC_Drive.sVdcCtrl.f16VdcErr, &gsACDC_Drive.sVdcCtrl.StopIntegFlag, &gsACDC_Drive.sVdcCtrl.sPIpAWParams);
		}

		gsACDC_Drive.sIlCtrl.a32IrefReq = MLIB_Div1Q_A32ss(gsACDC_Drive.sVdcCtrl.f16VdcCtrlOut, gsACDC_Drive.sPowerMetering.f16VGridRms);
		gsACDC_Drive.sIlCtrl.a32IrefReq = MLIB_Mul_A32(gsACDC_Drive.sIlCtrl.a32IrefReq, ACC32(1.4142));
	}	 
  
    PIT0->CTRL &= ~PIT_CTRL_PRF_MASK;		/* clear interrupt flag */
}
#pragma interrupt off

void GridCon_CtrlparamTune()
{
	SPLL_1PH_SOGI_PDCoeffConfig(&spll_obj,gsACDC_Drive.sPowerMetering.a32VGridFreqAvg,ACC32(SPLL_CTRL_FREQ/2),ACC32(SOGI_K_COEFF));
	
	if(gsACDC_Drive.u16WorkModeUsed == DC_TO_AC)
	{
	    /* grid current PR parameter calculation */ 
	    gsACDC_Drive.sIGridCtrl.sIGridFundamentalPRParams.w0 = MLIB_Mul_A32(gsACDC_Drive.sPowerMetering.a32VGridFreqAvg, ACC32(6.283185));		    
	    ComputePRCoeff(&gsACDC_Drive.sIGridCtrl.sIGridFundamentalPRParams,&gsACDC_Drive.sIGridCtrl.sIGridFundamental2p2zParams);		    		     
	    gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonicPRParams.w0 = MLIB_Mul_A32(gsACDC_Drive.sPowerMetering.a32VGridFreqAvg, ACC32(18.849556));	    
	    ComputePRCoeff(&gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonicPRParams,&gsACDC_Drive.sIGridCtrl.sIGrid3rdHarmonic2p2zParams);		    		     				    	
	    gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonicPRParams.w0 = MLIB_Mul_A32(gsACDC_Drive.sPowerMetering.a32VGridFreqAvg, ACC32(31.415927));    
	    ComputePRCoeff(&gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonicPRParams,&gsACDC_Drive.sIGridCtrl.sIGrid5thHarmonic2p2zParams);		    		     	
	    gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonicPRParams.w0 = MLIB_Mul_A32(gsACDC_Drive.sPowerMetering.a32VGridFreqAvg, ACC32(43.982297));		    
	    ComputePRCoeff(&gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonicPRParams,&gsACDC_Drive.sIGridCtrl.sIGrid7thHarmonic2p2zParams);		    		     		
	}
}

void OffGrid_CtrlparamTune()
{
	gsACDC_Drive.sVacCtrl.sVacFundamentalPRParams.w0 = MLIB_Mul_A32(gsACDC_Drive.sVacCtrl.a32VInvRefFreq, ACC32(6.283185));    		    		    		    		    
	ComputePRCoeff(&gsACDC_Drive.sVacCtrl.sVacFundamentalPRParams,&gsACDC_Drive.sVacCtrl.sVacFundamental2p2zParams);		    		         		    		
	gsACDC_Drive.sVacCtrl.sVac3rdHarmonicPRParams.w0 = MLIB_Mul_A32(gsACDC_Drive.sVacCtrl.a32VInvRefFreq, ACC32(18.849556));    		    			    
	ComputePRCoeff(&gsACDC_Drive.sVacCtrl.sVac3rdHarmonicPRParams,&gsACDC_Drive.sVacCtrl.sVac3rdHarmonic2p2zParams);		    		         		    		
	gsACDC_Drive.sVacCtrl.sVac5thHarmonicPRParams.w0 = MLIB_Mul_A32(gsACDC_Drive.sVacCtrl.a32VInvRefFreq, ACC32(31.415927));    		    				    
	ComputePRCoeff(&gsACDC_Drive.sVacCtrl.sVac5thHarmonicPRParams,&gsACDC_Drive.sVacCtrl.sVac5thHarmonic2p2zParams);		    		         		    		
	gsACDC_Drive.sVacCtrl.sVac7thHarmonicPRParams.w0 = MLIB_Mul_A32(gsACDC_Drive.sVacCtrl.a32VInvRefFreq, ACC32(43.982297));   		    			    
	ComputePRCoeff(&gsACDC_Drive.sVacCtrl.sVac7thHarmonicPRParams,&gsACDC_Drive.sVacCtrl.sVac7thHarmonic2p2zParams);
}

void ACDC_PowerMetering()
{	
	gsACDC_Drive.sPowerMetering.f16IacSqr = MLIB_Mul_F16(gsACDC_Drive.sIGridCtrl.f16IGridFilt,gsACDC_Drive.sIGridCtrl.f16IGridFilt);
	gsACDC_Drive.sPowerMetering.a32IacSqrSum = MLIB_Add_A32as(gsACDC_Drive.sPowerMetering.a32IacSqrSum,gsACDC_Drive.sPowerMetering.f16IacSqr);
	
	if(gsACDC_Drive.sFlag.VGridReadyforMetering)
	{
		if(gsACDC_Drive.u16WorkModeUsed == AC_TO_DC)
		{
			gsACDC_Drive.sPowerMetering.f16VIinst = MLIB_Mul_F16(gsACDC_Drive.sVacDetect.f16VGridFilt,MLIB_Neg_F16(gsACDC_Drive.sIGridCtrl.f16IGridFilt));
			gsACDC_Drive.sPowerMetering.a32VIinstSum = MLIB_Add_A32as(gsACDC_Drive.sPowerMetering.a32VIinstSum,gsACDC_Drive.sPowerMetering.f16VIinst);
			if(gsACDC_Drive.sFlag.VGridZeroCrossforMeter == 1)
			{
				if(gu16VGridCycleCnt <150) /* avoid wrong metering results during ac drop, short power outages in PFC mode are acceptable */
				{
					if(gsACDC_Ctrl.eState == RUN)
					{
					    gsACDC_Drive.bCloseLoopOnOff = 0;
					    ACDC_FASTBRIDGE_PWM_MASK();
					}
					gsACDC_Drive.sFlag.VGridReadyforMetering = 0;
					    			        		    	    
					gsACDC_Drive.sFlag.AcDrop = 1;
					gsACDC_Drive.u16VacRecoveryOkCnt = 0;
					guw32StartCnt = gu32TimerCnt;
				}
				
				/* record the cumulative value for metering calculation in the background loop */
				gsACDC_Drive.sPowerMetering.a32IacSqrSumSav = gsACDC_Drive.sPowerMetering.a32IacSqrSum;
				gsACDC_Drive.sPowerMetering.a32VIinstSumSav = gsACDC_Drive.sPowerMetering.a32VIinstSum;
				gsACDC_Drive.sPowerMetering.a32IacSqrSum = 0;
				gsACDC_Drive.sPowerMetering.a32VIinstSum = 0;				
			}	
		}
		else if(gsACDC_Drive.u16CurrentInvMode == GRIDCONNECTED_INV)
		{
			gsACDC_Drive.sPowerMetering.f16VIinst = MLIB_Mul_F16(gsACDC_Drive.sVacDetect.f16VGridFilt,gsACDC_Drive.sIGridCtrl.f16IGridFilt);
			gsACDC_Drive.sPowerMetering.a32VIinstSum = MLIB_Add_A32as(gsACDC_Drive.sPowerMetering.a32VIinstSum,gsACDC_Drive.sPowerMetering.f16VIinst);
			if(gsACDC_Drive.sFlag.VGridZeroCrossforMeter == 1)
			{
				/* power metering is executed at grid voltage zero-crossing point in grid-connected mode */
				gsACDC_Drive.sPowerMetering.a32IacSqrSumSav = gsACDC_Drive.sPowerMetering.a32IacSqrSum;
				gsACDC_Drive.sPowerMetering.a32VIinstSumSav = gsACDC_Drive.sPowerMetering.a32VIinstSum;
				gsACDC_Drive.sPowerMetering.a32IacSqrSum = 0;
				gsACDC_Drive.sPowerMetering.a32VIinstSum = 0; 
			}
		}
		
		gsACDC_Drive.sPowerMetering.f16VGridSqr = MLIB_Mul_F16(gsACDC_Drive.sVacDetect.f16VGridFilt,gsACDC_Drive.sVacDetect.f16VGridFilt);
		gsACDC_Drive.sPowerMetering.a32VGridSqrSum = MLIB_Add_A32as(gsACDC_Drive.sPowerMetering.a32VGridSqrSum,gsACDC_Drive.sPowerMetering.f16VGridSqr);				
		if(gsACDC_Drive.sFlag.VGridZeroCrossforMeter == 1)
		{   	
		    /* record the cumulative value for metering calculation in the background loop */
		    gsACDC_Drive.sPowerMetering.a32VGridSqrSumSav = gsACDC_Drive.sPowerMetering.a32VGridSqrSum;
		    gu16VGridCycleCntSav = gu16VGridCycleCnt;    	    	    	        	    	
		    /* clear the cumulative value for subsequent accumulation */
		    gsACDC_Drive.sPowerMetering.a32VGridSqrSum = 0;
		    gu16VGridCycleCnt = 0;		    	    	    	        	   	
		    gsACDC_Drive.sFlag.VGridZeroCrossforMeter = 0;
		    gsACDC_Drive.sFlag.VGridMeteringDone = 0;
		}
	}
	
	if(gsACDC_Drive.sFlag.VInvReadyforMetering)
	{
		gsACDC_Drive.sPowerMetering.f16VInvSqr = MLIB_Mul_F16(gsACDC_Drive.sVacCtrl.f16VInvFilt,gsACDC_Drive.sVacCtrl.f16VInvFilt);
		gsACDC_Drive.sPowerMetering.a32VInvSqrSum = MLIB_Add_A32as(gsACDC_Drive.sPowerMetering.a32VInvSqrSum,gsACDC_Drive.sPowerMetering.f16VInvSqr);
		gsACDC_Drive.sPowerMetering.f16VIinst = MLIB_Mul_F16(gsACDC_Drive.sVacCtrl.f16VInvFilt,gsACDC_Drive.sIGridCtrl.f16IGridFilt);    	    
		gsACDC_Drive.sPowerMetering.a32VIinstSum = MLIB_Add_A32as(gsACDC_Drive.sPowerMetering.a32VIinstSum,gsACDC_Drive.sPowerMetering.f16VIinst);	
		    	
		if(gsACDC_Drive.sFlag.VInvZeroCrossforMeter == 1)
		{   	
		    if(gsACDC_Drive.sVacDetect.u16FirstCycle)
		    {
		    	gu16VInvCycleCnt = gu16VInvCycleCnt - VAC_POL_CHG_NUM;
		    	gsACDC_Drive.sVacDetect.u16FirstCycle = 0;		
		    }
		    /* power metering is executed at inverter output voltage zero-crossing point in grid-connected mode */
		    gsACDC_Drive.sPowerMetering.a32VInvSqrSumSav = gsACDC_Drive.sPowerMetering.a32VInvSqrSum;
		    gsACDC_Drive.sPowerMetering.a32IacSqrSumSav = gsACDC_Drive.sPowerMetering.a32IacSqrSum;
		    gsACDC_Drive.sPowerMetering.a32VIinstSumSav = gsACDC_Drive.sPowerMetering.a32VIinstSum;
		    gu16VInvCycleCntSav = gu16VInvCycleCnt;
		    gsACDC_Drive.sFlag.VInvMeteringDone = 0;
		    	    	
		    gsACDC_Drive.sPowerMetering.a32VInvSqrSum = 0;
		    gsACDC_Drive.sPowerMetering.a32IacSqrSum = 0;
		    gsACDC_Drive.sPowerMetering.a32VIinstSum = 0;
		    gu16VInvCycleCnt = 0;
		    	   	
		    gsACDC_Drive.sFlag.VInvZeroCrossforMeter = 0;    
		}
	}
}
#pragma section CODES_IN_RAM end

void Temp_Sensing(uint16_t u16TempADValue)
{
	uint16_t u16Count;
	uint16_t u16LinearApproximation, u16ShiftedADVal;
	
	u16ShiftedADVal = u16TempADValue>>3;
	for(u16Count=0;u16Count<13;u16Count++)
	{
		if(u16ShiftedADVal>u16TempTable[u16Count])  break;
	}
	
	if(u16Count==0) gsACDC_Drive.u16ActualTempVal = 0;
	else if(u16Count==13) gsACDC_Drive.u16ActualTempVal = 120;
	else
	{
		u16LinearApproximation = 10*(u16TempTable[u16Count-1]-u16ShiftedADVal)/(u16TempTable[u16Count-1]-u16TempTable[u16Count]);
	    gsACDC_Drive.u16ActualTempVal = (u16Count-1)*10+u16LinearApproximation;
	}
}
