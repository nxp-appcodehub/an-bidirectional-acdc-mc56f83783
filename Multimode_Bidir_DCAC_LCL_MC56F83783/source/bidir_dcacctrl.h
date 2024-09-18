/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef BIDIR_DCACCTRL_H_
#define BIDIR_DCACCTRL_H_

/******************************************************************************
* Includes
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/

/* work mode */
#define DC_TO_AC     1  
#define AC_TO_DC     2
#define BOARD_TEST   0       /* 0= power conversion, 1= board test
                                DC-AC mode is selected when board test is enabled, don't connect the power supply to the AC side */

#define GRIDCONNECTED_INV    1
#define OFFGRID_INV          2

/* ******************** ======================= DC_TO_AC part =========================************************************/
/* ************====================== off-grid INV  ========================******************/
/* Build options for DC TO AC mode  
 * 0: open loop check the inverter
 * 1: single current loop control, given current reference
 * 2: voltage loop with inner current loop
 * */
#define MODE_OPTION                   2     /* select the working mode:
                                             0: open loop check the inverter
                                             1: single current loop control, given current reference
                                             2: voltage loop with inner current loop */
#define INV_VOLTAGE_OPTION            0     /* select the output voltage grade in off-grid mode, 220V is 0, 110V is 1 */


#define MODE_OPTIONONE_CURRENT_REF   1.0    /* reference current peak value at build option 1 */
#define INV_I_LOADON                 FRAC16(1.0/ISNS_SCALE) /* [A], when AC RMS current over the threshold,it is seen that load is on, used for non-linear load test */

#define INV_DECOUPLE_EN      1      /* 1=inverter decoupling control enable, 0=disable */

/* INV AC voltage reference parameters */
#if INV_VOLTAGE_OPTION == 0
#define INV_AC_FREQ          50.0   /* [Hz], output voltage frequency*/
#define INV_AC_REF_RMS       226.0  /* 220V required RMS of output ac voltage, set as 226 to compensate the TRIAC voltage drop */
#else
#define INV_AC_FREQ          60.0   /* [Hz], output voltage frequency*/
#define INV_AC_REF_RMS       114.0  /* 110V required RMS of output ac voltage, set as 114 to compensate the TRIAC voltage drop */
#endif

#define INV_SLOWBRIDGE_DT_CNT       2   	/* deadtime = CNT * control frequency of current loop */

/* voltage compensator parameters */
/* AC voltage fundamental wave controller parameters (PR controller) */
#define INV_SAMPLING_FREQ           20000  				/* [Hz], sampling frequency fs */
#define INV_VOL_1H_P_GAIN           0.3                 /* kp in continuous domain */
#define INV_VOL_1H_I_GAIN           300.0  				/* kr in continuous domain  */
#define INV_VOL_1H_WC               1.5708 				/* 2*pi*fc */
#define INV_VOL_1H_PR_UPPER_LIMIT   (10.0/ISNS_SCALE) 	/* voltage controller output as current reference,max = 10A */
#define INV_VOL_1H_PR_LOWER_LIMIT   (-10.0/ISNS_SCALE)

#define INV_VOL_3H_I_GAIN           100.0
#define INV_VOL_3H_WC               0.25
#define INV_VOL_5H_I_GAIN           100.0
#define INV_VOL_5H_WC               0.25
#define INV_VOL_7H_I_GAIN           100.0
#define INV_VOL_7H_WC               0.25

#define VAC_VOL_ERR_LIMIT        	 FRAC16(30.0/VAC_SCALE)   /* [V], maximum AC voltage error for compensator */

/********** current compensator parameters **********/
#define INV_CUR_P_GAIN           ACC32(0.1)    	/* kp */
#define INV_CUR_I_GAIN           ACC32(0.15)    /* ki*Ts */
#if INV_DECOUPLE_EN
#define INV_CUR_PI_UPPER_LIMIT   FRAC16(0.3)       
#define INV_CUR_PI_LOWER_LIMIT   FRAC16(-0.3)
#else
#define INV_CUR_PI_UPPER_LIMIT   FRAC16(0.95)       
#define INV_CUR_PI_LOWER_LIMIT   FRAC16(-0.95)
#endif
#define INV_IND_CUR_ERR_LIMIT    FRAC16(0.5/ISNS_SCALE)    /* [A], maximum current error for compensator*/
#define INV_DUTY_LIMIT           FRAC16(0.95)

/* ************====================== grid-connected INV  ========================******************/
#define GRIDCONNECTED_CURRENT_PEAK    2.5    /* [A], reference current peak value at grid-connected mode */

/***** grid current compensator parameters *****/
#define Q27(x)                   (x*134217728ul)
/* PR controller parameters */
#define INV_SAMPLING_FREQ               20000  		/* [Hz], sampling frequency fs */
#define INV_GRIDCUR_1H_P_GAIN           2.017       /* kp in continuous domain */
#define INV_GRIDCUR_1H_I_GAIN           625.0  		/* kr in continuous domain  */
#define INV_GRIDCUR_1H_WC               0.6283 		/* 2*pi*fc */

#define INV_GRIDCUR_3H_I_GAIN           200.0
#define INV_GRIDCUR_3H_WC               0.9425      
#define INV_GRIDCUR_5H_I_GAIN           200.0
#define INV_GRIDCUR_5H_WC               1.5708      
#define INV_GRIDCUR_7H_I_GAIN           100.0
#define INV_GRIDCUR_7H_WC               2.1991   
#define INV_GRIDCUR_9H_I_GAIN           100.0
#define INV_GRIDCUR_9H_WC               2.8274

#define INV_GRIDCUR_PR_LOWER_LIMIT      -0.2
#define INV_GRIDCUR_PR_UPPER_LIMIT      0.2

#define INV_GRIDCUR_P_GAIN              ACC32(1.0)
#define INV_GRIDCUR_I_GAIN              ACC32(0.2)

#define GRIDCUR_ERR_LIMIT               FRAC16(3.0/ISNS_SCALE)        

/* capacitor current feedback coefficient */
#define INV_HIC    0.06  //0.06=0.294/ICAP_SCALE

/* vac full feedforward coefficient */
#define INV_CTRL_FREQ        	 20000.0  		  /* [Hz], inverter control frequency */
#define INV_LCL_C                0.0000022  /* 2.2uF */
#define INV_LCL_L1               0.003268   /* 3.268mH */
#define INV_VacFeedForward_Kd1   FRAC32(INV_HIC*INV_LCL_C*INV_CTRL_FREQ) /* =0.00264 */
#define INV_VacFeedForward_Kd2   Q27(VAC_SCALE*INV_LCL_L1*INV_LCL_C*INV_CTRL_FREQ*INV_CTRL_FREQ/380.0) /* =6.11494 */

/* ************================== transition mode parameters  ===================***********/
#define INV_SYNC_P_GAIN          ACC32(3.0)
#define INV_SYNC_I_GAIN          ACC32(0.02)
#define INV_SYNC_PI_UPPER_LIMIT  FRAC16(1.0)  /* PI output used as the frequency adjustment value, max 1Hz*/
#define INV_SYNC_PI_LOWER_LIMIT  FRAC16(-1.0)

/* ********************================= AC_TO_DC part ====================************************************/
#define PFC_DUTY_FEEDFORWARD_EN    1 		/* 1=duty-feed-forward enable, 0=disable */

#define PFC_VDC_REF           	   380.0 	/* [V], DC bus voltage reference */
/* voltage compensator parameters */
#define PFC_VOL_P_GAIN     ACC32(0.15)		/* kp */
#define PFC_VOL_I_GAIN     ACC32(0.01)//ACC32(0.0165)	/* ki*Ts */
#define VDC_VOL_ERR_LIMIT      	FRAC16(50.0/VDC_SCALE) /* [V], maximum DC voltage error for compensator */

/* PFC pre-charger parameters */
#define VAC_TREND_CONFIRM_CNT      3 		/* Counter used to confirm if the Vac trend is changed */
#define VAC_TRIAC_THRESHOLD        30.0 	/* [V], If the Vac is below the threshold, the TRIAC will be close */
#define VAC_PRECHARGE_STEP         1.0 		/* [V], the step for PFC pre-charging process */
#define TRIAC_ON_TIME_CNT          1        /* [ctrol cycles], TRIAC on cycles*/

/* PFC soft-start parameters */
#define RAMPUP_VOL_VAL    1.0  		  		/* output voltage reference change step in soft start */
#define RAMPUP_PERIOD     2  		  		/* 1/2000*2 = 0.001s, output voltage reference change frequency in soft start */

/* PFC burst mode parameters */
#define PFC_VDC_BURSTON       	   (PFC_VDC_REF-8.0)      /* lower threshold voltage of peak-valley controller in light load mode */   
#define PFC_VDC_BURSTOFF      	   (PFC_VDC_REF-1.0)      /* upper threshold voltage of peak-valley controller in light load mode */
#define PFC_BURST_OFF_MIN_DURATION  3  				      /* [half-cycles], when output voltage drop to peak-valley controller valley within this 
                                         	 	 	 	     duration at light-load mode, converter returns to normal mode directly */ 
#define VDC_NORMAL_OVERSHOOT_LOW          (PFC_VDC_REF+8.0)		/* Enter burst mode when DC bus voltage is larger than command this value and voltage controller outputs lower limit [V] */ 
#define VDC_NORMAL_OVERSHOOT_HIGH         (PFC_VDC_REF+18.0)	/* Enter burst mode when DC bus voltage is larger than command this value [V] */ 
#define VDC_BURST_UNDER                   (PFC_VDC_REF-15.0)	/* During burst mode, when DC bus voltage is lesser than command this value , go to normal sub-state directly*/

#define INPUT_VRMS_DET_NUM          3
/* current compensator parameters */
#define PFC_VRMS_LOW_TH          FRAC16(140.0/VAC_SCALE)	/* AC voltage threshold for different PI parameters when RMS of vac < VRMST_LOW_VALUE */
#define PFC_VRMS_HIGH_TH         FRAC16(160.0/VAC_SCALE) 	/* AC voltage threshold for different PI parameters when RMS of vac > VRMS_HIGH_VALUE */
#define PFC_VAC_LOWRMS_LOWTH     20 						/* AC voltage threshold for different PI parameters near the zero-crossing point when working at low RMS mode */
#define PFC_VAC_HIGHRMS_LOWTH    30 						/* AC voltage threshold for different PI parameters near the zero-crossing point when working at high RMS mode */

#define PFC_CUR_P_GAIN_LVIN 	 ACC32(4.0)  			/* PI parameters when vac is low for zero-crossing soft-start */
#define PFC_CUR_I_GAIN_LVIN   	 ACC32(0.6869)
#define PFC_CUR_P_GAIN_LVRMS  	 ACC32(4.0) 			/* PI parameters when vac_rms<VRMS_LOW_TH */
#define PFC_CUR_I_GAIN_LVRMS  	 ACC32(0.1)
#define PFC_CUR_P_GAIN_HVRMS  	 ACC32(4.0) 			/* PI parameters when vac_rms>VRMS_HIGH_TH */
#define PFC_CUR_I_GAIN_HVRMS  	 ACC32(0.11)
#define PFC_CUR_ERR_LIMIT  		 FRAC16(5.0/ISNS_SCALE)

#define PFC_BURSTON_CURRENT      0.5                    /* fixed rms value of current reference in light-load mode*/
#define PFC_LOW_CURRENT          0.25 				    /* minimum rms value of current reference, fixed current reference rms used in light load mode */
#define PFC_HIGH_CURRENT         4.5 	 				/* maximum rms value of current reference */
#define PFC_IREF_HLIMIT          FRAC16(PFC_HIGH_CURRENT*1.414/ISNS_SCALE) 		/* High limit of Iref, FRAC16(4.5*1.414/ISNS_SCALE) */
#define PFC_DUTY_HLIMIT          FRAC16(0.95)    	/* MAX applicable duty cycle */
#define PFC_DUTY_LLIMIT          FRAC16(0.0)     	/* MIN applicable duty cycle */
#define PFC_CUR_PI_UP_LIMIT    	 FRAC16(0.95)    	/* upper limit of current PI controller output */ 
#define PFC_CUR_PI_LOW_LIMIT   	 FRAC16(-0.95)	 	/* lower limit of current PI controller output */

#define PFC_HIC                  0.065 //0.065=5*ICAP_SCALE/Kpwm 

/* compensation related parameters */
#define PFC_DCM_DUTY_COEFF   	  ACC32(7.6)       /* (2*L*ISNS_SCALE)/(Ts*VAC_SCALE), used in DCM duty calculation */
#define VDC_FEEDFORWARD_COMP_UP  (PFC_VDC_REF+8.0) /* During normal mode, when DC bus voltage is larger than this value, extra feed-forward compensation is added */
#define VDC_FEEDFORWARD_COMP_LOW (PFC_VDC_REF-8.0) /* During normal mode, when DC bus voltage is lower than this value, extra feed-forward compensation is added */

/* vac drop/recovery detection */
#define VAC_FAIL_THRESHOLD         FRAC16(10.0/VAC_SCALE) /* ac drop judgement threshold */ 
#define VAC_FAIL_CONFIRM_NUM 	   20 /*when ac sampling value is less than the drop threshold for this 
                                        number of consecutive times, the input voltage is considered power down. */
#define VAC_RECOVERY_THRESHOLD     FRAC16(100.0/VAC_SCALE)
#define VAC_RECOVERY_CONFIRM_NUM   5  /*when ac sampling value is larger than the recovery threshold for
                                        this number of consecutive times, the input voltage is considered power up again. */

#define ZERO_CROSS_SOFT_START_PWM_CYCLES 1 /* a gradually increasing duty cycle is applied instead of PI controller output for this number of PWM cycles after zero crossing of input voltage */
#define VAC_ZERO_CROSS_CNT         2  		/* detect the first 2 zero-crossing point when the PFC is just power up */

/* ********************=================== common part =====================************************************/
/* Based value */
#define VDC_SCALE      471.8  						/* [V], MAX measurable DC bus voltage */  
#define VAC_SCALE      808.0  						/* [V], MAX measurable AC voltage */
#define ISNS_SCALE     47.98  						/* [I], MAX measurable inductor current */
#define ICAP_SCALE     4.9    						/* [I], MAX measurable capacitor current */
#define VBIAS_SCALE    5.12   						/* [I], MAX measurable AC bias voltage */
#define VAC_TO_VDC     ACC32(VAC_SCALE/VDC_SCALE)	/* range conversion, used for standard unit numerical calculation */
#define VDC_TO_VAC     FRAC16(VDC_SCALE/VAC_SCALE)
#define VAC_MAX_COEFF  ACC32(1.414)                 /* coefficient for calculating peak unit value from effective value */

/* Fault thresholds */
#define ACDC_VAC_OVERVOLT_LIMIT     	380.0 		/* [V], Max value of AC voltage */
#define ACDC_VAC_RMS_LOW_LIMIT    	    FRAC16(80.0/VAC_SCALE)	/* [V], Min RMS of AC voltage */
#define ACDC_VAC_RMS_UP_LIMIT           FRAC16(265.0/VAC_SCALE)	/* [V], Max RMS of AC voltage */
#define ACDC_VAC_OVERFREQ_LIMIT     	ACC32(65.0) /* [Hz], Max frequency of AC voltage */
#define ACDC_VAC_UNDERFREQ_LIMIT    	ACC32(45.0) /* [Hz], Min frequency of AC voltage */
#define ACDC_ILOVER_LIMIT            	12.0 		/* [A], Max inductor current  */
#define ACDC_IGRIDOVER_LIMIT            12.0 		/* [A], Max inductor current  */
#define ACDC_TEMPERATURE_LIMIT      	0.128 		/* [], Max allowed temperature is 75 degree */ 
#define ACDC_VDC_OVERVOLT_LIMIT     	410.0		/* [V], Max value of DC bus voltage */
#define ACDC_VDC_UNDERVOLT_LIMIT     	310.0		/* [V], Min value of DC bus voltage */

#define INV_CUR_LIM_THRESHOLD           1000        /* if hw CBC current limitation counts exceeds the threshold, then trigger the over-current fault */

/* SPLL parameters */
#define SPLL_SOGI_LF_B0     	   ACC32(2.2228)
#define SPLL_SOGI_LF_B1     	   ACC32(-2.2204)
#define SPLL_CTRL_FREQ             20000.0   /* [Hz] SPLL run frequency */
#define SOGI_K_COEFF               0.5       /* second order generalized integrator(SOGI) based orthogonal system coefficient                                        the smaller the coefficient, the narrower the bandpass and the slower the dynamic response */

/* ************************ digital filter parameters ************************ */
/* AC voltage filter parameters, loop sample time = 0.00005[sec](20kHz) */
#define ACDC_VAC_IIR_B0         FRAC32(0.4208 / 2.0)		//fc = 4kHz
#define ACDC_VAC_IIR_B1         FRAC32(0.4208 / 2.0)
#define ACDC_VAC_IIR_A1         FRAC32(-0.1584 / -2.0)
/* inductor current filter parameters, loop sample time = 0.00005[sec](20kHz) */
#define ACDC_IL_IIR_B0          FRAC32(0.2452 / 2.0)		//fc = 2kHz
#define ACDC_IL_IIR_B1          FRAC32(0.2452 / 2.0)
#define ACDC_IL_IIR_A1          FRAC32(-0.5095 / -2.0)
/* Grid current filter parameters, loop sample time = 0.00005[sec](20kHz) */
#define INV_IGRID_IIR_B0        FRAC32(0.2452 / 2.0)		//fc = 2kHz
#define INV_IGRID_IIR_B1        FRAC32(0.2452 / 2.0)
#define INV_IGRID_IIR_A1        FRAC32(-0.5096 / -2.0)
/* DC bus voltage filter parameters, loop sample time = 0.00005[sec](20kHz) */
#define ACDC_U_DCB_IIR_B0       FRAC32(0.13 / 2.0)			//fc = 1kHz
#define ACDC_U_DCB_IIR_B1       FRAC32(0.13 / 2.0)
#define ACDC_U_DCB_IIR_A1       FRAC32(-0.74 / -2.0)
/* MA filter number for current offset, number of samples for averaging = 2^ACDC_SAMP_OFFSET_MA_WINDOW */
#define ACDC_SAMP_OFFSET_MA_WINDOW   3

/* sin generation and PWM parameters */
#define PFC_CTRL_FREQ        	 20000.0  							/* [Hz], PFC control frequency */
#define ACDC_PWM_FREQ        	 20000.0  							/* [Hz], switch frequency */
#define INV_SIN_STEP         	 FRAC32(2*INV_AC_FREQ/INV_CTRL_FREQ)/* step for sine generation */ 
#define INV_SINGEN_STEP_COFF     FRAC32(128/INV_CTRL_FREQ)
#define PWM_MODU_FREQ        	 100000 							/* [kHz], control chip pwm module frequency */
#define ACDC_PWM_HALF_PERIOD     (PWM_MODU_FREQ/ACDC_PWM_FREQ*500.0)/* half period = period/2 = 100000kHz/20000Hz*500 */

/* vac polarity detection related count*/
#define VAC_POL_CHG_NUM          	 3   	/* the polarity is accepted at least this value of consecutive times are consistent */  
#define VAC_POL_START_DET_NUM    	 77 	/* start polarity detection position, normally > 1/4 cycle sample numbers  */

/* time delay duration based on time base 50us */
#define SAMPOFFSET_CALIB_DURATION     4000  	/* 200ms */
#define FAULT_RELEASE_DURATION        60000 	/* 3000ms */
#define CUR_FAULT_CHECK_DURATION      4000  	/* 200ms */
#define VAC_DROP_MAX_DURATION         2000      /* 100ms */

#endif /* BIDIR_DCACCTRL_H_ */
