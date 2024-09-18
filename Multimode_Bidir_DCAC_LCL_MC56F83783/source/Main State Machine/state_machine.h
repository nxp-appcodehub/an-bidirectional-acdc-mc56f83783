/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2008 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
* SPDX-License-Identifier: BSD-3-Clause

***************************************************************************//*!
*
* @file      state_machine.h
* 
* @version   1.0.1.0
* 
* @brief     State machine
*
*******************************************************************************
*
* State machine.
*
******************************************************************************/

#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_
#include "cpu.h"
/******************************************************************************
* Includes
******************************************************************************/


/******************************************************************************
* Constants
******************************************************************************/
//#ifndef true
//#define true  ((bool_t)1)
//#endif
//
//#ifndef false
//#define false ((bool_t)0)
//#endif

/* Application state identification enum */
typedef enum {
    FAULT           = 0,
    INIT            = 1,
    STOP           	= 2,
	RUN				= 3,
} SM_APP_STATE_T;         

typedef unsigned short SM_APP_CTRL;
typedef unsigned long SM_APP_FAULT;

typedef void (*PFCN_VOID_VOID)(void); /* pointer to function */

/* User state machine functions structure */
typedef struct
{
	PFCN_VOID_VOID	Fault;
	PFCN_VOID_VOID	Init;
	PFCN_VOID_VOID	Stop;
	PFCN_VOID_VOID	Run;
} SM_APP_STATE_FCN_T;

/* User state-transition functions structure*/
typedef struct
{
	PFCN_VOID_VOID	FaultInit;
	PFCN_VOID_VOID	InitFault;
	PFCN_VOID_VOID	InitStop;
	PFCN_VOID_VOID	StopFault;
	PFCN_VOID_VOID	StopInit;
	PFCN_VOID_VOID	StopRun;
	PFCN_VOID_VOID	RunFault;
	PFCN_VOID_VOID	RunStop;
} SM_APP_TRANS_FCN_T;

/* State machine control structure */
typedef struct
{
    SM_APP_STATE_FCN_T const*	psState;			/* State functions */
    SM_APP_TRANS_FCN_T const* 	psTrans; 			/* Transition functions */
   	SM_APP_CTRL							uiCtrl;				/* Control flags */
    SM_APP_STATE_T						eState;				/* State */
} SM_APP_CTRL_T;

/* pointer to function with a pointer to state machine control structure */
typedef void (*PFCN_VOID_PSM)(SM_APP_CTRL_T *sAppCtrl); 


/* State machine control command flags */
#define SM_CTRL_NONE		0x0
#define SM_CTRL_FAULT		0x1
#define SM_CTRL_FAULT_CLEAR	0x2
#define SM_CTRL_INIT_DONE	0x4
#define SM_CTRL_STOP		0x8
#define SM_CTRL_START		0x10
#define SM_CTRL_STOP_ACK	0x20
#define SM_CTRL_RUN_ACK		0x40

/* State machine function table  */
extern PFCN_VOID_PSM gSM_STATE_TABLE[4];

/* State machine function */
extern inline void SM_StateMachine(SM_APP_CTRL_T *sAppCtrl)
{
	gSM_STATE_TABLE[sAppCtrl->eState](sAppCtrl);
}

#endif //_STATE_MACHINE_H_

