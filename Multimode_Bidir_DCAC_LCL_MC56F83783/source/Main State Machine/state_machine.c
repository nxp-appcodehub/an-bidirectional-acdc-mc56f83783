/******************************************************************************
* 
* Copyright (c) 2012 Freescale Semiconductor;
* All Rights Reserved                       
* SPDX-License-Identifier: BSD-3-Clause
******************************************************************************* 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
***************************************************************************//*!
*
* @file      state_machine.c
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Jul-26-2012
* 
* @brief     State machine
*
*******************************************************************************
*
* State machine.
*
******************************************************************************/


 
/******************************************************************************
* Includes
******************************************************************************/
#include "state_machine.h"
#include "hwcontrol.h"

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
* Local variables
******************************************************************************/

/******************************************************************************
* Local functions
******************************************************************************/

/*------------------------------------
 * Application state machine functions
 * ----------------------------------*/
static void SM_StateFault(SM_APP_CTRL_T *psAppCtrl);
static void SM_StateInit(SM_APP_CTRL_T *psAppCtrl);
static void SM_StateStop(SM_APP_CTRL_T *psAppCtrl);
static void SM_StateRun(SM_APP_CTRL_T *psAppCtrl);

/******************************************************************************
* Global functions
******************************************************************************/
/* State machine functions field (in pmem) */
PFCN_VOID_PSM gSM_STATE_TABLE[4] = {SM_StateFault, SM_StateInit, SM_StateStop, SM_StateRun};


/***************************************************************************//*!
*
* @brief   FAULT state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void SM_StateFault(SM_APP_CTRL_T *psAppCtrl)
{
    /* User Fault function */
    psAppCtrl->psState->Fault();
    
    /* if clear fault command flag */
    if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT_CLEAR) > 0 )
    {
	    /* Clear INIT_DONE, FAULT, FAULT_CLEAR flags */
	   psAppCtrl->uiCtrl &= ~(SM_CTRL_INIT_DONE | SM_CTRL_FAULT | SM_CTRL_FAULT_CLEAR);
    	
        /* User Fault to Init transition function */
        psAppCtrl->psTrans->FaultInit();

		/* Init state */
		psAppCtrl->eState = INIT;
    }
}

/***************************************************************************//*!
*
* @brief   Init state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void SM_StateInit(SM_APP_CTRL_T *psAppCtrl)
{
    /* User Init function */
    psAppCtrl->psState->Init();

	/* if fault flag */
	if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT) > 0)
	{
	    /* User Init to Fault transition function */
	    psAppCtrl->psTrans->InitFault();
	    
		/* Fault state */
		psAppCtrl->eState = FAULT;
	}
	/* if INIT_DONE flag */
	else if ((psAppCtrl->uiCtrl & SM_CTRL_INIT_DONE) > 0)
	{
	    /* Clear INIT_DONE, START_STOP, OM_CHANGE, STOP_ACK, RUN_ACK flags */
		psAppCtrl->uiCtrl &= ~(SM_CTRL_INIT_DONE | SM_CTRL_STOP | SM_CTRL_START | SM_CTRL_STOP_ACK | SM_CTRL_RUN_ACK);
		
 		/* User Init to Stop transition function */
		psAppCtrl->psTrans->InitStop();

		/* Stop state */
		psAppCtrl->eState = STOP;
	}
}

/***************************************************************************//*!
*
* @brief   STOP state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void SM_StateStop(SM_APP_CTRL_T *psAppCtrl)
{
    /* User Stop function */
    psAppCtrl->psState->Stop();

    /* if fault */
    if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT) > 0)
    {
	    /* User Stop to Fault transition function */
	    psAppCtrl->psTrans->StopFault();

		/* Fault state */
		psAppCtrl->eState = FAULT;
    }
	else if ((psAppCtrl->uiCtrl & SM_CTRL_START) > 0)
	{
		/* User Stop to Run transition function, user must set up the SM_CTRL_RUN_ACK
		flag to allow the RUN state */
		psAppCtrl->psTrans->StopRun();
		
		/* Clears the START command */
//		psAppCtrl->uiCtrl &= ~SM_CTRL_START;		

		if ((psAppCtrl->uiCtrl & SM_CTRL_RUN_ACK) > 0)
		{
			/* Clears the RUN_ACK flag */
			psAppCtrl->uiCtrl &= ~(SM_CTRL_RUN_ACK | SM_CTRL_START);
			
			/* Run state */
			psAppCtrl->eState = RUN;	
		}
	}
}

#pragma section CODES_IN_RAM begin
/***************************************************************************//*!
*
* @brief   RUN state
*
* @param   void
*
* @return  none
*
******************************************************************************/

static void SM_StateRun(SM_APP_CTRL_T *psAppCtrl)
{
    psAppCtrl->psState->Run();


	if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT) > 0)
    {
	    /* User Run to Fault transition function */
	    psAppCtrl->psTrans->RunFault();

		/* Fault state */
	    psAppCtrl->eState = FAULT; 	
    }
    else if ((psAppCtrl->uiCtrl & SM_CTRL_STOP) > 0)
	{
		/* User Run to Stop transition function, user must set up the SM_CTRL_STOP_ACK
		flag to allow the STOP state */
		psAppCtrl->psTrans->RunStop();
		
		/* Clears the STOP command */
//		psAppCtrl->uiCtrl &= ~SM_CTRL_STOP;

		if ((psAppCtrl->uiCtrl & SM_CTRL_STOP_ACK) > 0)
		{
			/* Clears the STOP_ACK flag */
			psAppCtrl->uiCtrl &= ~(SM_CTRL_STOP_ACK | SM_CTRL_STOP);
			
			/* Run state */
			psAppCtrl->eState = STOP;	
		}
	}
}
#pragma section CODES_IN_RAM end
/******************************************************************************
* Inline functions
******************************************************************************/




