/*
** ###################################################################
**     Processors:          MC56F83783AVLH
**                          MC56F83783VLH
**
**     Compiler:            CodeWarrior C/C++ for DSP M56800E
**     Reference manual:    Manual version TBD
**     Version:             rev. 0.1, 2019-09-23
**     Build:               b210615
**
**     Abstract:
**         Peripheral Access Layer for MC56F83783
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2021 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 0.1 (2019-09-23)
**         Initial version.
**
** ###################################################################
*/

/*!
 * @file MC56F83783.h
 * @version 0.1
 * @date 2019-09-23
 * @brief Peripheral Access Layer for MC56F83783
 *
 * Peripheral Access Layer for MC56F83783
 */

#ifndef _MC56F83783_H_
#define _MC56F83783_H_ /**< Symbol preventing repeated inclusion */

/** Memory map major version (memory maps with equal major version number are
 * compatible) */
#define MCU_MEM_MAP_VERSION 0x0000U
/** Memory map minor version */
#define MCU_MEM_MAP_VERSION_MINOR 0x0001U

/* ----------------------------------------------------------------------------
   -- Interrupt IRQ numbers
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Interrupt_IRQ_numbers Interrupt IRQ numbers
 * @{
 */

/** Interrupt Number Definitions */
#define NUMBER_OF_INT_IRQ 103 /**< Number of interrupts in the IRQ table */

typedef enum IRQn
{

    /* Device specific interrupts */
    STPCNT_IRQn            = 0,   /**< STPCNT interrupt */
    BKPT_IRQn              = 1,   /**< BKPT interrupt */
    TRBUF_IRQn             = 2,   /**< TRBUF interrupt */
    TX_REG_IRQn            = 3,   /**< TX_REG interrupt */
    RX_REG_IRQn            = 4,   /**< RX_REG interrupt */
    BUS_ERR_IRQn           = 5,   /**< BUS_ERR interrupt */
    Reserved6_IRQn         = 6,   /**< Reserved interrupt */
    Reserved7_IRQn         = 7,   /**< Reserved interrupt */
    Reserved8_IRQn         = 8,   /**< Reserved interrupt */
    XBARA_IRQn             = 9,   /**< XBARA interrupt */
    LVI1_IRQn              = 10,  /**< LVI1 interrupt */
    OCCS_IRQn              = 11,  /**< OCCS interrupt */
    TMRB_3_IRQn            = 12,  /**< TMRB_3 interrupt */
    TMRB_2_IRQn            = 13,  /**< TMRB_2 interrupt */
    TMRB_1_IRQn            = 14,  /**< TMRB_1 interrupt */
    TMRB_0_IRQn            = 15,  /**< TMRB_0 interrupt */
    TMRA_3_IRQn            = 16,  /**< TMRA_3 interrupt */
    TMRA_2_IRQn            = 17,  /**< TMRA_2 interrupt */
    TMRA_1_IRQn            = 18,  /**< TMRA_1 interrupt */
    TMRA_0_IRQn            = 19,  /**< TMRA_0 interrupt */
    ADC12_CC1_IRQn         = 20,  /**< ADC12_CC1 interrupt */
    ADC12_CC0_IRQn         = 21,  /**< ADC12_CC0 interrupt */
    ADC12_ERR_IRQn         = 22,  /**< ADC12_ERR interrupt */
    DMA_ERR_IRQn           = 23,  /**< DMA_ERR interrupt */
    DMA3_IRQn              = 24,  /**< DMA3 interrupt */
    DMA2_IRQn              = 25,  /**< DMA2 interrupt */
    DMA1_IRQn              = 26,  /**< DMA1 interrupt */
    DMA0_IRQn              = 27,  /**< DMA0 interrupt */
    CAN_MB_OR_IRQn         = 28,  /**< CAN_MB_OR interrupt */
    CAN_BUS_OFF_IRQn       = 29,  /**< CAN_BUS_OFF interrupt */
    CAN_ERROR_IRQn         = 30,  /**< CAN_ERROR interrupt */
    CAN_TX_WARN_IRQn       = 31,  /**< CAN_TX_WARN interrupt */
    CAN_RX_WARN_IRQn       = 32,  /**< CAN_RX_WARN interrupt */
    CAN_WAKEUP_IRQn        = 33,  /**< CAN_WAKEUP interrupt */
    Reserved34_IRQn        = 34,  /**< Reserved interrupt */
    Reserved35_IRQn        = 35,  /**< Reserved interrupt */
    Reserved36_IRQn        = 36,  /**< Reserved interrupt */
    Reserved37_IRQn        = 37,  /**< Reserved interrupt */
    QSCI1_RERR_IRQn        = 38,  /**< QSCI1_RERR interrupt */
    QSCI1_RCV_IRQn         = 39,  /**< QSCI1_RCV interrupt */
    QSCI1_TRIDLE_IRQn      = 40,  /**< QSCI1_TRIDLE interrupt */
    QSCI1_TDRE_IRQn        = 41,  /**< QSCI1_TDRE interrupt */
    QSCI0_RERR_IRQn        = 42,  /**< QSCI0_RERR interrupt */
    QSCI0_RCV_IRQn         = 43,  /**< QSCI0_RCV interrupt */
    QSCI0_TRIDLE_IRQn      = 44,  /**< QSCI0_TRIDLE interrupt */
    QSCI0_TDRE_IRQn        = 45,  /**< QSCI0_TDRE interrupt */
    Reserved46_IRQn        = 46,  /**< Reserved interrupt */
    Reserved47_IRQn        = 47,  /**< Reserved interrupt */
    Reserved48_IRQn        = 48,  /**< Reserved interrupt */
    Reserved49_IRQn        = 49,  /**< Reserved interrupt */
    QSPI0_XMIT_IRQn        = 50,  /**< QSPI0_XMIT interrupt */
    QSPI0_RCV_IRQn         = 51,  /**< QSPI0_RCV interrupt */
    I2C1_IRQn              = 52,  /**< I2C1 interrupt */
    I2C0_IRQn              = 53,  /**< I2C0 interrupt */
    Reserved54_IRQn        = 54,  /**< Reserved interrupt */
    Reserved55_IRQn        = 55,  /**< Reserved interrupt */
    Reserved56_IRQn        = 56,  /**< Reserved interrupt */
    eFlexPWMB_FAULT_IRQn   = 57,  /**< eFlexPWMB_FAULT interrupt */
    eFlexPWMB_RERR_IRQn    = 58,  /**< eFlexPWMB_RERR interrupt */
    eFlexPWMB_CAP_IRQn     = 59,  /**< eFlexPWMB_CAP interrupt */
    eFlexPWMB_RELOAD3_IRQn = 60,  /**< eFlexPWMB_RELOAD3 interrupt */
    eFlexPWMB_CMP3_IRQn    = 61,  /**< eFlexPWMB_CMP3 interrupt */
    eFlexPWMB_RELOAD2_IRQn = 62,  /**< eFlexPWMB_RELOAD2 interrupt */
    eFlexPWMB_CMP2_IRQn    = 63,  /**< eFlexPWMB_CMP2 interrupt */
    eFlexPWMB_RELOAD1_IRQn = 64,  /**< eFlexPWMB_RELOAD1 interrupt */
    eFlexPWMB_CMP1_IRQn    = 65,  /**< eFlexPWMB_CMP1 interrupt */
    eFlexPWMB_RELOAD0_IRQn = 66,  /**< eFlexPWMB_RELOAD0 interrupt */
    eFlexPWMB_CMP0_IRQn    = 67,  /**< eFlexPWMB_CMP0 interrupt */
    eFlexPWMA_FAULT_IRQn   = 68,  /**< eFlexPWMA_FAULT interrupt */
    eFlexPWMA_RERR_IRQn    = 69,  /**< eFlexPWMA_RERR interrupt */
    eFlexPWMA_CAP_IRQn     = 70,  /**< eFlexPWMA_CAP interrupt */
    eFlexPWMA_RELOAD3_IRQn = 71,  /**< eFlexPWMA_RELOAD3 interrupt */
    eFlexPWMA_CMP3_IRQn    = 72,  /**< eFlexPWMA_CMP3 interrupt */
    eFlexPWMA_RELOAD2_IRQn = 73,  /**< eFlexPWMA_RELOAD2 interrupt */
    eFlexPWMA_CMP2_IRQn    = 74,  /**< eFlexPWMA_CMP2 interrupt */
    eFlexPWMA_RELOAD1_IRQn = 75,  /**< eFlexPWMA_RELOAD1 interrupt */
    eFlexPWMA_CMP1_IRQn    = 76,  /**< eFlexPWMA_CMP1 interrupt */
    eFlexPWMA_RELOAD0_IRQn = 77,  /**< eFlexPWMA_RELOAD0 interrupt */
    eFlexPWMA_CMP0_IRQn    = 78,  /**< eFlexPWMA_CMP0 interrupt */
    FTFE_RDCOL_IRQn        = 79,  /**< FTFE_RDCOL interrupt */
    FTFE_CC_IRQn           = 80,  /**< FTFE_CC interrupt */
    CMPD_IRQn              = 81,  /**< CMPD interrupt */
    CMPC_IRQn              = 82,  /**< CMPC interrupt */
    CMPB_IRQn              = 83,  /**< CMPB interrupt */
    CMPA_IRQn              = 84,  /**< CMPA interrupt */
    PIT1_ROLLOVR_IRQn      = 85,  /**< PIT1_ROLLOVR interrupt */
    PIT0_ROLLOVR_IRQn      = 86,  /**< PIT0_ROLLOVR interrupt */
    FTFE_DFD_IRQn          = 87,  /**< FTFE_DFD interrupt */
    Reserved88_IRQn        = 88,  /**< Reserved interrupt */
    Reserved89_IRQn        = 89,  /**< Reserved interrupt */
    Reserved90_IRQn        = 90,  /**< Reserved interrupt */
    Reserved91_IRQn        = 91,  /**< Reserved interrupt */
    GPIOG_IRQn             = 92,  /**< GPIOG interrupt */
    GPIOF_IRQn             = 93,  /**< GPIOF interrupt */
    GPIOE_IRQn             = 94,  /**< GPIOE interrupt */
    GPIOD_IRQn             = 95,  /**< GPIOD interrupt */
    GPIOC_IRQn             = 96,  /**< GPIOC interrupt */
    GPIOB_IRQn             = 97,  /**< GPIOB interrupt */
    GPIOA_IRQn             = 98,  /**< GPIOA interrupt */
    COP_INT_IRQn           = 99,  /**< COP_INT interrupt */
    EWM_INT_IRQn           = 100, /**< EWM_INT interrupt */
    Reserved101_IRQn       = 101, /**< Reserved interrupt */
    Reserved102_IRQn       = 102  /**< Reserved interrupt */
} IRQn_Type;

/*!
 * @}
 */ /* end of group Interrupt_IRQ_numbers */

/* ----------------------------------------------------------------------------
   -- Includes and Typedefs
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Includes_And_Typedes Includes and Typedefs
 * @{
 */

#include <stdint.h>

#define __IO volatile
#define __I  const volatile
#define __O  volatile

#ifdef __cplusplus
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;
#endif

/*!
 * @}
 */ /* end of group Includes_And_Typedes */

/* ----------------------------------------------------------------------------
   -- Mapping Information
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Mapping_Information Mapping Information
 * @{
 */

/** Mapping Information */
/*! @brief CMP input mux control */
typedef enum _cmp_input_mux
{
    kCMP_CMPAInputMux0GPIOA1      = 0U, /**< CMPA input mux 0 from GPIOA1 */
    kCMP_CMPAInputMux1GPIOA2      = 1U, /**< CMPA input mux 1 from GPIOA2 */
    kCMP_CMPAInputMux2GPIOA3      = 2U, /**< CMPA input mux 2 from GPIOA3 */
    kCMP_CMPAInputMux3GPIOA0      = 3U, /**< CMPA input mux 3 from GPIOA0 */
    kCMP_CMPAInputMux4DACA        = 4U, /**< CMPA input mux 4 from DACA */
    kCMP_CMPAInputMux5GPIOC6      = 5U, /**< CMPA input mux 5 from GPIOC6 */
    kCMP_CMPAInputMux6DACB        = 6U, /**< CMPA input mux 6 from DACB */
    kCMP_CMPAInputMux7InternalDAC = 7U, /**< CMPA input mux 7 from CMP internal DAC */
    kCMP_CMPBInputMux0GPIOB1      = 0U, /**< CMPB input mux 0 from GPIOB1 */
    kCMP_CMPBInputMux1GPIOB6      = 1U, /**< CMPB input mux 1 from GPIOB6 */
    kCMP_CMPBInputMux2GPIOB7      = 2U, /**< CMPB input mux 2 from GPIOB7 */
    kCMP_CMPBInputMux3GPIOB0      = 3U, /**< CMPB input mux 3 from GPIOB0 */
    kCMP_CMPBInputMux4DACA        = 4U, /**< CMPB input mux 4 from DACA */
    kCMP_CMPBInputMux5GPIOC6      = 5U, /**< CMPB input mux 5 from GPIOC6 */
    kCMP_CMPBInputMux6DACB        = 6U, /**< CMPB input mux 6 from DACB */
    kCMP_CMPBInputMux7InternalDAC = 7U, /**< CMPB input mux 7 from CMP internal DAC */
    kCMP_CMPCInputMux0GPIOB3      = 0U, /**< CMPC input mux 0 from GPIOB3 */
    kCMP_CMPCInputMux1GPIOB4      = 1U, /**< CMPC input mux 1 from GPIOB4 */
    kCMP_CMPCInputMux2GPIOB5      = 2U, /**< CMPC input mux 2 from GPIOB5 */
    kCMP_CMPCInputMux3GPIOB2      = 3U, /**< CMPC input mux 3 from GPIOB2 */
    kCMP_CMPCInputMux4DACA        = 4U, /**< CMPC input mux 4 from DACA */
    kCMP_CMPCInputMux5GPIOC6      = 5U, /**< CMPC input mux 5 from GPIOC6 */
    kCMP_CMPCInputMux6DACB        = 6U, /**< CMPC input mux 6 from DACB */
    kCMP_CMPCInputMux7InternalDAC = 7U, /**< CMPC input mux 7 from CMP internal DAC */
    kCMP_CMPDInputMux0GPIOA4      = 0U, /**< CMPD input mux 0 from GPIOA4 */
    kCMP_CMPDInputMux1GPIOA8      = 1U, /**< CMPD input mux 1 from GPIOA8 */
    kCMP_CMPDInputMux2GPIOA9      = 2U, /**< CMPD input mux 2 from GPIOA9 */
    kCMP_CMPDInputMux3GPIOA10     = 3U, /**< CMPD input mux 3 from GPIOA10 */
    kCMP_CMPDInputMux4DACA        = 4U, /**< CMPD input mux 4 from DACA */
    kCMP_CMPDInputMux5GPIOC6      = 5U, /**< CMPD input mux 5 from GPIOC6 */
    kCMP_CMPDInputMux6DACB        = 6U, /**< CMPD input mux 6 from DACB */
    kCMP_CMPDInputMux7InternalDAC = 7U, /**< CMPD input mux 7 from CMP internal DAC */
} cmp_input_mux_t;

/*!
 * @addtogroup edma_request
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Structure for the DMA hardware request
 *
 * Defines the structure for the DMA hardware request collections. The user can configure the
 * hardware request into DMAMUX to trigger the DMA transfer accordingly. The index
 * of the hardware request varies according  to the to SoC.
 */
typedef enum _dma_request_source
{
    kDmaRequestMuxDisable                   = 0 | 0x100U,  /**< DMAMUX TriggerDisabled */
    kDmaRequestMuxSCI0RF                    = 2 | 0x100U,  /**< SCI0 Receive Full */
    kDmaRequestMuxSCI0TE                    = 3 | 0x100U,  /**< SCI0 Transmit Empty */
    kDmaRequestMuxSCI1RF                    = 4 | 0x100U,  /**< SCI1 Receive Full */
    kDmaRequestMuxSCI1TE                    = 5 | 0x100U,  /**< SCI1 Transmit Empty */
    kDmaRequestMuxSPI0RF                    = 12 | 0x100U, /**< SPI0 Receive Full */
    kDmaRequestMuxSPI0TE                    = 13 | 0x100U, /**< SPI0 Transmit Empty */
    kDmaRequestMuxCANFD                     = 16 | 0x100U, /**< CAN Msg Rx FIFO Full */
    kDmaRequestMuxI2C0                      = 18 | 0x100U, /**< I2C0 DMA Req */
    kDmaRequestMuxI2C1                      = 19 | 0x100U, /**< I2C1 DMA Req */
    kDmaRequestMuxFlexPWMACaptureSub0       = 22 | 0x100U, /**< PWMA Capture sub-module 0 */
    kDmaRequestMuxFlexPWMACaptureSub1       = 23 | 0x100U, /**< PWMA Capture sub-module 1 */
    kDmaRequestMuxFlexPWMACaptureSub2       = 24 | 0x100U, /**< PWMA Capture sub-module 2 */
    kDmaRequestMuxFlexPWMACaptureSub3       = 25 | 0x100U, /**< PWMA Capture sub-module 3 */
    kDmaRequestMuxFlexPWMAValueWriteSub0123 = 26 | 0x100U, /**< PWMA Value Write sub-module 0, 1, 2, 3 */
    kDmaRequestMuxFlexPWMBCaptureSub0       = 27 | 0x100U, /**< PWMB Capture sub-module 0 */
    kDmaRequestMuxFlexPWMBCaptureSub1       = 28 | 0x100U, /**< PWMB Capture sub-module 1 */
    kDmaRequestMuxFlexPWMBCaptureSub2       = 29 | 0x100U, /**< PWMB Capture sub-module 2 */
    kDmaRequestMuxFlexPWMBCaptureSub3       = 30 | 0x100U, /**< PWMB Capture sub-module 3 */
    kDmaRequestMuxFlexPWMBValueSub0123      = 31 | 0x100U, /**< PWMB Value sub-module 0, 1, 2, 3 */
    kDmaRequestMuxTMRA0CaptureCompare1      = 32 | 0x100U, /**< TMRA0 Capture or Compare 1 */
    kDmaRequestMuxTMRA0Compare2             = 33 | 0x100U, /**< TMRA0 Compare 2 */
    kDmaRequestMuxTMRA1CaptureCompare1      = 34 | 0x100U, /**< TMRA1 Capture or Compare 1 */
    kDmaRequestMuxTMRA1Compare2             = 35 | 0x100U, /**< TMRA1 Compare 2 */
    kDmaRequestMuxTMRA2CaptureCompare1      = 36 | 0x100U, /**< TMRA2 Capture or Compare 1 */
    kDmaRequestMuxTMRA2Compare2             = 37 | 0x100U, /**< TMRA2 Compare 2 */
    kDmaRequestMuxTMRA3CaptureCompare1      = 38 | 0x100U, /**< TMRA3 Capture or Compare 1 */
    kDmaRequestMuxTMRA3Compare2             = 39 | 0x100U, /**< TMRA3 Compare 2 */
    kDmaRequestMuxTMRB0CaptureCompare1      = 40 | 0x100U, /**< TMRB0 Capture or Compare 1 */
    kDmaRequestMuxTMRB0Compare2             = 41 | 0x100U, /**< TMRB0 Compare 2 */
    kDmaRequestMuxTMRB1CaptureCompare1      = 42 | 0x100U, /**< TMRB1 Capture or Compare 1 */
    kDmaRequestMuxTMRB1Compare2             = 43 | 0x100U, /**< TMRB1 Compare 2 */
    kDmaRequestMuxTMRB2CaptureCompare1      = 44 | 0x100U, /**< TMRB2 Capture or Compare 1 */
    kDmaRequestMuxTMRB2Compare2             = 45 | 0x100U, /**< TMRB2 Compare 2 */
    kDmaRequestMuxTMRB3CaptureCompare1      = 46 | 0x100U, /**< TMRB3 Capture or Compare 1 */
    kDmaRequestMuxTMRB3Compare2             = 47 | 0x100U, /**< TMRB3 Compare 2 */
    kDmaRequestMuxADCAES                    = 48 | 0x100U, /**< ADCA End of Scan */
    kDmaRequestMuxADCBES                    = 49 | 0x100U, /**< ADCB End of Scan */
    kDmaRequestMuxDACAFIFO                  = 50 | 0x100U, /**< DACA FIFO Water Mark */
    kDmaRequestMuxDACBFIFO                  = 51 | 0x100U, /**< DACB FIFO Water Mark */
    kDmaRequestMuxCMPA                      = 52 | 0x100U, /**< CMPA Toggle */
    kDmaRequestMuxCMPB                      = 53 | 0x100U, /**< CMPB Toggle */
    kDmaRequestMuxCMPC                      = 54 | 0x100U, /**< CMPC Toggle */
    kDmaRequestMuxCMPD                      = 55 | 0x100U, /**< CMPD Toggle */
    kDmaRequestMuxXBAR_DSC0                 = 56 | 0x100U, /**< XBAR DMA Req 0 */
    kDmaRequestMuxXBAR_DSC1                 = 57 | 0x100U, /**< XBAR DMA Req 1 */
    kDmaRequestMuxXBAR_DSC2                 = 58 | 0x100U, /**< XBAR DMA Req 2 */
    kDmaRequestMuxXBAR_DSC3                 = 59 | 0x100U, /**< XBAR DMA Req 3 */
    kDmaRequestMuxAlwaysOn60                = 60 | 0x100U, /**< Always on 60 */
    kDmaRequestMuxAlwaysOn61                = 61 | 0x100U, /**< Always on 61 */
    kDmaRequestMuxAlwaysOn62                = 62 | 0x100U, /**< Always on 62 */
    kDmaRequestMuxAlwaysOn63                = 63 | 0x100U, /**< Always on 63 */
} dma_request_source_t;

/* @} */

/*!
 * @addtogroup Interrupt_vector_numbers
 * @{
 */

/*!
 * @brief Structure for the vector table
 *
 * The vector table refer to interrupt vector table in RM
 * Generally used for fast interrupt setting, checking current interrupt type.
 */
typedef enum _vector_type
{
    kHW_RESET_VECTORn          = 0,   /**< Vector no.0 */
    kCOP_RESET_VECTORn         = 1,   /**< Vector no.1 */
    kILLEGAL_OP_VECTORn        = 2,   /**< Vector no.2 */
    kSWI3_VECTORn              = 3,   /**< Vector no.3 */
    kOVERFLOW_VECTORn          = 4,   /**< Vector no.4 */
    kMISALIGNED_VECTORn        = 5,   /**< Vector no.5 */
    kSTPCNT_VECTORn            = 6,   /**< Vector no.6 */
    kBKPT_VECTORn              = 7,   /**< Vector no.7 */
    kTRBUF_VECTORn             = 8,   /**< Vector no.8 */
    kTX_REG_VECTORn            = 9,   /**< Vector no.9 */
    kRX_REG_VECTORn            = 10,  /**< Vector no.10 */
    kBUS_ERR_VECTORn           = 11,  /**< Vector no.11 */
    kReserved12_VECTORn        = 12,  /**< Vector no.12 */
    kReserved13_VECTORn        = 13,  /**< Vector no.13 */
    kReserved14_VECTORn        = 14,  /**< Vector no.14 */
    kSWI2_VECTORn              = 15,  /**< Vector no.15 */
    kSWI1_VECTORn              = 16,  /**< Vector no.16 */
    kSWI0_VECTORn              = 17,  /**< Vector no.17 */
    kXBARA_VECTORn             = 18,  /**< Vector no.18 */
    kLVI1_VECTORn              = 19,  /**< Vector no.19 */
    kOCCS_VECTORn              = 20,  /**< Vector no.20 */
    kTMRB_3_VECTORn            = 21,  /**< Vector no.21 */
    kTMRB_2_VECTORn            = 22,  /**< Vector no.22 */
    kTMRB_1_VECTORn            = 23,  /**< Vector no.23 */
    kTMRB_0_VECTORn            = 24,  /**< Vector no.24 */
    kTMRA_3_VECTORn            = 25,  /**< Vector no.25 */
    kTMRA_2_VECTORn            = 26,  /**< Vector no.26 */
    kTMRA_1_VECTORn            = 27,  /**< Vector no.27 */
    kTMRA_0_VECTORn            = 28,  /**< Vector no.28 */
    kADC12_CC1_VECTORn         = 29,  /**< Vector no.29 */
    kADC12_CC0_VECTORn         = 30,  /**< Vector no.30 */
    kADC12_ERR_VECTORn         = 31,  /**< Vector no.31 */
    kDMA_ERR_VECTORn           = 32,  /**< Vector no.32 */
    kDMA3_VECTORn              = 33,  /**< Vector no.33 */
    kDMA2_VECTORn              = 34,  /**< Vector no.34 */
    kDMA1_VECTORn              = 35,  /**< Vector no.35 */
    kDMA0_VECTORn              = 36,  /**< Vector no.36 */
    kCAN_MB_OR_VECTORn         = 37,  /**< Vector no.37 */
    kCAN_BUS_OFF_VECTORn       = 38,  /**< Vector no.38 */
    kCAN_ERROR_VECTORn         = 39,  /**< Vector no.39 */
    kCAN_TX_WARN_VECTORn       = 40,  /**< Vector no.40 */
    kCAN_RX_WARN_VECTORn       = 41,  /**< Vector no.41 */
    kCAN_WAKEUP_VECTORn        = 42,  /**< Vector no.42 */
    kReserved43_VECTORn        = 43,  /**< Vector no.43 */
    kReserved44_VECTORn        = 44,  /**< Vector no.44 */
    kReserved45_VECTORn        = 45,  /**< Vector no.45 */
    kReserved46_VECTORn        = 46,  /**< Vector no.46 */
    kQSCI1_RERR_VECTORn        = 47,  /**< Vector no.47 */
    kQSCI1_RCV_VECTORn         = 48,  /**< Vector no.48 */
    kQSCI1_TRIDLE_VECTORn      = 49,  /**< Vector no.49 */
    kQSCI1_TDRE_VECTORn        = 50,  /**< Vector no.50 */
    kQSCI0_RERR_VECTORn        = 51,  /**< Vector no.51 */
    kQSCI0_RCV_VECTORn         = 52,  /**< Vector no.52 */
    kQSCI0_TRIDLE_VECTORn      = 53,  /**< Vector no.53 */
    kQSCI0_TDRE_VECTORn        = 54,  /**< Vector no.54 */
    kReserved55_VECTORn        = 55,  /**< Vector no.55 */
    kReserved56_VECTORn        = 56,  /**< Vector no.56 */
    kReserved57_VECTORn        = 57,  /**< Vector no.57 */
    kReserved58_VECTORn        = 58,  /**< Vector no.58 */
    kQSPI0_XMIT_VECTORn        = 59,  /**< Vector no.59 */
    kQSPI0_RCV_VECTORn         = 60,  /**< Vector no.60 */
    kI2C1_VECTORn              = 61,  /**< Vector no.61 */
    kI2C0_VECTORn              = 62,  /**< Vector no.62 */
    kReserved63_VECTORn        = 63,  /**< Vector no.63 */
    kReserved64_VECTORn        = 64,  /**< Vector no.64 */
    kReserved65_VECTORn        = 65,  /**< Vector no.65 */
    keFlexPWMB_FAULT_VECTORn   = 66,  /**< Vector no.66 */
    keFlexPWMB_RERR_VECTORn    = 67,  /**< Vector no.67 */
    keFlexPWMB_CAP_VECTORn     = 68,  /**< Vector no.68 */
    keFlexPWMB_RELOAD3_VECTORn = 69,  /**< Vector no.69 */
    keFlexPWMB_CMP3_VECTORn    = 70,  /**< Vector no.70 */
    keFlexPWMB_RELOAD2_VECTORn = 71,  /**< Vector no.71 */
    keFlexPWMB_CMP2_VECTORn    = 72,  /**< Vector no.72 */
    keFlexPWMB_RELOAD1_VECTORn = 73,  /**< Vector no.73 */
    keFlexPWMB_CMP1_VECTORn    = 74,  /**< Vector no.74 */
    keFlexPWMB_RELOAD0_VECTORn = 75,  /**< Vector no.75 */
    keFlexPWMB_CMP0_VECTORn    = 76,  /**< Vector no.76 */
    keFlexPWMA_FAULT_VECTORn   = 77,  /**< Vector no.77 */
    keFlexPWMA_RERR_VECTORn    = 78,  /**< Vector no.78 */
    keFlexPWMA_CAP_VECTORn     = 79,  /**< Vector no.79 */
    keFlexPWMA_RELOAD3_VECTORn = 80,  /**< Vector no.80 */
    keFlexPWMA_CMP3_VECTORn    = 81,  /**< Vector no.81 */
    keFlexPWMA_RELOAD2_VECTORn = 82,  /**< Vector no.82 */
    keFlexPWMA_CMP2_VECTORn    = 83,  /**< Vector no.83 */
    keFlexPWMA_RELOAD1_VECTORn = 84,  /**< Vector no.84 */
    keFlexPWMA_CMP1_VECTORn    = 85,  /**< Vector no.85 */
    keFlexPWMA_RELOAD0_VECTORn = 86,  /**< Vector no.86 */
    keFlexPWMA_CMP0_VECTORn    = 87,  /**< Vector no.87 */
    kFTFE_RDCOL_VECTORn        = 88,  /**< Vector no.88 */
    kFTFE_CC_VECTORn           = 89,  /**< Vector no.89 */
    kCMPD_VECTORn              = 90,  /**< Vector no.90 */
    kCMPC_VECTORn              = 91,  /**< Vector no.91 */
    kCMPB_VECTORn              = 92,  /**< Vector no.92 */
    kCMPA_VECTORn              = 93,  /**< Vector no.93 */
    kPIT1_ROLLOVR_VECTORn      = 94,  /**< Vector no.94 */
    kPIT0_ROLLOVR_VECTORn      = 95,  /**< Vector no.95 */
    kFTFE_DFD_VECTORn          = 96,  /**< Vector no.96 */
    kReserved97_VECTORn        = 97,  /**< Vector no.97 */
    kReserved98_VECTORn        = 98,  /**< Vector no.98 */
    kReserved99_VECTORn        = 99,  /**< Vector no.99 */
    kReserved100_VECTORn       = 100, /**< Vector no.100 */
    kGPIOG_VECTORn             = 101, /**< Vector no.101 */
    kGPIOF_VECTORn             = 102, /**< Vector no.102 */
    kGPIOE_VECTORn             = 103, /**< Vector no.103 */
    kGPIOD_VECTORn             = 104, /**< Vector no.104 */
    kGPIOC_VECTORn             = 105, /**< Vector no.105 */
    kGPIOB_VECTORn             = 106, /**< Vector no.106 */
    kGPIOA_VECTORn             = 107, /**< Vector no.107 */
    kCOP_INT_VECTORn           = 108, /**< Vector no.108 */
    kEWM_INT_VECTORn           = 109, /**< Vector no.109 */
    kSWILP_VECTORn             = 110, /**< Vector no.110 */
    kReserved111_VECTORn       = 111, /**< Vector no.111 */
} vector_type_t;

/* @} */

/*!
 * @addtogroup Pin_Mux
 * @{
 */

/*!
 * @brief Structure for the pin mux table
 *
 * The pin mux table is soc specificed.
 * Enumeration member bitfield:
 *   BIT0~BIT1: peripheral function selector.
 *   BIT4~BIT7: pin index.
 *   BIT8~BIT11: port index.
 *   Other BITs: 0, reserved.
 */
typedef enum _gpio_peripheral_mux
{
    kGPIO_Peri_A0_ANA0_and_CMPA_IN3 = 0x0000, /**< GPIO_A0, Pin No. 13 */
    kGPIO_Peri_A0_CMPC_O            = 0x0001, /**< GPIO_A0, Pin No. 13 */
    kGPIO_Peri_B1_ANB1_and_CMPB_IN0 = 0x0110, /**< GPIO_B1, Pin No. 25 */
    kGPIO_Peri_B1_DACB_O            = 0x0111, /**< GPIO_B1, Pin No. 25 */
    kGPIO_Peri_C0_EXTAL             = 0x0200, /**< GPIO_C0, Pin No. 3 */
    kGPIO_Peri_C0_CLKIN0            = 0x0201, /**< GPIO_C0, Pin No. 3 */
    kGPIO_Peri_C2_TXD0              = 0x0220, /**< GPIO_C2, Pin No. 5 */
    kGPIO_Peri_C2_TB0               = 0x0221, /**< GPIO_C2, Pin No. 5 */
    kGPIO_Peri_C2_XB_IN2            = 0x0222, /**< GPIO_C2, Pin No. 5 */
    kGPIO_Peri_C2_CLKO0             = 0x0223, /**< GPIO_C2, Pin No. 5 */
    kGPIO_Peri_C3_TA0               = 0x0230, /**< GPIO_C3, Pin No. 7 */
    kGPIO_Peri_C3_CMPA_O            = 0x0231, /**< GPIO_C3, Pin No. 7 */
    kGPIO_Peri_C3_RXD0              = 0x0232, /**< GPIO_C3, Pin No. 7 */
    kGPIO_Peri_C3_CLKIN1            = 0x0233, /**< GPIO_C3, Pin No. 7 */
    kGPIO_Peri_C4_TA1               = 0x0240, /**< GPIO_C4, Pin No. 8 */
    kGPIO_Peri_C4_CMPB_O            = 0x0241, /**< GPIO_C4, Pin No. 8 */
    kGPIO_Peri_C4_XB_IN8            = 0x0242, /**< GPIO_C4, Pin No. 8 */
    kGPIO_Peri_C4_EWM_OUT_B         = 0x0243, /**< GPIO_C4, Pin No. 8 */
    kGPIO_Peri_C5_DACA_O            = 0x0250, /**< GPIO_C5, Pin No. 18 */
    kGPIO_Peri_C5_XB_IN7            = 0x0251, /**< GPIO_C5, Pin No. 18 */
    kGPIO_Peri_C6_TA2               = 0x0260, /**< GPIO_C6, Pin No. 31 */
    kGPIO_Peri_C6_XB_IN3            = 0x0261, /**< GPIO_C6, Pin No. 31 */
    kGPIO_Peri_C6_CMP_REF           = 0x0262, /**< GPIO_C6, Pin No. 31 */
    kGPIO_Peri_C6_SS0_B             = 0x0263, /**< GPIO_C6, Pin No. 31 */
    kGPIO_Peri_C7_SS0_B             = 0x0270, /**< GPIO_C7, Pin No. 32 */
    kGPIO_Peri_C7_TXD0              = 0x0271, /**< GPIO_C7, Pin No. 32 */
    kGPIO_Peri_C7_XB_IN8            = 0x0272, /**< GPIO_C7, Pin No. 32 */
    kGPIO_Peri_C7_XB_OUT6           = 0x0273, /**< GPIO_C7, Pin No. 32 */
    kGPIO_Peri_C8_MISO0             = 0x0280, /**< GPIO_C8, Pin No. 33 */
    kGPIO_Peri_C8_RXD0              = 0x0281, /**< GPIO_C8, Pin No. 33 */
    kGPIO_Peri_C8_XB_IN9            = 0x0282, /**< GPIO_C8, Pin No. 33 */
    kGPIO_Peri_C9_SCLK0             = 0x0290, /**< GPIO_C9, Pin No. 34 */
    kGPIO_Peri_C9_XB_IN4            = 0x0291, /**< GPIO_C9, Pin No. 34 */
    kGPIO_Peri_C9_TXD0              = 0x0292, /**< GPIO_C9, Pin No. 34 */
    kGPIO_Peri_C9_XB_OUT8           = 0x0293, /**< GPIO_C9, Pin No. 34 */
    kGPIO_Peri_C10_MOSI0            = 0x02A0, /**< GPIO_C10, Pin No. 35 */
    kGPIO_Peri_C10_XB_IN5           = 0x02A1, /**< GPIO_C10, Pin No. 35 */
    kGPIO_Peri_C10_MISO0            = 0x02A2, /**< GPIO_C10, Pin No. 35 */
    kGPIO_Peri_C10_XB_OUT9          = 0x02A3, /**< GPIO_C10, Pin No. 35 */
    kGPIO_Peri_C11_CANTX            = 0x02B0, /**< GPIO_C11, Pin No. 37 */
    kGPIO_Peri_C11_SCL1             = 0x02B1, /**< GPIO_C11, Pin No. 37 */
    kGPIO_Peri_C11_TXD1             = 0x02B2, /**< GPIO_C11, Pin No. 37 */
    kGPIO_Peri_C12_CANRX            = 0x02C0, /**< GPIO_C12, Pin No. 38 */
    kGPIO_Peri_C12_SDA1             = 0x02C1, /**< GPIO_C12, Pin No. 38 */
    kGPIO_Peri_C12_RXD1             = 0x02C2, /**< GPIO_C12, Pin No. 38 */
    kGPIO_Peri_C13_TA3              = 0x02D0, /**< GPIO_C13, Pin No. 49 */
    kGPIO_Peri_C13_XB_IN6           = 0x02D1, /**< GPIO_C13, Pin No. 49 */
    kGPIO_Peri_C13_EWM_OUT_B        = 0x02D2, /**< GPIO_C13, Pin No. 49 */
    kGPIO_Peri_C14_SDA0             = 0x02E0, /**< GPIO_C14, Pin No. 55 */
    kGPIO_Peri_C14_XB_OUT4          = 0x02E1, /**< GPIO_C14, Pin No. 55 */
    kGPIO_Peri_C14_PWMA_FAULT4      = 0x02E2, /**< GPIO_C14, Pin No. 55 */
    kGPIO_Peri_C15_SCL0             = 0x02F0, /**< GPIO_C15, Pin No. 56 */
    kGPIO_Peri_C15_XB_OUT5          = 0x02F1, /**< GPIO_C15, Pin No. 56 */
    kGPIO_Peri_C15_PWMA_FAULT5      = 0x02F2, /**< GPIO_C15, Pin No. 56 */
    kGPIO_Peri_E0_PWMA_0B           = 0x0400, /**< GPIO_E0, Pin No. 45 */
    kGPIO_Peri_E0_XB_OUT4           = 0x0403, /**< GPIO_E0, Pin No. 45 */
    kGPIO_Peri_E1_PWMA_0A           = 0x0410, /**< GPIO_E1, Pin No. 46 */
    kGPIO_Peri_E1_XB_OUT5           = 0x0413, /**< GPIO_E1, Pin No. 46 */
    kGPIO_Peri_E2_PWMA_1B           = 0x0420, /**< GPIO_E2, Pin No. 47 */
    kGPIO_Peri_E2_XB_OUT6           = 0x0423, /**< GPIO_E2, Pin No. 47 */
    kGPIO_Peri_E3_PWMA_1A           = 0x0430, /**< GPIO_E3, Pin No. 48 */
    kGPIO_Peri_E3_XB_OUT7           = 0x0433, /**< GPIO_E3, Pin No. 48 */
    kGPIO_Peri_E4_PWMA_2B           = 0x0440, /**< GPIO_E4, Pin No. 51 */
    kGPIO_Peri_E4_XB_IN2            = 0x0441, /**< GPIO_E4, Pin No. 51 */
    kGPIO_Peri_E4_XB_OUT8           = 0x0443, /**< GPIO_E4, Pin No. 51 */
    kGPIO_Peri_E5_PWMA_2A           = 0x0450, /**< GPIO_E5, Pin No. 52 */
    kGPIO_Peri_E5_XB_IN3            = 0x0451, /**< GPIO_E5, Pin No. 52 */
    kGPIO_Peri_E5_XB_OUT9           = 0x0453, /**< GPIO_E5, Pin No. 52 */
    kGPIO_Peri_E6_PWMA_3B           = 0x0460, /**< GPIO_E6, Pin No. 53 */
    kGPIO_Peri_E6_XB_IN4            = 0x0461, /**< GPIO_E6, Pin No. 53 */
    kGPIO_Peri_E6_PWMB_2B           = 0x0462, /**< GPIO_E6, Pin No. 53 */
    kGPIO_Peri_E6_XB_OUT10          = 0x0463, /**< GPIO_E6, Pin No. 53 */
    kGPIO_Peri_E7_PWMA_3A           = 0x0470, /**< GPIO_E7, Pin No. 54 */
    kGPIO_Peri_E7_XB_IN5            = 0x0471, /**< GPIO_E7, Pin No. 54 */
    kGPIO_Peri_E7_PWMB_2A           = 0x0472, /**< GPIO_E7, Pin No. 54 */
    kGPIO_Peri_E7_XB_OUT11          = 0x0473, /**< GPIO_E7, Pin No. 54 */
    kGPIO_Peri_F0_XB_IN6            = 0x0500, /**< GPIO_F0, Pin No. 36 */
    kGPIO_Peri_F0_TB2               = 0x0501, /**< GPIO_F0, Pin No. 36 */
    kGPIO_Peri_F1_CLKO1             = 0x0510, /**< GPIO_F1, Pin No. 50 */
    kGPIO_Peri_F1_XB_IN7            = 0x0511, /**< GPIO_F1, Pin No. 50 */
    kGPIO_Peri_F1_CMPD_O            = 0x0512, /**< GPIO_F1, Pin No. 50 */
    kGPIO_Peri_F2_SCL1              = 0x0520, /**< GPIO_F2, Pin No. 39 */
    kGPIO_Peri_F2_XB_OUT6           = 0x0521, /**< GPIO_F2, Pin No. 39 */
    kGPIO_Peri_F3_SDA1              = 0x0530, /**< GPIO_F3, Pin No. 40 */
    kGPIO_Peri_F3_XB_OUT7           = 0x0531, /**< GPIO_F3, Pin No. 40 */
    kGPIO_Peri_F4_TXD1              = 0x0540, /**< GPIO_F4, Pin No. 41 */
    kGPIO_Peri_F4_XB_OUT8           = 0x0541, /**< GPIO_F4, Pin No. 41 */
    kGPIO_Peri_F4_PWMA_0X           = 0x0542, /**< GPIO_F4, Pin No. 41 */
    kGPIO_Peri_F4_PWMA_FAULT6       = 0x0543, /**< GPIO_F4, Pin No. 41 */
    kGPIO_Peri_F5_RXD1              = 0x0550, /**< GPIO_F5, Pin No. 42 */
    kGPIO_Peri_F5_XB_OUT9           = 0x0551, /**< GPIO_F5, Pin No. 42 */
    kGPIO_Peri_F5_PWMA_1X           = 0x0552, /**< GPIO_F5, Pin No. 42 */
    kGPIO_Peri_F5_PWMA_FAULT7       = 0x0553, /**< GPIO_F5, Pin No. 42 */
    kGPIO_Peri_F6_TB2               = 0x0560, /**< GPIO_F6, Pin No. 58 */
    kGPIO_Peri_F6_PWMA_3X           = 0x0561, /**< GPIO_F6, Pin No. 58 */
    kGPIO_Peri_F6_PWMB_3X           = 0x0562, /**< GPIO_F6, Pin No. 58 */
    kGPIO_Peri_F6_XB_IN2            = 0x0563, /**< GPIO_F6, Pin No. 58 */
    kGPIO_Peri_F7_TB3               = 0x0570, /**< GPIO_F7, Pin No. 59 */
    kGPIO_Peri_F7_CMPC_O            = 0x0571, /**< GPIO_F7, Pin No. 59 */
    kGPIO_Peri_F7_XB_IN3            = 0x0573, /**< GPIO_F7, Pin No. 59 */
    kGPIO_Peri_F8_RXD0              = 0x0580, /**< GPIO_F8, Pin No. 6 */
    kGPIO_Peri_F8_TB1               = 0x0581, /**< GPIO_F8, Pin No. 6 */
    kGPIO_Peri_F8_CMPD_O            = 0x0582, /**< GPIO_F8, Pin No. 6 */
    kGPIO_Peri_F8_PWMA_2X           = 0x0583, /**< GPIO_F8, Pin No. 6 */
} gpio_peripheral_mux_t;

/* @} */

typedef enum _sim_xbar_input_pwm_index
{
    kSIM_XBARIN20Index = 0U,  /**< xbar input 20. */
    kSIM_XBARIN21Index = 1U,  /**< xbar input 21. */
    kSIM_XBARIN22Index = 2U,  /**< xbar input 22. */
    kSIM_XBARIN23Index = 3U,  /**< xbar input 23. */
    kSIM_XBARIN24Index = 4U,  /**< xbar input 24. */
    kSIM_XBARIN25Index = 5U,  /**< xbar input 25. */
    kSIM_XBARIN26Index = 6U,  /**< xbar input 26. */
    kSIM_XBARIN27Index = 7U,  /**< xbar input 27. */
    kSIM_XBARIN28Index = 8U,  /**< xbar input 28. */
    kSIM_XBARIN29Index = 9U,  /**< xbar input 29. */
    kSIM_XBARIN30Index = 10U, /**< xbar input 30. */
    kSIM_XBARIN31Index = 11U, /**< xbar input 31. */
    kSIM_XBARIN32Index = 12U, /**< xbar input 32. */
    kSIM_XBARIN33Index = 13U, /**< xbar input 33. */
    kSIM_XBARIN34Index = 14U, /**< xbar input 34. */
    kSIM_XBARIN35Index = 15U, /**< xbar input 35. */
} sim_xbar_input_pwm_index_t;

typedef enum _sim_xbar_input_pwm_selection
{
    kSIM_XbarIn20PWMA0MuxTrig0 = 0U, /**< xbar input 20, PWMA0_MUX_TRIG0. */
    kSIM_XbarIn20PWMB0OutTrig0 = 1U, /**< xbar input 20, PWMB0_OUT_TRIG0. */
    kSIM_XbarIn21PWMA0MuxTrig1 = 0U, /**< xbar input 21, PWMA0_MUX_TRIG1. */
    kSIM_XbarIn21PWMB0OutTrig1 = 1U, /**< xbar input 21, PWMB0_OUT_TRIG1. */
    kSIM_XbarIn22PWMA1MuxTrig0 = 0U, /**< xbar input 22, PWMA1_MUX_TRIG0. */
    kSIM_XbarIn22PWMB1OutTrig0 = 1U, /**< xbar input 22, PWMB1_OUT_TRIG0. */
    kSIM_XbarIn23PWMA1MuxTrig1 = 0U, /**< xbar input 23, PWMA1_MUX_TRIG1. */
    kSIM_XbarIn23PWMB1OutTrig1 = 1U, /**< xbar input 23, PWMB1_OUT_TRIG1. */
    kSIM_XbarIn24PWMA2MuxTrig0 = 0U, /**< xbar input 24, PWMA2_MUX_TRIG0. */
    kSIM_XbarIn24PWMB2OutTrig0 = 1U, /**< xbar input 24, PWMB2_OUT_TRIG0. */
    kSIM_XbarIn25PWMA2MuxTrig1 = 0U, /**< xbar input 25, PWMA2_MUX_TRIG1. */
    kSIM_XbarIn25PWMB2OutTrig1 = 1U, /**< xbar input 25, PWMB2_OUT_TRIG1. */
    kSIM_XbarIn26PWMA3MuxTrig0 = 0U, /**< xbar input 26, PWMA3_MUX_TRIG0. */
    kSIM_XbarIn26PWMB3OutTrig0 = 1U, /**< xbar input 26, PWMB3_OUT_TRIG0. */
    kSIM_XbarIn27PWMA3MuxTrig1 = 0U, /**< xbar input 27, PWMA3_MUX_TRIG1. */
    kSIM_XbarIn27PWMB3OutTrig1 = 1U, /**< xbar input 27, PWMB3_OUT_TRIG1. */
    kSIM_XbarIn28PWMB0MuxTrig0 = 0U, /**< xbar input 28, PWMB0_MUX_TRIG0. */
    kSIM_XbarIn28PWMA0OutTrig0 = 1U, /**< xbar input 28, PWMA0_OUT_TRIG0. */
    kSIM_XbarIn29PWMB0MuxTrig1 = 0U, /**< xbar input 29, PWMB0_MUX_TRIG1. */
    kSIM_XbarIn29PWMA0OutTrig1 = 1U, /**< xbar input 29, PWMA0_OUT_TRIG1. */
    kSIM_XbarIn30PWMB1MuxTrig0 = 0U, /**< xbar input 30, PWMB1_MUX_TRIG0. */
    kSIM_XbarIn30PWMA1OutTrig0 = 1U, /**< xbar input 30, PWMA1_OUT_TRIG0. */
    kSIM_XbarIn31PWMB1MuxTrig1 = 0U, /**< xbar input 31, PWMB1_MUX_TRIG1. */
    kSIM_XbarIn31PWMA1OutTrig1 = 1U, /**< xbar input 31, PWMA1_OUT_TRIG1. */
    kSIM_XbarIn32PWMB2MuxTrig0 = 0U, /**< xbar input 32, PWMB2_MUX_TRIG0. */
    kSIM_XbarIn32PWMA2OutTrig0 = 1U, /**< xbar input 32, PWMA2_OUT_TRIG0. */
    kSIM_XbarIn33PWMB2MuxTrig1 = 0U, /**< xbar input 33, PWMB2_MUX_TRIG1. */
    kSIM_XbarIn33PWMA2OutTrig1 = 1U, /**< xbar input 33, PWMA2_OUT_TRIG1. */
    kSIM_XbarIn34PWMB3MuxTrig0 = 0U, /**< xbar input 34, PWMB3_MUX_TRIG0. */
    kSIM_XbarIn34PWMA3OutTrig0 = 1U, /**< xbar input 34, PWMA3_OUT_TRIG0. */
    kSIM_XbarIn35PWMB3MuxTrig1 = 0U, /**< xbar input 35, PWMB3_MUX_TRIG1. */
    kSIM_XbarIn35PWMA3OutTrig1 = 1U, /**< xbar input 35, PWMA3_OUT_TRIG1. */
} sim_xbar_input_pwm_selection_t;

typedef enum _sim_xbar_input_adc_tmr_index
{
    kSIM_XBARIN36Index = 0U,  /**< xbar input 36. */
    kSIM_XBARIN37Index = 1U,  /**< xbar input 37. */
    kSIM_XBARIN38Index = 2U,  /**< xbar input 38. */
    kSIM_XBARIN39Index = 3U,  /**< xbar input 39. */
    kSIM_XBARIN16Index = 8U,  /**< xbar input 16. */
    kSIM_XBARIN17Index = 9U,  /**< xbar input 17. */
    kSIM_XBARIN18Index = 10U, /**< xbar input 18. */
    kSIM_XBARIN19Index = 11U, /**< xbar input 19. */
} sim_xbar_input_adc_tmr_index_t;

typedef enum _sim_xbar_input_adc_tmr_selection
{
    kSIM_XBARIN36TMRA0        = 0U, /**< xbar input 36, TMRA0. */
    kSIM_XBARIN36ADCAN0limit  = 1U, /**< xbar input 36, ADC AN0 limit. */
    kSIM_XBARIN37TMRA1        = 0U, /**< xbar input 37, TMRA1. */
    kSIM_XBARIN37ADCAN1limit  = 1U, /**< xbar input 37, ADC AN1 limit. */
    kSIM_XBARIN38TMRA2        = 0U, /**< xbar input 38, TMRA2. */
    kSIM_XBARIN38ADCAN2limit  = 1U, /**< xbar input 38, ADC AN2 limit. */
    kSIM_XBARIN39TMRA3        = 0U, /**< xbar input 39, TMRA3. */
    kSIM_XBARIN39ADCAN3limit  = 1U, /**< xbar input 39, ADC AN3 limit. */
    kSIM_XBARIN16TMRB0        = 0U, /**< xbar input 16, TMRB0. */
    kSIM_XBARIN16ADCAN8limit  = 1U, /**< xbar input 16, ADC AN8 limit. */
    kSIM_XBARIN17TMRB1        = 0U, /**< xbar input 17, TMRB1. */
    kSIM_XBARIN17ADCAN9limit  = 1U, /**< xbar input 17, ADC AN9 limit. */
    kSIM_XBARIN18TMRB2        = 0U, /**< xbar input 18, TMRB2. */
    kSIM_XBARIN18ADCAN10limit = 1U, /**< xbar input 18, ADC AN10 limit. */
    kSIM_XBARIN19TMRB3        = 0U, /**< xbar input 19, TMRB3. */
    kSIM_XBARIN19ADCAN11limit = 1U, /**< xbar input 19, ADC AN11 limit. */
} sim_xbar_input_adc_tmr_selection_t;

typedef enum _sim_swReset_peri_index
{
    kSIM_SWResetGPIO    = 6U,  /**< GPIO Software Reset. */
    kSIM_SWResetTMRB    = 11U, /**< TMRB Software Reset. */
    kSIM_SWResetTMRA    = 15U, /**< TMRA Software Reset. */
    kSIM_SWResetFLEXCAN = 16U, /**< FlexCAN Software Reset. */
    kSIM_SWResetIIC1    = 21U, /**< IIC1 Software Reset. */
    kSIM_SWResetIIC0    = 22U, /**< IIC0 Software Reset. */
    kSIM_SWResetQSPI1   = 24U, /**< QSPI1 Software Reset. */
    kSIM_SWResetQSPI0   = 25U, /**< QSPI0 Software Reset. */
    kSIM_SWResetSCI2    = 26U, /**< SCI2 Software Reset. */
    kSIM_SWResetSCI1    = 27U, /**< SCI1 Software Reset. */
    kSIM_SWResetSCI0    = 28U, /**< SCI0 Software Reset. */
    kSIM_SWResetDACA    = 29U, /**< DACA Software Reset. */
    kSIM_SWResetDACB    = 30U, /**< DACB Software Reset. */
    kSIM_SWResetPIT1    = 34U, /**< PIT1 Software Reset. */
    kSIM_SWResetPIT0    = 35U, /**< PIT0 Software Reset. */
    kSIM_SWResetCRC     = 37U, /**< CRC Software Reset. */
    kSIM_SWResetCADC    = 39U, /**< Cyclic ADC Software Reset. */
    kSIM_SWResetCMP     = 44U, /**< CMP Software Reset. */
    kSIM_SWResetEWM     = 47U, /**< EWM Software Reset. */
    kSIM_SWResetPWMB    = 51U, /**< PWMB Software Reset. */
    kSIM_SWResetPWMA    = 55U, /**< PWMA Software Reset. */
    kSIM_SWResetUSBOTG  = 56U, /**< USB_OTG Software Reset. */
    kSIM_SWResetDMAMUX  = 57U, /**< DMA_MUX Software Reset. */
} sim_swReset_peri_index_t;

typedef enum _sim_internal_peri_index
{
    kSIM_PWMAFault0InputIndex = 0U,  /**< Select PWMA Fault 0 Input. */
    kSIM_PWMAFault1InputIndex = 1U,  /**< Select PWMA Fault 1 Input. */
    kSIM_PWMAFault2InputIndex = 2U,  /**< Select PWMA Fault 2 Input. */
    kSIM_PWMAFault3InputIndex = 3U,  /**< Select PWMA Fault 3 Input. */
    kSIM_PWMBFault0InputIndex = 4U,  /**< Select PWMB Fault 0 Input. */
    kSIM_PWMBFault1InputIndex = 5U,  /**< Select PWMB Fault 1 Input. */
    kSIM_PWMBFault2InputIndex = 6U,  /**< Select PWMB Fault 2 Input. */
    kSIM_TMRA0InputIndex      = 8U,  /**< Select TMRA0 Input. */
    kSIM_TMRA1InputIndex      = 9U,  /**< Select TMRA1 Input. */
    kSIM_TMRA2InputIndex      = 10U, /**< Select TMRA2 Input. */
    kSIM_TMRA3InputIndex      = 11U, /**< Select TMRA3 Input. */
    kSIM_TMRB0InputIndex      = 12U, /**< Select TMRB0 Input. */
    kSIM_TMRB1InputIndex      = 13U, /**< Select TMRB1 Input. */
    kSIM_TMRB2InputIndex      = 14U, /**< Select TMRB2 Input. */
    kSIM_TMRB3InputIndex      = 15U, /**< Select TMRB3 Input. */
} sim_internal_peri_index_t;

typedef enum _sim_internal_peri_input
{
    kSIM_PWMAFault0Input_GPIOE8          = 0U, /**< Select PWMA Fault 0 Input, GPIOE8. */
    kSIM_PWMAFault0Input_XB_OUT29        = 1U, /**< Select PWMA Fault 0 Input, XB_OUT29. */
    kSIM_PWMAFault1Input_GPIOE9          = 0U, /**< Select PWMA Fault 1 Input, GPIOE9. */
    kSIM_PWMAFault1Input_XB_OUT30        = 1U, /**< Select PWMA Fault 1 Input, XB_OUT30. */
    kSIM_PWMAFault2Input_GPIOG4          = 0U, /**< Select PWMA Fault 2 Input, GPIOG4. */
    kSIM_PWMAFault2Input_XB_OUT31        = 1U, /**< Select PWMA Fault 2 Input, XB_OUT31. */
    kSIM_PWMAFault3Input_GPIOG5          = 0U, /**< Select PWMA Fault 3 Input, GPIOG5. */
    kSIM_PWMAFault3Input_XB_OUT32        = 1U, /**< Select PWMA Fault 3 Input, XB_OUT32. */
    kSIM_PWMBFault0Input_GPIOF14         = 0U, /**< Select PWMB Fault 0 Input, GPIOF14. */
    kSIM_PWMBFault0Input_XB_OUT29        = 1U, /**< Select PWMB Fault 0 Input, XB_OUT29. */
    kSIM_PWMBFault1Input_GPIOF13         = 0U, /**< Select PWMB Fault 1 Input, GPIOF13. */
    kSIM_PWMBFault1Input_XB_OUT30        = 1U, /**< Select PWMB Fault 1 Input, XB_OUT30. */
    kSIM_PWMBFault2Input_GPIOF12         = 0U, /**< Select PWMB Fault 2 Input, GPIOF12. */
    kSIM_PWMBFault2Input_XB_OUT31        = 1U, /**< Select PWMB Fault 2 Input, XB_OUT31. */
    kSIM_TMRA0Input_GPIOC3               = 0U, /**< Select TMRA0 Input, GPIOC3. */
    kSIM_TMRA0Input_XB_OUT38             = 1U, /**< Select TMRA0 Input, XB_OUT38. */
    kSIM_TMRA1Input_GPIOC4               = 0U, /**< Select TMRA1 Input, GPIOC4. */
    kSIM_TMRA1Input_XB_OUT39             = 1U, /**< Select TMRA1 Input, XB_OUT39. */
    kSIM_TMRA2Input_GPIOC6_GPIOG8        = 0U, /**< Select TMRA2 Input, GPIOC6 or GPIOG8. */
    kSIM_TMRA2Input_XB_OUT40             = 1U, /**< Select TMRA2 Input, XB_OUT40. */
    kSIM_TMRA3Input_GPIOC13_GPIOG9       = 0U, /**< Select TMRA3 Input, GPIOC13 or GPIOG9. */
    kSIM_TMRA3Input_XB_OUT41             = 1U, /**< Select TMRA3 Input, XB_OUT41. */
    kSIM_TMRB0Input_GPIOC2               = 0U, /**< Select TMRB0 Input, GPIOC2. */
    kSIM_TMRB0Input_XB_OUT34             = 1U, /**< Select TMRB0 Input, XB_OUT34. */
    kSIM_TMRB1Input_GPIOF8               = 0U, /**< Select TMRB1 Input, GPIOF8. */
    kSIM_TMRB1Input_XB_OUT35             = 1U, /**< Select TMRB1 Input, XB_OUT35. */
    kSIM_TMRB2Input_GPIOF6_GPIOF0_GPIOG6 = 0U, /**< Select TMRB2 Input, GPIOF6, GPIOF0, or GPIOG6. */
    kSIM_TMRB2Input_XB_OUT36             = 1U, /**< Select TMRB2 Input, XB_OUT36. */
    kSIM_TMRB3Input_GPIOF7_GPIOG11       = 0U, /**< Select TMRB3 Input, GPIOF7 or GPIOG11. */
    kSIM_TMRB3Input_XB_OUT37             = 1U, /**< Select TMRB3 Input, XB_OUT37. */
} sim_internal_peri_input_t;

typedef enum _sim_software_contrl_register_index
{
    kSIM_SCR0 = 0U, /**< SCR0. */
    kSIM_SCR1 = 1U, /**< SCR1. */
    kSIM_SCR2 = 2U, /**< SCR2. */
    kSIM_SCR3 = 3U, /**< SCR3. */
} sim_software_contrl_register_index_t;

/*!
 * @brief The enumeration of boot over ride mode, FOPT[7:6] & ~ boot_override_mode determines the boot option,
 * the result 11b boots from ROM, while any other values boot from flash.
 */
typedef enum _sim_boot_override_mode
{
    kSIM_BootFromRomOrFlash = 0U, /**< Boot from ROM or Flash depending on FOPT[7:6] at next non-POR reset. */
    kSIM_BootFromFlash      = 1U, /**< Boot from flash at next non-POR reset. */
} sim_boot_override_mode_t;

typedef enum _xbar_input_signal
{
    kXBARA_InputVss                   = 0U,  /**< VSS output assigned to XBARA_IN0 input. */
    kXBARA_InputVdd                   = 1U,  /**< VDD output assigned to XBARA_IN1 input. */
    kXBARA_InputXbIn2                 = 2U,  /**< XB_IN2 output assigned to XBARA_IN2 input. */
    kXBARA_InputXbIn3                 = 3U,  /**< XB_IN3 output assigned to XBARA_IN3 input. */
    kXBARA_InputXbIn4                 = 4U,  /**< XB_IN4 output assigned to XBARA_IN4 input. */
    kXBARA_InputXbIn5                 = 5U,  /**< XB_IN5 output assigned to XBARA_IN5 input. */
    kXBARA_InputXbIn6                 = 6U,  /**< XB_IN6 output assigned to XBARA_IN6 input. */
    kXBARA_InputXbIn7                 = 7U,  /**< XB_IN7 output assigned to XBARA_IN7 input. */
    kXBARA_InputXbIn8                 = 8U,  /**< XB_IN8 output assigned to XBARA_IN8 input. */
    kXBARA_InputXbIn9                 = 9U,  /**< XB_IN9 output assigned to XBARA_IN9 input. */
    kXBARA_InputXbIn10                = 10U, /**< XB_IN10 output assigned to XBARA_IN10 input. */
    kXBARA_InputXbIn11                = 11U, /**< XB_IN11 output assigned to XBARA_IN11 input. */
    kXBARA_InputCmpaOut               = 12U, /**< CMPA_OUT output assigned to XBARA_IN12 input. */
    kXBARA_InputCmpbOut               = 13U, /**< CMPB_OUT output assigned to XBARA_IN13 input. */
    kXBARA_InputCmpcOut               = 14U, /**< CMPC_OUT output assigned to XBARA_IN14 input. */
    kXBARA_InputCmpdOut               = 15U, /**< CMPD_OUT output assigned to XBARA_IN15 input. */
    kXBARA_InputAn8LimitTb0Out        = 16U, /**< AN8_LIMIT_TB0_OUT output assigned to XBARA_IN16 input. */
    kXBARA_InputAn9LimitTb1Out        = 17U, /**< AN9_LIMIT_TB1_OUT output assigned to XBARA_IN17 input. */
    kXBARA_InputAn10LimitTb2Out       = 18U, /**< AN10_LIMIT_TB2_OUT output assigned to XBARA_IN18 input. */
    kXBARA_InputAn11LimitTb3Out       = 19U, /**< AN11_LIMIT_TB3_OUT output assigned to XBARA_IN19 input. */
    kXBARA_InputPwma0MuxPwmb0OutTrig0 = 20U, /**< PWMA0_MUX_PWMB0_OUT_TRIG0 output assigned to XBARA_IN20 input. */
    kXBARA_InputPwma0MuxPwmb0OutTrig1 = 21U, /**< PWMA0_MUX_PWMB0_OUT_TRIG1 output assigned to XBARA_IN21 input. */
    kXBARA_InputPwma1MuxPwmb1OutTrig0 = 22U, /**< PWMA1_MUX_PWMB1_OUT_TRIG0 output assigned to XBARA_IN22 input. */
    kXBARA_InputPwma1MuxPwmb1OutTrig1 = 23U, /**< PWMA1_MUX_PWMB1_OUT_TRIG1 output assigned to XBARA_IN23 input. */
    kXBARA_InputPwma2MuxPwmb2OutTrig0 = 24U, /**< PWMA2_MUX_PWMB2_OUT_TRIG0 output assigned to XBARA_IN24 input. */
    kXBARA_InputPwma2MuxPwmb2OutTrig1 = 25U, /**< PWMA2_MUX_PWMB2_OUT_TRIG1 output assigned to XBARA_IN25 input. */
    kXBARA_InputPwma3MuxPwmb3OutTrig0 = 26U, /**< PWMA3_MUX_PWMB3_OUT_TRIG0 output assigned to XBARA_IN26 input. */
    kXBARA_InputPwma3MuxPwmb3OutTrig1 = 27U, /**< PWMA3_MUX_PWMB3_OUT_TRIG1 output assigned to XBARA_IN27 input. */
    kXBARA_InputPwmb0MuxPwma0OutTrig0 = 28U, /**< PWMB0_MUX_PWMA0_OUT_TRIG0 output assigned to XBARA_IN28 input. */
    kXBARA_InputPwmb0MuxPwma0OutTrig1 = 29U, /**< PWMB0_MUX_PWMA0_OUT_TRIG1 output assigned to XBARA_IN29 input. */
    kXBARA_InputPwmb1MuxPwma1OutTrig0 = 30U, /**< PWMB1_MUX_PWMA1_OUT_TRIG0 output assigned to XBARA_IN30 input. */
    kXBARA_InputPwmb1MuxPwma1OutTrig1 = 31U, /**< PWMB1_MUX_PWMA1_OUT_TRIG1 output assigned to XBARA_IN31 input. */
    kXBARA_InputPwmb2MuxPwma2OutTrig0 = 32U, /**< PWMB2_MUX_PWMA2_OUT_TRIG0 output assigned to XBARA_IN32 input. */
    kXBARA_InputPwmb2MuxPwma2OutTrig1 = 33U, /**< PWMB2_MUX_PWMA2_OUT_TRIG1 output assigned to XBARA_IN33 input. */
    kXBARA_InputPwmb3MuxPwma3OutTrig0 = 34U, /**< PWMB3_MUX_PWMA3_OUT_TRIG0 output assigned to XBARA_IN34 input. */
    kXBARA_InputPwmb3MuxPwma3OutTrig1 = 35U, /**< PWMB3_MUX_PWMA3_OUT_TRIG1 output assigned to XBARA_IN35 input. */
    kXBARA_InputAn0LimitTa0Out        = 36U, /**< AN0_LIMIT_TA0_OUT output assigned to XBARA_IN36 input. */
    kXBARA_InputAn1LimitTa1Out        = 37U, /**< AN1_LIMIT_TA1_OUT output assigned to XBARA_IN37 input. */
    kXBARA_InputAn2LimitTa2Out        = 38U, /**< AN2_LIMIT_TA2_OUT output assigned to XBARA_IN38 input. */
    kXBARA_InputAn3LimitTa3Out        = 39U, /**< AN3_LIMIT_TA3_OUT output assigned to XBARA_IN39 input. */
    kXBARA_InputEvtg0Outa             = 40U, /**< EVTG0_OUTA output assigned to XBARA_IN40 input. */
    kXBARA_InputEvtg1Outa             = 41U, /**< EVTG1_OUTA output assigned to XBARA_IN41 input. */
    kXBARA_InputEvtg2Outa             = 42U, /**< EVTG2_OUTA output assigned to XBARA_IN42 input. */
    kXBARA_InputEvtg3Outa             = 43U, /**< EVTG3_OUTA output assigned to XBARA_IN43 input. */
    kXBARA_InputPit0SyncOut           = 44U, /**< PIT0_SYNC_OUT output assigned to XBARA_IN44 input. */
    kXBARA_InputPit1SyncOut           = 45U, /**< PIT1_SYNC_OUT output assigned to XBARA_IN45 input. */
    kXBARA_InputEvtg0Outb             = 46U, /**< EVTG0_OUTB output assigned to XBARA_IN46 input. */
    kXBARA_InputEvtg1Outb             = 47U, /**< EVTG1_OUTB output assigned to XBARA_IN47 input. */
    kXBARA_InputEvtg2Outb             = 48U, /**< EVTG2_OUTB output assigned to XBARA_IN48 input. */
    kXBARA_InputEvtg3Outb             = 49U, /**< EVTG3_OUTB output assigned to XBARA_IN49 input. */
    kXBARA_InputPwmbAllTrig           = 50U, /**< PWMB_ALL_TRIG output assigned to XBARA_IN50 input. */
    kXBARA_InputPwmaAllTrig           = 51U, /**< PWMA_ALL_TRIG output assigned to XBARA_IN51 input. */
    kXBARA_InputSci1Rxd               = 52U, /**< SCI1_RXD output assigned to XBARA_IN52 input. */
} xbar_input_signal_t;

typedef enum _xbar_output_signal
{
    kXBARA_OutputDmaReq0        = 0U,  /**< XBARA_OUT0 output assigned to DMA_REQ0 */
    kXBARA_OutputDmaReq1        = 1U,  /**< XBARA_OUT1 output assigned to DMA_REQ1 */
    kXBARA_OutputDmaReq2        = 2U,  /**< XBARA_OUT2 output assigned to DMA_REQ2 */
    kXBARA_OutputDmaReq3        = 3U,  /**< XBARA_OUT3 output assigned to DMA_REQ3 */
    kXBARA_OutputXbOut4         = 4U,  /**< XBARA_OUT4 output assigned to XB_OUT4 */
    kXBARA_OutputXbOut5         = 5U,  /**< XBARA_OUT5 output assigned to XB_OUT5 */
    kXBARA_OutputXbOut6         = 6U,  /**< XBARA_OUT6 output assigned to XB_OUT6 */
    kXBARA_OutputXbOut7         = 7U,  /**< XBARA_OUT7 output assigned to XB_OUT7 */
    kXBARA_OutputXbOut8         = 8U,  /**< XBARA_OUT8 output assigned to XB_OUT8 */
    kXBARA_OutputXbOut9         = 9U,  /**< XBARA_OUT9 output assigned to XB_OUT9 */
    kXBARA_OutputXbOut10        = 10U, /**< XBARA_OUT10 output assigned to XB_OUT10 */
    kXBARA_OutputXbOut11        = 11U, /**< XBARA_OUT11 output assigned to XB_OUT11 */
    kXBARA_OutputAdcaSync       = 12U, /**< XBARA_OUT12 output assigned to ADCA_SYNC */
    kXBARA_OutputAdcbSync       = 13U, /**< XBARA_OUT13 output assigned to ADCB_SYNC */
    kXBARA_OutputDacb12bSync    = 14U, /**< XBARA_OUT14 output assigned to DACB_12B_SYNC */
    kXBARA_OutputDaca12bSync    = 15U, /**< XBARA_OUT15 output assigned to DACA_12B_SYNC */
    kXBARA_OutputCmpa           = 16U, /**< XBARA_OUT16 output assigned to CMPA */
    kXBARA_OutputCmpb           = 17U, /**< XBARA_OUT17 output assigned to CMPB */
    kXBARA_OutputCmpc           = 18U, /**< XBARA_OUT18 output assigned to CMPC */
    kXBARA_OutputCmpd           = 19U, /**< XBARA_OUT19 output assigned to CMPD */
    kXBARA_OutputPwma0Pwmb0Exta = 20U, /**< XBARA_OUT20 output assigned to PWMA0_PWMB0_EXTA */
    kXBARA_OutputPwma1Pwmb1Exta = 21U, /**< XBARA_OUT21 output assigned to PWMA1_PWMB1_EXTA */
    kXBARA_OutputPwma2Pwmb2Exta = 22U, /**< XBARA_OUT22 output assigned to PWMA2_PWMB2_EXTA */
    kXBARA_OutputPwma3Pwmb3Exta = 23U, /**< XBARA_OUT23 output assigned to PWMA3_PWMB3_EXTA */
    kXBARA_OutputPwma0ExtSync   = 24U, /**< XBARA_OUT24 output assigned to PWMA0_EXT_SYNC */
    kXBARA_OutputPwma1ExtSync   = 25U, /**< XBARA_OUT25 output assigned to PWMA1_EXT_SYNC */
    kXBARA_OutputPwma2ExtSync   = 26U, /**< XBARA_OUT26 output assigned to PWMA2_EXT_SYNC */
    kXBARA_OutputPwma3ExtSync   = 27U, /**< XBARA_OUT27 output assigned to PWMA3_EXT_SYNC */
    kXBARA_OutputPwmaPwmbExtClk = 28U, /**< XBARA_OUT28 output assigned to PWMA_PWMB_EXT_CLK */
    kXBARA_OutputPwmaPwmbFault0 = 29U, /**< XBARA_OUT29 output assigned to PWMA_PWMB_FAULT0 */
    kXBARA_OutputPwmaPwmbFault1 = 30U, /**< XBARA_OUT30 output assigned to PWMA_PWMB_FAULT1 */
    kXBARA_OutputPwmaPwmbFault2 = 31U, /**< XBARA_OUT31 output assigned to PWMA_PWMB_FAULT2 */
    kXBARA_OutputPwmaPwmbFault3 = 32U, /**< XBARA_OUT32 output assigned to PWMA_PWMB_FAULT3 */
    kXBARA_OutputPwmaForce      = 33U, /**< XBARA_OUT33 output assigned to PWMA_FORCE */
    kXBARA_OutputTb0In          = 34U, /**< XBARA_OUT34 output assigned to TB0_IN */
    kXBARA_OutputTb1In          = 35U, /**< XBARA_OUT35 output assigned to TB1_IN */
    kXBARA_OutputTb2In          = 36U, /**< XBARA_OUT36 output assigned to TB2_IN */
    kXBARA_OutputTb3In          = 37U, /**< XBARA_OUT37 output assigned to TB3_IN */
    kXBARA_OutputTa0In          = 38U, /**< XBARA_OUT38 output assigned to TA0_IN */
    kXBARA_OutputTa1In          = 39U, /**< XBARA_OUT39 output assigned to TA1_IN */
    kXBARA_OutputTa2In          = 40U, /**< XBARA_OUT40 output assigned to TA2_IN */
    kXBARA_OutputTa3In          = 41U, /**< XBARA_OUT41 output assigned to TA3_IN */
    kXBARA_OutputPwmb0ExtSync   = 42U, /**< XBARA_OUT42 output assigned to PWMB0_EXT_SYNC */
    kXBARA_OutputPwmb1ExtSync   = 43U, /**< XBARA_OUT43 output assigned to PWMB1_EXT_SYNC */
    kXBARA_OutputPwmb2ExtSync   = 44U, /**< XBARA_OUT44 output assigned to PWMB2_EXT_SYNC */
    kXBARA_OutputPwmb3ExtSync   = 45U, /**< XBARA_OUT45 output assigned to PWMB3_EXT_SYNC */
    kXBARA_OutputPwmbForce      = 46U, /**< XBARA_OUT46 output assigned to PWMB_FORCE */
    kXBARA_OutputEvtg0A         = 47U, /**< XBARA_OUT47 output assigned to EVTG0_A */
    kXBARA_OutputEvtg0B         = 48U, /**< XBARA_OUT48 output assigned to EVTG0_B */
    kXBARA_OutputEvtg0C         = 49U, /**< XBARA_OUT49 output assigned to EVTG0_C */
    kXBARA_OutputEvtg0D         = 50U, /**< XBARA_OUT50 output assigned to EVTG0_D */
    kXBARA_OutputEvtg1A         = 51U, /**< XBARA_OUT51 output assigned to EVTG1_A */
    kXBARA_OutputEvtg1B         = 52U, /**< XBARA_OUT52 output assigned to EVTG1_B */
    kXBARA_OutputEvtg1C         = 53U, /**< XBARA_OUT53 output assigned to EVTG1_C */
    kXBARA_OutputEvtg1D         = 54U, /**< XBARA_OUT54 output assigned to EVTG1_D */
    kXBARA_OutputEvtg2A         = 55U, /**< XBARA_OUT55 output assigned to EVTG2_A */
    kXBARA_OutputEvtg2B         = 56U, /**< XBARA_OUT56 output assigned to EVTG2_B */
    kXBARA_OutputEvtg2C         = 57U, /**< XBARA_OUT57 output assigned to EVTG2_C */
    kXBARA_OutputEvtg2D         = 58U, /**< XBARA_OUT58 output assigned to EVTG2_D */
    kXBARA_OutputEvtg3A         = 59U, /**< XBARA_OUT59 output assigned to EVTG3_A */
    kXBARA_OutputEvtg3B         = 60U, /**< XBARA_OUT60 output assigned to EVTG3_B */
    kXBARA_OutputEvtg3C         = 61U, /**< XBARA_OUT61 output assigned to EVTG3_C */
    kXBARA_OutputEvtg3D         = 62U, /**< XBARA_OUT62 output assigned to EVTG3_D */
    kXBARA_OutputEwmIn          = 63U, /**< XBARA_OUT63 output assigned to EWM_IN */
} xbar_output_signal_t;

/*!
 * @}
 */ /* end of group Mapping_Information */

/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{
 */

/*
** Start of section using anonymous unions
*/

#if defined(__CWCC__)
#pragma push
#pragma cpp_extensions on
#else
#error Not supported compiler type
#endif

/* ----------------------------------------------------------------------------
   -- ADC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Peripheral_Access_Layer ADC Peripheral Access Layer
 * @{
 */

/** ADC - Register Layout Typedef */
typedef struct
{
    __IO uint16_t CTRL1;     /**< ADC Control Register 1, offset: 0x0 */
    __IO uint16_t CTRL2;     /**< ADC Control Register 2, offset: 0x1 */
    __IO uint16_t ZXCTRL1;   /**< ADC Zero Crossing Control 1 Register, offset: 0x2 */
    __IO uint16_t ZXCTRL2;   /**< ADC Zero Crossing Control 2 Register, offset: 0x3 */
    __IO uint16_t CLIST1;    /**< ADC Channel List Register 1, offset: 0x4 */
    __IO uint16_t CLIST2;    /**< ADC Channel List Register 2, offset: 0x5 */
    __IO uint16_t CLIST3;    /**< ADC Channel List Register 3, offset: 0x6 */
    __IO uint16_t CLIST4;    /**< ADC Channel List Register 4, offset: 0x7 */
    __IO uint16_t SDIS;      /**< ADC Sample Disable Register, offset: 0x8 */
    __IO uint16_t STAT;      /**< ADC Status Register, offset: 0x9 */
    __I uint16_t RDY;        /**< ADC Ready Register, offset: 0xA */
    __IO uint16_t LOLIMSTAT; /**< ADC Low Limit Status Register, offset: 0xB */
    __IO uint16_t HILIMSTAT; /**< ADC High Limit Status Register, offset: 0xC */
    __IO uint16_t ZXSTAT;    /**< ADC Zero Crossing Status Register, offset: 0xD */
    __IO uint16_t RSLT[16];  /**< ADC Result Registers with sign extension, array offset: 0xE, array step: 0x1 */
    __IO uint16_t LOLIM[16]; /**< ADC Low Limit Registers, array offset: 0x1E, array step: 0x1 */
    __IO uint16_t HILIM[16]; /**< ADC High Limit Registers, array offset: 0x2E, array step: 0x1 */
    __IO uint16_t OFFST[16]; /**< ADC Offset Registers, array offset: 0x3E, array step: 0x1 */
    __IO uint16_t PWR;       /**< ADC Power Control Register, offset: 0x4E */
    __IO uint16_t CAL;       /**< ADC Calibration Register, offset: 0x4F */
    __IO uint16_t GC1;       /**< Gain Control 1 Register, offset: 0x50 */
    __IO uint16_t GC2;       /**< Gain Control 2 Register, offset: 0x51 */
    __IO uint16_t SCTRL;     /**< ADC Scan Control Register, offset: 0x52 */
    __IO uint16_t PWR2;      /**< ADC Power Control Register 2, offset: 0x53 */
    __IO uint16_t CTRL3;     /**< ADC Control Register 3, offset: 0x54 */
    __IO uint16_t SCHLTEN;   /**< ADC Scan Interrupt Enable Register, offset: 0x55 */
    uint16_t RESERVED_0[2];
    __IO uint16_t ZXCTRL3;    /**< ADC Zero Crossing Control 3 Register, offset: 0x58 */
    __IO uint16_t CLIST5;     /**< ADC Channel List Register 5, offset: 0x59 */
    __IO uint16_t SDIS2;      /**< ADC Sample Disable Register 2, offset: 0x5A */
    __I uint16_t RDY2;        /**< ADC Ready Register 2, offset: 0x5B */
    __IO uint16_t LOLIMSTAT2; /**< ADC Low Limit Status Register 2, offset: 0x5C */
    __IO uint16_t HILIMSTAT2; /**< ADC High Limit Status Register 2, offset: 0x5D */
    __IO uint16_t ZXSTAT2;    /**< ADC Zero Crossing Status Register 2, offset: 0x5E */
    __IO uint16_t RSLT2[4];   /**< ADC Result Registers 2 with sign extension, array offset: 0x5F, array step: 0x1 */
    __IO uint16_t LOLIM2[4];  /**< ADC Low Limit Registers 2, array offset: 0x63, array step: 0x1 */
    __IO uint16_t HILIM2[4];  /**< ADC High Limit Registers 2, array offset: 0x67, array step: 0x1 */
    __IO uint16_t OFFST2[4];  /**< ADC Offset Registers 2, array offset: 0x6B, array step: 0x1 */
    __IO uint16_t GC3;        /**< Gain Control 3 Register, offset: 0x6F */
    __IO uint16_t SCTRL2;     /**< ADC Scan Control Register 2, offset: 0x70 */
    __IO uint16_t SCHLTEN2;   /**< ADC Scan Interrupt Enable Register 2, offset: 0x71 */
} ADC_Type;

/* ----------------------------------------------------------------------------
   -- ADC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Masks ADC Register Masks
 * @{
 */

/*! @name CTRL1 - ADC Control Register 1 */
/*! @{ */

#define ADC_CTRL1_SMODE_MASK  (0x7U)
#define ADC_CTRL1_SMODE_SHIFT (0U)
/*! SMODE - ADC Scan Mode Control
 *  0b000..Once (single) sequential - Upon start or an enabled sync signal, samples are taken one at a time
 *         starting with CLIST1[SAMPLE0], until the first disabled sample is encountered. If no disabled sample is
 *         encountered, conversion concludes after CLIST4[SAMPLE15]. If CLIST5[SAMPLE16] is enabled in SDIS2 then the
 *         scan will continue until the first disabled sample is encountered or when all 4 additional samples are
 *         completed. If the scan is initiated by a SYNC signal, only one scan is completed because the CTRL*[SYNC*]
 *         bit is cleared automatically by the initial SYNC detection. CTRL*[SYNC*] can be set again at any time
 *         during the scan.
 *  0b001..Once parallel - Upon start or an armed and enabled sync signal: In parallel, converter A converts
 *         SAMPLEs 0-7 , and converter B converts SAMPLEs 8-15 . When CTRL2[SIMULT] is 1 (default), scanning stops when
 *         either converter encounters a disabled sample or both converters complete all 8 samples. When
 *         CTRL2[SIMULT] is 0, a converter stops scanning when it encounters a disabled sample or completes all 8
 * samples. If additional samples are enabled in SDIS2 then the parallel scan will continue with converter A converting
 * SAMPLEs 16-17 and convert B converting SAMPLEs 18-19, until the first disabled sample is encountered or when each
 * converter completes 2 additional samples. If the scan is initiated by a SYNC signal, only one scan is completed
 * because the CTRL*[SYNC*] bit is cleared automatically by the initial SYNC detection. CTRL*[SYNC*] can be set again at
 * any time during the scan. If CTRL2[SIMULT] is 0, the B converter must be rearmed by writing the CTRL2[SYNC1] bit.
 *  0b010..Loop sequential - Upon an initial start or enabled sync pulse, up to 16 samples in the order SAMPLEs
 *         0-15 are taken one at a time until a disabled sample is encountered. If additional samples are enabled in
 *         the SDIS2 register, the scan will continue with SAMPLEs 16-19 until a disabled sample is encountered.
 *         The process repeats perpetually until the CTRL1[STOP0] bit is set. While a loop mode is running, any
 *         additional start commands or sync pulses are ignored unless the scan is paused using the SCTRL[SC*] bits. If
 *         PWR[APD] is the selected power mode control, PWR[PUDELAY] is applied only on the first conversion.
 *  0b011..Loop parallel - Upon an initial start or enabled sync pulse, converter A converts SAMPLEs 0-7 , and
 *         converter B converts SAMPLEs 8-15 . If additional samples are enabled in SDIS2 then the parallel scan will
 *         continue with converter A converting SAMPLEs 16-17 and convert B converting SAMPLEs 18-19, until the
 *         first disabled sample is encountered or when each converter completes 2 additional samples. Each time a
 *         converter completes its current scan, it immediately restarts its scan sequence. This process continues
 *         until the CTRL*[STOP*] bit is asserted. While a loop is running, any additional start commands or sync
 *         pulses are ignored unless the scan is paused using the SCTRL[SC*] bits. When CTRL2[SIMULT] is 1 (default),
 *         scanning restarts when either converter encounters a disabled sample. When CTRL2[SIMULT] is 0, a
 *         converter restarts scanning when it encounters a disabled sample. If PWR[APD] is the selected power mode
 *         control, PWR[PUDELAY] is applied only on the first conversion.
 *  0b100..Triggered sequential - Upon start or an enabled sync signal, samples are taken one at a time starting
 *         with CLIST1[SAMPLE0], until the first disabled sample is encountered. If no disabled sample is
 *         encountered, conversion concludes after CLIST4[SAMPLE15]. If CLIST5[SAMPLE16] is enabled in SDIS2 then the
 * scan will continue until the first disabled sample is encountered or when all 4 additional samples are completed. If
 * external sync is enabled, new scans start for each SYNC pulse that does not overlap with a current scan in progress.
 *  0b101..Triggered parallel (default) - Upon start or an enabled sync signal: In parallel, converter A converts
 *         SAMPLEs 0-7 , and converter B converts SAMPLEs 8-15 . When CTRL2[SIMULT] is 1 (default), scanning stops
 *         when either converter encounters a disabled sample. When CTRL2[SIMULT] is 0, a converter stops scanning
 *         when it encounters a disabled sample. If additional samples are enabled in SDIS2 then the parallel scan
 *         will continue with converter A converting SAMPLEs 16-17 and convert B converting SAMPLEs 18-19, until
 *         the first disabled sample is encountered or when each converter completes 2 additional samples. If
 *         external sync is enabled, new scans start for each SYNC pulse that does not overlap with a current scan in
 *         progress.
 *  0b11x..Reserved
 */
#define ADC_CTRL1_SMODE(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_SMODE_SHIFT)) & ADC_CTRL1_SMODE_MASK)

#define ADC_CTRL1_CHNCFG_L_MASK  (0xF0U)
#define ADC_CTRL1_CHNCFG_L_SHIFT (4U)
/*! CHNCFG_L - CHCNF (Channel Configure Low) bits
 *  0bxxx1..Inputs = ANA0-ANA1 : Configured as differential pair (ANA0 is + and ANA1 is -)
 *  0bxxx0..Inputs = ANA0-ANA1 : Both configured as single ended inputs
 *  0bxx1x..Inputs = ANA2-ANA3 : Configured as differential pair (ANA2 is + and ANA3 is -)
 *  0bxx0x..Inputs = ANA2-ANA3 : Both configured as single ended inputs
 *  0bx1xx..Inputs = ANB0-ANB1 : Configured as differential pair (ANB0 is + and ANB1 is -)
 *  0bx0xx..Inputs = ANB0-ANB1 : Both configured as single ended inputs
 *  0b1xxx..Inputs = ANB2-ANB3 : Configured as differential pair (ANB2 is + and ANB3 is -)
 *  0b0xxx..Inputs = ANB2-ANB3 : Both configured as single ended inputs
 */
#define ADC_CTRL1_CHNCFG_L(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_CHNCFG_L_SHIFT)) & ADC_CTRL1_CHNCFG_L_MASK)

#define ADC_CTRL1_HLMTIE_MASK  (0x100U)
#define ADC_CTRL1_HLMTIE_SHIFT (8U)
/*! HLMTIE - High Limit Interrupt Enable
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define ADC_CTRL1_HLMTIE(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_HLMTIE_SHIFT)) & ADC_CTRL1_HLMTIE_MASK)

#define ADC_CTRL1_LLMTIE_MASK  (0x200U)
#define ADC_CTRL1_LLMTIE_SHIFT (9U)
/*! LLMTIE - Low Limit Interrupt Enable
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define ADC_CTRL1_LLMTIE(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_LLMTIE_SHIFT)) & ADC_CTRL1_LLMTIE_MASK)

#define ADC_CTRL1_ZCIE_MASK  (0x400U)
#define ADC_CTRL1_ZCIE_SHIFT (10U)
/*! ZCIE - Zero Crossing Interrupt Enable
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define ADC_CTRL1_ZCIE(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_ZCIE_SHIFT)) & ADC_CTRL1_ZCIE_MASK)

#define ADC_CTRL1_EOSIE0_MASK  (0x800U)
#define ADC_CTRL1_EOSIE0_SHIFT (11U)
/*! EOSIE0 - End Of Scan Interrupt Enable
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define ADC_CTRL1_EOSIE0(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_EOSIE0_SHIFT)) & ADC_CTRL1_EOSIE0_MASK)

#define ADC_CTRL1_SYNC0_MASK  (0x1000U)
#define ADC_CTRL1_SYNC0_SHIFT (12U)
/*! SYNC0 - SYNC0 Enable
 *  0b0..Scan is initiated by a write to CTRL1[START0] only
 *  0b1..Use a SYNC0 input pulse or CTRL1[START0] to initiate a scan
 */
#define ADC_CTRL1_SYNC0(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_SYNC0_SHIFT)) & ADC_CTRL1_SYNC0_MASK)

#define ADC_CTRL1_START0_MASK  (0x2000U)
#define ADC_CTRL1_START0_SHIFT (13U)
/*! START0 - START0 Conversion
 *  0b0..No action
 *  0b1..Start command is issued
 */
#define ADC_CTRL1_START0(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_START0_SHIFT)) & ADC_CTRL1_START0_MASK)

#define ADC_CTRL1_STOP0_MASK  (0x4000U)
#define ADC_CTRL1_STOP0_SHIFT (14U)
/*! STOP0 - Stop
 *  0b0..Normal operation
 *  0b1..Stop mode
 */
#define ADC_CTRL1_STOP0(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_STOP0_SHIFT)) & ADC_CTRL1_STOP0_MASK)

#define ADC_CTRL1_DMAEN0_MASK  (0x8000U)
#define ADC_CTRL1_DMAEN0_SHIFT (15U)
/*! DMAEN0 - DMA enable
 *  0b0..DMA is not enabled.
 *  0b1..DMA is enabled.
 */
#define ADC_CTRL1_DMAEN0(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_DMAEN0_SHIFT)) & ADC_CTRL1_DMAEN0_MASK)
/*! @} */

/*! @name CTRL2 - ADC Control Register 2 */
/*! @{ */

#define ADC_CTRL2_DIV0_MASK  (0x3FU)
#define ADC_CTRL2_DIV0_SHIFT (0U)
/*! DIV0 - Clock Divisor Select
 */
#define ADC_CTRL2_DIV0(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_DIV0_SHIFT)) & ADC_CTRL2_DIV0_MASK)

#define ADC_CTRL2_SIMULT_MASK  (0x40U)
#define ADC_CTRL2_SIMULT_SHIFT (6U)
/*! SIMULT - Simultaneous mode
 *  0b0..Parallel scans done independently
 *  0b1..Parallel scans done simultaneously (default)
 */
#define ADC_CTRL2_SIMULT(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_SIMULT_SHIFT)) & ADC_CTRL2_SIMULT_MASK)

#define ADC_CTRL2_CHNCFG_H_MASK  (0x780U)
#define ADC_CTRL2_CHNCFG_H_SHIFT (7U)
/*! CHNCFG_H - CHCNF (Channel Configure High) bits
 *  0bxxx1..Inputs = ANA4-ANA5 : Configured as differential pair (ANA4 is + and ANA5 is -)
 *  0bxxx0..Inputs = ANA4-ANA5 : Both configured as single ended inputs
 *  0bxx1x..Inputs = ANA6-ANA7 : Configured as differential pair (ANA6 is + and ANA7 is -)
 *  0bxx0x..Inputs = ANA6-ANA7 : Both configured as single ended inputs
 *  0bx1xx..Inputs = ANA4-ANA5 : Configured as differential pair (ANB4 is + and ANB5 is -)
 *  0bx0xx..Inputs = ANA4-ANA5 : Both configured as single ended inputs
 *  0b1xxx..Inputs = ANA6-ANA7 : Configured as differential pair (ANB6 is + and ANB7 is -)
 *  0b0xxx..Inputs = ANA6-ANA7 : Both configured as single ended inputs
 */
#define ADC_CTRL2_CHNCFG_H(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_CHNCFG_H_SHIFT)) & ADC_CTRL2_CHNCFG_H_MASK)

#define ADC_CTRL2_EOSIE1_MASK  (0x800U)
#define ADC_CTRL2_EOSIE1_SHIFT (11U)
/*! EOSIE1 - End Of Scan Interrupt Enable
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define ADC_CTRL2_EOSIE1(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_EOSIE1_SHIFT)) & ADC_CTRL2_EOSIE1_MASK)

#define ADC_CTRL2_SYNC1_MASK  (0x1000U)
#define ADC_CTRL2_SYNC1_SHIFT (12U)
/*! SYNC1 - SYNC1 Enable
 *  0b0..B converter parallel scan is initiated by a write to CTRL2[START1] bit only
 *  0b1..Use a SYNC1 input pulse or CTRL2[START1] bit to initiate a B converter parallel scan
 */
#define ADC_CTRL2_SYNC1(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_SYNC1_SHIFT)) & ADC_CTRL2_SYNC1_MASK)

#define ADC_CTRL2_START1_MASK  (0x2000U)
#define ADC_CTRL2_START1_SHIFT (13U)
/*! START1 - START1 Conversion
 *  0b0..No action
 *  0b1..Start command is issued
 */
#define ADC_CTRL2_START1(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_START1_SHIFT)) & ADC_CTRL2_START1_MASK)

#define ADC_CTRL2_STOP1_MASK  (0x4000U)
#define ADC_CTRL2_STOP1_SHIFT (14U)
/*! STOP1 - Stop
 *  0b0..Normal operation
 *  0b1..Stop mode
 */
#define ADC_CTRL2_STOP1(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_STOP1_SHIFT)) & ADC_CTRL2_STOP1_MASK)

#define ADC_CTRL2_DMAEN1_MASK  (0x8000U)
#define ADC_CTRL2_DMAEN1_SHIFT (15U)
/*! DMAEN1 - DMA enable
 *  0b0..DMA is not enabled.
 *  0b1..DMA is enabled.
 */
#define ADC_CTRL2_DMAEN1(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_DMAEN1_SHIFT)) & ADC_CTRL2_DMAEN1_MASK)
/*! @} */

/*! @name ZXCTRL1 - ADC Zero Crossing Control 1 Register */
/*! @{ */

#define ADC_ZXCTRL1_ZCE0_MASK  (0x3U)
#define ADC_ZXCTRL1_ZCE0_SHIFT (0U)
/*! ZCE0 - Zero crossing enable 0
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE0(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE0_SHIFT)) & ADC_ZXCTRL1_ZCE0_MASK)

#define ADC_ZXCTRL1_ZCE1_MASK  (0xCU)
#define ADC_ZXCTRL1_ZCE1_SHIFT (2U)
/*! ZCE1 - Zero crossing enable 1
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE1(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE1_SHIFT)) & ADC_ZXCTRL1_ZCE1_MASK)

#define ADC_ZXCTRL1_ZCE2_MASK  (0x30U)
#define ADC_ZXCTRL1_ZCE2_SHIFT (4U)
/*! ZCE2 - Zero crossing enable 2
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE2(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE2_SHIFT)) & ADC_ZXCTRL1_ZCE2_MASK)

#define ADC_ZXCTRL1_ZCE3_MASK  (0xC0U)
#define ADC_ZXCTRL1_ZCE3_SHIFT (6U)
/*! ZCE3 - Zero crossing enable 3
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE3(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE3_SHIFT)) & ADC_ZXCTRL1_ZCE3_MASK)

#define ADC_ZXCTRL1_ZCE4_MASK  (0x300U)
#define ADC_ZXCTRL1_ZCE4_SHIFT (8U)
/*! ZCE4 - Zero crossing enable 4
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE4(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE4_SHIFT)) & ADC_ZXCTRL1_ZCE4_MASK)

#define ADC_ZXCTRL1_ZCE5_MASK  (0xC00U)
#define ADC_ZXCTRL1_ZCE5_SHIFT (10U)
/*! ZCE5 - Zero crossing enable 5
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE5(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE5_SHIFT)) & ADC_ZXCTRL1_ZCE5_MASK)

#define ADC_ZXCTRL1_ZCE6_MASK  (0x3000U)
#define ADC_ZXCTRL1_ZCE6_SHIFT (12U)
/*! ZCE6 - Zero crossing enable 6
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE6(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE6_SHIFT)) & ADC_ZXCTRL1_ZCE6_MASK)

#define ADC_ZXCTRL1_ZCE7_MASK  (0xC000U)
#define ADC_ZXCTRL1_ZCE7_SHIFT (14U)
/*! ZCE7 - Zero crossing enable 7
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE7(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE7_SHIFT)) & ADC_ZXCTRL1_ZCE7_MASK)
/*! @} */

/*! @name ZXCTRL2 - ADC Zero Crossing Control 2 Register */
/*! @{ */

#define ADC_ZXCTRL2_ZCE8_MASK  (0x3U)
#define ADC_ZXCTRL2_ZCE8_SHIFT (0U)
/*! ZCE8 - Zero crossing enable 8
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE8(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE8_SHIFT)) & ADC_ZXCTRL2_ZCE8_MASK)

#define ADC_ZXCTRL2_ZCE9_MASK  (0xCU)
#define ADC_ZXCTRL2_ZCE9_SHIFT (2U)
/*! ZCE9 - Zero crossing enable 9
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE9(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE9_SHIFT)) & ADC_ZXCTRL2_ZCE9_MASK)

#define ADC_ZXCTRL2_ZCE10_MASK  (0x30U)
#define ADC_ZXCTRL2_ZCE10_SHIFT (4U)
/*! ZCE10 - Zero crossing enable 10
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE10(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE10_SHIFT)) & ADC_ZXCTRL2_ZCE10_MASK)

#define ADC_ZXCTRL2_ZCE11_MASK  (0xC0U)
#define ADC_ZXCTRL2_ZCE11_SHIFT (6U)
/*! ZCE11 - Zero crossing enable 11
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE11(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE11_SHIFT)) & ADC_ZXCTRL2_ZCE11_MASK)

#define ADC_ZXCTRL2_ZCE12_MASK  (0x300U)
#define ADC_ZXCTRL2_ZCE12_SHIFT (8U)
/*! ZCE12 - Zero crossing enable 12
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE12(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE12_SHIFT)) & ADC_ZXCTRL2_ZCE12_MASK)

#define ADC_ZXCTRL2_ZCE13_MASK  (0xC00U)
#define ADC_ZXCTRL2_ZCE13_SHIFT (10U)
/*! ZCE13 - Zero crossing enable 13
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE13(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE13_SHIFT)) & ADC_ZXCTRL2_ZCE13_MASK)

#define ADC_ZXCTRL2_ZCE14_MASK  (0x3000U)
#define ADC_ZXCTRL2_ZCE14_SHIFT (12U)
/*! ZCE14 - Zero crossing enable 14
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE14(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE14_SHIFT)) & ADC_ZXCTRL2_ZCE14_MASK)

#define ADC_ZXCTRL2_ZCE15_MASK  (0xC000U)
#define ADC_ZXCTRL2_ZCE15_SHIFT (14U)
/*! ZCE15 - Zero crossing enable 15
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE15(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE15_SHIFT)) & ADC_ZXCTRL2_ZCE15_MASK)
/*! @} */

/*! @name CLIST1 - ADC Channel List Register 1 */
/*! @{ */

#define ADC_CLIST1_SAMPLE0_MASK  (0xFU)
#define ADC_CLIST1_SAMPLE0_SHIFT (0U)
/*! SAMPLE0 - Sample Field 0
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST1_SAMPLE0(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST1_SAMPLE0_SHIFT)) & ADC_CLIST1_SAMPLE0_MASK)

#define ADC_CLIST1_SAMPLE1_MASK  (0xF0U)
#define ADC_CLIST1_SAMPLE1_SHIFT (4U)
/*! SAMPLE1 - Sample Field 1
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST1_SAMPLE1(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST1_SAMPLE1_SHIFT)) & ADC_CLIST1_SAMPLE1_MASK)

#define ADC_CLIST1_SAMPLE2_MASK  (0xF00U)
#define ADC_CLIST1_SAMPLE2_SHIFT (8U)
/*! SAMPLE2 - Sample Field 2
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST1_SAMPLE2(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST1_SAMPLE2_SHIFT)) & ADC_CLIST1_SAMPLE2_MASK)

#define ADC_CLIST1_SAMPLE3_MASK  (0xF000U)
#define ADC_CLIST1_SAMPLE3_SHIFT (12U)
/*! SAMPLE3 - Sample Field 3
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST1_SAMPLE3(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST1_SAMPLE3_SHIFT)) & ADC_CLIST1_SAMPLE3_MASK)
/*! @} */

/*! @name CLIST2 - ADC Channel List Register 2 */
/*! @{ */

#define ADC_CLIST2_SAMPLE4_MASK  (0xFU)
#define ADC_CLIST2_SAMPLE4_SHIFT (0U)
/*! SAMPLE4 - Sample Field 4
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST2_SAMPLE4(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST2_SAMPLE4_SHIFT)) & ADC_CLIST2_SAMPLE4_MASK)

#define ADC_CLIST2_SAMPLE5_MASK  (0xF0U)
#define ADC_CLIST2_SAMPLE5_SHIFT (4U)
/*! SAMPLE5 - Sample Field 5
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST2_SAMPLE5(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST2_SAMPLE5_SHIFT)) & ADC_CLIST2_SAMPLE5_MASK)

#define ADC_CLIST2_SAMPLE6_MASK  (0xF00U)
#define ADC_CLIST2_SAMPLE6_SHIFT (8U)
/*! SAMPLE6 - Sample Field 6
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST2_SAMPLE6(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST2_SAMPLE6_SHIFT)) & ADC_CLIST2_SAMPLE6_MASK)

#define ADC_CLIST2_SAMPLE7_MASK  (0xF000U)
#define ADC_CLIST2_SAMPLE7_SHIFT (12U)
/*! SAMPLE7 - Sample Field 7
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST2_SAMPLE7(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST2_SAMPLE7_SHIFT)) & ADC_CLIST2_SAMPLE7_MASK)
/*! @} */

/*! @name CLIST3 - ADC Channel List Register 3 */
/*! @{ */

#define ADC_CLIST3_SAMPLE8_MASK  (0xFU)
#define ADC_CLIST3_SAMPLE8_SHIFT (0U)
/*! SAMPLE8 - Sample Field 8
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST3_SAMPLE8(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST3_SAMPLE8_SHIFT)) & ADC_CLIST3_SAMPLE8_MASK)

#define ADC_CLIST3_SAMPLE9_MASK  (0xF0U)
#define ADC_CLIST3_SAMPLE9_SHIFT (4U)
/*! SAMPLE9 - Sample Field 9
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST3_SAMPLE9(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST3_SAMPLE9_SHIFT)) & ADC_CLIST3_SAMPLE9_MASK)

#define ADC_CLIST3_SAMPLE10_MASK  (0xF00U)
#define ADC_CLIST3_SAMPLE10_SHIFT (8U)
/*! SAMPLE10 - Sample Field 10
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST3_SAMPLE10(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST3_SAMPLE10_SHIFT)) & ADC_CLIST3_SAMPLE10_MASK)

#define ADC_CLIST3_SAMPLE11_MASK  (0xF000U)
#define ADC_CLIST3_SAMPLE11_SHIFT (12U)
/*! SAMPLE11 - Sample Field 11
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST3_SAMPLE11(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST3_SAMPLE11_SHIFT)) & ADC_CLIST3_SAMPLE11_MASK)
/*! @} */

/*! @name CLIST4 - ADC Channel List Register 4 */
/*! @{ */

#define ADC_CLIST4_SAMPLE12_MASK  (0xFU)
#define ADC_CLIST4_SAMPLE12_SHIFT (0U)
/*! SAMPLE12 - Sample Field 12
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST4_SAMPLE12(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST4_SAMPLE12_SHIFT)) & ADC_CLIST4_SAMPLE12_MASK)

#define ADC_CLIST4_SAMPLE13_MASK  (0xF0U)
#define ADC_CLIST4_SAMPLE13_SHIFT (4U)
/*! SAMPLE13 - Sample Field 13
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST4_SAMPLE13(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST4_SAMPLE13_SHIFT)) & ADC_CLIST4_SAMPLE13_MASK)

#define ADC_CLIST4_SAMPLE14_MASK  (0xF00U)
#define ADC_CLIST4_SAMPLE14_SHIFT (8U)
/*! SAMPLE14 - Sample Field 14
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST4_SAMPLE14(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST4_SAMPLE14_SHIFT)) & ADC_CLIST4_SAMPLE14_MASK)

#define ADC_CLIST4_SAMPLE15_MASK  (0xF000U)
#define ADC_CLIST4_SAMPLE15_SHIFT (12U)
/*! SAMPLE15 - Sample Field 15
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST4_SAMPLE15(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST4_SAMPLE15_SHIFT)) & ADC_CLIST4_SAMPLE15_MASK)
/*! @} */

/*! @name SDIS - ADC Sample Disable Register */
/*! @{ */

#define ADC_SDIS_DS_MASK  (0xFFFFU)
#define ADC_SDIS_DS_SHIFT (0U)
/*! DS - Disable Sample Bits
 *  0b0000000000000000..SAMPLEx channel is enabled for ADC scan.
 *  0b0000000000000001..SAMPLEx channel is disabled for ADC scan and corresponding channels after SAMPLEx will also not
 * occur in an ADC scan.
 */
#define ADC_SDIS_DS(x) (((uint16_t)(((uint16_t)(x)) << ADC_SDIS_DS_SHIFT)) & ADC_SDIS_DS_MASK)
/*! @} */

/*! @name STAT - ADC Status Register */
/*! @{ */

#define ADC_STAT_HLMTI_MASK  (0x100U)
#define ADC_STAT_HLMTI_SHIFT (8U)
/*! HLMTI - High Limit Interrupt
 *  0b0..No high limit interrupt request
 *  0b1..High limit exceeded, IRQ pending if CTRL1[HLMTIE] is set
 */
#define ADC_STAT_HLMTI(x) (((uint16_t)(((uint16_t)(x)) << ADC_STAT_HLMTI_SHIFT)) & ADC_STAT_HLMTI_MASK)

#define ADC_STAT_LLMTI_MASK  (0x200U)
#define ADC_STAT_LLMTI_SHIFT (9U)
/*! LLMTI - Low Limit Interrupt
 *  0b0..No low limit interrupt request
 *  0b1..Low limit exceeded, IRQ pending if CTRL1[LLMTIE] is set
 */
#define ADC_STAT_LLMTI(x) (((uint16_t)(((uint16_t)(x)) << ADC_STAT_LLMTI_SHIFT)) & ADC_STAT_LLMTI_MASK)

#define ADC_STAT_ZCI_MASK  (0x400U)
#define ADC_STAT_ZCI_SHIFT (10U)
/*! ZCI - Zero Crossing Interrupt
 *  0b0..No zero crossing interrupt request
 *  0b1..Zero crossing encountered, IRQ pending if CTRL1[ZCIE] is set
 */
#define ADC_STAT_ZCI(x) (((uint16_t)(((uint16_t)(x)) << ADC_STAT_ZCI_SHIFT)) & ADC_STAT_ZCI_MASK)

#define ADC_STAT_EOSI0_MASK  (0x800U)
#define ADC_STAT_EOSI0_SHIFT (11U)
/*! EOSI0 - End of Scan Interrupt
 *  0b0..A scan cycle has not been completed, no end of scan IRQ pending
 *  0b1..A scan cycle has been completed, end of scan IRQ pending
 */
#define ADC_STAT_EOSI0(x) (((uint16_t)(((uint16_t)(x)) << ADC_STAT_EOSI0_SHIFT)) & ADC_STAT_EOSI0_MASK)

#define ADC_STAT_EOSI1_MASK  (0x1000U)
#define ADC_STAT_EOSI1_SHIFT (12U)
/*! EOSI1 - End of Scan Interrupt
 *  0b0..A scan cycle has not been completed, no end of scan IRQ pending
 *  0b1..A scan cycle has been completed, end of scan IRQ pending
 */
#define ADC_STAT_EOSI1(x) (((uint16_t)(((uint16_t)(x)) << ADC_STAT_EOSI1_SHIFT)) & ADC_STAT_EOSI1_MASK)

#define ADC_STAT_CIP1_MASK  (0x4000U)
#define ADC_STAT_CIP1_SHIFT (14U)
/*! CIP1 - Conversion in Progress
 *  0b0..Idle state
 *  0b1..A scan cycle is in progress. The ADC will ignore all sync pulses or start commands
 */
#define ADC_STAT_CIP1(x) (((uint16_t)(((uint16_t)(x)) << ADC_STAT_CIP1_SHIFT)) & ADC_STAT_CIP1_MASK)

#define ADC_STAT_CIP0_MASK  (0x8000U)
#define ADC_STAT_CIP0_SHIFT (15U)
/*! CIP0 - Conversion in Progress
 *  0b0..Idle state
 *  0b1..A scan cycle is in progress. The ADC will ignore all sync pulses or start commands
 */
#define ADC_STAT_CIP0(x) (((uint16_t)(((uint16_t)(x)) << ADC_STAT_CIP0_SHIFT)) & ADC_STAT_CIP0_MASK)
/*! @} */

/*! @name RDY - ADC Ready Register */
/*! @{ */

#define ADC_RDY_RDY_MASK  (0xFFFFU)
#define ADC_RDY_RDY_SHIFT (0U)
/*! RDY - Ready Sample
 *  0b0000000000000000..Sample not ready or has been read
 *  0b0000000000000001..Sample ready to be read
 */
#define ADC_RDY_RDY(x) (((uint16_t)(((uint16_t)(x)) << ADC_RDY_RDY_SHIFT)) & ADC_RDY_RDY_MASK)
/*! @} */

/*! @name LOLIMSTAT - ADC Low Limit Status Register */
/*! @{ */

#define ADC_LOLIMSTAT_LLS_MASK  (0xFFFFU)
#define ADC_LOLIMSTAT_LLS_SHIFT (0U)
/*! LLS - Low Limit Status Bits
 */
#define ADC_LOLIMSTAT_LLS(x) (((uint16_t)(((uint16_t)(x)) << ADC_LOLIMSTAT_LLS_SHIFT)) & ADC_LOLIMSTAT_LLS_MASK)
/*! @} */

/*! @name HILIMSTAT - ADC High Limit Status Register */
/*! @{ */

#define ADC_HILIMSTAT_HLS_MASK  (0xFFFFU)
#define ADC_HILIMSTAT_HLS_SHIFT (0U)
/*! HLS - High Limit Status Bits
 */
#define ADC_HILIMSTAT_HLS(x) (((uint16_t)(((uint16_t)(x)) << ADC_HILIMSTAT_HLS_SHIFT)) & ADC_HILIMSTAT_HLS_MASK)
/*! @} */

/*! @name ZXSTAT - ADC Zero Crossing Status Register */
/*! @{ */

#define ADC_ZXSTAT_ZCS_MASK  (0xFFFFU)
#define ADC_ZXSTAT_ZCS_SHIFT (0U)
/*! ZCS - Zero Crossing Status
 *  0b0000000000000000..Either: A sign change did not occur in a comparison between the current channelx result
 *                      and the previous channelx result, or Zero crossing control is disabled for channelx in the
 *                      zero crossing control register, ZXCTRL
 *  0b0000000000000001..In a comparison between the current channelx result and the previous channelx result, a
 *                      sign change condition occurred as defined in the zero crossing control register (ZXCTRL)
 */
#define ADC_ZXSTAT_ZCS(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXSTAT_ZCS_SHIFT)) & ADC_ZXSTAT_ZCS_MASK)
/*! @} */

/*! @name RSLT - ADC Result Registers with sign extension */
/*! @{ */

#define ADC_RSLT_RSLT_MASK  (0x7FF8U)
#define ADC_RSLT_RSLT_SHIFT (3U)
/*! RSLT - Digital Result of the Conversion
 */
#define ADC_RSLT_RSLT(x) (((uint16_t)(((uint16_t)(x)) << ADC_RSLT_RSLT_SHIFT)) & ADC_RSLT_RSLT_MASK)

#define ADC_RSLT_SEXT_MASK  (0x8000U)
#define ADC_RSLT_SEXT_SHIFT (15U)
/*! SEXT - Sign Extend
 */
#define ADC_RSLT_SEXT(x) (((uint16_t)(((uint16_t)(x)) << ADC_RSLT_SEXT_SHIFT)) & ADC_RSLT_SEXT_MASK)
/*! @} */

/* The count of ADC_RSLT */
#define ADC_RSLT_COUNT (16U)

/*! @name LOLIM - ADC Low Limit Registers */
/*! @{ */

#define ADC_LOLIM_LLMT_MASK  (0x7FF8U)
#define ADC_LOLIM_LLMT_SHIFT (3U)
/*! LLMT - Low Limit Bits
 */
#define ADC_LOLIM_LLMT(x) (((uint16_t)(((uint16_t)(x)) << ADC_LOLIM_LLMT_SHIFT)) & ADC_LOLIM_LLMT_MASK)
/*! @} */

/* The count of ADC_LOLIM */
#define ADC_LOLIM_COUNT (16U)

/*! @name HILIM - ADC High Limit Registers */
/*! @{ */

#define ADC_HILIM_HLMT_MASK  (0x7FF8U)
#define ADC_HILIM_HLMT_SHIFT (3U)
/*! HLMT - High Limit Bits
 */
#define ADC_HILIM_HLMT(x) (((uint16_t)(((uint16_t)(x)) << ADC_HILIM_HLMT_SHIFT)) & ADC_HILIM_HLMT_MASK)
/*! @} */

/* The count of ADC_HILIM */
#define ADC_HILIM_COUNT (16U)

/*! @name OFFST - ADC Offset Registers */
/*! @{ */

#define ADC_OFFST_OFFSET_MASK  (0x7FF8U)
#define ADC_OFFST_OFFSET_SHIFT (3U)
/*! OFFSET - ADC Offset Bits
 */
#define ADC_OFFST_OFFSET(x) (((uint16_t)(((uint16_t)(x)) << ADC_OFFST_OFFSET_SHIFT)) & ADC_OFFST_OFFSET_MASK)
/*! @} */

/* The count of ADC_OFFST */
#define ADC_OFFST_COUNT (16U)

/*! @name PWR - ADC Power Control Register */
/*! @{ */

#define ADC_PWR_PD0_MASK  (0x1U)
#define ADC_PWR_PD0_SHIFT (0U)
/*! PD0 - Manual Power Down for Converter A
 *  0b0..Power Up ADC converter A
 *  0b1..Power Down ADC converter A
 */
#define ADC_PWR_PD0(x) (((uint16_t)(((uint16_t)(x)) << ADC_PWR_PD0_SHIFT)) & ADC_PWR_PD0_MASK)

#define ADC_PWR_PD1_MASK  (0x2U)
#define ADC_PWR_PD1_SHIFT (1U)
/*! PD1 - Manual Power Down for Converter B
 *  0b0..Power Up ADC converter B
 *  0b1..Power Down ADC converter B
 */
#define ADC_PWR_PD1(x) (((uint16_t)(((uint16_t)(x)) << ADC_PWR_PD1_SHIFT)) & ADC_PWR_PD1_MASK)

#define ADC_PWR_APD_MASK  (0x8U)
#define ADC_PWR_APD_SHIFT (3U)
/*! APD - Auto Powerdown
 *  0b0..Auto Powerdown Mode is not active
 *  0b1..Auto Powerdown Mode is active
 */
#define ADC_PWR_APD(x) (((uint16_t)(((uint16_t)(x)) << ADC_PWR_APD_SHIFT)) & ADC_PWR_APD_MASK)

#define ADC_PWR_PUDELAY_MASK  (0x3F0U)
#define ADC_PWR_PUDELAY_SHIFT (4U)
/*! PUDELAY - Power Up Delay
 */
#define ADC_PWR_PUDELAY(x) (((uint16_t)(((uint16_t)(x)) << ADC_PWR_PUDELAY_SHIFT)) & ADC_PWR_PUDELAY_MASK)

#define ADC_PWR_PSTS0_MASK  (0x400U)
#define ADC_PWR_PSTS0_SHIFT (10U)
/*! PSTS0 - ADC Converter A Power Status
 *  0b0..ADC Converter A is currently powered up
 *  0b1..ADC Converter A is currently powered down
 */
#define ADC_PWR_PSTS0(x) (((uint16_t)(((uint16_t)(x)) << ADC_PWR_PSTS0_SHIFT)) & ADC_PWR_PSTS0_MASK)

#define ADC_PWR_PSTS1_MASK  (0x800U)
#define ADC_PWR_PSTS1_SHIFT (11U)
/*! PSTS1 - ADC Converter B Power Status
 *  0b0..ADC Converter B is currently powered up
 *  0b1..ADC Converter B is currently powered down
 */
#define ADC_PWR_PSTS1(x) (((uint16_t)(((uint16_t)(x)) << ADC_PWR_PSTS1_SHIFT)) & ADC_PWR_PSTS1_MASK)
/*! @} */

/*! @name CAL - ADC Calibration Register */
/*! @{ */

#define ADC_CAL_SEL_VREFL_A_MASK  (0x1000U)
#define ADC_CAL_SEL_VREFL_A_SHIFT (12U)
/*! SEL_VREFL_A - Select V REFLO Source
 *  0b0..Internal VSSA
 *  0b1..ANA3
 */
#define ADC_CAL_SEL_VREFL_A(x) (((uint16_t)(((uint16_t)(x)) << ADC_CAL_SEL_VREFL_A_SHIFT)) & ADC_CAL_SEL_VREFL_A_MASK)

#define ADC_CAL_SEL_VREFH_A_MASK  (0x2000U)
#define ADC_CAL_SEL_VREFH_A_SHIFT (13U)
/*! SEL_VREFH_A - Select V REFH Source
 *  0b0..Internal VDDA
 *  0b1..ANA2
 */
#define ADC_CAL_SEL_VREFH_A(x) (((uint16_t)(((uint16_t)(x)) << ADC_CAL_SEL_VREFH_A_SHIFT)) & ADC_CAL_SEL_VREFH_A_MASK)

#define ADC_CAL_SEL_VREFL_B_MASK  (0x4000U)
#define ADC_CAL_SEL_VREFL_B_SHIFT (14U)
/*! SEL_VREFL_B - Select V REFLO Source
 *  0b0..Internal VSSA
 *  0b1..ANB3
 */
#define ADC_CAL_SEL_VREFL_B(x) (((uint16_t)(((uint16_t)(x)) << ADC_CAL_SEL_VREFL_B_SHIFT)) & ADC_CAL_SEL_VREFL_B_MASK)

#define ADC_CAL_SEL_VREFH_B_MASK  (0x8000U)
#define ADC_CAL_SEL_VREFH_B_SHIFT (15U)
/*! SEL_VREFH_B - Select V REFH Source
 *  0b0..Internal VDDA
 *  0b1..ANB2
 */
#define ADC_CAL_SEL_VREFH_B(x) (((uint16_t)(((uint16_t)(x)) << ADC_CAL_SEL_VREFH_B_SHIFT)) & ADC_CAL_SEL_VREFH_B_MASK)
/*! @} */

/*! @name GC1 - Gain Control 1 Register */
/*! @{ */

#define ADC_GC1_GAIN0_MASK  (0x3U)
#define ADC_GC1_GAIN0_SHIFT (0U)
/*! GAIN0 - Gain Control Bit 0
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN0(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN0_SHIFT)) & ADC_GC1_GAIN0_MASK)

#define ADC_GC1_GAIN1_MASK  (0xCU)
#define ADC_GC1_GAIN1_SHIFT (2U)
/*! GAIN1 - Gain Control Bit 1
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN1(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN1_SHIFT)) & ADC_GC1_GAIN1_MASK)

#define ADC_GC1_GAIN2_MASK  (0x30U)
#define ADC_GC1_GAIN2_SHIFT (4U)
/*! GAIN2 - Gain Control Bit 2
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN2(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN2_SHIFT)) & ADC_GC1_GAIN2_MASK)

#define ADC_GC1_GAIN3_MASK  (0xC0U)
#define ADC_GC1_GAIN3_SHIFT (6U)
/*! GAIN3 - Gain Control Bit 3
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN3(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN3_SHIFT)) & ADC_GC1_GAIN3_MASK)

#define ADC_GC1_GAIN4_MASK  (0x300U)
#define ADC_GC1_GAIN4_SHIFT (8U)
/*! GAIN4 - Gain Control Bit 4
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN4(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN4_SHIFT)) & ADC_GC1_GAIN4_MASK)

#define ADC_GC1_GAIN5_MASK  (0xC00U)
#define ADC_GC1_GAIN5_SHIFT (10U)
/*! GAIN5 - Gain Control Bit 5
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN5(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN5_SHIFT)) & ADC_GC1_GAIN5_MASK)

#define ADC_GC1_GAIN6_MASK  (0x3000U)
#define ADC_GC1_GAIN6_SHIFT (12U)
/*! GAIN6 - Gain Control Bit 6
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN6(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN6_SHIFT)) & ADC_GC1_GAIN6_MASK)

#define ADC_GC1_GAIN7_MASK  (0xC000U)
#define ADC_GC1_GAIN7_SHIFT (14U)
/*! GAIN7 - Gain Control Bit 7
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN7(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN7_SHIFT)) & ADC_GC1_GAIN7_MASK)
/*! @} */

/*! @name GC2 - Gain Control 2 Register */
/*! @{ */

#define ADC_GC2_GAIN8_MASK  (0x3U)
#define ADC_GC2_GAIN8_SHIFT (0U)
/*! GAIN8 - Gain Control Bit 8
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN8(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN8_SHIFT)) & ADC_GC2_GAIN8_MASK)

#define ADC_GC2_GAIN9_MASK  (0xCU)
#define ADC_GC2_GAIN9_SHIFT (2U)
/*! GAIN9 - Gain Control Bit 9
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN9(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN9_SHIFT)) & ADC_GC2_GAIN9_MASK)

#define ADC_GC2_GAIN10_MASK  (0x30U)
#define ADC_GC2_GAIN10_SHIFT (4U)
/*! GAIN10 - Gain Control Bit 10
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN10(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN10_SHIFT)) & ADC_GC2_GAIN10_MASK)

#define ADC_GC2_GAIN11_MASK  (0xC0U)
#define ADC_GC2_GAIN11_SHIFT (6U)
/*! GAIN11 - Gain Control Bit 11
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN11(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN11_SHIFT)) & ADC_GC2_GAIN11_MASK)

#define ADC_GC2_GAIN12_MASK  (0x300U)
#define ADC_GC2_GAIN12_SHIFT (8U)
/*! GAIN12 - Gain Control Bit 12
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN12(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN12_SHIFT)) & ADC_GC2_GAIN12_MASK)

#define ADC_GC2_GAIN13_MASK  (0xC00U)
#define ADC_GC2_GAIN13_SHIFT (10U)
/*! GAIN13 - Gain Control Bit 13
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN13(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN13_SHIFT)) & ADC_GC2_GAIN13_MASK)

#define ADC_GC2_GAIN14_MASK  (0x3000U)
#define ADC_GC2_GAIN14_SHIFT (12U)
/*! GAIN14 - Gain Control Bit 14
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN14(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN14_SHIFT)) & ADC_GC2_GAIN14_MASK)

#define ADC_GC2_GAIN15_MASK  (0xC000U)
#define ADC_GC2_GAIN15_SHIFT (14U)
/*! GAIN15 - Gain Control Bit 15
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN15(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN15_SHIFT)) & ADC_GC2_GAIN15_MASK)
/*! @} */

/*! @name SCTRL - ADC Scan Control Register */
/*! @{ */

#define ADC_SCTRL_SC_MASK  (0xFFFFU)
#define ADC_SCTRL_SC_SHIFT (0U)
/*! SC - Scan Control Bits
 *  0b0000000000000000..Perform sample immediately after the completion of the current sample.
 *  0b0000000000000001..Delay sample until a new sync input occurs.
 */
#define ADC_SCTRL_SC(x) (((uint16_t)(((uint16_t)(x)) << ADC_SCTRL_SC_SHIFT)) & ADC_SCTRL_SC_MASK)
/*! @} */

/*! @name PWR2 - ADC Power Control Register 2 */
/*! @{ */

#define ADC_PWR2_DIV1_MASK  (0x3F00U)
#define ADC_PWR2_DIV1_SHIFT (8U)
/*! DIV1 - Clock Divisor Select
 */
#define ADC_PWR2_DIV1(x) (((uint16_t)(((uint16_t)(x)) << ADC_PWR2_DIV1_SHIFT)) & ADC_PWR2_DIV1_MASK)
/*! @} */

/*! @name CTRL3 - ADC Control Register 3 */
/*! @{ */

#define ADC_CTRL3_DMASRC_MASK  (0x40U)
#define ADC_CTRL3_DMASRC_SHIFT (6U)
/*! DMASRC - DMA Trigger Source
 *  0b0..DMA trigger source is end of scan interrupt
 *  0b1..DMA trigger source is RDY bits
 */
#define ADC_CTRL3_DMASRC(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL3_DMASRC_SHIFT)) & ADC_CTRL3_DMASRC_MASK)

#define ADC_CTRL3_UPDEN_L_MASK  (0xF00U)
#define ADC_CTRL3_UPDEN_L_SHIFT (8U)
/*! UPDEN_L - Unipolar Differential Enable Low bits
 *  0bxxx1..Inputs = ANA0-ANA1 : Unipolar differential mode enabled on ANA0-ANA1
 *  0bxxx0..Inputs = ANA0-ANA1 : Fully differential mode enabled on ANA0-ANA1
 *  0bxx1x..Inputs = ANA2-ANA3 : Unipolar differential mode enabled on ANA2-ANA3
 *  0bxx0x..Inputs = ANA2-ANA3 : Fully differential mode enabled on ANA2-ANA3
 *  0bx1xx..Inputs = ANB0-ANB1 : Unipolar differential mode enabled on ANB0-ANB1
 *  0bx0xx..Inputs = ANB0-ANB1 : Fully differential mode enabled on ANB0-ANB1
 *  0b1xxx..Inputs = ANB2-ANB3 : Unipolar differential mode enabled on ANB2-ANB3
 *  0b0xxx..Inputs = ANB2-ANB3 : Fully differential mode enabled on ANB2-ANB3
 */
#define ADC_CTRL3_UPDEN_L(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL3_UPDEN_L_SHIFT)) & ADC_CTRL3_UPDEN_L_MASK)

#define ADC_CTRL3_UPDEN_H_MASK  (0xF000U)
#define ADC_CTRL3_UPDEN_H_SHIFT (12U)
/*! UPDEN_H - Unipolar Differential Enable High bits
 *  0bxxx1..Inputs = ANA4-ANA5 : Unipolar differential mode enabled on ANA4-ANA5
 *  0bxxx0..Inputs = ANA4-ANA5 : Fully differential mode enabled on ANA4-ANA5
 *  0bxx1x..Inputs = ANA6-ANA7 : Unipolar differential mode enabled on ANA6-ANA7
 *  0bxx0x..Inputs = ANA6-ANA7 : Fully differential mode enabled on ANA6-ANA7
 *  0bx1xx..Inputs = ANB4-ANB5 : Unipolar differential mode enabled on ANB4-ANB5
 *  0bx0xx..Inputs = ANB4-ANB5 : Fully differential mode enabled on ANB4-ANB5
 *  0b1xxx..Inputs = ANB6-ANB7 : Unipolar differential mode enabled on ANB6-ANB7
 *  0b0xxx..Inputs = ANB6-ANB7 : Fully differential mode enabled on ANB6-ANB7
 */
#define ADC_CTRL3_UPDEN_H(x) (((uint16_t)(((uint16_t)(x)) << ADC_CTRL3_UPDEN_H_SHIFT)) & ADC_CTRL3_UPDEN_H_MASK)
/*! @} */

/*! @name SCHLTEN - ADC Scan Interrupt Enable Register */
/*! @{ */

#define ADC_SCHLTEN_SCHLTEN_MASK  (0xFFFFU)
#define ADC_SCHLTEN_SCHLTEN_SHIFT (0U)
/*! SCHLTEN - SCHLTEN
 *  0b0000000000000000..Scan interrupt is not enabled for this sample.
 *  0b0000000000000001..Scan interrupt is enabled for this sample.
 */
#define ADC_SCHLTEN_SCHLTEN(x) (((uint16_t)(((uint16_t)(x)) << ADC_SCHLTEN_SCHLTEN_SHIFT)) & ADC_SCHLTEN_SCHLTEN_MASK)
/*! @} */

/*! @name ZXCTRL3 - ADC Zero Crossing Control 3 Register */
/*! @{ */

#define ADC_ZXCTRL3_ZCE16_MASK  (0x3U)
#define ADC_ZXCTRL3_ZCE16_SHIFT (0U)
/*! ZCE16 - Zero crossing enable 16
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL3_ZCE16(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL3_ZCE16_SHIFT)) & ADC_ZXCTRL3_ZCE16_MASK)

#define ADC_ZXCTRL3_ZCE17_MASK  (0xCU)
#define ADC_ZXCTRL3_ZCE17_SHIFT (2U)
/*! ZCE17 - Zero crossing enable 17
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL3_ZCE17(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL3_ZCE17_SHIFT)) & ADC_ZXCTRL3_ZCE17_MASK)

#define ADC_ZXCTRL3_ZCE18_MASK  (0x30U)
#define ADC_ZXCTRL3_ZCE18_SHIFT (4U)
/*! ZCE18 - Zero crossing enable 18
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL3_ZCE18(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL3_ZCE18_SHIFT)) & ADC_ZXCTRL3_ZCE18_MASK)

#define ADC_ZXCTRL3_ZCE19_MASK  (0xC0U)
#define ADC_ZXCTRL3_ZCE19_SHIFT (6U)
/*! ZCE19 - Zero crossing enable 19
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL3_ZCE19(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL3_ZCE19_SHIFT)) & ADC_ZXCTRL3_ZCE19_MASK)
/*! @} */

/*! @name CLIST5 - ADC Channel List Register 5 */
/*! @{ */

#define ADC_CLIST5_SAMPLE16_MASK  (0x3U)
#define ADC_CLIST5_SAMPLE16_SHIFT (0U)
/*! SAMPLE16 - Sample Field 16
 *  0b00..Single Ended: ADCA temperature sensor
 *  0b01..Single Ended: ADCA analog input for on-chip generated signals
 *  0b10..Single Ended: ADCB temperature sensor
 *  0b11..Single Ended: ADCB analog input for on-chip generated signals
 */
#define ADC_CLIST5_SAMPLE16(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SAMPLE16_SHIFT)) & ADC_CLIST5_SAMPLE16_MASK)

#define ADC_CLIST5_SAMPLE17_MASK  (0xCU)
#define ADC_CLIST5_SAMPLE17_SHIFT (2U)
/*! SAMPLE17 - Sample Field 17
 *  0b00..Single Ended: ADCA temperature sensor
 *  0b01..Single Ended: ADCA analog input for on-chip generated signals
 *  0b10..Single Ended: ADCB temperature sensor
 *  0b11..Single Ended: ADCB analog input for on-chip generated signals
 */
#define ADC_CLIST5_SAMPLE17(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SAMPLE17_SHIFT)) & ADC_CLIST5_SAMPLE17_MASK)

#define ADC_CLIST5_SAMPLE18_MASK  (0x30U)
#define ADC_CLIST5_SAMPLE18_SHIFT (4U)
/*! SAMPLE18 - Sample Field 18
 *  0b00..Single Ended: ADCA temperature sensor
 *  0b01..Single Ended: ADCA analog input for on-chip generated signals
 *  0b10..Single Ended: ADCB temperature sensor
 *  0b11..Single Ended: ADC B analog input for on-chip generated signals
 */
#define ADC_CLIST5_SAMPLE18(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SAMPLE18_SHIFT)) & ADC_CLIST5_SAMPLE18_MASK)

#define ADC_CLIST5_SAMPLE19_MASK  (0xC0U)
#define ADC_CLIST5_SAMPLE19_SHIFT (6U)
/*! SAMPLE19 - Sample Field 19
 *  0b00..Single Ended: ADCA temperature sensor
 *  0b01..Single Ended: ADCA analog input for on-chip generated signals
 *  0b10..Single Ended: ADCB temperature sensor
 *  0b11..Single Ended: ADCB analog input for on-chip generated signals
 */
#define ADC_CLIST5_SAMPLE19(x) (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SAMPLE19_SHIFT)) & ADC_CLIST5_SAMPLE19_MASK)

#define ADC_CLIST5_SEL_TEMP_0_MASK  (0x100U)
#define ADC_CLIST5_SEL_TEMP_0_SHIFT (8U)
/*! SEL_TEMP_0 - Select Temperature Sensor Alternate Source
 *  0b0..Normal Operation (ADCA6)
 *  0b1..ADCA6 input is replaced with ADCA temperature sensor
 */
#define ADC_CLIST5_SEL_TEMP_0(x) \
    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SEL_TEMP_0_SHIFT)) & ADC_CLIST5_SEL_TEMP_0_MASK)

#define ADC_CLIST5_SEL_INTERNAL_0_MASK  (0x200U)
#define ADC_CLIST5_SEL_INTERNAL_0_SHIFT (9U)
/*! SEL_INTERNAL_0 - Select On-Chip Analog Input Alternate Source
 *  0b0..Normal Operation (ADCA7)
 *  0b1..ADCA7 input is replaced with ADCA on-chip analog input
 */
#define ADC_CLIST5_SEL_INTERNAL_0(x) \
    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SEL_INTERNAL_0_SHIFT)) & ADC_CLIST5_SEL_INTERNAL_0_MASK)

#define ADC_CLIST5_SEL_TEMP_1_MASK  (0x400U)
#define ADC_CLIST5_SEL_TEMP_1_SHIFT (10U)
/*! SEL_TEMP_1 - Select Temperature Sensor Alternate Source
 *  0b0..Normal Operation (ADCB6)
 *  0b1..ADCB6 input is replaced with ADCB temperature sensor
 */
#define ADC_CLIST5_SEL_TEMP_1(x) \
    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SEL_TEMP_1_SHIFT)) & ADC_CLIST5_SEL_TEMP_1_MASK)

#define ADC_CLIST5_SEL_INTERNAL_1_MASK  (0x800U)
#define ADC_CLIST5_SEL_INTERNAL_1_SHIFT (11U)
/*! SEL_INTERNAL_1 - Select On-Chip Analog Input Alternate Source
 *  0b0..Normal operation (ADCB7)
 *  0b1..ADCB7 input is replaced with ADCB on-chip analog input
 */
#define ADC_CLIST5_SEL_INTERNAL_1(x) \
    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SEL_INTERNAL_1_SHIFT)) & ADC_CLIST5_SEL_INTERNAL_1_MASK)
/*! @} */

/*! @name SDIS2 - ADC Sample Disable Register 2 */
/*! @{ */

#define ADC_SDIS2_DS_MASK  (0xFU)
#define ADC_SDIS2_DS_SHIFT (0U)
/*! DS - Disable Sample Bits
 *  0b0000..SAMPLEx channel is enabled for ADC scan.
 *  0b0001..SAMPLEx channel is disabled for ADC scan and corresponding channels after SAMPLEx will also not occur
 *          in an ADC scan. NOTE: Please note that enabling the four extra sample slots by themselves
 *          (ADC_SDIS=FFFF, and samples enabled in ADC_SDIS2 ) is supported only in once sequential mode. Sequential
 * loop, and parallel (both sequential and loop) modes are not supported. It is suggested to poll the ADC_RDY2 register
 * to check for conversion completion for this case.
 */
#define ADC_SDIS2_DS(x) (((uint16_t)(((uint16_t)(x)) << ADC_SDIS2_DS_SHIFT)) & ADC_SDIS2_DS_MASK)
/*! @} */

/*! @name RDY2 - ADC Ready Register 2 */
/*! @{ */

#define ADC_RDY2_RDY_MASK  (0xFU)
#define ADC_RDY2_RDY_SHIFT (0U)
/*! RDY - Ready Sample
 *  0b0000..Sample not ready or has been read
 *  0b0001..Sample ready to be read
 */
#define ADC_RDY2_RDY(x) (((uint16_t)(((uint16_t)(x)) << ADC_RDY2_RDY_SHIFT)) & ADC_RDY2_RDY_MASK)
/*! @} */

/*! @name LOLIMSTAT2 - ADC Low Limit Status Register 2 */
/*! @{ */

#define ADC_LOLIMSTAT2_LLS_MASK  (0xFU)
#define ADC_LOLIMSTAT2_LLS_SHIFT (0U)
/*! LLS - Low Limit Status Bits
 */
#define ADC_LOLIMSTAT2_LLS(x) (((uint16_t)(((uint16_t)(x)) << ADC_LOLIMSTAT2_LLS_SHIFT)) & ADC_LOLIMSTAT2_LLS_MASK)
/*! @} */

/*! @name HILIMSTAT2 - ADC High Limit Status Register 2 */
/*! @{ */

#define ADC_HILIMSTAT2_HLS_MASK  (0xFU)
#define ADC_HILIMSTAT2_HLS_SHIFT (0U)
/*! HLS - High Limit Status Bits
 */
#define ADC_HILIMSTAT2_HLS(x) (((uint16_t)(((uint16_t)(x)) << ADC_HILIMSTAT2_HLS_SHIFT)) & ADC_HILIMSTAT2_HLS_MASK)
/*! @} */

/*! @name ZXSTAT2 - ADC Zero Crossing Status Register 2 */
/*! @{ */

#define ADC_ZXSTAT2_ZCS_MASK  (0xFU)
#define ADC_ZXSTAT2_ZCS_SHIFT (0U)
/*! ZCS - Zero Crossing Status
 *  0b0000..Either: A sign change did not occur in a comparison between the current channelx result and the
 *          previous channelx result, or Zero crossing control is disabled for channelx in the zero crossing control
 *          register, ZXCTRL3
 *  0b0001..In a comparison between the current channelx result and the previous channelx result, a sign change
 *          condition occurred as defined in the zero crossing control register (ZXCTRL3)
 */
#define ADC_ZXSTAT2_ZCS(x) (((uint16_t)(((uint16_t)(x)) << ADC_ZXSTAT2_ZCS_SHIFT)) & ADC_ZXSTAT2_ZCS_MASK)
/*! @} */

/*! @name RSLT2 - ADC Result Registers 2 with sign extension */
/*! @{ */

#define ADC_RSLT2_RSLT_MASK  (0x7FF8U)
#define ADC_RSLT2_RSLT_SHIFT (3U)
/*! RSLT - Digital Result of the Conversion
 */
#define ADC_RSLT2_RSLT(x) (((uint16_t)(((uint16_t)(x)) << ADC_RSLT2_RSLT_SHIFT)) & ADC_RSLT2_RSLT_MASK)

#define ADC_RSLT2_SEXT_MASK  (0x8000U)
#define ADC_RSLT2_SEXT_SHIFT (15U)
/*! SEXT - Sign Extend
 */
#define ADC_RSLT2_SEXT(x) (((uint16_t)(((uint16_t)(x)) << ADC_RSLT2_SEXT_SHIFT)) & ADC_RSLT2_SEXT_MASK)
/*! @} */

/* The count of ADC_RSLT2 */
#define ADC_RSLT2_COUNT (4U)

/*! @name LOLIM2 - ADC Low Limit Registers 2 */
/*! @{ */

#define ADC_LOLIM2_LLMT_MASK  (0x7FF8U)
#define ADC_LOLIM2_LLMT_SHIFT (3U)
/*! LLMT - Low Limit Bits
 */
#define ADC_LOLIM2_LLMT(x) (((uint16_t)(((uint16_t)(x)) << ADC_LOLIM2_LLMT_SHIFT)) & ADC_LOLIM2_LLMT_MASK)
/*! @} */

/* The count of ADC_LOLIM2 */
#define ADC_LOLIM2_COUNT (4U)

/*! @name HILIM2 - ADC High Limit Registers 2 */
/*! @{ */

#define ADC_HILIM2_HLMT_MASK  (0x7FF8U)
#define ADC_HILIM2_HLMT_SHIFT (3U)
/*! HLMT - High Limit Bits
 */
#define ADC_HILIM2_HLMT(x) (((uint16_t)(((uint16_t)(x)) << ADC_HILIM2_HLMT_SHIFT)) & ADC_HILIM2_HLMT_MASK)
/*! @} */

/* The count of ADC_HILIM2 */
#define ADC_HILIM2_COUNT (4U)

/*! @name OFFST2 - ADC Offset Registers 2 */
/*! @{ */

#define ADC_OFFST2_OFFSET_MASK  (0x7FF8U)
#define ADC_OFFST2_OFFSET_SHIFT (3U)
/*! OFFSET - ADC Offset Bits
 */
#define ADC_OFFST2_OFFSET(x) (((uint16_t)(((uint16_t)(x)) << ADC_OFFST2_OFFSET_SHIFT)) & ADC_OFFST2_OFFSET_MASK)
/*! @} */

/* The count of ADC_OFFST2 */
#define ADC_OFFST2_COUNT (4U)

/*! @name GC3 - Gain Control 3 Register */
/*! @{ */

#define ADC_GC3_GAIN16_MASK  (0x3U)
#define ADC_GC3_GAIN16_SHIFT (0U)
/*! GAIN16 - Gain Control Bit 16
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC3_GAIN16(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC3_GAIN16_SHIFT)) & ADC_GC3_GAIN16_MASK)

#define ADC_GC3_GAIN17_MASK  (0xCU)
#define ADC_GC3_GAIN17_SHIFT (2U)
/*! GAIN17 - Gain Control Bit 17
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC3_GAIN17(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC3_GAIN17_SHIFT)) & ADC_GC3_GAIN17_MASK)

#define ADC_GC3_GAIN18_MASK  (0x30U)
#define ADC_GC3_GAIN18_SHIFT (4U)
/*! GAIN18 - Gain Control Bit 18
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC3_GAIN18(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC3_GAIN18_SHIFT)) & ADC_GC3_GAIN18_MASK)

#define ADC_GC3_GAIN19_MASK  (0xC0U)
#define ADC_GC3_GAIN19_SHIFT (6U)
/*! GAIN19 - Gain Control Bit 19
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC3_GAIN19(x) (((uint16_t)(((uint16_t)(x)) << ADC_GC3_GAIN19_SHIFT)) & ADC_GC3_GAIN19_MASK)
/*! @} */

/*! @name SCTRL2 - ADC Scan Control Register 2 */
/*! @{ */

#define ADC_SCTRL2_SC_MASK  (0xFU)
#define ADC_SCTRL2_SC_SHIFT (0U)
/*! SC - Scan Control Bits
 *  0b0000..Perform sample immediately after the completion of the current sample.
 *  0b0001..Delay sample until a new sync input occurs.
 */
#define ADC_SCTRL2_SC(x) (((uint16_t)(((uint16_t)(x)) << ADC_SCTRL2_SC_SHIFT)) & ADC_SCTRL2_SC_MASK)
/*! @} */

/*! @name SCHLTEN2 - ADC Scan Interrupt Enable Register 2 */
/*! @{ */

#define ADC_SCHLTEN2_SCHLTEN_MASK  (0xFU)
#define ADC_SCHLTEN2_SCHLTEN_SHIFT (0U)
/*! SCHLTEN - SCHLTEN
 *  0b0000..Scan interrupt is not enabled for this sample.
 *  0b0001..Scan interrupt is enabled for this sample.
 */
#define ADC_SCHLTEN2_SCHLTEN(x) \
    (((uint16_t)(((uint16_t)(x)) << ADC_SCHLTEN2_SCHLTEN_SHIFT)) & ADC_SCHLTEN2_SCHLTEN_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group ADC_Register_Masks */

/* ADC - Peripheral instance base addresses */
/** Peripheral ADC base address */
#define ADC_BASE (0xE500u)
/** Peripheral ADC base pointer */
#define ADC ((ADC_Type *)ADC_BASE)
/** Array initializer of ADC peripheral base addresses */
#define ADC_BASE_ADDRS \
    {                  \
        ADC_BASE       \
    }
/** Array initializer of ADC peripheral base pointers */
#define ADC_BASE_PTRS \
    {                 \
        ADC           \
    }
/* Backward compatibility */
#define ADC_CAL_SEL_VREFLO_A_MASK  ADC_CAL_SEL_VREFL_A_MASK
#define ADC_CAL_SEL_VREFLO_A_SHIFT ADC_CAL_SEL_VREFL_A_SHIFT
#define ADC_CAL_SEL_VREFLO_A(x)    ADC_CAL_SEL_VREFL_A(x)
#define ADC_CAL_SEL_VREFLO_B_MASK  ADC_CAL_SEL_VREFL_B_MASK
#define ADC_CAL_SEL_VREFLO_B_SHIFT ADC_CAL_SEL_VREFL_B_SHIFT
#define ADC_CAL_SEL_VREFLO_B(x)    ADC_CAL_SEL_VREFL_B(x)

/*!
 * @}
 */ /* end of group ADC_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- CAN Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Peripheral_Access_Layer CAN Peripheral Access Layer
 * @{
 */

/** CAN - Register Layout Typedef */
typedef struct
{
    __IO uint32_t MCR;   /**< Module Configuration register, offset: 0x0 */
    __IO uint32_t CTRL1; /**< Control 1 register, offset: 0x2 */
    __IO uint32_t TIMER; /**< Free Running Timer, offset: 0x4 */
    uint16_t RESERVED_0[2];
    __IO uint32_t RXMGMASK; /**< Rx Mailboxes Global Mask register, offset: 0x8 */
    __IO uint32_t RX14MASK; /**< Rx 14 Mask register, offset: 0xA */
    __IO uint32_t RX15MASK; /**< Rx 15 Mask register, offset: 0xC */
    __IO uint32_t ECR;      /**< Error Counter, offset: 0xE */
    __IO uint32_t ESR1;     /**< Error and Status 1 register, offset: 0x10 */
    uint16_t RESERVED_1[2];
    __IO uint32_t IMASK1; /**< Interrupt Masks 1 register, offset: 0x14 */
    uint16_t RESERVED_2[2];
    __IO uint32_t IFLAG1; /**< Interrupt Flags 1 register, offset: 0x18 */
    __IO uint32_t CTRL2;  /**< Control 2 register, offset: 0x1A */
    __I uint32_t ESR2;    /**< Error and Status 2 register, offset: 0x1C */
    uint16_t RESERVED_3[4];
    __I uint32_t CRCR;      /**< CRC register, offset: 0x22 */
    __IO uint32_t RXFGMASK; /**< Rx FIFO Global Mask register, offset: 0x24 */
    __I uint32_t RXFIR;     /**< Rx FIFO Information register, offset: 0x26 */
    __IO uint32_t CBT;      /**< CAN Bit Timing register, offset: 0x28 */
    uint16_t RESERVED_4[22];
    union
    { /* offset: 0x40 */
        struct
        {                          /* offset: 0x40, array step: 0x8 */
            __IO uint32_t CS;      /**< Message Buffer 0 CS Register..Message Buffer 31 CS Register, array offset: 0x40,
                                      array step: 0x8 */
            __IO uint32_t ID;      /**< Message Buffer 0 ID Register..Message Buffer 31 ID Register, array offset: 0x42,
                                      array step: 0x8 */
            __IO uint32_t WORD[2]; /**< Message Buffer 0 WORD_8B Register..Message Buffer 31 WORD_8B Register, array
                                      offset: 0x44, array step: index*0x8, index2*0x2 */
        } MB_8B[32];
        struct
        {                          /* offset: 0x40, array step: 0xC */
            __IO uint32_t CS;      /**< Message Buffer 0 CS Register..Message Buffer 20 CS Register, array offset: 0x40,
                                      array step: 0xC */
            __IO uint32_t ID;      /**< Message Buffer 0 ID Register..Message Buffer 20 ID Register, array offset: 0x42,
                                      array step: 0xC */
            __IO uint32_t WORD[4]; /**< Message Buffer 0 WORD_16B Register..Message Buffer 20 WORD_16B Register, array
                                      offset: 0x44, array step: index*0xC, index2*0x2 */
        } MB_16B[21];
        struct
        {                          /* offset: 0x40, array step: 0x14 */
            __IO uint32_t CS;      /**< Message Buffer 0 CS Register..Message Buffer 11 CS Register, array offset: 0x40,
                                      array step: 0x14 */
            __IO uint32_t ID;      /**< Message Buffer 0 ID Register..Message Buffer 11 ID Register, array offset: 0x42,
                                      array step: 0x14 */
            __IO uint32_t WORD[8]; /**< Message Buffer 0 WORD_32B Register..Message Buffer 11 WORD_32B Register, array
                                      offset: 0x44, array step: index*0x14, index2*0x2 */
        } MB_32B[12];
        struct
        {                     /* offset: 0x40, array step: 0x24 */
            __IO uint32_t CS; /**< Message Buffer 0 CS Register..Message Buffer 6 CS Register, array offset: 0x40, array
                                 step: 0x24 */
            __IO uint32_t ID; /**< Message Buffer 0 ID Register..Message Buffer 6 ID Register, array offset: 0x42, array
                                 step: 0x24 */
            __IO uint32_t WORD[16]; /**< Message Buffer 0 WORD_64B Register..Message Buffer 6 WORD_64B Register, array
                                       offset: 0x44, array step: index*0x24, index2*0x2 */
        } MB_64B[7];
        struct
        {                        /* offset: 0x40, array step: 0x8 */
            __IO uint32_t CS;    /**< Message Buffer 0 CS Register..Message Buffer 31 CS Register, array offset: 0x40,
                                    array step: 0x8 */
            __IO uint32_t ID;    /**< Message Buffer 0 ID Register..Message Buffer 31 ID Register, array offset: 0x42,
                                    array step: 0x8 */
            __IO uint32_t WORD0; /**< Message Buffer 0 WORD0 Register..Message Buffer 31 WORD0 Register, array offset:
                                    0x44, array step: 0x8 */
            __IO uint32_t WORD1; /**< Message Buffer 0 WORD1 Register..Message Buffer 31 WORD1 Register, array offset:
                                    0x46, array step: 0x8 */
        } MB[32];
    };
    uint16_t RESERVED_5[768];
    __IO uint32_t RXIMR[32]; /**< Rx Individual Mask registers, array offset: 0x440, array step: 0x2 */
    uint16_t RESERVED_6[384];
    __IO uint32_t FDCTRL; /**< CAN FD Control register, offset: 0x600 */
    __IO uint32_t FDCBT;  /**< CAN FD Bit Timing register, offset: 0x602 */
    __I uint32_t FDCRC;   /**< CAN FD CRC register, offset: 0x604 */
} CAN_Type;

/* ----------------------------------------------------------------------------
   -- CAN Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Masks CAN Register Masks
 * @{
 */

/*! @name MCR - Module Configuration register */
/*! @{ */

#define CAN_MCR_MAXMB_MASK  (0x7FUL)
#define CAN_MCR_MAXMB_SHIFT (0UL)
/*! MAXMB - Number Of The Last Message Buffer
 */
#define CAN_MCR_MAXMB(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_MAXMB_SHIFT)) & CAN_MCR_MAXMB_MASK)

#define CAN_MCR_IDAM_MASK  (0x300UL)
#define CAN_MCR_IDAM_SHIFT (8UL)
/*! IDAM - ID Acceptance Mode
 *  0b00..Format A: One full ID (standard and extended) per ID filter table element.
 *  0b01..Format B: Two full standard IDs or two partial 14-bit (standard and extended) IDs per ID filter table element.
 *  0b10..Format C: Four partial 8-bit standard IDs per ID filter table element.
 *  0b11..Format D: All frames rejected.
 */
#define CAN_MCR_IDAM(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_IDAM_SHIFT)) & CAN_MCR_IDAM_MASK)

#define CAN_MCR_FDEN_MASK  (0x800UL)
#define CAN_MCR_FDEN_SHIFT (11UL)
/*! FDEN - CAN FD operation enable
 *  0b1..CAN FD is enabled. FlexCAN is able to receive and transmit messages in both CAN FD and CAN 2.0 formats.
 *  0b0..CAN FD is disabled. FlexCAN is able to receive and transmit messages in CAN 2.0 format.
 */
#define CAN_MCR_FDEN(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_FDEN_SHIFT)) & CAN_MCR_FDEN_MASK)

#define CAN_MCR_AEN_MASK  (0x1000UL)
#define CAN_MCR_AEN_SHIFT (12UL)
/*! AEN - Abort Enable
 *  0b0..Abort disabled.
 *  0b1..Abort enabled.
 */
#define CAN_MCR_AEN(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_AEN_SHIFT)) & CAN_MCR_AEN_MASK)

#define CAN_MCR_LPRIOEN_MASK  (0x2000UL)
#define CAN_MCR_LPRIOEN_SHIFT (13UL)
/*! LPRIOEN - Local Priority Enable
 *  0b0..Local Priority disabled.
 *  0b1..Local Priority enabled.
 */
#define CAN_MCR_LPRIOEN(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_LPRIOEN_SHIFT)) & CAN_MCR_LPRIOEN_MASK)

#define CAN_MCR_DMA_MASK  (0x8000UL)
#define CAN_MCR_DMA_SHIFT (15UL)
/*! DMA - DMA Enable
 *  0b0..DMA feature for RX FIFO disabled.
 *  0b1..DMA feature for RX FIFO enabled.
 */
#define CAN_MCR_DMA(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_DMA_SHIFT)) & CAN_MCR_DMA_MASK)

#define CAN_MCR_IRMQ_MASK  (0x10000UL)
#define CAN_MCR_IRMQ_SHIFT (16UL)
/*! IRMQ - Individual Rx Masking And Queue Enable
 *  0b0..Individual Rx masking and queue feature are disabled. For backward compatibility with legacy
 *       applications, the reading of C/S word locks the MB even if it is EMPTY.
 *  0b1..Individual Rx masking and queue feature are enabled.
 */
#define CAN_MCR_IRMQ(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_IRMQ_SHIFT)) & CAN_MCR_IRMQ_MASK)

#define CAN_MCR_SRXDIS_MASK  (0x20000UL)
#define CAN_MCR_SRXDIS_SHIFT (17UL)
/*! SRXDIS - Self Reception Disable
 *  0b0..Self-reception enabled.
 *  0b1..Self-reception disabled.
 */
#define CAN_MCR_SRXDIS(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SRXDIS_SHIFT)) & CAN_MCR_SRXDIS_MASK)

#define CAN_MCR_DOZE_MASK  (0x40000UL)
#define CAN_MCR_DOZE_SHIFT (18UL)
/*! DOZE - Doze Mode Enable
 *  0b0..FlexCAN is not enabled to enter low-power mode when Doze mode is requested.
 *  0b1..FlexCAN is enabled to enter low-power mode when Doze mode is requested.
 */
#define CAN_MCR_DOZE(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_DOZE_SHIFT)) & CAN_MCR_DOZE_MASK)

#define CAN_MCR_WAKSRC_MASK  (0x80000UL)
#define CAN_MCR_WAKSRC_SHIFT (19UL)
/*! WAKSRC - Wake Up Source
 *  0b0..FlexCAN uses the unfiltered Rx input to detect recessive to dominant edges on the CAN bus.
 *  0b1..FlexCAN uses the filtered Rx input to detect recessive to dominant edges on the CAN bus.
 */
#define CAN_MCR_WAKSRC(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_WAKSRC_SHIFT)) & CAN_MCR_WAKSRC_MASK)

#define CAN_MCR_LPMACK_MASK  (0x100000UL)
#define CAN_MCR_LPMACK_SHIFT (20UL)
/*! LPMACK - Low-Power Mode Acknowledge
 *  0b0..FlexCAN is not in a low-power mode.
 *  0b1..FlexCAN is in a low-power mode.
 */
#define CAN_MCR_LPMACK(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_LPMACK_SHIFT)) & CAN_MCR_LPMACK_MASK)

#define CAN_MCR_WRNEN_MASK  (0x200000UL)
#define CAN_MCR_WRNEN_SHIFT (21UL)
/*! WRNEN - Warning Interrupt Enable
 *  0b0..TWRNINT and RWRNINT bits are zero, independent of the values in the error counters.
 *  0b1..TWRNINT and RWRNINT bits are set when the respective error counter transitions from less than 96 to greater
 * than or equal to 96.
 */
#define CAN_MCR_WRNEN(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_WRNEN_SHIFT)) & CAN_MCR_WRNEN_MASK)

#define CAN_MCR_SLFWAK_MASK  (0x400000UL)
#define CAN_MCR_SLFWAK_SHIFT (22UL)
/*! SLFWAK - Self Wake Up
 *  0b0..FlexCAN Self Wake Up feature is disabled.
 *  0b1..FlexCAN Self Wake Up feature is enabled.
 */
#define CAN_MCR_SLFWAK(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SLFWAK_SHIFT)) & CAN_MCR_SLFWAK_MASK)

#define CAN_MCR_SUPV_MASK  (0x800000UL)
#define CAN_MCR_SUPV_SHIFT (23UL)
/*! SUPV - Supervisor Mode
 *  0b0..FlexCAN is in User mode. Affected registers allow both Supervisor and Unrestricted accesses.
 *  0b1..FlexCAN is in Supervisor mode. Affected registers allow only Supervisor access. Unrestricted access
 *       behaves as though the access was done to an unimplemented register location.
 */
#define CAN_MCR_SUPV(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SUPV_SHIFT)) & CAN_MCR_SUPV_MASK)

#define CAN_MCR_FRZACK_MASK  (0x1000000UL)
#define CAN_MCR_FRZACK_SHIFT (24UL)
/*! FRZACK - Freeze Mode Acknowledge
 *  0b0..FlexCAN not in Freeze mode, prescaler running.
 *  0b1..FlexCAN in Freeze mode, prescaler stopped.
 */
#define CAN_MCR_FRZACK(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_FRZACK_SHIFT)) & CAN_MCR_FRZACK_MASK)

#define CAN_MCR_SOFTRST_MASK  (0x2000000UL)
#define CAN_MCR_SOFTRST_SHIFT (25UL)
/*! SOFTRST - Soft Reset
 *  0b0..No reset request.
 *  0b1..Resets the registers affected by soft reset.
 */
#define CAN_MCR_SOFTRST(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SOFTRST_SHIFT)) & CAN_MCR_SOFTRST_MASK)

#define CAN_MCR_WAKMSK_MASK  (0x4000000UL)
#define CAN_MCR_WAKMSK_SHIFT (26UL)
/*! WAKMSK - Wake Up Interrupt Mask
 *  0b0..Wake Up interrupt is disabled.
 *  0b1..Wake Up interrupt is enabled.
 */
#define CAN_MCR_WAKMSK(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_WAKMSK_SHIFT)) & CAN_MCR_WAKMSK_MASK)

#define CAN_MCR_NOTRDY_MASK  (0x8000000UL)
#define CAN_MCR_NOTRDY_SHIFT (27UL)
/*! NOTRDY - FlexCAN Not Ready
 *  0b0..FlexCAN module is either in Normal mode, Listen-Only mode, or Loop-Back mode.
 *  0b1..FlexCAN module is either in Disable mode, Doze mode, Stop mode, or Freeze mode.
 */
#define CAN_MCR_NOTRDY(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_NOTRDY_SHIFT)) & CAN_MCR_NOTRDY_MASK)

#define CAN_MCR_HALT_MASK  (0x10000000UL)
#define CAN_MCR_HALT_SHIFT (28UL)
/*! HALT - Halt FlexCAN
 *  0b0..No Freeze mode request.
 *  0b1..Enters Freeze mode if the FRZ bit is asserted.
 */
#define CAN_MCR_HALT(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_HALT_SHIFT)) & CAN_MCR_HALT_MASK)

#define CAN_MCR_RFEN_MASK  (0x20000000UL)
#define CAN_MCR_RFEN_SHIFT (29UL)
/*! RFEN - Rx FIFO Enable
 *  0b0..Rx FIFO not enabled.
 *  0b1..Rx FIFO enabled.
 */
#define CAN_MCR_RFEN(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_RFEN_SHIFT)) & CAN_MCR_RFEN_MASK)

#define CAN_MCR_FRZ_MASK  (0x40000000UL)
#define CAN_MCR_FRZ_SHIFT (30UL)
/*! FRZ - Freeze Enable
 *  0b0..Not enabled to enter Freeze mode.
 *  0b1..Enabled to enter Freeze mode.
 */
#define CAN_MCR_FRZ(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_FRZ_SHIFT)) & CAN_MCR_FRZ_MASK)

#define CAN_MCR_MDIS_MASK  (0x80000000UL)
#define CAN_MCR_MDIS_SHIFT (31UL)
/*! MDIS - Module Disable
 *  0b0..Enable the FlexCAN module.
 *  0b1..Disable the FlexCAN module.
 */
#define CAN_MCR_MDIS(x) (((uint32_t)(((uint32_t)(x)) << CAN_MCR_MDIS_SHIFT)) & CAN_MCR_MDIS_MASK)
/*! @} */

/*! @name CTRL1 - Control 1 register */
/*! @{ */

#define CAN_CTRL1_PROPSEG_MASK  (0x7UL)
#define CAN_CTRL1_PROPSEG_SHIFT (0UL)
/*! PROPSEG - Propagation Segment
 */
#define CAN_CTRL1_PROPSEG(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PROPSEG_SHIFT)) & CAN_CTRL1_PROPSEG_MASK)

#define CAN_CTRL1_LOM_MASK  (0x8UL)
#define CAN_CTRL1_LOM_SHIFT (3UL)
/*! LOM - Listen-Only Mode
 *  0b0..Listen-Only mode is deactivated.
 *  0b1..FlexCAN module operates in Listen-Only mode.
 */
#define CAN_CTRL1_LOM(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_LOM_SHIFT)) & CAN_CTRL1_LOM_MASK)

#define CAN_CTRL1_LBUF_MASK  (0x10UL)
#define CAN_CTRL1_LBUF_SHIFT (4UL)
/*! LBUF - Lowest Buffer Transmitted First
 *  0b0..Buffer with highest priority is transmitted first.
 *  0b1..Lowest number buffer is transmitted first.
 */
#define CAN_CTRL1_LBUF(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_LBUF_SHIFT)) & CAN_CTRL1_LBUF_MASK)

#define CAN_CTRL1_TSYN_MASK  (0x20UL)
#define CAN_CTRL1_TSYN_SHIFT (5UL)
/*! TSYN - Timer Sync
 *  0b0..Timer sync feature disabled
 *  0b1..Timer sync feature enabled
 */
#define CAN_CTRL1_TSYN(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_TSYN_SHIFT)) & CAN_CTRL1_TSYN_MASK)

#define CAN_CTRL1_BOFFREC_MASK  (0x40UL)
#define CAN_CTRL1_BOFFREC_SHIFT (6UL)
/*! BOFFREC - Bus Off Recovery
 *  0b0..Automatic recovering from Bus Off state enabled.
 *  0b1..Automatic recovering from Bus Off state disabled.
 */
#define CAN_CTRL1_BOFFREC(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_BOFFREC_SHIFT)) & CAN_CTRL1_BOFFREC_MASK)

#define CAN_CTRL1_SMP_MASK  (0x80UL)
#define CAN_CTRL1_SMP_SHIFT (7UL)
/*! SMP - CAN Bit Sampling
 *  0b0..Just one sample is used to determine the bit value.
 *  0b1..Three samples are used to determine the value of the received bit: the regular one (sample point) and two
 *       preceding samples; a majority rule is used.
 */
#define CAN_CTRL1_SMP(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_SMP_SHIFT)) & CAN_CTRL1_SMP_MASK)

#define CAN_CTRL1_RWRNMSK_MASK  (0x400UL)
#define CAN_CTRL1_RWRNMSK_SHIFT (10UL)
/*! RWRNMSK - Rx Warning Interrupt Mask
 *  0b0..Rx Warning interrupt disabled.
 *  0b1..Rx Warning interrupt enabled.
 */
#define CAN_CTRL1_RWRNMSK(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_RWRNMSK_SHIFT)) & CAN_CTRL1_RWRNMSK_MASK)

#define CAN_CTRL1_TWRNMSK_MASK  (0x800UL)
#define CAN_CTRL1_TWRNMSK_SHIFT (11UL)
/*! TWRNMSK - Tx Warning Interrupt Mask
 *  0b0..Tx Warning interrupt disabled.
 *  0b1..Tx Warning interrupt enabled.
 */
#define CAN_CTRL1_TWRNMSK(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_TWRNMSK_SHIFT)) & CAN_CTRL1_TWRNMSK_MASK)

#define CAN_CTRL1_LPB_MASK  (0x1000UL)
#define CAN_CTRL1_LPB_SHIFT (12UL)
/*! LPB - Loop Back Mode
 *  0b0..Loop Back disabled.
 *  0b1..Loop Back enabled.
 */
#define CAN_CTRL1_LPB(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_LPB_SHIFT)) & CAN_CTRL1_LPB_MASK)

#define CAN_CTRL1_CLKSRC_MASK  (0x2000UL)
#define CAN_CTRL1_CLKSRC_SHIFT (13UL)
/*! CLKSRC - CAN Engine Clock Source
 *  0b0..The CAN engine clock source is the oscillator clock. Under this condition, the oscillator clock frequency must
 * be lower than the bus clock. 0b1..The CAN engine clock source is the peripheral clock.
 */
#define CAN_CTRL1_CLKSRC(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_CLKSRC_SHIFT)) & CAN_CTRL1_CLKSRC_MASK)

#define CAN_CTRL1_ERRMSK_MASK  (0x4000UL)
#define CAN_CTRL1_ERRMSK_SHIFT (14UL)
/*! ERRMSK - Error Interrupt Mask
 *  0b0..Error interrupt disabled.
 *  0b1..Error interrupt enabled.
 */
#define CAN_CTRL1_ERRMSK(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_ERRMSK_SHIFT)) & CAN_CTRL1_ERRMSK_MASK)

#define CAN_CTRL1_BOFFMSK_MASK  (0x8000UL)
#define CAN_CTRL1_BOFFMSK_SHIFT (15UL)
/*! BOFFMSK - Bus Off Interrupt Mask
 *  0b0..Bus Off interrupt disabled.
 *  0b1..Bus Off interrupt enabled.
 */
#define CAN_CTRL1_BOFFMSK(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_BOFFMSK_SHIFT)) & CAN_CTRL1_BOFFMSK_MASK)

#define CAN_CTRL1_PSEG2_MASK  (0x70000UL)
#define CAN_CTRL1_PSEG2_SHIFT (16UL)
/*! PSEG2 - Phase Segment 2
 */
#define CAN_CTRL1_PSEG2(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PSEG2_SHIFT)) & CAN_CTRL1_PSEG2_MASK)

#define CAN_CTRL1_PSEG1_MASK  (0x380000UL)
#define CAN_CTRL1_PSEG1_SHIFT (19UL)
/*! PSEG1 - Phase Segment 1
 */
#define CAN_CTRL1_PSEG1(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PSEG1_SHIFT)) & CAN_CTRL1_PSEG1_MASK)

#define CAN_CTRL1_RJW_MASK  (0xC00000UL)
#define CAN_CTRL1_RJW_SHIFT (22UL)
/*! RJW - Resync Jump Width
 */
#define CAN_CTRL1_RJW(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_RJW_SHIFT)) & CAN_CTRL1_RJW_MASK)

#define CAN_CTRL1_PRESDIV_MASK  (0xFF000000UL)
#define CAN_CTRL1_PRESDIV_SHIFT (24UL)
/*! PRESDIV - Prescaler Division Factor
 */
#define CAN_CTRL1_PRESDIV(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PRESDIV_SHIFT)) & CAN_CTRL1_PRESDIV_MASK)
/*! @} */

/*! @name TIMER - Free Running Timer */
/*! @{ */

#define CAN_TIMER_TIMER_MASK  (0xFFFFUL)
#define CAN_TIMER_TIMER_SHIFT (0UL)
/*! TIMER - Timer Value
 */
#define CAN_TIMER_TIMER(x) (((uint32_t)(((uint32_t)(x)) << CAN_TIMER_TIMER_SHIFT)) & CAN_TIMER_TIMER_MASK)
/*! @} */

/*! @name RXMGMASK - Rx Mailboxes Global Mask register */
/*! @{ */

#define CAN_RXMGMASK_MG_MASK  (0xFFFFFFFFUL)
#define CAN_RXMGMASK_MG_SHIFT (0UL)
/*! MG - Rx Mailboxes Global Mask Bits
 */
#define CAN_RXMGMASK_MG(x) (((uint32_t)(((uint32_t)(x)) << CAN_RXMGMASK_MG_SHIFT)) & CAN_RXMGMASK_MG_MASK)
/*! @} */

/*! @name RX14MASK - Rx 14 Mask register */
/*! @{ */

#define CAN_RX14MASK_RX14M_MASK  (0xFFFFFFFFUL)
#define CAN_RX14MASK_RX14M_SHIFT (0UL)
/*! RX14M - Rx Buffer 14 Mask Bits
 */
#define CAN_RX14MASK_RX14M(x) (((uint32_t)(((uint32_t)(x)) << CAN_RX14MASK_RX14M_SHIFT)) & CAN_RX14MASK_RX14M_MASK)
/*! @} */

/*! @name RX15MASK - Rx 15 Mask register */
/*! @{ */

#define CAN_RX15MASK_RX15M_MASK  (0xFFFFFFFFUL)
#define CAN_RX15MASK_RX15M_SHIFT (0UL)
/*! RX15M - Rx Buffer 15 Mask Bits
 */
#define CAN_RX15MASK_RX15M(x) (((uint32_t)(((uint32_t)(x)) << CAN_RX15MASK_RX15M_SHIFT)) & CAN_RX15MASK_RX15M_MASK)
/*! @} */

/*! @name ECR - Error Counter */
/*! @{ */

#define CAN_ECR_TXERRCNT_MASK  (0xFFUL)
#define CAN_ECR_TXERRCNT_SHIFT (0UL)
/*! TXERRCNT - Transmit Error Counter
 */
#define CAN_ECR_TXERRCNT(x) (((uint32_t)(((uint32_t)(x)) << CAN_ECR_TXERRCNT_SHIFT)) & CAN_ECR_TXERRCNT_MASK)

#define CAN_ECR_RXERRCNT_MASK  (0xFF00UL)
#define CAN_ECR_RXERRCNT_SHIFT (8UL)
/*! RXERRCNT - Receive Error Counter
 */
#define CAN_ECR_RXERRCNT(x) (((uint32_t)(((uint32_t)(x)) << CAN_ECR_RXERRCNT_SHIFT)) & CAN_ECR_RXERRCNT_MASK)

#define CAN_ECR_TXERRCNT_FAST_MASK  (0xFF0000UL)
#define CAN_ECR_TXERRCNT_FAST_SHIFT (16UL)
/*! TXERRCNT_FAST - Transmit Error Counter for fast bits
 */
#define CAN_ECR_TXERRCNT_FAST(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_ECR_TXERRCNT_FAST_SHIFT)) & CAN_ECR_TXERRCNT_FAST_MASK)

#define CAN_ECR_RXERRCNT_FAST_MASK  (0xFF000000UL)
#define CAN_ECR_RXERRCNT_FAST_SHIFT (24UL)
/*! RXERRCNT_FAST - Receive Error Counter for fast bits
 */
#define CAN_ECR_RXERRCNT_FAST(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_ECR_RXERRCNT_FAST_SHIFT)) & CAN_ECR_RXERRCNT_FAST_MASK)
/*! @} */

/*! @name ESR1 - Error and Status 1 register */
/*! @{ */

#define CAN_ESR1_WAKINT_MASK  (0x1UL)
#define CAN_ESR1_WAKINT_SHIFT (0UL)
/*! WAKINT - Wake-Up Interrupt
 *  0b0..No such occurrence.
 *  0b1..Indicates a recessive to dominant transition was received on the CAN bus.
 */
#define CAN_ESR1_WAKINT(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_WAKINT_SHIFT)) & CAN_ESR1_WAKINT_MASK)

#define CAN_ESR1_ERRINT_MASK  (0x2UL)
#define CAN_ESR1_ERRINT_SHIFT (1UL)
/*! ERRINT - Error Interrupt
 *  0b0..No such occurrence.
 *  0b1..Indicates setting of any error bit in the Error and Status register.
 */
#define CAN_ESR1_ERRINT(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ERRINT_SHIFT)) & CAN_ESR1_ERRINT_MASK)

#define CAN_ESR1_BOFFINT_MASK  (0x4UL)
#define CAN_ESR1_BOFFINT_SHIFT (2UL)
/*! BOFFINT - Bus Off Interrupt
 *  0b0..No such occurrence.
 *  0b1..FlexCAN module entered Bus Off state.
 */
#define CAN_ESR1_BOFFINT(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BOFFINT_SHIFT)) & CAN_ESR1_BOFFINT_MASK)

#define CAN_ESR1_RX_MASK  (0x8UL)
#define CAN_ESR1_RX_SHIFT (3UL)
/*! RX - FlexCAN In Reception
 *  0b0..FlexCAN is not receiving a message.
 *  0b1..FlexCAN is receiving a message.
 */
#define CAN_ESR1_RX(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_RX_SHIFT)) & CAN_ESR1_RX_MASK)

#define CAN_ESR1_FLTCONF_MASK  (0x30UL)
#define CAN_ESR1_FLTCONF_SHIFT (4UL)
/*! FLTCONF - Fault Confinement State
 *  0b00..Error Active
 *  0b01..Error Passive
 *  0b1x..Bus Off
 */
#define CAN_ESR1_FLTCONF(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_FLTCONF_SHIFT)) & CAN_ESR1_FLTCONF_MASK)

#define CAN_ESR1_TX_MASK  (0x40UL)
#define CAN_ESR1_TX_SHIFT (6UL)
/*! TX - FlexCAN In Transmission
 *  0b0..FlexCAN is not transmitting a message.
 *  0b1..FlexCAN is transmitting a message.
 */
#define CAN_ESR1_TX(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_TX_SHIFT)) & CAN_ESR1_TX_MASK)

#define CAN_ESR1_IDLE_MASK  (0x80UL)
#define CAN_ESR1_IDLE_SHIFT (7UL)
/*! IDLE - IDLE
 *  0b0..No such occurrence.
 *  0b1..CAN bus is now IDLE.
 */
#define CAN_ESR1_IDLE(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_IDLE_SHIFT)) & CAN_ESR1_IDLE_MASK)

#define CAN_ESR1_RXWRN_MASK  (0x100UL)
#define CAN_ESR1_RXWRN_SHIFT (8UL)
/*! RXWRN - Rx Error Warning
 *  0b0..No such occurrence.
 *  0b1..RXERRCNT is greater than or equal to 96.
 */
#define CAN_ESR1_RXWRN(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_RXWRN_SHIFT)) & CAN_ESR1_RXWRN_MASK)

#define CAN_ESR1_TXWRN_MASK  (0x200UL)
#define CAN_ESR1_TXWRN_SHIFT (9UL)
/*! TXWRN - TX Error Warning
 *  0b0..No such occurrence.
 *  0b1..TXERRCNT is greater than or equal to 96.
 */
#define CAN_ESR1_TXWRN(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_TXWRN_SHIFT)) & CAN_ESR1_TXWRN_MASK)

#define CAN_ESR1_STFERR_MASK  (0x400UL)
#define CAN_ESR1_STFERR_SHIFT (10UL)
/*! STFERR - Stuffing Error
 *  0b0..No such occurrence.
 *  0b1..A stuffing error occurred since last read of this register.
 */
#define CAN_ESR1_STFERR(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_STFERR_SHIFT)) & CAN_ESR1_STFERR_MASK)

#define CAN_ESR1_FRMERR_MASK  (0x800UL)
#define CAN_ESR1_FRMERR_SHIFT (11UL)
/*! FRMERR - Form Error
 *  0b0..No such occurrence.
 *  0b1..A Form Error occurred since last read of this register.
 */
#define CAN_ESR1_FRMERR(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_FRMERR_SHIFT)) & CAN_ESR1_FRMERR_MASK)

#define CAN_ESR1_CRCERR_MASK  (0x1000UL)
#define CAN_ESR1_CRCERR_SHIFT (12UL)
/*! CRCERR - Cyclic Redundancy Check Error
 *  0b0..No such occurrence.
 *  0b1..A CRC error occurred since last read of this register.
 */
#define CAN_ESR1_CRCERR(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_CRCERR_SHIFT)) & CAN_ESR1_CRCERR_MASK)

#define CAN_ESR1_ACKERR_MASK  (0x2000UL)
#define CAN_ESR1_ACKERR_SHIFT (13UL)
/*! ACKERR - Acknowledge Error
 *  0b0..No such occurrence.
 *  0b1..An ACK error occurred since last read of this register.
 */
#define CAN_ESR1_ACKERR(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ACKERR_SHIFT)) & CAN_ESR1_ACKERR_MASK)

#define CAN_ESR1_BIT0ERR_MASK  (0x4000UL)
#define CAN_ESR1_BIT0ERR_SHIFT (14UL)
/*! BIT0ERR - Bit0 Error
 *  0b0..No such occurrence.
 *  0b1..At least one bit sent as dominant is received as recessive.
 */
#define CAN_ESR1_BIT0ERR(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT0ERR_SHIFT)) & CAN_ESR1_BIT0ERR_MASK)

#define CAN_ESR1_BIT1ERR_MASK  (0x8000UL)
#define CAN_ESR1_BIT1ERR_SHIFT (15UL)
/*! BIT1ERR - Bit1 Error
 *  0b0..No such occurrence.
 *  0b1..At least one bit sent as recessive is received as dominant.
 */
#define CAN_ESR1_BIT1ERR(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT1ERR_SHIFT)) & CAN_ESR1_BIT1ERR_MASK)

#define CAN_ESR1_RWRNINT_MASK  (0x10000UL)
#define CAN_ESR1_RWRNINT_SHIFT (16UL)
/*! RWRNINT - Rx Warning Interrupt Flag
 *  0b0..No such occurrence.
 *  0b1..The Rx error counter transitioned from less than 96 to greater than or equal to 96.
 */
#define CAN_ESR1_RWRNINT(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_RWRNINT_SHIFT)) & CAN_ESR1_RWRNINT_MASK)

#define CAN_ESR1_TWRNINT_MASK  (0x20000UL)
#define CAN_ESR1_TWRNINT_SHIFT (17UL)
/*! TWRNINT - Tx Warning Interrupt Flag
 *  0b0..No such occurrence.
 *  0b1..The Tx error counter transitioned from less than 96 to greater than or equal to 96.
 */
#define CAN_ESR1_TWRNINT(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_TWRNINT_SHIFT)) & CAN_ESR1_TWRNINT_MASK)

#define CAN_ESR1_SYNCH_MASK  (0x40000UL)
#define CAN_ESR1_SYNCH_SHIFT (18UL)
/*! SYNCH - CAN Synchronization Status
 *  0b0..FlexCAN is not synchronized to the CAN bus.
 *  0b1..FlexCAN is synchronized to the CAN bus.
 */
#define CAN_ESR1_SYNCH(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_SYNCH_SHIFT)) & CAN_ESR1_SYNCH_MASK)

#define CAN_ESR1_BOFFDONEINT_MASK  (0x80000UL)
#define CAN_ESR1_BOFFDONEINT_SHIFT (19UL)
/*! BOFFDONEINT - Bus Off Done Interrupt
 *  0b0..No such occurrence.
 *  0b1..FlexCAN module has completed Bus Off process.
 */
#define CAN_ESR1_BOFFDONEINT(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BOFFDONEINT_SHIFT)) & CAN_ESR1_BOFFDONEINT_MASK)

#define CAN_ESR1_ERRINT_FAST_MASK  (0x100000UL)
#define CAN_ESR1_ERRINT_FAST_SHIFT (20UL)
/*! ERRINT_FAST - Error interrupt for errors detected in Data Phase of CAN FD frames with BRS bit set
 *  0b0..No such occurrence.
 *  0b1..Indicates setting of any error bit detected in the data phase of CAN FD frames with the BRS bit set.
 */
#define CAN_ESR1_ERRINT_FAST(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ERRINT_FAST_SHIFT)) & CAN_ESR1_ERRINT_FAST_MASK)

#define CAN_ESR1_ERROVR_MASK  (0x200000UL)
#define CAN_ESR1_ERROVR_SHIFT (21UL)
/*! ERROVR - Error Overrun
 *  0b0..Overrun has not occurred.
 *  0b1..Overrun has occurred.
 */
#define CAN_ESR1_ERROVR(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ERROVR_SHIFT)) & CAN_ESR1_ERROVR_MASK)

#define CAN_ESR1_STFERR_FAST_MASK  (0x4000000UL)
#define CAN_ESR1_STFERR_FAST_SHIFT (26UL)
/*! STFERR_FAST - Stuffing Error in the Data Phase of CAN FD frames with the BRS bit set
 *  0b0..No such occurrence.
 *  0b1..A stuffing error occurred since last read of this register.
 */
#define CAN_ESR1_STFERR_FAST(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_STFERR_FAST_SHIFT)) & CAN_ESR1_STFERR_FAST_MASK)

#define CAN_ESR1_FRMERR_FAST_MASK  (0x8000000UL)
#define CAN_ESR1_FRMERR_FAST_SHIFT (27UL)
/*! FRMERR_FAST - Form Error in the Data Phase of CAN FD frames with the BRS bit set
 *  0b0..No such occurrence.
 *  0b1..A form error occurred since last read of this register.
 */
#define CAN_ESR1_FRMERR_FAST(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_FRMERR_FAST_SHIFT)) & CAN_ESR1_FRMERR_FAST_MASK)

#define CAN_ESR1_CRCERR_FAST_MASK  (0x10000000UL)
#define CAN_ESR1_CRCERR_FAST_SHIFT (28UL)
/*! CRCERR_FAST - Cyclic Redundancy Check Error in the CRC field of CAN FD frames with the BRS bit set
 *  0b0..No such occurrence.
 *  0b1..A CRC error occurred since last read of this register.
 */
#define CAN_ESR1_CRCERR_FAST(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_CRCERR_FAST_SHIFT)) & CAN_ESR1_CRCERR_FAST_MASK)

#define CAN_ESR1_BIT0ERR_FAST_MASK  (0x40000000UL)
#define CAN_ESR1_BIT0ERR_FAST_SHIFT (30UL)
/*! BIT0ERR_FAST - Bit0 Error in the Data Phase of CAN FD frames with the BRS bit set
 *  0b0..No such occurrence.
 *  0b1..At least one bit sent as dominant is received as recessive.
 */
#define CAN_ESR1_BIT0ERR_FAST(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT0ERR_FAST_SHIFT)) & CAN_ESR1_BIT0ERR_FAST_MASK)

#define CAN_ESR1_BIT1ERR_FAST_MASK  (0x80000000UL)
#define CAN_ESR1_BIT1ERR_FAST_SHIFT (31UL)
/*! BIT1ERR_FAST - Bit1 Error in the Data Phase of CAN FD frames with the BRS bit set
 *  0b0..No such occurrence.
 *  0b1..At least one bit sent as recessive is received as dominant.
 */
#define CAN_ESR1_BIT1ERR_FAST(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT1ERR_FAST_SHIFT)) & CAN_ESR1_BIT1ERR_FAST_MASK)
/*! @} */

/*! @name IMASK1 - Interrupt Masks 1 register */
/*! @{ */

#define CAN_IMASK1_BUF31TO0M_MASK  (0xFFFFFFFFUL)
#define CAN_IMASK1_BUF31TO0M_SHIFT (0UL)
/*! BUF31TO0M - Buffer MB i Mask
 */
#define CAN_IMASK1_BUF31TO0M(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_IMASK1_BUF31TO0M_SHIFT)) & CAN_IMASK1_BUF31TO0M_MASK)
/*! @} */

/*! @name IFLAG1 - Interrupt Flags 1 register */
/*! @{ */

#define CAN_IFLAG1_BUF0I_MASK  (0x1UL)
#define CAN_IFLAG1_BUF0I_SHIFT (0UL)
/*! BUF0I - Buffer MB0 Interrupt Or Clear FIFO bit
 *  0b0..The corresponding buffer has no occurrence of successfully completed transmission or reception when
 * MCR[RFEN]=0. 0b1..The corresponding buffer has successfully completed transmission or reception when MCR[RFEN]=0.
 */
#define CAN_IFLAG1_BUF0I(x) (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF0I_SHIFT)) & CAN_IFLAG1_BUF0I_MASK)

#define CAN_IFLAG1_BUF4TO1I_MASK  (0x1EUL)
#define CAN_IFLAG1_BUF4TO1I_SHIFT (1UL)
/*! BUF4TO1I - Buffer MB i Interrupt Or Reserved
 */
#define CAN_IFLAG1_BUF4TO1I(x) (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF4TO1I_SHIFT)) & CAN_IFLAG1_BUF4TO1I_MASK)

#define CAN_IFLAG1_BUF5I_MASK  (0x20UL)
#define CAN_IFLAG1_BUF5I_SHIFT (5UL)
/*! BUF5I - Buffer MB5 Interrupt Or Frames available in Rx FIFO
 *  0b0..No occurrence of MB5 completing transmission/reception when MCR[RFEN]=0, or of frame(s) available in the FIFO,
 * when MCR[RFEN]=1 0b1..MB5 completed transmission/reception when MCR[RFEN]=0, or frame(s) available in the Rx FIFO
 * when MCR[RFEN]=1. It generates a DMA request in case of MCR[RFEN] and MCR[DMA] are enabled.
 */
#define CAN_IFLAG1_BUF5I(x) (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF5I_SHIFT)) & CAN_IFLAG1_BUF5I_MASK)

#define CAN_IFLAG1_BUF6I_MASK  (0x40UL)
#define CAN_IFLAG1_BUF6I_SHIFT (6UL)
/*! BUF6I - Buffer MB6 Interrupt Or Rx FIFO Warning
 *  0b0..No occurrence of MB6 completing transmission/reception when MCR[RFEN]=0, or of Rx FIFO almost full when
 * MCR[RFEN]=1 0b1..MB6 completed transmission/reception when MCR[RFEN]=0, or Rx FIFO almost full when MCR[RFEN]=1
 */
#define CAN_IFLAG1_BUF6I(x) (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF6I_SHIFT)) & CAN_IFLAG1_BUF6I_MASK)

#define CAN_IFLAG1_BUF7I_MASK  (0x80UL)
#define CAN_IFLAG1_BUF7I_SHIFT (7UL)
/*! BUF7I - Buffer MB7 Interrupt Or Rx FIFO Overflow
 *  0b0..No occurrence of MB7 completing transmission/reception when MCR[RFEN]=0, or of Rx FIFO overflow when
 * MCR[RFEN]=1 0b1..MB7 completed transmission/reception when MCR[RFEN]=0, or Rx FIFO overflow when MCR[RFEN]=1
 */
#define CAN_IFLAG1_BUF7I(x) (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF7I_SHIFT)) & CAN_IFLAG1_BUF7I_MASK)

#define CAN_IFLAG1_BUF31TO8I_MASK  (0xFFFFFF00UL)
#define CAN_IFLAG1_BUF31TO8I_SHIFT (8UL)
/*! BUF31TO8I - Buffer MBi Interrupt
 */
#define CAN_IFLAG1_BUF31TO8I(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF31TO8I_SHIFT)) & CAN_IFLAG1_BUF31TO8I_MASK)
/*! @} */

/*! @name CTRL2 - Control 2 register */
/*! @{ */

#define CAN_CTRL2_EDFLTDIS_MASK  (0x800UL)
#define CAN_CTRL2_EDFLTDIS_SHIFT (11UL)
/*! EDFLTDIS - Edge Filter Disable
 *  0b0..Edge filter is enabled
 *  0b1..Edge filter is disabled
 */
#define CAN_CTRL2_EDFLTDIS(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_EDFLTDIS_SHIFT)) & CAN_CTRL2_EDFLTDIS_MASK)

#define CAN_CTRL2_ISOCANFDEN_MASK  (0x1000UL)
#define CAN_CTRL2_ISOCANFDEN_SHIFT (12UL)
/*! ISOCANFDEN - ISO CAN FD Enable
 *  0b0..FlexCAN operates using the non-ISO CAN FD protocol.
 *  0b1..FlexCAN operates using the ISO CAN FD protocol (ISO 11898-1).
 */
#define CAN_CTRL2_ISOCANFDEN(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_ISOCANFDEN_SHIFT)) & CAN_CTRL2_ISOCANFDEN_MASK)

#define CAN_CTRL2_PREXCEN_MASK  (0x4000UL)
#define CAN_CTRL2_PREXCEN_SHIFT (14UL)
/*! PREXCEN - Protocol Exception Enable
 *  0b0..Protocol exception is disabled.
 *  0b1..Protocol exception is enabled.
 */
#define CAN_CTRL2_PREXCEN(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_PREXCEN_SHIFT)) & CAN_CTRL2_PREXCEN_MASK)

#define CAN_CTRL2_TIMER_SRC_MASK  (0x8000UL)
#define CAN_CTRL2_TIMER_SRC_SHIFT (15UL)
/*! TIMER_SRC - Timer Source
 *  0b0..The free running timer is clocked by the CAN bit clock, which defines the baud rate on the CAN bus.
 *  0b1..The free running timer is clocked by an external time tick. The period can be either adjusted to be equal
 *       to the baud rate on the CAN bus, or a different value as required. See the device-specific section for
 *       details about the external time tick.
 */
#define CAN_CTRL2_TIMER_SRC(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_TIMER_SRC_SHIFT)) & CAN_CTRL2_TIMER_SRC_MASK)

#define CAN_CTRL2_EACEN_MASK  (0x10000UL)
#define CAN_CTRL2_EACEN_SHIFT (16UL)
/*! EACEN - Entire Frame Arbitration Field Comparison Enable For Rx Mailboxes
 *  0b0..Rx mailbox filter's IDE bit is always compared and RTR is never compared despite mask bits.
 *  0b1..Enables the comparison of both Rx mailbox filter's IDE and RTR bit with their corresponding bits within
 *       the incoming frame. Mask bits do apply.
 */
#define CAN_CTRL2_EACEN(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_EACEN_SHIFT)) & CAN_CTRL2_EACEN_MASK)

#define CAN_CTRL2_RRS_MASK  (0x20000UL)
#define CAN_CTRL2_RRS_SHIFT (17UL)
/*! RRS - Remote Request Storing
 *  0b0..Remote response frame is generated.
 *  0b1..Remote request frame is stored.
 */
#define CAN_CTRL2_RRS(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_RRS_SHIFT)) & CAN_CTRL2_RRS_MASK)

#define CAN_CTRL2_MRP_MASK  (0x40000UL)
#define CAN_CTRL2_MRP_SHIFT (18UL)
/*! MRP - Mailboxes Reception Priority
 *  0b0..Matching starts from Rx FIFO or Enhanced Rx FIFO and continues on mailboxes.
 *  0b1..Matching starts from mailboxes and continues on Rx FIFO.
 */
#define CAN_CTRL2_MRP(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_MRP_SHIFT)) & CAN_CTRL2_MRP_MASK)

#define CAN_CTRL2_TASD_MASK  (0xF80000UL)
#define CAN_CTRL2_TASD_SHIFT (19UL)
/*! TASD - Tx Arbitration Start Delay
 */
#define CAN_CTRL2_TASD(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_TASD_SHIFT)) & CAN_CTRL2_TASD_MASK)

#define CAN_CTRL2_RFFN_MASK  (0xF000000UL)
#define CAN_CTRL2_RFFN_SHIFT (24UL)
/*! RFFN - Number Of Rx FIFO Filters
 */
#define CAN_CTRL2_RFFN(x) (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_RFFN_SHIFT)) & CAN_CTRL2_RFFN_MASK)

#define CAN_CTRL2_BOFFDONEMSK_MASK  (0x40000000UL)
#define CAN_CTRL2_BOFFDONEMSK_SHIFT (30UL)
/*! BOFFDONEMSK - Bus Off Done Interrupt Mask
 *  0b0..Bus off done interrupt disabled.
 *  0b1..Bus off done interrupt enabled.
 */
#define CAN_CTRL2_BOFFDONEMSK(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_BOFFDONEMSK_SHIFT)) & CAN_CTRL2_BOFFDONEMSK_MASK)

#define CAN_CTRL2_ERRMSK_FAST_MASK  (0x80000000UL)
#define CAN_CTRL2_ERRMSK_FAST_SHIFT (31UL)
/*! ERRMSK_FAST - Error Interrupt Mask for errors detected in the data phase of fast CAN FD frames
 *  0b0..ERRINT_FAST error interrupt disabled.
 *  0b1..ERRINT_FAST error interrupt enabled.
 */
#define CAN_CTRL2_ERRMSK_FAST(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_ERRMSK_FAST_SHIFT)) & CAN_CTRL2_ERRMSK_FAST_MASK)
/*! @} */

/*! @name ESR2 - Error and Status 2 register */
/*! @{ */

#define CAN_ESR2_IMB_MASK  (0x2000UL)
#define CAN_ESR2_IMB_SHIFT (13UL)
/*! IMB - Inactive Mailbox
 *  0b0..If ESR2[VPS] is asserted, the ESR2[LPTM] is not an inactive mailbox.
 *  0b1..If ESR2[VPS] is asserted, there is at least one inactive mailbox. LPTM content is the number of the first one.
 */
#define CAN_ESR2_IMB(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR2_IMB_SHIFT)) & CAN_ESR2_IMB_MASK)

#define CAN_ESR2_VPS_MASK  (0x4000UL)
#define CAN_ESR2_VPS_SHIFT (14UL)
/*! VPS - Valid Priority Status
 *  0b0..Contents of IMB and LPTM are invalid.
 *  0b1..Contents of IMB and LPTM are valid.
 */
#define CAN_ESR2_VPS(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR2_VPS_SHIFT)) & CAN_ESR2_VPS_MASK)

#define CAN_ESR2_LPTM_MASK  (0x7F0000UL)
#define CAN_ESR2_LPTM_SHIFT (16UL)
/*! LPTM - Lowest Priority Tx Mailbox
 */
#define CAN_ESR2_LPTM(x) (((uint32_t)(((uint32_t)(x)) << CAN_ESR2_LPTM_SHIFT)) & CAN_ESR2_LPTM_MASK)
/*! @} */

/*! @name CRCR - CRC register */
/*! @{ */

#define CAN_CRCR_TXCRC_MASK  (0x7FFFUL)
#define CAN_CRCR_TXCRC_SHIFT (0UL)
/*! TXCRC - Transmitted CRC value
 */
#define CAN_CRCR_TXCRC(x) (((uint32_t)(((uint32_t)(x)) << CAN_CRCR_TXCRC_SHIFT)) & CAN_CRCR_TXCRC_MASK)

#define CAN_CRCR_MBCRC_MASK  (0x7F0000UL)
#define CAN_CRCR_MBCRC_SHIFT (16UL)
/*! MBCRC - CRC Mailbox
 */
#define CAN_CRCR_MBCRC(x) (((uint32_t)(((uint32_t)(x)) << CAN_CRCR_MBCRC_SHIFT)) & CAN_CRCR_MBCRC_MASK)
/*! @} */

/*! @name RXFGMASK - Rx FIFO Global Mask register */
/*! @{ */

#define CAN_RXFGMASK_FGM_MASK  (0xFFFFFFFFUL)
#define CAN_RXFGMASK_FGM_SHIFT (0UL)
/*! FGM - Rx FIFO Global Mask Bits
 */
#define CAN_RXFGMASK_FGM(x) (((uint32_t)(((uint32_t)(x)) << CAN_RXFGMASK_FGM_SHIFT)) & CAN_RXFGMASK_FGM_MASK)
/*! @} */

/*! @name RXFIR - Rx FIFO Information register */
/*! @{ */

#define CAN_RXFIR_IDHIT_MASK  (0x1FFUL)
#define CAN_RXFIR_IDHIT_SHIFT (0UL)
/*! IDHIT - Identifier Acceptance Filter Hit Indicator
 */
#define CAN_RXFIR_IDHIT(x) (((uint32_t)(((uint32_t)(x)) << CAN_RXFIR_IDHIT_SHIFT)) & CAN_RXFIR_IDHIT_MASK)
/*! @} */

/*! @name CBT - CAN Bit Timing register */
/*! @{ */

#define CAN_CBT_EPSEG2_MASK  (0x1FUL)
#define CAN_CBT_EPSEG2_SHIFT (0UL)
/*! EPSEG2 - Extended Phase Segment 2
 */
#define CAN_CBT_EPSEG2(x) (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPSEG2_SHIFT)) & CAN_CBT_EPSEG2_MASK)

#define CAN_CBT_EPSEG1_MASK  (0x3E0UL)
#define CAN_CBT_EPSEG1_SHIFT (5UL)
/*! EPSEG1 - Extended Phase Segment 1
 */
#define CAN_CBT_EPSEG1(x) (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPSEG1_SHIFT)) & CAN_CBT_EPSEG1_MASK)

#define CAN_CBT_EPROPSEG_MASK  (0xFC00UL)
#define CAN_CBT_EPROPSEG_SHIFT (10UL)
/*! EPROPSEG - Extended Propagation Segment
 */
#define CAN_CBT_EPROPSEG(x) (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPROPSEG_SHIFT)) & CAN_CBT_EPROPSEG_MASK)

#define CAN_CBT_ERJW_MASK  (0x1F0000UL)
#define CAN_CBT_ERJW_SHIFT (16UL)
/*! ERJW - Extended Resync Jump Width
 */
#define CAN_CBT_ERJW(x) (((uint32_t)(((uint32_t)(x)) << CAN_CBT_ERJW_SHIFT)) & CAN_CBT_ERJW_MASK)

#define CAN_CBT_EPRESDIV_MASK  (0x7FE00000UL)
#define CAN_CBT_EPRESDIV_SHIFT (21UL)
/*! EPRESDIV - Extended Prescaler Division Factor
 */
#define CAN_CBT_EPRESDIV(x) (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPRESDIV_SHIFT)) & CAN_CBT_EPRESDIV_MASK)

#define CAN_CBT_BTF_MASK  (0x80000000UL)
#define CAN_CBT_BTF_SHIFT (31UL)
/*! BTF - Bit Timing Format Enable
 *  0b0..Extended bit time definitions disabled.
 *  0b1..Extended bit time definitions enabled.
 */
#define CAN_CBT_BTF(x) (((uint32_t)(((uint32_t)(x)) << CAN_CBT_BTF_SHIFT)) & CAN_CBT_BTF_MASK)
/*! @} */

/* The count of CAN_CS */
#define CAN_CS_COUNT_MB8B (32U)

/* The count of CAN_ID */
#define CAN_ID_COUNT_MB8B (32U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB8B (32U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB8B2 (2U)

/* The count of CAN_CS */
#define CAN_CS_COUNT_MB16B (21U)

/* The count of CAN_ID */
#define CAN_ID_COUNT_MB16B (21U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB16B (21U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB16B2 (4U)

/* The count of CAN_CS */
#define CAN_CS_COUNT_MB32B (12U)

/* The count of CAN_ID */
#define CAN_ID_COUNT_MB32B (12U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB32B (12U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB32B2 (8U)

/*! @name CS - Message Buffer 0 CS Register..Message Buffer 6 CS Register */
/*! @{ */

#define CAN_CS_TIME_STAMP_MASK  (0xFFFFUL)
#define CAN_CS_TIME_STAMP_SHIFT (0UL)
/*! TIME_STAMP - Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running
 *    Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field
 *    appears on the CAN bus.
 */
#define CAN_CS_TIME_STAMP(x) (((uint32_t)(((uint32_t)(x)) << CAN_CS_TIME_STAMP_SHIFT)) & CAN_CS_TIME_STAMP_MASK)

#define CAN_CS_DLC_MASK  (0xF0000UL)
#define CAN_CS_DLC_SHIFT (16UL)
/*! DLC - Length of the data to be stored/transmitted.
 */
#define CAN_CS_DLC(x) (((uint32_t)(((uint32_t)(x)) << CAN_CS_DLC_SHIFT)) & CAN_CS_DLC_MASK)

#define CAN_CS_RTR_MASK  (0x100000UL)
#define CAN_CS_RTR_SHIFT (20UL)
/*! RTR - Remote Transmission Request. One/zero for remote/data frame.
 */
#define CAN_CS_RTR(x) (((uint32_t)(((uint32_t)(x)) << CAN_CS_RTR_SHIFT)) & CAN_CS_RTR_MASK)

#define CAN_CS_IDE_MASK  (0x200000UL)
#define CAN_CS_IDE_SHIFT (21UL)
/*! IDE - ID Extended. One/zero for extended/standard format frame.
 */
#define CAN_CS_IDE(x) (((uint32_t)(((uint32_t)(x)) << CAN_CS_IDE_SHIFT)) & CAN_CS_IDE_MASK)

#define CAN_CS_SRR_MASK  (0x400000UL)
#define CAN_CS_SRR_SHIFT (22UL)
/*! SRR - Substitute Remote Request. Contains a fixed recessive bit.
 */
#define CAN_CS_SRR(x) (((uint32_t)(((uint32_t)(x)) << CAN_CS_SRR_SHIFT)) & CAN_CS_SRR_MASK)

#define CAN_CS_CODE_MASK  (0xF000000UL)
#define CAN_CS_CODE_SHIFT (24UL)
/*! CODE - Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by
 *    the FlexCAN module itself, as part of the message buffer matching and arbitration process.
 */
#define CAN_CS_CODE(x) (((uint32_t)(((uint32_t)(x)) << CAN_CS_CODE_SHIFT)) & CAN_CS_CODE_MASK)

#define CAN_CS_ESI_MASK  (0x20000000UL)
#define CAN_CS_ESI_SHIFT (29UL)
/*! ESI - Error State Indicator. This bit indicates if the transmitting node is error active or error passive.
 */
#define CAN_CS_ESI(x) (((uint32_t)(((uint32_t)(x)) << CAN_CS_ESI_SHIFT)) & CAN_CS_ESI_MASK)

#define CAN_CS_BRS_MASK  (0x40000000UL)
#define CAN_CS_BRS_SHIFT (30UL)
/*! BRS - Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame.
 */
#define CAN_CS_BRS(x) (((uint32_t)(((uint32_t)(x)) << CAN_CS_BRS_SHIFT)) & CAN_CS_BRS_MASK)

#define CAN_CS_EDL_MASK  (0x80000000UL)
#define CAN_CS_EDL_SHIFT (31UL)
/*! EDL - Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames.
 *    The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010.
 */
#define CAN_CS_EDL(x) (((uint32_t)(((uint32_t)(x)) << CAN_CS_EDL_SHIFT)) & CAN_CS_EDL_MASK)
/*! @} */

/* The count of CAN_CS */
#define CAN_CS_COUNT_MB64B (7U)

/*! @name ID - Message Buffer 0 ID Register..Message Buffer 6 ID Register */
/*! @{ */

#define CAN_ID_EXT_MASK  (0x3FFFFUL)
#define CAN_ID_EXT_SHIFT (0UL)
/*! EXT - Contains extended (LOW word) identifier of message buffer.
 */
#define CAN_ID_EXT(x) (((uint32_t)(((uint32_t)(x)) << CAN_ID_EXT_SHIFT)) & CAN_ID_EXT_MASK)

#define CAN_ID_STD_MASK  (0x1FFC0000UL)
#define CAN_ID_STD_SHIFT (18UL)
/*! STD - Contains standard/extended (HIGH word) identifier of message buffer.
 */
#define CAN_ID_STD(x) (((uint32_t)(((uint32_t)(x)) << CAN_ID_STD_SHIFT)) & CAN_ID_STD_MASK)

#define CAN_ID_PRIO_MASK  (0xE0000000UL)
#define CAN_ID_PRIO_SHIFT (29UL)
/*! PRIO - Local priority. This 3-bit fieldis only used when LPRIO_EN bit is set in MCR and it only
 *    makes sense for Tx buffers. These bits are not transmitted. They are appended to the regular
 *    ID to define the transmission priority.
 */
#define CAN_ID_PRIO(x) (((uint32_t)(((uint32_t)(x)) << CAN_ID_PRIO_SHIFT)) & CAN_ID_PRIO_MASK)
/*! @} */

/* The count of CAN_ID */
#define CAN_ID_COUNT_MB64B (7U)

/*! @name WORD - Message Buffer 0 WORD_64B Register..Message Buffer 6 WORD_64B Register */
/*! @{ */

#define CAN_WORD_DATA_BYTE_3_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_3_SHIFT (0UL)
/*! DATA_BYTE_3 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_3(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_3_SHIFT)) & CAN_WORD_DATA_BYTE_3_MASK)

#define CAN_WORD_DATA_BYTE_7_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_7_SHIFT (0UL)
/*! DATA_BYTE_7 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_7(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_7_SHIFT)) & CAN_WORD_DATA_BYTE_7_MASK)

#define CAN_WORD_DATA_BYTE_11_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_11_SHIFT (0UL)
/*! DATA_BYTE_11 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_11(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_11_SHIFT)) & CAN_WORD_DATA_BYTE_11_MASK)

#define CAN_WORD_DATA_BYTE_15_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_15_SHIFT (0UL)
/*! DATA_BYTE_15 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_15(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_15_SHIFT)) & CAN_WORD_DATA_BYTE_15_MASK)

#define CAN_WORD_DATA_BYTE_19_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_19_SHIFT (0UL)
/*! DATA_BYTE_19 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_19(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_19_SHIFT)) & CAN_WORD_DATA_BYTE_19_MASK)

#define CAN_WORD_DATA_BYTE_23_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_23_SHIFT (0UL)
/*! DATA_BYTE_23 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_23(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_23_SHIFT)) & CAN_WORD_DATA_BYTE_23_MASK)

#define CAN_WORD_DATA_BYTE_27_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_27_SHIFT (0UL)
/*! DATA_BYTE_27 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_27(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_27_SHIFT)) & CAN_WORD_DATA_BYTE_27_MASK)

#define CAN_WORD_DATA_BYTE_31_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_31_SHIFT (0UL)
/*! DATA_BYTE_31 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_31(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_31_SHIFT)) & CAN_WORD_DATA_BYTE_31_MASK)

#define CAN_WORD_DATA_BYTE_35_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_35_SHIFT (0UL)
/*! DATA_BYTE_35 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_35(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_35_SHIFT)) & CAN_WORD_DATA_BYTE_35_MASK)

#define CAN_WORD_DATA_BYTE_39_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_39_SHIFT (0UL)
/*! DATA_BYTE_39 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_39(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_39_SHIFT)) & CAN_WORD_DATA_BYTE_39_MASK)

#define CAN_WORD_DATA_BYTE_43_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_43_SHIFT (0UL)
/*! DATA_BYTE_43 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_43(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_43_SHIFT)) & CAN_WORD_DATA_BYTE_43_MASK)

#define CAN_WORD_DATA_BYTE_47_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_47_SHIFT (0UL)
/*! DATA_BYTE_47 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_47(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_47_SHIFT)) & CAN_WORD_DATA_BYTE_47_MASK)

#define CAN_WORD_DATA_BYTE_51_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_51_SHIFT (0UL)
/*! DATA_BYTE_51 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_51(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_51_SHIFT)) & CAN_WORD_DATA_BYTE_51_MASK)

#define CAN_WORD_DATA_BYTE_55_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_55_SHIFT (0UL)
/*! DATA_BYTE_55 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_55(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_55_SHIFT)) & CAN_WORD_DATA_BYTE_55_MASK)

#define CAN_WORD_DATA_BYTE_59_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_59_SHIFT (0UL)
/*! DATA_BYTE_59 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_59(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_59_SHIFT)) & CAN_WORD_DATA_BYTE_59_MASK)

#define CAN_WORD_DATA_BYTE_63_MASK  (0xFFUL)
#define CAN_WORD_DATA_BYTE_63_SHIFT (0UL)
/*! DATA_BYTE_63 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_63(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_63_SHIFT)) & CAN_WORD_DATA_BYTE_63_MASK)

#define CAN_WORD_DATA_BYTE_2_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_2_SHIFT (8UL)
/*! DATA_BYTE_2 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_2(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_2_SHIFT)) & CAN_WORD_DATA_BYTE_2_MASK)

#define CAN_WORD_DATA_BYTE_6_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_6_SHIFT (8UL)
/*! DATA_BYTE_6 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_6(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_6_SHIFT)) & CAN_WORD_DATA_BYTE_6_MASK)

#define CAN_WORD_DATA_BYTE_10_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_10_SHIFT (8UL)
/*! DATA_BYTE_10 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_10(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_10_SHIFT)) & CAN_WORD_DATA_BYTE_10_MASK)

#define CAN_WORD_DATA_BYTE_14_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_14_SHIFT (8UL)
/*! DATA_BYTE_14 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_14(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_14_SHIFT)) & CAN_WORD_DATA_BYTE_14_MASK)

#define CAN_WORD_DATA_BYTE_18_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_18_SHIFT (8UL)
/*! DATA_BYTE_18 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_18(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_18_SHIFT)) & CAN_WORD_DATA_BYTE_18_MASK)

#define CAN_WORD_DATA_BYTE_22_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_22_SHIFT (8UL)
/*! DATA_BYTE_22 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_22(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_22_SHIFT)) & CAN_WORD_DATA_BYTE_22_MASK)

#define CAN_WORD_DATA_BYTE_26_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_26_SHIFT (8UL)
/*! DATA_BYTE_26 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_26(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_26_SHIFT)) & CAN_WORD_DATA_BYTE_26_MASK)

#define CAN_WORD_DATA_BYTE_30_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_30_SHIFT (8UL)
/*! DATA_BYTE_30 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_30(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_30_SHIFT)) & CAN_WORD_DATA_BYTE_30_MASK)

#define CAN_WORD_DATA_BYTE_34_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_34_SHIFT (8UL)
/*! DATA_BYTE_34 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_34(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_34_SHIFT)) & CAN_WORD_DATA_BYTE_34_MASK)

#define CAN_WORD_DATA_BYTE_38_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_38_SHIFT (8UL)
/*! DATA_BYTE_38 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_38(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_38_SHIFT)) & CAN_WORD_DATA_BYTE_38_MASK)

#define CAN_WORD_DATA_BYTE_42_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_42_SHIFT (8UL)
/*! DATA_BYTE_42 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_42(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_42_SHIFT)) & CAN_WORD_DATA_BYTE_42_MASK)

#define CAN_WORD_DATA_BYTE_46_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_46_SHIFT (8UL)
/*! DATA_BYTE_46 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_46(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_46_SHIFT)) & CAN_WORD_DATA_BYTE_46_MASK)

#define CAN_WORD_DATA_BYTE_50_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_50_SHIFT (8UL)
/*! DATA_BYTE_50 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_50(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_50_SHIFT)) & CAN_WORD_DATA_BYTE_50_MASK)

#define CAN_WORD_DATA_BYTE_54_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_54_SHIFT (8UL)
/*! DATA_BYTE_54 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_54(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_54_SHIFT)) & CAN_WORD_DATA_BYTE_54_MASK)

#define CAN_WORD_DATA_BYTE_58_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_58_SHIFT (8UL)
/*! DATA_BYTE_58 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_58(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_58_SHIFT)) & CAN_WORD_DATA_BYTE_58_MASK)

#define CAN_WORD_DATA_BYTE_62_MASK  (0xFF00UL)
#define CAN_WORD_DATA_BYTE_62_SHIFT (8UL)
/*! DATA_BYTE_62 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_62(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_62_SHIFT)) & CAN_WORD_DATA_BYTE_62_MASK)

#define CAN_WORD_DATA_BYTE_1_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_1_SHIFT (16UL)
/*! DATA_BYTE_1 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_1(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_1_SHIFT)) & CAN_WORD_DATA_BYTE_1_MASK)

#define CAN_WORD_DATA_BYTE_5_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_5_SHIFT (16UL)
/*! DATA_BYTE_5 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_5(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_5_SHIFT)) & CAN_WORD_DATA_BYTE_5_MASK)

#define CAN_WORD_DATA_BYTE_9_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_9_SHIFT (16UL)
/*! DATA_BYTE_9 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_9(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_9_SHIFT)) & CAN_WORD_DATA_BYTE_9_MASK)

#define CAN_WORD_DATA_BYTE_13_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_13_SHIFT (16UL)
/*! DATA_BYTE_13 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_13(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_13_SHIFT)) & CAN_WORD_DATA_BYTE_13_MASK)

#define CAN_WORD_DATA_BYTE_17_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_17_SHIFT (16UL)
/*! DATA_BYTE_17 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_17(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_17_SHIFT)) & CAN_WORD_DATA_BYTE_17_MASK)

#define CAN_WORD_DATA_BYTE_21_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_21_SHIFT (16UL)
/*! DATA_BYTE_21 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_21(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_21_SHIFT)) & CAN_WORD_DATA_BYTE_21_MASK)

#define CAN_WORD_DATA_BYTE_25_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_25_SHIFT (16UL)
/*! DATA_BYTE_25 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_25(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_25_SHIFT)) & CAN_WORD_DATA_BYTE_25_MASK)

#define CAN_WORD_DATA_BYTE_29_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_29_SHIFT (16UL)
/*! DATA_BYTE_29 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_29(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_29_SHIFT)) & CAN_WORD_DATA_BYTE_29_MASK)

#define CAN_WORD_DATA_BYTE_33_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_33_SHIFT (16UL)
/*! DATA_BYTE_33 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_33(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_33_SHIFT)) & CAN_WORD_DATA_BYTE_33_MASK)

#define CAN_WORD_DATA_BYTE_37_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_37_SHIFT (16UL)
/*! DATA_BYTE_37 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_37(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_37_SHIFT)) & CAN_WORD_DATA_BYTE_37_MASK)

#define CAN_WORD_DATA_BYTE_41_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_41_SHIFT (16UL)
/*! DATA_BYTE_41 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_41(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_41_SHIFT)) & CAN_WORD_DATA_BYTE_41_MASK)

#define CAN_WORD_DATA_BYTE_45_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_45_SHIFT (16UL)
/*! DATA_BYTE_45 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_45(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_45_SHIFT)) & CAN_WORD_DATA_BYTE_45_MASK)

#define CAN_WORD_DATA_BYTE_49_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_49_SHIFT (16UL)
/*! DATA_BYTE_49 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_49(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_49_SHIFT)) & CAN_WORD_DATA_BYTE_49_MASK)

#define CAN_WORD_DATA_BYTE_53_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_53_SHIFT (16UL)
/*! DATA_BYTE_53 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_53(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_53_SHIFT)) & CAN_WORD_DATA_BYTE_53_MASK)

#define CAN_WORD_DATA_BYTE_57_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_57_SHIFT (16UL)
/*! DATA_BYTE_57 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_57(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_57_SHIFT)) & CAN_WORD_DATA_BYTE_57_MASK)

#define CAN_WORD_DATA_BYTE_61_MASK  (0xFF0000UL)
#define CAN_WORD_DATA_BYTE_61_SHIFT (16UL)
/*! DATA_BYTE_61 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_61(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_61_SHIFT)) & CAN_WORD_DATA_BYTE_61_MASK)

#define CAN_WORD_DATA_BYTE_0_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_0_SHIFT (24UL)
/*! DATA_BYTE_0 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_0(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_0_SHIFT)) & CAN_WORD_DATA_BYTE_0_MASK)

#define CAN_WORD_DATA_BYTE_4_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_4_SHIFT (24UL)
/*! DATA_BYTE_4 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_4(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_4_SHIFT)) & CAN_WORD_DATA_BYTE_4_MASK)

#define CAN_WORD_DATA_BYTE_8_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_8_SHIFT (24UL)
/*! DATA_BYTE_8 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_8(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_8_SHIFT)) & CAN_WORD_DATA_BYTE_8_MASK)

#define CAN_WORD_DATA_BYTE_12_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_12_SHIFT (24UL)
/*! DATA_BYTE_12 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_12(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_12_SHIFT)) & CAN_WORD_DATA_BYTE_12_MASK)

#define CAN_WORD_DATA_BYTE_16_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_16_SHIFT (24UL)
/*! DATA_BYTE_16 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_16(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_16_SHIFT)) & CAN_WORD_DATA_BYTE_16_MASK)

#define CAN_WORD_DATA_BYTE_20_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_20_SHIFT (24UL)
/*! DATA_BYTE_20 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_20(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_20_SHIFT)) & CAN_WORD_DATA_BYTE_20_MASK)

#define CAN_WORD_DATA_BYTE_24_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_24_SHIFT (24UL)
/*! DATA_BYTE_24 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_24(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_24_SHIFT)) & CAN_WORD_DATA_BYTE_24_MASK)

#define CAN_WORD_DATA_BYTE_28_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_28_SHIFT (24UL)
/*! DATA_BYTE_28 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_28(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_28_SHIFT)) & CAN_WORD_DATA_BYTE_28_MASK)

#define CAN_WORD_DATA_BYTE_32_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_32_SHIFT (24UL)
/*! DATA_BYTE_32 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_32(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_32_SHIFT)) & CAN_WORD_DATA_BYTE_32_MASK)

#define CAN_WORD_DATA_BYTE_36_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_36_SHIFT (24UL)
/*! DATA_BYTE_36 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_36(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_36_SHIFT)) & CAN_WORD_DATA_BYTE_36_MASK)

#define CAN_WORD_DATA_BYTE_40_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_40_SHIFT (24UL)
/*! DATA_BYTE_40 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_40(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_40_SHIFT)) & CAN_WORD_DATA_BYTE_40_MASK)

#define CAN_WORD_DATA_BYTE_44_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_44_SHIFT (24UL)
/*! DATA_BYTE_44 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_44(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_44_SHIFT)) & CAN_WORD_DATA_BYTE_44_MASK)

#define CAN_WORD_DATA_BYTE_48_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_48_SHIFT (24UL)
/*! DATA_BYTE_48 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_48(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_48_SHIFT)) & CAN_WORD_DATA_BYTE_48_MASK)

#define CAN_WORD_DATA_BYTE_52_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_52_SHIFT (24UL)
/*! DATA_BYTE_52 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_52(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_52_SHIFT)) & CAN_WORD_DATA_BYTE_52_MASK)

#define CAN_WORD_DATA_BYTE_56_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_56_SHIFT (24UL)
/*! DATA_BYTE_56 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_56(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_56_SHIFT)) & CAN_WORD_DATA_BYTE_56_MASK)

#define CAN_WORD_DATA_BYTE_60_MASK  (0xFF000000UL)
#define CAN_WORD_DATA_BYTE_60_SHIFT (24UL)
/*! DATA_BYTE_60 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD_DATA_BYTE_60(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD_DATA_BYTE_60_SHIFT)) & CAN_WORD_DATA_BYTE_60_MASK)
/*! @} */

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB64B (7U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB64B2 (16U)

/* The count of CAN_CS */
#define CAN_CS_COUNT (32U)

/* The count of CAN_ID */
#define CAN_ID_COUNT (32U)

/*! @name WORD0 - Message Buffer 0 WORD0 Register..Message Buffer 31 WORD0 Register */
/*! @{ */

#define CAN_WORD0_DATA_BYTE_3_MASK  (0xFFUL)
#define CAN_WORD0_DATA_BYTE_3_SHIFT (0UL)
/*! DATA_BYTE_3 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD0_DATA_BYTE_3(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD0_DATA_BYTE_3_SHIFT)) & CAN_WORD0_DATA_BYTE_3_MASK)

#define CAN_WORD0_DATA_BYTE_2_MASK  (0xFF00UL)
#define CAN_WORD0_DATA_BYTE_2_SHIFT (8UL)
/*! DATA_BYTE_2 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD0_DATA_BYTE_2(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD0_DATA_BYTE_2_SHIFT)) & CAN_WORD0_DATA_BYTE_2_MASK)

#define CAN_WORD0_DATA_BYTE_1_MASK  (0xFF0000UL)
#define CAN_WORD0_DATA_BYTE_1_SHIFT (16UL)
/*! DATA_BYTE_1 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD0_DATA_BYTE_1(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD0_DATA_BYTE_1_SHIFT)) & CAN_WORD0_DATA_BYTE_1_MASK)

#define CAN_WORD0_DATA_BYTE_0_MASK  (0xFF000000UL)
#define CAN_WORD0_DATA_BYTE_0_SHIFT (24UL)
/*! DATA_BYTE_0 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD0_DATA_BYTE_0(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD0_DATA_BYTE_0_SHIFT)) & CAN_WORD0_DATA_BYTE_0_MASK)
/*! @} */

/* The count of CAN_WORD0 */
#define CAN_WORD0_COUNT (32U)

/*! @name WORD1 - Message Buffer 0 WORD1 Register..Message Buffer 31 WORD1 Register */
/*! @{ */

#define CAN_WORD1_DATA_BYTE_7_MASK  (0xFFUL)
#define CAN_WORD1_DATA_BYTE_7_SHIFT (0UL)
/*! DATA_BYTE_7 - Data byte 0 of Rx/Tx frame.
 */
#define CAN_WORD1_DATA_BYTE_7(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD1_DATA_BYTE_7_SHIFT)) & CAN_WORD1_DATA_BYTE_7_MASK)

#define CAN_WORD1_DATA_BYTE_6_MASK  (0xFF00UL)
#define CAN_WORD1_DATA_BYTE_6_SHIFT (8UL)
/*! DATA_BYTE_6 - Data byte 1 of Rx/Tx frame.
 */
#define CAN_WORD1_DATA_BYTE_6(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD1_DATA_BYTE_6_SHIFT)) & CAN_WORD1_DATA_BYTE_6_MASK)

#define CAN_WORD1_DATA_BYTE_5_MASK  (0xFF0000UL)
#define CAN_WORD1_DATA_BYTE_5_SHIFT (16UL)
/*! DATA_BYTE_5 - Data byte 2 of Rx/Tx frame.
 */
#define CAN_WORD1_DATA_BYTE_5(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD1_DATA_BYTE_5_SHIFT)) & CAN_WORD1_DATA_BYTE_5_MASK)

#define CAN_WORD1_DATA_BYTE_4_MASK  (0xFF000000UL)
#define CAN_WORD1_DATA_BYTE_4_SHIFT (24UL)
/*! DATA_BYTE_4 - Data byte 3 of Rx/Tx frame.
 */
#define CAN_WORD1_DATA_BYTE_4(x) \
    (((uint32_t)(((uint32_t)(x)) << CAN_WORD1_DATA_BYTE_4_SHIFT)) & CAN_WORD1_DATA_BYTE_4_MASK)
/*! @} */

/* The count of CAN_WORD1 */
#define CAN_WORD1_COUNT (32U)

/*! @name RXIMR - Rx Individual Mask registers */
/*! @{ */

#define CAN_RXIMR_MI_MASK  (0xFFFFFFFFUL)
#define CAN_RXIMR_MI_SHIFT (0UL)
/*! MI - Individual Mask Bits
 */
#define CAN_RXIMR_MI(x) (((uint32_t)(((uint32_t)(x)) << CAN_RXIMR_MI_SHIFT)) & CAN_RXIMR_MI_MASK)
/*! @} */

/* The count of CAN_RXIMR */
#define CAN_RXIMR_COUNT (32U)

/*! @name FDCTRL - CAN FD Control register */
/*! @{ */

#define CAN_FDCTRL_TDCVAL_MASK  (0x3FUL)
#define CAN_FDCTRL_TDCVAL_SHIFT (0UL)
/*! TDCVAL - Transceiver Delay Compensation Value
 */
#define CAN_FDCTRL_TDCVAL(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCVAL_SHIFT)) & CAN_FDCTRL_TDCVAL_MASK)

#define CAN_FDCTRL_TDCOFF_MASK  (0x1F00UL)
#define CAN_FDCTRL_TDCOFF_SHIFT (8UL)
/*! TDCOFF - Transceiver Delay Compensation Offset
 */
#define CAN_FDCTRL_TDCOFF(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCOFF_SHIFT)) & CAN_FDCTRL_TDCOFF_MASK)

#define CAN_FDCTRL_TDCFAIL_MASK  (0x4000UL)
#define CAN_FDCTRL_TDCFAIL_SHIFT (14UL)
/*! TDCFAIL - Transceiver Delay Compensation Fail
 *  0b0..Measured loop delay is in range.
 *  0b1..Measured loop delay is out of range.
 */
#define CAN_FDCTRL_TDCFAIL(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCFAIL_SHIFT)) & CAN_FDCTRL_TDCFAIL_MASK)

#define CAN_FDCTRL_TDCEN_MASK  (0x8000UL)
#define CAN_FDCTRL_TDCEN_SHIFT (15UL)
/*! TDCEN - Transceiver Delay Compensation Enable
 *  0b0..TDC is disabled
 *  0b1..TDC is enabled
 */
#define CAN_FDCTRL_TDCEN(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCEN_SHIFT)) & CAN_FDCTRL_TDCEN_MASK)

#define CAN_FDCTRL_MBDSR0_MASK  (0x30000UL)
#define CAN_FDCTRL_MBDSR0_SHIFT (16UL)
/*! MBDSR0 - Message Buffer Data Size for Region 0
 *  0b00..Selects 8 bytes per message buffer.
 *  0b01..Selects 16 bytes per message buffer.
 *  0b10..Selects 32 bytes per message buffer.
 *  0b11..Selects 64 bytes per message buffer.
 */
#define CAN_FDCTRL_MBDSR0(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_MBDSR0_SHIFT)) & CAN_FDCTRL_MBDSR0_MASK)

#define CAN_FDCTRL_FDRATE_MASK  (0x80000000UL)
#define CAN_FDCTRL_FDRATE_SHIFT (31UL)
/*! FDRATE - Bit Rate Switch Enable
 *  0b0..Transmit a frame in nominal rate. The BRS bit in the Tx MB has no effect.
 *  0b1..Transmit a frame with bit rate switching if the BRS bit in the Tx MB is recessive.
 */
#define CAN_FDCTRL_FDRATE(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_FDRATE_SHIFT)) & CAN_FDCTRL_FDRATE_MASK)
/*! @} */

/*! @name FDCBT - CAN FD Bit Timing register */
/*! @{ */

#define CAN_FDCBT_FPSEG2_MASK  (0x7UL)
#define CAN_FDCBT_FPSEG2_SHIFT (0UL)
/*! FPSEG2 - Fast Phase Segment 2
 */
#define CAN_FDCBT_FPSEG2(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPSEG2_SHIFT)) & CAN_FDCBT_FPSEG2_MASK)

#define CAN_FDCBT_FPSEG1_MASK  (0xE0UL)
#define CAN_FDCBT_FPSEG1_SHIFT (5UL)
/*! FPSEG1 - Fast Phase Segment 1
 */
#define CAN_FDCBT_FPSEG1(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPSEG1_SHIFT)) & CAN_FDCBT_FPSEG1_MASK)

#define CAN_FDCBT_FPROPSEG_MASK  (0x7C00UL)
#define CAN_FDCBT_FPROPSEG_SHIFT (10UL)
/*! FPROPSEG - Fast Propagation Segment
 */
#define CAN_FDCBT_FPROPSEG(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPROPSEG_SHIFT)) & CAN_FDCBT_FPROPSEG_MASK)

#define CAN_FDCBT_FRJW_MASK  (0x70000UL)
#define CAN_FDCBT_FRJW_SHIFT (16UL)
/*! FRJW - Fast Resync Jump Width
 */
#define CAN_FDCBT_FRJW(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FRJW_SHIFT)) & CAN_FDCBT_FRJW_MASK)

#define CAN_FDCBT_FPRESDIV_MASK  (0x3FF00000UL)
#define CAN_FDCBT_FPRESDIV_SHIFT (20UL)
/*! FPRESDIV - Fast Prescaler Division Factor
 */
#define CAN_FDCBT_FPRESDIV(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPRESDIV_SHIFT)) & CAN_FDCBT_FPRESDIV_MASK)
/*! @} */

/*! @name FDCRC - CAN FD CRC register */
/*! @{ */

#define CAN_FDCRC_FD_TXCRC_MASK  (0x1FFFFFUL)
#define CAN_FDCRC_FD_TXCRC_SHIFT (0UL)
/*! FD_TXCRC - Extended Transmitted CRC value
 */
#define CAN_FDCRC_FD_TXCRC(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCRC_FD_TXCRC_SHIFT)) & CAN_FDCRC_FD_TXCRC_MASK)

#define CAN_FDCRC_FD_MBCRC_MASK  (0x7F000000UL)
#define CAN_FDCRC_FD_MBCRC_SHIFT (24UL)
/*! FD_MBCRC - CRC Mailbox Number for FD_TXCRC
 */
#define CAN_FDCRC_FD_MBCRC(x) (((uint32_t)(((uint32_t)(x)) << CAN_FDCRC_FD_MBCRC_SHIFT)) & CAN_FDCRC_FD_MBCRC_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group CAN_Register_Masks */

/* CAN - Peripheral instance base addresses */
/** Peripheral CAN0 base address */
#define CAN0_BASE (0xF000u)
/** Peripheral CAN0 base pointer */
#define CAN0 ((CAN_Type *)CAN0_BASE)
/** Array initializer of CAN peripheral base addresses */
#define CAN_BASE_ADDRS \
    {                  \
        CAN0_BASE      \
    }
/** Array initializer of CAN peripheral base pointers */
#define CAN_BASE_PTRS \
    {                 \
        CAN0          \
    }
/** Interrupt vectors for the CAN peripheral type */
#define CAN_Rx_Warning_IRQS \
    {                       \
        CAN_RX_WARN_IRQn    \
    }
#define CAN_Tx_Warning_IRQS \
    {                       \
        CAN_TX_WARN_IRQn    \
    }
#define CAN_Wake_Up_IRQS \
    {                    \
        CAN_WAKEUP_IRQn  \
    }
#define CAN_Error_IRQS \
    {                  \
        CAN_ERROR_IRQn \
    }
#define CAN_Bus_Off_IRQS \
    {                    \
        CAN_BUS_OFF_IRQn \
    }
#define CAN_ORed_Message_buffer_IRQS \
    {                                \
        CAN_MB_OR_IRQn               \
    }

/*!
 * @}
 */ /* end of group CAN_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- CMP Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Peripheral_Access_Layer CMP Peripheral Access Layer
 * @{
 */

/** CMP - Register Layout Typedef */
typedef struct
{
    __IO uint16_t CR0;   /**< CMP Control Register 0, offset: 0x0 */
    __IO uint16_t CR1;   /**< CMP Control Register 1, offset: 0x1 */
    __IO uint16_t FPR;   /**< CMP Filter Period Register, offset: 0x2 */
    __IO uint16_t SCR;   /**< CMP Status and Control Register, offset: 0x3 */
    __IO uint16_t DACCR; /**< DAC Control Register, offset: 0x4 */
    __IO uint16_t MUXCR; /**< MUX Control Register, offset: 0x5 */
} CMP_Type;

/* ----------------------------------------------------------------------------
   -- CMP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Masks CMP Register Masks
 * @{
 */

/*! @name CR0 - CMP Control Register 0 */
/*! @{ */

#define CMP_CR0_HYSTCTR_MASK  (0x3U)
#define CMP_CR0_HYSTCTR_SHIFT (0U)
/*! HYSTCTR - Comparator hard block hysteresis control
 *  0b00..Level 0
 *  0b01..Level 1
 *  0b10..Level 2
 *  0b11..Level 3
 */
#define CMP_CR0_HYSTCTR(x) (((uint16_t)(((uint16_t)(x)) << CMP_CR0_HYSTCTR_SHIFT)) & CMP_CR0_HYSTCTR_MASK)

#define CMP_CR0_FILTER_CNT_MASK  (0x70U)
#define CMP_CR0_FILTER_CNT_SHIFT (4U)
/*! FILTER_CNT - Filter Sample Count
 *  0b000..Filter is disabled. If SE = 1, then COUT is a logic 0. This is not a legal state, and is not recommended. If
 * SE = 0, COUT = COUTA. 0b001..One sample must agree. The comparator output is simply sampled. 0b010..2 consecutive
 * samples must agree. 0b011..3 consecutive samples must agree. 0b100..4 consecutive samples must agree. 0b101..5
 * consecutive samples must agree. 0b110..6 consecutive samples must agree. 0b111..7 consecutive samples must agree.
 */
#define CMP_CR0_FILTER_CNT(x) (((uint16_t)(((uint16_t)(x)) << CMP_CR0_FILTER_CNT_SHIFT)) & CMP_CR0_FILTER_CNT_MASK)
/*! @} */

/*! @name CR1 - CMP Control Register 1 */
/*! @{ */

#define CMP_CR1_EN_MASK  (0x1U)
#define CMP_CR1_EN_SHIFT (0U)
/*! EN - Comparator Module Enable
 *  0b0..Analog Comparator is disabled.
 *  0b1..Analog Comparator is enabled.
 */
#define CMP_CR1_EN(x) (((uint16_t)(((uint16_t)(x)) << CMP_CR1_EN_SHIFT)) & CMP_CR1_EN_MASK)

#define CMP_CR1_OPE_MASK  (0x2U)
#define CMP_CR1_OPE_SHIFT (1U)
/*! OPE - Comparator Output Pin Enable
 *  0b0..CMPO is not available on the associated CMPO output pin. If the comparator does not own the pin, this field has
 * no effect. 0b1..CMPO is available on the associated CMPO output pin. The comparator output (CMPO) is driven out on
 * the associated CMPO output pin if the comparator owns the pin. If the comparator does not own the field, this bit has
 * no effect.
 */
#define CMP_CR1_OPE(x) (((uint16_t)(((uint16_t)(x)) << CMP_CR1_OPE_SHIFT)) & CMP_CR1_OPE_MASK)

#define CMP_CR1_COS_MASK  (0x4U)
#define CMP_CR1_COS_SHIFT (2U)
/*! COS - Comparator Output Select
 *  0b0..Set the filtered comparator output (CMPO) to equal COUT.
 *  0b1..Set the unfiltered comparator output (CMPO) to equal COUTA.
 */
#define CMP_CR1_COS(x) (((uint16_t)(((uint16_t)(x)) << CMP_CR1_COS_SHIFT)) & CMP_CR1_COS_MASK)

#define CMP_CR1_INV_MASK  (0x8U)
#define CMP_CR1_INV_SHIFT (3U)
/*! INV - Comparator INVERT
 *  0b0..Does not invert the comparator output.
 *  0b1..Inverts the comparator output.
 */
#define CMP_CR1_INV(x) (((uint16_t)(((uint16_t)(x)) << CMP_CR1_INV_SHIFT)) & CMP_CR1_INV_MASK)

#define CMP_CR1_PMODE_MASK  (0x10U)
#define CMP_CR1_PMODE_SHIFT (4U)
/*! PMODE - Power Mode Select
 *  0b0..Low-Speed (LS) Comparison mode selected. In this mode, CMP has slower output propagation delay and lower
 * current consumption. 0b1..High-Speed (HS) Comparison mode selected. In this mode, CMP has faster output propagation
 * delay and higher current consumption.
 */
#define CMP_CR1_PMODE(x) (((uint16_t)(((uint16_t)(x)) << CMP_CR1_PMODE_SHIFT)) & CMP_CR1_PMODE_MASK)

#define CMP_CR1_COWZ_MASK  (0x20U)
#define CMP_CR1_COWZ_SHIFT (5U)
/*! COWZ - COUTA out of window is zero enable.
 *  0b0..In windowing mode, when WINDOW signal changes from 1 to 0, COUTA holds the last latched value before WINDOW
 * signal falls to 0. 0b1..In windowing mode, when WINDOW signal changes from 1 to 0, COUTA is forced to 0.
 */
#define CMP_CR1_COWZ(x) (((uint16_t)(((uint16_t)(x)) << CMP_CR1_COWZ_SHIFT)) & CMP_CR1_COWZ_MASK)

#define CMP_CR1_WE_MASK  (0x40U)
#define CMP_CR1_WE_SHIFT (6U)
/*! WE - Windowing Enable
 *  0b0..Windowing mode is not selected.
 *  0b1..Windowing mode is selected.
 */
#define CMP_CR1_WE(x) (((uint16_t)(((uint16_t)(x)) << CMP_CR1_WE_SHIFT)) & CMP_CR1_WE_MASK)

#define CMP_CR1_SE_MASK  (0x80U)
#define CMP_CR1_SE_SHIFT (7U)
/*! SE - Sample Enable
 *  0b0..Sampling mode is not selected.
 *  0b1..Sampling mode is selected.
 */
#define CMP_CR1_SE(x) (((uint16_t)(((uint16_t)(x)) << CMP_CR1_SE_SHIFT)) & CMP_CR1_SE_MASK)
/*! @} */

/*! @name FPR - CMP Filter Period Register */
/*! @{ */

#define CMP_FPR_FILT_PER_MASK  (0xFFU)
#define CMP_FPR_FILT_PER_SHIFT (0U)
/*! FILT_PER - Filter Sample Period
 */
#define CMP_FPR_FILT_PER(x) (((uint16_t)(((uint16_t)(x)) << CMP_FPR_FILT_PER_SHIFT)) & CMP_FPR_FILT_PER_MASK)
/*! @} */

/*! @name SCR - CMP Status and Control Register */
/*! @{ */

#define CMP_SCR_COUT_MASK  (0x1U)
#define CMP_SCR_COUT_SHIFT (0U)
/*! COUT - Analog Comparator Output
 */
#define CMP_SCR_COUT(x) (((uint16_t)(((uint16_t)(x)) << CMP_SCR_COUT_SHIFT)) & CMP_SCR_COUT_MASK)

#define CMP_SCR_CFF_MASK  (0x2U)
#define CMP_SCR_CFF_SHIFT (1U)
/*! CFF - Analog Comparator Flag Falling
 *  0b0..Falling-edge on COUT has not been detected.
 *  0b1..Falling-edge on COUT has occurred.
 */
#define CMP_SCR_CFF(x) (((uint16_t)(((uint16_t)(x)) << CMP_SCR_CFF_SHIFT)) & CMP_SCR_CFF_MASK)

#define CMP_SCR_CFR_MASK  (0x4U)
#define CMP_SCR_CFR_SHIFT (2U)
/*! CFR - Analog Comparator Flag Rising
 *  0b0..Rising-edge on COUT has not been detected.
 *  0b1..Rising-edge on COUT has occurred.
 */
#define CMP_SCR_CFR(x) (((uint16_t)(((uint16_t)(x)) << CMP_SCR_CFR_SHIFT)) & CMP_SCR_CFR_MASK)

#define CMP_SCR_IEF_MASK  (0x8U)
#define CMP_SCR_IEF_SHIFT (3U)
/*! IEF - Comparator Interrupt Enable Falling
 *  0b0..Interrupt is disabled.
 *  0b1..Interrupt is enabled.
 */
#define CMP_SCR_IEF(x) (((uint16_t)(((uint16_t)(x)) << CMP_SCR_IEF_SHIFT)) & CMP_SCR_IEF_MASK)

#define CMP_SCR_IER_MASK  (0x10U)
#define CMP_SCR_IER_SHIFT (4U)
/*! IER - Comparator Interrupt Enable Rising
 *  0b0..Interrupt is disabled.
 *  0b1..Interrupt is enabled.
 */
#define CMP_SCR_IER(x) (((uint16_t)(((uint16_t)(x)) << CMP_SCR_IER_SHIFT)) & CMP_SCR_IER_MASK)

#define CMP_SCR_DMAEN_MASK  (0x40U)
#define CMP_SCR_DMAEN_SHIFT (6U)
/*! DMAEN - DMA Enable Control
 *  0b0..DMA is disabled.
 *  0b1..DMA is enabled.
 */
#define CMP_SCR_DMAEN(x) (((uint16_t)(((uint16_t)(x)) << CMP_SCR_DMAEN_SHIFT)) & CMP_SCR_DMAEN_MASK)
/*! @} */

/*! @name DACCR - DAC Control Register */
/*! @{ */

#define CMP_DACCR_VOSEL_MASK  (0xFFU)
#define CMP_DACCR_VOSEL_SHIFT (0U)
/*! VOSEL - DAC Output Voltage Select
 */
#define CMP_DACCR_VOSEL(x) (((uint16_t)(((uint16_t)(x)) << CMP_DACCR_VOSEL_SHIFT)) & CMP_DACCR_VOSEL_MASK)
/*! @} */

/*! @name MUXCR - MUX Control Register */
/*! @{ */

#define CMP_MUXCR_MSEL_MASK  (0x7U)
#define CMP_MUXCR_MSEL_SHIFT (0U)
/*! MSEL - Minus Input Mux Control
 *  0b000..IN0
 *  0b001..IN1
 *  0b010..IN2
 *  0b011..IN3
 *  0b100..IN4
 *  0b101..IN5
 *  0b110..IN6
 *  0b111..IN7
 */
#define CMP_MUXCR_MSEL(x) (((uint16_t)(((uint16_t)(x)) << CMP_MUXCR_MSEL_SHIFT)) & CMP_MUXCR_MSEL_MASK)

#define CMP_MUXCR_PSEL_MASK  (0x38U)
#define CMP_MUXCR_PSEL_SHIFT (3U)
/*! PSEL - Plus Input Mux Control
 *  0b000..IN0
 *  0b001..IN1
 *  0b010..IN2
 *  0b011..IN3
 *  0b100..IN4
 *  0b101..IN5
 *  0b110..IN6
 *  0b111..IN7
 */
#define CMP_MUXCR_PSEL(x) (((uint16_t)(((uint16_t)(x)) << CMP_MUXCR_PSEL_SHIFT)) & CMP_MUXCR_PSEL_MASK)

#define CMP_MUXCR_VRSEL_MASK  (0x40U)
#define CMP_MUXCR_VRSEL_SHIFT (6U)
/*! VRSEL - Supply Voltage Reference Source Select
 *  0b0..Vin1 is selected as resistor ladder network supply reference.
 *  0b1..Vin2 is selected as resistor ladder network supply reference.
 */
#define CMP_MUXCR_VRSEL(x) (((uint16_t)(((uint16_t)(x)) << CMP_MUXCR_VRSEL_SHIFT)) & CMP_MUXCR_VRSEL_MASK)

#define CMP_MUXCR_DACEN_MASK  (0x80U)
#define CMP_MUXCR_DACEN_SHIFT (7U)
/*! DACEN - DAC Enable
 *  0b0..DAC is disabled.
 *  0b1..DAC is enabled.
 */
#define CMP_MUXCR_DACEN(x) (((uint16_t)(((uint16_t)(x)) << CMP_MUXCR_DACEN_SHIFT)) & CMP_MUXCR_DACEN_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group CMP_Register_Masks */

/* CMP - Peripheral instance base addresses */
/** Peripheral CMPA base address */
#define CMPA_BASE (0xE020u)
/** Peripheral CMPA base pointer */
#define CMPA ((CMP_Type *)CMPA_BASE)
/** Peripheral CMPB base address */
#define CMPB_BASE (0xE028u)
/** Peripheral CMPB base pointer */
#define CMPB ((CMP_Type *)CMPB_BASE)
/** Peripheral CMPC base address */
#define CMPC_BASE (0xE030u)
/** Peripheral CMPC base pointer */
#define CMPC ((CMP_Type *)CMPC_BASE)
/** Peripheral CMPD base address */
#define CMPD_BASE (0xE038u)
/** Peripheral CMPD base pointer */
#define CMPD ((CMP_Type *)CMPD_BASE)
/** Array initializer of CMP peripheral base addresses */
#define CMP_BASE_ADDRS                             \
    {                                              \
        CMPA_BASE, CMPB_BASE, CMPC_BASE, CMPD_BASE \
    }
/** Array initializer of CMP peripheral base pointers */
#define CMP_BASE_PTRS          \
    {                          \
        CMPA, CMPB, CMPC, CMPD \
    }

/*!
 * @}
 */ /* end of group CMP_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- COP Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COP_Peripheral_Access_Layer COP Peripheral Access Layer
 * @{
 */

/** COP - Register Layout Typedef */
typedef struct
{
    __IO uint16_t CTRL;   /**< COP Control Register, offset: 0x0 */
    __IO uint16_t TOUT;   /**< COP Timeout Register, offset: 0x1 */
    __IO uint16_t CNTR;   /**< COP Counter Register, offset: 0x2 */
    __IO uint16_t INTVAL; /**< COP Interrupt Value Register, offset: 0x3 */
    __IO uint16_t WINDOW; /**< COP Window Timeout Register, offset: 0x4 */
} COP_Type;

/* ----------------------------------------------------------------------------
   -- COP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COP_Register_Masks COP Register Masks
 * @{
 */

/*! @name CTRL - COP Control Register */
/*! @{ */

#define COP_CTRL_CWP_MASK  (0x1U)
#define COP_CTRL_CWP_SHIFT (0U)
/*! CWP - COP Write Protect
 *  0b0..The CTRL, INTVAL , WINDOW and TOUT registers are readable and writable. (default)
 *  0b1..The CTRL, INTVAL , WINDOW and TOUT registers are read-only.
 */
#define COP_CTRL_CWP(x) (((uint16_t)(((uint16_t)(x)) << COP_CTRL_CWP_SHIFT)) & COP_CTRL_CWP_MASK)

#define COP_CTRL_CEN_MASK  (0x2U)
#define COP_CTRL_CEN_SHIFT (1U)
/*! CEN - COP Enable
 *  0b0..COP counter is disabled.
 *  0b1..COP counter is enabled. (default)
 */
#define COP_CTRL_CEN(x) (((uint16_t)(((uint16_t)(x)) << COP_CTRL_CEN_SHIFT)) & COP_CTRL_CEN_MASK)

#define COP_CTRL_CWEN_MASK  (0x4U)
#define COP_CTRL_CWEN_SHIFT (2U)
/*! CWEN - COP Wait Mode Enable
 *  0b0..COP counter stops in wait mode. (default)
 *  0b1..COP counter runs in wait mode if CEN is set to one.
 */
#define COP_CTRL_CWEN(x) (((uint16_t)(((uint16_t)(x)) << COP_CTRL_CWEN_SHIFT)) & COP_CTRL_CWEN_MASK)

#define COP_CTRL_CSEN_MASK  (0x8U)
#define COP_CTRL_CSEN_SHIFT (3U)
/*! CSEN - COP Stop Mode Enable
 *  0b0..COP counter stops in stop mode. (default)
 *  0b1..COP counter runs in stop mode if CEN is set to one.
 */
#define COP_CTRL_CSEN(x) (((uint16_t)(((uint16_t)(x)) << COP_CTRL_CSEN_SHIFT)) & COP_CTRL_CSEN_MASK)

#define COP_CTRL_CLOREN_MASK  (0x10U)
#define COP_CTRL_CLOREN_SHIFT (4U)
/*! CLOREN - COP Loss of Reference Enable
 *  0b0..COP loss of reference counter is disabled. (default)
 *  0b1..COP loss of reference counter is enabled.
 */
#define COP_CTRL_CLOREN(x) (((uint16_t)(((uint16_t)(x)) << COP_CTRL_CLOREN_SHIFT)) & COP_CTRL_CLOREN_MASK)

#define COP_CTRL_CLKSEL_MASK  (0x60U)
#define COP_CTRL_CLKSEL_SHIFT (5U)
/*! CLKSEL - Clock Source Select
 *  0b00..Relaxation oscillator output (ROSC) is used to clock the counter (default)
 *  0b01..Crystal oscillator output (COSC) is used to clock the counter
 *  0b10..IP bus clock is used to clock the counter Do not select the IP bus clock to clock the counter if the
 *        application requires the COP to wake the device from stop mode.
 *  0b11..Low speed oscillator is used to clock the counter
 */
#define COP_CTRL_CLKSEL(x) (((uint16_t)(((uint16_t)(x)) << COP_CTRL_CLKSEL_SHIFT)) & COP_CTRL_CLKSEL_MASK)

#define COP_CTRL_INTEN_MASK  (0x80U)
#define COP_CTRL_INTEN_SHIFT (7U)
/*! INTEN - Interrupt Enable
 *  0b0..COP interrupt is disabled. (default)
 *  0b1..COP interrupt is enabled.
 */
#define COP_CTRL_INTEN(x) (((uint16_t)(((uint16_t)(x)) << COP_CTRL_INTEN_SHIFT)) & COP_CTRL_INTEN_MASK)

#define COP_CTRL_PSS_MASK  (0x300U)
#define COP_CTRL_PSS_SHIFT (8U)
/*! PSS - Prescaler Select
 *  0b00..No division
 *  0b01..Divide by 16
 *  0b10..Divide by 256
 *  0b11..Divide by 1024
 */
#define COP_CTRL_PSS(x) (((uint16_t)(((uint16_t)(x)) << COP_CTRL_PSS_SHIFT)) & COP_CTRL_PSS_MASK)
/*! @} */

/*! @name TOUT - COP Timeout Register */
/*! @{ */

#define COP_TOUT_TIMEOUT_MASK  (0xFFFFU)
#define COP_TOUT_TIMEOUT_SHIFT (0U)
/*! TIMEOUT - COP Timeout Period
 */
#define COP_TOUT_TIMEOUT(x) (((uint16_t)(((uint16_t)(x)) << COP_TOUT_TIMEOUT_SHIFT)) & COP_TOUT_TIMEOUT_MASK)
/*! @} */

/*! @name CNTR - COP Counter Register */
/*! @{ */

#define COP_CNTR_COUNT_SERVICE_MASK  (0xFFFFU)
#define COP_CNTR_COUNT_SERVICE_SHIFT (0U)
/*! COUNT_SERVICE - COP Count/Service
 */
#define COP_CNTR_COUNT_SERVICE(x) \
    (((uint16_t)(((uint16_t)(x)) << COP_CNTR_COUNT_SERVICE_SHIFT)) & COP_CNTR_COUNT_SERVICE_MASK)
/*! @} */

/*! @name INTVAL - COP Interrupt Value Register */
/*! @{ */

#define COP_INTVAL_INTERRUPT_VALUE_MASK  (0xFFFFU)
#define COP_INTVAL_INTERRUPT_VALUE_SHIFT (0U)
/*! INTERRUPT_VALUE - COP Interrupt Value
 */
#define COP_INTVAL_INTERRUPT_VALUE(x) \
    (((uint16_t)(((uint16_t)(x)) << COP_INTVAL_INTERRUPT_VALUE_SHIFT)) & COP_INTVAL_INTERRUPT_VALUE_MASK)
/*! @} */

/*! @name WINDOW - COP Window Timeout Register */
/*! @{ */

#define COP_WINDOW_WINDOW_VALUE_MASK  (0xFFFFU)
#define COP_WINDOW_WINDOW_VALUE_SHIFT (0U)
/*! WINDOW_VALUE - COP Window Timeout Value
 */
#define COP_WINDOW_WINDOW_VALUE(x) \
    (((uint16_t)(((uint16_t)(x)) << COP_WINDOW_WINDOW_VALUE_SHIFT)) & COP_WINDOW_WINDOW_VALUE_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group COP_Register_Masks */

/* COP - Peripheral instance base addresses */
/** Peripheral COP base address */
#define COP_BASE (0xE320u)
/** Peripheral COP base pointer */
#define COP ((COP_Type *)COP_BASE)
/** Array initializer of COP peripheral base addresses */
#define COP_BASE_ADDRS \
    {                  \
        COP_BASE       \
    }
/** Array initializer of COP peripheral base pointers */
#define COP_BASE_PTRS \
    {                 \
        COP           \
    }

/*!
 * @}
 */ /* end of group COP_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- CRC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Peripheral_Access_Layer CRC Peripheral Access Layer
 * @{
 */

/** CRC - Register Layout Typedef */
typedef struct
{
    union
    {                         /* offset: 0x0 */
        __IO uint8_t DATA8;   /**< Low lower byte of CRC_DATA register., offset: 0x0 */
        __IO uint16_t DATA16; /**< Low word of CRC_DATA register., offset: 0x0 */
        __IO uint32_t DATA32; /**< CRC Data register, offset: 0x0 */
    };
    __IO uint32_t GPOLY; /**< CRC Polynomial register, offset: 0x2 */
    __IO uint32_t CTRL;  /**< CRC Control register, offset: 0x4 */
} CRC_Type;

/* ----------------------------------------------------------------------------
   -- CRC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Masks CRC Register Masks
 * @{
 */

/*! @name DATA8 - Low lower byte of CRC_DATA register. */
/*! @{ */

#define CRC_DATA8_DATA8_MASK  (0xFFU)
#define CRC_DATA8_DATA8_SHIFT (0U)
/*! DATA8 - DATALL stores the lowest 8 bits of the 32 bit DATA
 */
#define CRC_DATA8_DATA8(x) (((uint8_t)(((uint8_t)(x)) << CRC_DATA8_DATA8_SHIFT)) & CRC_DATA8_DATA8_MASK)
/*! @} */

/*! @name DATA16 - Low word of CRC_DATA register. */
/*! @{ */

#define CRC_DATA16_DATA16_MASK  (0xFFFFU)
#define CRC_DATA16_DATA16_SHIFT (0U)
/*! DATA16 - DATAL stores the lower 16 bits of the 16/32 bit DATA
 */
#define CRC_DATA16_DATA16(x) (((uint16_t)(((uint16_t)(x)) << CRC_DATA16_DATA16_SHIFT)) & CRC_DATA16_DATA16_MASK)
/*! @} */

/*! @name DATA32 - CRC Data register */
/*! @{ */

#define CRC_DATA32_LL_MASK  (0xFFUL)
#define CRC_DATA32_LL_SHIFT (0UL)
/*! LL - CRC Low Lower Byte
 */
#define CRC_DATA32_LL(x) (((uint32_t)(((uint32_t)(x)) << CRC_DATA32_LL_SHIFT)) & CRC_DATA32_LL_MASK)

#define CRC_DATA32_LU_MASK  (0xFF00UL)
#define CRC_DATA32_LU_SHIFT (8UL)
/*! LU - CRC Low Upper Byte
 */
#define CRC_DATA32_LU(x) (((uint32_t)(((uint32_t)(x)) << CRC_DATA32_LU_SHIFT)) & CRC_DATA32_LU_MASK)

#define CRC_DATA32_HL_MASK  (0xFF0000UL)
#define CRC_DATA32_HL_SHIFT (16UL)
/*! HL - CRC High Lower Byte
 */
#define CRC_DATA32_HL(x) (((uint32_t)(((uint32_t)(x)) << CRC_DATA32_HL_SHIFT)) & CRC_DATA32_HL_MASK)

#define CRC_DATA32_HU_MASK  (0xFF000000UL)
#define CRC_DATA32_HU_SHIFT (24UL)
/*! HU - CRC High Upper Byte
 */
#define CRC_DATA32_HU(x) (((uint32_t)(((uint32_t)(x)) << CRC_DATA32_HU_SHIFT)) & CRC_DATA32_HU_MASK)
/*! @} */

/*! @name GPOLY - CRC Polynomial register */
/*! @{ */

#define CRC_GPOLY_LOW_MASK  (0xFFFFUL)
#define CRC_GPOLY_LOW_SHIFT (0UL)
/*! LOW - Low Polynominal Half-word
 */
#define CRC_GPOLY_LOW(x) (((uint32_t)(((uint32_t)(x)) << CRC_GPOLY_LOW_SHIFT)) & CRC_GPOLY_LOW_MASK)

#define CRC_GPOLY_HIGH_MASK  (0xFFFF0000UL)
#define CRC_GPOLY_HIGH_SHIFT (16UL)
/*! HIGH - High Polynominal Half-word
 */
#define CRC_GPOLY_HIGH(x) (((uint32_t)(((uint32_t)(x)) << CRC_GPOLY_HIGH_SHIFT)) & CRC_GPOLY_HIGH_MASK)
/*! @} */

/*! @name CTRL - CRC Control register */
/*! @{ */

#define CRC_CTRL_TCRC_MASK  (0x1000000UL)
#define CRC_CTRL_TCRC_SHIFT (24UL)
/*! TCRC
 *  0b0..16-bit CRC protocol.
 *  0b1..32-bit CRC protocol.
 */
#define CRC_CTRL_TCRC(x) (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_TCRC_SHIFT)) & CRC_CTRL_TCRC_MASK)

#define CRC_CTRL_WAS_MASK  (0x2000000UL)
#define CRC_CTRL_WAS_SHIFT (25UL)
/*! WAS - Write CRC Data Register As Seed
 *  0b0..Writes to the CRC data register are data values.
 *  0b1..Writes to the CRC data register are seed values.
 */
#define CRC_CTRL_WAS(x) (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_WAS_SHIFT)) & CRC_CTRL_WAS_MASK)

#define CRC_CTRL_FXOR_MASK  (0x4000000UL)
#define CRC_CTRL_FXOR_SHIFT (26UL)
/*! FXOR - Complement Read Of CRC Data Register
 *  0b0..No XOR on reading.
 *  0b1..Invert or complement the read value of the CRC Data register.
 */
#define CRC_CTRL_FXOR(x) (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_FXOR_SHIFT)) & CRC_CTRL_FXOR_MASK)

#define CRC_CTRL_TOTR_MASK  (0x30000000UL)
#define CRC_CTRL_TOTR_SHIFT (28UL)
/*! TOTR - Type Of Transpose For Read
 *  0b00..No transposition.
 *  0b01..Bits in bytes are transposed; bytes are not transposed.
 *  0b10..Both bits in bytes and bytes are transposed.
 *  0b11..Only bytes are transposed; no bits in a byte are transposed.
 */
#define CRC_CTRL_TOTR(x) (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_TOTR_SHIFT)) & CRC_CTRL_TOTR_MASK)

#define CRC_CTRL_TOT_MASK  (0xC0000000UL)
#define CRC_CTRL_TOT_SHIFT (30UL)
/*! TOT - Type Of Transpose For Writes
 *  0b00..No transposition.
 *  0b01..Bits in bytes are transposed; bytes are not transposed.
 *  0b10..Both bits in bytes and bytes are transposed.
 *  0b11..Only bytes are transposed; no bits in a byte are transposed.
 */
#define CRC_CTRL_TOT(x) (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_TOT_SHIFT)) & CRC_CTRL_TOT_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group CRC_Register_Masks */

/* CRC - Peripheral instance base addresses */
/** Peripheral CRC base address */
#define CRC_BASE (0xE3A0u)
/** Peripheral CRC base pointer */
#define CRC ((CRC_Type *)CRC_BASE)
/** Array initializer of CRC peripheral base addresses */
#define CRC_BASE_ADDRS \
    {                  \
        CRC_BASE       \
    }
/** Array initializer of CRC peripheral base pointers */
#define CRC_BASE_PTRS \
    {                 \
        CRC           \
    }

/*!
 * @}
 */ /* end of group CRC_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- DAC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Peripheral_Access_Layer DAC Peripheral Access Layer
 * @{
 */

/** DAC - Register Layout Typedef */
typedef struct
{
    __IO uint16_t CTRL0; /**< Control Register 0, offset: 0x0 */
    union
    { /* offset: 0x1 */
        struct
        {                               /* offset: 0x1 */
            __IO uint16_t DATAREG_FMT0; /**< Buffered Data Register, offset: 0x1 */
            __IO uint16_t STEPVAL_FMT0; /**< Step Size Register, offset: 0x2 */
            __IO uint16_t MINVAL_FMT0;  /**< Minimum Value Register, offset: 0x3 */
            __IO uint16_t MAXVAL_FMT0;  /**< Maximum Value Register, offset: 0x4 */
        } FMT0;
        struct
        {                               /* offset: 0x1 */
            __IO uint16_t DATAREG_FMT1; /**< Buffered Data Register, offset: 0x1 */
            __IO uint16_t STEPVAL_FMT1; /**< Step Size Register, offset: 0x2 */
            __IO uint16_t MINVAL_FMT1;  /**< Minimum Value Register, offset: 0x3 */
            __IO uint16_t MAXVAL_FMT1;  /**< Maximum Value Register, offset: 0x4 */
        } FMT1;
    };
    __I uint16_t STATUS;   /**< Status Register, offset: 0x5 */
    __IO uint16_t CTRL1;   /**< Control Register 1, offset: 0x6 */
    __IO uint16_t COMPARE; /**< Compare Register, offset: 0x7 */
} DAC_Type;

/* ----------------------------------------------------------------------------
   -- DAC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Masks DAC Register Masks
 * @{
 */

/*! @name CTRL0 - Control Register 0 */
/*! @{ */

#define DAC_CTRL0_PDN_MASK  (0x1U)
#define DAC_CTRL0_PDN_SHIFT (0U)
/*! PDN - Power Down
 *  0b0..DAC is operational.
 *  0b1..DAC is powered down. (default)
 */
#define DAC_CTRL0_PDN(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_PDN_SHIFT)) & DAC_CTRL0_PDN_MASK)

#define DAC_CTRL0_FORMAT_MASK  (0x2U)
#define DAC_CTRL0_FORMAT_SHIFT (1U)
/*! FORMAT - Data Format
 *  0b0..Data words are right-justified (default)
 *  0b1..Data words are left-justified
 */
#define DAC_CTRL0_FORMAT(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_FORMAT_SHIFT)) & DAC_CTRL0_FORMAT_MASK)

#define DAC_CTRL0_SYNC_EN_MASK  (0x4U)
#define DAC_CTRL0_SYNC_EN_SHIFT (2U)
/*! SYNC_EN - Sync Enable
 *  0b0..Asynchronous mode. Data written to the buffered registers is used on the next clock cycle.
 *  0b1..Synchronous mode. SYNC_IN signal updates data in the buffered registers.
 */
#define DAC_CTRL0_SYNC_EN(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_SYNC_EN_SHIFT)) & DAC_CTRL0_SYNC_EN_MASK)

#define DAC_CTRL0_AUTO_MASK  (0x8U)
#define DAC_CTRL0_AUTO_SHIFT (3U)
/*! AUTO - Automatic Mode
 *  0b0..Normal mode. Automatic waveform generation disabled.
 *  0b1..Automatic waveform generation enabled.
 */
#define DAC_CTRL0_AUTO(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_AUTO_SHIFT)) & DAC_CTRL0_AUTO_MASK)

#define DAC_CTRL0_DOWN_MASK  (0x10U)
#define DAC_CTRL0_DOWN_SHIFT (4U)
/*! DOWN - Enable Down Counting
 *  0b0..Disable down-counting
 *  0b1..Enable down-counting
 */
#define DAC_CTRL0_DOWN(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_DOWN_SHIFT)) & DAC_CTRL0_DOWN_MASK)

#define DAC_CTRL0_UP_MASK  (0x20U)
#define DAC_CTRL0_UP_SHIFT (5U)
/*! UP - Enable Up-Counting
 *  0b0..Disable up-counting
 *  0b1..Enable up-counting
 */
#define DAC_CTRL0_UP(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_UP_SHIFT)) & DAC_CTRL0_UP_MASK)

#define DAC_CTRL0_HSLS_MASK  (0x40U)
#define DAC_CTRL0_HSLS_SHIFT (6U)
/*! HSLS - High/Low Speed
 *  0b0..High speed mode (default)
 *  0b1..Low speed mode
 */
#define DAC_CTRL0_HSLS(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_HSLS_SHIFT)) & DAC_CTRL0_HSLS_MASK)

#define DAC_CTRL0_DMA_EN_MASK  (0x80U)
#define DAC_CTRL0_DMA_EN_SHIFT (7U)
/*! DMA_EN - Enable DMA Support
 *  0b0..Disable DMA support (default)
 *  0b1..Enable DMA support
 */
#define DAC_CTRL0_DMA_EN(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_DMA_EN_SHIFT)) & DAC_CTRL0_DMA_EN_MASK)

#define DAC_CTRL0_WTMK_LVL_MASK  (0x300U)
#define DAC_CTRL0_WTMK_LVL_SHIFT (8U)
/*! WTMK_LVL - Watermark Level
 *  0b00..Watermark value is 0
 *  0b01..Watermark value is 2 (default)
 *  0b10..Watermark value is 4
 *  0b11..Watermark value is 6
 */
#define DAC_CTRL0_WTMK_LVL(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_WTMK_LVL_SHIFT)) & DAC_CTRL0_WTMK_LVL_MASK)

#define DAC_CTRL0_SYNCEDGE_MASK  (0xC00U)
#define DAC_CTRL0_SYNCEDGE_SHIFT (10U)
/*! SYNCEDGE - Sync edge
 *  0b00..No active edge is selected, therefore the SYNC input is ignored
 *  0b01..Updates occur on the falling edge of the SYNC input
 *  0b10..Updates occur on the rising edge of the SYNC input
 *  0b11..Updates occur on both edges of the SYNC input
 */
#define DAC_CTRL0_SYNCEDGE(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_SYNCEDGE_SHIFT)) & DAC_CTRL0_SYNCEDGE_MASK)

#define DAC_CTRL0_FILT_EN_MASK  (0x1000U)
#define DAC_CTRL0_FILT_EN_SHIFT (12U)
/*! FILT_EN - Glitch Filter Enable
 *  0b0..Disable glitch filter
 *  0b1..Enable glitch filter
 */
#define DAC_CTRL0_FILT_EN(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_FILT_EN_SHIFT)) & DAC_CTRL0_FILT_EN_MASK)

#define DAC_CTRL0_LDOK_MASK  (0x4000U)
#define DAC_CTRL0_LDOK_SHIFT (14U)
/*! LDOK - Load Okay
 *  0b0..Buffered values of STEPVAL, MINVAL, and MAXVAL will not be updated and the existing values will be reused.
 *  0b1..Buffered values of STEPVAL, MINVAL, and MAXVAL will be updated and used at active edge of SYNC_IN.
 */
#define DAC_CTRL0_LDOK(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_LDOK_SHIFT)) & DAC_CTRL0_LDOK_MASK)

#define DAC_CTRL0_ONESHOT_MASK  (0x8000U)
#define DAC_CTRL0_ONESHOT_SHIFT (15U)
/*! ONESHOT - One shot
 *  0b0..Automatic waveform generation logic will create a repeated (continuous) waveform upon receiving an active
 *       SYNC edge, otherwise the waveform repeats when it reaches its MIN or MAX value.
 *  0b1..Automatic waveform generation logic will create a single pattern and stop at the final value. It will
 *       remain at this final value until a new active edge occurs on the SYNC input, and then the waveform will be
 *       repeated.
 */
#define DAC_CTRL0_ONESHOT(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_ONESHOT_SHIFT)) & DAC_CTRL0_ONESHOT_MASK)
/*! @} */

/*! @name DATAREG_FMT0 - Buffered Data Register */
/*! @{ */

#define DAC_DATAREG_FMT0_DATA_MASK  (0xFFFU)
#define DAC_DATAREG_FMT0_DATA_SHIFT (0U)
/*! DATA - DAC data (right-justified)
 */
#define DAC_DATAREG_FMT0_DATA(x) \
    (((uint16_t)(((uint16_t)(x)) << DAC_DATAREG_FMT0_DATA_SHIFT)) & DAC_DATAREG_FMT0_DATA_MASK)
/*! @} */

/*! @name STEPVAL_FMT0 - Step Size Register */
/*! @{ */

#define DAC_STEPVAL_FMT0_STEP_MASK  (0xFFFU)
#define DAC_STEPVAL_FMT0_STEP_SHIFT (0U)
/*! STEP - STEP size (right-justified)
 */
#define DAC_STEPVAL_FMT0_STEP(x) \
    (((uint16_t)(((uint16_t)(x)) << DAC_STEPVAL_FMT0_STEP_SHIFT)) & DAC_STEPVAL_FMT0_STEP_MASK)
/*! @} */

/*! @name MINVAL_FMT0 - Minimum Value Register */
/*! @{ */

#define DAC_MINVAL_FMT0_MINVAL_MASK  (0xFFFU)
#define DAC_MINVAL_FMT0_MINVAL_SHIFT (0U)
/*! MINVAL - Minimum value (right-justified)
 */
#define DAC_MINVAL_FMT0_MINVAL(x) \
    (((uint16_t)(((uint16_t)(x)) << DAC_MINVAL_FMT0_MINVAL_SHIFT)) & DAC_MINVAL_FMT0_MINVAL_MASK)
/*! @} */

/*! @name MAXVAL_FMT0 - Maximum Value Register */
/*! @{ */

#define DAC_MAXVAL_FMT0_MAXVAL_MASK  (0xFFFU)
#define DAC_MAXVAL_FMT0_MAXVAL_SHIFT (0U)
/*! MAXVAL - Maximum value (right-justified)
 */
#define DAC_MAXVAL_FMT0_MAXVAL(x) \
    (((uint16_t)(((uint16_t)(x)) << DAC_MAXVAL_FMT0_MAXVAL_SHIFT)) & DAC_MAXVAL_FMT0_MAXVAL_MASK)
/*! @} */

/*! @name DATAREG_FMT1 - Buffered Data Register */
/*! @{ */

#define DAC_DATAREG_FMT1_DATA_MASK  (0xFFF0U)
#define DAC_DATAREG_FMT1_DATA_SHIFT (4U)
/*! DATA - DAC data (left-justified)
 */
#define DAC_DATAREG_FMT1_DATA(x) \
    (((uint16_t)(((uint16_t)(x)) << DAC_DATAREG_FMT1_DATA_SHIFT)) & DAC_DATAREG_FMT1_DATA_MASK)
/*! @} */

/*! @name STEPVAL_FMT1 - Step Size Register */
/*! @{ */

#define DAC_STEPVAL_FMT1_STEP_MASK  (0xFFF0U)
#define DAC_STEPVAL_FMT1_STEP_SHIFT (4U)
/*! STEP - STEP size (left-justified)
 */
#define DAC_STEPVAL_FMT1_STEP(x) \
    (((uint16_t)(((uint16_t)(x)) << DAC_STEPVAL_FMT1_STEP_SHIFT)) & DAC_STEPVAL_FMT1_STEP_MASK)
/*! @} */

/*! @name MINVAL_FMT1 - Minimum Value Register */
/*! @{ */

#define DAC_MINVAL_FMT1_MINVAL_MASK  (0xFFF0U)
#define DAC_MINVAL_FMT1_MINVAL_SHIFT (4U)
/*! MINVAL - Minimum value (left-justified)
 */
#define DAC_MINVAL_FMT1_MINVAL(x) \
    (((uint16_t)(((uint16_t)(x)) << DAC_MINVAL_FMT1_MINVAL_SHIFT)) & DAC_MINVAL_FMT1_MINVAL_MASK)
/*! @} */

/*! @name MAXVAL_FMT1 - Maximum Value Register */
/*! @{ */

#define DAC_MAXVAL_FMT1_MAXVAL_MASK  (0xFFF0U)
#define DAC_MAXVAL_FMT1_MAXVAL_SHIFT (4U)
/*! MAXVAL - Maximum value (left-justified)
 */
#define DAC_MAXVAL_FMT1_MAXVAL(x) \
    (((uint16_t)(((uint16_t)(x)) << DAC_MAXVAL_FMT1_MAXVAL_SHIFT)) & DAC_MAXVAL_FMT1_MAXVAL_MASK)
/*! @} */

/*! @name STATUS - Status Register */
/*! @{ */

#define DAC_STATUS_EMPTY_MASK  (0x1U)
#define DAC_STATUS_EMPTY_SHIFT (0U)
/*! EMPTY - Indicates that the FIFO is empty
 *  0b0..FIFO is not empty
 *  0b1..FIFO is empty (on reset)
 */
#define DAC_STATUS_EMPTY(x) (((uint16_t)(((uint16_t)(x)) << DAC_STATUS_EMPTY_SHIFT)) & DAC_STATUS_EMPTY_MASK)

#define DAC_STATUS_FULL_MASK  (0x2U)
#define DAC_STATUS_FULL_SHIFT (1U)
/*! FULL - Indicates that the FIFO is full
 *  0b0..FIFO is not full (on reset).
 *  0b1..FIFO is full.
 */
#define DAC_STATUS_FULL(x) (((uint16_t)(((uint16_t)(x)) << DAC_STATUS_FULL_SHIFT)) & DAC_STATUS_FULL_MASK)
/*! @} */

/*! @name CTRL1 - Control Register 1 */
/*! @{ */

#define DAC_CTRL1_FILT_CNT_MASK  (0x3FU)
#define DAC_CTRL1_FILT_CNT_SHIFT (0U)
/*! FILT_CNT - Glitch Filter Count
 */
#define DAC_CTRL1_FILT_CNT(x) (((uint16_t)(((uint16_t)(x)) << DAC_CTRL1_FILT_CNT_SHIFT)) & DAC_CTRL1_FILT_CNT_MASK)
/*! @} */

/*! @name COMPARE - Compare Register */
/*! @{ */

#define DAC_COMPARE_COMPARE_MASK  (0xFFFFU)
#define DAC_COMPARE_COMPARE_SHIFT (0U)
/*! COMPARE - Compare value
 */
#define DAC_COMPARE_COMPARE(x) (((uint16_t)(((uint16_t)(x)) << DAC_COMPARE_COMPARE_SHIFT)) & DAC_COMPARE_COMPARE_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group DAC_Register_Masks */

/* DAC - Peripheral instance base addresses */
/** Peripheral DACA base address */
#define DACA_BASE (0xE000u)
/** Peripheral DACA base pointer */
#define DACA ((DAC_Type *)DACA_BASE)
/** Peripheral DACB base address */
#define DACB_BASE (0xE010u)
/** Peripheral DACB base pointer */
#define DACB ((DAC_Type *)DACB_BASE)
/** Array initializer of DAC peripheral base addresses */
#define DAC_BASE_ADDRS       \
    {                        \
        DACA_BASE, DACB_BASE \
    }
/** Array initializer of DAC peripheral base pointers */
#define DAC_BASE_PTRS \
    {                 \
        DACA, DACB    \
    }

/*!
 * @}
 */ /* end of group DAC_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- DMA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Peripheral_Access_Layer DMA Peripheral Access Layer
 * @{
 */

/** DMA - Register Layout Typedef */
typedef struct
{
    __IO uint32_t CR; /**< Control Register, offset: 0x0 */
    __I uint32_t ES;  /**< Error Status Register, offset: 0x4 */
    uint8_t RESERVED_0[4];
    __IO uint32_t ERQ; /**< Enable Request Register, offset: 0xC */
    uint8_t RESERVED_1[4];
    __IO uint32_t EEI; /**< Enable Error Interrupt Register, offset: 0x14 */
    __O uint8_t CEEI;  /**< Clear Enable Error Interrupt Register, offset: 0x18 */
    __O uint8_t SEEI;  /**< Set Enable Error Interrupt Register, offset: 0x19 */
    __O uint8_t CERQ;  /**< Clear Enable Request Register, offset: 0x1A */
    __O uint8_t SERQ;  /**< Set Enable Request Register, offset: 0x1B */
    __O uint8_t CDNE;  /**< Clear DONE Status Bit Register, offset: 0x1C */
    __O uint8_t SSRT;  /**< Set START Bit Register, offset: 0x1D */
    __O uint8_t CERR;  /**< Clear Error Register, offset: 0x1E */
    __O uint8_t CINT;  /**< Clear Interrupt Request Register, offset: 0x1F */
    uint8_t RESERVED_2[4];
    __IO uint32_t INT; /**< Interrupt Request Register, offset: 0x24 */
    uint8_t RESERVED_3[4];
    __IO uint32_t ERR; /**< Error Register, offset: 0x2C */
    uint8_t RESERVED_4[4];
    __I uint32_t HRS; /**< Hardware Request Status Register, offset: 0x34 */
    uint8_t RESERVED_5[12];
    __IO uint32_t EARS; /**< Enable Asynchronous Request in Stop Register, offset: 0x44 */
    uint8_t RESERVED_6[184];
    __IO uint8_t DCHPRI3; /**< Channel Priority Register, offset: 0x100 */
    __IO uint8_t DCHPRI2; /**< Channel Priority Register, offset: 0x101 */
    __IO uint8_t DCHPRI1; /**< Channel Priority Register, offset: 0x102 */
    __IO uint8_t DCHPRI0; /**< Channel Priority Register, offset: 0x103 */
    uint8_t RESERVED_7[3836];
    struct
    {                        /* offset: 0x1000, array step: 0x20 */
        __IO uint32_t SADDR; /**< TCD Source Address, array offset: 0x1000, array step: 0x20 */
        __IO uint16_t SOFF;  /**< TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20 */
        __IO uint16_t ATTR;  /**< TCD Transfer Attributes, array offset: 0x1006, array step: 0x20 */
        union
        {                                 /* offset: 0x1008, array step: 0x20 */
            __IO uint32_t NBYTES_MLNO;    /**< TCD Minor Byte Count (Minor Loop Mapping Disabled), array offset: 0x1008,
                                             array step: 0x20 */
            __IO uint32_t NBYTES_MLOFFNO; /**< TCD Signed Minor Loop Offset (Minor Loop Mapping Enabled and Offset
                                             Disabled), array offset: 0x1008, array step: 0x20 */
            __IO uint32_t NBYTES_MLOFFYES; /**< TCD Signed Minor Loop Offset (Minor Loop Mapping and Offset Enabled),
                                              array offset: 0x1008, array step: 0x20 */
        };
        __IO int32_t SLAST;  /**< TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20 */
        __IO uint32_t DADDR; /**< TCD Destination Address, array offset: 0x1010, array step: 0x20 */
        __IO uint16_t DOFF;  /**< TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20 */
        union
        {                                 /* offset: 0x1016, array step: 0x20 */
            __IO uint16_t CITER_ELINKNO;  /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled),
                                             array offset: 0x1016, array step: 0x20 */
            __IO uint16_t CITER_ELINKYES; /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled),
                                             array offset: 0x1016, array step: 0x20 */
        };
        __IO int32_t DLAST_SGA; /**< TCD Last Destination Address Adjustment/Scatter Gather Address, array offset:
                                   0x1018, array step: 0x20 */
        __IO uint16_t CSR;      /**< TCD Control and Status, array offset: 0x101C, array step: 0x20 */
        union
        {                                 /* offset: 0x101E, array step: 0x20 */
            __IO uint16_t BITER_ELINKNO;  /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking
                                             Disabled), array offset: 0x101E, array step: 0x20 */
            __IO uint16_t BITER_ELINKYES; /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking
                                             Enabled), array offset: 0x101E, array step: 0x20 */
        };
    } TCD[4];
} DMA_Type;

/* ----------------------------------------------------------------------------
   -- DMA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Masks DMA Register Masks
 * @{
 */

/*! @name CR - Control Register */
/*! @{ */

#define DMA_CR_EBWR_MASK  (0x1UL)
#define DMA_CR_EBWR_SHIFT (0UL)
/*! EBWR - Enable Buffered Writes
 *  0b0..Buffered writes are disabled.
 *  0b1..Buffered writes are enabled.
 */
#define DMA_CR_EBWR(x) (((uint32_t)(((uint32_t)(x)) << DMA_CR_EBWR_SHIFT)) & DMA_CR_EBWR_MASK)

#define DMA_CR_EDBG_MASK  (0x2UL)
#define DMA_CR_EDBG_SHIFT (1UL)
/*! EDBG - Enable Debug
 *  0b0..When in debug mode, the DMA continues to operate.
 *  0b1..When in debug mode, the DMA stalls the start of a new channel. Executing channels are allowed to
 *       complete. Channel execution resumes when the system exits debug mode or the EDBG bit is cleared.
 */
#define DMA_CR_EDBG(x) (((uint32_t)(((uint32_t)(x)) << DMA_CR_EDBG_SHIFT)) & DMA_CR_EDBG_MASK)

#define DMA_CR_ERCA_MASK  (0x4UL)
#define DMA_CR_ERCA_SHIFT (2UL)
/*! ERCA - Enable Round Robin Channel Arbitration
 *  0b0..Fixed priority arbitration is used for channel selection .
 *  0b1..Round robin arbitration is used for channel selection .
 */
#define DMA_CR_ERCA(x) (((uint32_t)(((uint32_t)(x)) << DMA_CR_ERCA_SHIFT)) & DMA_CR_ERCA_MASK)

#define DMA_CR_HOE_MASK  (0x10UL)
#define DMA_CR_HOE_SHIFT (4UL)
/*! HOE - Halt On Error
 *  0b0..Normal operation
 *  0b1..Any error causes the HALT bit to set. Subsequently, all service requests are ignored until the HALT bit is
 * cleared.
 */
#define DMA_CR_HOE(x) (((uint32_t)(((uint32_t)(x)) << DMA_CR_HOE_SHIFT)) & DMA_CR_HOE_MASK)

#define DMA_CR_HALT_MASK  (0x20UL)
#define DMA_CR_HALT_SHIFT (5UL)
/*! HALT - Halt DMA Operations
 *  0b0..Normal operation
 *  0b1..Stall the start of any new channels. Executing channels are allowed to complete. Channel execution resumes when
 * this bit is cleared.
 */
#define DMA_CR_HALT(x) (((uint32_t)(((uint32_t)(x)) << DMA_CR_HALT_SHIFT)) & DMA_CR_HALT_MASK)

#define DMA_CR_CLM_MASK  (0x40UL)
#define DMA_CR_CLM_SHIFT (6UL)
/*! CLM - Continuous Link Mode
 *  0b0..A minor loop channel link made to itself goes through channel arbitration before being activated again.
 *  0b1..A minor loop channel link made to itself does not go through channel arbitration before being activated
 *       again. Upon minor loop completion, the channel activates again if that channel has a minor loop channel
 *       link enabled and the link channel is itself. This effectively applies the minor loop offsets and restarts the
 *       next minor loop.
 */
#define DMA_CR_CLM(x) (((uint32_t)(((uint32_t)(x)) << DMA_CR_CLM_SHIFT)) & DMA_CR_CLM_MASK)

#define DMA_CR_EMLM_MASK  (0x80UL)
#define DMA_CR_EMLM_SHIFT (7UL)
/*! EMLM - Enable Minor Loop Mapping
 *  0b0..Disabled. TCDn.word2 is defined as a 32-bit NBYTES field.
 *  0b1..Enabled. TCDn.word2 is redefined to include individual enable fields, an offset field, and the NBYTES
 *       field. The individual enable fields allow the minor loop offset to be applied to the source address, the
 *       destination address, or both. The NBYTES field is reduced when either offset is enabled.
 */
#define DMA_CR_EMLM(x) (((uint32_t)(((uint32_t)(x)) << DMA_CR_EMLM_SHIFT)) & DMA_CR_EMLM_MASK)

#define DMA_CR_ECX_MASK  (0x10000UL)
#define DMA_CR_ECX_SHIFT (16UL)
/*! ECX - Error Cancel Transfer
 *  0b0..Normal operation
 *  0b1..Cancel the remaining data transfer in the same fashion as the CX bit. Stop the executing channel and
 *       force the minor loop to finish. The cancel takes effect after the last write of the current read/write
 *       sequence. The ECX bit clears itself after the cancel is honored. In addition to cancelling the transfer, ECX
 *       treats the cancel as an error condition, thus updating the Error Status register (DMAx_ES) and generating an
 *       optional error interrupt.
 */
#define DMA_CR_ECX(x) (((uint32_t)(((uint32_t)(x)) << DMA_CR_ECX_SHIFT)) & DMA_CR_ECX_MASK)

#define DMA_CR_CX_MASK  (0x20000UL)
#define DMA_CR_CX_SHIFT (17UL)
/*! CX - Cancel Transfer
 *  0b0..Normal operation
 *  0b1..Cancel the remaining data transfer. Stop the executing channel and force the minor loop to finish. The
 *       cancel takes effect after the last write of the current read/write sequence. The CX bit clears itself after
 *       the cancel has been honored. This cancel retires the channel normally as if the minor loop was completed.
 */
#define DMA_CR_CX(x) (((uint32_t)(((uint32_t)(x)) << DMA_CR_CX_SHIFT)) & DMA_CR_CX_MASK)

#define DMA_CR_ACTIVE_MASK  (0x80000000UL)
#define DMA_CR_ACTIVE_SHIFT (31UL)
/*! ACTIVE - DMA Active Status
 *  0b0..eDMA is idle.
 *  0b1..eDMA is executing a channel.
 */
#define DMA_CR_ACTIVE(x) (((uint32_t)(((uint32_t)(x)) << DMA_CR_ACTIVE_SHIFT)) & DMA_CR_ACTIVE_MASK)
/*! @} */

/*! @name ES - Error Status Register */
/*! @{ */

#define DMA_ES_DBE_MASK  (0x1UL)
#define DMA_ES_DBE_SHIFT (0UL)
/*! DBE - Destination Bus Error
 *  0b0..No destination bus error
 *  0b1..The last recorded error was a bus error on a destination write
 */
#define DMA_ES_DBE(x) (((uint32_t)(((uint32_t)(x)) << DMA_ES_DBE_SHIFT)) & DMA_ES_DBE_MASK)

#define DMA_ES_SBE_MASK  (0x2UL)
#define DMA_ES_SBE_SHIFT (1UL)
/*! SBE - Source Bus Error
 *  0b0..No source bus error
 *  0b1..The last recorded error was a bus error on a source read
 */
#define DMA_ES_SBE(x) (((uint32_t)(((uint32_t)(x)) << DMA_ES_SBE_SHIFT)) & DMA_ES_SBE_MASK)

#define DMA_ES_SGE_MASK  (0x4UL)
#define DMA_ES_SGE_SHIFT (2UL)
/*! SGE - Scatter/Gather Configuration Error
 *  0b0..No scatter/gather configuration error
 *  0b1..The last recorded error was a configuration error detected in the TCDn_DLASTSGA field. This field is
 *       checked at the beginning of a scatter/gather operation after major loop completion if TCDn_CSR[ESG] is
 *       enabled. TCDn_DLASTSGA is not on a 32 byte boundary.
 */
#define DMA_ES_SGE(x) (((uint32_t)(((uint32_t)(x)) << DMA_ES_SGE_SHIFT)) & DMA_ES_SGE_MASK)

#define DMA_ES_NCE_MASK  (0x8UL)
#define DMA_ES_NCE_SHIFT (3UL)
/*! NCE - NBYTES/CITER Configuration Error
 *  0b0..No NBYTES/CITER configuration error
 *  0b1..The last recorded error was a configuration error detected in the TCDn_NBYTES or TCDn_CITER fields.
 *       TCDn_NBYTES is not a multiple of TCDn_ATTR[SSIZE] and TCDn_ATTR[DSIZE], or TCDn_CITER[CITER] is equal to zero,
 *       or TCDn_CITER[ELINK] is not equal to TCDn_BITER[ELINK]
 */
#define DMA_ES_NCE(x) (((uint32_t)(((uint32_t)(x)) << DMA_ES_NCE_SHIFT)) & DMA_ES_NCE_MASK)

#define DMA_ES_DOE_MASK  (0x10UL)
#define DMA_ES_DOE_SHIFT (4UL)
/*! DOE - Destination Offset Error
 *  0b0..No destination offset configuration error
 *  0b1..The last recorded error was a configuration error detected in the TCDn_DOFF field. TCDn_DOFF is inconsistent
 * with TCDn_ATTR[DSIZE].
 */
#define DMA_ES_DOE(x) (((uint32_t)(((uint32_t)(x)) << DMA_ES_DOE_SHIFT)) & DMA_ES_DOE_MASK)

#define DMA_ES_DAE_MASK  (0x20UL)
#define DMA_ES_DAE_SHIFT (5UL)
/*! DAE - Destination Address Error
 *  0b0..No destination address configuration error
 *  0b1..The last recorded error was a configuration error detected in the TCDn_DADDR field. TCDn_DADDR is inconsistent
 * with TCDn_ATTR[DSIZE].
 */
#define DMA_ES_DAE(x) (((uint32_t)(((uint32_t)(x)) << DMA_ES_DAE_SHIFT)) & DMA_ES_DAE_MASK)

#define DMA_ES_SOE_MASK  (0x40UL)
#define DMA_ES_SOE_SHIFT (6UL)
/*! SOE - Source Offset Error
 *  0b0..No source offset configuration error
 *  0b1..The last recorded error was a configuration error detected in the TCDn_SOFF field. TCDn_SOFF is inconsistent
 * with TCDn_ATTR[SSIZE].
 */
#define DMA_ES_SOE(x) (((uint32_t)(((uint32_t)(x)) << DMA_ES_SOE_SHIFT)) & DMA_ES_SOE_MASK)

#define DMA_ES_SAE_MASK  (0x80UL)
#define DMA_ES_SAE_SHIFT (7UL)
/*! SAE - Source Address Error
 *  0b0..No source address configuration error.
 *  0b1..The last recorded error was a configuration error detected in the TCDn_SADDR field. TCDn_SADDR is inconsistent
 * with TCDn_ATTR[SSIZE].
 */
#define DMA_ES_SAE(x) (((uint32_t)(((uint32_t)(x)) << DMA_ES_SAE_SHIFT)) & DMA_ES_SAE_MASK)

#define DMA_ES_ERRCHN_MASK  (0x300UL)
#define DMA_ES_ERRCHN_SHIFT (8UL)
/*! ERRCHN - Error Channel Number or Canceled Channel Number
 */
#define DMA_ES_ERRCHN(x) (((uint32_t)(((uint32_t)(x)) << DMA_ES_ERRCHN_SHIFT)) & DMA_ES_ERRCHN_MASK)

#define DMA_ES_CPE_MASK  (0x4000UL)
#define DMA_ES_CPE_SHIFT (14UL)
/*! CPE - Channel Priority Error
 *  0b0..No channel priority error
 *  0b1..The last recorded error was a configuration error in the channel priorities . Channel priorities are not
 * unique.
 */
#define DMA_ES_CPE(x) (((uint32_t)(((uint32_t)(x)) << DMA_ES_CPE_SHIFT)) & DMA_ES_CPE_MASK)

#define DMA_ES_ECX_MASK  (0x10000UL)
#define DMA_ES_ECX_SHIFT (16UL)
/*! ECX - Transfer Canceled
 *  0b0..No canceled transfers
 *  0b1..The last recorded entry was a canceled transfer by the error cancel transfer input
 */
#define DMA_ES_ECX(x) (((uint32_t)(((uint32_t)(x)) << DMA_ES_ECX_SHIFT)) & DMA_ES_ECX_MASK)

#define DMA_ES_VLD_MASK  (0x80000000UL)
#define DMA_ES_VLD_SHIFT (31UL)
/*! VLD - VLD
 *  0b0..No ERR bits are set.
 *  0b1..At least one ERR bit is set indicating a valid error exists that has not been cleared.
 */
#define DMA_ES_VLD(x) (((uint32_t)(((uint32_t)(x)) << DMA_ES_VLD_SHIFT)) & DMA_ES_VLD_MASK)
/*! @} */

/*! @name ERQ - Enable Request Register */
/*! @{ */

#define DMA_ERQ_ERQ0_MASK  (0x1UL)
#define DMA_ERQ_ERQ0_SHIFT (0UL)
/*! ERQ0 - Enable DMA Request 0
 *  0b0..The DMA request signal for the corresponding channel is disabled
 *  0b1..The DMA request signal for the corresponding channel is enabled
 */
#define DMA_ERQ_ERQ0(x) (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ0_SHIFT)) & DMA_ERQ_ERQ0_MASK)

#define DMA_ERQ_ERQ1_MASK  (0x2UL)
#define DMA_ERQ_ERQ1_SHIFT (1UL)
/*! ERQ1 - Enable DMA Request 1
 *  0b0..The DMA request signal for the corresponding channel is disabled
 *  0b1..The DMA request signal for the corresponding channel is enabled
 */
#define DMA_ERQ_ERQ1(x) (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ1_SHIFT)) & DMA_ERQ_ERQ1_MASK)

#define DMA_ERQ_ERQ2_MASK  (0x4UL)
#define DMA_ERQ_ERQ2_SHIFT (2UL)
/*! ERQ2 - Enable DMA Request 2
 *  0b0..The DMA request signal for the corresponding channel is disabled
 *  0b1..The DMA request signal for the corresponding channel is enabled
 */
#define DMA_ERQ_ERQ2(x) (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ2_SHIFT)) & DMA_ERQ_ERQ2_MASK)

#define DMA_ERQ_ERQ3_MASK  (0x8UL)
#define DMA_ERQ_ERQ3_SHIFT (3UL)
/*! ERQ3 - Enable DMA Request 3
 *  0b0..The DMA request signal for the corresponding channel is disabled
 *  0b1..The DMA request signal for the corresponding channel is enabled
 */
#define DMA_ERQ_ERQ3(x) (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ3_SHIFT)) & DMA_ERQ_ERQ3_MASK)
/*! @} */

/*! @name EEI - Enable Error Interrupt Register */
/*! @{ */

#define DMA_EEI_EEI0_MASK  (0x1UL)
#define DMA_EEI_EEI0_SHIFT (0UL)
/*! EEI0 - Enable Error Interrupt 0
 *  0b0..The error signal for corresponding channel does not generate an error interrupt
 *  0b1..The assertion of the error signal for corresponding channel generates an error interrupt request
 */
#define DMA_EEI_EEI0(x) (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI0_SHIFT)) & DMA_EEI_EEI0_MASK)

#define DMA_EEI_EEI1_MASK  (0x2UL)
#define DMA_EEI_EEI1_SHIFT (1UL)
/*! EEI1 - Enable Error Interrupt 1
 *  0b0..The error signal for corresponding channel does not generate an error interrupt
 *  0b1..The assertion of the error signal for corresponding channel generates an error interrupt request
 */
#define DMA_EEI_EEI1(x) (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI1_SHIFT)) & DMA_EEI_EEI1_MASK)

#define DMA_EEI_EEI2_MASK  (0x4UL)
#define DMA_EEI_EEI2_SHIFT (2UL)
/*! EEI2 - Enable Error Interrupt 2
 *  0b0..The error signal for corresponding channel does not generate an error interrupt
 *  0b1..The assertion of the error signal for corresponding channel generates an error interrupt request
 */
#define DMA_EEI_EEI2(x) (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI2_SHIFT)) & DMA_EEI_EEI2_MASK)

#define DMA_EEI_EEI3_MASK  (0x8UL)
#define DMA_EEI_EEI3_SHIFT (3UL)
/*! EEI3 - Enable Error Interrupt 3
 *  0b0..The error signal for corresponding channel does not generate an error interrupt
 *  0b1..The assertion of the error signal for corresponding channel generates an error interrupt request
 */
#define DMA_EEI_EEI3(x) (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI3_SHIFT)) & DMA_EEI_EEI3_MASK)
/*! @} */

/*! @name CEEI - Clear Enable Error Interrupt Register */
/*! @{ */

#define DMA_CEEI_CEEI_MASK  (0x3U)
#define DMA_CEEI_CEEI_SHIFT (0U)
/*! CEEI - Clear Enable Error Interrupt
 */
#define DMA_CEEI_CEEI(x) (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_CEEI_SHIFT)) & DMA_CEEI_CEEI_MASK)

#define DMA_CEEI_CAEE_MASK  (0x40U)
#define DMA_CEEI_CAEE_SHIFT (6U)
/*! CAEE - Clear All Enable Error Interrupts
 *  0b0..Clear only the EEI bit specified in the CEEI field
 *  0b1..Clear all bits in EEI
 */
#define DMA_CEEI_CAEE(x) (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_CAEE_SHIFT)) & DMA_CEEI_CAEE_MASK)

#define DMA_CEEI_NOP_MASK  (0x80U)
#define DMA_CEEI_NOP_SHIFT (7U)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_CEEI_NOP(x) (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_NOP_SHIFT)) & DMA_CEEI_NOP_MASK)
/*! @} */

/*! @name SEEI - Set Enable Error Interrupt Register */
/*! @{ */

#define DMA_SEEI_SEEI_MASK  (0x3U)
#define DMA_SEEI_SEEI_SHIFT (0U)
/*! SEEI - Set Enable Error Interrupt
 */
#define DMA_SEEI_SEEI(x) (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_SEEI_SHIFT)) & DMA_SEEI_SEEI_MASK)

#define DMA_SEEI_SAEE_MASK  (0x40U)
#define DMA_SEEI_SAEE_SHIFT (6U)
/*! SAEE - Sets All Enable Error Interrupts
 *  0b0..Set only the EEI bit specified in the SEEI field.
 *  0b1..Sets all bits in EEI
 */
#define DMA_SEEI_SAEE(x) (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_SAEE_SHIFT)) & DMA_SEEI_SAEE_MASK)

#define DMA_SEEI_NOP_MASK  (0x80U)
#define DMA_SEEI_NOP_SHIFT (7U)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_SEEI_NOP(x) (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_NOP_SHIFT)) & DMA_SEEI_NOP_MASK)
/*! @} */

/*! @name CERQ - Clear Enable Request Register */
/*! @{ */

#define DMA_CERQ_CERQ_MASK  (0x3U)
#define DMA_CERQ_CERQ_SHIFT (0U)
/*! CERQ - Clear Enable Request
 */
#define DMA_CERQ_CERQ(x) (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_CERQ_SHIFT)) & DMA_CERQ_CERQ_MASK)

#define DMA_CERQ_CAER_MASK  (0x40U)
#define DMA_CERQ_CAER_SHIFT (6U)
/*! CAER - Clear All Enable Requests
 *  0b0..Clear only the ERQ bit specified in the CERQ field
 *  0b1..Clear all bits in ERQ
 */
#define DMA_CERQ_CAER(x) (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_CAER_SHIFT)) & DMA_CERQ_CAER_MASK)

#define DMA_CERQ_NOP_MASK  (0x80U)
#define DMA_CERQ_NOP_SHIFT (7U)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_CERQ_NOP(x) (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_NOP_SHIFT)) & DMA_CERQ_NOP_MASK)
/*! @} */

/*! @name SERQ - Set Enable Request Register */
/*! @{ */

#define DMA_SERQ_SERQ_MASK  (0x3U)
#define DMA_SERQ_SERQ_SHIFT (0U)
/*! SERQ - Set Enable Request
 */
#define DMA_SERQ_SERQ(x) (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_SERQ_SHIFT)) & DMA_SERQ_SERQ_MASK)

#define DMA_SERQ_SAER_MASK  (0x40U)
#define DMA_SERQ_SAER_SHIFT (6U)
/*! SAER - Set All Enable Requests
 *  0b0..Set only the ERQ bit specified in the SERQ field
 *  0b1..Set all bits in ERQ
 */
#define DMA_SERQ_SAER(x) (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_SAER_SHIFT)) & DMA_SERQ_SAER_MASK)

#define DMA_SERQ_NOP_MASK  (0x80U)
#define DMA_SERQ_NOP_SHIFT (7U)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_SERQ_NOP(x) (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_NOP_SHIFT)) & DMA_SERQ_NOP_MASK)
/*! @} */

/*! @name CDNE - Clear DONE Status Bit Register */
/*! @{ */

#define DMA_CDNE_CDNE_MASK  (0x3U)
#define DMA_CDNE_CDNE_SHIFT (0U)
/*! CDNE - Clear DONE Bit
 */
#define DMA_CDNE_CDNE(x) (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_CDNE_SHIFT)) & DMA_CDNE_CDNE_MASK)

#define DMA_CDNE_CADN_MASK  (0x40U)
#define DMA_CDNE_CADN_SHIFT (6U)
/*! CADN - Clears All DONE Bits
 *  0b0..Clears only the TCDn_CSR[DONE] bit specified in the CDNE field
 *  0b1..Clears all bits in TCDn_CSR[DONE]
 */
#define DMA_CDNE_CADN(x) (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_CADN_SHIFT)) & DMA_CDNE_CADN_MASK)

#define DMA_CDNE_NOP_MASK  (0x80U)
#define DMA_CDNE_NOP_SHIFT (7U)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_CDNE_NOP(x) (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_NOP_SHIFT)) & DMA_CDNE_NOP_MASK)
/*! @} */

/*! @name SSRT - Set START Bit Register */
/*! @{ */

#define DMA_SSRT_SSRT_MASK  (0x3U)
#define DMA_SSRT_SSRT_SHIFT (0U)
/*! SSRT - Set START Bit
 */
#define DMA_SSRT_SSRT(x) (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_SSRT_SHIFT)) & DMA_SSRT_SSRT_MASK)

#define DMA_SSRT_SAST_MASK  (0x40U)
#define DMA_SSRT_SAST_SHIFT (6U)
/*! SAST - Set All START Bits (activates all channels)
 *  0b0..Set only the TCDn_CSR[START] bit specified in the SSRT field
 *  0b1..Set all bits in TCDn_CSR[START]
 */
#define DMA_SSRT_SAST(x) (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_SAST_SHIFT)) & DMA_SSRT_SAST_MASK)

#define DMA_SSRT_NOP_MASK  (0x80U)
#define DMA_SSRT_NOP_SHIFT (7U)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_SSRT_NOP(x) (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_NOP_SHIFT)) & DMA_SSRT_NOP_MASK)
/*! @} */

/*! @name CERR - Clear Error Register */
/*! @{ */

#define DMA_CERR_CERR_MASK  (0x3U)
#define DMA_CERR_CERR_SHIFT (0U)
/*! CERR - Clear Error Indicator
 */
#define DMA_CERR_CERR(x) (((uint8_t)(((uint8_t)(x)) << DMA_CERR_CERR_SHIFT)) & DMA_CERR_CERR_MASK)

#define DMA_CERR_CAEI_MASK  (0x40U)
#define DMA_CERR_CAEI_SHIFT (6U)
/*! CAEI - Clear All Error Indicators
 *  0b0..Clear only the ERR bit specified in the CERR field
 *  0b1..Clear all bits in ERR
 */
#define DMA_CERR_CAEI(x) (((uint8_t)(((uint8_t)(x)) << DMA_CERR_CAEI_SHIFT)) & DMA_CERR_CAEI_MASK)

#define DMA_CERR_NOP_MASK  (0x80U)
#define DMA_CERR_NOP_SHIFT (7U)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_CERR_NOP(x) (((uint8_t)(((uint8_t)(x)) << DMA_CERR_NOP_SHIFT)) & DMA_CERR_NOP_MASK)
/*! @} */

/*! @name CINT - Clear Interrupt Request Register */
/*! @{ */

#define DMA_CINT_CINT_MASK  (0x3U)
#define DMA_CINT_CINT_SHIFT (0U)
/*! CINT - Clear Interrupt Request
 */
#define DMA_CINT_CINT(x) (((uint8_t)(((uint8_t)(x)) << DMA_CINT_CINT_SHIFT)) & DMA_CINT_CINT_MASK)

#define DMA_CINT_CAIR_MASK  (0x40U)
#define DMA_CINT_CAIR_SHIFT (6U)
/*! CAIR - Clear All Interrupt Requests
 *  0b0..Clear only the INT bit specified in the CINT field
 *  0b1..Clear all bits in INT
 */
#define DMA_CINT_CAIR(x) (((uint8_t)(((uint8_t)(x)) << DMA_CINT_CAIR_SHIFT)) & DMA_CINT_CAIR_MASK)

#define DMA_CINT_NOP_MASK  (0x80U)
#define DMA_CINT_NOP_SHIFT (7U)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_CINT_NOP(x) (((uint8_t)(((uint8_t)(x)) << DMA_CINT_NOP_SHIFT)) & DMA_CINT_NOP_MASK)
/*! @} */

/*! @name INT - Interrupt Request Register */
/*! @{ */

#define DMA_INT_INT0_MASK  (0x1UL)
#define DMA_INT_INT0_SHIFT (0UL)
/*! INT0 - Interrupt Request 0
 *  0b0..The interrupt request for corresponding channel is cleared
 *  0b1..The interrupt request for corresponding channel is active
 */
#define DMA_INT_INT0(x) (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT0_SHIFT)) & DMA_INT_INT0_MASK)

#define DMA_INT_INT1_MASK  (0x2UL)
#define DMA_INT_INT1_SHIFT (1UL)
/*! INT1 - Interrupt Request 1
 *  0b0..The interrupt request for corresponding channel is cleared
 *  0b1..The interrupt request for corresponding channel is active
 */
#define DMA_INT_INT1(x) (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT1_SHIFT)) & DMA_INT_INT1_MASK)

#define DMA_INT_INT2_MASK  (0x4UL)
#define DMA_INT_INT2_SHIFT (2UL)
/*! INT2 - Interrupt Request 2
 *  0b0..The interrupt request for corresponding channel is cleared
 *  0b1..The interrupt request for corresponding channel is active
 */
#define DMA_INT_INT2(x) (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT2_SHIFT)) & DMA_INT_INT2_MASK)

#define DMA_INT_INT3_MASK  (0x8UL)
#define DMA_INT_INT3_SHIFT (3UL)
/*! INT3 - Interrupt Request 3
 *  0b0..The interrupt request for corresponding channel is cleared
 *  0b1..The interrupt request for corresponding channel is active
 */
#define DMA_INT_INT3(x) (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT3_SHIFT)) & DMA_INT_INT3_MASK)
/*! @} */

/*! @name ERR - Error Register */
/*! @{ */

#define DMA_ERR_ERR0_MASK  (0x1UL)
#define DMA_ERR_ERR0_SHIFT (0UL)
/*! ERR0 - Error In Channel 0
 *  0b0..An error in this channel has not occurred
 *  0b1..An error in this channel has occurred
 */
#define DMA_ERR_ERR0(x) (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR0_SHIFT)) & DMA_ERR_ERR0_MASK)

#define DMA_ERR_ERR1_MASK  (0x2UL)
#define DMA_ERR_ERR1_SHIFT (1UL)
/*! ERR1 - Error In Channel 1
 *  0b0..An error in this channel has not occurred
 *  0b1..An error in this channel has occurred
 */
#define DMA_ERR_ERR1(x) (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR1_SHIFT)) & DMA_ERR_ERR1_MASK)

#define DMA_ERR_ERR2_MASK  (0x4UL)
#define DMA_ERR_ERR2_SHIFT (2UL)
/*! ERR2 - Error In Channel 2
 *  0b0..An error in this channel has not occurred
 *  0b1..An error in this channel has occurred
 */
#define DMA_ERR_ERR2(x) (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR2_SHIFT)) & DMA_ERR_ERR2_MASK)

#define DMA_ERR_ERR3_MASK  (0x8UL)
#define DMA_ERR_ERR3_SHIFT (3UL)
/*! ERR3 - Error In Channel 3
 *  0b0..An error in this channel has not occurred
 *  0b1..An error in this channel has occurred
 */
#define DMA_ERR_ERR3(x) (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR3_SHIFT)) & DMA_ERR_ERR3_MASK)
/*! @} */

/*! @name HRS - Hardware Request Status Register */
/*! @{ */

#define DMA_HRS_HRS0_MASK  (0x1UL)
#define DMA_HRS_HRS0_SHIFT (0UL)
/*! HRS0 - Hardware Request Status Channel 0
 *  0b0..A hardware service request for channel 0 is not present
 *  0b1..A hardware service request for channel 0 is present
 */
#define DMA_HRS_HRS0(x) (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS0_SHIFT)) & DMA_HRS_HRS0_MASK)

#define DMA_HRS_HRS1_MASK  (0x2UL)
#define DMA_HRS_HRS1_SHIFT (1UL)
/*! HRS1 - Hardware Request Status Channel 1
 *  0b0..A hardware service request for channel 1 is not present
 *  0b1..A hardware service request for channel 1 is present
 */
#define DMA_HRS_HRS1(x) (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS1_SHIFT)) & DMA_HRS_HRS1_MASK)

#define DMA_HRS_HRS2_MASK  (0x4UL)
#define DMA_HRS_HRS2_SHIFT (2UL)
/*! HRS2 - Hardware Request Status Channel 2
 *  0b0..A hardware service request for channel 2 is not present
 *  0b1..A hardware service request for channel 2 is present
 */
#define DMA_HRS_HRS2(x) (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS2_SHIFT)) & DMA_HRS_HRS2_MASK)

#define DMA_HRS_HRS3_MASK  (0x8UL)
#define DMA_HRS_HRS3_SHIFT (3UL)
/*! HRS3 - Hardware Request Status Channel 3
 *  0b0..A hardware service request for channel 3 is not present
 *  0b1..A hardware service request for channel 3 is present
 */
#define DMA_HRS_HRS3(x) (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS3_SHIFT)) & DMA_HRS_HRS3_MASK)
/*! @} */

/*! @name EARS - Enable Asynchronous Request in Stop Register */
/*! @{ */

#define DMA_EARS_EDREQ_0_MASK  (0x1UL)
#define DMA_EARS_EDREQ_0_SHIFT (0UL)
/*! EDREQ_0 - Enable asynchronous DMA request in stop mode for channel 0.
 *  0b0..Disable asynchronous DMA request for channel 0.
 *  0b1..Enable asynchronous DMA request for channel 0.
 */
#define DMA_EARS_EDREQ_0(x) (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_0_SHIFT)) & DMA_EARS_EDREQ_0_MASK)

#define DMA_EARS_EDREQ_1_MASK  (0x2UL)
#define DMA_EARS_EDREQ_1_SHIFT (1UL)
/*! EDREQ_1 - Enable asynchronous DMA request in stop mode for channel 1.
 *  0b0..Disable asynchronous DMA request for channel 1
 *  0b1..Enable asynchronous DMA request for channel 1.
 */
#define DMA_EARS_EDREQ_1(x) (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_1_SHIFT)) & DMA_EARS_EDREQ_1_MASK)

#define DMA_EARS_EDREQ_2_MASK  (0x4UL)
#define DMA_EARS_EDREQ_2_SHIFT (2UL)
/*! EDREQ_2 - Enable asynchronous DMA request in stop mode for channel 2.
 *  0b0..Disable asynchronous DMA request for channel 2.
 *  0b1..Enable asynchronous DMA request for channel 2.
 */
#define DMA_EARS_EDREQ_2(x) (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_2_SHIFT)) & DMA_EARS_EDREQ_2_MASK)

#define DMA_EARS_EDREQ_3_MASK  (0x8UL)
#define DMA_EARS_EDREQ_3_SHIFT (3UL)
/*! EDREQ_3 - Enable asynchronous DMA request in stop mode for channel 3.
 *  0b0..Disable asynchronous DMA request for channel 3.
 *  0b1..Enable asynchronous DMA request for channel 3.
 */
#define DMA_EARS_EDREQ_3(x) (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_3_SHIFT)) & DMA_EARS_EDREQ_3_MASK)
/*! @} */

/*! @name DCHPRI3 - Channel Priority Register */
/*! @{ */

#define DMA_DCHPRI3_CHPRI_MASK  (0x3U)
#define DMA_DCHPRI3_CHPRI_SHIFT (0U)
/*! CHPRI - Channel n Arbitration Priority
 */
#define DMA_DCHPRI3_CHPRI(x) (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI3_CHPRI_SHIFT)) & DMA_DCHPRI3_CHPRI_MASK)

#define DMA_DCHPRI3_DPA_MASK  (0x40U)
#define DMA_DCHPRI3_DPA_SHIFT (6U)
/*! DPA - Disable Preempt Ability. This field resets to 0.
 *  0b0..Channel n can suspend a lower priority channel.
 *  0b1..Channel n cannot suspend any channel, regardless of channel priority.
 */
#define DMA_DCHPRI3_DPA(x) (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI3_DPA_SHIFT)) & DMA_DCHPRI3_DPA_MASK)

#define DMA_DCHPRI3_ECP_MASK  (0x80U)
#define DMA_DCHPRI3_ECP_SHIFT (7U)
/*! ECP - Enable Channel Preemption. This field resets to 0.
 *  0b0..Channel n cannot be suspended by a higher priority channel's service request.
 *  0b1..Channel n can be temporarily suspended by the service request of a higher priority channel.
 */
#define DMA_DCHPRI3_ECP(x) (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI3_ECP_SHIFT)) & DMA_DCHPRI3_ECP_MASK)
/*! @} */

/*! @name DCHPRI2 - Channel Priority Register */
/*! @{ */

#define DMA_DCHPRI2_CHPRI_MASK  (0x3U)
#define DMA_DCHPRI2_CHPRI_SHIFT (0U)
/*! CHPRI - Channel n Arbitration Priority
 */
#define DMA_DCHPRI2_CHPRI(x) (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI2_CHPRI_SHIFT)) & DMA_DCHPRI2_CHPRI_MASK)

#define DMA_DCHPRI2_DPA_MASK  (0x40U)
#define DMA_DCHPRI2_DPA_SHIFT (6U)
/*! DPA - Disable Preempt Ability. This field resets to 0.
 *  0b0..Channel n can suspend a lower priority channel.
 *  0b1..Channel n cannot suspend any channel, regardless of channel priority.
 */
#define DMA_DCHPRI2_DPA(x) (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI2_DPA_SHIFT)) & DMA_DCHPRI2_DPA_MASK)

#define DMA_DCHPRI2_ECP_MASK  (0x80U)
#define DMA_DCHPRI2_ECP_SHIFT (7U)
/*! ECP - Enable Channel Preemption. This field resets to 0.
 *  0b0..Channel n cannot be suspended by a higher priority channel's service request.
 *  0b1..Channel n can be temporarily suspended by the service request of a higher priority channel.
 */
#define DMA_DCHPRI2_ECP(x) (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI2_ECP_SHIFT)) & DMA_DCHPRI2_ECP_MASK)
/*! @} */

/*! @name DCHPRI1 - Channel Priority Register */
/*! @{ */

#define DMA_DCHPRI1_CHPRI_MASK  (0x3U)
#define DMA_DCHPRI1_CHPRI_SHIFT (0U)
/*! CHPRI - Channel n Arbitration Priority
 */
#define DMA_DCHPRI1_CHPRI(x) (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI1_CHPRI_SHIFT)) & DMA_DCHPRI1_CHPRI_MASK)

#define DMA_DCHPRI1_DPA_MASK  (0x40U)
#define DMA_DCHPRI1_DPA_SHIFT (6U)
/*! DPA - Disable Preempt Ability. This field resets to 0.
 *  0b0..Channel n can suspend a lower priority channel.
 *  0b1..Channel n cannot suspend any channel, regardless of channel priority.
 */
#define DMA_DCHPRI1_DPA(x) (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI1_DPA_SHIFT)) & DMA_DCHPRI1_DPA_MASK)

#define DMA_DCHPRI1_ECP_MASK  (0x80U)
#define DMA_DCHPRI1_ECP_SHIFT (7U)
/*! ECP - Enable Channel Preemption. This field resets to 0.
 *  0b0..Channel n cannot be suspended by a higher priority channel's service request.
 *  0b1..Channel n can be temporarily suspended by the service request of a higher priority channel.
 */
#define DMA_DCHPRI1_ECP(x) (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI1_ECP_SHIFT)) & DMA_DCHPRI1_ECP_MASK)
/*! @} */

/*! @name DCHPRI0 - Channel Priority Register */
/*! @{ */

#define DMA_DCHPRI0_CHPRI_MASK  (0x3U)
#define DMA_DCHPRI0_CHPRI_SHIFT (0U)
/*! CHPRI - Channel n Arbitration Priority
 */
#define DMA_DCHPRI0_CHPRI(x) (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI0_CHPRI_SHIFT)) & DMA_DCHPRI0_CHPRI_MASK)

#define DMA_DCHPRI0_DPA_MASK  (0x40U)
#define DMA_DCHPRI0_DPA_SHIFT (6U)
/*! DPA - Disable Preempt Ability. This field resets to 0.
 *  0b0..Channel n can suspend a lower priority channel.
 *  0b1..Channel n cannot suspend any channel, regardless of channel priority.
 */
#define DMA_DCHPRI0_DPA(x) (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI0_DPA_SHIFT)) & DMA_DCHPRI0_DPA_MASK)

#define DMA_DCHPRI0_ECP_MASK  (0x80U)
#define DMA_DCHPRI0_ECP_SHIFT (7U)
/*! ECP - Enable Channel Preemption. This field resets to 0.
 *  0b0..Channel n cannot be suspended by a higher priority channel's service request.
 *  0b1..Channel n can be temporarily suspended by the service request of a higher priority channel.
 */
#define DMA_DCHPRI0_ECP(x) (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI0_ECP_SHIFT)) & DMA_DCHPRI0_ECP_MASK)
/*! @} */

/*! @name SADDR - TCD Source Address */
/*! @{ */

#define DMA_SADDR_SADDR_MASK  (0xFFFFFFFFUL)
#define DMA_SADDR_SADDR_SHIFT (0UL)
/*! SADDR - Source Address
 */
#define DMA_SADDR_SADDR(x) (((uint32_t)(((uint32_t)(x)) << DMA_SADDR_SADDR_SHIFT)) & DMA_SADDR_SADDR_MASK)
/*! @} */

/* The count of DMA_SADDR */
#define DMA_SADDR_COUNT (4U)

/*! @name SOFF - TCD Signed Source Address Offset */
/*! @{ */

#define DMA_SOFF_SOFF_MASK  (0xFFFFU)
#define DMA_SOFF_SOFF_SHIFT (0U)
/*! SOFF - Source address signed offset
 */
#define DMA_SOFF_SOFF(x) (((uint16_t)(((uint16_t)(x)) << DMA_SOFF_SOFF_SHIFT)) & DMA_SOFF_SOFF_MASK)
/*! @} */

/* The count of DMA_SOFF */
#define DMA_SOFF_COUNT (4U)

/*! @name ATTR - TCD Transfer Attributes */
/*! @{ */

#define DMA_ATTR_DSIZE_MASK  (0x7U)
#define DMA_ATTR_DSIZE_SHIFT (0U)
/*! DSIZE - Destination data transfer size
 */
#define DMA_ATTR_DSIZE(x) (((uint16_t)(((uint16_t)(x)) << DMA_ATTR_DSIZE_SHIFT)) & DMA_ATTR_DSIZE_MASK)

#define DMA_ATTR_DMOD_MASK  (0xF8U)
#define DMA_ATTR_DMOD_SHIFT (3U)
/*! DMOD - Destination Address Modulo
 */
#define DMA_ATTR_DMOD(x) (((uint16_t)(((uint16_t)(x)) << DMA_ATTR_DMOD_SHIFT)) & DMA_ATTR_DMOD_MASK)

#define DMA_ATTR_SSIZE_MASK  (0x700U)
#define DMA_ATTR_SSIZE_SHIFT (8U)
/*! SSIZE - Source data transfer size
 *  0b000..8-bit
 *  0b001..16-bit
 *  0b010..32-bit
 *  0b011..Reserved
 *  0b100..16-byte
 *  0b101..Reserved
 *  0b110..Reserved
 *  0b111..Reserved
 */
#define DMA_ATTR_SSIZE(x) (((uint16_t)(((uint16_t)(x)) << DMA_ATTR_SSIZE_SHIFT)) & DMA_ATTR_SSIZE_MASK)

#define DMA_ATTR_SMOD_MASK  (0xF800U)
#define DMA_ATTR_SMOD_SHIFT (11U)
/*! SMOD - Source Address Modulo
 *  0b00000..Source address modulo feature is disabled
 *  0b00001-0b11111..This value defines a specific address range specified to be the value after SADDR + SOFF
 *                   calculation is performed on the original register value. Setting this field provides the ability
 *                   to implement a circular data queue easily. For data queues requiring power-of-2 size bytes, the
 *                   queue should start at a 0-modulo-size address and the SMOD field should be set to the
 *                   appropriate value for the queue, freezing the desired number of upper address bits. The value
 *                   programmed into this field specifies the number of lower address bits allowed to change. For a
 *                   circular queue application, the SOFF is typically set to the transfer size to implement
 *                   post-increment addressing with the SMOD function constraining the addresses to a 0-modulo-size
 * range.
 */
#define DMA_ATTR_SMOD(x) (((uint16_t)(((uint16_t)(x)) << DMA_ATTR_SMOD_SHIFT)) & DMA_ATTR_SMOD_MASK)
/*! @} */

/* The count of DMA_ATTR */
#define DMA_ATTR_COUNT (4U)

/*! @name NBYTES_MLNO - TCD Minor Byte Count (Minor Loop Mapping Disabled) */
/*! @{ */

#define DMA_NBYTES_MLNO_NBYTES_MASK  (0xFFFFFFFFUL)
#define DMA_NBYTES_MLNO_NBYTES_SHIFT (0UL)
/*! NBYTES - Minor Byte Transfer Count
 */
#define DMA_NBYTES_MLNO_NBYTES(x) \
    (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLNO_NBYTES_SHIFT)) & DMA_NBYTES_MLNO_NBYTES_MASK)
/*! @} */

/* The count of DMA_NBYTES_MLNO */
#define DMA_NBYTES_MLNO_COUNT (4U)

/*! @name NBYTES_MLOFFNO - TCD Signed Minor Loop Offset (Minor Loop Mapping Enabled and Offset Disabled) */
/*! @{ */

#define DMA_NBYTES_MLOFFNO_NBYTES_MASK  (0x3FFFFFFFUL)
#define DMA_NBYTES_MLOFFNO_NBYTES_SHIFT (0UL)
/*! NBYTES - Minor Byte Transfer Count
 */
#define DMA_NBYTES_MLOFFNO_NBYTES(x) \
    (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFNO_NBYTES_SHIFT)) & DMA_NBYTES_MLOFFNO_NBYTES_MASK)

#define DMA_NBYTES_MLOFFNO_DMLOE_MASK  (0x40000000UL)
#define DMA_NBYTES_MLOFFNO_DMLOE_SHIFT (30UL)
/*! DMLOE - Destination Minor Loop Offset enable
 *  0b0..The minor loop offset is not applied to the DADDR
 *  0b1..The minor loop offset is applied to the DADDR
 */
#define DMA_NBYTES_MLOFFNO_DMLOE(x) \
    (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFNO_DMLOE_SHIFT)) & DMA_NBYTES_MLOFFNO_DMLOE_MASK)

#define DMA_NBYTES_MLOFFNO_SMLOE_MASK  (0x80000000UL)
#define DMA_NBYTES_MLOFFNO_SMLOE_SHIFT (31UL)
/*! SMLOE - Source Minor Loop Offset Enable
 *  0b0..The minor loop offset is not applied to the SADDR
 *  0b1..The minor loop offset is applied to the SADDR
 */
#define DMA_NBYTES_MLOFFNO_SMLOE(x) \
    (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFNO_SMLOE_SHIFT)) & DMA_NBYTES_MLOFFNO_SMLOE_MASK)
/*! @} */

/* The count of DMA_NBYTES_MLOFFNO */
#define DMA_NBYTES_MLOFFNO_COUNT (4U)

/*! @name NBYTES_MLOFFYES - TCD Signed Minor Loop Offset (Minor Loop Mapping and Offset Enabled) */
/*! @{ */

#define DMA_NBYTES_MLOFFYES_NBYTES_MASK  (0x3FFUL)
#define DMA_NBYTES_MLOFFYES_NBYTES_SHIFT (0UL)
/*! NBYTES - Minor Byte Transfer Count
 */
#define DMA_NBYTES_MLOFFYES_NBYTES(x) \
    (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFYES_NBYTES_SHIFT)) & DMA_NBYTES_MLOFFYES_NBYTES_MASK)

#define DMA_NBYTES_MLOFFYES_MLOFF_MASK  (0x3FFFFC00UL)
#define DMA_NBYTES_MLOFFYES_MLOFF_SHIFT (10UL)
/*! MLOFF - If SMLOE or DMLOE is set, this field represents a sign-extended offset applied to the
 *    source or destination address to form the next-state value after the minor loop completes.
 */
#define DMA_NBYTES_MLOFFYES_MLOFF(x) \
    (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFYES_MLOFF_SHIFT)) & DMA_NBYTES_MLOFFYES_MLOFF_MASK)

#define DMA_NBYTES_MLOFFYES_DMLOE_MASK  (0x40000000UL)
#define DMA_NBYTES_MLOFFYES_DMLOE_SHIFT (30UL)
/*! DMLOE - Destination Minor Loop Offset enable
 *  0b0..The minor loop offset is not applied to the DADDR
 *  0b1..The minor loop offset is applied to the DADDR
 */
#define DMA_NBYTES_MLOFFYES_DMLOE(x) \
    (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFYES_DMLOE_SHIFT)) & DMA_NBYTES_MLOFFYES_DMLOE_MASK)

#define DMA_NBYTES_MLOFFYES_SMLOE_MASK  (0x80000000UL)
#define DMA_NBYTES_MLOFFYES_SMLOE_SHIFT (31UL)
/*! SMLOE - Source Minor Loop Offset Enable
 *  0b0..The minor loop offset is not applied to the SADDR
 *  0b1..The minor loop offset is applied to the SADDR
 */
#define DMA_NBYTES_MLOFFYES_SMLOE(x) \
    (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFYES_SMLOE_SHIFT)) & DMA_NBYTES_MLOFFYES_SMLOE_MASK)
/*! @} */

/* The count of DMA_NBYTES_MLOFFYES */
#define DMA_NBYTES_MLOFFYES_COUNT (4U)

/*! @name SLAST - TCD Last Source Address Adjustment */
/*! @{ */

#define DMA_SLAST_SLAST_MASK  (0xFFFFFFFFUL)
#define DMA_SLAST_SLAST_SHIFT (0UL)
/*! SLAST - Last Source Address Adjustment
 */
#define DMA_SLAST_SLAST(x) (((uint32_t)(((uint32_t)(x)) << DMA_SLAST_SLAST_SHIFT)) & DMA_SLAST_SLAST_MASK)
/*! @} */

/* The count of DMA_SLAST */
#define DMA_SLAST_COUNT (4U)

/*! @name DADDR - TCD Destination Address */
/*! @{ */

#define DMA_DADDR_DADDR_MASK  (0xFFFFFFFFUL)
#define DMA_DADDR_DADDR_SHIFT (0UL)
/*! DADDR - Destination Address
 */
#define DMA_DADDR_DADDR(x) (((uint32_t)(((uint32_t)(x)) << DMA_DADDR_DADDR_SHIFT)) & DMA_DADDR_DADDR_MASK)
/*! @} */

/* The count of DMA_DADDR */
#define DMA_DADDR_COUNT (4U)

/*! @name DOFF - TCD Signed Destination Address Offset */
/*! @{ */

#define DMA_DOFF_DOFF_MASK  (0xFFFFU)
#define DMA_DOFF_DOFF_SHIFT (0U)
/*! DOFF - Destination Address Signed Offset
 */
#define DMA_DOFF_DOFF(x) (((uint16_t)(((uint16_t)(x)) << DMA_DOFF_DOFF_SHIFT)) & DMA_DOFF_DOFF_MASK)
/*! @} */

/* The count of DMA_DOFF */
#define DMA_DOFF_COUNT (4U)

/*! @name CITER_ELINKNO - TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled) */
/*! @{ */

#define DMA_CITER_ELINKNO_CITER_MASK  (0x7FFFU)
#define DMA_CITER_ELINKNO_CITER_SHIFT (0U)
/*! CITER - Current Major Iteration Count
 */
#define DMA_CITER_ELINKNO_CITER(x) \
    (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKNO_CITER_SHIFT)) & DMA_CITER_ELINKNO_CITER_MASK)

#define DMA_CITER_ELINKNO_ELINK_MASK  (0x8000U)
#define DMA_CITER_ELINKNO_ELINK_SHIFT (15U)
/*! ELINK - Enable channel-to-channel linking on minor-loop complete
 *  0b0..The channel-to-channel linking is disabled
 *  0b1..The channel-to-channel linking is enabled
 */
#define DMA_CITER_ELINKNO_ELINK(x) \
    (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKNO_ELINK_SHIFT)) & DMA_CITER_ELINKNO_ELINK_MASK)
/*! @} */

/* The count of DMA_CITER_ELINKNO */
#define DMA_CITER_ELINKNO_COUNT (4U)

/*! @name CITER_ELINKYES - TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled) */
/*! @{ */

#define DMA_CITER_ELINKYES_CITER_MASK  (0x1FFU)
#define DMA_CITER_ELINKYES_CITER_SHIFT (0U)
/*! CITER - Current Major Iteration Count
 */
#define DMA_CITER_ELINKYES_CITER(x) \
    (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKYES_CITER_SHIFT)) & DMA_CITER_ELINKYES_CITER_MASK)

#define DMA_CITER_ELINKYES_LINKCH_MASK  (0x600U)
#define DMA_CITER_ELINKYES_LINKCH_SHIFT (9U)
/*! LINKCH - Minor Loop Link Channel Number
 */
#define DMA_CITER_ELINKYES_LINKCH(x) \
    (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKYES_LINKCH_SHIFT)) & DMA_CITER_ELINKYES_LINKCH_MASK)

#define DMA_CITER_ELINKYES_ELINK_MASK  (0x8000U)
#define DMA_CITER_ELINKYES_ELINK_SHIFT (15U)
/*! ELINK - Enable channel-to-channel linking on minor-loop complete
 *  0b0..The channel-to-channel linking is disabled
 *  0b1..The channel-to-channel linking is enabled
 */
#define DMA_CITER_ELINKYES_ELINK(x) \
    (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKYES_ELINK_SHIFT)) & DMA_CITER_ELINKYES_ELINK_MASK)
/*! @} */

/* The count of DMA_CITER_ELINKYES */
#define DMA_CITER_ELINKYES_COUNT (4U)

/*! @name DLAST_SGA - TCD Last Destination Address Adjustment/Scatter Gather Address */
/*! @{ */

#define DMA_DLAST_SGA_DLASTSGA_MASK  (0xFFFFFFFFUL)
#define DMA_DLAST_SGA_DLASTSGA_SHIFT (0UL)
/*! DLASTSGA - DLASTSGA
 */
#define DMA_DLAST_SGA_DLASTSGA(x) \
    (((uint32_t)(((uint32_t)(x)) << DMA_DLAST_SGA_DLASTSGA_SHIFT)) & DMA_DLAST_SGA_DLASTSGA_MASK)
/*! @} */

/* The count of DMA_DLAST_SGA */
#define DMA_DLAST_SGA_COUNT (4U)

/*! @name CSR - TCD Control and Status */
/*! @{ */

#define DMA_CSR_START_MASK  (0x1U)
#define DMA_CSR_START_SHIFT (0U)
/*! START - Channel Start
 *  0b0..The channel is not explicitly started.
 *  0b1..The channel is explicitly started via a software initiated service request.
 */
#define DMA_CSR_START(x) (((uint16_t)(((uint16_t)(x)) << DMA_CSR_START_SHIFT)) & DMA_CSR_START_MASK)

#define DMA_CSR_INTMAJOR_MASK  (0x2U)
#define DMA_CSR_INTMAJOR_SHIFT (1U)
/*! INTMAJOR - Enable an interrupt when major iteration count completes.
 *  0b0..The end-of-major loop interrupt is disabled.
 *  0b1..The end-of-major loop interrupt is enabled.
 */
#define DMA_CSR_INTMAJOR(x) (((uint16_t)(((uint16_t)(x)) << DMA_CSR_INTMAJOR_SHIFT)) & DMA_CSR_INTMAJOR_MASK)

#define DMA_CSR_INTHALF_MASK  (0x4U)
#define DMA_CSR_INTHALF_SHIFT (2U)
/*! INTHALF - Enable an interrupt when major counter is half complete.
 *  0b0..The half-point interrupt is disabled.
 *  0b1..The half-point interrupt is enabled.
 */
#define DMA_CSR_INTHALF(x) (((uint16_t)(((uint16_t)(x)) << DMA_CSR_INTHALF_SHIFT)) & DMA_CSR_INTHALF_MASK)

#define DMA_CSR_DREQ_MASK  (0x8U)
#define DMA_CSR_DREQ_SHIFT (3U)
/*! DREQ - Disable Request
 *  0b0..The channel's ERQ bit is not affected.
 *  0b1..The channel's ERQ bit is cleared when the major loop is complete.
 */
#define DMA_CSR_DREQ(x) (((uint16_t)(((uint16_t)(x)) << DMA_CSR_DREQ_SHIFT)) & DMA_CSR_DREQ_MASK)

#define DMA_CSR_ESG_MASK  (0x10U)
#define DMA_CSR_ESG_SHIFT (4U)
/*! ESG - Enable Scatter/Gather Processing
 *  0b0..The current channel's TCD is normal format.
 *  0b1..The current channel's TCD specifies a scatter gather format. The DLASTSGA field provides a memory pointer
 *       to the next TCD to be loaded into this channel after the major loop completes its execution.
 */
#define DMA_CSR_ESG(x) (((uint16_t)(((uint16_t)(x)) << DMA_CSR_ESG_SHIFT)) & DMA_CSR_ESG_MASK)

#define DMA_CSR_MAJORELINK_MASK  (0x20U)
#define DMA_CSR_MAJORELINK_SHIFT (5U)
/*! MAJORELINK - Enable channel-to-channel linking on major loop complete
 *  0b0..The channel-to-channel linking is disabled.
 *  0b1..The channel-to-channel linking is enabled.
 */
#define DMA_CSR_MAJORELINK(x) (((uint16_t)(((uint16_t)(x)) << DMA_CSR_MAJORELINK_SHIFT)) & DMA_CSR_MAJORELINK_MASK)

#define DMA_CSR_ACTIVE_MASK  (0x40U)
#define DMA_CSR_ACTIVE_SHIFT (6U)
/*! ACTIVE - Channel Active
 */
#define DMA_CSR_ACTIVE(x) (((uint16_t)(((uint16_t)(x)) << DMA_CSR_ACTIVE_SHIFT)) & DMA_CSR_ACTIVE_MASK)

#define DMA_CSR_DONE_MASK  (0x80U)
#define DMA_CSR_DONE_SHIFT (7U)
/*! DONE - Channel Done
 */
#define DMA_CSR_DONE(x) (((uint16_t)(((uint16_t)(x)) << DMA_CSR_DONE_SHIFT)) & DMA_CSR_DONE_MASK)

#define DMA_CSR_MAJORLINKCH_MASK  (0x300U)
#define DMA_CSR_MAJORLINKCH_SHIFT (8U)
/*! MAJORLINKCH - Major Loop Link Channel Number
 */
#define DMA_CSR_MAJORLINKCH(x) (((uint16_t)(((uint16_t)(x)) << DMA_CSR_MAJORLINKCH_SHIFT)) & DMA_CSR_MAJORLINKCH_MASK)

#define DMA_CSR_BWC_MASK  (0xC000U)
#define DMA_CSR_BWC_SHIFT (14U)
/*! BWC - Bandwidth Control
 *  0b00..No eDMA engine stalls.
 *  0b01..Reserved
 *  0b10..eDMA engine stalls for 4 cycles after each R/W.
 *  0b11..eDMA engine stalls for 8 cycles after each R/W.
 */
#define DMA_CSR_BWC(x) (((uint16_t)(((uint16_t)(x)) << DMA_CSR_BWC_SHIFT)) & DMA_CSR_BWC_MASK)
/*! @} */

/* The count of DMA_CSR */
#define DMA_CSR_COUNT (4U)

/*! @name BITER_ELINKNO - TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled) */
/*! @{ */

#define DMA_BITER_ELINKNO_BITER_MASK  (0x7FFFU)
#define DMA_BITER_ELINKNO_BITER_SHIFT (0U)
/*! BITER - Starting Major Iteration Count
 */
#define DMA_BITER_ELINKNO_BITER(x) \
    (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKNO_BITER_SHIFT)) & DMA_BITER_ELINKNO_BITER_MASK)

#define DMA_BITER_ELINKNO_ELINK_MASK  (0x8000U)
#define DMA_BITER_ELINKNO_ELINK_SHIFT (15U)
/*! ELINK - Enables channel-to-channel linking on minor loop complete
 *  0b0..The channel-to-channel linking is disabled
 *  0b1..The channel-to-channel linking is enabled
 */
#define DMA_BITER_ELINKNO_ELINK(x) \
    (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKNO_ELINK_SHIFT)) & DMA_BITER_ELINKNO_ELINK_MASK)
/*! @} */

/* The count of DMA_BITER_ELINKNO */
#define DMA_BITER_ELINKNO_COUNT (4U)

/*! @name BITER_ELINKYES - TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled) */
/*! @{ */

#define DMA_BITER_ELINKYES_BITER_MASK  (0x1FFU)
#define DMA_BITER_ELINKYES_BITER_SHIFT (0U)
/*! BITER - Starting major iteration count
 */
#define DMA_BITER_ELINKYES_BITER(x) \
    (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKYES_BITER_SHIFT)) & DMA_BITER_ELINKYES_BITER_MASK)

#define DMA_BITER_ELINKYES_LINKCH_MASK  (0x600U)
#define DMA_BITER_ELINKYES_LINKCH_SHIFT (9U)
/*! LINKCH - Link Channel Number
 */
#define DMA_BITER_ELINKYES_LINKCH(x) \
    (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKYES_LINKCH_SHIFT)) & DMA_BITER_ELINKYES_LINKCH_MASK)

#define DMA_BITER_ELINKYES_ELINK_MASK  (0x8000U)
#define DMA_BITER_ELINKYES_ELINK_SHIFT (15U)
/*! ELINK - Enables channel-to-channel linking on minor loop complete
 *  0b0..The channel-to-channel linking is disabled
 *  0b1..The channel-to-channel linking is enabled
 */
#define DMA_BITER_ELINKYES_ELINK(x) \
    (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKYES_ELINK_SHIFT)) & DMA_BITER_ELINKYES_ELINK_MASK)
/*! @} */

/* The count of DMA_BITER_ELINKYES */
#define DMA_BITER_ELINKYES_COUNT (4U)

/*!
 * @}
 */ /* end of group DMA_Register_Masks */

/* DMA - Peripheral instance base addresses */
/** Peripheral DMA0 base address */
#define DMA0_BASE (0xC800u)
/** Peripheral DMA0 base pointer */
#define DMA0 ((DMA_Type *)DMA0_BASE)
/** Array initializer of DMA peripheral base addresses */
#define DMA_BASE_ADDRS \
    {                  \
        DMA0_BASE      \
    }
/** Array initializer of DMA peripheral base pointers */
#define DMA_BASE_PTRS \
    {                 \
        DMA0          \
    }
/** Interrupt vectors for the DMA peripheral type */
#define DMA_CHN_IRQS                                   \
    {                                                  \
        {                                              \
            DMA0_IRQn, DMA1_IRQn, DMA2_IRQn, DMA3_IRQn \
        }                                              \
    }
#define DMA_ERROR_IRQS \
    {                  \
        DMA_ERR_IRQn   \
    }

/*!
 * @}
 */ /* end of group DMA_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- DMAMUX Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Peripheral_Access_Layer DMAMUX Peripheral Access Layer
 * @{
 */

/** DMAMUX - Register Layout Typedef */
typedef struct
{
    __IO uint8_t CHCFG[4]; /**< Channel Configuration register, array offset: 0x0, array step: 0x1 */
} DMAMUX_Type;

/* ----------------------------------------------------------------------------
   -- DMAMUX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Masks DMAMUX Register Masks
 * @{
 */

/*! @name CHCFG - Channel Configuration register */
/*! @{ */

#define DMAMUX_CHCFG_SOURCE_MASK  (0x3FU)
#define DMAMUX_CHCFG_SOURCE_SHIFT (0U)
/*! SOURCE - DMA Channel Source (Slot)
 */
#define DMAMUX_CHCFG_SOURCE(x) (((uint8_t)(((uint8_t)(x)) << DMAMUX_CHCFG_SOURCE_SHIFT)) & DMAMUX_CHCFG_SOURCE_MASK)

#define DMAMUX_CHCFG_ENBL_MASK  (0x80U)
#define DMAMUX_CHCFG_ENBL_SHIFT (7U)
/*! ENBL - DMA Channel Enable
 *  0b0..DMA channel is disabled. This mode is primarily used during configuration of the DMAMux. The DMA has
 *       separate channel enables/disables, which should be used to disable or reconfigure a DMA channel.
 *  0b1..DMA channel is enabled
 */
#define DMAMUX_CHCFG_ENBL(x) (((uint8_t)(((uint8_t)(x)) << DMAMUX_CHCFG_ENBL_SHIFT)) & DMAMUX_CHCFG_ENBL_MASK)
/*! @} */

/* The count of DMAMUX_CHCFG */
#define DMAMUX_CHCFG_COUNT (4U)

/*!
 * @}
 */ /* end of group DMAMUX_Register_Masks */

/* DMAMUX - Peripheral instance base addresses */
/** Peripheral DMAMUX base address */
#define DMAMUX_BASE (0xE3B0u)
/** Peripheral DMAMUX base pointer */
#define DMAMUX ((DMAMUX_Type *)DMAMUX_BASE)
/** Array initializer of DMAMUX peripheral base addresses */
#define DMAMUX_BASE_ADDRS \
    {                     \
        DMAMUX_BASE       \
    }
/** Array initializer of DMAMUX peripheral base pointers */
#define DMAMUX_BASE_PTRS \
    {                    \
        DMAMUX           \
    }

/*!
 * @}
 */ /* end of group DMAMUX_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- EVTG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EVTG_Peripheral_Access_Layer EVTG Peripheral Access Layer
 * @{
 */

/** EVTG - Register Layout Typedef */
typedef struct
{
    struct
    {                                  /* offset: 0x0, array step: 0x8 */
        __IO uint16_t EVTG_AOI0_BFT01; /**< AOI0 Boolean Function Term 0 and 1 Configuration Register, array offset:
                                          0x0, array step: 0x8 */
        __IO uint16_t EVTG_AOI0_BFT23; /**< AOI0 Boolean Function Term 2 and 3 Configuration Register, array offset:
                                          0x1, array step: 0x8 */
        __IO uint16_t EVTG_AOI1_BFT01; /**< AOI1 Boolean Function Term 0 and 1 Configuration Register, array offset:
                                          0x2, array step: 0x8 */
        __IO uint16_t EVTG_AOI1_BFT23; /**< AOI1 Boolean Function Term 2 and 3 Configuration Register, array offset:
                                          0x3, array step: 0x8 */
        uint16_t RESERVED_0[1];
        __IO uint16_t EVTG_CTRL;      /**< Control/Status Register, array offset: 0x5, array step: 0x8 */
        __IO uint16_t EVTG_AOI0_FILT; /**< AOI0 Input Filter Register, array offset: 0x6, array step: 0x8 */
        __IO uint16_t EVTG_AOI1_FILT; /**< AOI1 Input Filter Register, array offset: 0x7, array step: 0x8 */
    } EVTG_INST[4];
} EVTG_Type;

/* ----------------------------------------------------------------------------
   -- EVTG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EVTG_Register_Masks EVTG Register Masks
 * @{
 */

/*! @name EVTG_AOI0_BFT01 - AOI0 Boolean Function Term 0 and 1 Configuration Register */
/*! @{ */

#define EVTG_EVTG_AOI0_BFT01_PT1_DC_MASK  (0x3U)
#define EVTG_EVTG_AOI0_BFT01_PT1_DC_SHIFT (0U)
/*! PT1_DC - Product term 1, D input configuration
 */
#define EVTG_EVTG_AOI0_BFT01_PT1_DC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT1_DC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT1_DC_MASK)

#define EVTG_EVTG_AOI0_BFT01_PT1_CC_MASK  (0xCU)
#define EVTG_EVTG_AOI0_BFT01_PT1_CC_SHIFT (2U)
/*! PT1_CC - Product term 1, C input configuration
 */
#define EVTG_EVTG_AOI0_BFT01_PT1_CC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT1_CC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT1_CC_MASK)

#define EVTG_EVTG_AOI0_BFT01_PT1_BC_MASK  (0x30U)
#define EVTG_EVTG_AOI0_BFT01_PT1_BC_SHIFT (4U)
/*! PT1_BC - Product term 1, B input configuration
 */
#define EVTG_EVTG_AOI0_BFT01_PT1_BC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT1_BC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT1_BC_MASK)

#define EVTG_EVTG_AOI0_BFT01_PT1_AC_MASK  (0xC0U)
#define EVTG_EVTG_AOI0_BFT01_PT1_AC_SHIFT (6U)
/*! PT1_AC - Product term 1, A input configuration
 */
#define EVTG_EVTG_AOI0_BFT01_PT1_AC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT1_AC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT1_AC_MASK)

#define EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK  (0x300U)
#define EVTG_EVTG_AOI0_BFT01_PT0_DC_SHIFT (8U)
/*! PT0_DC - Product term 0, D input configuration
 */
#define EVTG_EVTG_AOI0_BFT01_PT0_DC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT0_DC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK)

#define EVTG_EVTG_AOI0_BFT01_PT0_CC_MASK  (0xC00U)
#define EVTG_EVTG_AOI0_BFT01_PT0_CC_SHIFT (10U)
/*! PT0_CC - Product term 0, C input configuration
 */
#define EVTG_EVTG_AOI0_BFT01_PT0_CC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT0_CC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT0_CC_MASK)

#define EVTG_EVTG_AOI0_BFT01_PT0_BC_MASK  (0x3000U)
#define EVTG_EVTG_AOI0_BFT01_PT0_BC_SHIFT (12U)
/*! PT0_BC - Product term 0, B input configuration
 */
#define EVTG_EVTG_AOI0_BFT01_PT0_BC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT0_BC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT0_BC_MASK)

#define EVTG_EVTG_AOI0_BFT01_PT0_AC_MASK  (0xC000U)
#define EVTG_EVTG_AOI0_BFT01_PT0_AC_SHIFT (14U)
/*! PT0_AC - Product term 0, A input configuration
 */
#define EVTG_EVTG_AOI0_BFT01_PT0_AC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT0_AC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT0_AC_MASK)
/*! @} */

/* The count of EVTG_EVTG_AOI0_BFT01 */
#define EVTG_EVTG_AOI0_BFT01_COUNT (4U)

/*! @name EVTG_AOI0_BFT23 - AOI0 Boolean Function Term 2 and 3 Configuration Register */
/*! @{ */

#define EVTG_EVTG_AOI0_BFT23_PT3_DC_MASK  (0x3U)
#define EVTG_EVTG_AOI0_BFT23_PT3_DC_SHIFT (0U)
/*! PT3_DC - Product term 3, D input configuration
 */
#define EVTG_EVTG_AOI0_BFT23_PT3_DC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT3_DC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT3_DC_MASK)

#define EVTG_EVTG_AOI0_BFT23_PT3_CC_MASK  (0xCU)
#define EVTG_EVTG_AOI0_BFT23_PT3_CC_SHIFT (2U)
/*! PT3_CC - Product term 3, C input configuration
 */
#define EVTG_EVTG_AOI0_BFT23_PT3_CC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT3_CC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT3_CC_MASK)

#define EVTG_EVTG_AOI0_BFT23_PT3_BC_MASK  (0x30U)
#define EVTG_EVTG_AOI0_BFT23_PT3_BC_SHIFT (4U)
/*! PT3_BC - Product term 3, B input configuration
 */
#define EVTG_EVTG_AOI0_BFT23_PT3_BC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT3_BC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT3_BC_MASK)

#define EVTG_EVTG_AOI0_BFT23_PT3_AC_MASK  (0xC0U)
#define EVTG_EVTG_AOI0_BFT23_PT3_AC_SHIFT (6U)
/*! PT3_AC - Product term 3, A input configuration
 */
#define EVTG_EVTG_AOI0_BFT23_PT3_AC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT3_AC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT3_AC_MASK)

#define EVTG_EVTG_AOI0_BFT23_PT2_DC_MASK  (0x300U)
#define EVTG_EVTG_AOI0_BFT23_PT2_DC_SHIFT (8U)
/*! PT2_DC - Product term 2, D input configuration
 */
#define EVTG_EVTG_AOI0_BFT23_PT2_DC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT2_DC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT2_DC_MASK)

#define EVTG_EVTG_AOI0_BFT23_PT2_CC_MASK  (0xC00U)
#define EVTG_EVTG_AOI0_BFT23_PT2_CC_SHIFT (10U)
/*! PT2_CC - Product term 2, C input configuration
 */
#define EVTG_EVTG_AOI0_BFT23_PT2_CC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT2_CC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT2_CC_MASK)

#define EVTG_EVTG_AOI0_BFT23_PT2_BC_MASK  (0x3000U)
#define EVTG_EVTG_AOI0_BFT23_PT2_BC_SHIFT (12U)
/*! PT2_BC - Product term 2, B input configuration
 */
#define EVTG_EVTG_AOI0_BFT23_PT2_BC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT2_BC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT2_BC_MASK)

#define EVTG_EVTG_AOI0_BFT23_PT2_AC_MASK  (0xC000U)
#define EVTG_EVTG_AOI0_BFT23_PT2_AC_SHIFT (14U)
/*! PT2_AC - Product term 2, A input configuration
 */
#define EVTG_EVTG_AOI0_BFT23_PT2_AC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT2_AC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT2_AC_MASK)
/*! @} */

/* The count of EVTG_EVTG_AOI0_BFT23 */
#define EVTG_EVTG_AOI0_BFT23_COUNT (4U)

/*! @name EVTG_AOI1_BFT01 - AOI1 Boolean Function Term 0 and 1 Configuration Register */
/*! @{ */

#define EVTG_EVTG_AOI1_BFT01_PT1_DC_MASK  (0x3U)
#define EVTG_EVTG_AOI1_BFT01_PT1_DC_SHIFT (0U)
/*! PT1_DC - Product term 1, D input configuration
 */
#define EVTG_EVTG_AOI1_BFT01_PT1_DC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT1_DC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT1_DC_MASK)

#define EVTG_EVTG_AOI1_BFT01_PT1_CC_MASK  (0xCU)
#define EVTG_EVTG_AOI1_BFT01_PT1_CC_SHIFT (2U)
/*! PT1_CC - Product term 1, C input configuration
 */
#define EVTG_EVTG_AOI1_BFT01_PT1_CC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT1_CC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT1_CC_MASK)

#define EVTG_EVTG_AOI1_BFT01_PT1_BC_MASK  (0x30U)
#define EVTG_EVTG_AOI1_BFT01_PT1_BC_SHIFT (4U)
/*! PT1_BC - Product term 1, B input configuration
 */
#define EVTG_EVTG_AOI1_BFT01_PT1_BC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT1_BC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT1_BC_MASK)

#define EVTG_EVTG_AOI1_BFT01_PT1_AC_MASK  (0xC0U)
#define EVTG_EVTG_AOI1_BFT01_PT1_AC_SHIFT (6U)
/*! PT1_AC - Product term 1, A input configuration
 */
#define EVTG_EVTG_AOI1_BFT01_PT1_AC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT1_AC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT1_AC_MASK)

#define EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK  (0x300U)
#define EVTG_EVTG_AOI1_BFT01_PT0_DC_SHIFT (8U)
/*! PT0_DC - Product term 0, D input configuration
 */
#define EVTG_EVTG_AOI1_BFT01_PT0_DC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT0_DC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK)

#define EVTG_EVTG_AOI1_BFT01_PT0_CC_MASK  (0xC00U)
#define EVTG_EVTG_AOI1_BFT01_PT0_CC_SHIFT (10U)
/*! PT0_CC - Product term 0, C input configuration
 */
#define EVTG_EVTG_AOI1_BFT01_PT0_CC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT0_CC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT0_CC_MASK)

#define EVTG_EVTG_AOI1_BFT01_PT0_BC_MASK  (0x3000U)
#define EVTG_EVTG_AOI1_BFT01_PT0_BC_SHIFT (12U)
/*! PT0_BC - Product term 0, B input configuration
 */
#define EVTG_EVTG_AOI1_BFT01_PT0_BC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT0_BC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT0_BC_MASK)

#define EVTG_EVTG_AOI1_BFT01_PT0_AC_MASK  (0xC000U)
#define EVTG_EVTG_AOI1_BFT01_PT0_AC_SHIFT (14U)
/*! PT0_AC - Product term 0, A input configuration
 */
#define EVTG_EVTG_AOI1_BFT01_PT0_AC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT0_AC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT0_AC_MASK)
/*! @} */

/* The count of EVTG_EVTG_AOI1_BFT01 */
#define EVTG_EVTG_AOI1_BFT01_COUNT (4U)

/*! @name EVTG_AOI1_BFT23 - AOI1 Boolean Function Term 2 and 3 Configuration Register */
/*! @{ */

#define EVTG_EVTG_AOI1_BFT23_PT3_DC_MASK  (0x3U)
#define EVTG_EVTG_AOI1_BFT23_PT3_DC_SHIFT (0U)
/*! PT3_DC - Product term 3, D input configuration
 */
#define EVTG_EVTG_AOI1_BFT23_PT3_DC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT3_DC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT3_DC_MASK)

#define EVTG_EVTG_AOI1_BFT23_PT3_CC_MASK  (0xCU)
#define EVTG_EVTG_AOI1_BFT23_PT3_CC_SHIFT (2U)
/*! PT3_CC - Product term 3, C input configuration
 */
#define EVTG_EVTG_AOI1_BFT23_PT3_CC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT3_CC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT3_CC_MASK)

#define EVTG_EVTG_AOI1_BFT23_PT3_BC_MASK  (0x30U)
#define EVTG_EVTG_AOI1_BFT23_PT3_BC_SHIFT (4U)
/*! PT3_BC - Product term 3, B input configuration
 */
#define EVTG_EVTG_AOI1_BFT23_PT3_BC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT3_BC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT3_BC_MASK)

#define EVTG_EVTG_AOI1_BFT23_PT3_AC_MASK  (0xC0U)
#define EVTG_EVTG_AOI1_BFT23_PT3_AC_SHIFT (6U)
/*! PT3_AC - Product term 3, A input configuration
 */
#define EVTG_EVTG_AOI1_BFT23_PT3_AC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT3_AC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT3_AC_MASK)

#define EVTG_EVTG_AOI1_BFT23_PT2_DC_MASK  (0x300U)
#define EVTG_EVTG_AOI1_BFT23_PT2_DC_SHIFT (8U)
/*! PT2_DC - Product term 2, D input configuration
 */
#define EVTG_EVTG_AOI1_BFT23_PT2_DC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT2_DC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT2_DC_MASK)

#define EVTG_EVTG_AOI1_BFT23_PT2_CC_MASK  (0xC00U)
#define EVTG_EVTG_AOI1_BFT23_PT2_CC_SHIFT (10U)
/*! PT2_CC - Product term 2, C input configuration
 */
#define EVTG_EVTG_AOI1_BFT23_PT2_CC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT2_CC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT2_CC_MASK)

#define EVTG_EVTG_AOI1_BFT23_PT2_BC_MASK  (0x3000U)
#define EVTG_EVTG_AOI1_BFT23_PT2_BC_SHIFT (12U)
/*! PT2_BC - Product term 2, B input configuration
 */
#define EVTG_EVTG_AOI1_BFT23_PT2_BC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT2_BC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT2_BC_MASK)

#define EVTG_EVTG_AOI1_BFT23_PT2_AC_MASK  (0xC000U)
#define EVTG_EVTG_AOI1_BFT23_PT2_AC_SHIFT (14U)
/*! PT2_AC - Product term 2, A input configuration
 */
#define EVTG_EVTG_AOI1_BFT23_PT2_AC(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT2_AC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT2_AC_MASK)
/*! @} */

/* The count of EVTG_EVTG_AOI1_BFT23 */
#define EVTG_EVTG_AOI1_BFT23_COUNT (4U)

/*! @name EVTG_CTRL - Control/Status Register */
/*! @{ */

#define EVTG_EVTG_CTRL_FF_INIT_MASK  (0x1U)
#define EVTG_EVTG_CTRL_FF_INIT_SHIFT (0U)
/*! FF_INIT - Configure flip-flop initial value
 */
#define EVTG_EVTG_CTRL_FF_INIT(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_CTRL_FF_INIT_SHIFT)) & EVTG_EVTG_CTRL_FF_INIT_MASK)

#define EVTG_EVTG_CTRL_INIT_EN_MASK  (0x2U)
#define EVTG_EVTG_CTRL_INIT_EN_SHIFT (1U)
/*! INIT_EN - Flip-flop initial output enable control
 */
#define EVTG_EVTG_CTRL_INIT_EN(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_CTRL_INIT_EN_SHIFT)) & EVTG_EVTG_CTRL_INIT_EN_MASK)

#define EVTG_EVTG_CTRL_MODE_SEL_MASK  (0x1CU)
#define EVTG_EVTG_CTRL_MODE_SEL_SHIFT (2U)
/*! MODE_SEL - Flip-Flop mode configure
 */
#define EVTG_EVTG_CTRL_MODE_SEL(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_CTRL_MODE_SEL_SHIFT)) & EVTG_EVTG_CTRL_MODE_SEL_MASK)

#define EVTG_EVTG_CTRL_FB_OVRD_MASK  (0xC0U)
#define EVTG_EVTG_CTRL_FB_OVRD_SHIFT (6U)
/*! FB_OVRD - EVTG output feedback override control
 */
#define EVTG_EVTG_CTRL_FB_OVRD(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_CTRL_FB_OVRD_SHIFT)) & EVTG_EVTG_CTRL_FB_OVRD_MASK)

#define EVTG_EVTG_CTRL_SYNC_CTRL_MASK  (0xF00U)
#define EVTG_EVTG_CTRL_SYNC_CTRL_SHIFT (8U)
/*! SYNC_CTRL - Four EVTG inputs synchronous with bus clk
 */
#define EVTG_EVTG_CTRL_SYNC_CTRL(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_CTRL_SYNC_CTRL_SHIFT)) & EVTG_EVTG_CTRL_SYNC_CTRL_MASK)
/*! @} */

/* The count of EVTG_EVTG_CTRL */
#define EVTG_EVTG_CTRL_COUNT (4U)

/*! @name EVTG_AOI0_FILT - AOI0 Input Filter Register */
/*! @{ */

#define EVTG_EVTG_AOI0_FILT_FILT_PER_MASK  (0xFFU)
#define EVTG_EVTG_AOI0_FILT_FILT_PER_SHIFT (0U)
/*! FILT_PER - Input Filter Sample Period
 */
#define EVTG_EVTG_AOI0_FILT_FILT_PER(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_FILT_FILT_PER_SHIFT)) & EVTG_EVTG_AOI0_FILT_FILT_PER_MASK)

#define EVTG_EVTG_AOI0_FILT_FILT_CNT_MASK  (0x700U)
#define EVTG_EVTG_AOI0_FILT_FILT_CNT_SHIFT (8U)
/*! FILT_CNT - Input Filter Sample Count
 */
#define EVTG_EVTG_AOI0_FILT_FILT_CNT(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_FILT_FILT_CNT_SHIFT)) & EVTG_EVTG_AOI0_FILT_FILT_CNT_MASK)
/*! @} */

/* The count of EVTG_EVTG_AOI0_FILT */
#define EVTG_EVTG_AOI0_FILT_COUNT (4U)

/*! @name EVTG_AOI1_FILT - AOI1 Input Filter Register */
/*! @{ */

#define EVTG_EVTG_AOI1_FILT_FILT_PER_MASK  (0xFFU)
#define EVTG_EVTG_AOI1_FILT_FILT_PER_SHIFT (0U)
/*! FILT_PER - Input Filter Sample Period
 */
#define EVTG_EVTG_AOI1_FILT_FILT_PER(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_FILT_FILT_PER_SHIFT)) & EVTG_EVTG_AOI1_FILT_FILT_PER_MASK)

#define EVTG_EVTG_AOI1_FILT_FILT_CNT_MASK  (0x700U)
#define EVTG_EVTG_AOI1_FILT_FILT_CNT_SHIFT (8U)
/*! FILT_CNT - Input Filter Sample Count
 */
#define EVTG_EVTG_AOI1_FILT_FILT_CNT(x) \
    (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_FILT_FILT_CNT_SHIFT)) & EVTG_EVTG_AOI1_FILT_FILT_CNT_MASK)
/*! @} */

/* The count of EVTG_EVTG_AOI1_FILT */
#define EVTG_EVTG_AOI1_FILT_COUNT (4U)

/*!
 * @}
 */ /* end of group EVTG_Register_Masks */

/* EVTG - Peripheral instance base addresses */
/** Peripheral EVTG base address */
#define EVTG_BASE (0xE380u)
/** Peripheral EVTG base pointer */
#define EVTG ((EVTG_Type *)EVTG_BASE)
/** Array initializer of EVTG peripheral base addresses */
#define EVTG_BASE_ADDRS \
    {                   \
        EVTG_BASE       \
    }
/** Array initializer of EVTG peripheral base pointers */
#define EVTG_BASE_PTRS \
    {                  \
        EVTG           \
    }

/*!
 * @}
 */ /* end of group EVTG_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- EWM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Peripheral_Access_Layer EWM Peripheral Access Layer
 * @{
 */

/** EWM - Register Layout Typedef */
typedef struct
{
    __IO uint16_t CTRL;         /**< Control Register, offset: 0x0 */
    __O uint16_t SERV;          /**< Service Register, offset: 0x1 */
    __IO uint16_t CMPL;         /**< Compare Low Register, offset: 0x2 */
    __IO uint16_t CMPH;         /**< Compare High Register, offset: 0x3 */
    __IO uint16_t CLKCTRL;      /**< Clock Control Register, offset: 0x4 */
    __IO uint16_t CLKPRESCALER; /**< Clock Prescaler Register, offset: 0x5 */
} EWM_Type;

/* ----------------------------------------------------------------------------
   -- EWM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Masks EWM Register Masks
 * @{
 */

/*! @name CTRL - Control Register */
/*! @{ */

#define EWM_CTRL_EWMEN_MASK  (0x1U)
#define EWM_CTRL_EWMEN_SHIFT (0U)
/*! EWMEN - EWM enable.
 */
#define EWM_CTRL_EWMEN(x) (((uint16_t)(((uint16_t)(x)) << EWM_CTRL_EWMEN_SHIFT)) & EWM_CTRL_EWMEN_MASK)

#define EWM_CTRL_ASSIN_MASK  (0x2U)
#define EWM_CTRL_ASSIN_SHIFT (1U)
/*! ASSIN - EWM_in's Assertion State Select.
 */
#define EWM_CTRL_ASSIN(x) (((uint16_t)(((uint16_t)(x)) << EWM_CTRL_ASSIN_SHIFT)) & EWM_CTRL_ASSIN_MASK)

#define EWM_CTRL_INEN_MASK  (0x4U)
#define EWM_CTRL_INEN_SHIFT (2U)
/*! INEN - Input Enable.
 */
#define EWM_CTRL_INEN(x) (((uint16_t)(((uint16_t)(x)) << EWM_CTRL_INEN_SHIFT)) & EWM_CTRL_INEN_MASK)

#define EWM_CTRL_INTEN_MASK  (0x8U)
#define EWM_CTRL_INTEN_SHIFT (3U)
/*! INTEN - Interrupt Enable.
 */
#define EWM_CTRL_INTEN(x) (((uint16_t)(((uint16_t)(x)) << EWM_CTRL_INTEN_SHIFT)) & EWM_CTRL_INTEN_MASK)
/*! @} */

/*! @name SERV - Service Register */
/*! @{ */

#define EWM_SERV_SERVICE_MASK  (0xFFU)
#define EWM_SERV_SERVICE_SHIFT (0U)
#define EWM_SERV_SERVICE(x)    (((uint16_t)(((uint16_t)(x)) << EWM_SERV_SERVICE_SHIFT)) & EWM_SERV_SERVICE_MASK)
/*! @} */

/*! @name CMPL - Compare Low Register */
/*! @{ */

#define EWM_CMPL_COMPAREL_MASK  (0xFFU)
#define EWM_CMPL_COMPAREL_SHIFT (0U)
#define EWM_CMPL_COMPAREL(x)    (((uint16_t)(((uint16_t)(x)) << EWM_CMPL_COMPAREL_SHIFT)) & EWM_CMPL_COMPAREL_MASK)
/*! @} */

/*! @name CMPH - Compare High Register */
/*! @{ */

#define EWM_CMPH_COMPAREH_MASK  (0xFFU)
#define EWM_CMPH_COMPAREH_SHIFT (0U)
#define EWM_CMPH_COMPAREH(x)    (((uint16_t)(((uint16_t)(x)) << EWM_CMPH_COMPAREH_SHIFT)) & EWM_CMPH_COMPAREH_MASK)
/*! @} */

/*! @name CLKCTRL - Clock Control Register */
/*! @{ */

#define EWM_CLKCTRL_CLKSEL_MASK  (0x3U)
#define EWM_CLKCTRL_CLKSEL_SHIFT (0U)
#define EWM_CLKCTRL_CLKSEL(x)    (((uint16_t)(((uint16_t)(x)) << EWM_CLKCTRL_CLKSEL_SHIFT)) & EWM_CLKCTRL_CLKSEL_MASK)
/*! @} */

/*! @name CLKPRESCALER - Clock Prescaler Register */
/*! @{ */

#define EWM_CLKPRESCALER_CLK_DIV_MASK  (0xFFU)
#define EWM_CLKPRESCALER_CLK_DIV_SHIFT (0U)
#define EWM_CLKPRESCALER_CLK_DIV(x) \
    (((uint16_t)(((uint16_t)(x)) << EWM_CLKPRESCALER_CLK_DIV_SHIFT)) & EWM_CLKPRESCALER_CLK_DIV_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group EWM_Register_Masks */

/* EWM - Peripheral instance base addresses */
/** Peripheral EWM base address */
#define EWM_BASE (0xE330u)
/** Peripheral EWM base pointer */
#define EWM ((EWM_Type *)EWM_BASE)
/** Array initializer of EWM peripheral base addresses */
#define EWM_BASE_ADDRS \
    {                  \
        EWM_BASE       \
    }
/** Array initializer of EWM peripheral base pointers */
#define EWM_BASE_PTRS \
    {                 \
        EWM           \
    }
/** Interrupt vectors for the EWM peripheral type */
#define EWM_IRQS     \
    {                \
        EWM_INT_IRQn \
    }

/*!
 * @}
 */ /* end of group EWM_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- FTFE Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFE_Peripheral_Access_Layer FTFE Peripheral Access Layer
 * @{
 */

/** FTFE - Register Layout Typedef */
typedef struct
{
    __IO uint8_t FSTAT;  /**< Flash Status Register, offset: 0x0 */
    __IO uint8_t FCNFG;  /**< Flash Configuration Register, offset: 0x1 */
    __I uint8_t FSEC;    /**< Flash Security Register, offset: 0x2 */
    __I uint8_t FOPT;    /**< Flash Option Register, offset: 0x3 */
    __IO uint8_t FCCOB3; /**< Flash Common Command Object Registers, offset: 0x4 */
    __IO uint8_t FCCOB2; /**< Flash Common Command Object Registers, offset: 0x5 */
    __IO uint8_t FCCOB1; /**< Flash Common Command Object Registers, offset: 0x6 */
    __IO uint8_t FCCOB0; /**< Flash Common Command Object Registers, offset: 0x7 */
    __IO uint8_t FCCOB7; /**< Flash Common Command Object Registers, offset: 0x8 */
    __IO uint8_t FCCOB6; /**< Flash Common Command Object Registers, offset: 0x9 */
    __IO uint8_t FCCOB5; /**< Flash Common Command Object Registers, offset: 0xA */
    __IO uint8_t FCCOB4; /**< Flash Common Command Object Registers, offset: 0xB */
    __IO uint8_t FCCOBB; /**< Flash Common Command Object Registers, offset: 0xC */
    __IO uint8_t FCCOBA; /**< Flash Common Command Object Registers, offset: 0xD */
    __IO uint8_t FCCOB9; /**< Flash Common Command Object Registers, offset: 0xE */
    __IO uint8_t FCCOB8; /**< Flash Common Command Object Registers, offset: 0xF */
    __IO uint8_t FPROT3; /**< Program Flash Protection Registers, offset: 0x10 */
    __IO uint8_t FPROT2; /**< Program Flash Protection Registers, offset: 0x11 */
    __IO uint8_t FPROT1; /**< Program Flash Protection Registers, offset: 0x12 */
    __IO uint8_t FPROT0; /**< Program Flash Protection Registers, offset: 0x13 */
    uint8_t RESERVED_0[26];
    __IO uint8_t FERSTAT; /**< Flash Error Status Register, offset: 0x2E */
    __IO uint8_t FERCNFG; /**< Flash Error Configuration Register, offset: 0x2F */
} FTFE_Type;

/* ----------------------------------------------------------------------------
   -- FTFE Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFE_Register_Masks FTFE Register Masks
 * @{
 */

/*! @name FSTAT - Flash Status Register */
/*! @{ */

#define FTFE_FSTAT_MGSTAT0_MASK  (0x1U)
#define FTFE_FSTAT_MGSTAT0_SHIFT (0U)
/*! MGSTAT0 - Memory Controller Command Completion Status Flag
 */
#define FTFE_FSTAT_MGSTAT0(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_MGSTAT0_SHIFT)) & FTFE_FSTAT_MGSTAT0_MASK)

#define FTFE_FSTAT_FPVIOL_MASK  (0x10U)
#define FTFE_FSTAT_FPVIOL_SHIFT (4U)
/*! FPVIOL - Flash Protection Violation Flag
 *  0b0..No protection violation detected
 *  0b1..Protection violation detected
 */
#define FTFE_FSTAT_FPVIOL(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_FPVIOL_SHIFT)) & FTFE_FSTAT_FPVIOL_MASK)

#define FTFE_FSTAT_ACCERR_MASK  (0x20U)
#define FTFE_FSTAT_ACCERR_SHIFT (5U)
/*! ACCERR - Flash Access Error Flag
 *  0b0..No access error detected
 *  0b1..Access error detected
 */
#define FTFE_FSTAT_ACCERR(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_ACCERR_SHIFT)) & FTFE_FSTAT_ACCERR_MASK)

#define FTFE_FSTAT_RDCOLERR_MASK  (0x40U)
#define FTFE_FSTAT_RDCOLERR_SHIFT (6U)
/*! RDCOLERR - FTFE Read Collision Error Flag
 *  0b0..No collision error detected
 *  0b1..Collision error detected
 */
#define FTFE_FSTAT_RDCOLERR(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_RDCOLERR_SHIFT)) & FTFE_FSTAT_RDCOLERR_MASK)

#define FTFE_FSTAT_CCIF_MASK  (0x80U)
#define FTFE_FSTAT_CCIF_SHIFT (7U)
/*! CCIF - Command Complete Interrupt Flag
 *  0b0..FTFE command in progress
 *  0b1..FTFE command has completed
 */
#define FTFE_FSTAT_CCIF(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_CCIF_SHIFT)) & FTFE_FSTAT_CCIF_MASK)
/*! @} */

/*! @name FCNFG - Flash Configuration Register */
/*! @{ */

#define FTFE_FCNFG_EEERDY_MASK  (0x1U)
#define FTFE_FCNFG_EEERDY_SHIFT (0U)
/*! EEERDY
 *  0b0..See RAMRDY for availability of programming acceleration RAM
 *  0b1..Reserved
 */
#define FTFE_FCNFG_EEERDY(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_EEERDY_SHIFT)) & FTFE_FCNFG_EEERDY_MASK)

#define FTFE_FCNFG_RAMRDY_MASK  (0x2U)
#define FTFE_FCNFG_RAMRDY_SHIFT (1U)
/*! RAMRDY - RAM Ready
 *  0b0..Programming acceleration RAM is not available
 *  0b1..Programming acceleration RAM is available
 */
#define FTFE_FCNFG_RAMRDY(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_RAMRDY_SHIFT)) & FTFE_FCNFG_RAMRDY_MASK)

#define FTFE_FCNFG_PFLSH_MASK  (0x4U)
#define FTFE_FCNFG_PFLSH_SHIFT (2U)
/*! PFLSH - This bit is reserved and always has the value 1.
 */
#define FTFE_FCNFG_PFLSH(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_PFLSH_SHIFT)) & FTFE_FCNFG_PFLSH_MASK)

#define FTFE_FCNFG_SWAP_MASK  (0x8U)
#define FTFE_FCNFG_SWAP_SHIFT (3U)
/*! SWAP - Swap
 *  0b0..Program flash 0 block is located at relative address 0x0000
 *  0b1..Program flash 1 block is located at relative address 0x0000
 */
#define FTFE_FCNFG_SWAP(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_SWAP_SHIFT)) & FTFE_FCNFG_SWAP_MASK)

#define FTFE_FCNFG_ERSSUSP_MASK  (0x10U)
#define FTFE_FCNFG_ERSSUSP_SHIFT (4U)
/*! ERSSUSP - Erase Suspend
 *  0b0..No suspend requested
 *  0b1..Suspend the current Erase Flash Sector command execution
 */
#define FTFE_FCNFG_ERSSUSP(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_ERSSUSP_SHIFT)) & FTFE_FCNFG_ERSSUSP_MASK)

#define FTFE_FCNFG_ERSAREQ_MASK  (0x20U)
#define FTFE_FCNFG_ERSAREQ_SHIFT (5U)
/*! ERSAREQ - Erase All Request
 *  0b0..No request or request complete
 *  0b1..Request to: run the Erase All Blocks command, verify the erased state, program the security byte in the
 *       Flash Configuration Field to the unsecure state, and release MCU security by setting the FSEC[SEC] field to
 *       the unsecure state
 */
#define FTFE_FCNFG_ERSAREQ(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_ERSAREQ_SHIFT)) & FTFE_FCNFG_ERSAREQ_MASK)

#define FTFE_FCNFG_RDCOLLIE_MASK  (0x40U)
#define FTFE_FCNFG_RDCOLLIE_SHIFT (6U)
/*! RDCOLLIE - Read Collision Error Interrupt Enable
 *  0b0..Read collision error interrupt disabled
 *  0b1..Read collision error interrupt enabled. An interrupt request is generated whenever an FTFE read collision
 *       error is detected (see the description of FSTAT[RDCOLERR]).
 */
#define FTFE_FCNFG_RDCOLLIE(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_RDCOLLIE_SHIFT)) & FTFE_FCNFG_RDCOLLIE_MASK)

#define FTFE_FCNFG_CCIE_MASK  (0x80U)
#define FTFE_FCNFG_CCIE_SHIFT (7U)
/*! CCIE - Command Complete Interrupt Enable
 *  0b0..Command complete interrupt disabled
 *  0b1..Command complete interrupt enabled. An interrupt request is generated whenever the FSTAT[CCIF] flag is set.
 */
#define FTFE_FCNFG_CCIE(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_CCIE_SHIFT)) & FTFE_FCNFG_CCIE_MASK)
/*! @} */

/*! @name FSEC - Flash Security Register */
/*! @{ */

#define FTFE_FSEC_SEC_MASK  (0x3U)
#define FTFE_FSEC_SEC_SHIFT (0U)
/*! SEC - Flash Security
 *  0b00..MCU security status is secure
 *  0b01..MCU security status is secure
 *  0b10..MCU security status is unsecure (The standard shipping condition of the FTFE is unsecure.)
 *  0b11..MCU security status is secure
 */
#define FTFE_FSEC_SEC(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FSEC_SEC_SHIFT)) & FTFE_FSEC_SEC_MASK)

#define FTFE_FSEC_FSLACC_MASK  (0xCU)
#define FTFE_FSEC_FSLACC_SHIFT (2U)
/*! FSLACC - Factory Security Level Access Code
 *  0b00..Factory access granted
 *  0b01..Factory access denied
 *  0b10..Factory access denied
 *  0b11..Factory access granted
 */
#define FTFE_FSEC_FSLACC(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FSEC_FSLACC_SHIFT)) & FTFE_FSEC_FSLACC_MASK)

#define FTFE_FSEC_MEEN_MASK  (0x30U)
#define FTFE_FSEC_MEEN_SHIFT (4U)
/*! MEEN - Mass Erase Enable Bits
 *  0b00..Mass erase is enabled
 *  0b01..Mass erase is enabled
 *  0b10..Mass erase is disabled
 *  0b11..Mass erase is enabled
 */
#define FTFE_FSEC_MEEN(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FSEC_MEEN_SHIFT)) & FTFE_FSEC_MEEN_MASK)

#define FTFE_FSEC_KEYEN_MASK  (0xC0U)
#define FTFE_FSEC_KEYEN_SHIFT (6U)
/*! KEYEN - Backdoor Key Security Enable
 *  0b00..Backdoor key access disabled
 *  0b01..Backdoor key access disabled (preferred KEYEN state to disable backdoor key access)
 *  0b10..Backdoor key access enabled
 *  0b11..Backdoor key access disabled
 */
#define FTFE_FSEC_KEYEN(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FSEC_KEYEN_SHIFT)) & FTFE_FSEC_KEYEN_MASK)
/*! @} */

/*! @name FOPT - Flash Option Register */
/*! @{ */

#define FTFE_FOPT_OPT_MASK  (0xFFU)
#define FTFE_FOPT_OPT_SHIFT (0U)
/*! OPT - Nonvolatile Option
 */
#define FTFE_FOPT_OPT(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FOPT_OPT_SHIFT)) & FTFE_FOPT_OPT_MASK)
/*! @} */

/*! @name FCCOB3 - Flash Common Command Object Registers */
/*! @{ */

#define FTFE_FCCOB3_CCOBn_MASK  (0xFFU)
#define FTFE_FCCOB3_CCOBn_SHIFT (0U)
#define FTFE_FCCOB3_CCOBn(x)    (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB3_CCOBn_SHIFT)) & FTFE_FCCOB3_CCOBn_MASK)
/*! @} */

/*! @name FCCOB2 - Flash Common Command Object Registers */
/*! @{ */

#define FTFE_FCCOB2_CCOBn_MASK  (0xFFU)
#define FTFE_FCCOB2_CCOBn_SHIFT (0U)
#define FTFE_FCCOB2_CCOBn(x)    (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB2_CCOBn_SHIFT)) & FTFE_FCCOB2_CCOBn_MASK)
/*! @} */

/*! @name FCCOB1 - Flash Common Command Object Registers */
/*! @{ */

#define FTFE_FCCOB1_CCOBn_MASK  (0xFFU)
#define FTFE_FCCOB1_CCOBn_SHIFT (0U)
#define FTFE_FCCOB1_CCOBn(x)    (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB1_CCOBn_SHIFT)) & FTFE_FCCOB1_CCOBn_MASK)
/*! @} */

/*! @name FCCOB0 - Flash Common Command Object Registers */
/*! @{ */

#define FTFE_FCCOB0_CCOBn_MASK  (0xFFU)
#define FTFE_FCCOB0_CCOBn_SHIFT (0U)
#define FTFE_FCCOB0_CCOBn(x)    (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB0_CCOBn_SHIFT)) & FTFE_FCCOB0_CCOBn_MASK)
/*! @} */

/*! @name FCCOB7 - Flash Common Command Object Registers */
/*! @{ */

#define FTFE_FCCOB7_CCOBn_MASK  (0xFFU)
#define FTFE_FCCOB7_CCOBn_SHIFT (0U)
#define FTFE_FCCOB7_CCOBn(x)    (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB7_CCOBn_SHIFT)) & FTFE_FCCOB7_CCOBn_MASK)
/*! @} */

/*! @name FCCOB6 - Flash Common Command Object Registers */
/*! @{ */

#define FTFE_FCCOB6_CCOBn_MASK  (0xFFU)
#define FTFE_FCCOB6_CCOBn_SHIFT (0U)
#define FTFE_FCCOB6_CCOBn(x)    (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB6_CCOBn_SHIFT)) & FTFE_FCCOB6_CCOBn_MASK)
/*! @} */

/*! @name FCCOB5 - Flash Common Command Object Registers */
/*! @{ */

#define FTFE_FCCOB5_CCOBn_MASK  (0xFFU)
#define FTFE_FCCOB5_CCOBn_SHIFT (0U)
#define FTFE_FCCOB5_CCOBn(x)    (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB5_CCOBn_SHIFT)) & FTFE_FCCOB5_CCOBn_MASK)
/*! @} */

/*! @name FCCOB4 - Flash Common Command Object Registers */
/*! @{ */

#define FTFE_FCCOB4_CCOBn_MASK  (0xFFU)
#define FTFE_FCCOB4_CCOBn_SHIFT (0U)
#define FTFE_FCCOB4_CCOBn(x)    (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB4_CCOBn_SHIFT)) & FTFE_FCCOB4_CCOBn_MASK)
/*! @} */

/*! @name FCCOBB - Flash Common Command Object Registers */
/*! @{ */

#define FTFE_FCCOBB_CCOBn_MASK  (0xFFU)
#define FTFE_FCCOBB_CCOBn_SHIFT (0U)
#define FTFE_FCCOBB_CCOBn(x)    (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOBB_CCOBn_SHIFT)) & FTFE_FCCOBB_CCOBn_MASK)
/*! @} */

/*! @name FCCOBA - Flash Common Command Object Registers */
/*! @{ */

#define FTFE_FCCOBA_CCOBn_MASK  (0xFFU)
#define FTFE_FCCOBA_CCOBn_SHIFT (0U)
#define FTFE_FCCOBA_CCOBn(x)    (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOBA_CCOBn_SHIFT)) & FTFE_FCCOBA_CCOBn_MASK)
/*! @} */

/*! @name FCCOB9 - Flash Common Command Object Registers */
/*! @{ */

#define FTFE_FCCOB9_CCOBn_MASK  (0xFFU)
#define FTFE_FCCOB9_CCOBn_SHIFT (0U)
#define FTFE_FCCOB9_CCOBn(x)    (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB9_CCOBn_SHIFT)) & FTFE_FCCOB9_CCOBn_MASK)
/*! @} */

/*! @name FCCOB8 - Flash Common Command Object Registers */
/*! @{ */

#define FTFE_FCCOB8_CCOBn_MASK  (0xFFU)
#define FTFE_FCCOB8_CCOBn_SHIFT (0U)
#define FTFE_FCCOB8_CCOBn(x)    (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB8_CCOBn_SHIFT)) & FTFE_FCCOB8_CCOBn_MASK)
/*! @} */

/*! @name FPROT3 - Program Flash Protection Registers */
/*! @{ */

#define FTFE_FPROT3_PROT_MASK  (0xFFU)
#define FTFE_FPROT3_PROT_SHIFT (0U)
/*! PROT - Program Flash Region Protect
 *  0b00000000..Program flash region is protected.
 *  0b00000001..Program flash region is not protected
 */
#define FTFE_FPROT3_PROT(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FPROT3_PROT_SHIFT)) & FTFE_FPROT3_PROT_MASK)
/*! @} */

/*! @name FPROT2 - Program Flash Protection Registers */
/*! @{ */

#define FTFE_FPROT2_PROT_MASK  (0xFFU)
#define FTFE_FPROT2_PROT_SHIFT (0U)
/*! PROT - Program Flash Region Protect
 *  0b00000000..Program flash region is protected.
 *  0b00000001..Program flash region is not protected
 */
#define FTFE_FPROT2_PROT(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FPROT2_PROT_SHIFT)) & FTFE_FPROT2_PROT_MASK)
/*! @} */

/*! @name FPROT1 - Program Flash Protection Registers */
/*! @{ */

#define FTFE_FPROT1_PROT_MASK  (0xFFU)
#define FTFE_FPROT1_PROT_SHIFT (0U)
/*! PROT - Program Flash Region Protect
 *  0b00000000..Program flash region is protected.
 *  0b00000001..Program flash region is not protected
 */
#define FTFE_FPROT1_PROT(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FPROT1_PROT_SHIFT)) & FTFE_FPROT1_PROT_MASK)
/*! @} */

/*! @name FPROT0 - Program Flash Protection Registers */
/*! @{ */

#define FTFE_FPROT0_PROT_MASK  (0xFFU)
#define FTFE_FPROT0_PROT_SHIFT (0U)
/*! PROT - Program Flash Region Protect
 *  0b00000000..Program flash region is protected.
 *  0b00000001..Program flash region is not protected
 */
#define FTFE_FPROT0_PROT(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FPROT0_PROT_SHIFT)) & FTFE_FPROT0_PROT_MASK)
/*! @} */

/*! @name FERSTAT - Flash Error Status Register */
/*! @{ */

#define FTFE_FERSTAT_DFDIF_MASK  (0x2U)
#define FTFE_FERSTAT_DFDIF_SHIFT (1U)
/*! DFDIF - Double Bit Fault Detect Interrupt Flag
 *  0b0..Double bit fault not detected during a valid flash read access from the platform flash controller
 *  0b1..Double bit fault detected (or FERCNFG[FDFD] is set) during a valid flash read access from the platform flash
 * controller
 */
#define FTFE_FERSTAT_DFDIF(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FERSTAT_DFDIF_SHIFT)) & FTFE_FERSTAT_DFDIF_MASK)
/*! @} */

/*! @name FERCNFG - Flash Error Configuration Register */
/*! @{ */

#define FTFE_FERCNFG_DFDIE_MASK  (0x2U)
#define FTFE_FERCNFG_DFDIE_SHIFT (1U)
/*! DFDIE - Double Bit Fault Detect Interrupt Enable
 *  0b0..Double bit fault detect interrupt disabled
 *  0b1..Double bit fault detect interrupt enabled. An interrupt request is generated whenever the FERSTAT[DFDIF] flag
 * is set.
 */
#define FTFE_FERCNFG_DFDIE(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FERCNFG_DFDIE_SHIFT)) & FTFE_FERCNFG_DFDIE_MASK)

#define FTFE_FERCNFG_FDFD_MASK  (0x20U)
#define FTFE_FERCNFG_FDFD_SHIFT (5U)
/*! FDFD - Force Double Bit Fault Detect
 *  0b0..FERSTAT[DFDIF] sets only if a double bit fault is detected during read access from the platform flash
 * controller 0b1..FERSTAT[DFDIF] sets during any valid flash read access from the platform flash controller. An
 * interrupt request is generated if the DFDIE bit is set.
 */
#define FTFE_FERCNFG_FDFD(x) (((uint8_t)(((uint8_t)(x)) << FTFE_FERCNFG_FDFD_SHIFT)) & FTFE_FERCNFG_FDFD_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group FTFE_Register_Masks */

/* FTFE - Peripheral instance base addresses */
/** Peripheral FTFE base address */
#define FTFE_BASE (0xE3C0u)
/** Peripheral FTFE base pointer */
#define FTFE ((FTFE_Type *)FTFE_BASE)
/** Array initializer of FTFE peripheral base addresses */
#define FTFE_BASE_ADDRS \
    {                   \
        FTFE_BASE       \
    }
/** Array initializer of FTFE peripheral base pointers */
#define FTFE_BASE_PTRS \
    {                  \
        FTFE           \
    }

/*!
 * @}
 */ /* end of group FTFE_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- GPIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Peripheral_Access_Layer GPIO Peripheral Access Layer
 * @{
 */

/** GPIO - Register Layout Typedef */
typedef struct
{
    __IO uint16_t PUR;    /**< GPIO Pull Resistor Enable Register, offset: 0x0 */
    __IO uint16_t DR;     /**< GPIO Data Register, offset: 0x1 */
    __IO uint16_t DDR;    /**< GPIO Data Direction Register, offset: 0x2 */
    __IO uint16_t PER;    /**< GPIO Peripheral Enable Register, offset: 0x3 */
    __IO uint16_t IAR;    /**< GPIO Interrupt Assert Register, offset: 0x4 */
    __IO uint16_t IENR;   /**< GPIO Interrupt Enable Register, offset: 0x5 */
    __IO uint16_t IPOLR;  /**< GPIO Interrupt Polarity Register, offset: 0x6 */
    __I uint16_t IPR;     /**< GPIO Interrupt Pending Register, offset: 0x7 */
    __IO uint16_t IESR;   /**< GPIO Interrupt Edge Sensitive Register, offset: 0x8 */
    __IO uint16_t PPMODE; /**< GPIO Push-Pull Mode Register, offset: 0x9 */
    __I uint16_t RAWDATA; /**< GPIO Raw Data Register, offset: 0xA */
    __IO uint16_t DRIVE;  /**< GPIO Drive Strength Control Register, offset: 0xB */
    __IO uint16_t PUS;    /**< GPIO Pull Resistor Type Select, offset: 0xC */
    __IO uint16_t SRE;    /**< Slew Rate Control Register, offset: 0xD */
} GPIO_Type;

/* ----------------------------------------------------------------------------
   -- GPIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Masks GPIO Register Masks
 * @{
 */

/*! @name PUR - GPIO Pull Resistor Enable Register */
/*! @{ */

#define GPIO_PUR_PU_MASK  (0xFFFFU)
#define GPIO_PUR_PU_SHIFT (0U)
/*! PU - Pull Resistor Enable Bits
 *  0b0000000000000000..Pull resistor is disabled
 *  0b0000000000000001..Pull resistor is enabled
 */
#define GPIO_PUR_PU(x) (((uint16_t)(((uint16_t)(x)) << GPIO_PUR_PU_SHIFT)) & GPIO_PUR_PU_MASK)
/*! @} */

/*! @name DR - GPIO Data Register */
/*! @{ */

#define GPIO_DR_D_MASK  (0xFFFFU)
#define GPIO_DR_D_SHIFT (0U)
/*! D - Data Bits
 */
#define GPIO_DR_D(x) (((uint16_t)(((uint16_t)(x)) << GPIO_DR_D_SHIFT)) & GPIO_DR_D_MASK)
/*! @} */

/*! @name DDR - GPIO Data Direction Register */
/*! @{ */

#define GPIO_DDR_DD_MASK  (0xFFFFU)
#define GPIO_DDR_DD_SHIFT (0U)
/*! DD - Data Direction Bits
 *  0b0000000000000000..Pin is an input
 *  0b0000000000000001..Pin is an output
 */
#define GPIO_DDR_DD(x) (((uint16_t)(((uint16_t)(x)) << GPIO_DDR_DD_SHIFT)) & GPIO_DDR_DD_MASK)
/*! @} */

/*! @name PER - GPIO Peripheral Enable Register */
/*! @{ */

#define GPIO_PER_PE_MASK  (0xFFFFU)
#define GPIO_PER_PE_SHIFT (0U)
/*! PE - Peripheral Enable Bits
 *  0b0000000000000000..Pin is for GPIO (GPIO mode)
 *  0b0000000000000001..Pin is for peripheral (peripheral mode)
 */
#define GPIO_PER_PE(x) (((uint16_t)(((uint16_t)(x)) << GPIO_PER_PE_SHIFT)) & GPIO_PER_PE_MASK)
/*! @} */

/*! @name IAR - GPIO Interrupt Assert Register */
/*! @{ */

#define GPIO_IAR_IA_MASK  (0xFFFFU)
#define GPIO_IAR_IA_SHIFT (0U)
/*! IA - Interrupt Assert Bits
 *  0b0000000000000000..Deassert software interrupt
 *  0b0000000000000001..Assert software interrupt
 */
#define GPIO_IAR_IA(x) (((uint16_t)(((uint16_t)(x)) << GPIO_IAR_IA_SHIFT)) & GPIO_IAR_IA_MASK)
/*! @} */

/*! @name IENR - GPIO Interrupt Enable Register */
/*! @{ */

#define GPIO_IENR_IEN_MASK  (0xFFFFU)
#define GPIO_IENR_IEN_SHIFT (0U)
/*! IEN - Interrupt Enable Bits
 *  0b0000000000000000..External Interrupt is disabled
 *  0b0000000000000001..External Interrupt is enabled
 */
#define GPIO_IENR_IEN(x) (((uint16_t)(((uint16_t)(x)) << GPIO_IENR_IEN_SHIFT)) & GPIO_IENR_IEN_MASK)
/*! @} */

/*! @name IPOLR - GPIO Interrupt Polarity Register */
/*! @{ */

#define GPIO_IPOLR_IPOL_MASK  (0xFFFFU)
#define GPIO_IPOLR_IPOL_SHIFT (0U)
/*! IPOL - Interrupt Polarity Bits
 *  0b0000000000000000..Interrupt occurred on rising edge
 *  0b0000000000000001..Interrupt occurred on falling edge
 */
#define GPIO_IPOLR_IPOL(x) (((uint16_t)(((uint16_t)(x)) << GPIO_IPOLR_IPOL_SHIFT)) & GPIO_IPOLR_IPOL_MASK)
/*! @} */

/*! @name IPR - GPIO Interrupt Pending Register */
/*! @{ */

#define GPIO_IPR_IP_MASK  (0xFFFFU)
#define GPIO_IPR_IP_SHIFT (0U)
/*! IP - Interrupt Pending Bits
 *  0b0000000000000000..No Interrupt
 *  0b0000000000000001..Interrupt occurred
 */
#define GPIO_IPR_IP(x) (((uint16_t)(((uint16_t)(x)) << GPIO_IPR_IP_SHIFT)) & GPIO_IPR_IP_MASK)
/*! @} */

/*! @name IESR - GPIO Interrupt Edge Sensitive Register */
/*! @{ */

#define GPIO_IESR_IES_MASK  (0xFFFFU)
#define GPIO_IESR_IES_SHIFT (0U)
/*! IES - Interrupt Edge-Sensitive Bits
 *  0b0000000000000000..No edge detected if read; no effect if writing
 *  0b0000000000000001..An edge detected if read; clear corresponding Interrupt Pending bit if writing
 */
#define GPIO_IESR_IES(x) (((uint16_t)(((uint16_t)(x)) << GPIO_IESR_IES_SHIFT)) & GPIO_IESR_IES_MASK)
/*! @} */

/*! @name PPMODE - GPIO Push-Pull Mode Register */
/*! @{ */

#define GPIO_PPMODE_PPMODE_MASK  (0xFFFFU)
#define GPIO_PPMODE_PPMODE_SHIFT (0U)
/*! PPMODE - Push-Pull Mode Bits
 *  0b0000000000000000..Open Drain Mode
 *  0b0000000000000001..Push-Pull Mode
 */
#define GPIO_PPMODE_PPMODE(x) (((uint16_t)(((uint16_t)(x)) << GPIO_PPMODE_PPMODE_SHIFT)) & GPIO_PPMODE_PPMODE_MASK)
/*! @} */

/*! @name RAWDATA - GPIO Raw Data Register */
/*! @{ */

#define GPIO_RAWDATA_RAW_DATA_MASK  (0xFFFFU)
#define GPIO_RAWDATA_RAW_DATA_SHIFT (0U)
/*! RAW_DATA - Raw Data Bits
 */
#define GPIO_RAWDATA_RAW_DATA(x) \
    (((uint16_t)(((uint16_t)(x)) << GPIO_RAWDATA_RAW_DATA_SHIFT)) & GPIO_RAWDATA_RAW_DATA_MASK)
/*! @} */

/*! @name DRIVE - GPIO Drive Strength Control Register */
/*! @{ */

#define GPIO_DRIVE_DRIVE_MASK  (0xFFFFU)
#define GPIO_DRIVE_DRIVE_SHIFT (0U)
/*! DRIVE - Drive Strength Selector Bits
 *  0b0000000000000000..Low drive strength
 *  0b0000000000000001..High drive strength
 */
#define GPIO_DRIVE_DRIVE(x) (((uint16_t)(((uint16_t)(x)) << GPIO_DRIVE_DRIVE_SHIFT)) & GPIO_DRIVE_DRIVE_MASK)
/*! @} */

/*! @name PUS - GPIO Pull Resistor Type Select */
/*! @{ */

#define GPIO_PUS_PUS_MASK  (0xFFFFU)
#define GPIO_PUS_PUS_SHIFT (0U)
/*! PUS - Pull Resistor Type Select Bits
 *  0b0000000000000000..Pulldown resistor
 *  0b0000000000000001..Pullup resistor
 */
#define GPIO_PUS_PUS(x) (((uint16_t)(((uint16_t)(x)) << GPIO_PUS_PUS_SHIFT)) & GPIO_PUS_PUS_MASK)
/*! @} */

/*! @name SRE - Slew Rate Control Register */
/*! @{ */

#define GPIO_SRE_SRE_MASK  (0xFFFFU)
#define GPIO_SRE_SRE_SHIFT (0U)
/*! SRE - Slew Rate Enable
 *  0b0000000000000000..Slew rate is enabled (the turn-on time of the output transistor is faster)
 *  0b0000000000000001..Slew rate is disabled (the turn-on time of the output transistor is slower)
 */
#define GPIO_SRE_SRE(x) (((uint16_t)(((uint16_t)(x)) << GPIO_SRE_SRE_SHIFT)) & GPIO_SRE_SRE_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group GPIO_Register_Masks */

/* GPIO - Peripheral instance base addresses */
/** Peripheral GPIOA base address */
#define GPIOA_BASE (0xE200u)
/** Peripheral GPIOA base pointer */
#define GPIOA ((GPIO_Type *)GPIOA_BASE)
/** Peripheral GPIOB base address */
#define GPIOB_BASE (0xE210u)
/** Peripheral GPIOB base pointer */
#define GPIOB ((GPIO_Type *)GPIOB_BASE)
/** Peripheral GPIOC base address */
#define GPIOC_BASE (0xE220u)
/** Peripheral GPIOC base pointer */
#define GPIOC ((GPIO_Type *)GPIOC_BASE)
/** Peripheral GPIOD base address */
#define GPIOD_BASE (0xE230u)
/** Peripheral GPIOD base pointer */
#define GPIOD ((GPIO_Type *)GPIOD_BASE)
/** Peripheral GPIOE base address */
#define GPIOE_BASE (0xE240u)
/** Peripheral GPIOE base pointer */
#define GPIOE ((GPIO_Type *)GPIOE_BASE)
/** Peripheral GPIOF base address */
#define GPIOF_BASE (0xE250u)
/** Peripheral GPIOF base pointer */
#define GPIOF ((GPIO_Type *)GPIOF_BASE)
/** Peripheral GPIOG base address */
#define GPIOG_BASE (0xE260u)
/** Peripheral GPIOG base pointer */
#define GPIOG ((GPIO_Type *)GPIOG_BASE)
/** Array initializer of GPIO peripheral base addresses */
#define GPIO_BASE_ADDRS                                                                    \
    {                                                                                      \
        GPIOA_BASE, GPIOB_BASE, GPIOC_BASE, GPIOD_BASE, GPIOE_BASE, GPIOF_BASE, GPIOG_BASE \
    }
/** Array initializer of GPIO peripheral base pointers */
#define GPIO_BASE_PTRS                                  \
    {                                                   \
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG \
    }

/*!
 * @}
 */ /* end of group GPIO_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- I2C Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Peripheral_Access_Layer I2C Peripheral Access Layer
 * @{
 */

/** I2C - Register Layout Typedef */
typedef struct
{
    __IO uint16_t A1;   /**< I2C Address Register 1, offset: 0x0 */
    __IO uint16_t F;    /**< I2C Frequency Divider register, offset: 0x1 */
    __IO uint16_t C1;   /**< I2C Control Register 1, offset: 0x2 */
    __IO uint16_t S;    /**< I2C Status register, offset: 0x3 */
    __IO uint16_t D;    /**< I2C Data I/O register, offset: 0x4 */
    __IO uint16_t C2;   /**< I2C Control Register 2, offset: 0x5 */
    __IO uint16_t FLT;  /**< I2C Programmable Input Glitch Filter Register, offset: 0x6 */
    __IO uint16_t RA;   /**< I2C Range Address register, offset: 0x7 */
    __IO uint16_t SMB;  /**< I2C SMBus Control and Status register, offset: 0x8 */
    __IO uint16_t A2;   /**< I2C Address Register 2, offset: 0x9 */
    __IO uint16_t SLTH; /**< I2C SCL Low Timeout Register High, offset: 0xA */
    __IO uint16_t SLTL; /**< I2C SCL Low Timeout Register Low, offset: 0xB */
} I2C_Type;

/* ----------------------------------------------------------------------------
   -- I2C Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Masks I2C Register Masks
 * @{
 */

/*! @name A1 - I2C Address Register 1 */
/*! @{ */

#define I2C_A1_AD_MASK  (0xFEU)
#define I2C_A1_AD_SHIFT (1U)
/*! AD - Address
 */
#define I2C_A1_AD(x) (((uint16_t)(((uint16_t)(x)) << I2C_A1_AD_SHIFT)) & I2C_A1_AD_MASK)
/*! @} */

/*! @name F - I2C Frequency Divider register */
/*! @{ */

#define I2C_F_ICR_MASK  (0x3FU)
#define I2C_F_ICR_SHIFT (0U)
/*! ICR - ClockRate
 */
#define I2C_F_ICR(x) (((uint16_t)(((uint16_t)(x)) << I2C_F_ICR_SHIFT)) & I2C_F_ICR_MASK)

#define I2C_F_MULT_MASK  (0xC0U)
#define I2C_F_MULT_SHIFT (6U)
/*! MULT - Multiplier Factor
 *  0b00..mul = 1
 *  0b01..mul = 2
 *  0b10..mul = 4
 *  0b11..Reserved
 */
#define I2C_F_MULT(x) (((uint16_t)(((uint16_t)(x)) << I2C_F_MULT_SHIFT)) & I2C_F_MULT_MASK)
/*! @} */

/*! @name C1 - I2C Control Register 1 */
/*! @{ */

#define I2C_C1_DMAEN_MASK  (0x1U)
#define I2C_C1_DMAEN_SHIFT (0U)
/*! DMAEN - DMA Enable
 *  0b0..All DMA signalling disabled.
 *  0b1..DMA transfer is enabled. While SMB[FACK] = 0, the following conditions trigger the DMA request: a data
 *       byte is received, and either address or data is transmitted. (ACK/NACK is automatic) the first byte received
 *       matches the A1 register or is a general call address. If any address matching occurs, S[IAAS] and S[TCF]
 *       are set. If the direction of transfer is known from master to slave, then it is not required to check
 *       S[SRW]. With this assumption, DMA can also be used in this case. In other cases, if the master reads data from
 *       the slave, then it is required to rewrite the C1 register operation. With this assumption, DMA cannot be
 *       used. When FACK = 1, an address or a data byte is transmitted.
 */
#define I2C_C1_DMAEN(x) (((uint16_t)(((uint16_t)(x)) << I2C_C1_DMAEN_SHIFT)) & I2C_C1_DMAEN_MASK)

#define I2C_C1_WUEN_MASK  (0x2U)
#define I2C_C1_WUEN_SHIFT (1U)
/*! WUEN - Wakeup Enable
 *  0b0..Normal operation. No interrupt generated when address matching in low power mode.
 *  0b1..Enables the wakeup function in low power mode.
 */
#define I2C_C1_WUEN(x) (((uint16_t)(((uint16_t)(x)) << I2C_C1_WUEN_SHIFT)) & I2C_C1_WUEN_MASK)

#define I2C_C1_RSTA_MASK  (0x4U)
#define I2C_C1_RSTA_SHIFT (2U)
/*! RSTA - Repeat START
 */
#define I2C_C1_RSTA(x) (((uint16_t)(((uint16_t)(x)) << I2C_C1_RSTA_SHIFT)) & I2C_C1_RSTA_MASK)

#define I2C_C1_TXAK_MASK  (0x8U)
#define I2C_C1_TXAK_SHIFT (3U)
/*! TXAK - Transmit Acknowledge Enable
 *  0b0..An acknowledge signal is sent to the bus on the following receiving byte (if FACK is cleared) or the
 *       current receiving byte (if FACK is set).
 *  0b1..No acknowledge signal is sent to the bus on the following receiving data byte (if FACK is cleared) or the
 *       current receiving data byte (if FACK is set).
 */
#define I2C_C1_TXAK(x) (((uint16_t)(((uint16_t)(x)) << I2C_C1_TXAK_SHIFT)) & I2C_C1_TXAK_MASK)

#define I2C_C1_TX_MASK  (0x10U)
#define I2C_C1_TX_SHIFT (4U)
/*! TX - Transmit Mode Select
 *  0b0..Receive
 *  0b1..Transmit
 */
#define I2C_C1_TX(x) (((uint16_t)(((uint16_t)(x)) << I2C_C1_TX_SHIFT)) & I2C_C1_TX_MASK)

#define I2C_C1_MST_MASK  (0x20U)
#define I2C_C1_MST_SHIFT (5U)
/*! MST - Master Mode Select
 *  0b0..Slave mode
 *  0b1..Master mode
 */
#define I2C_C1_MST(x) (((uint16_t)(((uint16_t)(x)) << I2C_C1_MST_SHIFT)) & I2C_C1_MST_MASK)

#define I2C_C1_IICIE_MASK  (0x40U)
#define I2C_C1_IICIE_SHIFT (6U)
/*! IICIE - I2C Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define I2C_C1_IICIE(x) (((uint16_t)(((uint16_t)(x)) << I2C_C1_IICIE_SHIFT)) & I2C_C1_IICIE_MASK)

#define I2C_C1_IICEN_MASK  (0x80U)
#define I2C_C1_IICEN_SHIFT (7U)
/*! IICEN - I2C Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define I2C_C1_IICEN(x) (((uint16_t)(((uint16_t)(x)) << I2C_C1_IICEN_SHIFT)) & I2C_C1_IICEN_MASK)
/*! @} */

/*! @name S - I2C Status register */
/*! @{ */

#define I2C_S_RXAK_MASK  (0x1U)
#define I2C_S_RXAK_SHIFT (0U)
/*! RXAK - Receive Acknowledge
 *  0b0..Acknowledge signal was received after the completion of one byte of data transmission on the bus
 *  0b1..No acknowledge signal detected
 */
#define I2C_S_RXAK(x) (((uint16_t)(((uint16_t)(x)) << I2C_S_RXAK_SHIFT)) & I2C_S_RXAK_MASK)

#define I2C_S_IICIF_MASK  (0x2U)
#define I2C_S_IICIF_SHIFT (1U)
/*! IICIF - Interrupt Flag
 *  0b0..No interrupt pending
 *  0b1..Interrupt pending
 */
#define I2C_S_IICIF(x) (((uint16_t)(((uint16_t)(x)) << I2C_S_IICIF_SHIFT)) & I2C_S_IICIF_MASK)

#define I2C_S_SRW_MASK  (0x4U)
#define I2C_S_SRW_SHIFT (2U)
/*! SRW - Slave Read/Write
 *  0b0..Slave receive, master writing to slave
 *  0b1..Slave transmit, master reading from slave
 */
#define I2C_S_SRW(x) (((uint16_t)(((uint16_t)(x)) << I2C_S_SRW_SHIFT)) & I2C_S_SRW_MASK)

#define I2C_S_RAM_MASK  (0x8U)
#define I2C_S_RAM_SHIFT (3U)
/*! RAM - Range Address Match
 *  0b0..Not addressed
 *  0b1..Addressed as a slave
 */
#define I2C_S_RAM(x) (((uint16_t)(((uint16_t)(x)) << I2C_S_RAM_SHIFT)) & I2C_S_RAM_MASK)

#define I2C_S_ARBL_MASK  (0x10U)
#define I2C_S_ARBL_SHIFT (4U)
/*! ARBL - Arbitration Lost
 *  0b0..Standard bus operation.
 *  0b1..Loss of arbitration.
 */
#define I2C_S_ARBL(x) (((uint16_t)(((uint16_t)(x)) << I2C_S_ARBL_SHIFT)) & I2C_S_ARBL_MASK)

#define I2C_S_BUSY_MASK  (0x20U)
#define I2C_S_BUSY_SHIFT (5U)
/*! BUSY - Bus Busy
 *  0b0..Bus is idle
 *  0b1..Bus is busy
 */
#define I2C_S_BUSY(x) (((uint16_t)(((uint16_t)(x)) << I2C_S_BUSY_SHIFT)) & I2C_S_BUSY_MASK)

#define I2C_S_IAAS_MASK  (0x40U)
#define I2C_S_IAAS_SHIFT (6U)
/*! IAAS - Addressed As A Slave
 *  0b0..Not addressed
 *  0b1..Addressed as a slave
 */
#define I2C_S_IAAS(x) (((uint16_t)(((uint16_t)(x)) << I2C_S_IAAS_SHIFT)) & I2C_S_IAAS_MASK)

#define I2C_S_TCF_MASK  (0x80U)
#define I2C_S_TCF_SHIFT (7U)
/*! TCF - Transfer Complete Flag
 *  0b0..Transfer in progress
 *  0b1..Transfer complete
 */
#define I2C_S_TCF(x) (((uint16_t)(((uint16_t)(x)) << I2C_S_TCF_SHIFT)) & I2C_S_TCF_MASK)
/*! @} */

/*! @name D - I2C Data I/O register */
/*! @{ */

#define I2C_D_DATA_MASK  (0xFFU)
#define I2C_D_DATA_SHIFT (0U)
/*! DATA - Data
 */
#define I2C_D_DATA(x) (((uint16_t)(((uint16_t)(x)) << I2C_D_DATA_SHIFT)) & I2C_D_DATA_MASK)
/*! @} */

/*! @name C2 - I2C Control Register 2 */
/*! @{ */

#define I2C_C2_AD_MASK  (0x7U)
#define I2C_C2_AD_SHIFT (0U)
/*! AD - Slave Address
 */
#define I2C_C2_AD(x) (((uint16_t)(((uint16_t)(x)) << I2C_C2_AD_SHIFT)) & I2C_C2_AD_MASK)

#define I2C_C2_RMEN_MASK  (0x8U)
#define I2C_C2_RMEN_SHIFT (3U)
/*! RMEN - Range Address Matching Enable
 *  0b0..Range mode disabled. No address matching occurs for an address within the range of values of the A1 and RA
 * registers. 0b1..Range mode enabled. Address matching occurs when a slave receives an address within the range of
 * values of the A1 and RA registers.
 */
#define I2C_C2_RMEN(x) (((uint16_t)(((uint16_t)(x)) << I2C_C2_RMEN_SHIFT)) & I2C_C2_RMEN_MASK)

#define I2C_C2_HDRS_MASK  (0x20U)
#define I2C_C2_HDRS_SHIFT (5U)
/*! HDRS - High Drive Select
 *  0b0..Normal drive mode
 *  0b1..High drive mode
 */
#define I2C_C2_HDRS(x) (((uint16_t)(((uint16_t)(x)) << I2C_C2_HDRS_SHIFT)) & I2C_C2_HDRS_MASK)

#define I2C_C2_ADEXT_MASK  (0x40U)
#define I2C_C2_ADEXT_SHIFT (6U)
/*! ADEXT - Address Extension
 *  0b0..7-bit address scheme
 *  0b1..10-bit address scheme
 */
#define I2C_C2_ADEXT(x) (((uint16_t)(((uint16_t)(x)) << I2C_C2_ADEXT_SHIFT)) & I2C_C2_ADEXT_MASK)

#define I2C_C2_GCAEN_MASK  (0x80U)
#define I2C_C2_GCAEN_SHIFT (7U)
/*! GCAEN - General Call Address Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define I2C_C2_GCAEN(x) (((uint16_t)(((uint16_t)(x)) << I2C_C2_GCAEN_SHIFT)) & I2C_C2_GCAEN_MASK)
/*! @} */

/*! @name FLT - I2C Programmable Input Glitch Filter Register */
/*! @{ */

#define I2C_FLT_FLT_MASK  (0xFU)
#define I2C_FLT_FLT_SHIFT (0U)
/*! FLT - I2C Programmable Filter Factor
 *  0b0000..No filter/bypass
 */
#define I2C_FLT_FLT(x) (((uint16_t)(((uint16_t)(x)) << I2C_FLT_FLT_SHIFT)) & I2C_FLT_FLT_MASK)

#define I2C_FLT_STARTF_MASK  (0x10U)
#define I2C_FLT_STARTF_SHIFT (4U)
/*! STARTF - I2C Bus Start Detect Flag
 *  0b0..No start happens on I2C bus
 *  0b1..Start detected on I2C bus
 */
#define I2C_FLT_STARTF(x) (((uint16_t)(((uint16_t)(x)) << I2C_FLT_STARTF_SHIFT)) & I2C_FLT_STARTF_MASK)

#define I2C_FLT_SSIE_MASK  (0x20U)
#define I2C_FLT_SSIE_SHIFT (5U)
/*! SSIE - I2C Bus Stop or Start Interrupt Enable
 *  0b0..Stop or start detection interrupt is disabled
 *  0b1..Stop or start detection interrupt is enabled
 */
#define I2C_FLT_SSIE(x) (((uint16_t)(((uint16_t)(x)) << I2C_FLT_SSIE_SHIFT)) & I2C_FLT_SSIE_MASK)

#define I2C_FLT_STOPF_MASK  (0x40U)
#define I2C_FLT_STOPF_SHIFT (6U)
/*! STOPF - I2C Bus Stop Detect Flag
 *  0b0..No stop happens on I2C bus
 *  0b1..Stop detected on I2C bus
 */
#define I2C_FLT_STOPF(x) (((uint16_t)(((uint16_t)(x)) << I2C_FLT_STOPF_SHIFT)) & I2C_FLT_STOPF_MASK)

#define I2C_FLT_SHEN_MASK  (0x80U)
#define I2C_FLT_SHEN_SHIFT (7U)
/*! SHEN - Stop Hold Enable
 *  0b0..Stop holdoff is disabled. The MCU's entry to stop mode is not gated.
 *  0b1..Stop holdoff is enabled.
 */
#define I2C_FLT_SHEN(x) (((uint16_t)(((uint16_t)(x)) << I2C_FLT_SHEN_SHIFT)) & I2C_FLT_SHEN_MASK)
/*! @} */

/*! @name RA - I2C Range Address register */
/*! @{ */

#define I2C_RA_RAD_MASK  (0xFEU)
#define I2C_RA_RAD_SHIFT (1U)
/*! RAD - Range Slave Address
 */
#define I2C_RA_RAD(x) (((uint16_t)(((uint16_t)(x)) << I2C_RA_RAD_SHIFT)) & I2C_RA_RAD_MASK)
/*! @} */

/*! @name SMB - I2C SMBus Control and Status register */
/*! @{ */

#define I2C_SMB_SHTF2IE_MASK  (0x1U)
#define I2C_SMB_SHTF2IE_SHIFT (0U)
/*! SHTF2IE - SHTF2 Interrupt Enable
 *  0b0..SHTF2 interrupt is disabled
 *  0b1..SHTF2 interrupt is enabled
 */
#define I2C_SMB_SHTF2IE(x) (((uint16_t)(((uint16_t)(x)) << I2C_SMB_SHTF2IE_SHIFT)) & I2C_SMB_SHTF2IE_MASK)

#define I2C_SMB_SHTF2_MASK  (0x2U)
#define I2C_SMB_SHTF2_SHIFT (1U)
/*! SHTF2 - SCL High Timeout Flag 2
 *  0b0..No SCL high and SDA low timeout occurs
 *  0b1..SCL high and SDA low timeout occurs
 */
#define I2C_SMB_SHTF2(x) (((uint16_t)(((uint16_t)(x)) << I2C_SMB_SHTF2_SHIFT)) & I2C_SMB_SHTF2_MASK)

#define I2C_SMB_SHTF1_MASK  (0x4U)
#define I2C_SMB_SHTF1_SHIFT (2U)
/*! SHTF1 - SCL High Timeout Flag 1
 *  0b0..No SCL high and SDA high timeout occurs
 *  0b1..SCL high and SDA high timeout occurs
 */
#define I2C_SMB_SHTF1(x) (((uint16_t)(((uint16_t)(x)) << I2C_SMB_SHTF1_SHIFT)) & I2C_SMB_SHTF1_MASK)

#define I2C_SMB_SLTF_MASK  (0x8U)
#define I2C_SMB_SLTF_SHIFT (3U)
/*! SLTF - SCL Low Timeout Flag
 *  0b0..No low timeout occurs
 *  0b1..Low timeout occurs
 */
#define I2C_SMB_SLTF(x) (((uint16_t)(((uint16_t)(x)) << I2C_SMB_SLTF_SHIFT)) & I2C_SMB_SLTF_MASK)

#define I2C_SMB_TCKSEL_MASK  (0x10U)
#define I2C_SMB_TCKSEL_SHIFT (4U)
/*! TCKSEL - Timeout Counter Clock Select
 *  0b0..Timeout counter counts at the frequency of the I2C module clock / 64
 *  0b1..Timeout counter counts at the frequency of the I2C module clock
 */
#define I2C_SMB_TCKSEL(x) (((uint16_t)(((uint16_t)(x)) << I2C_SMB_TCKSEL_SHIFT)) & I2C_SMB_TCKSEL_MASK)

#define I2C_SMB_SIICAEN_MASK  (0x20U)
#define I2C_SMB_SIICAEN_SHIFT (5U)
/*! SIICAEN - Second I2C Address Enable
 *  0b0..I2C address register 2 matching is disabled
 *  0b1..I2C address register 2 matching is enabled
 */
#define I2C_SMB_SIICAEN(x) (((uint16_t)(((uint16_t)(x)) << I2C_SMB_SIICAEN_SHIFT)) & I2C_SMB_SIICAEN_MASK)

#define I2C_SMB_ALERTEN_MASK  (0x40U)
#define I2C_SMB_ALERTEN_SHIFT (6U)
/*! ALERTEN - SMBus Alert Response Address Enable
 *  0b0..SMBus alert response address matching is disabled
 *  0b1..SMBus alert response address matching is enabled
 */
#define I2C_SMB_ALERTEN(x) (((uint16_t)(((uint16_t)(x)) << I2C_SMB_ALERTEN_SHIFT)) & I2C_SMB_ALERTEN_MASK)

#define I2C_SMB_FACK_MASK  (0x80U)
#define I2C_SMB_FACK_SHIFT (7U)
/*! FACK - Fast NACK/ACK Enable
 *  0b0..An ACK or NACK is sent on the following receiving data byte
 *  0b1..Writing 0 to TXAK after receiving a data byte generates an ACK. Writing 1 to TXAK after receiving a data byte
 * generates a NACK.
 */
#define I2C_SMB_FACK(x) (((uint16_t)(((uint16_t)(x)) << I2C_SMB_FACK_SHIFT)) & I2C_SMB_FACK_MASK)
/*! @} */

/*! @name A2 - I2C Address Register 2 */
/*! @{ */

#define I2C_A2_SAD_MASK  (0xFEU)
#define I2C_A2_SAD_SHIFT (1U)
/*! SAD - SMBus Address
 */
#define I2C_A2_SAD(x) (((uint16_t)(((uint16_t)(x)) << I2C_A2_SAD_SHIFT)) & I2C_A2_SAD_MASK)
/*! @} */

/*! @name SLTH - I2C SCL Low Timeout Register High */
/*! @{ */

#define I2C_SLTH_SSLT_MASK  (0xFFU)
#define I2C_SLTH_SSLT_SHIFT (0U)
/*! SSLT - SSLT[15:8]
 */
#define I2C_SLTH_SSLT(x) (((uint16_t)(((uint16_t)(x)) << I2C_SLTH_SSLT_SHIFT)) & I2C_SLTH_SSLT_MASK)
/*! @} */

/*! @name SLTL - I2C SCL Low Timeout Register Low */
/*! @{ */

#define I2C_SLTL_SSLT_MASK  (0xFFU)
#define I2C_SLTL_SSLT_SHIFT (0U)
/*! SSLT - SSLT[7:0]
 */
#define I2C_SLTL_SSLT(x) (((uint16_t)(((uint16_t)(x)) << I2C_SLTL_SSLT_SHIFT)) & I2C_SLTL_SSLT_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group I2C_Register_Masks */

/* I2C - Peripheral instance base addresses */
/** Peripheral I2C0 base address */
#define I2C0_BASE (0xE0E0u)
/** Peripheral I2C0 base pointer */
#define I2C0 ((I2C_Type *)I2C0_BASE)
/** Peripheral I2C1 base address */
#define I2C1_BASE (0xE0F0u)
/** Peripheral I2C1 base pointer */
#define I2C1 ((I2C_Type *)I2C1_BASE)
/** Array initializer of I2C peripheral base addresses */
#define I2C_BASE_ADDRS       \
    {                        \
        I2C0_BASE, I2C1_BASE \
    }
/** Array initializer of I2C peripheral base pointers */
#define I2C_BASE_PTRS \
    {                 \
        I2C0, I2C1    \
    }
/** Interrupt vectors for the I2C peripheral type */
#define I2C_IRQS             \
    {                        \
        I2C0_IRQn, I2C1_IRQn \
    }

/*!
 * @}
 */ /* end of group I2C_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- INTC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup INTC_Peripheral_Access_Layer INTC Peripheral Access Layer
 * @{
 */

/** INTC - Register Layout Typedef */
typedef struct
{
    __IO uint16_t IPR0;   /**< Interrupt Priority Register 0, offset: 0x0 */
    __IO uint16_t IPR1;   /**< Interrupt Priority Register 1, offset: 0x1 */
    __IO uint16_t IPR2;   /**< Interrupt Priority Register 2, offset: 0x2 */
    __IO uint16_t IPR3;   /**< Interrupt Priority Register 3, offset: 0x3 */
    __IO uint16_t IPR4;   /**< Interrupt Priority Register 4, offset: 0x4 */
    __IO uint16_t IPR5;   /**< Interrupt Priority Register 5, offset: 0x5 */
    __IO uint16_t IPR6;   /**< Interrupt Priority Register 6, offset: 0x6 */
    __IO uint16_t IPR7;   /**< Interrupt Priority Register 7, offset: 0x7 */
    __IO uint16_t IPR8;   /**< Interrupt Priority Register 8, offset: 0x8 */
    __IO uint16_t IPR9;   /**< Interrupt Priority Register 9, offset: 0x9 */
    __IO uint16_t IPR10;  /**< Interrupt Priority Register 10, offset: 0xA */
    __IO uint16_t IPR11;  /**< Interrupt Priority Register 11, offset: 0xB */
    __IO uint16_t IPR12;  /**< Interrupt Priority Register 12, offset: 0xC */
    __IO uint16_t VBA;    /**< Vector Base Address Register, offset: 0xD */
    __IO uint16_t FIM0;   /**< Fast Interrupt 0 Match Register, offset: 0xE */
    __IO uint16_t FIVAL0; /**< Fast Interrupt 0 Vector Address Low Register, offset: 0xF */
    __IO uint16_t FIVAH0; /**< Fast Interrupt 0 Vector Address High Register, offset: 0x10 */
    __IO uint16_t FIM1;   /**< Fast Interrupt 1 Match Register, offset: 0x11 */
    __IO uint16_t FIVAL1; /**< Fast Interrupt 1 Vector Address Low Register, offset: 0x12 */
    __IO uint16_t FIVAH1; /**< Fast Interrupt 1 Vector Address High Register, offset: 0x13 */
    __I uint16_t IRQP0;   /**< IRQ Pending Register 0, offset: 0x14 */
    __I uint16_t IRQP1;   /**< IRQ Pending Register 1, offset: 0x15 */
    __I uint16_t IRQP2;   /**< IRQ Pending Register 2, offset: 0x16 */
    __I uint16_t IRQP3;   /**< IRQ Pending Register 3, offset: 0x17 */
    __I uint16_t IRQP4;   /**< IRQ Pending Register 4, offset: 0x18 */
    __I uint16_t IRQP5;   /**< IRQ Pending Register 5, offset: 0x19 */
    __I uint16_t IRQP6;   /**< IRQ Pending Register 6, offset: 0x1A */
    __IO uint16_t CTRL;   /**< Control Register, offset: 0x1B */
} INTC_Type;

/* ----------------------------------------------------------------------------
   -- INTC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup INTC_Register_Masks INTC Register Masks
 * @{
 */

/*! @name IPR0 - Interrupt Priority Register 0 */
/*! @{ */

#define INTC_IPR0_STPCNT_MASK  (0x3U)
#define INTC_IPR0_STPCNT_SHIFT (0U)
/*! STPCNT - EOnCE Step Counter Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR0_STPCNT(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR0_STPCNT_SHIFT)) & INTC_IPR0_STPCNT_MASK)

#define INTC_IPR0_BKPT_MASK  (0xCU)
#define INTC_IPR0_BKPT_SHIFT (2U)
/*! BKPT - EOnCE Breakpoint Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR0_BKPT(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR0_BKPT_SHIFT)) & INTC_IPR0_BKPT_MASK)

#define INTC_IPR0_TRBUF_MASK  (0x30U)
#define INTC_IPR0_TRBUF_SHIFT (4U)
/*! TRBUF - EOnCE Trace Buffer Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR0_TRBUF(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR0_TRBUF_SHIFT)) & INTC_IPR0_TRBUF_MASK)

#define INTC_IPR0_TX_REG_MASK  (0xC0U)
#define INTC_IPR0_TX_REG_SHIFT (6U)
/*! TX_REG - EOnCE Transmit Register Empty Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR0_TX_REG(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR0_TX_REG_SHIFT)) & INTC_IPR0_TX_REG_MASK)

#define INTC_IPR0_RX_REG_MASK  (0x300U)
#define INTC_IPR0_RX_REG_SHIFT (8U)
/*! RX_REG - EOnCE Receive Register Full Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR0_RX_REG(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR0_RX_REG_SHIFT)) & INTC_IPR0_RX_REG_MASK)

#define INTC_IPR0_BUS_ERR_MASK  (0xC00U)
#define INTC_IPR0_BUS_ERR_SHIFT (10U)
/*! BUS_ERR - Bus Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR0_BUS_ERR(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR0_BUS_ERR_SHIFT)) & INTC_IPR0_BUS_ERR_MASK)
/*! @} */

/*! @name IPR1 - Interrupt Priority Register 1 */
/*! @{ */

#define INTC_IPR1_XBARA_MASK  (0xCU)
#define INTC_IPR1_XBARA_SHIFT (2U)
/*! XBARA - Inter-Peripheral Crossbar Switch A (XBARA) Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR1_XBARA(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_XBARA_SHIFT)) & INTC_IPR1_XBARA_MASK)

#define INTC_IPR1_LVI1_MASK  (0x30U)
#define INTC_IPR1_LVI1_SHIFT (4U)
/*! LVI1 - Low Voltage Detector Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR1_LVI1(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_LVI1_SHIFT)) & INTC_IPR1_LVI1_MASK)

#define INTC_IPR1_OCCS_MASK  (0xC0U)
#define INTC_IPR1_OCCS_SHIFT (6U)
/*! OCCS - PLL Loss of Reference or Change in Lock Status Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR1_OCCS(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_OCCS_SHIFT)) & INTC_IPR1_OCCS_MASK)

#define INTC_IPR1_TMRB_3_MASK  (0x300U)
#define INTC_IPR1_TMRB_3_SHIFT (8U)
/*! TMRB_3 - Timer B Channel 3 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR1_TMRB_3(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_TMRB_3_SHIFT)) & INTC_IPR1_TMRB_3_MASK)

#define INTC_IPR1_TMRB_2_MASK  (0xC00U)
#define INTC_IPR1_TMRB_2_SHIFT (10U)
/*! TMRB_2 - Timer B Channel 2 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR1_TMRB_2(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_TMRB_2_SHIFT)) & INTC_IPR1_TMRB_2_MASK)

#define INTC_IPR1_TMRB_1_MASK  (0x3000U)
#define INTC_IPR1_TMRB_1_SHIFT (12U)
/*! TMRB_1 - Timer B Channel 1 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR1_TMRB_1(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_TMRB_1_SHIFT)) & INTC_IPR1_TMRB_1_MASK)

#define INTC_IPR1_TMRB_0_MASK  (0xC000U)
#define INTC_IPR1_TMRB_0_SHIFT (14U)
/*! TMRB_0 - Timer B Channel 0 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR1_TMRB_0(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_TMRB_0_SHIFT)) & INTC_IPR1_TMRB_0_MASK)
/*! @} */

/*! @name IPR2 - Interrupt Priority Register 2 */
/*! @{ */

#define INTC_IPR2_TMRA_3_MASK  (0x3U)
#define INTC_IPR2_TMRA_3_SHIFT (0U)
/*! TMRA_3 - Timer A Channel 3 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_TMRA_3(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_TMRA_3_SHIFT)) & INTC_IPR2_TMRA_3_MASK)

#define INTC_IPR2_TMRA_2_MASK  (0xCU)
#define INTC_IPR2_TMRA_2_SHIFT (2U)
/*! TMRA_2 - Timer A Channel 2 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_TMRA_2(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_TMRA_2_SHIFT)) & INTC_IPR2_TMRA_2_MASK)

#define INTC_IPR2_TMRA_1_MASK  (0x30U)
#define INTC_IPR2_TMRA_1_SHIFT (4U)
/*! TMRA_1 - Timer A Channel 1 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_TMRA_1(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_TMRA_1_SHIFT)) & INTC_IPR2_TMRA_1_MASK)

#define INTC_IPR2_TMRA_0_MASK  (0xC0U)
#define INTC_IPR2_TMRA_0_SHIFT (6U)
/*! TMRA_0 - Timer A Channel 0 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_TMRA_0(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_TMRA_0_SHIFT)) & INTC_IPR2_TMRA_0_MASK)

#define INTC_IPR2_ADC_CC1_MASK  (0x300U)
#define INTC_IPR2_ADC_CC1_SHIFT (8U)
/*! ADC_CC1 - ADC_CYC Conversion Complete Interrupt Priority Level (converter B in non-simultaneous parallel scan mode)
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_ADC_CC1(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_ADC_CC1_SHIFT)) & INTC_IPR2_ADC_CC1_MASK)

#define INTC_IPR2_ADC_CC0_MASK  (0xC00U)
#define INTC_IPR2_ADC_CC0_SHIFT (10U)
/*! ADC_CC0 - ADC_CYC Conversion Complete Interrupt Priority Level (any scan type except converter B
 *    in non-simultaneous parallel scan mode)
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_ADC_CC0(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_ADC_CC0_SHIFT)) & INTC_IPR2_ADC_CC0_MASK)

#define INTC_IPR2_ADC_ERR_MASK  (0x3000U)
#define INTC_IPR2_ADC_ERR_SHIFT (12U)
/*! ADC_ERR - ADC_CYC Zero Crossing, High Limit, or Low Limit Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_ADC_ERR(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_ADC_ERR_SHIFT)) & INTC_IPR2_ADC_ERR_MASK)

#define INTC_IPR2_DMA_ERR_MASK  (0xC000U)
#define INTC_IPR2_DMA_ERR_SHIFT (14U)
/*! DMA_ERR - DMA Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_DMA_ERR(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_DMA_ERR_SHIFT)) & INTC_IPR2_DMA_ERR_MASK)
/*! @} */

/*! @name IPR3 - Interrupt Priority Register 3 */
/*! @{ */

#define INTC_IPR3_DMACH3_MASK  (0x3U)
#define INTC_IPR3_DMACH3_SHIFT (0U)
/*! DMACH3 - DMA Channel 3 Service Request Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_DMACH3(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_DMACH3_SHIFT)) & INTC_IPR3_DMACH3_MASK)

#define INTC_IPR3_DMACH2_MASK  (0xCU)
#define INTC_IPR3_DMACH2_SHIFT (2U)
/*! DMACH2 - DMA Channel 2 Service Request Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_DMACH2(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_DMACH2_SHIFT)) & INTC_IPR3_DMACH2_MASK)

#define INTC_IPR3_DMACH1_MASK  (0x30U)
#define INTC_IPR3_DMACH1_SHIFT (4U)
/*! DMACH1 - DMA Channel 1 Service Request Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_DMACH1(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_DMACH1_SHIFT)) & INTC_IPR3_DMACH1_MASK)

#define INTC_IPR3_DMACH0_MASK  (0xC0U)
#define INTC_IPR3_DMACH0_SHIFT (6U)
/*! DMACH0 - DMA Channel 0 Service Request Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_DMACH0(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_DMACH0_SHIFT)) & INTC_IPR3_DMACH0_MASK)

#define INTC_IPR3_CAN_MB_OR_MASK  (0x300U)
#define INTC_IPR3_CAN_MB_OR_SHIFT (8U)
/*! CAN_MB_OR - CAN Message Buffer Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_CAN_MB_OR(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_CAN_MB_OR_SHIFT)) & INTC_IPR3_CAN_MB_OR_MASK)

#define INTC_IPR3_CAN_BUS_OFF_MASK  (0xC00U)
#define INTC_IPR3_CAN_BUS_OFF_SHIFT (10U)
/*! CAN_BUS_OFF - CAN Bus Off Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_CAN_BUS_OFF(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_CAN_BUS_OFF_SHIFT)) & INTC_IPR3_CAN_BUS_OFF_MASK)

#define INTC_IPR3_CAN_ERROR_MASK  (0x3000U)
#define INTC_IPR3_CAN_ERROR_SHIFT (12U)
/*! CAN_ERROR - CAN Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_CAN_ERROR(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_CAN_ERROR_SHIFT)) & INTC_IPR3_CAN_ERROR_MASK)

#define INTC_IPR3_CAN_TX_WARN_MASK  (0xC000U)
#define INTC_IPR3_CAN_TX_WARN_SHIFT (14U)
/*! CAN_TX_WARN - CAN Transmit Warning Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_CAN_TX_WARN(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_CAN_TX_WARN_SHIFT)) & INTC_IPR3_CAN_TX_WARN_MASK)
/*! @} */

/*! @name IPR4 - Interrupt Priority Register 4 */
/*! @{ */

#define INTC_IPR4_CAN_RX_WARN_MASK  (0x3U)
#define INTC_IPR4_CAN_RX_WARN_SHIFT (0U)
/*! CAN_RX_WARN - CAN Receive Warning Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_CAN_RX_WARN(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_CAN_RX_WARN_SHIFT)) & INTC_IPR4_CAN_RX_WARN_MASK)

#define INTC_IPR4_CAN_WAKEUP_MASK  (0xCU)
#define INTC_IPR4_CAN_WAKEUP_SHIFT (2U)
/*! CAN_WAKEUP - CAN Wakeup Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_CAN_WAKEUP(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_CAN_WAKEUP_SHIFT)) & INTC_IPR4_CAN_WAKEUP_MASK)

#define INTC_IPR4_QSCI2_RERR_MASK  (0x30U)
#define INTC_IPR4_QSCI2_RERR_SHIFT (4U)
/*! QSCI2_RERR - QSCI 2 Receiver Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_QSCI2_RERR(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_QSCI2_RERR_SHIFT)) & INTC_IPR4_QSCI2_RERR_MASK)

#define INTC_IPR4_QSCI2_RCV_MASK  (0xC0U)
#define INTC_IPR4_QSCI2_RCV_SHIFT (6U)
/*! QSCI2_RCV - QSCI 2 Receive Data Register Full Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_QSCI2_RCV(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_QSCI2_RCV_SHIFT)) & INTC_IPR4_QSCI2_RCV_MASK)

#define INTC_IPR4_QSCI2_TRIDLE_MASK  (0x300U)
#define INTC_IPR4_QSCI2_TRIDLE_SHIFT (8U)
/*! QSCI2_TRIDLE - QSCI 2 Transmitter and Receiver Idle Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_QSCI2_TRIDLE(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_QSCI2_TRIDLE_SHIFT)) & INTC_IPR4_QSCI2_TRIDLE_MASK)

#define INTC_IPR4_QSCI2_TDRE_MASK  (0xC00U)
#define INTC_IPR4_QSCI2_TDRE_SHIFT (10U)
/*! QSCI2_TDRE - QSCI 2 Transmit Data Register Empty Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_QSCI2_TDRE(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_QSCI2_TDRE_SHIFT)) & INTC_IPR4_QSCI2_TDRE_MASK)

#define INTC_IPR4_QSCI1_RERR_MASK  (0x3000U)
#define INTC_IPR4_QSCI1_RERR_SHIFT (12U)
/*! QSCI1_RERR - QSCI 1 Receiver Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_QSCI1_RERR(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_QSCI1_RERR_SHIFT)) & INTC_IPR4_QSCI1_RERR_MASK)

#define INTC_IPR4_QSCI1_RCV_MASK  (0xC000U)
#define INTC_IPR4_QSCI1_RCV_SHIFT (14U)
/*! QSCI1_RCV - QSCI 1 Receive Data Register Full Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_QSCI1_RCV(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_QSCI1_RCV_SHIFT)) & INTC_IPR4_QSCI1_RCV_MASK)
/*! @} */

/*! @name IPR5 - Interrupt Priority Register 5 */
/*! @{ */

#define INTC_IPR5_QSCI1_TRIDLE_MASK  (0x3U)
#define INTC_IPR5_QSCI1_TRIDLE_SHIFT (0U)
/*! QSCI1_TRIDLE - QSCI 1 Transmitter and Receiver Idle Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR5_QSCI1_TRIDLE(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR5_QSCI1_TRIDLE_SHIFT)) & INTC_IPR5_QSCI1_TRIDLE_MASK)

#define INTC_IPR5_QSCI1_TDRE_MASK  (0xCU)
#define INTC_IPR5_QSCI1_TDRE_SHIFT (2U)
/*! QSCI1_TDRE - QSCI 1 Transmit Data Register Empty Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR5_QSCI1_TDRE(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR5_QSCI1_TDRE_SHIFT)) & INTC_IPR5_QSCI1_TDRE_MASK)

#define INTC_IPR5_QSCI0_RERR_MASK  (0x30U)
#define INTC_IPR5_QSCI0_RERR_SHIFT (4U)
/*! QSCI0_RERR - QSCI0 Receiver Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR5_QSCI0_RERR(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR5_QSCI0_RERR_SHIFT)) & INTC_IPR5_QSCI0_RERR_MASK)

#define INTC_IPR5_QSCI0_RCV_MASK  (0xC0U)
#define INTC_IPR5_QSCI0_RCV_SHIFT (6U)
/*! QSCI0_RCV - QSCI 0 Receive Data Register Full Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR5_QSCI0_RCV(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR5_QSCI0_RCV_SHIFT)) & INTC_IPR5_QSCI0_RCV_MASK)

#define INTC_IPR5_QSCI0_TRIDLE_MASK  (0x300U)
#define INTC_IPR5_QSCI0_TRIDLE_SHIFT (8U)
/*! QSCI0_TRIDLE - QSCI 0 Transmitter and Receiver Idle Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR5_QSCI0_TRIDLE(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR5_QSCI0_TRIDLE_SHIFT)) & INTC_IPR5_QSCI0_TRIDLE_MASK)

#define INTC_IPR5_QSCI0_TDRE_MASK  (0xC00U)
#define INTC_IPR5_QSCI0_TDRE_SHIFT (10U)
/*! QSCI0_TDRE - QSCI 0 Transmit Data Register Empty Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR5_QSCI0_TDRE(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR5_QSCI0_TDRE_SHIFT)) & INTC_IPR5_QSCI0_TDRE_MASK)
/*! @} */

/*! @name IPR6 - Interrupt Priority Register 6 */
/*! @{ */

#define INTC_IPR6_QSPI1_XMIT_MASK  (0x3U)
#define INTC_IPR6_QSPI1_XMIT_SHIFT (0U)
/*! QSPI1_XMIT - QSPI1 Transmitter Empty Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR6_QSPI1_XMIT(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR6_QSPI1_XMIT_SHIFT)) & INTC_IPR6_QSPI1_XMIT_MASK)

#define INTC_IPR6_QSPI1_RCV_MASK  (0xCU)
#define INTC_IPR6_QSPI1_RCV_SHIFT (2U)
/*! QSPI1_RCV - QSPI1 Receiver Full Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR6_QSPI1_RCV(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR6_QSPI1_RCV_SHIFT)) & INTC_IPR6_QSPI1_RCV_MASK)

#define INTC_IPR6_QSPI0_XMIT_MASK  (0x30U)
#define INTC_IPR6_QSPI0_XMIT_SHIFT (4U)
/*! QSPI0_XMIT - QSPI0 Transmitter Empty Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR6_QSPI0_XMIT(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR6_QSPI0_XMIT_SHIFT)) & INTC_IPR6_QSPI0_XMIT_MASK)

#define INTC_IPR6_QSPI0_RCV_MASK  (0xC0U)
#define INTC_IPR6_QSPI0_RCV_SHIFT (6U)
/*! QSPI0_RCV - QSPI0 Receiver Full Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR6_QSPI0_RCV(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR6_QSPI0_RCV_SHIFT)) & INTC_IPR6_QSPI0_RCV_MASK)

#define INTC_IPR6_IIC1_MASK  (0x300U)
#define INTC_IPR6_IIC1_SHIFT (8U)
/*! IIC1 - IIC1 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR6_IIC1(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR6_IIC1_SHIFT)) & INTC_IPR6_IIC1_MASK)

#define INTC_IPR6_IIC0_MASK  (0xC00U)
#define INTC_IPR6_IIC0_SHIFT (10U)
/*! IIC0 - IIC0 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR6_IIC0(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR6_IIC0_SHIFT)) & INTC_IPR6_IIC0_MASK)
/*! @} */

/*! @name IPR7 - Interrupt Priority Register 7 */
/*! @{ */

#define INTC_IPR7_PWMB_FAULT_MASK  (0xCU)
#define INTC_IPR7_PWMB_FAULT_SHIFT (2U)
/*! PWMB_FAULT - PWMB Fault Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_FAULT(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_FAULT_SHIFT)) & INTC_IPR7_PWMB_FAULT_MASK)

#define INTC_IPR7_PWMB_RERR_MASK  (0x30U)
#define INTC_IPR7_PWMB_RERR_SHIFT (4U)
/*! PWMB_RERR - PWMB Reload Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_RERR(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_RERR_SHIFT)) & INTC_IPR7_PWMB_RERR_MASK)

#define INTC_IPR7_PWMB_CAP_MASK  (0xC0U)
#define INTC_IPR7_PWMB_CAP_SHIFT (6U)
/*! PWMB_CAP - PWMB Submodule Capture Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_CAP(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_CAP_SHIFT)) & INTC_IPR7_PWMB_CAP_MASK)

#define INTC_IPR7_PWMB_RELOAD3_MASK  (0x300U)
#define INTC_IPR7_PWMB_RELOAD3_SHIFT (8U)
/*! PWMB_RELOAD3 - PWMB Submodule 3 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_RELOAD3(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_RELOAD3_SHIFT)) & INTC_IPR7_PWMB_RELOAD3_MASK)

#define INTC_IPR7_PWMB_CMP3_MASK  (0xC00U)
#define INTC_IPR7_PWMB_CMP3_SHIFT (10U)
/*! PWMB_CMP3 - PWMB Submodule 3 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_CMP3(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_CMP3_SHIFT)) & INTC_IPR7_PWMB_CMP3_MASK)

#define INTC_IPR7_PWMB_RELOAD2_MASK  (0x3000U)
#define INTC_IPR7_PWMB_RELOAD2_SHIFT (12U)
/*! PWMB_RELOAD2 - PWMB Submodule 2 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_RELOAD2(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_RELOAD2_SHIFT)) & INTC_IPR7_PWMB_RELOAD2_MASK)

#define INTC_IPR7_PWMB_CMP2_MASK  (0xC000U)
#define INTC_IPR7_PWMB_CMP2_SHIFT (14U)
/*! PWMB_CMP2 - PWMB Submodule 2 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_CMP2(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_CMP2_SHIFT)) & INTC_IPR7_PWMB_CMP2_MASK)
/*! @} */

/*! @name IPR8 - Interrupt Priority Register 8 */
/*! @{ */

#define INTC_IPR8_PWMB_RELOAD1_MASK  (0x3U)
#define INTC_IPR8_PWMB_RELOAD1_SHIFT (0U)
/*! PWMB_RELOAD1 - PWMB Submodule 1 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMB_RELOAD1(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMB_RELOAD1_SHIFT)) & INTC_IPR8_PWMB_RELOAD1_MASK)

#define INTC_IPR8_PWMB_CMP1_MASK  (0xCU)
#define INTC_IPR8_PWMB_CMP1_SHIFT (2U)
/*! PWMB_CMP1 - PWMB Submodule 1 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMB_CMP1(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMB_CMP1_SHIFT)) & INTC_IPR8_PWMB_CMP1_MASK)

#define INTC_IPR8_PWMB_RELOAD0_MASK  (0x30U)
#define INTC_IPR8_PWMB_RELOAD0_SHIFT (4U)
/*! PWMB_RELOAD0 - PWMB Submodule 0 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMB_RELOAD0(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMB_RELOAD0_SHIFT)) & INTC_IPR8_PWMB_RELOAD0_MASK)

#define INTC_IPR8_PWMB_CMP0_MASK  (0xC0U)
#define INTC_IPR8_PWMB_CMP0_SHIFT (6U)
/*! PWMB_CMP0 - PWMB Submodule 0 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMB_CMP0(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMB_CMP0_SHIFT)) & INTC_IPR8_PWMB_CMP0_MASK)

#define INTC_IPR8_PWMA_FAULT_MASK  (0x300U)
#define INTC_IPR8_PWMA_FAULT_SHIFT (8U)
/*! PWMA_FAULT - PWMA Fault Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMA_FAULT(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMA_FAULT_SHIFT)) & INTC_IPR8_PWMA_FAULT_MASK)

#define INTC_IPR8_PWMA_RERR_MASK  (0xC00U)
#define INTC_IPR8_PWMA_RERR_SHIFT (10U)
/*! PWMA_RERR - PWMA Reload Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMA_RERR(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMA_RERR_SHIFT)) & INTC_IPR8_PWMA_RERR_MASK)

#define INTC_IPR8_PWMA_CAP_MASK  (0x3000U)
#define INTC_IPR8_PWMA_CAP_SHIFT (12U)
/*! PWMA_CAP - PWMA Submodule Capture Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMA_CAP(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMA_CAP_SHIFT)) & INTC_IPR8_PWMA_CAP_MASK)

#define INTC_IPR8_PWMA_RELOAD3_MASK  (0xC000U)
#define INTC_IPR8_PWMA_RELOAD3_SHIFT (14U)
/*! PWMA_RELOAD3 - PWMA Submodule 3 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMA_RELOAD3(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMA_RELOAD3_SHIFT)) & INTC_IPR8_PWMA_RELOAD3_MASK)
/*! @} */

/*! @name IPR9 - Interrupt Priority Register 9 */
/*! @{ */

#define INTC_IPR9_PWMA_CMP3_MASK  (0x3U)
#define INTC_IPR9_PWMA_CMP3_SHIFT (0U)
/*! PWMA_CMP3 - PWMA Submodule 3 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_CMP3(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_CMP3_SHIFT)) & INTC_IPR9_PWMA_CMP3_MASK)

#define INTC_IPR9_PWMA_RELOAD2_MASK  (0xCU)
#define INTC_IPR9_PWMA_RELOAD2_SHIFT (2U)
/*! PWMA_RELOAD2 - PWMA Submodule 2 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_RELOAD2(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_RELOAD2_SHIFT)) & INTC_IPR9_PWMA_RELOAD2_MASK)

#define INTC_IPR9_PWMA_CMP2_MASK  (0x30U)
#define INTC_IPR9_PWMA_CMP2_SHIFT (4U)
/*! PWMA_CMP2 - PWMA Submodule 2 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_CMP2(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_CMP2_SHIFT)) & INTC_IPR9_PWMA_CMP2_MASK)

#define INTC_IPR9_PWMA_RELOAD1_MASK  (0xC0U)
#define INTC_IPR9_PWMA_RELOAD1_SHIFT (6U)
/*! PWMA_RELOAD1 - PWMA Submodule 1 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_RELOAD1(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_RELOAD1_SHIFT)) & INTC_IPR9_PWMA_RELOAD1_MASK)

#define INTC_IPR9_PWMA_CMP1_MASK  (0x300U)
#define INTC_IPR9_PWMA_CMP1_SHIFT (8U)
/*! PWMA_CMP1 - PWMA Submodule 1 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_CMP1(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_CMP1_SHIFT)) & INTC_IPR9_PWMA_CMP1_MASK)

#define INTC_IPR9_PWMA_RELOAD0_MASK  (0xC00U)
#define INTC_IPR9_PWMA_RELOAD0_SHIFT (10U)
/*! PWMA_RELOAD0 - PWMA Submodule 0 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_RELOAD0(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_RELOAD0_SHIFT)) & INTC_IPR9_PWMA_RELOAD0_MASK)

#define INTC_IPR9_PWMA_CMP0_MASK  (0x3000U)
#define INTC_IPR9_PWMA_CMP0_SHIFT (12U)
/*! PWMA_CMP0 - PWMA Submodule 0 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_CMP0(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_CMP0_SHIFT)) & INTC_IPR9_PWMA_CMP0_MASK)

#define INTC_IPR9_FTFE_RDCOL_MASK  (0xC000U)
#define INTC_IPR9_FTFE_RDCOL_SHIFT (14U)
/*! FTFE_RDCOL - Flash module Access Error Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_FTFE_RDCOL(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_FTFE_RDCOL_SHIFT)) & INTC_IPR9_FTFE_RDCOL_MASK)
/*! @} */

/*! @name IPR10 - Interrupt Priority Register 10 */
/*! @{ */

#define INTC_IPR10_FTFE_CC_MASK  (0x3U)
#define INTC_IPR10_FTFE_CC_SHIFT (0U)
/*! FTFE_CC - Flash module Command Complete Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_FTFE_CC(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_FTFE_CC_SHIFT)) & INTC_IPR10_FTFE_CC_MASK)

#define INTC_IPR10_CMPD_MASK  (0xCU)
#define INTC_IPR10_CMPD_SHIFT (2U)
/*! CMPD - Comparator D Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_CMPD(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_CMPD_SHIFT)) & INTC_IPR10_CMPD_MASK)

#define INTC_IPR10_CMPC_MASK  (0x30U)
#define INTC_IPR10_CMPC_SHIFT (4U)
/*! CMPC - Comparator C Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_CMPC(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_CMPC_SHIFT)) & INTC_IPR10_CMPC_MASK)

#define INTC_IPR10_CMPB_MASK  (0xC0U)
#define INTC_IPR10_CMPB_SHIFT (6U)
/*! CMPB - Comparator B Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_CMPB(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_CMPB_SHIFT)) & INTC_IPR10_CMPB_MASK)

#define INTC_IPR10_CMPA_MASK  (0x300U)
#define INTC_IPR10_CMPA_SHIFT (8U)
/*! CMPA - Comparator A Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_CMPA(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_CMPA_SHIFT)) & INTC_IPR10_CMPA_MASK)

#define INTC_IPR10_PIT1_ROLLOVR_MASK  (0xC00U)
#define INTC_IPR10_PIT1_ROLLOVR_SHIFT (10U)
/*! PIT1_ROLLOVR - PIT1 Roll Over Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_PIT1_ROLLOVR(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_PIT1_ROLLOVR_SHIFT)) & INTC_IPR10_PIT1_ROLLOVR_MASK)

#define INTC_IPR10_PIT0_ROLLOVR_MASK  (0x3000U)
#define INTC_IPR10_PIT0_ROLLOVR_SHIFT (12U)
/*! PIT0_ROLLOVR - PIT0 Roll Over Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_PIT0_ROLLOVR(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_PIT0_ROLLOVR_SHIFT)) & INTC_IPR10_PIT0_ROLLOVR_MASK)

#define INTC_IPR10_FTFE_DFD_MASK  (0xC000U)
#define INTC_IPR10_FTFE_DFD_SHIFT (14U)
/*! FTFE_DFD - Flash module Double-bit Fault Detect Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_FTFE_DFD(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_FTFE_DFD_SHIFT)) & INTC_IPR10_FTFE_DFD_MASK)
/*! @} */

/*! @name IPR11 - Interrupt Priority Register 11 */
/*! @{ */

#define INTC_IPR11_GPIOG_MASK  (0x300U)
#define INTC_IPR11_GPIOG_SHIFT (8U)
/*! GPIOG - GPIO G Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR11_GPIOG(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR11_GPIOG_SHIFT)) & INTC_IPR11_GPIOG_MASK)

#define INTC_IPR11_GPIOF_MASK  (0xC00U)
#define INTC_IPR11_GPIOF_SHIFT (10U)
/*! GPIOF - GPIO F Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR11_GPIOF(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR11_GPIOF_SHIFT)) & INTC_IPR11_GPIOF_MASK)

#define INTC_IPR11_GPIOE_MASK  (0x3000U)
#define INTC_IPR11_GPIOE_SHIFT (12U)
/*! GPIOE - GPIO E Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR11_GPIOE(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR11_GPIOE_SHIFT)) & INTC_IPR11_GPIOE_MASK)

#define INTC_IPR11_GPIOD_MASK  (0xC000U)
#define INTC_IPR11_GPIOD_SHIFT (14U)
/*! GPIOD - GPIO D Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR11_GPIOD(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR11_GPIOD_SHIFT)) & INTC_IPR11_GPIOD_MASK)
/*! @} */

/*! @name IPR12 - Interrupt Priority Register 12 */
/*! @{ */

#define INTC_IPR12_GPIOC_MASK  (0x3U)
#define INTC_IPR12_GPIOC_SHIFT (0U)
/*! GPIOC - GPIO C Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR12_GPIOC(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR12_GPIOC_SHIFT)) & INTC_IPR12_GPIOC_MASK)

#define INTC_IPR12_GPIOB_MASK  (0xCU)
#define INTC_IPR12_GPIOB_SHIFT (2U)
/*! GPIOB - GPIO B Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR12_GPIOB(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR12_GPIOB_SHIFT)) & INTC_IPR12_GPIOB_MASK)

#define INTC_IPR12_GPIOA_MASK  (0x30U)
#define INTC_IPR12_GPIOA_SHIFT (4U)
/*! GPIOA - GPIO A Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR12_GPIOA(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR12_GPIOA_SHIFT)) & INTC_IPR12_GPIOA_MASK)

#define INTC_IPR12_COP_INT_MASK  (0xC0U)
#define INTC_IPR12_COP_INT_SHIFT (6U)
/*! COP_INT - COP Watchdog Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR12_COP_INT(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR12_COP_INT_SHIFT)) & INTC_IPR12_COP_INT_MASK)

#define INTC_IPR12_EWM_INT_MASK  (0x300U)
#define INTC_IPR12_EWM_INT_SHIFT (8U)
/*! EWM_INT - External Watchdog Monitor Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR12_EWM_INT(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR12_EWM_INT_SHIFT)) & INTC_IPR12_EWM_INT_MASK)

#define INTC_IPR12_USB_MASK  (0x3000U)
#define INTC_IPR12_USB_SHIFT (12U)
/*! USB - USB Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR12_USB(x) (((uint16_t)(((uint16_t)(x)) << INTC_IPR12_USB_SHIFT)) & INTC_IPR12_USB_MASK)
/*! @} */

/*! @name VBA - Vector Base Address Register */
/*! @{ */

#define INTC_VBA_VECTOR_BASE_ADDRESS_MASK  (0x1FFFU)
#define INTC_VBA_VECTOR_BASE_ADDRESS_SHIFT (0U)
/*! VECTOR_BASE_ADDRESS - Interrupt Vector Base Address
 */
#define INTC_VBA_VECTOR_BASE_ADDRESS(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_VBA_VECTOR_BASE_ADDRESS_SHIFT)) & INTC_VBA_VECTOR_BASE_ADDRESS_MASK)
/*! @} */

/*! @name FIM0 - Fast Interrupt 0 Match Register */
/*! @{ */

#define INTC_FIM0_FAST_INTERRUPT_0_MASK  (0x7FU)
#define INTC_FIM0_FAST_INTERRUPT_0_SHIFT (0U)
/*! FAST_INTERRUPT_0 - Fast Interrupt 0 Vector Number
 */
#define INTC_FIM0_FAST_INTERRUPT_0(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_FIM0_FAST_INTERRUPT_0_SHIFT)) & INTC_FIM0_FAST_INTERRUPT_0_MASK)
/*! @} */

/*! @name FIVAL0 - Fast Interrupt 0 Vector Address Low Register */
/*! @{ */

#define INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW_MASK  (0xFFFFU)
#define INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW_SHIFT (0U)
#define INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW_SHIFT)) & \
     INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW_MASK)
/*! @} */

/*! @name FIVAH0 - Fast Interrupt 0 Vector Address High Register */
/*! @{ */

#define INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_MASK  (0x1FU)
#define INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_SHIFT (0U)
/*! FI_0_VECTOR_ADDRESS_HIGH - Upper 5 bits of vector address for fast interrupt 0
 */
#define INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_SHIFT)) & \
     INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_MASK)
/*! @} */

/*! @name FIM1 - Fast Interrupt 1 Match Register */
/*! @{ */

#define INTC_FIM1_FAST_INTERRUPT_1_MASK  (0x7FU)
#define INTC_FIM1_FAST_INTERRUPT_1_SHIFT (0U)
/*! FAST_INTERRUPT_1 - Fast Interrupt 1 Vector Number
 */
#define INTC_FIM1_FAST_INTERRUPT_1(x) \
    (((uint16_t)(((uint16_t)(x)) << INTC_FIM1_FAST_INTERRUPT_1_SHIFT)) & INTC_FIM1_FAST_INTERRUPT_1_MASK)
/*! @} */

/*! @name FIVAL1 - Fast Interrupt 1 Vector Address Low Register */
/*! @{ */

#define INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW_MASK  (0xFFFFU)
#define INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW_SHIFT (0U)
/*! FI_1_VECTOR_ADDRESS_LOW - Lower 16 bits of vector address for fast interrupt 1
 */
#define INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW_SHIFT)) & \
     INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW_MASK)
/*! @} */

/*! @name FIVAH1 - Fast Interrupt 1 Vector Address High Register */
/*! @{ */

#define INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_MASK  (0x1FU)
#define INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_SHIFT (0U)
#define INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_SHIFT)) & \
     INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_MASK)
/*! @} */

/*! @name IRQP0 - IRQ Pending Register 0 */
/*! @{ */

#define INTC_IRQP0_PENDING_MASK  (0xFFFEU)
#define INTC_IRQP0_PENDING_SHIFT (1U)
/*! PENDING - Pending IRQs
 *  0b000000000000000..IRQ pending for this vector number
 *  0b000000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP0_PENDING(x) (((uint16_t)(((uint16_t)(x)) << INTC_IRQP0_PENDING_SHIFT)) & INTC_IRQP0_PENDING_MASK)
/*! @} */

/*! @name IRQP1 - IRQ Pending Register 1 */
/*! @{ */

#define INTC_IRQP1_PENDING_MASK  (0xFFFFU)
#define INTC_IRQP1_PENDING_SHIFT (0U)
/*! PENDING - Pending IRQs
 *  0b0000000000000000..IRQ pending for this vector number
 *  0b0000000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP1_PENDING(x) (((uint16_t)(((uint16_t)(x)) << INTC_IRQP1_PENDING_SHIFT)) & INTC_IRQP1_PENDING_MASK)
/*! @} */

/*! @name IRQP2 - IRQ Pending Register 2 */
/*! @{ */

#define INTC_IRQP2_PENDING_MASK  (0xFFFFU)
#define INTC_IRQP2_PENDING_SHIFT (0U)
/*! PENDING - Pending IRQs
 *  0b0000000000000000..IRQ pending for this vector number
 *  0b0000000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP2_PENDING(x) (((uint16_t)(((uint16_t)(x)) << INTC_IRQP2_PENDING_SHIFT)) & INTC_IRQP2_PENDING_MASK)
/*! @} */

/*! @name IRQP3 - IRQ Pending Register 3 */
/*! @{ */

#define INTC_IRQP3_PENDING_MASK  (0xFFFFU)
#define INTC_IRQP3_PENDING_SHIFT (0U)
/*! PENDING - Pending IRQs
 *  0b0000000000000000..IRQ pending for this vector number
 *  0b0000000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP3_PENDING(x) (((uint16_t)(((uint16_t)(x)) << INTC_IRQP3_PENDING_SHIFT)) & INTC_IRQP3_PENDING_MASK)
/*! @} */

/*! @name IRQP4 - IRQ Pending Register 4 */
/*! @{ */

#define INTC_IRQP4_PENDING_MASK  (0xFFFFU)
#define INTC_IRQP4_PENDING_SHIFT (0U)
/*! PENDING - Pending IRQs
 *  0b0000000000000000..IRQ pending for this vector number
 *  0b0000000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP4_PENDING(x) (((uint16_t)(((uint16_t)(x)) << INTC_IRQP4_PENDING_SHIFT)) & INTC_IRQP4_PENDING_MASK)
/*! @} */

/*! @name IRQP5 - IRQ Pending Register 5 */
/*! @{ */

#define INTC_IRQP5_PENDING_MASK  (0xFFFFU)
#define INTC_IRQP5_PENDING_SHIFT (0U)
/*! PENDING - Pending IRQs
 *  0b0000000000000000..IRQ pending for this vector number
 *  0b0000000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP5_PENDING(x) (((uint16_t)(((uint16_t)(x)) << INTC_IRQP5_PENDING_SHIFT)) & INTC_IRQP5_PENDING_MASK)
/*! @} */

/*! @name IRQP6 - IRQ Pending Register 6 */
/*! @{ */

#define INTC_IRQP6_PENDING_MASK  (0x3FFFU)
#define INTC_IRQP6_PENDING_SHIFT (0U)
/*! PENDING - Pending IRQs
 *  0b00000000000000..IRQ pending for this vector number
 *  0b00000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP6_PENDING(x) (((uint16_t)(((uint16_t)(x)) << INTC_IRQP6_PENDING_SHIFT)) & INTC_IRQP6_PENDING_MASK)
/*! @} */

/*! @name CTRL - Control Register */
/*! @{ */

#define INTC_CTRL_INT_DIS_MASK  (0x20U)
#define INTC_CTRL_INT_DIS_SHIFT (5U)
/*! INT_DIS - Interrupt disable
 *  0b0..Normal operation. (default)
 *  0b1..All interrupts disabled.
 */
#define INTC_CTRL_INT_DIS(x) (((uint16_t)(((uint16_t)(x)) << INTC_CTRL_INT_DIS_SHIFT)) & INTC_CTRL_INT_DIS_MASK)

#define INTC_CTRL_VAB_MASK  (0x1FC0U)
#define INTC_CTRL_VAB_SHIFT (6U)
/*! VAB - Vector number
 */
#define INTC_CTRL_VAB(x) (((uint16_t)(((uint16_t)(x)) << INTC_CTRL_VAB_SHIFT)) & INTC_CTRL_VAB_MASK)

#define INTC_CTRL_IPIC_MASK  (0x6000U)
#define INTC_CTRL_IPIC_SHIFT (13U)
/*! IPIC - Interrupt Priority Level
 *  0b00..Required nested exception priority levels are 0, 1, 2, or 3.
 *  0b01..Required nested exception priority levels are 1, 2, or 3.
 *  0b10..Required nested exception priority levels are 2 or3.
 *  0b11..Required nested exception priority level is 3.
 */
#define INTC_CTRL_IPIC(x) (((uint16_t)(((uint16_t)(x)) << INTC_CTRL_IPIC_SHIFT)) & INTC_CTRL_IPIC_MASK)

#define INTC_CTRL_INT_MASK  (0x8000U)
#define INTC_CTRL_INT_SHIFT (15U)
/*! INT - Interrupt
 *  0b0..No interrupt is being sent to the core.
 *  0b1..An interrupt is being sent to the core.
 */
#define INTC_CTRL_INT(x) (((uint16_t)(((uint16_t)(x)) << INTC_CTRL_INT_SHIFT)) & INTC_CTRL_INT_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group INTC_Register_Masks */

/* INTC - Peripheral instance base addresses */
/** Peripheral INTC base address */
#define INTC_BASE (0xE300u)
/** Peripheral INTC base pointer */
#define INTC ((INTC_Type *)INTC_BASE)
/** Array initializer of INTC peripheral base addresses */
#define INTC_BASE_ADDRS \
    {                   \
        INTC_BASE       \
    }
/** Array initializer of INTC peripheral base pointers */
#define INTC_BASE_PTRS \
    {                  \
        INTC           \
    }

/*!
 * @}
 */ /* end of group INTC_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- MCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Peripheral_Access_Layer MCM Peripheral Access Layer
 * @{
 */

/** MCM - Register Layout Typedef */
typedef struct
{
    uint8_t RESERVED_0[8];
    __I uint16_t PLASC; /**< Crossbar switch (AXBS) slave configuration, offset: 0x8 */
    __I uint16_t PLAMC; /**< Crossbar switch (AXBS) master configuration, offset: 0xA */
    __IO uint32_t CPCR; /**< Core control register, offset: 0xC */
    __I uint32_t CFADR; /**< Core fault address register, offset: 0x10 */
    __I uint8_t CFATR;  /**< Core fault attributes register, offset: 0x14 */
    __I uint8_t CFLOC;  /**< Core fault location register, offset: 0x15 */
    __IO uint8_t CFIER; /**< Core fault interrupt enable register, offset: 0x16 */
    __IO uint8_t CFISR; /**< MCM interrupt status register, offset: 0x17 */
    __I uint32_t CFDTR; /**< Core fault data register, offset: 0x18 */
    uint8_t RESERVED_1[4];
    __IO uint32_t RPCR;      /**< Resource Protection Control Register, offset: 0x20 */
    __IO uint32_t UFLASHBAR; /**< User Flash Base Address Register, offset: 0x24 */
    __IO uint32_t UPRAMBAR;  /**< User Program RAM Base Address Register, offset: 0x28 */
    __IO uint32_t UBROMBAR;  /**< User Boot ROM Base Address Register, offset: 0x2C */
    __IO uint32_t SRPOSP;    /**< Resource Protection Other Stack Pointer, offset: 0x30 */
    __IO uint32_t SRPIPC;    /**< Memory Protection Illegal PC, offset: 0x34 */
    __IO uint32_t SRPMPC;    /**< Resource Protection Misaligned PC, offset: 0x38 */
} MCM_Type;

/* ----------------------------------------------------------------------------
   -- MCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Masks MCM Register Masks
 * @{
 */

/*! @name PLASC - Crossbar switch (AXBS) slave configuration */
/*! @{ */

#define MCM_PLASC_ASC_MASK  (0xFFU)
#define MCM_PLASC_ASC_SHIFT (0U)
/*! ASC - Each bit in the ASC field indicates if there is a corresponding connection to the AXBS
 *    slave input port. For this device, this field always read 0x0F.
 *  0b00000000..A bus slave connection to AXBS input port n is absent
 *  0b00000001..A bus slave connection to AXBS input port n is present
 */
#define MCM_PLASC_ASC(x) (((uint16_t)(((uint16_t)(x)) << MCM_PLASC_ASC_SHIFT)) & MCM_PLASC_ASC_MASK)

#define MCM_PLASC_DP64_MASK  (0x8000U)
#define MCM_PLASC_DP64_SHIFT (15U)
/*! DP64 - Indicates if the datapath is 32 or 64 bits wide
 *  0b0..Datapath width is 32 bits
 *  0b1..Datapath width is 64 bits
 */
#define MCM_PLASC_DP64(x) (((uint16_t)(((uint16_t)(x)) << MCM_PLASC_DP64_SHIFT)) & MCM_PLASC_DP64_MASK)
/*! @} */

/*! @name PLAMC - Crossbar switch (AXBS) master configuration */
/*! @{ */

#define MCM_PLAMC_AMC_MASK  (0xFFU)
#define MCM_PLAMC_AMC_SHIFT (0U)
/*! AMC - Each bit in the AMC field indicates if there is a corresponding connection to the AXBS
 *    master input port. For this device, this field always reads 0x0F.
 *  0b00000000..A bus master connection to AXBS input port n is absent
 *  0b00000001..A bus master connection to AXBS input port n is present
 */
#define MCM_PLAMC_AMC(x) (((uint16_t)(((uint16_t)(x)) << MCM_PLAMC_AMC_SHIFT)) & MCM_PLAMC_AMC_MASK)
/*! @} */

/*! @name CPCR - Core control register */
/*! @{ */

#define MCM_CPCR_FCCCLR_MASK  (0x400UL)
#define MCM_CPCR_FCCCLR_SHIFT (10UL)
/*! FCCCLR - Clear Flash Controller Cache
 *  0b0..Writing logic 0 to this field is ignored.
 *  0b1..Writing logic 1 to this field clears the cache, 1 cycle active.
 */
#define MCM_CPCR_FCCCLR(x) (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCCCLR_SHIFT)) & MCM_CPCR_FCCCLR_MASK)

#define MCM_CPCR_FCDCDIS_MASK  (0x800UL)
#define MCM_CPCR_FCDCDIS_SHIFT (11UL)
/*! FCDCDIS - Disable Flash Controller Data Caching
 *  0b0..enable
 *  0b1..disable
 */
#define MCM_CPCR_FCDCDIS(x) (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCDCDIS_SHIFT)) & MCM_CPCR_FCDCDIS_MASK)

#define MCM_CPCR_FCICDIS_MASK  (0x1000UL)
#define MCM_CPCR_FCICDIS_SHIFT (12UL)
/*! FCICDIS - Disable Flash Controller Instruction Caching
 *  0b0..enable
 *  0b1..disable
 */
#define MCM_CPCR_FCICDIS(x) (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCICDIS_SHIFT)) & MCM_CPCR_FCICDIS_MASK)

#define MCM_CPCR_FCCDIS_MASK  (0x2000UL)
#define MCM_CPCR_FCCDIS_SHIFT (13UL)
/*! FCCDIS - Disable Flash Controller Cache
 *  0b0..enable
 *  0b1..disable
 */
#define MCM_CPCR_FCCDIS(x) (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCCDIS_SHIFT)) & MCM_CPCR_FCCDIS_MASK)

#define MCM_CPCR_FCDSPDIS_MASK  (0x4000UL)
#define MCM_CPCR_FCDSPDIS_SHIFT (14UL)
/*! FCDSPDIS - Disable Flash Controller Data Speculation
 *  0b0..enable
 *  0b1..disable
 */
#define MCM_CPCR_FCDSPDIS(x) (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCDSPDIS_SHIFT)) & MCM_CPCR_FCDSPDIS_MASK)

#define MCM_CPCR_FCSPDIS_MASK  (0x8000UL)
#define MCM_CPCR_FCSPDIS_SHIFT (15UL)
/*! FCSPDIS - Disable Flash Controller Speculation
 *  0b0..enable
 *  0b1..disable
 */
#define MCM_CPCR_FCSPDIS(x) (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCSPDIS_SHIFT)) & MCM_CPCR_FCSPDIS_MASK)

#define MCM_CPCR_INSDIS_MASK  (0x10000UL)
#define MCM_CPCR_INSDIS_SHIFT (16UL)
/*! INSDIS - Disable instructions supported only by DSP56800EX core
 *  0b0..BFSC and 32-bit multiply and MAC instructions enabled
 *  0b1..BFSC and 32-bit multiply and MAC instructions disabled
 */
#define MCM_CPCR_INSDIS(x) (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_INSDIS_SHIFT)) & MCM_CPCR_INSDIS_MASK)

#define MCM_CPCR_RCDIS_MASK  (0x20000UL)
#define MCM_CPCR_RCDIS_SHIFT (17UL)
/*! RCDIS - Disable core reverse carry
 *  0b0..Core reverse carry enabled
 *  0b1..Core reverse carry disabled
 */
#define MCM_CPCR_RCDIS(x) (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_RCDIS_SHIFT)) & MCM_CPCR_RCDIS_MASK)

#define MCM_CPCR_SRDIS_MASK  (0x40000UL)
#define MCM_CPCR_SRDIS_SHIFT (18UL)
/*! SRDIS - Disable core new shadow region
 *  0b0..Core new shadow region enabled
 *  0b1..Core new shadow region disabled
 */
#define MCM_CPCR_SRDIS(x) (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_SRDIS_SHIFT)) & MCM_CPCR_SRDIS_MASK)

#define MCM_CPCR_IBDIS_MASK  (0x80000UL)
#define MCM_CPCR_IBDIS_SHIFT (19UL)
/*! IBDIS - Disable core instruction buffer
 *  0b0..Core longword instruction buffer enabled
 *  0b1..Core longword instruction buffer disabled
 */
#define MCM_CPCR_IBDIS(x) (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_IBDIS_SHIFT)) & MCM_CPCR_IBDIS_MASK)

#define MCM_CPCR_FCSDIS_MASK  (0x100000UL)
#define MCM_CPCR_FCSDIS_SHIFT (20UL)
/*! FCSDIS - Disable Flash Memory Controller stall
 *  0b0..Stall logic is enabled. While a flash memory command is executing, a flash memory access can occur
 *       without causing a bus error. The flash memory command completes execution, and then the flash memory access
 *       occurs.
 *  0b1..Stall logic is disabled. While a flash memory command is executing, an attempted flash memory access causes a
 * bus error.
 */
#define MCM_CPCR_FCSDIS(x) (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCSDIS_SHIFT)) & MCM_CPCR_FCSDIS_MASK)

#define MCM_CPCR_XBARARB_MASK  (0x80000000UL)
#define MCM_CPCR_XBARARB_SHIFT (31UL)
/*! XBARARB - Select DMA Controller priority in AXBS Crossbar Switch arbitration scheme
 *  0b0..Fixed-priority arbitration is selected: DSC core has a higher priority than the DMA Controller's priority
 *  0b1..Round-robin priority arbitration is selected: DMA Controller and DSC core have equal priority
 */
#define MCM_CPCR_XBARARB(x) (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_XBARARB_SHIFT)) & MCM_CPCR_XBARARB_MASK)
/*! @} */

/*! @name CFADR - Core fault address register */
/*! @{ */

#define MCM_CFADR_ADDR_MASK  (0xFFFFFFFFUL)
#define MCM_CFADR_ADDR_SHIFT (0UL)
/*! ADDR - Indicates the faulting address of the last core access terminated with an error response.
 */
#define MCM_CFADR_ADDR(x) (((uint32_t)(((uint32_t)(x)) << MCM_CFADR_ADDR_SHIFT)) & MCM_CFADR_ADDR_MASK)
/*! @} */

/*! @name CFATR - Core fault attributes register */
/*! @{ */

#define MCM_CFATR_TYPE_MASK  (0x1U)
#define MCM_CFATR_TYPE_SHIFT (0U)
/*! TYPE - Type of last faulted core access
 *  0b0..Instruction
 *  0b1..Data
 */
#define MCM_CFATR_TYPE(x) (((uint8_t)(((uint8_t)(x)) << MCM_CFATR_TYPE_SHIFT)) & MCM_CFATR_TYPE_MASK)

#define MCM_CFATR_BUFFER_MASK  (0x4U)
#define MCM_CFATR_BUFFER_SHIFT (2U)
/*! BUFFER - Indicates if last faulted core access was bufferable
 *  0b0..Non-bufferable
 *  0b1..Bufferable
 */
#define MCM_CFATR_BUFFER(x) (((uint8_t)(((uint8_t)(x)) << MCM_CFATR_BUFFER_SHIFT)) & MCM_CFATR_BUFFER_MASK)

#define MCM_CFATR_SIZE_MASK  (0x70U)
#define MCM_CFATR_SIZE_SHIFT (4U)
/*! SIZE - Size of last faulted core access
 *  0b000..8-bit
 *  0b001..16-bit
 *  0b010..32-bit
 */
#define MCM_CFATR_SIZE(x) (((uint8_t)(((uint8_t)(x)) << MCM_CFATR_SIZE_SHIFT)) & MCM_CFATR_SIZE_MASK)

#define MCM_CFATR_DIR_MASK  (0x80U)
#define MCM_CFATR_DIR_SHIFT (7U)
/*! DIR - Direction of last faulted core access
 *  0b0..Core read access
 *  0b1..Core write access
 */
#define MCM_CFATR_DIR(x) (((uint8_t)(((uint8_t)(x)) << MCM_CFATR_DIR_SHIFT)) & MCM_CFATR_DIR_MASK)
/*! @} */

/*! @name CFLOC - Core fault location register */
/*! @{ */

#define MCM_CFLOC_LOC_MASK  (0xC0U)
#define MCM_CFLOC_LOC_SHIFT (6U)
/*! LOC - Location of last captured fault
 *  0b00..Error occurred on M0 (instruction bus)
 *  0b01..Error occured on M1 (operand A bus)
 *  0b10..Error occured on M2 (operand B bus)
 *  0b11..Reserved
 */
#define MCM_CFLOC_LOC(x) (((uint8_t)(((uint8_t)(x)) << MCM_CFLOC_LOC_SHIFT)) & MCM_CFLOC_LOC_MASK)
/*! @} */

/*! @name CFIER - Core fault interrupt enable register */
/*! @{ */

#define MCM_CFIER_ECFEI_MASK  (0x80U)
#define MCM_CFIER_ECFEI_SHIFT (7U)
/*! ECFEI - Enable core fault error interrupt
 *  0b0..Do not generate an error interrupt on a faulted system bus cycle
 *  0b1..Generate an error interrupt to the interrupt controller on a faulted system bus cycle
 */
#define MCM_CFIER_ECFEI(x) (((uint8_t)(((uint8_t)(x)) << MCM_CFIER_ECFEI_SHIFT)) & MCM_CFIER_ECFEI_MASK)
/*! @} */

/*! @name CFISR - MCM interrupt status register */
/*! @{ */

#define MCM_CFISR_CFEDL_MASK  (0x40U)
#define MCM_CFISR_CFEDL_SHIFT (6U)
/*! CFEDL - Core fault error data lost flag
 *  0b0..No bus error data lost
 *  0b1..A bus error has occured before the previous error condition was cleared.
 */
#define MCM_CFISR_CFEDL(x) (((uint8_t)(((uint8_t)(x)) << MCM_CFISR_CFEDL_SHIFT)) & MCM_CFISR_CFEDL_MASK)

#define MCM_CFISR_CFEI_MASK  (0x80U)
#define MCM_CFISR_CFEI_SHIFT (7U)
/*! CFEI - Core fault error interrupt flag
 *  0b0..No bus error
 *  0b1..A bus error has occurred. The faulting address, attributes (and possibly write data) are captured in the
 *       CFADR, CFATR, and CFDTR registers. The error interrupt is enabled only if CFIER[ECFEI] is set.
 */
#define MCM_CFISR_CFEI(x) (((uint8_t)(((uint8_t)(x)) << MCM_CFISR_CFEI_SHIFT)) & MCM_CFISR_CFEI_MASK)
/*! @} */

/*! @name CFDTR - Core fault data register */
/*! @{ */

#define MCM_CFDTR_DATA_MASK  (0xFFFFFFFFUL)
#define MCM_CFDTR_DATA_SHIFT (0UL)
/*! DATA - Contains write data associated with the faulting access of the last internal bus write
 *    access. The data value is taken directly from the write data bus. Read data is not captured.
 */
#define MCM_CFDTR_DATA(x) (((uint32_t)(((uint32_t)(x)) << MCM_CFDTR_DATA_SHIFT)) & MCM_CFDTR_DATA_MASK)
/*! @} */

/*! @name RPCR - Resource Protection Control Register */
/*! @{ */

#define MCM_RPCR_RPE_MASK  (0x1UL)
#define MCM_RPCR_RPE_SHIFT (0UL)
/*! RPE - Resource Protection Enable
 *  0b0..Resource protection disabled
 *  0b1..Resource protection enabled
 */
#define MCM_RPCR_RPE(x) (((uint32_t)(((uint32_t)(x)) << MCM_RPCR_RPE_SHIFT)) & MCM_RPCR_RPE_MASK)

#define MCM_RPCR_RL_MASK  (0x2UL)
#define MCM_RPCR_RL_SHIFT (1UL)
/*! RL - Register Lock
 *  0b0..RP register values may be changed
 *  0b1..RP registers are locked and may not be changed until after a system reset
 */
#define MCM_RPCR_RL(x) (((uint32_t)(((uint32_t)(x)) << MCM_RPCR_RL_SHIFT)) & MCM_RPCR_RL_MASK)
/*! @} */

/*! @name UFLASHBAR - User Flash Base Address Register */
/*! @{ */

#define MCM_UFLASHBAR_FBA_MASK  (0x3F000UL)
#define MCM_UFLASHBAR_FBA_SHIFT (12UL)
/*! FBA - Flash Base Address for User Region
 */
#define MCM_UFLASHBAR_FBA(x) (((uint32_t)(((uint32_t)(x)) << MCM_UFLASHBAR_FBA_SHIFT)) & MCM_UFLASHBAR_FBA_MASK)
/*! @} */

/*! @name UPRAMBAR - User Program RAM Base Address Register */
/*! @{ */

#define MCM_UPRAMBAR_RBA_MASK  (0x7F00UL)
#define MCM_UPRAMBAR_RBA_SHIFT (8UL)
/*! RBA - Program RAM Base Address for User Region
 */
#define MCM_UPRAMBAR_RBA(x) (((uint32_t)(((uint32_t)(x)) << MCM_UPRAMBAR_RBA_SHIFT)) & MCM_UPRAMBAR_RBA_MASK)
/*! @} */

/*! @name UBROMBAR - User Boot ROM Base Address Register */
/*! @{ */

#define MCM_UBROMBAR_RBA_MASK  (0xFFFFFFFFUL)
#define MCM_UBROMBAR_RBA_SHIFT (0UL)
#define MCM_UBROMBAR_RBA(x)    (((uint32_t)(((uint32_t)(x)) << MCM_UBROMBAR_RBA_SHIFT)) & MCM_UBROMBAR_RBA_MASK)
/*! @} */

/*! @name SRPOSP - Resource Protection Other Stack Pointer */
/*! @{ */

#define MCM_SRPOSP_SRPOSP_MASK  (0xFFFFFFUL)
#define MCM_SRPOSP_SRPOSP_SHIFT (0UL)
/*! SRPOSP - Resource protection "other" SP
 */
#define MCM_SRPOSP_SRPOSP(x) (((uint32_t)(((uint32_t)(x)) << MCM_SRPOSP_SRPOSP_SHIFT)) & MCM_SRPOSP_SRPOSP_MASK)
/*! @} */

/*! @name SRPIPC - Memory Protection Illegal PC */
/*! @{ */

#define MCM_SRPIPC_SRPIFPC_MASK  (0x1FFFFFUL)
#define MCM_SRPIPC_SRPIFPC_SHIFT (0UL)
/*! SRPIFPC - Resource Protection Illegal Faulting PC
 */
#define MCM_SRPIPC_SRPIFPC(x) (((uint32_t)(((uint32_t)(x)) << MCM_SRPIPC_SRPIFPC_SHIFT)) & MCM_SRPIPC_SRPIFPC_MASK)

#define MCM_SRPIPC_SRPIFV_MASK  (0x80000000UL)
#define MCM_SRPIPC_SRPIFV_SHIFT (31UL)
/*! SRPIFV - Resource Protection Illegal Fault Valid
 */
#define MCM_SRPIPC_SRPIFV(x) (((uint32_t)(((uint32_t)(x)) << MCM_SRPIPC_SRPIFV_SHIFT)) & MCM_SRPIPC_SRPIFV_MASK)
/*! @} */

/*! @name SRPMPC - Resource Protection Misaligned PC */
/*! @{ */

#define MCM_SRPMPC_SRPMFPC_MASK  (0x1FFFFFUL)
#define MCM_SRPMPC_SRPMFPC_SHIFT (0UL)
/*! SRPMFPC - Resource Protection Misaligned Faulting PC
 */
#define MCM_SRPMPC_SRPMFPC(x) (((uint32_t)(((uint32_t)(x)) << MCM_SRPMPC_SRPMFPC_SHIFT)) & MCM_SRPMPC_SRPMFPC_MASK)

#define MCM_SRPMPC_SRPMFV_MASK  (0x80000000UL)
#define MCM_SRPMPC_SRPMFV_SHIFT (31UL)
/*! SRPMFV - Resource Protection Misaligned Fault Valid
 */
#define MCM_SRPMPC_SRPMFV(x) (((uint32_t)(((uint32_t)(x)) << MCM_SRPMPC_SRPMFV_SHIFT)) & MCM_SRPMPC_SRPMFV_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group MCM_Register_Masks */

/* MCM - Peripheral instance base addresses */
/** Peripheral MCM base address */
#define MCM_BASE (0xC000u)
/** Peripheral MCM base pointer */
#define MCM ((MCM_Type *)MCM_BASE)
/** Array initializer of MCM peripheral base addresses */
#define MCM_BASE_ADDRS \
    {                  \
        MCM_BASE       \
    }
/** Array initializer of MCM peripheral base pointers */
#define MCM_BASE_PTRS \
    {                 \
        MCM           \
    }

/*!
 * @}
 */ /* end of group MCM_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- OCCS Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OCCS_Peripheral_Access_Layer OCCS Peripheral Access Layer
 * @{
 */

/** OCCS - Register Layout Typedef */
typedef struct
{
    __IO uint16_t CTRL;  /**< PLL Control Register, offset: 0x0 */
    __IO uint16_t DIVBY; /**< PLL Divide-By Register, offset: 0x1 */
    __IO uint16_t STAT;  /**< OCCS Status Register, offset: 0x2 */
    uint16_t RESERVED_0[1];
    __IO uint16_t OSCTL1;  /**< Oscillator Control Register 1, offset: 0x4 */
    __IO uint16_t OSCTL2;  /**< Oscillator Control Register 2, offset: 0x5 */
    __IO uint16_t CLKCHKR; /**< External Clock Check Reference, offset: 0x6 */
    __I uint16_t CLKCHKT;  /**< External Clock Check Target, offset: 0x7 */
    __IO uint16_t PROT;    /**< Protection Register, offset: 0x8 */
} OCCS_Type;

/* ----------------------------------------------------------------------------
   -- OCCS Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OCCS_Register_Masks OCCS Register Masks
 * @{
 */

/*! @name CTRL - PLL Control Register */
/*! @{ */

#define OCCS_CTRL_ZSRC_MASK  (0x1U)
#define OCCS_CTRL_ZSRC_SHIFT (0U)
/*! ZSRC - CLOCK Source
 *  0b0..MSTR_OSC
 *  0b1..PLL output divided by 2
 */
#define OCCS_CTRL_ZSRC(x) (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_ZSRC_SHIFT)) & OCCS_CTRL_ZSRC_MASK)

#define OCCS_CTRL_PRECS_MASK  (0xCU)
#define OCCS_CTRL_PRECS_SHIFT (2U)
/*! PRECS - Prescaler Clock Select
 *  0b00..8 MHz clock derived from 48 MHz RC oscillator divided by 6 is selected (reset value)
 *  0b01..External reference is selected
 *  0b10..200 kHz RC oscillator is selected
 *  0b11..48 MHz RC oscillator is selected
 */
#define OCCS_CTRL_PRECS(x) (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_PRECS_SHIFT)) & OCCS_CTRL_PRECS_MASK)

#define OCCS_CTRL_PLLPD_MASK  (0x10U)
#define OCCS_CTRL_PLLPD_SHIFT (4U)
/*! PLLPD - PLL Power Down
 *  0b0..PLL enabled
 *  0b1..PLL powered down
 */
#define OCCS_CTRL_PLLPD(x) (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_PLLPD_SHIFT)) & OCCS_CTRL_PLLPD_MASK)

#define OCCS_CTRL_LCKON_MASK  (0x80U)
#define OCCS_CTRL_LCKON_SHIFT (7U)
/*! LCKON - Lock Detector On
 *  0b0..Lock detector disabled
 *  0b1..Lock detector enabled
 */
#define OCCS_CTRL_LCKON(x) (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_LCKON_SHIFT)) & OCCS_CTRL_LCKON_MASK)

#define OCCS_CTRL_LOCIE_MASK  (0x800U)
#define OCCS_CTRL_LOCIE_SHIFT (11U)
/*! LOCIE - Loss of Reference Clock Interrupt Enable
 *  0b0..Interrupt disabled.
 *  0b1..Interrupt enabled.
 */
#define OCCS_CTRL_LOCIE(x) (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_LOCIE_SHIFT)) & OCCS_CTRL_LOCIE_MASK)

#define OCCS_CTRL_PLLIE0_MASK  (0x3000U)
#define OCCS_CTRL_PLLIE0_SHIFT (12U)
/*! PLLIE0 - PLL Interrupt Enable 0
 *  0b00..Disable interrupt.
 *  0b01..Enable interrupt on any rising edge of LCK0.
 *  0b10..Enable interrupt on falling edge of LCK0.
 *  0b11..Enable interrupt on any edge change of LCK0.
 */
#define OCCS_CTRL_PLLIE0(x) (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_PLLIE0_SHIFT)) & OCCS_CTRL_PLLIE0_MASK)

#define OCCS_CTRL_PLLIE1_MASK  (0xC000U)
#define OCCS_CTRL_PLLIE1_SHIFT (14U)
/*! PLLIE1 - PLL Interrupt Enable 1
 *  0b00..Disable interrupt.
 *  0b01..Enable interrupt on any rising edge of LCK1.
 *  0b10..Enable interrupt on falling edge of LCK1.
 *  0b11..Enable interrupt on any edge change of LCK1.
 */
#define OCCS_CTRL_PLLIE1(x) (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_PLLIE1_SHIFT)) & OCCS_CTRL_PLLIE1_MASK)
/*! @} */

/*! @name DIVBY - PLL Divide-By Register */
/*! @{ */

#define OCCS_DIVBY_PLLDB_MASK  (0x3FU)
#define OCCS_DIVBY_PLLDB_SHIFT (0U)
/*! PLLDB - PLL Divide By
 */
#define OCCS_DIVBY_PLLDB(x) (((uint16_t)(((uint16_t)(x)) << OCCS_DIVBY_PLLDB_SHIFT)) & OCCS_DIVBY_PLLDB_MASK)

#define OCCS_DIVBY_COD_MASK  (0xF00U)
#define OCCS_DIVBY_COD_SHIFT (8U)
/*! COD - Clock Output Divide or Postscaler
 *  0b0000..Divide clock output by 1.
 *  0b0001..Divide clock output by 2.
 *  0b0010..Divide clock output by 4.
 *  0b0011..Divide clock output by 8.
 *  0b0100..Divide clock output by 16.
 *  0b0101..Divide clock output by 32.
 *  0b0110..Divide clock output by 64.
 *  0b0111..Divide clock output by 128.
 *  0b1xxx..Divide clock output by 256.
 */
#define OCCS_DIVBY_COD(x) (((uint16_t)(((uint16_t)(x)) << OCCS_DIVBY_COD_SHIFT)) & OCCS_DIVBY_COD_MASK)

#define OCCS_DIVBY_LORTP_MASK  (0xF000U)
#define OCCS_DIVBY_LORTP_SHIFT (12U)
/*! LORTP - Loss of Reference Clock Trip Point
 */
#define OCCS_DIVBY_LORTP(x) (((uint16_t)(((uint16_t)(x)) << OCCS_DIVBY_LORTP_SHIFT)) & OCCS_DIVBY_LORTP_MASK)
/*! @} */

/*! @name STAT - OCCS Status Register */
/*! @{ */

#define OCCS_STAT_ZSRCS_MASK  (0x3U)
#define OCCS_STAT_ZSRCS_SHIFT (0U)
/*! ZSRCS - CLOCK Source Status
 *  0b00..MSTR_OSC
 *  0b01..PLL output divided by 2
 *  0b1x..Synchronization in progress
 */
#define OCCS_STAT_ZSRCS(x) (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_ZSRCS_SHIFT)) & OCCS_STAT_ZSRCS_MASK)

#define OCCS_STAT_PLLPDN_MASK  (0x10U)
#define OCCS_STAT_PLLPDN_SHIFT (4U)
/*! PLLPDN - PLL Power Down
 *  0b0..PLL not powered down.
 *  0b1..PLL powered down.
 */
#define OCCS_STAT_PLLPDN(x) (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_PLLPDN_SHIFT)) & OCCS_STAT_PLLPDN_MASK)

#define OCCS_STAT_LCK0_MASK  (0x20U)
#define OCCS_STAT_LCK0_SHIFT (5U)
/*! LCK0 - PLL Lock 0 Status
 *  0b0..PLL is unlocked.
 *  0b1..PLL is locked (coarse).
 */
#define OCCS_STAT_LCK0(x) (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_LCK0_SHIFT)) & OCCS_STAT_LCK0_MASK)

#define OCCS_STAT_LCK1_MASK  (0x40U)
#define OCCS_STAT_LCK1_SHIFT (6U)
/*! LCK1 - PLL Lock 1 Status
 *  0b0..PLL is unlocked.
 *  0b1..PLL is locked (fine).
 */
#define OCCS_STAT_LCK1(x) (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_LCK1_SHIFT)) & OCCS_STAT_LCK1_MASK)

#define OCCS_STAT_OSC_OK_MASK  (0x80U)
#define OCCS_STAT_OSC_OK_SHIFT (7U)
/*! OSC_OK - OSC_OK Indicator from XOSC
 *  0b0..Oscillator clock is still not stable, or XOSC is disabled.
 *  0b1..Oscillator clock is stable after crystal oscillator startup.
 */
#define OCCS_STAT_OSC_OK(x) (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_OSC_OK_SHIFT)) & OCCS_STAT_OSC_OK_MASK)

#define OCCS_STAT_MON_FAILURE_MASK  (0x100U)
#define OCCS_STAT_MON_FAILURE_SHIFT (8U)
/*! MON_FAILURE - XOSC Clock Monitor Failure Indicator. If MON_ENABLE is enabled, this flag
 *    indicates that an XOSC clock failure was detected. If MON_ENABLE is disabled, no failure is indicated.
 *  0b0..No clock failure, or XOSC clock monitor is disabled.
 *  0b1..Clock failure detected on XOSC clock when clock monitor is enabled.
 */
#define OCCS_STAT_MON_FAILURE(x) \
    (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_MON_FAILURE_SHIFT)) & OCCS_STAT_MON_FAILURE_MASK)

#define OCCS_STAT_LOCI_MASK  (0x2000U)
#define OCCS_STAT_LOCI_SHIFT (13U)
/*! LOCI - Loss of Reference Clock Interrupt
 *  0b0..Oscillator clock normal.
 *  0b1..Loss of oscillator clock detected.
 */
#define OCCS_STAT_LOCI(x) (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_LOCI_SHIFT)) & OCCS_STAT_LOCI_MASK)

#define OCCS_STAT_LOLI0_MASK  (0x4000U)
#define OCCS_STAT_LOLI0_SHIFT (14U)
/*! LOLI0 - PLL Lock or Loss of Lock Interrupt 0
 *  0b0..No lock or loss of lock event has occurred.
 *  0b1..PLL lock status based on PLLIE0.
 */
#define OCCS_STAT_LOLI0(x) (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_LOLI0_SHIFT)) & OCCS_STAT_LOLI0_MASK)

#define OCCS_STAT_LOLI1_MASK  (0x8000U)
#define OCCS_STAT_LOLI1_SHIFT (15U)
/*! LOLI1 - PLL Lock or Loss of Lock Interrupt 1
 *  0b0..No lock or loss of lock event has occurred.
 *  0b1..PLL lock status based on PLLIE1.
 */
#define OCCS_STAT_LOLI1(x) (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_LOLI1_SHIFT)) & OCCS_STAT_LOLI1_MASK)
/*! @} */

/*! @name OSCTL1 - Oscillator Control Register 1 */
/*! @{ */

#define OCCS_OSCTL1_EXT_SEL_MASK  (0x400U)
#define OCCS_OSCTL1_EXT_SEL_SHIFT (10U)
/*! EXT_SEL - External Clock In Select
 *  0b0..Use the output of the crystal oscillator as the external clock input.
 *  0b1..Use CLKIN as the external clock input.
 */
#define OCCS_OSCTL1_EXT_SEL(x) (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL1_EXT_SEL_SHIFT)) & OCCS_OSCTL1_EXT_SEL_MASK)

#define OCCS_OSCTL1_CLK_MODE_MASK  (0x1000U)
#define OCCS_OSCTL1_CLK_MODE_SHIFT (12U)
/*! CLK_MODE - Crystal Oscillator Clock Mode
 *  0b0..Crystal oscillator is in FSP mode (COHL=0) or LCP mode (COHL=1), when COPD=0.
 *  0b1..External clock bypass mode. This enables the crystal oscillator's external clock bypass mode and allows
 *       an external clock source on the EXTAL input of the oscillator to propagate directly to the oscillator's
 *       clock output.
 */
#define OCCS_OSCTL1_CLK_MODE(x) \
    (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL1_CLK_MODE_SHIFT)) & OCCS_OSCTL1_CLK_MODE_MASK)

#define OCCS_OSCTL1_COHL_MASK  (0x2000U)
#define OCCS_OSCTL1_COHL_SHIFT (13U)
/*! COHL - Crystal Oscillator High/Low Power Level
 *  0b0..High power mode.
 *  0b1..Low power mode.
 */
#define OCCS_OSCTL1_COHL(x) (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL1_COHL_SHIFT)) & OCCS_OSCTL1_COHL_MASK)
/*! @} */

/*! @name OSCTL2 - Oscillator Control Register 2 */
/*! @{ */

#define OCCS_OSCTL2_FREQ_TRIM200K_MASK  (0x1FFU)
#define OCCS_OSCTL2_FREQ_TRIM200K_SHIFT (0U)
/*! FREQ_TRIM200K - 200 kHz Internal RC Oscillator Frequency Trim
 */
#define OCCS_OSCTL2_FREQ_TRIM200K(x) \
    (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL2_FREQ_TRIM200K_SHIFT)) & OCCS_OSCTL2_FREQ_TRIM200K_MASK)

#define OCCS_OSCTL2_MON_ENABLE_MASK  (0x200U)
#define OCCS_OSCTL2_MON_ENABLE_SHIFT (9U)
/*! MON_ENABLE - XOSC Clock Monitor Enable Control
 *  0b0..XOSC Clock Monitor is disabled.
 *  0b1..XOSC Clock Monitor is enabled.
 */
#define OCCS_OSCTL2_MON_ENABLE(x) \
    (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL2_MON_ENABLE_SHIFT)) & OCCS_OSCTL2_MON_ENABLE_MASK)

#define OCCS_OSCTL2_COPD_MASK  (0x4000U)
#define OCCS_OSCTL2_COPD_SHIFT (14U)
/*! COPD - Crystal Oscillator Power Down
 *  0b0..Crystal oscillator is powered on.
 *  0b1..Crystal oscillator is powered down.
 */
#define OCCS_OSCTL2_COPD(x) (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL2_COPD_SHIFT)) & OCCS_OSCTL2_COPD_MASK)

#define OCCS_OSCTL2_ROPD200K_MASK  (0x8000U)
#define OCCS_OSCTL2_ROPD200K_SHIFT (15U)
/*! ROPD200K - 200 kHz RC Oscillator Power Down
 */
#define OCCS_OSCTL2_ROPD200K(x) \
    (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL2_ROPD200K_SHIFT)) & OCCS_OSCTL2_ROPD200K_MASK)
/*! @} */

/*! @name CLKCHKR - External Clock Check Reference */
/*! @{ */

#define OCCS_CLKCHKR_REF_CNT_MASK  (0x7FU)
#define OCCS_CLKCHKR_REF_CNT_SHIFT (0U)
/*! REF_CNT - Reference Count
 */
#define OCCS_CLKCHKR_REF_CNT(x) \
    (((uint16_t)(((uint16_t)(x)) << OCCS_CLKCHKR_REF_CNT_SHIFT)) & OCCS_CLKCHKR_REF_CNT_MASK)

#define OCCS_CLKCHKR_CHK_ENA_MASK  (0x8000U)
#define OCCS_CLKCHKR_CHK_ENA_SHIFT (15U)
/*! CHK_ENA - Check Enable
 *  0b0..Writing a low while the clock checking operation is in progress stops the check in its current state.
 *       Reading a low after a check has been started indicates that the check operation is complete and the final
 *       values are valid in the REF_CNT and TARGET_CNT fields.
 *  0b1..Writing a one clears the REF_CNT and TARGET_CNT fields and starts the clock checking function. The
 *       CLK_ENA bit remains high while the operation is in progress.
 */
#define OCCS_CLKCHKR_CHK_ENA(x) \
    (((uint16_t)(((uint16_t)(x)) << OCCS_CLKCHKR_CHK_ENA_SHIFT)) & OCCS_CLKCHKR_CHK_ENA_MASK)
/*! @} */

/*! @name CLKCHKT - External Clock Check Target */
/*! @{ */

#define OCCS_CLKCHKT_TARGET_CNT_MASK  (0x7FFU)
#define OCCS_CLKCHKT_TARGET_CNT_SHIFT (0U)
/*! TARGET_CNT - CLKCHKT Target Count
 */
#define OCCS_CLKCHKT_TARGET_CNT(x) \
    (((uint16_t)(((uint16_t)(x)) << OCCS_CLKCHKT_TARGET_CNT_SHIFT)) & OCCS_CLKCHKT_TARGET_CNT_MASK)
/*! @} */

/*! @name PROT - Protection Register */
/*! @{ */

#define OCCS_PROT_PLLEP_MASK  (0x3U)
#define OCCS_PROT_PLLEP_SHIFT (0U)
/*! PLLEP - PLL Enable Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define OCCS_PROT_PLLEP(x) (((uint16_t)(((uint16_t)(x)) << OCCS_PROT_PLLEP_SHIFT)) & OCCS_PROT_PLLEP_MASK)

#define OCCS_PROT_OSCEP_MASK  (0xCU)
#define OCCS_PROT_OSCEP_SHIFT (2U)
/*! OSCEP - Oscillator Enable Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define OCCS_PROT_OSCEP(x) (((uint16_t)(((uint16_t)(x)) << OCCS_PROT_OSCEP_SHIFT)) & OCCS_PROT_OSCEP_MASK)

#define OCCS_PROT_FRQEP_MASK  (0x30U)
#define OCCS_PROT_FRQEP_SHIFT (4U)
/*! FRQEP - Frequency Enable Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define OCCS_PROT_FRQEP(x) (((uint16_t)(((uint16_t)(x)) << OCCS_PROT_FRQEP_SHIFT)) & OCCS_PROT_FRQEP_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group OCCS_Register_Masks */

/* OCCS - Peripheral instance base addresses */
/** Peripheral OCCS base address */
#define OCCS_BASE (0xE2B0u)
/** Peripheral OCCS base pointer */
#define OCCS ((OCCS_Type *)OCCS_BASE)
/** Array initializer of OCCS peripheral base addresses */
#define OCCS_BASE_ADDRS \
    {                   \
        OCCS_BASE       \
    }
/** Array initializer of OCCS peripheral base pointers */
#define OCCS_BASE_PTRS \
    {                  \
        OCCS           \
    }

/*!
 * @}
 */ /* end of group OCCS_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- PIT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Peripheral_Access_Layer PIT Peripheral Access Layer
 * @{
 */

/** PIT - Register Layout Typedef */
typedef struct
{
    __IO uint16_t CTRL; /**< PIT Control Register, offset: 0x0 */
    __IO uint16_t MOD;  /**< PIT Modulo Register, offset: 0x1 */
    __I uint16_t CNTR;  /**< PIT Counter Register, offset: 0x2 */
} PIT_Type;

/* ----------------------------------------------------------------------------
   -- PIT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Masks PIT Register Masks
 * @{
 */

/*! @name CTRL - PIT Control Register */
/*! @{ */

#define PIT_CTRL_CNT_EN_MASK  (0x1U)
#define PIT_CTRL_CNT_EN_SHIFT (0U)
/*! CNT_EN - Count Enable
 *  0b0..PIT counter reset (default).
 *  0b1..PIT counter active.
 */
#define PIT_CTRL_CNT_EN(x) (((uint16_t)(((uint16_t)(x)) << PIT_CTRL_CNT_EN_SHIFT)) & PIT_CTRL_CNT_EN_MASK)

#define PIT_CTRL_PRIE_MASK  (0x2U)
#define PIT_CTRL_PRIE_SHIFT (1U)
/*! PRIE - PIT Roll-Over Interrupt Enable.
 *  0b0..PIT roll-over interrupt disabled (default).
 *  0b1..PIT roll-over interrupt enabled.
 */
#define PIT_CTRL_PRIE(x) (((uint16_t)(((uint16_t)(x)) << PIT_CTRL_PRIE_SHIFT)) & PIT_CTRL_PRIE_MASK)

#define PIT_CTRL_PRF_MASK  (0x4U)
#define PIT_CTRL_PRF_SHIFT (2U)
/*! PRF - PIT Roll-Over Flag.
 *  0b0..PIT counter has not reached the modulo value. (default)
 *  0b1..PIT counter has reached the modulo value.
 */
#define PIT_CTRL_PRF(x) (((uint16_t)(((uint16_t)(x)) << PIT_CTRL_PRF_SHIFT)) & PIT_CTRL_PRF_MASK)

#define PIT_CTRL_PRESCALER_MASK  (0x78U)
#define PIT_CTRL_PRESCALER_SHIFT (3U)
/*! PRESCALER
 *  0b0000..Clock
 *  0b0001..Clock divided by 2
 *  0b0010..Clock divided by 4
 *  0b0011..Clock divided by 8
 *  0b0100..Clock divided by 16
 *  0b0101..Clock divided by 32
 *  0b0110..Clock divided by 64
 *  0b0111..Clock divided by 128
 *  0b1000..Clock divided by 256
 *  0b1001..Clock divided by 512
 *  0b1010..Clock divided by 1024
 *  0b1011..Clock divided by 2048
 *  0b1100..Clock divided by 4096
 *  0b1101..Clock divided by 8192
 *  0b1110..Clock divided by 16384
 *  0b1111..Clock divided by 32768
 */
#define PIT_CTRL_PRESCALER(x) (((uint16_t)(((uint16_t)(x)) << PIT_CTRL_PRESCALER_SHIFT)) & PIT_CTRL_PRESCALER_MASK)

#define PIT_CTRL_CLKSEL_MASK  (0x300U)
#define PIT_CTRL_CLKSEL_SHIFT (8U)
/*! CLKSEL
 *  0b00..Selects IPBus clock
 *  0b01..Selects alternate clock 1
 *  0b10..Selects alternate clock 2
 *  0b11..Selects alternate clock 3
 */
#define PIT_CTRL_CLKSEL(x) (((uint16_t)(((uint16_t)(x)) << PIT_CTRL_CLKSEL_SHIFT)) & PIT_CTRL_CLKSEL_MASK)

#define PIT_CTRL_SLAVE_MASK  (0x8000U)
#define PIT_CTRL_SLAVE_SHIFT (15U)
/*! SLAVE
 *  0b0..CNT_EN from this PIT is used to control operation (default).
 *  0b1..CNT_EN from master PIT is used to control operation.
 */
#define PIT_CTRL_SLAVE(x) (((uint16_t)(((uint16_t)(x)) << PIT_CTRL_SLAVE_SHIFT)) & PIT_CTRL_SLAVE_MASK)
/*! @} */

/*! @name MOD - PIT Modulo Register */
/*! @{ */

#define PIT_MOD_MODULO_VALUE_MASK  (0xFFFFU)
#define PIT_MOD_MODULO_VALUE_SHIFT (0U)
#define PIT_MOD_MODULO_VALUE(x) \
    (((uint16_t)(((uint16_t)(x)) << PIT_MOD_MODULO_VALUE_SHIFT)) & PIT_MOD_MODULO_VALUE_MASK)
/*! @} */

/*! @name CNTR - PIT Counter Register */
/*! @{ */

#define PIT_CNTR_COUNTER_VALUE_MASK  (0xFFFFU)
#define PIT_CNTR_COUNTER_VALUE_SHIFT (0U)
#define PIT_CNTR_COUNTER_VALUE(x) \
    (((uint16_t)(((uint16_t)(x)) << PIT_CNTR_COUNTER_VALUE_SHIFT)) & PIT_CNTR_COUNTER_VALUE_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group PIT_Register_Masks */

/* PIT - Peripheral instance base addresses */
/** Peripheral PIT0 base address */
#define PIT0_BASE (0xE100u)
/** Peripheral PIT0 base pointer */
#define PIT0 ((PIT_Type *)PIT0_BASE)
/** Peripheral PIT1 base address */
#define PIT1_BASE (0xE110u)
/** Peripheral PIT1 base pointer */
#define PIT1 ((PIT_Type *)PIT1_BASE)
/** Array initializer of PIT peripheral base addresses */
#define PIT_BASE_ADDRS       \
    {                        \
        PIT0_BASE, PIT1_BASE \
    }
/** Array initializer of PIT peripheral base pointers */
#define PIT_BASE_PTRS \
    {                 \
        PIT0, PIT1    \
    }
/** Interrupt vectors for the PIT peripheral type */
#define PIT_IRQS                             \
    {                                        \
        PIT0_ROLLOVR_IRQn, PIT1_ROLLOVR_IRQn \
    }

/*!
 * @}
 */ /* end of group PIT_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- PMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Peripheral_Access_Layer PMC Peripheral Access Layer
 * @{
 */

/** PMC - Register Layout Typedef */
typedef struct
{
    __IO uint16_t CTRL; /**< Control Register, offset: 0x0 */
    __IO uint16_t STS;  /**< Status Register, offset: 0x1 */
} PMC_Type;

/* ----------------------------------------------------------------------------
   -- PMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Masks PMC Register Masks
 * @{
 */

/*! @name CTRL - Control Register */
/*! @{ */

#define PMC_CTRL_LV22IE_MASK  (0x1U)
#define PMC_CTRL_LV22IE_SHIFT (0U)
/*! LV22IE - 2.2 V Low Voltage Interrupt Enable
 *  0b0..Disable setting the low voltage interrupt.
 *  0b1..Enable setting the low voltage interrupt.
 */
#define PMC_CTRL_LV22IE(x) (((uint16_t)(((uint16_t)(x)) << PMC_CTRL_LV22IE_SHIFT)) & PMC_CTRL_LV22IE_MASK)

#define PMC_CTRL_LV27IE_MASK  (0x2U)
#define PMC_CTRL_LV27IE_SHIFT (1U)
/*! LV27IE - 2.7 V Low Voltage Interrupt Enable
 *  0b0..Disable setting the low voltage interrupt.
 *  0b1..Enable setting the low voltage interrupt.
 */
#define PMC_CTRL_LV27IE(x) (((uint16_t)(((uint16_t)(x)) << PMC_CTRL_LV27IE_SHIFT)) & PMC_CTRL_LV27IE_MASK)

#define PMC_CTRL_HV22IE_MASK  (0x4U)
#define PMC_CTRL_HV22IE_SHIFT (2U)
/*! HV22IE - 2.2 V High Voltage Interrupt Enable
 *  0b0..Disable setting the high voltage interrupt.
 *  0b1..Enable setting the high voltage interrupt.
 */
#define PMC_CTRL_HV22IE(x) (((uint16_t)(((uint16_t)(x)) << PMC_CTRL_HV22IE_SHIFT)) & PMC_CTRL_HV22IE_MASK)

#define PMC_CTRL_HV27IE_MASK  (0x8U)
#define PMC_CTRL_HV27IE_SHIFT (3U)
/*! HV27IE - 2.7 V High Voltage Interrupt Enable
 *  0b0..Disable setting the high voltage interrupt.
 *  0b1..Enable setting the high voltage interrupt.
 */
#define PMC_CTRL_HV27IE(x) (((uint16_t)(((uint16_t)(x)) << PMC_CTRL_HV27IE_SHIFT)) & PMC_CTRL_HV27IE_MASK)

#define PMC_CTRL_VRBEN_MASK  (0x80U)
#define PMC_CTRL_VRBEN_SHIFT (7U)
/*! VRBEN - Voltage Reference Buffer Enable
 *  0b0..Disable voltage reference buffering.
 *  0b1..Enable voltage reference buffering.
 */
#define PMC_CTRL_VRBEN(x) (((uint16_t)(((uint16_t)(x)) << PMC_CTRL_VRBEN_SHIFT)) & PMC_CTRL_VRBEN_MASK)

#define PMC_CTRL_TRIM_MASK  (0xF000U)
#define PMC_CTRL_TRIM_SHIFT (12U)
/*! TRIM - Bandgap Trim
 */
#define PMC_CTRL_TRIM(x) (((uint16_t)(((uint16_t)(x)) << PMC_CTRL_TRIM_SHIFT)) & PMC_CTRL_TRIM_MASK)
/*! @} */

/*! @name STS - Status Register */
/*! @{ */

#define PMC_STS_LV22F_MASK  (0x1U)
#define PMC_STS_LV22F_SHIFT (0U)
/*! LV22F - 2.2 V Low Voltage Flag
 *  0b0..3.3 V supply is not below the 2.2 V threshold.
 *  0b1..3.3 V supply is below the 2.2 V threshold.
 */
#define PMC_STS_LV22F(x) (((uint16_t)(((uint16_t)(x)) << PMC_STS_LV22F_SHIFT)) & PMC_STS_LV22F_MASK)

#define PMC_STS_LV27F_MASK  (0x2U)
#define PMC_STS_LV27F_SHIFT (1U)
/*! LV27F - 2.7 V Low Voltage Flag
 *  0b0..3.3 V supply is not below the 2.7 V threshold.
 *  0b1..3.3 V supply is below the 2.7 V threshold.
 */
#define PMC_STS_LV27F(x) (((uint16_t)(((uint16_t)(x)) << PMC_STS_LV27F_SHIFT)) & PMC_STS_LV27F_MASK)

#define PMC_STS_SLV22F_MASK  (0x4U)
#define PMC_STS_SLV22F_SHIFT (2U)
/*! SLV22F - Sticky 2.2 V Low Voltage Flag
 *  0b0..3.3 V supply has not dropped below the 2.2 V threshold.
 *  0b1..3.3 V supply has dropped below the 2.2 V threshold.
 */
#define PMC_STS_SLV22F(x) (((uint16_t)(((uint16_t)(x)) << PMC_STS_SLV22F_SHIFT)) & PMC_STS_SLV22F_MASK)

#define PMC_STS_SLV27F_MASK  (0x8U)
#define PMC_STS_SLV27F_SHIFT (3U)
/*! SLV27F - Sticky 2.7 V Low Voltage Flag
 *  0b0..3.3 V supply has not dropped below the 2.7 V threshold.
 *  0b1..3.3 V supply has dropped below the 2.7 V threshold.
 */
#define PMC_STS_SLV27F(x) (((uint16_t)(((uint16_t)(x)) << PMC_STS_SLV27F_SHIFT)) & PMC_STS_SLV27F_MASK)

#define PMC_STS_LVI_MASK  (0x10U)
#define PMC_STS_LVI_SHIFT (4U)
/*! LVI - Low Voltage Interrupt
 *  0b0..Low voltage interrupt cleared.
 *  0b1..Low voltage interrupt asserted.
 */
#define PMC_STS_LVI(x) (((uint16_t)(((uint16_t)(x)) << PMC_STS_LVI_SHIFT)) & PMC_STS_LVI_MASK)

#define PMC_STS_SR27_MASK  (0x20U)
#define PMC_STS_SR27_SHIFT (5U)
/*! SR27 - Small Regulator 2.7 V Active Flag
 *  0b0..The small regulator 2.7 V supply is not ready to be used.
 *  0b1..The small regulator 2.7 V supply is ready to be used.
 */
#define PMC_STS_SR27(x) (((uint16_t)(((uint16_t)(x)) << PMC_STS_SR27_SHIFT)) & PMC_STS_SR27_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group PMC_Register_Masks */

/* PMC - Peripheral instance base addresses */
/** Peripheral PMC base address */
#define PMC_BASE (0xE2A0u)
/** Peripheral PMC base pointer */
#define PMC ((PMC_Type *)PMC_BASE)
/** Array initializer of PMC peripheral base addresses */
#define PMC_BASE_ADDRS \
    {                  \
        PMC_BASE       \
    }
/** Array initializer of PMC peripheral base pointers */
#define PMC_BASE_PTRS \
    {                 \
        PMC           \
    }

/*!
 * @}
 */ /* end of group PMC_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- PWM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Peripheral_Access_Layer PWM Peripheral Access Layer
 * @{
 */

/** PWM - Register Layout Typedef */
typedef struct
{
    struct
    {                        /* offset: 0x0, array step: 0x30 */
        __I uint16_t CNT;    /**< Counter Register, array offset: 0x0, array step: 0x30 */
        __IO uint16_t INIT;  /**< Initial Count Register, array offset: 0x1, array step: 0x30 */
        __IO uint16_t CTRL2; /**< Control 2 Register, array offset: 0x2, array step: 0x30 */
        __IO uint16_t CTRL;  /**< Control Register, array offset: 0x3, array step: 0x30 */
        uint16_t RESERVED_0[1];
        __IO uint16_t VAL0;      /**< Value Register 0, array offset: 0x5, array step: 0x30 */
        __IO uint16_t FRACVAL1;  /**< Fractional Value Register 1, array offset: 0x6, array step: 0x30 */
        __IO uint16_t VAL1;      /**< Value Register 1, array offset: 0x7, array step: 0x30 */
        __IO uint16_t FRACVAL2;  /**< Fractional Value Register 2, array offset: 0x8, array step: 0x30 */
        __IO uint16_t VAL2;      /**< Value Register 2, array offset: 0x9, array step: 0x30 */
        __IO uint16_t FRACVAL3;  /**< Fractional Value Register 3, array offset: 0xA, array step: 0x30 */
        __IO uint16_t VAL3;      /**< Value Register 3, array offset: 0xB, array step: 0x30 */
        __IO uint16_t FRACVAL4;  /**< Fractional Value Register 4, array offset: 0xC, array step: 0x30 */
        __IO uint16_t VAL4;      /**< Value Register 4, array offset: 0xD, array step: 0x30 */
        __IO uint16_t FRACVAL5;  /**< Fractional Value Register 5, array offset: 0xE, array step: 0x30 */
        __IO uint16_t VAL5;      /**< Value Register 5, array offset: 0xF, array step: 0x30 */
        __IO uint16_t FRCTRL;    /**< Fractional Control Register, array offset: 0x10, array step: 0x30 */
        __IO uint16_t OCTRL;     /**< Output Control Register, array offset: 0x11, array step: 0x30 */
        __IO uint16_t STS;       /**< Status Register, array offset: 0x12, array step: 0x30 */
        __IO uint16_t INTEN;     /**< Interrupt Enable Register, array offset: 0x13, array step: 0x30 */
        __IO uint16_t DMAEN;     /**< DMA Enable Register, array offset: 0x14, array step: 0x30 */
        __IO uint16_t TCTRL;     /**< Output Trigger Control Register, array offset: 0x15, array step: 0x30 */
        __IO uint16_t DISMAP[2]; /**< Fault Disable Mapping Register 0..Fault Disable Mapping Register 1, array offset:
                                    0x16, array step: index*0x30, index2*0x1 */
        __IO uint16_t DTCNT0;    /**< Deadtime Count Register 0, array offset: 0x18, array step: 0x30 */
        __IO uint16_t DTCNT1;    /**< Deadtime Count Register 1, array offset: 0x19, array step: 0x30 */
        __IO uint16_t CAPTCTRLA; /**< Capture Control A Register, array offset: 0x1A, array step: 0x30 */
        __IO uint16_t CAPTCOMPA; /**< Capture Compare A Register, array offset: 0x1B, array step: 0x30 */
        __IO uint16_t CAPTCTRLB; /**< Capture Control B Register, array offset: 0x1C, array step: 0x30 */
        __IO uint16_t CAPTCOMPB; /**< Capture Compare B Register, array offset: 0x1D, array step: 0x30 */
        __IO uint16_t CAPTCTRLX; /**< Capture Control X Register, array offset: 0x1E, array step: 0x30 */
        __IO uint16_t CAPTCOMPX; /**< Capture Compare X Register, array offset: 0x1F, array step: 0x30 */
        __I uint16_t CVAL0;      /**< Capture Value 0 Register, array offset: 0x20, array step: 0x30 */
        __I uint16_t CVAL0CYC;   /**< Capture Value 0 Cycle Register, array offset: 0x21, array step: 0x30 */
        __I uint16_t CVAL1;      /**< Capture Value 1 Register, array offset: 0x22, array step: 0x30 */
        __I uint16_t CVAL1CYC;   /**< Capture Value 1 Cycle Register, array offset: 0x23, array step: 0x30 */
        __I uint16_t CVAL2;      /**< Capture Value 2 Register, array offset: 0x24, array step: 0x30 */
        __I uint16_t CVAL2CYC;   /**< Capture Value 2 Cycle Register, array offset: 0x25, array step: 0x30 */
        __I uint16_t CVAL3;      /**< Capture Value 3 Register, array offset: 0x26, array step: 0x30 */
        __I uint16_t CVAL3CYC;   /**< Capture Value 3 Cycle Register, array offset: 0x27, array step: 0x30 */
        __I uint16_t CVAL4;      /**< Capture Value 4 Register, array offset: 0x28, array step: 0x30 */
        __I uint16_t CVAL4CYC;   /**< Capture Value 4 Cycle Register, array offset: 0x29, array step: 0x30 */
        __I uint16_t CVAL5;      /**< Capture Value 5 Register, array offset: 0x2A, array step: 0x30 */
        __I uint16_t CVAL5CYC;   /**< Capture Value 5 Cycle Register, array offset: 0x2B, array step: 0x30 */
        __IO uint16_t PHASEDLY;  /**< Phase Delay Register, array offset: 0x2C, array step: 0x30 */
        uint16_t RESERVED_1[3];
    } SM[4];
    __IO uint16_t OUTEN;    /**< Output Enable Register, offset: 0xC0 */
    __IO uint16_t MASK;     /**< Mask Register, offset: 0xC1 */
    __IO uint16_t SWCOUT;   /**< Software Controlled Output Register, offset: 0xC2 */
    __IO uint16_t DTSRCSEL; /**< PWM Source Select Register, offset: 0xC3 */
    __IO uint16_t MCTRL;    /**< Master Control Register, offset: 0xC4 */
    __IO uint16_t MCTRL2;   /**< Master Control 2 Register, offset: 0xC5 */
    struct
    {                         /* offset: 0xC6, array step: 0x6 */
        __IO uint16_t FCTRL;  /**< Fault Control Register, array offset: 0xC6, array step: 0x6 */
        __IO uint16_t FSTS;   /**< Fault Status Register, array offset: 0xC7, array step: 0x6 */
        __IO uint16_t FFILT;  /**< Fault Filter Register, array offset: 0xC8, array step: 0x6 */
        __IO uint16_t FTST;   /**< Fault Test Register, array offset: 0xC9, array step: 0x6 */
        __IO uint16_t FCTRL2; /**< Fault Control 2 Register, array offset: 0xCA, array step: 0x6 */
        uint16_t RESERVED_0[1];
    } FAULT[2];
} PWM_Type;

/* ----------------------------------------------------------------------------
   -- PWM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Register_Masks PWM Register Masks
 * @{
 */

/*! @name CNT - Counter Register */
/*! @{ */

#define PWM_CNT_CNT_MASK  (0xFFFFU)
#define PWM_CNT_CNT_SHIFT (0U)
/*! CNT - Counter Register Bits
 */
#define PWM_CNT_CNT(x) (((uint16_t)(((uint16_t)(x)) << PWM_CNT_CNT_SHIFT)) & PWM_CNT_CNT_MASK)
/*! @} */

/* The count of PWM_CNT */
#define PWM_CNT_COUNT (4U)

/*! @name INIT - Initial Count Register */
/*! @{ */

#define PWM_INIT_INIT_MASK  (0xFFFFU)
#define PWM_INIT_INIT_SHIFT (0U)
/*! INIT - Initial Count Register Bits
 */
#define PWM_INIT_INIT(x) (((uint16_t)(((uint16_t)(x)) << PWM_INIT_INIT_SHIFT)) & PWM_INIT_INIT_MASK)
/*! @} */

/* The count of PWM_INIT */
#define PWM_INIT_COUNT (4U)

/*! @name CTRL2 - Control 2 Register */
/*! @{ */

#define PWM_CTRL2_CLK_SEL_MASK  (0x3U)
#define PWM_CTRL2_CLK_SEL_SHIFT (0U)
/*! CLK_SEL - Clock Source Select
 *  0b00..The IPBus clock is used as the clock for the local prescaler and counter.
 *  0b01..EXT_CLK is used as the clock for the local prescaler and counter.
 *  0b10..Submodule 0's clock (AUX_CLK) is used as the source clock for the local prescaler and counter. This
 *        setting should not be used in submodule 0 as it will force the clock to logic 0.
 *  0b11..reserved
 */
#define PWM_CTRL2_CLK_SEL(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_CLK_SEL_SHIFT)) & PWM_CTRL2_CLK_SEL_MASK)

#define PWM_CTRL2_RELOAD_SEL_MASK  (0x4U)
#define PWM_CTRL2_RELOAD_SEL_SHIFT (2U)
/*! RELOAD_SEL - Reload Source Select
 *  0b0..The local RELOAD signal is used to reload registers.
 *  0b1..The master RELOAD signal (from submodule 0) is used to reload registers. This setting should not be used
 *       in submodule 0 as it will force the RELOAD signal to logic 0.
 */
#define PWM_CTRL2_RELOAD_SEL(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_RELOAD_SEL_SHIFT)) & PWM_CTRL2_RELOAD_SEL_MASK)

#define PWM_CTRL2_FORCE_SEL_MASK  (0x38U)
#define PWM_CTRL2_FORCE_SEL_SHIFT (3U)
/*! FORCE_SEL - This read/write bit determines the source of the FORCE OUTPUT signal for this submodule.
 *  0b000..The local force signal, CTRL2[FORCE], from this submodule is used to force updates.
 *  0b001..The master force signal from submodule 0 is used to force updates. This setting should not be used in
 *         submodule 0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b010..The local reload signal from this submodule is used to force updates without regard to the state of LDOK.
 *  0b011..The master reload signal from submodule0 is used to force updates if LDOK is set. This setting should
 *         not be used in submodule0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b100..The local sync signal from this submodule is used to force updates.
 *  0b101..The master sync signal from submodule0 is used to force updates. This setting should not be used in
 *         submodule0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b110..The external force signal, EXT_FORCE, from outside the PWM module causes updates.
 *  0b111..The external sync signal, EXT_SYNC, from outside the PWM module causes updates.
 */
#define PWM_CTRL2_FORCE_SEL(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_FORCE_SEL_SHIFT)) & PWM_CTRL2_FORCE_SEL_MASK)

#define PWM_CTRL2_FORCE_MASK  (0x40U)
#define PWM_CTRL2_FORCE_SHIFT (6U)
/*! FORCE - Force Initialization
 */
#define PWM_CTRL2_FORCE(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_FORCE_SHIFT)) & PWM_CTRL2_FORCE_MASK)

#define PWM_CTRL2_FRCEN_MASK  (0x80U)
#define PWM_CTRL2_FRCEN_SHIFT (7U)
/*! FRCEN - FRCEN
 *  0b0..Initialization from a FORCE_OUT is disabled.
 *  0b1..Initialization from a FORCE_OUT is enabled.
 */
#define PWM_CTRL2_FRCEN(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_FRCEN_SHIFT)) & PWM_CTRL2_FRCEN_MASK)

#define PWM_CTRL2_INIT_SEL_MASK  (0x300U)
#define PWM_CTRL2_INIT_SEL_SHIFT (8U)
/*! INIT_SEL - Initialization Control Select
 *  0b00..Local sync (PWM_X) causes initialization.
 *  0b01..Master reload from submodule 0 causes initialization. This setting should not be used in submodule 0 as
 *        it will force the INIT signal to logic 0. The submodule counter will only reinitialize when a master
 *        reload occurs.
 *  0b10..Master sync from submodule 0 causes initialization. This setting should not be used in submodule 0 as it
 *        will force the INIT signal to logic 0.
 *  0b11..EXT_SYNC causes initialization.
 */
#define PWM_CTRL2_INIT_SEL(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_INIT_SEL_SHIFT)) & PWM_CTRL2_INIT_SEL_MASK)

#define PWM_CTRL2_PWMX_INIT_MASK  (0x400U)
#define PWM_CTRL2_PWMX_INIT_SHIFT (10U)
/*! PWMX_INIT - PWM_X Initial Value
 */
#define PWM_CTRL2_PWMX_INIT(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_PWMX_INIT_SHIFT)) & PWM_CTRL2_PWMX_INIT_MASK)

#define PWM_CTRL2_PWM45_INIT_MASK  (0x800U)
#define PWM_CTRL2_PWM45_INIT_SHIFT (11U)
/*! PWM45_INIT - PWM45 Initial Value
 */
#define PWM_CTRL2_PWM45_INIT(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_PWM45_INIT_SHIFT)) & PWM_CTRL2_PWM45_INIT_MASK)

#define PWM_CTRL2_PWM23_INIT_MASK  (0x1000U)
#define PWM_CTRL2_PWM23_INIT_SHIFT (12U)
/*! PWM23_INIT - PWM23 Initial Value
 */
#define PWM_CTRL2_PWM23_INIT(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_PWM23_INIT_SHIFT)) & PWM_CTRL2_PWM23_INIT_MASK)

#define PWM_CTRL2_INDEP_MASK  (0x2000U)
#define PWM_CTRL2_INDEP_SHIFT (13U)
/*! INDEP - Independent or Complementary Pair Operation
 *  0b0..PWM_A and PWM_B form a complementary PWM pair.
 *  0b1..PWM_A and PWM_B outputs are independent PWMs.
 */
#define PWM_CTRL2_INDEP(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_INDEP_SHIFT)) & PWM_CTRL2_INDEP_MASK)

#define PWM_CTRL2_WAITEN_MASK  (0x4000U)
#define PWM_CTRL2_WAITEN_SHIFT (14U)
/*! WAITEN - WAIT Enable
 */
#define PWM_CTRL2_WAITEN(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_WAITEN_SHIFT)) & PWM_CTRL2_WAITEN_MASK)

#define PWM_CTRL2_DBGEN_MASK  (0x8000U)
#define PWM_CTRL2_DBGEN_SHIFT (15U)
/*! DBGEN - Debug Enable
 */
#define PWM_CTRL2_DBGEN(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL2_DBGEN_SHIFT)) & PWM_CTRL2_DBGEN_MASK)
/*! @} */

/* The count of PWM_CTRL2 */
#define PWM_CTRL2_COUNT (4U)

/*! @name CTRL - Control Register */
/*! @{ */

#define PWM_CTRL_DBLEN_MASK  (0x1U)
#define PWM_CTRL_DBLEN_SHIFT (0U)
/*! DBLEN - Double Switching Enable
 *  0b0..Double switching disabled.
 *  0b1..Double switching enabled.
 */
#define PWM_CTRL_DBLEN(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_DBLEN_SHIFT)) & PWM_CTRL_DBLEN_MASK)

#define PWM_CTRL_DBLX_MASK  (0x2U)
#define PWM_CTRL_DBLX_SHIFT (1U)
/*! DBLX - PWMX Double Switching Enable
 *  0b0..PWMX double pulse disabled.
 *  0b1..PWMX double pulse enabled.
 */
#define PWM_CTRL_DBLX(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_DBLX_SHIFT)) & PWM_CTRL_DBLX_MASK)

#define PWM_CTRL_LDMOD_MASK  (0x4U)
#define PWM_CTRL_LDMOD_SHIFT (2U)
/*! LDMOD - Load Mode Select
 *  0b0..Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set.
 *  0b1..Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set.
 *       In this case it is not necessary to set CTRL[FULL] or CTRL[HALF].
 */
#define PWM_CTRL_LDMOD(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_LDMOD_SHIFT)) & PWM_CTRL_LDMOD_MASK)

#define PWM_CTRL_SPLIT_MASK  (0x8U)
#define PWM_CTRL_SPLIT_SHIFT (3U)
/*! SPLIT - Split the DBLPWM signal to PWMA and PWMB
 *  0b0..DBLPWM is not split. PWMA and PWMB each have double pulses.
 *  0b1..DBLPWM is split to PWMA and PWMB.
 */
#define PWM_CTRL_SPLIT(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_SPLIT_SHIFT)) & PWM_CTRL_SPLIT_MASK)

#define PWM_CTRL_PRSC_MASK  (0x70U)
#define PWM_CTRL_PRSC_SHIFT (4U)
/*! PRSC - Prescaler
 *  0b000..PWM clock frequency = fclk
 *  0b001..PWM clock frequency = fclk/2
 *  0b010..PWM clock frequency = fclk/4
 *  0b011..PWM clock frequency = fclk/8
 *  0b100..PWM clock frequency = fclk/16
 *  0b101..PWM clock frequency = fclk/32
 *  0b110..PWM clock frequency = fclk/64
 *  0b111..PWM clock frequency = fclk/128
 */
#define PWM_CTRL_PRSC(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_PRSC_SHIFT)) & PWM_CTRL_PRSC_MASK)

#define PWM_CTRL_COMPMODE_MASK  (0x80U)
#define PWM_CTRL_COMPMODE_SHIFT (7U)
/*! COMPMODE - Compare Mode
 *  0b0..The VAL* registers and the PWM counter are compared using an "equal to" method. This means that PWM edges
 *       are only produced when the counter is equal to one of the VAL* register values. This implies that a PWMA
 *       output that is high at the end of a period will maintain this state until a match with VAL3 clears the
 *       output in the following period.
 *  0b1..The VAL* registers and the PWM counter are compared using an "equal to or greater than" method. This
 *       means that PWM edges are produced when the counter is equal to or greater than one of the VAL* register
 *       values. This implies that a PWMA output that is high at the end of a period could go low at the start of the
 *       next period if the starting counter value is greater than (but not necessarily equal to) the new VAL3 value.
 */
#define PWM_CTRL_COMPMODE(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_COMPMODE_SHIFT)) & PWM_CTRL_COMPMODE_MASK)

#define PWM_CTRL_DT_MASK  (0x300U)
#define PWM_CTRL_DT_SHIFT (8U)
/*! DT - Deadtime
 */
#define PWM_CTRL_DT(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_DT_SHIFT)) & PWM_CTRL_DT_MASK)

#define PWM_CTRL_FULL_MASK  (0x400U)
#define PWM_CTRL_FULL_SHIFT (10U)
/*! FULL - Full Cycle Reload
 *  0b0..Full-cycle reloads disabled.
 *  0b1..Full-cycle reloads enabled.
 */
#define PWM_CTRL_FULL(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_FULL_SHIFT)) & PWM_CTRL_FULL_MASK)

#define PWM_CTRL_HALF_MASK  (0x800U)
#define PWM_CTRL_HALF_SHIFT (11U)
/*! HALF - Half Cycle Reload
 *  0b0..Half-cycle reloads disabled.
 *  0b1..Half-cycle reloads enabled.
 */
#define PWM_CTRL_HALF(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_HALF_SHIFT)) & PWM_CTRL_HALF_MASK)

#define PWM_CTRL_LDFQ_MASK  (0xF000U)
#define PWM_CTRL_LDFQ_SHIFT (12U)
/*! LDFQ - Load Frequency
 *  0b0000..Every PWM opportunity
 *  0b0001..Every 2 PWM opportunities
 *  0b0010..Every 3 PWM opportunities
 *  0b0011..Every 4 PWM opportunities
 *  0b0100..Every 5 PWM opportunities
 *  0b0101..Every 6 PWM opportunities
 *  0b0110..Every 7 PWM opportunities
 *  0b0111..Every 8 PWM opportunities
 *  0b1000..Every 9 PWM opportunities
 *  0b1001..Every 10 PWM opportunities
 *  0b1010..Every 11 PWM opportunities
 *  0b1011..Every 12 PWM opportunities
 *  0b1100..Every 13 PWM opportunities
 *  0b1101..Every 14 PWM opportunities
 *  0b1110..Every 15 PWM opportunities
 *  0b1111..Every 16 PWM opportunities
 */
#define PWM_CTRL_LDFQ(x) (((uint16_t)(((uint16_t)(x)) << PWM_CTRL_LDFQ_SHIFT)) & PWM_CTRL_LDFQ_MASK)
/*! @} */

/* The count of PWM_CTRL */
#define PWM_CTRL_COUNT (4U)

/*! @name VAL0 - Value Register 0 */
/*! @{ */

#define PWM_VAL0_VAL0_MASK  (0xFFFFU)
#define PWM_VAL0_VAL0_SHIFT (0U)
/*! VAL0 - Value Register 0
 */
#define PWM_VAL0_VAL0(x) (((uint16_t)(((uint16_t)(x)) << PWM_VAL0_VAL0_SHIFT)) & PWM_VAL0_VAL0_MASK)
/*! @} */

/* The count of PWM_VAL0 */
#define PWM_VAL0_COUNT (4U)

/*! @name FRACVAL1 - Fractional Value Register 1 */
/*! @{ */

#define PWM_FRACVAL1_FRACVAL1_MASK  (0xF800U)
#define PWM_FRACVAL1_FRACVAL1_SHIFT (11U)
/*! FRACVAL1 - Fractional Value 1 Register
 */
#define PWM_FRACVAL1_FRACVAL1(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_FRACVAL1_FRACVAL1_SHIFT)) & PWM_FRACVAL1_FRACVAL1_MASK)
/*! @} */

/* The count of PWM_FRACVAL1 */
#define PWM_FRACVAL1_COUNT (4U)

/*! @name VAL1 - Value Register 1 */
/*! @{ */

#define PWM_VAL1_VAL1_MASK  (0xFFFFU)
#define PWM_VAL1_VAL1_SHIFT (0U)
/*! VAL1 - Value Register 1
 */
#define PWM_VAL1_VAL1(x) (((uint16_t)(((uint16_t)(x)) << PWM_VAL1_VAL1_SHIFT)) & PWM_VAL1_VAL1_MASK)
/*! @} */

/* The count of PWM_VAL1 */
#define PWM_VAL1_COUNT (4U)

/*! @name FRACVAL2 - Fractional Value Register 2 */
/*! @{ */

#define PWM_FRACVAL2_FRACVAL2_MASK  (0xF800U)
#define PWM_FRACVAL2_FRACVAL2_SHIFT (11U)
/*! FRACVAL2 - Fractional Value 2
 */
#define PWM_FRACVAL2_FRACVAL2(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_FRACVAL2_FRACVAL2_SHIFT)) & PWM_FRACVAL2_FRACVAL2_MASK)
/*! @} */

/* The count of PWM_FRACVAL2 */
#define PWM_FRACVAL2_COUNT (4U)

/*! @name VAL2 - Value Register 2 */
/*! @{ */

#define PWM_VAL2_VAL2_MASK  (0xFFFFU)
#define PWM_VAL2_VAL2_SHIFT (0U)
/*! VAL2 - Value Register 2
 */
#define PWM_VAL2_VAL2(x) (((uint16_t)(((uint16_t)(x)) << PWM_VAL2_VAL2_SHIFT)) & PWM_VAL2_VAL2_MASK)
/*! @} */

/* The count of PWM_VAL2 */
#define PWM_VAL2_COUNT (4U)

/*! @name FRACVAL3 - Fractional Value Register 3 */
/*! @{ */

#define PWM_FRACVAL3_FRACVAL3_MASK  (0xF800U)
#define PWM_FRACVAL3_FRACVAL3_SHIFT (11U)
/*! FRACVAL3 - Fractional Value 3
 */
#define PWM_FRACVAL3_FRACVAL3(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_FRACVAL3_FRACVAL3_SHIFT)) & PWM_FRACVAL3_FRACVAL3_MASK)
/*! @} */

/* The count of PWM_FRACVAL3 */
#define PWM_FRACVAL3_COUNT (4U)

/*! @name VAL3 - Value Register 3 */
/*! @{ */

#define PWM_VAL3_VAL3_MASK  (0xFFFFU)
#define PWM_VAL3_VAL3_SHIFT (0U)
/*! VAL3 - Value Register 3
 */
#define PWM_VAL3_VAL3(x) (((uint16_t)(((uint16_t)(x)) << PWM_VAL3_VAL3_SHIFT)) & PWM_VAL3_VAL3_MASK)
/*! @} */

/* The count of PWM_VAL3 */
#define PWM_VAL3_COUNT (4U)

/*! @name FRACVAL4 - Fractional Value Register 4 */
/*! @{ */

#define PWM_FRACVAL4_FRACVAL4_MASK  (0xF800U)
#define PWM_FRACVAL4_FRACVAL4_SHIFT (11U)
/*! FRACVAL4 - Fractional Value 4
 */
#define PWM_FRACVAL4_FRACVAL4(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_FRACVAL4_FRACVAL4_SHIFT)) & PWM_FRACVAL4_FRACVAL4_MASK)
/*! @} */

/* The count of PWM_FRACVAL4 */
#define PWM_FRACVAL4_COUNT (4U)

/*! @name VAL4 - Value Register 4 */
/*! @{ */

#define PWM_VAL4_VAL4_MASK  (0xFFFFU)
#define PWM_VAL4_VAL4_SHIFT (0U)
/*! VAL4 - Value Register 4
 */
#define PWM_VAL4_VAL4(x) (((uint16_t)(((uint16_t)(x)) << PWM_VAL4_VAL4_SHIFT)) & PWM_VAL4_VAL4_MASK)
/*! @} */

/* The count of PWM_VAL4 */
#define PWM_VAL4_COUNT (4U)

/*! @name FRACVAL5 - Fractional Value Register 5 */
/*! @{ */

#define PWM_FRACVAL5_FRACVAL5_MASK  (0xF800U)
#define PWM_FRACVAL5_FRACVAL5_SHIFT (11U)
/*! FRACVAL5 - Fractional Value 5
 */
#define PWM_FRACVAL5_FRACVAL5(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_FRACVAL5_FRACVAL5_SHIFT)) & PWM_FRACVAL5_FRACVAL5_MASK)
/*! @} */

/* The count of PWM_FRACVAL5 */
#define PWM_FRACVAL5_COUNT (4U)

/*! @name VAL5 - Value Register 5 */
/*! @{ */

#define PWM_VAL5_VAL5_MASK  (0xFFFFU)
#define PWM_VAL5_VAL5_SHIFT (0U)
/*! VAL5 - Value Register 5
 */
#define PWM_VAL5_VAL5(x) (((uint16_t)(((uint16_t)(x)) << PWM_VAL5_VAL5_SHIFT)) & PWM_VAL5_VAL5_MASK)
/*! @} */

/* The count of PWM_VAL5 */
#define PWM_VAL5_COUNT (4U)

/*! @name FRCTRL - Fractional Control Register */
/*! @{ */

#define PWM_FRCTRL_FRAC1_EN_MASK  (0x2U)
#define PWM_FRCTRL_FRAC1_EN_SHIFT (1U)
/*! FRAC1_EN - Fractional Cycle PWM Period Enable
 *  0b0..Disable fractional cycle length for the PWM period.
 *  0b1..Enable fractional cycle length for the PWM period.
 */
#define PWM_FRCTRL_FRAC1_EN(x) (((uint16_t)(((uint16_t)(x)) << PWM_FRCTRL_FRAC1_EN_SHIFT)) & PWM_FRCTRL_FRAC1_EN_MASK)

#define PWM_FRCTRL_FRAC23_EN_MASK  (0x4U)
#define PWM_FRCTRL_FRAC23_EN_SHIFT (2U)
/*! FRAC23_EN - Fractional Cycle Placement Enable for PWM_A
 *  0b0..Disable fractional cycle placement for PWM_A.
 *  0b1..Enable fractional cycle placement for PWM_A.
 */
#define PWM_FRCTRL_FRAC23_EN(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_FRCTRL_FRAC23_EN_SHIFT)) & PWM_FRCTRL_FRAC23_EN_MASK)

#define PWM_FRCTRL_FRAC45_EN_MASK  (0x10U)
#define PWM_FRCTRL_FRAC45_EN_SHIFT (4U)
/*! FRAC45_EN - Fractional Cycle Placement Enable for PWM_B
 *  0b0..Disable fractional cycle placement for PWM_B.
 *  0b1..Enable fractional cycle placement for PWM_B.
 */
#define PWM_FRCTRL_FRAC45_EN(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_FRCTRL_FRAC45_EN_SHIFT)) & PWM_FRCTRL_FRAC45_EN_MASK)

#define PWM_FRCTRL_FRAC_PU_MASK  (0x100U)
#define PWM_FRCTRL_FRAC_PU_SHIFT (8U)
/*! FRAC_PU - Fractional Delay Circuit Power Up
 *  0b0..Turn off fractional delay logic.
 *  0b1..Power up fractional delay logic.
 */
#define PWM_FRCTRL_FRAC_PU(x) (((uint16_t)(((uint16_t)(x)) << PWM_FRCTRL_FRAC_PU_SHIFT)) & PWM_FRCTRL_FRAC_PU_MASK)

#define PWM_FRCTRL_TEST_MASK  (0x8000U)
#define PWM_FRCTRL_TEST_SHIFT (15U)
/*! TEST - Test Status Bit
 */
#define PWM_FRCTRL_TEST(x) (((uint16_t)(((uint16_t)(x)) << PWM_FRCTRL_TEST_SHIFT)) & PWM_FRCTRL_TEST_MASK)
/*! @} */

/* The count of PWM_FRCTRL */
#define PWM_FRCTRL_COUNT (4U)

/*! @name OCTRL - Output Control Register */
/*! @{ */

#define PWM_OCTRL_PWMXFS_MASK  (0x3U)
#define PWM_OCTRL_PWMXFS_SHIFT (0U)
/*! PWMXFS - PWM_X Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_OCTRL_PWMXFS(x) (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_PWMXFS_SHIFT)) & PWM_OCTRL_PWMXFS_MASK)

#define PWM_OCTRL_PWMBFS_MASK  (0xCU)
#define PWM_OCTRL_PWMBFS_SHIFT (2U)
/*! PWMBFS - PWM_B Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_OCTRL_PWMBFS(x) (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_PWMBFS_SHIFT)) & PWM_OCTRL_PWMBFS_MASK)

#define PWM_OCTRL_PWMAFS_MASK  (0x30U)
#define PWM_OCTRL_PWMAFS_SHIFT (4U)
/*! PWMAFS - PWM_A Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_OCTRL_PWMAFS(x) (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_PWMAFS_SHIFT)) & PWM_OCTRL_PWMAFS_MASK)

#define PWM_OCTRL_POLX_MASK  (0x100U)
#define PWM_OCTRL_POLX_SHIFT (8U)
/*! POLX - PWM_X Output Polarity
 *  0b0..PWM_X output not inverted. A high level on the PWM_X pin represents the "on" or "active" state.
 *  0b1..PWM_X output inverted. A low level on the PWM_X pin represents the "on" or "active" state.
 */
#define PWM_OCTRL_POLX(x) (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_POLX_SHIFT)) & PWM_OCTRL_POLX_MASK)

#define PWM_OCTRL_POLB_MASK  (0x200U)
#define PWM_OCTRL_POLB_SHIFT (9U)
/*! POLB - PWM_B Output Polarity
 *  0b0..PWM_B output not inverted. A high level on the PWM_B pin represents the "on" or "active" state.
 *  0b1..PWM_B output inverted. A low level on the PWM_B pin represents the "on" or "active" state.
 */
#define PWM_OCTRL_POLB(x) (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_POLB_SHIFT)) & PWM_OCTRL_POLB_MASK)

#define PWM_OCTRL_POLA_MASK  (0x400U)
#define PWM_OCTRL_POLA_SHIFT (10U)
/*! POLA - PWM_A Output Polarity
 *  0b0..PWM_A output not inverted. A high level on the PWM_A pin represents the "on" or "active" state.
 *  0b1..PWM_A output inverted. A low level on the PWM_A pin represents the "on" or "active" state.
 */
#define PWM_OCTRL_POLA(x) (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_POLA_SHIFT)) & PWM_OCTRL_POLA_MASK)

#define PWM_OCTRL_PWMX_IN_MASK  (0x2000U)
#define PWM_OCTRL_PWMX_IN_SHIFT (13U)
/*! PWMX_IN - PWM_X Input
 */
#define PWM_OCTRL_PWMX_IN(x) (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_PWMX_IN_SHIFT)) & PWM_OCTRL_PWMX_IN_MASK)

#define PWM_OCTRL_PWMB_IN_MASK  (0x4000U)
#define PWM_OCTRL_PWMB_IN_SHIFT (14U)
/*! PWMB_IN - PWM_B Input
 */
#define PWM_OCTRL_PWMB_IN(x) (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_PWMB_IN_SHIFT)) & PWM_OCTRL_PWMB_IN_MASK)

#define PWM_OCTRL_PWMA_IN_MASK  (0x8000U)
#define PWM_OCTRL_PWMA_IN_SHIFT (15U)
/*! PWMA_IN - PWM_A Input
 */
#define PWM_OCTRL_PWMA_IN(x) (((uint16_t)(((uint16_t)(x)) << PWM_OCTRL_PWMA_IN_SHIFT)) & PWM_OCTRL_PWMA_IN_MASK)
/*! @} */

/* The count of PWM_OCTRL */
#define PWM_OCTRL_COUNT (4U)

/*! @name STS - Status Register */
/*! @{ */

#define PWM_STS_CMPF_MASK  (0x3FU)
#define PWM_STS_CMPF_SHIFT (0U)
/*! CMPF - Compare Flags
 *  0b000000..No compare event has occurred for a particular VALx value.
 *  0b000001..A compare event has occurred for a particular VALx value.
 */
#define PWM_STS_CMPF(x) (((uint16_t)(((uint16_t)(x)) << PWM_STS_CMPF_SHIFT)) & PWM_STS_CMPF_MASK)

#define PWM_STS_CFX0_MASK  (0x40U)
#define PWM_STS_CFX0_SHIFT (6U)
/*! CFX0 - Capture Flag X0
 */
#define PWM_STS_CFX0(x) (((uint16_t)(((uint16_t)(x)) << PWM_STS_CFX0_SHIFT)) & PWM_STS_CFX0_MASK)

#define PWM_STS_CFX1_MASK  (0x80U)
#define PWM_STS_CFX1_SHIFT (7U)
/*! CFX1 - Capture Flag X1
 */
#define PWM_STS_CFX1(x) (((uint16_t)(((uint16_t)(x)) << PWM_STS_CFX1_SHIFT)) & PWM_STS_CFX1_MASK)

#define PWM_STS_CFB0_MASK  (0x100U)
#define PWM_STS_CFB0_SHIFT (8U)
/*! CFB0 - Capture Flag B0
 */
#define PWM_STS_CFB0(x) (((uint16_t)(((uint16_t)(x)) << PWM_STS_CFB0_SHIFT)) & PWM_STS_CFB0_MASK)

#define PWM_STS_CFB1_MASK  (0x200U)
#define PWM_STS_CFB1_SHIFT (9U)
/*! CFB1 - Capture Flag B1
 */
#define PWM_STS_CFB1(x) (((uint16_t)(((uint16_t)(x)) << PWM_STS_CFB1_SHIFT)) & PWM_STS_CFB1_MASK)

#define PWM_STS_CFA0_MASK  (0x400U)
#define PWM_STS_CFA0_SHIFT (10U)
/*! CFA0 - Capture Flag A0
 */
#define PWM_STS_CFA0(x) (((uint16_t)(((uint16_t)(x)) << PWM_STS_CFA0_SHIFT)) & PWM_STS_CFA0_MASK)

#define PWM_STS_CFA1_MASK  (0x800U)
#define PWM_STS_CFA1_SHIFT (11U)
/*! CFA1 - Capture Flag A1
 */
#define PWM_STS_CFA1(x) (((uint16_t)(((uint16_t)(x)) << PWM_STS_CFA1_SHIFT)) & PWM_STS_CFA1_MASK)

#define PWM_STS_RF_MASK  (0x1000U)
#define PWM_STS_RF_SHIFT (12U)
/*! RF - Reload Flag
 *  0b0..No new reload cycle since last STS[RF] clearing
 *  0b1..New reload cycle since last STS[RF] clearing
 */
#define PWM_STS_RF(x) (((uint16_t)(((uint16_t)(x)) << PWM_STS_RF_SHIFT)) & PWM_STS_RF_MASK)

#define PWM_STS_REF_MASK  (0x2000U)
#define PWM_STS_REF_SHIFT (13U)
/*! REF - Reload Error Flag
 *  0b0..No reload error occurred.
 *  0b1..Reload signal occurred with non-coherent data and MCTRL[LDOK] = 0.
 */
#define PWM_STS_REF(x) (((uint16_t)(((uint16_t)(x)) << PWM_STS_REF_SHIFT)) & PWM_STS_REF_MASK)

#define PWM_STS_RUF_MASK  (0x4000U)
#define PWM_STS_RUF_SHIFT (14U)
/*! RUF - Registers Updated Flag
 *  0b0..No register update has occurred since last reload.
 *  0b1..At least one of the double buffered registers has been updated since the last reload.
 */
#define PWM_STS_RUF(x) (((uint16_t)(((uint16_t)(x)) << PWM_STS_RUF_SHIFT)) & PWM_STS_RUF_MASK)
/*! @} */

/* The count of PWM_STS */
#define PWM_STS_COUNT (4U)

/*! @name INTEN - Interrupt Enable Register */
/*! @{ */

#define PWM_INTEN_CMPIE_MASK  (0x3FU)
#define PWM_INTEN_CMPIE_SHIFT (0U)
/*! CMPIE - Compare Interrupt Enables
 *  0b000000..The corresponding STS[CMPF] bit will not cause an interrupt request.
 *  0b000001..The corresponding STS[CMPF] bit will cause an interrupt request.
 */
#define PWM_INTEN_CMPIE(x) (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_CMPIE_SHIFT)) & PWM_INTEN_CMPIE_MASK)

#define PWM_INTEN_CX0IE_MASK  (0x40U)
#define PWM_INTEN_CX0IE_SHIFT (6U)
/*! CX0IE - Capture X 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFX0].
 *  0b1..Interrupt request enabled for STS[CFX0].
 */
#define PWM_INTEN_CX0IE(x) (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_CX0IE_SHIFT)) & PWM_INTEN_CX0IE_MASK)

#define PWM_INTEN_CX1IE_MASK  (0x80U)
#define PWM_INTEN_CX1IE_SHIFT (7U)
/*! CX1IE - Capture X 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFX1].
 *  0b1..Interrupt request enabled for STS[CFX1].
 */
#define PWM_INTEN_CX1IE(x) (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_CX1IE_SHIFT)) & PWM_INTEN_CX1IE_MASK)

#define PWM_INTEN_CB0IE_MASK  (0x100U)
#define PWM_INTEN_CB0IE_SHIFT (8U)
/*! CB0IE - Capture B 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFB0].
 *  0b1..Interrupt request enabled for STS[CFB0].
 */
#define PWM_INTEN_CB0IE(x) (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_CB0IE_SHIFT)) & PWM_INTEN_CB0IE_MASK)

#define PWM_INTEN_CB1IE_MASK  (0x200U)
#define PWM_INTEN_CB1IE_SHIFT (9U)
/*! CB1IE - Capture B 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFB1].
 *  0b1..Interrupt request enabled for STS[CFB1].
 */
#define PWM_INTEN_CB1IE(x) (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_CB1IE_SHIFT)) & PWM_INTEN_CB1IE_MASK)

#define PWM_INTEN_CA0IE_MASK  (0x400U)
#define PWM_INTEN_CA0IE_SHIFT (10U)
/*! CA0IE - Capture A 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFA0].
 *  0b1..Interrupt request enabled for STS[CFA0].
 */
#define PWM_INTEN_CA0IE(x) (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_CA0IE_SHIFT)) & PWM_INTEN_CA0IE_MASK)

#define PWM_INTEN_CA1IE_MASK  (0x800U)
#define PWM_INTEN_CA1IE_SHIFT (11U)
/*! CA1IE - Capture A 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFA1].
 *  0b1..Interrupt request enabled for STS[CFA1].
 */
#define PWM_INTEN_CA1IE(x) (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_CA1IE_SHIFT)) & PWM_INTEN_CA1IE_MASK)

#define PWM_INTEN_RIE_MASK  (0x1000U)
#define PWM_INTEN_RIE_SHIFT (12U)
/*! RIE - Reload Interrupt Enable
 *  0b0..STS[RF] CPU interrupt requests disabled
 *  0b1..STS[RF] CPU interrupt requests enabled
 */
#define PWM_INTEN_RIE(x) (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_RIE_SHIFT)) & PWM_INTEN_RIE_MASK)

#define PWM_INTEN_REIE_MASK  (0x2000U)
#define PWM_INTEN_REIE_SHIFT (13U)
/*! REIE - Reload Error Interrupt Enable
 *  0b0..STS[REF] CPU interrupt requests disabled
 *  0b1..STS[REF] CPU interrupt requests enabled
 */
#define PWM_INTEN_REIE(x) (((uint16_t)(((uint16_t)(x)) << PWM_INTEN_REIE_SHIFT)) & PWM_INTEN_REIE_MASK)
/*! @} */

/* The count of PWM_INTEN */
#define PWM_INTEN_COUNT (4U)

/*! @name DMAEN - DMA Enable Register */
/*! @{ */

#define PWM_DMAEN_CX0DE_MASK  (0x1U)
#define PWM_DMAEN_CX0DE_SHIFT (0U)
/*! CX0DE - Capture X0 FIFO DMA Enable
 */
#define PWM_DMAEN_CX0DE(x) (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_CX0DE_SHIFT)) & PWM_DMAEN_CX0DE_MASK)

#define PWM_DMAEN_CX1DE_MASK  (0x2U)
#define PWM_DMAEN_CX1DE_SHIFT (1U)
/*! CX1DE - Capture X1 FIFO DMA Enable
 */
#define PWM_DMAEN_CX1DE(x) (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_CX1DE_SHIFT)) & PWM_DMAEN_CX1DE_MASK)

#define PWM_DMAEN_CB0DE_MASK  (0x4U)
#define PWM_DMAEN_CB0DE_SHIFT (2U)
/*! CB0DE - Capture B0 FIFO DMA Enable
 */
#define PWM_DMAEN_CB0DE(x) (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_CB0DE_SHIFT)) & PWM_DMAEN_CB0DE_MASK)

#define PWM_DMAEN_CB1DE_MASK  (0x8U)
#define PWM_DMAEN_CB1DE_SHIFT (3U)
/*! CB1DE - Capture B1 FIFO DMA Enable
 */
#define PWM_DMAEN_CB1DE(x) (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_CB1DE_SHIFT)) & PWM_DMAEN_CB1DE_MASK)

#define PWM_DMAEN_CA0DE_MASK  (0x10U)
#define PWM_DMAEN_CA0DE_SHIFT (4U)
/*! CA0DE - Capture A0 FIFO DMA Enable
 */
#define PWM_DMAEN_CA0DE(x) (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_CA0DE_SHIFT)) & PWM_DMAEN_CA0DE_MASK)

#define PWM_DMAEN_CA1DE_MASK  (0x20U)
#define PWM_DMAEN_CA1DE_SHIFT (5U)
/*! CA1DE - Capture A1 FIFO DMA Enable
 */
#define PWM_DMAEN_CA1DE(x) (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_CA1DE_SHIFT)) & PWM_DMAEN_CA1DE_MASK)

#define PWM_DMAEN_CAPTDE_MASK  (0xC0U)
#define PWM_DMAEN_CAPTDE_SHIFT (6U)
/*! CAPTDE - Capture DMA Enable Source Select
 *  0b00..Read DMA requests disabled.
 *  0b01..Exceeding a FIFO watermark sets the DMA read request. This requires at least one of DMAEN[CA1DE],
 *        DMAEN[CA0DE], DMAEN[CB1DE], DMAEN[CB0DE], DMAEN[CX1DE], or DMAEN[CX0DE] to also be set in order to determine
 * to which watermark(s) the DMA request is sensitive. 0b10..A local sync (VAL1 matches counter) sets the read DMA
 * request. 0b11..A local reload (STS[RF] being set) sets the read DMA request.
 */
#define PWM_DMAEN_CAPTDE(x) (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_CAPTDE_SHIFT)) & PWM_DMAEN_CAPTDE_MASK)

#define PWM_DMAEN_FAND_MASK  (0x100U)
#define PWM_DMAEN_FAND_SHIFT (8U)
/*! FAND - FIFO Watermark AND Control
 *  0b0..Selected FIFO watermarks are OR'ed together.
 *  0b1..Selected FIFO watermarks are AND'ed together.
 */
#define PWM_DMAEN_FAND(x) (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_FAND_SHIFT)) & PWM_DMAEN_FAND_MASK)

#define PWM_DMAEN_VALDE_MASK  (0x200U)
#define PWM_DMAEN_VALDE_SHIFT (9U)
/*! VALDE - Value Registers DMA Enable
 *  0b0..DMA write requests disabled
 *  0b1..DMA write requests for the VALx and FRACVALx registers enabled
 */
#define PWM_DMAEN_VALDE(x) (((uint16_t)(((uint16_t)(x)) << PWM_DMAEN_VALDE_SHIFT)) & PWM_DMAEN_VALDE_MASK)
/*! @} */

/* The count of PWM_DMAEN */
#define PWM_DMAEN_COUNT (4U)

/*! @name TCTRL - Output Trigger Control Register */
/*! @{ */

#define PWM_TCTRL_OUT_TRIG_EN_MASK  (0x3FU)
#define PWM_TCTRL_OUT_TRIG_EN_SHIFT (0U)
/*! OUT_TRIG_EN - Output Trigger Enables
 *  0b000000..PWM_OUT_TRIGx will not set when the counter value matches the VALx value.
 *  0b000001..PWM_OUT_TRIGx will set when the counter value matches the VALx value.
 */
#define PWM_TCTRL_OUT_TRIG_EN(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_TCTRL_OUT_TRIG_EN_SHIFT)) & PWM_TCTRL_OUT_TRIG_EN_MASK)

#define PWM_TCTRL_TRGFRQ_MASK  (0x1000U)
#define PWM_TCTRL_TRGFRQ_SHIFT (12U)
/*! TRGFRQ - Trigger frequency
 *  0b0..Trigger outputs are generated during every PWM period even if the PWM is not reloaded every period due to
 * CTRL[LDFQ] being non-zero. 0b1..Trigger outputs are generated only during the final PWM period prior to a reload
 * opportunity when the PWM is not reloaded every period due to CTRL[LDFQ] being non-zero.
 */
#define PWM_TCTRL_TRGFRQ(x) (((uint16_t)(((uint16_t)(x)) << PWM_TCTRL_TRGFRQ_SHIFT)) & PWM_TCTRL_TRGFRQ_MASK)

#define PWM_TCTRL_PWBOT1_MASK  (0x4000U)
#define PWM_TCTRL_PWBOT1_SHIFT (14U)
/*! PWBOT1 - Output Trigger 1 Source Select
 *  0b0..Route the PWM_OUT_TRIG1 signal to PWM_OUT_TRIG1 port.
 *  0b1..Route the PWMB output to the PWM_OUT_TRIG1 port.
 */
#define PWM_TCTRL_PWBOT1(x) (((uint16_t)(((uint16_t)(x)) << PWM_TCTRL_PWBOT1_SHIFT)) & PWM_TCTRL_PWBOT1_MASK)

#define PWM_TCTRL_PWAOT0_MASK  (0x8000U)
#define PWM_TCTRL_PWAOT0_SHIFT (15U)
/*! PWAOT0 - Output Trigger 0 Source Select
 *  0b0..Route the PWM_OUT_TRIG0 signal to PWM_OUT_TRIG0 port.
 *  0b1..Route the PWMA output to the PWM_OUT_TRIG0 port.
 */
#define PWM_TCTRL_PWAOT0(x) (((uint16_t)(((uint16_t)(x)) << PWM_TCTRL_PWAOT0_SHIFT)) & PWM_TCTRL_PWAOT0_MASK)
/*! @} */

/* The count of PWM_TCTRL */
#define PWM_TCTRL_COUNT (4U)

/*! @name DISMAP - Fault Disable Mapping Register 0..Fault Disable Mapping Register 1 */
/*! @{ */

#define PWM_DISMAP_DIS0A_MASK  (0xFU)
#define PWM_DISMAP_DIS0A_SHIFT (0U)
/*! DIS0A - PWM_A Fault Disable Mask 0
 */
#define PWM_DISMAP_DIS0A(x) (((uint16_t)(((uint16_t)(x)) << PWM_DISMAP_DIS0A_SHIFT)) & PWM_DISMAP_DIS0A_MASK)

#define PWM_DISMAP_DIS1A_MASK  (0xFU)
#define PWM_DISMAP_DIS1A_SHIFT (0U)
/*! DIS1A - PWM_A Fault Disable Mask 1
 */
#define PWM_DISMAP_DIS1A(x) (((uint16_t)(((uint16_t)(x)) << PWM_DISMAP_DIS1A_SHIFT)) & PWM_DISMAP_DIS1A_MASK)

#define PWM_DISMAP_DIS0B_MASK  (0xF0U)
#define PWM_DISMAP_DIS0B_SHIFT (4U)
/*! DIS0B - PWM_B Fault Disable Mask 0
 */
#define PWM_DISMAP_DIS0B(x) (((uint16_t)(((uint16_t)(x)) << PWM_DISMAP_DIS0B_SHIFT)) & PWM_DISMAP_DIS0B_MASK)

#define PWM_DISMAP_DIS1B_MASK  (0xF0U)
#define PWM_DISMAP_DIS1B_SHIFT (4U)
/*! DIS1B - PWM_B Fault Disable Mask 1
 */
#define PWM_DISMAP_DIS1B(x) (((uint16_t)(((uint16_t)(x)) << PWM_DISMAP_DIS1B_SHIFT)) & PWM_DISMAP_DIS1B_MASK)

#define PWM_DISMAP_DIS0X_MASK  (0xF00U)
#define PWM_DISMAP_DIS0X_SHIFT (8U)
/*! DIS0X - PWM_X Fault Disable Mask 0
 */
#define PWM_DISMAP_DIS0X(x) (((uint16_t)(((uint16_t)(x)) << PWM_DISMAP_DIS0X_SHIFT)) & PWM_DISMAP_DIS0X_MASK)

#define PWM_DISMAP_DIS1X_MASK  (0xF00U)
#define PWM_DISMAP_DIS1X_SHIFT (8U)
/*! DIS1X - PWM_X Fault Disable Mask 1
 */
#define PWM_DISMAP_DIS1X(x) (((uint16_t)(((uint16_t)(x)) << PWM_DISMAP_DIS1X_SHIFT)) & PWM_DISMAP_DIS1X_MASK)
/*! @} */

/* The count of PWM_DISMAP */
#define PWM_DISMAP_COUNT (4U)

/* The count of PWM_DISMAP */
#define PWM_DISMAP_COUNT2 (2U)

/*! @name DTCNT0 - Deadtime Count Register 0 */
/*! @{ */

#define PWM_DTCNT0_DTCNT0_MASK  (0xFFFFU)
#define PWM_DTCNT0_DTCNT0_SHIFT (0U)
/*! DTCNT0 - DTCNT0
 */
#define PWM_DTCNT0_DTCNT0(x) (((uint16_t)(((uint16_t)(x)) << PWM_DTCNT0_DTCNT0_SHIFT)) & PWM_DTCNT0_DTCNT0_MASK)
/*! @} */

/* The count of PWM_DTCNT0 */
#define PWM_DTCNT0_COUNT (4U)

/*! @name DTCNT1 - Deadtime Count Register 1 */
/*! @{ */

#define PWM_DTCNT1_DTCNT1_MASK  (0xFFFFU)
#define PWM_DTCNT1_DTCNT1_SHIFT (0U)
/*! DTCNT1 - DTCNT1
 */
#define PWM_DTCNT1_DTCNT1(x) (((uint16_t)(((uint16_t)(x)) << PWM_DTCNT1_DTCNT1_SHIFT)) & PWM_DTCNT1_DTCNT1_MASK)
/*! @} */

/* The count of PWM_DTCNT1 */
#define PWM_DTCNT1_COUNT (4U)

/*! @name CAPTCTRLA - Capture Control A Register */
/*! @{ */

#define PWM_CAPTCTRLA_ARMA_MASK  (0x1U)
#define PWM_CAPTCTRLA_ARMA_SHIFT (0U)
/*! ARMA - Arm A
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLA[EDGAx] is enabled.
 */
#define PWM_CAPTCTRLA_ARMA(x) (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLA_ARMA_SHIFT)) & PWM_CAPTCTRLA_ARMA_MASK)

#define PWM_CAPTCTRLA_ONESHOTA_MASK  (0x2U)
#define PWM_CAPTCTRLA_ONESHOTA_SHIFT (1U)
/*! ONESHOTA - One Shot Mode A
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after CAPTCTRLA[ARMA] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1
 *       is armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed.
 *       The process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after CAPTCTRLA[ARMA] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and CAPTCTRLA[ARMA] is cleared. No
 *       further captures will be performed until CAPTCTRLA[ARMA] is set again.If only one of the capture circuits is
 *       enabled, then a single capture will occur on the enabled capture circuit and CAPTCTRLA[ARMA] is then cleared.
 */
#define PWM_CAPTCTRLA_ONESHOTA(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLA_ONESHOTA_SHIFT)) & PWM_CAPTCTRLA_ONESHOTA_MASK)

#define PWM_CAPTCTRLA_EDGA0_MASK  (0xCU)
#define PWM_CAPTCTRLA_EDGA0_SHIFT (2U)
/*! EDGA0 - Edge A 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_CAPTCTRLA_EDGA0(x) (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLA_EDGA0_SHIFT)) & PWM_CAPTCTRLA_EDGA0_MASK)

#define PWM_CAPTCTRLA_EDGA1_MASK  (0x30U)
#define PWM_CAPTCTRLA_EDGA1_SHIFT (4U)
/*! EDGA1 - Edge A 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_CAPTCTRLA_EDGA1(x) (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLA_EDGA1_SHIFT)) & PWM_CAPTCTRLA_EDGA1_MASK)

#define PWM_CAPTCTRLA_INP_SELA_MASK  (0x40U)
#define PWM_CAPTCTRLA_INP_SELA_SHIFT (6U)
/*! INP_SELA - Input Select A
 *  0b0..Raw PWM_A input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLA[EDGA0] and
 *       CAPTCTRLA[EDGA1] fields are ignored. The software must still place a value other than 00 in either or both of
 * the CAPTCTLRA[EDGA0] and/or CAPTCTRLA[EDGA1] fields in order to enable one or both of the capture registers.
 */
#define PWM_CAPTCTRLA_INP_SELA(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLA_INP_SELA_SHIFT)) & PWM_CAPTCTRLA_INP_SELA_MASK)

#define PWM_CAPTCTRLA_EDGCNTA_EN_MASK  (0x80U)
#define PWM_CAPTCTRLA_EDGCNTA_EN_SHIFT (7U)
/*! EDGCNTA_EN - Edge Counter A Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_CAPTCTRLA_EDGCNTA_EN(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLA_EDGCNTA_EN_SHIFT)) & PWM_CAPTCTRLA_EDGCNTA_EN_MASK)

#define PWM_CAPTCTRLA_CFAWM_MASK  (0x300U)
#define PWM_CAPTCTRLA_CFAWM_SHIFT (8U)
/*! CFAWM - Capture A FIFOs Water Mark
 */
#define PWM_CAPTCTRLA_CFAWM(x) (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLA_CFAWM_SHIFT)) & PWM_CAPTCTRLA_CFAWM_MASK)

#define PWM_CAPTCTRLA_CA0CNT_MASK  (0x1C00U)
#define PWM_CAPTCTRLA_CA0CNT_SHIFT (10U)
/*! CA0CNT - Capture A0 FIFO Word Count
 */
#define PWM_CAPTCTRLA_CA0CNT(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLA_CA0CNT_SHIFT)) & PWM_CAPTCTRLA_CA0CNT_MASK)

#define PWM_CAPTCTRLA_CA1CNT_MASK  (0xE000U)
#define PWM_CAPTCTRLA_CA1CNT_SHIFT (13U)
/*! CA1CNT - Capture A1 FIFO Word Count
 */
#define PWM_CAPTCTRLA_CA1CNT(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLA_CA1CNT_SHIFT)) & PWM_CAPTCTRLA_CA1CNT_MASK)
/*! @} */

/* The count of PWM_CAPTCTRLA */
#define PWM_CAPTCTRLA_COUNT (4U)

/*! @name CAPTCOMPA - Capture Compare A Register */
/*! @{ */

#define PWM_CAPTCOMPA_EDGCMPA_MASK  (0xFFU)
#define PWM_CAPTCOMPA_EDGCMPA_SHIFT (0U)
/*! EDGCMPA - Edge Compare A
 */
#define PWM_CAPTCOMPA_EDGCMPA(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCOMPA_EDGCMPA_SHIFT)) & PWM_CAPTCOMPA_EDGCMPA_MASK)

#define PWM_CAPTCOMPA_EDGCNTA_MASK  (0xFF00U)
#define PWM_CAPTCOMPA_EDGCNTA_SHIFT (8U)
/*! EDGCNTA - Edge Counter A
 */
#define PWM_CAPTCOMPA_EDGCNTA(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCOMPA_EDGCNTA_SHIFT)) & PWM_CAPTCOMPA_EDGCNTA_MASK)
/*! @} */

/* The count of PWM_CAPTCOMPA */
#define PWM_CAPTCOMPA_COUNT (4U)

/*! @name CAPTCTRLB - Capture Control B Register */
/*! @{ */

#define PWM_CAPTCTRLB_ARMB_MASK  (0x1U)
#define PWM_CAPTCTRLB_ARMB_SHIFT (0U)
/*! ARMB - Arm B
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLB[EDGBx] is enabled.
 */
#define PWM_CAPTCTRLB_ARMB(x) (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLB_ARMB_SHIFT)) & PWM_CAPTCTRLB_ARMB_MASK)

#define PWM_CAPTCTRLB_ONESHOTB_MASK  (0x2U)
#define PWM_CAPTCTRLB_ONESHOTB_SHIFT (1U)
/*! ONESHOTB - One Shot Mode B
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after CAPTCTRLB[ARMB] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1
 *       is armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed.
 *       The process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after CAPTCTRLB[ARMB] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and CAPTCTRLB[ARMB] is cleared. No
 *       further captures will be performed until CAPTCTRLB[ARMB] is set again.If only one of the capture circuits is
 *       enabled, then a single capture will occur on the enabled capture circuit and CAPTCTRLB[ARMB] is then cleared.
 */
#define PWM_CAPTCTRLB_ONESHOTB(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLB_ONESHOTB_SHIFT)) & PWM_CAPTCTRLB_ONESHOTB_MASK)

#define PWM_CAPTCTRLB_EDGB0_MASK  (0xCU)
#define PWM_CAPTCTRLB_EDGB0_SHIFT (2U)
/*! EDGB0 - Edge B 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_CAPTCTRLB_EDGB0(x) (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLB_EDGB0_SHIFT)) & PWM_CAPTCTRLB_EDGB0_MASK)

#define PWM_CAPTCTRLB_EDGB1_MASK  (0x30U)
#define PWM_CAPTCTRLB_EDGB1_SHIFT (4U)
/*! EDGB1 - Edge B 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_CAPTCTRLB_EDGB1(x) (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLB_EDGB1_SHIFT)) & PWM_CAPTCTRLB_EDGB1_MASK)

#define PWM_CAPTCTRLB_INP_SELB_MASK  (0x40U)
#define PWM_CAPTCTRLB_INP_SELB_SHIFT (6U)
/*! INP_SELB - Input Select B
 *  0b0..Raw PWM_B input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLB[EDGB0] and
 *       CAPTCTRLB[EDGB1] fields are ignored. The software must still place a value other than 00 in either or both of
 * the CAPTCTLRB[EDGB0] and/or CAPTCTRLB[EDGB1] fields in order to enable one or both of the capture registers.
 */
#define PWM_CAPTCTRLB_INP_SELB(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLB_INP_SELB_SHIFT)) & PWM_CAPTCTRLB_INP_SELB_MASK)

#define PWM_CAPTCTRLB_EDGCNTB_EN_MASK  (0x80U)
#define PWM_CAPTCTRLB_EDGCNTB_EN_SHIFT (7U)
/*! EDGCNTB_EN - Edge Counter B Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_CAPTCTRLB_EDGCNTB_EN(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLB_EDGCNTB_EN_SHIFT)) & PWM_CAPTCTRLB_EDGCNTB_EN_MASK)

#define PWM_CAPTCTRLB_CFBWM_MASK  (0x300U)
#define PWM_CAPTCTRLB_CFBWM_SHIFT (8U)
/*! CFBWM - Capture B FIFOs Water Mark
 */
#define PWM_CAPTCTRLB_CFBWM(x) (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLB_CFBWM_SHIFT)) & PWM_CAPTCTRLB_CFBWM_MASK)

#define PWM_CAPTCTRLB_CB0CNT_MASK  (0x1C00U)
#define PWM_CAPTCTRLB_CB0CNT_SHIFT (10U)
/*! CB0CNT - Capture B0 FIFO Word Count
 */
#define PWM_CAPTCTRLB_CB0CNT(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLB_CB0CNT_SHIFT)) & PWM_CAPTCTRLB_CB0CNT_MASK)

#define PWM_CAPTCTRLB_CB1CNT_MASK  (0xE000U)
#define PWM_CAPTCTRLB_CB1CNT_SHIFT (13U)
/*! CB1CNT - Capture B1 FIFO Word Count
 */
#define PWM_CAPTCTRLB_CB1CNT(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLB_CB1CNT_SHIFT)) & PWM_CAPTCTRLB_CB1CNT_MASK)
/*! @} */

/* The count of PWM_CAPTCTRLB */
#define PWM_CAPTCTRLB_COUNT (4U)

/*! @name CAPTCOMPB - Capture Compare B Register */
/*! @{ */

#define PWM_CAPTCOMPB_EDGCMPB_MASK  (0xFFU)
#define PWM_CAPTCOMPB_EDGCMPB_SHIFT (0U)
/*! EDGCMPB - Edge Compare B
 */
#define PWM_CAPTCOMPB_EDGCMPB(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCOMPB_EDGCMPB_SHIFT)) & PWM_CAPTCOMPB_EDGCMPB_MASK)

#define PWM_CAPTCOMPB_EDGCNTB_MASK  (0xFF00U)
#define PWM_CAPTCOMPB_EDGCNTB_SHIFT (8U)
/*! EDGCNTB - Edge Counter B
 */
#define PWM_CAPTCOMPB_EDGCNTB(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCOMPB_EDGCNTB_SHIFT)) & PWM_CAPTCOMPB_EDGCNTB_MASK)
/*! @} */

/* The count of PWM_CAPTCOMPB */
#define PWM_CAPTCOMPB_COUNT (4U)

/*! @name CAPTCTRLX - Capture Control X Register */
/*! @{ */

#define PWM_CAPTCTRLX_ARMX_MASK  (0x1U)
#define PWM_CAPTCTRLX_ARMX_SHIFT (0U)
/*! ARMX - Arm X
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLX[EDGXx] is enabled.
 */
#define PWM_CAPTCTRLX_ARMX(x) (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_ARMX_SHIFT)) & PWM_CAPTCTRLX_ARMX_MASK)

#define PWM_CAPTCTRLX_ONESHOTX_MASK  (0x2U)
#define PWM_CAPTCTRLX_ONESHOTX_SHIFT (1U)
/*! ONESHOTX - One Shot Mode Aux
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after the ARMX bit is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed. The
 *       process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after the ARMX bit is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and the ARMX bit is cleared. No further
 *       captures will be performed until the ARMX bit is set again.If only one of the capture circuits is enabled,
 *       then a single capture will occur on the enabled capture circuit and the ARMX bit is then cleared.
 */
#define PWM_CAPTCTRLX_ONESHOTX(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_ONESHOTX_SHIFT)) & PWM_CAPTCTRLX_ONESHOTX_MASK)

#define PWM_CAPTCTRLX_EDGX0_MASK  (0xCU)
#define PWM_CAPTCTRLX_EDGX0_SHIFT (2U)
/*! EDGX0 - Edge X 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_CAPTCTRLX_EDGX0(x) (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_EDGX0_SHIFT)) & PWM_CAPTCTRLX_EDGX0_MASK)

#define PWM_CAPTCTRLX_EDGX1_MASK  (0x30U)
#define PWM_CAPTCTRLX_EDGX1_SHIFT (4U)
/*! EDGX1 - Edge X 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_CAPTCTRLX_EDGX1(x) (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_EDGX1_SHIFT)) & PWM_CAPTCTRLX_EDGX1_MASK)

#define PWM_CAPTCTRLX_INP_SELX_MASK  (0x40U)
#define PWM_CAPTCTRLX_INP_SELX_SHIFT (6U)
/*! INP_SELX - Input Select X
 *  0b0..Raw PWM_X input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLX[EDGX0] and
 *       CAPTCTRLX[EDGX1] fields are ignored. The software must still place a value other than 00 in either or both of
 * the CAPTCTLRX[EDGX0] and/or CAPTCTRLX[EDGX1] fields in order to enable one or both of the capture registers.
 */
#define PWM_CAPTCTRLX_INP_SELX(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_INP_SELX_SHIFT)) & PWM_CAPTCTRLX_INP_SELX_MASK)

#define PWM_CAPTCTRLX_EDGCNTX_EN_MASK  (0x80U)
#define PWM_CAPTCTRLX_EDGCNTX_EN_SHIFT (7U)
/*! EDGCNTX_EN - Edge Counter X Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_CAPTCTRLX_EDGCNTX_EN(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_EDGCNTX_EN_SHIFT)) & PWM_CAPTCTRLX_EDGCNTX_EN_MASK)

#define PWM_CAPTCTRLX_CFXWM_MASK  (0x300U)
#define PWM_CAPTCTRLX_CFXWM_SHIFT (8U)
/*! CFXWM - Capture X FIFOs Water Mark
 */
#define PWM_CAPTCTRLX_CFXWM(x) (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_CFXWM_SHIFT)) & PWM_CAPTCTRLX_CFXWM_MASK)

#define PWM_CAPTCTRLX_CX0CNT_MASK  (0x1C00U)
#define PWM_CAPTCTRLX_CX0CNT_SHIFT (10U)
/*! CX0CNT - Capture X0 FIFO Word Count
 */
#define PWM_CAPTCTRLX_CX0CNT(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_CX0CNT_SHIFT)) & PWM_CAPTCTRLX_CX0CNT_MASK)

#define PWM_CAPTCTRLX_CX1CNT_MASK  (0xE000U)
#define PWM_CAPTCTRLX_CX1CNT_SHIFT (13U)
/*! CX1CNT - Capture X1 FIFO Word Count
 */
#define PWM_CAPTCTRLX_CX1CNT(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCTRLX_CX1CNT_SHIFT)) & PWM_CAPTCTRLX_CX1CNT_MASK)
/*! @} */

/* The count of PWM_CAPTCTRLX */
#define PWM_CAPTCTRLX_COUNT (4U)

/*! @name CAPTCOMPX - Capture Compare X Register */
/*! @{ */

#define PWM_CAPTCOMPX_EDGCMPX_MASK  (0xFFU)
#define PWM_CAPTCOMPX_EDGCMPX_SHIFT (0U)
/*! EDGCMPX - Edge Compare X
 */
#define PWM_CAPTCOMPX_EDGCMPX(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCOMPX_EDGCMPX_SHIFT)) & PWM_CAPTCOMPX_EDGCMPX_MASK)

#define PWM_CAPTCOMPX_EDGCNTX_MASK  (0xFF00U)
#define PWM_CAPTCOMPX_EDGCNTX_SHIFT (8U)
/*! EDGCNTX - Edge Counter X
 */
#define PWM_CAPTCOMPX_EDGCNTX(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CAPTCOMPX_EDGCNTX_SHIFT)) & PWM_CAPTCOMPX_EDGCNTX_MASK)
/*! @} */

/* The count of PWM_CAPTCOMPX */
#define PWM_CAPTCOMPX_COUNT (4U)

/*! @name CVAL0 - Capture Value 0 Register */
/*! @{ */

#define PWM_CVAL0_CAPTVAL0_MASK  (0xFFFFU)
#define PWM_CVAL0_CAPTVAL0_SHIFT (0U)
/*! CAPTVAL0 - CAPTVAL0
 */
#define PWM_CVAL0_CAPTVAL0(x) (((uint16_t)(((uint16_t)(x)) << PWM_CVAL0_CAPTVAL0_SHIFT)) & PWM_CVAL0_CAPTVAL0_MASK)
/*! @} */

/* The count of PWM_CVAL0 */
#define PWM_CVAL0_COUNT (4U)

/*! @name CVAL0CYC - Capture Value 0 Cycle Register */
/*! @{ */

#define PWM_CVAL0CYC_CVAL0CYC_MASK  (0xFU)
#define PWM_CVAL0CYC_CVAL0CYC_SHIFT (0U)
/*! CVAL0CYC - CVAL0CYC
 */
#define PWM_CVAL0CYC_CVAL0CYC(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CVAL0CYC_CVAL0CYC_SHIFT)) & PWM_CVAL0CYC_CVAL0CYC_MASK)
/*! @} */

/* The count of PWM_CVAL0CYC */
#define PWM_CVAL0CYC_COUNT (4U)

/*! @name CVAL1 - Capture Value 1 Register */
/*! @{ */

#define PWM_CVAL1_CAPTVAL1_MASK  (0xFFFFU)
#define PWM_CVAL1_CAPTVAL1_SHIFT (0U)
/*! CAPTVAL1 - CAPTVAL1
 */
#define PWM_CVAL1_CAPTVAL1(x) (((uint16_t)(((uint16_t)(x)) << PWM_CVAL1_CAPTVAL1_SHIFT)) & PWM_CVAL1_CAPTVAL1_MASK)
/*! @} */

/* The count of PWM_CVAL1 */
#define PWM_CVAL1_COUNT (4U)

/*! @name CVAL1CYC - Capture Value 1 Cycle Register */
/*! @{ */

#define PWM_CVAL1CYC_CVAL1CYC_MASK  (0xFU)
#define PWM_CVAL1CYC_CVAL1CYC_SHIFT (0U)
/*! CVAL1CYC - CVAL1CYC
 */
#define PWM_CVAL1CYC_CVAL1CYC(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CVAL1CYC_CVAL1CYC_SHIFT)) & PWM_CVAL1CYC_CVAL1CYC_MASK)
/*! @} */

/* The count of PWM_CVAL1CYC */
#define PWM_CVAL1CYC_COUNT (4U)

/*! @name CVAL2 - Capture Value 2 Register */
/*! @{ */

#define PWM_CVAL2_CAPTVAL2_MASK  (0xFFFFU)
#define PWM_CVAL2_CAPTVAL2_SHIFT (0U)
/*! CAPTVAL2 - CAPTVAL2
 */
#define PWM_CVAL2_CAPTVAL2(x) (((uint16_t)(((uint16_t)(x)) << PWM_CVAL2_CAPTVAL2_SHIFT)) & PWM_CVAL2_CAPTVAL2_MASK)
/*! @} */

/* The count of PWM_CVAL2 */
#define PWM_CVAL2_COUNT (4U)

/*! @name CVAL2CYC - Capture Value 2 Cycle Register */
/*! @{ */

#define PWM_CVAL2CYC_CVAL2CYC_MASK  (0xFU)
#define PWM_CVAL2CYC_CVAL2CYC_SHIFT (0U)
/*! CVAL2CYC - CVAL2CYC
 */
#define PWM_CVAL2CYC_CVAL2CYC(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CVAL2CYC_CVAL2CYC_SHIFT)) & PWM_CVAL2CYC_CVAL2CYC_MASK)
/*! @} */

/* The count of PWM_CVAL2CYC */
#define PWM_CVAL2CYC_COUNT (4U)

/*! @name CVAL3 - Capture Value 3 Register */
/*! @{ */

#define PWM_CVAL3_CAPTVAL3_MASK  (0xFFFFU)
#define PWM_CVAL3_CAPTVAL3_SHIFT (0U)
/*! CAPTVAL3 - CAPTVAL3
 */
#define PWM_CVAL3_CAPTVAL3(x) (((uint16_t)(((uint16_t)(x)) << PWM_CVAL3_CAPTVAL3_SHIFT)) & PWM_CVAL3_CAPTVAL3_MASK)
/*! @} */

/* The count of PWM_CVAL3 */
#define PWM_CVAL3_COUNT (4U)

/*! @name CVAL3CYC - Capture Value 3 Cycle Register */
/*! @{ */

#define PWM_CVAL3CYC_CVAL3CYC_MASK  (0xFU)
#define PWM_CVAL3CYC_CVAL3CYC_SHIFT (0U)
/*! CVAL3CYC - CVAL3CYC
 */
#define PWM_CVAL3CYC_CVAL3CYC(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CVAL3CYC_CVAL3CYC_SHIFT)) & PWM_CVAL3CYC_CVAL3CYC_MASK)
/*! @} */

/* The count of PWM_CVAL3CYC */
#define PWM_CVAL3CYC_COUNT (4U)

/*! @name CVAL4 - Capture Value 4 Register */
/*! @{ */

#define PWM_CVAL4_CAPTVAL4_MASK  (0xFFFFU)
#define PWM_CVAL4_CAPTVAL4_SHIFT (0U)
/*! CAPTVAL4 - CAPTVAL4
 */
#define PWM_CVAL4_CAPTVAL4(x) (((uint16_t)(((uint16_t)(x)) << PWM_CVAL4_CAPTVAL4_SHIFT)) & PWM_CVAL4_CAPTVAL4_MASK)
/*! @} */

/* The count of PWM_CVAL4 */
#define PWM_CVAL4_COUNT (4U)

/*! @name CVAL4CYC - Capture Value 4 Cycle Register */
/*! @{ */

#define PWM_CVAL4CYC_CVAL4CYC_MASK  (0xFU)
#define PWM_CVAL4CYC_CVAL4CYC_SHIFT (0U)
/*! CVAL4CYC - CVAL4CYC
 */
#define PWM_CVAL4CYC_CVAL4CYC(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CVAL4CYC_CVAL4CYC_SHIFT)) & PWM_CVAL4CYC_CVAL4CYC_MASK)
/*! @} */

/* The count of PWM_CVAL4CYC */
#define PWM_CVAL4CYC_COUNT (4U)

/*! @name CVAL5 - Capture Value 5 Register */
/*! @{ */

#define PWM_CVAL5_CAPTVAL5_MASK  (0xFFFFU)
#define PWM_CVAL5_CAPTVAL5_SHIFT (0U)
/*! CAPTVAL5 - CAPTVAL5
 */
#define PWM_CVAL5_CAPTVAL5(x) (((uint16_t)(((uint16_t)(x)) << PWM_CVAL5_CAPTVAL5_SHIFT)) & PWM_CVAL5_CAPTVAL5_MASK)
/*! @} */

/* The count of PWM_CVAL5 */
#define PWM_CVAL5_COUNT (4U)

/*! @name CVAL5CYC - Capture Value 5 Cycle Register */
/*! @{ */

#define PWM_CVAL5CYC_CVAL5CYC_MASK  (0xFU)
#define PWM_CVAL5CYC_CVAL5CYC_SHIFT (0U)
/*! CVAL5CYC - CVAL5CYC
 */
#define PWM_CVAL5CYC_CVAL5CYC(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_CVAL5CYC_CVAL5CYC_SHIFT)) & PWM_CVAL5CYC_CVAL5CYC_MASK)
/*! @} */

/* The count of PWM_CVAL5CYC */
#define PWM_CVAL5CYC_COUNT (4U)

/*! @name PHASEDLY - Phase Delay Register */
/*! @{ */

#define PWM_PHASEDLY_PHASEDLY_MASK  (0xFFFFU)
#define PWM_PHASEDLY_PHASEDLY_SHIFT (0U)
/*! PHASEDLY - Initial Count Register Bits
 */
#define PWM_PHASEDLY_PHASEDLY(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_PHASEDLY_PHASEDLY_SHIFT)) & PWM_PHASEDLY_PHASEDLY_MASK)
/*! @} */

/* The count of PWM_PHASEDLY */
#define PWM_PHASEDLY_COUNT (4U)

/*! @name OUTEN - Output Enable Register */
/*! @{ */

#define PWM_OUTEN_PWMX_EN_MASK  (0xFU)
#define PWM_OUTEN_PWMX_EN_SHIFT (0U)
/*! PWMX_EN - PWM_X Output Enables
 *  0b0000..PWM_X output disabled.
 *  0b0001..PWM_X output enabled.
 */
#define PWM_OUTEN_PWMX_EN(x) (((uint16_t)(((uint16_t)(x)) << PWM_OUTEN_PWMX_EN_SHIFT)) & PWM_OUTEN_PWMX_EN_MASK)

#define PWM_OUTEN_PWMB_EN_MASK  (0xF0U)
#define PWM_OUTEN_PWMB_EN_SHIFT (4U)
/*! PWMB_EN - PWM_B Output Enables
 *  0b0000..PWM_B output disabled.
 *  0b0001..PWM_B output enabled.
 */
#define PWM_OUTEN_PWMB_EN(x) (((uint16_t)(((uint16_t)(x)) << PWM_OUTEN_PWMB_EN_SHIFT)) & PWM_OUTEN_PWMB_EN_MASK)

#define PWM_OUTEN_PWMA_EN_MASK  (0xF00U)
#define PWM_OUTEN_PWMA_EN_SHIFT (8U)
/*! PWMA_EN - PWM_A Output Enables
 *  0b0000..PWM_A output disabled.
 *  0b0001..PWM_A output enabled.
 */
#define PWM_OUTEN_PWMA_EN(x) (((uint16_t)(((uint16_t)(x)) << PWM_OUTEN_PWMA_EN_SHIFT)) & PWM_OUTEN_PWMA_EN_MASK)
/*! @} */

/*! @name MASK - Mask Register */
/*! @{ */

#define PWM_MASK_MASKX_MASK  (0xFU)
#define PWM_MASK_MASKX_SHIFT (0U)
/*! MASKX - PWM_X Masks
 *  0b0000..PWM_X output normal.
 *  0b0001..PWM_X output masked.
 */
#define PWM_MASK_MASKX(x) (((uint16_t)(((uint16_t)(x)) << PWM_MASK_MASKX_SHIFT)) & PWM_MASK_MASKX_MASK)

#define PWM_MASK_MASKB_MASK  (0xF0U)
#define PWM_MASK_MASKB_SHIFT (4U)
/*! MASKB - PWM_B Masks
 *  0b0000..PWM_B output normal.
 *  0b0001..PWM_B output masked.
 */
#define PWM_MASK_MASKB(x) (((uint16_t)(((uint16_t)(x)) << PWM_MASK_MASKB_SHIFT)) & PWM_MASK_MASKB_MASK)

#define PWM_MASK_MASKA_MASK  (0xF00U)
#define PWM_MASK_MASKA_SHIFT (8U)
/*! MASKA - PWM_A Masks
 *  0b0000..PWM_A output normal.
 *  0b0001..PWM_A output masked.
 */
#define PWM_MASK_MASKA(x) (((uint16_t)(((uint16_t)(x)) << PWM_MASK_MASKA_SHIFT)) & PWM_MASK_MASKA_MASK)

#define PWM_MASK_UPDATE_MASK_MASK  (0xF000U)
#define PWM_MASK_UPDATE_MASK_SHIFT (12U)
/*! UPDATE_MASK - Update Mask Bits Immediately
 *  0b0000..Normal operation. MASK* bits within the corresponding submodule are not updated until a FORCE_OUT event
 * occurs within the submodule. 0b0001..Immediate operation. MASK* bits within the corresponding submodule are updated
 * on the following clock edge after setting this bit.
 */
#define PWM_MASK_UPDATE_MASK(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_MASK_UPDATE_MASK_SHIFT)) & PWM_MASK_UPDATE_MASK_MASK)
/*! @} */

/*! @name SWCOUT - Software Controlled Output Register */
/*! @{ */

#define PWM_SWCOUT_SM0OUT45_MASK  (0x1U)
#define PWM_SWCOUT_SM0OUT45_SHIFT (0U)
/*! SM0OUT45 - Submodule 0 Software Controlled Output 45
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 0 instead of PWM45.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 0 instead of PWM45.
 */
#define PWM_SWCOUT_SM0OUT45(x) (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM0OUT45_SHIFT)) & PWM_SWCOUT_SM0OUT45_MASK)

#define PWM_SWCOUT_SM0OUT23_MASK  (0x2U)
#define PWM_SWCOUT_SM0OUT23_SHIFT (1U)
/*! SM0OUT23 - Submodule 0 Software Controlled Output 23
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 0 instead of PWM23.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 0 instead of PWM23.
 */
#define PWM_SWCOUT_SM0OUT23(x) (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM0OUT23_SHIFT)) & PWM_SWCOUT_SM0OUT23_MASK)

#define PWM_SWCOUT_SM1OUT45_MASK  (0x4U)
#define PWM_SWCOUT_SM1OUT45_SHIFT (2U)
/*! SM1OUT45 - Submodule 1 Software Controlled Output 45
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 1 instead of PWM45.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 1 instead of PWM45.
 */
#define PWM_SWCOUT_SM1OUT45(x) (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM1OUT45_SHIFT)) & PWM_SWCOUT_SM1OUT45_MASK)

#define PWM_SWCOUT_SM1OUT23_MASK  (0x8U)
#define PWM_SWCOUT_SM1OUT23_SHIFT (3U)
/*! SM1OUT23 - Submodule 1 Software Controlled Output 23
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 1 instead of PWM23.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 1 instead of PWM23.
 */
#define PWM_SWCOUT_SM1OUT23(x) (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM1OUT23_SHIFT)) & PWM_SWCOUT_SM1OUT23_MASK)

#define PWM_SWCOUT_SM2OUT45_MASK  (0x10U)
#define PWM_SWCOUT_SM2OUT45_SHIFT (4U)
/*! SM2OUT45 - Submodule 2 Software Controlled Output 45
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 2 instead of PWM45.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 2 instead of PWM45.
 */
#define PWM_SWCOUT_SM2OUT45(x) (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM2OUT45_SHIFT)) & PWM_SWCOUT_SM2OUT45_MASK)

#define PWM_SWCOUT_SM2OUT23_MASK  (0x20U)
#define PWM_SWCOUT_SM2OUT23_SHIFT (5U)
/*! SM2OUT23 - Submodule 2 Software Controlled Output 23
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 2 instead of PWM23.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 2 instead of PWM23.
 */
#define PWM_SWCOUT_SM2OUT23(x) (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM2OUT23_SHIFT)) & PWM_SWCOUT_SM2OUT23_MASK)

#define PWM_SWCOUT_SM3OUT45_MASK  (0x40U)
#define PWM_SWCOUT_SM3OUT45_SHIFT (6U)
/*! SM3OUT45 - Submodule 3 Software Controlled Output 45
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 3 instead of PWM45.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 3 instead of PWM45.
 */
#define PWM_SWCOUT_SM3OUT45(x) (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM3OUT45_SHIFT)) & PWM_SWCOUT_SM3OUT45_MASK)

#define PWM_SWCOUT_SM3OUT23_MASK  (0x80U)
#define PWM_SWCOUT_SM3OUT23_SHIFT (7U)
/*! SM3OUT23 - Submodule 3 Software Controlled Output 23
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 3 instead of PWM23.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 3 instead of PWM23.
 */
#define PWM_SWCOUT_SM3OUT23(x) (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM3OUT23_SHIFT)) & PWM_SWCOUT_SM3OUT23_MASK)
/*! @} */

/*! @name DTSRCSEL - PWM Source Select Register */
/*! @{ */

#define PWM_DTSRCSEL_SM0SEL45_MASK  (0x3U)
#define PWM_DTSRCSEL_SM0SEL45_SHIFT (0U)
/*! SM0SEL45 - Submodule 0 PWM45 Control Select
 *  0b00..Generated SM0PWM45 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM0PWM45 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM0OUT45] is used by the deadtime logic.
 *  0b11..PWM0_EXTB signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM0SEL45(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM0SEL45_SHIFT)) & PWM_DTSRCSEL_SM0SEL45_MASK)

#define PWM_DTSRCSEL_SM0SEL23_MASK  (0xCU)
#define PWM_DTSRCSEL_SM0SEL23_SHIFT (2U)
/*! SM0SEL23 - Submodule 0 PWM23 Control Select
 *  0b00..Generated SM0PWM23 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM0PWM23 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM0OUT23] is used by the deadtime logic.
 *  0b11..PWM0_EXTA signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM0SEL23(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM0SEL23_SHIFT)) & PWM_DTSRCSEL_SM0SEL23_MASK)

#define PWM_DTSRCSEL_SM1SEL45_MASK  (0x30U)
#define PWM_DTSRCSEL_SM1SEL45_SHIFT (4U)
/*! SM1SEL45 - Submodule 1 PWM45 Control Select
 *  0b00..Generated SM1PWM45 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM1PWM45 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM1OUT45] is used by the deadtime logic.
 *  0b11..PWM1_EXTB signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM1SEL45(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM1SEL45_SHIFT)) & PWM_DTSRCSEL_SM1SEL45_MASK)

#define PWM_DTSRCSEL_SM1SEL23_MASK  (0xC0U)
#define PWM_DTSRCSEL_SM1SEL23_SHIFT (6U)
/*! SM1SEL23 - Submodule 1 PWM23 Control Select
 *  0b00..Generated SM1PWM23 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM1PWM23 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM1OUT23] is used by the deadtime logic.
 *  0b11..PWM1_EXTA signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM1SEL23(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM1SEL23_SHIFT)) & PWM_DTSRCSEL_SM1SEL23_MASK)

#define PWM_DTSRCSEL_SM2SEL45_MASK  (0x300U)
#define PWM_DTSRCSEL_SM2SEL45_SHIFT (8U)
/*! SM2SEL45 - Submodule 2 PWM45 Control Select
 *  0b00..Generated SM2PWM45 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM2PWM45 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM2OUT45] is used by the deadtime logic.
 *  0b11..PWM2_EXTB signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM2SEL45(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM2SEL45_SHIFT)) & PWM_DTSRCSEL_SM2SEL45_MASK)

#define PWM_DTSRCSEL_SM2SEL23_MASK  (0xC00U)
#define PWM_DTSRCSEL_SM2SEL23_SHIFT (10U)
/*! SM2SEL23 - Submodule 2 PWM23 Control Select
 *  0b00..Generated SM2PWM23 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM2PWM23 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM2OUT23] is used by the deadtime logic.
 *  0b11..PWM2_EXTA signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM2SEL23(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM2SEL23_SHIFT)) & PWM_DTSRCSEL_SM2SEL23_MASK)

#define PWM_DTSRCSEL_SM3SEL45_MASK  (0x3000U)
#define PWM_DTSRCSEL_SM3SEL45_SHIFT (12U)
/*! SM3SEL45 - Submodule 3 PWM45 Control Select
 *  0b00..Generated SM3PWM45 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM3PWM45 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM3OUT45] is used by the deadtime logic.
 *  0b11..PWM3_EXTB signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM3SEL45(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM3SEL45_SHIFT)) & PWM_DTSRCSEL_SM3SEL45_MASK)

#define PWM_DTSRCSEL_SM3SEL23_MASK  (0xC000U)
#define PWM_DTSRCSEL_SM3SEL23_SHIFT (14U)
/*! SM3SEL23 - Submodule 3 PWM23 Control Select
 *  0b00..Generated SM3PWM23 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM3PWM23 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM3OUT23] is used by the deadtime logic.
 *  0b11..PWM3_EXTA signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM3SEL23(x) \
    (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM3SEL23_SHIFT)) & PWM_DTSRCSEL_SM3SEL23_MASK)
/*! @} */

/*! @name MCTRL - Master Control Register */
/*! @{ */

#define PWM_MCTRL_LDOK_MASK  (0xFU)
#define PWM_MCTRL_LDOK_SHIFT (0U)
/*! LDOK - Load Okay
 *  0b0000..Do not load new values.
 *  0b0001..Load prescaler, modulus, and PWM values of the corresponding submodule.
 */
#define PWM_MCTRL_LDOK(x) (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL_LDOK_SHIFT)) & PWM_MCTRL_LDOK_MASK)

#define PWM_MCTRL_CLDOK_MASK  (0xF0U)
#define PWM_MCTRL_CLDOK_SHIFT (4U)
/*! CLDOK - Clear Load Okay
 */
#define PWM_MCTRL_CLDOK(x) (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL_CLDOK_SHIFT)) & PWM_MCTRL_CLDOK_MASK)

#define PWM_MCTRL_RUN_MASK  (0xF00U)
#define PWM_MCTRL_RUN_SHIFT (8U)
/*! RUN - Run
 *  0b0000..PWM generator is disabled in the corresponding submodule.
 *  0b0001..PWM generator is enabled in the corresponding submodule.
 */
#define PWM_MCTRL_RUN(x) (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL_RUN_SHIFT)) & PWM_MCTRL_RUN_MASK)

#define PWM_MCTRL_IPOL_MASK  (0xF000U)
#define PWM_MCTRL_IPOL_SHIFT (12U)
/*! IPOL - Current Polarity
 *  0b0000..PWM23 is used to generate complementary PWM pair in the corresponding submodule.
 *  0b0001..PWM45 is used to generate complementary PWM pair in the corresponding submodule.
 */
#define PWM_MCTRL_IPOL(x) (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL_IPOL_SHIFT)) & PWM_MCTRL_IPOL_MASK)
/*! @} */

/*! @name MCTRL2 - Master Control 2 Register */
/*! @{ */

#define PWM_MCTRL2_MONPLL_MASK  (0x3U)
#define PWM_MCTRL2_MONPLL_SHIFT (0U)
/*! MONPLL - Monitor PLL State
 *  0b00..Not locked. Do not monitor PLL operation. Resetting of the fractional delay block in case of PLL losing lock
 * will be controlled by software. 0b01..Not locked. Monitor PLL operation to automatically disable the fractional delay
 * block when the PLL encounters problems. 0b10..Locked. Do not monitor PLL operation. Resetting of the fractional delay
 * block in case of PLL losing lock will be controlled by software. These bits are write protected until the next reset.
 *  0b11..Locked. Monitor PLL operation to automatically disable the fractional delay block when the PLL
 *        encounters problems. These bits are write protected until the next reset.
 */
#define PWM_MCTRL2_MONPLL(x) (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL2_MONPLL_SHIFT)) & PWM_MCTRL2_MONPLL_MASK)
/*! @} */

/*! @name FCTRL - Fault Control Register */
/*! @{ */

#define PWM_FCTRL_FIE_MASK  (0xFU)
#define PWM_FCTRL_FIE_SHIFT (0U)
/*! FIE - Fault Interrupt Enables
 *  0b0000..FAULTx CPU interrupt requests disabled.
 *  0b0001..FAULTx CPU interrupt requests enabled.
 */
#define PWM_FCTRL_FIE(x) (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL_FIE_SHIFT)) & PWM_FCTRL_FIE_MASK)

#define PWM_FCTRL_FSAFE_MASK  (0xF0U)
#define PWM_FCTRL_FSAFE_SHIFT (4U)
/*! FSAFE - Fault Safety Mode
 *  0b0000..Normal mode. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear at the
 *          start of a half cycle or full cycle depending on the states of FSTS[FHALF] and FSTS[FFULL] without regard
 *          to the state of FSTS[FFPINx]. If neither FHALF nor FFULL is setm then the fault condition cannot be
 *          cleared. The PWM outputs disabled by this fault input will not be re-enabled until the actual FAULTx input
 *          signal de-asserts since the fault input will combinationally disable the PWM outputs (as programmed in
 *          DISMAPn).
 *  0b0001..Safe mode. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear and
 *          FSTS[FFPINx] is clear at the start of a half cycle or full cycle depending on the states of FSTS[FHALF] and
 *          FSTS[FFULL]. If neither FHLAF nor FFULL is set, then the fault condition cannot be cleared.
 */
#define PWM_FCTRL_FSAFE(x) (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL_FSAFE_SHIFT)) & PWM_FCTRL_FSAFE_MASK)

#define PWM_FCTRL_FAUTO_MASK  (0xF00U)
#define PWM_FCTRL_FAUTO_SHIFT (8U)
/*! FAUTO - Automatic Fault Clearing
 *  0b0000..Manual fault clearing. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear
 *          at the start of a half cycle or full cycle depending the states of FSTS[FHALF] and FSTS[FFULL]. If
 *          neither FFULL nor FHALF is set, then the fault condition cannot be cleared. This is further controlled by
 *          FCTRL[FSAFE].
 *  0b0001..Automatic fault clearing. PWM outputs disabled by this fault are enabled when FSTS[FFPINx] is clear at
 *          the start of a half cycle or full cycle depending on the states of FSTS[FHALF] and FSTS[FFULL] without
 *          regard to the state of FSTS[FFLAGx]. If neither FFULL nor FHALF is set, then the fault condition
 *          cannot be cleared.
 */
#define PWM_FCTRL_FAUTO(x) (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL_FAUTO_SHIFT)) & PWM_FCTRL_FAUTO_MASK)

#define PWM_FCTRL_FLVL_MASK  (0xF000U)
#define PWM_FCTRL_FLVL_SHIFT (12U)
/*! FLVL - Fault Level
 *  0b0000..A logic 0 on the fault input indicates a fault condition.
 *  0b0001..A logic 1 on the fault input indicates a fault condition.
 */
#define PWM_FCTRL_FLVL(x) (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL_FLVL_SHIFT)) & PWM_FCTRL_FLVL_MASK)
/*! @} */

/* The count of PWM_FCTRL */
#define PWM_FCTRL_COUNT (2U)

/*! @name FSTS - Fault Status Register */
/*! @{ */

#define PWM_FSTS_FFLAG_MASK  (0xFU)
#define PWM_FSTS_FFLAG_SHIFT (0U)
/*! FFLAG - Fault Flags
 *  0b0000..No fault on the FAULTx pin.
 *  0b0001..Fault on the FAULTx pin.
 */
#define PWM_FSTS_FFLAG(x) (((uint16_t)(((uint16_t)(x)) << PWM_FSTS_FFLAG_SHIFT)) & PWM_FSTS_FFLAG_MASK)

#define PWM_FSTS_FFULL_MASK  (0xF0U)
#define PWM_FSTS_FFULL_SHIFT (4U)
/*! FFULL - Full Cycle
 *  0b0000..PWM outputs are not re-enabled at the start of a full cycle
 *  0b0001..PWM outputs are re-enabled at the start of a full cycle
 */
#define PWM_FSTS_FFULL(x) (((uint16_t)(((uint16_t)(x)) << PWM_FSTS_FFULL_SHIFT)) & PWM_FSTS_FFULL_MASK)

#define PWM_FSTS_FFPIN_MASK  (0xF00U)
#define PWM_FSTS_FFPIN_SHIFT (8U)
/*! FFPIN - Filtered Fault Pins
 */
#define PWM_FSTS_FFPIN(x) (((uint16_t)(((uint16_t)(x)) << PWM_FSTS_FFPIN_SHIFT)) & PWM_FSTS_FFPIN_MASK)

#define PWM_FSTS_FHALF_MASK  (0xF000U)
#define PWM_FSTS_FHALF_SHIFT (12U)
/*! FHALF - Half Cycle Fault Recovery
 *  0b0000..PWM outputs are not re-enabled at the start of a half cycle.
 *  0b0001..PWM outputs are re-enabled at the start of a half cycle (as defined by VAL0).
 */
#define PWM_FSTS_FHALF(x) (((uint16_t)(((uint16_t)(x)) << PWM_FSTS_FHALF_SHIFT)) & PWM_FSTS_FHALF_MASK)
/*! @} */

/* The count of PWM_FSTS */
#define PWM_FSTS_COUNT (2U)

/*! @name FFILT - Fault Filter Register */
/*! @{ */

#define PWM_FFILT_FILT_PER_MASK  (0xFFU)
#define PWM_FFILT_FILT_PER_SHIFT (0U)
/*! FILT_PER - Fault Filter Period
 */
#define PWM_FFILT_FILT_PER(x) (((uint16_t)(((uint16_t)(x)) << PWM_FFILT_FILT_PER_SHIFT)) & PWM_FFILT_FILT_PER_MASK)

#define PWM_FFILT_FILT_CNT_MASK  (0x700U)
#define PWM_FFILT_FILT_CNT_SHIFT (8U)
/*! FILT_CNT - Fault Filter Count
 */
#define PWM_FFILT_FILT_CNT(x) (((uint16_t)(((uint16_t)(x)) << PWM_FFILT_FILT_CNT_SHIFT)) & PWM_FFILT_FILT_CNT_MASK)

#define PWM_FFILT_GSTR_MASK  (0x8000U)
#define PWM_FFILT_GSTR_SHIFT (15U)
/*! GSTR - Fault Glitch Stretch Enable
 *  0b0..Fault input glitch stretching is disabled.
 *  0b1..Input fault signals will be stretched to at least 2 IPBus clock cycles.
 */
#define PWM_FFILT_GSTR(x) (((uint16_t)(((uint16_t)(x)) << PWM_FFILT_GSTR_SHIFT)) & PWM_FFILT_GSTR_MASK)
/*! @} */

/* The count of PWM_FFILT */
#define PWM_FFILT_COUNT (2U)

/*! @name FTST - Fault Test Register */
/*! @{ */

#define PWM_FTST_FTEST_MASK  (0x1U)
#define PWM_FTST_FTEST_SHIFT (0U)
/*! FTEST - Fault Test
 *  0b0..No fault
 *  0b1..Cause a simulated fault
 */
#define PWM_FTST_FTEST(x) (((uint16_t)(((uint16_t)(x)) << PWM_FTST_FTEST_SHIFT)) & PWM_FTST_FTEST_MASK)
/*! @} */

/* The count of PWM_FTST */
#define PWM_FTST_COUNT (2U)

/*! @name FCTRL2 - Fault Control 2 Register */
/*! @{ */

#define PWM_FCTRL2_NOCOMB_MASK  (0xFU)
#define PWM_FCTRL2_NOCOMB_SHIFT (0U)
/*! NOCOMB - No Combinational Path From Fault Input To PWM Output
 *  0b0000..There is a combinational link from the fault inputs to the PWM outputs. The fault inputs are combined
 *          with the filtered and latched fault signals to disable the PWM outputs.
 *  0b0001..The direct combinational path from the fault inputs to the PWM outputs is disabled and the filtered
 *          and latched fault signals are used to disable the PWM outputs.
 */
#define PWM_FCTRL2_NOCOMB(x) (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL2_NOCOMB_SHIFT)) & PWM_FCTRL2_NOCOMB_MASK)
/*! @} */

/* The count of PWM_FCTRL2 */
#define PWM_FCTRL2_COUNT (2U)

/*!
 * @}
 */ /* end of group PWM_Register_Masks */

/* PWM - Peripheral instance base addresses */
/** Peripheral PWMA base address */
#define PWMA_BASE (0xE600u)
/** Peripheral PWMA base pointer */
#define PWMA ((PWM_Type *)PWMA_BASE)
/** Peripheral PWMB base address */
#define PWMB_BASE (0xE700u)
/** Peripheral PWMB base pointer */
#define PWMB ((PWM_Type *)PWMB_BASE)
/** Array initializer of PWM peripheral base addresses */
#define PWM_BASE_ADDRS       \
    {                        \
        PWMA_BASE, PWMB_BASE \
    }
/** Array initializer of PWM peripheral base pointers */
#define PWM_BASE_PTRS \
    {                 \
        PWMA, PWMB    \
    }
/** Interrupt vectors for the PWM peripheral type */
#define PWM_CMP_IRQS                                                                           \
    {                                                                                          \
        {eFlexPWMA_CMP0_IRQn, eFlexPWMA_CMP1_IRQn, eFlexPWMA_CMP2_IRQn, eFlexPWMA_CMP3_IRQn},  \
        {                                                                                      \
            eFlexPWMB_CMP0_IRQn, eFlexPWMB_CMP1_IRQn, eFlexPWMB_CMP2_IRQn, eFlexPWMB_CMP3_IRQn \
        }                                                                                      \
    }
#define PWM_RELOAD_IRQS                                                                                    \
    {                                                                                                      \
        {eFlexPWMA_RELOAD0_IRQn, eFlexPWMA_RELOAD1_IRQn, eFlexPWMA_RELOAD2_IRQn, eFlexPWMA_RELOAD3_IRQn},  \
        {                                                                                                  \
            eFlexPWMB_RELOAD0_IRQn, eFlexPWMB_RELOAD1_IRQn, eFlexPWMB_RELOAD2_IRQn, eFlexPWMB_RELOAD3_IRQn \
        }                                                                                                  \
    }
#define PWM_FAULT_IRQS                             \
    {                                              \
        eFlexPWMA_FAULT_IRQn, eFlexPWMB_FAULT_IRQn \
    }

/*!
 * @}
 */ /* end of group PWM_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- QSCI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSCI_Peripheral_Access_Layer QSCI Peripheral Access Layer
 * @{
 */

/** QSCI - Register Layout Typedef */
typedef struct
{
    __IO uint16_t RATE;  /**< QSCI Baud Rate Register, offset: 0x0 */
    __IO uint16_t CTRL1; /**< QSCI Control Register 1, offset: 0x1 */
    __IO uint16_t CTRL2; /**< QSCI Control Register 2, offset: 0x2 */
    __IO uint16_t STAT;  /**< QSCI Status Register, offset: 0x3 */
    __IO uint16_t DATA;  /**< QSCI Data Register, offset: 0x4 */
    __IO uint16_t CTRL3; /**< QSCI Control Register 3, offset: 0x5 */
} QSCI_Type;

/* ----------------------------------------------------------------------------
   -- QSCI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSCI_Register_Masks QSCI Register Masks
 * @{
 */

/*! @name RATE - QSCI Baud Rate Register */
/*! @{ */

#define QSCI_RATE_FRAC_SBR_MASK  (0x7U)
#define QSCI_RATE_FRAC_SBR_SHIFT (0U)
/*! FRAC_SBR - Fractional SCI Baud Rate divider, a value from 0 to 7 that is divided by 8
 */
#define QSCI_RATE_FRAC_SBR(x) (((uint16_t)(((uint16_t)(x)) << QSCI_RATE_FRAC_SBR_SHIFT)) & QSCI_RATE_FRAC_SBR_MASK)

#define QSCI_RATE_SBRL_MASK  (0xFFF8U)
#define QSCI_RATE_SBRL_SHIFT (3U)
/*! SBRL - Low order bits of SCI Baud Rate divider, which combine with the CTRL3[SBRH] field to form a value from 1 to
 * 65535
 */
#define QSCI_RATE_SBRL(x) (((uint16_t)(((uint16_t)(x)) << QSCI_RATE_SBRL_SHIFT)) & QSCI_RATE_SBRL_MASK)
/*! @} */

/*! @name CTRL1 - QSCI Control Register 1 */
/*! @{ */

#define QSCI_CTRL1_SBK_MASK  (0x1U)
#define QSCI_CTRL1_SBK_SHIFT (0U)
/*! SBK - Send Break
 *  0b0..No break characters
 *  0b1..Transmit break characters
 */
#define QSCI_CTRL1_SBK(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_SBK_SHIFT)) & QSCI_CTRL1_SBK_MASK)

#define QSCI_CTRL1_RWU_MASK  (0x2U)
#define QSCI_CTRL1_RWU_SHIFT (1U)
/*! RWU - Receiver Wake-up
 *  0b0..Normal operation
 *  0b1..Standby state
 */
#define QSCI_CTRL1_RWU(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_RWU_SHIFT)) & QSCI_CTRL1_RWU_MASK)

#define QSCI_CTRL1_RE_MASK  (0x4U)
#define QSCI_CTRL1_RE_SHIFT (2U)
/*! RE - Receiver Enable
 *  0b0..Receiver disabled
 *  0b1..Receiver enabled
 */
#define QSCI_CTRL1_RE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_RE_SHIFT)) & QSCI_CTRL1_RE_MASK)

#define QSCI_CTRL1_TE_MASK  (0x8U)
#define QSCI_CTRL1_TE_SHIFT (3U)
/*! TE - Transmitter Enable
 *  0b0..Transmitter disabled
 *  0b1..Transmitter enabled
 */
#define QSCI_CTRL1_TE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_TE_SHIFT)) & QSCI_CTRL1_TE_MASK)

#define QSCI_CTRL1_REIE_MASK  (0x10U)
#define QSCI_CTRL1_REIE_SHIFT (4U)
/*! REIE - Receive Error Interrupt Enable
 *  0b0..Error interrupt requests disabled
 *  0b1..Error interrupt requests enabled
 */
#define QSCI_CTRL1_REIE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_REIE_SHIFT)) & QSCI_CTRL1_REIE_MASK)

#define QSCI_CTRL1_RFIE_MASK  (0x20U)
#define QSCI_CTRL1_RFIE_SHIFT (5U)
/*! RFIE - Receiver Full Interrupt Enable
 *  0b0..STAT[RDRF] and STAT[OR] interrupt requests disabled
 *  0b1..STAT[RDRF] and STAT[OR] interrupt requests enabled
 */
#define QSCI_CTRL1_RFIE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_RFIE_SHIFT)) & QSCI_CTRL1_RFIE_MASK)

#define QSCI_CTRL1_TIIE_MASK  (0x40U)
#define QSCI_CTRL1_TIIE_SHIFT (6U)
/*! TIIE - Transmitter Idle Interrupt Enable
 *  0b0..STAT[TIDLE] interrupt requests disabled
 *  0b1..STAT[TIDLE] interrupt requests enabled
 */
#define QSCI_CTRL1_TIIE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_TIIE_SHIFT)) & QSCI_CTRL1_TIIE_MASK)

#define QSCI_CTRL1_TEIE_MASK  (0x80U)
#define QSCI_CTRL1_TEIE_SHIFT (7U)
/*! TEIE - Transmitter Empty Interrupt Enable
 *  0b0..STAT[TDRE] interrupt requests disabled
 *  0b1..STAT[TDRE] interrupt requests enabled
 */
#define QSCI_CTRL1_TEIE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_TEIE_SHIFT)) & QSCI_CTRL1_TEIE_MASK)

#define QSCI_CTRL1_PT_MASK  (0x100U)
#define QSCI_CTRL1_PT_SHIFT (8U)
/*! PT - Parity Type
 *  0b0..Even parity
 *  0b1..Odd parity
 */
#define QSCI_CTRL1_PT(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_PT_SHIFT)) & QSCI_CTRL1_PT_MASK)

#define QSCI_CTRL1_PE_MASK  (0x200U)
#define QSCI_CTRL1_PE_SHIFT (9U)
/*! PE - Parity Enable
 *  0b0..Parity function disabled
 *  0b1..Parity function enabled
 */
#define QSCI_CTRL1_PE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_PE_SHIFT)) & QSCI_CTRL1_PE_MASK)

#define QSCI_CTRL1_POL_MASK  (0x400U)
#define QSCI_CTRL1_POL_SHIFT (10U)
/*! POL - Polarity
 *  0b0..Don't invert transmit and receive data bits (normal mode)
 *  0b1..Invert transmit and receive data bits (inverted mode)
 */
#define QSCI_CTRL1_POL(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_POL_SHIFT)) & QSCI_CTRL1_POL_MASK)

#define QSCI_CTRL1_WAKE_MASK  (0x800U)
#define QSCI_CTRL1_WAKE_SHIFT (11U)
/*! WAKE - Wake-up Condition
 *  0b0..Idle line wake-up
 *  0b1..Address mark wake-up
 */
#define QSCI_CTRL1_WAKE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_WAKE_SHIFT)) & QSCI_CTRL1_WAKE_MASK)

#define QSCI_CTRL1_M_MASK  (0x1000U)
#define QSCI_CTRL1_M_SHIFT (12U)
/*! M - Data Format Mode
 *  0b0..One start bit, eight data bits, one stop bit
 *  0b1..One start bit, nine data bits, one stop bit
 */
#define QSCI_CTRL1_M(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_M_SHIFT)) & QSCI_CTRL1_M_MASK)

#define QSCI_CTRL1_RSRC_MASK  (0x2000U)
#define QSCI_CTRL1_RSRC_SHIFT (13U)
/*! RSRC - Receiver Source
 *  0b0..Receiver input internally connected to transmitter output
 *  0b1..Receiver input connected to TXD pin
 */
#define QSCI_CTRL1_RSRC(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_RSRC_SHIFT)) & QSCI_CTRL1_RSRC_MASK)

#define QSCI_CTRL1_SWAI_MASK  (0x4000U)
#define QSCI_CTRL1_SWAI_SHIFT (14U)
/*! SWAI - Stop in Wait Mode
 *  0b0..SCI enabled in wait mode
 *  0b1..SCI disabled in wait mode
 */
#define QSCI_CTRL1_SWAI(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_SWAI_SHIFT)) & QSCI_CTRL1_SWAI_MASK)

#define QSCI_CTRL1_LOOP_MASK  (0x8000U)
#define QSCI_CTRL1_LOOP_SHIFT (15U)
/*! LOOP - Loop Select
 *  0b0..Normal operation, regardless of the value of RSRC
 *  0b1..When RSRC = 0: Loop mode with internal TXD fed back to RXD
 *  0b1..When RSRC = 1: Single-wire mode with TXD output fed back to RXD
 */
#define QSCI_CTRL1_LOOP(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_LOOP_SHIFT)) & QSCI_CTRL1_LOOP_MASK)
/*! @} */

/*! @name CTRL2 - QSCI Control Register 2 */
/*! @{ */

#define QSCI_CTRL2_RDE_MASK  (0x1U)
#define QSCI_CTRL2_RDE_SHIFT (0U)
/*! RDE - Receiver DMA Enable
 *  0b0..Receive DMA disabled
 *  0b1..Receive DMA enabled
 */
#define QSCI_CTRL2_RDE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_RDE_SHIFT)) & QSCI_CTRL2_RDE_MASK)

#define QSCI_CTRL2_TDE_MASK  (0x2U)
#define QSCI_CTRL2_TDE_SHIFT (1U)
/*! TDE - Transmitter DMA Enable
 *  0b0..Transmit DMA disabled
 *  0b1..Transmit DMA enabled
 */
#define QSCI_CTRL2_TDE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_TDE_SHIFT)) & QSCI_CTRL2_TDE_MASK)

#define QSCI_CTRL2_RIIE_MASK  (0x4U)
#define QSCI_CTRL2_RIIE_SHIFT (2U)
/*! RIIE - Receiver Idle Interrupt Enable
 */
#define QSCI_CTRL2_RIIE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_RIIE_SHIFT)) & QSCI_CTRL2_RIIE_MASK)

#define QSCI_CTRL2_LINMODE_MASK  (0x8U)
#define QSCI_CTRL2_LINMODE_SHIFT (3U)
/*! LINMODE - Enable LIN Slave Mode
 *  0b0..The LIN auto baud feature is disabled and the RATE register maintains whatever value the processor writes to
 * it. 0b1..Enable LIN slave functionality. This includes a search for the break character followed by a sync character
 * (0x55) from the master LIN device. When the break is detected (11 consecutive samples of zero), the subsequent sync
 * character is used to measure the baud rate of the transmitting master, and the RATE register is automatically
 * reloaded with the value needed to "match" that baud rate.
 */
#define QSCI_CTRL2_LINMODE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_LINMODE_SHIFT)) & QSCI_CTRL2_LINMODE_MASK)

#define QSCI_CTRL2_RIEIE_MASK  (0x10U)
#define QSCI_CTRL2_RIEIE_SHIFT (4U)
/*! RIEIE - Receiver Input Edge Interrupt Enable
 *  0b0..Receiver input edge interrupt request disabled.
 *  0b1..Receiver input edge interrupt request enabled.
 */
#define QSCI_CTRL2_RIEIE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_RIEIE_SHIFT)) & QSCI_CTRL2_RIEIE_MASK)

#define QSCI_CTRL2_FIFO_EN_MASK  (0x20U)
#define QSCI_CTRL2_FIFO_EN_SHIFT (5U)
/*! FIFO_EN - FIFO Enable
 *  0b0..FIFOs are disabled.
 *  0b1..FIFOs are enabled.
 */
#define QSCI_CTRL2_FIFO_EN(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_FIFO_EN_SHIFT)) & QSCI_CTRL2_FIFO_EN_MASK)

#define QSCI_CTRL2_RFWM_MASK  (0xC0U)
#define QSCI_CTRL2_RFWM_SHIFT (6U)
/*! RFWM - Receive FIFO Full Water Mark
 *  0b00..RDRF is set when at least 1 word is in the FIFO
 *  0b01..RDRF is set when at least 2 words are in the FIFO
 *  0b10..RDRF is set when at least 3 words are in the FIFO
 *  0b11..RDRF is set when at least 4 words are in the FIFO
 */
#define QSCI_CTRL2_RFWM(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_RFWM_SHIFT)) & QSCI_CTRL2_RFWM_MASK)

#define QSCI_CTRL2_RFCNT_MASK  (0x700U)
#define QSCI_CTRL2_RFCNT_SHIFT (8U)
/*! RFCNT - Receive FIFO Count
 *  0b000..0 words in RX FIFO
 *  0b001..1 word in RX FIFO
 *  0b010..2 words in RX FIFO
 *  0b011..3 words in RX FIFO
 *  0b100..4 words in RX FIFO
 *  0b101..Reserved
 *  0b110..Reserved
 *  0b111..Reserved
 */
#define QSCI_CTRL2_RFCNT(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_RFCNT_SHIFT)) & QSCI_CTRL2_RFCNT_MASK)

#define QSCI_CTRL2_TFWM_MASK  (0x1800U)
#define QSCI_CTRL2_TFWM_SHIFT (11U)
/*! TFWM - Transmit FIFO Empty Water Mark
 *  0b00..TDRE is set when 0 words are in the FIFO
 *  0b01..TDRE is set when 1 or fewer words are in the FIFO
 *  0b10..TDRE is set when 2 or fewer words are in the FIFO
 *  0b11..TDRE is set when 3 or fewer words are in the FIFO
 */
#define QSCI_CTRL2_TFWM(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_TFWM_SHIFT)) & QSCI_CTRL2_TFWM_MASK)

#define QSCI_CTRL2_TFCNT_MASK  (0xE000U)
#define QSCI_CTRL2_TFCNT_SHIFT (13U)
/*! TFCNT - Transmit FIFO Count
 *  0b000..0 words in Tx FIFO
 *  0b001..1 word in Tx FIFO
 *  0b010..2 words in Tx FIFO
 *  0b011..3 words in Tx FIFO
 *  0b100..4 words in Tx FIFO
 *  0b101..Reserved
 *  0b110..Reserved
 *  0b111..Reserved
 */
#define QSCI_CTRL2_TFCNT(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_TFCNT_SHIFT)) & QSCI_CTRL2_TFCNT_MASK)
/*! @} */

/*! @name STAT - QSCI Status Register */
/*! @{ */

#define QSCI_STAT_RAF_MASK  (0x1U)
#define QSCI_STAT_RAF_SHIFT (0U)
/*! RAF - Receiver Active Flag
 *  0b0..No reception in progress
 *  0b1..Reception in progress
 */
#define QSCI_STAT_RAF(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_RAF_SHIFT)) & QSCI_STAT_RAF_MASK)

#define QSCI_STAT_RDMA_MASK  (0x2U)
#define QSCI_STAT_RDMA_SHIFT (1U)
/*! RDMA - Receive DMA Request
 *  0b0..Either CTRL2[RDE] is cleared or CTRL2[RDE] is set and CTRL2[RFCNT] is 0.
 *  0b1..CTRL2[RDE] is set and CTRL2[RFCNT] is currently above 0.
 */
#define QSCI_STAT_RDMA(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_RDMA_SHIFT)) & QSCI_STAT_RDMA_MASK)

#define QSCI_STAT_TDMA_MASK  (0x4U)
#define QSCI_STAT_TDMA_SHIFT (2U)
/*! TDMA - Transmit DMA Request
 *  0b0..Either CTRL2[TDE] is cleared or CTRL2[TDE] is set and CTRL2[TFCNT] is at its maximum value.
 *  0b1..CTRL2[TDE] is set and CTRL2[TFCNT] is currently below its maximum value.
 */
#define QSCI_STAT_TDMA(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_TDMA_SHIFT)) & QSCI_STAT_TDMA_MASK)

#define QSCI_STAT_LSE_MASK  (0x8U)
#define QSCI_STAT_LSE_SHIFT (3U)
/*! LSE - LIN Sync Error
 *  0b0..No error occurred since CTRL2[LINMODE] was enabled or the bit was last cleared
 *  0b1..A sync error prevented loading of the RATE register with a revised value after the break was detected.
 */
#define QSCI_STAT_LSE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_LSE_SHIFT)) & QSCI_STAT_LSE_MASK)

#define QSCI_STAT_RIEF_MASK  (0x10U)
#define QSCI_STAT_RIEF_SHIFT (4U)
/*! RIEF - Receiver Input Edge Flag
 *  0b0..No active edge on the receive pin has occured.
 *  0b1..An active edge on the receive pin has occured.
 */
#define QSCI_STAT_RIEF(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_RIEF_SHIFT)) & QSCI_STAT_RIEF_MASK)

#define QSCI_STAT_PF_MASK  (0x100U)
#define QSCI_STAT_PF_SHIFT (8U)
/*! PF - Parity Error Flag
 *  0b0..No parity error
 *  0b1..Parity error
 */
#define QSCI_STAT_PF(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_PF_SHIFT)) & QSCI_STAT_PF_MASK)

#define QSCI_STAT_FE_MASK  (0x200U)
#define QSCI_STAT_FE_SHIFT (9U)
/*! FE - Framing Error Flag
 *  0b0..No framing error
 *  0b1..Framing error
 */
#define QSCI_STAT_FE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_FE_SHIFT)) & QSCI_STAT_FE_MASK)

#define QSCI_STAT_NF_MASK  (0x400U)
#define QSCI_STAT_NF_SHIFT (10U)
/*! NF - Noise Flag
 *  0b0..No noise
 *  0b1..Noise
 */
#define QSCI_STAT_NF(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_NF_SHIFT)) & QSCI_STAT_NF_MASK)

#define QSCI_STAT_OR_MASK  (0x800U)
#define QSCI_STAT_OR_SHIFT (11U)
/*! OR - Overrun Flag
 *  0b0..No overrun
 *  0b1..Overrun
 */
#define QSCI_STAT_OR(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_OR_SHIFT)) & QSCI_STAT_OR_MASK)

#define QSCI_STAT_RIDLE_MASK  (0x1000U)
#define QSCI_STAT_RIDLE_SHIFT (12U)
/*! RIDLE - Receiver Idle Line Flag
 *  0b0..Receiver input is either active now or has never become active since RIDLE was last cleared
 *  0b1..Receiver input has become idle (after receiving a valid frame)
 */
#define QSCI_STAT_RIDLE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_RIDLE_SHIFT)) & QSCI_STAT_RIDLE_MASK)

#define QSCI_STAT_RDRF_MASK  (0x2000U)
#define QSCI_STAT_RDRF_SHIFT (13U)
/*! RDRF - Receive Data Register Full Flag
 *  0b0..RX FIFO word count is at or below watermark
 *  0b1..RX FIFO word count is above watermark
 */
#define QSCI_STAT_RDRF(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_RDRF_SHIFT)) & QSCI_STAT_RDRF_MASK)

#define QSCI_STAT_TIDLE_MASK  (0x4000U)
#define QSCI_STAT_TIDLE_SHIFT (14U)
/*! TIDLE - Transmitter Idle Flag
 *  0b0..Transmission in progress
 *  0b1..No transmission in progress
 */
#define QSCI_STAT_TIDLE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_TIDLE_SHIFT)) & QSCI_STAT_TIDLE_MASK)

#define QSCI_STAT_TDRE_MASK  (0x8000U)
#define QSCI_STAT_TDRE_SHIFT (15U)
/*! TDRE - Transmit Data Register Empty Flag
 *  0b0..TX FIFO word count is above watermark
 *  0b1..TX FIFO word count is at or below watermark
 */
#define QSCI_STAT_TDRE(x) (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_TDRE_SHIFT)) & QSCI_STAT_TDRE_MASK)
/*! @} */

/*! @name DATA - QSCI Data Register */
/*! @{ */

#define QSCI_DATA_RECEIVE_TRANSMIT_DATA_MASK  (0x1FFU)
#define QSCI_DATA_RECEIVE_TRANSMIT_DATA_SHIFT (0U)
#define QSCI_DATA_RECEIVE_TRANSMIT_DATA(x) \
    (((uint16_t)(((uint16_t)(x)) << QSCI_DATA_RECEIVE_TRANSMIT_DATA_SHIFT)) & QSCI_DATA_RECEIVE_TRANSMIT_DATA_MASK)
/*! @} */

/*! @name CTRL3 - QSCI Control Register 3 */
/*! @{ */

#define QSCI_CTRL3_SHEN_MASK  (0x1U)
#define QSCI_CTRL3_SHEN_SHIFT (0U)
/*! SHEN - Stop mode entry hold off
 *  0b0..Stop mode hold off is disabled.
 *  0b1..Stop mode holdoff is enabled.
 */
#define QSCI_CTRL3_SHEN(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL3_SHEN_SHIFT)) & QSCI_CTRL3_SHEN_MASK)

#define QSCI_CTRL3_SBRH_MASK  (0xE000U)
#define QSCI_CTRL3_SBRH_SHIFT (13U)
/*! SBRH - High order bits of SCI Baud Rate divider, which combine with the RATE[SBRL] field to form a value from 1 to
 * 65535
 */
#define QSCI_CTRL3_SBRH(x) (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL3_SBRH_SHIFT)) & QSCI_CTRL3_SBRH_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group QSCI_Register_Masks */

/* QSCI - Peripheral instance base addresses */
/** Peripheral QSCI0 base address */
#define QSCI0_BASE (0xE080u)
/** Peripheral QSCI0 base pointer */
#define QSCI0 ((QSCI_Type *)QSCI0_BASE)
/** Peripheral QSCI1 base address */
#define QSCI1_BASE (0xE090u)
/** Peripheral QSCI1 base pointer */
#define QSCI1 ((QSCI_Type *)QSCI1_BASE)
/** Array initializer of QSCI peripheral base addresses */
#define QSCI_BASE_ADDRS        \
    {                          \
        QSCI0_BASE, QSCI1_BASE \
    }
/** Array initializer of QSCI peripheral base pointers */
#define QSCI_BASE_PTRS \
    {                  \
        QSCI0, QSCI1   \
    }
/** Interrupt vectors for the QSCI peripheral type */
#define QSCI_RX_IRQS                   \
    {                                  \
        QSCI0_RCV_IRQn, QSCI1_RCV_IRQn \
    }
#define QSCI_RX_ERR_IRQS                 \
    {                                    \
        QSCI0_RERR_IRQn, QSCI1_RERR_IRQn \
    }
#define QSCI_TX_IRQS                     \
    {                                    \
        QSCI0_TDRE_IRQn, QSCI1_TDRE_IRQn \
    }
#define QSCI_TRIDLE_IRQS                     \
    {                                        \
        QSCI0_TRIDLE_IRQn, QSCI1_TRIDLE_IRQn \
    }

/*!
 * @}
 */ /* end of group QSCI_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- QSPI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSPI_Peripheral_Access_Layer QSPI Peripheral Access Layer
 * @{
 */

/** QSPI - Register Layout Typedef */
typedef struct
{
    __IO uint16_t SPSCR;  /**< SPI Status and Control Register, offset: 0x0 */
    __IO uint16_t SPDSR;  /**< SPI Data Size and Control Register, offset: 0x1 */
    __I uint16_t SPDRR;   /**< SPI Data Receive Register, offset: 0x2 */
    __O uint16_t SPDTR;   /**< SPI Data Transmit Register, offset: 0x3 */
    __IO uint16_t SPFIFO; /**< SPI FIFO Control Register, offset: 0x4 */
    __IO uint16_t SPWAIT; /**< SPI Word Delay Register, offset: 0x5 */
    __IO uint16_t SPCTL2; /**< SPI Control Register 2, offset: 0x6 */
} QSPI_Type;

/* ----------------------------------------------------------------------------
   -- QSPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSPI_Register_Masks QSPI Register Masks
 * @{
 */

/*! @name SPSCR - SPI Status and Control Register */
/*! @{ */

#define QSPI_SPSCR_SPTE_MASK  (0x1U)
#define QSPI_SPSCR_SPTE_SHIFT (0U)
/*! SPTE - SPI Transmitter Empty
 *  0b0..Transmit data register or FIFO is not empty. (If using the FIFO, read TFCNT to determine how many words can be
 * written safely.) 0b1..Transmit data register or FIFO is empty.
 */
#define QSPI_SPSCR_SPTE(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPTE_SHIFT)) & QSPI_SPSCR_SPTE_MASK)

#define QSPI_SPSCR_MODF_MASK  (0x2U)
#define QSPI_SPSCR_MODF_SHIFT (1U)
/*! MODF - Mode Fault
 *  0b0..SS_B pin at appropriate logic level
 *  0b1..SS_B pin at inappropriate logic level
 */
#define QSPI_SPSCR_MODF(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_MODF_SHIFT)) & QSPI_SPSCR_MODF_MASK)

#define QSPI_SPSCR_OVRF_MASK  (0x4U)
#define QSPI_SPSCR_OVRF_SHIFT (2U)
/*! OVRF - Overflow
 *  0b0..No overflow
 *  0b1..Overflow
 */
#define QSPI_SPSCR_OVRF(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_OVRF_SHIFT)) & QSPI_SPSCR_OVRF_MASK)

#define QSPI_SPSCR_SPRF_MASK  (0x8U)
#define QSPI_SPSCR_SPRF_SHIFT (3U)
/*! SPRF - SPI Receiver Full
 *  0b0..Receive data register or FIFO is not full. (If using the FIFO, read RFCNT to determine the number of valid
 * words available.) 0b1..Receive data register or FIFO is full.
 */
#define QSPI_SPSCR_SPRF(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPRF_SHIFT)) & QSPI_SPSCR_SPRF_MASK)

#define QSPI_SPSCR_SPTIE_MASK  (0x10U)
#define QSPI_SPSCR_SPTIE_SHIFT (4U)
/*! SPTIE - Transmit Interrupt Enable
 *  0b0..SPTE interrupt requests disabled
 *  0b1..SPTE interrupt requests enabled
 */
#define QSPI_SPSCR_SPTIE(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPTIE_SHIFT)) & QSPI_SPSCR_SPTIE_MASK)

#define QSPI_SPSCR_SPE_MASK  (0x20U)
#define QSPI_SPSCR_SPE_SHIFT (5U)
/*! SPE - SPI Enable
 *  0b0..SPI module disabled
 *  0b1..SPI module enabled
 */
#define QSPI_SPSCR_SPE(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPE_SHIFT)) & QSPI_SPSCR_SPE_MASK)

#define QSPI_SPSCR_CPHA_MASK  (0x40U)
#define QSPI_SPSCR_CPHA_SHIFT (6U)
/*! CPHA - Clock Phase
 */
#define QSPI_SPSCR_CPHA(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_CPHA_SHIFT)) & QSPI_SPSCR_CPHA_MASK)

#define QSPI_SPSCR_CPOL_MASK  (0x80U)
#define QSPI_SPSCR_CPOL_SHIFT (7U)
/*! CPOL - Clock Polarity
 *  0b0..Rising edge of SCLK starts transaction
 *  0b1..Falling edge of SCLK starts transaction
 */
#define QSPI_SPSCR_CPOL(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_CPOL_SHIFT)) & QSPI_SPSCR_CPOL_MASK)

#define QSPI_SPSCR_SPMSTR_MASK  (0x100U)
#define QSPI_SPSCR_SPMSTR_SHIFT (8U)
/*! SPMSTR - SPI Master
 *  0b0..Slave mode
 *  0b1..Master mode
 */
#define QSPI_SPSCR_SPMSTR(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPMSTR_SHIFT)) & QSPI_SPSCR_SPMSTR_MASK)

#define QSPI_SPSCR_SPRIE_MASK  (0x200U)
#define QSPI_SPSCR_SPRIE_SHIFT (9U)
/*! SPRIE - SPI Receiver Interrupt Enable
 *  0b0..SPRF interrupt requests disabled
 *  0b1..SPRF interrupt requests enabled
 */
#define QSPI_SPSCR_SPRIE(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPRIE_SHIFT)) & QSPI_SPSCR_SPRIE_MASK)

#define QSPI_SPSCR_MODFEN_MASK  (0x400U)
#define QSPI_SPSCR_MODFEN_SHIFT (10U)
/*! MODFEN - Mode Fault Enable
 */
#define QSPI_SPSCR_MODFEN(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_MODFEN_SHIFT)) & QSPI_SPSCR_MODFEN_MASK)

#define QSPI_SPSCR_ERRIE_MASK  (0x800U)
#define QSPI_SPSCR_ERRIE_SHIFT (11U)
/*! ERRIE - Error Interrupt Enable
 *  0b0..MODF and OVRF cannot generate device interrupt requests
 *  0b1..MODF and OVRF can generate device interrupt requests
 */
#define QSPI_SPSCR_ERRIE(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_ERRIE_SHIFT)) & QSPI_SPSCR_ERRIE_MASK)

#define QSPI_SPSCR_DSO_MASK  (0x1000U)
#define QSPI_SPSCR_DSO_SHIFT (12U)
/*! DSO - Data Shift Order
 *  0b0..MSB transmitted first (MSB -> LSB)
 *  0b1..LSB transmitted first (LSB -> MSB)
 */
#define QSPI_SPSCR_DSO(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_DSO_SHIFT)) & QSPI_SPSCR_DSO_MASK)

#define QSPI_SPSCR_SPR_MASK  (0xE000U)
#define QSPI_SPSCR_SPR_SHIFT (13U)
/*! SPR - SPI Baud Rate Select
 *  0b000..BD = 2 when SPR3 = 0, BD = 512 when SPR3 = 1 (double BD when BD2X = 1)
 *  0b001..BD = 4 when SPR3 = 0, BD = 1024 when SPR3 = 1 (double BD when BD2X = 1)
 *  0b010..BD = 8 when SPR3 = 0, BD = 2048 when SPR3 = 1 (double BD when BD2X = 1)
 *  0b011..BD = 16 when SPR3 = 0, BD = 4096 when SPR3 = 1 (double BD when BD2X = 1)
 *  0b100..BD = 32 when SPR3 = 0, BD = 8192 when SPR3 = 1 (double BD when BD2X = 1)
 *  0b101..BD = 64 when SPR3 = 0 (double BD when BD2X = 1), BD = 16384 when SPR3 = 1 (regardless of BD2X)
 *  0b110..BD = 128 when SPR3 = 0 (double BD when BD2X = 1), BD = 16384 when SPR3 = 1 (regardless of BD2X)
 *  0b111..BD = 256 when SPR3 = 0 (double BD when BD2X = 1), BD = 16384 when SPR3 = 1 (regardless of BD2X)
 */
#define QSPI_SPSCR_SPR(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPR_SHIFT)) & QSPI_SPSCR_SPR_MASK)
/*! @} */

/*! @name SPDSR - SPI Data Size and Control Register */
/*! @{ */

#define QSPI_SPDSR_DS_MASK  (0xFU)
#define QSPI_SPDSR_DS_SHIFT (0U)
/*! DS - Transaction data size
 *  0b0000..Not allowed
 *  0b0001..2 bits transaction data size
 *  0b0010..3 bits transaction data size
 *  0b0011..4 bits transaction data size
 *  0b0100..5 bits transaction data size
 *  0b0101..6 bits transaction data size
 *  0b0110..7 bits transaction data size
 *  0b0111..8 bits transaction data size
 *  0b1000..9 bits transaction data size
 *  0b1001..10 bits transaction data size
 *  0b1010..11 bits transaction data size
 *  0b1011..12 bits transaction data size
 *  0b1100..13 bits transaction data size
 *  0b1101..14 bits transaction data size
 *  0b1110..15 bits transaction data size
 *  0b1111..16 bits transaction data size
 */
#define QSPI_SPDSR_DS(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_DS_SHIFT)) & QSPI_SPDSR_DS_MASK)

#define QSPI_SPDSR_SPR3_MASK  (0x10U)
#define QSPI_SPDSR_SPR3_SHIFT (4U)
/*! SPR3 - SPI Baud Rate Select
 */
#define QSPI_SPDSR_SPR3(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SPR3_SHIFT)) & QSPI_SPDSR_SPR3_MASK)

#define QSPI_SPDSR_SSB_OVER_MASK  (0x20U)
#define QSPI_SPDSR_SSB_OVER_SHIFT (5U)
/*! SSB_OVER - SS_B Override
 *  0b0..SS_B internal module input is selected to be connected to a GPIO pin.
 *  0b1..SS_B internal module input is selected to be equal to SPMSTR.
 */
#define QSPI_SPDSR_SSB_OVER(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_OVER_SHIFT)) & QSPI_SPDSR_SSB_OVER_MASK)

#define QSPI_SPDSR_SSB_STRB_MASK  (0x40U)
#define QSPI_SPDSR_SSB_STRB_SHIFT (6U)
/*! SSB_STRB - SS_B Strobe Mode
 *  0b0..No SS_B pulse between words.
 *  0b1..SS_B output signal is pulsed high between words. This adds 1.5 baud clocks to the total word period. The
 *       idle state of SS_B is low unless SSB_AUTO is high and then the idle state is high. Do not use if MODFEN =
 *       1.
 */
#define QSPI_SPDSR_SSB_STRB(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_STRB_SHIFT)) & QSPI_SPDSR_SSB_STRB_MASK)

#define QSPI_SPDSR_SSB_DDR_MASK  (0x80U)
#define QSPI_SPDSR_SSB_DDR_SHIFT (7U)
/*! SSB_DDR - SS_B Data Direction
 *  0b0..SS_B is configured as an input pin. Use this setting in slave mode or in master mode with MODFEN=1.
 *  0b1..SS_B is configured as an output pin. Use this setting in master mode with MODFEN=0.
 */
#define QSPI_SPDSR_SSB_DDR(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_DDR_SHIFT)) & QSPI_SPDSR_SSB_DDR_MASK)

#define QSPI_SPDSR_SSB_AUTO_MASK  (0x100U)
#define QSPI_SPDSR_SSB_AUTO_SHIFT (8U)
/*! SSB_AUTO - SS_B Automatic Mode
 *  0b0..SS_B output signal is software generated by directly manipulating the various bits in this register or
 *       the GPIO registers (compatible with legacy SPI software).
 *  0b1..SS_B output signal is hardware generated to create the initial falling edge and final rising edge. The
 *       idle state of the SS_B is high. Do not use if MODFEN = 1.
 */
#define QSPI_SPDSR_SSB_AUTO(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_AUTO_SHIFT)) & QSPI_SPDSR_SSB_AUTO_MASK)

#define QSPI_SPDSR_SSB_ODM_MASK  (0x200U)
#define QSPI_SPDSR_SSB_ODM_SHIFT (9U)
/*! SSB_ODM - SS_B Open Drain Mode
 *  0b0..SS_B is configured for high and low drive. This mode is generally used in single master systems.
 *  0b1..SS_B is configured as an open drain pin (only drives low output level). This mode is useful for multiple master
 * systems.
 */
#define QSPI_SPDSR_SSB_ODM(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_ODM_SHIFT)) & QSPI_SPDSR_SSB_ODM_MASK)

#define QSPI_SPDSR_SSB_DATA_MASK  (0x400U)
#define QSPI_SPDSR_SSB_DATA_SHIFT (10U)
/*! SSB_DATA - SS_B Data
 *  0b0..SS_B pin is driven low if SSB_DDR=1
 *  0b1..SS_B pin is driven high if SSB_DDR=1
 */
#define QSPI_SPDSR_SSB_DATA(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_DATA_SHIFT)) & QSPI_SPDSR_SSB_DATA_MASK)

#define QSPI_SPDSR_SSB_IN_MASK  (0x800U)
#define QSPI_SPDSR_SSB_IN_SHIFT (11U)
/*! SSB_IN - SS_B Input
 */
#define QSPI_SPDSR_SSB_IN(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_IN_SHIFT)) & QSPI_SPDSR_SSB_IN_MASK)

#define QSPI_SPDSR_BD2X_MASK  (0x1000U)
#define QSPI_SPDSR_BD2X_SHIFT (12U)
/*! BD2X - Baud Divisor Times
 */
#define QSPI_SPDSR_BD2X(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_BD2X_SHIFT)) & QSPI_SPDSR_BD2X_MASK)

#define QSPI_SPDSR_RDMAEN_MASK  (0x2000U)
#define QSPI_SPDSR_RDMAEN_SHIFT (13U)
/*! RDMAEN - Receive DMA Enable
 */
#define QSPI_SPDSR_RDMAEN(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_RDMAEN_SHIFT)) & QSPI_SPDSR_RDMAEN_MASK)

#define QSPI_SPDSR_TDMAEN_MASK  (0x4000U)
#define QSPI_SPDSR_TDMAEN_SHIFT (14U)
/*! TDMAEN - Transmit DMA Enable
 */
#define QSPI_SPDSR_TDMAEN(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_TDMAEN_SHIFT)) & QSPI_SPDSR_TDMAEN_MASK)

#define QSPI_SPDSR_WOM_MASK  (0x8000U)
#define QSPI_SPDSR_WOM_SHIFT (15U)
/*! WOM - Wired-OR Mode
 *  0b0..The SPI pins are configured as push-pull drivers.
 *  0b1..The SPI pins are configured as open-drain drivers with the pull-ups disabled.
 */
#define QSPI_SPDSR_WOM(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_WOM_SHIFT)) & QSPI_SPDSR_WOM_MASK)
/*! @} */

/*! @name SPDRR - SPI Data Receive Register */
/*! @{ */

#define QSPI_SPDRR_R0_MASK  (0x1U)
#define QSPI_SPDRR_R0_SHIFT (0U)
/*! R0 - Receive Data Bit 0
 */
#define QSPI_SPDRR_R0(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R0_SHIFT)) & QSPI_SPDRR_R0_MASK)

#define QSPI_SPDRR_R1_MASK  (0x2U)
#define QSPI_SPDRR_R1_SHIFT (1U)
/*! R1 - Receive Data Bit 1
 */
#define QSPI_SPDRR_R1(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R1_SHIFT)) & QSPI_SPDRR_R1_MASK)

#define QSPI_SPDRR_R2_MASK  (0x4U)
#define QSPI_SPDRR_R2_SHIFT (2U)
/*! R2 - Receive Data Bit 2
 */
#define QSPI_SPDRR_R2(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R2_SHIFT)) & QSPI_SPDRR_R2_MASK)

#define QSPI_SPDRR_R3_MASK  (0x8U)
#define QSPI_SPDRR_R3_SHIFT (3U)
/*! R3 - Receive Data Bit 3
 */
#define QSPI_SPDRR_R3(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R3_SHIFT)) & QSPI_SPDRR_R3_MASK)

#define QSPI_SPDRR_R4_MASK  (0x10U)
#define QSPI_SPDRR_R4_SHIFT (4U)
/*! R4 - Receive Data Bit 4
 */
#define QSPI_SPDRR_R4(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R4_SHIFT)) & QSPI_SPDRR_R4_MASK)

#define QSPI_SPDRR_R5_MASK  (0x20U)
#define QSPI_SPDRR_R5_SHIFT (5U)
/*! R5 - Receive Data Bit 5
 */
#define QSPI_SPDRR_R5(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R5_SHIFT)) & QSPI_SPDRR_R5_MASK)

#define QSPI_SPDRR_R6_MASK  (0x40U)
#define QSPI_SPDRR_R6_SHIFT (6U)
/*! R6 - Receive Data Bit 6
 */
#define QSPI_SPDRR_R6(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R6_SHIFT)) & QSPI_SPDRR_R6_MASK)

#define QSPI_SPDRR_R7_MASK  (0x80U)
#define QSPI_SPDRR_R7_SHIFT (7U)
/*! R7 - Receive Data Bit 7
 */
#define QSPI_SPDRR_R7(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R7_SHIFT)) & QSPI_SPDRR_R7_MASK)

#define QSPI_SPDRR_R8_MASK  (0x100U)
#define QSPI_SPDRR_R8_SHIFT (8U)
/*! R8 - Receive Data Bit 8
 */
#define QSPI_SPDRR_R8(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R8_SHIFT)) & QSPI_SPDRR_R8_MASK)

#define QSPI_SPDRR_R9_MASK  (0x200U)
#define QSPI_SPDRR_R9_SHIFT (9U)
/*! R9 - Receive Data Bit 9
 */
#define QSPI_SPDRR_R9(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R9_SHIFT)) & QSPI_SPDRR_R9_MASK)

#define QSPI_SPDRR_R10_MASK  (0x400U)
#define QSPI_SPDRR_R10_SHIFT (10U)
/*! R10 - Receive Data Bit 10
 */
#define QSPI_SPDRR_R10(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R10_SHIFT)) & QSPI_SPDRR_R10_MASK)

#define QSPI_SPDRR_R11_MASK  (0x800U)
#define QSPI_SPDRR_R11_SHIFT (11U)
/*! R11 - Receive Data Bit 11
 */
#define QSPI_SPDRR_R11(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R11_SHIFT)) & QSPI_SPDRR_R11_MASK)

#define QSPI_SPDRR_R12_MASK  (0x1000U)
#define QSPI_SPDRR_R12_SHIFT (12U)
/*! R12 - Receive Data Bit 12
 */
#define QSPI_SPDRR_R12(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R12_SHIFT)) & QSPI_SPDRR_R12_MASK)

#define QSPI_SPDRR_R13_MASK  (0x2000U)
#define QSPI_SPDRR_R13_SHIFT (13U)
/*! R13 - Receive Data Bit 13
 */
#define QSPI_SPDRR_R13(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R13_SHIFT)) & QSPI_SPDRR_R13_MASK)

#define QSPI_SPDRR_R14_MASK  (0x4000U)
#define QSPI_SPDRR_R14_SHIFT (14U)
/*! R14 - Receive Data Bit 14
 */
#define QSPI_SPDRR_R14(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R14_SHIFT)) & QSPI_SPDRR_R14_MASK)

#define QSPI_SPDRR_R15_MASK  (0x8000U)
#define QSPI_SPDRR_R15_SHIFT (15U)
/*! R15 - Receive Data Bit 15
 */
#define QSPI_SPDRR_R15(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R15_SHIFT)) & QSPI_SPDRR_R15_MASK)
/*! @} */

/*! @name SPDTR - SPI Data Transmit Register */
/*! @{ */

#define QSPI_SPDTR_T0_MASK  (0x1U)
#define QSPI_SPDTR_T0_SHIFT (0U)
/*! T0 - Transmit Data Bit 0
 */
#define QSPI_SPDTR_T0(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T0_SHIFT)) & QSPI_SPDTR_T0_MASK)

#define QSPI_SPDTR_T1_MASK  (0x2U)
#define QSPI_SPDTR_T1_SHIFT (1U)
/*! T1 - Transmit Data Bit 1
 */
#define QSPI_SPDTR_T1(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T1_SHIFT)) & QSPI_SPDTR_T1_MASK)

#define QSPI_SPDTR_T2_MASK  (0x4U)
#define QSPI_SPDTR_T2_SHIFT (2U)
/*! T2 - Transmit Data Bit 2
 */
#define QSPI_SPDTR_T2(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T2_SHIFT)) & QSPI_SPDTR_T2_MASK)

#define QSPI_SPDTR_T3_MASK  (0x8U)
#define QSPI_SPDTR_T3_SHIFT (3U)
/*! T3 - Transmit Data Bit 3
 */
#define QSPI_SPDTR_T3(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T3_SHIFT)) & QSPI_SPDTR_T3_MASK)

#define QSPI_SPDTR_T4_MASK  (0x10U)
#define QSPI_SPDTR_T4_SHIFT (4U)
/*! T4 - Transmit Data Bit 4
 */
#define QSPI_SPDTR_T4(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T4_SHIFT)) & QSPI_SPDTR_T4_MASK)

#define QSPI_SPDTR_T5_MASK  (0x20U)
#define QSPI_SPDTR_T5_SHIFT (5U)
/*! T5 - Transmit Data Bit 5
 */
#define QSPI_SPDTR_T5(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T5_SHIFT)) & QSPI_SPDTR_T5_MASK)

#define QSPI_SPDTR_T6_MASK  (0x40U)
#define QSPI_SPDTR_T6_SHIFT (6U)
/*! T6 - Transmit Data Bit 6
 */
#define QSPI_SPDTR_T6(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T6_SHIFT)) & QSPI_SPDTR_T6_MASK)

#define QSPI_SPDTR_T7_MASK  (0x80U)
#define QSPI_SPDTR_T7_SHIFT (7U)
/*! T7 - Transmit Data Bit 7
 */
#define QSPI_SPDTR_T7(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T7_SHIFT)) & QSPI_SPDTR_T7_MASK)

#define QSPI_SPDTR_T8_MASK  (0x100U)
#define QSPI_SPDTR_T8_SHIFT (8U)
/*! T8 - Transmit Data Bit 8
 */
#define QSPI_SPDTR_T8(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T8_SHIFT)) & QSPI_SPDTR_T8_MASK)

#define QSPI_SPDTR_T9_MASK  (0x200U)
#define QSPI_SPDTR_T9_SHIFT (9U)
/*! T9 - Transmit Data Bit 9
 */
#define QSPI_SPDTR_T9(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T9_SHIFT)) & QSPI_SPDTR_T9_MASK)

#define QSPI_SPDTR_T10_MASK  (0x400U)
#define QSPI_SPDTR_T10_SHIFT (10U)
/*! T10 - Transmit Data Bit 10
 */
#define QSPI_SPDTR_T10(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T10_SHIFT)) & QSPI_SPDTR_T10_MASK)

#define QSPI_SPDTR_T11_MASK  (0x800U)
#define QSPI_SPDTR_T11_SHIFT (11U)
/*! T11 - Transmit Data Bit 11
 */
#define QSPI_SPDTR_T11(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T11_SHIFT)) & QSPI_SPDTR_T11_MASK)

#define QSPI_SPDTR_T12_MASK  (0x1000U)
#define QSPI_SPDTR_T12_SHIFT (12U)
/*! T12 - Transmit Data Bit 12
 */
#define QSPI_SPDTR_T12(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T12_SHIFT)) & QSPI_SPDTR_T12_MASK)

#define QSPI_SPDTR_T13_MASK  (0x2000U)
#define QSPI_SPDTR_T13_SHIFT (13U)
/*! T13 - Transmit Data Bit 13
 */
#define QSPI_SPDTR_T13(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T13_SHIFT)) & QSPI_SPDTR_T13_MASK)

#define QSPI_SPDTR_T14_MASK  (0x4000U)
#define QSPI_SPDTR_T14_SHIFT (14U)
/*! T14 - Transmit Data Bit 14
 */
#define QSPI_SPDTR_T14(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T14_SHIFT)) & QSPI_SPDTR_T14_MASK)

#define QSPI_SPDTR_T15_MASK  (0x8000U)
#define QSPI_SPDTR_T15_SHIFT (15U)
/*! T15 - Transmit Data Bit 15
 */
#define QSPI_SPDTR_T15(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T15_SHIFT)) & QSPI_SPDTR_T15_MASK)
/*! @} */

/*! @name SPFIFO - SPI FIFO Control Register */
/*! @{ */

#define QSPI_SPFIFO_FIFO_ENA_MASK  (0x1U)
#define QSPI_SPFIFO_FIFO_ENA_SHIFT (0U)
/*! FIFO_ENA - FIFO Enable
 *  0b0..FIFOs are disabled and reset.
 *  0b1..FIFOs are enabled. FIFOs retain their status even if SPE is set to 0.
 */
#define QSPI_SPFIFO_FIFO_ENA(x) \
    (((uint16_t)(((uint16_t)(x)) << QSPI_SPFIFO_FIFO_ENA_SHIFT)) & QSPI_SPFIFO_FIFO_ENA_MASK)

#define QSPI_SPFIFO_RFWM_MASK  (0xCU)
#define QSPI_SPFIFO_RFWM_SHIFT (2U)
/*! RFWM - Rx FIFO Watermark
 *  0b00..Receive interrupt active when Rx FIFO has at least one word used
 *  0b01..Receive interrupt active when Rx FIFO has at least two words used
 *  0b10..Receive interrupt active when Rx FIFO has at least three words used
 *  0b11..Receive interrupt active when Rx FIFO is full
 */
#define QSPI_SPFIFO_RFWM(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPFIFO_RFWM_SHIFT)) & QSPI_SPFIFO_RFWM_MASK)

#define QSPI_SPFIFO_TFWM_MASK  (0x60U)
#define QSPI_SPFIFO_TFWM_SHIFT (5U)
/*! TFWM - Tx FIFO Watermark
 *  0b00..Transmit interrupt active when Tx FIFO is empty
 *  0b01..Transmit interrupt active when Tx FIFO has one or fewer words available
 *  0b10..Transmit interrupt active when Tx FIFO has two or fewer words available
 *  0b11..Transmit interrupt active when Tx FIFO has three or fewer words available
 */
#define QSPI_SPFIFO_TFWM(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPFIFO_TFWM_SHIFT)) & QSPI_SPFIFO_TFWM_MASK)

#define QSPI_SPFIFO_RFCNT_MASK  (0x700U)
#define QSPI_SPFIFO_RFCNT_SHIFT (8U)
/*! RFCNT - RX FIFO Level
 *  0b000..Rx FIFO empty
 *  0b001..One word used in Rx FIFO
 *  0b010..Two words used in Rx FIFO
 *  0b011..Three words used in Rx FIFO
 *  0b100..Rx FIFO full (if enabled Receiver Full Interrupt asserted)
 */
#define QSPI_SPFIFO_RFCNT(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPFIFO_RFCNT_SHIFT)) & QSPI_SPFIFO_RFCNT_MASK)

#define QSPI_SPFIFO_TFCNT_MASK  (0x7000U)
#define QSPI_SPFIFO_TFCNT_SHIFT (12U)
/*! TFCNT - TX FIFO Level
 *  0b000..Tx FIFO empty (if enabled Transmit Empty Interrupt asserted)
 *  0b001..One word used in Tx FIFO
 *  0b010..Two words used in Tx FIFO
 *  0b011..Three words used in Tx FIFO
 *  0b100..Tx FIFO full
 */
#define QSPI_SPFIFO_TFCNT(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPFIFO_TFCNT_SHIFT)) & QSPI_SPFIFO_TFCNT_MASK)
/*! @} */

/*! @name SPWAIT - SPI Word Delay Register */
/*! @{ */

#define QSPI_SPWAIT_WAIT_MASK  (0x1FFFU)
#define QSPI_SPWAIT_WAIT_SHIFT (0U)
/*! WAIT - Wait Delay
 */
#define QSPI_SPWAIT_WAIT(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPWAIT_WAIT_SHIFT)) & QSPI_SPWAIT_WAIT_MASK)
/*! @} */

/*! @name SPCTL2 - SPI Control Register 2 */
/*! @{ */

#define QSPI_SPCTL2_SHEN_MASK  (0x1U)
#define QSPI_SPCTL2_SHEN_SHIFT (0U)
/*! SHEN - Stop Mode Holdoff Enable
 *  0b0..Disable stop mode holdoff .
 *  0b1..Enable stop mode holdoff while the SPI is transmitting/receiving.
 */
#define QSPI_SPCTL2_SHEN(x) (((uint16_t)(((uint16_t)(x)) << QSPI_SPCTL2_SHEN_SHIFT)) & QSPI_SPCTL2_SHEN_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group QSPI_Register_Masks */

/* QSPI - Peripheral instance base addresses */
/** Peripheral QSPI0 base address */
#define QSPI0_BASE (0xE0B0u)
/** Peripheral QSPI0 base pointer */
#define QSPI0 ((QSPI_Type *)QSPI0_BASE)
/** Array initializer of QSPI peripheral base addresses */
#define QSPI_BASE_ADDRS \
    {                   \
        QSPI0_BASE      \
    }
/** Array initializer of QSPI peripheral base pointers */
#define QSPI_BASE_PTRS \
    {                  \
        QSPI0          \
    }
/** Interrupt vectors for the QSPI peripheral type */
#define QSPI_RX_IRQS   \
    {                  \
        QSPI0_RCV_IRQn \
    }
#define QSPI_TX_IRQS    \
    {                   \
        QSPI0_XMIT_IRQn \
    }

/*!
 * @}
 */ /* end of group QSPI_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- SIM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Peripheral_Access_Layer SIM Peripheral Access Layer
 * @{
 */

/** SIM - Register Layout Typedef */
typedef struct
{
    __IO uint16_t CTRL; /**< Control Register, offset: 0x0 */
    __I uint16_t RSTAT; /**< Reset Status Register, offset: 0x1 */
    __IO uint16_t SCR0; /**< Software Control Register, offset: 0x2 */
    __IO uint16_t SCR1; /**< Software Control Register, offset: 0x3 */
    __IO uint16_t SCR2; /**< Software Control Register, offset: 0x4 */
    __IO uint16_t SCR3; /**< Software Control Register, offset: 0x5 */
    uint16_t MSHID;     /**< Most Significant Half of JTAG ID, offset: 0x6 */
    uint16_t LSHID;     /**< Least Significant Half of JTAG ID, offset: 0x7 */
    __IO uint16_t PWR;  /**< Power Control Register, offset: 0x8 */
    uint16_t RESERVED_0[1];
    __IO uint16_t CLKOUT; /**< Clock Output Select Register, offset: 0xA */
    __IO uint16_t PCR;    /**< Peripheral Clock Rate Register, offset: 0xB */
    __IO uint16_t PCE0;   /**< Peripheral Clock Enable Register 0, offset: 0xC */
    __IO uint16_t PCE1;   /**< Peripheral Clock Enable Register 1, offset: 0xD */
    __IO uint16_t PCE2;   /**< Peripheral Clock Enable Register 2, offset: 0xE */
    __IO uint16_t PCE3;   /**< Peripheral Clock Enable Register 3, offset: 0xF */
    __IO uint16_t SD0;    /**< Peripheral Clock STOP Disable Register 0, offset: 0x10 */
    __IO uint16_t SD1;    /**< Peripheral Clock STOP Disable Register 1, offset: 0x11 */
    __IO uint16_t SD2;    /**< Peripheral Clock STOP Disable Register 2, offset: 0x12 */
    __IO uint16_t SD3;    /**< Peripheral Clock STOP Disable Register 3, offset: 0x13 */
    __IO uint16_t IOSAHI; /**< I/O Short Address Location Register, offset: 0x14 */
    __IO uint16_t IOSALO; /**< I/O Short Address Location Register, offset: 0x15 */
    __IO uint16_t PROT;   /**< Protection Register, offset: 0x16 */
    __IO uint16_t GPSAL;  /**< GPIOA LSBs Peripheral Select Register, offset: 0x17 */
    __IO uint16_t GPSAH;  /**< GPIOA MSBs Peripheral Select Register, offset: 0x18 */
    __IO uint16_t GPSBL;  /**< GPIOB LSBs Peripheral Select Register, offset: 0x19 */
    __IO uint16_t GPSBH;  /**< GPIOB MSBs Peripheral Select Register, offset: 0x1A */
    __IO uint16_t GPSCL;  /**< GPIOC LSBs Peripheral Select Register, offset: 0x1B */
    __IO uint16_t GPSCH;  /**< GPIOC MSBs Peripheral Select Register, offset: 0x1C */
    __IO uint16_t GPSDL;  /**< GPIOD LSBs Peripheral Select Register, offset: 0x1D */
    uint16_t RESERVED_1[1];
    __IO uint16_t GPSEL; /**< GPIOE LSBs Peripheral Select Register, offset: 0x1F */
    __IO uint16_t GPSEH; /**< GPIOE MSBs Peripheral Select Register, offset: 0x20 */
    __IO uint16_t GPSFL; /**< GPIOF LSBs Peripheral Select Register, offset: 0x21 */
    __IO uint16_t GPSFH; /**< GPIOF MSBs Peripheral Select Register, offset: 0x22 */
    __IO uint16_t GPSGL; /**< GPIOG LSBs Peripheral Select Register, offset: 0x23 */
    __IO uint16_t GPSGH; /**< GPIOG MSBs Peripheral Select Register, offset: 0x24 */
    uint16_t RESERVED_2[4];
    __IO uint16_t IPS0;    /**< Internal Peripheral Select Register 0, offset: 0x29 */
    __IO uint16_t MISC0;   /**< Miscellaneous Register 0, offset: 0x2A */
    __IO uint16_t PSWR0;   /**< Peripheral Software Reset Register 0, offset: 0x2B */
    __IO uint16_t PSWR1;   /**< Peripheral Software Reset Register 1, offset: 0x2C */
    __IO uint16_t PSWR2;   /**< Peripheral Software Reset Register 2, offset: 0x2D */
    __IO uint16_t PSWR3;   /**< Peripheral Software Reset Register 3, offset: 0x2E */
    __IO uint16_t PWRMODE; /**< Power Mode Register, offset: 0x2F */
    uint16_t RESERVED_3[12];
    __I uint16_t NVMOPT6_LOW; /**< Non-Volatile Memory Option Register 6 (Low), offset: 0x3C */
    uint16_t RESERVED_4[15];
    __IO uint16_t PWM_SEL;            /**< PWM Select Register, offset: 0x4C */
    __IO uint16_t ADC_TMR_SEL;        /**< ADC and TMR Select Register, offset: 0x4D */
    __IO uint16_t BOOT_MODE_OVERRIDE; /**< Boot Mode Override Register, offset: 0x4E */
} SIM_Type;

/* ----------------------------------------------------------------------------
   -- SIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Masks SIM Register Masks
 * @{
 */

/*! @name CTRL - Control Register */
/*! @{ */

#define SIM_CTRL_WAIT_DISABLE_MASK  (0x3U)
#define SIM_CTRL_WAIT_DISABLE_SHIFT (0U)
/*! WAIT_DISABLE - WAIT Disable
 *  0b00..Wait mode is entered when the DSC core executes a WAIT instruction.
 *  0b01..The DSC core WAIT instruction does not cause entry into wait mode.
 *  0b10..Wait mode is entered when the DSC core executes a WAIT instruction, and the WAIT_disable field is write
 * protected until the next reset. 0b11..The DSC core WAIT instruction does not cause entry into wait mode, and the
 * WAIT_disable field is write protected until the next reset.
 */
#define SIM_CTRL_WAIT_DISABLE(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_CTRL_WAIT_DISABLE_SHIFT)) & SIM_CTRL_WAIT_DISABLE_MASK)

#define SIM_CTRL_STOP_DISABLE_MASK  (0xCU)
#define SIM_CTRL_STOP_DISABLE_SHIFT (2U)
/*! STOP_DISABLE - STOP Disable
 *  0b00..Stop mode is entered when the DSC core executes a STOP instruction.
 *  0b01..The DSC core STOP instruction does not cause entry into stop mode.
 *  0b10..Stop mode is entered when the DSC core executes a STOP instruction, and the STOP_disable field is write
 * protected until the next reset. 0b11..The DSC core STOP instruction does not cause entry into stop mode, and the
 * STOP_disable field is write protected until the next reset.
 */
#define SIM_CTRL_STOP_DISABLE(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_CTRL_STOP_DISABLE_SHIFT)) & SIM_CTRL_STOP_DISABLE_MASK)

#define SIM_CTRL_SWRST_MASK  (0x10U)
#define SIM_CTRL_SWRST_SHIFT (4U)
/*! SWRST - SOFTWARE RESET
 */
#define SIM_CTRL_SWRST(x) (((uint16_t)(((uint16_t)(x)) << SIM_CTRL_SWRST_SHIFT)) & SIM_CTRL_SWRST_MASK)

#define SIM_CTRL_ONCEEBL_MASK  (0x20U)
#define SIM_CTRL_ONCEEBL_SHIFT (5U)
/*! ONCEEBL - OnCE Enable
 *  0b0..The OnCE clock to the DSC core is enabled when the core TAP is enabled.
 *  0b1..The OnCE clock to the DSC core is always enabled.
 */
#define SIM_CTRL_ONCEEBL(x) (((uint16_t)(((uint16_t)(x)) << SIM_CTRL_ONCEEBL_SHIFT)) & SIM_CTRL_ONCEEBL_MASK)

#define SIM_CTRL_DMAEBL_MASK  (0x1C0U)
#define SIM_CTRL_DMAEBL_SHIFT (6U)
/*! DMAEBL - DMA Enable
 *  0b000..DMA module is disabled.
 *  0b001..DMA module is enabled in run mode only.
 *  0b010..DMA module is enabled in run and wait modes only.
 *  0b011..DMA module is enabled in all power modes.In this option, the MCU cannot switch to WAIT or STOP mode.
 *  0b100..DMA module is disabled and the DMAEbl field is write protected until the next reset.
 *  0b101..DMA module is enabled in run mode only and the DMAEbl field is write protected until the next reset.
 *  0b110..DMA module is enabled in run and wait modes only and the DMAEbl field is write protected until the next
 * reset. 0b111..DMA module is enabled in all low power modes and the DMAEbl field is write protected until the next
 *         reset.In this option, the MCU cannot switch to WAIT or STOP mode.
 */
#define SIM_CTRL_DMAEBL(x) (((uint16_t)(((uint16_t)(x)) << SIM_CTRL_DMAEBL_SHIFT)) & SIM_CTRL_DMAEBL_MASK)

#define SIM_CTRL_RST_FILT_MASK  (0x400U)
#define SIM_CTRL_RST_FILT_SHIFT (10U)
/*! RST_FILT - External Reset Padcell Input Filter Enable
 *  0b0..Input filter on external reset disabled
 *  0b1..Input filter on external reset enabled
 */
#define SIM_CTRL_RST_FILT(x) (((uint16_t)(((uint16_t)(x)) << SIM_CTRL_RST_FILT_SHIFT)) & SIM_CTRL_RST_FILT_MASK)
/*! @} */

/*! @name RSTAT - Reset Status Register */
/*! @{ */

#define SIM_RSTAT_POR_MASK  (0x4U)
#define SIM_RSTAT_POR_SHIFT (2U)
/*! POR - Power-on Reset
 */
#define SIM_RSTAT_POR(x) (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_POR_SHIFT)) & SIM_RSTAT_POR_MASK)

#define SIM_RSTAT_EXTR_MASK  (0x8U)
#define SIM_RSTAT_EXTR_SHIFT (3U)
/*! EXTR - External Reset
 */
#define SIM_RSTAT_EXTR(x) (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_EXTR_SHIFT)) & SIM_RSTAT_EXTR_MASK)

#define SIM_RSTAT_COP_LOR_MASK  (0x10U)
#define SIM_RSTAT_COP_LOR_SHIFT (4U)
/*! COP_LOR - COP Loss of Reference Reset
 */
#define SIM_RSTAT_COP_LOR(x) (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_COP_LOR_SHIFT)) & SIM_RSTAT_COP_LOR_MASK)

#define SIM_RSTAT_COP_CPU_MASK  (0x20U)
#define SIM_RSTAT_COP_CPU_SHIFT (5U)
/*! COP_CPU - COP CPU Time-out Reset
 */
#define SIM_RSTAT_COP_CPU(x) (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_COP_CPU_SHIFT)) & SIM_RSTAT_COP_CPU_MASK)

#define SIM_RSTAT_SWR_MASK  (0x40U)
#define SIM_RSTAT_SWR_SHIFT (6U)
/*! SWR - Software Reset
 */
#define SIM_RSTAT_SWR(x) (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_SWR_SHIFT)) & SIM_RSTAT_SWR_MASK)

#define SIM_RSTAT_COP_WIN_MASK  (0x100U)
#define SIM_RSTAT_COP_WIN_SHIFT (8U)
/*! COP_WIN - COP Window Time-out Reset
 */
#define SIM_RSTAT_COP_WIN(x) (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_COP_WIN_SHIFT)) & SIM_RSTAT_COP_WIN_MASK)

#define SIM_RSTAT_BOOT_MODE_STATUS_MASK  (0x600U)
#define SIM_RSTAT_BOOT_MODE_STATUS_SHIFT (9U)
/*! BOOT_MODE_STATUS - Boot mode.
 */
#define SIM_RSTAT_BOOT_MODE_STATUS(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_BOOT_MODE_STATUS_SHIFT)) & SIM_RSTAT_BOOT_MODE_STATUS_MASK)
/*! @} */

/*! @name SCR0 - Software Control Register */
/*! @{ */

#define SIM_SCR0_SCR0_MASK  (0xFFFFU)
#define SIM_SCR0_SCR0_SHIFT (0U)
/*! SCR0 - Software Control Data
 */
#define SIM_SCR0_SCR0(x) (((uint16_t)(((uint16_t)(x)) << SIM_SCR0_SCR0_SHIFT)) & SIM_SCR0_SCR0_MASK)
/*! @} */

/*! @name SCR1 - Software Control Register */
/*! @{ */

#define SIM_SCR1_SCR1_MASK  (0xFFFFU)
#define SIM_SCR1_SCR1_SHIFT (0U)
/*! SCR1 - Software Control Data
 */
#define SIM_SCR1_SCR1(x) (((uint16_t)(((uint16_t)(x)) << SIM_SCR1_SCR1_SHIFT)) & SIM_SCR1_SCR1_MASK)
/*! @} */

/*! @name SCR2 - Software Control Register */
/*! @{ */

#define SIM_SCR2_SCR2_MASK  (0xFFFFU)
#define SIM_SCR2_SCR2_SHIFT (0U)
/*! SCR2 - Software Control Data
 */
#define SIM_SCR2_SCR2(x) (((uint16_t)(((uint16_t)(x)) << SIM_SCR2_SCR2_SHIFT)) & SIM_SCR2_SCR2_MASK)
/*! @} */

/*! @name SCR3 - Software Control Register */
/*! @{ */

#define SIM_SCR3_SCR3_MASK  (0xFFFFU)
#define SIM_SCR3_SCR3_SHIFT (0U)
/*! SCR3 - Software Control Data
 */
#define SIM_SCR3_SCR3(x) (((uint16_t)(((uint16_t)(x)) << SIM_SCR3_SCR3_SHIFT)) & SIM_SCR3_SCR3_MASK)
/*! @} */

/*! @name PWR - Power Control Register */
/*! @{ */

#define SIM_PWR_LRSTDBY_MASK  (0x3U)
#define SIM_PWR_LRSTDBY_SHIFT (0U)
/*! LRSTDBY - Large Regulator Standby Control
 *  0b00..Large regulator placed in normal mode (default).
 *  0b01..Large regulator placed in standby mode.
 *  0b10..Large regulator placed in normal mode and LRSTDBY is write protected until device reset.
 *  0b11..Large regulator placed in standby mode and LRSTDBY is write protected until device reset.
 */
#define SIM_PWR_LRSTDBY(x) (((uint16_t)(((uint16_t)(x)) << SIM_PWR_LRSTDBY_SHIFT)) & SIM_PWR_LRSTDBY_MASK)

#define SIM_PWR_SR27STDBY_MASK  (0xCU)
#define SIM_PWR_SR27STDBY_SHIFT (2U)
/*! SR27STDBY - Small Regulator 2.7 V Supply Standby Control
 *  0b00..Small regulator 2.7 V supply placed in normal mode (default).
 *  0b01..Small regulator 2.7 V supply placed in standby mode.
 *  0b10..Small regulator 2.7 V supply placed in normal mode and SR27STDBY is write protected until chip reset.
 *  0b11..Small regulator 2.7 V supply placed in standby mode and SR27STDBY is write protected until chip reset.
 */
#define SIM_PWR_SR27STDBY(x) (((uint16_t)(((uint16_t)(x)) << SIM_PWR_SR27STDBY_SHIFT)) & SIM_PWR_SR27STDBY_MASK)

#define SIM_PWR_SR27PDN_MASK  (0x30U)
#define SIM_PWR_SR27PDN_SHIFT (4U)
/*! SR27PDN - Small Regulator 2.7 V Supply Powerdown Control
 *  0b00..Small regulator placed in normal mode (default).
 *  0b01..Small regulator placed in powerdown mode.
 *  0b10..Small regulator placed in normal mode and SR27PDN is write protected until chip reset.
 *  0b11..Small regulator placed in powerdown mode and SR27PDN is write protected until chip reset.
 */
#define SIM_PWR_SR27PDN(x) (((uint16_t)(((uint16_t)(x)) << SIM_PWR_SR27PDN_SHIFT)) & SIM_PWR_SR27PDN_MASK)

#define SIM_PWR_SR12STDBY_MASK  (0xC0U)
#define SIM_PWR_SR12STDBY_SHIFT (6U)
/*! SR12STDBY - Small Regulator 1.2 V Supply Standby Control
 *  0b00..Small regulator 1.2 V supply placed in normal mode (default).
 *  0b01..Small regulator 1.2 V supply placed in standby mode.
 *  0b10..Small regulator 1.2 V supply placed in normal mode and SR12STDBY is write protected until chip reset.
 *  0b11..Small regulator 1.2 V supply placed in standby mode and SR12STDBY is write protected until chip reset.
 */
#define SIM_PWR_SR12STDBY(x) (((uint16_t)(((uint16_t)(x)) << SIM_PWR_SR12STDBY_SHIFT)) & SIM_PWR_SR12STDBY_MASK)
/*! @} */

/*! @name CLKOUT - Clock Output Select Register */
/*! @{ */

#define SIM_CLKOUT_CLKOSEL0_MASK  (0x7U)
#define SIM_CLKOUT_CLKOSEL0_SHIFT (0U)
/*! CLKOSEL0 - CLKOUT0 Select
 *  0b000..Function = BUS_CLK continuous after reset
 *  0b001..Function = 2X_BUS_CLK continuous after reset
 *  0b010..Function = DIV4_BUS_CLK continuous after reset
 *  0b011..Function = MSTR_OSC (master clock) continuous after reset
 *  0b100..output of the MUX choosing between 8M RC and 48M RC div 6
 *  0b101..ROSC_200K (200 kHz relaxation oscillator clock)
 *  0b110..Reserved. For normal operation, do not write 11x.
 *  0b111..Reserved. For normal operation, do not write 11x.
 */
#define SIM_CLKOUT_CLKOSEL0(x) (((uint16_t)(((uint16_t)(x)) << SIM_CLKOUT_CLKOSEL0_SHIFT)) & SIM_CLKOUT_CLKOSEL0_MASK)

#define SIM_CLKOUT_CLKDIS0_MASK  (0x20U)
#define SIM_CLKOUT_CLKDIS0_SHIFT (5U)
/*! CLKDIS0 - Disable for CLKOUT0
 *  0b0..CLKOUT0 output is enabled and outputs the signal indicated by CLKOSEL0
 *  0b1..CLKOUT0 is disabled
 */
#define SIM_CLKOUT_CLKDIS0(x) (((uint16_t)(((uint16_t)(x)) << SIM_CLKOUT_CLKDIS0_SHIFT)) & SIM_CLKOUT_CLKDIS0_MASK)

#define SIM_CLKOUT_CLKOSEL1_MASK  (0x380U)
#define SIM_CLKOUT_CLKOSEL1_SHIFT (7U)
/*! CLKOSEL1 - CLKOUT1 Select
 *  0b000..Function = BUS_CLK continuous after reset
 *  0b001..Function = 2X_BUS_CLK continuous after reset
 *  0b010..Function = DIV4_BUS_CLK continuous after reset
 *  0b011..Function = MSTR_OSC (master clock) continuous after reset
 *  0b100..output of the MUX choosing between 8M RC and 48M RC div 6
 *  0b101..ROSC_200K (200 kHz relaxation oscillator clock)
 *  0b110..Reserved. For normal operation, do not write 11x.
 *  0b111..Reserved. For normal operation, do not write 11x.
 */
#define SIM_CLKOUT_CLKOSEL1(x) (((uint16_t)(((uint16_t)(x)) << SIM_CLKOUT_CLKOSEL1_SHIFT)) & SIM_CLKOUT_CLKOSEL1_MASK)

#define SIM_CLKOUT_CLKDIS1_MASK  (0x1000U)
#define SIM_CLKOUT_CLKDIS1_SHIFT (12U)
/*! CLKDIS1 - Disable for CLKOUT1
 *  0b0..CLKOUT1 output is enabled and outputs the signal indicated by CLKOSEL1
 *  0b1..CLKOUT1 is disabled
 */
#define SIM_CLKOUT_CLKDIS1(x) (((uint16_t)(((uint16_t)(x)) << SIM_CLKOUT_CLKDIS1_SHIFT)) & SIM_CLKOUT_CLKDIS1_MASK)

#define SIM_CLKOUT_CLKODIV_MASK  (0xE000U)
#define SIM_CLKOUT_CLKODIV_SHIFT (13U)
/*! CLKODIV - CLKOUT divide factor
 *  0b000..Divide by 1
 *  0b001..Divide by 2
 *  0b010..Divide by 4
 *  0b011..Divide by 8
 *  0b100..Divide by 16
 *  0b101..Divide by 32
 *  0b110..Divide by 64
 *  0b111..Divide by 128
 */
#define SIM_CLKOUT_CLKODIV(x) (((uint16_t)(((uint16_t)(x)) << SIM_CLKOUT_CLKODIV_SHIFT)) & SIM_CLKOUT_CLKODIV_MASK)
/*! @} */

/*! @name PCR - Peripheral Clock Rate Register */
/*! @{ */

#define SIM_PCR_IIC1_CR_MASK  (0x200U)
#define SIM_PCR_IIC1_CR_SHIFT (9U)
/*! IIC1_CR - IIC1 filter Clock Rate
 *  0b0..IIC1 filter clock rate equals core clock rate (default)
 *  0b1..IIC1 filter clock rate equals 2X core clock rate
 */
#define SIM_PCR_IIC1_CR(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCR_IIC1_CR_SHIFT)) & SIM_PCR_IIC1_CR_MASK)

#define SIM_PCR_IIC0_CR_MASK  (0x400U)
#define SIM_PCR_IIC0_CR_SHIFT (10U)
/*! IIC0_CR - IIC0 filter Clock Rate
 *  0b0..IIC0 filter clock rate equals core clock rate (default)
 *  0b1..IIC0 filter clock rate equals 2X core clock rate
 */
#define SIM_PCR_IIC0_CR(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCR_IIC0_CR_SHIFT)) & SIM_PCR_IIC0_CR_MASK)

#define SIM_PCR_SCI2_CR_MASK  (0x800U)
#define SIM_PCR_SCI2_CR_SHIFT (11U)
/*! SCI2_CR - SCI2 Clock Rate
 *  0b0..SCI2 clock rate equals core clock rate (default)
 *  0b1..SCI2 clock rate equals 2X core clock rate
 */
#define SIM_PCR_SCI2_CR(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCR_SCI2_CR_SHIFT)) & SIM_PCR_SCI2_CR_MASK)

#define SIM_PCR_SCI1_CR_MASK  (0x1000U)
#define SIM_PCR_SCI1_CR_SHIFT (12U)
/*! SCI1_CR - SCI1 Clock Rate
 *  0b0..SCI1 clock rate equals core clock rate (default)
 *  0b1..SCI1 clock rate equals 2X core clock rate
 */
#define SIM_PCR_SCI1_CR(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCR_SCI1_CR_SHIFT)) & SIM_PCR_SCI1_CR_MASK)

#define SIM_PCR_SCI0_CR_MASK  (0x2000U)
#define SIM_PCR_SCI0_CR_SHIFT (13U)
/*! SCI0_CR - SCI0 Clock Rate
 *  0b0..SCI0 clock rate equals core clock rate (default)
 *  0b1..SCI0 clock rate equals 2X core clock rate
 */
#define SIM_PCR_SCI0_CR(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCR_SCI0_CR_SHIFT)) & SIM_PCR_SCI0_CR_MASK)
/*! @} */

/*! @name PCE0 - Peripheral Clock Enable Register 0 */
/*! @{ */

#define SIM_PCE0_GPIOG_MASK  (0x1U)
#define SIM_PCE0_GPIOG_SHIFT (0U)
/*! GPIOG - GPIOG IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOG(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOG_SHIFT)) & SIM_PCE0_GPIOG_MASK)

#define SIM_PCE0_GPIOF_MASK  (0x2U)
#define SIM_PCE0_GPIOF_SHIFT (1U)
/*! GPIOF - GPIOF IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOF(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOF_SHIFT)) & SIM_PCE0_GPIOF_MASK)

#define SIM_PCE0_GPIOE_MASK  (0x4U)
#define SIM_PCE0_GPIOE_SHIFT (2U)
/*! GPIOE - GPIOE IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOE(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOE_SHIFT)) & SIM_PCE0_GPIOE_MASK)

#define SIM_PCE0_GPIOD_MASK  (0x8U)
#define SIM_PCE0_GPIOD_SHIFT (3U)
/*! GPIOD - GPIOD IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOD(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOD_SHIFT)) & SIM_PCE0_GPIOD_MASK)

#define SIM_PCE0_GPIOC_MASK  (0x10U)
#define SIM_PCE0_GPIOC_SHIFT (4U)
/*! GPIOC - GPIOC IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOC(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOC_SHIFT)) & SIM_PCE0_GPIOC_MASK)

#define SIM_PCE0_GPIOB_MASK  (0x20U)
#define SIM_PCE0_GPIOB_SHIFT (5U)
/*! GPIOB - GPIOB IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOB(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOB_SHIFT)) & SIM_PCE0_GPIOB_MASK)

#define SIM_PCE0_GPIOA_MASK  (0x40U)
#define SIM_PCE0_GPIOA_SHIFT (6U)
/*! GPIOA - GPIOA IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOA(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOA_SHIFT)) & SIM_PCE0_GPIOA_MASK)

#define SIM_PCE0_TB3_MASK  (0x100U)
#define SIM_PCE0_TB3_SHIFT (8U)
/*! TB3 - TMRB3 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TB3(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TB3_SHIFT)) & SIM_PCE0_TB3_MASK)

#define SIM_PCE0_TB2_MASK  (0x200U)
#define SIM_PCE0_TB2_SHIFT (9U)
/*! TB2 - TMRB2 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TB2(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TB2_SHIFT)) & SIM_PCE0_TB2_MASK)

#define SIM_PCE0_TB1_MASK  (0x400U)
#define SIM_PCE0_TB1_SHIFT (10U)
/*! TB1 - TMRB1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TB1(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TB1_SHIFT)) & SIM_PCE0_TB1_MASK)

#define SIM_PCE0_TB0_MASK  (0x800U)
#define SIM_PCE0_TB0_SHIFT (11U)
/*! TB0 - TMRB0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TB0(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TB0_SHIFT)) & SIM_PCE0_TB0_MASK)

#define SIM_PCE0_TA3_MASK  (0x1000U)
#define SIM_PCE0_TA3_SHIFT (12U)
/*! TA3 - TMRA3 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TA3(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TA3_SHIFT)) & SIM_PCE0_TA3_MASK)

#define SIM_PCE0_TA2_MASK  (0x2000U)
#define SIM_PCE0_TA2_SHIFT (13U)
/*! TA2 - TMRA2 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TA2(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TA2_SHIFT)) & SIM_PCE0_TA2_MASK)

#define SIM_PCE0_TA1_MASK  (0x4000U)
#define SIM_PCE0_TA1_SHIFT (14U)
/*! TA1 - TMRA1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TA1(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TA1_SHIFT)) & SIM_PCE0_TA1_MASK)

#define SIM_PCE0_TA0_MASK  (0x8000U)
#define SIM_PCE0_TA0_SHIFT (15U)
/*! TA0 - TMRA0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TA0(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TA0_SHIFT)) & SIM_PCE0_TA0_MASK)
/*! @} */

/*! @name PCE1 - Peripheral Clock Enable Register 1 */
/*! @{ */

#define SIM_PCE1_FLEXCAN_MASK  (0x1U)
#define SIM_PCE1_FLEXCAN_SHIFT (0U)
/*! FLEXCAN - FlexCAN IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_FLEXCAN(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_FLEXCAN_SHIFT)) & SIM_PCE1_FLEXCAN_MASK)

#define SIM_PCE1_IIC1_MASK  (0x20U)
#define SIM_PCE1_IIC1_SHIFT (5U)
/*! IIC1 - IIC1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_IIC1(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_IIC1_SHIFT)) & SIM_PCE1_IIC1_MASK)

#define SIM_PCE1_IIC0_MASK  (0x40U)
#define SIM_PCE1_IIC0_SHIFT (6U)
/*! IIC0 - IIC0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_IIC0(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_IIC0_SHIFT)) & SIM_PCE1_IIC0_MASK)

#define SIM_PCE1_QSPI1_MASK  (0x100U)
#define SIM_PCE1_QSPI1_SHIFT (8U)
/*! QSPI1 - QSPI1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_QSPI1(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_QSPI1_SHIFT)) & SIM_PCE1_QSPI1_MASK)

#define SIM_PCE1_QSPI0_MASK  (0x200U)
#define SIM_PCE1_QSPI0_SHIFT (9U)
/*! QSPI0 - QSPI0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_QSPI0(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_QSPI0_SHIFT)) & SIM_PCE1_QSPI0_MASK)

#define SIM_PCE1_SCI2_MASK  (0x400U)
#define SIM_PCE1_SCI2_SHIFT (10U)
/*! SCI2 - SCI2 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_SCI2(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_SCI2_SHIFT)) & SIM_PCE1_SCI2_MASK)

#define SIM_PCE1_SCI1_MASK  (0x800U)
#define SIM_PCE1_SCI1_SHIFT (11U)
/*! SCI1 - SCI1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_SCI1(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_SCI1_SHIFT)) & SIM_PCE1_SCI1_MASK)

#define SIM_PCE1_SCI0_MASK  (0x1000U)
#define SIM_PCE1_SCI0_SHIFT (12U)
/*! SCI0 - SCI0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_SCI0(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_SCI0_SHIFT)) & SIM_PCE1_SCI0_MASK)

#define SIM_PCE1_DACA_MASK  (0x2000U)
#define SIM_PCE1_DACA_SHIFT (13U)
/*! DACA - DACA IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_DACA(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_DACA_SHIFT)) & SIM_PCE1_DACA_MASK)

#define SIM_PCE1_DACB_MASK  (0x4000U)
#define SIM_PCE1_DACB_SHIFT (14U)
/*! DACB - DACB IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_DACB(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_DACB_SHIFT)) & SIM_PCE1_DACB_MASK)
/*! @} */

/*! @name PCE2 - Peripheral Clock Enable Register 2 */
/*! @{ */

#define SIM_PCE2_PIT1_MASK  (0x4U)
#define SIM_PCE2_PIT1_SHIFT (2U)
/*! PIT1 - Programmable Interval Timer IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_PIT1(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_PIT1_SHIFT)) & SIM_PCE2_PIT1_MASK)

#define SIM_PCE2_PIT0_MASK  (0x8U)
#define SIM_PCE2_PIT0_SHIFT (3U)
/*! PIT0 - Programmable Interval Timer IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_PIT0(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_PIT0_SHIFT)) & SIM_PCE2_PIT0_MASK)

#define SIM_PCE2_CRC_MASK  (0x20U)
#define SIM_PCE2_CRC_SHIFT (5U)
/*! CRC - CRC IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_CRC(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_CRC_SHIFT)) & SIM_PCE2_CRC_MASK)

#define SIM_PCE2_CYCADC_MASK  (0x80U)
#define SIM_PCE2_CYCADC_SHIFT (7U)
/*! CYCADC - Cyclic ADC IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_CYCADC(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_CYCADC_SHIFT)) & SIM_PCE2_CYCADC_MASK)

#define SIM_PCE2_CMPD_MASK  (0x200U)
#define SIM_PCE2_CMPD_SHIFT (9U)
/*! CMPD - CMPD IPBus Clock Enable (enables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_CMPD(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_CMPD_SHIFT)) & SIM_PCE2_CMPD_MASK)

#define SIM_PCE2_CMPC_MASK  (0x400U)
#define SIM_PCE2_CMPC_SHIFT (10U)
/*! CMPC - CMPC IPBus Clock Enable (enables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_CMPC(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_CMPC_SHIFT)) & SIM_PCE2_CMPC_MASK)

#define SIM_PCE2_CMPB_MASK  (0x800U)
#define SIM_PCE2_CMPB_SHIFT (11U)
/*! CMPB - CMPB IPBus Clock Enable (enables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_CMPB(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_CMPB_SHIFT)) & SIM_PCE2_CMPB_MASK)

#define SIM_PCE2_CMPA_MASK  (0x1000U)
#define SIM_PCE2_CMPA_SHIFT (12U)
/*! CMPA - CMPA IPBus Clock Enable (enables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_CMPA(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_CMPA_SHIFT)) & SIM_PCE2_CMPA_MASK)
/*! @} */

/*! @name PCE3 - Peripheral Clock Enable Register 3 */
/*! @{ */

#define SIM_PCE3_PWMBCH3_MASK  (0x1U)
#define SIM_PCE3_PWMBCH3_SHIFT (0U)
/*! PWMBCH3 - PWMB Channel 3 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMBCH3(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMBCH3_SHIFT)) & SIM_PCE3_PWMBCH3_MASK)

#define SIM_PCE3_PWMBCH2_MASK  (0x2U)
#define SIM_PCE3_PWMBCH2_SHIFT (1U)
/*! PWMBCH2 - PWMB Channel 2 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMBCH2(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMBCH2_SHIFT)) & SIM_PCE3_PWMBCH2_MASK)

#define SIM_PCE3_PWMBCH1_MASK  (0x4U)
#define SIM_PCE3_PWMBCH1_SHIFT (2U)
/*! PWMBCH1 - PWMB Channel 1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMBCH1(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMBCH1_SHIFT)) & SIM_PCE3_PWMBCH1_MASK)

#define SIM_PCE3_PWMBCH0_MASK  (0x8U)
#define SIM_PCE3_PWMBCH0_SHIFT (3U)
/*! PWMBCH0 - PWMB Channel 0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMBCH0(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMBCH0_SHIFT)) & SIM_PCE3_PWMBCH0_MASK)

#define SIM_PCE3_PWMACH3_MASK  (0x10U)
#define SIM_PCE3_PWMACH3_SHIFT (4U)
/*! PWMACH3 - PWMA Channel 3 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMACH3(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMACH3_SHIFT)) & SIM_PCE3_PWMACH3_MASK)

#define SIM_PCE3_PWMACH2_MASK  (0x20U)
#define SIM_PCE3_PWMACH2_SHIFT (5U)
/*! PWMACH2 - PWMA Channel 2 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMACH2(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMACH2_SHIFT)) & SIM_PCE3_PWMACH2_MASK)

#define SIM_PCE3_PWMACH1_MASK  (0x40U)
#define SIM_PCE3_PWMACH1_SHIFT (6U)
/*! PWMACH1 - PWMA Channel 1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMACH1(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMACH1_SHIFT)) & SIM_PCE3_PWMACH1_MASK)

#define SIM_PCE3_PWMACH0_MASK  (0x80U)
#define SIM_PCE3_PWMACH0_SHIFT (7U)
/*! PWMACH0 - PWMA Channel 0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMACH0(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMACH0_SHIFT)) & SIM_PCE3_PWMACH0_MASK)

#define SIM_PCE3_USB_MASK  (0x100U)
#define SIM_PCE3_USB_SHIFT (8U)
/*! USB - USB IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_USB(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_USB_SHIFT)) & SIM_PCE3_USB_MASK)

#define SIM_PCE3_ROM_MASK  (0x200U)
#define SIM_PCE3_ROM_SHIFT (9U)
/*! ROM - ROM IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_ROM(x) (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_ROM_SHIFT)) & SIM_PCE3_ROM_MASK)
/*! @} */

/*! @name SD0 - Peripheral Clock STOP Disable Register 0 */
/*! @{ */

#define SIM_SD0_GPIOG_MASK  (0x1U)
#define SIM_SD0_GPIOG_SHIFT (0U)
/*! GPIOG - GPIOG IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOG(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOG_SHIFT)) & SIM_SD0_GPIOG_MASK)

#define SIM_SD0_GPIOF_MASK  (0x2U)
#define SIM_SD0_GPIOF_SHIFT (1U)
/*! GPIOF - GPIOF IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOF(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOF_SHIFT)) & SIM_SD0_GPIOF_MASK)

#define SIM_SD0_GPIOE_MASK  (0x4U)
#define SIM_SD0_GPIOE_SHIFT (2U)
/*! GPIOE - GPIOE IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOE(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOE_SHIFT)) & SIM_SD0_GPIOE_MASK)

#define SIM_SD0_GPIOD_MASK  (0x8U)
#define SIM_SD0_GPIOD_SHIFT (3U)
/*! GPIOD - GPIOD IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOD(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOD_SHIFT)) & SIM_SD0_GPIOD_MASK)

#define SIM_SD0_GPIOC_MASK  (0x10U)
#define SIM_SD0_GPIOC_SHIFT (4U)
/*! GPIOC - GPIOC IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOC(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOC_SHIFT)) & SIM_SD0_GPIOC_MASK)

#define SIM_SD0_GPIOB_MASK  (0x20U)
#define SIM_SD0_GPIOB_SHIFT (5U)
/*! GPIOB - GPIOB IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOB(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOB_SHIFT)) & SIM_SD0_GPIOB_MASK)

#define SIM_SD0_GPIOA_MASK  (0x40U)
#define SIM_SD0_GPIOA_SHIFT (6U)
/*! GPIOA - GPIOA IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOA(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOA_SHIFT)) & SIM_SD0_GPIOA_MASK)

#define SIM_SD0_TB3_MASK  (0x100U)
#define SIM_SD0_TB3_SHIFT (8U)
/*! TB3 - TMRB3 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TB3(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TB3_SHIFT)) & SIM_SD0_TB3_MASK)

#define SIM_SD0_TB2_MASK  (0x200U)
#define SIM_SD0_TB2_SHIFT (9U)
/*! TB2 - TMRB2 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TB2(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TB2_SHIFT)) & SIM_SD0_TB2_MASK)

#define SIM_SD0_TB1_MASK  (0x400U)
#define SIM_SD0_TB1_SHIFT (10U)
/*! TB1 - TMRB1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TB1(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TB1_SHIFT)) & SIM_SD0_TB1_MASK)

#define SIM_SD0_TB0_MASK  (0x800U)
#define SIM_SD0_TB0_SHIFT (11U)
/*! TB0 - TMRB0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TB0(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TB0_SHIFT)) & SIM_SD0_TB0_MASK)

#define SIM_SD0_TA3_MASK  (0x1000U)
#define SIM_SD0_TA3_SHIFT (12U)
/*! TA3 - TMRA3 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TA3(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TA3_SHIFT)) & SIM_SD0_TA3_MASK)

#define SIM_SD0_TA2_MASK  (0x2000U)
#define SIM_SD0_TA2_SHIFT (13U)
/*! TA2 - TMRA2 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TA2(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TA2_SHIFT)) & SIM_SD0_TA2_MASK)

#define SIM_SD0_TA1_MASK  (0x4000U)
#define SIM_SD0_TA1_SHIFT (14U)
/*! TA1 - TMRA1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TA1(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TA1_SHIFT)) & SIM_SD0_TA1_MASK)

#define SIM_SD0_TA0_MASK  (0x8000U)
#define SIM_SD0_TA0_SHIFT (15U)
/*! TA0 - TMRA0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TA0(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TA0_SHIFT)) & SIM_SD0_TA0_MASK)
/*! @} */

/*! @name SD1 - Peripheral Clock STOP Disable Register 1 */
/*! @{ */

#define SIM_SD1_FLEXCAN_MASK  (0x1U)
#define SIM_SD1_FLEXCAN_SHIFT (0U)
/*! FLEXCAN - FlexCAN IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_FLEXCAN(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD1_FLEXCAN_SHIFT)) & SIM_SD1_FLEXCAN_MASK)

#define SIM_SD1_IIC1_MASK  (0x20U)
#define SIM_SD1_IIC1_SHIFT (5U)
/*! IIC1 - IIC1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode, but the IIC1 module will not enter stop mode.
 */
#define SIM_SD1_IIC1(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD1_IIC1_SHIFT)) & SIM_SD1_IIC1_MASK)

#define SIM_SD1_IIC0_MASK  (0x40U)
#define SIM_SD1_IIC0_SHIFT (6U)
/*! IIC0 - IIC0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode, but the IIC0 module will not enter stop mode.
 */
#define SIM_SD1_IIC0(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD1_IIC0_SHIFT)) & SIM_SD1_IIC0_MASK)

#define SIM_SD1_QSPI1_MASK  (0x100U)
#define SIM_SD1_QSPI1_SHIFT (8U)
/*! QSPI1 - QSPI1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_QSPI1(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD1_QSPI1_SHIFT)) & SIM_SD1_QSPI1_MASK)

#define SIM_SD1_QSPI0_MASK  (0x200U)
#define SIM_SD1_QSPI0_SHIFT (9U)
/*! QSPI0 - QSPI0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_QSPI0(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD1_QSPI0_SHIFT)) & SIM_SD1_QSPI0_MASK)

#define SIM_SD1_SCI2_MASK  (0x400U)
#define SIM_SD1_SCI2_SHIFT (10U)
/*! SCI2 - SCI2 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_SCI2(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD1_SCI2_SHIFT)) & SIM_SD1_SCI2_MASK)

#define SIM_SD1_SCI1_MASK  (0x800U)
#define SIM_SD1_SCI1_SHIFT (11U)
/*! SCI1 - SCI1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_SCI1(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD1_SCI1_SHIFT)) & SIM_SD1_SCI1_MASK)

#define SIM_SD1_SCI0_MASK  (0x1000U)
#define SIM_SD1_SCI0_SHIFT (12U)
/*! SCI0 - SCI0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_SCI0(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD1_SCI0_SHIFT)) & SIM_SD1_SCI0_MASK)

#define SIM_SD1_DACA_MASK  (0x2000U)
#define SIM_SD1_DACA_SHIFT (13U)
/*! DACA - DACA IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_DACA(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD1_DACA_SHIFT)) & SIM_SD1_DACA_MASK)

#define SIM_SD1_DACB_MASK  (0x4000U)
#define SIM_SD1_DACB_SHIFT (14U)
/*! DACB - DACB IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_DACB(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD1_DACB_SHIFT)) & SIM_SD1_DACB_MASK)
/*! @} */

/*! @name SD2 - Peripheral Clock STOP Disable Register 2 */
/*! @{ */

#define SIM_SD2_PIT1_MASK  (0x4U)
#define SIM_SD2_PIT1_SHIFT (2U)
/*! PIT1 - Programmable Interval Timer IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_PIT1(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD2_PIT1_SHIFT)) & SIM_SD2_PIT1_MASK)

#define SIM_SD2_PIT0_MASK  (0x8U)
#define SIM_SD2_PIT0_SHIFT (3U)
/*! PIT0 - Programmable Interval Timer IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_PIT0(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD2_PIT0_SHIFT)) & SIM_SD2_PIT0_MASK)

#define SIM_SD2_CRC_MASK  (0x20U)
#define SIM_SD2_CRC_SHIFT (5U)
/*! CRC - CRC IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_CRC(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD2_CRC_SHIFT)) & SIM_SD2_CRC_MASK)

#define SIM_SD2_CYCADC_MASK  (0x80U)
#define SIM_SD2_CYCADC_SHIFT (7U)
/*! CYCADC - Cyclic ADC IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_CYCADC(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD2_CYCADC_SHIFT)) & SIM_SD2_CYCADC_MASK)

#define SIM_SD2_CMPD_MASK  (0x200U)
#define SIM_SD2_CMPD_SHIFT (9U)
/*! CMPD - CMPD IPBus STOP Disable (disables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_CMPD(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD2_CMPD_SHIFT)) & SIM_SD2_CMPD_MASK)

#define SIM_SD2_CMPC_MASK  (0x400U)
#define SIM_SD2_CMPC_SHIFT (10U)
/*! CMPC - CMPC IPBus STOP Disable (disables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_CMPC(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD2_CMPC_SHIFT)) & SIM_SD2_CMPC_MASK)

#define SIM_SD2_CMPB_MASK  (0x800U)
#define SIM_SD2_CMPB_SHIFT (11U)
/*! CMPB - CMPB IPBus STOP Disable (disables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_CMPB(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD2_CMPB_SHIFT)) & SIM_SD2_CMPB_MASK)

#define SIM_SD2_CMPA_MASK  (0x1000U)
#define SIM_SD2_CMPA_SHIFT (12U)
/*! CMPA - CMPA IPBus STOP Disable (disables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_CMPA(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD2_CMPA_SHIFT)) & SIM_SD2_CMPA_MASK)
/*! @} */

/*! @name SD3 - Peripheral Clock STOP Disable Register 3 */
/*! @{ */

#define SIM_SD3_PWMBCH3_MASK  (0x1U)
#define SIM_SD3_PWMBCH3_SHIFT (0U)
/*! PWMBCH3 - PWMB Channel 3 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMBCH3(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMBCH3_SHIFT)) & SIM_SD3_PWMBCH3_MASK)

#define SIM_SD3_PWMBCH2_MASK  (0x2U)
#define SIM_SD3_PWMBCH2_SHIFT (1U)
/*! PWMBCH2 - PWMB Channel 2 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMBCH2(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMBCH2_SHIFT)) & SIM_SD3_PWMBCH2_MASK)

#define SIM_SD3_PWMBCH1_MASK  (0x4U)
#define SIM_SD3_PWMBCH1_SHIFT (2U)
/*! PWMBCH1 - PWMB Channel 1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMBCH1(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMBCH1_SHIFT)) & SIM_SD3_PWMBCH1_MASK)

#define SIM_SD3_PWMBCH0_MASK  (0x8U)
#define SIM_SD3_PWMBCH0_SHIFT (3U)
/*! PWMBCH0 - PWMB Channel 0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMBCH0(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMBCH0_SHIFT)) & SIM_SD3_PWMBCH0_MASK)

#define SIM_SD3_PWMACH3_MASK  (0x10U)
#define SIM_SD3_PWMACH3_SHIFT (4U)
/*! PWMACH3 - PWMA Channel 3 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMACH3(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMACH3_SHIFT)) & SIM_SD3_PWMACH3_MASK)

#define SIM_SD3_PWMACH2_MASK  (0x20U)
#define SIM_SD3_PWMACH2_SHIFT (5U)
/*! PWMACH2 - PWMA Channel 2 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMACH2(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMACH2_SHIFT)) & SIM_SD3_PWMACH2_MASK)

#define SIM_SD3_PWMACH1_MASK  (0x40U)
#define SIM_SD3_PWMACH1_SHIFT (6U)
/*! PWMACH1 - PWMA Channel 1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMACH1(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMACH1_SHIFT)) & SIM_SD3_PWMACH1_MASK)

#define SIM_SD3_PWMACH0_MASK  (0x80U)
#define SIM_SD3_PWMACH0_SHIFT (7U)
/*! PWMACH0 - PWMA Channel 0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMACH0(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMACH0_SHIFT)) & SIM_SD3_PWMACH0_MASK)

#define SIM_SD3_USB_MASK  (0x100U)
#define SIM_SD3_USB_SHIFT (8U)
/*! USB - USB IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_USB(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD3_USB_SHIFT)) & SIM_SD3_USB_MASK)

#define SIM_SD3_ROM_MASK  (0x200U)
#define SIM_SD3_ROM_SHIFT (9U)
/*! ROM - ROM IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_ROM(x) (((uint16_t)(((uint16_t)(x)) << SIM_SD3_ROM_SHIFT)) & SIM_SD3_ROM_MASK)
/*! @} */

/*! @name IOSAHI - I/O Short Address Location Register */
/*! @{ */

#define SIM_IOSAHI_ISAL23_22_MASK  (0x3U)
#define SIM_IOSAHI_ISAL23_22_SHIFT (0U)
/*! ISAL23_22 - Bits 23:22 of the address
 */
#define SIM_IOSAHI_ISAL23_22(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_IOSAHI_ISAL23_22_SHIFT)) & SIM_IOSAHI_ISAL23_22_MASK)
/*! @} */

/*! @name IOSALO - I/O Short Address Location Register */
/*! @{ */

#define SIM_IOSALO_ISAL21_6_MASK  (0xFFFFU)
#define SIM_IOSALO_ISAL21_6_SHIFT (0U)
/*! ISAL21_6 - Bits 21:6 of the address
 */
#define SIM_IOSALO_ISAL21_6(x) (((uint16_t)(((uint16_t)(x)) << SIM_IOSALO_ISAL21_6_SHIFT)) & SIM_IOSALO_ISAL21_6_MASK)
/*! @} */

/*! @name PROT - Protection Register */
/*! @{ */

#define SIM_PROT_GIPSP_MASK  (0x3U)
#define SIM_PROT_GIPSP_SHIFT (0U)
/*! GIPSP - GPIO and Internal Peripheral Select Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define SIM_PROT_GIPSP(x) (((uint16_t)(((uint16_t)(x)) << SIM_PROT_GIPSP_SHIFT)) & SIM_PROT_GIPSP_MASK)

#define SIM_PROT_PCEP_MASK  (0xCU)
#define SIM_PROT_PCEP_SHIFT (2U)
/*! PCEP - Peripheral Clock Enable Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define SIM_PROT_PCEP(x) (((uint16_t)(((uint16_t)(x)) << SIM_PROT_PCEP_SHIFT)) & SIM_PROT_PCEP_MASK)

#define SIM_PROT_GDP_MASK  (0x30U)
#define SIM_PROT_GDP_SHIFT (4U)
/*! GDP - GPIO Port D Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define SIM_PROT_GDP(x) (((uint16_t)(((uint16_t)(x)) << SIM_PROT_GDP_SHIFT)) & SIM_PROT_GDP_MASK)

#define SIM_PROT_PMODE_MASK  (0xC0U)
#define SIM_PROT_PMODE_SHIFT (6U)
/*! PMODE - Power Mode Control Write Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define SIM_PROT_PMODE(x) (((uint16_t)(((uint16_t)(x)) << SIM_PROT_PMODE_SHIFT)) & SIM_PROT_PMODE_MASK)
/*! @} */

/*! @name GPSAL - GPIOA LSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSAL_A0_MASK  (0x1U)
#define SIM_GPSAL_A0_SHIFT (0U)
/*! A0 - Configure GPIO A0
 *  0b0..Function = ANA0/CMPA_IN3; Peripheral = ADC/CMPA; Direction = IN
 *  0b1..Function = CMPC_O; Peripheral = CMPC; Direction = OUT
 */
#define SIM_GPSAL_A0(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSAL_A0_SHIFT)) & SIM_GPSAL_A0_MASK)
/*! @} */

/*! @name GPSAH - GPIOA MSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSAH_A11_MASK  (0xC0U)
#define SIM_GPSAH_A11_SHIFT (6U)
/*! A11 - Configure GPIO A11
 *  0b00..Function = CMPC_O; Peripheral = CMPC; Direction = OUT
 *  0b01..Function = XB_IN9; Peripheral = XBAR; Direction = IN
 *  0b10..Function = XB_OUT10; Peripheral = XBAR; Direction = OUT
 *  0b11..Function = USB_SOFOUT; Peripheral = USB; Direction = OUT
 */
#define SIM_GPSAH_A11(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSAH_A11_SHIFT)) & SIM_GPSAH_A11_MASK)
/*! @} */

/*! @name GPSBL - GPIOB LSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSBL_B1_MASK  (0x4U)
#define SIM_GPSBL_B1_SHIFT (2U)
/*! B1 - Configure GPIO B1
 *  0b0..Function = ANB1/CMPB_IN0; Peripheral = ADC/CMPB; Direction = IN
 *  0b1..Function = DACB_O; Peripheral = DAC; Direction = OUT
 */
#define SIM_GPSBL_B1(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSBL_B1_SHIFT)) & SIM_GPSBL_B1_MASK)
/*! @} */

/*! @name GPSBH - GPIOB MSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSBH_B8_MASK  (0x3U)
#define SIM_GPSBH_B8_SHIFT (0U)
/*! B8 - Configure GPIO B8
 *  0b00..Function = CMPD_O; Peripheral = CMPD; Direction = OUT
 *  0b01..Function = XB_IN8; Peripheral = XBAR; Direction = IN
 *  0b10..Function = XB_OUT11; Peripheral = XBAR; Direction = OUT
 *  0b11..reserved
 */
#define SIM_GPSBH_B8(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSBH_B8_SHIFT)) & SIM_GPSBH_B8_MASK)
/*! @} */

/*! @name GPSCL - GPIOC LSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSCL_C0_MASK  (0x1U)
#define SIM_GPSCL_C0_SHIFT (0U)
/*! C0 - Configure GPIO C0
 *  0b0..Function = EXTAL; Peripheral = OSC; Direction = IN
 *  0b1..Function = CLKIN0; Peripheral = OCCS; Direction = IN
 */
#define SIM_GPSCL_C0(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C0_SHIFT)) & SIM_GPSCL_C0_MASK)

#define SIM_GPSCL_C2_MASK  (0x30U)
#define SIM_GPSCL_C2_SHIFT (4U)
/*! C2 - Configure GPIO C2
 *  0b00..Function = TXD0; Peripheral = SCI0; Direction = IO
 *  0b01..Function = TB0; Peripheral = TMRB; Direction = IO
 *  0b10..Function = XB_IN2; Peripheral = XBAR; Direction = IN
 *  0b11..Function = CLKOUT0; Peripheral = OCCS; Direction = OUT
 */
#define SIM_GPSCL_C2(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C2_SHIFT)) & SIM_GPSCL_C2_MASK)

#define SIM_GPSCL_C3_MASK  (0xC0U)
#define SIM_GPSCL_C3_SHIFT (6U)
/*! C3
 *  0b00..Function = TA0; Peripheral = TMRA; Direction = IO
 *  0b01..Function = CMPA_O; Peripheral = CMPA; Direction = OUT
 *  0b10..Function = RXD0; Peripheral = SCI0; Direction = IN
 *  0b11..Function = CLKIN1; Peripheral = OCCS; Direction = IN
 */
#define SIM_GPSCL_C3(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C3_SHIFT)) & SIM_GPSCL_C3_MASK)

#define SIM_GPSCL_C4_MASK  (0x300U)
#define SIM_GPSCL_C4_SHIFT (8U)
/*! C4 - Configure GPIO C4
 *  0b00..Function = TA1; Peripheral = TMRA; Direction = IO
 *  0b01..Function = CMPB_O; Peripheral = CMPB; Direction = OUT
 *  0b10..Function = XB_IN8; Peripheral = XBAR; Direction = IN
 *  0b11..Function = EWM_OUT_B; Peripheral = EWM; Direction = OUT
 */
#define SIM_GPSCL_C4(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C4_SHIFT)) & SIM_GPSCL_C4_MASK)

#define SIM_GPSCL_C5_MASK  (0x400U)
#define SIM_GPSCL_C5_SHIFT (10U)
/*! C5 - Configure GPIO C5
 *  0b0..Function = DACA_O; Peripheral = DAC; Direction = OUT
 *  0b1..Function = XB_IN7; Peripheral = XBAR; Direction = IN
 */
#define SIM_GPSCL_C5(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C5_SHIFT)) & SIM_GPSCL_C5_MASK)

#define SIM_GPSCL_C6_MASK  (0x3000U)
#define SIM_GPSCL_C6_SHIFT (12U)
/*! C6 - Configure GPIO C6
 *  0b00..Function = TA2; Peripheral = TMRA; Direction = IO
 *  0b01..Function = XB_IN3; Peripheral = XBAR; Direction = IN
 *  0b10..Function = CMP_REF; Peripheral = CMP A/B/C/D; Direction = IN
 *  0b11..Function = SS0_B; Peripheral = SPI0; Direction = IO
 */
#define SIM_GPSCL_C6(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C6_SHIFT)) & SIM_GPSCL_C6_MASK)

#define SIM_GPSCL_C7_MASK  (0xC000U)
#define SIM_GPSCL_C7_SHIFT (14U)
/*! C7 - Configure GPIO C7
 *  0b00..Function = SS0_B; Peripheral = SPI0; Direction = IO
 *  0b01..Function = TXD0; Peripheral = SCI0; Direction = OUT
 *  0b10..Function = XB_IN8; Peripheral = XBAR; Direction = IN
 *  0b11..Function = XB_OUT6; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSCL_C7(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C7_SHIFT)) & SIM_GPSCL_C7_MASK)
/*! @} */

/*! @name GPSCH - GPIOC MSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSCH_C8_MASK  (0x3U)
#define SIM_GPSCH_C8_SHIFT (0U)
/*! C8 - Configure GPIO C8
 *  0b00..Function = MISO0; Peripheral = SPI0; Direction = IO
 *  0b01..Function = RXD0; Peripheral = SCI0; Direction = IN
 *  0b10..Function = XB_IN9; Peripheral = XBAR; Direction = IN
 *  0b11..Reserved
 */
#define SIM_GPSCH_C8(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C8_SHIFT)) & SIM_GPSCH_C8_MASK)

#define SIM_GPSCH_C9_MASK  (0xCU)
#define SIM_GPSCH_C9_SHIFT (2U)
/*! C9 - Configure GPIO C9
 *  0b00..Function = SCLK0; Peripheral = SPI0; Direction = IO
 *  0b01..Function = XB_IN4; Peripheral = XBAR; Direction = IN
 *  0b10..Function = TXD0; Peripheral = SCI0; Direction = OUT
 *  0b11..Function = XB_OUT8; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSCH_C9(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C9_SHIFT)) & SIM_GPSCH_C9_MASK)

#define SIM_GPSCH_C10_MASK  (0x30U)
#define SIM_GPSCH_C10_SHIFT (4U)
/*! C10 - Configure GPIO C10
 *  0b00..Function = MOSI0; Peripheral = SPI0; Direction = IO
 *  0b01..Function = XB_IN5; Peripheral = XBAR; Direction = IN
 *  0b10..Function = MISO0; Peripheral = SPI0; Direction = IO
 *  0b11..Function = XB_OUT9; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSCH_C10(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C10_SHIFT)) & SIM_GPSCH_C10_MASK)

#define SIM_GPSCH_C11_MASK  (0xC0U)
#define SIM_GPSCH_C11_SHIFT (6U)
/*! C11 - Configure GPIO C11
 *  0b00..Function = CANTX; Peripheral = FlexCAN; Direction = OUT
 *  0b01..Function = SCL1; Peripheral = IIC1; Direction = IO
 *  0b10..Function = TXD1; Peripheral = SCI1; Direction = IO
 *  0b11..Reserved
 */
#define SIM_GPSCH_C11(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C11_SHIFT)) & SIM_GPSCH_C11_MASK)

#define SIM_GPSCH_C12_MASK  (0x300U)
#define SIM_GPSCH_C12_SHIFT (8U)
/*! C12 - Configure GPIO C12
 *  0b00..Function = CANRX; Peripheral = FlexCAN; Direction = IN
 *  0b01..Function = SDA1; Peripheral = IIC1; Direction = IO
 *  0b10..Function = RXD1; Peripheral = SCI1; Direction = IN
 *  0b11..Reserved
 */
#define SIM_GPSCH_C12(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C12_SHIFT)) & SIM_GPSCH_C12_MASK)

#define SIM_GPSCH_C13_MASK  (0xC00U)
#define SIM_GPSCH_C13_SHIFT (10U)
/*! C13 - Configure GPIO C13
 *  0b00..Function = TA3; Peripheral = TMRA; Direction = IO
 *  0b01..Function = XB_IN6; Peripheral = XBAR; Direction = IN
 *  0b10..Function = EWM_OUT_B; Peripheral = EWM; Direction = OUT
 *  0b11..Reserved
 */
#define SIM_GPSCH_C13(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C13_SHIFT)) & SIM_GPSCH_C13_MASK)

#define SIM_GPSCH_C14_MASK  (0x3000U)
#define SIM_GPSCH_C14_SHIFT (12U)
/*! C14 - Configure GPIO C14
 *  0b00..Function = SDA0; Peripheral = IIC0; Direction = IO
 *  0b01..Function = XB_OUT4; Peripheral = XBAR; Direction = OUT
 *  0b10..Function = PWMA_FAULT4; Peripheral = PWMA; Direction = IN
 *  0b11..reserved
 */
#define SIM_GPSCH_C14(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C14_SHIFT)) & SIM_GPSCH_C14_MASK)

#define SIM_GPSCH_C15_MASK  (0xC000U)
#define SIM_GPSCH_C15_SHIFT (14U)
/*! C15 - Configure GPIO C15
 *  0b00..Function = SCL0; Peripheral = IIC0; Direction = IO
 *  0b01..Function = XB_OUT5; Peripheral = XBAR; Direction = OUT
 *  0b10..Function = PWMA_FAULT5; Peripheral = PWMA; Direction = IN
 *  0b11..reserved
 */
#define SIM_GPSCH_C15(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C15_SHIFT)) & SIM_GPSCH_C15_MASK)
/*! @} */

/*! @name GPSDL - GPIOD LSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSDL_D5_MASK  (0xC00U)
#define SIM_GPSDL_D5_SHIFT (10U)
/*! D5 - Configure GPIO D5
 *  0b00..Function = RXD2; Peripheral = SCI2; Direction = IN
 *  0b01..Function = XB_IN5; Peripheral = XBAR; Direction = IN
 *  0b10..Function = XB_OUT9; Peripheral = XBAR; Direction = OUT
 *  0b11..Reserved
 */
#define SIM_GPSDL_D5(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSDL_D5_SHIFT)) & SIM_GPSDL_D5_MASK)

#define SIM_GPSDL_D6_MASK  (0x3000U)
#define SIM_GPSDL_D6_SHIFT (12U)
/*! D6 - Configure GPIO D6
 *  0b00..Function = TXD2; Peripheral = SCI2; Direction = OUT
 *  0b01..Function = XB_IN4; Peripheral = XBAR; Direction = IN
 *  0b10..Function = XB_OUT8; Peripheral = XBAR; Direction = OUT
 *  0b11..Reserved
 */
#define SIM_GPSDL_D6(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSDL_D6_SHIFT)) & SIM_GPSDL_D6_MASK)

#define SIM_GPSDL_D7_MASK  (0xC000U)
#define SIM_GPSDL_D7_SHIFT (14U)
/*! D7 - Configure GPIO D7
 *  0b00..Function = XB_OUT11; Peripheral = XBAR; Direction = OUT
 *  0b01..Function = XB_IN7; Peripheral = XBAR; Direction = IN
 *  0b10..Function = MISO1; Peripheral = SPI1; Direction = IO
 *  0b11..Reserved
 */
#define SIM_GPSDL_D7(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSDL_D7_SHIFT)) & SIM_GPSDL_D7_MASK)
/*! @} */

/*! @name GPSEL - GPIOE LSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSEL_E0_MASK  (0x3U)
#define SIM_GPSEL_E0_SHIFT (0U)
/*! E0 - Configure GPIO E0
 *  0b00..Function = PWMA_0B; Peripheral = PWMA; Direction = IO
 *  0b01..reserved
 *  0b10..reserved
 *  0b11..Function = XB_OUT4; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E0(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E0_SHIFT)) & SIM_GPSEL_E0_MASK)

#define SIM_GPSEL_E1_MASK  (0xCU)
#define SIM_GPSEL_E1_SHIFT (2U)
/*! E1 - Configure GPIO E1
 *  0b00..Function = PWMA_0A; Peripheral = PWMA; Direction = IO
 *  0b01..reserved
 *  0b10..reserved
 *  0b11..Function = XB_OUT5; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E1(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E1_SHIFT)) & SIM_GPSEL_E1_MASK)

#define SIM_GPSEL_E2_MASK  (0x30U)
#define SIM_GPSEL_E2_SHIFT (4U)
/*! E2 - Configure GPIO E2
 *  0b00..Function = PWMA_1B; Peripheral = PWMA; Direction = IO
 *  0b01..reserved
 *  0b10..reserved
 *  0b11..Function = XB_OUT6; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E2(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E2_SHIFT)) & SIM_GPSEL_E2_MASK)

#define SIM_GPSEL_E3_MASK  (0xC0U)
#define SIM_GPSEL_E3_SHIFT (6U)
/*! E3 - Configure GPIO E3
 *  0b00..Function = PWMA_1A; Peripheral = PWMA; Direction = IO
 *  0b01..reserved
 *  0b10..reserved
 *  0b11..Function = XB_OUT7; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E3(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E3_SHIFT)) & SIM_GPSEL_E3_MASK)

#define SIM_GPSEL_E4_MASK  (0x300U)
#define SIM_GPSEL_E4_SHIFT (8U)
/*! E4 - Configure GPIO E4
 *  0b00..Function = PWMA_2B; Peripheral = PWMA; Direction = IO
 *  0b01..Function = XB_IN2; Peripheral = XBAR; Direction = IN
 *  0b10..reserved
 *  0b11..Function = XB_OUT8; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E4(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E4_SHIFT)) & SIM_GPSEL_E4_MASK)

#define SIM_GPSEL_E5_MASK  (0xC00U)
#define SIM_GPSEL_E5_SHIFT (10U)
/*! E5 - Configure GPIO E5
 *  0b00..Function = PWMA_2A; Peripheral = PWMA; Direction = IO
 *  0b01..Function = XB_IN3; Peripheral = XBAR; Direction = IN
 *  0b10..reserved
 *  0b11..Function = XB_OUT9; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E5(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E5_SHIFT)) & SIM_GPSEL_E5_MASK)

#define SIM_GPSEL_E6_MASK  (0x3000U)
#define SIM_GPSEL_E6_SHIFT (12U)
/*! E6 - Configure GPIO E6
 *  0b00..Function = PWMA_3B; Peripheral = PWMA; Direction = IO
 *  0b01..Function = XB_IN4; Peripheral = XBAR; Direction = IN
 *  0b10..Function = PWMB_2B; Peripheral = PWMB; Direction = IO
 *  0b11..Function = XB_OUT10; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E6(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E6_SHIFT)) & SIM_GPSEL_E6_MASK)

#define SIM_GPSEL_E7_MASK  (0xC000U)
#define SIM_GPSEL_E7_SHIFT (14U)
/*! E7 - Configure GPIO E7
 *  0b00..Function = PWMA_3A; Peripheral = PWMA; Direction = IO
 *  0b01..Function = XB_IN5; Peripheral = XBAR; Direction = IN
 *  0b10..Function = PWMB_2A; Peripheral = PWMB; Direction = IO
 *  0b11..Function = XB_OUT11; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E7(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E7_SHIFT)) & SIM_GPSEL_E7_MASK)
/*! @} */

/*! @name GPSEH - GPIOE MSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSEH_E8_MASK  (0x3U)
#define SIM_GPSEH_E8_SHIFT (0U)
/*! E8 - Configure GPIO E8
 *  0b00..Function = PWMB_2B; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_FAULT0; Peripheral = PWMA; Direction = IN
 *  0b10..reserved
 *  0b11..Function = XB_OUT8; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEH_E8(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSEH_E8_SHIFT)) & SIM_GPSEH_E8_MASK)

#define SIM_GPSEH_E9_MASK  (0xCU)
#define SIM_GPSEH_E9_SHIFT (2U)
/*! E9 - Configure GPIO E9
 *  0b00..Function = PWMB_2A; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_FAULT1; Peripheral = PWMA; Direction = IN
 *  0b10..reserved
 *  0b11..Function = XB_OUT9; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEH_E9(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSEH_E9_SHIFT)) & SIM_GPSEH_E9_MASK)
/*! @} */

/*! @name GPSFL - GPIOF LSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSFL_F0_MASK  (0x3U)
#define SIM_GPSFL_F0_SHIFT (0U)
/*! F0 - Configure GPIO F0
 *  0b00..Function = XB_IN6; Peripheral = XBAR; Direction = IN
 *  0b01..Function = TB2; Peripheral = TMRB; Direction = IO
 *  0b10..Function = SCLK1; Peripheral = SPI1; Direction = IO
 *  0b11..Reserved
 */
#define SIM_GPSFL_F0(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F0_SHIFT)) & SIM_GPSFL_F0_MASK)

#define SIM_GPSFL_F1_MASK  (0xCU)
#define SIM_GPSFL_F1_SHIFT (2U)
/*! F1 - Configure GPIO F1
 *  0b00..Function = CLKOUT1; Peripheral = OCCS; Direction = OUT
 *  0b01..Function = XB_IN7; Peripheral = XBAR; Direction = IN
 *  0b10..Function = CMPD_O; Peripheral = CMPD; Direction = OUT
 *  0b11..Reserved
 */
#define SIM_GPSFL_F1(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F1_SHIFT)) & SIM_GPSFL_F1_MASK)

#define SIM_GPSFL_F2_MASK  (0x30U)
#define SIM_GPSFL_F2_SHIFT (4U)
/*! F2 - Configure GPIO F2
 *  0b00..Function = SCL1; Peripheral = IIC1; Direction = IO
 *  0b01..Function = XB_OUT6; Peripheral = XBAR; Direction = OUT
 *  0b10..Function = MISO1; Peripheral = SPI1; Direction = IO
 *  0b11..reserved
 */
#define SIM_GPSFL_F2(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F2_SHIFT)) & SIM_GPSFL_F2_MASK)

#define SIM_GPSFL_F3_MASK  (0xC0U)
#define SIM_GPSFL_F3_SHIFT (6U)
/*! F3 - Configure GPIO F3
 *  0b00..Function = SDA1; Peripheral = IIC1; Direction = IO
 *  0b01..Function = XB_OUT7; Peripheral = XBAR; Direction = OUT
 *  0b10..Function = MOSI1; Peripheral = SPI1; Direction = IO
 *  0b11..reserved
 */
#define SIM_GPSFL_F3(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F3_SHIFT)) & SIM_GPSFL_F3_MASK)

#define SIM_GPSFL_F4_MASK  (0x300U)
#define SIM_GPSFL_F4_SHIFT (8U)
/*! F4 - Configure GPIO F4
 *  0b00..Function = TXD1; Peripheral = SCI1; Direction = IO
 *  0b01..Function = XB_OUT8; Peripheral = XBAR; Direction = OUT
 *  0b10..Function = PWMA_0X; Peripheral = PWMA; Direction = IO
 *  0b11..Function = PWMA_FAULT6; Peripheral = PWMA; Direction = IN
 */
#define SIM_GPSFL_F4(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F4_SHIFT)) & SIM_GPSFL_F4_MASK)

#define SIM_GPSFL_F5_MASK  (0xC00U)
#define SIM_GPSFL_F5_SHIFT (10U)
/*! F5 - Configure GPIO F5
 *  0b00..Function = RXD1; Peripheral = SCI1; Direction = IN
 *  0b01..Function = XB_OUT9; Peripheral = XBAR; Direction = OUT
 *  0b10..Function = PWMA_1X; Peripheral = PWMA; Direction = IO
 *  0b11..Function = PWMA_FAULT7; Peripheral = PWMA; Direction = IN
 */
#define SIM_GPSFL_F5(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F5_SHIFT)) & SIM_GPSFL_F5_MASK)

#define SIM_GPSFL_F6_MASK  (0x3000U)
#define SIM_GPSFL_F6_SHIFT (12U)
/*! F6 - Configure GPIO F6
 *  0b00..Function = TB2; Peripheral = TMRB; Direction = IO
 *  0b01..Function = PWMA_3X; Peripheral = PWMA; Direction = IO
 *  0b10..Function = PWMB_3X; Peripheral = PWMB; Direction = IO
 *  0b11..Function = XB_IN2; Peripheral = XBAR; Direction = IN
 */
#define SIM_GPSFL_F6(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F6_SHIFT)) & SIM_GPSFL_F6_MASK)

#define SIM_GPSFL_F7_MASK  (0xC000U)
#define SIM_GPSFL_F7_SHIFT (14U)
/*! F7 - Configure GPIO F7
 *  0b00..Function = TB3; Peripheral = TMRB; Direction = IO
 *  0b01..Function = CMPC_O; Peripheral = HSCMPC; Direction = OUT
 *  0b10..Function = SS1_B; Peripheral = SPI1; Direction = IO
 *  0b11..Function = XB_IN3; Peripheral = XBAR; Direction = IN
 */
#define SIM_GPSFL_F7(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F7_SHIFT)) & SIM_GPSFL_F7_MASK)
/*! @} */

/*! @name GPSFH - GPIOF MSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSFH_F8_MASK  (0x3U)
#define SIM_GPSFH_F8_SHIFT (0U)
/*! F8 - Configure GPIO F8
 *  0b00..Function = RXD0; Peripheral = SCI0; Direction = IN
 *  0b01..Function =TB1; Peripheral = TMRB; Direction = IO
 *  0b10..Function = CMPD_O; Peripheral = HSCMPD; Direction = OUT
 *  0b11..Function = PWMA_2X; Peripheral = PWMA; Direction = IO
 */
#define SIM_GPSFH_F8(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F8_SHIFT)) & SIM_GPSFH_F8_MASK)

#define SIM_GPSFH_F9_MASK  (0xCU)
#define SIM_GPSFH_F9_SHIFT (2U)
/*! F9 - Configure GPIO F9
 *  0b00..Function = RXD2; Peripheral = SCI2; Direction = IN
 *  0b01..Function = PWMA_FAULT7; Peripheral = PWMA; Direction = IN
 *  0b10..Function = PWMB_FAULT7; Peripheral = PWMB; Direction = IN
 *  0b11..Function = XB_OUT11; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSFH_F9(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F9_SHIFT)) & SIM_GPSFH_F9_MASK)

#define SIM_GPSFH_F10_MASK  (0x30U)
#define SIM_GPSFH_F10_SHIFT (4U)
/*! F10 - Configure GPIO F10
 *  0b00..Function = TXD2; Peripheral = SCI2; Direction = IO
 *  0b01..Function = PWMA_FAULT6; Peripheral = PWMA; Direction = IN
 *  0b10..Function = PWMB_FAULT6; Peripheral = PWMB; Direction = IN
 *  0b11..Function = XB_OUT10; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSFH_F10(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F10_SHIFT)) & SIM_GPSFH_F10_MASK)

#define SIM_GPSFH_F11_MASK  (0x40U)
#define SIM_GPSFH_F11_SHIFT (6U)
/*! F11 - Configure GPIO F11
 *  0b0..Function = TXD0; Peripheral = SCI0; Direction = IO
 *  0b1..Function = XB_IN11; Peripheral = XBAR; Direction = IN
 */
#define SIM_GPSFH_F11(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F11_SHIFT)) & SIM_GPSFH_F11_MASK)

#define SIM_GPSFH_F12_MASK  (0x100U)
#define SIM_GPSFH_F12_SHIFT (8U)
/*! F12 - Configure GPIO F12
 *  0b0..Function = MISO1; Peripheral = SPI1; Direction = IO
 *  0b1..Function = PWMB_FAULT2; Peripheral = PWMB; Direction = IN
 */
#define SIM_GPSFH_F12(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F12_SHIFT)) & SIM_GPSFH_F12_MASK)

#define SIM_GPSFH_F13_MASK  (0x400U)
#define SIM_GPSFH_F13_SHIFT (10U)
/*! F13 - Configure GPIO F13
 *  0b0..Function = MOSI1; Peripheral = SPI1; Direction = IO
 *  0b1..Function = PWMB_FAULT1; Peripheral = PWMB; Direction = IN
 */
#define SIM_GPSFH_F13(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F13_SHIFT)) & SIM_GPSFH_F13_MASK)

#define SIM_GPSFH_F14_MASK  (0x1000U)
#define SIM_GPSFH_F14_SHIFT (12U)
/*! F14 - Configure GPIO F14
 *  0b0..Function = SCLK1; Peripheral = SPI1; Direction = IO
 *  0b1..Function = PWMB_FAULT0; Peripheral = PWMB; Direction = IN
 */
#define SIM_GPSFH_F14(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F14_SHIFT)) & SIM_GPSFH_F14_MASK)

#define SIM_GPSFH_F15_MASK  (0x4000U)
#define SIM_GPSFH_F15_SHIFT (14U)
/*! F15 - Configure GPIO F15
 *  0b0..Function = RXD0; Peripheral = SCI0; Direction = IN
 *  0b1..Function = XB_IN10; Peripheral = XBAR; Direction = IN
 */
#define SIM_GPSFH_F15(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F15_SHIFT)) & SIM_GPSFH_F15_MASK)
/*! @} */

/*! @name GPSGL - GPIOG LSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSGL_G0_MASK  (0x1U)
#define SIM_GPSGL_G0_SHIFT (0U)
/*! G0 - Configure GPIO G0
 *  0b0..Function = PWMB_1B; Peripheral = PWMB; Direction = IO
 *  0b1..Function = XB_OUT6; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G0(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G0_SHIFT)) & SIM_GPSGL_G0_MASK)

#define SIM_GPSGL_G1_MASK  (0x4U)
#define SIM_GPSGL_G1_SHIFT (2U)
/*! G1 - Configure GPIO G1
 *  0b0..Function = PWMB_1A; Peripheral = PWMB; Direction = IO
 *  0b1..Function = XB_OUT7; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G1(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G1_SHIFT)) & SIM_GPSGL_G1_MASK)

#define SIM_GPSGL_G2_MASK  (0x10U)
#define SIM_GPSGL_G2_SHIFT (4U)
/*! G2 - Configure GPIO G2
 *  0b0..Function = PWMB_0B; Peripheral = PWMB; Direction = IO
 *  0b1..Function = XB_OUT4; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G2(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G2_SHIFT)) & SIM_GPSGL_G2_MASK)

#define SIM_GPSGL_G3_MASK  (0x40U)
#define SIM_GPSGL_G3_SHIFT (6U)
/*! G3 - Configure GPIO G3
 *  0b0..Function = PWMB_0A; Peripheral = PWMB; Direction = IO
 *  0b1..Function = XB_OUT5; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G3(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G3_SHIFT)) & SIM_GPSGL_G3_MASK)

#define SIM_GPSGL_G4_MASK  (0x300U)
#define SIM_GPSGL_G4_SHIFT (8U)
/*! G4 - Configure GPIO G4
 *  0b00..Function = PWMB_3B; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_FAULT2; Peripheral = PWMA; Direction = IN
 *  0b10..reserved
 *  0b11..Function = XB_OUT10; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G4(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G4_SHIFT)) & SIM_GPSGL_G4_MASK)

#define SIM_GPSGL_G5_MASK  (0xC00U)
#define SIM_GPSGL_G5_SHIFT (10U)
/*! G5 - Configure GPIO G5
 *  0b00..Function = PWMB_3A; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_FAULT3; Peripheral = PWMA; Direction = IN
 *  0b10..reserved
 *  0b11..Function = XB_OUT11; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G5(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G5_SHIFT)) & SIM_GPSGL_G5_MASK)

#define SIM_GPSGL_G6_MASK  (0x3000U)
#define SIM_GPSGL_G6_SHIFT (12U)
/*! G6 - Configure GPIO G6
 *  0b00..Function = PWMA_FAULT4; Peripheral = PWMA; Direction = IN
 *  0b01..Function = PWMB_FAULT4; Peripheral = PWMB; Direction = IN
 *  0b10..Function = TB2; Peripheral = TMRB; Direction = IO
 *  0b11..Function = XB_OUT8; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G6(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G6_SHIFT)) & SIM_GPSGL_G6_MASK)

#define SIM_GPSGL_G7_MASK  (0xC000U)
#define SIM_GPSGL_G7_SHIFT (14U)
/*! G7 - Configure GPIO G7
 *  0b00..Function = PWMA_FAULT5; Peripheral = PWMA; Direction = IN
 *  0b01..Function = PWMB_FAULT5; Peripheral = PWMB; Direction = IN
 *  0b10..Function = XB_OUT9; Peripheral = XBAR; Direction = OUT
 *  0b11..Function = CLKIN2; Peripheral = USB; Direction = IN
 */
#define SIM_GPSGL_G7(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G7_SHIFT)) & SIM_GPSGL_G7_MASK)
/*! @} */

/*! @name GPSGH - GPIOG MSBs Peripheral Select Register */
/*! @{ */

#define SIM_GPSGH_G8_MASK  (0x3U)
#define SIM_GPSGH_G8_SHIFT (0U)
/*! G8 - Configure GPIO G8
 *  0b00..Function = PWMB_0X; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_0X; Peripheral = PWMA; Direction = IO
 *  0b10..Function = TA2; Peripheral = TMRA; Direction = IO
 *  0b11..Function = XB_OUT10; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGH_G8(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSGH_G8_SHIFT)) & SIM_GPSGH_G8_MASK)

#define SIM_GPSGH_G9_MASK  (0xCU)
#define SIM_GPSGH_G9_SHIFT (2U)
/*! G9 - Configure GPIO G9
 *  0b00..Function = PWMB_1X; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_1X; Peripheral = PWMA; Direction = IO
 *  0b10..Function = TA3; Peripheral = TMRA; Direction = IO
 *  0b11..Function = XB_OUT11; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGH_G9(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSGH_G9_SHIFT)) & SIM_GPSGH_G9_MASK)

#define SIM_GPSGH_G10_MASK  (0x30U)
#define SIM_GPSGH_G10_SHIFT (4U)
/*! G10 - Configure GPIO G10
 *  0b00..Function = PWMB_2X; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_2X; Peripheral = PWMA; Direction = IO
 *  0b10..Function = XB_IN8; Peripheral = XBAR; Direction = IN
 *  0b11..Reserved
 */
#define SIM_GPSGH_G10(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSGH_G10_SHIFT)) & SIM_GPSGH_G10_MASK)

#define SIM_GPSGH_G11_MASK  (0xC0U)
#define SIM_GPSGH_G11_SHIFT (6U)
/*! G11 - Configure GPIO G11
 *  0b00..Function = TB3; Peripheral = TMRB; Direction = IO
 *  0b01..Function = CLKOUT0; Peripheral = OCCS; Direction = OUT
 *  0b10..Function = MOSI1; Peripheral = SPI1; Direction = IO
 *  0b11..Reserved
 */
#define SIM_GPSGH_G11(x) (((uint16_t)(((uint16_t)(x)) << SIM_GPSGH_G11_SHIFT)) & SIM_GPSGH_G11_MASK)
/*! @} */

/*! @name IPS0 - Internal Peripheral Select Register 0 */
/*! @{ */

#define SIM_IPS0_PWMAF0_MASK  (0x1U)
#define SIM_IPS0_PWMAF0_SHIFT (0U)
/*! PWMAF0 - Select PWMA Fault 0 Input
 *  0b0..Function = GPIOE8; Peripheral = GPIOE; Direction = IN
 *  0b1..Function = XB_OUT29; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMAF0(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMAF0_SHIFT)) & SIM_IPS0_PWMAF0_MASK)

#define SIM_IPS0_PWMAF1_MASK  (0x2U)
#define SIM_IPS0_PWMAF1_SHIFT (1U)
/*! PWMAF1 - Select PWMA Fault 1 Input
 *  0b0..Function = GPIOE9; Peripheral = GPIOE; Direction = IN
 *  0b1..Function = XB_OUT30; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMAF1(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMAF1_SHIFT)) & SIM_IPS0_PWMAF1_MASK)

#define SIM_IPS0_PWMAF2_MASK  (0x4U)
#define SIM_IPS0_PWMAF2_SHIFT (2U)
/*! PWMAF2 - Select PWMA Fault 2 Input
 *  0b0..Function = GPIOG4; Peripheral = GPIOG; Direction = IN
 *  0b1..Function = XB_OUT31; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMAF2(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMAF2_SHIFT)) & SIM_IPS0_PWMAF2_MASK)

#define SIM_IPS0_PWMAF3_MASK  (0x8U)
#define SIM_IPS0_PWMAF3_SHIFT (3U)
/*! PWMAF3 - Select PWMA Fault 3 Input
 *  0b0..Function = GPIOG5; Peripheral = GPIOG; Direction = IN
 *  0b1..Function = XB_OUT32; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMAF3(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMAF3_SHIFT)) & SIM_IPS0_PWMAF3_MASK)

#define SIM_IPS0_PWMBF0_MASK  (0x10U)
#define SIM_IPS0_PWMBF0_SHIFT (4U)
/*! PWMBF0 - Select PWMB Fault 0 Input
 *  0b0..Function = GPIOF14; Peripheral = GPIOF; Direction = IN
 *  0b1..Function = XB_OUT29; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMBF0(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMBF0_SHIFT)) & SIM_IPS0_PWMBF0_MASK)

#define SIM_IPS0_PWMBF1_MASK  (0x20U)
#define SIM_IPS0_PWMBF1_SHIFT (5U)
/*! PWMBF1 - Select PWMB Fault 1 Input
 *  0b0..Function = GPIOF13; Peripheral = GPIOF; Direction = IN
 *  0b1..Function = XB_OUT30; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMBF1(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMBF1_SHIFT)) & SIM_IPS0_PWMBF1_MASK)

#define SIM_IPS0_PWMBF2_MASK  (0x40U)
#define SIM_IPS0_PWMBF2_SHIFT (6U)
/*! PWMBF2 - Select PWMB Fault 2 Input
 *  0b0..Function = GPIOF12; Peripheral = GPIOF; Direction = IN
 *  0b1..Function = XB_OUT31; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMBF2(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMBF2_SHIFT)) & SIM_IPS0_PWMBF2_MASK)

#define SIM_IPS0_TA0_MASK  (0x100U)
#define SIM_IPS0_TA0_SHIFT (8U)
/*! TA0 - Select TMRA0 Input
 *  0b0..Function = GPIOC3; Peripheral = GPIOC; Direction = IN
 *  0b1..Function = XB_OUT38; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TA0(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TA0_SHIFT)) & SIM_IPS0_TA0_MASK)

#define SIM_IPS0_TA1_MASK  (0x200U)
#define SIM_IPS0_TA1_SHIFT (9U)
/*! TA1 - Select TMRA1 Input
 *  0b0..Function = GPIOC4; Peripheral = GPIOC; Direction = IN
 *  0b1..Function = XB_OUT39; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TA1(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TA1_SHIFT)) & SIM_IPS0_TA1_MASK)

#define SIM_IPS0_TA2_MASK  (0x400U)
#define SIM_IPS0_TA2_SHIFT (10U)
/*! TA2 - Select TMRA2 Input
 *  0b0..Function = GPIOC6 or GPIOG8; Peripheral = GPIOC; Direction = IN
 *  0b1..Function = XB_OUT40; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TA2(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TA2_SHIFT)) & SIM_IPS0_TA2_MASK)

#define SIM_IPS0_TA3_MASK  (0x800U)
#define SIM_IPS0_TA3_SHIFT (11U)
/*! TA3 - Select TMRA3 Input
 *  0b0..Function = GPIOC13 or GPIOG9; Peripheral = GPIOC; Direction = IN
 *  0b1..Function = XB_OUT41; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TA3(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TA3_SHIFT)) & SIM_IPS0_TA3_MASK)

#define SIM_IPS0_TB0_MASK  (0x1000U)
#define SIM_IPS0_TB0_SHIFT (12U)
/*! TB0 - Select TMRB0 Input
 *  0b0..Function = GPIOC2; Peripheral = GPIOC; Direction = IN
 *  0b1..Function = XB_OUT34; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TB0(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TB0_SHIFT)) & SIM_IPS0_TB0_MASK)

#define SIM_IPS0_TB1_MASK  (0x2000U)
#define SIM_IPS0_TB1_SHIFT (13U)
/*! TB1 - Select TMRB1 Input
 *  0b0..Function = GPIOF8; Peripheral = GPIOF; Direction = IN
 *  0b1..Function = XB_OUT35; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TB1(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TB1_SHIFT)) & SIM_IPS0_TB1_MASK)

#define SIM_IPS0_TB2_MASK  (0x4000U)
#define SIM_IPS0_TB2_SHIFT (14U)
/*! TB2 - Select TMRB2 Input
 *  0b0..Function = GPIOF6, GPIOF0, or GPIOG6; Peripheral = GPIOF; Direction = IN
 *  0b1..Function = XB_OUT36; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TB2(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TB2_SHIFT)) & SIM_IPS0_TB2_MASK)

#define SIM_IPS0_TB3_MASK  (0x8000U)
#define SIM_IPS0_TB3_SHIFT (15U)
/*! TB3 - Select TMRB3 Input
 *  0b0..Function = GPIOF7 or GPIOG11; Peripheral = GPIOF; Direction = IN
 *  0b1..Function = XB_OUT37; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TB3(x) (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TB3_SHIFT)) & SIM_IPS0_TB3_MASK)
/*! @} */

/*! @name MISC0 - Miscellaneous Register 0 */
/*! @{ */

#define SIM_MISC0_PIT_MSTR_MASK  (0x1U)
#define SIM_MISC0_PIT_MSTR_SHIFT (0U)
/*! PIT_MSTR - Select Master Programmable Interval Timer (PIT)
 *  0b0..PIT0 is master PIT and PIT1 is slave PIT
 *  0b1..PIT1 is master PIT and PIT0 is slave PIT
 */
#define SIM_MISC0_PIT_MSTR(x) (((uint16_t)(((uint16_t)(x)) << SIM_MISC0_PIT_MSTR_SHIFT)) & SIM_MISC0_PIT_MSTR_MASK)

#define SIM_MISC0_CLKINSEL_MASK  (0x2U)
#define SIM_MISC0_CLKINSEL_SHIFT (1U)
/*! CLKINSEL - CLKIN Select
 *  0b0..CLKIN0 (GPIOC0 alt1) is selected as CLKIN
 *  0b1..CLKIN1 (GPIOC3 alt3) is selected as CLKIN
 */
#define SIM_MISC0_CLKINSEL(x) (((uint16_t)(((uint16_t)(x)) << SIM_MISC0_CLKINSEL_SHIFT)) & SIM_MISC0_CLKINSEL_MASK)

#define SIM_MISC0_SCTRL_REORDER_MASK  (0x8U)
#define SIM_MISC0_SCTRL_REORDER_SHIFT (3U)
/*! SCTRL_REORDER
 *  0b0..Normal order
 *  0b1..Enable the re-ordering of ADC scan control bits
 */
#define SIM_MISC0_SCTRL_REORDER(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_MISC0_SCTRL_REORDER_SHIFT)) & SIM_MISC0_SCTRL_REORDER_MASK)

#define SIM_MISC0_IRC48M_EN_MASK  (0x40U)
#define SIM_MISC0_IRC48M_EN_SHIFT (6U)
/*! IRC48M_EN - IRC48M enable bit
 *  0b0..IRC48MHz disable
 *  0b1..IRC48MHz enable
 */
#define SIM_MISC0_IRC48M_EN(x) (((uint16_t)(((uint16_t)(x)) << SIM_MISC0_IRC48M_EN_SHIFT)) & SIM_MISC0_IRC48M_EN_MASK)
/*! @} */

/*! @name PSWR0 - Peripheral Software Reset Register 0 */
/*! @{ */

#define SIM_PSWR0_GPIO_MASK  (0x40U)
#define SIM_PSWR0_GPIO_SHIFT (6U)
/*! GPIO - GPIO Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR0_GPIO(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR0_GPIO_SHIFT)) & SIM_PSWR0_GPIO_MASK)

#define SIM_PSWR0_TB_MASK  (0x800U)
#define SIM_PSWR0_TB_SHIFT (11U)
/*! TB - TMRB Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR0_TB(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR0_TB_SHIFT)) & SIM_PSWR0_TB_MASK)

#define SIM_PSWR0_TA_MASK  (0x8000U)
#define SIM_PSWR0_TA_SHIFT (15U)
/*! TA - TMRA Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR0_TA(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR0_TA_SHIFT)) & SIM_PSWR0_TA_MASK)
/*! @} */

/*! @name PSWR1 - Peripheral Software Reset Register 1 */
/*! @{ */

#define SIM_PSWR1_FLEXCAN_MASK  (0x1U)
#define SIM_PSWR1_FLEXCAN_SHIFT (0U)
/*! FLEXCAN - FlexCAN Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_FLEXCAN(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_FLEXCAN_SHIFT)) & SIM_PSWR1_FLEXCAN_MASK)

#define SIM_PSWR1_IIC1_MASK  (0x20U)
#define SIM_PSWR1_IIC1_SHIFT (5U)
/*! IIC1 - IIC1 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_IIC1(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_IIC1_SHIFT)) & SIM_PSWR1_IIC1_MASK)

#define SIM_PSWR1_IIC0_MASK  (0x40U)
#define SIM_PSWR1_IIC0_SHIFT (6U)
/*! IIC0 - IIC0 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_IIC0(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_IIC0_SHIFT)) & SIM_PSWR1_IIC0_MASK)

#define SIM_PSWR1_QSPI1_MASK  (0x100U)
#define SIM_PSWR1_QSPI1_SHIFT (8U)
/*! QSPI1 - QSPI1 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_QSPI1(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_QSPI1_SHIFT)) & SIM_PSWR1_QSPI1_MASK)

#define SIM_PSWR1_QSPI0_MASK  (0x200U)
#define SIM_PSWR1_QSPI0_SHIFT (9U)
/*! QSPI0 - QSPI0 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_QSPI0(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_QSPI0_SHIFT)) & SIM_PSWR1_QSPI0_MASK)

#define SIM_PSWR1_SCI2_MASK  (0x400U)
#define SIM_PSWR1_SCI2_SHIFT (10U)
/*! SCI2 - SCI2 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_SCI2(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_SCI2_SHIFT)) & SIM_PSWR1_SCI2_MASK)

#define SIM_PSWR1_SCI1_MASK  (0x800U)
#define SIM_PSWR1_SCI1_SHIFT (11U)
/*! SCI1 - SCI1 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_SCI1(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_SCI1_SHIFT)) & SIM_PSWR1_SCI1_MASK)

#define SIM_PSWR1_SCI0_MASK  (0x1000U)
#define SIM_PSWR1_SCI0_SHIFT (12U)
/*! SCI0 - SCI0 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_SCI0(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_SCI0_SHIFT)) & SIM_PSWR1_SCI0_MASK)

#define SIM_PSWR1_DACA_MASK  (0x2000U)
#define SIM_PSWR1_DACA_SHIFT (13U)
/*! DACA - DACA Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_DACA(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_DACA_SHIFT)) & SIM_PSWR1_DACA_MASK)

#define SIM_PSWR1_DACB_MASK  (0x4000U)
#define SIM_PSWR1_DACB_SHIFT (14U)
/*! DACB - DACB Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_DACB(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_DACB_SHIFT)) & SIM_PSWR1_DACB_MASK)
/*! @} */

/*! @name PSWR2 - Peripheral Software Reset Register 2 */
/*! @{ */

#define SIM_PSWR2_PIT1_MASK  (0x4U)
#define SIM_PSWR2_PIT1_SHIFT (2U)
/*! PIT1 - Programmable Interval Timer Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR2_PIT1(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR2_PIT1_SHIFT)) & SIM_PSWR2_PIT1_MASK)

#define SIM_PSWR2_PIT0_MASK  (0x8U)
#define SIM_PSWR2_PIT0_SHIFT (3U)
/*! PIT0 - Programmable Interval Timer Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR2_PIT0(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR2_PIT0_SHIFT)) & SIM_PSWR2_PIT0_MASK)

#define SIM_PSWR2_CRC_MASK  (0x20U)
#define SIM_PSWR2_CRC_SHIFT (5U)
/*! CRC - CRC Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR2_CRC(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR2_CRC_SHIFT)) & SIM_PSWR2_CRC_MASK)

#define SIM_PSWR2_CYCADC_MASK  (0x80U)
#define SIM_PSWR2_CYCADC_SHIFT (7U)
/*! CYCADC - Cyclic ADC Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR2_CYCADC(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR2_CYCADC_SHIFT)) & SIM_PSWR2_CYCADC_MASK)

#define SIM_PSWR2_CMP_MASK  (0x1000U)
#define SIM_PSWR2_CMP_SHIFT (12U)
/*! CMP - CMP Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR2_CMP(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR2_CMP_SHIFT)) & SIM_PSWR2_CMP_MASK)

#define SIM_PSWR2_EWM_MASK  (0x8000U)
#define SIM_PSWR2_EWM_SHIFT (15U)
/*! EWM - EWM Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR2_EWM(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR2_EWM_SHIFT)) & SIM_PSWR2_EWM_MASK)
/*! @} */

/*! @name PSWR3 - Peripheral Software Reset Register 3 */
/*! @{ */

#define SIM_PSWR3_PWMB_MASK  (0x8U)
#define SIM_PSWR3_PWMB_SHIFT (3U)
/*! PWMB - PWMB Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR3_PWMB(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR3_PWMB_SHIFT)) & SIM_PSWR3_PWMB_MASK)

#define SIM_PSWR3_PWMA_MASK  (0x80U)
#define SIM_PSWR3_PWMA_SHIFT (7U)
/*! PWMA - PWMA Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR3_PWMA(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR3_PWMA_SHIFT)) & SIM_PSWR3_PWMA_MASK)

#define SIM_PSWR3_USB_OTG_MASK  (0x100U)
#define SIM_PSWR3_USB_OTG_SHIFT (8U)
/*! USB_OTG - USB_OTG Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR3_USB_OTG(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR3_USB_OTG_SHIFT)) & SIM_PSWR3_USB_OTG_MASK)

#define SIM_PSWR3_DMA_MUX_MASK  (0x200U)
#define SIM_PSWR3_DMA_MUX_SHIFT (9U)
/*! DMA_MUX - DMA_MUX Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR3_DMA_MUX(x) (((uint16_t)(((uint16_t)(x)) << SIM_PSWR3_DMA_MUX_SHIFT)) & SIM_PSWR3_DMA_MUX_MASK)
/*! @} */

/*! @name PWRMODE - Power Mode Register */
/*! @{ */

#define SIM_PWRMODE_VLPMODE_MASK  (0x1U)
#define SIM_PWRMODE_VLPMODE_SHIFT (0U)
/*! VLPMODE - VLPMODE Entry/Exit
 *  0b0..Start exit from VLPMODE
 *  0b1..Start entry to VLPMODE
 */
#define SIM_PWRMODE_VLPMODE(x) (((uint16_t)(((uint16_t)(x)) << SIM_PWRMODE_VLPMODE_SHIFT)) & SIM_PWRMODE_VLPMODE_MASK)

#define SIM_PWRMODE_LPMODE_MASK  (0x2U)
#define SIM_PWRMODE_LPMODE_SHIFT (1U)
/*! LPMODE - LPMODE Entry/Exit
 *  0b0..Start exit from LPMODE
 *  0b1..Start entry to LPMODE
 */
#define SIM_PWRMODE_LPMODE(x) (((uint16_t)(((uint16_t)(x)) << SIM_PWRMODE_LPMODE_SHIFT)) & SIM_PWRMODE_LPMODE_MASK)

#define SIM_PWRMODE_VLPMS_MASK  (0x100U)
#define SIM_PWRMODE_VLPMS_SHIFT (8U)
/*! VLPMS - VLPMODE Status Indicator
 *  0b0..Not in VLPMODE
 *  0b1..In VLPMODE
 */
#define SIM_PWRMODE_VLPMS(x) (((uint16_t)(((uint16_t)(x)) << SIM_PWRMODE_VLPMS_SHIFT)) & SIM_PWRMODE_VLPMS_MASK)

#define SIM_PWRMODE_LPMS_MASK  (0x200U)
#define SIM_PWRMODE_LPMS_SHIFT (9U)
/*! LPMS - LPMODE Status Indicator
 *  0b0..Not in LPMODE
 *  0b1..In LPMODE
 */
#define SIM_PWRMODE_LPMS(x) (((uint16_t)(((uint16_t)(x)) << SIM_PWRMODE_LPMS_SHIFT)) & SIM_PWRMODE_LPMS_MASK)
/*! @} */

/*! @name NVMOPT6_LOW - Non-Volatile Memory Option Register 6 (Low) */
/*! @{ */

#define SIM_NVMOPT6_LOW_ROSC_200K_FTRIM_MASK  (0x1FFU)
#define SIM_NVMOPT6_LOW_ROSC_200K_FTRIM_SHIFT (0U)
#define SIM_NVMOPT6_LOW_ROSC_200K_FTRIM(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_NVMOPT6_LOW_ROSC_200K_FTRIM_SHIFT)) & SIM_NVMOPT6_LOW_ROSC_200K_FTRIM_MASK)

#define SIM_NVMOPT6_LOW_PMC_BGTRIM_MASK  (0xF000U)
#define SIM_NVMOPT6_LOW_PMC_BGTRIM_SHIFT (12U)
#define SIM_NVMOPT6_LOW_PMC_BGTRIM(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_NVMOPT6_LOW_PMC_BGTRIM_SHIFT)) & SIM_NVMOPT6_LOW_PMC_BGTRIM_MASK)
/*! @} */

/*! @name PWM_SEL - PWM Select Register */
/*! @{ */

#define SIM_PWM_SEL_XBAR_IN20_MASK  (0x1U)
#define SIM_PWM_SEL_XBAR_IN20_SHIFT (0U)
/*! XBAR_IN20
 *  0b0..PWMA0_MUX_TRIG0
 *  0b1..PWMB0_OUT_TRIG0
 */
#define SIM_PWM_SEL_XBAR_IN20(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN20_SHIFT)) & SIM_PWM_SEL_XBAR_IN20_MASK)

#define SIM_PWM_SEL_XBAR_IN21_MASK  (0x2U)
#define SIM_PWM_SEL_XBAR_IN21_SHIFT (1U)
/*! XBAR_IN21
 *  0b0..PWMA0_MUX_TRIG1
 *  0b1..PWMB0_OUT_TRIG1
 */
#define SIM_PWM_SEL_XBAR_IN21(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN21_SHIFT)) & SIM_PWM_SEL_XBAR_IN21_MASK)

#define SIM_PWM_SEL_XBAR_IN22_MASK  (0x4U)
#define SIM_PWM_SEL_XBAR_IN22_SHIFT (2U)
/*! XBAR_IN22
 *  0b0..PWMA1_MUX_TRIG0
 *  0b1..PWMB1_OUT_TRIG0
 */
#define SIM_PWM_SEL_XBAR_IN22(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN22_SHIFT)) & SIM_PWM_SEL_XBAR_IN22_MASK)

#define SIM_PWM_SEL_XBAR_IN23_MASK  (0x8U)
#define SIM_PWM_SEL_XBAR_IN23_SHIFT (3U)
/*! XBAR_IN23
 *  0b0..PWMA1_MUX_TRIG1
 *  0b1..PWMB1_OUT_TRIG1
 */
#define SIM_PWM_SEL_XBAR_IN23(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN23_SHIFT)) & SIM_PWM_SEL_XBAR_IN23_MASK)

#define SIM_PWM_SEL_XBAR_IN24_MASK  (0x10U)
#define SIM_PWM_SEL_XBAR_IN24_SHIFT (4U)
/*! XBAR_IN24
 *  0b0..PWMA2_MUX_TRIG0
 *  0b1..PWMB2_OUT_TRIG0
 */
#define SIM_PWM_SEL_XBAR_IN24(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN24_SHIFT)) & SIM_PWM_SEL_XBAR_IN24_MASK)

#define SIM_PWM_SEL_XBAR_IN25_MASK  (0x20U)
#define SIM_PWM_SEL_XBAR_IN25_SHIFT (5U)
/*! XBAR_IN25
 *  0b0..PWMA2_MUX_TRIG1
 *  0b1..PWMB2_OUT_TRIG1
 */
#define SIM_PWM_SEL_XBAR_IN25(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN25_SHIFT)) & SIM_PWM_SEL_XBAR_IN25_MASK)

#define SIM_PWM_SEL_XBAR_IN26_MASK  (0x40U)
#define SIM_PWM_SEL_XBAR_IN26_SHIFT (6U)
/*! XBAR_IN26
 *  0b0..PWMA3_MUX_TRIG0
 *  0b1..PWMB3_OUT_TRIG0
 */
#define SIM_PWM_SEL_XBAR_IN26(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN26_SHIFT)) & SIM_PWM_SEL_XBAR_IN26_MASK)

#define SIM_PWM_SEL_XBAR_IN27_MASK  (0x80U)
#define SIM_PWM_SEL_XBAR_IN27_SHIFT (7U)
/*! XBAR_IN27
 *  0b0..PWMA3_MUX_TRIG1
 *  0b1..PWMB3_OUT_TRIG1
 */
#define SIM_PWM_SEL_XBAR_IN27(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN27_SHIFT)) & SIM_PWM_SEL_XBAR_IN27_MASK)

#define SIM_PWM_SEL_XBAR_IN28_MASK  (0x100U)
#define SIM_PWM_SEL_XBAR_IN28_SHIFT (8U)
/*! XBAR_IN28
 *  0b0..PWMB0_MUX_TRIG0
 *  0b1..PWMA0_OUT_TRIG0
 */
#define SIM_PWM_SEL_XBAR_IN28(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN28_SHIFT)) & SIM_PWM_SEL_XBAR_IN28_MASK)

#define SIM_PWM_SEL_XBAR_IN29_MASK  (0x200U)
#define SIM_PWM_SEL_XBAR_IN29_SHIFT (9U)
/*! XBAR_IN29
 *  0b0..PWMB0_MUX_TRIG1
 *  0b1..PWMA0_OUT_TRIG1
 */
#define SIM_PWM_SEL_XBAR_IN29(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN29_SHIFT)) & SIM_PWM_SEL_XBAR_IN29_MASK)

#define SIM_PWM_SEL_XBAR_IN30_MASK  (0x400U)
#define SIM_PWM_SEL_XBAR_IN30_SHIFT (10U)
/*! XBAR_IN30
 *  0b0..PWMB1_MUX_TRIG0
 *  0b1..PWMA1_OUT_TRIG0
 */
#define SIM_PWM_SEL_XBAR_IN30(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN30_SHIFT)) & SIM_PWM_SEL_XBAR_IN30_MASK)

#define SIM_PWM_SEL_XBAR_IN31_MASK  (0x800U)
#define SIM_PWM_SEL_XBAR_IN31_SHIFT (11U)
/*! XBAR_IN31
 *  0b0..PWMB1_MUX_TRIG1
 *  0b1..PWMA1_OUT_TRIG1
 */
#define SIM_PWM_SEL_XBAR_IN31(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN31_SHIFT)) & SIM_PWM_SEL_XBAR_IN31_MASK)

#define SIM_PWM_SEL_XBAR_IN32_MASK  (0x1000U)
#define SIM_PWM_SEL_XBAR_IN32_SHIFT (12U)
/*! XBAR_IN32
 *  0b0..PWMB2_MUX_TRIG0
 *  0b1..PWMA2_OUT_TRIG0
 */
#define SIM_PWM_SEL_XBAR_IN32(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN32_SHIFT)) & SIM_PWM_SEL_XBAR_IN32_MASK)

#define SIM_PWM_SEL_XBAR_IN33_MASK  (0x2000U)
#define SIM_PWM_SEL_XBAR_IN33_SHIFT (13U)
/*! XBAR_IN33
 *  0b0..PWMB2_MUX_TRIG1
 *  0b1..PWMA2_OUT_TRIG1
 */
#define SIM_PWM_SEL_XBAR_IN33(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN33_SHIFT)) & SIM_PWM_SEL_XBAR_IN33_MASK)

#define SIM_PWM_SEL_XBAR_IN34_MASK  (0x4000U)
#define SIM_PWM_SEL_XBAR_IN34_SHIFT (14U)
/*! XBAR_IN34
 *  0b0..PWMB3_MUX_TRIG0
 *  0b1..PWMA3_OUT_TRIG0
 */
#define SIM_PWM_SEL_XBAR_IN34(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN34_SHIFT)) & SIM_PWM_SEL_XBAR_IN34_MASK)

#define SIM_PWM_SEL_XBAR_IN35_MASK  (0x8000U)
#define SIM_PWM_SEL_XBAR_IN35_SHIFT (15U)
/*! XBAR_IN35
 *  0b0..PWMB3_MUX_TRIG1
 *  0b1..PWMA3_OUT_TRIG1
 */
#define SIM_PWM_SEL_XBAR_IN35(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN35_SHIFT)) & SIM_PWM_SEL_XBAR_IN35_MASK)
/*! @} */

/*! @name ADC_TMR_SEL - ADC and TMR Select Register */
/*! @{ */

#define SIM_ADC_TMR_SEL_XBAR_IN36_MASK  (0x1U)
#define SIM_ADC_TMR_SEL_XBAR_IN36_SHIFT (0U)
/*! XBAR_IN36
 *  0b0..TMRA0
 *  0b1..ADC AN0 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN36(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN36_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN36_MASK)

#define SIM_ADC_TMR_SEL_XBAR_IN37_MASK  (0x2U)
#define SIM_ADC_TMR_SEL_XBAR_IN37_SHIFT (1U)
/*! XBAR_IN37
 *  0b0..TMRA1
 *  0b1..ADC AN1 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN37(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN37_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN37_MASK)

#define SIM_ADC_TMR_SEL_XBAR_IN38_MASK  (0x4U)
#define SIM_ADC_TMR_SEL_XBAR_IN38_SHIFT (2U)
/*! XBAR_IN38
 *  0b0..TMRA2
 *  0b1..ADC AN2 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN38(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN38_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN38_MASK)

#define SIM_ADC_TMR_SEL_XBAR_IN39_MASK  (0x8U)
#define SIM_ADC_TMR_SEL_XBAR_IN39_SHIFT (3U)
/*! XBAR_IN39
 *  0b0..TMRA3
 *  0b1..ADC AN3 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN39(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN39_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN39_MASK)

#define SIM_ADC_TMR_SEL_XBAR_IN16_MASK  (0x100U)
#define SIM_ADC_TMR_SEL_XBAR_IN16_SHIFT (8U)
/*! XBAR_IN16
 *  0b0..TMRB0
 *  0b1..ADC AN8 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN16(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN16_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN16_MASK)

#define SIM_ADC_TMR_SEL_XBAR_IN17_MASK  (0x200U)
#define SIM_ADC_TMR_SEL_XBAR_IN17_SHIFT (9U)
/*! XBAR_IN17
 *  0b0..TMRB1
 *  0b1..ADC AN9 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN17(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN17_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN17_MASK)

#define SIM_ADC_TMR_SEL_XBAR_IN18_MASK  (0x400U)
#define SIM_ADC_TMR_SEL_XBAR_IN18_SHIFT (10U)
/*! XBAR_IN18
 *  0b0..TMRB2
 *  0b1..ADC AN10 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN18(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN18_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN18_MASK)

#define SIM_ADC_TMR_SEL_XBAR_IN19_MASK  (0x800U)
#define SIM_ADC_TMR_SEL_XBAR_IN19_SHIFT (11U)
/*! XBAR_IN19
 *  0b0..TMRB3
 *  0b1..ADC AN11 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN19(x) \
    (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN19_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN19_MASK)
/*! @} */

/*! @name BOOT_MODE_OVERRIDE - Boot Mode Override Register */
/*! @{ */

#define SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE_MASK  (0x3U)
#define SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE_SHIFT (0U)
/*! BOOT_OVERRIDE - FOPT[7:6]&(~BOOT_OVERRIDE) determines the boot option. BOOT_OVERRIDE acts as a mask to FOPT[7:6].
 *  0b00..FOPT[7:6] not masked.
 *  0b01..FOPT[7] is not masked. FOPT[6] is masked.
 *  0b10..FOPT[7] is masked. FOPT[6] is not masked.
 *  0b11..FOPT[7:6] masked.
 */
#define SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE_SHIFT)) & \
     SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group SIM_Register_Masks */

/* SIM - Peripheral instance base addresses */
/** Peripheral SIM base address */
#define SIM_BASE (0xE400u)
/** Peripheral SIM base pointer */
#define SIM ((SIM_Type *)SIM_BASE)
/** Array initializer of SIM peripheral base addresses */
#define SIM_BASE_ADDRS \
    {                  \
        SIM_BASE       \
    }
/** Array initializer of SIM peripheral base pointers */
#define SIM_BASE_PTRS \
    {                 \
        SIM           \
    }

/*!
 * @}
 */ /* end of group SIM_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- TMR Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TMR_Peripheral_Access_Layer TMR Peripheral Access Layer
 * @{
 */

/** TMR - Register Layout Typedef */
typedef struct
{
    struct
    {                         /* offset: 0x0, array step: 0x10 */
        __IO uint16_t COMP1;  /**< Timer Channel Compare Register 1, array offset: 0x0, array step: 0x10 */
        __IO uint16_t COMP2;  /**< Timer Channel Compare Register 2, array offset: 0x1, array step: 0x10 */
        __IO uint16_t CAPT;   /**< Timer Channel Capture Register, array offset: 0x2, array step: 0x10 */
        __IO uint16_t LOAD;   /**< Timer Channel Load Register, array offset: 0x3, array step: 0x10 */
        __IO uint16_t HOLD;   /**< Timer Channel Hold Register, array offset: 0x4, array step: 0x10 */
        __IO uint16_t CNTR;   /**< Timer Channel Counter Register, array offset: 0x5, array step: 0x10 */
        __IO uint16_t CTRL;   /**< Timer Channel Control Register, array offset: 0x6, array step: 0x10 */
        __IO uint16_t SCTRL;  /**< Timer Channel Status and Control Register, array offset: 0x7, array step: 0x10 */
        __IO uint16_t CMPLD1; /**< Timer Channel Comparator Load Register 1, array offset: 0x8, array step: 0x10 */
        __IO uint16_t CMPLD2; /**< Timer Channel Comparator Load Register 2, array offset: 0x9, array step: 0x10 */
        __IO uint16_t
            CSCTRL; /**< Timer Channel Comparator Status and Control Register, array offset: 0xA, array step: 0x10 */
        __IO uint16_t FILT; /**< Timer Channel Input Filter Register, array offset: 0xB, array step: 0x10 */
        __IO uint16_t DMA;  /**< Timer Channel DMA Enable Register, array offset: 0xC, array step: 0x10 */
        uint16_t RESERVED_0[2];
        __IO uint16_t ENBL; /**< Timer Channel Enable Register, array offset: 0xF, array step: 0x10, this item is not
                               available for all array instances */
    } CHANNEL[4];
} TMR_Type;

/* ----------------------------------------------------------------------------
   -- TMR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TMR_Register_Masks TMR Register Masks
 * @{
 */

/*! @name COMP1 - Timer Channel Compare Register 1 */
/*! @{ */

#define TMR_COMP1_COMPARISON_1_MASK  (0xFFFFU)
#define TMR_COMP1_COMPARISON_1_SHIFT (0U)
/*! COMPARISON_1 - Comparison Value 1
 */
#define TMR_COMP1_COMPARISON_1(x) \
    (((uint16_t)(((uint16_t)(x)) << TMR_COMP1_COMPARISON_1_SHIFT)) & TMR_COMP1_COMPARISON_1_MASK)
/*! @} */

/* The count of TMR_COMP1 */
#define TMR_COMP1_COUNT (4U)

/*! @name COMP2 - Timer Channel Compare Register 2 */
/*! @{ */

#define TMR_COMP2_COMPARISON_2_MASK  (0xFFFFU)
#define TMR_COMP2_COMPARISON_2_SHIFT (0U)
/*! COMPARISON_2 - Comparison Value 2
 */
#define TMR_COMP2_COMPARISON_2(x) \
    (((uint16_t)(((uint16_t)(x)) << TMR_COMP2_COMPARISON_2_SHIFT)) & TMR_COMP2_COMPARISON_2_MASK)
/*! @} */

/* The count of TMR_COMP2 */
#define TMR_COMP2_COUNT (4U)

/*! @name CAPT - Timer Channel Capture Register */
/*! @{ */

#define TMR_CAPT_CAPTURE_MASK  (0xFFFFU)
#define TMR_CAPT_CAPTURE_SHIFT (0U)
/*! CAPTURE - Capture Value
 */
#define TMR_CAPT_CAPTURE(x) (((uint16_t)(((uint16_t)(x)) << TMR_CAPT_CAPTURE_SHIFT)) & TMR_CAPT_CAPTURE_MASK)
/*! @} */

/* The count of TMR_CAPT */
#define TMR_CAPT_COUNT (4U)

/*! @name LOAD - Timer Channel Load Register */
/*! @{ */

#define TMR_LOAD_LOAD_MASK  (0xFFFFU)
#define TMR_LOAD_LOAD_SHIFT (0U)
/*! LOAD - Timer Load Register
 */
#define TMR_LOAD_LOAD(x) (((uint16_t)(((uint16_t)(x)) << TMR_LOAD_LOAD_SHIFT)) & TMR_LOAD_LOAD_MASK)
/*! @} */

/* The count of TMR_LOAD */
#define TMR_LOAD_COUNT (4U)

/*! @name HOLD - Timer Channel Hold Register */
/*! @{ */

#define TMR_HOLD_HOLD_MASK  (0xFFFFU)
#define TMR_HOLD_HOLD_SHIFT (0U)
#define TMR_HOLD_HOLD(x)    (((uint16_t)(((uint16_t)(x)) << TMR_HOLD_HOLD_SHIFT)) & TMR_HOLD_HOLD_MASK)
/*! @} */

/* The count of TMR_HOLD */
#define TMR_HOLD_COUNT (4U)

/*! @name CNTR - Timer Channel Counter Register */
/*! @{ */

#define TMR_CNTR_COUNTER_MASK  (0xFFFFU)
#define TMR_CNTR_COUNTER_SHIFT (0U)
#define TMR_CNTR_COUNTER(x)    (((uint16_t)(((uint16_t)(x)) << TMR_CNTR_COUNTER_SHIFT)) & TMR_CNTR_COUNTER_MASK)
/*! @} */

/* The count of TMR_CNTR */
#define TMR_CNTR_COUNT (4U)

/*! @name CTRL - Timer Channel Control Register */
/*! @{ */

#define TMR_CTRL_OUTMODE_MASK  (0x7U)
#define TMR_CTRL_OUTMODE_SHIFT (0U)
/*! OUTMODE - Output Mode
 *  0b000..Asserted while counter is active
 *  0b001..Clear OFLAG output on successful compare
 *  0b010..Set OFLAG output on successful compare
 *  0b011..Toggle OFLAG output on successful compare
 *  0b100..Toggle OFLAG output using alternating compare registers
 *  0b101..Set on compare, cleared on secondary source input edge
 *  0b110..Set on compare, cleared on counter rollover
 *  0b111..Enable gated clock output while counter is active
 */
#define TMR_CTRL_OUTMODE(x) (((uint16_t)(((uint16_t)(x)) << TMR_CTRL_OUTMODE_SHIFT)) & TMR_CTRL_OUTMODE_MASK)

#define TMR_CTRL_COINIT_MASK  (0x8U)
#define TMR_CTRL_COINIT_SHIFT (3U)
/*! COINIT - Co-Channel Initialization
 *  0b0..Co-channel counter/timers cannot force a re-initialization of this counter/timer
 *  0b1..Co-channel counter/timers may force a re-initialization of this counter/timer
 */
#define TMR_CTRL_COINIT(x) (((uint16_t)(((uint16_t)(x)) << TMR_CTRL_COINIT_SHIFT)) & TMR_CTRL_COINIT_MASK)

#define TMR_CTRL_DIR_MASK  (0x10U)
#define TMR_CTRL_DIR_SHIFT (4U)
/*! DIR - Count Direction
 *  0b0..Count up.
 *  0b1..Count down.
 */
#define TMR_CTRL_DIR(x) (((uint16_t)(((uint16_t)(x)) << TMR_CTRL_DIR_SHIFT)) & TMR_CTRL_DIR_MASK)

#define TMR_CTRL_LENGTH_MASK  (0x20U)
#define TMR_CTRL_LENGTH_SHIFT (5U)
/*! LENGTH - Count Length
 *  0b0..Count until roll over at $FFFF and then continue by re-initializing the counter from the LOAD register.
 *  0b1..Count until compare, then re-initialize using the LOAD regsiter. If counting up, a successful compare
 *       occurs when the counter reaches a COMP1 value. If counting down, a successful compare occurs when the counter
 *       reaches a COMP2 value. When output mode $4 is used, alternating values of COMP1 and COMP2 are used to
 *       generate successful comparisons. For example, the counter counts until a COMP1 value is reached,
 *       re-initializes, counts until COMP2 value is reached, re-initializes, counts until COMP1 value is reached, and
 * so on.
 */
#define TMR_CTRL_LENGTH(x) (((uint16_t)(((uint16_t)(x)) << TMR_CTRL_LENGTH_SHIFT)) & TMR_CTRL_LENGTH_MASK)

#define TMR_CTRL_ONCE_MASK  (0x40U)
#define TMR_CTRL_ONCE_SHIFT (6U)
/*! ONCE - Count Once
 *  0b0..Count repeatedly.
 *  0b1..Count until compare and then stop. If counting up, a successful compare occurs when the counter reaches a
 *       COMP1 value. If counting down, a successful compare occurs when the counter reaches a COMP2 value. When
 *       output mode $4 is used, the counter re-initializes after reaching the COMP1 value, continues to count to
 *       the COMP2 value, and then stops.
 */
#define TMR_CTRL_ONCE(x) (((uint16_t)(((uint16_t)(x)) << TMR_CTRL_ONCE_SHIFT)) & TMR_CTRL_ONCE_MASK)

#define TMR_CTRL_SCS_MASK  (0x180U)
#define TMR_CTRL_SCS_SHIFT (7U)
/*! SCS - Secondary Count Source
 *  0b00..Counter 0 input pin
 *  0b01..Counter 1 input pin
 *  0b10..Counter 2 input pin
 *  0b11..Counter 3 input pin
 */
#define TMR_CTRL_SCS(x) (((uint16_t)(((uint16_t)(x)) << TMR_CTRL_SCS_SHIFT)) & TMR_CTRL_SCS_MASK)

#define TMR_CTRL_PCS_MASK  (0x1E00U)
#define TMR_CTRL_PCS_SHIFT (9U)
/*! PCS - Primary Count Source
 *  0b0000..Counter 0 input pin
 *  0b0001..Counter 1 input pin
 *  0b0010..Counter 2 input pin
 *  0b0011..Counter 3 input pin
 *  0b0100..Counter 0 output
 *  0b0101..Counter 1 output
 *  0b0110..Counter 2 output
 *  0b0111..Counter 3 output
 *  0b1000..IP bus clock divide by 1 prescaler
 *  0b1001..IP bus clock divide by 2 prescaler
 *  0b1010..IP bus clock divide by 4 prescaler
 *  0b1011..IP bus clock divide by 8 prescaler
 *  0b1100..IP bus clock divide by 16 prescaler
 *  0b1101..IP bus clock divide by 32 prescaler
 *  0b1110..IP bus clock divide by 64 prescaler
 *  0b1111..IP bus clock divide by 128 prescaler
 */
#define TMR_CTRL_PCS(x) (((uint16_t)(((uint16_t)(x)) << TMR_CTRL_PCS_SHIFT)) & TMR_CTRL_PCS_MASK)

#define TMR_CTRL_CM_MASK  (0xE000U)
#define TMR_CTRL_CM_SHIFT (13U)
/*! CM - Count Mode
 *  0b000..No operation
 *  0b001..Count rising edges of primary sourceRising edges are counted only when SCTRL[IPS] = 0. Falling edges
 *         are counted when SCTRL[IPS] = 1. If the primary count source is IP bus clock divide by 1, only rising
 *         edges are counted regardless of the value of SCTRL[IPS].
 *  0b010..Count rising and falling edges of primary sourceIP bus clock divide by 1 cannot be used as a primary count
 * source in edge count mode. 0b011..Count rising edges of primary source while secondary input high active
 *  0b100..Quadrature count mode, uses primary and secondary sources
 *  0b101..Count rising edges of primary source; secondary source specifies directionRising edges are counted only
 *         when SCTRL[IPS] = 0. Falling edges are counted when SCTRL[IPS] = 1.
 *  0b110..Edge of secondary source triggers primary count until compare
 *  0b111..Cascaded counter mode (up/down)The primary count source must be set to one of the counter outputs.
 */
#define TMR_CTRL_CM(x) (((uint16_t)(((uint16_t)(x)) << TMR_CTRL_CM_SHIFT)) & TMR_CTRL_CM_MASK)
/*! @} */

/* The count of TMR_CTRL */
#define TMR_CTRL_COUNT (4U)

/*! @name SCTRL - Timer Channel Status and Control Register */
/*! @{ */

#define TMR_SCTRL_OEN_MASK  (0x1U)
#define TMR_SCTRL_OEN_SHIFT (0U)
/*! OEN - Output Enable
 *  0b0..The external pin is configured as an input.
 *  0b1..The OFLAG output signal is driven on the external pin. Other timer groups using this external pin as
 *       their input see the driven value. The polarity of the signal is determined by OPS.
 */
#define TMR_SCTRL_OEN(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_OEN_SHIFT)) & TMR_SCTRL_OEN_MASK)

#define TMR_SCTRL_OPS_MASK  (0x2U)
#define TMR_SCTRL_OPS_SHIFT (1U)
/*! OPS - Output Polarity Select
 *  0b0..True polarity.
 *  0b1..Inverted polarity.
 */
#define TMR_SCTRL_OPS(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_OPS_SHIFT)) & TMR_SCTRL_OPS_MASK)

#define TMR_SCTRL_FORCE_MASK  (0x4U)
#define TMR_SCTRL_FORCE_SHIFT (2U)
/*! FORCE - Force OFLAG Output
 */
#define TMR_SCTRL_FORCE(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_FORCE_SHIFT)) & TMR_SCTRL_FORCE_MASK)

#define TMR_SCTRL_VAL_MASK  (0x8U)
#define TMR_SCTRL_VAL_SHIFT (3U)
/*! VAL - Forced OFLAG Value
 */
#define TMR_SCTRL_VAL(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_VAL_SHIFT)) & TMR_SCTRL_VAL_MASK)

#define TMR_SCTRL_EEOF_MASK  (0x10U)
#define TMR_SCTRL_EEOF_SHIFT (4U)
/*! EEOF - Enable External OFLAG Force
 */
#define TMR_SCTRL_EEOF(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_EEOF_SHIFT)) & TMR_SCTRL_EEOF_MASK)

#define TMR_SCTRL_MSTR_MASK  (0x20U)
#define TMR_SCTRL_MSTR_SHIFT (5U)
/*! MSTR - Master Mode
 */
#define TMR_SCTRL_MSTR(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_MSTR_SHIFT)) & TMR_SCTRL_MSTR_MASK)

#define TMR_SCTRL_CAPTURE_MODE_MASK  (0xC0U)
#define TMR_SCTRL_CAPTURE_MODE_SHIFT (6U)
/*! CAPTURE_MODE - Input Capture Mode
 *  0b00..Capture function is disabled
 *  0b01..Load capture register on rising edge (when IPS=0) or falling edge (when IPS=1) of input
 *  0b10..Load capture register on falling edge (when IPS=0) or rising edge (when IPS=1) of input
 *  0b11..Load capture register on both edges of input
 */
#define TMR_SCTRL_CAPTURE_MODE(x) \
    (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_CAPTURE_MODE_SHIFT)) & TMR_SCTRL_CAPTURE_MODE_MASK)

#define TMR_SCTRL_INPUT_MASK  (0x100U)
#define TMR_SCTRL_INPUT_SHIFT (8U)
/*! INPUT - External Input Signal
 */
#define TMR_SCTRL_INPUT(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_INPUT_SHIFT)) & TMR_SCTRL_INPUT_MASK)

#define TMR_SCTRL_IPS_MASK  (0x200U)
#define TMR_SCTRL_IPS_SHIFT (9U)
/*! IPS - Input Polarity Select
 */
#define TMR_SCTRL_IPS(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_IPS_SHIFT)) & TMR_SCTRL_IPS_MASK)

#define TMR_SCTRL_IEFIE_MASK  (0x400U)
#define TMR_SCTRL_IEFIE_SHIFT (10U)
/*! IEFIE - Input Edge Flag Interrupt Enable
 */
#define TMR_SCTRL_IEFIE(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_IEFIE_SHIFT)) & TMR_SCTRL_IEFIE_MASK)

#define TMR_SCTRL_IEF_MASK  (0x800U)
#define TMR_SCTRL_IEF_SHIFT (11U)
/*! IEF - Input Edge Flag
 */
#define TMR_SCTRL_IEF(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_IEF_SHIFT)) & TMR_SCTRL_IEF_MASK)

#define TMR_SCTRL_TOFIE_MASK  (0x1000U)
#define TMR_SCTRL_TOFIE_SHIFT (12U)
/*! TOFIE - Timer Overflow Flag Interrupt Enable
 */
#define TMR_SCTRL_TOFIE(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_TOFIE_SHIFT)) & TMR_SCTRL_TOFIE_MASK)

#define TMR_SCTRL_TOF_MASK  (0x2000U)
#define TMR_SCTRL_TOF_SHIFT (13U)
/*! TOF - Timer Overflow Flag
 */
#define TMR_SCTRL_TOF(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_TOF_SHIFT)) & TMR_SCTRL_TOF_MASK)

#define TMR_SCTRL_TCFIE_MASK  (0x4000U)
#define TMR_SCTRL_TCFIE_SHIFT (14U)
/*! TCFIE - Timer Compare Flag Interrupt Enable
 */
#define TMR_SCTRL_TCFIE(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_TCFIE_SHIFT)) & TMR_SCTRL_TCFIE_MASK)

#define TMR_SCTRL_TCF_MASK  (0x8000U)
#define TMR_SCTRL_TCF_SHIFT (15U)
/*! TCF - Timer Compare Flag
 */
#define TMR_SCTRL_TCF(x) (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL_TCF_SHIFT)) & TMR_SCTRL_TCF_MASK)
/*! @} */

/* The count of TMR_SCTRL */
#define TMR_SCTRL_COUNT (4U)

/*! @name CMPLD1 - Timer Channel Comparator Load Register 1 */
/*! @{ */

#define TMR_CMPLD1_COMPARATOR_LOAD_1_MASK  (0xFFFFU)
#define TMR_CMPLD1_COMPARATOR_LOAD_1_SHIFT (0U)
#define TMR_CMPLD1_COMPARATOR_LOAD_1(x) \
    (((uint16_t)(((uint16_t)(x)) << TMR_CMPLD1_COMPARATOR_LOAD_1_SHIFT)) & TMR_CMPLD1_COMPARATOR_LOAD_1_MASK)
/*! @} */

/* The count of TMR_CMPLD1 */
#define TMR_CMPLD1_COUNT (4U)

/*! @name CMPLD2 - Timer Channel Comparator Load Register 2 */
/*! @{ */

#define TMR_CMPLD2_COMPARATOR_LOAD_2_MASK  (0xFFFFU)
#define TMR_CMPLD2_COMPARATOR_LOAD_2_SHIFT (0U)
#define TMR_CMPLD2_COMPARATOR_LOAD_2(x) \
    (((uint16_t)(((uint16_t)(x)) << TMR_CMPLD2_COMPARATOR_LOAD_2_SHIFT)) & TMR_CMPLD2_COMPARATOR_LOAD_2_MASK)
/*! @} */

/* The count of TMR_CMPLD2 */
#define TMR_CMPLD2_COUNT (4U)

/*! @name CSCTRL - Timer Channel Comparator Status and Control Register */
/*! @{ */

#define TMR_CSCTRL_CL1_MASK  (0x3U)
#define TMR_CSCTRL_CL1_SHIFT (0U)
/*! CL1 - Compare Load Control 1
 *  0b00..Never preload
 *  0b01..Load upon successful compare with the value in COMP1
 *  0b10..Load upon successful compare with the value in COMP2
 *  0b11..Reserved
 */
#define TMR_CSCTRL_CL1(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_CL1_SHIFT)) & TMR_CSCTRL_CL1_MASK)

#define TMR_CSCTRL_CL2_MASK  (0xCU)
#define TMR_CSCTRL_CL2_SHIFT (2U)
/*! CL2 - Compare Load Control 2
 *  0b00..Never preload
 *  0b01..Load upon successful compare with the value in COMP1
 *  0b10..Load upon successful compare with the value in COMP2
 *  0b11..Reserved
 */
#define TMR_CSCTRL_CL2(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_CL2_SHIFT)) & TMR_CSCTRL_CL2_MASK)

#define TMR_CSCTRL_TCF1_MASK  (0x10U)
#define TMR_CSCTRL_TCF1_SHIFT (4U)
/*! TCF1 - Timer Compare 1 Interrupt Flag
 */
#define TMR_CSCTRL_TCF1(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_TCF1_SHIFT)) & TMR_CSCTRL_TCF1_MASK)

#define TMR_CSCTRL_TCF2_MASK  (0x20U)
#define TMR_CSCTRL_TCF2_SHIFT (5U)
/*! TCF2 - Timer Compare 2 Interrupt Flag
 */
#define TMR_CSCTRL_TCF2(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_TCF2_SHIFT)) & TMR_CSCTRL_TCF2_MASK)

#define TMR_CSCTRL_TCF1EN_MASK  (0x40U)
#define TMR_CSCTRL_TCF1EN_SHIFT (6U)
/*! TCF1EN - Timer Compare 1 Interrupt Enable
 */
#define TMR_CSCTRL_TCF1EN(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_TCF1EN_SHIFT)) & TMR_CSCTRL_TCF1EN_MASK)

#define TMR_CSCTRL_TCF2EN_MASK  (0x80U)
#define TMR_CSCTRL_TCF2EN_SHIFT (7U)
/*! TCF2EN - Timer Compare 2 Interrupt Enable
 */
#define TMR_CSCTRL_TCF2EN(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_TCF2EN_SHIFT)) & TMR_CSCTRL_TCF2EN_MASK)

#define TMR_CSCTRL_OFLAG_MASK  (0x100U)
#define TMR_CSCTRL_OFLAG_SHIFT (8U)
/*! OFLAG - Output flag
 */
#define TMR_CSCTRL_OFLAG(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_OFLAG_SHIFT)) & TMR_CSCTRL_OFLAG_MASK)

#define TMR_CSCTRL_UP_MASK  (0x200U)
#define TMR_CSCTRL_UP_SHIFT (9U)
/*! UP - Counting Direction Indicator
 *  0b0..The last count was in the DOWN direction.
 *  0b1..The last count was in the UP direction.
 */
#define TMR_CSCTRL_UP(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_UP_SHIFT)) & TMR_CSCTRL_UP_MASK)

#define TMR_CSCTRL_TCI_MASK  (0x400U)
#define TMR_CSCTRL_TCI_SHIFT (10U)
/*! TCI - Triggered Count Initialization Control
 *  0b0..Stop counter upon receiving a second trigger event while still counting from the first trigger event.
 *  0b1..Reload the counter upon receiving a second trigger event while still counting from the first trigger event.
 */
#define TMR_CSCTRL_TCI(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_TCI_SHIFT)) & TMR_CSCTRL_TCI_MASK)

#define TMR_CSCTRL_ROC_MASK  (0x800U)
#define TMR_CSCTRL_ROC_SHIFT (11U)
/*! ROC - Reload on Capture
 *  0b0..Do not reload the counter on a capture event.
 *  0b1..Reload the counter on a capture event.
 */
#define TMR_CSCTRL_ROC(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_ROC_SHIFT)) & TMR_CSCTRL_ROC_MASK)

#define TMR_CSCTRL_ALT_LOAD_MASK  (0x1000U)
#define TMR_CSCTRL_ALT_LOAD_SHIFT (12U)
/*! ALT_LOAD - Alternative Load Enable
 *  0b0..Counter can be re-initialized only with the LOAD register.
 *  0b1..Counter can be re-initialized with the LOAD or CMPLD2 registers depending on count direction.
 */
#define TMR_CSCTRL_ALT_LOAD(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_ALT_LOAD_SHIFT)) & TMR_CSCTRL_ALT_LOAD_MASK)

#define TMR_CSCTRL_FAULT_MASK  (0x2000U)
#define TMR_CSCTRL_FAULT_SHIFT (13U)
/*! FAULT - Fault Enable
 *  0b0..Fault function disabled.
 *  0b1..Fault function enabled.
 */
#define TMR_CSCTRL_FAULT(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_FAULT_SHIFT)) & TMR_CSCTRL_FAULT_MASK)

#define TMR_CSCTRL_DBG_EN_MASK  (0xC000U)
#define TMR_CSCTRL_DBG_EN_SHIFT (14U)
/*! DBG_EN - Debug Actions Enable
 *  0b00..Continue with normal operation during debug mode. (default)
 *  0b01..Halt TMR counter during debug mode.
 *  0b10..Force TMR output to logic 0 (prior to consideration of SCTRL[OPS]).
 *  0b11..Both halt counter and force output to 0 during debug mode.
 */
#define TMR_CSCTRL_DBG_EN(x) (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL_DBG_EN_SHIFT)) & TMR_CSCTRL_DBG_EN_MASK)
/*! @} */

/* The count of TMR_CSCTRL */
#define TMR_CSCTRL_COUNT (4U)

/*! @name FILT - Timer Channel Input Filter Register */
/*! @{ */

#define TMR_FILT_FILT_PER_MASK  (0xFFU)
#define TMR_FILT_FILT_PER_SHIFT (0U)
/*! FILT_PER - Input Filter Sample Period
 */
#define TMR_FILT_FILT_PER(x) (((uint16_t)(((uint16_t)(x)) << TMR_FILT_FILT_PER_SHIFT)) & TMR_FILT_FILT_PER_MASK)

#define TMR_FILT_FILT_CNT_MASK  (0x700U)
#define TMR_FILT_FILT_CNT_SHIFT (8U)
/*! FILT_CNT - Input Filter Sample Count
 */
#define TMR_FILT_FILT_CNT(x) (((uint16_t)(((uint16_t)(x)) << TMR_FILT_FILT_CNT_SHIFT)) & TMR_FILT_FILT_CNT_MASK)
/*! @} */

/* The count of TMR_FILT */
#define TMR_FILT_COUNT (4U)

/*! @name DMA - Timer Channel DMA Enable Register */
/*! @{ */

#define TMR_DMA_IEFDE_MASK  (0x1U)
#define TMR_DMA_IEFDE_SHIFT (0U)
/*! IEFDE - Input Edge Flag DMA Enable
 */
#define TMR_DMA_IEFDE(x) (((uint16_t)(((uint16_t)(x)) << TMR_DMA_IEFDE_SHIFT)) & TMR_DMA_IEFDE_MASK)

#define TMR_DMA_CMPLD1DE_MASK  (0x2U)
#define TMR_DMA_CMPLD1DE_SHIFT (1U)
/*! CMPLD1DE - Comparator Preload Register 1 DMA Enable
 */
#define TMR_DMA_CMPLD1DE(x) (((uint16_t)(((uint16_t)(x)) << TMR_DMA_CMPLD1DE_SHIFT)) & TMR_DMA_CMPLD1DE_MASK)

#define TMR_DMA_CMPLD2DE_MASK  (0x4U)
#define TMR_DMA_CMPLD2DE_SHIFT (2U)
/*! CMPLD2DE - Comparator Preload Register 2 DMA Enable
 */
#define TMR_DMA_CMPLD2DE(x) (((uint16_t)(((uint16_t)(x)) << TMR_DMA_CMPLD2DE_SHIFT)) & TMR_DMA_CMPLD2DE_MASK)
/*! @} */

/* The count of TMR_DMA */
#define TMR_DMA_COUNT (4U)

/*! @name ENBL - Timer Channel Enable Register */
/*! @{ */

#define TMR_ENBL_ENBL_MASK  (0xFU)
#define TMR_ENBL_ENBL_SHIFT (0U)
/*! ENBL - Timer Channel Enable
 *  0b0000..Timer channel is disabled.
 *  0b0001..Timer channel is enabled. (default)
 */
#define TMR_ENBL_ENBL(x) (((uint16_t)(((uint16_t)(x)) << TMR_ENBL_ENBL_SHIFT)) & TMR_ENBL_ENBL_MASK)
/*! @} */

/* The count of TMR_ENBL */
#define TMR_ENBL_COUNT (4U)

/*!
 * @}
 */ /* end of group TMR_Register_Masks */

/* TMR - Peripheral instance base addresses */
/** Peripheral TMRA base address */
#define TMRA_BASE (0xE140u)
/** Peripheral TMRA base pointer */
#define TMRA ((TMR_Type *)TMRA_BASE)
/** Peripheral TMRB base address */
#define TMRB_BASE (0xE180u)
/** Peripheral TMRB base pointer */
#define TMRB ((TMR_Type *)TMRB_BASE)
/** Array initializer of TMR peripheral base addresses */
#define TMR_BASE_ADDRS       \
    {                        \
        TMRA_BASE, TMRB_BASE \
    }
/** Array initializer of TMR peripheral base pointers */
#define TMR_BASE_PTRS \
    {                 \
        TMRA, TMRB    \
    }

/*!
 * @}
 */ /* end of group TMR_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- XBARA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARA_Peripheral_Access_Layer XBARA Peripheral Access Layer
 * @{
 */

/** XBARA - Register Layout Typedef */
typedef struct
{
    __IO uint16_t SEL0;  /**< Crossbar A Select Register 0, offset: 0x0 */
    __IO uint16_t SEL1;  /**< Crossbar A Select Register 1, offset: 0x1 */
    __IO uint16_t SEL2;  /**< Crossbar A Select Register 2, offset: 0x2 */
    __IO uint16_t SEL3;  /**< Crossbar A Select Register 3, offset: 0x3 */
    __IO uint16_t SEL4;  /**< Crossbar A Select Register 4, offset: 0x4 */
    __IO uint16_t SEL5;  /**< Crossbar A Select Register 5, offset: 0x5 */
    __IO uint16_t SEL6;  /**< Crossbar A Select Register 6, offset: 0x6 */
    __IO uint16_t SEL7;  /**< Crossbar A Select Register 7, offset: 0x7 */
    __IO uint16_t SEL8;  /**< Crossbar A Select Register 8, offset: 0x8 */
    __IO uint16_t SEL9;  /**< Crossbar A Select Register 9, offset: 0x9 */
    __IO uint16_t SEL10; /**< Crossbar A Select Register 10, offset: 0xA */
    __IO uint16_t SEL11; /**< Crossbar A Select Register 11, offset: 0xB */
    __IO uint16_t SEL12; /**< Crossbar A Select Register 12, offset: 0xC */
    __IO uint16_t SEL13; /**< Crossbar A Select Register 13, offset: 0xD */
    __IO uint16_t SEL14; /**< Crossbar A Select Register 14, offset: 0xE */
    __IO uint16_t SEL15; /**< Crossbar A Select Register 15, offset: 0xF */
    __IO uint16_t SEL16; /**< Crossbar A Select Register 16, offset: 0x10 */
    __IO uint16_t SEL17; /**< Crossbar A Select Register 17, offset: 0x11 */
    __IO uint16_t SEL18; /**< Crossbar A Select Register 18, offset: 0x12 */
    __IO uint16_t SEL19; /**< Crossbar A Select Register 19, offset: 0x13 */
    __IO uint16_t SEL20; /**< Crossbar A Select Register 20, offset: 0x14 */
    __IO uint16_t SEL21; /**< Crossbar A Select Register 21, offset: 0x15 */
    __IO uint16_t SEL22; /**< Crossbar A Select Register 22, offset: 0x16 */
    __IO uint16_t SEL23; /**< Crossbar A Select Register 23, offset: 0x17 */
    __IO uint16_t SEL24; /**< Crossbar A Select Register 24, offset: 0x18 */
    __IO uint16_t SEL25; /**< Crossbar A Select Register 25, offset: 0x19 */
    __IO uint16_t SEL26; /**< Crossbar A Select Register 26, offset: 0x1A */
    __IO uint16_t SEL27; /**< Crossbar A Select Register 27, offset: 0x1B */
    __IO uint16_t SEL28; /**< Crossbar A Select Register 28, offset: 0x1C */
    __IO uint16_t SEL29; /**< Crossbar A Select Register 29, offset: 0x1D */
    __IO uint16_t SEL30; /**< Crossbar A Select Register 30, offset: 0x1E */
    __IO uint16_t SEL31; /**< Crossbar A Select Register 31, offset: 0x1F */
    __IO uint16_t CTRL0; /**< Crossbar A Control Register 0, offset: 0x20 */
    __IO uint16_t CTRL1; /**< Crossbar A Control Register 1, offset: 0x21 */
} XBARA_Type;

/* ----------------------------------------------------------------------------
   -- XBARA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARA_Register_Masks XBARA Register Masks
 * @{
 */

/*! @name SEL0 - Crossbar A Select Register 0 */
/*! @{ */

#define XBARA_SEL0_SEL0_MASK  (0x3FU)
#define XBARA_SEL0_SEL0_SHIFT (0U)
#define XBARA_SEL0_SEL0(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL0_SEL0_SHIFT)) & XBARA_SEL0_SEL0_MASK)

#define XBARA_SEL0_SEL1_MASK  (0x3F00U)
#define XBARA_SEL0_SEL1_SHIFT (8U)
#define XBARA_SEL0_SEL1(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL0_SEL1_SHIFT)) & XBARA_SEL0_SEL1_MASK)
/*! @} */

/*! @name SEL1 - Crossbar A Select Register 1 */
/*! @{ */

#define XBARA_SEL1_SEL2_MASK  (0x3FU)
#define XBARA_SEL1_SEL2_SHIFT (0U)
#define XBARA_SEL1_SEL2(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL1_SEL2_SHIFT)) & XBARA_SEL1_SEL2_MASK)

#define XBARA_SEL1_SEL3_MASK  (0x3F00U)
#define XBARA_SEL1_SEL3_SHIFT (8U)
#define XBARA_SEL1_SEL3(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL1_SEL3_SHIFT)) & XBARA_SEL1_SEL3_MASK)
/*! @} */

/*! @name SEL2 - Crossbar A Select Register 2 */
/*! @{ */

#define XBARA_SEL2_SEL4_MASK  (0x3FU)
#define XBARA_SEL2_SEL4_SHIFT (0U)
#define XBARA_SEL2_SEL4(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL2_SEL4_SHIFT)) & XBARA_SEL2_SEL4_MASK)

#define XBARA_SEL2_SEL5_MASK  (0x3F00U)
#define XBARA_SEL2_SEL5_SHIFT (8U)
#define XBARA_SEL2_SEL5(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL2_SEL5_SHIFT)) & XBARA_SEL2_SEL5_MASK)
/*! @} */

/*! @name SEL3 - Crossbar A Select Register 3 */
/*! @{ */

#define XBARA_SEL3_SEL6_MASK  (0x3FU)
#define XBARA_SEL3_SEL6_SHIFT (0U)
#define XBARA_SEL3_SEL6(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL3_SEL6_SHIFT)) & XBARA_SEL3_SEL6_MASK)

#define XBARA_SEL3_SEL7_MASK  (0x3F00U)
#define XBARA_SEL3_SEL7_SHIFT (8U)
#define XBARA_SEL3_SEL7(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL3_SEL7_SHIFT)) & XBARA_SEL3_SEL7_MASK)
/*! @} */

/*! @name SEL4 - Crossbar A Select Register 4 */
/*! @{ */

#define XBARA_SEL4_SEL8_MASK  (0x3FU)
#define XBARA_SEL4_SEL8_SHIFT (0U)
#define XBARA_SEL4_SEL8(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL4_SEL8_SHIFT)) & XBARA_SEL4_SEL8_MASK)

#define XBARA_SEL4_SEL9_MASK  (0x3F00U)
#define XBARA_SEL4_SEL9_SHIFT (8U)
#define XBARA_SEL4_SEL9(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL4_SEL9_SHIFT)) & XBARA_SEL4_SEL9_MASK)
/*! @} */

/*! @name SEL5 - Crossbar A Select Register 5 */
/*! @{ */

#define XBARA_SEL5_SEL10_MASK  (0x3FU)
#define XBARA_SEL5_SEL10_SHIFT (0U)
#define XBARA_SEL5_SEL10(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL5_SEL10_SHIFT)) & XBARA_SEL5_SEL10_MASK)

#define XBARA_SEL5_SEL11_MASK  (0x3F00U)
#define XBARA_SEL5_SEL11_SHIFT (8U)
#define XBARA_SEL5_SEL11(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL5_SEL11_SHIFT)) & XBARA_SEL5_SEL11_MASK)
/*! @} */

/*! @name SEL6 - Crossbar A Select Register 6 */
/*! @{ */

#define XBARA_SEL6_SEL12_MASK  (0x3FU)
#define XBARA_SEL6_SEL12_SHIFT (0U)
#define XBARA_SEL6_SEL12(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL6_SEL12_SHIFT)) & XBARA_SEL6_SEL12_MASK)

#define XBARA_SEL6_SEL13_MASK  (0x3F00U)
#define XBARA_SEL6_SEL13_SHIFT (8U)
#define XBARA_SEL6_SEL13(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL6_SEL13_SHIFT)) & XBARA_SEL6_SEL13_MASK)
/*! @} */

/*! @name SEL7 - Crossbar A Select Register 7 */
/*! @{ */

#define XBARA_SEL7_SEL14_MASK  (0x3FU)
#define XBARA_SEL7_SEL14_SHIFT (0U)
#define XBARA_SEL7_SEL14(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL7_SEL14_SHIFT)) & XBARA_SEL7_SEL14_MASK)

#define XBARA_SEL7_SEL15_MASK  (0x3F00U)
#define XBARA_SEL7_SEL15_SHIFT (8U)
#define XBARA_SEL7_SEL15(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL7_SEL15_SHIFT)) & XBARA_SEL7_SEL15_MASK)
/*! @} */

/*! @name SEL8 - Crossbar A Select Register 8 */
/*! @{ */

#define XBARA_SEL8_SEL16_MASK  (0x3FU)
#define XBARA_SEL8_SEL16_SHIFT (0U)
#define XBARA_SEL8_SEL16(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL8_SEL16_SHIFT)) & XBARA_SEL8_SEL16_MASK)

#define XBARA_SEL8_SEL17_MASK  (0x3F00U)
#define XBARA_SEL8_SEL17_SHIFT (8U)
#define XBARA_SEL8_SEL17(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL8_SEL17_SHIFT)) & XBARA_SEL8_SEL17_MASK)
/*! @} */

/*! @name SEL9 - Crossbar A Select Register 9 */
/*! @{ */

#define XBARA_SEL9_SEL18_MASK  (0x3FU)
#define XBARA_SEL9_SEL18_SHIFT (0U)
#define XBARA_SEL9_SEL18(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL9_SEL18_SHIFT)) & XBARA_SEL9_SEL18_MASK)

#define XBARA_SEL9_SEL19_MASK  (0x3F00U)
#define XBARA_SEL9_SEL19_SHIFT (8U)
#define XBARA_SEL9_SEL19(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL9_SEL19_SHIFT)) & XBARA_SEL9_SEL19_MASK)
/*! @} */

/*! @name SEL10 - Crossbar A Select Register 10 */
/*! @{ */

#define XBARA_SEL10_SEL20_MASK  (0x3FU)
#define XBARA_SEL10_SEL20_SHIFT (0U)
#define XBARA_SEL10_SEL20(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL10_SEL20_SHIFT)) & XBARA_SEL10_SEL20_MASK)

#define XBARA_SEL10_SEL21_MASK  (0x3F00U)
#define XBARA_SEL10_SEL21_SHIFT (8U)
#define XBARA_SEL10_SEL21(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL10_SEL21_SHIFT)) & XBARA_SEL10_SEL21_MASK)
/*! @} */

/*! @name SEL11 - Crossbar A Select Register 11 */
/*! @{ */

#define XBARA_SEL11_SEL22_MASK  (0x3FU)
#define XBARA_SEL11_SEL22_SHIFT (0U)
#define XBARA_SEL11_SEL22(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL11_SEL22_SHIFT)) & XBARA_SEL11_SEL22_MASK)

#define XBARA_SEL11_SEL23_MASK  (0x3F00U)
#define XBARA_SEL11_SEL23_SHIFT (8U)
#define XBARA_SEL11_SEL23(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL11_SEL23_SHIFT)) & XBARA_SEL11_SEL23_MASK)
/*! @} */

/*! @name SEL12 - Crossbar A Select Register 12 */
/*! @{ */

#define XBARA_SEL12_SEL24_MASK  (0x3FU)
#define XBARA_SEL12_SEL24_SHIFT (0U)
#define XBARA_SEL12_SEL24(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL12_SEL24_SHIFT)) & XBARA_SEL12_SEL24_MASK)

#define XBARA_SEL12_SEL25_MASK  (0x3F00U)
#define XBARA_SEL12_SEL25_SHIFT (8U)
#define XBARA_SEL12_SEL25(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL12_SEL25_SHIFT)) & XBARA_SEL12_SEL25_MASK)
/*! @} */

/*! @name SEL13 - Crossbar A Select Register 13 */
/*! @{ */

#define XBARA_SEL13_SEL26_MASK  (0x3FU)
#define XBARA_SEL13_SEL26_SHIFT (0U)
#define XBARA_SEL13_SEL26(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL13_SEL26_SHIFT)) & XBARA_SEL13_SEL26_MASK)

#define XBARA_SEL13_SEL27_MASK  (0x3F00U)
#define XBARA_SEL13_SEL27_SHIFT (8U)
#define XBARA_SEL13_SEL27(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL13_SEL27_SHIFT)) & XBARA_SEL13_SEL27_MASK)
/*! @} */

/*! @name SEL14 - Crossbar A Select Register 14 */
/*! @{ */

#define XBARA_SEL14_SEL28_MASK  (0x3FU)
#define XBARA_SEL14_SEL28_SHIFT (0U)
#define XBARA_SEL14_SEL28(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL14_SEL28_SHIFT)) & XBARA_SEL14_SEL28_MASK)

#define XBARA_SEL14_SEL29_MASK  (0x3F00U)
#define XBARA_SEL14_SEL29_SHIFT (8U)
#define XBARA_SEL14_SEL29(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL14_SEL29_SHIFT)) & XBARA_SEL14_SEL29_MASK)
/*! @} */

/*! @name SEL15 - Crossbar A Select Register 15 */
/*! @{ */

#define XBARA_SEL15_SEL30_MASK  (0x3FU)
#define XBARA_SEL15_SEL30_SHIFT (0U)
#define XBARA_SEL15_SEL30(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL15_SEL30_SHIFT)) & XBARA_SEL15_SEL30_MASK)

#define XBARA_SEL15_SEL31_MASK  (0x3F00U)
#define XBARA_SEL15_SEL31_SHIFT (8U)
#define XBARA_SEL15_SEL31(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL15_SEL31_SHIFT)) & XBARA_SEL15_SEL31_MASK)
/*! @} */

/*! @name SEL16 - Crossbar A Select Register 16 */
/*! @{ */

#define XBARA_SEL16_SEL32_MASK  (0x3FU)
#define XBARA_SEL16_SEL32_SHIFT (0U)
#define XBARA_SEL16_SEL32(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL16_SEL32_SHIFT)) & XBARA_SEL16_SEL32_MASK)

#define XBARA_SEL16_SEL33_MASK  (0x3F00U)
#define XBARA_SEL16_SEL33_SHIFT (8U)
#define XBARA_SEL16_SEL33(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL16_SEL33_SHIFT)) & XBARA_SEL16_SEL33_MASK)
/*! @} */

/*! @name SEL17 - Crossbar A Select Register 17 */
/*! @{ */

#define XBARA_SEL17_SEL34_MASK  (0x3FU)
#define XBARA_SEL17_SEL34_SHIFT (0U)
#define XBARA_SEL17_SEL34(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL17_SEL34_SHIFT)) & XBARA_SEL17_SEL34_MASK)

#define XBARA_SEL17_SEL35_MASK  (0x3F00U)
#define XBARA_SEL17_SEL35_SHIFT (8U)
#define XBARA_SEL17_SEL35(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL17_SEL35_SHIFT)) & XBARA_SEL17_SEL35_MASK)
/*! @} */

/*! @name SEL18 - Crossbar A Select Register 18 */
/*! @{ */

#define XBARA_SEL18_SEL36_MASK  (0x3FU)
#define XBARA_SEL18_SEL36_SHIFT (0U)
#define XBARA_SEL18_SEL36(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL18_SEL36_SHIFT)) & XBARA_SEL18_SEL36_MASK)

#define XBARA_SEL18_SEL37_MASK  (0x3F00U)
#define XBARA_SEL18_SEL37_SHIFT (8U)
#define XBARA_SEL18_SEL37(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL18_SEL37_SHIFT)) & XBARA_SEL18_SEL37_MASK)
/*! @} */

/*! @name SEL19 - Crossbar A Select Register 19 */
/*! @{ */

#define XBARA_SEL19_SEL38_MASK  (0x3FU)
#define XBARA_SEL19_SEL38_SHIFT (0U)
#define XBARA_SEL19_SEL38(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL19_SEL38_SHIFT)) & XBARA_SEL19_SEL38_MASK)

#define XBARA_SEL19_SEL39_MASK  (0x3F00U)
#define XBARA_SEL19_SEL39_SHIFT (8U)
#define XBARA_SEL19_SEL39(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL19_SEL39_SHIFT)) & XBARA_SEL19_SEL39_MASK)
/*! @} */

/*! @name SEL20 - Crossbar A Select Register 20 */
/*! @{ */

#define XBARA_SEL20_SEL40_MASK  (0x3FU)
#define XBARA_SEL20_SEL40_SHIFT (0U)
#define XBARA_SEL20_SEL40(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL20_SEL40_SHIFT)) & XBARA_SEL20_SEL40_MASK)

#define XBARA_SEL20_SEL41_MASK  (0x3F00U)
#define XBARA_SEL20_SEL41_SHIFT (8U)
#define XBARA_SEL20_SEL41(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL20_SEL41_SHIFT)) & XBARA_SEL20_SEL41_MASK)
/*! @} */

/*! @name SEL21 - Crossbar A Select Register 21 */
/*! @{ */

#define XBARA_SEL21_SEL42_MASK  (0x3FU)
#define XBARA_SEL21_SEL42_SHIFT (0U)
#define XBARA_SEL21_SEL42(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL21_SEL42_SHIFT)) & XBARA_SEL21_SEL42_MASK)

#define XBARA_SEL21_SEL43_MASK  (0x3F00U)
#define XBARA_SEL21_SEL43_SHIFT (8U)
#define XBARA_SEL21_SEL43(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL21_SEL43_SHIFT)) & XBARA_SEL21_SEL43_MASK)
/*! @} */

/*! @name SEL22 - Crossbar A Select Register 22 */
/*! @{ */

#define XBARA_SEL22_SEL44_MASK  (0x3FU)
#define XBARA_SEL22_SEL44_SHIFT (0U)
#define XBARA_SEL22_SEL44(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL22_SEL44_SHIFT)) & XBARA_SEL22_SEL44_MASK)

#define XBARA_SEL22_SEL45_MASK  (0x3F00U)
#define XBARA_SEL22_SEL45_SHIFT (8U)
#define XBARA_SEL22_SEL45(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL22_SEL45_SHIFT)) & XBARA_SEL22_SEL45_MASK)
/*! @} */

/*! @name SEL23 - Crossbar A Select Register 23 */
/*! @{ */

#define XBARA_SEL23_SEL46_MASK  (0x3FU)
#define XBARA_SEL23_SEL46_SHIFT (0U)
#define XBARA_SEL23_SEL46(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL23_SEL46_SHIFT)) & XBARA_SEL23_SEL46_MASK)

#define XBARA_SEL23_SEL47_MASK  (0x3F00U)
#define XBARA_SEL23_SEL47_SHIFT (8U)
#define XBARA_SEL23_SEL47(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL23_SEL47_SHIFT)) & XBARA_SEL23_SEL47_MASK)
/*! @} */

/*! @name SEL24 - Crossbar A Select Register 24 */
/*! @{ */

#define XBARA_SEL24_SEL48_MASK  (0x3FU)
#define XBARA_SEL24_SEL48_SHIFT (0U)
#define XBARA_SEL24_SEL48(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL24_SEL48_SHIFT)) & XBARA_SEL24_SEL48_MASK)

#define XBARA_SEL24_SEL49_MASK  (0x3F00U)
#define XBARA_SEL24_SEL49_SHIFT (8U)
#define XBARA_SEL24_SEL49(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL24_SEL49_SHIFT)) & XBARA_SEL24_SEL49_MASK)
/*! @} */

/*! @name SEL25 - Crossbar A Select Register 25 */
/*! @{ */

#define XBARA_SEL25_SEL50_MASK  (0x3FU)
#define XBARA_SEL25_SEL50_SHIFT (0U)
#define XBARA_SEL25_SEL50(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL25_SEL50_SHIFT)) & XBARA_SEL25_SEL50_MASK)

#define XBARA_SEL25_SEL51_MASK  (0x3F00U)
#define XBARA_SEL25_SEL51_SHIFT (8U)
#define XBARA_SEL25_SEL51(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL25_SEL51_SHIFT)) & XBARA_SEL25_SEL51_MASK)
/*! @} */

/*! @name SEL26 - Crossbar A Select Register 26 */
/*! @{ */

#define XBARA_SEL26_SEL52_MASK  (0x3FU)
#define XBARA_SEL26_SEL52_SHIFT (0U)
#define XBARA_SEL26_SEL52(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL26_SEL52_SHIFT)) & XBARA_SEL26_SEL52_MASK)

#define XBARA_SEL26_SEL53_MASK  (0x3F00U)
#define XBARA_SEL26_SEL53_SHIFT (8U)
#define XBARA_SEL26_SEL53(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL26_SEL53_SHIFT)) & XBARA_SEL26_SEL53_MASK)
/*! @} */

/*! @name SEL27 - Crossbar A Select Register 27 */
/*! @{ */

#define XBARA_SEL27_SEL54_MASK  (0x3FU)
#define XBARA_SEL27_SEL54_SHIFT (0U)
#define XBARA_SEL27_SEL54(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL27_SEL54_SHIFT)) & XBARA_SEL27_SEL54_MASK)

#define XBARA_SEL27_SEL55_MASK  (0x3F00U)
#define XBARA_SEL27_SEL55_SHIFT (8U)
#define XBARA_SEL27_SEL55(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL27_SEL55_SHIFT)) & XBARA_SEL27_SEL55_MASK)
/*! @} */

/*! @name SEL28 - Crossbar A Select Register 28 */
/*! @{ */

#define XBARA_SEL28_SEL56_MASK  (0x3FU)
#define XBARA_SEL28_SEL56_SHIFT (0U)
#define XBARA_SEL28_SEL56(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL28_SEL56_SHIFT)) & XBARA_SEL28_SEL56_MASK)

#define XBARA_SEL28_SEL57_MASK  (0x3F00U)
#define XBARA_SEL28_SEL57_SHIFT (8U)
#define XBARA_SEL28_SEL57(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL28_SEL57_SHIFT)) & XBARA_SEL28_SEL57_MASK)
/*! @} */

/*! @name SEL29 - Crossbar A Select Register 29 */
/*! @{ */

#define XBARA_SEL29_SEL58_MASK  (0x3FU)
#define XBARA_SEL29_SEL58_SHIFT (0U)
#define XBARA_SEL29_SEL58(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL29_SEL58_SHIFT)) & XBARA_SEL29_SEL58_MASK)

#define XBARA_SEL29_SEL59_MASK  (0x3F00U)
#define XBARA_SEL29_SEL59_SHIFT (8U)
#define XBARA_SEL29_SEL59(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL29_SEL59_SHIFT)) & XBARA_SEL29_SEL59_MASK)
/*! @} */

/*! @name SEL30 - Crossbar A Select Register 30 */
/*! @{ */

#define XBARA_SEL30_SEL60_MASK  (0x3FU)
#define XBARA_SEL30_SEL60_SHIFT (0U)
#define XBARA_SEL30_SEL60(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL30_SEL60_SHIFT)) & XBARA_SEL30_SEL60_MASK)

#define XBARA_SEL30_SEL61_MASK  (0x3F00U)
#define XBARA_SEL30_SEL61_SHIFT (8U)
#define XBARA_SEL30_SEL61(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL30_SEL61_SHIFT)) & XBARA_SEL30_SEL61_MASK)
/*! @} */

/*! @name SEL31 - Crossbar A Select Register 31 */
/*! @{ */

#define XBARA_SEL31_SEL62_MASK  (0x3FU)
#define XBARA_SEL31_SEL62_SHIFT (0U)
#define XBARA_SEL31_SEL62(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL31_SEL62_SHIFT)) & XBARA_SEL31_SEL62_MASK)

#define XBARA_SEL31_SEL63_MASK  (0x3F00U)
#define XBARA_SEL31_SEL63_SHIFT (8U)
#define XBARA_SEL31_SEL63(x)    (((uint16_t)(((uint16_t)(x)) << XBARA_SEL31_SEL63_SHIFT)) & XBARA_SEL31_SEL63_MASK)
/*! @} */

/*! @name CTRL0 - Crossbar A Control Register 0 */
/*! @{ */

#define XBARA_CTRL0_DEN0_MASK  (0x1U)
#define XBARA_CTRL0_DEN0_SHIFT (0U)
/*! DEN0 - DMA Enable for XBAR_OUT0
 *  0b0..DMA disabled
 *  0b1..DMA enabled
 */
#define XBARA_CTRL0_DEN0(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_DEN0_SHIFT)) & XBARA_CTRL0_DEN0_MASK)

#define XBARA_CTRL0_IEN0_MASK  (0x2U)
#define XBARA_CTRL0_IEN0_SHIFT (1U)
/*! IEN0 - Interrupt Enable for XBAR_OUT0
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define XBARA_CTRL0_IEN0(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_IEN0_SHIFT)) & XBARA_CTRL0_IEN0_MASK)

#define XBARA_CTRL0_EDGE0_MASK  (0xCU)
#define XBARA_CTRL0_EDGE0_SHIFT (2U)
/*! EDGE0 - Active edge for edge detection on XBAR_OUT0
 *  0b00..STS0 never asserts
 *  0b01..STS0 asserts on rising edges of XBAR_OUT0
 *  0b10..STS0 asserts on falling edges of XBAR_OUT0
 *  0b11..STS0 asserts on rising and falling edges of XBAR_OUT0
 */
#define XBARA_CTRL0_EDGE0(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_EDGE0_SHIFT)) & XBARA_CTRL0_EDGE0_MASK)

#define XBARA_CTRL0_STS0_MASK  (0x10U)
#define XBARA_CTRL0_STS0_SHIFT (4U)
/*! STS0 - Edge detection status for XBAR_OUT0
 *  0b0..Active edge not yet detected on XBAR_OUT0
 *  0b1..Active edge detected on XBAR_OUT0
 */
#define XBARA_CTRL0_STS0(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_STS0_SHIFT)) & XBARA_CTRL0_STS0_MASK)

#define XBARA_CTRL0_DEN1_MASK  (0x100U)
#define XBARA_CTRL0_DEN1_SHIFT (8U)
/*! DEN1 - DMA Enable for XBAR_OUT1
 *  0b0..DMA disabled
 *  0b1..DMA enabled
 */
#define XBARA_CTRL0_DEN1(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_DEN1_SHIFT)) & XBARA_CTRL0_DEN1_MASK)

#define XBARA_CTRL0_IEN1_MASK  (0x200U)
#define XBARA_CTRL0_IEN1_SHIFT (9U)
/*! IEN1 - Interrupt Enable for XBAR_OUT1
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define XBARA_CTRL0_IEN1(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_IEN1_SHIFT)) & XBARA_CTRL0_IEN1_MASK)

#define XBARA_CTRL0_EDGE1_MASK  (0xC00U)
#define XBARA_CTRL0_EDGE1_SHIFT (10U)
/*! EDGE1 - Active edge for edge detection on XBAR_OUT1
 *  0b00..STS1 never asserts
 *  0b01..STS1 asserts on rising edges of XBAR_OUT1
 *  0b10..STS1 asserts on falling edges of XBAR_OUT1
 *  0b11..STS1 asserts on rising and falling edges of XBAR_OUT1
 */
#define XBARA_CTRL0_EDGE1(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_EDGE1_SHIFT)) & XBARA_CTRL0_EDGE1_MASK)

#define XBARA_CTRL0_STS1_MASK  (0x1000U)
#define XBARA_CTRL0_STS1_SHIFT (12U)
/*! STS1 - Edge detection status for XBAR_OUT1
 *  0b0..Active edge not yet detected on XBAR_OUT1
 *  0b1..Active edge detected on XBAR_OUT1
 */
#define XBARA_CTRL0_STS1(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_STS1_SHIFT)) & XBARA_CTRL0_STS1_MASK)
/*! @} */

/*! @name CTRL1 - Crossbar A Control Register 1 */
/*! @{ */

#define XBARA_CTRL1_DEN2_MASK  (0x1U)
#define XBARA_CTRL1_DEN2_SHIFT (0U)
/*! DEN2 - DMA Enable for XBAR_OUT2
 *  0b0..DMA disabled
 *  0b1..DMA enabled
 */
#define XBARA_CTRL1_DEN2(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_DEN2_SHIFT)) & XBARA_CTRL1_DEN2_MASK)

#define XBARA_CTRL1_IEN2_MASK  (0x2U)
#define XBARA_CTRL1_IEN2_SHIFT (1U)
/*! IEN2 - Interrupt Enable for XBAR_OUT2
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define XBARA_CTRL1_IEN2(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_IEN2_SHIFT)) & XBARA_CTRL1_IEN2_MASK)

#define XBARA_CTRL1_EDGE2_MASK  (0xCU)
#define XBARA_CTRL1_EDGE2_SHIFT (2U)
/*! EDGE2 - Active edge for edge detection on XBAR_OUT2
 *  0b00..STS2 never asserts
 *  0b01..STS2 asserts on rising edges of XBAR_OUT2
 *  0b10..STS2 asserts on falling edges of XBAR_OUT2
 *  0b11..STS2 asserts on rising and falling edges of XBAR_OUT2
 */
#define XBARA_CTRL1_EDGE2(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_EDGE2_SHIFT)) & XBARA_CTRL1_EDGE2_MASK)

#define XBARA_CTRL1_STS2_MASK  (0x10U)
#define XBARA_CTRL1_STS2_SHIFT (4U)
/*! STS2 - Edge detection status for XBAR_OUT2
 *  0b0..Active edge not yet detected on XBAR_OUT2
 *  0b1..Active edge detected on XBAR_OUT2
 */
#define XBARA_CTRL1_STS2(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_STS2_SHIFT)) & XBARA_CTRL1_STS2_MASK)

#define XBARA_CTRL1_DEN3_MASK  (0x100U)
#define XBARA_CTRL1_DEN3_SHIFT (8U)
/*! DEN3 - DMA Enable for XBAR_OUT3
 *  0b0..DMA disabled
 *  0b1..DMA enabled
 */
#define XBARA_CTRL1_DEN3(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_DEN3_SHIFT)) & XBARA_CTRL1_DEN3_MASK)

#define XBARA_CTRL1_IEN3_MASK  (0x200U)
#define XBARA_CTRL1_IEN3_SHIFT (9U)
/*! IEN3 - Interrupt Enable for XBAR_OUT3
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define XBARA_CTRL1_IEN3(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_IEN3_SHIFT)) & XBARA_CTRL1_IEN3_MASK)

#define XBARA_CTRL1_EDGE3_MASK  (0xC00U)
#define XBARA_CTRL1_EDGE3_SHIFT (10U)
/*! EDGE3 - Active edge for edge detection on XBAR_OUT3
 *  0b00..STS3 never asserts
 *  0b01..STS3 asserts on rising edges of XBAR_OUT3
 *  0b10..STS3 asserts on falling edges of XBAR_OUT3
 *  0b11..STS3 asserts on rising and falling edges of XBAR_OUT3
 */
#define XBARA_CTRL1_EDGE3(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_EDGE3_SHIFT)) & XBARA_CTRL1_EDGE3_MASK)

#define XBARA_CTRL1_STS3_MASK  (0x1000U)
#define XBARA_CTRL1_STS3_SHIFT (12U)
/*! STS3 - Edge detection status for XBAR_OUT3
 *  0b0..Active edge not yet detected on XBAR_OUT3
 *  0b1..Active edge detected on XBAR_OUT3
 */
#define XBARA_CTRL1_STS3(x) (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_STS3_SHIFT)) & XBARA_CTRL1_STS3_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group XBARA_Register_Masks */

/* XBARA - Peripheral instance base addresses */
/** Peripheral XBARA base address */
#define XBARA_BASE (0xE340u)
/** Peripheral XBARA base pointer */
#define XBARA ((XBARA_Type *)XBARA_BASE)
/** Array initializer of XBARA peripheral base addresses */
#define XBARA_BASE_ADDRS \
    {                    \
        XBARA_BASE       \
    }
/** Array initializer of XBARA peripheral base pointers */
#define XBARA_BASE_PTRS \
    {                   \
        XBARA           \
    }

/*!
 * @}
 */ /* end of group XBARA_Peripheral_Access_Layer */

/*
** End of section using anonymous unions
*/

#if defined(__CWCC__)
#pragma pop
#else
#error Not supported compiler type
#endif

/*!
 * @}
 */ /* end of group Peripheral_access_layer */

/* ----------------------------------------------------------------------------
   -- Macros for use with bit field definitions (xxx_SHIFT, xxx_MASK).
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Bit_Field_Generic_Macros Macros for use with bit field definitions (xxx_SHIFT, xxx_MASK).
 * @{
 */

/**
 * @brief Mask and left-shift a bit field value for use in a register bit range.
 * @param field Name of the register bit field.
 * @param value Value of the bit field.
 * @return Masked and shifted value.
 */
#define NXP_VAL2FLD(field, value) (((value) << (field##_SHIFT)) & (field##_MASK))
/**
 * @brief Mask and right-shift a register value to extract a bit field value.
 * @param field Name of the register bit field.
 * @param value Value of the register.
 * @return Masked and shifted bit field value.
 */
#define NXP_FLD2VAL(field, value) (((value) & (field##_MASK)) >> (field##_SHIFT))

/*!
 * @}
 */ /* end of group Bit_Field_Generic_Macros */

/* ----------------------------------------------------------------------------
   -- SDK Compatibility
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDK_Compatibility_Symbols SDK Compatibility
 * @{
 */

/* No SDK compatibility issues. */

/*!
 * @}
 */ /* end of group SDK_Compatibility_Symbols */

#endif /* _MC56F83783_H_ */
