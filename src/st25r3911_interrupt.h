/******************************************************************************
  * \attention
  *
  * <h2><center>&copy; COPYRIGHT 2021 STMicroelectronics</center></h2>
  *
  * Licensed under ST MIX MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        www.st.com/mix_myliberty
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*! \file
 *
 *  \author SRA
 *
 *  \brief ST25R3911 Interrupt header file
 *
 *
 * \addtogroup RFAL
 * @{
 *
 * \addtogroup RFAL-HAL
 * \brief RFAL Hardware Abstraction Layer
 * @{
 *
 * \addtogroup ST25R3911
 * \brief RFAL ST25R3911 Driver
 * @{
 * 
 * \addtogroup ST25R3911_Interrupt
 * \brief RFAL ST25R3911 Interrupt
 * @{
 *
 */

#ifndef ST25R3911_INTERRUPT_H
#define ST25R3911_INTERRUPT_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/


/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

/* Main interrupt register. */
#define ST25R3911_IRQ_MASK_ALL             (uint32_t)(0xFFFFFFU) /*!< All ST25R3911 interrupt sources                              */
#define ST25R3911_IRQ_MASK_NONE            (uint32_t)(0U)        /*!< No ST25R3911 interrupt source                                */
#define ST25R3911_IRQ_MASK_OSC             (uint32_t)(0x80U)     /*!< ST25R3911 oscillator stable interrupt                        */
#define ST25R3911_IRQ_MASK_FWL             (uint32_t)(0x40U)     /*!< ST25R3911 FIFO water level interrupt                         */
#define ST25R3911_IRQ_MASK_RXS             (uint32_t)(0x20U)     /*!< ST25R3911 start of receive interrupt                         */
#define ST25R3911_IRQ_MASK_RXE             (uint32_t)(0x10U)     /*!< ST25R3911 end of receive interrupt                           */
#define ST25R3911_IRQ_MASK_TXE             (uint32_t)(0x08U)     /*!< ST25R3911 end of transmission interrupt                      */
#define ST25R3911_IRQ_MASK_COL             (uint32_t)(0x04U)     /*!< ST25R3911 bit collision interrupt                            */

/* Timer and NFC interrupt register. */
#define ST25R3911_IRQ_MASK_DCT             (uint32_t)(0x8000U)   /*!< ST25R3911 termination of direct command interrupt            */
#define ST25R3911_IRQ_MASK_NRE             (uint32_t)(0x4000U)   /*!< ST25R3911 no-response timer expired interrupt                */
#define ST25R3911_IRQ_MASK_GPE             (uint32_t)(0x2000U)   /*!< ST25R3911 general purpose timer expired interrupt            */
#define ST25R3911_IRQ_MASK_EON             (uint32_t)(0x1000U)   /*!< ST25R3911 external field on interrupt                        */
#define ST25R3911_IRQ_MASK_EOF             (uint32_t)(0x0800U)   /*!< ST25R3911 external field off interrupt                       */
#define ST25R3911_IRQ_MASK_CAC             (uint32_t)(0x0400U)   /*!< ST25R3911 collision during RF collision avoidance interrupt  */
#define ST25R3911_IRQ_MASK_CAT             (uint32_t)(0x0200U)   /*!< ST25R3911 minimum guard time expired interrupt               */
#define ST25R3911_IRQ_MASK_NFCT            (uint32_t)(0x0100U)   /*!< ST25R3911 initiator bit rate recognized interrupt            */

/* Error and wake-up interrupt register. */
#define ST25R3911_IRQ_MASK_CRC             (uint32_t)(0x800000U) /*!< ST25R3911 CRC error interrupt                                */
#define ST25R3911_IRQ_MASK_PAR             (uint32_t)(0x400000U) /*!< ST25R3911 parity error interrupt                             */
#define ST25R3911_IRQ_MASK_ERR2            (uint32_t)(0x200000U) /*!< ST25R3911 soft framing error interrupt                       */
#define ST25R3911_IRQ_MASK_ERR1            (uint32_t)(0x100000U) /*!< ST25R3911 hard framing error interrupt                       */
#define ST25R3911_IRQ_MASK_WT              (uint32_t)(0x080000U) /*!< ST25R3911 wake-up interrupt                                  */
#define ST25R3911_IRQ_MASK_WAM             (uint32_t)(0x040000U) /*!< ST25R3911 wake-up due to amplitude interrupt                 */
#define ST25R3911_IRQ_MASK_WPH             (uint32_t)(0x020000U) /*!< ST25R3911 wake-up due to phase interrupt                     */
#define ST25R3911_IRQ_MASK_WCAP            (uint32_t)(0x010000U) /*!< ST25R3911 wake-up due to capacitance measurement             */


#define ST25R3911_IRQ_MASK_TIM             (0x02U)               /*!< additional interrupts in ST25R3911_REG_IRQ_TIMER_NFC         */
#define ST25R3911_IRQ_MASK_ERR             (0x01U)               /*!< additional interrupts in ST25R3911_REG_IRQ_ERROR_WUP         */


/*! Holds current and previous interrupt callback pointer as well as current Interrupt status and mask */
typedef struct
{
    void      (*prevCallback)(void); /*!< call back function for 3911 interrupt               */
    void      (*callback)(void);     /*!< call back function for 3911 interrupt               */
    uint32_t  status;                /*!< latest interrupt status                             */
    uint32_t  mask;                  /*!< Interrupt mask. Negative mask = ST25R3911 mask regs */
}t_st25r3911Interrupt;

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/


#endif /* ST25R3911_ISR_H */

/**
  * @}
  *
  * @}
  *
  * @}
  * 
  * @}
  */
