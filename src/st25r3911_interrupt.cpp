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
 *  \brief ST25R3911 Interrupt handling
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "rfal_rfst25r3911.h"
#include "st25r3911_interrupt.h"
#include "st25r3911_com.h"
#include "st25r3911.h"
#include "st_errno.h"
#include "nfc_utils.h"

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

/*! Length of the interrupt registers       */
#define ST25R3911_INT_REGS_LEN          ( (ST25R3911_REG_IRQ_ERROR_WUP - ST25R3911_REG_IRQ_MAIN) + 1U )

/*
 ******************************************************************************
 * LOCAL DATA TYPES
 ******************************************************************************
 */


/*
******************************************************************************
* GLOBAL VARIABLES
******************************************************************************
*/


/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
void RfalRfST25R3911BClass::st25r3911InitInterrupts( void )
{   
    st25r3911interrupt.callback     = NULL;
    st25r3911interrupt.prevCallback = NULL;
    st25r3911interrupt.status       = ST25R3911_IRQ_MASK_NONE;
    st25r3911interrupt.mask         = ST25R3911_IRQ_MASK_NONE;
}

void RfalRfST25R3911BClass::st25r3911Isr( void )
{
    st25r3911CheckForReceivedInterrupts();
    
    if (NULL != st25r3911interrupt.callback)
    {
        st25r3911interrupt.callback();
    }
}

void RfalRfST25R3911BClass::st25r3911CheckForReceivedInterrupts( void )
{
    uint8_t  iregs[ST25R3911_INT_REGS_LEN];
    uint32_t irqStatus; 

    irqStatus = ST25R3911_IRQ_MASK_NONE;
    ST_MEMSET( iregs, (int32_t)(ST25R3911_IRQ_MASK_ALL & 0xFFU), ST25R3911_INT_REGS_LEN );  /* MISRA 10.3 */
        
    /* In case the IRQ is Edge (not Level) triggered read IRQs until done */
    while( digitalRead(int_pin) == HIGH )
    {
        st25r3911ReadMultipleRegisters(ST25R3911_REG_IRQ_MAIN, iregs, sizeof(iregs));
       
        irqStatus |= (uint32_t)iregs[0];
        irqStatus |= (uint32_t)iregs[1]<<8;
        irqStatus |= (uint32_t)iregs[2]<<16;
    }
    
    /* Forward all interrupts, even masked ones to application. */
    st25r3911interrupt.status |= irqStatus;
}


void RfalRfST25R3911BClass::st25r3911ModifyInterrupts(uint32_t clr_mask, uint32_t set_mask)
{
    uint8_t i;
    uint32_t old_mask;
    uint32_t new_mask;

    old_mask = st25r3911interrupt.mask;
    new_mask = (~old_mask & set_mask) | (old_mask & clr_mask);
    st25r3911interrupt.mask &= ~clr_mask;
    st25r3911interrupt.mask |= set_mask;
    for (i=0; i<3U ; i++)
    { 
        if (((new_mask >> (i*8U)) & 0xffU) == 0U) {
            continue;
        }
        st25r3911WriteRegister((ST25R3911_REG_IRQ_MASK_MAIN + i), (uint8_t)((st25r3911interrupt.mask>>(i*8U))&0xffU));
    }
    return;
}


uint32_t RfalRfST25R3911BClass::st25r3911WaitForInterruptsTimed(uint32_t mask, uint16_t tmo)
{
    uint32_t tmr;
    uint32_t status;
   
    tmr = timerCalculateTimer(tmo);
    do 
    {
        status = (st25r3911interrupt.status & mask);
    } while( ( !timerIsExpired( tmr ) || (tmo == 0U)) && (status == 0U) );

    status = st25r3911interrupt.status & mask;
    
    st25r3911interrupt.status &= ~status;
    
    return status;
}

uint32_t RfalRfST25R3911BClass::st25r3911GetInterrupt(uint32_t mask)
{
    uint32_t irqs;

    irqs = (st25r3911interrupt.status & mask);
    if (irqs != ST25R3911_IRQ_MASK_NONE)
    {
        st25r3911interrupt.status &= ~irqs;
    }
    return irqs;
}

void RfalRfST25R3911BClass::st25r3911EnableInterrupts(uint32_t mask)
{
    st25r3911ModifyInterrupts(mask,0);
}

void RfalRfST25R3911BClass::st25r3911DisableInterrupts(uint32_t mask)
{
    st25r3911ModifyInterrupts(0,mask);
}

void RfalRfST25R3911BClass::st25r3911ClearInterrupts( void )
{
    uint8_t iregs[3];

    st25r3911ReadMultipleRegisters(ST25R3911_REG_IRQ_MAIN, iregs, 3);

    st25r3911interrupt.status = 0;

    return;
}

void RfalRfST25R3911BClass::st25r3911IRQCallbackSet( void (*cb)(void) )
{
    st25r3911interrupt.prevCallback = st25r3911interrupt.callback;
    st25r3911interrupt.callback     = cb;
}

void RfalRfST25R3911BClass::st25r3911IRQCallbackRestore( void )
{
    st25r3911interrupt.callback     = st25r3911interrupt.prevCallback;
    st25r3911interrupt.prevCallback = NULL;
}

