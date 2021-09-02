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
 *  \brief Implementation of ST25R3911 communication.
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "rfal_rfst25r3911.h"
#include "st25r3911_com.h"
#include "st25r3911.h"
#include "nfc_utils.h"


/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/

#define ST25R3911_WRITE_MODE  (0U)                           /*!< ST25R3911 SPI Operation Mode: Write                            */
#define ST25R3911_READ_MODE   (1U << 6)                      /*!< ST25R3911 SPI Operation Mode: Read                             */
#define ST25R3911_FIFO_LOAD   (2U << 6)                      /*!< ST25R3911 SPI Operation Mode: FIFO Load                        */
#define ST25R3911_FIFO_READ   (0xBFU)                        /*!< ST25R3911 SPI Operation Mode: FIFO Read                        */
#define ST25R3911_CMD_MODE    (3U << 6)                      /*!< ST25R3911 SPI Operation Mode: Direct Command                   */

#define ST25R3911_CMD_LEN     (1U)                           /*!< ST25R3911 CMD length                                           */
#define ST25R3911_BUF_LEN     (ST25R3911_CMD_LEN+ST25R3911_FIFO_DEPTH)  /*!< ST25R3911 communication buffer: CMD + FIFO length   */

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/


/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
void RfalRfST25R3911BClass::st25r3911ReadRegister(uint8_t reg, uint8_t* value)
{ 
    uint8_t  buf[2];
  
    buf[0] = (reg | ST25R3911_READ_MODE);
    buf[1] = 0;

    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));

    digitalWrite(cs_pin, LOW);
  
    dev_spi->transfer((void *)buf, 2);

    digitalWrite(cs_pin, HIGH);
    dev_spi->endTransaction();
  
    if(value != NULL)
    {
      *value = buf[1];
    }

    return;
}


void RfalRfST25R3911BClass::st25r3911ReadMultipleRegisters(uint8_t reg, uint8_t* values, uint8_t length)
{ 
    if (length > 0U)
    {
        /* Setting Transaction Parameters */
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
        digitalWrite(cs_pin, LOW);

        /* Since the result comes one byte later, let's first transmit the adddress with discarding the result */
        dev_spi->transfer((reg | ST25R3911_READ_MODE));

        dev_spi->transfer((void *)values, length);

        digitalWrite(cs_pin, HIGH);
        dev_spi->endTransaction();
    }
    
    return;
}

void RfalRfST25R3911BClass::st25r3911ReadTestRegister(uint8_t reg, uint8_t* value)
{
    uint8_t  buf[3];

    buf[0] = ST25R3911_CMD_TEST_ACCESS;
    buf[1] = (reg | ST25R3911_READ_MODE);
    buf[2] = 0x00;

    /* Setting Transaction Parameters */
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
    digitalWrite(cs_pin, LOW);
  
    dev_spi->transfer((void *)buf, 3);
    
    if(value != NULL)
    {
      *value = buf[2];
    }
    
    digitalWrite(cs_pin, HIGH);

    dev_spi->endTransaction();

    return;
}

void RfalRfST25R3911BClass::st25r3911WriteTestRegister(uint8_t reg, uint8_t value)
{
    uint8_t  buf[3];

    buf[0] = ST25R3911_CMD_TEST_ACCESS;
    buf[1] = (reg | ST25R3911_WRITE_MODE);
    buf[2] = value;
  
    /* Setting Transaction Parameters */
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
    digitalWrite(cs_pin, LOW);
  
    dev_spi->transfer((void *)buf, 3);

    digitalWrite(cs_pin, HIGH);

    dev_spi->endTransaction();

    return;
}

void RfalRfST25R3911BClass::st25r3911WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t buf[2];

    buf[0] = reg | ST25R3911_WRITE_MODE;
    buf[1] = value;

    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));

    digitalWrite(cs_pin, LOW);
  
    dev_spi->transfer((void *)buf, 2);

    digitalWrite(cs_pin, HIGH);
    dev_spi->endTransaction();

    return;
}

void RfalRfST25R3911BClass::st25r3911ClrRegisterBits( uint8_t reg, uint8_t clr_mask )
{
    uint8_t tmp;

    st25r3911ReadRegister(reg, &tmp);
    tmp &= ~clr_mask;
    st25r3911WriteRegister(reg, tmp);
    
    return;
}


void RfalRfST25R3911BClass::st25r3911SetRegisterBits( uint8_t reg, uint8_t set_mask )
{
    uint8_t tmp;

    st25r3911ReadRegister(reg, &tmp);
    tmp |= set_mask;
    st25r3911WriteRegister(reg, tmp);
    
    return;
}

void RfalRfST25R3911BClass::st25r3911ChangeRegisterBits(uint8_t reg, uint8_t valueMask, uint8_t value)
{
    st25r3911ModifyRegister(reg, valueMask, (valueMask & value) );
}

void RfalRfST25R3911BClass::st25r3911ModifyRegister(uint8_t reg, uint8_t clr_mask, uint8_t set_mask)
{
    uint8_t tmp;

    st25r3911ReadRegister(reg, &tmp);

    /* mask out the bits we don't want to change */
    tmp &= ~clr_mask;
    /* set the new value */
    tmp |= set_mask;
    st25r3911WriteRegister(reg, tmp);

    return;
}

void RfalRfST25R3911BClass::st25r3911ChangeTestRegisterBits( uint8_t reg, uint8_t valueMask, uint8_t value )
{
    uint8_t    rdVal;
    uint8_t    wrVal;
    
    /* Read current reg value */
    st25r3911ReadTestRegister(reg, &rdVal);
    
    /* Compute new value */
    wrVal  = (rdVal & ~valueMask);
    wrVal |= (value & valueMask);
    
    /* Write new reg value */
    st25r3911WriteTestRegister(reg, wrVal );
    
    return;
}

void RfalRfST25R3911BClass::st25r3911WriteMultipleRegisters(uint8_t reg, const uint8_t* values, uint8_t length)
{
    if (length > 0U)
    {
        /* Setting Transaction Parameters */
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
        digitalWrite(cs_pin, LOW);

        /* Since the result comes one byte later, let's first transmit the adddress with discarding the result */
        dev_spi->transfer((reg | ST25R3911_WRITE_MODE));

        dev_spi->transfer((void *)values, length);

        digitalWrite(cs_pin, HIGH);
        dev_spi->endTransaction();
    }
    
    return;
}


void RfalRfST25R3911BClass::st25r3911WriteFifo(const uint8_t* values, uint8_t length)
{
    if (length > 0U)
    {
        /* Setting Transaction Parameters */
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
        digitalWrite(cs_pin, LOW);

        /* Since the result comes one byte later, let's first transmit the adddress with discarding the result */
        dev_spi->transfer(ST25R3911_FIFO_LOAD);

        dev_spi->transfer((void *)values, length);

        digitalWrite(cs_pin, HIGH);
        dev_spi->endTransaction();
    }

    return;
}

void RfalRfST25R3911BClass::st25r3911ReadFifo(uint8_t* buf, uint8_t length)
{
    if(length > 0U)
    {
        /* Setting Transaction Parameters */
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
        digitalWrite(cs_pin, LOW);

        /* Since the result comes one byte later, let's first transmit the adddress with discarding the result */
        dev_spi->transfer(ST25R3911_FIFO_READ);

        dev_spi->transfer((void *)buf, length);

        digitalWrite(cs_pin, HIGH);
        dev_spi->endTransaction();
    }

    return;
}

void RfalRfST25R3911BClass::st25r3911ExecuteCommand( uint8_t cmd )
{
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));

    digitalWrite(cs_pin, LOW);
  
    dev_spi->transfer((cmd | ST25R3911_CMD_MODE));

    digitalWrite(cs_pin, HIGH);
    dev_spi->endTransaction();

    return;
}


void RfalRfST25R3911BClass::st25r3911ExecuteCommands(const uint8_t *cmds, uint8_t length)
{
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));

    digitalWrite(cs_pin, LOW);
  
    dev_spi->transfer((void *)cmds, length);

    digitalWrite(cs_pin, HIGH);
    dev_spi->endTransaction();

    return;
}

bool RfalRfST25R3911BClass::st25r3911IsRegValid( uint8_t reg )
{
    if( !(( (int16_t)reg >= (int16_t)ST25R3911_REG_IO_CONF1) && (reg <= ST25R3911_REG_CAPACITANCE_MEASURE_RESULT)) &&  (reg != ST25R3911_REG_IC_IDENTITY)  )
    {
        return false;
    }
    return true;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

