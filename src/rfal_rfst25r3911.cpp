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
 *  \brief RF Abstraction Layer (RFAL)
 *  
 *  RFAL implementation for ST25R3911
 */


/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "rfal_rfst25r3911.h"

/*******************************************************************************/
RfalRfST25R3911BClass::RfalRfST25R3911BClass(SPIClass *spi, int cs_pin, int int_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), int_pin(int_pin), spi_speed(spi_speed)
{
  memset(&gRFAL, 0, sizeof(rfal));
  memset(&gRfalAnalogConfigMgmt, 0, sizeof(rfalAnalogConfigMgmt));
  memset(&iso15693PhyConfig, 0, sizeof(iso15693PhyConfig_t));
  st25r3911NoResponseTime_64fcs = 0;
  memset((void *)&st25r3911interrupt, 0, sizeof(t_st25r3911Interrupt));
  timerStopwatchTick = 0;
}


ReturnCode RfalRfST25R3911BClass::rfalInitialize( void )
{
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    rfalAnalogConfigInitialize();              /* Initialize RFAL's Analog Configs */

    st25r3911InitInterrupts();
    
    /* Initialize chip */
    st25r3911Initialize();
    
    /* Check expected chip: ST25R3911 */
    if( !st25r3911CheckChipID( NULL ) )
    {
        return ERR_HW_MISMATCH;
    }
    
    /* Disable any previous observation mode */
    rfalST25R3911ObsModeDisable();
    
    /*******************************************************************************/    
    /* Apply RF Chip generic initialization */
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_INIT) );

    /*******************************************************************************/
    /* Set FIFO Water Levels to be used */
    st25r3911ChangeRegisterBits( ST25R3911_REG_IO_CONF1, (ST25R3911_REG_IO_CONF1_fifo_lt | ST25R3911_REG_IO_CONF1_fifo_lr), (ST25R3911_REG_IO_CONF1_fifo_lt_32bytes | ST25R3911_REG_IO_CONF1_fifo_lr_64bytes) );
    
    /* Always have CRC in FIFO upon reception  */
    st25r3911SetRegisterBits( ST25R3911_REG_AUX, ST25R3911_REG_AUX_crc_2_fifo );
    
    /* Enable External Field Detector */
    st25r3911SetRegisterBits( ST25R3911_REG_AUX, ST25R3911_REG_AUX_en_fd );
    
    /* Clear FIFO status local copy */
    rfalFIFOStatusClear();
    
    /*******************************************************************************/
    gRFAL.state              = RFAL_STATE_INIT;
    gRFAL.mode               = RFAL_MODE_NONE;
    gRFAL.field              = false;
    
    /* Set RFAL default configs */
    gRFAL.conf.obsvModeTx    = RFAL_OBSMODE_DISABLE;
    gRFAL.conf.obsvModeRx    = RFAL_OBSMODE_DISABLE;
    gRFAL.conf.eHandling     = RFAL_ERRORHANDLING_NONE;
    
    /* Transceive set to IDLE */
    gRFAL.TxRx.lastState     = RFAL_TXRX_STATE_IDLE;
    gRFAL.TxRx.state         = RFAL_TXRX_STATE_IDLE;
    
    /* Disable all timings */
    gRFAL.timings.FDTListen  = RFAL_TIMING_NONE;
    gRFAL.timings.FDTPoll    = RFAL_TIMING_NONE;
    gRFAL.timings.GT         = RFAL_TIMING_NONE;
    
    gRFAL.tmr.GT             = RFAL_TIMING_NONE;
    
    gRFAL.callbacks.preTxRx  = NULL;
    gRFAL.callbacks.postTxRx = NULL;
  
    /* Initialize NFC-V Data */
    gRFAL.nfcvData.ignoreBits = 0;

    /* Initialize Wake-Up Mode */
    gRFAL.wum.state = RFAL_WUM_STATE_NOT_INIT;
    
    /*******************************************************************************/    
    /* Perform Automatic Calibration (if configured to do so).                     *
     * Registers set by rfalSetAnalogConfig will tell rfalCalibrate what to perform*/
    rfalCalibrate();
    
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalCalibrate( void )
{
    uint16_t resValue;
    
    /* Check if RFAL is not initialized */
    if( gRFAL.state == RFAL_STATE_IDLE )
    {
        return ERR_WRONG_STATE;
    }

    /*******************************************************************************/
    /* Perform ST25R3911 regulators and antenna calibration                        */
    /*******************************************************************************/
    
    /* Automatic regulator adjustment only performed if not set manually on Analog Configs */
    if( st25r3911CheckReg( ST25R3911_REG_REGULATOR_CONTROL, ST25R3911_REG_REGULATOR_CONTROL_reg_s, 0x00 ) )       
    {
        /* Adjust the regulators so that Antenna Calibrate has better Regulator values */
        st25r3911AdjustRegulators( &resValue );
    }
    
    /* Automatic Antenna calibration only performed if not set manually on Analog Configs */
    if( st25r3911CheckReg( ST25R3911_REG_ANT_CAL_CONTROL, ST25R3911_REG_ANT_CAL_CONTROL_trim_s, 0x00 ) )
    {
        st25r3911CalibrateAntenna( (uint8_t*) &resValue );
      
        /*******************************************************************************/
        /* REMARK: Silicon workaround ST25R3911 Errata #1.5                            */
        /* Always run the command Calibrate Antenna twice                              */
        st25r3911CalibrateAntenna( (uint8_t*) &resValue );                
        /*******************************************************************************/
        
    }
    else
    {
        /* If no antenna calibration is performed there is no need to perform second regulator adjustment again */
        return ERR_NONE; 
    }
    
    if( st25r3911CheckReg( ST25R3911_REG_REGULATOR_CONTROL, ST25R3911_REG_REGULATOR_CONTROL_reg_s, 0x00 ) )
    {
        /* Adjust the regulators again with the Antenna calibrated */
        st25r3911AdjustRegulators( &resValue );
    }
    
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalAdjustRegulators( uint16_t* result )
{
    /*******************************************************************************/
    /* Make use of the Automatic Adjust  */
    st25r3911ClrRegisterBits( ST25R3911_REG_REGULATOR_CONTROL, ST25R3911_REG_REGULATOR_CONTROL_reg_s );
    
    return st25r3911AdjustRegulators( result );
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalSetUpperLayerCallback( rfalUpperLayerCallback pFunc )
{
    st25r3911IRQCallbackSet( pFunc );
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalSetPreTxRxCallback( rfalPreTxRxCallback pFunc )
{
    gRFAL.callbacks.preTxRx = pFunc;
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalSetPostTxRxCallback( rfalPostTxRxCallback pFunc )
{
    gRFAL.callbacks.postTxRx = pFunc;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalDeinitialize( void )
{
    /* Deinitialize chip */
    st25r3911Deinitialize();
    
    /* Set Analog configurations for deinitialization */
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_DEINIT) );
 
    gRFAL.state = RFAL_STATE_IDLE;
    return ERR_NONE;
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalSetObsvMode( uint8_t txMode, uint8_t rxMode )
{
    gRFAL.conf.obsvModeTx = txMode;
    gRFAL.conf.obsvModeRx = rxMode;
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalGetObsvMode( uint8_t* txMode, uint8_t* rxMode )
{
    if(txMode != NULL)
    {
        *txMode = gRFAL.conf.obsvModeTx;
    }
    
    if(rxMode != NULL)
    {
        *rxMode = gRFAL.conf.obsvModeRx;
    }
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalDisableObsvMode( void )
{
    gRFAL.conf.obsvModeTx = RFAL_OBSMODE_DISABLE;
    gRFAL.conf.obsvModeRx = RFAL_OBSMODE_DISABLE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalSetMode( rfalMode mode, rfalBitRate txBR, rfalBitRate rxBR )
{

    /* Check if RFAL is not initialized */
    if( gRFAL.state == RFAL_STATE_IDLE )
    {
        return ERR_WRONG_STATE;
    }
    
    /* Check allowed bit rate value */
    if( (txBR == RFAL_BR_KEEP) || (rxBR == RFAL_BR_KEEP) )
    {
        return ERR_PARAM;
    }
   
    switch( mode )
    {
        /*******************************************************************************/
        case RFAL_MODE_POLL_NFCA:
            
            /* Disable wake up mode, if set */
            st25r3911ClrRegisterBits( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_wu );
            
            /* Enable ISO14443A mode */
            st25r3911WriteRegister(ST25R3911_REG_MODE, ST25R3911_REG_MODE_om_iso14443a);
            
            /* Set Analog configurations for this mode and bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX) );
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX) );
            break;
            
        /*******************************************************************************/
        case RFAL_MODE_POLL_NFCA_T1T:
            /* Disable wake up mode, if set */
            st25r3911ClrRegisterBits( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_wu );
            
            /* Enable Topaz mode */
            st25r3911WriteRegister( ST25R3911_REG_MODE, ST25R3911_REG_MODE_om_topaz );
            
            /* Set Analog configurations for this mode and bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX) );
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX) );
            break;
            
        /*******************************************************************************/
        case RFAL_MODE_POLL_NFCB:
            
            /* Disable wake up mode, if set */
            st25r3911ClrRegisterBits( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_wu );
            
            /* Enable ISO14443B mode */
            st25r3911WriteRegister(ST25R3911_REG_MODE, ST25R3911_REG_MODE_om_iso14443b);
            
            /* Set the EGT, SOF, EOF and EOF */
            st25r3911ChangeRegisterBits(  ST25R3911_REG_ISO14443B_1, 
                                      (ST25R3911_REG_ISO14443B_1_mask_egt | ST25R3911_REG_ISO14443B_1_mask_sof | ST25R3911_REG_ISO14443B_1_mask_eof), 
                                      ( (0U<<ST25R3911_REG_ISO14443B_1_shift_egt) | ST25R3911_REG_ISO14443B_1_sof_0_10etu | ST25R3911_REG_ISO14443B_1_sof_1_2etu) );
                        
            /* Set the minimum TR1, SOF, EOF and EOF12 */
            st25r3911ChangeRegisterBits( ST25R3911_REG_ISO14443B_2, 
                                      (ST25R3911_REG_ISO14443B_2_mask_tr1 | ST25R3911_REG_ISO14443B_2_no_sof | ST25R3911_REG_ISO14443B_2_no_eof |ST25R3911_REG_ISO14443B_2_eof_12),
                                      (ST25R3911_REG_ISO14443B_2_tr1_80fs80fs | ST25R3911_REG_ISO14443B_2_eof_12_10to11etu ) );


            /* Set Analog configurations for this mode and bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX) );
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX) );
            break;
            
        /*******************************************************************************/
        case RFAL_MODE_POLL_B_PRIME:
            
            /* Disable wake up mode, if set */
            st25r3911ClrRegisterBits( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_wu );
            
            /* Enable ISO14443B mode */
            st25r3911WriteRegister(ST25R3911_REG_MODE, ST25R3911_REG_MODE_om_iso14443b);
            
            /* Set the EGT, SOF, EOF and EOF */
            st25r3911ChangeRegisterBits(  ST25R3911_REG_ISO14443B_1, 
                                      (ST25R3911_REG_ISO14443B_1_mask_egt | ST25R3911_REG_ISO14443B_1_mask_sof | ST25R3911_REG_ISO14443B_1_mask_eof), 
                                      ( (0U<<ST25R3911_REG_ISO14443B_1_shift_egt) | ST25R3911_REG_ISO14443B_1_sof_0_10etu | ST25R3911_REG_ISO14443B_1_sof_1_2etu) );
                        
            /* Set the minimum TR1, EOF and EOF12 */
            st25r3911ChangeRegisterBits( ST25R3911_REG_ISO14443B_2, 
                                      (ST25R3911_REG_ISO14443B_2_mask_tr1 | ST25R3911_REG_ISO14443B_2_no_sof | ST25R3911_REG_ISO14443B_2_no_eof |ST25R3911_REG_ISO14443B_2_eof_12),
                                      (ST25R3911_REG_ISO14443B_2_tr1_80fs80fs | ST25R3911_REG_ISO14443B_2_no_sof | ST25R3911_REG_ISO14443B_2_eof_12_10to12etu ) );


            /* Set Analog configurations for this mode and bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX) );
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX) );
            break;
            
        /*******************************************************************************/
        case RFAL_MODE_POLL_B_CTS:
            
            /* Disable wake up mode, if set */
            st25r3911ClrRegisterBits( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_wu );
            
            /* Enable ISO14443B mode */
            st25r3911WriteRegister(ST25R3911_REG_MODE, ST25R3911_REG_MODE_om_iso14443b);
            
            /* Set the EGT, SOF, EOF and EOF */
            st25r3911ChangeRegisterBits(  ST25R3911_REG_ISO14443B_1, 
                                      (ST25R3911_REG_ISO14443B_1_mask_egt | ST25R3911_REG_ISO14443B_1_mask_sof | ST25R3911_REG_ISO14443B_1_mask_eof), 
                                      ( (0U<<ST25R3911_REG_ISO14443B_1_shift_egt) | ST25R3911_REG_ISO14443B_1_sof_0_10etu | ST25R3911_REG_ISO14443B_1_sof_1_2etu) );
                        
            /* Set the minimum TR1, clear SOF, EOF and EOF12 */
            st25r3911ChangeRegisterBits( ST25R3911_REG_ISO14443B_2, 
                                      (ST25R3911_REG_ISO14443B_2_mask_tr1 | ST25R3911_REG_ISO14443B_2_no_sof | ST25R3911_REG_ISO14443B_2_no_eof |ST25R3911_REG_ISO14443B_2_eof_12),
                                      (ST25R3911_REG_ISO14443B_2_tr1_80fs80fs | ST25R3911_REG_ISO14443B_2_no_sof | ST25R3911_REG_ISO14443B_2_no_eof ) );


            /* Set Analog configurations for this mode and bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX) );
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX) );
            break;
            
        /*******************************************************************************/
        case RFAL_MODE_POLL_NFCF:
            
            /* Disable wake up mode, if set */
            st25r3911ClrRegisterBits( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_wu );
            
            /* Enable FeliCa mode */
            st25r3911WriteRegister( ST25R3911_REG_MODE, ST25R3911_REG_MODE_om_felica );
            
            /* Set Analog configurations for this mode and bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX) );
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX) );
            break;
        
        /*******************************************************************************/
        case RFAL_MODE_POLL_NFCV:
        case RFAL_MODE_POLL_PICOPASS:
        
            /* Disable wake up mode, if set */
            st25r3911ClrRegisterBits( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_wu );
            
            /* Set Analog configurations for this mode and bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX) );
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX) );
            break;

        /*******************************************************************************/
        case RFAL_MODE_POLL_ACTIVE_P2P:
            
            /* Set NFCIP1 active communication initiator mode and Enable NFC Automatic Response RF Collision Avoidance */
            st25r3911WriteRegister(ST25R3911_REG_MODE, (ST25R3911_REG_MODE_targ_init | ST25R3911_REG_MODE_om_nfc | ST25R3911_REG_MODE_nfc_ar) );
            
            /* Set GPT to start after end of TX, as GPT is used in active communication mode to timeout the field switching off */
            /* The field is turned off 37.76us after the end of the transmission  Trfw                                          */
            st25r3911StartGPTimer_8fcs( (uint16_t)rfalConv1fcTo8fc( RFAL_AP2P_FIELDOFF_TRFW ), ST25R3911_REG_GPT_CONTROL_gptc_etx_nfc );
            
            /* Enable External Field Detector */
            st25r3911SetRegisterBits( ST25R3911_REG_AUX, ST25R3911_REG_AUX_en_fd );
            
            /* Set Analog configurations for this mode and bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX) );
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX) );
            break;
        
        /*******************************************************************************/
        case RFAL_MODE_LISTEN_ACTIVE_P2P:

            /* Set NFCIP1 active communication initiator mode and Enable NFC Automatic Response RF Collision Avoidance */
            st25r3911WriteRegister(ST25R3911_REG_MODE, (ST25R3911_REG_MODE_targ_targ | ST25R3911_REG_MODE_om_nfcip1_normal_mode | ST25R3911_REG_MODE_nfc_ar) );
            
            /* Set GPT to start after end of TX, as GPT is used in active communication mode to timeout the field switching off */
            /* The field is turned off 37.76us after the end of the transmission  Trfw                                          */
            st25r3911StartGPTimer_8fcs( (uint16_t)rfalConv1fcTo8fc( RFAL_AP2P_FIELDOFF_TRFW ), ST25R3911_REG_GPT_CONTROL_gptc_etx_nfc );
            
            /* Enable External Field Detector */
            st25r3911SetRegisterBits( ST25R3911_REG_AUX, ST25R3911_REG_AUX_en_fd );
            
            /* Set Analog configurations for this mode and bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX) );
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX) );
            break;
            
        /*******************************************************************************/
        case RFAL_MODE_LISTEN_NFCA:
        case RFAL_MODE_LISTEN_NFCB:
        case RFAL_MODE_LISTEN_NFCF:
            return ERR_NOTSUPP;
            
        /*******************************************************************************/
        default:
            return ERR_NOT_IMPLEMENTED;
    }
    
    /* Set state as STATE_MODE_SET only if not initialized yet (PSL) */
    gRFAL.state = ((gRFAL.state < RFAL_STATE_MODE_SET) ? RFAL_STATE_MODE_SET : gRFAL.state);
    gRFAL.mode  = mode;
    
    /* Apply the given bit rate */
    return rfalSetBitRate(txBR, rxBR);
}


/*******************************************************************************/
rfalMode RfalRfST25R3911BClass::rfalGetMode( void )
{
    return gRFAL.mode;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalSetBitRate( rfalBitRate txBR, rfalBitRate rxBR )
{
    ReturnCode ret;
    
    /* Check if RFAL is not initialized */
    if( gRFAL.state == RFAL_STATE_IDLE )
    {
        return ERR_WRONG_STATE;
    }
   
    /* Store the new Bit Rates */
    gRFAL.txBR = ((txBR == RFAL_BR_KEEP) ? gRFAL.txBR : txBR);
    gRFAL.rxBR = ((rxBR == RFAL_BR_KEEP) ? gRFAL.rxBR : rxBR);
    
    /* Update the bitrate reg if not in NFCV mode (streaming) */
    if( (RFAL_MODE_POLL_NFCV != gRFAL.mode) && (RFAL_MODE_POLL_PICOPASS != gRFAL.mode) )
    {
        EXIT_ON_ERR( ret, st25r3911SetBitrate( (uint8_t)gRFAL.txBR, (uint8_t)gRFAL.rxBR ) );
    }
    
    
    switch( gRFAL.mode )
    {
        /*******************************************************************************/
        case RFAL_MODE_POLL_NFCA:
        case RFAL_MODE_POLL_NFCA_T1T:
            
            /* Set Analog configurations for this bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_POLL_COMMON) );
            rfalSetAnalogConfig( (rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX ) );
            rfalSetAnalogConfig( (rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX ) );
            break;
            
        /*******************************************************************************/
        case RFAL_MODE_POLL_NFCB:
        case RFAL_MODE_POLL_B_PRIME:
        case RFAL_MODE_POLL_B_CTS:
            
            /* Set Analog configurations for this bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_POLL_COMMON) );
            rfalSetAnalogConfig( (rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX ) );
            rfalSetAnalogConfig( (rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX ) );
            break;
            
        /*******************************************************************************/
        case RFAL_MODE_POLL_NFCF:
            
            /* Set Analog configurations for this bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_POLL_COMMON) );
            rfalSetAnalogConfig( (rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX ) );
            rfalSetAnalogConfig( (rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX ) );
            break;
        
        /*******************************************************************************/
        case RFAL_MODE_POLL_NFCV:
        case RFAL_MODE_POLL_PICOPASS:
                if( ((gRFAL.rxBR != RFAL_BR_26p48) && (gRFAL.rxBR != RFAL_BR_52p97) && (gRFAL.rxBR != RFAL_BR_106) && (gRFAL.rxBR != RFAL_BR_212))
                        || ((gRFAL.txBR != RFAL_BR_1p66) && (gRFAL.txBR != RFAL_BR_26p48)) )
                {
                    return ERR_PARAM;
                }
        
                {
                    const struct iso15693StreamConfig *isoStreamConfig;
                    struct st25r3911StreamConfig      streamConf;
                    iso15693PhyConfig_t               config;
                    
                    /* Set the coding configuration for configuring ISO15693 */
                    config.coding     = (( gRFAL.txBR == RFAL_BR_1p66  ) ? ISO15693_VCD_CODING_1_256 : ISO15693_VCD_CODING_1_4);
                    switch (gRFAL.rxBR){
                        case RFAL_BR_52p97:
                            config.speedMode = 1;
                            break;
                        case RFAL_BR_106:
                            config.speedMode = 2;
                            break;
                        case RFAL_BR_212:
                            config.speedMode = 3;
                            break;
                        default:
                            config.speedMode = 0;
                            break;
                    }
                    
                    iso15693PhyConfigure(&config, &isoStreamConfig);   /* Convert ISO15693 config into StreamConfig */
                    
                    /* MISRA 11.3 - Cannot point directly into different object type, copy to local var */
                    streamConf.din                  = isoStreamConfig->din;
                    streamConf.dout                 = isoStreamConfig->dout;
                    streamConf.report_period_length = isoStreamConfig->report_period_length;
                    streamConf.useBPSK              = isoStreamConfig->useBPSK;
                    st25r3911StreamConfigure(&streamConf);
                }
    
                /* Set Analog configurations for this bit rate */
                rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_POLL_COMMON) );
                rfalSetAnalogConfig( (rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX ) );
                rfalSetAnalogConfig( (rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX ) );
                break;
                
        
        /*******************************************************************************/
        case RFAL_MODE_POLL_ACTIVE_P2P:
            
            /* Set Analog configurations for this bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_POLL_COMMON) );
            rfalSetAnalogConfig( (rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX ) );
            rfalSetAnalogConfig( (rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX ) );
            break;
        
        /*******************************************************************************/
        case RFAL_MODE_LISTEN_ACTIVE_P2P:
            
            /* Set Analog configurations for this bit rate */
            rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_LISTEN_COMMON) );
            rfalSetAnalogConfig( (rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | rfalConvBR2ACBR(gRFAL.txBR) | RFAL_ANALOG_CONFIG_TX ) );
            rfalSetAnalogConfig( (rfalAnalogConfigId)(RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | rfalConvBR2ACBR(gRFAL.rxBR) | RFAL_ANALOG_CONFIG_RX ) );
            break;
            
        /*******************************************************************************/
        case RFAL_MODE_LISTEN_NFCA:
        case RFAL_MODE_LISTEN_NFCB:
        case RFAL_MODE_LISTEN_NFCF:
        case RFAL_MODE_NONE:
            return ERR_WRONG_STATE;
            
        /*******************************************************************************/
        default:
            return ERR_NOT_IMPLEMENTED;
    }
    
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalGetBitRate( rfalBitRate *txBR, rfalBitRate *rxBR )
{
    if( (gRFAL.state == RFAL_STATE_IDLE) || (gRFAL.mode == RFAL_MODE_NONE) )
    {
        return ERR_WRONG_STATE;
    }
    
    if( txBR != NULL )
    {
        *txBR = gRFAL.txBR;
    }
    
    if( rxBR != NULL )
    {
        *rxBR = gRFAL.rxBR;
    }
    
    return ERR_NONE;
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalSetErrorHandling( rfalEHandling eHandling )
{
    gRFAL.conf.eHandling = eHandling;
}


/*******************************************************************************/
rfalEHandling RfalRfST25R3911BClass::rfalGetErrorHandling( void )
{
    return gRFAL.conf.eHandling;
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalSetFDTPoll( uint32_t FDTPoll )
{
    gRFAL.timings.FDTPoll = MIN( FDTPoll, RFAL_ST25R3911_GPT_MAX_1FC );
}


/*******************************************************************************/
uint32_t RfalRfST25R3911BClass::rfalGetFDTPoll( void )
{
    return gRFAL.timings.FDTPoll;
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalSetFDTListen( uint32_t FDTListen )
{
    gRFAL.timings.FDTListen = MIN( FDTListen, RFAL_ST25R3911_MRT_MAX_1FC);
}

/*******************************************************************************/
uint32_t RfalRfST25R3911BClass::rfalGetFDTListen( void )
{
    return gRFAL.timings.FDTListen;
}

void RfalRfST25R3911BClass::rfalSetGT( uint32_t GT )
{
    gRFAL.timings.GT = MIN( GT, RFAL_ST25R3911_GT_MAX_1FC );
}

/*******************************************************************************/
uint32_t RfalRfST25R3911BClass::rfalGetGT( void )
{
    return gRFAL.timings.GT;
}

/*******************************************************************************/
bool RfalRfST25R3911BClass::rfalIsGTExpired( void )
{
    if( gRFAL.tmr.GT != RFAL_TIMING_NONE )
    {
        if( !rfalTimerisExpired( gRFAL.tmr.GT ) )
        {
            return false;
        }
    }    
    return true;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalFieldOnAndStartGT( void )
{
    ReturnCode  ret;
    
    /* Check if RFAL has been initialized (Oscillator should be running) and also
     * if a direct register access has been performed and left the Oscillator Off */
    if( !st25r3911IsOscOn() || (gRFAL.state < RFAL_STATE_INIT) )
    {
        return ERR_WRONG_STATE;
    }
    
    ret = ERR_NONE;
    
    /* Set Analog configurations for Field On event */
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_FIELD_ON) );
    
    /*******************************************************************************/
    /* Perform collision avoidance and turn field On if not already On */
    if( !st25r3911IsTxEnabled() || !gRFAL.field )
    {
        /* Use Thresholds set by AnalogConfig */
        ret = st25r3911PerformCollisionAvoidance( ST25R3911_CMD_RESPONSE_RF_COLLISION_0, ST25R3911_THRESHOLD_DO_NOT_SET, ST25R3911_THRESHOLD_DO_NOT_SET, 0 );
        
        gRFAL.field = st25r3911IsTxEnabled();
        
        /* Only turn on Receiver and Transmitter if field was successfully turned On */
        if(gRFAL.field)
        {            
            st25r3911TxRxOn(); /* Enable Tx and Rx (Tx is already On) */
        }
    }
    
    /*******************************************************************************/
    /* Start GT timer in case the GT value is set */
    if( (gRFAL.timings.GT != RFAL_TIMING_NONE) )
    {
        /* Ensure that a SW timer doesn't have a lower value then the minimum  */
        rfalTimerStart( gRFAL.tmr.GT, rfalConv1fcToMs( MAX( (gRFAL.timings.GT), RFAL_ST25R3911_GT_MIN_1FC) ) );
    }
    
    return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalFieldOff( void )
{
    /* Check whether a TxRx is not yet finished */
    if( gRFAL.TxRx.state != RFAL_TXRX_STATE_IDLE )
    {
        rfalCleanupTransceive();
    }
    
    /* Disable Tx and Rx */
    st25r3911TxRxOff();
    
    /* Set Analog configurations for Field Off event */
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_FIELD_OFF) );
    gRFAL.field = false;
    
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalStartTransceive( const rfalTransceiveContext *ctx )
{
    uint32_t FxTAdj;  /* FWT or FDT adjustment calculation */
    
    /* Check for valid parameters */
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }
    
    /* Ensure that RFAL is already Initialized and the mode has been set */
    if( (gRFAL.state >= RFAL_STATE_MODE_SET) /*&& (gRFAL.TxRx.state == RFAL_TXRX_STATE_INIT )*/ )
    {
        /*******************************************************************************/
        /* Check whether the field is already On, otherwise no TXE will be received  */
        if( !st25r3911IsTxEnabled() && (!rfalIsModePassiveListen( gRFAL.mode ) && (ctx->txBuf != NULL)) )
        {
            return ERR_WRONG_STATE;
        }
        
        gRFAL.TxRx.ctx = *ctx;
        
        /*******************************************************************************/
        if( gRFAL.timings.FDTListen != RFAL_TIMING_NONE )
        {
            /* Calculate MRT adjustment accordingly to the current mode */
            FxTAdj = RFAL_FDT_LISTEN_MRT_ADJUSTMENT;
            if(gRFAL.mode == RFAL_MODE_POLL_NFCA)      { FxTAdj += (uint32_t)RFAL_FDT_LISTEN_A_ADJUSTMENT; }
            if(gRFAL.mode == RFAL_MODE_POLL_NFCA_T1T)  { FxTAdj += (uint32_t)RFAL_FDT_LISTEN_A_ADJUSTMENT; }
            if(gRFAL.mode == RFAL_MODE_POLL_NFCB)      { FxTAdj += (uint32_t)RFAL_FDT_LISTEN_B_ADJUSTMENT; }
            if(gRFAL.mode == RFAL_MODE_POLL_NFCV)      { FxTAdj += (uint32_t)RFAL_FDT_LISTEN_V_ADJUSTMENT; }
            
            
            /* Set Minimum FDT(Listen) in which PICC is not allowed to send a response */
            st25r3911WriteRegister( ST25R3911_REG_MASK_RX_TIMER, (uint8_t)rfalConv1fcTo64fc( (FxTAdj > gRFAL.timings.FDTListen) ? RFAL_ST25R3911_MRT_MIN_1FC : (gRFAL.timings.FDTListen - FxTAdj) ) );
        }
        
        /*******************************************************************************/
        /* FDT Poll will be loaded in rfalPrepareTransceive() once the previous was expired */
        
        /*******************************************************************************/
        if( rfalIsModePassiveComm( gRFAL.mode ) )  /* Passive Comms */
        {
            if( (gRFAL.TxRx.ctx.fwt != RFAL_FWT_NONE) && (gRFAL.TxRx.ctx.fwt != 0U) )
            {
                /* Ensure proper timing configuration */
                if( gRFAL.timings.FDTListen >= gRFAL.TxRx.ctx.fwt )
                {
                    return ERR_PARAM;
                }
        
                FxTAdj = RFAL_FWT_ADJUSTMENT;
                if(gRFAL.mode == RFAL_MODE_POLL_NFCA)      { FxTAdj += (uint32_t)RFAL_FWT_A_ADJUSTMENT;    }
                if(gRFAL.mode == RFAL_MODE_POLL_NFCA_T1T)  { FxTAdj += (uint32_t)RFAL_FWT_A_ADJUSTMENT;    }
                if(gRFAL.mode == RFAL_MODE_POLL_NFCB)      { FxTAdj += (uint32_t)RFAL_FWT_B_ADJUSTMENT;    }
                if(gRFAL.mode == RFAL_MODE_POLL_NFCF)      
                {
                    FxTAdj += (uint32_t)((gRFAL.txBR == RFAL_BR_212) ? RFAL_FWT_F_212_ADJUSTMENT : RFAL_FWT_F_424_ADJUSTMENT );
                }
                
                /* Ensure that the given FWT doesn't exceed NRT maximum */
                gRFAL.TxRx.ctx.fwt = MIN( (gRFAL.TxRx.ctx.fwt + FxTAdj), RFAL_ST25R3911_NRT_MAX_1FC );
                
                /* Set FWT in the NRT */
                st25r3911SetNoResponseTime_64fcs( rfalConv1fcTo64fc( gRFAL.TxRx.ctx.fwt ) );
            }
            else
            {
                /* Disable NRT, no NRE will be triggered, therefore wait endlessly for Rx */
                st25r3911SetNoResponseTime_64fcs( RFAL_ST25R3911_NRT_DISABLED );
            }
        }
        else /* Active Comms */
        {
            /* Setup NRT timer for rf response RF collision timeout. */
            st25r3911SetNoResponseTime_64fcs( rfalConv1fcTo64fc(RFAL_AP2P_FIELDON_TADTTRFW) );
            
            /* In Active Mode No Response Timer cannot be used to measure FWT a SW timer is used instead */
        }
        
        gRFAL.state       = RFAL_STATE_TXRX;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_IDLE;
        gRFAL.TxRx.status = ERR_BUSY;
        gRFAL.TxRx.rxse   = false;
      
        /*******************************************************************************/
        if( (RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode) )
        { /* Exchange receive buffer with internal buffer */
            gRFAL.nfcvData.origCtx = gRFAL.TxRx.ctx;

            gRFAL.TxRx.ctx.rxBuf    = ((gRFAL.nfcvData.origCtx.rxBuf != NULL) ? gRFAL.nfcvData.codingBuffer : NULL);
            gRFAL.TxRx.ctx.rxBufLen = (uint16_t)rfalConvBytesToBits(sizeof(gRFAL.nfcvData.codingBuffer));
            gRFAL.TxRx.ctx.flags = (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL
                                 | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP
                                 | (uint32_t)RFAL_TXRX_FLAGS_NFCIP1_OFF
                                 | (uint32_t)(gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_AGC_OFF)
                                 | (uint32_t)RFAL_TXRX_FLAGS_PAR_RX_KEEP
                                 | (uint32_t)RFAL_TXRX_FLAGS_PAR_TX_NONE;
            
            /* In NFCV a TxRx with a valid txBuf and txBufSize==0 indicates to send an EOF */
            /* Skip logic below that would go directly into receive                        */
            if ( gRFAL.TxRx.ctx.txBuf != NULL )
            {
                return  ERR_NONE;
            }
        }

        
        /*******************************************************************************/
        /* Check if the Transceive start performing Tx or goes directly to Rx          */
        if( (gRFAL.TxRx.ctx.txBuf == NULL) || (gRFAL.TxRx.ctx.txBufLen == 0U) )
        {
            /* Clear FIFO, Clear and Enable the Interrupts */
            rfalPrepareTransceive( );
            
            /* Disable our field upon a Rx reEnable on AP2P */
            if( rfalIsModeActiveComm(gRFAL.mode) )
            {
                st25r3911TxOff();
            }
            
            /* No Tx done, enable the Receiver */
            st25r3911ExecuteCommand( ST25R3911_CMD_UNMASK_RECEIVE_DATA );

            /* Start NRT manually, if FWT = 0 (wait endlessly for Rx) chip will ignore anyhow */
            st25r3911ExecuteCommand( ST25R3911_CMD_START_NO_RESPONSE_TIMER );
            
            gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_IDLE;
        }
        
        return ERR_NONE;
    }
    
    return ERR_WRONG_STATE;
}


/*******************************************************************************/
bool RfalRfST25R3911BClass::rfalIsTransceiveInTx( void )
{
    return ( (gRFAL.TxRx.state >= RFAL_TXRX_STATE_TX_IDLE) && (gRFAL.TxRx.state < RFAL_TXRX_STATE_RX_IDLE) );
}


/*******************************************************************************/
bool RfalRfST25R3911BClass::rfalIsTransceiveInRx( void )
{
    return (gRFAL.TxRx.state >= RFAL_TXRX_STATE_RX_IDLE);
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalTransceiveBlockingTx( uint8_t* txBuf, uint16_t txBufLen, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t* actLen, uint32_t flags, uint32_t fwt )
{
    ReturnCode               ret;
    rfalTransceiveContext    ctx;
    
    rfalCreateByteFlagsTxRxContext( ctx, txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt );
    EXIT_ON_ERR( ret, rfalStartTransceive( &ctx ) );
    
    return rfalTransceiveRunBlockingTx();
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalTransceiveRunBlockingTx( void )
{
    ReturnCode  ret;
        
    do{
        rfalWorker();
        ret = rfalGetTransceiveStatus();
    }
    while( rfalIsTransceiveInTx() && (ret == ERR_BUSY) );
    
    if( rfalIsTransceiveInRx() )
    {
        return ERR_NONE;
    }
    
    return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalTransceiveBlockingRx( void )
{
    ReturnCode ret;
    
    do{
        rfalWorker();
        ret = rfalGetTransceiveStatus();
    }
    while( rfalIsTransceiveInRx() && (ret == ERR_BUSY) );
        
    return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalTransceiveBlockingTxRx( uint8_t* txBuf, uint16_t txBufLen, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t* actLen, uint32_t flags, uint32_t fwt )
{
    ReturnCode ret;
    
    EXIT_ON_ERR( ret, rfalTransceiveBlockingTx( txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt ) );
    ret = rfalTransceiveBlockingRx();
    
    /* Convert received bits to bytes */
    if( actLen != NULL )
    {
        *actLen =  rfalConvBitsToBytes(*actLen);
    }
    
    return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalRunTransceiveWorker( void )
{
    if( gRFAL.state == RFAL_STATE_TXRX )
    {     
        /* Run Tx or Rx state machines */
        if( rfalIsTransceiveInTx() )
        {
            rfalTransceiveTx();
            return rfalGetTransceiveStatus();
        }
        
        if( rfalIsTransceiveInRx() )
        {
            rfalTransceiveRx();
            return rfalGetTransceiveStatus();
        }
    }    
    return ERR_WRONG_STATE;
}

/*******************************************************************************/
rfalTransceiveState RfalRfST25R3911BClass::rfalGetTransceiveState( void )
{
    return gRFAL.TxRx.state;
}

ReturnCode RfalRfST25R3911BClass::rfalGetTransceiveStatus( void )
{
    return ((gRFAL.TxRx.state == RFAL_TXRX_STATE_IDLE) ? gRFAL.TxRx.status : ERR_BUSY);
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalGetTransceiveRSSI( uint16_t *rssi )
{
    uint16_t amRSSI;
    uint16_t pmRSSI;
    
    if( rssi == NULL )
    {
        return ERR_PARAM;
    }
    
    /* Check if Manual channel is enabled */
    if( st25r3911CheckReg( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_rx_man, ST25R3911_REG_OP_CONTROL_rx_man ) )
    {
        st25r3911GetRSSI( &amRSSI, &pmRSSI );
        
        /* Check which channel is selected */
        *rssi = ( st25r3911CheckReg( ST25R3911_REG_RX_CONF1, ST25R3911_REG_RX_CONF1_ch_sel, ST25R3911_REG_RX_CONF1_ch_sel ) ? pmRSSI : amRSSI );
        return ERR_NONE;
    }
    
    *rssi = 0;
    return ERR_NOTSUPP;
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalWorker( void )
{
    switch( gRFAL.state )
    {
        case RFAL_STATE_TXRX:
            rfalRunTransceiveWorker();
            break;
        case RFAL_STATE_WUM:
            rfalRunWakeUpModeWorker();
            break;
            
        /* Nothing to be done */
        default:            
            /* MISRA 16.4: no empty default statement (a comment being enough) */
            break;
    }
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalErrorHandling( void )
{
    bool    rxHasIncParError;
    uint8_t fifoBytesToRead;
    uint8_t reEnRx[] = { ST25R3911_CMD_CLEAR_FIFO, ST25R3911_CMD_UNMASK_RECEIVE_DATA };
    

    fifoBytesToRead = rfalFIFOStatusGetNumBytes();
    
    
    /*******************************************************************************/
    /* EMVCo                                                                       */
    /*******************************************************************************/
    if( gRFAL.conf.eHandling == RFAL_ERRORHANDLING_EMVCO )
    {
        /*******************************************************************************/
        /* EMD Handling - NFC Forum Digital 1.1  4.1.1.1 ; EMVCo 2.6  4.9.2            */
        /* ReEnable the receiver on frames with a length < 4 bytes, upon:              */
        /*   - Collision or Framing error detected                                     */
        /*   - Residual bits are detected (hard framing error)                         */
        /*   - Parity error                                                            */
        /*   - CRC error                                                               */
        /*******************************************************************************/        
     
        /* Check if reception has incompete bytes or parity error */
        rxHasIncParError = ( rfalFIFOStatusIsIncompleteByte() ? true : rfalFIFOStatusIsMissingPar() );   /* MISRA 13.5 */
        
        /* In case there are residual bits decrement FIFO bytes */
        if( (fifoBytesToRead > 0U) && rxHasIncParError)
        {
            fifoBytesToRead--;
        }
            
        if( ( (gRFAL.fifo.bytesTotal + fifoBytesToRead) < RFAL_EMVCO_RX_MAXLEN )            &&
            ( (gRFAL.TxRx.status == ERR_RF_COLLISION) || (gRFAL.TxRx.status == ERR_FRAMING) || 
              (gRFAL.TxRx.status == ERR_PAR)          || (gRFAL.TxRx.status == ERR_CRC)     || 
              rxHasIncParError                                                               ) )
        {
            /* Ignore this reception, ReEnable receiver */
            st25r3911ExecuteCommands( reEnRx, sizeof(reEnRx) );
            
            rfalFIFOStatusClear();
            gRFAL.fifo.bytesTotal = 0;
            gRFAL.TxRx.status = ERR_BUSY;
            gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_WAIT_RXS;
        }
        return;
    }

    /*******************************************************************************/
    /* ISO14443A Mode                                                              */
    /*******************************************************************************/
    if( gRFAL.mode == RFAL_MODE_POLL_NFCA )
    {
        
        /*******************************************************************************/
        /* If we received one incomplete byte (not a block and a incomplete byte at    *
         * the end) we will raise a specific error ( support for T2T 4 bit ACK / NAK )   *
         * Otherwise just leave it as an CRC/FRAMING/PAR error                         */    
        /*******************************************************************************/
        if( (gRFAL.TxRx.status == ERR_PAR) || (gRFAL.TxRx.status == ERR_CRC) )
        {
            if( rfalFIFOStatusIsIncompleteByte() && (fifoBytesToRead == RFAL_NFC_RX_INCOMPLETE_LEN) )
            {
                st25r3911ReadFifo( (uint8_t*)(gRFAL.TxRx.ctx.rxBuf), fifoBytesToRead );
                if( (gRFAL.TxRx.ctx.rxRcvdLen) != NULL )
                {
                    *gRFAL.TxRx.ctx.rxRcvdLen = rfalFIFOGetNumIncompleteBits();
                }
                
                gRFAL.TxRx.status = ERR_INCOMPLETE_BYTE;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
            }
        }
    }
    
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalCleanupTransceive( void )
{
    /*******************************************************************************/
    /* Transceive flags                                                            */
    /*******************************************************************************/
    
    /* Restore default settings on NFCIP1 mode, Receiving parity + CRC bits and manual Tx Parity*/
    st25r3911ClrRegisterBits( ST25R3911_REG_ISO14443A_NFC, (ST25R3911_REG_ISO14443A_NFC_no_tx_par | ST25R3911_REG_ISO14443A_NFC_no_rx_par | ST25R3911_REG_ISO14443A_NFC_nfc_f0) );
    
    /* Restore AGC enabled */
    st25r3911SetRegisterBits( ST25R3911_REG_RX_CONF2, ST25R3911_REG_RX_CONF2_agc_en );
    
    /*******************************************************************************/
    
    
    
    /*******************************************************************************/
    /* Execute Post Transceive Callback                                            */
    /*******************************************************************************/
    if( gRFAL.callbacks.postTxRx != NULL )
    {
        gRFAL.callbacks.postTxRx();
    }
    /*******************************************************************************/

}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalPrepareTransceive( void )
{
    uint32_t maskInterrupts;
    uint8_t  reg;
    
    /*******************************************************************************/
    /* In the EMVCo mode the NRT will continue to run.                             *
     * For the clear to stop it, the EMV mode has to be disabled before            */
    st25r3911ClrRegisterBits( ST25R3911_REG_GPT_CONTROL, ST25R3911_REG_GPT_CONTROL_nrt_emv );
    
    /* Reset receive logic */
    st25r3911ExecuteCommand( ST25R3911_CMD_CLEAR_FIFO );
    
    /* Reset Rx Gain */
    st25r3911ExecuteCommand( ST25R3911_CMD_CLEAR_SQUELCH );
    
    
    /*******************************************************************************/
    /* FDT Poll                                                                    */
    /*******************************************************************************/
    if( rfalIsModePassiveComm( gRFAL.mode ) )  /* Passive Comms */
    {
       /* In Passive communications General Purpose Timer is used to measure FDT Poll */
       if( gRFAL.timings.FDTPoll != RFAL_TIMING_NONE )
       {
           /* Configure GPT to start at RX end */
           st25r3911StartGPTimer_8fcs( (uint16_t)rfalConv1fcTo8fc( MIN( gRFAL.timings.FDTPoll, (gRFAL.timings.FDTPoll - RFAL_FDT_POLL_ADJUSTMENT) ) ), ST25R3911_REG_GPT_CONTROL_gptc_erx );
       }
    }
    
    
    /*******************************************************************************/
    /* Execute Pre Transceive Callback                                             */
    /*******************************************************************************/
    if( gRFAL.callbacks.preTxRx != NULL )
    {
        gRFAL.callbacks.preTxRx();
    }
    /*******************************************************************************/
    
    maskInterrupts = ( ST25R3911_IRQ_MASK_FWL  | ST25R3911_IRQ_MASK_TXE  |
                       ST25R3911_IRQ_MASK_RXS  | ST25R3911_IRQ_MASK_RXE  |
                       ST25R3911_IRQ_MASK_FWL  | ST25R3911_IRQ_MASK_NRE  |
                       ST25R3911_IRQ_MASK_PAR  | ST25R3911_IRQ_MASK_CRC  |
                       ST25R3911_IRQ_MASK_ERR1 | ST25R3911_IRQ_MASK_ERR2  );
    
    
    /*******************************************************************************/
    /* Transceive flags                                                            */
    /*******************************************************************************/
    
    reg = (ST25R3911_REG_ISO14443A_NFC_no_tx_par_off | ST25R3911_REG_ISO14443A_NFC_no_rx_par_off | ST25R3911_REG_ISO14443A_NFC_nfc_f0_off);
    
    /* Check if NFCIP1 mode is to be enabled */
    if( (gRFAL.TxRx.ctx.flags & (uint8_t)RFAL_TXRX_FLAGS_NFCIP1_ON) != 0U )
    {
        reg |= ST25R3911_REG_ISO14443A_NFC_nfc_f0;
    }
    
    /* Check if Parity check is to be skipped and to keep the parity + CRC bits in FIFO */
    if( (gRFAL.TxRx.ctx.flags & (uint8_t)RFAL_TXRX_FLAGS_PAR_RX_KEEP) != 0U )
    {
        reg |= ST25R3911_REG_ISO14443A_NFC_no_rx_par;
    }

    /* Check if automatic Parity bits is to be disabled */
    if( (gRFAL.TxRx.ctx.flags & (uint8_t)RFAL_TXRX_FLAGS_PAR_TX_NONE) != 0U )
    {
        reg |= ST25R3911_REG_ISO14443A_NFC_no_tx_par;
    }
    
    /* Apply current TxRx flags on ISO14443A and NFC 106kb/s Settings Register */
    st25r3911ChangeRegisterBits( ST25R3911_REG_ISO14443A_NFC, (ST25R3911_REG_ISO14443A_NFC_no_tx_par | ST25R3911_REG_ISO14443A_NFC_no_rx_par | ST25R3911_REG_ISO14443A_NFC_nfc_f0), reg );
    
    
    /* Check if AGC is to be disabled */
    if( (gRFAL.TxRx.ctx.flags & (uint8_t)RFAL_TXRX_FLAGS_AGC_OFF) != 0U )
    {
        st25r3911ClrRegisterBits( ST25R3911_REG_RX_CONF2, ST25R3911_REG_RX_CONF2_agc_en );
    }
    else
    {
        st25r3911SetRegisterBits( ST25R3911_REG_RX_CONF2, ST25R3911_REG_RX_CONF2_agc_en );
    }
    /*******************************************************************************/
    
    

    /*******************************************************************************/
    /* EMVCo NRT mode                                                              */
    /*******************************************************************************/
    if( gRFAL.conf.eHandling == RFAL_ERRORHANDLING_EMVCO )
    {
        st25r3911SetRegisterBits( ST25R3911_REG_GPT_CONTROL, ST25R3911_REG_GPT_CONTROL_nrt_emv );
    }
    else
    {
        st25r3911ClrRegisterBits( ST25R3911_REG_GPT_CONTROL, ST25R3911_REG_GPT_CONTROL_nrt_emv );
    }
    /*******************************************************************************/
    
    
    
    /* In Active comms enable also External Field interrupts  */
    if( rfalIsModeActiveComm( gRFAL.mode ) )
    {
        maskInterrupts |= ( ST25R3911_IRQ_MASK_EOF  | ST25R3911_IRQ_MASK_EON | ST25R3911_IRQ_MASK_CAT | ST25R3911_IRQ_MASK_CAC );
    }
    
    
    /*******************************************************************************/
    /* clear and enable these interrupts */
    st25r3911GetInterrupt( maskInterrupts );
    st25r3911EnableInterrupts( maskInterrupts );
    
    /* Clear FIFO status local copy */
    rfalFIFOStatusClear();
}

/*******************************************************************************/
void RfalRfST25R3911BClass::rfalTransceiveTx( void )
{
    volatile uint32_t irqs;
    uint16_t          tmp;
    ReturnCode        ret;
    
    /* Supress warning in case NFC-V feature is disabled */
    ret = ERR_NONE;
    NO_WARNING(ret);
    
    
    irqs = ST25R3911_IRQ_MASK_NONE;
    
    if( gRFAL.TxRx.state != gRFAL.TxRx.lastState )
    {        
        /* rfalLogD( "RFAL: lastSt: %d curSt: %d \r\n", gRFAL.TxRx.lastState, gRFAL.TxRx.state ); */
        gRFAL.TxRx.lastState = gRFAL.TxRx.state;
    }
    
    switch( gRFAL.TxRx.state )
    {
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_IDLE:
            
            /* Nothing to do */
            
            gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_WAIT_GT ;
            /* fall through */
            
            
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_WAIT_GT:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            if( !rfalIsGTExpired() )
            {
                break;
            }
            
            gRFAL.tmr.GT = RFAL_TIMING_NONE;
            
            gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_WAIT_FDT;
            /* fall through */
            
            
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_WAIT_FDT:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            /* Only in Passive communications GPT is used to measure FDT Poll */
            if( rfalIsModePassiveComm( gRFAL.mode ) )
            {
                if( st25r3911IsGPTRunning() )
                {                
                   break;
                }
            }
            
            gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_TRANSMIT;
            /* fall through */
            
        
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_TRANSMIT:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            /* Clear FIFO, Clear and Enable the Interrupts */
            rfalPrepareTransceive( );

            /* Calculate when Water Level Interrupt will be triggered */
            gRFAL.fifo.expWL = (uint16_t)( st25r3911CheckReg( ST25R3911_REG_IO_CONF1, ST25R3911_REG_IO_CONF1_fifo_lt, ST25R3911_REG_IO_CONF1_fifo_lt_16bytes) ? RFAL_FIFO_OUT_LT_16 : RFAL_FIFO_OUT_LT_32 );

            /*******************************************************************************/
            /* In NFC-V streaming mode, the FIFO needs to be loaded with the coded bits    */
            if( (RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode) )
            {
            #if 0
                /* Debugging code: output the payload bits by writing into the FIFO and subsequent clearing */
                st25r3911WriteFifo(gRFAL.TxRx.ctx.txBuf, rfalConvBitsToBytes(gRFAL.TxRx.ctx.txBufLen));
                st25r3911ExecuteCommand( ST25R3911_CMD_CLEAR_FIFO );
            #endif
                /* Calculate the bytes needed to be Written into FIFO (a incomplete byte will be added as 1byte) */
                gRFAL.nfcvData.nfcvOffset = 0;
                ret = iso15693VCDCode(gRFAL.TxRx.ctx.txBuf, rfalConvBitsToBytes(gRFAL.TxRx.ctx.txBufLen), (((gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL) != 0U)?false:true),(((gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_NFCV_FLAG_MANUAL) != 0U)?false:true), (RFAL_MODE_POLL_PICOPASS == gRFAL.mode),
                          &gRFAL.fifo.bytesTotal, &gRFAL.nfcvData.nfcvOffset, gRFAL.nfcvData.codingBuffer, MIN( (uint16_t)ST25R3911_FIFO_DEPTH, (uint16_t)sizeof(gRFAL.nfcvData.codingBuffer) ), &gRFAL.fifo.bytesWritten);

                if( (ret != ERR_NONE) && (ret != ERR_AGAIN) )
                {
                    gRFAL.TxRx.status = ret;
                    gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
                    break;
                }
                /* Set the number of full bytes and bits to be transmitted */
                st25r3911SetNumTxBits( rfalConvBytesToBits(gRFAL.fifo.bytesTotal) );

                /* Load FIFO with coded bytes */
                /* TODO: check bytesWritten does not exceed 255 */
                st25r3911WriteFifo( gRFAL.nfcvData.codingBuffer, (uint8_t)gRFAL.fifo.bytesWritten );

            }
            /*******************************************************************************/
            else
            {
                /* Calculate the bytes needed to be Written into FIFO (a incomplete byte will be added as 1byte) */
                gRFAL.fifo.bytesTotal = (uint16_t)rfalCalcNumBytes(gRFAL.TxRx.ctx.txBufLen);
                
                /* Set the number of full bytes and bits to be transmitted */
                st25r3911SetNumTxBits( gRFAL.TxRx.ctx.txBufLen );
                
                /* Load FIFO with total length or FIFO's maximum */
                gRFAL.fifo.bytesWritten = MIN( gRFAL.fifo.bytesTotal, ST25R3911_FIFO_DEPTH );
                st25r3911WriteFifo( gRFAL.TxRx.ctx.txBuf, (uint8_t)gRFAL.fifo.bytesWritten );
            }
        
            /*Check if Observation Mode is enabled and set it on ST25R391x */
            rfalCheckEnableObsModeTx(); 
            
            /*******************************************************************************/
            /* Trigger/Start transmission                                                  */
            if( (gRFAL.TxRx.ctx.flags & (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL) != 0U )
            {
                st25r3911ExecuteCommand( ST25R3911_CMD_TRANSMIT_WITHOUT_CRC );
            }
            else
            {
                st25r3911ExecuteCommand( ST25R3911_CMD_TRANSMIT_WITH_CRC );
            }
             
            /* Check if a WL level is expected or TXE should come */
            gRFAL.TxRx.state = (( gRFAL.fifo.bytesWritten < gRFAL.fifo.bytesTotal ) ? RFAL_TXRX_STATE_TX_WAIT_WL : RFAL_TXRX_STATE_TX_WAIT_TXE);
            break;

        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_WAIT_WL:
            
            irqs = st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_FWL | ST25R3911_IRQ_MASK_TXE) );            
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
               break;  /* No interrupt to process */
            }
            
            if( ((irqs & ST25R3911_IRQ_MASK_FWL) != 0U) && ((irqs & ST25R3911_IRQ_MASK_TXE) == 0U) )
            {
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_RELOAD_FIFO;
            }
            else
            {
                gRFAL.TxRx.status = ERR_IO;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
                break;
            }
            
            /* fall through */
            
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_RELOAD_FIFO:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            /*******************************************************************************/
            /* In NFC-V streaming mode, the FIFO needs to be loaded with the coded bits    */
            if( (RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode) )
            {
                uint16_t maxLen;
                                
                /* Load FIFO with the remaining length or maximum available (which fit on the coding buffer) */
                maxLen = (uint16_t)MIN( (gRFAL.fifo.bytesTotal - gRFAL.fifo.bytesWritten), gRFAL.fifo.expWL);
                maxLen = (uint16_t)MIN( maxLen, sizeof(gRFAL.nfcvData.codingBuffer) );
                tmp    = 0;

                /* Calculate the bytes needed to be Written into FIFO (a incomplete byte will be added as 1byte) */
                ret = iso15693VCDCode(gRFAL.TxRx.ctx.txBuf, rfalConvBitsToBytes(gRFAL.TxRx.ctx.txBufLen), (((gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL) != 0U)?false:true), (((gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_NFCV_FLAG_MANUAL) != 0U)?false:true), (RFAL_MODE_POLL_PICOPASS == gRFAL.mode),
                          &gRFAL.fifo.bytesTotal, &gRFAL.nfcvData.nfcvOffset, gRFAL.nfcvData.codingBuffer, maxLen, &tmp);

                if( (ret != ERR_NONE) && (ret != ERR_AGAIN) )
                {
                    gRFAL.TxRx.status = ret;
                    gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
                    break;
                }

                /* Load FIFO with coded bytes */
                /* TODO: check tmp does not exceed 255 */
                st25r3911WriteFifo( gRFAL.nfcvData.codingBuffer, (uint8_t)tmp );
            }
            /*******************************************************************************/
            else
            {
                /* Load FIFO with the remaining length or maximum available */
                tmp = MIN( (gRFAL.fifo.bytesTotal - gRFAL.fifo.bytesWritten), gRFAL.fifo.expWL);       /* tmp holds the number of bytes written on this iteration */
                /* TODO: check tmp does not exceed 255 */
                st25r3911WriteFifo( &gRFAL.TxRx.ctx.txBuf[gRFAL.fifo.bytesWritten], (uint8_t)tmp );
            }
            
            /* Update total written bytes to FIFO */
            gRFAL.fifo.bytesWritten += tmp;
            
            /* Check if a WL level is expected or TXE should come */
            gRFAL.TxRx.state = (( gRFAL.fifo.bytesWritten < gRFAL.fifo.bytesTotal ) ? RFAL_TXRX_STATE_TX_WAIT_WL : RFAL_TXRX_STATE_TX_WAIT_TXE);
            break;
            
            
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_WAIT_TXE:
           
            irqs = st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_FWL | ST25R3911_IRQ_MASK_TXE) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
               break;  /* No interrupt to process */
            }
                        
            
            if( (irqs & ST25R3911_IRQ_MASK_TXE) != 0U )
            {
                /* In Active comm start SW timer to measure FWT */
                if( rfalIsModeActiveComm( gRFAL.mode) && (gRFAL.TxRx.ctx.fwt != RFAL_FWT_NONE) && (gRFAL.TxRx.ctx.fwt != 0U) ) 
                {
                    rfalTimerStart( gRFAL.tmr.FWT, rfalConv1fcToMs( gRFAL.TxRx.ctx.fwt ) );
                }
                
                gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_DONE;
            }
            else if( (irqs & ST25R3911_IRQ_MASK_FWL) != 0U )
            {
                /*******************************************************************************/
                /* REMARK: Silicon workaround ST25R3911 Errata #TBD                            */
                /* ST25R3911 may send a WL even when all bytes have been written to FIFO       */
                /*******************************************************************************/
                break;  /* Ignore ST25R3911 FIFO WL if total TxLen is already on the FIFO */
            }
            else
            {
               gRFAL.TxRx.status = ERR_IO;
               gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
               break;
            }
            
            /* fall through */
           
        
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_DONE:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            /* If no rxBuf is provided do not wait/expect Rx */
            if( gRFAL.TxRx.ctx.rxBuf == NULL )
            {
                /*Check if Observation Mode was enabled and disable it on ST25R391x */
                rfalCheckDisableObsMode();
                
                /* Clean up Transceive */
                rfalCleanupTransceive();
                                
                gRFAL.TxRx.status = ERR_NONE;
                gRFAL.TxRx.state  =  RFAL_TXRX_STATE_IDLE;
                break;
            }
            
            rfalCheckEnableObsModeRx();
            
            /* Goto Rx */
            gRFAL.TxRx.state  =  RFAL_TXRX_STATE_RX_IDLE;
            break;
           
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_FAIL:
            
            /* Error should be assigned by previous state */
            if( gRFAL.TxRx.status == ERR_BUSY )
            {
                gRFAL.TxRx.status = ERR_SYSTEM;
            }
            
            /*Check if Observation Mode was enabled and disable it on ST25R391x */
            rfalCheckDisableObsMode();
            
            /* Clean up Transceive */
            rfalCleanupTransceive();
            
            gRFAL.TxRx.state = RFAL_TXRX_STATE_IDLE;
            break;
        
        /*******************************************************************************/
        default:
            gRFAL.TxRx.status = ERR_SYSTEM;
            gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
            break;
    }
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalTransceiveRx( void )
{
    volatile uint32_t irqs;
    uint8_t           tmp;
    uint8_t           aux;
    
    irqs = ST25R3911_IRQ_MASK_NONE;
    
    if( gRFAL.TxRx.state != gRFAL.TxRx.lastState )
    {        
        /* rfalLogD( "RFAL: lastSt: %d curSt: %d \r\n", gRFAL.TxRx.lastState, gRFAL.TxRx.state ); */
        gRFAL.TxRx.lastState = gRFAL.TxRx.state;
    }
    
    switch( gRFAL.TxRx.state )
    {
        /*******************************************************************************/
        case RFAL_TXRX_STATE_RX_IDLE:
            
            /* Clear rx counters */
            gRFAL.fifo.bytesWritten   = 0;    // Total bytes written on RxBuffer
            gRFAL.fifo.bytesTotal     = 0;    // Total bytes in FIFO will now be from Rx
            if( gRFAL.TxRx.ctx.rxRcvdLen != NULL )
            {
                *gRFAL.TxRx.ctx.rxRcvdLen = 0;
            }
            
            gRFAL.TxRx.state = ( rfalIsModeActiveComm( gRFAL.mode ) ? RFAL_TXRX_STATE_RX_WAIT_EON : RFAL_TXRX_STATE_RX_WAIT_RXS );
            break;
           
           
        /*******************************************************************************/
        case RFAL_TXRX_STATE_RX_WAIT_RXS:
        
            /*******************************************************************************/
            /* If in Active comm, Check if FWT SW timer has expired */
            if( rfalIsModeActiveComm( gRFAL.mode ) && (gRFAL.TxRx.ctx.fwt != RFAL_FWT_NONE) && (gRFAL.TxRx.ctx.fwt != 0U) )
            {
                if( rfalTimerisExpired( gRFAL.tmr.FWT ) )  
                {
                    gRFAL.TxRx.status = ERR_TIMEOUT;
                    gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                    break;
                }
            }
            
            /*******************************************************************************/
            irqs = st25r3911GetInterrupt( ( ST25R3911_IRQ_MASK_RXS | ST25R3911_IRQ_MASK_NRE | ST25R3911_IRQ_MASK_EOF | ST25R3911_IRQ_MASK_RXE) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
                break;  /* No interrupt to process */
            }
            
                        
            /*******************************************************************************/
            /* REMARK: Silicon workaround ST25R3911 Errata #1.7                            */
            /* NRE interrupt may be triggered twice                                        */
            /* Ignore NRE if is detected together with no Rx Start                         */
            /*******************************************************************************/
            
            /* Only raise Timeout if NRE is detected with no Rx Start (NRT EMV mode)       */
            if( ((irqs & ST25R3911_IRQ_MASK_NRE) != 0U) && ((irqs & ST25R3911_IRQ_MASK_RXS) == 0U) )
            {
                gRFAL.TxRx.status = ERR_TIMEOUT;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                break;
            }
            
            /* Only raise Link Loss if EOF is detected with no Rx Start */
            if( ((irqs & ST25R3911_IRQ_MASK_EOF) != 0U) && ((irqs & ST25R3911_IRQ_MASK_RXS) == 0U) )
            {
                /* In AP2P a Field On has already occurred - treat this as timeout | mute */
                gRFAL.TxRx.status = ( rfalIsModeActiveComm( gRFAL.mode ) ? ERR_TIMEOUT : ERR_LINK_LOSS );
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                break;
            }
            
            if( (irqs & ST25R3911_IRQ_MASK_RXS) != 0U )
            {
                /* If we got RXS + RXE together, jump directly into RFAL_TXRX_STATE_RX_ERR_CHECK */
                if( (irqs & ST25R3911_IRQ_MASK_RXE) != 0U )
                {
                    gRFAL.TxRx.rxse  = true;
                    gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_ERR_CHECK;
                    break;
                }
                else
                {
                    /*******************************************************************************/
                    /* REMARK: Silicon workaround ST25R3911 Errata #1.1                            */
                    /* Rarely on corrupted frames I_rxs gets signaled but I_rxe is not signaled    */
                    /* Use a SW timer to handle an eventual missing RXE                            */
                    rfalTimerStart( gRFAL.tmr.RXE, RFAL_NORXE_TOUT );
                    /*******************************************************************************/
                    
                    gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_WAIT_RXE;
                }
            }
            else if( (irqs & ST25R3911_IRQ_MASK_RXE) != 0U )
            {
                /*******************************************************************************/
                /* REMARK: Silicon workaround ST25R3911 Errata #1.9                            */
                /* ST25R3911 may indicate RXE without RXS previously, this happens upon some   */
                /* noise or incomplete byte frames with less than 4 bits                       */
                /*******************************************************************************/
                
                gRFAL.TxRx.status = ERR_IO;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                
                rfalErrorHandling();
                break;
            }
            else
            {
               gRFAL.TxRx.status = ERR_IO;
               gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
               break;
            }
            
            /* fall through */
            
            
        /*******************************************************************************/    
        case RFAL_TXRX_STATE_RX_WAIT_RXE:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            irqs = st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_RXE | ST25R3911_IRQ_MASK_FWL | ST25R3911_IRQ_MASK_EOF) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
                /*******************************************************************************/
                /* REMARK: Silicon workaround ST25R3911B Errata #1.1                           */
                /* ST25R3911 may indicate RXS without RXE afterwards, this happens rarely on   */
                /* corrupted frames.                                                           */
                /* SW timer is used to timeout upon a missing RXE                              */
                if( rfalTimerisExpired( gRFAL.tmr.RXE ) )
                {
                    gRFAL.TxRx.status = ERR_FRAMING;
                    gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                }
                /*******************************************************************************/
                    
                break;  /* No interrupt to process */
            }
            
            if( ((irqs & ST25R3911_IRQ_MASK_FWL) != 0U) && ((irqs & ST25R3911_IRQ_MASK_RXE) == 0U) )
            {
                gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_READ_FIFO;
                break;
            }
            
            gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_ERR_CHECK;
            /* fall through */
            
            
        /*******************************************************************************/
        case RFAL_TXRX_STATE_RX_ERR_CHECK:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
        
            /* Retrieve and check for any error irqs */
            irqs |= st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_CRC | ST25R3911_IRQ_MASK_PAR | ST25R3911_IRQ_MASK_ERR1 | ST25R3911_IRQ_MASK_ERR2 | ST25R3911_IRQ_MASK_COL) );
        
            if( (irqs & ST25R3911_IRQ_MASK_ERR1) != 0U )
            {
                gRFAL.TxRx.status = ERR_FRAMING;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;
                
                /* Check if there's a specific error handling for this */
                rfalErrorHandling();
                break;
            }
            /* Discard Soft Framing errors if not in EMVCo error handling */
            else if( ((irqs & ST25R3911_IRQ_MASK_ERR2) != 0U) && (gRFAL.conf.eHandling == RFAL_ERRORHANDLING_EMVCO) )
            {
                gRFAL.TxRx.status = ERR_FRAMING;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;
                
                /* Check if there's a specific error handling for this */
                rfalErrorHandling();
                break;
            }
            else if( (irqs & ST25R3911_IRQ_MASK_PAR) != 0U )
            {
                gRFAL.TxRx.status = ERR_PAR;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;
                
                /* Check if there's a specific error handling for this */
                rfalErrorHandling();
                break;
            }
            else if( (irqs & ST25R3911_IRQ_MASK_CRC) != 0U )
            {
                gRFAL.TxRx.status = ERR_CRC;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;
                
                /* Check if there's a specific error handling for this */
                rfalErrorHandling();
                break;
            }
            else if( (irqs & ST25R3911_IRQ_MASK_COL) != 0U )
            {
                gRFAL.TxRx.status = ERR_RF_COLLISION;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;
                
                /* Check if there's a specific error handling for this */
                rfalErrorHandling();
                break;
            }
            else if( ((irqs & ST25R3911_IRQ_MASK_EOF) != 0U) && ((irqs & ST25R3911_IRQ_MASK_RXE) == 0U) )
            {
                 gRFAL.TxRx.status = ERR_LINK_LOSS;
                 gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                 break;
            }
            else if( ((irqs & ST25R3911_IRQ_MASK_RXE) != 0U) || (gRFAL.TxRx.rxse) )
            {
                /* Reception ended without any error indication,                  *
                 * check FIFO status for malformed or incomplete frames           */
                
                /* Check if the reception ends with an incomplete byte (residual bits) */
                if( rfalFIFOStatusIsIncompleteByte() )
                {
                   gRFAL.TxRx.status = ERR_INCOMPLETE_BYTE;
                }
                /* Check if the reception ends with missing parity bit */
                else if( rfalFIFOStatusIsMissingPar() )
                {
                   gRFAL.TxRx.status = ERR_FRAMING;
                }
                else
                {
                    /* MISRA 15.7 - Empty else */
                }
                
                gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_READ_DATA;
            }
            else
            {
                gRFAL.TxRx.status = ERR_IO;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                break;
            }
                        
            /* fall through */
            
            
        /*******************************************************************************/    
        case RFAL_TXRX_STATE_RX_READ_DATA:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
                        
            tmp = rfalFIFOStatusGetNumBytes();
                        
            /*******************************************************************************/
            /* Check if CRC should not be placed in rxBuf                                  */
            if( ((gRFAL.TxRx.ctx.flags & (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP) == 0U) )
            {
                /* Check if CRC is being placed into the FIFO and if received frame was bigger than CRC */
                if( st25r3911IsCRCinFIFO() && ((gRFAL.fifo.bytesTotal + tmp) > 0U) )
                {
                    /* By default CRC will not be placed into the rxBuffer */
                    if( ( tmp > (uint8_t)RFAL_CRC_LEN) )  
                    {
                        tmp -= (uint8_t)RFAL_CRC_LEN;
                    }
                    /* If the CRC was already placed into rxBuffer (due to WL interrupt where CRC was already in FIFO Read)
                     * cannot remove it from rxBuf. Can only remove it from rxBufLen not indicate the presence of CRC    */ 
                    else if(gRFAL.fifo.bytesTotal > (uint16_t)RFAL_CRC_LEN)                       
                    {                        
                        gRFAL.fifo.bytesTotal -= (uint16_t)RFAL_CRC_LEN;
                    }
                    else
                    {
                        /* MISRA 15.7 - Empty else */
                    }
                }
            }
            
            gRFAL.fifo.bytesTotal += tmp;                    /* add to total bytes counter */
            
            /*******************************************************************************/
            /* Check if remaining bytes fit on the rxBuf available                         */
            if( gRFAL.fifo.bytesTotal > rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen) )
            {
                tmp = (uint8_t)( rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen) - gRFAL.fifo.bytesWritten);
                
                gRFAL.TxRx.status = ERR_NOMEM;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
            }

            /*******************************************************************************/
            /* Retrieve remaining bytes from FIFO to rxBuf, and assign total length rcvd   */
            st25r3911ReadFifo( &gRFAL.TxRx.ctx.rxBuf[gRFAL.fifo.bytesWritten], tmp);
            if( (gRFAL.TxRx.ctx.rxRcvdLen != NULL) )
            {
                (*gRFAL.TxRx.ctx.rxRcvdLen) = (uint16_t)rfalConvBytesToBits( gRFAL.fifo.bytesTotal );
                if( rfalFIFOStatusIsIncompleteByte() )
                {
                    (*gRFAL.TxRx.ctx.rxRcvdLen) -= (RFAL_BITS_IN_BYTE - rfalFIFOGetNumIncompleteBits());
                }
            }

            /*******************************************************************************/
            /* Decode sub bit stream into payload bits for NFCV, if no error found so far  */
            if( ((RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode)) && (gRFAL.TxRx.status == ERR_BUSY) )
            {
                ReturnCode ret;
                uint16_t offset = 0;

                ret = iso15693VICCDecode(gRFAL.TxRx.ctx.rxBuf, gRFAL.fifo.bytesTotal,
                        gRFAL.nfcvData.origCtx.rxBuf, rfalConvBitsToBytes(gRFAL.nfcvData.origCtx.rxBufLen), &offset, gRFAL.nfcvData.origCtx.rxRcvdLen, gRFAL.nfcvData.ignoreBits, (RFAL_MODE_POLL_PICOPASS == gRFAL.mode) );

                if( ((ERR_NONE == ret) || (ERR_CRC == ret))
                     && (((uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP & gRFAL.nfcvData.origCtx.flags) == 0U)
                     &&  ((*gRFAL.nfcvData.origCtx.rxRcvdLen % RFAL_BITS_IN_BYTE) == 0U)
                     &&  (*gRFAL.nfcvData.origCtx.rxRcvdLen >= rfalConvBytesToBits(RFAL_CRC_LEN) )
                   )
                {
                   *gRFAL.nfcvData.origCtx.rxRcvdLen -= (uint16_t)rfalConvBytesToBits(RFAL_CRC_LEN); /* Remove CRC */
                }
                
                /* Restore original ctx */
                gRFAL.TxRx.ctx    = gRFAL.nfcvData.origCtx;
                gRFAL.TxRx.status = ((ret != ERR_NONE) ? ret : ERR_BUSY);
            }
            
            /*******************************************************************************/
            /* If an error as been marked/detected don't fall into to RX_DONE  */
            if( gRFAL.TxRx.status != ERR_BUSY )
            {
                gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_FAIL;
                break;
            }
            
            if( rfalIsModeActiveComm( gRFAL.mode ) )
            {
                gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_WAIT_EOF;
                break;
            }
            
            gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_DONE;
            /* fall through */
                            
            
        /*******************************************************************************/
        case RFAL_TXRX_STATE_RX_DONE:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            /*Check if Observation Mode was enabled and disable it on ST25R391x */
            rfalCheckDisableObsMode();
            
            /* Clean up Transceive */
            rfalCleanupTransceive();

            
            gRFAL.TxRx.status = ERR_NONE;
            gRFAL.TxRx.state  = RFAL_TXRX_STATE_IDLE;
            break;
            
            
        /*******************************************************************************/
        case RFAL_TXRX_STATE_RX_READ_FIFO:
        
            /*******************************************************************************/
            /* REMARK: Silicon workaround ST25R3911B Errata #1.1                           */
            /* ST25R3911 may indicate RXS without RXE afterwards, this happens rarely on   */
            /* corrupted frames.                                                           */
            /* Re-Start SW timer to handle an eventual missing RXE                         */
            rfalTimerStart( gRFAL.tmr.RXE, RFAL_NORXE_TOUT );
            /*******************************************************************************/        
                    
        
            tmp = rfalFIFOStatusGetNumBytes();
            gRFAL.fifo.bytesTotal += tmp;
            
            /*******************************************************************************/
            /* Calculate the amount of bytes that still fits in rxBuf                      */
            aux = (uint8_t)(( gRFAL.fifo.bytesTotal > rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen) ) ? (rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen) - gRFAL.fifo.bytesWritten) : tmp);
            
            /*******************************************************************************/
            /* Retrieve incoming bytes from FIFO to rxBuf, and store already read amount   */
            st25r3911ReadFifo( &gRFAL.TxRx.ctx.rxBuf[gRFAL.fifo.bytesWritten], aux);
            gRFAL.fifo.bytesWritten += aux;
            
            /*******************************************************************************/
            /* If the bytes already read were not the full FIFO WL, dump the remaining     *
             * FIFO so that ST25R391x can continue with reception                          */
            if( aux < tmp )
            {
                st25r3911ReadFifo( NULL, (tmp - aux) );
            }
            
            rfalFIFOStatusClear();
            gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_WAIT_RXE;
            break;
            
            
        /*******************************************************************************/
        case RFAL_TXRX_STATE_RX_FAIL:
            
            /*Check if Observation Mode was enabled and disable it on ST25R391x */
            rfalCheckDisableObsMode();
            
            /* Clean up Transceive */
            rfalCleanupTransceive();
            
            /* Error should be assigned by previous state */
            if( gRFAL.TxRx.status == ERR_BUSY )
            {                
                gRFAL.TxRx.status = ERR_SYSTEM;
            }
             
            /*rfalLogD( "RFAL: curSt: %d  Error: %d \r\n", gRFAL.TxRx.state, gRFAL.TxRx.status );*/
            gRFAL.TxRx.state = RFAL_TXRX_STATE_IDLE;
            break;
        
        
        /*******************************************************************************/    
        case RFAL_TXRX_STATE_RX_WAIT_EON:
            
            irqs = st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_EON | ST25R3911_IRQ_MASK_NRE) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
                break;  /* No interrupt to process */
            }
            
            if( (irqs & ST25R3911_IRQ_MASK_EON) != 0U )
            {
                gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_WAIT_RXS;
            }
            
            if( (irqs & ST25R3911_IRQ_MASK_NRE) != 0U )
            {
                /* ST25R3911 uses the NRT to measure other device's Field On max time: Tadt + (n x Trfw)  */
                gRFAL.TxRx.status = ERR_LINK_LOSS;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
            }
            break;

        
        /*******************************************************************************/    
        case RFAL_TXRX_STATE_RX_WAIT_EOF:
           
            irqs = st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_CAT | ST25R3911_IRQ_MASK_CAC) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
               break;  /* No interrupt to process */
            }
            
            if( (irqs & ST25R3911_IRQ_MASK_CAT) != 0U )
            {
               gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_DONE;
            }
            else if( (irqs & ST25R3911_IRQ_MASK_CAC) != 0U )
            {
               gRFAL.TxRx.status = ERR_RF_COLLISION;
               gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
            }
            else
            {
               gRFAL.TxRx.status = ERR_IO;
               gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
            }
            break;
            
            
        /*******************************************************************************/
        default:
            gRFAL.TxRx.status = ERR_SYSTEM;
            gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
            break;           
    }    
}

/*******************************************************************************/
void RfalRfST25R3911BClass::rfalFIFOStatusUpdate( void )
{
    if(gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] == RFAL_FIFO_STATUS_INVALID)
    {
        st25r3911ReadMultipleRegisters( ST25R3911_REG_FIFO_RX_STATUS1, gRFAL.fifo.status, ST25R3911_FIFO_STATUS_LEN );
    }
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalFIFOStatusClear( void )
{
    gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] = RFAL_FIFO_STATUS_INVALID;
}


/*******************************************************************************/
uint8_t RfalRfST25R3911BClass::rfalFIFOStatusGetNumBytes( void )
{
    rfalFIFOStatusUpdate();
    
    return gRFAL.fifo.status[RFAL_FIFO_STATUS_REG1]; 
   
}


/*******************************************************************************/
bool RfalRfST25R3911BClass::rfalFIFOStatusIsIncompleteByte( void )
{
    rfalFIFOStatusUpdate();
    return ((gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] & (ST25R3911_REG_FIFO_RX_STATUS2_mask_fifo_lb | ST25R3911_REG_FIFO_RX_STATUS2_fifo_ncp)) != 0U);
}


/*******************************************************************************/
bool RfalRfST25R3911BClass::rfalFIFOStatusIsMissingPar( void )
{
    rfalFIFOStatusUpdate();
    return ((gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] & ST25R3911_REG_FIFO_RX_STATUS2_np_lb) != 0U);
}


/*******************************************************************************/
uint8_t RfalRfST25R3911BClass::rfalFIFOGetNumIncompleteBits( void )
{
    rfalFIFOStatusUpdate();
    return ((gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] & ST25R3911_REG_FIFO_RX_STATUS2_mask_fifo_lb) >> ST25R3911_REG_FIFO_RX_STATUS2_shift_fifo_lb);
}

/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalISO14443ATransceiveShortFrame( rfal14443AShortFrameCmd txCmd, uint8_t* rxBuf, uint8_t rxBufLen, uint16_t* rxRcvdLen, uint32_t fwt )
{
    ReturnCode ret;
    uint8_t    directCmd;

    /* Check if RFAL is properly initialized */
    if( !st25r3911IsTxEnabled() || (gRFAL.state < RFAL_STATE_MODE_SET) || (( gRFAL.mode != RFAL_MODE_POLL_NFCA ) && ( gRFAL.mode != RFAL_MODE_POLL_NFCA_T1T )) )
    {
        return ERR_WRONG_STATE;
    }    
    
    /* Check for valid parameters */
    if( (rxBuf == NULL) || (rxRcvdLen == NULL) || (fwt == RFAL_FWT_NONE) )
    {
        return ERR_PARAM;
    }
    
    /*******************************************************************************/
    /* Select the Direct Command to be performed                                   */
    switch (txCmd)
    {
        case RFAL_14443A_SHORTFRAME_CMD_WUPA:
            directCmd = ST25R3911_CMD_TRANSMIT_WUPA;
            break;
            
        case RFAL_14443A_SHORTFRAME_CMD_REQA:
            directCmd = ST25R3911_CMD_TRANSMIT_REQA;
            break;
            
        default:
            return ERR_PARAM;
    }
    
    
    /*******************************************************************************/
    /* Enable anti collision to recognise collision in first byte of SENS_REQ */
    st25r3911SetRegisterBits( ST25R3911_REG_ISO14443A_NFC, ST25R3911_REG_ISO14443A_NFC_antcl);
    
    /* Disable CRC while receiving since ATQA has no CRC included */
    st25r3911SetRegisterBits( ST25R3911_REG_AUX, ST25R3911_REG_AUX_no_crc_rx );
    
    
    /*******************************************************************************/
    /* Wait for GT and FDT */
    while( !rfalIsGTExpired() )      { /* MISRA 15.6: mandatory brackets */ };
    while( st25r3911IsGPTRunning() ) { /* MISRA 15.6: mandatory brackets */ };
    
    gRFAL.tmr.GT = RFAL_TIMING_NONE;

    
    /*******************************************************************************/
    /* Prepare for Transceive, Receive only (bypass Tx states) */
    gRFAL.TxRx.ctx.flags     = ((uint32_t) RFAL_TXRX_FLAGS_CRC_TX_MANUAL | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP );
    gRFAL.TxRx.ctx.rxBuf     = rxBuf;
    gRFAL.TxRx.ctx.rxBufLen  = rxBufLen;
    gRFAL.TxRx.ctx.rxRcvdLen = rxRcvdLen;
    
    /*******************************************************************************/
    /* Load NRT with FWT */
    st25r3911SetNoResponseTime_64fcs( rfalConv1fcTo64fc( MIN( (fwt + RFAL_FWT_ADJUSTMENT + RFAL_FWT_A_ADJUSTMENT), RFAL_ST25R3911_NRT_MAX_1FC ) ) );
    
    if( gRFAL.timings.FDTListen != RFAL_TIMING_NONE )
    {
        /* Set Minimum FDT(Listen) in which PICC is not allowed to send a response */
        st25r3911WriteRegister( ST25R3911_REG_MASK_RX_TIMER, (uint8_t)rfalConv1fcTo64fc( ((RFAL_FDT_LISTEN_MRT_ADJUSTMENT + RFAL_FDT_LISTEN_A_ADJUSTMENT) > gRFAL.timings.FDTListen) ? RFAL_ST25R3911_MRT_MIN_1FC : (gRFAL.timings.FDTListen - (RFAL_FDT_LISTEN_MRT_ADJUSTMENT + RFAL_FDT_LISTEN_A_ADJUSTMENT)) ) );
    }
    
    /* In Passive communications General Purpose Timer is used to measure FDT Poll */
    if( gRFAL.timings.FDTPoll != RFAL_TIMING_NONE )
    {
        /* Configure GPT to start at RX end */
        st25r3911StartGPTimer_8fcs( (uint16_t)rfalConv1fcTo8fc( MIN( gRFAL.timings.FDTPoll, (gRFAL.timings.FDTPoll - RFAL_FDT_POLL_ADJUSTMENT) ) ), ST25R3911_REG_GPT_CONTROL_gptc_erx );
    }
    
    /*******************************************************************************/
    rfalPrepareTransceive();
    
    /* Also enable bit collision interrupt */
    st25r3911GetInterrupt( ST25R3911_IRQ_MASK_COL );
    st25r3911EnableInterrupts( ST25R3911_IRQ_MASK_COL );
    
    /*Check if Observation Mode is enabled and set it on ST25R391x */
    rfalCheckEnableObsModeTx();
    
    /*******************************************************************************/
    /* Chip bug: Clear nbtx bits before sending WUPA/REQA - otherwise ST25R3911 will report parity error */
    st25r3911WriteRegister( ST25R3911_REG_NUM_TX_BYTES2, 0);

    /* Send either WUPA or REQA. All affected tags will backscatter ATQA and change to READY state */
    st25r3911ExecuteCommand( directCmd );
    
    /* Wait for TXE */
    if( st25r3911WaitForInterruptsTimed( ST25R3911_IRQ_MASK_TXE, (uint16_t)MAX( rfalConv1fcToMs( fwt ), RFAL_ST25R3911_SW_TMR_MIN_1MS ) ) == 0U)
    {
        ret = ERR_IO;
    }
    else
    {
        /*Check if Observation Mode is enabled and set it on ST25R391x */
        rfalCheckEnableObsModeRx();
        
        /* Jump into a transceive Rx state for reception (bypass Tx states) */
        gRFAL.state       = RFAL_STATE_TXRX;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_IDLE;
        gRFAL.TxRx.status = ERR_BUSY;
        
        /* Execute Transceive Rx blocking */
        ret = rfalTransceiveBlockingRx();
    }
    
    
    /* Disable Collision interrupt */
    st25r3911DisableInterrupts( (ST25R3911_IRQ_MASK_COL) );
    
    /* Disable anti collision again */
    st25r3911ClrRegisterBits( ST25R3911_REG_ISO14443A_NFC, ST25R3911_REG_ISO14443A_NFC_antcl );
    
    /* ReEnable CRC on Rx */
    st25r3911ClrRegisterBits(ST25R3911_REG_AUX, ST25R3911_REG_AUX_no_crc_rx );
    
    return ret;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalISO14443ATransceiveAnticollisionFrame( uint8_t *buf, uint8_t *bytesToSend, uint8_t *bitsToSend, uint16_t *rxLength, uint32_t fwt )
{
    ReturnCode            ret;
    rfalTransceiveContext ctx;
    uint8_t               collByte;
    uint8_t               collData;
    
    /* Check if RFAL is properly initialized */
    if( (gRFAL.state < RFAL_STATE_MODE_SET) || ( gRFAL.mode != RFAL_MODE_POLL_NFCA ) )
    {
        return ERR_WRONG_STATE;
    }
    
    /* Check for valid parameters */
    if( (buf == NULL) || (bytesToSend == NULL) || (bitsToSend == NULL) || (rxLength == NULL) )
    {
        return ERR_PARAM;
    }
    
    /*******************************************************************************/
    /* Set speficic Analog Config for Anticolission if needed */
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_ANTICOL) );
    
    
    /*******************************************************************************/
    /* Enable anti collision to recognise collision in first byte of SENS_REQ */
    st25r3911SetRegisterBits( ST25R3911_REG_ISO14443A_NFC, ST25R3911_REG_ISO14443A_NFC_antcl );
    
    /* Disable CRC while receiving */
    st25r3911SetRegisterBits( ST25R3911_REG_AUX, ST25R3911_REG_AUX_no_crc_rx );
    
    
    
    /*******************************************************************************/
    /* Prepare for Transceive                                                      */
    ctx.flags     = ( (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP | (uint32_t)RFAL_TXRX_FLAGS_AGC_OFF );  /* Disable Automatic Gain Control (AGC) for better detection of collision */
    ctx.txBuf     = buf;
    ctx.txBufLen  = (uint16_t)(rfalConvBytesToBits( *bytesToSend ) + *bitsToSend );
    ctx.rxBuf     = &buf[*bytesToSend];
    ctx.rxBufLen  = (uint16_t)rfalConvBytesToBits( RFAL_ISO14443A_SDD_RES_LEN );
    ctx.rxRcvdLen = rxLength;
    ctx.fwt       = fwt;
    
    rfalStartTransceive( &ctx );
    
    /* Additionally enable bit collision interrupt */
    st25r3911GetInterrupt( ST25R3911_IRQ_MASK_COL );
    st25r3911EnableInterrupts( ST25R3911_IRQ_MASK_COL );
    
    /*******************************************************************************/
    collByte = 0;
    
    /* save the collision byte */
    if ((*bitsToSend) > 0U)
    {
        buf[(*bytesToSend)] <<= (RFAL_BITS_IN_BYTE - (*bitsToSend));
        buf[(*bytesToSend)] >>= (RFAL_BITS_IN_BYTE - (*bitsToSend));
        collByte = buf[(*bytesToSend)];
    }
    
    
    /*******************************************************************************/
    /* Run Transceive blocking */
    ret = rfalTransceiveRunBlockingTx();
    if( ret == ERR_NONE)
    {
       ret = rfalTransceiveBlockingRx();
    
       /*******************************************************************************/
       if ((*bitsToSend) > 0U)
       {
           buf[(*bytesToSend)] >>= (*bitsToSend);
           buf[(*bytesToSend)] <<= (*bitsToSend);
           buf[(*bytesToSend)] |= collByte;
       }
       
       if( (ERR_RF_COLLISION == ret) )
       {
           /* read out collision register */
           st25r3911ReadRegister( ST25R3911_REG_COLLISION_STATUS, &collData);

           (*bytesToSend) = ((collData >> ST25R3911_REG_COLLISION_STATUS_shift_c_byte) & 0x0FU); // 4-bits Byte information
           (*bitsToSend)  = ((collData >> ST25R3911_REG_COLLISION_STATUS_shift_c_bit)  & 0x07U); // 3-bits bit information

       }
    }
    
   
    /*******************************************************************************/
    /* Disable Collision interrupt */
    st25r3911DisableInterrupts( (ST25R3911_IRQ_MASK_COL) );
    
    /* Disable anti collision again */
    st25r3911ClrRegisterBits( ST25R3911_REG_ISO14443A_NFC, ST25R3911_REG_ISO14443A_NFC_antcl );
    
    /* ReEnable CRC on Rx */
    st25r3911ClrRegisterBits( ST25R3911_REG_AUX, ST25R3911_REG_AUX_no_crc_rx );
    /*******************************************************************************/
    
    
    /* Restore common Analog configurations for this mode */
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX) );
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX) );
    
    return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalISO15693TransceiveAnticollisionFrame( uint8_t *txBuf, uint8_t txBufLen, uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *actLen )
{
    ReturnCode            ret;
    rfalTransceiveContext ctx;
    
    /* Check if RFAL is properly initialized */
    if( (gRFAL.state < RFAL_STATE_MODE_SET) || ( gRFAL.mode != RFAL_MODE_POLL_NFCV ) )
    {
        return ERR_WRONG_STATE;
    }
    
    /*******************************************************************************/
    /* Set speficic Analog Config for Anticolission if needed */
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_ANTICOL) );

    
    /* Ignoring collisions before the UID (RES_FLAG + DSFID) */
    gRFAL.nfcvData.ignoreBits = (uint16_t)RFAL_ISO15693_IGNORE_BITS;
    
    /*******************************************************************************/
    /* Prepare for Transceive  */
    ctx.flags     = ((txBufLen==0U)?(uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL:(uint32_t)RFAL_TXRX_FLAGS_CRC_TX_AUTO) | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP | (uint32_t)RFAL_TXRX_FLAGS_AGC_OFF | ((txBufLen==0U)?(uint32_t)RFAL_TXRX_FLAGS_NFCV_FLAG_MANUAL:(uint32_t)RFAL_TXRX_FLAGS_NFCV_FLAG_AUTO); /* Disable Automatic Gain Control (AGC) for better detection of collision */
    ctx.txBuf     = txBuf;
    ctx.txBufLen  = (uint16_t)rfalConvBytesToBits(txBufLen);
    ctx.rxBuf     = rxBuf;
    ctx.rxBufLen  = (uint16_t)rfalConvBytesToBits(rxBufLen);
    ctx.rxRcvdLen = actLen;
    ctx.fwt       = rfalConv64fcTo1fc(ISO15693_FWT);
    
    rfalStartTransceive( &ctx );
    
    /*******************************************************************************/
    /* Run Transceive blocking */
    ret = rfalTransceiveRunBlockingTx();
    if( ret == ERR_NONE)
    {
        ret = rfalTransceiveBlockingRx();
    }
    
    /* Restore common Analog configurations for this mode */
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX) );
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX) );
        
    gRFAL.nfcvData.ignoreBits = 0;
    return ret;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalISO15693TransceiveEOFAnticollision( uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *actLen )
{
    uint8_t dummy;

    return rfalISO15693TransceiveAnticollisionFrame( &dummy, 0, rxBuf, rxBufLen, actLen );
}

/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalISO15693TransceiveEOF( uint8_t *rxBuf, uint8_t rxBufLen, uint16_t *actLen )
{
    ReturnCode ret;
    uint8_t    dummy;
    
    /* Check if RFAL is properly initialized */
    if( ( gRFAL.state < RFAL_STATE_MODE_SET ) || ( gRFAL.mode != RFAL_MODE_POLL_NFCV ) )
    {
        return ERR_WRONG_STATE;
    }
    
    /*******************************************************************************/
    /* Run Transceive blocking */
    ret = rfalTransceiveBlockingTxRx( &dummy,
                                      0,
                                      rxBuf,
                                      rxBufLen,
                                      actLen,
                                      ( (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP |(uint32_t)RFAL_TXRX_FLAGS_AGC_ON ),
                                      rfalConv64fcTo1fc(ISO15693_FWT) );
    return ret;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalFeliCaPoll( rfalFeliCaPollSlots slots, uint16_t sysCode, uint8_t reqCode, rfalFeliCaPollRes* pollResList, uint8_t pollResListSize, uint8_t *devicesDetected, uint8_t *collisionsDetected )
{
    ReturnCode        ret;
    uint8_t           frame[RFAL_FELICA_POLL_REQ_LEN - RFAL_FELICA_LEN_LEN];  // LEN is added by ST25R3911 automatically
    uint16_t          actLen;
    uint8_t           frameIdx;
    uint8_t           devDetected;
    uint8_t           colDetected;
    rfalEHandling     curHandling;
    uint8_t           nbSlots;
        
    /* Check if RFAL is properly initialized */
    if( (gRFAL.state < RFAL_STATE_MODE_SET) || ( gRFAL.mode != RFAL_MODE_POLL_NFCF ) )
    {
        return ERR_WRONG_STATE;
    }
    
    frameIdx    = 0;
    colDetected = 0;
    devDetected = 0;
    nbSlots     = (uint8_t)slots;
    
    /*******************************************************************************/
    /* Compute SENSF_REQ frame */
    frame[frameIdx++] = (uint8_t)FELICA_CMD_POLLING; /* CMD: SENF_REQ                       */
    frame[frameIdx++] = (uint8_t)(sysCode >> 8);     /* System Code (SC)                    */
    frame[frameIdx++] = (uint8_t)(sysCode & 0xFFU);  /* System Code (SC)                    */
    frame[frameIdx++] = reqCode;                     /* Communication Parameter Request (RC)*/
    frame[frameIdx++] = nbSlots;                     /* TimeSlot (TSN)                      */
    
    
    /*******************************************************************************/
    /* NRT should not stop on reception - Use EMVCo mode to run NRT in nrt_emv     *
     * ERRORHANDLING_EMVCO has no special handling for NFC-F mode                  */
    curHandling = gRFAL.conf.eHandling;
    rfalSetErrorHandling( RFAL_ERRORHANDLING_EMVCO );
    
    /*******************************************************************************/
    /* Run transceive blocking, 
     * Calculate Total Response Time in(64/fc): 
     *                       512 PICC process time + (n * 256 Time Slot duration)  */
    ret = rfalTransceiveBlockingTx( frame, 
                                    (uint16_t)frameIdx, 
                                    (uint8_t*)gRFAL.nfcfData.pollResponses, 
                                    RFAL_FELICA_POLL_RES_LEN, 
                                    &actLen,
                                    (RFAL_TXRX_FLAGS_DEFAULT),
                                    rfalConv64fcTo1fc( RFAL_FELICA_POLL_DELAY_TIME + (RFAL_FELICA_POLL_SLOT_TIME * ((uint32_t)nbSlots + 1U)) ) );
    
    /*******************************************************************************/
    /* If Tx OK, Wait for all responses, store them as soon as they appear         */
    if( ret == ERR_NONE )
    {
        bool timeout;

        do 
        {
            ret = rfalTransceiveBlockingRx();
            if( ret == ERR_TIMEOUT )
            {
                /* Upon timeout the full Poll Delay + (Slot time)*(nbSlots) has expired */
                timeout = true;
            }
            else
            {
                /* Reception done, reEnabled Rx for following Slot */
                st25r3911ExecuteCommand( ST25R3911_CMD_UNMASK_RECEIVE_DATA );
                st25r3911ExecuteCommand( ST25R3911_CMD_CLEAR_SQUELCH );
                
                /* If the reception was OK, new device found */
                if( ret == ERR_NONE )
                {
                   devDetected++;
                   
                   /* Overwrite the Transceive context for the next reception */
                   gRFAL.TxRx.ctx.rxBuf = (uint8_t*)gRFAL.nfcfData.pollResponses[devDetected];
                }
                /* If the reception was not OK, mark as collision */
                else
                {
                    colDetected++;
                }
                
                /* Check whether NRT has expired meanwhile  */
                timeout = st25r3911CheckReg( ST25R3911_REG_REGULATOR_RESULT, ST25R3911_REG_REGULATOR_RESULT_nrt_on, 0x00 );
                if( !timeout )
                {
                    /* Jump again into transceive Rx state for the following reception */
                    gRFAL.TxRx.status = ERR_BUSY;
                    gRFAL.state       = RFAL_STATE_TXRX;
                    gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_IDLE;
                }
            }
        }while( ((nbSlots--) != 0U) && !timeout );
    }
    
    /*******************************************************************************/
    /* Restore NRT to normal mode - back to previous error handling */
    rfalSetErrorHandling( curHandling );
    
    /*******************************************************************************/
    /* Assign output parameters if requested                                       */
    
    if( (pollResList != NULL) && (pollResListSize > 0U) && (devDetected > 0U) )
    {
        ST_MEMCPY( pollResList, gRFAL.nfcfData.pollResponses, (RFAL_FELICA_POLL_RES_LEN * (uint32_t)MIN(pollResListSize, devDetected) ) );
    }
    
    if( devicesDetected != NULL )
    {
        *devicesDetected = devDetected;
    }
    
    if( collisionsDetected != NULL )
    {
        *collisionsDetected = colDetected;
    }
    
    return (( (colDetected != 0U) || (devDetected != 0U)) ? ERR_NONE : ret);
}


/*******************************************************************************/
bool RfalRfST25R3911BClass::rfalIsExtFieldOn( void )
{
    return st25r3911IsExtFieldOn();
}

/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalListenStart( uint32_t lmMask, const rfalLmConfPA *confA, const rfalLmConfPB *confB, const rfalLmConfPF *confF, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rxLen )
{
  (void)lmMask;
  (void)confA;
  (void)confB;
  (void)confF;
  (void)rxBuf;
  (void)rxBufLen;
  (void)rxLen;
  return ERR_NOTSUPP;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalListenSleepStart( rfalLmState sleepSt, uint8_t *rxBuf, uint16_t rxBufLen, uint16_t *rxLen )
{
  (void)sleepSt;
  (void)rxBuf;
  (void)rxBufLen;
  (void)rxLen;
  return ERR_NOTSUPP;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalListenStop( void )
{
  return ERR_NOTSUPP;
}

/*******************************************************************************/
rfalLmState RfalRfST25R3911BClass::rfalListenGetState( bool *dataFlag, rfalBitRate *lastBR )
{
  (void)dataFlag;
  (void)lastBR;
  return RFAL_LM_STATE_NOT_INIT;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalListenSetState( rfalLmState newSt )
{
  (void)newSt;
  return ERR_NOTSUPP;
}


/*******************************************************************************
 *  Wake-Up Mode                                                               *
 *******************************************************************************/

/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalWakeUpModeStart( const rfalWakeUpConfig *config )
{
    uint8_t                aux;
    uint8_t                reg;
    uint32_t               irqs;
    
    /* The Wake-Up procedure is explained in detail in Application Note: AN4985 */
    
    if( config == NULL )
    {
        gRFAL.wum.cfg.period      = RFAL_WUM_PERIOD_500MS;
        gRFAL.wum.cfg.irqTout     = false;
        gRFAL.wum.cfg.swTagDetect = false;
      
        gRFAL.wum.cfg.indAmp.enabled   = true;
        gRFAL.wum.cfg.indPha.enabled   = true;
        gRFAL.wum.cfg.cap.enabled      = false;
        gRFAL.wum.cfg.indAmp.delta     = 2U;
        gRFAL.wum.cfg.indAmp.reference = RFAL_WUM_REFERENCE_AUTO;
        gRFAL.wum.cfg.indAmp.autoAvg   = false;
        gRFAL.wum.cfg.indPha.delta     = 2U;
        gRFAL.wum.cfg.indPha.reference = RFAL_WUM_REFERENCE_AUTO;
        gRFAL.wum.cfg.indPha.autoAvg   = false;
    }
    else
    {
        gRFAL.wum.cfg = *config;
    }
    
    
    /* Check for valid configuration */
    if( (gRFAL.wum.cfg.cap.enabled  && (gRFAL.wum.cfg.indAmp.enabled || gRFAL.wum.cfg.indPha.enabled))  || 
        (!gRFAL.wum.cfg.cap.enabled && !gRFAL.wum.cfg.indAmp.enabled && !gRFAL.wum.cfg.indPha.enabled)  ||
         gRFAL.wum.cfg.swTagDetect                                                                         )
    {
        return ERR_PARAM;
    }
    
    irqs = ST25R3911_IRQ_MASK_NONE;
    
    
    /* Disable Tx, Rx, External Field Detector and set default ISO14443A mode */
    st25r3911TxRxOff();
    st25r3911ClrRegisterBits( ST25R3911_REG_AUX, ST25R3911_REG_AUX_en_fd );
    st25r3911ChangeRegisterBits(ST25R3911_REG_MODE, (ST25R3911_REG_MODE_targ | ST25R3911_REG_MODE_mask_om), (ST25R3911_REG_MODE_targ_init | ST25R3911_REG_MODE_om_iso14443a) );
    
    /* Set Analog configurations for Wake-up On event */
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_WAKEUP_ON) );
    
    /*******************************************************************************/
    /* Prepare Wake-Up Timer Control Register */
    reg  = (uint8_t)(((uint8_t)gRFAL.wum.cfg.period & 0x0FU) << ST25R3911_REG_WUP_TIMER_CONTROL_shift_wut);
    reg |= (uint8_t)(((uint8_t)gRFAL.wum.cfg.period < (uint8_t)RFAL_WUM_PERIOD_100MS) ? ST25R3911_REG_WUP_TIMER_CONTROL_wur : 0x00U);
    
    if(gRFAL.wum.cfg.irqTout)
    {
        reg  |= ST25R3911_REG_WUP_TIMER_CONTROL_wto;
        irqs |= ST25R3911_IRQ_MASK_WT;
    }
    
    /*******************************************************************************/
    /* Check if Inductive Amplitude is to be performed */
    if( gRFAL.wum.cfg.indAmp.enabled )
    {
        aux  = (uint8_t)((gRFAL.wum.cfg.indAmp.delta) << ST25R3911_REG_AMPLITUDE_MEASURE_CONF_shift_am_d);
        aux |= (uint8_t)(gRFAL.wum.cfg.indAmp.aaInclMeas ? ST25R3911_REG_AMPLITUDE_MEASURE_CONF_am_aam : 0x00U);
        aux |= (uint8_t)(((uint8_t)gRFAL.wum.cfg.indAmp.aaWeight << ST25R3911_REG_AMPLITUDE_MEASURE_CONF_shift_am_aew) & ST25R3911_REG_AMPLITUDE_MEASURE_CONF_mask_am_aew);
        aux |= (uint8_t)(gRFAL.wum.cfg.indAmp.autoAvg ? ST25R3911_REG_AMPLITUDE_MEASURE_CONF_am_ae : 0x00U);
        
        st25r3911WriteRegister( ST25R3911_REG_AMPLITUDE_MEASURE_CONF, aux );
        
        /* Only need to set the reference if not using Auto Average */
        if( !gRFAL.wum.cfg.indAmp.autoAvg )
        {
            if( gRFAL.wum.cfg.indAmp.reference == RFAL_WUM_REFERENCE_AUTO )
            {
                st25r3911MeasureAmplitude( &gRFAL.wum.cfg.indAmp.reference );
            }
            st25r3911WriteRegister( ST25R3911_REG_AMPLITUDE_MEASURE_REF, gRFAL.wum.cfg.indAmp.reference );
        }
        
        reg  |= ST25R3911_REG_WUP_TIMER_CONTROL_wam;
        irqs |= ST25R3911_IRQ_MASK_WAM;
    }
    
    /*******************************************************************************/
    /* Check if Inductive Phase is to be performed */
    if( gRFAL.wum.cfg.indPha.enabled )
    {
        aux  = (uint8_t)((gRFAL.wum.cfg.indPha.delta) << ST25R3911_REG_PHASE_MEASURE_CONF_shift_pm_d);
        aux |= (uint8_t)(gRFAL.wum.cfg.indPha.aaInclMeas ? ST25R3911_REG_PHASE_MEASURE_CONF_pm_aam : 0x00U);
        aux |= (uint8_t)(((uint8_t)gRFAL.wum.cfg.indPha.aaWeight << ST25R3911_REG_PHASE_MEASURE_CONF_shift_pm_aew) & ST25R3911_REG_PHASE_MEASURE_CONF_mask_pm_aew);
        aux |= (uint8_t)(gRFAL.wum.cfg.indPha.autoAvg ? ST25R3911_REG_PHASE_MEASURE_CONF_pm_ae : 0x00U);
        
        st25r3911WriteRegister( ST25R3911_REG_PHASE_MEASURE_CONF, aux );
        
        /* Only need to set the reference if not using Auto Average */
        if( !gRFAL.wum.cfg.indPha.autoAvg )
        {
            if( gRFAL.wum.cfg.indPha.reference == RFAL_WUM_REFERENCE_AUTO )
            {
                st25r3911MeasurePhase( &gRFAL.wum.cfg.indPha.reference );
            }
            st25r3911WriteRegister( ST25R3911_REG_PHASE_MEASURE_REF, gRFAL.wum.cfg.indPha.reference );
        }
        
        reg  |= ST25R3911_REG_WUP_TIMER_CONTROL_wph;
        irqs |= ST25R3911_IRQ_MASK_WPH;
    }
    
    /*******************************************************************************/
    /* Check if Capacitive is to be performed */
    if( gRFAL.wum.cfg.cap.enabled )
    {
        /*******************************************************************************/
        /* Perform Capacitive sensor calibration */
        
        /* Disable Oscillator and Field */
        st25r3911ClrRegisterBits( ST25R3911_REG_OP_CONTROL, (ST25R3911_REG_OP_CONTROL_en | ST25R3911_REG_OP_CONTROL_tx_en) );
        
        /* Sensor gain should be configured on Analog Config */
        
        /* Perform calibration procedure */
        st25r3911CalibrateCapacitiveSensor( NULL );
        
        
        /*******************************************************************************/
        aux  = (uint8_t)((gRFAL.wum.cfg.cap.delta) << ST25R3911_REG_CAPACITANCE_MEASURE_CONF_shift_cm_d);
        aux |= (uint8_t)(gRFAL.wum.cfg.cap.aaInclMeas ? ST25R3911_REG_CAPACITANCE_MEASURE_CONF_cm_aam : 0x00U);
        aux |= (uint8_t)(((uint8_t)gRFAL.wum.cfg.cap.aaWeight << ST25R3911_REG_CAPACITANCE_MEASURE_CONF_shift_cm_aew) & ST25R3911_REG_CAPACITANCE_MEASURE_CONF_mask_cm_aew);
        aux |= (uint8_t)(gRFAL.wum.cfg.cap.autoAvg ? ST25R3911_REG_CAPACITANCE_MEASURE_CONF_cm_ae : 0x00U);
        
        st25r3911WriteRegister( ST25R3911_REG_CAPACITANCE_MEASURE_CONF, aux );
        
        /* Only need to set the reference if not using Auto Average */
        if( !gRFAL.wum.cfg.cap.autoAvg )
        {
            if( gRFAL.wum.cfg.indPha.reference == RFAL_WUM_REFERENCE_AUTO )
            {
                st25r3911MeasureCapacitance( &gRFAL.wum.cfg.cap.reference );
            }
            st25r3911WriteRegister( ST25R3911_REG_CAPACITANCE_MEASURE_REF, gRFAL.wum.cfg.cap.reference );
        }
        
        reg  |= ST25R3911_REG_WUP_TIMER_CONTROL_wcap;
        irqs |= ST25R3911_IRQ_MASK_WCAP;
    }
    
    /* Disable and clear all interrupts except Wake-Up IRQs */
    st25r3911DisableInterrupts( ST25R3911_IRQ_MASK_ALL );
    st25r3911GetInterrupt( irqs );
    st25r3911EnableInterrupts( irqs );
    
    /* Enable Low Power Wake-Up Mode */
    st25r3911WriteRegister( ST25R3911_REG_WUP_TIMER_CONTROL, reg );
    st25r3911ChangeRegisterBits( ST25R3911_REG_OP_CONTROL, (ST25R3911_REG_OP_CONTROL_en | ST25R3911_REG_OP_CONTROL_wu), ST25R3911_REG_OP_CONTROL_wu );
    
    gRFAL.wum.state = RFAL_WUM_STATE_ENABLED;
    gRFAL.state     = RFAL_STATE_WUM;  
      
    return ERR_NONE;
}


/*******************************************************************************/
bool RfalRfST25R3911BClass::rfalWakeUpModeHasWoke( void )
{   
    return (gRFAL.wum.state >= RFAL_WUM_STATE_ENABLED_WOKE);
}


/*******************************************************************************/
void RfalRfST25R3911BClass::rfalRunWakeUpModeWorker( void )
{
    uint32_t irqs;
    
    if( gRFAL.state != RFAL_STATE_WUM )
    {
        return;
    }
    
    switch( gRFAL.wum.state )
    {
        case RFAL_WUM_STATE_ENABLED:
        case RFAL_WUM_STATE_ENABLED_WOKE:
            
            irqs = st25r3911GetInterrupt( ( ST25R3911_IRQ_MASK_WT | ST25R3911_IRQ_MASK_WAM | ST25R3911_IRQ_MASK_WPH | ST25R3911_IRQ_MASK_WCAP ) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
               break;  /* No interrupt to process */
            }
            
            /*******************************************************************************/
            /* Check and mark which measurement(s) cause interrupt */
            if((irqs & ST25R3911_IRQ_MASK_WAM) != 0U)
            {
                gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
            }
            
            if((irqs & ST25R3911_IRQ_MASK_WPH) != 0U)
            {
                gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
            }
            
            if((irqs & ST25R3911_IRQ_MASK_WCAP) != 0U)
            {
                gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
            }
            break;
            
        default:
            /* MISRA 16.4: no empty default statement (a comment being enough) */
            break;
    }
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalWakeUpModeStop( void )
{
    if( gRFAL.wum.state == RFAL_WUM_STATE_NOT_INIT )
    {
        return ERR_WRONG_STATE;
    }
    
    gRFAL.wum.state = RFAL_WUM_STATE_NOT_INIT;
    
    /* Re-Enable External Field Detector */
    st25r3911SetRegisterBits( ST25R3911_REG_AUX, ST25R3911_REG_AUX_en_fd );
    
    /* Disable Wake-Up Mode */
    st25r3911ClrRegisterBits( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_wu );
    st25r3911DisableInterrupts( (ST25R3911_IRQ_MASK_WT | ST25R3911_IRQ_MASK_WAM | ST25R3911_IRQ_MASK_WPH | ST25R3911_IRQ_MASK_WCAP) );
    
    /* Re-Enable the Oscillator */
    st25r3911OscOn();
    
    /* Set Analog configurations for Wake-up Off event */
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_WAKEUP_OFF) );
      
    return ERR_NONE;
}


/*******************************************************************************
 *  RF Chip                                                                    *
 *******************************************************************************/
 
/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipWriteReg( uint16_t reg, const uint8_t* values, uint8_t len )
{
    if( !st25r3911IsRegValid( (uint8_t)reg) )
    {
        return ERR_PARAM;
    }
    
    st25r3911WriteMultipleRegisters( (uint8_t)reg, values, len );
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipReadReg( uint16_t reg, uint8_t* values, uint8_t len )
{
    if( !st25r3911IsRegValid( (uint8_t)reg) )
    {
        return ERR_PARAM;
    }
    
    st25r3911ReadMultipleRegisters( (uint8_t)reg, values, len );
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipExecCmd( uint16_t cmd )
{
    if( !st25r3911IsCmdValid( (uint8_t)cmd) )
    {
        return ERR_PARAM;
    }
    
    st25r3911ExecuteCommand( (uint8_t) cmd );
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipWriteTestReg( uint16_t reg, uint8_t value )
{
    st25r3911WriteTestRegister( (uint8_t)reg, value );
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipReadTestReg( uint16_t reg, uint8_t* value )
{
    st25r3911ReadTestRegister( (uint8_t)reg, value );
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipChangeRegBits( uint16_t reg, uint8_t valueMask, uint8_t value )
{
    st25r3911ChangeRegisterBits( (uint8_t)reg, valueMask, value );
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipChangeTestRegBits( uint16_t reg, uint8_t valueMask, uint8_t value )
{
    st25r3911ChangeTestRegisterBits( (uint8_t)reg, valueMask, value );
    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipSetRFO( uint8_t rfo )
{
    st25r3911WriteRegister( ST25R3911_REG_RFO_AM_OFF_LEVEL, rfo );

    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipGetRFO( uint8_t* result )
{
    st25r3911ReadRegister(ST25R3911_REG_RFO_AM_ON_LEVEL, result);

    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipMeasureAmplitude( uint8_t* result )
{
    st25r3911MeasureAmplitude( result );

    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipMeasurePhase( uint8_t* result )
{
    st25r3911MeasurePhase( result );

    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipMeasureCapacitance( uint8_t* result )
{
    st25r3911MeasureCapacitance( result );

    return ERR_NONE;
}


/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::rfalChipMeasurePowerSupply( uint8_t param, uint8_t* result )
{
    *result = st25r3911MeasurePowerSupply( param );

    return ERR_NONE;
}



/*******************************************************************************/

/* All bitrates defined in ST25R3911B registers are nibbles. This rfal code 
 * up there only works if equality to values of enum rfalBitrate is guaranteed: */
extern uint8_t  equalityGuard_RFAL_BR_106[(ST25R3911_REG_BIT_RATE_rxrate_106==(uint8_t)RFAL_BR_106)?1:(-1)];
extern uint8_t  equalityGuard_RFAL_BR_212[(ST25R3911_REG_BIT_RATE_rxrate_212==(uint8_t)RFAL_BR_212)?1:(-1)];
extern uint8_t  equalityGuard_RFAL_BR_424[(ST25R3911_REG_BIT_RATE_rxrate_424==(uint8_t)RFAL_BR_424)?1:(-1)];
extern uint8_t  equalityGuard_RFAL_BR_848[(ST25R3911_REG_BIT_RATE_rxrate_848==(uint8_t)RFAL_BR_848)?1:(-1)];
extern uint8_t  equalityGuard_RFAL_BR_1695[(ST25R3911_REG_BIT_RATE_rxrate_1695==(uint8_t)RFAL_BR_1695)?1:(-1)];
extern uint8_t  equalityGuard_RFAL_BR_3390[(ST25R3911_REG_BIT_RATE_rxrate_3390==(uint8_t)RFAL_BR_3390)?1:(-1)];
extern uint8_t  equalityGuard_RFAL_BR_6780[(ST25R3911_REG_BIT_RATE_rxrate_6780==(uint8_t)RFAL_BR_6780)?1:(-1)];
