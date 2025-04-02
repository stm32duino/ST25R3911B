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
 *  \brief ST25R3911 high level interface
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "rfal_rfst25r3911.h"
#include "st25r3911.h"
#include "st25r3911_com.h"
#include "st25r3911_interrupt.h"
#include "nfc_utils.h"

/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/

#define ST25R3911_OSC_STABLE_TIMEOUT           10U /*!< Timeout for Oscillator to get stable, datasheet: 700us, take 5 ms */
#define ST25R3911_CA_TIMEOUT                   10U /*!< Timeout for Collision Avoidance command                           */
#define ST25R3911_TOUT_CALIBRATE_CAP_SENSOR    4U  /*!< Max duration Calibrate Capacitive Sensor command   Datasheet: 3ms */

/*
******************************************************************************
* LOCAL CONSTANTS
******************************************************************************
*/

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

void RfalRfST25R3911BClass::st25r3911TxRxOn(void)
{
  st25r3911SetRegisterBits(ST25R3911_REG_OP_CONTROL, (ST25R3911_REG_OP_CONTROL_rx_en | ST25R3911_REG_OP_CONTROL_tx_en));
}

void RfalRfST25R3911BClass::st25r3911TxRxOff(void)
{
  st25r3911ClrRegisterBits(ST25R3911_REG_OP_CONTROL, (ST25R3911_REG_OP_CONTROL_rx_en | ST25R3911_REG_OP_CONTROL_tx_en));
}


void RfalRfST25R3911BClass::st25r3911OscOn(void)
{
  /* Check if oscillator is already turned on and stable                                                */
  /* Use ST25R3911_REG_OP_CONTROL_en instead of ST25R3911_REG_AUX_DISPLAY_osc_ok to be on the safe side */
  if (!st25r3911CheckReg(ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_en, ST25R3911_REG_OP_CONTROL_en)) {
    /* Clear any eventual previous oscillator IRQ */
    st25r3911GetInterrupt(ST25R3911_IRQ_MASK_OSC);

    /* enable oscillator frequency stable interrupt */
    st25r3911EnableInterrupts(ST25R3911_IRQ_MASK_OSC);

    /* enable oscillator and regulator output */
    st25r3911ModifyRegister(ST25R3911_REG_OP_CONTROL, 0x00, ST25R3911_REG_OP_CONTROL_en);

    /* wait for the oscillator interrupt */
    st25r3911WaitForInterruptsTimed(ST25R3911_IRQ_MASK_OSC, ST25R3911_OSC_STABLE_TIMEOUT);
    st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_OSC);
  }

}


void RfalRfST25R3911BClass::st25r3911Initialize(void)
{
  uint16_t vdd_mV;

  /* first, reset the st25r3911 */
  st25r3911ExecuteCommand(ST25R3911_CMD_SET_DEFAULT);

  /* Set Operation Control Register to default value */
  st25r3911WriteRegister(ST25R3911_REG_OP_CONTROL, 0x00);

  /* enable pull downs on miso line */
  st25r3911ModifyRegister(ST25R3911_REG_IO_CONF2, 0,
                          ST25R3911_REG_IO_CONF2_miso_pd1 |
                          ST25R3911_REG_IO_CONF2_miso_pd2);

  /* after reset all interrupts are enabled. so disable them at first */
  st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_ALL);
  /* and clear them, just to be sure... */
  st25r3911ClearInterrupts();

  st25r3911OscOn();

  /* Measure vdd and set sup3V bit accordingly */
  vdd_mV = st25r3911MeasureVoltage(ST25R3911_REG_REGULATOR_CONTROL_mpsv_vdd);

  st25r3911ModifyRegister(ST25R3911_REG_IO_CONF2,
                          ST25R3911_REG_IO_CONF2_sup3V,
                          (uint8_t)((vdd_mV < 3600U) ? ST25R3911_REG_IO_CONF2_sup3V : 0U));

  /* Make sure Transmitter and Receiver are disabled */
  st25r3911TxRxOff();

  return;
}

void RfalRfST25R3911BClass::st25r3911Deinitialize(void)
{
  st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_ALL);

  /* Disable Tx and Rx, Keep OSC */
  st25r3911TxRxOff();

  return;
}

ReturnCode RfalRfST25R3911BClass::st25r3911AdjustRegulators(uint16_t *result_mV)
{
  uint8_t result;
  uint8_t io_conf2;
  ReturnCode err = ERR_NONE;

  /* Reset logic and set regulated voltages to be defined by result of Adjust Regulators command */
  st25r3911SetRegisterBits(ST25R3911_REG_REGULATOR_CONTROL, ST25R3911_REG_REGULATOR_CONTROL_reg_s);
  st25r3911ClrRegisterBits(ST25R3911_REG_REGULATOR_CONTROL, ST25R3911_REG_REGULATOR_CONTROL_reg_s);

  st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_ADJUST_REGULATORS,
                                      ST25R3911_REG_REGULATOR_RESULT,
                                      5,
                                      &result);

  st25r3911ReadRegister(ST25R3911_REG_IO_CONF2, &io_conf2);

  result >>= ST25R3911_REG_REGULATOR_RESULT_shift_reg;
  result -= 5U;
  if (result_mV != NULL) {
    if ((io_conf2 & ST25R3911_REG_IO_CONF2_sup3V) != 0U) {
      *result_mV = 2400;
      *result_mV += (uint16_t)result * 100U;
    } else {
      *result_mV = 3900;
      *result_mV += (uint16_t)result * 120U;
    }
  }
  return err;
}

void RfalRfST25R3911BClass::st25r3911MeasureAmplitude(uint8_t *result)
{
  st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_MEASURE_AMPLITUDE,
                                      ST25R3911_REG_AD_RESULT,
                                      10,
                                      result);
}

void RfalRfST25R3911BClass::st25r3911MeasurePhase(uint8_t *result)
{
  st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_MEASURE_PHASE,
                                      ST25R3911_REG_AD_RESULT,
                                      10,
                                      result);
}

void RfalRfST25R3911BClass::st25r3911MeasureCapacitance(uint8_t *result)
{
  st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_MEASURE_CAPACITANCE,
                                      ST25R3911_REG_AD_RESULT,
                                      10,
                                      result);
}

void RfalRfST25R3911BClass::st25r3911CalibrateAntenna(uint8_t *result)
{
  st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_CALIBRATE_ANTENNA,
                                      ST25R3911_REG_ANT_CAL_RESULT,
                                      10,
                                      result);
}

void RfalRfST25R3911BClass::st25r3911CalibrateModulationDepth(uint8_t *result)
{
  st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_CALIBRATE_MODULATION,
                                      ST25R3911_REG_AM_MOD_DEPTH_RESULT,
                                      10,
                                      result);
}


ReturnCode RfalRfST25R3911BClass::st25r3911CalibrateCapacitiveSensor(uint8_t *result)
{
  ReturnCode ret;
  uint8_t    res;

  /* Clear Manual calibration values to enable automatic calibration mode */
  st25r3911ClrRegisterBits(ST25R3911_REG_CAP_SENSOR_CONTROL, ST25R3916_REG_CAP_SENSOR_CONTROL_mask_cs_mcal);

  /* Execute automatic calibration */
  ret = st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_CALIBRATE_C_SENSOR, ST25R3911_REG_CAP_SENSOR_RESULT, ST25R3911_TOUT_CALIBRATE_CAP_SENSOR, &res);

  /* Check whether the calibration was successull */
  if (((res & ST25R3911_REG_CAP_SENSOR_RESULT_cs_cal_end) != ST25R3911_REG_CAP_SENSOR_RESULT_cs_cal_end) ||
      ((res & ST25R3911_REG_CAP_SENSOR_RESULT_cs_cal_err) == ST25R3911_REG_CAP_SENSOR_RESULT_cs_cal_err) || (ret != ERR_NONE)) {
    return ERR_IO;
  }

  if (result != NULL) {
    (*result) = (uint8_t)(res >> ST25R3911_REG_CAP_SENSOR_CONTROL_shift_cs_mcal);
  }

  return ERR_NONE;
}


ReturnCode RfalRfST25R3911BClass::st25r3911SetBitrate(uint8_t txRate, uint8_t rxRate)
{
  uint8_t reg;

  st25r3911ReadRegister(ST25R3911_REG_BIT_RATE, &reg);
  if (rxRate != ST25R3911_BR_DO_NOT_SET) {
    if (rxRate > ST25R3911_BR_3390) {
      return ERR_PARAM;
    } else {
      reg = (uint8_t)(reg & ~ST25R3911_REG_BIT_RATE_mask_rxrate);     /* MISRA 10.3 */
      reg |= rxRate << ST25R3911_REG_BIT_RATE_shift_rxrate;
    }
  }
  if (txRate != ST25R3911_BR_DO_NOT_SET) {
    if (txRate > ST25R3911_BR_6780) {
      return ERR_PARAM;
    } else {
      reg = (uint8_t)(reg & ~ST25R3911_REG_BIT_RATE_mask_txrate);     /* MISRA 10.3 */
      reg |= txRate << ST25R3911_REG_BIT_RATE_shift_txrate;
    }
  }
  st25r3911WriteRegister(ST25R3911_REG_BIT_RATE, reg);

  return ERR_NONE;
}

uint8_t RfalRfST25R3911BClass::st25r3911MeasurePowerSupply(uint8_t mpsv)
{
  uint8_t result;

  /* Set the source of direct command: Measure Power Supply Voltage */
  st25r3911ChangeRegisterBits(ST25R3911_REG_REGULATOR_CONTROL, ST25R3911_REG_REGULATOR_CONTROL_mask_mpsv, mpsv);

  /* Execute command: Measure Power Supply Voltage */
  st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_MEASURE_VDD, ST25R3911_REG_AD_RESULT, 10, &result);

  return result;
}

uint16_t RfalRfST25R3911BClass::st25r3911MeasureVoltage(uint8_t mpsv)
{
  uint8_t result;
  uint16_t mV;

  result = st25r3911MeasurePowerSupply(mpsv);

  mV = ((uint16_t)result) * 23U;
  mV += ((((uint16_t)result) * 438U) + 500U) / 1000U;

  return mV;
}


uint8_t RfalRfST25R3911BClass::st25r3911GetNumFIFOLastBits(void)
{
  uint8_t  reg;

  st25r3911ReadRegister(ST25R3911_REG_FIFO_RX_STATUS2, &reg);

  return ((reg & ST25R3911_REG_FIFO_RX_STATUS2_mask_fifo_lb) >> ST25R3911_REG_FIFO_RX_STATUS2_shift_fifo_lb);
}

uint32_t RfalRfST25R3911BClass::st25r3911GetNoResponseTime_64fcs(void)
{
  return st25r3911NoResponseTime_64fcs;
}

void RfalRfST25R3911BClass::st25r3911StartGPTimer_8fcs(uint16_t gpt_8fcs, uint8_t trigger_source)
{
  st25r3911SetGPTime_8fcs(gpt_8fcs);

  st25r3911ModifyRegister(ST25R3911_REG_GPT_CONTROL,
                          ST25R3911_REG_GPT_CONTROL_gptc_mask,
                          trigger_source);
  if (trigger_source == 0U) {
    st25r3911ExecuteCommand(ST25R3911_CMD_START_GP_TIMER);
  }

  return;
}

void RfalRfST25R3911BClass::st25r3911SetGPTime_8fcs(uint16_t gpt_8fcs)
{
  st25r3911WriteRegister(ST25R3911_REG_GPT1, (uint8_t)(gpt_8fcs >> 8));
  st25r3911WriteRegister(ST25R3911_REG_GPT2, (uint8_t)(gpt_8fcs & 0xffU));

  return;
}

bool RfalRfST25R3911BClass::st25r3911CheckReg(uint8_t reg, uint8_t mask, uint8_t value)
{
  uint8_t regVal;

  regVal = 0;
  st25r3911ReadRegister(reg, &regVal);

  return ((regVal & mask) == value);
}


bool RfalRfST25R3911BClass::st25r3911CheckChipID(uint8_t *rev)
{
  uint8_t ID;

  ID = 0;
  st25r3911ReadRegister(ST25R3911_REG_IC_IDENTITY, &ID);

  /* Check if IC Identity Register contains ST25R3911's IC type code */
  if ((ID & ST25R3911_REG_IC_IDENTITY_mask_ic_type) != ST25R3911_REG_IC_IDENTITY_ic_type) {
    return false;
  }

  if (rev != NULL) {
    *rev = (ID & ST25R3911_REG_IC_IDENTITY_mask_ic_rev);
  }

  return true;
}

ReturnCode RfalRfST25R3911BClass::st25r3911SetNoResponseTime_64fcs(uint32_t nrt_64fcs)
{
  ReturnCode err = ERR_NONE;
  uint8_t nrt_step = 0;
  uint32_t noResponseTime_64fcs = nrt_64fcs;      /* MISRA 17.8: Use intermediate variable */

  st25r3911NoResponseTime_64fcs = noResponseTime_64fcs;
  if (noResponseTime_64fcs > (uint32_t)0xFFFFU) {
    nrt_step = ST25R3911_REG_GPT_CONTROL_nrt_step;
    noResponseTime_64fcs = (noResponseTime_64fcs + 63U) / 64U;
    if (noResponseTime_64fcs > (uint32_t)0xFFFFU) {
      noResponseTime_64fcs = 0xFFFFU;
      err = ERR_PARAM;
    }
    st25r3911NoResponseTime_64fcs = 64U * noResponseTime_64fcs;
  }

  st25r3911ModifyRegister(ST25R3911_REG_GPT_CONTROL, ST25R3911_REG_GPT_CONTROL_nrt_step, nrt_step);
  st25r3911WriteRegister(ST25R3911_REG_NO_RESPONSE_TIMER1, (uint8_t)(noResponseTime_64fcs >> 8));
  st25r3911WriteRegister(ST25R3911_REG_NO_RESPONSE_TIMER2, (uint8_t)(noResponseTime_64fcs & 0xffU));

  return err;
}

ReturnCode RfalRfST25R3911BClass::st25r3911SetStartNoResponseTime_64fcs(uint32_t nrt_64fcs)
{
  ReturnCode err;

  err = st25r3911SetNoResponseTime_64fcs(nrt_64fcs);
  if (err == ERR_NONE) {
    st25r3911ExecuteCommand(ST25R3911_CMD_START_NO_RESPONSE_TIMER);
  }

  return err;
}

ReturnCode RfalRfST25R3911BClass::st25r3911PerformCollisionAvoidance(uint8_t FieldONCmd, uint8_t pdThreshold, uint8_t caThreshold, uint8_t nTRFW)
{
  uint8_t  treMask;
  uint32_t irqs;

  if ((FieldONCmd != ST25R3911_CMD_INITIAL_RF_COLLISION)    &&
      (FieldONCmd != ST25R3911_CMD_RESPONSE_RF_COLLISION_0) &&
      (FieldONCmd != ST25R3911_CMD_RESPONSE_RF_COLLISION_N)) {
    return ERR_PARAM;
  }

  /* Check if new thresholds are to be applied */
  if ((pdThreshold != ST25R3911_THRESHOLD_DO_NOT_SET) || (caThreshold != ST25R3911_THRESHOLD_DO_NOT_SET)) {
    treMask = 0;

    if (pdThreshold != ST25R3911_THRESHOLD_DO_NOT_SET) {
      treMask |= ST25R3911_REG_FIELD_THRESHOLD_mask_trg;
    }

    if (caThreshold != ST25R3911_THRESHOLD_DO_NOT_SET) {
      treMask |= ST25R3911_REG_FIELD_THRESHOLD_mask_rfe;
    }

    /* Set Detection Threshold and|or Collision Avoidance Threshold */
    st25r3911ChangeRegisterBits(ST25R3911_REG_FIELD_THRESHOLD, treMask, (pdThreshold & ST25R3911_REG_FIELD_THRESHOLD_mask_trg) | (caThreshold & ST25R3911_REG_FIELD_THRESHOLD_mask_rfe));
  }

  /* Set n x TRFW */
  st25r3911ModifyRegister(ST25R3911_REG_AUX, ST25R3911_REG_AUX_mask_nfc_n, (nTRFW & ST25R3911_REG_AUX_mask_nfc_n));

  /* Enable and clear CA specific interrupts and execute command */
  st25r3911EnableInterrupts((ST25R3911_IRQ_MASK_CAC | ST25R3911_IRQ_MASK_CAT));
  st25r3911GetInterrupt((ST25R3911_IRQ_MASK_CAC | ST25R3911_IRQ_MASK_CAT));

  st25r3911ExecuteCommand(FieldONCmd);

  irqs = st25r3911WaitForInterruptsTimed(ST25R3911_IRQ_MASK_CAC | ST25R3911_IRQ_MASK_CAT, ST25R3911_CA_TIMEOUT);

  /* Clear any previous External Field events and disable CA specific interrupts */
  st25r3911GetInterrupt((ST25R3911_IRQ_MASK_EOF | ST25R3911_IRQ_MASK_EON));
  st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_CAC | ST25R3911_IRQ_MASK_CAT);


  if ((ST25R3911_IRQ_MASK_CAC & irqs) != 0U) {                          /* Collision occurred */
    return ERR_RF_COLLISION;
  }

  if ((ST25R3911_IRQ_MASK_CAT & irqs) != 0U) {                          /* No Collision detected, Field On */
    return ERR_NONE;
  }

  /* No interrupt detected */
  return ERR_INTERNAL;
}

ReturnCode RfalRfST25R3911BClass::st25r3911GetRegsDump(uint8_t *resRegDump, uint8_t *sizeRegDump)
{
  uint8_t regIt;
  uint8_t regDump[ST25R3911_REG_IC_IDENTITY + 1U];

  if (!sizeRegDump || !resRegDump) {
    return ERR_PARAM;
  }

  for (regIt = ST25R3911_REG_IO_CONF1; regIt < SIZEOF_ARRAY(regDump); regIt++) {
    st25r3911ReadRegister(regIt, &regDump[regIt]);
  }

  *sizeRegDump = MIN(*sizeRegDump, regIt);
  if (*sizeRegDump > 0U) {                                  /* MISRA 21.18 */
    ST_MEMCPY(resRegDump, regDump, *sizeRegDump);
  }

  return ERR_NONE;
}


void RfalRfST25R3911BClass::st25r3911SetNumTxBits(uint32_t nBits)
{
  st25r3911WriteRegister(ST25R3911_REG_NUM_TX_BYTES2, (uint8_t)((nBits >> 0) & 0xffU));
  st25r3911WriteRegister(ST25R3911_REG_NUM_TX_BYTES1, (uint8_t)((nBits >> 8) & 0xffU));
}


bool RfalRfST25R3911BClass::st25r3911IsCmdValid(uint8_t cmd)
{
  if (!((cmd >= ST25R3911_CMD_SET_DEFAULT)       && (cmd <= ST25R3911_CMD_ANALOG_PRESET))           &&
      !((cmd >= ST25R3911_CMD_MASK_RECEIVE_DATA) && (cmd <= ST25R3911_CMD_CLEAR_RSSI))              &&
      !((cmd >= ST25R3911_CMD_TRANSPARENT_MODE)  && (cmd <= ST25R3911_CMD_START_NO_RESPONSE_TIMER)) &&
      !((cmd >= ST25R3911_CMD_TEST_CLEARA)       && (cmd <= ST25R3911_CMD_FUSE_PPROM))) {
    return false;
  }
  return true;
}

ReturnCode RfalRfST25R3911BClass::st25r3911StreamConfigure(const struct st25r3911StreamConfig *config)
{
  uint8_t smd = 0;
  uint8_t mode;

  if (config->useBPSK != 0U) {
    mode = ST25R3911_REG_MODE_om_bpsk_stream;
    if ((config->din < 2U) || (config->din > 4U)) { /* not in fc/4 .. fc/16 */
      return ERR_PARAM;
    }
    smd |= (4U - config->din) << ST25R3911_REG_STREAM_MODE_shift_scf;

  } else {
    mode = ST25R3911_REG_MODE_om_subcarrier_stream;
    if ((config->din < 3U) || (config->din > 6U)) { /* not in fc/8 .. fc/64 */
      return ERR_PARAM;
    }
    smd |= (6U - config->din) << ST25R3911_REG_STREAM_MODE_shift_scf;
    if (config->report_period_length == 0U) {
      return ERR_PARAM;
    }
  }

  if ((config->dout < 1U) || (config->dout > 7U)) { /* not in fc/2 .. fc/128 */
    return ERR_PARAM;
  }
  smd |= (7U - config->dout) << ST25R3911_REG_STREAM_MODE_shift_stx;

  if (config->report_period_length > 3U) {
    return ERR_PARAM;
  }
  smd |= config->report_period_length << ST25R3911_REG_STREAM_MODE_shift_scp;

  st25r3911WriteRegister(ST25R3911_REG_STREAM_MODE, smd);
  st25r3911ChangeRegisterBits(ST25R3911_REG_MODE, ST25R3911_REG_MODE_mask_om, mode);

  return ERR_NONE;
}

/*******************************************************************************/
ReturnCode RfalRfST25R3911BClass::st25r3911GetRSSI(uint16_t *amRssi, uint16_t *pmRssi)
{
  /*******************************************************************************/
  /* MISRA 8.9 An object should be defined at block scope if its identifier only appears in a single function */
  /*< ST25R3911  RSSI Display Reg values:0  1   2   3   4   5   6    7    8   9    a     b    c    d  e  f    */
  const uint16_t st25r3911Rssi2mV[] = { 0, 20, 27, 37, 52, 72, 99, 136, 190, 262, 357, 500, 686, 950, 0, 0 };

  /* ST25R3911 2/3 stage gain reduction [dB]   0    0    0    0    0    3    6    9   12   15   18  na na na na na */
  const uint16_t st25r3911Gain2Percent[] = { 100, 100, 100, 100, 100, 141, 200, 281, 398, 562, 794, 1, 1, 1, 1, 1 };
  /*******************************************************************************/

  uint8_t  rssi;
  uint8_t  gainRed;

  st25r3911ReadRegister(ST25R3911_REG_RSSI_RESULT, &rssi);
  st25r3911ReadRegister(ST25R3911_REG_GAIN_RED_STATE, &gainRed);

  if (amRssi != NULL) {
    *amRssi = (uint16_t)(((uint32_t)st25r3911Rssi2mV[(rssi >> ST25R3911_REG_RSSI_RESULT_shift_rssi_am) ] * (uint32_t)st25r3911Gain2Percent[(gainRed >> ST25R3911_REG_GAIN_RED_STATE_shift_gs_am) ]) / 100U);
  }

  if (pmRssi != NULL) {
    *pmRssi = (uint16_t)(((uint32_t)st25r3911Rssi2mV[(rssi & ST25R3911_REG_RSSI_RESULT_mask_rssi_pm) ] * (uint32_t)st25r3911Gain2Percent[(gainRed & ST25R3911_REG_GAIN_RED_STATE_mask_gs_pm) ]) / 100U);
  }

  return ERR_NONE;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
/*!
 *****************************************************************************
 *  \brief  Executes a direct command and returns the result
 *
 *  This function executes the direct command given by \a cmd waits for
 *  \a sleeptime for I_dct and returns the result read from register \a resreg.
 *  No checking of the validity of the cmd is performed.
 *
 *  \param[in] cmd: direct command to execute.
 *  \param[in] resreg: Address of the register containing the result.
 *  \param[in] sleeptime: time in milliseconds to wait before reading the result.
 *  \param[out] result: 8 bit long result
 *
 *****************************************************************************
 */
ReturnCode RfalRfST25R3911BClass::st25r3911ExecuteCommandAndGetResult(uint8_t cmd, uint8_t resreg, uint8_t sleeptime, uint8_t *result)
{

  st25r3911EnableInterrupts(ST25R3911_IRQ_MASK_DCT);
  st25r3911GetInterrupt(ST25R3911_IRQ_MASK_DCT);
  st25r3911ExecuteCommand(cmd);
  st25r3911WaitForInterruptsTimed(ST25R3911_IRQ_MASK_DCT, sleeptime);
  st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_DCT);

  /* read out the result if the pointer is not NULL */
  if (result != NULL) {
    st25r3911ReadRegister(resreg, result);
  }

  return ERR_NONE;
}
