#include <stdio.h>
#include "sf-l6470.h"

using namespace std;

SFL6470::~SFL6470()
{
    //dtor
}

/** @brief init - Setup the initial state of the object
  *
  */
void SFL6470::init(SPI& bus)
{
    m_bus = &bus;
    m_bus->setMode(3);
    m_bus->setBPW(8);
    m_invertDir = 0;
    m_msMode = 128;     // Power on default
}




///////////////////////////////////////////////////////////////////////////////////////////////////
// Set Functions
//

/** @brief setParam - Write parameter fields to dSPIN.
  *
  * Realize the "set parameter" function, to write to the various registers in
  * the dSPIN chip.
  */
void SFL6470::setParam(uint8_t param, uint32_t value)
{
    dSPIN_Xfer(dSPIN_SET_PARAM | param);
    paramHandler(param, value);
}

/** @brief getAccel - Set the dSPIN acceleration value.
  *
  * Sets the acceleration value in steps per second per second.  Translates
  * to the dSPIN's 12 bit value.  Returns the calculated register value.
  *
  * @spss - The acceleration value in steps per second per second.
  */
uint32_t SFL6470::setAccel(float spss)
{
    float temp = spss * 0.137438;
    uint32_t regVal = (uint32_t) long(temp);

    if(regVal > 0x00000FFF)
        regVal = 0x00000FFF;

    setParam(dSPIN_ACC, regVal);

    return regVal;
}

/** @brief getDecel - Set the dSPIN deceleration value.
  *
  * Sets the deceleration value in steps per second per second.  Translates
  * to the dSPIN's 12 bit value.
  *
  * @spss - The deceleration value in steps per second per second.
  */
uint32_t SFL6470::setDecel(float spss)
{
    float temp = spss * 0.137438;
    uint32_t regVal = (uint32_t) long(temp);

    if(regVal > 0x00000FFF)
        regVal = 0x00000FFF;

    setParam(dSPIN_DEC, regVal);

    return regVal;
}

/** @brief setMaxSpeed - Set the dSPIN maximum speed value.
  *
  * Sets the maximum speed cap for the motor.  Translates the value to the native
  * 10 bit value of the dSPIN register.
  */
uint32_t SFL6470::setMaxSpeed(float sps)
{
    float temp = sps * .065536;
    uint32_t regVal = (unsigned long) long(temp);
    if( regVal > 0x000003FF)
        regVal = 0x000003FF;

    setParam(dSPIN_MAX_SPEED, regVal);

    return regVal;
}

/** @brief setMicroSteps - Sets the number of microsteps per full step.
  *
  * Sets the microstep mode for the dSPIN chip.  Valid values are 0-128 in
  * powers of 2.  Invalid values are rounded down to next lower valid value.
  * returns the previous setting.
  *
  * @val - integer valuse 0 to 128
  */
int SFL6470::setMicroSteps(int val)
{
    int oldVal = m_msMode;
    int regVal = 0;
    if (val >= 128)
    {
        regVal = dSPIN_STEP_SEL_1_128;
        m_msMode = 128;
    }
    else if (val >= 64)
    {
        regVal = dSPIN_STEP_SEL_1_64;
        m_msMode = 64;
    }
    else if (val >= 32)
    {
        regVal = dSPIN_STEP_SEL_1_32;
        m_msMode = 32;
    }
    else if (val >= 16)
    {
        regVal = dSPIN_STEP_SEL_1_16;
        m_msMode = 16;
    }
    else if (val >= 8)
    {
        regVal = dSPIN_STEP_SEL_1_8;
        m_msMode = 8;
    }
    else if (val >= 4)
    {
        regVal = dSPIN_STEP_SEL_1_4;
        m_msMode = 4;
    }
    else if (val >= 2)
    {
        regVal = dSPIN_STEP_SEL_1_2;
        m_msMode = 2;
    }
    else
    {
       regVal = dSPIN_STEP_SEL_1;
       m_msMode = 1;
    }

    setParam(dSPIN_STEP_MODE, regVal);
    if (getError() == dSPIN_ERR_NOEXEC) m_msMode = oldVal;
    return oldVal;
}




///////////////////////////////////////////////////////////////////////////////////////////////////
// Get functions
//

/** @brief getParam - Read parameter fields from dSPIN.
  *
  * Realize the "get parameter" function, to read the various registers in
  * the dSPIN chip.
  */
uint32_t SFL6470::getParam(uint8_t param)
{
    dSPIN_Xfer(dSPIN_GET_PARAM | param);
    return paramHandler(param, 0);
}

/** @brief isBusy - Check the status register for busy.
  *
  * Returns the motor status value if the driver is busy.  Otherwise
  * returns 0.
  */
uint8_t SFL6470::isBusy()
{
    uint32_t temp = getParam(dSPIN_STATUS);
    uint32_t busy = temp & dSPIN_STATUS_BUSY;

    if (busy == 0)
    {
        temp &= dSPIN_STATUS_MOT_STATUS;
        temp >>= 5;
        //if (temp == 0) temp = 0xFF;
        return temp;
    }

    return 0;
}

/** @brief getStatus - Returns the value of the 16bit status register
  *
  * Fetch and return the 16-bit value in the STATUS register. Resets
  * any warning flags and exits any error states. Using GetParam()
  * to read STATUS does not clear these values.
  */
uint32_t SFL6470::getStatus()
{
    int temp = 0;
    dSPIN_Xfer(dSPIN_GET_STATUS);
    temp = dSPIN_Xfer(0)<<8;
    temp |= dSPIN_Xfer(0);
    return temp;
}

/** @brief getError - Returns error code based on status register flags.
  *
  * Returns an error code from 0 - 127.  Each bit represents a differnt possible
  * error condition.
  */
uint32_t SFL6470::getError()
{
    uint32_t result = 0;
    uint32_t status = getStatus();
    uint32_t temp = 0;
    //printf("SFL6470::getError - STATUS [0x%04X]\n", status);

    temp = status & dSPIN_STATUS_NOTPERF_CMD;
    //printf("SFL6470::getError - NOEXEC [0x%04X]\n", temp);
    if (temp) result |= dSPIN_ERR_NOEXEC;

    temp = status & dSPIN_STATUS_WRONG_CMD;
    //printf("SFL6470::getError - BADCMD [0x%04X]\n", temp);
    if (temp) result |= dSPIN_ERR_BADCMD;

    temp = status & dSPIN_STATUS_UVLO;
    //printf("SFL6470::getError - UVLO   [0x%04X]\n", temp);
    if (temp == 0) result |= dSPIN_ERR_UVLO;

    temp = status & dSPIN_STATUS_TH_SD;
    //printf("SFL6470::getError - THSHTD [0x%04X]\n", temp);
    if (temp == 0) result |= dSPIN_ERR_THSHTD;

    temp = status & dSPIN_STATUS_OCD;
    //printf("SFL6470::getError - OVERC  [0x%04X]\n", temp);
    if (temp == 0) result |= dSPIN_ERR_OVERC;

    temp = status & dSPIN_STATUS_STEP_LOSS_A;
    //printf("SFL6470::getError - STALLA [0x%04X]\n", temp);
    if (temp == 0) result |= dSPIN_ERR_STALLA;

    temp = status & dSPIN_STATUS_STEP_LOSS_B;
    //printf("SFL6470::getError - STALLB [0x%04X]\n", temp);
    if (temp == 0) result |= dSPIN_ERR_STALLB;

    return result;
}

float SFL6470::getAccel()
{
    float result = 0.0;
    int32_t regVal = 0;

    regVal = getParam(dSPIN_ACC);
    result = regVal/0.137438;

    return result;
}

float SFL6470::getDecel()
{
    float result = 0.0;
    int32_t regVal = 0;

    regVal = getParam(dSPIN_DEC);
    result = regVal/0.137438;

    return result;
}

float SFL6470::getMaxSpeed()
{
    float result = 0.0;
    uint32_t regVal = getParam(dSPIN_MAX_SPEED);

    result = regVal/0.065536;
    return result;
}

float SFL6470::getMinSpeed()
{
    float result = 0.0;
    uint32_t regVal = getParam(dSPIN_MIN_SPEED);

    result = regVal/0.065536;
    return result;
}




///////////////////////////////////////////////////////////////////////////////////////////////////
// Device Commands
//
/** @brief run - Move the motor at the specified speed and direction.
  *
  * Motor will run indefinately in the direction specified and at the
  * step rate specfied by the spd parameter in steps per second.
  *
  * @dir - Motor direction 0=REV | 1=FWD
  * @spd - Motor speed 0 - 15625
  */
void SFL6470::run(uint8_t dir, float spd)
{
    uint32_t regVal = (spd * 67.108864) + 0.5;
    if (regVal > 0xFFFFF) regVal = 0xFFFFF;
    dir = dirInvert(dir);
    dSPIN_Xfer(dSPIN_RUN | dir);
    dSPIN_Xfer((uint8_t)(regVal >> 16));
    dSPIN_Xfer((uint8_t)(regVal >> 8));
    dSPIN_Xfer((uint8_t)(regVal));
}

/** @brief move - Move the motor the specified number of steps and direction.
  *
  * Motor will accelerate, run to the relative number of steps specified and in the direction
  * specified, and decelerate.
  *
  * @dir    - Motor direction 0=REV | 1=FWD
  * @steps  - Number of steps to move.
  */
void SFL6470::move(uint8_t dir, uint32_t steps)
{
    if (steps > 0x3FFFFF) steps = 0x3FFFFF;
    dir = dirInvert(dir);
    dSPIN_Xfer(dSPIN_MOVE | dir);
    dSPIN_Xfer((uint8_t)(steps >> 16));
    dSPIN_Xfer((uint8_t)(steps >> 8));
    dSPIN_Xfer((uint8_t)(steps));
}

/** @brief gotoPos - Move the motor to the absolute microstep position specified.
  *
  * Motor will accelerate, run to the relative number of microsteps specified and in the direction
  * that is the shortest distance, and decelerate.
  *
  * @pos  - Absolute microstep postion to move to.
  */
void SFL6470::gotoPos(uint32_t pos)
{
  dSPIN_Xfer(dSPIN_GOTO);
  if (pos > 0x3FFFFF) pos = 0x3FFFFF;
  dSPIN_Xfer((uint8_t)(pos >> 16));
  dSPIN_Xfer((uint8_t)(pos >> 8));
  dSPIN_Xfer((uint8_t)(pos));
}

/** @brief gotoPos (dir) - Move the motor to the absolute microstep position specified in the direction specified.
  *
  * Motor will accelerate, run to the relative number of microsteps specified and in the direction
  * that is specified, and decelerate.
  *
  * @dir  - Direction to rotate.
  * @pos  - Absolute microstep postion to move to.
  */
void SFL6470::gotoPos(uint8_t dir, uint32_t pos)
{
    dir = dirInvert(dir);
    if (pos > 0x3FFFFF) pos = 0x3FFFFF;
    dSPIN_Xfer(dSPIN_GOTO_DIR | dir);
    dSPIN_Xfer((uint8_t)(pos >> 16));
    dSPIN_Xfer((uint8_t)(pos >> 8));
    dSPIN_Xfer((uint8_t)(pos));
}


void SFL6470::goUntil(uint8_t act, uint8_t dir, float spd)
{
    int spdVal = (spd * 67.108864) + 0.5;
    dir = dirInvert(dir);
    if (spdVal > 0x3FFFFF) spdVal = 0x3FFFFF;
    dSPIN_Xfer(dSPIN_GO_UNTIL | act | dir);
    dSPIN_Xfer((uint8_t)(spdVal >> 16));
    dSPIN_Xfer((uint8_t)(spdVal >> 8));
    dSPIN_Xfer((uint8_t)(spdVal));
}



// Generalization of the subsections of the register read/write functionality.
//  We want the end user to just write the value without worrying about length,
//  so we pass a bit length parameter from the calling function.
uint32_t SFL6470::procParam(uint32_t value, uint8_t bit_len)
{
    uint32_t ret_val=0;

    uint8_t byte_len = bit_len/8;   // How many BYTES do we have?
    if (bit_len%8 > 0)              // Make sure not to lose any partial byte values.
        byte_len++;

    // Let's make sure our value has no spurious bits set, and if the value was too
    //  high, max it out.
    uint32_t mask = 0xffffffff >> (32-bit_len);
    if (value > mask)
        value = mask;

    // The following three if statements handle the various possible byte length
    //  transfers- it'll be no less than 1 but no more than 3 bytes of data.
    // dSPIN_Xfer() sends a byte out through SPI and returns a byte received
    //  over SPI- when calling it, we typecast a shifted version of the masked
    //  value, then we shift the received value back by the same amount and
    //  store it until return time.
    if (byte_len == 3)
    {
        ret_val |= dSPIN_Xfer((uint8_t)(value>>16)) << 16;
        //Serial.println(ret_val, HEX);
    }

    if (byte_len >= 2)
    {
        ret_val |= dSPIN_Xfer((uint8_t)(value>>8)) << 8;
        //Serial.println(ret_val, HEX);
    }

    if (byte_len >= 1)
    {
        ret_val |= dSPIN_Xfer((uint8_t)value);
        //Serial.println(ret_val, HEX);
    }

    // Return the received values. Mask off any unnecessary bits, just for
    //  the sake of thoroughness- we don't EXPECT to see anything outside
    //  the bit length range but better to be safe than sorry.
    return (ret_val & mask);
}

uint32_t SFL6470::paramHandler(uint8_t param, uint32_t value)
{
  uint32_t ret_val = 0;   // This is a temp for the value to return.

  // This switch structure handles the appropriate action for each register.
  //  This is necessary since not all registers are of the same length, either
  //  bit-wise or byte-wise, so we want to make sure we mask out any spurious
  //  bits and do the right number of transfers. That is handled by the Param()
  //  function, in most cases, but for 1-byte or smaller transfers, we call
  //  dSPIN_Xfer() directly.
  switch (param)
  {
    // ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
    //  in two's complement. At power up, this value is 0. It cannot be written when
    //  the motor is running, but at any other time, it can be updated to change the
    //  interpreted position of the motor.
    case dSPIN_ABS_POS:
      ret_val = procParam(value, 22);
      break;
    // EL_POS is the current electrical position in the step generation cycle. It can
    //  be set when the motor is not in motion. Value is 0 on power up.
    case dSPIN_EL_POS:
      ret_val = procParam(value, 9);
      break;
    // MARK is a second position other than 0 that the motor can be told to go to. As
    //  with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.
    case dSPIN_MARK:
      ret_val = procParam(value, 22);
      break;
    // SPEED contains information about the current speed. It is read-only. It does
    //  NOT provide direction information.
    case dSPIN_SPEED:
      ret_val = procParam(0, 20);
      break;
    // ACC and DEC set the acceleration and deceleration rates. Set ACC to 0xFFF
    //  to get infinite acceleration/decelaeration- there is no way to get infinite
    //  deceleration w/o infinite acceleration (except the HARD STOP command).
    //  Cannot be written while motor is running. Both default to 0x08A on power up.
    // AccCalc() and DecCalc() functions exist to convert steps/s/s values into
    //  12-bit values for these two registers.
    case dSPIN_ACC:
      ret_val = procParam(value, 12);
      break;
    case dSPIN_DEC:
      ret_val = procParam(value, 12);
      break;
    // MAX_SPEED is just what it says- any command which attempts to set the speed
    //  of the motor above this value will simply cause the motor to turn at this
    //  speed. Value is 0x041 on power up.
    // MaxSpdCalc() function exists to convert steps/s value into a 10-bit value
    //  for this register.
    case dSPIN_MAX_SPEED:
      ret_val = procParam(value, 10);
      break;
    // MIN_SPEED controls two things- the activation of the low-speed optimization
    //  feature and the lowest speed the motor will be allowed to operate at. LSPD_OPT
    //  is the 13th bit, and when it is set, the minimum allowed speed is automatically
    //  set to zero. This value is 0 on startup.
    // MinSpdCalc() function exists to convert steps/s value into a 12-bit value for this
    //  register. SetLSPDOpt() function exists to enable/disable the optimization feature.
    case dSPIN_MIN_SPEED:
      ret_val = procParam(value, 12);
      break;
    // FS_SPD register contains a threshold value above which microstepping is disabled
    //  and the dSPIN operates in full-step mode. Defaults to 0x027 on power up.
    // FSCalc() function exists to convert steps/s value into 10-bit integer for this
    //  register.
    case dSPIN_FS_SPD:
      ret_val = procParam(value, 10);
      break;
    // KVAL is the maximum voltage of the PWM outputs. These 8-bit values are ratiometric
    //  representations: 255 for full output voltage, 128 for half, etc. Default is 0x29.
    // The implications of different KVAL settings is too complex to dig into here, but
    //  it will usually work to max the value for RUN, ACC, and DEC. Maxing the value for
    //  HOLD may result in excessive power dissipation when the motor is not running.
    case dSPIN_KVAL_HOLD:
      ret_val = dSPIN_Xfer((uint8_t)value);
      break;
    case dSPIN_KVAL_RUN:
      ret_val = dSPIN_Xfer((uint8_t)value);
      break;
    case dSPIN_KVAL_ACC:
      ret_val = dSPIN_Xfer((uint8_t)value);
      break;
    case dSPIN_KVAL_DEC:
      ret_val = dSPIN_Xfer((uint8_t)value);
      break;
    // INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
    //  compensation functionality. Please see the datasheet for details of this
    //  function- it is too complex to discuss here. Default values seem to work
    //  well enough.
    case dSPIN_INT_SPD:
      ret_val = procParam(value, 14);
      break;
    case dSPIN_ST_SLP:
      ret_val = dSPIN_Xfer((uint8_t)value);
      break;
    case dSPIN_FN_SLP_ACC:
      ret_val = dSPIN_Xfer((uint8_t)value);
      break;
    case dSPIN_FN_SLP_DEC:
      ret_val = dSPIN_Xfer((uint8_t)value);
      break;
    // K_THERM is motor winding thermal drift compensation. Please see the datasheet
    //  for full details on operation- the default value should be okay for most users.
    case dSPIN_K_THERM:
      ret_val = dSPIN_Xfer((uint8_t)value & 0x0F);
      break;
    // ADC_OUT is a read-only register containing the result of the ADC measurements.
    //  This is less useful than it sounds; see the datasheet for more information.
    case dSPIN_ADC_OUT:
      ret_val = dSPIN_Xfer(0);
      break;
    // Set the overcurrent threshold. Ranges from 375mA to 6A in steps of 375mA.
    //  A set of defined constants is provided for the user's convenience. Default
    //  value is 3.375A- 0x08. This is a 4-bit value.
    case dSPIN_OCD_TH:
      ret_val = dSPIN_Xfer((uint8_t)value & 0x0F);
      break;
    // Stall current threshold. Defaults to 0x40, or 2.03A. Value is from 31.25mA to
    //  4A in 31.25mA steps. This is a 7-bit value.
    case dSPIN_STALL_TH:
      ret_val = dSPIN_Xfer((uint8_t)value & 0x7F);
      break;
    // STEP_MODE controls the microstepping settings, as well as the generation of an
    //  output signal from the dSPIN. Bits 2:0 control the number of microsteps per
    //  step the part will generate. Bit 7 controls whether the BUSY/SYNC pin outputs
    //  a BUSY signal or a step synchronization signal. Bits 6:4 control the frequency
    //  of the output signal relative to the full-step frequency; see datasheet for
    //  that relationship as it is too complex to reproduce here.
    // Most likely, only the microsteps per step value will be needed; there is a set
    //  of constants provided for ease of use of these values.
    case dSPIN_STEP_MODE:
      ret_val = dSPIN_Xfer((uint8_t)value);
      break;
    // ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of constants
    //  is provided to make this easy to interpret. By default, ALL alarms will trigger the
    //  FLAG pin.
    case dSPIN_ALARM_EN:
      ret_val = dSPIN_Xfer((uint8_t)value);
      break;
    // CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
    //  set of reasonably self-explanatory constants is provided, but users should refer
    //  to the datasheet before modifying the contents of this register to be certain they
    //  understand the implications of their modifications. Value on boot is 0x2E88; this
    //  can be a useful way to verify proper start up and operation of the dSPIN chip.
    case dSPIN_CONFIG:
      ret_val = procParam(value, 16);
      break;
    // STATUS contains read-only information about the current condition of the chip. A
    //  comprehensive set of constants for masking and testing this register is provided, but
    //  users should refer to the datasheet to ensure that they fully understand each one of
    //  the bits in the register.
    case dSPIN_STATUS:  // STATUS is a read-only register
      ret_val = procParam(0, 16);
      break;
    default:
      ret_val = dSPIN_Xfer((uint8_t)(value));
      break;
  }
  return ret_val;
}

uint8_t SFL6470::dSPIN_Xfer(uint8_t data)
{
    uint8_t data_out = data;
    m_bus->openBus();
    m_bus->rwData(&data_out, 1);
    m_bus->closeBus();
    return data_out;
}

uint8_t SFL6470::dirInvert(uint8_t dir)
{
    if (m_invertDir)
    {
        if(dir) dir = 0;
        else dir = 1;
    }

    return dir;
}
