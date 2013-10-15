#include <stdio.h>
#include <stdlib.h>
#include "l6470.h"

/* L6470::L6470()
{
    m_bus = 0;
    m_invertDir = 0;
    m_msMode = 0;
}*/


/** @brief Creates an L6470 object using an already created SPI bus object.
 *
 *  This constructs a new L6470 object using a stack created SPI object that
 *  is owned elsewhere.  This object will not manage the SPI object's memory.
 *
 *  @param bus A reference to an existing object implementing ISPI interface
 *  @param cfg Optional configuration register value to pass to the chip.
 */
L6470::L6470(ISPI& bus, uint32_t cfg)
{
    m_bus       = &bus;
    m_bus->setMode(3);
    m_bus->setBPW(8);
    m_invertDir = 0;
    m_msMode    = 128;     // Power on default
    resetDev();            // Ensure device is fully reset to power-on default
    if (cfg)
        setConfig(cfg);
    else
        setConfig(dSPIN_CONFIG_PWM_DIV_1          | dSPIN_CONFIG_PWM_MUL_2
                  | dSPIN_CONFIG_SR_290V_us       | dSPIN_CONFIG_OC_SD_DISABLE
                  | dSPIN_CONFIG_VS_COMP_ENABLE   | dSPIN_CONFIG_SW_HARD_STOP
                  | dSPIN_CONFIG_INT_16MHZ);
}


/** @brief Creates an L6470 object using a dynamicly created SPI bus object.
 *
 *  This constructs a new L6470 object using a heap created SPI object that
 *  is owned by this object.  This object will manage the SPI object's memory 
 *  and will delete it when destroyed.  The SPI object should not be shared 
 *  with any other objects and it would be best to create it directly in the 
 *  declaration of L6470.
 *
 *  @param p_bus A pointer to dynamic object implementing ISPI interface.
 *  @param cfg Optional configuration register value to pass to the chip.
 */
L6470::L6470(ISPI* p_bus, uint32_t cfg)
{
    m_invertDir = 0;
    m_msMode    = 128;     // Power on default
    
    if(p_bus)
    {
        m_bus = p_bus;
        m_ownBus = 1;
        m_bus->setMode(3);
        m_bus->setBPW(8);
        resetDev();        // Ensure device is fully reset to power-on default
        if (cfg)
            setConfig(cfg);
        else
            setConfig(dSPIN_CONFIG_PWM_DIV_1          | dSPIN_CONFIG_PWM_MUL_2
                      | dSPIN_CONFIG_SR_290V_us       | dSPIN_CONFIG_OC_SD_DISABLE
                      | dSPIN_CONFIG_VS_COMP_ENABLE   | dSPIN_CONFIG_SW_HARD_STOP
                      | dSPIN_CONFIG_INT_16MHZ);
    }
    else
        m_bus = NULL;
}


/** @brief Destroys the object and cleans up dynamically created SPI bus if present.
 *
 */
 L6470::~L6470()
{
    if (m_ownBus)
        delete m_bus;
}


/** @brief Setup the step motion parameters
 *
 *  @param microStp The microstep setting
 *  @param maxSpd The maximum speed in steps per second for the motor (Dfl 500)
 *  @param acc The acceleration in steps/sec/sec (Dfl 100)
 *  @param dec The deceleration in steps/sec/sec (Dfl 100)
 */
void L6470::initMotion(uint8_t microStp, float maxSpd,
                       float acc, float dec)
{
    uint8_t msCheck = 0;
    uint8_t abort = 3;
    setMaxSpeed(maxSpd);
    setAccel(acc);
    setDecel(dec);

    do
    {
        msCheck = setMicroSteps(microStp);
        abort--;
        if (abort == 0)
            break;
    }
    while (msCheck != microStp);
    
}


/** @brief Setup the BackEMF parameters of the chip
 *
 *  @param k_hld Sets the holding current ADC value
 *  @param k_mv Sets the ADC parameters for current in MOV/ACC/DEC
 *  @param int_spd  TODO
 *  @param st_slp   TODO
 *  @param slp_acc  TODO
 *
 */
void L6470::initBEMF(uint32_t k_hld, uint32_t k_mv, uint32_t int_spd,
                     uint32_t st_slp, uint32_t slp_acc)
{
    setParam(dSPIN_KVAL_HOLD, k_hld);
    setParam(dSPIN_KVAL_ACC, k_mv);
    setParam(dSPIN_KVAL_DEC, k_mv);
    setParam(dSPIN_KVAL_RUN, k_mv);
    setParam(dSPIN_INT_SPD, int_spd);
    setParam(dSPIN_ST_SLP, st_slp);
    setParam(dSPIN_FN_SLP_ACC, slp_acc);
    setParam(dSPIN_FN_SLP_DEC, slp_acc);
}


/** @brief Write parameter fields to dSPIN chip.
 *
 *  @param param The register const for holding the paramter
 *  @param value The value to store in the register
 */
void L6470::setParam(uint8_t param, uint32_t value)
{
    dspin_xfer(dSPIN_SET_PARAM | param);
    paramHandler(param, value);
}


/** @brief Set the dSPIN acceleration value.
  *
  * Sets the acceleration value in steps per second per second.  Translates
  * to the dSPIN's 12 bit value.  Returns the calculated register value.
  *
  * @param spss The acceleration value in steps per second per second.
  */
void L6470::setAccel(float spss)
{
    float temp = spss * 0.137438;
    uint32_t regVal = (uint32_t) long(temp);

    if(regVal > 0x00000FFF)
        regVal = 0x00000FFF;

    setParam(dSPIN_ACC, regVal);
}


/** @brief Set the dSPIN deceleration value.
  *
  * Sets the deceleration value in steps per second per second.  Translates
  * to the dSPIN's 12 bit value.
  *
  * @param spss The deceleration value in steps per second per second.
  */
void L6470::setDecel(float spss)
{
    float temp = spss * 0.137438;
    uint32_t regVal = (uint32_t) long(temp);

    if(regVal > 0x00000FFF)
        regVal = 0x00000FFF;

    setParam(dSPIN_DEC, regVal);
}


/** @brief Set the maximum speed parameter value.
 *
 *  Sets the maximum speed cap for the motor.  Translates the value to the 
 *  native 10 bit value of the dSPIN register.  Resolution is 15.25 steps/sec 
 *  per increment of the number.
 *  
 *  @param sps Maximum speed in steps per second
 */
void L6470::setMaxSpeed(float sps)
{
    float temp = sps * .065536;
    uint32_t regVal = (unsigned long) long(temp);
    if( regVal > 0x000003FF)
        regVal = 0x000003FF;

    setParam(dSPIN_MAX_SPEED, regVal);
}


/** @brief Set the minimum speed parameter value.
 *
 *  Sets the minimum speed floor for the motor.  Translates the value to the 
 *  native 12 bit value of the dSPIN register.  Resolution is .238 steps/sec per
 *  increment of the number.
 *
 *  @param sps Maximum speed in steps per second
 */
void L6470::setMinSpeed(float sps)
{
    float temp = sps * 4.1943;
    uint32_t regVal = (unsigned long) long(temp);
    if( regVal > 0x00000FFF)
        regVal = 0x00000FFF;
    
    setParam(dSPIN_MIN_SPEED, regVal);
}


/** @brief Set the full step threshold speed to the specfied step/sec value
 *
 *  Sets the threshold after which the chip will drive the stepper motor using
 *  a full step sequence rather than microsteps.  Same bit size and resolution
 *  as MAX_SPEED.
 */
void L6470::setFullStepThreshold(float sps)
{
    float temp = (sps * 0.065536) - 0.5;
    uint32_t regVal = (unsigned long) long(temp);
    if( regVal > 0x000003FF)
        regVal = 0x000003FF;
    
    setParam(dSPIN_FS_SPD, regVal);
}


/** @brief Sets the number of microsteps per full step.
 *
 *  Sets the microstep mode for the dSPIN chip.  Valid values are 0-128 in
 *  powers of 2.  Invalid values are rounded down to next lower valid value.
 *  returns the previous setting.
 *
 *  @param val value 0 to 128
 *  @return uint8_t: New Microstep Setting
 */
uint8_t L6470::setMicroSteps(uint8_t val)
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
    int err = getError();
    
    if (err == dSPIN_ERR_NOEXEC)
        m_msMode = oldVal;
    
    return m_msMode;
}


/** @brief Set the ABS_POS register to the provided value.
 *
 *  @param pos The postion value to store in the register as a signed value relative to 0.
 *  @return int32_t The previous position
 */
void L6470::setPosition(int32_t pos)
{
    pos &= 0x3FFFFF;                 // Limit value and preserve sign
    setParam(dSPIN_ABS_POS, pos);
}


/** @brief Set the motor position to the specified value relative to 0 in FULL STEPS.
 *
 *  @param pos The new motor position value to store in ABS_POS
 */
 void L6470::setPosition_FS(int32_t pos)
{
    setPosition(pos * m_msMode);
}


/** @brief Set the current position as the marked position
 *
 *  @return int32_t: The marked position in microsteps
 */
int32_t L6470::setMark()
{
    int32_t result = getPosition();
    setParam(dSPIN_MARK, result);
    return result;
}


/** @brief Set the configuration register to the provided value.
 *
 *  Sets the chip configuration register.  A large set of constants are defined
 *  in l6470-support.h for building the register value.  If any part of the 
 *  register is being changed the entire register must be defined.
 *
 *  @param cfg The configuration value to write to the register.
 */
int L6470::setConfig(uint32_t cfg)
{
    setParam(dSPIN_CONFIG, cfg);
    return 0;
}


/** @brief If set to true, inverts the meaning of the direction bit.
 *
 *  @param inv 0 = Non-inverted | 1 = Inverted
 */
 void L6470::invert(uint8_t inv)
{
    if (inv) m_invertDir = 1;
    else m_invertDir = 0;
}


/** @brief Returns 1 if direction is inverted 0 otherwise.
 *
 *  @return uint8_t: Inverted setting
 */
 uint8_t L6470::isInverted()
{
    return m_invertDir;
}


/** @brief Get the motor direction from the device status register
 *
 *  @return uint8_t: The direction bit from the status register.
 */
uint8_t L6470::getDir()
{
    uint16_t status = getStatus();
    status &= dSPIN_STATUS_DIR;
    if (status)
        return dirInvert(1);
    else
        return dirInvert(0);
}


/** @brief Read parameter fields from dSPIN.
 *
 * @param param Parameter constant to retrieve proper parameter from chip.
 * @return uint32_t value stored at the selected register
 */
uint32_t L6470::getParam(uint8_t param)
{
    dspin_xfer(dSPIN_GET_PARAM | param);
    return paramHandler(param, 0);
}


/** @brief Check the status register for busy.
 *
 * Returns the motor status value if the driver is busy.  Otherwise
 * returns 0.
 *
 * @return uint8_t: returns 0 if not busy or motor status if busy.
 */
uint32_t L6470::isBusy()
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


/** @brief Returns the value of the 16bit status register
 *
 * Fetch and return the 16-bit value in the STATUS register. Resets
 * any warning flags and exits any error states. Using GetParam()
 * to read STATUS does not clear these values.
 *
 * @return uint32_t: Status register value
 */
uint32_t L6470::getStatus()
{
    int temp = 0;
    dspin_xfer(dSPIN_GET_STATUS);
    temp = dspin_xfer(0)<<8;
    temp |= dspin_xfer(0);
    return temp;
}


/** @brief Returns the value stored in the chip configuration register
 *
 *  @return uint32_t: Register value containing all the configuration bits
 */
 uint32_t L6470::getConfig()
{
    return getParam(dSPIN_CONFIG);
}


/** @brief Get the current motor position stored in the device
 *
 *  @return int32_t: Signed value for microsteps relative to the home 0 position.
 */
int32_t L6470::getPosition()
{
    int32_t regVal = getParam(dSPIN_ABS_POS);
    int32_t result = 0;
    
    if(regVal > 0x1FFFFF)
    {
        result = regVal + 0xFFC00000;
        return result;
    }
    else
        return regVal;
}


/** @brief Returns the current position stored in the chip as FULL STEPS
 *
 *  This function returns the current position of the motor in full steps
 *  as a 32 bit integer.  Returns either a positive or negative number relative
 *  to the current 0 home position.
 *
 *  @return int32_t: Position value as signed value relative to home in full steps.
 */
 int32_t L6470::getPosition_FS()
{
    float pos = (getPosition() / m_msMode) + 0.5;
    return (int32_t)pos;
}


/** @brief Returns error code based on status register flags.
 *
 * Returns an error code from 0 - 127.  Each bit represents a differnt possible
 * error condition.
 *
 * @return uint32_t: Error code
 */
uint32_t L6470::getError()
{
    uint32_t result = 0;
    uint32_t status = getStatus();
    uint32_t temp = 0;

    temp = status & dSPIN_STATUS_NOTPERF_CMD;
    if (temp) result |= dSPIN_ERR_NOEXEC;

    temp = status & dSPIN_STATUS_WRONG_CMD;
    if (temp) result |= dSPIN_ERR_BADCMD;

    temp = status & dSPIN_STATUS_UVLO;
    if (temp == 0) result |= dSPIN_ERR_UVLO;

    temp = status & dSPIN_STATUS_TH_SD;
    if (temp == 0) result |= dSPIN_ERR_THSHTD;

    temp = status & dSPIN_STATUS_OCD;
    if (temp == 0) result |= dSPIN_ERR_OVERC;

    temp = status & dSPIN_STATUS_STEP_LOSS_A;
    if (temp == 0) result |= dSPIN_ERR_STALLA;

    temp = status & dSPIN_STATUS_STEP_LOSS_B;
    if (temp == 0) result |= dSPIN_ERR_STALLB;

    return result;
}


/** @brief Get the dSPIN acceleration value.
 *
 * Gets the acceleration value in steps per second per second.
 *
 * @return float: The acceleration value in steps per second per second.
 */
float L6470::getAccel()
{
    float result = 0.0;
    int32_t regVal = 0;

    regVal = getParam(dSPIN_ACC);
    result = regVal/0.137438;

    return result;
}


/** @brief Get the dSPIN deceleration value.
 *
 * Gets the deceleration value in steps per second per second.
 *
 * @return float: The deceleration value in steps per second per second.
 */
float L6470::getDecel()
{
    float result = 0.0;
    int32_t regVal = 0;

    regVal = getParam(dSPIN_DEC);
    result = regVal/0.137438;

    return result;
}


/** @brief Get the dSPIN maximum speed value in steps per second.
 *
 * @return float: The maximum speed value.
 */
float L6470::getMaxSpeed()
{
    float result = 0.0;
    uint32_t regVal = getParam(dSPIN_MAX_SPEED);

    result = regVal/0.065536;
    return result;
}


/** @brief Get the dSPIN minimum speed value in steps per second.
 *
 * @return float: The minimum speed value.
 */
float L6470::getMinSpeed()
{
    float result = 0.0;
    uint32_t regVal = getParam(dSPIN_MIN_SPEED);

    result = regVal/4.1943;
    return result;
}


/** @brief Get the full step threshold value from the device
 *
 *  @return float: The full step theshold value in steps/sec
 */
float L6470::getFullStepThreshold()
{
    float result = 0.0;
    uint32_t regVal = getParam(dSPIN_FS_SPD);
    
    result = regVal/0.065536;
    return result;
}


/** @brief Get the current movement speed from the device (read only)
 *
 *  @return float: Current speed in steps per second.
 */
float L6470::getSpeed()
{
    float result = 0.0;
    uint32_t regVal = getParam(dSPIN_SPEED);
    result = regVal/67.108864;
    return result;
}


/** @brief Returns the number of microsteps per full step.
 *
 *  @return int: Microsteps per full physical motor step
 */
 uint8_t L6470::getMicroSteps()
{
    return m_msMode;
}


/** @brief Reset the dSPIN chip to power on defaults
 *
 */
 void L6470::resetDev()
{
    dspin_xfer(dSPIN_RESET_DEVICE);
}


/** @brief Move the motor at the specified speed and direction.
  *
  * Motor will run indefinately in the direction specified and at the
  * step rate specfied by the spd parameter in steps per second.
  *
  * @param dir Motor direction 0=REV | 1=FWD
  * @param spd Motor speed 0 - 15625
  */
void L6470::run(uint8_t dir, float spd)
{
    uint32_t regVal = (spd * 67.108864) + 0.5;
    if (regVal > 0xFFFFF) regVal = 0xFFFFF;
    dir = dirInvert(dir);
    dspin_xfer(dSPIN_RUN | dir);
    dspin_xfer((uint8_t)(regVal >> 16));
    dspin_xfer((uint8_t)(regVal >> 8));
    dspin_xfer((uint8_t)(regVal));
}


/** @brief Move the motor the specified number of steps and direction.
 *
 *  Motor will accelerate, run to the relative number of steps specified and
 *  in the direction
 *  specified, and decelerate.
 *
 *  @param dir Motor direction 0=REV | 1=FWD
 *  @param steps Number of steps to move.
 */
void L6470::move(int32_t steps)
{
    int dir = 0;
    
    if (steps > 0)
        dir = 1;
    
    uint32_t abs_steps = abs(steps);
    if (abs_steps > 0x3FFFFF) abs_steps = 0x3FFFFF;
    dir = dirInvert(dir);
    dspin_xfer(dSPIN_MOVE | dir);
    dspin_xfer((uint8_t)(abs_steps >> 16));
    dspin_xfer((uint8_t)(abs_steps >> 8));
    dspin_xfer((uint8_t)(abs_steps));
}


/** @brief Move the motor the specified number of FULL STEPS and direction.
 *
 * Motor will accelerate, run to the relative number of FULL STEPS specified
 * and in the direction specified, then decelerate.
 *
 * @param  dir Motor direction 0=REV | 1=FWD
 * @param  steps Number of FULL STEPS to move.
 */
 void L6470::move_FS(int32_t steps)
{
    steps *= m_msMode;
    move(steps);
}


/** @brief Move the motor to the absolute microstep position specified.
 *
 *  Motor will accelerate, run to the relative number of microsteps specified
 *  in the direction that is the shortest distance, and decelerate.
 *
 *  @param pos Absolute microstep postion to move to.
 */
void L6470::gotoPosABS(int32_t pos)
{
    uint32_t abs_pos = (uint32_t)pos;
    dspin_xfer(dSPIN_GOTO);
    if (abs_pos > 0x3FFFFF) abs_pos &= 0x3FFFFF;
    dspin_xfer((uint8_t)(abs_pos >> 16));
    dspin_xfer((uint8_t)(abs_pos >> 8));
    dspin_xfer((uint8_t)(abs_pos));
}


/** @brief Moves to the specfied absolute position in FULL STEPS in the direction specified.
 *
 *  @param pos The position to move to in FULL Steps
 */
 void L6470::gotoPosABS_FS(int32_t pos)
{
    pos *= m_msMode;
    gotoPosABS(pos);
}


/** @brief Move to the microstep position specified in the direction specified by the sign
 *
 *  Motor will accelerate, run to the relative number of microsteps specified
 *  and in the direction implied by the sign of the pos value.
 *
 *  @param  pos Microstep postion to move to.
 */
void L6470::gotoPos(int32_t pos)
{
    int dir = 0;
    
    if (pos > 0)
        pos = 1;
    
    dir = dirInvert(dir);
    uint32_t abs_pos = (uint32_t)pos;
    if (abs_pos > 0x3FFFFF) abs_pos &= 0x3FFFFF;
    
    dspin_xfer(dSPIN_GOTO_DIR | dir);
    dspin_xfer((uint8_t)(abs_pos >> 16));
    dspin_xfer((uint8_t)(abs_pos >> 8));
    dspin_xfer((uint8_t)(abs_pos));
}


/** @brief Moves the motor to the specfied absolute position in FULL STEPS.
 *
 *  @param pos The absolute position to move to in FULL Steps
 */
 void L6470::gotoPos_FS(int32_t pos)
{
    pos *= m_msMode;
    gotoPos(pos);
}


/** @brief Return to the abs position 0 at MAX_SPEED in shortest path possible.
 *
 */
 void L6470::gotoHome()
{
    dspin_xfer(dSPIN_GO_HOME);
}


/** @brief Goto positon in MARK register at MAX_SPEED in shortest distance
 *
 */
 void L6470::gotoMark()
{
    dspin_xfer(dSPIN_GO_MARK);
}


/** @brief Move at the indicated speed until a switch event is received.
 *
 *  @param act Resets the ABS_POS register [1] or Copies postion to MARK [0].
 *  @param dir Direction of motor rotation.
 *  @param spd Speed of movement.
 */
void L6470::goUntil(uint8_t act, uint8_t dir, float spd)
{
    uint32_t spdVal = (spd * 67.108864) + 0.5;
    dir = dirInvert(dir);
    if (spdVal > 0x3FFFFF) spdVal = 0x3FFFFF;
    
    dspin_xfer(dSPIN_GO_UNTIL | act | dir);
    dspin_xfer((uint8_t)(spdVal >> 16));
    dspin_xfer((uint8_t)(spdVal >> 8));
    dspin_xfer((uint8_t)(spdVal));
}


/** @brief Halt using the deceleration curve.
 *
 */
 void L6470::softStop()
{
    dspin_xfer(dSPIN_SOFT_STOP);
}


/** @brief Halt using infinite deceleration.
 *
 */
 void L6470::hardStop()
{
    dspin_xfer(dSPIN_HARD_STOP);
}


/** @brief Halt using deceleration curve and put bridges in Hi-Z.
 *
 */
 void L6470::softHiZ()
{
    dspin_xfer(dSPIN_SOFT_HIZ);
}


/** @brief Put the bridges in Hi-Z state immediately with no deceleration.
 *
 */
 void L6470::hardHiZ()
{
    dspin_xfer(dSPIN_HARD_HIZ);
}


 void L6470::releaseSW(uint8_t act, uint8_t dir)
{
    dspin_xfer(dSPIN_RELEASE_SW | act | dir);
}


/*   Generalization of the subsections of the register read/write functionality.
 *   We want the end user to just write the value without worrying about length,
 *   so we pass a bit length parameter from the calling function.
 */
uint32_t L6470::procParam(uint32_t value, uint8_t bit_len)
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
    // dspin_xfer() sends a byte out through SPI and returns a byte received
    //  over SPI- when calling it, we typecast a shifted version of the masked
    //  value, then we shift the received value back by the same amount and
    //  store it until return time.
    if (byte_len == 3)
    {
        ret_val |= dspin_xfer((uint8_t)(value>>16)) << 16;
    }

    if (byte_len >= 2)
    {
        ret_val |= dspin_xfer((uint8_t)(value>>8)) << 8;
    }

    if (byte_len >= 1)
    {
        ret_val |= dspin_xfer((uint8_t)value);
    }

    // Return the received values. Mask off any unnecessary bits, just for
    //  the sake of thoroughness- we don't EXPECT to see anything outside
    //  the bit length range but better to be safe than sorry.
    return (ret_val & mask);
}


/*  This function handles the variable length parameters for the various
 *  chip registers.  Since different parameters take different numbers of
 *  bits we select the correct size for the parameter and pass that to the next 
 *  function along with the proper bit length.
 */
uint32_t L6470::paramHandler(uint8_t param, uint32_t value)
{
  uint32_t ret_val = 0;

  switch (param)
  {
    //  ABS_POS is the current absolute offset from home. It is a 22 bit number
    //  expressed in two's complement. At power up, this value is 0. It cannot
    //  be written when the motor is running, but at any other time, it can be
    //  updated to change the interpreted position of the motor.
    case dSPIN_ABS_POS:
      ret_val = procParam(value, 22);
      break;
          
    //  EL_POS is the current electrical position in the step generation cycle.
    //  It can be set when the motor is not in motion. Value is 0 on power up.
    case dSPIN_EL_POS:
      ret_val = procParam(value, 9);
      break;
          
    //  MARK is a second position other than 0 that the motor can be told to go
    //  to. As with ABS_POS, it is 22-bit two's complement. Value is 0 on power
    //  up.
    case dSPIN_MARK:
      ret_val = procParam(value, 22);
      break;
          
    //  SPEED contains information about the current speed. It is read-only. It
    //  does NOT provide direction information.
    case dSPIN_SPEED:
      ret_val = procParam(0, 20);
      break;
          
    //  ACC and DEC set the acceleration and deceleration rates. Set ACC to
    //  0xFFF to get infinite acceleration/decelaeration- there is no way to
    //  get infinite deceleration w/o infinite acceleration (except the HARD
    //  STOP command). Cannot be written while motor is running. Both default
    //  to 0x08A on power up.
    case dSPIN_ACC:
      ret_val = procParam(value, 12);
      break;
          
    case dSPIN_DEC:
      ret_val = procParam(value, 12);
      break;
          
    //  MAX_SPEED is just what it says: any command which attempts to set the
    //  speed of the motor above this value will simply cause the motor to turn
    //  at this speed. Value is 0x041 on power up.
    case dSPIN_MAX_SPEED:
      ret_val = procParam(value, 10);
      break;
          
    //  MIN_SPEED controls two things- the activation of the low-speed
    //  optimization feature and the lowest speed the motor will be allowed to
    //  operate at. LSPD_OPT is the 13th bit, and when it is set, the minimum
    //  allowed speed is automatically set to zero. This value is 0 on startup.
    //  SetLSPDOpt() function exists to enable/disable the optimization feature.
    case dSPIN_MIN_SPEED:
      ret_val = procParam(value, 12);
      break;
          
    //  FS_SPD register contains a threshold value above which microstepping is
    //  disabled and the dSPIN operates in full-step mode. Defaults to 0x027 on
    //  power up.
    case dSPIN_FS_SPD:
      ret_val = procParam(value, 10);
      break;
          
    //  KVAL is the maximum voltage of the PWM outputs. These 8-bit values are
    //  ratiometric representations: 255 for full output voltage, 128 half, etc.
    //  Default is 0x29. The implications of different KVAL settings is too
    //  complex to dig into here, but it will usually work to max the value for
    //  RUN, ACC, and DEC. Maxing the value for HOLD may result in excessive
    //  power dissipation when the motor is not running.
    case dSPIN_KVAL_HOLD:
      ret_val = dspin_xfer((uint8_t)value);
      break;
          
    case dSPIN_KVAL_RUN:
      ret_val = dspin_xfer((uint8_t)value);
      break;
          
    case dSPIN_KVAL_ACC:
      ret_val = dspin_xfer((uint8_t)value);
      break;
          
    case dSPIN_KVAL_DEC:
      ret_val = dspin_xfer((uint8_t)value);
      break;
          
    //  INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back
    //  EMF compensation functionality. Please see the datasheet for details of
    //  this function, it is too complex to discuss here. Default values seem
    //  to work well enough.
    case dSPIN_INT_SPD:
      ret_val = procParam(value, 14);
      break;
          
    case dSPIN_ST_SLP:
      ret_val = dspin_xfer((uint8_t)value);
      break;
          
    case dSPIN_FN_SLP_ACC:
      ret_val = dspin_xfer((uint8_t)value);
      break;
          
    case dSPIN_FN_SLP_DEC:
      ret_val = dspin_xfer((uint8_t)value);
      break;
          
    //  K_THERM is motor winding thermal drift compensation. Please see the
    //  datasheet for full details on operation- the default value should be
    //  okay for most use.
    case dSPIN_K_THERM:
      ret_val = dspin_xfer((uint8_t)value & 0x0F);
      break;
          
    //  ADC_OUT is a read-only register containing the result of the ADC
    //  measurements. This is less useful than it sounds; see the datasheet
    //  for more information.
    case dSPIN_ADC_OUT:
      ret_val = dspin_xfer(0);
      break;
          
    //  Set the overcurrent threshold. Ranges from 375mA to 6A in steps of
    //  375mA. A set of defined constants is provided for the user's
    //  convenience. Default value is 3.375A- 0x08. This is a 4-bit value.
    case dSPIN_OCD_TH:
      ret_val = dspin_xfer((uint8_t)value & 0x0F);
      break;
          
    //  Stall current threshold. Defaults to 0x40, or 2.03A. Value is from
    //  31.25mA to 4A in 31.25mA steps. This is a 7-bit value.
    case dSPIN_STALL_TH:
      ret_val = dspin_xfer((uint8_t)value & 0x7F);
      break;
          
    //  STEP_MODE controls the microstepping settings, as well as the generation
    //  of an output signal from the dSPIN. Bits 2:0 control the number of
    //  microsteps per step the part will generate. Bit 7 controls whether the
    //  BUSY/SYNC pin outputs a BUSY signal or a step synchronization signal.
    //  Bits 6:4 control the frequency of the output signal relative to the
    //  full-step frequency; see datasheet for that relationship as it is too
    //  complex to reproduce here. Most likely, only the microsteps per step
    //  value will be needed; there is a set of constants provided for ease of
    //  use of these values.
    case dSPIN_STEP_MODE:
      ret_val = dspin_xfer((uint8_t)value);
      break;
          
    // ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of
    //  constants is provided to make this easy to interpret. By default,
    //  ALL alarms will trigger the FLAG pin.
    case dSPIN_ALARM_EN:
      ret_val = dspin_xfer((uint8_t)value);
      break;
          
    //  CONFIG contains some assorted configuration bits and fields. A fairly
    //  comprehensive set of reasonably self-explanatory constants is provided,
    //  but users should refer to the datasheet before modifying the contents
    //  of this register to be certain they understand the implications of their
    //  modifications. Value on boot is 0x2E88; this can be a useful way to
    //  verify proper start up and operation of the dSPIN chip.
    case dSPIN_CONFIG:
      ret_val = procParam(value, 16);
      break;
          
    //  STATUS contains read-only information about the current condition of the
    //  chip. A comprehensive set of constants for masking and testing this
    //  register is provided, but users should refer to the datasheet to ensure
    //  that they fully understand each one of the bits in the register.
    case dSPIN_STATUS:
      ret_val = procParam(0, 16);
      break;
          
    default:
      ret_val = dspin_xfer((uint8_t)(value));
      break;
  }
  return ret_val;
}


/*  
 *  Moves bytes over the SPI interface
 */
uint8_t L6470::dspin_xfer(uint8_t data)
{
    if(!m_bus)
        return -1;
    uint8_t data_out = data;
    m_bus->openBus();
    m_bus->rwData(&data_out, 1);
    m_bus->closeBus();
    return data_out;
}


/*
 *  Returns the corrected direction based on the objects invert setting
 */
uint8_t L6470::dirInvert(uint8_t dir)
{
    if (m_invertDir)
    {
        if(dir)
            dir = 0;
        else
            dir = 1;
    }

    return dir;
}



/*
 Copyright (C) 2013 Kyle Crane
 
 Adapted from example code for the SparkFun Breakout Board BOB-10859
 
 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 of the Software, and to permit persons to whom the Software is furnished to do
 so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */