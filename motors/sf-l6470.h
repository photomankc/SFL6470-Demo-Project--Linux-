#ifndef SFL6470_H
#define SFL6470_H

#include "sf-l6470-support.h"
#include "spi.h"
#include <stdint.h>
//#include <wiringPiSPI.h>

class SFL6470
{
    protected:
        SPI*    m_bus;                                          // Pointer to the SPI bus this device is using
        int     m_invertDir;                                    // Indicator to invert direction value
        int     m_msMode;                                       // Tracks current microstep mode

    public:
        SFL6470();
        SFL6470(SPI& bus);
        virtual ~SFL6470();
        void    init(SPI& bus);                                 // Setup initial state of the object

        /********** Set Functions ************/
        void        setParam(uint8_t param, uint32_t value);    // Sets a device parameter register
        uint32_t    setAccel(float spss);                       // Sets the device acceleration in steps per second per second
        uint32_t    setDecel(float spss);                       // Sets the device deceleration in steps per second per second
        uint32_t    setMaxSpeed(float sps);                     // Sets the maximum speed for the motor in steps per second
        uint32_t    setFSCutOff(float sps);                     // Sets the full step cut-off in steps per second
        uint32_t    setINTSpeed(float sps);                     // Sets the acceleration slope change point in steps per second
        int         setMicroSteps(int val);                     // Sets the number of microsteps per full step position (1 - 128) by powers of two.
        void        invert(uint8_t inv);                        // Inverts the meaning of the direction bit 0=Not Inverted | 1=Inverted

        /********** Get Functions ************/
        uint32_t    getParam(uint8_t param);                    // Reads a device paramter register
        uint8_t     isBusy();                                   // Returns 1-3 if the status register shows the device is busy, 0 otherwise
        uint8_t     isInverted();                               // Returns 1 if the motor direction is inverted, 0 if not.
        uint32_t    getStatus();                                // Returns the status register of the device, resets any flag conditions
        uint8_t     getDir();                                   // Returns 1 if FWD, 0 if REV
        uint32_t    getError();                                 // Returns an error value based on the status register if any errors are present
        float       getAccel();                                 // Returns the acceleration value as steps per second per second
        float       getDecel();                                 // Returns the deceleration value as steps per second per second
        float       getMaxSpeed();                              // Returns the maximum speed as steps per second
        float       getMinSpeed();
        float       getFSCutOff();                              // Returns the full step cut-off as steps per second
        float       getINTSpeed();                              // Returns the acceleration slope change speed in steps per second
        int         getMicroSteps();                            // Returns the number of microsteps per full step of the current config

        /********** Device Commands ***********/
        void        resetDev();                                 // Returns the device to power-up settings
        void        run(uint8_t dir, float spd);                // Runs the motor at a set speed and direction indefinately
        void        move(uint8_t dir, uint32_t steps);          // Moves the motor at MAX_SPD in the direction specified for specified microsteps
        void        moveFS(uint8_t dir, uint32_t steps);        // Moves the motor at MAX_SPD in the direction specified for specified number of FULL STEPS
        void        gotoPos(uint32_t pos);                      // Goto an absolute position
        void        gotoPosFS(uint32_t pos);                    // Goto an absolute position in FULL STEPS
        void        gotoPos(uint8_t dir, uint32_t pos);         // Goto an absoulte position in the direction specfied.
        void        gotoPosFS(uint8_t dir, uint32_t pos);       // Goto an absoulte position in FULL STEPS in the direction specfied.
        void        goUntil(uint8_t act, uint8_t dir, float spd); // Moves int the direction and speed specified until a falling SW edge is detected
        void        releaseSW(uint8_t act, uint8_t dir);        // Moves in the direction specified at MIN_SPEED until a rising edge on SW is detected
        void        goHome();                                   // Goto position 0
        void        goMark();                                   // Goto marked position
        void        resetPos();                                 // Reset the position counter to 0
        void        softStop();                                 // Stop with deceleration
        void        hardStop();                                 // Stop without deceleration
        void        softHiZ();                                  // Decelerate and place the bridges in HiZ state
        void        hardHiZ();                                  // Place bridges in HiZ state without deceleration

    protected:
        uint32_t    paramHandler(uint8_t param, uint32_t value); // Performs nessary actions for the various command parameters
        uint32_t    procParam(uint32_t value, uint8_t bit_len);  // Handles the varible bit-length parameters
        uint8_t     dSPIN_Xfer(uint8_t data);                    // Performs the SPI byte transfer to the device
        uint8_t     dirInvert(uint8_t dir);                      // Returns the corrected direction bit based on the inversion setting.
};

inline SFL6470::SFL6470()
{
    m_bus = 0;
    m_invertDir = 0;
    m_msMode = 0;
}

inline SFL6470::SFL6470(SPI& bus)
{
    init(bus);
}

/** @brief invert - If set to true, inverts the meaning of the direction bit.
  *
  * @inv - 0=Non-inverted | 1=Inverted
  */
inline void SFL6470::invert(uint8_t inv)
{
    if (inv) m_invertDir = 1;
    else m_invertDir = 0;
}

/** @brief isInverted - Returns true if direction is inverted false otherwise.
  */
inline uint8_t SFL6470::isInverted()
{
    return m_invertDir;
}

/** @brief getMicroSteps - Returns the number of microsteps per full step.
  */
inline int SFL6470::getMicroSteps()
{
    return m_msMode;
}

/** @brief moveFS - Move the motor the specified number of FULL STEPS and direction.
  *
  * Motor will accelerate, run to the relative number of FULL STEPS specified and in the
  * direction specified, then decelerate.
  *
  * @dir    - Motor direction 0=REV | 1=FWD
  * @steps  - Number of FULL STEPS to move.
  */
inline void SFL6470::moveFS(uint8_t dir, uint32_t steps)
{
    steps *= m_msMode;
    move(dir, steps);
}

inline void SFL6470::gotoPosFS(uint32_t pos)
{
    pos *= m_msMode;
    gotoPos(pos);
}

inline void SFL6470::gotoPosFS(uint8_t dir, uint32_t pos)
{
    pos *= m_msMode;
    gotoPos(dir, pos);
}

inline void SFL6470::releaseSW(uint8_t act, uint8_t dir)
{
    dSPIN_Xfer(dSPIN_RELEASE_SW | act | dir);
}

/** @brief goHome - Return to the absolute position 0 at MAX_SPEED in shortest path possible.
  */
inline void SFL6470::goHome()
{
    dSPIN_Xfer(dSPIN_GO_HOME);
}

/** @brief goMark - Goto positon stored in MARK register at MAX_SPEED in shortest distance
  */
inline void SFL6470::goMark()
{
    dSPIN_Xfer(dSPIN_GO_MARK);
}

/** @brief resetPos - Reset the position counter to zero
  */
inline void SFL6470::resetPos()
{
    dSPIN_Xfer(dSPIN_RESET_POS);
}

/** @brief resetDev - Reset the dSPIN chip to power on defaults
  */
inline void SFL6470::resetDev()
{
    dSPIN_Xfer(dSPIN_RESET_DEVICE);
}

/** @brief softStop - Bring the motor to a halt using the deceleration curve.
  */
inline void SFL6470::softStop()
{
    dSPIN_Xfer(dSPIN_SOFT_STOP);
}

/** @brief hardStop - Bring the motor to a halt using infinite deceleration.
  */
inline void SFL6470::hardStop()
{
    dSPIN_Xfer(dSPIN_HARD_STOP);
}

/** @brief softHiZ - Bring the motor to a halt using deceleration curve and put bridges in Hi-Z.
  */
inline void SFL6470::softHiZ()
{
    dSPIN_Xfer(dSPIN_SOFT_HIZ);
}

/** @brief softHiZ - Put the bridges in Hi-Z state immediately with no deceleration.
  */
inline void SFL6470::hardHiZ()
{
    dSPIN_Xfer(dSPIN_HARD_HIZ);
}

#endif // SF-L6470_H

