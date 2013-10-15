#ifndef L6470_H
#define L6470_H

#include "l6470-support.h"
#include "ispi.h"
#include <stdint.h>


/** @brief Class to interface to the STI L6470 stepper motor driver chip
 *
 *  This class is designed to provide a code interface to the SPI based L6470
 *  Stepper Motor driver chip.  To use this device you will need to have 
 *  access to an SPI bus.  The object wraps the functionality of the SPI 
 *  commands and parameters to the chip.  This object requires you to provide
 *  it with a suitible SPI interface object that can be used to communicate to
 *  the hardware.  This class will not own the SPI object, but will expect
 *  exclusive use of it.  See the file "l6470-support.h" for a list of
 *  constants used extensively throughout the operation of the chip.
 *
 *  Be aware that most functions are non-blocking.  The chip performs the 
 *  needed stepping to move the motors. Functions that command motion will
 *  return immediately before that motion is completed.
 */
class L6470
{
protected:
    ISPI*   m_bus;
    int     m_ownBus;
    int     m_invertDir;
    int     m_msMode;

public:
    L6470(ISPI& bus, uint32_t cfg=0);
    L6470(ISPI* p_bus, uint32_t cfg=0);
    virtual ~L6470();
    void    initMotion(uint8_t microStp, float maxSpd = 500,
                       float acc=100, float dec=100);
    void    initBEMF(uint32_t k_hld, uint32_t k_mv, uint32_t int_spd, uint32_t st_slp,
                     uint32_t slp_acc);

    /********** Set Functions ************/
    void        setParam(uint8_t param, uint32_t value);
    void        setAccel(float spss);
    void        setDecel(float spss);
    void        setMaxSpeed(float sps);
    void        setMinSpeed(float sps);
    void        setFullStepThreshold(float sps);
    uint8_t     setMicroSteps(uint8_t val);
    void        setPosition(int32_t);
    void        setPosition_FS(int32_t);
    int32_t     setMark();
    int         setConfig(uint32_t cfg);
    void        invert(uint8_t inv);

    /********** Get Functions ************/
    uint32_t    getParam(uint8_t param);
    uint32_t    isBusy();
    uint8_t     isInverted();
    uint32_t    getStatus();
    uint32_t    getConfig();
    uint8_t     getDir();
    int32_t     getPosition();
    int32_t     getPosition_FS();
    uint32_t    getError();
    float       getAccel();
    float       getDecel();
    float       getMaxSpeed();
    float       getMinSpeed();
    float       getFullStepThreshold();
    float       getSpeed();
    uint8_t     getMicroSteps();
    
    /********** Device Commands ***********/
    void        resetDev();
    void        run(uint8_t dir, float spd);
    void        move(int32_t steps);
    void        move_FS(int32_t steps);
    void        gotoPosABS(int32_t pos);
    void        gotoPosABS_FS(int32_t pos);
    void        gotoPos(int32_t pos);
    void        gotoPos_FS(int32_t pos);
    void        gotoHome();
    void        gotoMark();
    void        goUntil(uint8_t act, uint8_t dir, float spd);
    void        releaseSW(uint8_t act, uint8_t dir);
    void        resetPos();
    void        softStop();
    void        hardStop();
    void        softHiZ();
    void        hardHiZ();

protected:
    uint32_t    paramHandler(uint8_t param, uint32_t value);
    uint32_t    procParam(uint32_t value, uint8_t bit_len);
    uint8_t     dspin_xfer(uint8_t data);
    uint8_t     dirInvert(uint8_t dir);
};



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

#endif // SF-L6470_H

