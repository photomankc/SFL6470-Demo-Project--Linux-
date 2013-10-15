#ifndef __SFL6470__ISPI__
#define __SFL6470__ISPI__

#include <stdint.h>

/** @brief Interface for SPI communication.
 *
 *  Device specfic SPI implementation should implement this interface to allow
 *  portable use of SPI dependent objects. This interface
 *
 */
class ISPI
{
protected:
    int         m_speed;
    uint8_t     m_spiMode;
    uint8_t     m_spiBPW;
    
public:
    virtual ~ISPI() {};
    virtual int openBus()=0;
    virtual int closeBus()=0;
    virtual int isReady()=0;
    virtual int setBPW(int val)=0;
    virtual int setSpeed(int val) = 0;
    virtual int setMode(int val) = 0;
    
    virtual int      rwData(uint8_t *data, uint8_t len)=0;
    virtual uint8_t  rwByte(uint8_t bt)=0;
    virtual uint16_t rwWord(uint16_t wd)=0;
};

/*
 Copyright (C) 2013 Kyle Crane
 
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

#endif /* defined(__SFL6470__ISPI__) */
