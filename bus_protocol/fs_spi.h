#ifndef FS_SPI_H
#define FS_SPI_H

#include <string>
#include <stdint.h>
#include "ispi.h"

/** @brief SPI implementation for Linux systems using file devices
 *
 *  This class implements an interface to the SPI buses provided via the /dev
 *  device file interface on Linux systems.  It implements the ISPI interface
 *  to allow SPI dependent devices to access these devices without being aware
 *  of the Linux file-system symantics.
 *
 */
class FS_SPI : public ISPI
{
protected:
    int 	    m_fd;
    std::string m_fname;
    uint16_t    m_spiDelay;             // Don't know what this is for

public:
    FS_SPI(const std::string& fn);
    FS_SPI(int speed, const std::string& fn);
    virtual ~FS_SPI();

    int openBus();
    int closeBus();
    int isReady();
    int setBPW(int val);
    int setSpeed(int val);
    int setMode(int val);

    int      rwData(uint8_t *data, uint8_t len);
    uint8_t  rwByte(uint8_t bt);
    uint16_t rwWord(uint16_t wd);

protected:
    void init(int speed, const std::string& fn);
};

/** @brief Construct an SPI object on the file device with a default clock rate.
 *
 *  @param fn std::string containing the filename to open.
 */
inline FS_SPI::FS_SPI(const std::string& fn)
{
    init(500000, fn);
}

/** @brief Construct an SPI object on the file device with the requested clock rate.
 *
 *  @param speed Clock speed for the SPI bus.
 *  @param fn The std::string containing the filename to open.
 */
inline FS_SPI::FS_SPI(int speed, const std::string& fn)
{
    init(speed, fn);
}

/** @brief Indicates that the SPI object has a valid file handle.
  *
  * @return int: 1 - Has a valid file handle to the SPI device file. 0 - Otherwise.
  */
inline int FS_SPI::isReady()
{
    if(m_fd > 0)
        return 1;
    else
        return 0;
}


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

#endif // FS_SPI_H
