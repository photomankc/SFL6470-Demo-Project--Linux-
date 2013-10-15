////////////////////////////////////////////////////////////////////////////////////////////////
//
//  i2c.h
//
//  Definition file for I2C wrapper object to Linux I2C userland device driver.  Allows for a simple
//  interface to the I2C related function calls and ioctls.
//

#ifndef I2C_H
#define I2C_H


#include "i2c-dev.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

enum I2CERR									// I2C Error codes for errno inspection.
{
	ERR_I2C_GEN	= -1,
	ERR_I2C_BSY 	= -2,
	ERR_I2C_FILE 	= -3,
	ERR_I2C_IO		= -4,
	ERR_I2C_RNG = -5,
};

class I2C
{
private:
    int 		m_file; 							// File handle to Linux I2C device file
    char* 	m_fname; 						// Filename for the Linux I2C device file

public:
    I2C(char const* fname);
    ~I2C();

    int openBus(uint8_t slaveAdr);									// Open bus using standard Linux file driver
    int closeBus();																// Close the open bus file
	int isReady();                                      						// Is the object ready to use

    int tx(uint8_t* bytes, int count);                  			// Transmit a buffer of @count bytes to an address on the bus
    int txReg(uint8_t reg, uint8_t* bytes, int count);  	// Transmit a buffer of bytes to an address with register offset
    int txByte(uint8_t bt);                             					// Transmit a byte to an address on the bus
    int txByte(uint8_t reg, uint8_t byte);              			// Transmit a byte to an address on the bus with register offset
    int txWord(uint16_t wd);                            				// Transmit a word to an address on the bus
    int txWord(uint8_t reg, uint16_t wd);               		// Transmit a word to an address on the bus with register offset

    int         		rx(uint8_t* bytes, int count);          		// Read a buffer of @count bytes from an address on the bus
    int         		rxReg(uint8_t reg, uint8_t* bytes);     	// Read a buffer of bytes from the an address on the bus with offset
    uint8_t     	rxByte();                               					// Read a byte from an address on the bus
    uint8_t     	rxByte(uint8_t reg);                    			// Read a byte from an address on the bus with register offset
    uint16_t    rxWord();                               					// Read a word from an address on the bus
    uint16_t    rxWord(uint8_t reg);                    			// Read a word from an address on the bus with register offset
};

#endif // I2C_H
