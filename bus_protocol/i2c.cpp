////////////////////////////////////////////////////////////////////////////////////////////////
//
//  i2c.cpp
//
//  Implementation file for an I2C communication object.  This is a wrapper to the functions
//  required to open and communicate over an I2C.
//

#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include "i2c.h"

I2C::I2C(char const* fname)
{
	m_fname = new char[32];
	strcpy(m_fname, fname);
	m_file = 0;
}

I2C::~I2C()
{
	delete m_fname;
	closeBus();
}

int I2C::openBus(uint8_t slaveAdr)
{
	int tmOut = 50;											// Wait until the object has been closed if it is still in use
	while (m_file != 0)									// Times out after approx 5ms and returns error value.
	{
			usleep(100);
			tmOut--;
			if (tmOut == 0)
				return  ERR_I2C_BSY;
	}

	tmOut = 50;												// Attempt to open the device file.  If still open or other
    m_file = open(m_fname, O_RDWR);		// error then retry for approx 5ms and return error code
	if (m_file < 0)												// if still not sucessful.
	{
		while(m_file < 0)
		{
			tmOut --;
			usleep(100);
			m_file = open(m_fname, O_RDWR);
		}
		return ERR_I2C_FILE;
	}

	if (ioctl(m_file, I2C_SLAVE, slaveAdr) < 0)	// Attempt to set the proper IOCTL values to communicate
		return ERR_I2C_IO;									// target device address.   Return error if not sucessful.

    return 0;
}

int I2C::closeBus()
{
    if (m_file > 0)
    {
        close(m_file);
        m_file = 0;
        return 0;
    }

    return ERR_I2C_GEN;
}

int I2C::isReady()
{
    struct stat st;
	if (stat(m_fname, &st) == 0)
		return 1;
	else
		return 0;
}

int I2C::tx(uint8_t* bytes, int count)
{// Write the data bytes to the device.
 //
    int result = -1;

    result = write(m_file, bytes, count);
    if ( result != count)
		result = ERR_I2C_IO;

    return result;
}

int I2C::txReg(uint8_t reg, uint8_t* bytes, int count)
{// Write data bytes to the specified register on the device.

	int result = -1;
	result = i2c_smbus_write_i2c_block_data(m_file, reg, count, bytes);
	if (result != count)
		result = ERR_I2C_IO;

	return result;
}

int I2C::txByte(uint8_t bt)
{
	int result = -1;
	result = write(m_file, &bt, 1);
	if (result != 1)
		result = ERR_I2C_IO;

	return result;
}

int I2C::txByte(uint8_t reg, uint8_t bt)
{
	return i2c_smbus_write_byte_data(m_file, reg, bt);
}

int I2C::txWord(uint16_t wd)
{
	int result = -1;
	uint8_t bytes[2];
	bytes [0] =  wd >> 8;
	bytes [1] =  wd;
	result = write(m_file, bytes, 2);
	if (result != 2)
		result = ERR_I2C_IO;

	return result;
}

int I2C::txWord(uint8_t reg, uint16_t wd)
{
	return i2c_smbus_write_word_data(m_file, reg, wd);
}


int I2C::rx(uint8_t* bytes, int count)
{
    int result = -1;
    result = read(m_file, bytes, count);
    if ( result != count)
		result = ERR_I2C_IO;

    return result;
}

int I2C::rxReg(uint8_t reg, uint8_t* bytes)
{
	return i2c_smbus_read_i2c_block_data(m_file, reg, bytes);
}

uint8_t I2C::rxByte()
{
	uint8_t bt;

	if (rx(&bt, 1) != 1)
		return ERR_I2C_IO;

	return bt;
}

uint8_t I2C::rxByte(uint8_t reg)
{
	return i2c_smbus_read_byte_data(m_file, reg);
}

uint16_t I2C::rxWord()
{
	uint8_t bt[2];

	if (rx(bt, 2) != 2)
		return ERR_I2C_IO;

	return (bt[0]<<8 | bt[1]);
}

uint16_t I2C::rxWord(uint8_t reg)
{
	return i2c_smbus_read_word_data(m_file, reg);
}
