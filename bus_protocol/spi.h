#ifndef SPI_H
#define SPI_H

#include <string>
#include <stdint.h>


class SPI
{
protected:
        int 	    m_fd;                   // File descriptor for the bus
        std::string m_fname;                // File name for the given channel
        int         m_speed;                // Requested speed.
        uint8_t     m_spiMode;              // Clock/latching mode of the bus
        uint8_t     m_spiBPW;               // Bits per word
        uint16_t    m_spiDelay;             // NO IDEA?

public:
    SPI(const std::string& fn);                    // Create an SPI object on the requested channel with default speed
    SPI(int speed, const std::string& fn);         // Create an SPI object with requested channel and speed
        virtual ~SPI();

        int openBus();                      // Open the bus file for reading and writing
        int closeBus();                     // Close the bus file if it is open
        int isReady();                      // Is the bus ready to use
        int setBPW(int val);                // Set the bits per word, returns the previous setting
        int setSpeed(int val);              // Set the bus speed, returns the previous setting
        int setMode(int val);               // Set the bus clock/latching mode.  Valid range: 0-3

        int      rwData(uint8_t *data, uint8_t len); // Send and receive a block of data.
        uint8_t  rwByte(uint8_t bt);        // Send and receive one byte of data
        uint16_t rwWord(uint16_t wd);       // Send and receive one word of data

protected:
    void init(int speed, const std::string& fn);
};

/**
  * Construct an SPI object on the requested channel with a default
  * clock rate.
  *
  */
inline SPI::SPI(const std::string& fn)
{
    init(500000, fn);
}

/**
  * Construct an SPI object on the requested channel with the requested
  * clock rate.
  *
  */
inline SPI::SPI(int speed, const std::string& fn)
{
    init(speed, fn);
}

/** @brief isReady - Indicates that the SPI object has a valid file handle.
  *
  * Returns true if the object has a valid file handle to the SPI device file.
  * Otherwise returns false.
  *
  */
inline int SPI::isReady()
{
    if(m_fd > 0)
        return 1;
    else
        return 0;
}

#endif // SPI_H
