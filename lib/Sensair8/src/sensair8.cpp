/**
 *  @file    sensair8.h
 *  @author  peter c
 *  @date    11/24/2021
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Sensair 8 interface to read CO2 values, nothing fancy.
 *   https://senseair.com/products/size-counts/s8-commercial/
 *
 */

#include <Streaming.h>

#include "sensair8.h"

Sensair8::Sensair8(
    SoftwareSerial *port, // serial port reference
    Stream *debugPort) : _command{0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25}
{

    _co2Port = port;
    isHardwareSerial = false;
    _debugPort = debugPort;
}

Sensair8::Sensair8(
    HardwareSerial *port, // serial port reference
    Stream *debugPort) : _command{0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25}
{
    _co2Port = port;
    isHardwareSerial = true;
    _debugPort = debugPort;
}

void Sensair8::portBegin(uint16_t baud)
{
    if (isHardwareSerial)
        static_cast<HardwareSerial *>(_co2Port)->begin(baud);
    else
        static_cast<SoftwareSerial *>(_co2Port)->begin(baud);

    isPortOpen = true;
}

int16_t Sensair8::getCO2(uint16_t retries)
{
    int16_t retVal = COMMAND_TX_FAIL;
    if (!isPortOpen)
        portBegin(SENSAIR_BAUD);
    if (_debugPort)
        *_debugPort << "in getCO2()..." << endl;

    if (sendCommand(_command, sizeof(_command), retries) == COMMAND_TX_OK)
    {
        if (readReply(_responseBuffer, RESPONSE_BUFFER_SZ, retries) != COMMAND_RX_FAIL)
        {
            // Calculate CRC include CRC data, result should be 0 if good
            if (calcCRC(_responseBuffer, RESPONSE_BUFFER_SZ) == 0)
                retVal = 256 * _responseBuffer[3] + _responseBuffer[4];
            else
                retVal = COMMAND_RX_BAD_CRC;
        }
    }
    return retVal;
}

int16_t Sensair8::sendCommand(uint8_t cmd[], uint16_t size, uint16_t retries)
{
    int16_t retryCount = 0;
    emptyInputStream();
    while (!(_co2Port->available()))
    {
        retryCount++;
        if (_debugPort)
            *_debugPort << "Sending Command and waiting" << endl;

        _co2Port->write(_command, size);
        delay(50);
        retryCount++;
        if (retryCount > (retries))
            return COMMAND_TX_FAIL;
    }
    return COMMAND_TX_OK;
}

int16_t Sensair8::readReply(uint8_t reponseBuffer[], uint16_t size, uint16_t retries)
{
    int16_t retryCount = 0;
    int16_t retVal = COMMAND_RX_FAIL;

    while (_co2Port->available() < size)
    {
        retryCount++;
        // waited long enough, abort readin and clear out the input stream
        if (retryCount > retries)
        {
            emptyInputStream();
            break;
        }
        delay(50);
    }

    for (int i = 0; i < size; i++)
    {
        _responseBuffer[i] = _co2Port->read();
        retVal += i + 1;
        if (_debugPort)
        {
            *_debugPort << "Rx Buffer at  index " << i << " assigned value of ";
            _debugPort->println(_responseBuffer[i], HEX);
        }
    }
    return retVal;
}

void Sensair8::emptyInputStream()
{
    if (_co2Port->available())
    {
        if (_debugPort)
            *_debugPort << "Disgarding " << _co2Port->available() << " bytes from CO2 sensor" << endl;

        while (_co2Port->available())
        {

            _co2Port->read();
            delay(10);
        }
    }
}

int16_t Sensair8::calcCRC(uint8_t buffer[], uint16_t size)
{
    uint16_t crc(0xFFFF);

    for (int pos = 0; pos < size; pos++)
    {
        crc ^= (uint16_t)buffer[pos]; // XOR byte into least sig. byte of crc

        for (int i = 8; i != 0; i--)
        { // Loop over each bit
            if ((crc & 0x0001) != 0)
            {              // If the LSB is set
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else           // Else LSB is not set
                crc >>= 1; // Just shift right
        }
    }

    return crc;
}
