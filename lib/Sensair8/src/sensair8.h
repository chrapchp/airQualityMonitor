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
 *   ideas from https://gitlab.com/tue-umphy/co2mofetten/software/arduino-libraries/Sensair8
 *
 *
 */

#ifndef SENSAIR8_H
#define SENSAIR8_H

#include <stdint.h>
#include <SoftwareSerial.h>

#define RESPONSE_BUFFER_SZ 7

#define COMMAND_SZ 7
#define SENSAIR_BAUD 9600
#define COMMAND_TX_FAIL -1
#define COMMAND_TX_OK 0
#define COMMAND_RX_FAIL -1
#define COMMAND_RX_BAD_CRC -2


class Sensair8
{
public:
    Sensair8(
        HardwareSerial *port, // serial port reference
        Stream *debugPort = NULL // debug port, if used, assumes port is already opened
    );

    Sensair8(
        SoftwareSerial *port, // serial port reference
        Stream *debugPort = NULL // debug port, if used, assumes port is already opened
    );
    

    int16_t getCO2(uint16_t retries = 3);


private:
    Stream *_co2Port; // serial port reference
    uint8_t _responseBuffer[RESPONSE_BUFFER_SZ];
    Stream *_debugPort;

    bool isPortOpen = false;
    bool isHardwareSerial = false;

    void portBegin( uint16_t baud);

    // quasi modbus formated command to read CO2 from sensair
    // not sure why the 44 for read input register that proper modbus 04
    uint8_t _command[COMMAND_SZ];

    // read size bytes and try retries time from an serial stream
    // return COMMAND_TX_OK or COMMAND_TX_FAIL
    int16_t sendCommand(uint8_t cmd[], uint16_t size, uint16_t retries);

    // return COMMAND_RX_FAIL if failed to read a response or # of bytes read in the buffer
    int16_t readReply(uint8_t reponseBuffer[], uint16_t size, uint16_t retries);

    // clear the input stream buffer
    void emptyInputStream();

    // calculate CRC
    // https://ctlsys.com/support/how_to_compute_the_modbus_rtu_message_crc/
    int16_t calcCRC(uint8_t buffer[], uint16_t size);
};

#endif