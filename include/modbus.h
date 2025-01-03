/*
 * File: modbus.h
 * Author: Gantulga G
 * Date: Oct 23 2024
 *
 * Description:
 * TODO
 *
 * License:
 * This software is licensed under the Apache 2.0 License. See the LICENSE file
 * for details.
 *
 * Note:
 * TODO
 */

#ifndef __MODBUS__
#define __MODBUS__
#include <HardwareSerial.h>

#define MODBUS_REQUEST_READ_HOLDING 0x03
#define MODBUS_REQUEST_READ_INPUT 0x04
#define MODBUS_REQUEST_WRITE_SINGLE 0x06
#define MODBUS_REQUEST_WRITE_MULTIPLE 0x10

#define MODBUS_TX_BUFFER_SIZE 100
#define MODBUS_RX_BUFFER_SIZE 100

#define MODBUS_RX_TIMEOUT_MS 5000

/* CRC type punning union */
union CRC_CODE
{
    uint16_t word;   // Result from CRC checksum
    uint8_t byte[2]; // Bytes to write to TX buffer
};

class Modbus
{
public:
    Modbus(int8_t id, HardwareSerial *serial, int8_t rx, int8_t tx, int8_t de = -1, int8_t re = -1, bool crc = false);
    ~Modbus();

    void begin(uint32_t baudrate);
    uint8_t readHolding(uint16_t address, uint8_t read_count);
    uint8_t readInput(uint16_t address, uint8_t read_count);
    uint8_t writeSingle(uint16_t address, uint16_t data);
    uint8_t writeMultiple(uint16_t address, uint8_t *data, uint8_t write_count);
    uint32_t parseRX(uint8_t index, uint8_t size);

protected:
    void init_transfer(uint8_t request_type, uint16_t address);
    void serial_write(uint8_t *buffer, uint8_t len);
    uint16_t calculate_crc(uint8_t *buf, uint8_t len);
    uint8_t listen(uint8_t request_type);

public:
    uint8_t ID = 1;

protected:
    HardwareSerial *_serial;
    bool _crc = false;
    uint8_t _rx = 0;
    uint8_t _tx = 0;
    int8_t _de = -1;
    int8_t _re = -1;
    uint8_t _rx_buffer[MODBUS_RX_BUFFER_SIZE];
    uint8_t _tx_buffer[MODBUS_TX_BUFFER_SIZE];
};
#endif