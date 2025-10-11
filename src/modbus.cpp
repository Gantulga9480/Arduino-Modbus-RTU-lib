/*
 * File: modbus.cpp
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

#include "modbus.h"

#ifdef DEBUG
#include <debug.h>
#ifdef MODBUS_DEBUG
#define MODBUS_DEBUG_PRINT(...) DBG_RAW(__VA_ARGS__)
#else
#define MODBUS_DEBUG_PRINT(...) \
  do                            \
  {                             \
  } while (0);
#endif
#else
#define MODBUS_DEBUG_PRINT(...) \
  do                            \
  {                             \
  } while (0);
#endif

#ifndef MODBUS_USE_SW_SERIAL
Modbus::Modbus(int8_t id, HardwareSerial *serial, int8_t de, int8_t re, bool crc)
    : ID(id), _serial(serial), _de(de), _re(re), _crc(crc)
#else
Modbus::Modbus(int8_t id, SoftwareSerial *serial, int8_t de, int8_t re, bool crc)
    : ID(id), _serial(serial), _de(de), _re(re), _crc(crc)
    #endif
{
  if (_de > -1)
    pinMode(_de, OUTPUT);
  if (_re > -1)
    pinMode(_re, OUTPUT);
}

Modbus::~Modbus()
{
  _serial->end();
}

uint32_t Modbus::parseRX(uint8_t index, uint8_t size)
{
  uint32_t value = 0;
  uint8_t i = 3;
  for (; i < (size + 3); i++)
    value |= (_rx_buffer[i + index] << (8 * (size - (i + 1 - 3))));
  return value;
}

void Modbus::init_transfer(uint8_t request_type, uint16_t address)
{
  /* Clear TX buffer */
  memset(_tx_buffer, 0, 40);

  /* Set slave ID and request type */
  _tx_buffer[0] = ID;
  _tx_buffer[1] = request_type; // Read or write
  _tx_buffer[2] = address >> 8; // Requested address to read/write high byte
  _tx_buffer[3] = address;      // Requested address to read/write low byte
}

uint8_t Modbus::readHolding(uint16_t address, uint8_t read_count)
{
  /* Initialize TX buffer for read request */
  init_transfer(MODBUS_REQUEST_READ_HOLDING, address);

  /* Number of registers to read */
  _tx_buffer[4] = 0;
  _tx_buffer[5] = read_count;

  /* Compute CRC16 code from current tx_buffer data */
  CRC_CODE crc;
  crc.word = calculate_crc(_tx_buffer, 6);

  /* Place CRC16 code into tx_buffer */
  _tx_buffer[6] = crc.byte[1];
  _tx_buffer[7] = crc.byte[0];

  /* Send request command to slave device over UART */
  serial_write(_tx_buffer, 8);

  return listen(MODBUS_REQUEST_READ_HOLDING);
}

uint8_t Modbus::readInput(uint16_t address, uint8_t read_count)
{
  /* Initialize TX buffer for read request */
  init_transfer(MODBUS_REQUEST_READ_INPUT, address);

  /* Number of registers to read */
  _tx_buffer[4] = 0;
  _tx_buffer[5] = read_count;

  /* Compute CRC16 code from current tx_buffer data */
  CRC_CODE crc;
  crc.word = calculate_crc(_tx_buffer, 6);

  /* Place CRC16 code into tx_buffer */
  _tx_buffer[6] = crc.byte[1];
  _tx_buffer[7] = crc.byte[0];

  /* Send request command to slave device over UART */
  serial_write(_tx_buffer, 8);

  return listen(MODBUS_REQUEST_READ_INPUT);
}

uint8_t Modbus::writeSingle(uint16_t address, uint16_t data)
{
  /* Initialize TX buffer for write request */
  init_transfer(MODBUS_REQUEST_WRITE_SINGLE, address);

  /* A $data byte to write to register at $address */
  _tx_buffer[4] = data >> 8; // Data high byte
  _tx_buffer[5] = data;      // Data low byte

  /* Compute CRC16 code from current tx_buffer data */
  CRC_CODE crc;
  crc.word = calculate_crc(_tx_buffer, 6);

  /* Place CRC16 code into tx_buffer */
  _tx_buffer[6] = crc.byte[1];
  _tx_buffer[7] = crc.byte[0];

  /* Send request command to slave device over UART */
  serial_write(_tx_buffer, 8);

  return listen(MODBUS_REQUEST_WRITE_SINGLE);
}

uint8_t Modbus::writeMultiple(uint16_t address, uint8_t *data, uint8_t write_count)
{
  /* Initialize TX buffer for write request */
  init_transfer(MODBUS_REQUEST_WRITE_MULTIPLE, address);

  /* Number of registers to write */
  _tx_buffer[5] = 0;
  _tx_buffer[5] = write_count;

  /* Data length */
  _tx_buffer[6] = write_count * 2;

  /* Write data to TX buffer */
  uint8_t i = 0;
  for (; i < write_count * 2; i++)
  {
    _tx_buffer[i + 7] = data[write_count * 2 - 1 - i];
  }

  /* Compute CRC16 code from current tx_buffer data */
  CRC_CODE crc;
  crc.word = calculate_crc(_tx_buffer, i + 7);

  /* Place CRC16 code into tx_buffer */
  _tx_buffer[7 + i] = crc.byte[1];
  _tx_buffer[8 + i] = crc.byte[0];

  /* Send request command to slave device over UART */
  serial_write(_tx_buffer, 9 + i);

  return listen(MODBUS_REQUEST_WRITE_MULTIPLE);
}

void Modbus::serial_write(uint8_t *buffer, uint8_t len)
{
  if (_de > -1)
    digitalWrite(_de, HIGH);
  if (_re > -1)
    digitalWrite(_re, HIGH);
  MODBUS_DEBUG_PRINT("RTU:TX >> ");
  for (uint8_t i = 0; i < len; i++)
  {
    _serial->write(buffer[i]);
    MODBUS_DEBUG_PRINT("%02X ", buffer[i]);
  }
  MODBUS_DEBUG_PRINT("\n");
  _serial->flush();
  if (_de > -1)
    digitalWrite(_de, LOW);
  if (_re > -1)
    digitalWrite(_re, LOW);
}

uint8_t Modbus::listen(uint8_t request_type)
{
  uint8_t received_data = 0;
  uint8_t remaining_data_length = 0;
  uint8_t data_idx = 0;
  uint8_t status = 1; // OK

  MODBUS_DEBUG_PRINT("RTU:RX << ");

  /* Clear RX buffer before getting data */
  memset(_rx_buffer, 0, MODBUS_RX_BUFFER_SIZE);

  uint64_t last_byte_time_ms = millis();

  while (((millis() - last_byte_time_ms) <= MODBUS_RX_TIMEOUT_MS))
  {
    if (_serial->available())
    {
      received_data = _serial->read();
      _rx_buffer[data_idx++] = received_data;
      last_byte_time_ms = millis();
      MODBUS_DEBUG_PRINT("%02X ", received_data);
      switch (data_idx)
      {
      case 1: // Slave device ID in first position (Removed ID check)
        break;
      case 2: // Request type echo in second position
        if (received_data != request_type)
          status = 0; // Error
        break;
      case 3: // Remaining data length in third position if reading registers (bytes, +2 CRC checksum bytes)
        remaining_data_length = request_type <= MODBUS_REQUEST_READ_INPUT ? received_data + 2 : MODBUS_RX_BUFFER_SIZE;
        break;
      default:
        // IF status OK, consume all remaining data
        // ELSE consume until timeout
        break;
      }
    }
    // IF received all data stop listening RX line
    if (data_idx == remaining_data_length + 3)
      break;

    // IF RX buffer is full stop listening RX line
    if (data_idx == MODBUS_RX_BUFFER_SIZE)
      break;

    // yield current task
    delay(1);
  }
  MODBUS_DEBUG_PRINT("\n");

  if (data_idx < 3)
    status = 0; // Read nothing within MODBUS_RX_TIMEOUT_MS

  if (status)
  {
    CRC_CODE received_crc;
    received_crc.byte[1] = _rx_buffer[data_idx - 2];
    received_crc.byte[0] = _rx_buffer[data_idx - 1];

    CRC_CODE computed_crc;
    computed_crc.word = calculate_crc(_rx_buffer, data_idx - 2);

    if (received_crc.word != computed_crc.word)
    {
      MODBUS_DEBUG_PRINT("RTU:CRC CHECK FAILED!\n");
      status = 0;
    }
  }

  if (!status)
  {
    MODBUS_DEBUG_PRINT("RTU:READ FAILED!\n");
  }

  return status;
}

uint16_t Modbus::calculate_crc(uint8_t *buf, uint8_t len)
{
  uint8_t hi, lo, i;
  uint16_t crc = 0xFFFF;
  for (i = 0; i < len; i++)
  {
    uint8_t j, chk;
    crc = crc ^ *buf;
    for (j = 0; j < 8; j++)
    {
      chk = (uint8_t)(crc & 1);
      crc = crc >> 1;
      crc = crc & 0x7fff;
      if (chk == 1)
        crc = crc ^ 0xa001;
      crc = crc & 0xffff;
    }
    buf++;
  }
  hi = (uint8_t)(crc % 256);
  lo = (uint8_t)(crc / 256);
  crc = (((uint16_t)(hi)) << 8) | lo;
  return crc;
}
