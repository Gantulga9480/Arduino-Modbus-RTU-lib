#ifndef __MODBUS__
#define __MODBUS__
#include <HardwareSerial.h>

#define MODBUS_REQUEST_READ_HOLDING 0x03
#define MODBUS_REQUEST_WRITE_SINGLE 0x06

#define MODBUS_TX_BUFFER_SIZE 40
#define MODBUS_RX_BUFFER_SIZE 40

#define MODBUS_RX_TIMEOUT_MS 1000

#ifdef DEBUG
#define DEBUG_PRINT(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) \
    do                   \
    {                    \
    } while (0)
#endif

/* CRC type punning union */
union CRC_CODE
{
    uint16_t word;   // Result from CRC checksum
    uint8_t byte[2]; // Bytes to write to TX buffer
};

class Modbus
{
public:
    Modbus(int8_t id, HardwareSerial *serial, int8_t rx, int8_t tx, bool crc = false);
    ~Modbus();

    void begin(uint32_t baudrate);
    uint8_t readHolding(uint8_t address, uint8_t read_count);
    uint8_t writeSingle(uint8_t address, uint8_t data);
    uint32_t parseRX(uint8_t index, uint8_t size);

private:
    void init_transfer(uint8_t request_type, uint8_t address);
    void serial_write(uint8_t *buffer, uint8_t len);
    uint16_t calculate_crc(uint8_t *buf, uint8_t len);
    uint8_t listen(uint8_t request_type);

public:
    uint8_t ID = 1;
    uint8_t _rx_buffer[MODBUS_RX_BUFFER_SIZE];
    uint8_t _tx_buffer[MODBUS_TX_BUFFER_SIZE];

private:
    HardwareSerial *_serial;
    uint8_t _rx = 0;
    uint8_t _tx = 0;
    bool _crc = false;
};
#endif