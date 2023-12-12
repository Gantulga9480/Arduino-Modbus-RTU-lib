#ifndef __FLOW_METER__
#define __FLOW_METER__
#include <HardwareSerial.h>

#define MODBUS_REQUEST_READ_HOLDING 0x03
#define MODBUS_REQUEST_WRITE_SINGLE 0x06

#define MODBUS_TX_BUFFER_SIZE       40
#define MODBUS_RX_BUFFER_SIZE       40

#define MODBUS_RX_TIMEOUT_MS        1000

/* CRC type punning union */
union CRC_CODE
{
  uint16_t word;    // Result from CRC checksum
  uint8_t byte[2];  // Bytes to write to TX buffer
};

class FlowMeter
{
public:
    FlowMeter(int8_t id, HardwareSerial *serial, int8_t rx, int8_t tx,  bool crc = false);
    ~FlowMeter();

    void begin(uint32_t baudrate);

    float readTotalLiters();
    bool reset();

private:
    void init_transfer(uint8_t request_type, uint8_t address, uint8_t count);
    void serial_write(uint8_t *buffer, uint8_t len);
    uint8_t read(uint8_t address, uint8_t read_count = 1);
    uint8_t write(uint8_t address, uint8_t data);
    uint8_t listen(uint8_t request_type);
    uint16_t CRC(uint8_t *buf, uint8_t len);
    uint32_t parse(uint8_t index, uint8_t size);

public:
    uint8_t ID = 1;
    bool serial_debug = false;

private:
    HardwareSerial *_serial;
    uint16_t _baudrate = 4800;
    uint8_t _rx = 0;
    uint8_t _tx = 0;
    bool _crc = false;
    uint8_t rx_buffer[MODBUS_RX_BUFFER_SIZE];
    uint8_t tx_buffer[MODBUS_TX_BUFFER_SIZE];
    uint8_t tx_buffer_len = 0;
    uint8_t rx_buffer_len = 0;
};
#endif