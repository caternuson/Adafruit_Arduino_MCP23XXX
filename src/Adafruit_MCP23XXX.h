#ifndef __ADAFRUIT_MCP23XXX_H__
#define __ADAFRUIT_MCP23XXX_H__

#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>


// registers
#define MCP23XXX_IODIR 0x00   //!< I/O direction register
#define MCP23XXX_IPOL 0x01    //!< Input polarity register
#define MCP23XXX_GPINTEN 0x02 //!< Interrupt-on-change control register
#define MCP23XXX_DEFVAL                                                        \
  0x03 //!< Default compare register for interrupt-on-change
#define MCP23XXX_INTCON 0x04 //!< Interrupt control register
#define MCP23XXX_IOCON 0x05  //!< Configuration register
#define MCP23XXX_GPPU 0x06   //!< Pull-up resistor configuration register
#define MCP23XXX_INTF 0x07   //!< Interrupt flag register
#define MCP23XXX_INTCAP 0x08 //!< Interrupt capture register
#define MCP23XXX_GPIO 0x09   //!< Port register
#define MCP23XXX_OLAT 0x0A   //!< Output latch register

#define MCP23XXX_ADDR  0x20   //!< Default I2C Address
#define MCP23XXX_SPI_WRITE 0x40   //!< Opcode for SPI write
#define MCP23XXX_SPI_READ  0x41   //!< Opcode for SPI read

#define PORT(pin) ((pin < 8) ? 0 : 1)
#define MASK(pin) (1 << (pin % 8))

class Adafruit_MCP23XXX {
public:
  // init
  bool begin_I2C(uint8_t i2c_addr = MCP23XXX_ADDR, TwoWire *wire = &Wire);
  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin);

  // main Arduino API methods
  void pinMode(uint8_t pin, uint8_t mode);
  uint8_t digitalRead(uint8_t pin);
  void digitalWrite(uint8_t pin, uint8_t value);

  // bulk access
  uint8_t readGPIO(uint8_t port=0);
  void writeGPIO(uint8_t value, uint8_t port=0);

  // interrupts
  void configureInterrupt(uint8_t mirroring, uint8_t open, uint8_t polarity);
  void enableInterrupt(uint8_t pin, bool useDefVal=false);
  void disableInterrupt(uint8_t pin);
  uint8_t getDefaultValue(uint8_t pin);
  void setDefaultValue(uint8_t pin, uint8_t value);
  uint8_t getLastInterruptPin();

protected:
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface
  uint8_t pinCount; ///< Total number of GPIO pins
  uint8_t readRegister(uint8_t addr);
  void writeRegister(uint8_t addr, uint8_t value);
  uint16_t readRegister16(uint8_t addr);
  void writeRegister16(uint8_t addr, uint16_t value);
  uint8_t getRegister(uint8_t baseAddress, uint8_t port=0);

private:
  uint8_t buffer[4];
};

#endif