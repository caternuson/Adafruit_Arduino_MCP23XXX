#include "Adafruit_MCP23XXX.h"

bool Adafruit_MCP23XXX::begin_I2C(uint8_t i2c_addr, TwoWire *wire) {
  i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);
  return i2c_dev->begin();
}

bool Adafruit_MCP23XXX::begin_SPI(uint8_t cs_pin, SPIClass *theSPI) {
  spi_dev = new Adafruit_SPIDevice(cs_pin, 1000000, SPI_BITORDER_MSBFIRST, SPI_MODE0, theSPI);
  return spi_dev->begin();
}

bool Adafruit_MCP23XXX::begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin) {
  spi_dev = new Adafruit_SPIDevice(cs_pin, sck_pin, miso_pin, mosi_pin);
  return spi_dev->begin();
}

void Adafruit_MCP23XXX::pinMode(uint8_t pin, uint8_t mode) {
  uint8_t iodir_reg = getRegister(MCP23XXX_IODIR, PORT(pin));
  uint8_t gppu_reg = getRegister(MCP23XXX_GPPU, PORT(pin));

  uint8_t iodir = readRegister(iodir_reg);

  if (mode == OUTPUT) {
    // clear for output
    iodir &= ~MASK(pin);
  } else {
    // set for input
    iodir |= MASK(pin);
    // also configure internal pull-up
    uint8_t gppu = readRegister(gppu_reg);
    if (mode == INPUT_PULLUP) {
      // set to enable
      gppu |= MASK(pin);
    } else {
      // clear to disable
      gppu &= ~MASK(pin);
    }
    writeRegister(gppu_reg, gppu);
  }
  writeRegister(iodir_reg, iodir);
}

uint8_t Adafruit_MCP23XXX::digitalRead(uint8_t pin) {
  if (pin >= pinCount) return 0;
  return ((readGPIO(PORT(pin)) & MASK(pin)) == 0) ? 0 : 1;
}

void Adafruit_MCP23XXX::digitalWrite(uint8_t pin, uint8_t value) {
  uint8_t gpio = readGPIO(PORT(pin));
  if (value == HIGH) {
    gpio |= MASK(pin);
  } else {
    gpio &= ~MASK(pin);
  }
  writeGPIO(gpio, PORT(pin));
}

uint8_t Adafruit_MCP23XXX::readGPIO(uint8_t port) {
  return readRegister(getRegister(MCP23XXX_GPIO, port));
}

void Adafruit_MCP23XXX::writeGPIO(uint8_t value, uint8_t port) {
  writeRegister(getRegister(MCP23XXX_GPIO, port), value);
}

void Adafruit_MCP23XXX::configureInterrupt(uint8_t mirroring, uint8_t open, uint8_t polarity) {
  uint8_t iocon = readRegister(getRegister(MCP23XXX_IOCON));
  if (mirroring) iocon |= 1 << 6; else iocon &= ~(1 << 6);
  if (open) iocon |= 1 << 2; else iocon &= ~(1 << 2);
  if (polarity) iocon |= 1 << 1; else iocon &= ~(1 << 1);
  writeRegister(getRegister(MCP23XXX_IOCON), iocon);
}

void Adafruit_MCP23XXX::enableInterrupt(uint8_t pin, bool useDefVal) {
  // enable it
  uint8_t reg = getRegister(MCP23XXX_GPINTEN, PORT(pin));
  uint8_t gpinten = readRegister(reg);
  gpinten |= MASK(pin);
  writeRegister(reg, gpinten);
  // set comparison mode
  reg = getRegister(MCP23XXX_INTCON, PORT(pin));
  uint8_t intcon = readRegister(reg);
  if (useDefVal) intcon |= MASK(pin); else intcon &= ~MASK(pin);
  writeRegister(reg, intcon);
}

void Adafruit_MCP23XXX::disableInterrupt(uint8_t pin) {
  uint8_t reg = getRegister(MCP23XXX_GPINTEN, PORT(pin));
  uint8_t gpinten = readRegister(reg);
  gpinten &= ~MASK(pin);
  writeRegister(reg, gpinten);
}

uint8_t Adafruit_MCP23XXX::getDefaultValue(uint8_t pin) {
  uint8_t defval = readRegister(getRegister(MCP23XXX_DEFVAL, PORT(pin)));
  return ( (defval & MASK(pin) == 0) ? 0 : 1);
}

void Adafruit_MCP23XXX::setDefaultValue(uint8_t pin, uint8_t value) {
  uint8_t reg = getRegister(MCP23XXX_DEFVAL, PORT(pin));
  uint8_t defval = readRegister(reg);
  if (value) defval |= MASK(pin); else defval &= ~MASK(pin);
  writeRegister(reg, defval);
}

uint8_t Adafruit_MCP23XXX::getLastInterruptPin() {
  uint8_t intf = readRegister(getRegister(MCP23XXX_INTF));
  // Port A
  for (uint8_t pin = 0; pin < 8; pin++) {
    if (intf & (1 << pin)) return pin;
  }
  // Port B
  if (pinCount > 8) {
    intf = readRegister(getRegister(MCP23XXX_INTF, 1));
    for (uint8_t pin = 0; pin < 8; pin++) {
      if (intf & (1 << pin)) return pin + 8;
    }
  }
  return 0;
}

uint8_t Adafruit_MCP23XXX::readRegister(uint8_t addr) {
  if (i2c_dev) {
    buffer[0] = addr;
    i2c_dev->write_then_read(buffer, 1, buffer, 1, false);
  } else if (spi_dev) {
    buffer[0] = MCP23XXX_SPI_READ;
    buffer[1] = addr;
    spi_dev->write_then_read(buffer, 2, buffer, 1);
  }
  return buffer[0];
}

void Adafruit_MCP23XXX::writeRegister(uint8_t addr, uint8_t value) {
  if (i2c_dev) {
    buffer[0] = addr;
    buffer[1] = value;
    i2c_dev->write(buffer, 2);
  } else if (spi_dev) {
    buffer[0] = MCP23XXX_SPI_WRITE;
    buffer[1] = addr;
    buffer[2] = value;
    spi_dev->write(buffer, 3);
  }
}

uint16_t Adafruit_MCP23XXX::readRegister16(uint8_t addr) {
  if (i2c_dev) {
    buffer[0] = addr;
    i2c_dev->write_then_read(buffer, 1, buffer, 2, false);
  } else if (spi_dev) {
    buffer[0] = MCP23XXX_SPI_READ;
    buffer[1] = addr;
    spi_dev->write_then_read(buffer, 2, buffer, 2);
  }
  return buffer[0] | (buffer[1] << 1);
}

void Adafruit_MCP23XXX::writeRegister16(uint8_t addr, uint16_t value) {
  if (i2c_dev) {
    buffer[0] = addr;
    buffer[1] = value & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    i2c_dev->write(buffer, 3);
  } else if (spi_dev) {
    buffer[0] = MCP23XXX_SPI_WRITE;
    buffer[1] = addr;
    buffer[2] = value & 0xFF;
    buffer[3] = (value >> 8) & 0xFF;
    spi_dev->write(buffer, 4);
  }
}

uint8_t Adafruit_MCP23XXX::getRegister(uint8_t baseAddress, uint8_t port) {
  // MCP23x08
  uint8_t reg = baseAddress;
  // MCP23x17 BANK=0
  if (pinCount > 8) {
    reg *= 2;
    // Port B
    if (port) reg++;
  }
  return reg;
}