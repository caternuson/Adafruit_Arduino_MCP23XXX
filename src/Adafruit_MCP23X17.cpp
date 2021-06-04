#include "Adafruit_MCP23X17.h"

Adafruit_MCP23X17::Adafruit_MCP23X17() {
  pinCount = 16;
}

uint8_t Adafruit_MCP23X17::readGPIOA() {
  return readGPIO(0);
}

void Adafruit_MCP23X17::writeGPIOA(uint8_t value) {
  writeGPIO(value, 0);
}

uint8_t Adafruit_MCP23X17::readGPIOB() {
  return readGPIO(1);
}

void Adafruit_MCP23X17::writeGPIOB(uint8_t value) {
  writeGPIO(value, 1);
}

uint16_t Adafruit_MCP23X17::readGPIOAB() {
  return readRegister16(getRegister(MCP23XXX_GPIO));
}

void Adafruit_MCP23X17::writeGPIOAB(uint16_t value) {
  writeRegister16(getRegister(MCP23XXX_GPIO), value);
}

