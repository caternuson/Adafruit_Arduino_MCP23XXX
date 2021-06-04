#ifndef __ADAFRUIT_MCP23X17_H__
#define __ADAFRUIT_MCP23X17_H__

#include "Adafruit_MCP23XXX.h"

class Adafruit_MCP23X17: public Adafruit_MCP23XXX {
public:
  Adafruit_MCP23X17();

  uint8_t readGPIOA();
  void writeGPIOA(uint8_t value);
  uint8_t readGPIOB();
  void writeGPIOB(uint8_t value);
  uint16_t readGPIOAB();
  void writeGPIOAB(uint16_t value);
};

#endif