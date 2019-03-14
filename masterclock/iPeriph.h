#pragma once
#include "cmsis_os.h"
class IPeriph
{
public:
  virtual void init() = 0;
  virtual void deInit() = 0;
  virtual void transmitBuffUInt8(uint8_t *buff, uint16_t length, uint32_t timeout=1000)=0;
  virtual void transmitUInt8(uint8_t data, uint32_t timeout=1000) = 0;
  virtual void transmitRecieveUint8(uint8_t tx, uint8_t* rx, uint32_t timeout = 1000)=0;
};

