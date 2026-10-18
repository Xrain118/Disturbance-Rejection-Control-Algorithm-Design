#pragma once
#include "Arduino.h"
// Only the UART/RTOS boundary is mocked; DeadlineServo and BusGuard are production headers.
class SMS_STS {
 public: MockSerial* pSerial=nullptr;
 protected:
  virtual int readSCS(unsigned char*,int)=0;
  virtual int writeSCS(unsigned char*,int)=0;
  virtual int writeSCS(unsigned char)=0;
  virtual void rFlushSCS()=0;
  virtual void wFlushSCS()=0;
};
