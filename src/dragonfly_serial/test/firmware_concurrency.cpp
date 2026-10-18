#include "telemetry_platform.h"
#include <cassert>
#include <iostream>
class Probe: public DeadlineServo {
 public:using DeadlineServo::readSCS;using DeadlineServo::writeSCS;
};
int main() {
  telemetryPlatformInit();
  Probe servo;MockSerial uart;servo.pSerial=&uart;
  uint8_t byte=0;
  auto begin=esp_timer_get_time();
  {BusGuard guard;assert(guard);assert(servo.readSCS(&byte,1)==0);}
  assert(esp_timer_get_time()-begin<20000); // OS simulation tolerance, not MCU timing certification.
  uart.rx={1,2};
  {BusGuard guard;assert(guard);assert(servo.readSCS(&byte,1)==1 && byte==1);}
  uart.blocked=true;
  {BusGuard guard;assert(guard);assert(servo.writeSCS(&byte,1)==0);}
  uart.blocked=false;
  auto response=[&](uint8_t id,bool good){
    std::vector<uint8_t> packet={255,255,id,17,0};
    for(int i=0;i<15;++i)packet.push_back(i);
    uint8_t sum=id+17;for(int i=0;i<15;++i)sum+=i;
    packet.push_back(good?uint8_t(~sum):0);
    uart.pending.insert(uart.pending.end(),packet.begin(),packet.end());
  };
  uint8_t raw[15],fault=0;
  response(12,true);response(11,true);
  {BusGuard guard;assert(servo.readFeedback(11,raw,fault));assert(raw[14]==14);}
  response(12,true);
  {BusGuard guard;assert(!servo.readFeedback(11,raw,fault));}
  response(11,false);
  {BusGuard guard;assert(!servo.readFeedback(11,raw,fault));}
  std::atomic<int> inside{0},collisions{0},samples{0};
  auto transaction=[&](){if(++inside!=1)++collisions;vTaskDelay(1);--inside;};
  // Long motion/replay is a sequence of short writes plus waits, never a motion-wide lock.
  std::thread motion([&](){for(int i=0;i<60;++i){servoCall(transaction);vTaskDelay(4);}});
  std::thread queuedCommands([&](){for(int i=0;i<40;++i){servoCall(transaction);vTaskDelay(6);}});
  std::thread acquisition([&](){for(int i=0;i<100;++i){{BusGuard guard;if(guard){transaction();++samples;}}vTaskDelay(2);}});
  motion.join();queuedCommands.join();acquisition.join();
  assert(collisions==0 && samples>20);
  for(int i=0;i<100;++i)queueHostLine(i==99?"latest\n":"old\n",i==99?7:4,true);
  HostLine line{};assert(xQueueReceive(hostTelemetry,&line,0)==pdTRUE);
  assert(std::string(line.text,line.size)=="latest\n" && telemetryDropped==99);
  for(int i=0;i<100;++i)queueHostLine("log\n",4);
  assert(hostLogDropped==96 && uxQueueMessagesWaiting(hostLogs)==4);
  std::cout<<"bounded reads/writes, concurrent motion/replay/commands, latest-only queue passed; samples="<<samples<<"\n";
}
