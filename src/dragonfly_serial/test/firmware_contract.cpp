// Compile against the EXTERNAL firmware headers and installed ArduinoJson.
// No firmware sources are vendored into this repository.
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>
#include "telemetry_wire.h"

int main(int argc,char** argv) {
  assert(argc==3);
  std::ofstream generated(argv[2]);
  using namespace TelemetryMath;
  assert(signedWord(0x8001)==-1);
  assert(signedWord(0xffff)==-32767);
  assert(signedWord(0x0401,10)==-1);
  assert(std::abs(torque(0,-1)+.00701175475)<1e-8);
  assert(std::abs(torque(6,100)-.5290687675)<1e-6);
  uint32_t missed=0;
  assert(nextDeadline(0,19000,20000,missed)==20000 && missed==0);
  assert(nextDeadline(0,65000,20000,missed)==80000 && missed==3);
  for(auto name:{"valid.json","worst.json"}) {
    std::ifstream input(std::string(argv[1])+"/"+name);
    JsonDocument vector;
    assert(deserializeJson(vector,input)==DeserializationError::Ok);
    ArmSnapshot snap{};
    snap.seq=vector["seq"];snap.timeUs=vector["t_us"];
    snap.scanUs=vector["scan_us"];snap.overrun=vector["overrun"];
    for(unsigned i=0;i<7;++i) {
      auto& s=snap.sample[i];s.valid=true;
      s.pos=vector["pos"][i];s.speed=vector["speed"][i];
      s.current=vector["current"][i];s.load=vector["load"][i];
      s.voltage=vector["voltage"][i];s.temperature=vector["temperature"][i];
      s.status=vector["status"][i];s.readUs=vector["read_us"][i];
    }
    JsonDocument wire;
    fillTelemetry(wire,snap,vector["dev"],vector["boot"],vector["rate"],vector["dropped"]);
    assert(!wire.overflowed() && measureJson(wire)+1<=1200);
    for(unsigned i=0;i<7;++i) {
      assert(std::abs(wire["tau"][i].as<double>()-vector["tau"][i].as<double>())<.00003);
      assert(wire["current"][i].as<int>()==vector["current"][i].as<int>());
    }
    serializeJson(wire,generated);generated<<"\n";
    std::cout<<name<<" firmware bytes="<<measureJson(wire)+1<<"\n";
    // Failed transaction gets a fresh invalid slot, never the previous measurement.
    snap.sample[2]=ServoSample{};
    wire.clear();fillTelemetry(wire,snap,"test",1,50,0);
    assert(!wire["ok"][2].as<bool>());
    for(auto key:{"pos","speed","current","load","voltage","temperature","status","error","tau","read_us"})
      assert(wire[key][2].isNull());
  }
  std::cout<<"firmware math, scheduler, nulls and shared wire vectors passed\n";
}
