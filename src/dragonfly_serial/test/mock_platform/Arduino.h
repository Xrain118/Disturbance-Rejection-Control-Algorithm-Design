#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
using std::min;
class Print { public: virtual size_t write(uint8_t)=0; virtual ~Print()=default; };
class Stream: public Print {
 public:
  virtual int available()=0;virtual int read()=0;virtual int peek()=0;
  virtual void flush()=0;
};
class MockSerial: public Stream {
 public:
  std::deque<uint8_t> rx,pending;bool blocked=false;
  int available() override {return rx.size();}
  int read() override {if(rx.empty())return -1;int b=rx.front();rx.pop_front();return b;}
  int peek() override {return rx.empty()?-1:rx.front();}
  void flush() override {}
  int availableForWrite(){return blocked?0:128;}
  size_t write(uint8_t) override {return 1;}
  size_t write(const uint8_t*,size_t n){if(blocked)return 0;rx.insert(rx.end(),pending.begin(),pending.end());pending.clear();return n;}
};
inline MockSerial Serial;
