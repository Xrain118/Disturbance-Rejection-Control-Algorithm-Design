#pragma once
#include <cassert>
#include <chrono>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY 0xffffffff
#define pdMS_TO_TICKS(x) (x)
#define configASSERT(x) assert(x)
using SemaphoreHandle_t=std::timed_mutex*;
inline auto xSemaphoreCreateMutex(){return new std::timed_mutex;}
inline int xSemaphoreTake(SemaphoreHandle_t m,unsigned t){if(t==portMAX_DELAY){m->lock();return 1;}return m->try_lock_for(std::chrono::milliseconds(t));}
inline void xSemaphoreGive(SemaphoreHandle_t m){m->unlock();}
inline void taskYIELD(){std::this_thread::yield();}
inline void vTaskDelay(unsigned t){std::this_thread::sleep_for(std::chrono::milliseconds(t));}
inline int xTaskCreate(void(*)(void*),const char*,unsigned,void*,unsigned,void*){return pdPASS;}
struct MockQueue {size_t capacity,size;std::deque<std::vector<char>> data;std::mutex mutex;};
using QueueHandle_t=MockQueue*;
inline auto xQueueCreate(unsigned n,unsigned size){auto q=new MockQueue;q->capacity=n;q->size=size;return q;}
inline unsigned uxQueueMessagesWaiting(QueueHandle_t q){std::lock_guard<std::mutex> lock(q->mutex);return q->data.size();}
inline int xQueueSend(QueueHandle_t q,const void* data,unsigned){std::lock_guard<std::mutex> lock(q->mutex);if(q->data.size()==q->capacity)return 0;q->data.emplace_back((char*)data,(char*)data+q->size);return 1;}
inline void xQueueOverwrite(QueueHandle_t q,const void* data){std::lock_guard<std::mutex> lock(q->mutex);q->data.clear();q->data.emplace_back((char*)data,(char*)data+q->size);}
inline int xQueueReceive(QueueHandle_t q,void* data,unsigned){std::lock_guard<std::mutex> lock(q->mutex);if(q->data.empty())return 0;memcpy(data,q->data.front().data(),q->size);q->data.pop_front();return 1;}
