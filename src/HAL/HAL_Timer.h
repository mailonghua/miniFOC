#ifndef __HAL_TIMER_H__
#define __HAL_TIMER_H__
#include <Ticker.h>
#include <string>
#include <vector>
#include <semphr.h>
#include "LOG.h"
/*https://www.cnblogs.com/manastudent/p/17126402.html*/
using namespace std;
DEBUG_SET_LEVEL(DEBUG_LEVEL_DEBUG);
/*
Ex:
  sys_timer* timePtr = sys_timer::get_instance();
  timePtr->CreateStartTimmer(1000,time1_callback);
*/

static SemaphoreHandle_t xMutexCallBack;

typedef std::function<void(void *)> time_callback_t;
// 描述一个定时器对象
typedef struct
{
  int32_t time_ms;
  time_callback_t callbackPtr; // 时间回调函数
  void *paraPtr;
  Ticker *tickerPtr; // 时间句柄
  int32_t handle;
} timeItem_t;
// 用于描述同一个定时时间的一组定时时间的对象
class TimeObj
{
public:
  TimeObj() { subPtr.reserve(10); }
  vector<timeItem_t *> subPtr;
  void callback()
  {
    xSemaphoreTake(xMutexCallBack, portMAX_DELAY);
    for (int numCB = 0; numCB < subPtr.size(); numCB++)
    {
      if (nullptr == subPtr[numCB]->callbackPtr)
      {
        ERR("This is bugs...\n");
      }
      else
      {
        subPtr[numCB]->callbackPtr(subPtr[numCB]->paraPtr);
      }
    }
    xSemaphoreGive(xMutexCallBack);
  }
};
// 单例模式
class sys_timer
{
public:
  sys_timer()
  {
    INFO("Start create timmer...\n");
    xMutexCallBack = xSemaphoreCreateMutex();
  }
  sys_timer(const sys_timer &) = delete;            // 显示禁用带参构造
  sys_timer &operator=(const sys_timer &) = delete; // 禁用拷贝构造

  static sys_timer *get_instance()
  {
    static sys_timer instance; // 通过局部静态变量保证了线程安全
    return &instance;
  }

  int32_t CreateStartTimmer(uint16_t time_ms, const time_callback_t cb,
                            void *para = NULL)
  {
    if ((time_ms == 0) || (cb == NULL))
    {
      ERR("Input create para error[time=%d][ptr=%p]\n", time_ms, cb);
      return -1;
    }
    else
    {
      timeItem_t *timeObj = new timeItem_t();
      Ticker *timeFd = new Ticker();
      timeObj->time_ms = time_ms;
      timeObj->tickerPtr = timeFd;
      timeObj->callbackPtr = cb;
      timeObj->paraPtr = para;
      timeObj->handle = ++currentTimeHandle;
      push_in_timmer(timeObj);
    }
    return currentTimeHandle;
  }

  bool DeleteTimmer(int32_t timeHandle)
  {
    if (timeHandle <= 0)
    {
      ERR("Input timeHandle error...This is bug\n");
      return false;
    }
    for (uint32_t i = 0; i < timeListObj.size(); i++)
    {
      TimeObj *timerPtr = timeListObj[i];
      for (uint32_t j = 0; j < timerPtr->subPtr.size(); j++)
      {
        if (timerPtr->subPtr[j]->handle == timeHandle)
        {
          xSemaphoreTake(xMutexCallBack, portMAX_DELAY);
          // 当前要关闭包含启动定时器句柄的那个time 对象
          if (j == 0)
          {
            if (timerPtr->subPtr.size() == 1)
            { // 只剩下一个则直接关闭硬件timer
              timerPtr->subPtr[0]->tickerPtr->detach();
            }
            else
            { // 将启动定时器的句柄传递到下一个存在的timer对象中，为了最后关闭定时器使用
              timerPtr->subPtr[1]->tickerPtr = timerPtr->subPtr[0]->tickerPtr;
            }
          }
          timerPtr->subPtr.erase(timerPtr->subPtr.begin() + j);
          xSemaphoreGive(xMutexCallBack);
          return true;
        }
      }
    }
    ERR("Error not find %d timer...\n", timeHandle);
    return false;
  }

private:
  // push in
  void push_in_timmer(timeItem_t *para)
  {
    bool exist = false;
    if ((para->time_ms != 0) && (para->callbackPtr != nullptr) &&
        (para->tickerPtr != nullptr))
    {
      for (int32_t i = 0; i < timeListObj.size(); i++)
      {
        if (timeListObj[i]->subPtr[0]->time_ms ==
            para->time_ms)
        { // 已经有现有的定时器，则直接添加进去对应的vector
          xSemaphoreTake(xMutexCallBack, portMAX_DELAY);
          timeListObj[i]->subPtr.push_back(para);
          xSemaphoreGive(xMutexCallBack);
          exist = true;
          INFO("Success insert timer array...\n");
          break;
        }
      }
      if (exist == false)
      { // 没有现成的定时器，则新建
        TimeObj *newTimer = new TimeObj();
        newTimer->subPtr.push_back(para);
        this->timeListObj.push_back(newTimer);
        para->tickerPtr->attach_ms<void *>(para->time_ms, getCallBack,
                                           (void *)newTimer);
        INFO("Create time success....\n");
      }
    }
  }

private:
  static void getCallBack(void *ptr) { ((TimeObj *)ptr)->callback(); }
  vector<TimeObj *> timeListObj;
  int32_t currentTimeHandle;
};
#endif