/**
 *  @file    DA_NonBlockingDelay.h
 *  @author  peter c
 *  @date    4/14/2017
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Simple class to deal with non-blocking delay
 *  11/15/21 - Changed abs to labs new compile error on platformio
 */
#include <Streaming.h>
#include "DA_NonBlockingDelay.h"
DA_NonBlockingDelay::DA_NonBlockingDelay(uint32_t aDelayinMillSeconds,
                                         void (*callBack)())
{
  onDelayComplete = callBack;
  desiredDelay = aDelayinMillSeconds;
  previousTime = millis();
}

void DA_NonBlockingDelay::setDelay(uint32_t aDelayinMillSeconds)
{
  desiredDelay = aDelayinMillSeconds;
}

void DA_NonBlockingDelay::setCallback(void (*callBack)())
{
  onDelayComplete = callBack;
}

void DA_NonBlockingDelay::serialize(Stream *aOutputStream, bool includeCR)
{
  *aOutputStream << "{delay:" << desiredDelay << " previousTime:" << previousTime;
  *aOutputStream << endl;
}

void DA_NonBlockingDelay::refresh()
{
  uint32_t currentTime;

  if (!paused)
  {
    currentTime = millis();

    if ((labs(currentTime - previousTime)) >= desiredDelay)
    {
      previousTime = currentTime;

      if ((onDelayComplete != NULL))
      {
        onDelayComplete();
      }
    }
  }
}

void DA_NonBlockingDelay::pause()
{
  paused = true;
  pausedTime = millis() - previousTime;
}

void DA_NonBlockingDelay::resume()
{
  paused = false;
  previousTime = millis() + pausedTime;
  pausedTime = 0;
}

void DA_NonBlockingDelay::reset()
{
  previousTime = millis();
}
