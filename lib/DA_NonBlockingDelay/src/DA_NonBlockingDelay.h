/**
 *  @file    DA_NonBlockingDelay.h
 *  @author  peter c
 *  @date    4/14/2017
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Simple class to deal with non-blocking delay
 */

#ifndef DA_NONBLOCKINIGDELAY_H
#define DA_NONBLOCKINIGDELAY_H


#include <Stream.h>


class DA_NonBlockingDelay {
public:

  DA_NonBlockingDelay(uint32_t aDelayinMillSeconds,
                      void(*callBack)());
  void setDelay(uint32_t aDelayinMillSeconds);
  void setCallback(void (*callBack)());
  void serialize(Stream *aOutputStream,
                 bool            includeCR);
  void refresh();
  void pause();
  void resume();
  void reset();

private:

  uint32_t desiredDelay = 0;
  uint32_t pausedTime = 0;
  void (*onDelayComplete)() = NULL;
  uint32_t previousTime = 0;
  bool paused                = false;
};

#endif // ifndef DA_NONBLOCKINIGDELAY_H
