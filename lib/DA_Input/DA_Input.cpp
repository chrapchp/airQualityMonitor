#include <Streaming.h>
#include "DA_Input.h"
DA_Input::DA_Input(IO_TYPE aInputType, uint8_t aPin)
{
  inputType = aInputType;
  pin       = aPin;
  pinMode(pin, INPUT);
}

DA_Input::DA_Input(IO_TYPE aInputType)
{
  inputType = aInputType;
  pin       = -1; // input without a pin
}

void DA_Input::refresh()
{
  if (pollingEnabled)
  {
    unsigned long currentTime = millis();

    if (currentTime - lastUpdateTime > pollingInterval)
    {
      lastUpdateTime = currentTime;
      onRefresh();
    }
  }
}

void DA_Input::setEnabled( bool aMode )
{
  pollingEnabled = aMode;
  if( aMode == false )
      rawValue = 0;
}

void DA_Input::setPollingInterval(unsigned long aPollingInterval)
{
  if( aPollingInterval > 0 )
    pollingInterval = aPollingInterval;
}

void DA_Input::serialize(Stream *aOutputStream, bool includeCR)
{
  *aOutputStream << "{pin:" << pin << " inputType:" <<
  (inputType ==
   discrete ? 'D' : 'A') << " pollingInterval:" << pollingInterval <<
    "pollingEnabled:" << pollingEnabled << " }";

  if (includeCR) *aOutputStream << endl;
}

void DA_Input::suspendPoll()
{
  pollingEnabled = false;
}

void DA_Input::resumePoll()
{
  pollingEnabled = true;
}
