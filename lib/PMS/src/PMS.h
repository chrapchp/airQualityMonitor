#ifndef PMS_H
#define PMS_H

/* From https://github.com/SwapBap/PMS */

// modified to handle passive and active modes 
// 

#include "Stream.h"

class PMS
{
public:
  static const uint16_t TIMEOUT_TIME = 1000;          //ms
  static const uint16_t ACTIVE_READ_INTERVAL = 5000;  //ms
  static const uint16_t READ_DELAY = 1000;            //ms
  static const uint16_t WAKEUP_DELAY = 1000;          //ms
  static const uint16_t DEEP_SLEEP_DELAY = 1000 * 30; //ms

  typedef enum
  {
    SLEEPING,
    WAKING,
    READING,
    IDLE
  } STATE;

  typedef enum 
  {
    STATUS_WAITING,
    STATUS_OK
  } STATUS;


  typedef enum 
  {
    MODE_ACTIVE,
    MODE_PASSIVE
  } MODE;

  static const uint16_t BAUD_RATE = 9600;

  struct DATA
  {
    // Standard Particles, CF=1
    uint16_t PM_SP_UG_1_0;
    uint16_t PM_SP_UG_2_5;
    uint16_t PM_SP_UG_10_0;

    // Atmospheric environment
    uint16_t PM_AE_UG_1_0;
    uint16_t PM_AE_UG_2_5;
    uint16_t PM_AE_UG_10_0;

    // Total particles
    uint16_t PM_TOTALPARTICLES_0_3;
    uint16_t PM_TOTALPARTICLES_0_5;
    uint16_t PM_TOTALPARTICLES_1_0;
    uint16_t PM_TOTALPARTICLES_2_5;
    uint16_t PM_TOTALPARTICLES_5_0;
    uint16_t PM_TOTALPARTICLES_10_0;
  };

  PMS(Stream&,  void(*callBack)(),  Stream&  );
  PMS(Stream&,  void(*callBack)() );

  void sleep();
  void wakeUp();
  void activeMode();
  void passiveMode();

  void requestRead();
  bool read(DATA &data);
  bool readUntil(DATA &data, uint16_t timeout = TIMEOUT_TIME);

  // added peter c
  void setReadReadyCallback(void (*callBack)());
  void refresh();
  void setSleepDuration( uint32_t duration);
  void setActivePollRate( uint32_t rate);
  PMS::STATE getState();
  // clear the input stream buffer
  void emptyInputStream();

private:
  uint8_t _payload[24];
  Stream *_stream;

  DATA *_data;
  STATUS _status;
  MODE _mode = MODE_ACTIVE;


  uint8_t _index = 0;
  uint16_t _frameLen;
  uint16_t _checksum;
  uint16_t _calculatedChecksum;

  void loop();

  // added peter c
  STATE _state = IDLE;
  Stream *_debugPort;
  uint32_t _activeReadInterval = ACTIVE_READ_INTERVAL;
  uint32_t _passiveSleepDuration = DEEP_SLEEP_DELAY;

  void (*onReadReady)() = NULL;
  uint32_t _previousTime = 0;


};

#endif
