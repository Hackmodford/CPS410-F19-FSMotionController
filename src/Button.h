#include "Arduino.h"
#ifndef FSMOTIONCONTROLLER_BUTTON_H
#define FSMOTIONCONTROLLER_BUTTON_H

// Using code found here: http://blog.cornbeast.com/2014/02/arduino-button-class/
class Button
{
private:
  int lastValue;
  unsigned long lastDebounceTime;
  void (*pCallback)(Button *);

public:
  String label;
  int pin;
  int state;
  bool pressed;
  void read();
  Button(int PIN, void (*pCallbackFunction)(Button *));
};

#endif //FSMOTIONCONTROLLER_BUTTON_H