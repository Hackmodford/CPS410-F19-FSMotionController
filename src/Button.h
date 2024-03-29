#ifndef FSMOTIONCONTROLLER_BUTTON_H
#define FSMOTIONCONTROLLER_BUTTON_H

#include "Arduino.h"

using namespace std;

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
  Button(int PIN, void (*pCallbackFunction)(Button *), uint32_t ulMode = INPUT_PULLUP);
};

#endif //FSMOTIONCONTROLLER_BUTTON_H