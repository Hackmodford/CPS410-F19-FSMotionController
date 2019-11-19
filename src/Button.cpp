#include "Arduino.h"
#include "Button.h"

using namespace std;

Button::Button(int PIN, void (*pCallbackFunction)(Button *), uint32_t ulMode)
{
  pin = PIN;
  pCallback = pCallbackFunction;
  pinMode(pin, ulMode);
  state = digitalRead(pin);
  pressed = state == LOW;
  lastValue = state;
  lastDebounceTime = 0;
}

void Button::read()
{
  int value = digitalRead(pin);
  if (value != lastValue)
  {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > 10)
  {
    if (value != state)
    {
      state = value;
      pressed = state == LOW;
      if (pCallback != NULL) {
        pCallback(this);
      }
    }
  }
  lastValue = value;
}
