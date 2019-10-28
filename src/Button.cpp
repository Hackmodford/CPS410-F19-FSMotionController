#include "Arduino.h"
#include "Button.h"

Button::Button(int PIN, void (*pCallbackFunction)(Button*))
{
  pin = PIN;
  pCallback = pCallbackFunction;
  pinMode(pin, INPUT_PULLUP);
  state = digitalRead(pin);
  pressed = state == LOW;
  lastValue = state;
  lastDebounceTime = 0;
}

void Button::read()
{
  int value = digitalRead(pin);
  if (value != lastValue){
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > 10){
    if (value != state){
      state = value;
	  pressed = state == LOW;
      pCallback(this);
    }
  }
  lastValue = value;
}
