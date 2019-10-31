#ifndef FSMOTIONCONTROLLER_LOOPTIMER_H
#define FSMOTIONCONTROLLER_LOOPTIMER_H

#include <Arduino.h>

using namespace std;

class LoopTimer
{
public:
   LoopTimer(unsigned long interval) : interval(interval){};
   bool shouldTrigger();

protected:
private:
   const unsigned long interval;     //report every tenth of a second
   unsigned long previousMillis = 0; //holds the count for every loop pass
};

#endif