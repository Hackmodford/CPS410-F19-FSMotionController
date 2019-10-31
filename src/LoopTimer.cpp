#include "LoopTimer.h"

using namespace std;

bool LoopTimer::shouldTrigger()
{
   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis > interval)
   {
      previousMillis = currentMillis;
      return true;
   }
   return false;
}