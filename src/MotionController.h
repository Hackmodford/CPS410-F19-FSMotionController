#ifndef FSMOTIONCONTROLLER_MOTIONCONTROLLER_H
#define FSMOTIONCONTROLLER_MOTIONCONTROLLER_H

using namespace std;

class MotionController
{
public:
   void manualIncreasePitch();
   void manualDecreasePitch();
   void manualIncreaseRoll();
   void manualDecreaseRoll();
   void manualIncreaseLift();
   void manualDecreaseLift();

private:
   void computePID();
};

#endif