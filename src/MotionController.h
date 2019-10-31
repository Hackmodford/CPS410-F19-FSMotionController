#ifndef FSMOTIONCONTROLLER_MOTIONCONTROLLER_H
#define FSMOTIONCONTROLLER_MOTIONCONTROLLER_H

#include <Encoder.h>
#include <PID_v1.h>

#include "Dac.h"

using namespace std;

#define PITCH_ENCODER_MIN 0
#define PITCH_ENCODER_MAX 12600    //105*120 and 360 degree
#define PITCH_ENCODER_180_DEG 6300 // 180 degree

#define ROLL_ENCODER_MIN 0
#define ROLL_ENCODER_MAX 8640     //72*120 and 360 degree
#define ROLL_ENCODER_180_DEG 4320 // 180 degree

#define MAX_MANUAL_SPEED 3400

enum state
{
   stopped = 0,
   starting = 1,
   running = 2,
   ending = 3,
   emergency_stop = 99,
   manual = 4
};

class MotionController
{
public:
   state simState;

   int PitchSetpoint = 0;
   int PitchValue = 0;
   double kP_Pitch = 1;
   double kI_Pitch = 1;
   double kD_Pitch = 1;
   double PID_P_Setpoint, PID_P_Input, PID_P_Output;

   int RollSetpoint = 0;
   int RollValue = 0;
   double kP_Roll = 1;
   double kI_Roll = 1;
   double kD_Roll = 1;
   double PID_R_Setpoint, PID_R_Input, PID_R_Output;

   MotionController(DAC *dac, Encoder *pitchEncoder, Encoder *rollEncoder);

   void begin();

   void update();

   void manualIncreasePitch(bool end);
   void manualDecreasePitch(bool end);
   void manualIncreaseRoll(bool end);
   void manualDecreaseRoll(bool end);
   void manualIncreaseLift(bool end);
   void manualDecreaseLift(bool end);

   int constrainedPitchSetpoint(unsigned int value);
   int constrainedRollSetpoint(unsigned int value);

   void startSimulation();
   void endSimulation();
   void emergencyStop();

private:
   DAC *m_dac;
   Encoder *m_encoderPitch;
   Encoder *m_encoderRoll;
   PID PID_Pitch;
   PID PID_Roll;

   void computePID();
   void readEncoderData();
   bool moveUp();
   bool moveDown();
};

#endif