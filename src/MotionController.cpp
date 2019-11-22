#include "MotionController.h"
#include "Button.h"
#include "IO_Pins.h"

using namespace std;

MotionController::MotionController(
    DAC *dac,
    Encoder *pitchEncoder,
    Encoder *rollEncoder,
    Button *topSWA,
    Button *topSWB,
    Button *bottomSW,
    Button *stopButton)
    : PID_Pitch(PID(&PID_P_Input, &PID_P_Output, &PID_P_Setpoint, kP_Pitch, kI_Pitch, kD_Pitch, REVERSE)),
      PID_Roll(PID(&PID_R_Input, &PID_R_Output, &PID_R_Setpoint, kP_Roll, kI_Roll, kD_Roll, REVERSE))
{
   m_dac = dac;
   m_encoderPitch = pitchEncoder;
   m_encoderRoll = rollEncoder;
   m_topSWA = topSWA;
   m_topSWB = topSWB;
   m_bottomSW = bottomSW;
   m_stopButton = stopButton;
   // Use negative min to allow PID calcuation to reverse motor.
   // But keep range of 0-255
   PID_Pitch.SetOutputLimits(INT16_MIN, INT16_MAX);
   PID_Roll.SetOutputLimits(INT16_MIN, INT16_MAX);

   PID_Pitch.SetMode(AUTOMATIC);
   PID_Roll.SetMode(AUTOMATIC);

   PitchSetpoint = PITCH_ENCODER_180_DEG;
   RollSetpoint = ROLL_ENCODER_180_DEG;

   pitchEncoder->write(PitchSetpoint);
   rollEncoder->write(RollSetpoint);

   simState = stopped;
}

void MotionController::begin()
{
   m_dac->begin();
}

void MotionController::update()
{
   checkSetSwitches();
   m_stopButton->read();
   readEncoderData();
   switch (simState)
   {
   case stopped:
      PID_P_Output = 0;
      PID_R_Output = 0;
      m_dac->clear();
      digitalWrite(DO_DOWN_CO, LOW);
      digitalWrite(DO_UP_CO, LOW);
      break;
   case starting:
      if (m_stopButton->pressed)
      {
         // occupant pressed stop button or seatbelt came undone.
         //simState = ending;
         Serial.println("Stopped in starting state");
         emergencyStop();
         break;
      }
      // Lift for 1 second.
      // Disconnect pitch motor allowing to move.
      // Balance
      // Raise
      PID_P_Output = 0;
      PID_R_Output = 0;
      m_dac->setChannel(A, 0);
      m_dac->setChannel(B, 0);
      if (moveUp())
      {
         simState = running;
      }
      break;
   case running:
      if (!m_topSWA->pressed || !m_topSWB->pressed)
      {
         Serial.println("Top Switches not pressed. Ending running state.");
         emergencyStop();
         break;
      }
      if (m_stopButton->pressed)
      {
         // Panic Button -> End State
         // Just home it don't lower
         //simState = ending;
         Serial.println("Pressed Stop button in running state.");
         emergencyStop();
         break;
      }
      computePID();
      m_dac->setChannel(A, (int)PID_P_Output);
      m_dac->setChannel(B, (int)PID_R_Output);
      m_dac->setChannel(C, 0);
      m_dac->setChannel(D, 0);
      break;
   case ending:
      // Go to neutral position
      // Lower
      PID_P_Output = 0;
      PID_R_Output = 0;
      m_dac->setChannel(A, 0);
      m_dac->setChannel(B, 0);
      if (moveDown())
      {
         simState = stopped;
      }
      break;
   case emergency_stop:
      // No power to motors
      // Ability to exit emergency stop.
      m_dac->clear();
      break;
   case manual:
      break;
   }
}

void MotionController::checkSetSwitches()
{
   m_topSWA->read();
   m_topSWB->read();
   m_bottomSW->read();
}

/**
 * @brief Retrieves data from encoder
 *
 * Updates the PitchValue and RollValue with data from the encoders.
 */
void MotionController::readEncoderData()
{
   PitchValue = m_encoderPitch->read();

   if (PitchValue < 0)
      PitchValue += PITCH_ENCODER_MAX;
   PitchValue %= PITCH_ENCODER_MAX;

   RollValue = m_encoderRoll->read();
   if (RollValue < 0)
      RollValue += ROLL_ENCODER_MAX;
   RollValue %= ROLL_ENCODER_MAX;
}

/**
 * @brief Computes PID outputs for pitch and roll.
 * 
 * Adjust setpoints for 360 rotation and calls Compute() on PID objects.
 */
void MotionController::computePID()
{
   //Update PID Setpoints
   // Normalize the heading to be within 180° of Setpoint (desired heading: 0-359°)
   // Found here: https://forum.arduino.cc/index.php?topic=272298.0

   if (PitchValue - PitchSetpoint < -PITCH_ENCODER_180_DEG)
   {
      PID_P_Setpoint = PitchSetpoint - PITCH_ENCODER_MAX;
   }
   else if (PitchValue - PitchSetpoint > PITCH_ENCODER_180_DEG)
   {
      PID_P_Setpoint = PitchSetpoint + PITCH_ENCODER_MAX;
   }
   else
   {
      PID_P_Setpoint = PitchSetpoint;
   }

   if (RollValue - RollSetpoint < -ROLL_ENCODER_180_DEG)
   {
      PID_R_Setpoint = RollSetpoint - ROLL_ENCODER_MAX;
   }
   else if (RollValue - RollSetpoint > ROLL_ENCODER_180_DEG)
   {
      PID_R_Setpoint = RollSetpoint + ROLL_ENCODER_MAX;
   }
   else
   {
      PID_R_Setpoint = RollSetpoint;
   }

   //Update PID Inputs
   PID_P_Input = PitchValue;
   PID_R_Input = RollValue;

   PID_Pitch.Compute();
   PID_Roll.Compute();
}

bool MotionController::moveUp()
{
   if (m_topSWB->pressed)
      Serial.println("B is pressed");
   if (m_topSWA->pressed)
      Serial.println("A is pressed");
   if (m_topSWA->pressed && m_topSWB->pressed)
   {
      delay(250);
      digitalWrite(DO_UP_CO, LOW);
      return true;
   }
   digitalWrite(DO_UP_CO, HIGH);
   return false;
}

bool MotionController::moveDown()
{
   if (m_bottomSW->pressed)
      Serial.println("Bottom switch is pressed");
   if (m_bottomSW->pressed)
   {
      digitalWrite(DO_DOWN_CO, LOW);
      return true;
   }
   digitalWrite(DO_DOWN_CO, HIGH);
   return false;
}

void MotionController::manualIncreasePitch(bool end)
{
   if (end)
   {
      PID_P_Output = 0;
      m_dac->setChannel(A, PID_P_Output);
   }
   else
   {
      simState = manual;
      PID_P_Output = MAX_MANUAL_SPEED; //update for reporting.
      m_dac->setChannel(A, PID_P_Output);
   }
}

void MotionController::manualDecreasePitch(bool end)
{
   if (end)
   {
      PID_P_Output = 0;
      m_dac->setChannel(A, PID_P_Output);
   }
   else
   {
      simState = manual;
      PID_P_Output = -MAX_MANUAL_SPEED; //update for reporting.
      m_dac->setChannel(A, PID_P_Output);
   }
}

void MotionController::manualIncreaseRoll(bool end)
{
   if (end)
   {
      PID_R_Output = 0;
      m_dac->setChannel(B, PID_P_Output);
   }
   else
   {
      simState = manual;
      PID_R_Output = MAX_MANUAL_SPEED; //update for reporting.
      m_dac->setChannel(B, PID_R_Output);
   }
}

void MotionController::manualDecreaseRoll(bool end)
{
   if (end)
   {
      PID_R_Output = 0;
      m_dac->setChannel(B, PID_P_Output);
   }
   else
   {
      simState = manual;
      PID_R_Output = -MAX_MANUAL_SPEED; //update for reporting.
      m_dac->setChannel(B, PID_R_Output);
   }
}

void MotionController::manualIncreaseLift(bool end)
{
   if (m_topSWA->pressed && m_topSWB->pressed)
   {
      //TODO: turn off up pin
      return;
   }
   if (end)
   {
      //TODO: turn off up pin.
   }
   else
   {
      //TODO: turn on up pin
   }
}

void MotionController::manualDecreaseLift(bool end)
{
   if (m_bottomSW->pressed)
   {
      //TODO: turn off down pin
   }
   if (end)
   {
      //TODO: turn off down pin
   }
   else
   {
      //TODO: turn on down pin
   }
}

void MotionController::startSimulation()
{
   if (simState == emergency_stop)
   {
      Serial.println("Cannot start simulation due to emergency stop.");
      return;
   }
   if (simState != stopped)
   {
      Serial.println("Cannot start simulation as the simulator is not stopped.");
      return;
   }
   simState = starting;
}

void MotionController::endSimulation()
{
   if (simState == emergency_stop)
   {
      Serial.println("Cannot start simulation due to emergency stop.");
      return;
   }
   if (simState != running)
   {
      Serial.println("Cannot start simulation as the simulator is not running.");
      return;
   }
   simState = ending;
}

void MotionController::emergencyStop()
{
   //TODO: turn off up an down pins
   m_dac->clear();

   digitalWrite(DO_INC_CW, LOW);
   digitalWrite(DO_DEC_CW, LOW);
   digitalWrite(DO_UP_CO, LOW);
   digitalWrite(DO_P_DIS, LOW);
   digitalWrite(DO_PRESSURE, LOW);
   digitalWrite(DO_DOWN_CO, LOW);

   simState = emergency_stop;
}

int MotionController::constrainedPitchSetpoint(unsigned int value)
{
   unsigned int x = uMap(value, 0, UINT32_MAX, PITCH_ENCODER_MIN, PITCH_ENCODER_MAX);
   return x;
}

int MotionController::constrainedRollSetpoint(unsigned int value)
{
   return uMap(value, 0, UINT32_MAX, ROLL_ENCODER_MIN, ROLL_ENCODER_MAX);
}

unsigned long long MotionController::uMap(unsigned long long x, unsigned long long in_min, unsigned long long in_max, unsigned long long out_min, unsigned long long out_max)
{

   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
