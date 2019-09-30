#include <Arduino.h>
#include <PID_v1.h>
#include <Encoder.h>

#define INPUT_MIN 0
#define INPUT_MAX 255

#define PITCH_ENCODER_MIN 0
#define PITCH_ENCODER_MAX 12600 //105*120 and 360 degree
#define PITCH_ENCODER_180_DEG 6300 // 180 degree

#define kP_Pitch 1
#define kI_Pitch 1
#define kD_Pitch 1

#define ROLL_ENCODER_MIN 0
#define ROLL_ENCODER_MAX 8640 //72*120 and 360 degree
#define ROLL_ENCODER_180_DEG 4320 // 180 degree

#define kP_Roll 1
#define kI_Roll 1
#define kD_Roll 1

#define motorPitchPin 2
#define motorRollPin 3
#define motorCommonPin 4

#define encoderPitchAPin 22
#define encoderPitchBPin 23

#define encoderRollAPin 24
#define encoderRollBPin 25

#define loop_test_times 1//20000  //Run loop 20000 times then calculate time

int loop_counter = 0;         //holds the count for every loop pass

int PitchSetpoint = 0;
int RollSetpoint = 0;

int PitchValue = 0;
int RollValue = 0;

double PID_P_Setpoint, PID_P_Input, PID_P_Output;
double PID_R_Setpoint, PID_R_Input, PID_R_Output;

Encoder EncoderPitch(encoderPitchAPin, encoderPitchBPin);
Encoder EncoderRoll(encoderRollAPin, encoderRollBPin);

PID PID_Pitch(&PID_P_Input, &PID_P_Output, &PID_P_Setpoint, kP_Pitch, kI_Pitch, kD_Pitch, REVERSE);
PID PID_Roll(&PID_R_Input, &PID_R_Output, &PID_R_Setpoint, kP_Roll, kI_Roll, kD_Roll, REVERSE);

void readSerialCommand();
void readEncoderData();
void computePID();
void moveMotors();

void setup() {
  Serial.begin(115200); // start the serial monitor link

  PitchSetpoint = 0;//PITCH_ENCODER_180_DEG;//PITCH_ENCODER_MAX - 10;

  pinMode(motorPitchPin, OUTPUT);
  pinMode(motorRollPin, OUTPUT);

  // Use negative min to allow PID calcuation to reverse motor.
  // But keep range of 0-255
  PID_Pitch.SetOutputLimits(-128,127);
  PID_Roll.SetOutputLimits(-128,127);

  PID_Pitch.SetMode(AUTOMATIC);
  PID_Roll.SetMode(AUTOMATIC);
  
}

void loop() {
  loop_counter++;
  //readSerialCommand();  //where we want to go.
  readEncoderData(); //where we currently are.
  computePID();      //how we are going to get there.
  moveMotors();
  if (loop_counter == loop_test_times) {
    loop_counter = 0;
    //Serial.println("Test");
    // <roll_pwm-1byte><roll_pos_2byte><pitch_pwm-1byte><pitch_pos_2byte>
    // Serial.write((byte)0x00);
    // Serial.write((int)PID_P_Output);
    // Serial.write(PitchValue);
    // Serial.write((int)PID_R_Output);
    // Serial.write(RollValue);
    Serial.println(PitchValue);
  }
}

void readSerialCommand()
{
  if (Serial.available() > 2) {
    Serial.println("Receiving Data:");
    switch ((char)Serial.read())
    {
    case 'P': {
      Serial.println("Pitch Command");
      int pIn = Serial.read() << 8 & Serial.read();
      PitchSetpoint = map(pIn, 0, 0xFFFF, PITCH_ENCODER_MIN, PITCH_ENCODER_MAX);
      break;
    }
    case 'R': {
      int rIn = Serial.read() << 8 & Serial.read();
      RollSetpoint = map(rIn, 0, 0xFFFF, ROLL_ENCODER_MIN, ROLL_ENCODER_MAX);
      Serial.println("Roll Command");
    }
    default:
      break;
    }
  }
  if (Serial.available() > 16) Serial.flush();
}

void readEncoderData()
{
  PitchValue = EncoderPitch.read();
  if (PitchValue < 0) PitchValue += PITCH_ENCODER_MAX;
  PitchValue %= PITCH_ENCODER_MAX;

  RollValue = EncoderRoll.read();
  if (RollValue < 0) RollValue += ROLL_ENCODER_MAX;
  RollValue %= ROLL_ENCODER_MAX;
}

void computePID()
{
  //Update PID Setpoints
  // Normalize the heading to be within 180° of Setpoint (desired heading: 0-359°)
  // Found here: https://forum.arduino.cc/index.php?topic=272298.0
 
  if (PitchValue - PitchSetpoint < -PITCH_ENCODER_180_DEG) {
    PID_P_Setpoint = PitchSetpoint - PITCH_ENCODER_MAX;
  } else if (PitchValue - PitchSetpoint > PITCH_ENCODER_180_DEG) {
    PID_P_Setpoint = PitchSetpoint + PITCH_ENCODER_MAX;
  } else {
    PID_P_Setpoint = PitchSetpoint;
  }
  
  if (RollValue - RollSetpoint < -ROLL_ENCODER_180_DEG) {
    PID_R_Setpoint = RollSetpoint - ROLL_ENCODER_MAX;
  } else if (RollValue - RollSetpoint > ROLL_ENCODER_180_DEG) {
    PID_R_Setpoint = RollSetpoint + ROLL_ENCODER_MAX;
  } else {
    PID_R_Setpoint = RollSetpoint;
  }

  //Update PID Inputs
  PID_P_Input = PitchValue;
  PID_R_Input = RollValue; 
  
  PID_Pitch.Compute();
  PID_Roll.Compute();
}

void moveMotors()
{
  analogWrite(motorPitchPin, PID_P_Output + 128);
  analogWrite(motorRollPin, PID_R_Output + 128);
}
