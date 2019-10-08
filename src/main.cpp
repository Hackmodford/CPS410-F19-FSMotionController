//#pragma once
#include <Arduino.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SD.h>

#include "config.h"
using namespace std;

#define CHIP_SELECT_PIN 4

#define INPUT_MIN 0
#define INPUT_MAX 255

#define PITCH_ENCODER_MIN 0
#define PITCH_ENCODER_MAX 12600    //105*120 and 360 degree
#define PITCH_ENCODER_180_DEG 6300 // 180 degree

#define ROLL_ENCODER_MIN 0
#define ROLL_ENCODER_MAX 8640     //72*120 and 360 degree
#define ROLL_ENCODER_180_DEG 4320 // 180 degree

#define motorPitchPin 2
#define motorRollPin 3
#define motorCommonPin 5 //pin 4 is used by SD card

#define encoderPitchAPin 22
#define encoderPitchBPin 23

#define encoderRollAPin 24
#define encoderRollBPin 25

#define REPORT_INTERVAL 100 //report every tenth of a second
long previousMillis = 0;    //holds the count for every loop pass

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

Encoder EncoderPitch(encoderPitchAPin, encoderPitchBPin);
Encoder EncoderRoll(encoderRollAPin, encoderRollBPin);

PID PID_Pitch(&PID_P_Input, &PID_P_Output, &PID_P_Setpoint, kP_Pitch, kI_Pitch, kD_Pitch, REVERSE);
PID PID_Roll(&PID_R_Input, &PID_R_Output, &PID_R_Setpoint, kP_Roll, kI_Roll, kD_Roll, REVERSE);

byte mac[6]; // device mac address

IPAddress ip; // address to listen
uint16_t localPort; // port to listen

IPAddress ipOut; // address to broadcast
uint16_t outPort; // port to send

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; // buffer to hold incoming packet

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

enum state
{
   stopped = 0,
   starting = 1,
   running = 2,
   ending = 3,
   emergency_stop = 99
};
state simState;

void loadConfiguration();
void setupUDP();
void readUDP();
//void readSerialCommand();
void readEncoderData();
void computePID();
void moveMotors();
void report();

void startSimulation();
void endSimulation();
void emergencyStop();

bool moveUp();   //returns true when simulator is in top position.
bool moveDown(); //returns true when simulator is in bottom position.

void setup()
{
   //test LEDS
   pinMode(7, OUTPUT);
   pinMode(6, OUTPUT);

   Serial.begin(115200);

   PitchSetpoint = 0;
   RollSetpoint = 0;
   simState = stopped;

   pinMode(motorPitchPin, OUTPUT);
   pinMode(motorRollPin, OUTPUT);

   // Use negative min to allow PID calcuation to reverse motor.
   // But keep range of 0-255
   PID_Pitch.SetOutputLimits(-255, 255);
   PID_Roll.SetOutputLimits(-255, 255);

   PID_Pitch.SetMode(AUTOMATIC);
   PID_Roll.SetMode(AUTOMATIC);

   //initializing the SD card
   Serial.print("Initializing SD card...");
   if (!SD.begin(CHIP_SELECT_PIN))
   {
      Serial.println("Card failed, or not present");
      while(true); //don't run...
   }
   else
   {
      Serial.println("card initialized.");
   }

   loadConfiguration();
   setupUDP();
}

void loadConfiguration()
{
   Config config;
   config.getMACAddress(mac);
   config.getIPAddress(ip);
   config.getPort(localPort);
   config.getOutgoingIPAddress(ipOut);
   config.getOutgoingPort(outPort);
   config.getKP_Pitch(kP_Pitch);
   config.getKI_Pitch(kI_Pitch);
   config.getKD_Pitch(kD_Pitch);
   config.getKP_Roll(kP_Roll);
   config.getKI_Roll(kI_Roll);
   config.getKD_Roll(kD_Roll);
}

void setupUDP()
{
   // start the Ethernet
   Ethernet.begin(mac, ip);

   // Check for Ethernet hardware present
   if (Ethernet.hardwareStatus() == EthernetNoHardware)
   {
      Serial.println("Ethernet shield was not found. Sorry, can't run without hardware. :(");
      while(true);
   }
   Udp.begin(localPort);
}

void loop()
{
   readUDP();         //where we want to go.
   readEncoderData(); //where we currently are.

   switch (simState)
   {
   case stopped:
      PID_P_Output = 0;
      PID_R_Output = 0;
      moveMotors();
      break;
   case starting:
      // Balance
      // Raise
      if (moveUp())
      {
         simState = running;
      }
      break;
   case running:
      // Two Switches -> Emergency Stop
      // Panic Button -> End State
      computePID();
      moveMotors();
      break;
   case ending:
      // Go to neutral position
      // Lower
      if (moveDown())
      {
         simState = stopped;
      }
      break;
   case emergency_stop:
      // No power to motors
      // Ability to exit emergency stop.
      break;
   }

   digitalWrite(7, simState != running); // red LED
   digitalWrite(6, simState == running); // green LED

   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis > REPORT_INTERVAL)
   {
      previousMillis = currentMillis;
      report();
   }
}

/**
 * @brief Listens for UDP commands.
 * 
 * Listens for UDP commands on the ports specified at top of file.
 * If a command is recognized, it triggers the action.
 * 
 * Note: This function is not responsible for determining if the action is valid.
 */
void readUDP()
{

   int packetSize = Udp.parsePacket();
   if (packetSize == 0)
      return;

   // read the packet into packetBufffer
   Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);

   switch (packetBuffer[0])
   {
   case '0':
      //emergency stop
      emergencyStop();
      Serial.println("Received Emergency Stop Command");
      break;
   case 'S':
      //start
      startSimulation();
      Serial.println("Received Start Command");
      break;
   case 'E':
      //stop
      endSimulation();
      Serial.println("Received End Command");
      break;
   case 'M':
      //update set points
      if (packetSize != 5)
      {
         Serial.println("Malformed move command");
         break;
      }
      memcpy(&PitchSetpoint, &packetBuffer[1], sizeof(int));
      memcpy(&RollSetpoint, &packetBuffer[3], sizeof(int));
      break;
   case '1':
      // update pitch pid values
      if (packetSize != 24)
      {
         Serial.println("Malformed update pitch PID command");
         break;
      }
      memcpy(&kP_Pitch, &packetBuffer[1], sizeof(double));
      memcpy(&kI_Pitch, &packetBuffer[9], sizeof(double));
      memcpy(&kD_Pitch, &packetBuffer[17], sizeof(double));
      break;
   case '2':
      //update roll pid values
      if (packetSize != 24)
      {
         Serial.println("Malformed update roll PID command");
         break;
      }
      memcpy(&kP_Roll, &packetBuffer[1], sizeof(double));
      memcpy(&kI_Roll, &packetBuffer[9], sizeof(double));
      memcpy(&kD_Roll, &packetBuffer[17], sizeof(double));
   default:
      break;
   }
}

/**
 * @brief Retrieves data from encoder
 * 
 * Updates the PitchValue and RollValue with data from the encoders.
 */
void readEncoderData()
{
   PitchValue = EncoderPitch.read();
   if (PitchValue < 0)
      PitchValue += PITCH_ENCODER_MAX;
   PitchValue %= PITCH_ENCODER_MAX;

   RollValue = EncoderRoll.read();
   if (RollValue < 0)
      RollValue += ROLL_ENCODER_MAX;
   RollValue %= ROLL_ENCODER_MAX;
}

/**
 * @brief Computes PID outputs for pitch and roll.
 * 
 * Adjust setpoints for 360 rotation and calls Compute() on PID objects.
 */
void computePID()
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

void moveMotors()
{
   analogWrite(motorPitchPin, abs(PID_P_Output));
   analogWrite(motorRollPin, abs(PID_R_Output));
}

//byte legend
//0-1 Pitch Setpoint
//2-3 Pitch Value
//4-5 Roll Setpoint
//6-7 Roll Value
//8 Pitch PWM
//9 Roll PWM
//10 State
//11 kP Pitch
//19 kI Pitch
//27 kD Pitch
//35 kP Roll
//43 kI Roll
//51 kD Roll
void report()
{

   byte pitchPWM = (byte)(abs(PID_P_Output));
   byte rollPWM = (byte)(abs(PID_R_Output));
   byte bufferSize = 59;
   char *ReplyBuffer = new char[bufferSize];

   memcpy(&ReplyBuffer[0], &PitchSetpoint, sizeof(int));
   memcpy(&ReplyBuffer[2], &PitchValue, sizeof(int));
   memcpy(&ReplyBuffer[4], &RollSetpoint, sizeof(int));
   memcpy(&ReplyBuffer[6], &RollValue, sizeof(int));
   memcpy(&ReplyBuffer[8], &pitchPWM, sizeof(byte));
   memcpy(&ReplyBuffer[9], &rollPWM, sizeof(byte));
   memcpy(&ReplyBuffer[10], &simState, sizeof(byte));
   memcpy(&ReplyBuffer[11], &kP_Pitch, sizeof(double));
   memcpy(&ReplyBuffer[19], &kI_Pitch, sizeof(double));
   memcpy(&ReplyBuffer[27], &kD_Pitch, sizeof(double));
   memcpy(&ReplyBuffer[35], &kP_Roll, sizeof(double));
   memcpy(&ReplyBuffer[43], &kI_Roll, sizeof(double));
   memcpy(&ReplyBuffer[51], &kD_Roll, sizeof(double));

   Udp.beginPacket(ipOut, outPort);
   Udp.write(ReplyBuffer, bufferSize);
   Udp.endPacket();

   delete ReplyBuffer;
}

void startSimulation()
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
   report();
   delay(2000);
}

void endSimulation()
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
   report();
   delay(2000);
}

void emergencyStop()
{
   simState = emergency_stop;
   analogWrite(motorPitchPin, 0);
   analogWrite(motorRollPin, 0);
}

int upTemp = 0;
bool moveUp()
{
   //This function is incomplete.
   //just some code to simulate moving up.
   if (upTemp < 5000)
   {
      upTemp++;
      return false;
   }
   upTemp = 0;
   return true;
}

int downTemp = 0;
bool moveDown()
{
   //This function is incomplete.
   //just some code to simulate moving down.
   if (downTemp < 5000)
   {
      downTemp++;
      return false;
   }
   downTemp = 0;
   return true;
}
