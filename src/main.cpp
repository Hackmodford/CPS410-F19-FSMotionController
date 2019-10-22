//#pragma once
#include <Arduino.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SD.h>

#include "config.h"
#include "DAC.h"

using namespace std;

#define SPI_SD_PIN 4
#define SPI_ETHERNET_PIN 10
#define SPI_DAC_PIN 52
#define encoderPitchAPin 22
#define encoderPitchBPin 23
#define encoderRollAPin 24
#define encoderRollBPin 25
#define STOP_SWITCH_TOP_A_PIN 26
#define STOP_SWITCH_TOP_B_PIN 27
#define STOP_SWITCH_BOTTOM_PIN 28

#define PITCH_ENCODER_MIN 0
#define PITCH_ENCODER_MAX 12600    //105*120 and 360 degree
#define PITCH_ENCODER_180_DEG 6300 // 180 degree

#define ROLL_ENCODER_MIN 0
#define ROLL_ENCODER_MAX 8640     //72*120 and 360 degree
#define ROLL_ENCODER_180_DEG 4320 // 180 degree

#define MAX_MANUAL_SPEED 328

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

DAC DAC_Sim(SPI_DAC_PIN, 32, 33);

byte mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // device mac address

IPAddress ip;       // address to listen
uint16_t localPort; // port to listen

IPAddress ipOut;  // address to broadcast
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
   emergency_stop = 99,
   manual = 4
};
state simState;

void loadConfiguration();
void setupUDP();
void readUDP();
//void readSerialCommand();
void readEncoderData();
void computePID();
void report();

void startSimulation();
void endSimulation();
void emergencyStop();

bool moveUp();   //returns true when simulator is in top position.
bool moveDown(); //returns true when simulator is in bottom position.

void manualIncreasePitch();
void manualDecreasePitch();
void manualIncreaseRoll();
void manualDecreaseRoll();
void manualIncreaseLift();
void manualDecreaseLift();

void setup()
{
   //test LEDS
   pinMode(7, OUTPUT);
   pinMode(6, OUTPUT);

   pinMode(STOP_SWITCH_TOP_A_PIN, INPUT);
   pinMode(STOP_SWITCH_TOP_B_PIN, INPUT);
   pinMode(STOP_SWITCH_BOTTOM_PIN, INPUT);

   Serial.begin(115200);

   PitchSetpoint = 0;
   RollSetpoint = 0;
   simState = stopped;

   // Use negative min to allow PID calcuation to reverse motor.
   // But keep range of 0-255
   PID_Pitch.SetOutputLimits(INT16_MIN, INT16_MAX);
   PID_Roll.SetOutputLimits(INT16_MIN, INT16_MAX);

   PID_Pitch.SetMode(AUTOMATIC);
   PID_Roll.SetMode(AUTOMATIC);

   //initializing the SD card
   Serial.print("Initializing SD card...");
   if (!SD.begin(SPI_SD_PIN))
   {
      Serial.println("Card failed, or not present");
      //use default settings
      ip = IPAddress(192, 168, 1, 6);
      localPort = 8888;
      ipOut = IPAddress(192, 168, 1, 5);
      outPort = 8888;
      kP_Pitch = 1;
      kI_Pitch = 1;
      kD_Pitch = 1;
      kP_Roll = 1;
      kI_Roll = 1;
      kD_Roll = 1;
   }
   else
   {
      Serial.println("card initialized.");
      loadConfiguration();
   }
   SD.end();
   setupUDP();
   DAC_Sim.begin();
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
      while (true)
         ;
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
      DAC_Sim.clear();
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
      DAC_Sim.setValues((int)PID_P_Output, (int)PID_R_Output, 0, 0);
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
      DAC_Sim.clear();
      break;
   case manual:
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
   {
      //emergency stop
      emergencyStop();
      Serial.println("Received Emergency Stop Command");
      break;
   }
   case 'S':
   {
      //start
      startSimulation();
      Serial.println("Received Start Command");
      break;
   }
   case 'E':
   {
      //stop
      endSimulation();
      Serial.println("Received End Command");
      break;
   }
   case 'M':
   {
      //update set points
      if (packetSize != 5)
      {
         Serial.println("Malformed move command");
         break;
      }
      if (simState != running)
      {
         Serial.println("Ignoring move command. Simulator is not running.");
         break;
      }
      memcpy(&PitchSetpoint, &packetBuffer[1], sizeof(int));
      memcpy(&RollSetpoint, &packetBuffer[3], sizeof(int));
      break;
   }
   case 'm':
   {
      if (packetSize == 2) {
			switch (packetBuffer[1])
			{
				case 'P':
				{
					manualIncreasePitch();
					break;
				}
				case 'p':
				{
					manualDecreasePitch();
					break;
				}
				case 'R':
				{
					manualIncreaseRoll();
					break;
				}
				case 'r':
				{
					manualDecreaseRoll();
					break;
				}
				case 'L':
				{
					manualIncreaseLift();
					break;
				}
				case 'l':
				{
					manualDecreaseLift();
					break;
				}
				case 's':
				{
					DAC_Sim.clear();
					break;
				}
			}
      }
      break;
   }
   case '1':
   {
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
   }
   case '2':
   {
      //update roll pid values
      if (packetSize != 24)
      {
         Serial.println("Malformed update roll PID command");
         break;
      }
      memcpy(&kP_Roll, &packetBuffer[1], sizeof(double));
      memcpy(&kI_Roll, &packetBuffer[9], sizeof(double));
      memcpy(&kD_Roll, &packetBuffer[17], sizeof(double));
      break;
   }
   case 'G':
   { //update coarse gain
      //[command][channel][cg]
      if (packetSize != 3)
      {
         Serial.println("Malformed set coarse gain command");
         break;
      }
      channelOption channel = byteToChannelOption(packetBuffer[1]);
      coarseGainOption option = byteToCoarseGainOption(packetBuffer[2]);
      DAC_Sim.setCoarseGain(channel, option);
      break;
   }
   case 'g':
   { //update fine gain
      //[command][channel][fg]
      if (packetSize != 3)
      {
         Serial.println("Malformed set fine gain command");
         break;
      }
      channelOption channel = byteToChannelOption(packetBuffer[1]);
      DAC_Sim.setFineGain(channel, packetBuffer[2]); //Byte with range (-32, 31)
      break;
   }
   case 'O':
   { //update offset
      //[command][channel][offset]
      if (packetSize != 3)
      {
         Serial.println("Malformed set offset command");
         break;
      }
      channelOption channel = byteToChannelOption(packetBuffer[1]);
      DAC_Sim.setOffset(channel, packetBuffer[2]);
      break;
   }
   default:
   {
      Serial.println("Received unknown command");
      break;
   }
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

   int pitchPWM = (int)PID_P_Output;
   int rollPWM = (int)PID_R_Output;
   byte bufferSize = 62;
   char *ReplyBuffer = new char[bufferSize];

   memcpy(&ReplyBuffer[0], &PitchSetpoint, sizeof(int));
   memcpy(&ReplyBuffer[2], &PitchValue, sizeof(int));
   memcpy(&ReplyBuffer[4], &RollSetpoint, sizeof(int));
   memcpy(&ReplyBuffer[6], &RollValue, sizeof(int));
   memcpy(&ReplyBuffer[8], &pitchPWM, sizeof(int));
   memcpy(&ReplyBuffer[10], &rollPWM, sizeof(int));
   memcpy(&ReplyBuffer[12], &simState, sizeof(byte));
   memcpy(&ReplyBuffer[13], &kP_Pitch, sizeof(double));
   memcpy(&ReplyBuffer[21], &kI_Pitch, sizeof(double));
   memcpy(&ReplyBuffer[29], &kD_Pitch, sizeof(double));
   memcpy(&ReplyBuffer[37], &kP_Roll, sizeof(double));
   memcpy(&ReplyBuffer[45], &kI_Roll, sizeof(double));
   memcpy(&ReplyBuffer[53], &kD_Roll, sizeof(double));

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
}

void emergencyStop()
{
	DAC_Sim.clear();
   simState = emergency_stop;
}

int upTemp = 0;
bool moveUp()
{
   // int a = digitalRead(STOP_SWITCH_TOP_A_PIN);
   // int b = digitalRead(STOP_SWITCH_TOP_B_PIN);
   // if (a == HIGH || b == HIGH)
   // {
   //    //move up
   // } else {
   //    return true;
   // }
   // return false;
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
   // PitchSetpoint = 0;
   // PitchSetpoint = 0;
   // if (PitchValue != PitchSetpoint)
   // {
   //    //rotate to start position
   //    computePID();
   //    //PID_X_Output could be constrained so that it doesn't move too fast.
   //    DAC_Sim.setValues((int)PID_P_Output, (int)PID_R_Output, 0, 0);
   //    return false;
   // }

   // if (digitalRead(STOP_SWITCH_BOTTOM_PIN) == HIGH)
   // {
   //    // move down
   // }
   // return true;

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

void manualIncreasePitch()
{
	simState = manual;
	PID_P_Output = MAX_MANUAL_SPEED; //update for reporting.
	DAC_Sim.setValues(PID_P_Output, 0, 0, 0);
}

void manualDecreasePitch()
{
	simState = manual;
	PID_P_Output = -MAX_MANUAL_SPEED; //update for reporting.
	DAC_Sim.setValues(PID_P_Output, 0, 0, 0);
}

void manualIncreaseRoll()
{
	simState = manual;
	PID_R_Output = MAX_MANUAL_SPEED; //update for reporting.
	DAC_Sim.setValues(0, PID_R_Output, 0, 0);
}

void manualDecreaseRoll()
{
	simState = manual;
	PID_R_Output = -MAX_MANUAL_SPEED; //update for reporting.
	DAC_Sim.setValues(0, PID_R_Output, 0, 0);
}

void manualIncreaseLift()
{
	simState = manual;
	DAC_Sim.setValues(0, 0, MAX_MANUAL_SPEED, 0);
}

void manualDecreaseLift()
{
	simState = manual;
	DAC_Sim.setValues(0, 0, -MAX_MANUAL_SPEED, 0);
}
