//#pragma once
#include <Arduino.h>
#include <Encoder.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SD.h>

#include "Config.h"
#include "DAC.h"
#include "Button.h"
#include "MotionController.h"
#include "LoopTimer.h"
#include "IO_Pins.h"

using namespace std;

#define SPI_SD_PIN 4
#define SPI_ETHERNET_PIN 10
#define SPI_DAC_PIN 52

#define encoderRollBPin 22
#define encoderRollAPin 23
#define encoderPitchAPin 24
#define encoderPitchBPin 25

#define BTN_ESTOP_PIN 12
#define BTN_P_INCREASE_PIN 11
#define BTN_P_DECREASE_PIN 9
#define BTN_R_INCREASE_PIN 8
#define BTN_R_DECREASE_PIN 7
#define BTN_L_INCREASE_PIN 6
#define BTN_L_DECREASE_PIN 5

//A timer to report stats ever 100 miliseconds.
LoopTimer reportTimer(100);

Encoder EncoderPitch(encoderPitchAPin, encoderPitchBPin);
Encoder EncoderRoll(encoderRollAPin, encoderRollBPin);

DAC DAC_Sim(SPI_DAC_PIN);

Button topSetSwitchA = Button(DI_TOP_A, NULL);
Button topSetSwitchB = Button(DI_TOP_B, NULL);
Button bottomSetSwitch = Button(DI_BOTTOM, NULL);
Button stopButton = Button(DI_STOP, NULL, INPUT);
Button homeButton = Button(DI_HOME, NULL);
Button canopyButton = Button(DI_CANOPY, NULL);

MotionController mc(
    &DAC_Sim,
    &EncoderPitch,
    &EncoderRoll,
    &topSetSwitchA,
    &topSetSwitchB,
    &bottomSetSwitch,
    &stopButton);

byte mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // device mac address

IPAddress ip;       // address to listen
uint16_t localPort; // port to listen

IPAddress ipOut;  // address to broadcast
uint16_t outPort; // port to send

#define UDP_MAX_PACKET_SIZE 25
// buffers for receiving and sending data
char packetBuffer[UDP_MAX_PACKET_SIZE]; // buffer to hold incoming packet

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

void loadConfiguration();
void setupUDP();
void readUDP();
void readButtons();
void report();

void increasePitchCallback(Button *button);
void decreasePitchCallback(Button *button);
void increaseRollCallback(Button *button);
void decreaseRollCallback(Button *button);
void increaseLiftCallback(Button *button);
void decreaseLiftCallback(Button *button);
void emergencyStopCallback(Button *button);

Button buttonPitchIncrease = Button(BTN_P_INCREASE_PIN, &increasePitchCallback);
Button buttonPitchDecrease = Button(BTN_P_DECREASE_PIN, &decreasePitchCallback);
Button buttonRollIncrease = Button(BTN_R_INCREASE_PIN, &increaseRollCallback);
Button buttonRollDecrease = Button(BTN_R_DECREASE_PIN, &decreaseRollCallback);
Button buttonLiftIncrease = Button(BTN_L_INCREASE_PIN, &increaseLiftCallback);
Button buttonLiftDecrease = Button(BTN_L_DECREASE_PIN, &decreaseLiftCallback);
Button buttonEmergencyStop = Button(BTN_ESTOP_PIN, &emergencyStopCallback);

void reverseBytes(void *start, int size);

void setup()
{
   pinMode(DO_INC_CW, OUTPUT);
   pinMode(DO_DEC_CW, OUTPUT);
   pinMode(DO_UP_CO, OUTPUT);
   pinMode(DO_P_DIS, OUTPUT);
   pinMode(DO_PRESSURE, OUTPUT);
   pinMode(DO_DOWN_CO, OUTPUT);

   digitalWrite(DO_INC_CW, LOW);
   digitalWrite(DO_DEC_CW, LOW);
   digitalWrite(DO_UP_CO, LOW);
   digitalWrite(DO_P_DIS, LOW);
   digitalWrite(DO_PRESSURE, LOW);
   digitalWrite(DO_DOWN_CO, LOW);

   Serial.begin(115200);

   //initializing the SD card
   Serial.print("Initializing SD card...");
   if (SD.begin(SPI_SD_PIN))
   {
      Serial.println("card initialized.");
      loadConfiguration();
   }
   else
   {
      Serial.println("Card failed, or not present");
      //use default settings
      ip = IPAddress(192, 168, 1, 6);
      localPort = 8888;
      ipOut = IPAddress(192, 168, 1, 5);
      outPort = 8888;
   }

   //DAC_Sim.setChannelLimit(All, 1200);
   DAC_Sim.setChannelLimit(All, 2400);

   setupUDP();
   mc.begin();
}

void loadConfiguration()
{
   Config config;
   config.getMACAddress(mac);
   config.getIPAddress(ip);
   config.getPort(localPort);
   config.getOutgoingIPAddress(ipOut);
   config.getOutgoingPort(outPort);
   config.getKP_Pitch(mc.kP_Pitch);
   config.getKI_Pitch(mc.kI_Pitch);
   config.getKD_Pitch(mc.kD_Pitch);
   config.getKP_Roll(mc.kP_Roll);
   config.getKI_Roll(mc.kI_Roll);
   config.getKD_Roll(mc.kD_Roll);
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
   readButtons();
   readUDP(); //where we want to go.
   mc.update();

   if (reportTimer.shouldTrigger())
   {
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
   if (packetSize == 0) {
      return;
   }
   // read the packet into packetBufffer
   Udp.read(packetBuffer, UDP_MAX_PACKET_SIZE);

   switch (packetBuffer[0])
   {
   case '0':
   {
      //emergency stop
      mc.emergencyStop();
      Serial.println("Received Emergency Stop Command");
      break;
   }
   case 'S':
   {
      //start
      mc.startSimulation();
      Serial.println("Received Start Command");
      break;
   }
   case 'E':
   {
      //stop
      mc.endSimulation();
      Serial.println("Received End Command");
      break;
   }
   case 'M':
   {
      // update set points
      if (packetSize < 9)
      {
         Serial.println("Malformed move command");
         break;
      }
      // if (mc.simState != running)
      // {
      //    Serial.println("Ignoring move command. Simulator is not running.");
      //    break;
      // }

      //convert big endian message to little endian
      char pitchBuffer[sizeof(unsigned int)] = "";
      char rollBuffer[sizeof(unsigned int)] = "";
      memcpy(&pitchBuffer, &packetBuffer[1], sizeof(unsigned int));
      memcpy(&rollBuffer, &packetBuffer[5], sizeof(unsigned int));
      reverseBytes(pitchBuffer, sizeof(unsigned int));
      reverseBytes(rollBuffer, sizeof(unsigned int));

      unsigned int ST_PitchSetpoint = 0;
      unsigned int ST_RollSetpoint = 0;
      memcpy(&ST_PitchSetpoint, pitchBuffer, sizeof(unsigned int));
      memcpy(&ST_RollSetpoint, rollBuffer, sizeof(unsigned int));

      mc.PitchSetpoint = mc.constrainedPitchSetpoint(ST_PitchSetpoint);
      mc.RollSetpoint = mc.constrainedRollSetpoint(ST_RollSetpoint);
      break;
   }
   case 'm':
   {
      if (packetSize == 2)
      {
         switch (packetBuffer[1])
         {
         case 'P':
         {
            mc.manualIncreasePitch(false);
            break;
         }
         case 'p':
         {
            mc.manualDecreasePitch(false);
            break;
         }
         case 'R':
         {
            mc.manualIncreaseRoll(false);
            break;
         }
         case 'r':
         {
            mc.manualDecreaseRoll(false);
            break;
         }
         case 'L':
         {
            mc.manualIncreaseLift(false);
            break;
         }
         case 'l':
         {
            mc.manualDecreaseLift(false);
            break;
         }
         case 's':
         {
            mc.manualIncreasePitch(true);
            mc.manualDecreasePitch(true);
            mc.manualIncreaseRoll(true);
            mc.manualDecreaseRoll(true);
            mc.manualIncreaseLift(true);
            mc.manualDecreaseLift(true);
            break;
         }
         }
      }
      break;
   }
   case '1':
   {
      // update pitch pid values
      if (packetSize != 25)
      {
         Serial.println("Malformed update pitch PID command");
         break;
      }
      memcpy(&mc.kP_Pitch, &packetBuffer[1], sizeof(double));
      memcpy(&mc.kI_Pitch, &packetBuffer[9], sizeof(double));
      memcpy(&mc.kD_Pitch, &packetBuffer[17], sizeof(double));
      break;
   }
   case '2':
   {
      //update roll pid values
      if (packetSize != 25)
      {
         Serial.println("Malformed update roll PID command");
         break;
      }
      memcpy(&mc.kP_Roll, &packetBuffer[1], sizeof(double));
      memcpy(&mc.kI_Roll, &packetBuffer[9], sizeof(double));
      memcpy(&mc.kD_Roll, &packetBuffer[17], sizeof(double));
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

void readButtons()
{
   buttonEmergencyStop.read();

   buttonLiftIncrease.read();
   buttonLiftDecrease.read();

   if (buttonLiftIncrease.pressed || buttonLiftDecrease.pressed)
   {
      //ignore other movement if lifting/lowering.
      return;
   }
   buttonPitchIncrease.read();
   buttonPitchDecrease.read();
   buttonRollIncrease.read();
   buttonRollDecrease.read();

   homeButton.read();
   canopyButton.read();
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
   int pitchPWM = (int)mc.PID_P_Output;
   int rollPWM = (int)mc.PID_R_Output;
   byte bufferSize = 63;
   char *ReplyBuffer = new char[bufferSize];
   byte inputs = 0;
   byte outputs = 0;

   if (topSetSwitchA.pressed)
      inputs |= (1 << 0);
   if (topSetSwitchB.pressed)
      inputs |= (1 << 1);
   if (bottomSetSwitch.pressed)
      inputs |= (1 << 2);
   if (homeButton.pressed)
      inputs |= (1 << 3);
   if (canopyButton.pressed)
      inputs |= (1 << 4);
   if (stopButton.pressed)
      inputs |= (1 << 5);

   if (digitalRead(DO_UP_CO))
      outputs |= (1 << 0);
   if (digitalRead(DO_DOWN_CO))
      outputs |= (1 << 1);
   if (digitalRead(DO_INC_CW))
      outputs |= (1 << 2);
   if (digitalRead(DO_DEC_CW))
      outputs |= (1 << 3);
   if (digitalRead(DO_PRESSURE))
      outputs |= (1 << 4);
   if (digitalRead(DO_P_DIS))
      outputs |= (1 << 5);

   memcpy(&ReplyBuffer[0], &mc.PitchSetpoint, sizeof(int));
   memcpy(&ReplyBuffer[2], &mc.PitchValue, sizeof(int));
   memcpy(&ReplyBuffer[4], &mc.RollSetpoint, sizeof(int));
   memcpy(&ReplyBuffer[6], &mc.RollValue, sizeof(int));
   memcpy(&ReplyBuffer[8], &pitchPWM, sizeof(int));
   memcpy(&ReplyBuffer[10], &rollPWM, sizeof(int));
   memcpy(&ReplyBuffer[12], &mc.simState, sizeof(byte));
   memcpy(&ReplyBuffer[13], &mc.kP_Pitch, sizeof(double));
   memcpy(&ReplyBuffer[21], &mc.kI_Pitch, sizeof(double));
   memcpy(&ReplyBuffer[29], &mc.kD_Pitch, sizeof(double));
   memcpy(&ReplyBuffer[37], &mc.kP_Roll, sizeof(double));
   memcpy(&ReplyBuffer[45], &mc.kI_Roll, sizeof(double));
   memcpy(&ReplyBuffer[53], &mc.kD_Roll, sizeof(double));
   memcpy(&ReplyBuffer[61], &inputs, sizeof(byte));
   memcpy(&ReplyBuffer[62], &outputs, sizeof(byte));

   Udp.beginPacket(ipOut, outPort);
   Udp.write(ReplyBuffer, bufferSize);
   Udp.endPacket();

   delete ReplyBuffer;
}

void emergencyStopCallback(Button *button)
{
   if (button->pressed)
   {
      Serial.println("Pressed Emergency Stop Button");
      mc.emergencyStop();
   }
}

void increasePitchCallback(Button *button)
{
   mc.manualIncreasePitch(!button->pressed);
}

void decreasePitchCallback(Button *button)
{
   mc.manualDecreasePitch(!button->pressed);
}

void increaseRollCallback(Button *button)
{
   mc.manualIncreaseRoll(!button->pressed);
}

void decreaseRollCallback(Button *button)
{
   mc.manualDecreaseRoll(!button->pressed);
}

void increaseLiftCallback(Button *button)
{
   mc.manualIncreaseLift(!button->pressed);
}

void decreaseLiftCallback(Button *button)
{
   mc.manualDecreaseLift(!button->pressed);
}

void reverseBytes(void *start, int size)
{
   byte *lo = (byte *)start;
   byte *hi = (byte *)start + size - 1;
   byte swap;
   while (lo < hi)
   {
      swap = *lo;
      *lo++ = *hi;
      *hi-- = swap;
   }
}
