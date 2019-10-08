#include <Arduino.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

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
#define motorCommonPin 4

#define encoderPitchAPin 22
#define encoderPitchBPin 23

#define encoderRollAPin 24
#define encoderRollBPin 25

#define REPORT_INTERVAL 100 //report every tenth of a second
long previousMillis = 0; //holds the count for every loop pass

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

//Mac and IP Address used for UDP connection.
byte mac[] = {
    0xA3, 0x03, 0x5C, 0x93, 0xEF, 0xD1};
IPAddress ip(192, 168, 1, 6);  // The arduino's IP Address
unsigned int localPort = 8888; // local port to listen on

IPAddress ipOut(192, 168, 1, 5); // The IP to send messages to.
unsigned int outPort = 8888;

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
   pinMode(7, OUTPUT);
   pinMode(6, OUTPUT);
   #if DEBUG
   Serial.begin(115200); // start the serial monitor link
   #endif

   PitchSetpoint = 0; //PITCH_ENCODER_180_DEG;//PITCH_ENCODER_MAX - 10;
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

   setupUDP();
}

void setupUDP()
{
   // start the Ethernet
   Ethernet.begin(mac, ip);

   // Check for Ethernet hardware present
   if (Ethernet.hardwareStatus() == EthernetNoHardware)
   {
      #if DEBUG
      Serial.println("Ethernet shield was not found. Sorry, can't run without hardware. :(");
      #endif
      return;
   }
   if (Ethernet.linkStatus() == LinkOFF)
   {
      #if DEBUG
      Serial.println("Ethernet cable is not connected.");
      #endif
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
      digitalWrite(7, HIGH);
      digitalWrite(6, LOW);
      PID_P_Output = 0;
      PID_R_Output = 0;
      moveMotors();
      break;
   case starting:
      if (moveUp())
      {
         simState = running;
      }
      break;
   case running:
      digitalWrite(7, LOW);
      digitalWrite(6, HIGH);
      computePID();
      moveMotors();
      break;
   case ending:
      if (moveDown())
      {
         simState = stopped;
      }
      break;
   case emergency_stop:
      break;
   }

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
      #if DEBUG
      Serial.println("Received Emergency Stop Command");
      #endif
      break;
   case 'S':
      //start
      startSimulation();
      #if DEBUG
      Serial.println("Received Start Command");
      #endif
      break;
   case 'E':
      //stop
      endSimulation();
      #if DEBUG
      Serial.println("Received End Command");
      #endif
      break;
   case 'M':
      //update set points
      if (packetSize != 5)
      {
         #if DEBUG
         Serial.println("Malformed move command");
         #endif
         break;
      }
      memcpy(&PitchSetpoint, &packetBuffer[1], sizeof(int));
      memcpy(&RollSetpoint, &packetBuffer[3], sizeof(int));
      break;
   case '1':
      // update pitch pid values
      if (packetSize != 24)
      {
         #if DEBUG
         Serial.println("Malformed update pitch PID command");
         #endif
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
         #if DEBUG
         Serial.println("Malformed update roll PID command");
         #endif
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
      #if DEBUG
      Serial.println("Cannot start simulation due to emergency stop.");
      #endif
      return;
   }
   if (simState != stopped)
   {
      #if DEBUG
      Serial.println("Cannot start simulation as the simulator is not stopped.");
      #endif
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
      #if DEBUG
      Serial.println("Cannot start simulation due to emergency stop.");
      #endif
      return;
   }
   if (simState != running)
   {
      #if DEBUG
      Serial.println("Cannot start simulation as the simulator is not running.");
      #endif
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
