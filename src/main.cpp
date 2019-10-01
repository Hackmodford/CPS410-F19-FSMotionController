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

#define kP_Pitch 1
#define kI_Pitch 1
#define kD_Pitch 1

#define ROLL_ENCODER_MIN 0
#define ROLL_ENCODER_MAX 8640     //72*120 and 360 degree
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

#define loop_test_times 20000 //Run loop 20000 times then calculate time

int loop_counter = 0; //holds the count for every loop pass

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

//Mac and IP Address used for UDP connection.
byte mac[] = {
    0xA3, 0x03, 0x5C, 0x93, 0xEF, 0xD1};
IPAddress ip(192, 168, 1, 7);  // The arduino's IP Address
unsigned int localPort = 8888; // local port to listen on

IPAddress ipOut(192, 168, 1, 6); // The IP to send messages to.
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
   Serial.begin(115200); // start the serial monitor link
   while (!Serial)
   {
      ; // wait for serial port to connect. Needed for native USB port only
   }

   PitchSetpoint = 0; //PITCH_ENCODER_180_DEG;//PITCH_ENCODER_MAX - 10;
   RollSetpoint = 0;
   simState = stopped;

   pinMode(motorPitchPin, OUTPUT);
   pinMode(motorRollPin, OUTPUT);

   // Use negative min to allow PID calcuation to reverse motor.
   // But keep range of 0-255
   PID_Pitch.SetOutputLimits(-128, 127);
   PID_Roll.SetOutputLimits(-128, 127);

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
      Serial.println("Ethernet shield was not found. Sorry, can't run without hardware. :(");
      return;
   }
   if (Ethernet.linkStatus() == LinkOFF)
   {
      Serial.println("Ethernet cable is not connected.");
   }
   Udp.begin(localPort);
}

void loop()
{
   loop_counter++;

   //readUDP();         //where we want to go.
   readEncoderData(); //where we currently are.

   switch (simState)
   {
   case stopped:
      break;
   case starting:
      if (moveUp())
      {
         simState = running;
      }
      break;
   case running:
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

   if (loop_counter == loop_test_times)
   {
      loop_counter = 0;
      report();
      switch (simState)
      {
      case stopped:
         Serial.println("State = Stopped");
         break;
      case starting:
         Serial.println("State = Starting");
         break;
      case running:
         Serial.println("State = Running");
         break;
      case ending:
         Serial.println("State = Ending");
         break;
      case emergency_stop:
         Serial.println("State = EMERGENCY STOP");
         break;
      }
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
   Serial.println("Contents:");
   Serial.println(packetBuffer);

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
      if (packetSize != 5)
      {
         Serial.println("Malformed move command");
         break;
      }
      PitchSetpoint = 0;
      PitchSetpoint = (packetBuffer[1] << 8) | packetBuffer[2];
      RollSetpoint = 0;
      RollSetpoint = (packetBuffer[3] << 8) | packetBuffer[4];
      break;
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
   analogWrite(motorPitchPin, PID_P_Output + 128);
   analogWrite(motorRollPin, PID_R_Output + 128);
}

void report()
{
   //byte map
   //0-1 Pitch Setpoint
   //2-3 Pitch Value
   //4-5 Roll Setpoint
   //6-7 Roll Value
   //8 Pitch PWM
   //9 Roll PWM

   byte pitchPWM = (byte)PID_P_Output;
   byte rollPWM = (byte)PID_R_Output;

   char ReplyBuffer[10] = {
       (byte)(PitchSetpoint >> 8),
       (byte)PitchSetpoint,
       (byte)(PitchValue >> 8),
       (byte)PitchValue,
       (byte)(RollSetpoint >> 8),
       (byte)RollSetpoint,
       (byte)(RollValue >> 8),
       (byte)RollValue,
       pitchPWM,
       rollPWM};

   Udp.beginPacket(ipOut, outPort);
   Udp.write(ReplyBuffer);
   Udp.endPacket();
   Serial.println("Sent data to monitor.");
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
   Serial.println("Starting simulation.");
   delay(5000);
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
   delay(5000);
   simState = ending;
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
