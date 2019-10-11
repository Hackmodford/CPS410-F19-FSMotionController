#include "DAC.h"

using namespace std;

#define FINE_GAIN_MIN -32
#define FINE_GAIN_MAX  31

#define COARSE_GAIN_OPTION_1

#define SPI_DAC_SETTINGS SPISettings(30000000/*84*/, MSBFIRST, SPI_MODE0)

int _csPin = 0;
int _ldacPin = 0;
int _clrPin = 0;

DAC::DAC(int csPin, int ldacPin, int clrPin)
{
   _csPin = csPin;
   _ldacPin = ldacPin;
   _clrPin = clrPin;

   pinMode(_ldacPin, OUTPUT);
   pinMode(_clrPin, OUTPUT);

   digitalWrite(_ldacPin, HIGH); //Load DAC pin for DAC. Make it LOW if not in use.
   digitalWrite(_clrPin, HIGH);  // Asynchronous clear pin for DAC. Make it HIGH if you are not using it
}

void DAC::setValues(int valueA, int valueB, int valueC, int valueD)
{
   SPI.beginTransaction(SPI_DAC_SETTINGS);

   uint32_t dacCmd1 = W | REGISTER_DATA | CHANNEL_A | valueA;
   uint32_t dacCmd2 = W | REGISTER_DATA | CHANNEL_B | valueB;
   uint32_t dacCmd3 = W | REGISTER_DATA | CHANNEL_C | valueC;
   uint32_t dacCmd4 = W | REGISTER_DATA | CHANNEL_D | valueD;

   transferCmd(dacCmd1, SPI_CONTINUE);
   transferCmd(dacCmd2, SPI_CONTINUE);
   transferCmd(dacCmd3, SPI_CONTINUE);
   transferCmd(dacCmd4, SPI_LAST);

   //triger simultaneous load
   digitalWrite(_ldacPin, LOW);
   delayMicroseconds(0.003);
   digitalWrite(_ldacPin, HIGH);

   SPI.endTransaction();
}

void DAC::setCoarseGain(channelOption channel, coarseGainOption option)
{
   uint32_t cmd = W | REGISTER_CGAIN | channel | option;
   SPI.beginTransaction(SPI_DAC_SETTINGS);
   transferCmd(cmd, SPI_LAST);
   SPI.endTransaction();
}

void DAC::setFineGain(channelOption channel, char value)
{
   //value is 6-bit signed.
   value = constrain(value, FINE_GAIN_MIN, FINE_GAIN_MAX);
   value = ((value >> 2) & 0x32) | (value & 0x31);
   uint32_t cmd = W | REGISTER_FGAIN | channel | (value && 0x3F);
   SPI.beginTransaction(SPI_DAC_SETTINGS);
   transferCmd(cmd, SPI_LAST);
   SPI.endTransaction();
}

void DAC::setOffset(channelOption channel, byte value)
{
   uint32_t cmd = W | REGISTER_OFFSET | channel | value;
   SPI.beginTransaction(SPI_DAC_SETTINGS);
   transferCmd(cmd, SPI_LAST);
   SPI.endTransaction();
}

void DAC::clear()
{
   uint32_t cmd = W | REGISTER_FUNC | FUNC_CLR;
   SPI.beginTransaction(SPI_DAC_SETTINGS);
   transferCmd(cmd, SPI_LAST);
   SPI.endTransaction();
}

void DAC::load()
{
   uint32_t cmd = W | REGISTER_FUNC | FUNC_LD;
   SPI.beginTransaction(SPI_DAC_SETTINGS);
   transferCmd(cmd, SPI_LAST);
   SPI.endTransaction();
}

void DAC::transferCmd(uint32_t cmd, SPITransferMode mode)
{
   byte cmdBytes[3] = {};
   cmdToByteArray(cmd, cmdBytes);
   SPI.transfer(_csPin, cmdBytes[0], SPI_CONTINUE);
   SPI.transfer(_csPin, cmdBytes[1], SPI_CONTINUE);
   SPI.transfer(_csPin, cmdBytes[2], mode);
}

void DAC::cmdToByteArray(int x, byte out[3])
{
   out[0] = ((byte)((x >> 16) & 0xFF));
   out[0] = ((byte)((x >> 8) & 0xFF));
   out[1] = ((byte)(x & 0xFF));
}
