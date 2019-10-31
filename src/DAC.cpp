#include "DAC.h"

using namespace std;

#define FINE_GAIN_MIN -32
#define FINE_GAIN_MAX 31

#define COARSE_GAIN_OPTION_1

int _csPin = 0;
int _ldacPin = 0;
int _clrPin = 0;

channelOption byteToChannelOption(byte x)
{
   switch (x)
   {
   case 0:
      return A;
      break;
   case 1:
      return B;
      break;
   case 2:
      return C;
      break;
   case 3:
      return D;
      break;
   case 4:
      return All;
      break;
   }
   return A;
}

coarseGainOption byteToCoarseGainOption(byte x)
{
   switch (x)
   {
   case 0:
      return GAIN_LOW;
      break;
   case 1:
      return GAIN_MED;
      break;
   case 2:
      return GAIN_HIGH;
      break;
   }
   return GAIN_LOW;
}

DAC::DAC(int csPin)
{
   _csPin = csPin;
}

void DAC::begin()
{
   SPI.begin(_csPin);
   SPI.setClockDivider(_csPin, 3);     //This can probably be sped up now that the rest of the code is better optimized. Limited by ADC
   SPI.setDataMode(_csPin, SPI_MODE1); //This should be 3 for the AD7732
}

void DAC::setChannelLimit(channelOption channel, int value)
{
   switch (channel)
   {
   case A:
      maxChannelALimit = value;
      break;
   case B:
      maxChannelBLimit = value;
      break;
   case C:
      maxChannelCLimit = value;
      break;
   case D:
      maxChannelDLimit = value;
      break;
   case All:
      maxChannelALimit = value;
      maxChannelBLimit = value;
      maxChannelCLimit = value;
      maxChannelDLimit = value;
      break;
   }
}

int DAC::getChannelLimit(channelOption channel)
{
   switch (channel)
   {
   case A:
      return maxChannelALimit;
   case B:
      return maxChannelBLimit;
   case C:
      return maxChannelCLimit;
   case D:
      return maxChannelDLimit;
   case All:
      return 0;
   }
   return 0;
}

void DAC::setChannel(channelOption channel, int value)
{
   //The bitmask for the values is necessary because of the conversion
   //from signed to unsigned. We want the actual bits, that a fancy
   //conversion to an unsigned 32 bit number.
   if (value > 0) {
      value = min(value, getChannelLimit(channel));
   } else if (value < 0) {
      value = max(value, -getChannelLimit(channel));
   }

   uint32_t cmd = W | REGISTER_DATA | channel | (value & 0xFFFF);
   transferCmd(cmd);
}

void DAC::setCoarseGain(channelOption channel, coarseGainOption option)
{
   uint32_t cmd = W | REGISTER_CGAIN | channel | option;
   transferCmd(cmd);
}

void DAC::setFineGain(channelOption channel, char value)
{
   //value is converted to a 6-bit signed representation.
   value = constrain(value, FINE_GAIN_MIN, FINE_GAIN_MAX);
   value = ((value >> 2) & 0x32) | (value & 0x31);
   uint32_t cmd = W | REGISTER_FGAIN | channel | (value && 0x3F);
   transferCmd(cmd);
}

void DAC::setOffset(channelOption channel, byte value)
{
   uint32_t cmd = W | REGISTER_OFFSET | channel | value;
   transferCmd(cmd);
}

void DAC::clear()
{
   uint32_t cmd = W | REGISTER_FUNC | FUNC_CLR;
   transferCmd(cmd);
}

void DAC::load()
{
   uint32_t cmd = W | REGISTER_FUNC | FUNC_LD;
   transferCmd(cmd);
}

void DAC::transferCmd(uint32_t cmd)
{
   byte cmdBytes[3];
   cmdToByteArray(cmd, cmdBytes);
   SPI.transfer(_csPin, cmdBytes, 3);
}

void DAC::cmdToByteArray(int x, byte out[3])
{
   out[0] = (x >> 16) & 0xFF;
   out[1] = (x >> 8) & 0xFF;
   out[2] = (x >> 0) & 0xFF;
}
