#ifndef FSMOTIONCONTROLLER_DAC_H
#define FSMOTIONCONTROLLER_DAC_H

#include <IPAddress.h>
#include <Arduino.h>
#include <SPI.h>

using namespace std;

#define W 0x000000 //write
#define R 0x800000 //read

#define REGISTER_FUNC 0x000000
#define REGISTER_DATA 0x100000
#define REGISTER_CGAIN 0x180000
#define REGISTER_FGAIN 0x200000
#define REGISTER_OFFSET 0x280000

#define FUNC_NOP 0x000000
#define FUNC_CLR 0x040000
#define FUNC_LD 0x050000

#define CHANNEL_A 0x000000
#define CHANNEL_B 0x010000
#define CHANNEL_C 0x020000
#define CHANNEL_D 0x030000
#define CHANNEL_ALL 0x040000

enum channelOption
{
   A = CHANNEL_A,
   B = CHANNEL_B,
   C = CHANNEL_C,
   D = CHANNEL_D,
   All = CHANNEL_ALL
};

enum coarseGainOption
{
   GAIN_LOW = 0x0,
   GAIN_MED = 0x1,
   GAIN_HIGH = 0x2
};

channelOption byteToChannelOption(byte x);
coarseGainOption byteToCoarseGainOption(byte x);

class DAC
{
public:
   DAC(int csPin);
   
   void begin();
   void setChannelLimit(channelOption channel, int value);
   void setChannel(channelOption channel, int value);
   void setCoarseGain(channelOption channel, coarseGainOption option);

   // value is a 6-bit 2's complement number and is in
   // increments of 1 LSB according to the data sheet.
   void setFineGain(channelOption channel, char value);

   // value is a 8-bit 2's complement number and is in crements
   // of 1/8 LSB.
   void setOffset(channelOption channel, byte value);

   void clear();
   void load();

private:
   int maxChannelALimit = INT16_MAX;
   int maxChannelBLimit = INT16_MAX;
   int maxChannelCLimit = INT16_MAX;
   int maxChannelDLimit = INT16_MAX;

   void cmdToByteArray(int x, byte out[3]);
   int getChannelLimit(channelOption channel);
   void transferCmd(uint32_t cmd);
};

#endif //FSMOTIONCONTROLLER_DAC_H