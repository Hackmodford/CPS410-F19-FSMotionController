#ifndef FSMOTIONCONTROLLER_DAC_H
#define FSMOTIONCONTROLLER_DAC_H

#include <IPAddress.h>
#include <Arduino.h>
#include <SPI.h>

using namespace std;

#define R               0x000000
#define W               0x800000

#define REGISTER_FUNC   0x000000
#define REGISTER_DATA   0x100000
#define REGISTER_CGAIN  0x180000
#define REGISTER_FGAIN  0x200000
#define REGISTER_OFFSET 0x280000

#define FUNC_NOP        0x000000
#define FUNC_CLR        0x040000
#define FUNC_LD         0x050000

#define CHANNEL_A       0x000000
#define CHANNEL_B       0x010000
#define CHANNEL_C       0x020000
#define CHANNEL_D       0x030000
#define CHANNEL_ALL     0x040000

enum channelOption {
   A = CHANNEL_A,
   B = CHANNEL_B,
   C = CHANNEL_C,
   D = CHANNEL_D,
   All = CHANNEL_ALL
};

enum coarseGainOption {
   GAIN_LOW = 0x0,
   GAIN_MED = 0x1,
   GAIN_HIGH = 0x2
};

class DAC
{
public:
   DAC(int csPin, int ldacPin, int clrPin);

   void setValues(int valueA, int valueB, int valueC, int valueD);

   void setCoarseGain(channelOption channel, coarseGainOption option);

   // value is a 6-bit 2's complement number and is in
   // increments of 1 LSB according to the data sheet.
   void setFineGain(channelOption channel, byte value);

   // value is a 8-bit 2's complement number and is in crements
   // of 1/8 LSB.
   void setOffset(channelOption channel, byte value);

   void clear();
   void load();

private:
   void cmdToByteArray(int x, byte out[3]);
   void transferCmd(uint32_t cmd, SPITransferMode mode);
};

#endif //FSMOTIONCONTROLLER_DAC_H