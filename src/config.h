#ifndef FSMOTIONCONTROLLER_CONFIG_H
#define FSMOTIONCONTROLLER_CONFIG_H

#include <IPAddress.h>

using namespace std;

class Config {

   public:
      Config();

      bool getMACAddress(uint8_t mac[6]);
      bool getIPAddress(IPAddress& ip);
      bool getPort(uint16_t& port);
      bool getOutgoingIPAddress(IPAddress& ip);
      bool getOutgoingPort(uint16_t& port);
      bool getKP_Pitch(double& p);
      bool getKI_Pitch(double& i);
      bool getKD_Pitch(double& d);
      bool getKP_Roll(double& p);
      bool getKI_Roll(double& i);
      bool getKD_Roll(double& d);
   private:
      void printErrorMessage(uint8_t e, bool eol);
};

#endif //FSMOTIONCONTROLLER_CONFIG_H