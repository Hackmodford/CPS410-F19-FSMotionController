#include <IniFile.h>
#include "config.h"
using namespace std;

const char *filename = "/config.ini";
const size_t bufferLen = 80;
char buffer[bufferLen];
IniFile ini = IniFile(filename);

Config::Config()
{
   //ini = Inifile(filename);
   if (!ini.open())
   {
      Serial.print("Ini file ");
      Serial.print(filename);
      Serial.println(" does not exist");
   }
   else
   {
      Serial.println("Config file exists");
   }

   if (!ini.validate(buffer, bufferLen))
   {
      Serial.print("ini file ");
      Serial.print(ini.getFilename());
      Serial.print(" not valid: ");
      printErrorMessage(ini.getError(), true);
   }
}

bool Config::getMACAddress(uint8_t mac[6])
{
   return ini.getMACAddress("Network", "mac", buffer, bufferLen, mac);
}

bool Config::getIPAddress(IPAddress& ip)
{
   return ini.getIPAddress("Network", "ip", buffer, bufferLen, ip);
}

bool Config::getPort(uint16_t &port)
{
   return ini.getValue("Network", "port", buffer, bufferLen, port);
}

bool Config::getOutgoingIPAddress(IPAddress &ip)
{
   return ini.getIPAddress("Network", "ip_output", buffer, bufferLen, ip);
}

bool Config::getOutgoingPort(uint16_t &port)
{
   return ini.getValue("Network", "port_output", buffer, bufferLen, port);
}

bool Config::getKP_Pitch(double &p)
{
   return ini.getValue("Pitch", "kP", buffer, bufferLen, p);
}

bool Config::getKI_Pitch(double &i)
{
   return ini.getValue("Pitch", "kI", buffer, bufferLen, i);
}

bool Config::getKD_Pitch(double &d)
{
   return ini.getValue("Pitch", "kD", buffer, bufferLen, d);
}

bool Config::getKP_Roll(double &p)
{
   return ini.getValue("Roll", "kP", buffer, bufferLen, p);
}

bool Config::getKI_Roll(double &i)
{
   return ini.getValue("Roll", "kI", buffer, bufferLen, i);
}
bool Config::getKD_Roll(double &d)
{
   return ini.getValue("Roll", "kD", buffer, bufferLen, d);
}

void Config::printErrorMessage(uint8_t e, bool eol)
{
   switch (e)
   {
   case IniFile::errorNoError:
      Serial.print("no error");
      break;
   case IniFile::errorFileNotFound:
      Serial.print("file not found");
      break;
   case IniFile::errorFileNotOpen:
      Serial.print("file not open");
      break;
   case IniFile::errorBufferTooSmall:
      Serial.print("buffer too small");
      break;
   case IniFile::errorSeekError:
      Serial.print("seek error");
      break;
   case IniFile::errorSectionNotFound:
      Serial.print("section not found");
      break;
   case IniFile::errorKeyNotFound:
      Serial.print("key not found");
      break;
   case IniFile::errorEndOfFile:
      Serial.print("end of file");
      break;
   case IniFile::errorUnknownError:
      Serial.print("unknown error");
      break;
   default:
      Serial.print("unknown error value");
      break;
   }
   if (eol)
      Serial.println();
}
