#include <EEPROM.h>

//
// map of registers
// 0-1 - pid_Pitch_P
// 2-3 - pid_Pitch_I
// 4-5 - pid_Pitch_D
// 6-7 - pid_Roll_P
// 8-9 - pid_Roll_I
// 10-11 - pid_Roll_D
// 12-13 - pid_Yaw_P
// 14-15 - pid_Yaw_I
// 16-17 - pid_Yaw_D

void initialize_Enterprise()
{
  for(int i=0; i<3; i++)
  {
    pid_Pitch_Settings[i]=(double)((EEPROM.read(2*i))|(EEPROM.read(2*i+1)<<8))/100;
    pid_Roll_Settings[i]=(double)((EEPROM.read(2*i+6))|(EEPROM.read(2*i+7)<<8))/100;
    pid_Yaw_Settings[i]=(double)((EEPROM.read(2*i+12))|(EEPROM.read(2*i+13)<<8))/100;
  }
}

void flash_PID_Settings()
{
  int value;
  for(int i=0; i<3; i++)
  {
    value=(int)((pid_Pitch_Settings[i])*100);
    EEPROM.write(2*i, value & 0xFF);
    EEPROM.write(2*i+1, (value >> 8) & 0xF);
    
    value=(int)((pid_Roll_Settings[i])*100);
    EEPROM.write(2*i+6, value & 0xFF);
    EEPROM.write(2*i+7, (value >> 8) & 0xF);
    
    value=(int)((pid_Yaw_Settings[i])*100);
    EEPROM.write(2*i+12, value & 0xFF);
    EEPROM.write(2*i+13, (value >> 8) & 0xF);
    
    //pid_Pitch_Settings[i]=(double)((EEPROM.read(i*2))|(EEPROM.read(2*i+1)<<8))/100;
    //pid_Roll_Settings[i]=(double)((EEPROM.read(i*2+6))|(EEPROM.read(2*i+7)<<8))/100;
    //pid_Yaw_Settings[i]=(double)((EEPROM.read(i*2+12))|(EEPROM.read(2*i+13)<<8))/100;
  }
}
