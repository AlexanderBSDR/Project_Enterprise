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

//18-19 - gains_Speed[0]
//20-21 - gains_Speed[1]
//22-23 - gains_Speed[2]
//24-25 - gains_Speed[3]

//26-27 - gains_Pitch[0]
//28-29 - gains_Pitch[1]
//30-31 - gains_Pitch[2]
//32-33 - gains_Pitch[3]

//34-35 - gains_Roll[0]
//36-37 - gains_Roll[1]
//38-39 - gains_Roll[2]
//40-41 - gains_Roll[3]

//42-43 - gains_Yaw[0]
//44-45 - gains_Yaw[1]
//46-47 - gains_Yaw[2]
//48-49 - gains_Yaw[3]


void initialize_Enterprise_EEPROM()
{
  for(int i=0; i<3; i++)
  {
    pid_Pitch_Settings[i]=(double)((EEPROM.read(2*i))|(EEPROM.read(2*i+1)<<8))/100;
    pid_Roll_Settings[i]=(double)((EEPROM.read(2*i+6))|(EEPROM.read(2*i+7)<<8))/100;
    pid_Yaw_Settings[i]=(double)((EEPROM.read(2*i+12))|(EEPROM.read(2*i+13)<<8))/100;
    
    gains_Speed[i]=(double)((EEPROM.read(2*i+18))|(EEPROM.read(2*i+19)<<8))/100;
    gains_Pitch[i]=(double)((EEPROM.read(2*i+26))|(EEPROM.read(2*i+27)<<8))/100;
    gains_Roll[i]=(double)((EEPROM.read(2*i+34))|(EEPROM.read(2*i+35)<<8))/100;
    gains_Yaw[i]=(double)((EEPROM.read(2*i+42))|(EEPROM.read(2*i+43)<<8))/100;
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
  }
}

void flash_Mixer_Settings()
{
  int value;
  for(int i=0; i<3; i++)
  {
    
    value=(int)((gains_Speed[i])*100);
    EEPROM.write(2*i+18, value & 0xFF);
    EEPROM.write(2*i+19, (value >> 8) & 0xF);
    
    value=(int)((gains_Pitch[i])*100);
    EEPROM.write(2*i+26, value & 0xFF);
    EEPROM.write(2*i+27, (value >> 8) & 0xF);
    
    value=(int)((gains_Roll[i])*100);
    EEPROM.write(2*i+34, value & 0xFF);
    EEPROM.write(2*i+35, (value >> 8) & 0xF);
    
    value=(int)((gains_Yaw[i])*100);
    EEPROM.write(2*i+42, value & 0xFF);
    EEPROM.write(2*i+43, (value >> 8) & 0xF);

  }
}
