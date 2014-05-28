#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>

#include "DOF_Sensors.h"

#define CALC_EXECUTION_TIME 1
#define USE_EEPROM 0
#define DEBUG_MODE 1

#define ENGINE_ONE_PIN 5
#define ENGINE_TWO_PIN 6
#define ENGINE_THREE_PIN 11
#define ENGINE_FOUR_PIN 10
#define LED_PIN 13

#define ENGINEMAXPWM 2000
#define ENGINEMINPWM 1000
#define ENGINESTARTUPPWM 900


#define MAX_SPEED_ANGLES 360
#define MAX_PITCH_ANGLES 90
#define MAX_ROLL_ANGLES 90
#define MAX_YAW_ANGLES 90
#define MAX_SPEED_STEP (ENGINEMAXPWM-ENGINEMINPWM)/MAX_SPEED_ANGLES

extern double acc_vector[3];
unsigned int enginesX[4];

unsigned int altitude=0;

double gains_Speed[4]={1, 1, 1, 1};
double gains_Pitch[4]={1, 1, 1, 1};
double gains_Roll[4]={1, 1, 1, 1};
double gains_Yaw[4]={1, 1, 1, 1};

double pid_Pitch_Settings[3]={2,0,0};
double pid_Roll_Settings[3]={2,0,0}; 
double pid_Yaw_Settings[3]={2,0,0};

unsigned int timerDataUpdate=100;

#define maxPacketSize 32
//[0xFF][0x20]//[2][2][2][2]-engines//[2][2][2]-accell//[2][2][2]-gyro//[2][2][2]-compass//[2]-battery//[2]-height
//2+8+6+6+6+2+2=32

double actual_Angles[3];
double set_Angles[3];
double controller_Angles[3];
double set_Speed;

unsigned int batteryPower=0;
unsigned long previousMillis = 0;

bool isManual=false;

Servo enginesX_obj[4];

PID controller_X(&actual_Angles[0], &controller_Angles[0], &set_Angles[0], pid_Roll_Settings[0], pid_Roll_Settings[1], pid_Roll_Settings[2], DIRECT);
PID controller_Y(&actual_Angles[1], &controller_Angles[1], &set_Angles[1], pid_Pitch_Settings[0], pid_Pitch_Settings[1], pid_Pitch_Settings[2], DIRECT);
PID controller_Z(&actual_Angles[2], &controller_Angles[2], &set_Angles[2], pid_Yaw_Settings[0], pid_Yaw_Settings[1], pid_Yaw_Settings[2], DIRECT);


void setup() {
  #if USE_EEPROM==1
  initialize_Enterprise_EEPROM();
  #endif
  initSensorsStick();

  enginesX_obj[0].attach(ENGINE_ONE_PIN);
  enginesX_obj[0].writeMicroseconds(ENGINESTARTUPPWM);
  
  enginesX_obj[1].attach(ENGINE_TWO_PIN);
  enginesX_obj[1].writeMicroseconds(ENGINESTARTUPPWM);
  
  enginesX_obj[2].attach(ENGINE_THREE_PIN);
  enginesX_obj[2].writeMicroseconds(ENGINESTARTUPPWM);
  
  enginesX_obj[3].attach(ENGINE_FOUR_PIN);
  enginesX_obj[3].writeMicroseconds(ENGINESTARTUPPWM);
  
  controller_X.SetTunings(pid_Roll_Settings[0], pid_Roll_Settings[1], pid_Roll_Settings[2]);
  controller_Y.SetTunings(pid_Pitch_Settings[0], pid_Pitch_Settings[1], pid_Pitch_Settings[2]);
  controller_Z.SetTunings(pid_Yaw_Settings[0], pid_Yaw_Settings[1], pid_Yaw_Settings[2]);

  controller_X.SetMode(AUTOMATIC);
  controller_Y.SetMode(AUTOMATIC);
  controller_Z.SetMode(AUTOMATIC); 
 
  controller_X.SetOutputLimits(-MAX_PITCH_ANGLES, MAX_PITCH_ANGLES);
  controller_Y.SetOutputLimits(-MAX_ROLL_ANGLES, MAX_ROLL_ANGLES);
  controller_Z.SetOutputLimits(-MAX_YAW_ANGLES, MAX_YAW_ANGLES);

  controller_X.SetSampleTime(10);
  controller_Y.SetSampleTime(10);
  controller_Z.SetSampleTime(10);

  Serial.begin(115200);
  Serial.write("Enterprise is at your commands!\n");
  
}

void loop() {
  
  //digitalWrite(LED_PIN, HIGH);
  #if CALC_EXECUTION_TIME==1
    unsigned long start=micros();
  #endif
  
  //#1 - get current parameters (actual_Angles)
  getAngles(actual_Angles);
  
  //#2 - calculate if we need to make any adjustments (controller_Angles)
  controller_Y.Compute(); //pitch
  controller_X.Compute(); // roll
  controller_Z.Compute(); //yaw

  //#3 - do adjustments to engines
  //if(set_Angles[4]>0)
  if (isManual==false)
    generateEnginesSpeeds(set_Speed, controller_Angles[1], controller_Angles[0], controller_Angles[2]);
  updateEngineParameters();
  
  //#4 - check if we have new commands (set_Angles) 
  while(Serial.available() > 0)
  {
    delay(1); //let's delete it later!
    while (Serial.read() == 0xFF)
    {
      double v = 0;
      char str[maxPacketSize];
   
      switch (Serial.read())
      {
        case 0x01:
        {
          isManual=true;
          for(int i=0; i<4; i++)
            enginesX[i] = (unsigned int)((Serial.read())|(Serial.read()<<8));
            
          updateEngineParameters();

          #if DEBUG_MODE==1
          Serial.print("Engines:   ");
          for (int i=0; i<4; i++)
          {
            Serial.print(enginesX[i]);
            Serial.print("/");
          }
          Serial.println();
          #endif

          break;
        };

        case 0x02:
        {
          set_Speed = (double)((int)(((Serial.read())|(Serial.read()<<8))));
          isManual=false;
          
          #if DEBUG_MODE==1
          sprintf(str, "S_C: %26d\n", (int)set_Speed); /////////////////////////////
          Serial.write(str);
          #endif 
          break;
        };

        case 0x03: 
        {
          v = (double)((int)(((Serial.read())|(Serial.read()<<8))));
          isManual=false;
         // set_Angles[1]+=v;
         
          #if DEBUG_MODE==1
          sprintf(str, "P_C: %26d\n", (int)v); /////////////////////////////
          Serial.write(str);
          #endif
          break;
        };

        case 0x04:
        {
          v = (double)((int)(((Serial.read())|(Serial.read()<<8))));
          isManual=false;
          //set_Angles[0]+=v;
          
          #if DEBUG_MODE==1
          sprintf(str, "R_C: %26d\n", (int)v); /////////////////////////////
          Serial.write(str);
          #endif
          break;
        };
        
        case 0x33:
        {
          v = (double)((int)(((Serial.read())|(Serial.read()<<8))));
          isManual=false;
          //set_Angles[2]+=v;
          
          #if DEBUG_MODE==1
          sprintf(str, "Y_C: %26d\n", (int)v); /////////////////////////////
          Serial.write(str);
          #endif
          break;
        };
          
          case 0x05: ///speed mixer settings
          {
          for(int i=0; i<4; i++)
            gains_Speed[i] = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
            
          #if DEBUG_MODE==1
          Serial.print("Speed:     ");
          for (int i=0; i<4; i++)
          {
            Serial.print(gains_Speed[i], 2);
            Serial.print("/");
          }
          Serial.println();
          #endif
          
          #if USE_EEPROM==1
            flash_Mixer_Settings();
          #endif
          
          break;
          };
          
          case 0x06: ///pitch mixer settings
          {
          for(int i=0; i<4; i++)
            gains_Pitch[i] = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
            
          #if DEBUG_MODE==1
          Serial.print("Pitch:     ");
          for (int i=0; i<4; i++)
          {
            Serial.print(gains_Pitch[i], 2);
            Serial.print("/");
          }
          Serial.println();
          #endif
          
          #if USE_EEPROM==1
            flash_Mixer_Settings();
          #endif
          
          break;
          };
          
          case 0x07: ///roll mixer settings
          {
            for(int i=0; i<4; i++)
              gains_Roll[i] = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
              
            #if DEBUG_MODE==1
            Serial.print("Roll:      ");
            for (int i=0; i<4; i++)
            {
              Serial.print(gains_Roll[i], 2);
              Serial.print("/");
            }
            Serial.println();
            #endif
            
            #if USE_EEPROM==1
              flash_Mixer_Settings();
            #endif
            
            break;
          };
                    
          case 0x30: ///yaw mixer settings
          {
            for(int i=0; i<4; i++)
              gains_Roll[i] = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
              
              
            #if DEBUG_MODE==1
            Serial.print("Yaw:      ");
            for (int i=0; i<4; i++)
            {
              Serial.print(gains_Yaw[i], 2);
              Serial.print("/");
            }
            Serial.println();
            #endif
            
            #if USE_EEPROM==1
              flash_Mixer_Settings();
            #endif
            
            break;
          };
          
          case 0x08: ///pitch mixer settings
          {
            for(int i=0; i<3; i++)
              pid_Pitch_Settings[i] = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
            for(int i=0; i<3; i++)
              pid_Roll_Settings[i] = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
            for(int i=0; i<3; i++)
              pid_Yaw_Settings[i] = (double)((int)(Serial.read())|(Serial.read()<<8))/100;

            #if DEBUG_MODE==1
            Serial.print("P:");
            Serial.print(pid_Pitch_Settings[0]);
            Serial.print("/");
            Serial.print(pid_Pitch_Settings[1]);
            Serial.print("/");
            Serial.print(pid_Pitch_Settings[2]);
            Serial.print("/");
            Serial.print(pid_Roll_Settings[0]);
            Serial.print("/");
            Serial.print(pid_Roll_Settings[1]);
            Serial.print("/");
            Serial.print(pid_Roll_Settings[2]);
            Serial.println();
            #endif
          
            #if USE_EEPROM==1
              flash_PID_Settings();
            #endif
          
            controller_Z.SetTunings(pid_Yaw_Settings[0], pid_Yaw_Settings[1], pid_Yaw_Settings[2]);
            controller_Y.SetTunings(pid_Pitch_Settings[0], pid_Pitch_Settings[1], pid_Pitch_Settings[2]);
            controller_X.SetTunings(pid_Roll_Settings[0], pid_Roll_Settings[1], pid_Roll_Settings[2]);
 
            #if DEBUG_MODE==1         
            printVibrations();  //<-----------
            #endif
            break;
          };

          case 0x09:
          {
            timerDataUpdate=(unsigned int)((Serial.read())|(Serial.read()<<8));
            break; 
          };
      }
    }
  }
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis > timerDataUpdate)
  {
    previousMillis = currentMillis;
    
    //getting current battery status and send info to USS
    batteryPower=analogRead(3);
    sendData();
    digitalWrite(LED_PIN, LOW);
  }
  
  #if CALC_EXECUTION_TIME==1
    unsigned long stop=micros();
    altitude=(unsigned int)(stop-start);
  #endif
  
}

void checkEnginesX()
{
  for(int i=0; i<4; i++)
  {
    enginesX[i]=min(ENGINEMAXPWM, enginesX[i]);
    enginesX[i]=max(ENGINEMINPWM, enginesX[i]);
  }
}

void generateEnginesSpeeds(double speed_input, double pitch_input, double roll_input, double yaw_input)
{  
  
  //speed settings
  for(int i=0; i<4; i++)
    enginesX[i]=(unsigned int)(speed_input*MAX_SPEED_STEP*gains_Speed[i]+ENGINEMINPWM);
 
  //yaw settettings 
  enginesX[0]+=(unsigned int)(yaw_input*gains_Yaw[0]);
  enginesX[1]-=(unsigned int)(yaw_input*gains_Yaw[1]);
  enginesX[2]+=(unsigned int)(yaw_input*gains_Yaw[2]);
  enginesX[3]-=(unsigned int)(yaw_input*gains_Yaw[3]);
  
  
  //pitch settings
  enginesX[0]+=(unsigned int)(pitch_input*gains_Pitch[0]);
  enginesX[1]+=(unsigned int)(pitch_input*gains_Pitch[1]);
  enginesX[2]-=(unsigned int)(pitch_input*gains_Pitch[2]);
  enginesX[3]-=(unsigned int)(pitch_input*gains_Pitch[3]);
  
  //roll settings
  enginesX[0]-=(unsigned int)(roll_input*gains_Roll[0]);
  enginesX[1]+=(unsigned int)(roll_input*gains_Roll[1]);
  enginesX[2]-=(unsigned int)(roll_input*gains_Roll[2]);
  enginesX[3]+=(unsigned int)(roll_input*gains_Roll[3]);
   
  checkEnginesX();
}


void sendData ()
{
  int value=0;
  uint8_t buffer[maxPacketSize];

  buffer[0] = 0xFF;
  buffer[1] = 0x20;
  
  //engines data
  for (int i=0; i<4; i++)
  {
    buffer[2*i+2]=enginesX[i] & 0xFF;
    buffer[2*i+3]=(enginesX[i] >> 8) & 0xFF;
  }

  //estAngles
  for(int i=0; i<3; i++)
  {
    value=(int)(actual_Angles[i]*10);
    buffer[2*i+10] = value & 0xFF;
    buffer[2*i+11] = (value >> 8) & 0xFF;
  }
 
  //set Angles
  for(int i=0; i<3; i++)
  {
    value=(int)(set_Angles[i]*10);
    buffer[2*i+16] = value & 0xFF;
    buffer[2*i+17] = (value >> 8) & 0xFF;
  }
  
  //controller Angles
  for(int i=0; i<3; i++)
  {
    value=(int)(controller_Angles[i]*10);
    buffer[2*i+22] = value & 0xFF;
    buffer[2*i+23] = (value >> 8) & 0xFF;
  }
  
  //battery power
  buffer[28] = batteryPower & 0xFF;
  buffer[29] = (batteryPower>>8) & 0xFF;
  
  //altitude
  buffer[30] = altitude & 0xFF;
  buffer[31] = (altitude>>8) & 0xFF;
    
  Serial.write(buffer, maxPacketSize);
}

void sendPIDSettings()
{
  uint8_t buffer[maxPacketSize];

  buffer[0] = 0xFF;
  buffer[1] = 0x20;
  int value;
  
  for(int i=0; i<3; i++)
  {
    value=(int)(pid_Pitch_Settings[i]);
    buffer[2*i+2] = value & 0xFF;
    buffer[2*i+3] = (value >> 8) & 0xFF;
  }
  
  for(int i=0; i<3; i++)
  {
    value=(int)(pid_Roll_Settings[i]);
    buffer[2*i+8] = value & 0xFF;
    buffer[2*i+9] = (value >> 8) & 0xFF;
  }
  
  for(int i=0; i<3; i++)
  {
    value=(int)(pid_Yaw_Settings[i]);
    buffer[2*i+14] = value & 0xFF;
    buffer[2*i+15] = (value >> 8) & 0xFF;
  }
  
  Serial.write(buffer, maxPacketSize);
}

void sendMixerSettings()
{
}

void updateEngineParameters()
{
  for(int i=0; i<4; i++)
   enginesX_obj[i].writeMicroseconds(enginesX[i]);
}

void printVibrations()
{
  Serial.print("Vibrations x/y/z:");  
  Serial.print(abs(acc_vector[0]), 2);
  Serial.print("/");
  Serial.print(abs(acc_vector[1]), 2);
  Serial.print("/");
  Serial.print(abs(acc_vector[2]), 2);
  Serial.println();
}
