#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>

#include "DOF_Sensors.h"

#define ENGINE_ONE_PIN 5
#define ENGINE_TWO_PIN 6
#define ENGINE_THREE_PIN 10
#define ENGINE_FOUR_PIN 11

#define LED_PIN 13
#define ENGINEMAXPWM 2000
#define ENGINEMINPWM 1000

#define MAX_SPEED_ANGLES 360
#define MAX_PITCH_ANGLES 180
#define MAX_ROLL_ANGLES 180
#define MAX_YAW_ANGLES 180

#define MAX_SPEED_STEP (ENGINEMAXPWM-ENGINEMINPWM)/MAX_SPEED_ANGLES
//#define MAX_PITCH_STEP (ENGINEMAXPWM-ENGINEMINPWM)/MAX_PITCH_ANGLES
//#define MAX_PITCH_STEP
//#define MAX_ROLL_STEP (ENGINEMAXPWM-ENGINEMINPWM)/MAX_ROLL_ANGLES
#define MAX_YAW_STEP (ENGINEMAXPWM-ENGINEMINPWM)/MAX_YAW_ANGLES

unsigned int enginesX[4]={ENGINEMINPWM, ENGINEMINPWM, ENGINEMINPWM, ENGINEMINPWM};
//unsigned int engineOne=ENGINEMINPWM;
//unsigned int engineTwo=ENGINEMINPWM;
//unsigned int engineThree=ENGINEMINPWM;
//unsigned int engineFour=ENGINEMINPWM;

//mixer's settings
//double gains_E1_Speed=1, gains_E2_Speed=1, gains_E3_Speed=1, gains_E4_Speed=1;
//double gains_E1_Pitch=1, gains_E2_Pitch=1, gains_E3_Pitch=1, gains_E4_Pitch=1;
//double gains_E1_Roll=1, gains_E2_Roll=1, gains_E3_Roll=1, gains_E4_Roll=1;

double gains_Speed[4]={1, 1, 1, 1};
double gains_Pitch[4]={1, 1, 1, 1};
double gains_Roll[4]={1, 1, 1, 1};
double gains_Yaw[4]={1, 1, 1, 1};


double pid_Pitch_Settings[3], pid_Roll_Settings[3], pid_Yaw_Settings[3];

//double pid_Pitch_P=0.5, pid_Roll_P=0.5;
//double pid_Pitch_I=0.0, pid_Roll_I=0.0;
//double pid_Pitch_D=0.0, pid_Roll_D=0.0;

unsigned int timerDataUpdate=200;

#define maxPacketSize 32
//[0xFF][0x20]//[2][2][2][2]-engines//[2][2][2]-accell//[2][2][2]-gyro//[2][2][2]-compass//[2]-battery//[2]-height
//2+8+6+6+6+2+2=32

double actual_Angles[4];
double set_Angles[4];
double controller_Angles[4];

unsigned int batteryPower=0;
unsigned long previousMillis = 0;

Servo enginesX_obj[4];

double l;

PID controller_X(&actual_Angles[0], &controller_Angles[0], &set_Angles[0], pid_Roll_Settings[0], pid_Roll_Settings[1], pid_Roll_Settings[2], DIRECT);
PID controller_Y(&actual_Angles[1], &controller_Angles[1], &set_Angles[1], pid_Pitch_Settings[0], pid_Pitch_Settings[1], pid_Pitch_Settings[2], DIRECT);

void setup() {
  initialize_Enterprise();
  initSensorsStick();

  enginesX_obj[0].attach(ENGINE_ONE_PIN);
  //engine1.writeMicroseconds(engineOne);
  
  enginesX_obj[1].attach(ENGINE_TWO_PIN);
  //engine2.writeMicroseconds(engineTwo);
  
  enginesX_obj[2].attach(ENGINE_THREE_PIN);
  //engine3.writeMicroseconds(engineThree);
  
  enginesX_obj[3].attach(ENGINE_FOUR_PIN);
  //engine4.writeMicroseconds(engineFour);

//  controller_X.SetMode(AUTOMATIC);
//  controller_Y.SetMode(AUTOMATIC); 
  controller_X.SetOutputLimits(-ENGINEMINPWM/2, ENGINEMINPWM/2);
  controller_Y.SetOutputLimits(-ENGINEMINPWM/2, ENGINEMINPWM/2);
  controller_X.SetSampleTime(10);
  controller_Y.SetSampleTime(10);

  Serial.begin(115200);
  Serial.write("Enterprise is at your commands!\n");
  
}

void loop() {
  
  digitalWrite(LED_PIN, HIGH);
  
  //#1 - get current parameters (actual_Angles)
  getAngles(actual_Angles);
  //#2 - calculate if we need to make any adjustments (controller_Angles)
  controller_X.Compute(); // roll
  controller_Y.Compute(); //pitch
  //#3 - do adjustments to engines
  generateEnginesSpeeds(set_Angles[4], controller_Angles[1], controller_Angles[0]);
  updateEngineParameters();
  
  //#4 - check if we have new commands (set_Angles) 
  while (Serial.available() > 0)
  {
    delay(1); //let's delete it later!
    while (Serial.read() == 0xFF)
    {
      double value = 0;
      char str[maxPacketSize];
   
      switch (Serial.read())
      {
        case 0x01:
          for(int i=0; i<4; i++)
            enginesX[i] = (unsigned int)((Serial.read())|(Serial.read()<<8));
            
          updateEngineParameters();

          Serial.print("Engines:   ");
          for (int i=0; i<4; i++)
          {
            Serial.print(enginesX[i]);
            Serial.print("/");
          }
          Serial.println();

          break;

        case 0x02:
          value = (double)((int)(((Serial.read())|(Serial.read()<<8))));
          set_Angles[4]=value;
          
          sprintf(str, "S_C: %26d\n", (int)value); /////////////////////////////
          Serial.write(str);
          break;

        case 0x03: 
          value = (double)((int)(((Serial.read())|(Serial.read()<<8))));
          set_Angles[1]+=value;

          sprintf(str, "P_C: %26d\n", (int)value); /////////////////////////////
          Serial.write(str);
          break;

        case 0x04:
          value = (double)((int)(((Serial.read())|(Serial.read()<<8))));
          set_Angles[0]+=value;

          sprintf(str, "R_C: %26d\n", (int)value); /////////////////////////////
          Serial.write(str);
          break;
          
          case 0x05: ///speed mixer settings
          for(int i=0; i<4; i++)
            gains_Speed[i] = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
//          gains_E2_Speed = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
//          gains_E3_Speed = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
//          gains_E4_Speed = (double)((int)(Serial.read())|(Serial.read()<<8))/100;

          //sprintf(str, "S: [%2.2f][%2.2f][%2.2f][%2.2f]\n", gains_E1_Speed, gains_E2_Speed, gains_E3_Speed, gains_E4_Speed); /////////////////////////////
          Serial.print("Speed:     ");
          for (int i=0; i<4; i++)
          {
            Serial.print(gains_Speed[i], 2);
            Serial.print("/");
          }
          Serial.println();
          break;
          
          case 0x06: ///pitch mixer settings
          for(int i=0; i<4; i++)
            gains_Pitch[i] = (double)((int)(Serial.read())|(Serial.read()<<8))/100;

 //         sprintf(str, "S: [%2.2f][%2.2f][%2.2f][%2.2f]\n", gains_E1_Pitch, gains_E2_Pitch, gains_E3_Pitch, gains_E4_Pitch); /////////////////////////////
 //         Serial.write(str);
          Serial.print("Pitch:     ");
          for (int i=0; i<4; i++)
          {
            Serial.print(gains_Pitch[i], 2);
            Serial.print("/");
          }
          Serial.println();
          break;
          
          
          case 0x07: ///roll mixer settings
          for(int i=0; i<4; i++)
            gains_Roll[i] = (double)((int)(Serial.read())|(Serial.read()<<8))/100;

     //     sprintf(str, "S: [%2.2f][%2.2f][%2.2f][%2.2f]\n", gains_E1_Roll, gains_E2_Roll, gains_E3_Roll, gains_E4_Roll); /////////////////////////////
     //     Serial.write(str);
          Serial.print("Roll:      ");
          for (int i=0; i<4; i++)
          {
            Serial.print(gains_Roll[i], 2);
            Serial.print("/");
          }
          Serial.println();
          break;
          
          case 0x08: ///pitch mixer settings
          for(int i=0; i<3; i++)
            pid_Pitch_Settings[i] = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
          //pid_Pitch_D = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
          //pid_Pitch_I = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
          for(int i=0; i<3; i++)
            pid_Roll_Settings[i] = (double)((int)(Serial.read())|(Serial.read()<<8))/100;

//          pid_Roll_P = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
//          pid_Roll_D = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
//          pid_Roll_I = (double)((int)(Serial.read())|(Serial.read()<<8))/100;

 //         sprintf(str, "S: [%2.2f][%2.2f][%2.2f][%2.2f]\n", gains_E1_Pitch, gains_E2_Pitch, gains_E3_Pitch, gains_E4_Pitch); /////////////////////////////
 //         Serial.write(str);
          Serial.print("PP P/I/D: [");
          Serial.print(pid_Pitch_Settings[0]);
          Serial.print("][");
          Serial.print(pid_Pitch_Settings[1]);
          Serial.print("][");
          Serial.print(pid_Pitch_Settings[2]);
          Serial.println("]");
          
          Serial.print("RL P/I/D: [");
          Serial.print(pid_Roll_Settings[0]);
          Serial.print("][");
          Serial.print(pid_Roll_Settings[1]);
          Serial.print("][");
          Serial.print(pid_Roll_Settings[2]);
          Serial.println("]");
          
          flash_PID_Settings();
          
          controller_Y.SetTunings(pid_Pitch_Settings[0], pid_Pitch_Settings[1], pid_Pitch_Settings[2]);
          controller_X.SetTunings(pid_Roll_Settings[0], pid_Roll_Settings[1], pid_Roll_Settings[2]);
          
          break;

          case 0x09:
          timerDataUpdate=(unsigned int)((Serial.read())|(Serial.read()<<8));
          break; 
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
}

void checkEnginesX()
{
  for(int i=0; i<4; i++)
  {
    enginesX[i]=min(ENGINEMAXPWM, enginesX[i]);
    enginesX[i]=max(ENGINEMINPWM, enginesX[i]);
  }
}

void generateEnginesSpeeds(double speed_input, double pitch_input, double roll_input)
{
  //speed settings
  for(int i=0; i<4; i++)
    enginesX[i]=speed_input*MAX_SPEED_STEP*gains_Speed[i]+ENGINEMINPWM;

  
  //pitch settings
  enginesX[0]+=pitch_input*gains_Pitch[0];
  enginesX[1]+=pitch_input*gains_Pitch[1];
  enginesX[2]-=pitch_input*gains_Pitch[2];
  enginesX[3]-=pitch_input*gains_Pitch[3];
  
  //roll settings
  enginesX[0]-=roll_input*gains_Roll[0];
  enginesX[1]+=roll_input*gains_Roll[1];
  enginesX[2]-=roll_input*gains_Roll[2];
  enginesX[3]+=roll_input*gains_Roll[3];
  
  checkEnginesX();  
}


void sendData ()
{
  int value;
  int altitude=20000;
  uint8_t buffer[maxPacketSize];

  buffer[0] = 0xFF;
  buffer[1] = 0x20;
  
  //engines data
  for (int i=0; i<4; i++)
  {
    buffer[2*i+2]=enginesX[i] & 0xFF;
    buffer[2*i+3]=(enginesX[i] >> 8) & 0xFF;
  }
 // buffer[2]=engineOne & 0xFF;
 // buffer[3]=(engineOne>>8) & 0xFF;
 // buffer[4]=engineTwo & 0xFF;
 // buffer[5]=(engineTwo>>8) & 0xFF;
 // buffer[6]=engineThree & 0xFF;
 // buffer[7]=(engineThree>>8) & 0xFF;
 // buffer[8]=engineFour & 0xFF;
 // buffer[9]=(engineFour>>8) & 0xFF;

  //estAngles
  for(int i=0; i<3; i++)
  {
    value=(int)(actual_Angles[i]*10);
    buffer[2*i+10] = value & 0xFF;
    buffer[2*i+11] = (value >> 8) & 0xFF;
  }
    
//  x=(int)(actual_Angles[0]*10);
//  y=(int)(actual_Angles[1]*10);
//  z=(int)(actual_Angles[2]*10);
  
//  buffer[10] = x & 0xFF;
//  buffer[11] = (x>>8) & 0xFF;
//  buffer[12] = y & 0xFF;
//  buffer[13] = (y>>8) & 0xFF;
//  buffer[14] = z & 0xFF;
//  buffer[15] = (z>>8) & 0xFF;
 
  //set Angles
  for(int i=0; i<3; i++)
  {
    value=(int)(set_Angles[i]*10);
    buffer[2*i+16] = value & 0xFF;
    buffer[2*i+17] = (value >> 8) & 0xFF;
  }
//  x=(int)(set_Angles[0]*10);
//  y=(int)(set_Angles[1]*10);
//  z=(int)(set_Angles[2]*10);
  
//  buffer[16] = x & 0xFF;
//  buffer[17] = (x>>8) & 0xFF;
//  buffer[18] = y & 0xFF;
//  buffer[19] = (y>>8) & 0xFF;
//  buffer[20] = z & 0xFF;
//  buffer[21] = (z>>8) & 0xFF;
  
  //controller Angles
  for(int i=0; i<3; i++)
  {
    value=(int)(controller_Angles[i]);
    buffer[2*i+22] = value & 0xFF;
    buffer[2*i+23] = (value >> 8) & 0xFF;
  }
//  x=(int)(controller_Angles[0]);
//  y=(int)(controller_Angles[1]);
//  z=(int)(controller_Angles[2]);
  
//  buffer[22] = x & 0xFF;
//  buffer[23] = (x>>8) & 0xFF;
//  buffer[24] = y & 0xFF;
//  buffer[25] = (y>>8) & 0xFF;
//  buffer[26] = z & 0xFF;
//  buffer[27] = (z>>8) & 0xFF;
  
  //battery power
  buffer[28] = batteryPower & 0xFF;
  buffer[29] = (batteryPower>>8) & 0xFF;
  
  //altitude
  buffer[30] = altitude & 0xFF;
  buffer[31] = (altitude>>8) & 0xFF;
    
  Serial.write(buffer, maxPacketSize);
}

void updateEngineParameters()
{
  for(int i=0; i<4; i++)
   enginesX_obj[i].writeMicroseconds(enginesX[i]);
}

///// TO BE DELETED

/*void controlMixerPitch(double ang)
{ 
  engineOne+=ang*gains_E1_Pitch;
  engineTwo+=ang*gains_E2_Pitch;
  
  engineThree-=ang*gains_E3_Pitch;
  engineFour-=ang*gains_E4_Pitch;
  
  checkEnginesX();
  updateEngineParameters();

}

void controlMixerRoll(double ang)
{
  engineOne-=ang*gains_E1_Roll;
  engineTwo+=ang*gains_E2_Roll;
  engineThree-=ang*gains_E3_Roll;
  engineFour+=ang*gains_E4_Roll;
  
  checkEnginesX();
  updateEngineParameters();
}

void controlMixerSpeed(double ang)
{
  engineOne=ang*MAX_SPEED_STEP*gains_E1_Speed+ENGINEMINPWM;
  engineTwo=ang*MAX_SPEED_STEP*gains_E2_Speed+ENGINEMINPWM;
  engineThree=ang*MAX_SPEED_STEP*gains_E3_Speed+ENGINEMINPWM;
  engineFour=ang*MAX_SPEED_STEP*gains_E4_Speed+ENGINEMINPWM;

  checkEnginesX();
  updateEngineParameters();
}


void setSpeed(double ang)
{
  set_Angles[4]=ang;
  //controlMixerSpeed(ang);
}

void setPitch(double ang)
{
 // double temp=set_Angles[1];

  
  //controlMixerPitch(ang);
  //controller_Y.Compute();
  //controlMixerPitch(set_Angles[1]-temp);
}

void setRoll(double ang)
{
  //double temp=set_Angles[0];
  set_Angles[0]+=ang;
  //controlMixerRoll(ang);
  //controller_X.Compute();
  //controlMixerRoll(set_Angles[0]-temp);
}
*/
