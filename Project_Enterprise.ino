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
#define MAX_PITCH_STEP (ENGINEMAXPWM-ENGINEMINPWM)/MAX_PITCH_ANGLES
#define MAX_ROLL_STEP (ENGINEMAXPWM-ENGINEMINPWM)/MAX_ROLL_ANGLES
#define MAX_YAW_STEP (ENGINEMAXPWM-ENGINEMINPWM)/MAX_YAW_ANGLES

unsigned int engineOne=ENGINEMINPWM;
unsigned int engineTwo=ENGINEMINPWM;
unsigned int engineThree=ENGINEMINPWM;
unsigned int engineFour=ENGINEMINPWM;

//mixer's settings
double gains_E1_Speed=1, gains_E2_Speed=1, gains_E3_Speed=1, gains_E4_Speed=1;
double gains_E1_Pitch=1, gains_E2_Pitch=1, gains_E3_Pitch=1, gains_E4_Pitch=1;
double gains_E1_Roll=1, gains_E2_Roll=1, gains_E3_Roll=1, gains_E4_Roll=1;

unsigned int timerDataUpdate=200;

#define maxPacketSize 32
//[0xFF][0x20]//[2][2][2][2]-engines//[2][2][2]-accell//[2][2][2]-gyro//[2][2][2]-compass//[2]-battery//[2]-height
//2+8+6+6+6+2+2=32

double actual_Angles[4];
double set_Angles[4];
double controller_Angles[4];

unsigned int batteryPower=0;
unsigned long previousMillis = 0;

Servo engine1, engine2, engine3, engine4;

double l;

PID controller_X(&actual_Angles[0], &controller_Angles[0], &set_Angles[0], 1, 0, 0, DIRECT);
PID controller_Y(&actual_Angles[1], &controller_Angles[1], &set_Angles[1], 1, 0, 0, DIRECT);

void setup() {
  
  initSensorsStick();

  engine1.attach(ENGINE_ONE_PIN);
  engine1.writeMicroseconds(engineOne);
  
  engine2.attach(ENGINE_TWO_PIN);
  engine2.writeMicroseconds(engineTwo);
  
  engine3.attach(ENGINE_THREE_PIN);
  engine3.writeMicroseconds(engineThree);
  
  engine4.attach(ENGINE_FOUR_PIN);
  engine4.writeMicroseconds(engineFour);

  controller_X.SetMode(AUTOMATIC);
  controller_Y.SetMode(AUTOMATIC); 
  controller_X.SetOutputLimits(-180, 180);
  controller_Y.SetOutputLimits(-180, 180);
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
  controller_X.Compute();
  controller_Y.Compute();
  
  //#3 - do adjustments to engines
  controlMixerPitch(controller_Angles[1]);
  controlMixerRoll(controller_Angles[0]);
  
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
          engineOne = (unsigned int)((Serial.read())|(Serial.read()<<8));
          engineTwo = (unsigned int)((Serial.read())|(Serial.read()<<8));
          engineThree = (unsigned int)((Serial.read())|(Serial.read()<<8));
          engineFour = (unsigned int)((Serial.read())|(Serial.read()<<8));

          updateEngineParameters();

          sprintf(str, "[%5d][%5d][%5d][%5d]   \n", engineOne,engineTwo,engineThree,engineFour); /////////////////////////////
          Serial.write(str);

          break;

        case 0x02:
          value = (double)((int)(((Serial.read())|(Serial.read()<<8))));
          setSpeed(value);

          sprintf(str, "S_C: %26d\n", (int)value); /////////////////////////////
          Serial.write(str);
          break;

        case 0x03: 
          value = (double)((int)(((Serial.read())|(Serial.read()<<8))));
          setPitch(value);

          sprintf(str, "P_C: %26d\n", (int)value); /////////////////////////////
          Serial.write(str);
          break;

        case 0x04:
          value = (double)((int)(((Serial.read())|(Serial.read()<<8))));
          setRoll(value);

          sprintf(str, "R_C: %26d\n", (int)value); /////////////////////////////
          Serial.write(str);
          break;
          
          case 0x05: ///speed mixer settings
                    
                    
          gains_E1_Speed = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
          gains_E2_Speed = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
          gains_E3_Speed = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
          gains_E4_Speed = (double)((int)(Serial.read())|(Serial.read()<<8))/100;

          sprintf(str, "S: [%2.2f][%2.2f][%2.2f][%2.2f]\n", gains_E1_Speed, gains_E2_Speed, gains_E3_Speed, gains_E4_Speed); /////////////////////////////
          Serial.print("S: [ ");
          Serial.print(gains_E1_Speed);
          Serial.print("][ ");
          Serial.print(gains_E2_Speed);
          Serial.print("][ ");
          Serial.print(gains_E3_Speed);
          Serial.print("][ ");
          Serial.print(gains_E4_Speed);
          Serial.println("]");
          break;
          
          case 0x06: ///pitch mixer settings
          gains_E1_Pitch = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
          gains_E2_Pitch = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
          gains_E3_Pitch = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
          gains_E4_Pitch = (double)((int)(Serial.read())|(Serial.read()<<8))/100;

 //         sprintf(str, "S: [%2.2f][%2.2f][%2.2f][%2.2f]\n", gains_E1_Pitch, gains_E2_Pitch, gains_E3_Pitch, gains_E4_Pitch); /////////////////////////////
 //         Serial.write(str);
          Serial.print("P: [ ");
          Serial.print(gains_E1_Pitch);
          Serial.print("][ ");
          Serial.print(gains_E2_Pitch);
          Serial.print("][ ");
          Serial.print(gains_E3_Pitch);
          Serial.print("][ ");
          Serial.print(gains_E4_Pitch);
          Serial.println("]");
          break;
          
          
          case 0x07: ///roll mixer settings
          gains_E1_Roll = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
          gains_E2_Roll = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
          gains_E3_Roll = (double)((int)(Serial.read())|(Serial.read()<<8))/100;
          gains_E4_Roll = (double)((int)(Serial.read())|(Serial.read()<<8))/100;

     //     sprintf(str, "S: [%2.2f][%2.2f][%2.2f][%2.2f]\n", gains_E1_Roll, gains_E2_Roll, gains_E3_Roll, gains_E4_Roll); /////////////////////////////
     //     Serial.write(str);
          Serial.print("R: [ ");
          Serial.print(gains_E1_Roll);
          Serial.print("][ ");
          Serial.print(gains_E2_Roll);
          Serial.print("][ ");
          Serial.print(gains_E3_Roll);
          Serial.print("][ ");
          Serial.print(gains_E4_Roll);
          Serial.println("]");
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

void setSpeed(double ang)
{
  set_Angles[4]=ang;
  controlMixerSpeed(ang);
}

void setPitch(double ang)
{
  set_Angles[0]=ang;
  controlMixerPitch(ang);
}

void setRoll(double ang)
{
  set_Angles[2]=ang;
  controlMixerRoll(ang);
}

void controlMixerSpeed(double ang)
{
  engineOne+=ang*MAX_SPEED_STEP*gains_E1_Speed;
  engineTwo+=ang*MAX_SPEED_STEP*gains_E2_Speed;
  engineThree+=ang*MAX_SPEED_STEP*gains_E3_Speed;
  engineFour+=ang*MAX_SPEED_STEP*gains_E4_Speed;
  
  checkEnginesX();
  updateEngineParameters();
}

void checkEnginesX()
{
  engineOne=min(ENGINEMAXPWM, engineOne);
  engineTwo=min(ENGINEMAXPWM, engineTwo);
  engineThree=min(ENGINEMAXPWM, engineThree);
  engineFour=min(ENGINEMAXPWM, engineFour);
  
  engineOne=max(ENGINEMINPWM, engineOne);
  engineTwo=max(ENGINEMINPWM, engineTwo);
  engineThree=max(ENGINEMINPWM, engineThree);
  engineFour=max(ENGINEMINPWM, engineFour);  
}

void controlMixerPitch(double ang)
{ 
  engineOne+=ang*MAX_PITCH_STEP*gains_E1_Pitch;
  engineTwo+=ang*MAX_PITCH_STEP*gains_E2_Pitch;
  
  engineThree-=ang*MAX_PITCH_STEP*gains_E3_Pitch;
  engineFour-=ang*MAX_PITCH_STEP*gains_E4_Pitch;
  
  checkEnginesX();
  updateEngineParameters();

}

void controlMixerRoll(double ang)
{
  engineTwo+=ang*MAX_ROLL_STEP*gains_E2_Roll;
  engineFour+=ang*MAX_ROLL_STEP*gains_E4_Roll;
  
  engineOne-=ang*MAX_ROLL_STEP*gains_E1_Roll;
  engineThree-=ang*MAX_ROLL_STEP*gains_E3_Roll;
  
  checkEnginesX();
  updateEngineParameters();
}

void sendData ()
{
  int x, y, z;
  int altitude=20000;
  uint8_t buffer[maxPacketSize];

  buffer[0] = 0xFF;
  buffer[1] = 0x20;
  
  //engines data
  buffer[2]=engineOne & 0xFF;
  buffer[3]=(engineOne>>8) & 0xFF;
  buffer[4]=engineTwo & 0xFF;
  buffer[5]=(engineTwo>>8) & 0xFF;
  buffer[6]=engineThree & 0xFF;
  buffer[7]=(engineThree>>8) & 0xFF;
  buffer[8]=engineFour & 0xFF;
  buffer[9]=(engineFour>>8) & 0xFF;

  //estAngles
  x=(int)(actual_Angles[0]*10);
  y=(int)(actual_Angles[1]*10);
  z=(int)(actual_Angles[2]*10);
  
  buffer[10] = x & 0xFF;
  buffer[11] = (x>>8) & 0xFF;
  buffer[12] = y & 0xFF;
  buffer[13] = (y>>8) & 0xFF;
  buffer[14] = z & 0xFF;
  buffer[15] = (z>>8) & 0xFF;
 
  //set Angles
  x=(int)(set_Angles[0]*10);
  y=(int)(set_Angles[1]*10);
  z=(int)(set_Angles[2]*10);
  
  buffer[16] = x & 0xFF;
  buffer[17] = (x>>8) & 0xFF;
  buffer[18] = y & 0xFF;
  buffer[19] = (y>>8) & 0xFF;
  buffer[20] = z & 0xFF;
  buffer[21] = (z>>8) & 0xFF;
  
  //controller Angles
  x=(int)(controller_Angles[0]*10);
  y=(int)(controller_Angles[1]*10);
  z=(int)(controller_Angles[2]*10);
  
  buffer[22] = x & 0xFF;
  buffer[23] = (x>>8) & 0xFF;
  buffer[24] = y & 0xFF;
  buffer[25] = (y>>8) & 0xFF;
  buffer[26] = z & 0xFF;
  buffer[27] = (z>>8) & 0xFF;
  
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
   engine1.writeMicroseconds(engineOne);
   engine2.writeMicroseconds(engineTwo);
   engine3.writeMicroseconds(engineThree);
   engine4.writeMicroseconds(engineFour);
}

