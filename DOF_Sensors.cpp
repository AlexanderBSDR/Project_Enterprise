#include "DOF_Sensors.h"

byte buff_sensors[MAX_SENSORS_DATABYTES];
int gyro_offset_x=0;
int gyro_offset_y=0;
int gyro_offset_z=0;

double gyro_weight=0.98;

double gyro_angles[3];
double acc_vector[3];

long lastTime=0;
int interval=0;

int ACC_LPF_VALUE=4;


double estimated_angle_x=0;
double estimated_angle_y=0;


//byte buff_sensors[MAX_SENSORS_DATABYTES];

//int gyro_offset_x=0;
//int gyro_offset_y=0;
//int gyro_offset_z=0;

//float gyro_weight=0.93;

//float gyro_angles[3];
//long lastTime=0;
//int interval=0;

//float estimated_angle_x=0;
//float estimated_angle_y=0;

void initSensorsStick()
{
  TWBR = ((F_CPU / 400000L) - 16)/2;
  Wire.begin();
  Serial.begin(115200);
  
  init_ADXL345(1); //calibration on each start-up
  init_ITG3200();
  lastTime=millis();
}

void getAngles(double *Angles)
{
  double gyro_rate_vector[3];
  read_ADXL345(acc_vector);
  read_ITG3205(gyro_rate_vector);
  
  long currentTime=millis();
  interval=currentTime-lastTime;
  lastTime=currentTime;
  
  for(int i=0; i<3; i++)
  {
    acc_vector[i]*=ACC_CONV;
    gyro_rate_vector[i]*=GYRO_CONV;
    gyro_angles[i]+=gyro_rate_vector[i]*(double)interval/1000.0f;
  }
           
  //complementary filter
  estimated_angle_x=gyro_weight*(estimated_angle_x+gyro_rate_vector[0]*(double)interval/1000.0f)+(1-gyro_weight)*(atan2(acc_vector[0], acc_vector[2])/PI*180);
  estimated_angle_y=gyro_weight*(estimated_angle_y+gyro_rate_vector[1]*(double)interval/1000.0f)+(1-gyro_weight)*(atan2(acc_vector[1], acc_vector[2])/PI*180);
  
  Angles[0]=estimated_angle_x;
  Angles[1]=estimated_angle_y;
  Angles[2]=gyro_angles[2];
 
}

void read_ITG3205(double *coords)
{
  readFrom(ITG3205_DEVICE, 0x1B, ITG3205_DATABYTES, buff_sensors);
  
 // t=float((buff_sensors[0] << 8) | buff_sensors[1]);
  
  coords[0]=(float)(((buff_sensors[2] << 8) | buff_sensors[3])-gyro_offset_x);
  coords[1]=float(((buff_sensors[4] << 8) | buff_sensors[5])-gyro_offset_y);
  coords[2]=float(((buff_sensors[6] << 8) | buff_sensors[7])-gyro_offset_z);
  
}

void init_ITG3200()
{

//  writeTo(ITG3205_DEVICE, ITG3205_PWR_MGM, 0x00);
//  writeTo(ITG3205_DEVICE, ITG3205_SMPLRT_DIV, 0x0f);
//  writeTo(ITG3205_DEVICE, ITG3205_DLPF_FS, 0x1A); //1e
//  writeTo(ITG3205_DEVICE, ITG3205_INT_CFG, 0x00);

  delay(100); //added
  writeTo(ITG3205_DEVICE, 0x3E, 0x80); //reset a device
  delay(5);
  writeTo(ITG3205_DEVICE, 0x16, ITG3205_INT_CFG);
  delay(5);
  writeTo(ITG3205_DEVICE, 0x3E, 0x03);
  delay(100);
  
  for (int i=0; i<ITG3205_CALIBRATION_SAMPLE; i++)
    {
      readFrom(ITG3205_DEVICE, 0x1B, ITG3205_DATABYTES, buff_sensors);    
      
      gyro_offset_x+=((buff_sensors[2]) << 8) | buff_sensors[3];
      gyro_offset_y+=((buff_sensors[4]) << 8) | buff_sensors[5];
      gyro_offset_z+=((buff_sensors[6]) << 8) | buff_sensors[7];
     // delay(10);
    }
    gyro_offset_x/=ITG3205_CALIBRATION_SAMPLE;
    gyro_offset_y/=ITG3205_CALIBRATION_SAMPLE;
    gyro_offset_z/=ITG3205_CALIBRATION_SAMPLE;
}

void read_ADXL345(double *coords)
{
  readFrom(ADXL345_DEVICE, 0x32, ADXL345_DATABYTES , buff_sensors);
  
  coords[0]=coords[0]*(1-1/2^ACC_LPF_VALUE)+((float)(((buff_sensors[1]) << 8) | buff_sensors[0]))*(1/2^ACC_LPF_VALUE);
  coords[1]=coords[1]*(1-1/2^ACC_LPF_VALUE)+((float)(((buff_sensors[3]) << 8) | buff_sensors[2]))*(1/2^ACC_LPF_VALUE);
  coords[2]=coords[2]*(1-1/2^ACC_LPF_VALUE)+((float)(((buff_sensors[5]) << 8) | buff_sensors[4]))*(1/2^ACC_LPF_VALUE);
  
  //coords[0] = (float)(((buff_sensors[1]) << 8) | buff_sensors[0]);   
  //coords[1] = (float)(((buff_sensors[3])<< 8) | buff_sensors[2]);
  //coords[2] = (float)(((buff_sensors[5]) << 8) | buff_sensors[4]);
}

void init_ADXL345(int c)
{ 
  delay(10); //// added
  writeTo(ADXL345_DEVICE , 0x2D, 0x08); //changed order
  writeTo(ADXL345_DEVICE , 0x31, 0x0B); //changed order
  writeTo(ADXL345_DEVICE, 0X2C, 0x0F); //changed order  
  
  if(c==0)
  {
      writeTo(ADXL345_DEVICE , 0x1E, (byte)(-ADXL345_OFFSET_X/4));
      writeTo(ADXL345_DEVICE , 0x1F, (byte)(-ADXL345_OFFSET_Y/4));
      writeTo(ADXL345_DEVICE , 0x20, (byte)(-((ADXL345_OFFSET_Z)/4)));
  }
  
  else
  {
    long offset_x=0;
    long offset_y=0;
    long offset_z=0;
    
    writeTo(ADXL345_DEVICE , 0x1E, 0);
    writeTo(ADXL345_DEVICE , 0x1F, 0);
    writeTo(ADXL345_DEVICE , 0x20, 0);
    for (int i=0; i<ADXL345_CALIBRATION_SAMPLE; i++)
    {
      readFrom(ADXL345_DEVICE , 0x32, ADXL345_DATABYTES , buff_sensors);
    
      offset_x+=(((int)buff_sensors[1]) << 8) | buff_sensors[0];
      offset_y+=(((int)buff_sensors[3]) << 8) | buff_sensors[2];
      offset_z+=(((int)buff_sensors[5]) << 8) | buff_sensors[4];
    }
    offset_x/=ADXL345_CALIBRATION_SAMPLE;
    offset_y/=ADXL345_CALIBRATION_SAMPLE;
    offset_z/=ADXL345_CALIBRATION_SAMPLE;
  
    writeTo(ADXL345_DEVICE , 0x1E, -(byte)(offset_x/4));
    writeTo(ADXL345_DEVICE , 0x1F, -(byte)((offset_y/4)));
    writeTo(ADXL345_DEVICE , 0x20, -(byte)(((offset_z-256))/4));
  }
}

void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); 
   Wire.write(address);
   Wire.write(val);
   Wire.endTransmission();
}

void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); 
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.beginTransmission(device);
  Wire.requestFrom(device, num);
  
  int i = 0;
  while(Wire.available())
  { 
    buff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
}


