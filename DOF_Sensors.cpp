#include "DOF_Sensors.h"

byte buff_sensors[MAX_SENSORS_DATABYTES];
int gyro_offset_x=0;
int gyro_offset_y=0;
int gyro_offset_z=0;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

double gyro_weight=0.98;

double gyro_angles[3];
double acc_vector[3];
double mag_vector[3];

long mag_offset_x=0;
long mag_offset_y=0;
long mag_offset_z=0;


long lastTime=0;
int interval=0;

int ACC_LPF_VALUE=4;

double estimated_angle_x=0;
double estimated_angle_y=0;
double estimated_angle_z=0;

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
  cbi (PORTC, 4);
  cbi (PORTC, 5);
  TWBR = ((F_CPU / 400000L) - 16)/2;
  Wire.begin();
//  Serial.begin(115200);
  
  init_ADXL345(1); //calibration on each start-up
  init_ITG3200();
  init_HMC5883L();
  lastTime=millis();
}

  double angle1=0;


void getAngles(double *Angles)
{
  double gyro_rate_vector[3];
  read_ADXL345(acc_vector);
  read_ITG3205(gyro_rate_vector);
  read_HMC5883L(mag_vector);
  
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
  
    double angle= atan2((double)mag_vector[0],(double)mag_vector[1]);
    if(angle < 0)
    angle += 2*PI; 
    float headingDegrees = angle * 180/M_PI;
     if (headingDegrees >= 1 && headingDegrees < 240) 
  {
    headingDegrees = map(headingDegrees,0,239,0,179);
  }
  else if (headingDegrees >= 240)
  {
    headingDegrees =  map(headingDegrees,240,360,180,360);
  }
  if (angle1==0) angle1=headingDegrees;
  double gyro_weight2=0.5;
  estimated_angle_z=gyro_weight2*(estimated_angle_z+gyro_rate_vector[2]*(double)interval/1000.0f)+(1-gyro_weight2)*(headingDegrees-angle1);

  Angles[1]=estimated_angle_x;
  Angles[0]=estimated_angle_y;
  Angles[2]=gyro_angles[2];
  Angles[3]=headingDegrees-angle1;
  Angles[4]=estimated_angle_z;
 
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

void read_HMC5883L(double *coords)
{
  readFrom(HMC5883L_DEVICE, HMC5883L_DATA_REGISTER, HMC5883L_DATABYTES, buff_sensors);
  
  coords[0]=(float)(((buff_sensors[0]) << 8) | buff_sensors[1])*1.3-mag_offset_x;
  coords[2]=(float)(((buff_sensors[2]) << 8) | buff_sensors[3])*1.3-mag_offset_z;
  coords[1]=(float)(((buff_sensors[4]) << 8) | buff_sensors[5])*1.3-mag_offset_y;
}

void init_HMC5883L()
{ 
  delay(10); //// added
  writeTo(HMC5883L_DEVICE, 0x00, 0x70); //to increase HZ clock
  writeTo(HMC5883L_DEVICE, 0x01, 0x20); //to increase HZ clock
  writeTo(HMC5883L_DEVICE, 0x02, 0x00);
  
/*  for (int i=0; i<HMC5883L_CALIBRATION_SAMPLE; i++)
  {
    readFrom(HMC5883L_DEVICE, HMC5883L_DATA_REGISTER, HMC5883L_DATABYTES, buff_sensors);
    
      mag_offset_x+=(float)(((buff_sensors[0]) << 8) | buff_sensors[1])*1.3;
      mag_offset_z+=(float)(((buff_sensors[2]) << 8) | buff_sensors[3])*1.3;
      mag_offset_y+=(float)(((buff_sensors[4]) << 8) | buff_sensors[5])*1.3;
    }
    mag_offset_x/=HMC5883L_CALIBRATION_SAMPLE;
    mag_offset_y/=HMC5883L_CALIBRATION_SAMPLE;
    mag_offset_z/=HMC5883L_CALIBRATION_SAMPLE;*/
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

// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
/*
void getADC() {
  
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);
    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[4]<<8) | rawADC[5]) ,
                     ((rawADC[2]<<8) | rawADC[3]) );
}


#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2



void Mag_init() {
  int32_t xyz_total[3]={0,0,0};  // 32 bit totals so they won't overflow.
  bool bret=true;                // Error indicator

  delay(50);  //Wait before start
  writeTo(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias

  // Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
  // The new gain setting is effective from the second measurement and on.

  writeTo(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
  writeTo(MAG_ADDRESS,HMC58X3_R_MODE, 1);
  delay(100);
  getADC();  //Get one sample, and discard it

  for (uint8_t i=0; i<10; i++) { //Collect 10 samples
    i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
    delay(100);
    getADC();   // Get the raw values in case the scales have already been changed.
                
    // Since the measurements are noisy, they should be averaged rather than taking the max.
    xyz_total[0]+=imu.magADC[0];
    xyz_total[1]+=imu.magADC[1];
    xyz_total[2]+=imu.magADC[2];
                
    // Detect saturation.
    if (-(1<<12) >= min(imu.magADC[0],min(imu.magADC[1],imu.magADC[2]))) {
      bret=false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  // Apply the negative bias. (Same gain)
  i2c_writeReg(MAG_ADDRESS,HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
  for (uint8_t i=0; i<10; i++) { 
    i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
    delay(100);
    getADC();  // Get the raw values in case the scales have already been changed.
                
    // Since the measurements are noisy, they should be averaged.
    xyz_total[0]-=imu.magADC[0];
    xyz_total[1]-=imu.magADC[1];
    xyz_total[2]-=imu.magADC[2];

    // Detect saturation.
    if (-(1<<12) >= min(imu.magADC[0],min(imu.magADC[1],imu.magADC[2]))) {
      bret=false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  magGain[0]=fabs(820.0*HMC58X3_X_SELF_TEST_GAUSS*2.0*10.0/xyz_total[0]);
  magGain[1]=fabs(820.0*HMC58X3_Y_SELF_TEST_GAUSS*2.0*10.0/xyz_total[1]);
  magGain[2]=fabs(820.0*HMC58X3_Z_SELF_TEST_GAUSS*2.0*10.0/xyz_total[2]);

  // leave test mode
  i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_CONFA ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
  i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_CONFB ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
  i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_MODE  ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
  delay(100);
  magInit = 1;

  if (!bret) { //Something went wrong so get a best guess
    magGain[0] = 1.0;
    magGain[1] = 1.0;
    magGain[2] = 1.0;
  }
} //  Mag_init().*/
