/*#include "DOF_Sensors.h"

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

double mag_coords_gainError[3]={0.94,0.93,0.86};
double mag_coords_offsetError[3]={-54,200,247};
double compass_gain=1.22;


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
  init_HMC5883L(0);
  lastTime=millis();
}

  double angle1=0;

double angle_old=0;
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
  double angle= -atan2((double)mag_vector[1],(double)mag_vector[0])* 180/M_PI+180;
  
  /*  double angle= atan2((double)mag_vector[0],(double)mag_vector[1]);
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
  
  
  if (angle1==0) angle1=angle;
  double gyro_weight2=0.9;
   // if(abs(angle-angle1)>180) angle=angle_old;

  estimated_angle_z=gyro_weight2*(estimated_angle_z+gyro_rate_vector[2]*(double)interval/1000.0f)+(1-gyro_weight2)*(angle-angle1);
  Angles[1]=estimated_angle_x;
  Angles[0]=estimated_angle_y;
  ////////////////////
  ////////////////////
  ////////////////////
  //Angles[2]=gyro_angles[2];
  //Angles[3]=headingDegrees-angle1;
  //Angles[3]=0;
  //Angles[4]=estimated_angle_z;
  Angles[2]=gyro_angles[2];
  Angles[3]=angle-angle1;
  Angles[4]=estimated_angle_z;
 //Angles[2]=mag_vector[0];
 //Angles[3]=mag_vector[1];
 //Angles[4]=mag_vector[2];
 angle_old=angle;
 
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


double buff_sensors_mag[3];
double old_buff_sensors_mag[3];


void read_HMC5883L(double *coords)
{
  //writeTo(HMC5883L_DEVICE, 0x02, 0b10000000); //0b00000000
  
 //  writeTo(HMC5883L_DEVICE, 0x00, 0b01110000); //0b000/100/00
    //writeTo(HMC5883L_DEVICE, 0x01, 0b00100000); //gains 1.3
 //   writeTo(HMC5883L_DEVICE, 0x02, 0b00000001); //0b00000000
  readFrom(HMC5883L_DEVICE, HMC5883L_DATA_REGISTER, HMC5883L_DATABYTES, buff_sensors);
  
 // coords[0]=(float)(((buff_sensors[0]) << 8) | buff_sensors[1])*compass_gain*mag_coords_gainError[0]+mag_coords_offsetError[0];
 // coords[2]=(float)(((buff_sensors[2]) << 8) | buff_sensors[3])*compass_gain*mag_coords_gainError[1]+mag_coords_offsetError[1];
 // coords[1]=(float)(((buff_sensors[4]) << 8) | buff_sensors[5])*compass_gain*mag_coords_gainError[2]+mag_coords_offsetError[2];

//  buff_sensors_mag[0]=(float)(((buff_sensors[0]) << 8) | buff_sensors[1]);
//  buff_sensors_mag[2]=(float)(((buff_sensors[2]) << 8) | buff_sensors[3]);
//  buff_sensors_mag[1]=(float)(((buff_sensors[4]) << 8) | buff_sensors[5]);


  coords[0]=(float)(((buff_sensors[0]) << 8) | buff_sensors[1])*compass_gain*mag_coords_gainError[0]+mag_coords_offsetError[0];
  coords[2]=(float)(((buff_sensors[2]) << 8) | buff_sensors[3])*compass_gain*mag_coords_gainError[1]+mag_coords_offsetError[1];
  coords[1]=(float)(((buff_sensors[4]) << 8) | buff_sensors[5])*compass_gain*mag_coords_gainError[2]+mag_coords_offsetError[1];
  
//  #define MIN_FREQ 100
//  #define MAX_FREQ 4000
  //if (abs(buff_sensors_mag[0])>MAX_FREQ | abs(buff_sensors_mag[0])<MIN_FREQ | abs(buff_sensors_mag[1])>MAX_FREQ | abs(buff_sensors_mag[1])<MIN_FREQ  | abs(buff_sensors_mag[2])>MAX_FREQ | abs(buff_sensors_mag[2])<MIN_FREQ )
//  if(
//  {
//    buff_sensors_mag[0]=old_buff_sensors_mag[0];
//    buff_sensors_mag[1]=old_buff_sensors_mag[1];
//    buff_sensors_mag[2]=old_buff_sensors_mag[2];
//  }
//  else
//  {
//    coords[0]=buff_sensors_mag[0]*compass_gain*mag_coords_gainError[0]+mag_coords_offsetError[0];
 //   coords[2]=buff_sensors_mag[2]*compass_gain*mag_coords_gainError[1]+mag_coords_offsetError[1];
//    coords[1]=buff_sensors_mag[1]*compass_gain*mag_coords_gainError[2]+mag_coords_offsetError[2];   
//  }
//  old_buff_sensors_mag[0]=buff_sensors_mag[0];
//  old_buff_sensors_mag[1]=buff_sensors_mag[1];
//  old_buff_sensors_mag[2]=buff_sensors_mag[2];

}

void read_RawHMC5883L(double *coords)
{
 // writeTo(HMC5883L_DEVICE, 0x02, 0b10000001);
 
  readFrom(HMC5883L_DEVICE, HMC5883L_DATA_REGISTER, HMC5883L_DATABYTES, buff_sensors);
  
  coords[0]=(float)(((buff_sensors[0]) << 8) | buff_sensors[1]);
  coords[2]=(float)(((buff_sensors[2]) << 8) | buff_sensors[3]);
  coords[1]=(float)(((buff_sensors[4]) << 8) | buff_sensors[5]);
}

void init_HMC5883L(int c)
{ 
  double compass_XY_excitation=1160, compass_Z_excitation=1080, compass_cal_x_offset=116, compass_cal_y_offset=225, compass_cal_x_gain=1.1, compass_cal_y_gain=1.12;
  double mag_coords[3];
  double mag_coords_scaled[3]; 
  if(c==1 | c==3)
  {
    //calibrating compass
    writeTo(HMC5883L_DEVICE, 0x00, 0b01110001);
    writeTo(HMC5883L_DEVICE, 0x01, 0b01000000);  



    read_RawHMC5883L(mag_coords);
    while(mag_coords[0]<200 | mag_coords[1]<200 | mag_coords[2]<200)
    {
      Serial.print("Here--- ");
      Serial.print(mag_coords[0]);
      Serial.print("--- ");
      Serial.print(mag_coords[1]);
      Serial.print("--- ");
      Serial.println(mag_coords[2]);
    
      read_RawHMC5883L(mag_coords);
    }
    
    mag_coords_scaled[0]=mag_coords[0]*compass_gain;
    mag_coords_scaled[1]=mag_coords[1]*compass_gain;
    mag_coords_scaled[2]=mag_coords[2]*compass_gain;
  
    mag_coords_gainError[0]=(float)compass_XY_excitation/mag_coords_scaled[0];
    mag_coords_gainError[1]=(float)compass_XY_excitation/mag_coords_scaled[1];
    mag_coords_gainError[2]=(float)compass_Z_excitation/mag_coords_scaled[2];
  
    writeTo(HMC5883L_DEVICE, 0x00, 0b01110010);
  
    read_RawHMC5883L(mag_coords);
    while(mag_coords[0]>-200 | mag_coords[1]>-200 | mag_coords[2]>-200)
    {

      read_RawHMC5883L(mag_coords);
    }
  
    mag_coords_scaled[0]=mag_coords[0]*compass_gain;
    mag_coords_scaled[1]=mag_coords[1]*compass_gain;
    mag_coords_scaled[2]=mag_coords[2]*compass_gain;  
  
    mag_coords_gainError[0]=(float)((compass_XY_excitation/abs(mag_coords_scaled[0]))+mag_coords_gainError[0])/2;
    mag_coords_gainError[1]=(float)((compass_XY_excitation/abs(mag_coords_scaled[1]))+mag_coords_gainError[1])/2;
    mag_coords_gainError[2]=(float)((compass_Z_excitation/abs(mag_coords_scaled[2]))+mag_coords_gainError[2])/2;
  
    Serial.print("x_gain_offset = ");
    Serial.println(mag_coords_gainError[0]);
    Serial.print("y_gain_offset = ");
    Serial.println(mag_coords_gainError[1]);
    Serial.print("z_gain_offset = ");
    Serial.println(mag_coords_gainError[2]);
  
    Serial.println("Waiting 3 sec...");
    delay(3000);
  }
  if(c==2 | c==3)
  {
    ///////////////offset estimation
      writeTo(HMC5883L_DEVICE, 0x00, 0x70); //to increase HZ clock
    writeTo(HMC5883L_DEVICE, 0x01, 0x20); //to increase HZ clock
    writeTo(HMC5883L_DEVICE, 0x02, 0x00);
    
    Serial.println("Please rotate the magnetometer 2 or 3 times in compelete circules within one minute");
  
    for (byte i=0; i<10; i++)
      read_RawHMC5883L(mag_coords);
    
    float x_max=-4000, y_max=-4000, z_max=-4000;
    float x_min=4000, y_min=4000, z_min=4000;
  
    unsigned long t=millis();
    while(millis()-t<=30000)
    {
      read_RawHMC5883L(mag_coords);
      
      Serial.print("Here2--- ");
      Serial.print(mag_coords[0]);
      Serial.print("--- ");
      Serial.print(mag_coords[1]);
      Serial.print("--- ");
      Serial.println(mag_coords[2]);      
    
      mag_coords_scaled[0]=(float)mag_coords[0]*compass_gain*mag_coords_gainError[0];
      mag_coords_scaled[1]=(float)mag_coords[1]*compass_gain*mag_coords_gainError[1];
      mag_coords_scaled[2]=(float)mag_coords[2]*compass_gain*mag_coords_gainError[2];
    
      x_max=max(x_max, mag_coords_scaled[0]);
      y_max=max(y_max, mag_coords_scaled[1]);
      z_max=max(z_max, mag_coords_scaled[2]);
    
      x_min=min(x_min, mag_coords_scaled[0]);
      y_min=min(y_min, mag_coords_scaled[1]);
      z_min=min(z_min, mag_coords_scaled[2]);
    }
  
    
    mag_coords_offsetError[0]=((x_max-x_min)/2)-x_max;
    mag_coords_offsetError[1]=((y_max-y_min)/2)-y_max;
    mag_coords_offsetError[2]=((z_max-z_min)/2)-z_max;
      
    Serial.print("x_offset = ");
    Serial.println(mag_coords_offsetError[0]);
    Serial.print("y_offset = ");
    Serial.println(mag_coords_offsetError[1]);
    Serial.print("z_offset = ");
    Serial.println(mag_coords_offsetError[2]);
  
    Serial.println("Waiting 3 sec...");
    delay(3000);  
  }
  
    writeTo(HMC5883L_DEVICE, 0x00, 0b00010000); //0b000/100/00
    writeTo(HMC5883L_DEVICE, 0x01, 0b00100000); //gains 1.3
    writeTo(HMC5883L_DEVICE, 0x02, 0b10000000); //0b00000000
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
} //  Mag_init().

*/
