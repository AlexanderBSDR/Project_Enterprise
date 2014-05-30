#include "Arduino.h"
#include <Wire.h>

#ifndef DOF_Sensors_h
#define DOF_Sensors_h

#define ADXL345_DEVICE (0x53)
#define ADXL345_DATABYTES (6) 
#define MAX_SENSORS_DATABYTES 8 
#define ADXL345_CALIBRATION_SAMPLE 100
#define HMC5883L_CALIBRATION_SAMPLE 5


#define ADXL345_OFFSET_X -131
#define ADXL345_OFFSET_Y 28
#define ADXL345_OFFSET_Z 433

#define ITG3205_DEVICE (0x68)
#define ITG3205_SMPLRT_DIV 0x00 //old value - 0x15
#define ITG3205_DLPF_FS 0x16
#define ITG3205_INT_CFG 0x18 //old value 0x17
#define ITG3205_PWR_MGM 0x3E
#define ITG3205_DATABYTES 8

#define ITG3205_CALIBRATION_SAMPLE 100

#define ITG3205_OFFSET_X 0
#define ITG3205_OFFSET_Y 0
#define ITG3205_OFFSET_Z 0

#define ACC_CONV 0.0039
#define GYRO_CONV 1/14.375

#define HMC5883L_DEVICE 0x1E
#define HMC5883L_DATA_REGISTER 0x03
#define HMC5883L_DATABYTES 6

extern double estimated_angle_x;
extern double estimated_angle_y;

void initSensorsStick();


void getAngles(double *Angles);

void printAngles(double *angles);
void read_ITG3205(double *coords);

void init_ITG3200();

void read_ADXL345(double *coords);
void read_HMC5883L(double *coords);


void init_ADXL345(int c);
void init_HMC5883L(int c);

void writeTo(int device, byte address, byte val);

void readFrom(int device, byte address, int num, byte buff[]);

#endif

