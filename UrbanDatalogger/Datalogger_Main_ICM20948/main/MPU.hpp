#ifndef MPU_H
#define MPU_H

#include "Arduino.h"
#include <math.h>
#include <ICM20948_WE.h>

extern ICM20948_WE myIMU;
extern volatile float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
extern volatile float angleAccel;
extern volatile float angleFiltered;
extern volatile float alpha;
extern volatile unsigned long lastTimeMPU;

extern float MPUangle;
extern float MPUgyro;

extern float yawOffset;
extern float pitchOffset;
extern float rollOffset;


void executaMPU();

#endif