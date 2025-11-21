#ifndef MPU_H
#define MPU_H

#include "Arduino.h"
#include <math.h>
#include "MPU9250.h"


extern MPU9250 mpu;

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

extern volatile float relativeYaw;
extern volatile float relativeRoll;
extern volatile float relativePitch;

void executaMPU();
void print_calibration();
void taravaloresiniciais(); // Sua nova função

#endif