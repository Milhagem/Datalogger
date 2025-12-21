#ifndef MPU_H
#define MPU_H

#define MPU6500_ADDR 0x68
#define sampleSizeMPU 3

#include <Wire.h>
#include <MPU6500_WE.h>
#include "Arduino.h"

extern unsigned long t_prev;

extern MPU6500_WE myMPU6500;

// Variáveis Instantâneas
extern float accelX_MPU, accelY_MPU, accelZ_MPU;
extern float angleX_MPU, angleY_MPU, angleZ_MPU;

// Variáveis de Média (Histórico e Final)
extern int indexMPU; 

extern volatile float media_accX_MPU[];
extern volatile float media_accY_MPU[];
extern volatile float media_accZ_MPU[];
extern volatile float media_angX_MPU[];
extern volatile float media_angY_MPU[];
extern volatile float media_angZ_MPU[];

extern float mediaAccX_MPU, mediaAccY_MPU, mediaAccZ_MPU;
extern float mediaAngX_MPU, mediaAngY_MPU, mediaAngZ_MPU;


void executaMPU();
void mediavaloresMPU();
void printvaloresMPU();

#endif