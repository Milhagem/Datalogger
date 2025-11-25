#ifndef MPU_H
#define MPU_H

#include <Wire.h>
#include <MPU6500_WE.h>
#include "Arduino.h"


// Declaração de que o objeto existe 
extern MPU6500_WE myMPU6500;

// Variáveis globais compartilhadas para armazenar os resultados
extern float angleX;
extern float angleY;
extern float angleZ;
extern float g_accelX;
extern float g_accelY;
extern float g_accelZ;


void executaMPU();

#endif