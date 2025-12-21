#ifndef ICM_H 
#define ICM_H

#include "Arduino.h"
#include <math.h>
#include <ICM20948_WE.h>

#define sampleSizeICM 3
#define ICM20948_ADDR 0x69

extern ICM20948_WE myIMU;
extern unsigned long tprev;

// --- Variáveis Instantâneas (Valor lido na hora) ---
extern float accelX_ICM, accelY_ICM, accelZ_ICM;
extern float angleX_ICM, angleY_ICM, angleZ_ICM; // Z é o integrado

// --- Variáveis para Média Móvel (Histórico e Médias Finais) ---
extern int indexICM;

// Vetores (Arrays) para guardar o histórico
extern volatile float media_accX_ICM[];
extern volatile float media_accY_ICM[];
extern volatile float media_accZ_ICM[];
extern volatile float media_angX_ICM[];
extern volatile float media_angY_ICM[];
extern volatile float media_angZ_ICM[];

// Variáveis Finais (O valor da média calculada)
extern float mediaAccX_ICM, mediaAccY_ICM, mediaAccZ_ICM;
extern float mediaAngX_ICM, mediaAngY_ICM, mediaAngZ_ICM;

extern float pitch, roll;

void executaICM();
void printvaloresICM();
void mediavaloresICM();

#endif