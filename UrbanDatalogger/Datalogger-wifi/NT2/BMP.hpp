#ifndef BMP_H
#define BMP_H

#include "Arduino.h"
#include "Adafruit_BMP3XX.h"

#define sampleSizeBMP 7

//Presão do mar
#define SEALEVELPRESSURE_HPA (1015.0)

extern Adafruit_BMP3XX bmp;
extern volatile float valoresAltura[];
extern volatile float valoresTemperatura[];
extern volatile int indexA;
extern volatile int indexT;
extern volatile float mediaA;
extern volatile float mediaT;
extern float offsetAltura;
extern unsigned long lastTimer;

//Função para calulalar a media movel variação de altura e da temperatura na traseira do veiculo
void calculaMedia(float &valorA, float &valorT);
void zeraAltura();

#endif