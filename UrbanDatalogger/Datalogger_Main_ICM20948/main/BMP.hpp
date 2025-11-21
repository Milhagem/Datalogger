#ifndef BMP_H
#define BMP_H

#include "Arduino.h"

#define sampleSize 3


//Função para calulalar a media movel variação de altura e da temperatura na traseira do veiculo
void calculaMedia(float &valorA, float &valorT);

#endif