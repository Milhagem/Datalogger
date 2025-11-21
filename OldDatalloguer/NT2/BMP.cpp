#include "BMP.hpp"

volatile float valoresAltura[sampleSize];
volatile float valoresTemperatura[sampleSize];
volatile int indexA;
volatile int indexT;
volatile float mediaA;
volatile float mediaT;

void calculaMedia(float &valorA, float &valorT){

    valoresAltura[indexA] = valorA;
    indexA = (indexA + 1) % sampleSize;
    valoresTemperatura[indexT] = valorT;
    indexT = (indexT + 1) % sampleSize;

    mediaA = 0;
    mediaT = 0;
    for (int i = 0; i < sampleSize; i++) {
      mediaA += valoresAltura[i];
      mediaT += valoresTemperatura[i];
    }
    mediaA /= sampleSize;
    mediaT /= sampleSize;
    valorA = mediaA;
    valorT = mediaT;
}