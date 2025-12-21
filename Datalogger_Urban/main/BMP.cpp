#include "BMP.hpp"


void calculaMedia(float &valorA, float &valorT){

    valoresAltura[indexA] = valorA;
    indexA = (indexA + 1) % sampleSizeBMP;
    valoresTemperatura[indexT] = valorT;
    indexT = (indexT + 1) % sampleSizeBMP;

    mediaA = 0;
    mediaT = 0;
    for (int i = 0; i < sampleSizeBMP; i++) {
      mediaA += valoresAltura[i];
      mediaT += valoresTemperatura[i];
    }
    mediaA /= sampleSizeBMP;
    mediaT /= sampleSizeBMP;
    valorA = mediaA;
    valorT = mediaT;

    Serial.print("Altura: ");
    Serial.println(mediaA);
    Serial.print("Temperatura: ");
    Serial.println(mediaT);
}

void zeraAltura() {
    //DEFINIR ALTURA ZERO (TARA)-------------------
  Serial.println("Aguardando 5 segundos para estabilização do sensor...");
  
  lastTimer = millis();
  
  while(millis() - lastTimer < 5000) {
    bmp.readAltitude(SEALEVELPRESSURE_HPA); 
    delay(50); 
  }
  

  Serial.println("Definindo altura ZERO...");
  float somaAlt = 0;

  for(int i=0; i<20; i++){
    somaAlt += bmp.readAltitude(SEALEVELPRESSURE_HPA);
    delay(100);
  }
  offsetAltura = somaAlt / 20.0;
  Serial.print("Offset definido: "); Serial.println(offsetAltura);
  lastTimer = millis();
}
