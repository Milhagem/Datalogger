#include "Velocidade.hpp"


//Velocidade Da Roda ------------------------------------------------
volatile unsigned long pulseInterval;     // ms
volatile unsigned long lastPulseInterval; // ms
volatile unsigned long pulseIntervals[sampleSize];
volatile int pulseIndex;
volatile unsigned long lastTime;          // ms
volatile unsigned long timeAjust;         // ms
volatile unsigned long timerCalcVel;      // ms
volatile unsigned long lastTimerCalcVel;  // ms
volatile unsigned long timeEstabilizaVel;
volatile float velocidade;
unsigned long lastTimerTax;  // ms

//--------------------------------------------------------------------

//Velocidade Pino M --------------------------------------------------

volatile unsigned long pulseIntervalM;     // ms
volatile unsigned long lastPulseIntervalM; // ms
volatile unsigned long pulseIntervalsM[sampleSize];
volatile int pulseIndexM;
volatile float velocidadeM;
unsigned long lastTimerTaxM;  // ms

//--------------------------------------------------------------------

void calculaVelocidade(float &veloc) {

  if(millis() - lastTimerTax >= taxaAtualizacaoVel) {
    lastTimerTax = millis();

    detachInterrupt(digitalPinToInterrupt(pinSensorHall));
    unsigned long averagePulseInterval = 0;
    // filtro media movel -----------------------------------
    for (int i = 0; i < sampleSize; i++) {
      averagePulseInterval += pulseIntervals[i];
    }
    averagePulseInterval /= sampleSize;
    //-------------------------------------------------------

    //veloc = circunfRoda/(pulsosPorVolta*averagePulseInterval) * MS_to_S * MPS_to_KMPH;
    //float velocOld = veloc;
    float rpm = 0;
    if (averagePulseInterval > 0) {
      rpm = (60 * 1000) / (pulsosPorVolta * averagePulseInterval);
    } else {
      rpm = 0;
    }
    veloc = rpm;

    //veloc = filtroVelocVariacoesGrandes(velocOld, velocNew);

    attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
    Serial.print("veloc:");
  }
}

void calculaVelocidadeM(float &veloc) {

  if(millis() - lastTimerTaxM >= taxaAtualizacaoVel) {
    lastTimerTaxM = millis();

    detachInterrupt(digitalPinToInterrupt(pinSensorM));
    unsigned long averagePulseIntervalM = 0;
    // filtro media movel -----------------------------------
    for (int i = 0; i < sampleSize; i++) {
      averagePulseIntervalM += pulseIntervalsM[i];
    }
    averagePulseIntervalM /= sampleSize;
    //-------------------------------------------------------

    //veloc = (5.0 / averagePulseIntervalM) * (1.8*60*3.15 / 105);
    unsigned long rpm = 0;
    if (averagePulseIntervalM > 0) {
      rpm = (4.16 * 1000) / averagePulseIntervalM;  // Cálculo do RPM
    } else {
      rpm = 0;  // Evita divisão por zero
    }
    veloc = rpm;

    attachInterrupt(digitalPinToInterrupt(pinSensorM), calcM, RISING);
    Serial.print("veloc:");
  }
}


void calc() {
  pulseInterval = millis() - lastPulseInterval;
  lastPulseInterval = millis();
  pulseIntervals[pulseIndex] = pulseInterval;
  pulseIndex = (pulseIndex + 1) % sampleSize; 
}

void calcM() {
  pulseIntervalM = millis() - lastPulseIntervalM;
  lastPulseIntervalM = millis();
  pulseIntervalsM[pulseIndexM] = pulseIntervalM;
  pulseIndexM = (pulseIndexM + 1) % sampleSize; 
}

