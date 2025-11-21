#include "Adafruit_BMP3XX.h"
#include "BMP.hpp"
#include "MPU.hpp"
#include <SoftwareSerial.h>

const int GPS_RX_PIN = 14;  // Pino conectado ao GPS TX
const int GPS_TX_PIN = 12;  // Pino conectado ao GPS RX

Adafruit_BMP3XX bmp;
//BMP -----------------------------------------------------------------------
#define SEALEVELPRESSURE_HPA (1015.0) // Atualize diariamente!
float groundLevelPressureHPA;

int PWR_PIN = 5;
int SDA_PIN = 2;
int SCL_PIN = 1;

//---------------------------------------------------------------------------
//Variaveis para analise ----------------------------------------------------
float BMPaltura   = 0;
float BMPtemp     = 0;

//--------------------------------------------------------------------------
//Variavel de tempo --------------------------------------------------------
unsigned long lastTimer;

//--------------------------------------------------------------------------
//-----------------------------------------------------------------------

//Variaveis Referentes ao BMP -------------------------------------------
extern volatile float valoresAltura[sampleSize];
extern volatile float valoresTemperatura[sampleSize];
extern volatile int indexA;
extern volatile int indexT;
extern volatile float mediaA;
extern volatile float mediaT;
//-----------------------------------------------------------------------
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(2000);

  gpsSerial.begin(9600);
  //---------------------IMU configs-------------------
  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
  }
  else{
    Serial.println("ICM20948 is connected");
  }

  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
  Serial.println("Done!"); 
  
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);    
  myIMU.setAccSampleRateDivider(10);

  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);  
  
  //----------------------Zera vetores de medias moveis---------
  for(int i = 0; i < sampleSize; i++) {valoresAltura[i] = 0;}
  for(int i = 0; i < sampleSize; i++) {valoresTemperatura[i] = 0;}


  //-----------------------Inicializa BMP-----------------------
  if (!bmp.begin_I2C(0x77)) {
    Serial.println("Sensor não encontrado!");
    Serial.println("BMP F");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  delay(100);
  groundLevelPressureHPA = bmp.readPressure() / 100.0F;
}


void loop() {
  if(millis() - lastTimer >= 500){
    lastTimer = millis();
    
    //Execução BMP
    BMPaltura = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    BMPtemp = bmp.temperature;
    calculaMedia(BMPaltura, BMPtemp);

    executaMPU();
  }

  while((millis() % 1000)){
    if (gpsSerial.available()) {
    // Read the available data
    char gps_reading = gpsSerial.read();
    
    // Pass it to the Serial Monitor
    Serial.write(gps_reading);
    }
    //delay(1000);
  }
  Serial.println("");
}

