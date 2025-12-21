#include "Adafruit_BMP3XX.h"
#include "BMP.hpp"
#include "MPU.hpp"
#include <SoftwareSerial.h>

#define MPU6500_ADDR 0x68
// Variáveis para controle de tempo e cálculo do ângulo Z
unsigned long t_prev = 0;

// --- CRIAÇÃO DOS OBJETOS E VARIÁVEIS GLOBAIS ---
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
Adafruit_BMP3XX bmp;

const int GPS_RX_PIN = 14;  // Pino conectado ao GPS TX
const int GPS_TX_PIN = 12;  // Pino conectado ao GPS RX

//BMP -----------------------------------------------------------------------
#define SEALEVELPRESSURE_HPA (1015.0) // Atualize diariamente!
float groundLevelPressureHPA;

#define SDA_PIN 4 // GPIO 4
#define SCL_PIN 5 // GPIO 5

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

// Definição real das variáveis compartilhadas
float angleX = 0, angleY = 0, angleZ = 0;
float g_accelX = 0, g_accelY = 0, g_accelZ = 0;

//-----------------------------------------------------------------------

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

void setup() {
  Serial.begin(9600);
  Wire.begin(); 
  delay(2000);

  gpsSerial.begin(9600);
  
  //-------------Configurações e calibração Giroscópio------------------

  if(!myMPU6500.init()){
    Serial.println("MPU6500 não respondeu!");
  } else {
    Serial.println("MPU6500 conectado.");
  }

  Serial.println("Calibrando... Mantenha parado e plano!");
  delay(1000);
  
  // A calibração é CRUCIAL para o ângulo Z não ficar rodando sozinho
  myMPU6500.autoOffsets();
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6); // Filtro para suavizar
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);

  Serial.println("Pronto!");
  t_prev = millis();
  
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
  if(millis() - lastTimer >= 200){
    lastTimer = millis();
    
    //Execução BMP
    BMPaltura = bmp.readAltitude(SEALEVELPRESSURE_HPA);
     BMPtemp = bmp.temperature;
    //calculaMedia(BMPaltura, BMPtemp);

    executaMPU();
    //delay(1000);
  }

  /*while((millis() % 1000)){
    if (gpsSerial.available()) {
   // Read the available data
    char gps_reading = gpsSerial.read();
    
    // Pass it to the Serial Monitor
    Serial.write(gps_reading);
    }
    //delay(1000);
  }*/

  //Serial.println("");
}

