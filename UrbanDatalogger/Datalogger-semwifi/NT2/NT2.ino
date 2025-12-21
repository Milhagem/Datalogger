#include "Adafruit_BMP3XX.h"
#include "BMP.hpp"
#include "MPU.hpp"
#include "ICM.hpp"
#include <TinyGPS++.h>


//Pinos da comunicação i2c
#define SDA_PIN 8 // GPIO 4
#define SCL_PIN 9 // GPIO 5

//endereço do giroscópio
#define MPU6500_ADDR 0x68

//Variavel de tempo --------------------------------------------------------
unsigned long lastTimer;


//Giroscópio MPU-------------------------------------------------------------------
unsigned long t_prev = 0;
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

float accelX_MPU = 0, accelY_MPU = 0, accelZ_MPU = 0;
float angleX_MPU = 0, angleY_MPU = 0, angleZ_MPU = 0;

int indexMPU = 0;

volatile float media_accX_MPU[sampleSizeMPU];
volatile float media_accY_MPU[sampleSizeMPU];
volatile float media_accZ_MPU[sampleSizeMPU];
volatile float media_angX_MPU[sampleSizeMPU];
volatile float media_angY_MPU[sampleSizeMPU];
volatile float media_angZ_MPU[sampleSizeMPU];

float mediaAccX_MPU = 0, mediaAccY_MPU = 0, mediaAccZ_MPU = 0;
float mediaAngX_MPU = 0, mediaAngY_MPU = 0, mediaAngZ_MPU = 0;



//Giroscópio ICM-----------------------------------------------------------------------
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

unsigned long tprev = 0;

float accelX_ICM = 0, accelY_ICM = 0, accelZ_ICM = 0;
float angleX_ICM = 0, angleY_ICM = 0, angleZ_ICM = 0; 
float pitch = 0;
float roll  = 0;

int indexICM = 0;
volatile float media_accX_ICM[sampleSizeICM];
volatile float media_accY_ICM[sampleSizeICM];
volatile float media_accZ_ICM[sampleSizeICM];
volatile float media_angX_ICM[sampleSizeICM];
volatile float media_angY_ICM[sampleSizeICM];
volatile float media_angZ_ICM[sampleSizeICM];

// Médias Finais
float mediaAccX_ICM = 0, mediaAccY_ICM = 0, mediaAccZ_ICM = 0;
float mediaAngX_ICM = 0, mediaAngY_ICM = 0, mediaAngZ_ICM = 0;



//Sensor de Altura---------------------------------------------------------------------
Adafruit_BMP3XX bmp;
volatile float valoresAltura[sampleSizeBMP];
volatile float valoresTemperatura[sampleSizeBMP];
volatile int indexA = 0;
volatile int indexT = 0;
volatile float mediaA = 0;
volatile float mediaT = 0;
float BMPaltura   = 0;
float BMPtemp     = 0;
float offsetAltura = 0;



//Sensor de GPS -----------------------------------------------------------------------
const int GPS_RX_PIN = 17;  // Pino conectado ao GPS TX// PINO BRANCO
const int GPS_TX_PIN = 16;  // Pino conectado ao GPS RX// PINO VERDE
HardwareSerial gpsSerial(2);

TinyGPSPlus gps;       // <--- 2. Criamos o "tradutor" (chamado gps)
float currentLat = 0.0; // <--- 3. Criamos variáveis para guardar o valor final
float currentLon = 0.0;



void setup() {
  Serial.begin(9600);
  Wire.begin(); 
  delay(2000);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  //-------------Configurações e calibração IMU------------------

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


  //-------------Configurações e Calibração do ICM------------------------------
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
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);  


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

  //função no BMP.hpp para zerar altura
  zeraAltura();


  //----------------------Zera vetores de medias moveis do BMP, MPU e ICM---------
  for(int i = 0; i < sampleSizeBMP; i++) {valoresAltura[i] = 0;}
  for(int i = 0; i < sampleSizeBMP; i++) {valoresTemperatura[i] = 0;}

  for(int i = 0; i < sampleSizeMPU; i++) {
        media_accX_MPU[i] = 0; media_accY_MPU[i] = 0; media_accZ_MPU[i] = 0;
        media_angX_MPU[i] = 0; media_angY_MPU[i] = 0; media_angZ_MPU[i] = 0;
  }
  for(int i = 0; i < sampleSizeICM; i++) {
      media_accX_ICM[i] = 0; media_accY_ICM[i] = 0; media_accZ_ICM[i] = 0;
      media_angX_ICM[i] = 0; media_angY_ICM[i] = 0; media_angZ_ICM[i] = 0;
  } 

}



void loop() {

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if(millis() - lastTimer >= 1000){
    lastTimer = millis();
    
    //Execução BMP
    BMPaltura = (bmp.readAltitude(SEALEVELPRESSURE_HPA) - offsetAltura) * 100;
    BMPtemp = bmp.temperature;
    calculaMedia(BMPaltura, BMPtemp);

    executaMPU();
    mediavaloresMPU();
    printvaloresMPU();
    
    executaICM();
    mediavaloresICM();
    printvaloresICM();

    if (gps.location.isValid()) {
      currentLat = gps.location.lat();
      currentLon = gps.location.lng();
    }
    
    Serial.println();
    Serial.println();
    Serial.print("Latitude: ");
    Serial.print(currentLat);
    Serial.print("Longitude: ");
    Serial.print(currentLon);   
    Serial.println();
    Serial.println();

  }

  /*while((millis() % 1000)){
    if (gpsSerial.available()) {
   // Read the available data
    char gps_reading = gpsSerial.read();
    
    // Pass it to the Serial Monitor
    Serial.write(gps_reading);
    }
  }

  Serial.println("");*/
}

