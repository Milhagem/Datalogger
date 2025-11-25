#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x69

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
// Variáveis para o filtro complementar
volatile float angleAccel = 0.0;      // Ângulo calculado a partir do acelerômetro
volatile float angleFiltered = 0.0;   // Ângulo resultante após o filtro complementar
volatile float alpha = 0.98;          // Fator de ponderação (quanto maior, maior o peso do giroscópio)
volatile unsigned long lastTimeMPU = 0;

float MPUangle    = 0;
float MPUgyro     = 0;

// Variavéis para guardar o "ponto zero" (offsets)
float yawOffset   = 0;
float pitchOffset = 0;
float rollOffset  = 0;

void executaMPU(){

  xyzFloat gVal;
  xyzFloat accRaw;
  xyzFloat corrAccRaw;
  myIMU.readSensor();
  myIMU.getAccRawValues(&accRaw);
  myIMU.getCorrectedAccRawValues(&corrAccRaw);
  myIMU.getGValues(&gVal);
  float resultantG = myIMU.getResultantG(&gVal);
   
  Serial.println("g-values (x,y,z):");
  Serial.print(gVal.x);
  Serial.print("   ");
  Serial.print(gVal.y);
  Serial.print("   ");
  Serial.println(gVal.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG * 9.81);
  Serial.println("*************************************");

  xyzFloat gyrRaw; 
  xyzFloat gyr;
  myIMU.readSensor();
  myIMU.getCorrectedGyrRawValues(&gyrRaw);
  myIMU.getGyrValues(&gyr);

  Serial.println("Raw gyroscope values (x,y,z):");
  Serial.print(gyrRaw.x);
  Serial.print("   ");
  Serial.print(gyrRaw.y);
  Serial.print("   ");
  Serial.println(gyrRaw.z);

  Serial.println("Gyroscope values (x,y,z):");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);
  Serial.println();
 

}

void print_calibration() {

}

void taravaloresiniciais() {

}