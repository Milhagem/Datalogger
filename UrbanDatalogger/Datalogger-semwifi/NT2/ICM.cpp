#include <Wire.h>
#include "ICM.hpp"


void executaICM(){
  // 2. Calcular o tempo passado desde a última leitura (dt)
  unsigned long tnow = millis();
  float delta = (tnow - tprev) / 1000.0; // Converte ms para segundos
  tprev = tnow;

  xyzFloat gyrRaw;  
  xyzFloat gyr;
  xyzFloat gValue;
  xyzFloat angle;
  myIMU.readSensor();
  myIMU.getGValues(&gValue);
  myIMU.getAngles(&angle);
  myIMU.getGyrValues(&gyr);

  pitch = myIMU.getPitch();
  roll  = myIMU.getRoll();
 
   if (abs(gyr.z) > 1.0) { 
      angleZ_ICM += gyr.z * delta;
  }

  accelX_ICM = gValue.x * 9.81;
  accelY_ICM = gValue.y * 9.81;
  accelZ_ICM = gValue.z * 9.81;

  angleX_ICM = angle.y;
  angleY_ICM = angle.x;


}

void printvaloresICM() {
  Serial.println("G values (x,y,z):");
  Serial.print(mediaAccX_ICM);
  Serial.print("   ");
  Serial.print(mediaAccY_ICM);
  Serial.print("   ");
  Serial.println(mediaAccZ_ICM);
  Serial.println("Angles (x,y,z):");
  Serial.print(mediaAngX_ICM);
  Serial.print("   ");
  Serial.print(mediaAngY_ICM);
  Serial.print("   ");
  Serial.println(mediaAngZ_ICM);
  Serial.println("Pitch and roll");
  Serial.print(pitch);
  Serial.print("   ");
  Serial.print(roll);
  Serial.println();
}

void mediavaloresICM() {
    
    // 1. Armazena os valores atuais nos vetores
    media_accX_ICM[indexICM] = accelX_ICM;
    media_accY_ICM[indexICM] = accelY_ICM;
    media_accZ_ICM[indexICM] = accelZ_ICM;
    
    media_angX_ICM[indexICM] = angleX_ICM;
    media_angY_ICM[indexICM] = angleY_ICM;
    media_angZ_ICM[indexICM] = angleZ_ICM;

    // 2. Atualiza o índice circular
    indexICM = (indexICM + 1) % sampleSizeICM;

    // 3. Soma temporária
    float somaAccX = 0.0, somaAccY = 0.0, somaAccZ = 0.0;
    float somaAngX = 0.0, somaAngY = 0.0, somaAngZ = 0.0;

    for (int i = 0; i < sampleSizeICM; i++) {
        somaAccX += media_accX_ICM[i];
        somaAccY += media_accY_ICM[i];
        somaAccZ += media_accZ_ICM[i];
        
        somaAngX += media_angX_ICM[i];
        somaAngY += media_angY_ICM[i];
        somaAngZ += media_angZ_ICM[i];
    }

    // 4. Calcula a Média Final
    mediaAccX_ICM = somaAccX / sampleSizeICM;
    mediaAccY_ICM = somaAccY / sampleSizeICM;
    mediaAccZ_ICM = somaAccZ / sampleSizeICM;

    mediaAngX_ICM = somaAngX / sampleSizeICM;
    mediaAngY_ICM = somaAngY / sampleSizeICM;
    mediaAngZ_ICM = somaAngZ / sampleSizeICM;
}
