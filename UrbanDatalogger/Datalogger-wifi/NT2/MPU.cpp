#include "MPU.hpp"
#include <math.h>



void executaMPU(){
   // 1. Ler dados brutos
  xyzFloat gValue = myMPU6500.getGValues();    // Aceleração (g)
  xyzFloat gyr = myMPU6500.getGyrValues();     // Giroscópio (graus/s)

  // 2. Calcular o tempo passado desde a última leitura (dt)
  unsigned long t_now = millis();
  float dt = (t_now - t_prev) / 1000.0; // Converte ms para segundos
  t_prev = t_now;


// 3. Integração do Z (Yaw)
  if (abs(gyr.z) > 1.0) { 
      angleZ_MPU += gyr.z * dt;
  }
  
  // Normalização do ângulo Z (-180 a 180)
  if (angleZ_MPU >= 180.0) {
      angleZ_MPU -= 360.0;
  }
  else if (angleZ_MPU < -180.0) {
      angleZ_MPU += 360.0;
  }

  // Atualiza variáveis globais com sufixo _MPU
  angleY_MPU = myMPU6500.getPitch();
  angleX_MPU = myMPU6500.getRoll();

  accelX_MPU = gValue.x * 9.81;
  accelY_MPU = gValue.y * 9.81;
  accelZ_MPU = gValue.z * 9.81;
}



void printvaloresMPU() {
  // -- Acelerações --
  Serial.print("ACEL (g) ->  X:");
  Serial.print(mediaAccX_MPU);
  Serial.print("  Y:");
  Serial.print(mediaAccY_MPU);
  Serial.print("  Z:");
  Serial.print(mediaAccZ_MPU);

  Serial.print("    |    "); // Separador

  // -- Ângulos --
  Serial.print("ANGULOS (Graus) ->  X (Roll):");
  Serial.print(mediaAngX_MPU);
  Serial.print("  Y (Pitch):");
  Serial.print(mediaAngY_MPU);
  Serial.print("  Z (Yaw):");
  Serial.println(mediaAngZ_MPU);

}



void mediavaloresMPU() {
    
    // 1. Armazena no histórico
    media_accX_MPU[indexMPU] = accelX_MPU;
    media_accY_MPU[indexMPU] = accelY_MPU;
    media_accZ_MPU[indexMPU] = accelZ_MPU;
    
    media_angX_MPU[indexMPU] = angleX_MPU;
    media_angY_MPU[indexMPU] = angleY_MPU;
    media_angZ_MPU[indexMPU] = angleZ_MPU;

    // 2. Atualiza índice
    indexMPU = (indexMPU + 1) % sampleSizeMPU;

    // 3. Soma temporária
    float somaAccX = 0.0, somaAccY = 0.0, somaAccZ = 0.0;
    float somaAngX = 0.0, somaAngY = 0.0, somaAngZ = 0.0;

    for (int i = 0; i < sampleSizeMPU; i++) {
        somaAccX += media_accX_MPU[i];
        somaAccY += media_accY_MPU[i];
        somaAccZ += media_accZ_MPU[i];
        
        somaAngX += media_angX_MPU[i];
        somaAngY += media_angY_MPU[i];
        somaAngZ += media_angZ_MPU[i];
    }

    // 4. Média Final Global
    mediaAccX_MPU = somaAccX / sampleSizeMPU;
    mediaAccY_MPU = somaAccY / sampleSizeMPU;
    mediaAccZ_MPU = somaAccZ / sampleSizeMPU;

    mediaAngX_MPU = somaAngX / sampleSizeMPU;
    mediaAngY_MPU = somaAngY / sampleSizeMPU;
    mediaAngZ_MPU = somaAngZ / sampleSizeMPU;
}