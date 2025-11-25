#include "MPU.hpp"

extern unsigned long t_prev;

void executaMPU(){
   // 1. Ler dados brutos
  xyzFloat gValue = myMPU6500.getGValues();    // Aceleração (g)
  xyzFloat gyr = myMPU6500.getGyrValues();     // Giroscópio (graus/s)

  // 2. Calcular o tempo passado desde a última leitura (dt)
  unsigned long t_now = millis();
  float dt = (t_now - t_prev) / 1000.0; // Converte ms para segundos
  t_prev = t_now;

  // ---------------------------------------------------------
  // 3. CÁLCULO DOS ÂNGULOS
  // ---------------------------------------------------------

  // Ângulo X (Roll) e Y (Pitch) usando Trigonometria no Acelerômetro
  // Isso funciona medindo onde a gravidade (1g) está puxando
  // Atualiza as variáveis GLOBAIS (definidas no header)
  angleX = atan2(gValue.y, sqrt(gValue.x * gValue.x + gValue.z * gValue.z)) * 180.0 / PI;
  angleY = atan2(-gValue.x, sqrt(gValue.y * gValue.y + gValue.z * gValue.z)) * 180.0 / PI;

  // Integração do Z
  if (abs(gyr.z) > 1.0) { 
     angleZ += gyr.z * dt;
  }

  // Atualiza acelerações globais
  g_accelX = gValue.x * 9.81;
  g_accelY = gValue.y * 9.81;
  g_accelZ = gValue.z * 9.81;
  // -- Acelerações --
  Serial.print("ACEL (g) ->  X:");
  Serial.print(g_accelX);
  Serial.print("  Y:");
  Serial.print(g_accelY);
  Serial.print("  Z:");
  Serial.print(g_accelZ);

  Serial.print("    |    "); // Separador

  // -- Ângulos --
  Serial.print("ANGULOS (Graus) ->  X (Roll):");
  Serial.print(angleX);
  Serial.print("  Y (Pitch):");
  Serial.print(angleY);
  Serial.print("  Z (Yaw):");
  Serial.println(angleZ);

  delay(50);
}