#include "MPU.hpp"


// Endereço I2C do MPU
volatile const int MPU_ADDR = 0x68;

// Variáveis para os dados do MPU6050
volatile int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// Variáveis para o filtro complementar
volatile float angleAccel = 0.0;      // Ângulo calculado a partir do acelerômetro
volatile float angleFiltered = 0.0;   // Ângulo resultante após o filtro complementar
volatile float alpha = 0.98;          // Fator de ponderação (quanto maior, maior o peso do giroscópio)
volatile unsigned long lastTimeMPU = 0;

void executaMPU(float &angle, float &gyre){
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    // Converte os valores do acelerômetro para unidades de g 
    // (supondo sensibilidade de ±2g → 16384 LSB/g)
    float AcXg = AcX / 16384.0;
    float AcYg = AcY / 16384.0;
    float AcZg = AcZ / 16384.0;

    // Calcula o ângulo a partir dos dados do acelerômetro
    // Fórmula: angleAccel = arctan(AcX / sqrt(AcY² + AcZ²)) * (180/PI)
    angleAccel = atan2(AcXg, sqrt(AcYg * AcYg + AcZg * AcZg)) * (180.0 / M_PI);

    // Converte a leitura do giroscópio para graus por segundo para o eixo Y
    // (sensibilidade padrão para ±250°/s: 131 LSB/(°/s))
    float gyroY = GyY / 131.0;

    // Calcula o intervalo de tempo (dt) em segundos
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimeMPU) / 1000.0;
    lastTimeMPU = currentTime;

    // Aplica o filtro complementar:
    // Combina o ângulo obtido pelo acelerômetro e a integração da taxa de rotação do giroscópio
    angleFiltered = alpha * (angleFiltered + gyroY * dt) + (1 - alpha) * angleAccel;
    angle = angleFiltered;
    gyre = gyroY;

}