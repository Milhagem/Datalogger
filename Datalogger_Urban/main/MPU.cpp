#include "MPU.hpp"
MPU9250 mpu;
// Variáveis para os dados do MPU6050
volatile float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

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

volatile float relativeYaw = 0;
volatile float relativeRoll = 0;
volatile float relativePitch = 0;

void executaMPU(){


    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            GyY = mpu.getPitch();
            GyX = mpu.getRoll();
            GyZ = mpu.getYaw();
            AcX = mpu.getLinearAccX() * 9.81;
            AcY = mpu.getLinearAccY() * 9.81;
            AcZ = mpu.getLinearAccZ() * 9.81;

            // Calcula os ângulos RELATIVOS (subtraindo o offset)
            relativeYaw   = GyZ - yawOffset;
            relativePitch = GyY - pitchOffset;
            relativeRoll  = GyX - rollOffset;
            prev_ms = millis();
        }
    }
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

void taravaloresiniciais() {
    Serial.println("Definindo o ponto zero inicial...");
    bool leuOffset = false;
    while (!leuOffset) {
        if (mpu.update()) {
            static uint32_t prev_ms = millis();
            if (millis() > prev_ms + 5000) {
                yawOffset   = mpu.getYaw();
                pitchOffset = mpu.getPitch();
                rollOffset  = mpu.getRoll();
                leuOffset   = true;
            }
        }
        delay(10); // Pequeno delay para não sobrecarregar
    }
}