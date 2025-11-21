#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "Adafruit_BMP3XX.h"
#include "BMP.hpp"
#include "MPU.hpp"

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

// ======== CONFIGURAÇÕES DE REDE ========
const char* ssid = "K41S_2075f";
const char* password = "evelynlinda";
const char* mqtt_server = "broker.hivemq.com";   // IP do seu Node-RED ou broker Mosquitto

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

// ======== FUNÇÃO PARA CONECTAR AO WI-FI ========
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

// ======== FUNÇÃO DE RECONEXÃO MQTT ========
void reconnect() {
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    String clientId = "ESP8266-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("Conectado ao broker!");
    } else {
      Serial.print("falhou, rc=");
      Serial.print(client.state());
      Serial.println(" — tentando novamente em 5s");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  Wire.begin();
  delay(5000);

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

// ======== LOOP PRINCIPAL ========
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();


  if (millis() - lastMsg > 2000) {  // envia a cada 2 segundos
    lastMsg = millis();

    //Execução BMP
    BMPaltura = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    BMPtemp = bmp.temperature;
    calculaMedia(BMPaltura, BMPtemp);

    //execução do giroscopio
    executaMPU();
    
    // Monta o JSON
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"AcX\":%.2f,\"Altura\":%.1f}",
             gVal.x * 9.81, BMPaltura);

    Serial.print("Publicando: ");
    Serial.println(payload);

    // Publica no tópico
    client.publish("sensor/esp8266", payload);
  }
}
