#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#include <Velocidade.hpp>
#include <LiquidCrystal_I2C.h>
#include "FS.h"
#include "SD_MMC.h"
#include "Adafruit_BMP3XX.h"
#include "BMP.hpp"
#include <MPU.hpp>


#define pinSensorHall 34


//BMP -----------------------------------------------------------------------
  #define SEALEVELPRESSURE_HPA (1015.0) // Atualize diariamente!

// LilyGO T-SIM7000G Pinout (Pinos funcionamento Gps)------------------------
#define UART_BAUD   115200
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT  Serial1
//---------------------------------------------------------------------------

// Pinos referentes a Escrita no SD -----------------------------------------
#define SD_MISO 2 
#define SD_CS   13
#define SD_MOSI 15
#define SD_SCLK 14
#define PWR_PIN 4
//#define LED_PIN 12

#define ONE_BIT_MODE  true


//Variaveis para analise ----------------------------------------------------

    float velocidadeRoda = 0.0;
    float velocidadeMotor = 0.0;
    
    float GPSlat      = 0;
    float GPSlon      = 0;
    float GPSspeed    = 0;
    float GPSalt      = 0;
    int   GPSvsat     = 0;
    int   GPSusat     = 0;
    float GPSaccuracy = 0;
    int   GPSyear     = 0;
    int   GPSmonth    = 0;
    int   GPSday      = 0;
    int   GPShour     = 0;
    int   GPSmin      = 0;
    int   GPSsec      = 0;

    float BMPaltura   = 0;
    float BMPtemp     = 0;

    float MPUangle    = 0;
    float MPUgyro     = 0;

//--------------------------------------------------------------------------
//Variavel de tempo --------------------------------------------------------
unsigned long lastTimer;

//--------------------------------------------------------------------------

//Variaveis Externas -------------------------------------------------------
  //Velocidade Da Roda -----------------------------------------------------
  extern volatile unsigned long pulseInterval;     // ms
  extern volatile unsigned long lastPulseInterval; // ms
  extern volatile unsigned long pulseIntervals[sampleSize];
  extern volatile int pulseIndex;
  extern volatile unsigned long lastTime;          // ms
  extern volatile unsigned long timeAjust;         // ms
  extern volatile unsigned long timerCalcVel;      // ms
  extern volatile unsigned long lastTimerCalcVel;  // ms
  extern volatile unsigned long timeEstabilizaVel;
  extern volatile float velocidade;
  extern unsigned long lastTimerTax;  // ms

  //-----------------------------------------------------------------------

  //Velocidade Pino M -----------------------------------------------------

  extern volatile unsigned long pulseIntervalM;     // ms
  extern volatile unsigned long lastPulseIntervalM; // ms
  extern volatile unsigned long pulseIntervalsM[sampleSize];
  extern volatile int pulseIndexM;
  extern volatile float velocidadeM;
  extern unsigned long lastTimerTaxM;  // ms

  //-----------------------------------------------------------------------

  //Variaveis Referentes ao BMP -------------------------------------------
  extern volatile float valoresAltura[sampleSize];
  extern volatile float valoresTemperatura[sampleSize];
  extern volatile int indexA;
  extern volatile int indexT;
  extern volatile float mediaA;
  extern volatile float mediaT;

  Adafruit_BMP3XX bmp;
  //-----------------------------------------------------------------------

  //Variaveis Referentes ao MPU -------------------------------------------

  extern volatile const int MPU_ADDR;

  // Variáveis para os dados do MPU6050
  extern volatile int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

  // Variáveis para o filtro complementar
  extern volatile float angleAccel;      // Ângulo calculado a partir do acelerômetro
  extern volatile float angleFiltered;   // Ângulo resultante após o filtro complementar
  extern volatile float alpha;          // Fator de ponderação (quanto maior, maior o peso do giroscópio)
  extern volatile unsigned long lastTimeMPU;

  //-----------------------------------------------------------------------

  //Display

  LiquidCrystal_I2C lcd(0x27, 16, 2);

TinyGsm modem(SerialAT);

void setup() {
  Serial.begin(115200);
  SerialMon.println("Place your board outside to catch satelite signal");

  lcd.init();
  lcd.backlight();
  lcd.print("Inicializando...");

  pinMode(pinSensorHall, INPUT);
  pinMode(pinSensorM, INPUT);

  //Turn on the modem
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(200);
  digitalWrite(PWR_PIN, LOW);

  //Wire.begin(SDA_PIN, SCL_PIN);

  delay(500);
  
  // Set module baud rate and UART pins
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  if (!modem.restart()) {
    SerialMon.println("Failed to restart modem, attempting to continue without restarting");
  }

  delay(500);
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.println(" SGPIO=0,4,1,1 false ");
  }

  modem.enableGPS();
  
  //Zera vetores de medias moveis
  for(int i = 0; i < sampleSize; i++){pulseIntervals[i] = 0;}
  for(int i = 0; i < sampleSize; i++){pulseIntervalsM[i] = 0;}
  for(int i = 0; i < sampleSize; i++) {valoresAltura[i] = 0;}
  for(int i = 0; i < sampleSize; i++) {valoresTemperatura[i] = 0;}


  //Seta pinos cartão sd
  pinMode(SD_MISO, INPUT_PULLUP);
  pinMode(SD_CS, INPUT_PULLUP);
  pinMode(SD_MOSI, INPUT_PULLUP);
  pinMode(PWR_PIN, INPUT_PULLUP);
  //pinMode(LED_PIN, INPUT_PULLUP);

  //Verifica SD
  if(!SD_MMC.begin("/sdcard", ONE_BIT_MODE)){
        Serial.println("falha sd");
        return;
    } else {
        Serial.println("sucesso sd");
    }
    
    File myFile = SD_MMC.open("/Teste.txt", FILE_WRITE);


  // Inicializa BMP
  if (!bmp.begin_I2C(0x77)) {
    Serial.println("Sensor não encontrado!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.println("BMP F");
    //while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);

  // Configura o MPU6050: liga o sensor
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Registro PWR_MGMT_1
  Wire.write(0);    // Zera o registro para ligar o sensor
  Wire.endTransmission(true);



  attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
  //attachInterrupt(digitalPinToInterrupt(pinSensorM), calcM, RISING);

  lcd.clear();
}

void loop() {

  /*modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.println(" SGPIO=0,4,1,1 false ");
  }

  modem.enableGPS();*/

  if(millis() - lastTimer >= 1000){
    lastTimer = millis();
    modem.enableGPS();

    calculaVelocidade(velocidadeRoda);

    //calculaVelocidadeM(velocidadeMotor);
    
    //Execução BMP
    BMPaltura = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    BMPtemp = bmp.temperature;
    calculaMedia(BMPaltura, BMPtemp);

    //Execução MPU
    executaMPU(MPUangle, MPUgyro);

    detachInterrupt(digitalPinToInterrupt(pinSensorHall));
    //detachInterrupt(digitalPinToInterrupt(pinSensorM));

    for (int8_t i = 15; i; i--) {
      SerialMon.println("Requesting current GPS/GNSS/GLONASS location");
      if (modem.getGPS(&GPSlat, &GPSlon, &GPSspeed, &GPSalt, &GPSvsat, &GPSusat, &GPSaccuracy,
                      &GPSyear, &GPSmonth, &GPSday, &GPShour, &GPSmin, &GPSsec)) {
        //Debug
        /*SerialMon.println("Latitude: " + String(GPSlat, 8) + "\tLongitude: " + String(GPSlon, 8));
        SerialMon.println("Speed: " + String(GPSspeed) + "\tAltitude: " + String(GPSalt));
        //SerialMon.println("Visible Satellites: " + String(GPSvsat) + "\tUsed Satellites: " + String(GPSusat));
        SerialMon.println("Accuracy: " + String(GPSaccuracy));
        SerialMon.println("Year: " + String(GPSyear) + "\tMonth: " + String(GPSmonth) + "\tDay: " + String(GPSday));
        SerialMon.println("Hour: " + String(GPShour) + "\tMinute: " + String(GPSmin) + "\tSecond: " + String(GPSsec));*/
        break;
      } 
      else {
        //SerialMon.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 1s.");
        //lcd.clear();
        //lcd.setCursor(0, 0);
        //lcd.print("la:  ");
        //lcd.print("Carregando...");
        /*GPSlat      = 0;
        GPSlon      = 0;
        GPSspeed    = 0;
        GPSalt      = 0;
        GPSvsat     = 0;
        GPSusat     = 0;
        GPSaccuracy = 0;
        GPSyear     = 0;
        GPSmonth    = 0;
        GPSday      = 0;
        GPShour     = 0;
        GPSmin      = 0;
        GPSsec      = 0;*/
        //delay(1000L);
      }
    }

    /*lcd.setCursor(0, 1);
    lcd.print(GPSlat);
    lcd.setCursor(8,1);
    lcd.print(GPSlon);*/

    //modem.disableGPS();

    //modem.disableGPS();

    File myFile = SD_MMC.open("/TESTE_GPS.TXT", FILE_APPEND);

    if (myFile){
    } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Erro SD");
    }
    
    char buffer[500];
    snprintf(buffer, sizeof(buffer), "lat: %.8f lon: %.8f V: %.1f M: %.1f MPUA: %.2f MPUG: %.2f BMPA: %.2f BMPT: %.2f HORA: %.f MINUTO: %.f SEGUNDO: %.f",GPSlat, GPSlon, velocidadeRoda, velocidadeMotor, MPUangle, MPUgyro, BMPaltura, BMPtemp, GPShour , GPSmin, GPSsec);
    myFile.println(buffer);

    myFile.close();

    attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
    //attachInterrupt(digitalPinToInterrupt(pinSensorM), calcM, RISING);

    //lcd.clear();

    lcd.setBacklight(HIGH);
    /* 
    lcd.setCursor(0, 0);
    lcd.print("V:  ");
    lcd.print(velocidadeRoda, 1);
    lcd.print(" RPM");*/

    lcd.setCursor(0, 0);
    lcd.print(GPSlat, 8);
    lcd.setCursor(0, 1);
    lcd.print(GPSlon, 8);


    /*lcd.setCursor(0, 1);
    lcd.print("M:  ");
    lcd.print(velocidadeMotor, 1);*/
    
    //Teste bmp mpu;
    /*lcd.setCursor(0, 0);
    lcd.print("bmp:  ");
    lcd.print(BMPaltura, 1);
    lcd.print(" ");

    lcd.setCursor(0, 1);
    lcd.print("A: ");
    lcd.print(MPUangle, 1);
    lcd.setCursor(8, 1);
    lcd.print("G: ");
    lcd.print(MPUgyro, 1);*/

  }
}
