
const int GPS_RX_PIN = 17;  // Pino conectado ao GPS TX// PINO BRANCO
const int GPS_TX_PIN = 16;  // Pino conectado ao GPS RX// PINO VERDE

HardwareSerial gpsSerial(2);

void setup() {
  // Initialize software serial for GPS communication
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Initialize the hardware serial port to communicate with the computer
  Serial.begin(9600);
}

void loop() {
  // Check if data is available from GPS
  if (gpsSerial.available()) {
    // Read the available data
    char gps_reading = gpsSerial.read();
    
    // Pass it to the Serial Monitor
    Serial.write(gps_reading);
  }
}