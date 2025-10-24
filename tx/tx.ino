/*
 * Barebones Flight Computer - Transmitter (TX)
 * Reads sensors at maximum reliable rate, logs locally, and transmits via LoRa
 */

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LSM9DS1.h>
#include <TinyGPS++.h>
#include <LittleFS.h>

// LoRa pins
#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS   25
#define LORA_RST  14
#define LORA_DIO0 4

// LoRa frequency
#define LORA_FREQ 433E6

// GPS UART
#define GPS_RX 16
#define GPS_TX 17

// Sensors
Adafruit_BMP280 bmp;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// Data buffers
File logFile;
unsigned long lastTransmit = 0;
unsigned long lastLog = 0;
unsigned long packetCounter = 0;

// Sensor data structure (packed for efficient transmission)
struct SensorData {
  uint32_t timestamp;
  float pressure;
  float temperature;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
  float gpsLat, gpsLng;
  float gpsAlt;
  float gpsSpeed;
  uint8_t gpsSats;
} __attribute__((packed));

SensorData data;

// Forward declarations
void logData();
void transmitData();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("TX: Initializing...");

  // Initialize I2C
  Wire.begin();

  // Initialize BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 init failed!");
  } else {
    // Set highest sampling rate (normal mode, oversampling x1)
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X1,
                    Adafruit_BMP280::SAMPLING_X1,
                    Adafruit_BMP280::FILTER_OFF,
                    Adafruit_BMP280::STANDBY_MS_1); // ~157Hz
    Serial.println("BMP280 OK");
  }

  // Initialize LSM9DS1
  if (!lsm.begin()) {
    Serial.println("LSM9DS1 init failed!");
  } else {
    // Set high data rates
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
    Serial.println("LSM9DS1 OK");
  }

  // Initialize GPS
  gpsSerial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS OK");

  // Initialize LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  // Configure LoRa for good range and speed balance
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  LoRa.setTxPower(20);
  Serial.println("LoRa OK");

  // Initialize filesystem
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS init failed!");
  } else {
    Serial.println("LittleFS OK");
  }

  // Initialize GPS data to prevent garbage values
  data.gpsLat = 0.0;
  data.gpsLng = 0.0;
  data.gpsAlt = 0.0;
  data.gpsSpeed = 0.0;
  data.gpsSats = 0;

  // Initialize timing variables properly
  lastTransmit = millis();
  lastLog = millis();

  Serial.println("TX: Ready!");
  delay(100); // Small delay to stabilize
}

void loop() {
  static unsigned long counter = 0;
  counter++;
  
  unsigned long now = millis();
  data.timestamp = now;
  
  // Read BMP280
  data.temperature = bmp.readTemperature();
  data.pressure = bmp.readPressure();
  
  // Read LSM9DS1
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  data.accelX = a.acceleration.x;
  data.accelY = a.acceleration.y;
  data.accelZ = a.acceleration.z;
  data.gyroX = g.gyro.x;
  data.gyroY = g.gyro.y;
  data.gyroZ = g.gyro.z;
  data.magX = m.magnetic.x;
  data.magY = m.magnetic.y;
  data.magZ = m.magnetic.z;
  
  // Read GPS (non-blocking)
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isValid()) {
    data.gpsLat = gps.location.lat();
    data.gpsLng = gps.location.lng();
    data.gpsAlt = gps.altitude.meters();
    data.gpsSpeed = gps.speed.mps();
    data.gpsSats = gps.satellites.value();
  }
  
  // Show heartbeat
  if (counter % 10 == 0) {
    Serial.print(".");
  }
  
  // Transmit via LoRa every 1 second
  if (now >= lastTransmit && (now - lastTransmit) >= 1000) {
    lastTransmit = now;
    transmitData();
  }
  
  delay(10);
}

void logData() {
  logFile = LittleFS.open("/flight.log", "a");
  if (logFile) {
    // Write CSV format - using print instead of printf to avoid potential issues
    logFile.print(data.timestamp);
    logFile.print(",");
    logFile.print(data.temperature);
    logFile.print(",");
    logFile.print(data.pressure);
    logFile.print(",");
    logFile.print(data.accelX);
    logFile.print(",");
    logFile.print(data.accelY);
    logFile.print(",");
    logFile.print(data.accelZ);
    logFile.print(",");
    logFile.print(data.gyroX);
    logFile.print(",");
    logFile.print(data.gyroY);
    logFile.print(",");
    logFile.print(data.gyroZ);
    logFile.print(",");
    logFile.print(data.magX);
    logFile.print(",");
    logFile.print(data.magY);
    logFile.print(",");
    logFile.print(data.magZ);
    logFile.print(",");
    logFile.print(data.gpsLat, 6);
    logFile.print(",");
    logFile.print(data.gpsLng, 6);
    logFile.print(",");
    logFile.print(data.gpsAlt);
    logFile.print(",");
    logFile.print(data.gpsSpeed);
    logFile.print(",");
    logFile.println(data.gpsSats);
    logFile.close();
  }
}

void transmitData() {
  // Send binary data structure for efficiency
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&data, sizeof(data));
  LoRa.endPacket();

  packetCounter++;
  Serial.print("TX #");
  Serial.print(packetCounter);
  Serial.print(": T=");
  Serial.print(data.temperature);
  Serial.print(" P=");
  Serial.print(data.pressure);
  Serial.print(" A=(");
  Serial.print(data.accelX);
  Serial.print(",");
  Serial.print(data.accelY);
  Serial.print(",");
  Serial.print(data.accelZ);
  Serial.print(") GPS=");
  Serial.print(data.gpsLat, 4);
  Serial.print(",");
  Serial.println(data.gpsLng, 4);
}
