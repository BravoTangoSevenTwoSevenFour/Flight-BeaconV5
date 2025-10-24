/*
 * Barebones Flight Computer - Receiver (RX)
 * Receives LoRa telemetry, displays in terminal, and logs locally
 */

#include <SPI.h>
#include <LoRa.h>
#include <LittleFS.h>

// LoRa pins (same as TX)
#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS   25
#define LORA_RST  14
#define LORA_DIO0 4

// LoRa frequency (must match TX)
#define LORA_FREQ 433E6

// Sensor data structure (must match TX)
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

File logFile;
unsigned long packetCounter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("RX: Initializing...");

  // Initialize LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  // Configure LoRa (must match TX)
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  Serial.println("LoRa OK");

  // Initialize filesystem
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS init failed!");
  } else {
    Serial.println("LittleFS OK");
    // Create new log file with timestamp
    logFile = LittleFS.open("/received.log", "w");
    if (logFile) {
      logFile.println("timestamp,temp,pressure,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lng,alt,speed,sats,rssi,snr");
      logFile.close();
    }
  }

  Serial.println("RX: Ready - waiting for data...");
  Serial.println("---");
}

void loop() {
  // Check for incoming packets
  int packetSize = LoRa.parsePacket();

  if (packetSize > 0) {
    // Read packet
    if (packetSize == sizeof(SensorData)) {
      SensorData data;
      LoRa.readBytes((uint8_t*)&data, sizeof(data));

      // Get signal quality
      int rssi = LoRa.packetRssi();
      float snr = LoRa.packetSnr();

      packetCounter++;

      // Display in terminal
      Serial.println("=== PACKET #" + String(packetCounter) + " ===");
      Serial.printf("Time:        %lu ms\n", data.timestamp);
      Serial.printf("Temperature: %.2f C\n", data.temperature);
      Serial.printf("Pressure:    %.2f Pa (%.2f hPa)\n", data.pressure, data.pressure / 100.0);
      Serial.printf("Accel:       X=%.2f Y=%.2f Z=%.2f m/s²\n", data.accelX, data.accelY, data.accelZ);
      Serial.printf("Gyro:        X=%.2f Y=%.2f Z=%.2f rad/s\n", data.gyroX, data.gyroY, data.gyroZ);
      Serial.printf("Mag:         X=%.2f Y=%.2f Z=%.2f uT\n", data.magX, data.magY, data.magZ);

      if (data.gpsSats > 0) {
        Serial.printf("GPS:         %.6f, %.6f\n", data.gpsLat, data.gpsLng);
        Serial.printf("Altitude:    %.2f m\n", data.gpsAlt);
        Serial.printf("Speed:       %.2f m/s\n", data.gpsSpeed);
        Serial.printf("Satellites:  %d\n", data.gpsSats);
      } else {
        Serial.println("GPS:         No fix");
      }

      Serial.printf("RSSI:        %d dBm\n", rssi);
      Serial.printf("SNR:         %.2f dB\n", snr);
      Serial.println("---");

      // Log to file
      logFile = LittleFS.open("/received.log", "a");
      if (logFile) {
        logFile.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%d,%d,%.2f\n",
                       data.timestamp,
                       data.temperature, data.pressure,
                       data.accelX, data.accelY, data.accelZ,
                       data.gyroX, data.gyroY, data.gyroZ,
                       data.magX, data.magY, data.magZ,
                       data.gpsLat, data.gpsLng, data.gpsAlt, data.gpsSpeed,
                       data.gpsSats,
                       rssi, snr);
        logFile.close();
      }

    } else {
      Serial.printf("Received packet with unexpected size: %d bytes (expected %d)\n",
                    packetSize, sizeof(SensorData));
    }
  }
}
