# Quick Start Guide

## Overview

This is a **barebones, simple implementation** of a flight computer system:
- **TX (Transmitter)**: Reads sensors at maximum reliable rates, logs data locally, transmits via LoRa
- **RX (Receiver)**: Receives data via LoRa, displays in terminal, logs locally

## Hardware Required

### Transmitter (TX)
- ESP32 development board
- BMP280 (pressure/temperature) - I2C address 0x76
- LSM9DS1 (9-axis IMU) - I2C
- LC76G GPS module - UART
- SX1278 LoRa module (433MHz) - SPI

### Receiver (RX)
- ESP32 development board
- SX1278 LoRa module (433MHz) - SPI

## Wiring

### Both TX and RX (LoRa Module)
```
LoRa SX1278    ESP32
-----------    -----
MOSI           GPIO 23
MISO           GPIO 19
SCK            GPIO 18
NSS/CS         GPIO 25
RST            GPIO 14
DIO0           GPIO 4
VCC            3.3V
GND            GND
```

### TX Only (Sensors)

**I2C Sensors (SDA=21, SCL=22):**
```
BMP280 & LSM9DS1
SDA            GPIO 21
SCL            GPIO 22
VCC            3.3V
GND            GND
```

**GPS Module:**
```
LC76G GPS      ESP32
---------      -----
TX             GPIO 16 (RX1)
RX             GPIO 17 (TX1)
VCC            3.3V
GND            GND
```

## Building and Uploading

### Install PlatformIO
```bash
# Install PlatformIO CLI
pip install platformio

# Or use VS Code with PlatformIO IDE extension
```

### Build and Upload TX
```bash
# Build transmitter
pio run -e tx

# Upload to TX board
pio run -e tx -t upload

# Monitor TX serial output
pio device monitor -b 115200
```

### Build and Upload RX
```bash
# Build receiver
pio run -e rx

# Upload to RX board
pio run -e rx -t upload

# Monitor RX serial output (displays received data)
pio device monitor -b 115200
```

## Operation

1. **Power on TX**: It will initialize sensors and start reading/logging at ~100Hz, transmitting at ~5Hz
2. **Power on RX**: It will wait for packets and display all received data in the terminal
3. **Monitor RX terminal**: All telemetry is displayed in real-time and logged to `/received.log`
4. **TX logs locally**: All sensor readings are logged to `/flight.log` at ~100Hz

## Sensor Rates

- **BMP280**: ~157Hz (maximum reliable rate with oversampling x1)
- **LSM9DS1 (Accel/Gyro)**: Up to 952Hz capable (configured for high-speed)
- **LSM9DS1 (Mag)**: ~80Hz capable
- **GPS**: 1-10Hz (module dependent)
- **Local logging**: ~100Hz (10ms interval)
- **LoRa transmission**: ~5Hz (200ms interval)

## Log Files

### TX Log File (`/flight.log`)
CSV format with columns:
```
timestamp,temp,pressure,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lng,alt,speed,sats
```

### RX Log File (`/received.log`)
CSV format with columns:
```
timestamp,temp,pressure,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lng,alt,speed,sats,rssi,snr
```

## LoRa Configuration

Both TX and RX are configured identically:
- **Frequency**: 433 MHz
- **Spreading Factor**: 9 (good range/speed balance)
- **Bandwidth**: 125 kHz
- **Coding Rate**: 4/6
- **TX Power**: 20 dBm (max)
- **Expected Range**: 2+ km line-of-sight

## Troubleshooting

- **No sensor detected**: Check I2C wiring and addresses
- **No LoRa connection**: Verify both boards use same frequency/settings and antennas are connected
- **GPS no fix**: Ensure clear sky view, wait 30-60 seconds for first fix
- **No log files**: Check LittleFS initialization in serial monitor

## Data Format

The system transmits a compact binary structure (~72 bytes) containing:
- Timestamp (4 bytes)
- Pressure, temperature (8 bytes)
- Accelerometer X,Y,Z (12 bytes)
- Gyroscope X,Y,Z (12 bytes)
- Magnetometer X,Y,Z (12 bytes)
- GPS lat, lng, altitude, speed (16 bytes)
- GPS satellite count (1 byte)

At SF9/BW125/CR4/6, this allows ~5 packets/second with good reliability.
