#include <Arduino.h>
#include <Wire.h>
#include "MinimalBMP581.h"

// -------- Pin map (ESP32-S3 GPIO numbers) --------
static constexpr int PIN_I2C_SDA    = 15;   // IO15
static constexpr int PIN_I2C_SCL    = 14;   // IO14
static constexpr int PIN_HEARTBEAT  = 12;   // IO12 (LED)
static constexpr int PIN_UART_RX    = 36;   // IO36 (external device TX -> this RX)
static constexpr int PIN_UART_TX    = 37;   // IO37 (this TX -> external device RX)

// I2C address for BMP581 breakout (SparkFun default)
static constexpr uint8_t BMP581_ADDR = 0x47;

// UART settings
static constexpr uint32_t UART_BAUD = 115200;

// Create sensor driver instance
MinimalBMP581 bmp;

// Use Serial1 for the external UART link
HardwareSerial& SensorUart = Serial1;

// Heartbeat timing
unsigned long lastBlink = 0;
const unsigned long blinkPeriodMs = 500; // 2 Hz blink

// Sensor read timing (print every 200 ms = 5 Hz for readability)
unsigned long lastRead = 0;
const unsigned long readPeriodMs = 200;

void setup()
{
  // Basic debug console over USB (optional)
  Serial.begin(115200);
  delay(200);

  // Heartbeat LED
  pinMode(PIN_HEARTBEAT, OUTPUT);
  digitalWrite(PIN_HEARTBEAT, LOW);

  // I2C on custom pins (400 kHz is safe)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  // Initialize BMP581 (retry loop so wiring mistakes are obvious)
  while (bmp.begin(Wire, BMP581_ADDR) != BMP5_OK) {
    Serial.println("BMP581 not found. Check wiring/address. Retrying in 1s...");
    delay(1000);
  }

  // Minimal config: enable pressure, modest OSR, ODR 25 Hz -> NORMAL mode
  if (bmp.configure(BMP5_ODR_25_HZ) != BMP5_OK) {
    Serial.println("BMP581 configure failed. Halting.");
    while (true) {
      digitalWrite(PIN_HEARTBEAT, !digitalWrite(PIN_HEARTBEAT, HIGH)); // fast flash on error
      delay(100);
    }
  }

  // Secondary UART to stream sensor data out on pins
  // Note: Serial1.begin(baud, config, RX, TX)
  SensorUart.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

  Serial.println("BMP581 ready. Streaming data on Serial (USB) and Serial1 (IO36/IO37).");
}

void loop()
{
  // Heartbeat LED
  unsigned long now = millis();
  if (now - lastBlink >= blinkPeriodMs) {
    lastBlink = now;
    digitalWrite(PIN_HEARTBEAT, !digitalRead(PIN_HEARTBEAT));
  }

  // Read sensor and send to both USB Serial and external UART
  if (now - lastRead >= readPeriodMs) {
    lastRead = now;

    float p_pa = NAN, t_c = NAN;
    int8_t rslt = bmp.read(p_pa, t_c);
    if (rslt == BMP5_OK) {
      // Print to USB console
      Serial.print("P[Pa]="); Serial.print(p_pa, 2);
      Serial.print("  T[C]="); Serial.println(t_c, 2);

      // Send CSV line to the external UART (RX=IO36, TX=IO37)
      // Example format: "P,101325.12,T,24.56\n"
      SensorUart.print("P,");  SensorUart.print(p_pa, 2);
      SensorUart.print(",T,"); SensorUart.println(t_c, 2);

      // If the receiver expects a specific format (e.g., JSON or plain numbers), adapt here.
      // e.g., SensorUart.printf("{\"P\":%.2f,\"T\":%.2f}\n", p_pa, t_c);
    } else {
      Serial.print("bmp.read error="); Serial.println(rslt);
    }
  }
}
