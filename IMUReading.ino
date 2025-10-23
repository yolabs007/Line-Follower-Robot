#include <Arduino.h>
#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ── Keep your same-style drivers ───────────────────────────────────────────────
BluetoothSerial SerialBT;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);   // default address 0x28 (ADR low)

// Optional: if you wired RST to a GPIO, set pin here; otherwise leave -1
#define BNO_RST_PIN   -1

// ── Helpers ───────────────────────────────────────────────────────────────────
void printCalStatus() {
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(" Cal(sys,gr,ac,mg)=");
  Serial.print((int)sys); Serial.print(',');
  Serial.print((int)gyro); Serial.print(',');
  Serial.print((int)accel); Serial.print(',');
  Serial.print((int)mag);

  SerialBT.print(" Cal(sys,gr,ac,mg)=");
  SerialBT.print((int)sys); SerialBT.print(',');
  SerialBT.print((int)gyro); SerialBT.print(',');
  SerialBT.print((int)accel); SerialBT.print(',');
  SerialBT.print((int)mag);
}

void hardResetIfWired() {
  if (BNO_RST_PIN < 0) return;
  pinMode(BNO_RST_PIN, OUTPUT);
  digitalWrite(BNO_RST_PIN, HIGH);  delay(5);
  digitalWrite(BNO_RST_PIN, LOW);   delay(10);
  digitalWrite(BNO_RST_PIN, HIGH);  delay(700);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C on ESP32 default pins (SDA=21, SCL=22). 400 kHz is fine.
  Wire.begin(21, 22, 400000);

  // Bluetooth
  SerialBT.begin("ESP32_IMU");
  Serial.println("\n[IMU] Bluetooth Ready (ESP32_IMU).");
  SerialBT.println("[IMU] Bluetooth Ready (ESP32_IMU).");

  hardResetIfWired();

  // Try default address 0x28 first
  if (!bno.begin()) {
    Serial.println("[IMU] Not found at 0x28. Trying 0x29...");
    SerialBT.println("[IMU] Not found at 0x28. Trying 0x29...");
    // Re-init with alt address
    Adafruit_BNO055 bno_alt = Adafruit_BNO055(55, 0x29);
    bno = bno_alt;
    if (!bno.begin()) {
      Serial.println("[IMU] BNO055 NOT DETECTED. Check wiring/address.");
      SerialBT.println("[IMU] BNO055 NOT DETECTED. Check wiring/address.");
      while (1) delay(10);
    }
  }

  // Use external crystal if present (most breakouts have it)
  bno.setExtCrystalUse(true);

  Serial.println("[IMU] BNO055 ready. Printing data...");
  SerialBT.println("[IMU] BNO055 ready. Printing data...");

  // Small settle
  delay(1000);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // Euler angles (degrees): x=yaw(heading), y=roll, z=pitch
  sensors_event_t event;
  bno.getEvent(&event);

  // Raw vectors
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);     // rad/s
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // m/s^2
  imu::Vector<3> mag   = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);  // uT

  // Temperature
  int8_t tempC = bno.getTemp();

  // ── Print to Serial ──
  Serial.print("Yaw/Roll/Pitch: ");
  Serial.print(event.orientation.x, 1); Serial.print(", ");
  Serial.print(event.orientation.y, 1); Serial.print(", ");
  Serial.print(event.orientation.z, 1);

  Serial.print(" | Gyro(rad/s): ");
  Serial.print(gyro.x(), 3); Serial.print(", ");
  Serial.print(gyro.y(), 3); Serial.print(", ");
  Serial.print(gyro.z(), 3);

  Serial.print(" | Accel(m/s^2): ");
  Serial.print(accel.x(), 2); Serial.print(", ");
  Serial.print(accel.y(), 2); Serial.print(", ");
  Serial.print(accel.z(), 2);

  Serial.print(" | Mag(uT): ");
  Serial.print(mag.x(), 1); Serial.print(", ");
  Serial.print(mag.y(), 1); Serial.print(", ");
  Serial.print(mag.z(), 1);

  Serial.print(" | Temp(C): ");
  Serial.print(tempC);

  printCalStatus();
  Serial.println();

  // ── Print to Bluetooth ──
  SerialBT.print("YPR: ");
  SerialBT.print(event.orientation.x, 1); SerialBT.print(", ");
  SerialBT.print(event.orientation.y, 1); SerialBT.print(", ");
  SerialBT.print(event.orientation.z, 1);

  SerialBT.print(" | G:");
  SerialBT.print(gyro.x(), 2); SerialBT.print(',');
  SerialBT.print(gyro.y(), 2); SerialBT.print(',');
  SerialBT.print(gyro.z(), 2);

  SerialBT.print(" | A:");
  SerialBT.print(accel.x(), 2); SerialBT.print(',');
  SerialBT.print(accel.y(), 2); SerialBT.print(',');
  SerialBT.print(accel.z(), 2);

  SerialBT.print(" | M:");
  SerialBT.print(mag.x(), 1); SerialBT.print(',');
  SerialBT.print(mag.y(), 1); SerialBT.print(',');
  SerialBT.print(mag.z(), 1);

  SerialBT.print(" | T:");
  SerialBT.print(tempC);

  SerialBT.print(" |");
  printCalStatus();
  SerialBT.println();

  delay(100);  // ~10 Hz
}
