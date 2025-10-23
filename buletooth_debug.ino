
/*
1. Written by Rahul Sharma for Yolabs - 23rd Oct'25
2. To demonstrate a super simple Serial BT connection. You will need this to debug your vehicles 
3. Please note keep the name of your ESP32 unique and if in past your ESP had a different name your phone still may remeber the old name
STEP to connect 
1. Go to phone bluetooth and connect the ESP32 
2. Download Bluetooth serial app 
3. Find the device and connect and vola that all 

*/

#include "BluetoothSerial.h"

// ── Keep your same-style drivers ───────────────────────────────────────────────
BluetoothSerial SerialBT;
int Count = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);


  // Bluetooth
  SerialBT.begin("ESP32_IMU");
  Serial.println("\n bluetooth is ready connect it using  bluetooth serial app");
}
// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {

  // ── Print to Serial ──
  Count += 1;
  Serial.println("count ");Serial.print(Count);
  // print for phone screen over bluetooth
  SerialBT.println("count ");SerialBT.print(Count);

  delay(1000);  // ~10 Hz
}
