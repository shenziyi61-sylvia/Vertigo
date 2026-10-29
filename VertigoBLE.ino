// Board: Seeed nRF52 mbed-enabled Boards -> Seeed XIAO BLE Sense.
// Libraries: ArduinoBLE and Seeed Arduino LSM6DS3.
#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <math.h>

LSM6DS3 imu(I2C_MODE, 0x6A);
BLEService service("743f1000-42b0-4c30-9e5d-760be3e72100");
BLECharacteristic sample("743f1001-42b0-4c30-9e5d-760be3e72100", BLERead | BLENotify, 16, true);
uint32_t lastSample = 0;

void setup() {
  Serial.begin(115200);
  // No while(!Serial): must boot on battery without a computer.
  imu.settings.gyroRange = 245;
  imu.settings.accelRange = 2;
  if (imu.begin() != 0 || !BLE.begin()) {
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) {
      digitalWrite(LED_BUILTIN, LOW); delay(150);
      digitalWrite(LED_BUILTIN, HIGH); delay(150);
    }
  }
  BLE.setLocalName("VertigoSense");
  BLE.setAdvertisedService(service);
  service.addCharacteristic(sample);
  BLE.addService(service);
  BLE.advertise();
}

void loop() {
  BLE.poll();
  uint32_t now = micros();
  if ((uint32_t)(now - lastSample) < 20000) return; // 50 Hz
  lastSample = now;
  if (!BLE.connected() || !sample.subscribed()) return;
  float values[6] = {imu.readFloatGyroX(), imu.readFloatGyroY(), imu.readFloatGyroZ(),
                     imu.readFloatAccelX(), imu.readFloatAccelY(), imu.readFloatAccelZ()};
  uint8_t packet[16];
  for (int i = 0; i < 4; ++i) packet[i] = (now >> (8*i)) & 0xff;
  for (int i = 0; i < 6; ++i) {
    if (!isfinite(values[i])) return;
    int16_t v = (int16_t)lroundf(values[i] * (i < 3 ? 100.0f : 1000.0f));
    packet[4 + 2*i] = (uint16_t)v & 0xff;
    packet[5 + 2*i] = ((uint16_t)v >> 8) & 0xff;
  }
  sample.writeValue(packet, sizeof(packet));
}
