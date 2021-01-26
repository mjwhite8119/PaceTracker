#include <Arduino.h>
#include "config.h"
#include "OLED.h"
#include "IMU.h"
#include "Tracker.h"
#include <Wire.h>

// Define Tracker
Tracker tracker;

// ----------------------------------------------------------
// Scan the I2C bus for devices
// ----------------------------------------------------------
void i2cScan() {
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

// ----------------------------------------------------------
// Setup
// ----------------------------------------------------------
void setup() {

  // Setup to write to the serial console for debugging
  Serial.begin(115200); while(!Serial && !Serial.available()){}

  // This is the GPIO pin of the LED that is on the ESP32
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup the OLED.
  #if USE_OLED
    setupOLED();
  #endif

  // Diagnostic test for I2C connectivity
  i2cScan();

  // Initialize the IMU
  tracker.imu.initMPU6050();

  // Get the offset of the gyro.  The gyro takes a few
  // seconds to settle. So wait a little.
  clearDisplayBelowHeader();
  for (int i=9; i > 0; i--) {
    drawText(2, 0, "Settling gyro");
    drawText(2, 90, String(i));
    delay(1000);
  } 

  // Display the gyro offset
  tracker.setGyroOffset();
  clearDisplayBelowHeader();
  drawText(0, 0, "Gyro offset");
  float offset = tracker.getGyroOffset().Degrees();
  drawText(0, 70, String((int)offset));

  // Draw labels for heading
  drawText(1, 0, "Heading");
  drawText(1, 70, "degrees");

}

// ----------------------------------------------------------
// Main loop
// ----------------------------------------------------------
void loop() {

  // Display the current heading adjusted for offset
  Rotation2d heading = tracker.getRotation(); 

  // Subtract the gyro offset
  heading -= tracker.getGyroOffset();
  drawText(1, 50, String((int)heading.Degrees()));

  // Acceleration
  VectorInt16 accel = tracker.getAcceleration();
  drawText(2, 0, String(accel.x));
  drawText(2, 45, String(accel.y));
  drawText(2, 90, String(accel.z));

  delay(100);
  
}