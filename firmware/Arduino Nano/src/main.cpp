#include <Arduino.h>
#include <Wire.h>

#define MUX_Address 0x70 // TCA9548A Encoders address

void selectI2CChannels(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(MUX_Address);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
    Serial.begin(9600);
    while (!Serial);  // Wait for serial connection (for Leonardo & Micro)
    Serial.println("I2C Scanner running...");

    Wire.begin();
}

void loop() {
    Serial.println("Scanning for I2C devices...");
    
    for (byte address = 3; address <= 0x77; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {  // No error = device found
            Serial.print("Found device at 0x");
            Serial.println(address, HEX);
        }
    }
    
    Serial.println("Scan complete.\n");
    delay(5000);  // Wait 5 seconds before scanning again
}