// #include <Arduino.h>
// #include <Wire.h>
// 
// #define MUX_Address 0x70 // TCA9548A Encoders address
// 
// void selectI2CChannels(uint8_t i) {
//   if (i > 7) return;
//   Wire.beginTransmission(MUX_Address);
//   Wire.write(1 << i);
//   Wire.endTransmission();
// }
// 
// void setup() {
//     Serial.begin(9600);
//     while (!Serial);  // Wait for serial connection (for Leonardo & Micro)
//     Serial.println("I2C Scanner running...");
// 
//     Wire.begin();
// }
// 
// void loop() {
//     Serial.println("Scanning for I2C devices...");
//     
//     for (byte address = 3; address <= 0x77; address++) {
//         Wire.beginTransmission(address);
//         if (Wire.endTransmission() == 0) {  // No error = device found
//             Serial.print("Found device at 0x");
//             Serial.println(address, HEX);
//         }
//     }
//     
//     Serial.println("Scan complete.\n");
//     delay(5000);  // Wait 5 seconds before scanning again
// }
// 
// #include <Arduino.h>
// #include <SPI.h>
// #include <Wire.h>
// #include <SimpleFOC.h>
// #include <SimpleFOCDrivers.h>
// 
// // MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t _angle_register_msb)
// //  chip_address         - I2C chip address
// //  bit_resolution       - resolution of the sensor
// //  angle_register_msb   - angle read register msb
// //  bits_used_msb        - number of used bits in msb register
// MagneticSensorI2C as5600 = MagneticSensorI2C(0x36, 12, 0x0E, 4);
// 
// void setup() {
//   // monitoring port
//   Serial.begin(115200);
// 
//   // init magnetic sensor hardware
//   as5600.init();
// 
//   Serial.println("AS5600 ready");
//   _delay(1000);
// }
// 
// void loop() {
//   // IMPORTANT - call as frequently as possible
//   // update the sensor values 
//   as5600.update();
//   // display the angle and the angular velocity to the terminal
//   Serial.print(as5600.getAngle());
//   Serial.print("\t");
//   Serial.println(as5600.getVelocity());
// }