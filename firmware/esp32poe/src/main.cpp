// Using Wireshark, it has been confirmed that this script is funtional
// TODO: Automatically determine computer IP, receive IP from DHCP, improve packet data storage

#include <SPI.h>
#include <ETH.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define DEBUG true

// Setup local IMU at I2C address 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Setup server/client
const char* serverIP = "192.168.1.236"; 
const int serverPort = 2718;

WiFiClient client;

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorStatus(void)
{
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void displayCalStatus(void)
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void sendData(sensors_event_t* event)
{
  double x, y, z, header;

  if (event->type == SENSOR_TYPE_ORIENTATION)
  {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    header = 0; // 0 for orientation
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION)
  {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    header = 1; // 1 for linear acceleration
  }
  else
  {
    x = -99999;
    y = -99999;
    z = -99999;
  }

  // Convert float values to byte array
  uint8_t data[25]; // 8 bytes per float * 3 doubles + 1 header
  data[0] = header;
  memcpy(&data[1], &x, sizeof(double));
  memcpy(&data[9], &y, sizeof(double));
  memcpy(&data[17], &z, sizeof(double));

  if (DEBUG) 
  {
    // Send the data to the server
    if (client.connect(serverIP, serverPort)) 
    {
      Serial.println("Connected to server.");
      client.write(data, sizeof(data));
      Serial.println("Orientation data sent.");
      client.stop();
    } 
    else 
    {
      Serial.println("Failed to connect to server.");
    }
  }
}

void setup(void) 
{
  ETH.begin();
  ETH.config(IPAddress(192, 168, 1, 100), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));

  if (DEBUG) 
  {
    Serial.begin(115200);
    while (!Serial) delay(100);  // wait for serial port to open
    
    if(!bno.begin())
    {
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    
    delay(1000);
    
    // Display debugging info
    displaySensorDetails();
    displaySensorStatus();

    // Wait for Ethernet connection
    Serial.println("Waiting for Ethernet connection...");
    while (!ETH.linkUp()) 
    {
      delay(100);
      Serial.print(".");
    }
    Serial.println("\nEthernet connected!");

    // Print the IP address
    Serial.println("IP Address: " + ETH.localIP().toString());
  }
  else 
  {
    while (!ETH.linkUp()) delay(100);
  }
  
  // Configure BNO055
  bno.setExtCrystalUse(true);
}

void loop(void) 
{
  // Get new IMU events
  sensors_event_t orientation, linearAccel;
  bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccel, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Package the data into a byte array to send over the network
  sendData(&orientation);
  sendData(&linearAccel);
  
  if(DEBUG) displayCalStatus();

  // Wait for new data from IMU
  delay(BNO055_SAMPLERATE_DELAY_MS); 
}