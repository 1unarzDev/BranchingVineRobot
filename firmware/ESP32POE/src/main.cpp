// Using Wireshark, it has been confirmed that this script is funtional
// TODO: Automatically determine computer IP, receive IP from DHCP, improve packet data storage

#include <ETH.h>
#include <WiFi.h>

const char* serverIP = "192.168.1.236"; 
const int serverPort = 2718;

WiFiClient client;

void sendData(const uint8_t* data, size_t length) {
  if (client.connect(serverIP, serverPort)) {
    Serial.println("Connected to server.");
    client.write(data, length);
    Serial.println("Data sent.");
    client.stop();
  } else {
    Serial.println("Failed to connect to server.");
  }
}

void setup() {
  Serial.begin(115200);
  ETH.begin();
  ETH.config(IPAddress(192, 168, 1, 100), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));

  // Wait for Ethernet connection
  Serial.println("Waiting for Ethernet connection...");
  while (!ETH.linkUp()) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nEthernet connected!");

  // Print the IP address
  Serial.println("IP Address: " + ETH.localIP().toString());
}

void loop() {
  // Collect your data
  uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
  size_t dataLength = sizeof(data);

  // Send the data
  sendData(data, dataLength);

  delay(1000); // Adjust delay as needed
}