#include <ETH.h>
#include <WebSocketsClient.h>
#include <ESPmDNS.h>

WebSocketsClient webSocket;
const char* serverHostname = "server.local";
const uint16_t serverPort = 8080;
const char* uri = "/";

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_BIN) {
        Serial.print("Received binary data: ");
        for (size_t i = 0; i < length; i++) {
            Serial.printf("%02X ", payload[i]);
        }
        Serial.println();
    }
}

void setup() {
    Serial.begin(115200);
    ETH.begin();

    while (!ETH.linkUp()) {
        delay(1000);
        Serial.println("Waiting for Ethernet...");
    }

    if (!MDNS.begin("esp32")) {
        Serial.println("Error starting mDNS");
        while (1) delay(1000);
    }

    IPAddress serverIP = MDNS.queryHost(serverHostname);
    if (serverIP == INADDR_NONE) {
        Serial.println("Failed to resolve server hostname");
        while (1) delay(1000);
    }

    webSocket.begin(serverIP.toString().c_str(), serverPort, uri);
    webSocket.onEvent(webSocketEvent);
}

void loop() {
    webSocket.loop();
    static uint32_t lastSendTime = 0;
    if (millis() - lastSendTime > 2000) {
        lastSendTime = millis();
        uint8_t binaryData[] = {0x01, 0x02, 0x03, 0x04};
        webSocket.sendBIN(binaryData, sizeof(binaryData));
    }
}