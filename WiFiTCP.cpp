#include "WiFiTCP.h"

WiFiTCP::WiFiTCP() : server(nullptr), _port(0) {}

void WiFiTCP::begin(const char* ssid, const char* password, uint16_t port) {
    _port = port;
    WiFi.softAP(ssid, password);
    server = new WiFiServer(_port);
    server->begin();
}

void WiFiTCP::handleClient(String dataToSend) {
    if (!client || !client.connected()) {
        client = server->available();
    }
    if (client && client.connected()) {
        client.println(dataToSend);
    }
}

String WiFiTCP::getCommand() {
    if (!client || !client.connected()) {
        client = server->available();
    }
    if (client && client.connected() && client.available()) {
        return client.readStringUntil('\n');
    }
    return "";
}

bool WiFiTCP::isClientConnected() {
    return client && client.connected();
}

void WiFiTCP::disconnectClient() {
    if (client && client.connected()) {
        client.stop();
    }
}
