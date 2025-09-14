#ifndef WIFITCP_H
#define WIFITCP_H

#include <WiFi.h>

class WiFiTCP {
public:
    WiFiTCP();
    void begin(const char* ssid, const char* password, uint16_t port);
    void handleClient(String dataToSend);
    String getCommand();
    bool isClientConnected();
    void disconnectClient();
private:
    WiFiServer* server;
    WiFiClient client;
    uint16_t _port;
};

#endif // WIFITCP_H
