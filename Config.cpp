#include "Config.h"

IPAddress apIP(192, 168, 4, 1);
WiFiServer offlineServer(OFFLINE_TCP_PORT);
HardwareSerial neogps(1);
HardwareSerial& modemSerial = Serial2;
TinyGPSPlus gps;
TinyGsm modem(modemSerial); // Use Serial2 for A7680C/SIM7600
TinyGsmClient client(modem);

const char* DEVICE_SERIAL = "H4G-5063-9840-2175";
//TCP Server (Using Google Cloud Instance)
const char* tcpServer = "34.124.143.132";
const int tcpPort = 6111;
const int MAX_TCP_FAILURES = 10;
// // TCP Server (Using Ngrok)
// const char* tcpServer = "18.141.106.224";  // Update with your Ngrok host
// const int tcpPort = 16751;                 // Update with your Ngrok assigned port