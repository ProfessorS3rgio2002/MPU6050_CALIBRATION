#ifndef CONFIG_H
#define CONFIG_H
#define FIRMWARE_VERSION "1.0.0"
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024
extern const char* DEVICE_SERIAL;

#include <TinyGSM.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>


#define OFFLINE_TCP_PORT 5000

// #define RXD2 16 
// #define TXD2 17

#define RXD2 17 
#define TXD2 16

// #define RXD_SIM800 5
// #define TXD_SIM800 4
#define RXD_SIM800 4
#define TXD_SIM800 5

// #define IGNITION_SWITCH 25
#define IGNITION_SWITCH 33
#define THEFT_RELAY_PIN 26

// #define IGNITION_SWITCH 25
// #define THEFT_RELAY_PIN 26

#define LED_BUILTIN 2  // ESP32 built-in LED pin (usually pin 2)


extern IPAddress apIP;
extern WiFiServer offlineServer;
extern HardwareSerial neogps;
// extern SoftwareSerial sim800l;
extern TinyGPSPlus gps;
extern HardwareSerial& modemSerial;
extern TinyGsm modem;
extern TinyGsmClient client;
extern const char* tcpServer;
extern const int tcpPort;
extern const int MAX_TCP_FAILURES;
extern int keepAliveInterval;

#endif

// Wiring of the Device

// SIM800L
// RX = 5
// TX = 4

// GPS MODULE
// RX = 16 - 17
// TX = 17 - 16

// MPU6050 
// SCL = 22
// SDA = 21

// THEFT RELAY 
// PIN = 26

// Ignition Switch 
// PIN = 32
