#ifndef WIRELESS_H
#define WIRELESS_H
#include <WiFi.h>
#include <string.h>
#include "MQTT/mqttSerial.h"
#include "Config/config.h"

struct WifiDetails
{
  const String SSID;
  const int32_t RSSI;
  const wifi_auth_mode_t EncryptionType;
};

extern WifiDetails **lastWifiScanResults;
extern int16_t lastWifiScanResultAmount;

void start_standalone_wifi();

void setup_wifi();

void scan_wifi_delete_result();

void scan_wifi();

#endif