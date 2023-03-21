#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <LittleFS.h>
#include "parameterDef.hpp"
#include "commandDef.hpp"
#include "ArduinoJson.h"

#define CONFIG_FILE "/config.json"
#define MODELS_CONFIG_SIZE 1024*10

struct Config
{
    bool configStored;
    bool STANDALONE_WIFI;
    String SSID;
    String SSID_PASSWORD;
    bool SSID_STATIC_IP;
    String SSID_IP;
    String SSID_SUBNET;
    String SSID_GATEWAY;
    String SSID_PRIMARY_DNS;
    String SSID_SECONDARY_DNS;
    String MQTT_SERVER;
    String MQTT_USERNAME;
    String MQTT_PASSWORD;
    bool MQTT_USE_JSONTABLE;
    bool MQTT_USE_ONETOPIC;
    String MQTT_TOPIC_NAME;
    String MQTT_ONETOPIC_NAME;
    uint16_t MQTT_PORT;
    uint32_t FREQUENCY;
    uint8_t PIN_RX;
    uint8_t PIN_TX;
    uint8_t PIN_HEATING;
    uint8_t PIN_COOLING;
    bool SG_ENABLED;
    uint8_t PIN_SG1;
    uint8_t PIN_SG2;
    bool SG_RELAY_HIGH_TRIGGER;
    bool CAN_ENABLED;
    uint8_t PIN_CAN_RX;
    uint8_t PIN_CAN_TX;
    uint8_t CAN_SPEED_KBPS;
    uint8_t PIN_ENABLE_CONFIG;
    size_t PARAMETERS_LENGTH;
    ParameterDef** PARAMETERS;
    size_t COMMANDS_LENGTH;
    CommandDef** COMMANDS;
    char* WEBUI_SELECTION_VALUES;

    ~Config();
};

extern Config* config;

void readConfig();

void saveConfig();

#endif