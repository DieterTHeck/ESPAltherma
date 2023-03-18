#include "mqtt.h"

WiFiClient espClient;
PubSubClient client(espClient);

void sendValues()
{
  Serial.printf("Sending values in MQTT.\n");
#ifdef ARDUINO_M5Stick_C
  //Add M5 APX values
  snprintf(jsonbuff + strlen(jsonbuff),MAX_MSG_SIZE - strlen(jsonbuff) , "\"%s\":\"%.3gV\",\"%s\":\"%gmA\",", "M5VIN", M5.Axp.GetVinVoltage(),"M5AmpIn", M5.Axp.GetVinCurrent());
  snprintf(jsonbuff + strlen(jsonbuff),MAX_MSG_SIZE - strlen(jsonbuff) , "\"%s\":\"%.3gV\",\"%s\":\"%gmA\",", "M5BatV", M5.Axp.GetBatVoltage(),"M5BatCur", M5.Axp.GetBatCurrent());
  snprintf(jsonbuff + strlen(jsonbuff),MAX_MSG_SIZE - strlen(jsonbuff) , "\"%s\":\"%.3gmW\",", "M5BatPwr", M5.Axp.GetBatPower());
#endif
  snprintf(jsonbuff + strlen(jsonbuff),MAX_MSG_SIZE - strlen(jsonbuff) , "\"%s\":\"%ddBm\",", "WifiRSSI", WiFi.RSSI());

  jsonbuff[strlen(jsonbuff) - 1] = '}';

  if(config->MQTT_USE_JSONTABLE)
    strcat(jsonbuff,"]");

  client.publish(MQTT_attr, jsonbuff);

  if(config->MQTT_USE_JSONTABLE)
    strcpy(jsonbuff, "[{\0");
  else
    strcpy(jsonbuff, "{\0");
}

void reconnect()
{
  //in case loopback as server is set, skip connecting (debug purpose)
  if(config->MQTT_SERVER.compareTo("127.0.0.1") == 0 || config->MQTT_SERVER.compareTo( "localhost") == 0)
  {
    mqttSerial.print("Found loopback MQTT server, skiping connection...\n");
    return;
  }

  // Loop until we're reconnected
  int i = 0;
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...\n");

    if (client.connect("ESPAltherma-dev", config->MQTT_USERNAME.c_str(), config->MQTT_PASSWORD.c_str(), MQTT_lwt, 0, true, "Offline"))
    {
      Serial.println("connected!");
      client.publish("homeassistant/sensor/espAltherma/config", "{\"name\":\"AlthermaSensors\",\"stat_t\":\"~/STATESENS\",\"avty_t\":\"~/LWT\",\"pl_avail\":\"Online\",\"pl_not_avail\":\"Offline\",\"uniq_id\":\"espaltherma\",\"device\":{\"identifiers\":[\"ESPAltherma\"]}, \"~\":\"espaltherma\",\"json_attr_t\":\"~/ATTR\"}", true);
      client.publish(MQTT_lwt, "Online", true);
      client.publish("homeassistant/switch/espAltherma/config", "{\"name\":\"Altherma\",\"cmd_t\":\"~/POWER\",\"stat_t\":\"~/STATE\",\"pl_off\":\"OFF\",\"pl_on\":\"ON\",\"~\":\"espaltherma\"}", true);

      // Subscribe
      client.subscribe("espaltherma/POWER");

      if(config->SG_ENABLED)
      {
        client.publish("homeassistant/sg/espAltherma/config", "{\"name\":\"AlthermaSmartGrid\",\"cmd_t\":\"~/set\",\"stat_t\":\"~/state\",\"~\":\"espaltherma/sg\"}", true);
        client.subscribe("espaltherma/sg/set");
      }
    }
    else
    {
      Serial.printf("failed, rc=%d, try again in 5 seconds", client.state());
      unsigned long start = millis();
      while (millis() < start + 5000) { }

      if (i++ == 100)
      {
        Serial.printf("Tried for 500 sec, rebooting now.");
        esp_restart();
      }
    }
  }
}

void callbackTherm(byte *payload, unsigned int length)
{
  payload[length] = '\0';

  // Is it ON or OFF?
  // Ok I'm not super proud of this, but it works :p
  if (payload[1] == 'F')
  { //turn off
    digitalWrite(config->PIN_THERM, HIGH);
    savePersistence(HIGH);
    client.publish("espaltherma/STATE", "OFF");
    mqttSerial.println("Turned OFF");
  }
  else if (payload[1] == 'N')
  { //turn on
    digitalWrite(config->PIN_THERM, LOW);
    savePersistence(LOW);
    client.publish("espaltherma/STATE", "ON");
    mqttSerial.println("Turned ON");
  }
  else if (payload[0] == 'R')//R(eset/eboot)
  {
    mqttSerial.println("Rebooting");
    delay(100);
    esp_restart();
  }
  else
  {
    Serial.printf("Unknown message: %s\n", payload);
  }
}

//Smartgrid callbacks
void callbackSg(byte *payload, unsigned int length)
{
  payload[length] = '\0';

  if (payload[0] == '0')
  {
    // Set SG 0 mode => SG1 = INACTIVE, SG2 = INACTIVE
    digitalWrite(config->PIN_SG1, SG_RELAY_INACTIVE_STATE);
    digitalWrite(config->PIN_SG2, SG_RELAY_INACTIVE_STATE);
    client.publish("espaltherma/sg/state", "0");
    Serial.println("Set SG mode to 0 - Normal operation");
  }
  else if (payload[0] == '1')
  {
    // Set SG 1 mode => SG1 = INACTIVE, SG2 = ACTIVE
    digitalWrite(config->PIN_SG1, SG_RELAY_INACTIVE_STATE);
    digitalWrite(config->PIN_SG2, SG_RELAY_ACTIVE_STATE);
    client.publish("espaltherma/sg/state", "1");
    Serial.println("Set SG mode to 1 - Forced OFF");
  }
  else if (payload[0] == '2')
  {
    // Set SG 2 mode => SG1 = ACTIVE, SG2 = INACTIVE
    digitalWrite(config->PIN_SG1, SG_RELAY_ACTIVE_STATE);
    digitalWrite(config->PIN_SG2, SG_RELAY_INACTIVE_STATE);
    client.publish("espaltherma/sg/state", "2");
    Serial.println("Set SG mode to 2 - Recommended ON");
  }
  else if (payload[0] == '3')
  {
    // Set SG 3 mode => SG1 = ACTIVE, SG2 = ACTIVE
    digitalWrite(config->PIN_SG1, SG_RELAY_ACTIVE_STATE);
    digitalWrite(config->PIN_SG2, SG_RELAY_ACTIVE_STATE);
    client.publish("espaltherma/sg/state", "3");
    Serial.println("Set SG mode to 3 - Forced ON");
  }
  else
  {
    Serial.printf("Unknown message: %s\n", payload);
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.printf("Message arrived [%s] : %s\n", topic, payload);

  if (strcmp(topic, "espaltherma/POWER") == 0)
  {
    callbackTherm(payload, length);
  }
  else if (config->SG_ENABLED && strcmp(topic, "espaltherma/sg/set") == 0)
  {
    callbackSg(payload, length);
  }
  else
  {
    Serial.printf("Unknown topic: %s\n", topic);
  }
}