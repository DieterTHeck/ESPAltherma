#ifndef MQTT_SERIAL_H
#define MQTT_SERIAL_H

#include <PubSubClient.h>
#include <WiFi.h>
#include <WebSerialLite.h>
#include "Stream.h"
#include "ArduinoC.hpp"

class MQTTSerial: public Stream
{
public:
    inline void begin() {};

    inline size_t write(uint8_t)
    {
        return 0;
    };

    inline int available(void)
    {
        return 0;
    };

    inline int availableForWrite(void)
    {
        return 0;
    };

    inline int peek(void)
    {
        return 0;
    };

    inline int read(void)
    {
        return 0;
    };

    inline void flush(void){};

    inline size_t write(const char * s)
    {
        return write((uint8_t*) s, strlen(s));
    }

    inline size_t write(unsigned long n)
    {
        return write((uint8_t) n);
    }

    inline size_t write(long n)
    {
        return write((uint8_t) n);
    }

    inline size_t write(unsigned int n)
    {
        return write((uint8_t) n);
    }

    inline size_t write(int n)
    {
        return write((uint8_t) n);
    }

    MQTTSerial();
    size_t write(const uint8_t *buffer, size_t size);
    ~MQTTSerial();
};

extern MQTTSerial mqttSerial;

#endif