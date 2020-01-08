#ifndef __ESP8266WIFICLIENT_H__
#define __ESP8266WIFICLIENT_H__

#include <ESP8266.h>
#include <Client.h>

class ESP8266WiFiClient : public Client {
public:
    ESP8266WiFiClient(ESP8266 const* wifi);
    virtual int connect(IPAddress ip, uint16_t port);
    virtual int connect(const char* host, uint16_t port);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t* buf, size_t size);
    virtual int available();
    virtual int read();
    virtual int read(uint8_t* buf, size_t size);
    virtual int peek();
    virtual void flush();
    virtual void stop();
    virtual uint8_t connected();
    virtual operator bool();
private:
    ESP8266 const* _wifi;
};
#endif