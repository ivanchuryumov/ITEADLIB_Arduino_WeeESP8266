#include "ESP8266WiFiClient.h"
#include <stdio.h>

ESP8266WiFiClient::ESP8266WiFiClient(ESP8266 const* wifi): _wifi(wifi)
{
}

int ESP8266WiFiClient::connect(const char* host, uint16_t port) {
	return _wifi->createTCP(host, port);
}

size_t ESP8266WiFiClient::write(const uint8_t* buf, size_t size) {
	return _wifi->send(buf, size) ? size : 0;
}

size_t ESP8266WiFiClient::write(uint8_t val) {
	return _wifi->write(val);
}

int ESP8266WiFiClient::read(uint8_t* buf, size_t size) {
	return _wifi->recv(buf, size, 1000);
}

int ESP8266WiFiClient::read() {
	return _wifi->read();
}

uint8_t ESP8266WiFiClient::connected() {
	return strstr(_wifi->getIPStatus(), "STATUS:4") == nullptr ? 1 : 0;
}

int ESP8266WiFiClient::connect(IPAddress ip, uint16_t port) {
	static const char ipstr[16];
	sprintf(ipstr, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
	return this->connect(ipstr, port);
}

int ESP8266WiFiClient::peek() {
	return _wifi->peek();
}

void ESP8266WiFiClient::flush() {
	_wifi->flush();
}

void ESP8266WiFiClient::stop() {
	_wifi->releaseTCP();
}

int ESP8266WiFiClient::available() {
	return _wifi->available();
}

ESP8266WiFiClient::operator bool() {
	return _wifi != nullptr;
}