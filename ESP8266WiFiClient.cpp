#include "ESP8266WiFiClient.h"

ESP8266WiFiClient::ESP8266WiFiClient(ESP8266 const* wifi): _wifi(wifi)
{
}

int ESP8266WiFiClient::connect(const char* host, uint16_t port) {
	return _wifi->createTCP(host, port);
}

size_t ESP8266WiFiClient::write(const uint8_t* buf, size_t size) {
	return _wifi->send(buf, size) ? size : 0;
}

size_t ESP8266WiFiClient::write(uint8_t) {
	return 0;
}

int ESP8266WiFiClient::read(uint8_t* buf, size_t size) {
	return _wifi->recv(buf, size, 1000);
}

int ESP8266WiFiClient::read() {
	Serial.println("read()");
	return 0;
}

uint8_t ESP8266WiFiClient::connected() {
	return strstr(_wifi->getIPStatus(), "STATUS:4") == nullptr ? 1 : 0;
}

int ESP8266WiFiClient::connect(IPAddress ip, uint16_t port) {
	return 0;
}

int ESP8266WiFiClient::peek() {
	Serial.println("peek()");
	return 0;
}

void ESP8266WiFiClient::flush() {
	Serial.println("flush()");
}

void ESP8266WiFiClient::stop() {
	_wifi->releaseTCP();
}

int ESP8266WiFiClient::available() {
	return 1;
}

ESP8266WiFiClient::operator bool() {
	return true;
}