/**
 * @file ESP8266.cpp
 * @brief The implementation of class ESP8266. 
 * @author Wu Pengfei<pengfei.wu@itead.cc> 
 * @date 2015.02
 * 
 * @par Copyright:
 * Copyright (c) 2015 ITEAD Intelligent Systems Co., Ltd. \n\n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version. \n\n
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "ESP8266.h"
#include <stdlib.h>
#include <ctype.h>

#define LOG_OUTPUT_DEBUG            (1)
#define LOG_OUTPUT_DEBUG_PREFIX     (0)

#define logDebug(arg)\
    do {\
        if (LOG_OUTPUT_DEBUG)\
        {\
            if (LOG_OUTPUT_DEBUG_PREFIX)\
            {\
                Serial.print("[LOG Debug: ");\
                Serial.print((const char*)__FILE__);\
                Serial.print(",");\
                Serial.print((unsigned int)__LINE__);\
                Serial.print(",");\
                Serial.print((const char*)__FUNCTION__);\
                Serial.print("] ");\
            }\
            Serial.println(arg);\
        }\
    } while(0)

#ifdef ESP8266_USE_SOFTWARE_SERIAL
ESP8266::ESP8266(SoftwareSerial &uart, uint32_t baud): m_puart(&uart)
{
    if (F_CPU == 8000000L) {
        m_puart->begin(baud >> 1);
    }
    else {
        m_puart->begin(baud);
    }
    rx_empty();
}
#else
ESP8266::ESP8266(HardwareSerial &uart, uint32_t baud): m_puart(&uart)
{
    if (F_CPU == 8000000L) {
        m_puart->begin(baud / 2);
    }
    else {
        m_puart->begin(baud);
    }
    rx_empty();
}
#endif

static char ESP8266::_receive_buffer[_ESP_MAX_READ_BUFF];
static char ESP8266::_receive_char;
static bool ESP8266::_dataStarted = false;
static size_t ESP8266::_dataLenth = SIZE_MAX;

size_t ESP8266::write(uint8_t byte)
{
    return m_puart->write(byte);
}

int ESP8266::peek(void)
{
    return m_puart->peek();
}

int ESP8266::read(void)
{
    return m_puart->read();
}

void ESP8266::flush(void)
{
    return m_puart->flush();
}

int ESP8266::available(void)
{
    return m_puart->available();
}

bool ESP8266::kick(void)
{
    return eAT();
}

bool ESP8266::restart(void)
{
    unsigned long start;
    if (eATRST()) {
        delay(2000);
        start = millis();
        while (millis() - start < 3000) {
            if (eAT()) {
                delay(1500); /* Waiting for stable */
                return true;
            }
            delay(100);
        }
    }
    return false;
}

const char* ESP8266::getVersion(void)
{
    const char* version;
    eATGMR(version);
    return version;
}

bool ESP8266::setOprToStation(void)
{
    uint8_t mode;
    if (!qATCWMODE(&mode)) {
        return false;
    }
    if (mode == 1) {
        return true;
    } else {
        if (sATCWMODE(1) && restart()) {
            return true;
        } else {
            return false;
        }
    }
}

bool ESP8266::setOprToSoftAP(void)
{
    uint8_t mode;
    if (!qATCWMODE(&mode)) {
        return false;
    }
    if (mode == 2) {
        return true;
    } else {
        if (sATCWMODE(2) && restart()) {
            return true;
        } else {
            return false;
        }
    }
}

bool  ESP8266::setBaudrate(uint32_t baudrate) {
    return sATUART_CUR(baudrate);
}

bool ESP8266::setOprToStationSoftAP(void)
{
    uint8_t mode;
    if (!qATCWMODE(&mode)) {
        return false;
    }
    if (mode == 3) {
        return true;
    } else {
        if (sATCWMODE(3) && restart()) {
            return true;
        } else {
            return false;
        }
    }
}

const char* ESP8266::getAPList(void)
{
    const char* list;
    eATCWLAP(list);
    return list;
}

bool ESP8266::joinAP(const char* ssid, const char* pwd)
{
    return sATCWJAP(ssid, pwd);
}

bool ESP8266::enableClientDHCP(uint8_t mode, boolean enabled)
{
    return sATCWDHCP(mode, enabled);
}

bool ESP8266::leaveAP(void)
{
    return eATCWQAP();
}

bool ESP8266::setSoftAPParam(const char* ssid, const char* pwd, uint8_t chl, uint8_t ecn)
{
    return sATCWSAP(ssid, pwd, chl, ecn);
}

const char* ESP8266::getJoinedDeviceIP(void)
{
    const char* list;
    eATCWLIF(list);
    return list;
}

const char* ESP8266::getIPStatus(void)
{
    const char* list;
    eATCIPSTATUS(list);
    return list;
}

const char* ESP8266::getLocalIP(void)
{
    const char* list;
    eATCIFSR(list);
    return list;
}

bool ESP8266::enableMUX(void)
{
    return sATCIPMUX(1);
}

bool ESP8266::disableMUX(void)
{
    return sATCIPMUX(0);
}

bool ESP8266::createTCP(const char* addr, uint32_t port)
{
    return sATCIPSTARTSingle("TCP", addr, port);
}

bool ESP8266::releaseTCP(void)
{
    return eATCIPCLOSESingle();
}

bool ESP8266::registerUDP(const char* addr, uint32_t port)
{
    return sATCIPSTARTSingle("UDP", addr, port);
}

bool ESP8266::unregisterUDP(void)
{
    return eATCIPCLOSESingle();
}

bool ESP8266::createTCP(uint8_t mux_id, const char* addr, uint32_t port)
{
    return sATCIPSTARTMultiple(mux_id, "TCP", addr, port);
}

bool ESP8266::releaseTCP(uint8_t mux_id)
{
    return sATCIPCLOSEMulitple(mux_id);
}

bool ESP8266::registerUDP(uint8_t mux_id, const char* addr, uint32_t port)
{
    return sATCIPSTARTMultiple(mux_id, "UDP", addr, port);
}

bool ESP8266::unregisterUDP(uint8_t mux_id)
{
    return sATCIPCLOSEMulitple(mux_id);
}

bool ESP8266::setTCPServerTimeout(uint32_t timeout)
{
    return sATCIPSTO(timeout);
}

bool ESP8266::startTCPServer(uint32_t port)
{
    if (sATCIPSERVER(1, port)) {
        return true;
    }
    return false;
}

bool ESP8266::stopTCPServer(void)
{
    sATCIPSERVER(0);
    restart();
    return false;
}

bool ESP8266::startServer(uint32_t port)
{
    return startTCPServer(port);
}

bool ESP8266::stopServer(void)
{
    return stopTCPServer();
}

bool ESP8266::send(const uint8_t *buffer, uint32_t len)
{
    return sATCIPSENDSingle(buffer, len);
}

bool ESP8266::send(uint8_t mux_id, const uint8_t *buffer, uint32_t len)
{
    return sATCIPSENDMultiple(mux_id, buffer, len);
}

uint32_t ESP8266::recv(uint8_t *buffer, uint32_t buffer_size, uint32_t timeout)
{
    return recvPkg(buffer, buffer_size, NULL, timeout, NULL);
}

uint32_t ESP8266::recv(uint8_t mux_id, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout)
{
    uint8_t id;
    uint32_t ret;
    ret = recvPkg(buffer, buffer_size, NULL, timeout, &id);
    if (ret > 0 && id == mux_id) {
        return ret;
    }
    return 0;
}

uint32_t ESP8266::recv(uint8_t *coming_mux_id, uint8_t *buffer, uint32_t buffer_size, uint32_t timeout)
{
    return recvPkg(buffer, buffer_size, NULL, timeout, coming_mux_id);
}

/*----------------------------------------------------------------------------*/
/* +IPD,<id>,<len>:<data> */
/* +IPD,<len>:<data> */


uint32_t ESP8266::recvPkg(uint8_t* buffer, uint32_t buffer_size, uint32_t* data_len, uint32_t timeout, uint8_t* coming_mux_id)
{
    char infobuff[10];
    bool colonFound = false;
    bool plusFound = false;
    bool infoStarted = false;

    if (buffer == NULL) {
        return 0;
    }
    size_t ii = 0;
    size_t ib = 0;
    uint32_t start = millis();
    while (millis() - start < timeout && ii < buffer_size && ii < _dataLenth) {
        if (m_puart->available() <= 0) {
            continue;
        }
        _receive_char = m_puart->read();
        if (_dataStarted) {
            buffer[ii++] = _receive_char;
        }
        else if (plusFound) {
            if (_receive_char == ':') {
                _dataStarted = true;
                if (infoStarted) {
                    infobuff[ib] = '\0';
                    _dataLenth = atoi(infobuff);
                }
            }
            else if (_receive_char == ',') {
                if (infoStarted && coming_mux_id != nullptr) {
                    infobuff[ib] = '\0';
                    *coming_mux_id = atoi(infobuff);
                }
                else {
                    infoStarted = true;
                    ib = 0;
                }
            }
            else if (infoStarted && ib < sizeof(infobuff) - 1) {
                infobuff[ib++] = _receive_char;
            }
        }
        else if (_receive_char == '+') {
            plusFound = true;
        }
    }

    if (data_len != nullptr) {
        *data_len = _dataLenth;
    }

    if (ii < buffer_size) {
        rx_empty();
    }

    /*
    char str[93];
    memset(str, '\0', 93);
    unsigned char* pin = _receive_buffer;
    const char* hex = "0123456789ABCDEF";
    char* pout = str;
    int i3 = 0;
    for (; i3 < ii && i3 < 30; ++i3) {
        *pout++ = hex[(*pin >> 4) & 0xF];
        *pout++ = hex[(*pin++) & 0xF];
        *pout++ = ':';
    }
    *pout++ = hex[(*pin >> 4) & 0xF];
    *pout++ = hex[(*pin) & 0xF];
    *pout = 0;
    logDebug(str);*/
    return ii;
}


void ESP8266::rx_empty(void) 
{
    _dataStarted = false;
    _dataLenth = SIZE_MAX;
    while(m_puart->available() > 0) {
        m_puart->read();
    }
}

const char* ESP8266::recvconst(const char* target, uint32_t timeout)
{
    const char* stopWord[] = { target };
    return readWithTimeout(timeout, stopWord, 1);
}

const char* ESP8266::readWithTimeout(uint32_t timeout, const char* stopWord[], size_t stopWordCount)
{
    size_t i = 0;
    bool found = false;
    unsigned long start = millis();
    while (i < _ESP_MAX_READ_BUFF && millis() - start < timeout && !found) {
        if (m_puart->available() > 0) {
            _receive_char = m_puart->read();
            if (!isprint(_receive_char) && !isspace(_receive_char)) continue;
            _receive_buffer[i++] = _receive_char;
        } else {
            _receive_buffer[i] = '\0';
            for (size_t j = 0; j < stopWordCount; ++j) {
                if (strstr(_receive_buffer, stopWord[j]) != nullptr) {
                    found = true;
                }
            }
        }
    }
    if (i < _ESP_MAX_READ_BUFF) {
        _receive_buffer[i] = '\0';
    } else {
        _receive_buffer[_ESP_MAX_READ_BUFF - 1] = '\0';
    }
    return _receive_buffer;
}

const char* ESP8266::recvconst(const char* target1, const char* target2, uint32_t timeout)
{
    const char* stopWord[] = { target1, target2 };
    return readWithTimeout(timeout, stopWord, 2);
}

const char* ESP8266::recvconst(const char* target1, const char* target2, const char* target3, uint32_t timeout)
{
    const char* stopWord[] = { target1, target2, target3 };
    return readWithTimeout(timeout, stopWord, 3);
}

bool ESP8266::recvFind(const char* target, uint32_t timeout)
{
    const char* data_tmp;
    data_tmp = recvconst(target, timeout);
    return strstr(data_tmp, target) != nullptr;
}

bool ESP8266::recvFindAndFilter(const char* target, const char* begin, const char* end, const char* &data, uint32_t timeout)
{
    const char* data_tmp = recvconst(target, timeout);
    if (strstr(data_tmp, target) != nullptr) {
        const char* index1 = strstr(data_tmp, begin);
        char* index2 = strstr(data_tmp, end);
        if (index1 != nullptr && index2 != nullptr) {
            data = index1 + strlen(begin);
            *index2 = '\0';
            return true;
        }
    }
    data = nullptr;
    return false;
}

bool ESP8266::eAT(void)
{
    rx_empty();
    m_puart->println("AT");
    return recvFind("OK");
}

bool ESP8266::eATRST(void) 
{
    rx_empty();
    m_puart->println("AT+RST");
    return recvFind("OK");
}

bool ESP8266::eATGMR(const char* &version)
{
    rx_empty();
    m_puart->println("AT+GMR");
    return recvFindAndFilter("OK", "\r\r\n", "\r\nOK", version); 
}

bool ESP8266::qATCWMODE(uint8_t *mode) 
{
    const char* str_mode;
    bool ret;
    if (!mode) {
        return false;
    }
    rx_empty();
    m_puart->println("AT+CWMODE?");
    ret = recvFindAndFilter("OK", "+CWMODE:", "\r\n\r\nOK", str_mode); 
    if (ret) {
        *mode = atoi(str_mode);
        return true;
    } else {
        return false;
    }
}

bool ESP8266::sATCWMODE(uint8_t mode)
{
    const char* data;
    rx_empty();
    m_puart->print("AT+CWMODE=");
    m_puart->println(mode);
    
    data = recvconst("OK", "no change");
    if (strstr(data, "OK") != nullptr || strstr(data, "no change") != nullptr) {
        return true;
    }
    return false;
}

bool ESP8266::sATCWJAP(const char* ssid, const char* pwd)
{
    const char* data;
    rx_empty();
    m_puart->print("AT+CWJAP=\"");
    m_puart->print(ssid);
    m_puart->print("\",\"");
    m_puart->print(pwd);
    m_puart->println("\"");
    
    data = recvconst("OK", "FAIL", 10000);
    if (strstr(data, "OK") != nullptr) {
        return true;
    }
    return false;
}

bool ESP8266::sATCWDHCP(uint8_t mode, boolean enabled)
{
	const char* strEn = "0";
	if (enabled) {
		strEn = "1";
	}
	
	
    const char* data;
    rx_empty();
    m_puart->print("AT+CWDHCP=");
    m_puart->print(strEn);
    m_puart->print(",");
    m_puart->println(mode);
    
    data = recvconst("OK", "FAIL", 10000);
    if (strstr(data, "OK") != nullptr) {
        return true;
    }
    return false;
}

bool ESP8266::eATCWLAP(const char* &list)
{
    const char* data;
    rx_empty();
    m_puart->println("AT+CWLAP");
    return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list, 10000);
}

bool ESP8266::eATCWQAP(void)
{
    const char* data;
    rx_empty();
    m_puart->println("AT+CWQAP");
    return recvFind("OK");
}

bool ESP8266::sATCWSAP(const char* ssid, const char* pwd, uint8_t chl, uint8_t ecn)
{
    const char* data;
    rx_empty();
    m_puart->print("AT+CWSAP=\"");
    m_puart->print(ssid);
    m_puart->print("\",\"");
    m_puart->print(pwd);
    m_puart->print("\",");
    m_puart->print(chl);
    m_puart->print(",");
    m_puart->println(ecn);
    
    data = recvconst("OK", "ERROR", 5000);
    if (strstr(data, "OK") != nullptr) {
        return true;
    }
    return false;
}

bool ESP8266::sATUART_CUR(uint32_t baudrate, uint8_t databits, uint8_t stopbits, uint8_t parity, uint8_t flow_control)
{
    const char* data;
    rx_empty();
    m_puart->print("AT+UART_CUR=");
    m_puart->print(baudrate);
    m_puart->print(",");
    m_puart->print(databits);
    m_puart->print(",");
    m_puart->print(stopbits);
    m_puart->print(",");
    m_puart->print(parity);
    m_puart->print(",");
    m_puart->println(flow_control);
    if (F_CPU == 8000000L) {
        m_puart->begin(baudrate >> 1);
    }
    else {
        m_puart->begin(baudrate);
    }
    return true;
}

bool ESP8266::eATCWLIF(const char* &list)
{
    const char* data;
    rx_empty();
    m_puart->println("AT+CWLIF");
    return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list);
}
bool ESP8266::eATCIPSTATUS(const char* &list)
{
    delay(100);
    rx_empty();
    m_puart->println("AT+CIPSTATUS");
    return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list);
}
bool ESP8266::sATCIPSTARTSingle(const char* type, const char* addr, uint32_t port)
{
    const char* data;
    rx_empty();
    m_puart->print("AT+CIPSTART=\"");
    m_puart->print(type);
    m_puart->print("\",\"");
    m_puart->print(addr);
    m_puart->print("\",");
    m_puart->println(port);
    
    data = recvconst("OK", "ERROR", "ALREADY CONNECT", 10000);
    if (strstr(data, "OK") != nullptr  || strstr(data, "ALREADY CONNECT") != nullptr) {
        return true;
    }
    return false;
}
bool ESP8266::sATCIPSTARTMultiple(uint8_t mux_id, const char* type, const char* addr, uint32_t port)
{
    const char* data;
    rx_empty();
    m_puart->print("AT+CIPSTART=");
    m_puart->print(mux_id);
    m_puart->print(",\"");
    m_puart->print(type);
    m_puart->print("\",\"");
    m_puart->print(addr);
    m_puart->print("\",");
    m_puart->println(port);
    
    data = recvconst("OK", "ERROR", "ALREADY CONNECT", 10000);
    if (strstr(data, "OK") != nullptr  || strstr(data, "ALREADY CONNECT") != nullptr) {
        return true;
    }
    return false;
}
bool ESP8266::sATCIPSENDSingle(const uint8_t *buffer, uint32_t len)
{
    rx_empty();
    m_puart->print("AT+CIPSEND=");
    m_puart->println(len);
    if (recvFind(">", 2000)) {
        rx_empty();
        for (uint32_t i = 0; i < len; i++) {
            m_puart->write(buffer[i]);
        }
        return recvFind("SEND OK", 500);
    }
    return false;
}
bool ESP8266::sATCIPSENDMultiple(uint8_t mux_id, const uint8_t *buffer, uint32_t len)
{
    rx_empty();
    m_puart->print("AT+CIPSEND=");
    m_puart->print(mux_id);
    m_puart->print(",");
    m_puart->println(len);
    if (recvFind(">", 5000)) {
        rx_empty();
        for (uint32_t i = 0; i < len; i++) {
            m_puart->write(buffer[i]);
        }
        return recvFind("SEND OK", 10000);
    }
    return false;
}
bool ESP8266::sATCIPCLOSEMulitple(uint8_t mux_id)
{
    const char* data;
    rx_empty();
    m_puart->print("AT+CIPCLOSE=");
    m_puart->println(mux_id);
    
    data = recvconst("OK", "link is not", 5000);
    if (strstr(data, "OK") != nullptr  || strstr(data, "link is not") != nullptr) {
        return true;
    }
    return false;
}
bool ESP8266::eATCIPCLOSESingle(void)
{
    rx_empty();
    m_puart->println("AT+CIPCLOSE");
    return recvFind("OK", 5000);
}
bool ESP8266::eATCIFSR(const char* &list)
{
    rx_empty();
    m_puart->println("AT+CIFSR");
    return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", list);
}
bool ESP8266::sATCIPMUX(uint8_t mode)
{
    const char* data;
    rx_empty();
    m_puart->print("AT+CIPMUX=");
    m_puart->println(mode);
    
    data = recvconst("OK", "Link is builded");
    if (strstr(data, "OK") != nullptr) {
        return true;
    }
    return false;
}
bool ESP8266::sATCIPSERVER(uint8_t mode, uint32_t port)
{
    const char* data;
    if (mode) {
        rx_empty();
        m_puart->print("AT+CIPSERVER=1,");
        m_puart->println(port);
        
        data = recvconst("OK", "no change");
        if (strstr(data, "OK") != nullptr  || strstr(data, "no change") != nullptr) {
            return true;
        }
        return false;
    } else {
        rx_empty();
        m_puart->println("AT+CIPSERVER=0");
        return recvFind("\r\r\n");
    }
}
bool ESP8266::sATCIPSTO(uint32_t timeout)
{
    rx_empty();
    m_puart->print("AT+CIPSTO=");
    m_puart->println(timeout);
    return recvFind("OK");
}

