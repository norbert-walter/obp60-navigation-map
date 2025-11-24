#pragma once
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HTTPClient.h>

class NetworkClient {
public:
    NetworkClient(size_t jsonBufferSize);

    bool fetchAndDecompressJson(const String& url);
    DynamicJsonDocument& json();
    bool isValid() const;

private:
    DynamicJsonDocument _doc;
    size_t _jsonBufferSize;
    bool _valid;

    int skipGzipHeader(const uint8_t* data, size_t len);
    bool httpGetGzip(String url, uint8_t*& outData, size_t& outLen);
};

