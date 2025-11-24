#include "NetworkClient.h"
#include <HTTPClient.h>
#include <mbedtls/base64.h>
extern "C" {
  #include "puff.h"            // Library for gzip (Library supports only DEFLATE)
}

// Construktor
NetworkClient::NetworkClient(size_t jsonBufferSize)
    : _doc(jsonBufferSize), _jsonBufferSize(jsonBufferSize), _valid(false)
{
}

//  GZIP Hheader parser
int NetworkClient::skipGzipHeader(const uint8_t* data, size_t len) {
    if (len < 10) return -1;  // Invalid
    if (data[0] != 0x1F || data[1] != 0x8B || data[2] != 0x08) return -2;  // No gzip / no DEFLATE

    int pos = 10;  // Base header
    uint8_t flags = data[3];

    if (flags & 0x04) {  // FEXTRA
        if (pos + 2 > len) return -3;
        uint16_t xlen = data[pos] + (data[pos + 1] << 8);
        pos += 2 + xlen;
    }
    if (flags & 0x08) {  // FNAME
        while (pos < len && data[pos] != 0) pos++;
        pos++;
    }
    if (flags & 0x10) {  // FCOMMENT
        while (pos < len && data[pos] != 0) pos++;
        pos++;
    }
    if (flags & 0x02) pos += 2;  // FHCRC

    if (pos >= len) return -4;

    return pos;  // Start position of DEFLATE data stream
}

//  HTTP GET with GZIP loading function
bool NetworkClient::httpGetGzip(String url, uint8_t*& outData, size_t& outLen) {
    HTTPClient http;
    WiFiClient client;

    http.begin(client, url);
    http.addHeader("Accept-Encoding", "gzip");

    int httpCode = http.GET();
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("HTTP ERROR: %d\n", httpCode);
        return false;
    }

    WiFiClient* stream = http.getStreamPtr();
    
    const size_t COMPRESSED_MAX_SIZE = _jsonBufferSize / 2;
    uint8_t* compressed = new uint8_t[COMPRESSED_MAX_SIZE];
    size_t compLen = 0;

    // Read stream
    while (http.connected() && stream->available() && compLen < COMPRESSED_MAX_SIZE) {
        int c = stream->read();
        if (c < 0) break;
        compressed[compLen++] = uint8_t(c);
    }

    http.end();

    outData = compressed;
    outLen = compLen;

    return true;
}

// Read content from URL deflate gzip and parse JSON content
bool NetworkClient::fetchAndDecompressJson(const String& url) {
    _valid = false;

    uint8_t* compressed = nullptr;
    size_t compressedLen = 0;

    // Load gzip
    if (!httpGetGzip(url, compressed, compressedLen)) {
        return false;
    }

    // Analyze gzip header and set the correct position for DEFLATE
    int deflatePos = skipGzipHeader(compressed, compressedLen);
    if (deflatePos < 0) {
        Serial.printf("Invalid GZIP header: %d\n", deflatePos);
        delete[] compressed;
        return false;
    }

    // Deflate the gzip content
    uint8_t* uncompressed = new uint8_t[_jsonBufferSize];
    unsigned long outLen = _jsonBufferSize;
    unsigned long srcLen = compressedLen - deflatePos;

    int result = puff(
        uncompressed, &outLen,
        compressed + deflatePos, &srcLen
    );

    delete[] compressed;

    if (result != 0) {
        Serial.printf("puff() ERR: %d\n", result);
        delete[] uncompressed;
        return false;
    }

    // Parse JSON
    auto err = deserializeJson(_doc, uncompressed, outLen);
    delete[] uncompressed;

    if (err) {
        Serial.printf("JSON parse error: %s\n", err.c_str());
        return false;
    }

    _valid = true;
    return true;
}

// ---------------------------------------------------------
DynamicJsonDocument& NetworkClient::json() {
    return _doc;
}

bool NetworkClient::isValid() const {
    return _valid;
}
