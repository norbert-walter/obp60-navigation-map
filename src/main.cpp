/// @file    main.cpp
/// @brief   Test fuctions for multi function display OBP60
/// @example main.cpp
/// @author  Norbert Walter
/// @org     Open Boat Projects
/// @licence GPL 2.0

#include <Arduino.h>
#include "freertos/FreeRTOS.h"  // Needed for task supervision
#include "freertos/task.h"      // Needed for task supervision
#include "mbedtls/base64.h"     // Library for Base64 encoding/decoding
#include <Wire.h>               // I2C
#include <GxEPD2_BW.h>          // E-Ink display
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

extern "C" {
  #include "puff.h"            // Library for gzip (Library supports only DEFLATE)
}

// FreeFonts from Adafruit_GFX
#include "Ubuntu_Bold8pt7b.h"
#include "Ubuntu_Bold12pt7b.h"
#include "Ubuntu_Bold16pt7b.h"
#include "Ubuntu_Bold20pt7b.h"
// OBP logo
#include "Logo_OBP_400x300_sw.h"
#include "50x50_sw.h"
#include "400x300_sw.h"

#ifdef BOARD_OBP60S3
  // E-Ink pin definition OPB60
  #define OBP_SPI_CS 39     // CS
  #define OBP_SPI_DC 40     // DC
  #define OBP_SPI_RST 41    // RST
  #define OBP_SPI_BUSY 42   // BUSY
  #define OBP_SPI_CLK 38    // CLK
  #define OBP_SPI_DIN 48    // DIN
  #define GxEPD_WIDTH 400   // Display width
  #define GxEPD_HEIGHT 300  // Display height
#endif

#ifdef BOARD_OBP40S3
  // E-Ink pin definition OPB40
  #define OBP_SPI_CS 45     // CS
  #define OBP_SPI_DC 46     // DC
  #define OBP_SPI_RST 47    // RST
  #define OBP_SPI_BUSY 48   // BUSY
  #define OBP_SPI_CLK 12    // CLK
  #define OBP_SPI_DIN 11    // DIN
  #define GxEPD_WIDTH 400   // Display width
  #define GxEPD_HEIGHT 300  // Display height
#endif

// Limits
#define MAX_DELAY 1500      // Max delay for data receiving in ms
#define JSON_BUFFER 30000   // Max buffer size for JSON content (30 kB picture + values)

SET_LOOP_TASK_STACK_SIZE(24 * 1024); // 24 KB stack size for loop task

// SPI pin definitions for E-Ink display class
// GxEPD2_BW<GxEPD2_420_GYE042A87, GxEPD2_420_GYE042A87::HEIGHT> display(GxEPD2_420_GYE042A87(OBP_SPI_CS, OBP_SPI_DC, OBP_SPI_RST, OBP_SPI_BUSY)); // GYE042A87, 400x300, SSD1683 (no inking)
GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> display(GxEPD2_420_GDEY042T81(OBP_SPI_CS, OBP_SPI_DC, OBP_SPI_RST, OBP_SPI_BUSY)); // GDEY042T81, 400x300, SSD1683 (no inking)

// WiFi credentials
const char* ssid = "MySSID";
const char* password = "MyPassword";

// JSON document for received data
DynamicJsonDocument doc(JSON_BUFFER);
bool jsonValid = false;

int loopCounter = 0;
int angle = 0;

// Analyze function GZIP-Header  (only very simple parser)
int skipGzipHeader(const uint8_t* data, size_t len) {
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

  if (pos >= len) return -4;  // Header incomplete
  return pos;  // Start position of DEFLATE data stream
}

void setup() {

  // Initialize digital pins
  delay(1000);
  #ifdef BOARD_OBP60S3
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);  //Power line on for 5V and 3.3V       
    delay(100);
  #endif

  #ifdef BOARD_OBP40S3
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);  //Power e-paper display
    pinMode(41, OUTPUT);
    digitalWrite(41, HIGH);  //Power LED     
    delay(100);
  #endif

  // Initialize E‑Ink display
  display.init(115200);
  display.setTextColor(GxEPD_BLACK);
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  display.fillScreen(GxEPD_WHITE);
  display.nextPage();
  display.drawBitmap(0, 0, gImage_Logo_OBP_400x300_sw, display.width(), display.height(), GxEPD_BLACK);
  display.nextPage();

  // Initialize serial ports
  Serial.begin(115200);                     // USB serial port

  //WiFi login
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("WiFi connected with: ");
  Serial.println(ssid);

  // Ready to start
  tone(16, 4000); // Buzzer GPIO16 4kHz
  delay(200);     // Duration 200ms
  noTone(16);     // Disable beep
  Serial.println("Boot Beep 4kHz");
}

void loop() {
  static TaskHandle_t loopTaskHandle = NULL;
  if (loopTaskHandle == NULL) {
    loopTaskHandle = xTaskGetCurrentTaskHandle();
  }

  unsigned long startTime = 0;
  unsigned long endTime = 0;

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    WiFiClient client;

    // Display rotating map
    angle = (loopCounter % 360);  // 1° per request

    // URL sample
    // http://192.168.1.67:8080/get_image_json?zoom=15&lat=53.9028&lon=11.4441&mrot=10&mtype=9&dtype=1&width=400&height=300&debug=1
    // URL for OBP map service
    String url = "http://norbert-walter.dnshome.de/get_image_json?zoom=15&lat=53.9028&lon=11.4441&mrot=" + String(angle) + "&mtype=9&dtype=1&width=400&height=250&debug=1";
    // URL for self hosted map service (needs the port 8080)
    // String url = "http://192.168.1.67:8080/get_image_json?zoom=15&lat=53.9028&lon=11.4441&mrot=" + String(angle) + "&mtype=9&dtype=1&width=400&height=250&debug=1";

    loopCounter++;

    Serial.printf("Start HTTP request 0 ms\n");
    startTime = millis(); // Start time measurement
    
    http.begin(client, url);
    http.addHeader("Accept-Encoding", "gzip");

    // Start HTTP request
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK && (millis() - startTime < MAX_DELAY)) {
      WiFiClient* stream = http.getStreamPtr();

      const size_t COMPRESSED_MAX_SIZE = JSON_BUFFER / 2;
      uint8_t* compressedData = new uint8_t[COMPRESSED_MAX_SIZE];
      size_t compressedLength = 0;

      // Read Gzip response
      unsigned long lastTime = millis();
      int blockwise = 0;
      while (http.connected() && stream->available() && compressedLength < COMPRESSED_MAX_SIZE) {
        if(blockwise == 1){
          // Read stream block by block (Is helpully for slow or critical connections.
          // However increases the read time to 5000 ms because it use many tiny blocks.)
          size_t readLen = stream->readBytes(compressedData + compressedLength, 512); // or 1024 Byte
          if (readLen == 0) break;
          compressedLength += readLen;
        }
        else{
          // Read directly as a complete block
          compressedData[compressedLength++] = stream->read();
          if (compressedLength % 1024 == 0) {
            Serial.printf("%u Bytes in %lu ms\n", compressedLength, millis() - lastTime);
            lastTime = millis();
          }
        }
        
      }
      endTime = millis() - startTime;
      Serial.printf("gzip answare (%d Byte) received in %lu ms\n",compressedLength, endTime);

      // Find gzip header end (on this oisition beginsDEFLATE)
      int deflateStart = skipGzipHeader(compressedData, compressedLength);
      if (deflateStart < 0) {
        Serial.printf("Unvalid gzip header: Code %d\n", deflateStart);
        delete[] compressedData;
        return;
      }

      // Skip gzip header and decompress DEFLATE part
      uint8_t* uncompressedData = new uint8_t[JSON_BUFFER];
      unsigned long uncompressedSize = JSON_BUFFER;
      unsigned long sourceLen = compressedLength - deflateStart;

      int result = puff(uncompressedData, &uncompressedSize,
                        compressedData + deflateStart, &sourceLen);
      delete[] compressedData;
      endTime = millis() - startTime;
      Serial.printf("gzip answare (%d Byte) unpacked in %lu ms\n",uncompressedSize, endTime);

      // If decompression was successful, parse the JSON string
      if (result == 0) {
        DeserializationError error = deserializeJson(doc, uncompressedData, uncompressedSize);
        delete[] uncompressedData;

        // If no Error on parsing then is JSON content usable
        if (!error) {
          jsonValid = true;
          endTime = millis() - startTime;
          Serial.printf("JSON read in %lu ms.\n", endTime);
//          Serial.println(doc.as<String>().substring(0, 200));
        } else {
          jsonValid = false;
          Serial.print("JSON error after unpacked gzip: ");
          Serial.println(error.c_str());
        }
      } else {
        jsonValid = false;
        Serial.printf("Unpack error (puff): Code %d\n", result);
      }
    } else {
      jsonValid = false;
      endTime = millis() - startTime;
      Serial.printf("HTTP error: %d (in %lu ms)\n", httpCode, endTime);
    }

    http.end();
  }

  // Set read values to defaults
  int numPix = 0;
  float lat = 0;
  float lon = 0;
  int widthPicture = 0;
  int heightPicture = 0;
  int mapType = 0;
  int rotAngle = 0;

  // Read and display JSON contents
  if (jsonValid) {
    if (doc.containsKey("number_pixels")) {
      numPix = doc["number_pixels"];
      Serial.printf("Num Pixel: %d\n", numPix);
    }
    if (doc.containsKey("latitude")) {
      lat = doc["latitude"];
      Serial.printf("Latitude: %.4f\n", lat);
    }
    if (doc.containsKey("longitude")) {
      lon = doc["longitude"];
      Serial.printf("Longitude: %.4f\n", lon);String b64 = doc["bytes_b64"];
    }
    if (doc.containsKey("width")) {
      widthPicture = doc["width"];
      Serial.printf("X: %d\n", widthPicture);
    }
    if (doc.containsKey("height")) {
      heightPicture = doc["height"];
      Serial.printf("Y: %d\n", heightPicture);
    }
    if (doc.containsKey("map_type")) {
      mapType = doc["map_type"];
      Serial.printf("Map Type: %d\n", mapType);
    }
    if (doc.containsKey("rotation_angle")) {
      rotAngle = doc["rotation_angle"];
      Serial.printf("Rot Angle: %d\n", rotAngle);
    }

    // Convert JSON image data (Base64) to image e-Paper data
    unsigned char imageData_sw[numPix];   // Binary Image Array
    if (doc.containsKey("picture_base64")) {
      String bytesBase64 = doc["picture_base64"];
      //Serial.println(bytesBase64.substring(0, 200));

      size_t decodedLen = 0;
      // Decoding with mbedtls_base64_decode
      int ret = mbedtls_base64_decode(
        imageData_sw,                       // Destination buffer
        sizeof(imageData_sw),               // Maximum size
        &decodedLen,                        // Actual size after decoding
        (const unsigned char*)bytesBase64.c_str(),  // Input Base64 string
        bytesBase64.length()
      );  
    }
    endTime = millis() - startTime;
    Serial.printf("Picture build in %lu ms\n", endTime);

    // Render image
    if((loopCounter % 60) == 0){  // After 60 images full refresh
      display.setFullWindow();    // Set full update
/*
        display.fillScreen(GxEPD_BLACK);
        display.drawBitmap(0, 0, imageData_sw, widthPicture, heightPicture, GxEPD_WHITE);
        display.nextPage();
*/
    }
    else{
      display.setPartialWindow(0, 25, widthPicture, heightPicture); // Set partial update
    }
    display.fillScreen(GxEPD_WHITE);
    display.drawBitmap(0, 25, imageData_sw, widthPicture, heightPicture, GxEPD_BLACK);
//    display.drawBitmap(0, 0, gImage_50x50_sw, 50, 50, GxEPD_BLACK);
//    display.drawBitmap(0, 0, gImage_400x300_sw, 400, 300, GxEPD_BLACK);
    display.nextPage();

    endTime = millis() - startTime;
    Serial.printf("Picture outpt in %lu ms\n", endTime);
  }

  // Stack monitoring
  UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(loopTaskHandle);
  Serial.printf("Loop-Task: Free Stack: %u Bytes\n", highWaterMark * sizeof(StackType_t));

//  Serial.println("New request in 3s");
//  delay(3000);
}

