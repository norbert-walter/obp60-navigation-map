#ifndef _OBP60NAVIGATIONMAP_H
#define _OBP60NAVIGATIONMAP_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"   // Needed for task supervision
#include "freertos/task.h"       // Needed for task supervision
#include "mbedtls/base64.h"      // Library for Base64 encoding/decoding
#include <Wire.h>                // I2C
#include <GxEPD2_BW.h>           // E-Ink display
#include <WiFi.h>                // WiFi connection
#include <HTTPClient.h>          // HTTP connection
#include <ArduinoJson.h>         // JSON parser

extern "C" {
  #include "puff.h"              // Library for gzip (Library supports only DEFLATE)
}

// Limits
#define MAX_DELAY 1500      // Max delay for WiFi data receiving in ms
#define JSON_BUFFER 30000   // Max buffer size for JSON content (30 kB picture + values)

// Important for valid navigation map receiving
SET_LOOP_TASK_STACK_SIZE(24 * 1024); // 24 KB stack size for loop task

// JSON document for received data
DynamicJsonDocument doc(JSON_BUFFER);

// Analyze function GZIP-Header  (only very simple parser)
int skipGzipHeader(const uint8_t* data, size_t len);

// Load navigation map from OBP Web service 
int loadNavigationMap(
    String baseURL = "norbert-walter.dnshome.de",
    int port = 80,
    int zoomLevel = 15,
    float latitude = 53.9028,
    float longitude = 11.4441,
    int angle = 0,
    int mapType = 9,
    int ditheringType = 1,
    int width = 400,
    int height = 300,
    int cutout = 0,
    int border = 2,
    int symbol = 2,
    int symbolSize = 15,
    int symbolRotation = 0,
    int grid = 1
    );

// Extract navigation map as B&W picture and return as Binary Image Array
// The Binary Image Array can be direct display with grafic commands
unsigned char extractNavigationMap(int numberOfPixels);

#endif
