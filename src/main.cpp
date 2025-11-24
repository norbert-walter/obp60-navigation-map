/// @file    main.cpp
/// @brief   Test fuctions for multi function display OBP60
/// @example main.cpp
/// @author  Norbert Walter
/// @org     Open Boat Projects
/// @licence GPL 2.0

#include <Arduino.h>
//#include <WiFi.h>
#include <Wire.h>               // I2C
#include <GxEPD2_BW.h>          // E-Ink display
#include "NetworkClient.h"
#include "ImageDecoder.h"

// FreeFonts from Adafruit_GFX
#include "Ubuntu_Bold8pt7b.h"
#include "Ubuntu_Bold12pt7b.h"
#include "Ubuntu_Bold16pt7b.h"
#include "Ubuntu_Bold20pt7b.h"
// OBP logo
#include "Logo_OBP_400x300_sw.h"

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

NetworkClient net(JSON_BUFFER);
ImageDecoder decoder;

// SPI pin definitions for E-Ink display class
// GxEPD2_BW<GxEPD2_420_GYE042A87, GxEPD2_420_GYE042A87::HEIGHT> display(GxEPD2_420_GYE042A87(OBP_SPI_CS, OBP_SPI_DC, OBP_SPI_RST, OBP_SPI_BUSY)); // GYE042A87, 400x300, SSD1683 (no inking)
GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> display(GxEPD2_420_GDEY042T81(OBP_SPI_CS, OBP_SPI_DC, OBP_SPI_RST, OBP_SPI_BUSY)); // GDEY042T81, 400x300, SSD1683 (no inking)

// WiFi credentials
const char* ssid = "MySSID";
const char* password = "MyPassword";

int loopCounter = 0;
int angle = 0;

//################################################
//
// Setup section
//
//################################################

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

//################################################
//
// Loop section
//
//################################################

void loop() {
    // Rotate he picture in 1° steps
    int angle = loopCounter % 360;

    // URL to OBP Maps Converter
    String url = String("http://norbert-walter.dnshome.de/get_image_json?")
                 + "zoom=15&lat=53.9028&lon=11.4441&mrot=" + angle +
                   "&mtype=9&dtype=1&width=400&height=250";

    // If a network connection to URL
    if (net.fetchAndDecompressJson(url)) {        // Connect to URL and read gzip answare and deflate JSON content
        auto& json = net.json();                  // Parse JSON content
        int numPix = json["number_pixels"] | 0;   // Read number of picture pixels
        String b64 = json["picture_base64"] | ""; // Read the Base64 bit steram content (picture)
        static uint8_t imageData[400 * 300];      // Set picture buffer
        size_t decodedSize = 0;                   // Reset decoded size of Basse64 bit stream content

        decoder.decodeBase64(b64, imageData, sizeof(imageData), decodedSize); // Decode Basse64 bit stream content

        // Render image
        // After 60 images full refresh
        if((loopCounter % 60) == 0){
          display.setFullWindow(); 
        }
        // Otherwise partial refresh
        else{
          display.setPartialWindow(0, 25, display.width(), display.height());
        }
        display.fillScreen(GxEPD_WHITE);  // Clear screen
        display.drawBitmap(0, 25, imageData, display.width(), display.height(), GxEPD_BLACK); // Show picture with Y offset 25 pixel
        display.nextPage();
        }

    loopCounter++;  
}

