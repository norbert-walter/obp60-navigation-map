/// @file    main.cpp
/// @brief   Test fuctions for multi function display OBP60
/// @example main.cpp
/// @author  Norbert Walter
/// @org     Open Boat Projects
/// @licence GPL 2.0

#include <Arduino.h>
#include "OBP60NavigationMap.h" // Include all neccessary libs

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

// SPI pin definitions for E-Ink display class
// GxEPD2_BW<GxEPD2_420_GYE042A87, GxEPD2_420_GYE042A87::HEIGHT> display(GxEPD2_420_GYE042A87(OBP_SPI_CS, OBP_SPI_DC, OBP_SPI_RST, OBP_SPI_BUSY)); // GYE042A87, 400x300, SSD1683 (no inking)
GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> display(GxEPD2_420_GDEY042T81(OBP_SPI_CS, OBP_SPI_DC, OBP_SPI_RST, OBP_SPI_BUSY)); // GDEY042T81, 400x300, SSD1683 (no inking)

// WiFi credentials
const char* ssid = "MySSID";
const char* password = "MyPassword";

int loopCounter = 0;
int angle = 0;

//###############################################################
//
// Setup section
//
//###############################################################

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

  // Initialize E‑Ink display and show logo
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
  Serial.begin(115200); // USB serial port

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

//###############################################################
//
// Loop section
//
//###############################################################

void loop() {
  // Activate tast supervisor
  static TaskHandle_t loopTaskHandle = NULL;
  if (loopTaskHandle == NULL) {
    loopTaskHandle = xTaskGetCurrentTaskHandle();
  }
  
  // Reset start and end time
  unsigned long startTime = 0;
  unsigned long endTime = 0;
  
  // Display rotating angle
  angle = (loopCounter % 360);  // 1° per request
  loopCounter++;

  // Picture size vor navigation map
  int widthPicture = 400;
  int heightPicture = 300;
  int numberOfPixels = 0;
  
  // Load navigation map from OBP Web service 
  numberOfPixels = loadNavigationMap(
    "norbert-walter.dnshome.de",
    80,
    15,
    53.9028,
    11.4441,
    angle,
    9,
    1,
    widthPicture,
    heightPicture,
    0,
    2,
    2,
    15,
    angle,
    1
    );

  // Declare a Binary Image Array for navigation map
  unsigned char imageData_sw[numberOfPixels];
  
  // Extract navigation map as B&W picture and return as Binary Image Array
  imageData_sw[numberOfPixels] = extractNavigationMap(numberOfPixels);

    // Render image
    startTime = millis(); // Start time measurement
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

    // Stack monitoring
    UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(loopTaskHandle);
    Serial.printf("Loop-Task: Free Stack: %u Bytes\n", highWaterMark * sizeof(StackType_t));

//  Serial.println("New request in 3s");
//  delay(3000);
}

