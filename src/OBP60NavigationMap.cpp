#if defined BOARD_OBP60S3 || defined BOARD_OBP40S3

#include <OBP60NavigationMap.h>

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

// Load navigation map from OBP Web service 
int loadNavigationMap(
    String baseURL,
    int port,
    int zoomLevel,
    float latitude,
    float longitude,
    int angle,
    int mapType,
    int ditheringType,
    int width,
    int height,
    int cutout,
    int border,
    int symbol,
    int symbolSize,
    int symbolRotation,
    int grid
    ){
  
  // Reset start and end time
  unsigned long startTime = 0;
  unsigned long endTime = 0;

  // Reset number of pixels for navigation map
  int numberOfPixels = 0;
  bool jsonValid = false;
  
  // If WiFi connection established then get the navigation map
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    WiFiClient client;

    String url = "http://norbert-walter.dnshome.de/get_image_json?zoom=15&lat=53.9028&lon=11.4441&mrot=" + String(angle) + "&mtype=9&dtype=1&width=400&height=250&cutout=0&border=2&symbol=2&ssize=15&srot=" + String(angle) + "&grid=1";
    
    Serial.printf("Start HTTP request 0 ms\n");
    startTime = millis(); // Start time measurement
    
    http.begin(client, url);
    http.addHeader("Accept-Encoding", "gzip");  // Accept gzip content

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

      // Find gzip header end (on this poisition begins DEFLATE)
      int deflateStart = skipGzipHeader(compressedData, compressedLength);
      if (deflateStart < 0) {
        Serial.printf("Unvalid gzip header: Code %d\n", deflateStart);
        delete[] compressedData;
        return 0;
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

    if(jsonValid == true){
        numberOfPixels = width * height;
    }
    else{
        numberOfPixels = 0;
    }
    return numberOfPixels;
  }
}

// Extract navigation map as B&W picture and return as Binary Image Array
// The Binary Image Array can be direct display with grafic commands
unsigned char extractNavigationMap(int numberOfPixels){

  // Reset start and end time
  unsigned long startTime = millis(); // Start time measurement
  unsigned long endTime = 0;
  
  // Set read values to defaults
  int numPix = 0;
  float lat = 0;
  float lon = 0;
  int widthPicture = 0;
  int heightPicture = 0;
  int mapType = 0;
  int rotAngle = 0;

  // Read and display JSON contents
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

    // Return the navigation map as Binary Image Array
    return imageData_sw[numPix];

}

#endif
