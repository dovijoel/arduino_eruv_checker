#include <Adafruit_GPS.h>
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include <Eruv.h>
#include <vector>

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

AsyncWebServer server(80);

const char *ssid = "Joel Wifi";                       // WiFi AP SSID
const char *password = "passwordisstillnotapassword"; // WiFi AP Password

std::vector<std::vector<Coordinate>> eruvs;
std::vector<std::vector<Coordinate>> exclusions;

// Function to convert a JSON array to a vector of Coordinates
std::vector<Coordinate> rawJsonArrayToPolygon(JsonArray jsonArray)
{
  std::vector<Coordinate> polygon;
  for (size_t i = 0; i < jsonArray.size(); i++)
  {
    Coordinate coord;
    coord.latitude = jsonArray[i][1];
    coord.longitude = jsonArray[i][0];
    polygon.push_back(coord);
  }
  return polygon;
}

// Function to parse JSON and populate Coordinate vectors
void parseEruvJson()
{
  File file = SPIFFS.open("/eruv.json", "r");
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  // Allocate a buffer to store the JSON file
  StaticJsonDocument<16384> doc; // Adjust size based on JSON complexity
  DeserializationError error = deserializeJson(doc, file);
  if (error)
  {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    file.close();
    return;
  }
  file.close();

  // Parse "eruvs" polygons
  JsonArray raw_eruvs = doc["eruvs"];
  eruvs.clear(); // Clear any existing data
  for (JsonArray polygon : raw_eruvs)
  {
    eruvs.push_back(rawJsonArrayToPolygon(polygon));
  }

  // Parse "eruvExclusionAreas" polygons
  JsonArray eruvExclusionAreas = doc["eruvExclusionAreas"];
  exclusions.clear(); // Clear any existing data
  for (JsonArray polygon : eruvExclusionAreas)
  {
    exclusions.push_back(rawJsonArrayToPolygon(polygon));
  }

  Serial.println("Eruv polygons parsed successfully.");
}


// the values in the NMEA sentences are in the format DDMM.MMMM
// where DD is degrees and MM.MMMM is minutes
// this function converts that to decimal degrees
// for example, 1234.5678 becomes 12.5763 degrees
// 1234.5678 = 12 degrees + 34.5678 minutes
// 12 degrees + (34.5678/60) = 12.5763 degrees
float decimalDegrees(float nmeaCoord) {
  uint16_t wholeDegrees = 0.01*nmeaCoord;
  return wholeDegrees + (nmeaCoord - 100.0*wholeDegrees)/60.0;
}

void nmeaReceived(uint8_t *data, size_t len)
{
  WebSerial.println("Received Data...");
   String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  if (d.length() > 0) {
          WebSerial.println("Received NMEA string: " + d);

          // Parse the NMEA string
          char nmeaCharArray[d.length() + 1];
          d.toCharArray(nmeaCharArray, d.length() + 1);
          if (GPS.parse(nmeaCharArray)) { // Parse the NMEA sentence
            if (true) { // Check if GPS has a valid fix
              WebSerial.print("Location: ");
              WebSerial.print(GPS.latitude, 4);
              WebSerial.print(GPS.lat);
              WebSerial.print(", ");
              WebSerial.print(GPS.longitude, 4);
              WebSerial.println(GPS.lon);
              WebSerial.print("Speed (knots): ");
              WebSerial.println(GPS.speed);
              WebSerial.print("Angle: ");
              WebSerial.println(GPS.angle);
              WebSerial.print("Altitude: ");
              WebSerial.println(GPS.altitude);
              Coordinate currentLocation = {decimalDegrees(GPS.latitude), decimalDegrees(GPS.longitude)};
              if (GPS.lat == 'S') {
                currentLocation.latitude = -currentLocation.latitude;
              }
              if (GPS.lon == 'W') {
                currentLocation.longitude = -currentLocation.longitude;
              }
              WebSerial.print("Current Location: ");
              WebSerial.print(currentLocation.latitude);
              WebSerial.print(", ");
              WebSerial.println(currentLocation.longitude);
              
              // Check if the current location is inside the eruv
              WebSerial.println("Number of eruvs: " + String(eruvs.size()));
              WebSerial.println("Number of exclusions: " + String(exclusions.size()));
              if (isPointInEruv(eruvs, exclusions, currentLocation, WebSerial)) {
                WebSerial.println("Inside the eruv.");
              } else {
                WebSerial.println("Outside the eruv.");
              }
            } else {
              WebSerial.println("No GPS fix.");
            }
          } else {
            WebSerial.println("Failed to parse NMEA string.");
          }

          // Clear the buffer for the next sentence
          d = "";
        }
      } 

void setup()
{
  // while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Starting eruv checker.");

  parseEruvJson();

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(0x10); // The I2C address to use is 0x10
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPS.println(PMTK_Q_RELEASE);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.printf("WiFi Failed!\n");
    return;
  }

  // Once connected, print IP
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hi! This is WebSerial demo. You can access webserial interface at http://" + WiFi.localIP().toString() + "/webserial"); });

  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);

  WebSerial.onMessage(nmeaReceived);

  // Start server
  server.begin();
}

void loop() // run over and over again
{
  // // read data from the GPS in the 'main loop'
  // char c = GPS.read();
  // // if you want to debug, this is a good time to do it!
  // if (GPSECHO)
  //   if (c)
  //     WebSerial.print(c);
  // // if a sentence is received, we can check the checksum, parse it...
  // if (GPS.newNMEAreceived())
  // {
  //   // a tricky thing here is if we print the NMEA sentence, or data
  //   // we end up not listening and catching other sentences!
  //   // so be very wary if using OUTPUT_ALLDATA and trying to print out data
  //   WebSerial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
  //   if (!GPS.parse(GPS.lastNMEA()))    // this also sets the newNMEAreceived() flag to false
  //     return;                          // we can fail to parse a sentence in which case we should just wait for another
  // }

  // // approximately every 2 seconds or so, print out the current stats
  // if (millis() - timer > 2000)
  // {
  //   timer = millis(); // reset the timer
  // WebSerial.print("\nTime: ");
  // if (GPS.hour < 10)
  // {
  //   WebSerial.print('0');
  // }
  // WebSerial.print(GPS.hour, DEC);
  // WebSerial.print(':');
  // if (GPS.minute < 10)
  // {
  //   WebSerial.print('0');
  // }
  // WebSerial.print(GPS.minute, DEC);
  // WebSerial.print(':');
  // if (GPS.seconds < 10)
  // {
  //   WebSerial.print('0');
  // }
  // WebSerial.print(GPS.seconds, DEC);
  // WebSerial.print('.');
  // if (GPS.milliseconds < 10)
  // {
  //   WebSerial.print("00");
  // }
  // else if (GPS.milliseconds > 9 && GPS.milliseconds < 100)
  // {
  //   WebSerial.print("0");
  // }
  // WebSerial.println(GPS.milliseconds);
  // WebSerial.print("Date: ");
  // WebSerial.print(GPS.day, DEC);
  // WebSerial.print('/');
  // WebSerial.print(GPS.month, DEC);
  // WebSerial.print("/20");
  // WebSerial.println(GPS.year, DEC);
  // WebSerial.print("Fix: ");
  // WebSerial.print((int)GPS.fix);
  // WebSerial.print(" quality: ");
  // WebSerial.println((int)GPS.fixquality);
  // if (GPS.fix)
  // {
  //   WebSerial.print("Location: ");
  //   WebSerial.print(GPS.latitude, 4);
  //   WebSerial.print(GPS.lat);
  //   WebSerial.print(", ");
  //   WebSerial.print(GPS.longitude, 4);
  //   WebSerial.println(GPS.lon);
  //   WebSerial.print("Speed (knots): ");
  //   WebSerial.println(GPS.speed);
  //   WebSerial.print("Angle: ");
  //   WebSerial.println(GPS.angle);
  //   WebSerial.print("Altitude: ");
  //   WebSerial.println(GPS.altitude);
  //   WebSerial.print("Satellites: ");
  //   WebSerial.println((int)GPS.satellites);
  // }
  // }
}