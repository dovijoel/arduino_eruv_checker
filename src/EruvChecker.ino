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
#include <MultiPrint.h>

MultiPrint multiPrinter;

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true

uint32_t timer = millis();

AsyncWebServer server(80);

const char *ssid = "Flamingos also for sale here"; // WiFi AP SSID
const char *password = "0723697875";               // WiFi AP Password

std::vector<std::vector<Coordinate>> eruvs;
std::vector<std::vector<Coordinate>> exclusions;

bool isInEruv = false;

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
    multiPrinter.println("Failed to open file for reading");
    return;
  }

  // Allocate a buffer to store the JSON file
  StaticJsonDocument<16384> doc; // Adjust size based on JSON complexity
  DeserializationError error = deserializeJson(doc, file);
  if (error)
  {
    multiPrinter.print("Failed to parse JSON: ");
    multiPrinter.println(error.c_str());
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

  multiPrinter.println("Eruv polygons parsed successfully.");
}

// the values in the NMEA sentences are in the format DDMM.MMMM
// where DD is degrees and MM.MMMM is minutes
// this function converts that to decimal degrees
// for example, 1234.5678 becomes 12.5763 degrees
// 1234.5678 = 12 degrees + 34.5678 minutes
// 12 degrees + (34.5678/60) = 12.5763 degrees
float decimalDegrees(float nmeaCoord)
{
  uint16_t wholeDegrees = 0.01 * nmeaCoord;
  return wholeDegrees + (nmeaCoord - 100.0 * wholeDegrees) / 60.0;
}

bool nmeaCheck(String nmea)
{
  if (nmea.length() > 0)
  {
    // Parse the NMEA string
    multiPrinter.println(nmea);
    bool isInEruv = false;
    char nmeaCharArray[nmea.length() + 1];
    nmea.toCharArray(nmeaCharArray, nmea.length() + 1);
    if (GPS.parse(nmeaCharArray))
    { // Parse the NMEA sentence
      if (GPS.fix)
      { // Check if GPS has a valid fix, keeping true for debugging
        multiPrinter.print("Location: ");
        multiPrinter.print(GPS.latitude, 4);
        multiPrinter.print(GPS.lat);
        multiPrinter.print(", ");
        multiPrinter.print(GPS.longitude, 4);
        multiPrinter.println(GPS.lon);
        multiPrinter.print("Speed (knots): ");
        multiPrinter.println(GPS.speed);
        multiPrinter.print("Angle: ");
        multiPrinter.println(GPS.angle);
        multiPrinter.print("Altitude: ");
        multiPrinter.println(GPS.altitude);
        Coordinate currentLocation = {decimalDegrees(GPS.latitude), decimalDegrees(GPS.longitude)};
        if (GPS.lat == 'S')
        {
          currentLocation.latitude = -currentLocation.latitude;
        }
        if (GPS.lon == 'W')
        {
          currentLocation.longitude = -currentLocation.longitude;
        }
        multiPrinter.print("Current Location: ");
        multiPrinter.print(currentLocation.latitude);
        multiPrinter.print(", ");
        multiPrinter.println(currentLocation.longitude);

        // Check if the current location is inside the eruv
        multiPrinter.println("Number of eruvs: " + String(eruvs.size()));
        multiPrinter.println("Number of exclusions: " + String(exclusions.size()));
        if (isPointInEruv(eruvs, exclusions, currentLocation, WebSerial))
        {
          isInEruv = true;
          multiPrinter.println("Inside the eruv.");
        }
        else
        {
          multiPrinter.println("Outside the eruv.");
        }
      }
      else
      {
        // multiPrinter.println("No GPS fix.");
      }
    }
    else
    {
      // multiPrinter.println("Failed to parse NMEA string.");
    }
  }
  else
  {
    multiPrinter.println("Empty");
  }
}

void setup()
{
  // while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  multiPrinter.println("Starting eruv checker.");

  // set up led pins
  pinMode(18, OUTPUT);
  if (!SPIFFS.begin()) {
      multiPrinter.println("Card Mount Failed");
      return;
    }
  multiPrinter.println("Starting eruv checker.");
  ;
  if (!SPIFFS.begin())
  {
    multiPrinter.println("Card Mount Failed");
    return;
  }
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
    multiPrinter.printf("WiFi Failed!\n");
    // return;
  }

  // Once connected, print IP
  multiPrinter.print("IP Address: ");
  multiPrinter.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hi! This is WebSerial demo. You can access webserial interface at http://" + WiFi.localIP().toString() + "/webserial"); });

  // WebSerial is accessible at "<IP Address>/webserial" in browser
  multiPrinter.begin(&server);

  WebSerial.onMessage(nmeaCheck);

  // Start server
  server.begin();
}

void loop()
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c)
      multiPrinter.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived())
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    char *nmea = GPS.lastNMEA();
    if (nmea != NULL)
    {
      multiPrinter.println(nmea);
    }
    else
    {
      multiPrinter.println("NMEA is NULL");
    }

    // if (!GPS.parse(nmea)) // this also sets the newNMEAreceived() flag to false
    //   return;             // we can fail to parse a sentence in which case we should just wait for another

    if (strstr(nmea, "GGA") != NULL)
    {
      isInEruv = nmeaCheck(nmea); // this also sets the newNMEAreceived() flag to false
    }
  }
  digitalWrite(19, isInEruv);
  digitalWrite(18, !isInEruv);
  // sleep(2);
}
