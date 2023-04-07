#include <TinyGPS++.h>

// Initialize the GPS sensor
TinyGPSPlus gps;
#define gpsSerial Serial1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  gpsSerial.begin(9600);

  // Send command to set the GPS module in power save mode
  gpsSerial.println("$PUBX,41,1,0003,0001,0,0,0,0*25\r\n");

  // Wait for the GPS module to enter power save mode
  delay(1000);
}

void loop() {
  // exit sleep mode
  gpsSerial.println("$PUBX,41,0,0003,0000,0,0,0,0*27\r\n");

  // Read GPS data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Print GPS data if valid
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat());
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng());
  }
  else {
    Serial.println("Could not read GPS data");
  }

  // Put the GPS module in sleep mode for 10 seconds
  gpsSerial.println("$PUBX,41,1,0003,0000,0,0,0,0*26\r\n");
  Serial.println("GPS sleep");
  delay(10000);
}

