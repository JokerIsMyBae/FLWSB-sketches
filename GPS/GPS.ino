#include <TinyGPS++.h>

// Initialize the GPS sensor
TinyGPSPlus gps;
#define GPS_SERIAL Serial1

void setup() {
  // Start the serial communication
  Serial.begin(9600);
  GPS_SERIAL.begin(9600);

  // Wait for GPS to start up
  delay(1000);
}

void loop() {
  // Read GPS data
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }

  // Print GPS data if valid
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Longitude: ");
    Serial.println(gps.location.lng(), 6);
  }
  else {
    Serial.println("error");
  }

  // Wait a bit before next reading
  delay(3000);
}
