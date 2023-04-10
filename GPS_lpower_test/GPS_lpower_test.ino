// RXM message:
// https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
// p.153

// checksum
// https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
// p.98

#include <TinyGPS++.h>

// Initialize the GPS sensor
TinyGPSPlus gps;
#define GPS_SERIAL Serial1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  GPS_SERIAL.begin(9600);

  // Construct UBX-CFG-RXM message payload
  byte payload[] = {0x02, 0x08, 0x01};

  // the complete message
  byte message[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x08, 0x01};

  byte pm2_message[] = {
    0xB5, 0x62, // Header
    0x06, 0x3B, // Class and ID
    0x2C, // Length of message (44)

    // Payload
    0x00,       // version
    0x00,       // reserved1
    0x00,       // reserved2
    0x00,       // reserved3
    0x42, 0x40, 0x0F, 0x00, // flags
    0x00, 0x00, 0x00, 0x00, // updatePeriod (in ms)
    0x00, 0x00, 0x00, 0x00, // searchPeriod (in ms)
    0x7D, 0x13, 0x00, 0x00, // gridOffset (in ms)
    0x0A, 0x00,             // onTime (in s)
    0x05, 0x00,             // minAcqTime (in s)
    0x00, 0x00,             // reserved4
    0x00, 0x00,             // reserved5
    0x00, 0x00, 0x00, 0x00, // reserved6
    0x00, 0x00, 0x00, 0x00, // reserved7
    0x00,                   // reserved8
    0x00,                   // reserved9
    0x00, 0x00,             // reserved10
    0x00, 0x00, 0x00, 0x00  // reserved11
  };

  // Calculate message checksum
  byte ck_a = 0, ck_b = 0;
  for (int i = 0; i < sizeof(message); i++) {
    ck_a += message[i];
    ck_b += ck_a;
  }

  // Construct full UBX-CFG-RXM message with checksum
  message[sizeof(message)] = ck_a;
  message[sizeof(message) + 1] = ck_b;

  // Calculate pm2_message checksum
  ck_a = 0, ck_b = 0;
  for (int i = 0; i < sizeof(pm2_message); i++) {
    ck_a += pm2_message[i];
    ck_b += ck_a;
  }

  // Construct full UBX-CFG-RXM message with checksum
  pm2_message[sizeof(pm2_message)] = ck_a;
  pm2_message[sizeof(pm2_message) + 1] = ck_b;

  // Send message to GPS module via UART
  for (int i = 0; i < sizeof(message); i++) {
    GPS_SERIAL.write(message[i]);
  }

  delay(1000);

  // Send pm2_message to GPS module via UART
  for (int i = 0; i < sizeof(pm2_message); i++) {
    GPS_SERIAL.write(pm2_message[i]);
  }

  // // Wait for response from GPS module (optional)
  // delay(5000);
  // while (GPS_SERIAL.available() > 0) {
  //   Serial.print((char)GPS_SERIAL.read());
  // }

}

void loop() {
  // Read GPS data
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }

  // Print GPS data if valid
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat());
    Serial.print(" Longitude: ");
    Serial.println(gps.location.lng());
  }
  else {
    Serial.println("error");
  }

  // Wait a bit before next reading
  delay(30000);
}
