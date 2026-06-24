/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete instructions at https://RandomNerdTutorials.com/esp32-neo-m8n-gps-logger-google-earth/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/

#include <TinyGPS++.h>

// Define the RX and TX pins for Serial 2
#define RXD2 20
#define TXD2 21

#define GPS_BAUD 9600

// The TinyGPS++ object
TinyGPSPlus gps;

// Create an instance of the HardwareSerial class for Serial 1
HardwareSerial gpsSerial(1);

void setup() {
  // Serial Monitor
  Serial.begin(115200);

  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial 2 started at 9600 baud rate");
}

// void loop() {
//   // This sketch displays information every time a new sentence is correctly encoded.
//   unsigned long start = millis();

//   while (millis() - start < 1000) {
//     // Serial.println ("inside while");
//     while (gpsSerial.available() > 0) {
//       char gpsData = gpsSerial.read();
//       // Serial.print(gpsData);

//       gps.encode(gpsData);
//     }
//     if (gps.location.isUpdated()) {
//       Serial.print("LAT: ");
//       Serial.println(gps.location.lat(), 6);
//       Serial.print("LONG: ");
//       Serial.println(gps.location.lng(), 6);
//       Serial.print("SPEED (km/h) = ");
//       Serial.println(gps.speed.kmph());
//       Serial.print("ALT (min)= ");
//       Serial.println(gps.altitude.meters());
//       Serial.print("HDOP = ");
//       Serial.println(gps.hdop.value() / 100.0);
//       Serial.print("Satellites = ");
//       Serial.println(gps.satellites.value());
//       Serial.print("Time in UTC: ");
//       Serial.println(String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
//       Serial.println("");
//     }
//   }
// }

void loop() {
  while (gpsSerial.available() > 0) {
    char gpsData = gpsSerial.read();

    // gps.encode() returns true when a full sentence is successfully parsed
    if (gps.encode(gpsData)) {

      // Check if the parsed sentence actually contains a valid location
      if (gps.location.isValid()) {

        // Check if it's a NEW valid location
        if (gps.location.isUpdated()) {
          Serial.print("LAT: ");
          Serial.println(gps.location.lat(), 6);
          Serial.print("LONG: ");
          Serial.println(gps.location.lng(), 6);
          Serial.print("SPEED (km/h) = ");
          Serial.println(gps.speed.kmph());
          Serial.print("ALT (min)= ");
          Serial.println(gps.altitude.meters());
          Serial.print("HDOP = ");
          Serial.println(gps.hdop.value() / 100.0);
          Serial.print("Satellites = ");
          Serial.println(gps.satellites.value());
          Serial.print("Time in UTC: ");
          Serial.println(String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
          Serial.println("");
//
        }

      } else {
        // This will print if data is encoding, but satellites aren't locked
        Serial.print("Data encoded, but no satellite fix yet. Satellites in view: ");
        Serial.println(gps.satellites.value());
      }
    }
  }

  // Diagnostic check: If 5 seconds pass and the library hasn't processed any characters,
  // the RX/TX lines are likely disconnected or reversed.
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("ERROR: No GPS data received. Check RX/TX wiring.");
    delay(1000);
  }
}