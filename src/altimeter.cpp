#include "../include/altimeter.h"
#include "Adafruit_BMP3XX.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

bool check_apogee(double altitude) {
    // Example logic: Apogee is detected if altitude stops increasing
    static double previousAltitude = 0;
    static bool apogeeDetected = false;

    if (altitude < previousAltitude && !apogeeDetected) {
        apogeeDetected = true;
        return true; // Apogee detected
    }

    previousAltitude = altitude;
    return false; // Apogee not detected
}

float previousAltitude = 0;
long previousTime = 0;
float previousVelocity = 0; // Track previous velocity

Adafruit_BMP3XX bmp; // Create the BMP object



void processAltitudeData(float& previousAltitude, long& previousTime, float& previousVelocity) {
    if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }

    float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    long currentTime = millis();
    long timeDiff = currentTime - previousTime;
    float altitudeDiff = altitude - previousAltitude;
    float velocity = 0;

    // Calculate velocity if previous time is available
    if (previousTime != 0) {
        velocity = altitudeDiff / (timeDiff / 1000.0); // Velocity in meters per second
    }

    // Print altitude and velocity data
    Serial.print("Altitude = ");
    Serial.print(altitude);
    Serial.print(" m, Velocity = ");
    Serial.print(velocity);
    Serial.print(" m/s, ");

    // Determine and print rocket state
    if (previousTime != 0) {
        if (velocity > 0) {
            Serial.println("Rising");
        } else if (velocity < 0) {
            Serial.println("Falling");
        } else {
            Serial.println("Apogee or Stationary");
        }

        // Apogee detection: Check for velocity sign change
        if (previousVelocity > 0 && velocity < 0) {
            Serial.println("Approaching Apogee");
        }
    } else {
        Serial.println("Initializing");
    }

    // Update previous altitude, time, and velocity for next iteration
    previousAltitude = altitude;
    previousTime = currentTime;
    previousVelocity = velocity;
}


//float sample_altimeter() {
    //bmp.readAltitude(SEALEVELPRESSURE_HPA)
    //return 0.0;
//}