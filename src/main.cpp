#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp; 
// put function declarations here:


float previousAltitude = 0;
long previousTime = 0;
float previousVelocity = 0; // Track previous velocity

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
      //if (!bmp.begin_I2C()) { // I2C Mode 
      if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) { // SPI Mode
        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
        while (1);
      }

      //Oversampling and Filter Initialization
      bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
      bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    }


void loop() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  long currentTime = millis();
  long timeDiff = currentTime - previousTime;
  float altitudeDiff = altitude - previousAltitude;
  float velocity = 0;
  
  //*This section is used to find when apogee is reached by calculating the differences in altitude across time */
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
      Serial.println("Approaching Apogee"); // Add an approaching apogee message
    }
  
  } else {
    Serial.println("Initializing");
  }
  
  // Update previous altitude, time, and velocity for next iteration
  previousAltitude = altitude;
  previousTime = currentTime;
  previousVelocity = velocity;
  
  // Delay for a short period (adjust as needed for sampling rate)
  delay(100);

  Serial.print("Temperature = ");
    Serial.print(bmp.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");


    Serial.println();
    delay(2000);
    
  // put your main code here, to run repeatedly:
}

// put function definitions here:
