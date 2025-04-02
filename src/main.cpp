#include "../include/altimeter.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10


Adafruit_BMP3XX bmp; 


float previousAltitude = 0;
long previousTime = 0;
float previousVelocity = 0; // Track previous velocity

// Function declaration
void processAltitudeData(float &previousAltitude, long &previousTime, float &previousVelocity);

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
  processAltitudeData(previousAltitude, previousTime, previousVelocity);
  
  
  // Delay for a short period (adjust as needed for sampling rate)
  delay(100);
// This section is the rest of the code from the BMP390 Datasheet
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
    

}

