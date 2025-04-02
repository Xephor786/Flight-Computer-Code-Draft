#pragma once

#define SEALEVELPRESSURE_HPA (1013.25)

void processAltitudeData(float &previousAltitude, long &previousTime, float &previousVelocity);
bool check_apogee(double altitude);
//
//float sample_altimeter();