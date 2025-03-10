/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 0
#define INTERRUPT_PIN 4 //Not used

SFEVL53L1X tof1;
SFEVL53L1X tof2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
int t = 0;

void setup(void)
{
  Wire.begin();

  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW); //Turn off 2nd sensor with xshut
  tof1.setI2CAddress(0x30); //give 1st sensor different address
  digitalWrite(SHUTDOWN_PIN, HIGH); // Turn on second sensor

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  if (tof1.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  if (tof2.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");
  tof1.setDistanceModeShort();
  tof2.setDistanceModeShort();
  t = (int)micros();
}

void loop(void)
{

  tof1.startRanging(); //Write configuration bytes to initiate measurement
  tof2.startRanging(); //Write configuration bytes to initiate measurement
  t = micros();
  if (tof1.checkForDataReady())
  {
    int distance1 = tof1.getDistance(); //Get the result of the measurement from the sensor
    tof1.clearInterrupt();
    tof1.stopRanging();
    Serial.print("Distance1(mm): ");
    Serial.println(distance1);
  }
  if (tof2.checkForDataReady())
  {
    int distance2 = tof2.getDistance(); //Get the result of the measurement from the sensor
    tof2.clearInterrupt();
    tof2.stopRanging();
    Serial.print("Distance2(mm): ");
    Serial.println(distance2);
  }

  //int elapsed = micros()-t;

  //float distanceInches = distance * 0.0393701;
  //float distanceFeet = distanceInches / 12.0;

  //Serial.print("\tDistance(ft): ");
  //Serial.print(distanceFeet, 2);
  //Serial.print("\tMeasurement time (us): ");
  //Serial.println(elapsed);
  //Serial.print("Loop time (ms): ");
  //Serial.println(millis());
}
