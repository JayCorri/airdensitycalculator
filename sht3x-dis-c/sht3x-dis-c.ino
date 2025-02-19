/*
 *
 * Project: Ghost
 * Experiment: 1
 * 1/28/2023
 * Air Density Calculator
 * Components: Arduino R3 (MCU), M5Stack ENV III (SENSOR)
 * Purpose: Calculate Air Density based on temperature, humidity, and air pressure measurements
 * Parameters:
 * Temperature [-40,120]˚C; Highest Accuracy [0,60] ±0.2˚C; working temp [0,40]˚C 
 * Humidity 10-90%RH ±2%
 * Air Pressure [300,1100]hPa; 0.06Pa resolution; ±3.9Pa
 * Calculation Error: ±#%
 * SHT30:0x44 (temperature and humidity)
 * QMP6988:0x70 (air pressure)
 * Programming Goal:
 * 1. Initiate and run communication between MCU and SENSOR
 * 2. Measure temperautre, humidity. and air pressure
 * 3. Calculate Air density
 * 4. Communicate air density to monitor
 * 
 */ 

#include <Wire.h> //load i2c protocol library
#include <SHTSensor.h> //load sht3x library
SHTSensor sht;
float temp;
float rh;
//int sht = 0x44;
#define QMP 0x70
#define PRESS_TXD0 0xf9
#define PRESS_TXD1 0xf8
#define PRESS_TXD2 0xf7
int kpa1, kpa2, kpa3;

void setup() {
  // put your setup code here, to run once:
Wire.begin(); //begin i2c protocol as a controller
Serial.begin(9600);
delay(1000);
if (sht.init()){
  Serial.print("init() success\n");
}
else{
  Serial.print("sht.init() failed\n");
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
}


}

void loop() {
  if (sht.readSample()) {
  }
  else {
   Serial.print("Error in readSample()\n");
  }
temp = (sht.getTemperature()*9/5)+32;
Serial.print("Temperature: ");
Serial.print(temp);
Serial.println("°F");
float rh = sht.getHumidity();
Serial.print("Humidity: ");
Serial.print(rh);
Serial.println("%");
delay(1000);


}