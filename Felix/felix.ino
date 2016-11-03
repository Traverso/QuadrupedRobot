/*************************************************** 
 Felix Quadruped Robot
 Version:2.0
 Author: Ronald Jaramillo @RonaldXJT
 Web: www.burningservos.com
 ****************************************************/
#include <Wire.h>
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "includes.h"


Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


void setup() {
  while(!Serial);  
  delay(500);
  
  Serial.begin(115200);
  Serial.println("Felix V2.0");

  legSetup();
  homePosition();

  calibrate();
  return;
  
 
  //communicationSetup();
  controllerSetup();

  
  loadWalk(CURRENT_WALK,CURRENT_STEP_COUNT,MOVING_FORWARD,0);
}


void loop() {
  //communicationLoop();
  controllerLoop();
}


