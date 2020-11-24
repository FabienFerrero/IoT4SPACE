/*
 *  _                                   ____                       
 * | |    __ _  ___ _   _ _ __   __ _  / ___| _ __   __ _  ___ ___ 
 * | |   / _` |/ __| | | | '_ \ / _` | \___ \| '_ \ / _` |/ __/ _ \
 * | |__| (_| | (__| |_| | | | | (_| |  ___) | |_) | (_| | (_|  __/
 * |_____\__,_|\___|\__,_|_| |_|\__,_| |____/| .__/ \__,_|\___\___|
 *                                           |_|                   
 * Copyright (C) 2020 Lacuna Space Ltd.
 *
 * Description: CW for antenna analysis example sketch
 * 
 * License: Revised BSD License, see LICENSE-LACUNA.TXT file included in the project
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef REGION
#define REGION R_EU868
#endif

// Interval between transmissions
#define TX_INTERVAL 1

#include <LibLacuna.h>

static lsLoraTxParams txParams;

void setup() {
  Serial.begin(9600);

  pinMode(LS_LED_BLUE, OUTPUT);

  pinMode(LS_GPS_ENABLE, OUTPUT);
  digitalWrite(LS_GPS_ENABLE, LOW);
  pinMode(LS_GPS_V_BCKP, OUTPUT);
  digitalWrite(LS_GPS_V_BCKP, LOW);
  // initialize the pushbutton pin as an input:
  pinMode(LS_USER_BUTTON, INPUT);

  while (!Serial && millis() < 3000);
  
  Serial.println("Initializing");

  Serial.print("Configured Region: ");
  #if REGION == R_EU868
    Serial.println("Europe 862-870 Mhz");
  #elif REGION  == R_US915
    Serial.println("US 902-928 Mhz");
  #elif REGION == R_AS923
    Serial.println("Asia 923 Mhz");
  #elif REGION == R_IN865
    Serial.println("India 865-867 Mhz");
  #else 
    Serial.println("Undefined");  
  #endif

  // SX1262 configuration for lacuna LS200 board
  lsSX126xConfig cfg;
  lsCreateDefaultSX126xConfig(&cfg);

  // Initialize SX1262
  int result = lsInitSX126x(&cfg);
  Serial.print("E22/SX1262: ");
  Serial.println(lsErrorToString(result));
 
  // transmission parameters for CW
  lsCreateDefaultLoraTxParams(&txParams);
 
}

void loop() {

  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(100);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);

  txParams.power = 21;
  txParams.frequency = 862000000;

  Serial.print("Frequency: ");
  Serial.print(txParams.frequency/1e6);
  Serial.print(" Mhz, Power: ");
  Serial.print(txParams.power);
  Serial.println(" dBm.");


  while ( digitalRead(LS_USER_BUTTON)== HIGH){
  
  // Transmit CW at 868 Mhz for 5 seconds
  Serial.println("Starting CW"); 
  int cw_result  = lsStartCW(&txParams);
  delay(1000);
  Serial.println("Stopping CW"); 
  cw_result  = lsStopCW();
  }

  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(100);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(3000);

  while ( digitalRead(LS_USER_BUTTON)== HIGH){

  txParams.power = 22;
  txParams.frequency = 920000000;
  // Transmit CW at 868 Mhz for 5 seconds
  Serial.println("Starting CW"); 
  int cw_result  = lsStartCW(&txParams);
  delay(1000);
  Serial.println("Stopping CW"); 
  cw_result  = lsStopCW();
  }

  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(100);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(3000);


  while ( digitalRead(LS_USER_BUTTON)== HIGH){

  // Sweep power
  int power = 21;
Serial.println("Starting CW power sweep");
for (power=16; power<=22;power++){
  Serial.println(power);
  txParams.power = power;
  int cw_result  = lsStartCW(&txParams);
  delay(1000);
  Serial.println("Stopping CW"); 
  cw_result  = lsStopCW();
  delay(100);
}
  }

  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(100);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(3000);
  

  
  while ( digitalRead(LS_USER_BUTTON)== HIGH){

  // Sweep 862 to 865 Mhz in 1000 kHz steps of 10 ms duration
  Serial.println("Starting sweep CW");
int  power = 21; 
  txParams.power = power;
  int sweep_result  = lsSweepCW(&txParams, 800000000, 960000000, 1000000, 100);
  Serial.println("Sweep CW finished"); 

  }
  
  delay(TX_INTERVAL*1000);

}
