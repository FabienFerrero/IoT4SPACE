/*
 *  _                                   ____                       
 * | |    __ _  ___ _   _ _ __   __ _  / ___| _ __   __ _  ___ ___ 
 * | |   / _` |/ __| | | | '_ \ / _` | \___ \| '_ \ / _` |/ __/ _ \
 * | |__| (_| | (__| |_| | | | | (_| |  ___) | |_) | (_| | (_|  __/
 * |_____\__,_|\___|\__,_|_| |_|\__,_| |____/| .__/ \__,_|\___\___|
 *                                           |_|                   
 * Copyright (C) 2019 Lacuna Space Ltd.
 *
 * Description: Simple LoRaWAN example sketch
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

// Keys and device address are MSB
static byte networkKey[] = { 0x75, 0xA3, 0x0E, 0xD2, 0xD0, 0xB7, 0xC7, 0xDA, 0x35, 0xC5, 0xE9, 0xE5, 0xD7, 0xFE, 0x28, 0xE5 };

static byte appKey[] = { 0x71, 0x1E, 0xAE, 0xBA, 0x4B, 0xFE, 0x3F, 0x25, 0xE8, 0x7F, 0x5D, 0x9C, 0x8D, 0xC0, 0xA3, 0xDA };
// Replace with your device address
static byte deviceAddress[] = { 0x26, 0x01, 0x15, 0xEF };

static char payload[255];
const String mytext = "test message";

static lsLoraWANParams loraWANParams;
static lsLoraSatTxParams SattxParams;
static lsLoraTxParams txParams;

int counter = 99;

void setup() {
  Serial.begin(9600);

  pinMode(LS_LED_BLUE, OUTPUT);

  pinMode(LS_GPS_ENABLE, OUTPUT);
  digitalWrite(LS_GPS_ENABLE, LOW);
  pinMode(LS_GPS_V_BCKP, OUTPUT);
  digitalWrite(LS_GPS_V_BCKP, LOW);

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

  // LoRaWAN session parameters
  lsCreateDefaultLoraWANParams(&loraWANParams, networkKey, appKey, deviceAddress);
  loraWANParams.txPort = 1;
  loraWANParams.rxEnable = true;

// Sat session parameters
  SattxParams.power = 21;
 
  // transmission parameters for Lacuna satellites
  lsCreateDefaultLoraSatTxParams(&SattxParams);

  // transmission parameters for terrestrial LoRa
  lsCreateDefaultLoraTxParams(&txParams);
  txParams.spreadingFactor = lsLoraSpreadingFactor_7;

  Serial.print("Terrestrial Uplink Frequency: ");
  Serial.println(txParams.frequency/1e6);
 
}

void loop() {

  counter = counter + 1; // increment counter

  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(100);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);

  if (counter == 100) {

    counter = 0;

  Serial.println("Sending LoRa message"); 
  mytext.toCharArray(payload, 255); 
  int lora_result  = lsSendLoraWAN(&loraWANParams, &txParams, (byte *)payload, mytext.length());

  if (loraWANParams.rxpayloadLength) {
    Serial.println("Downlink received");
    Serial.print("Length: ");
    Serial.println(loraWANParams.rxpayloadLength);
    Serial.print("Port: ");
    Serial.println(loraWANParams.rxPort);
    Serial.print("Frame Counter: ");
    Serial.println(loraWANParams.framecounter_down);
    Serial.print("SNR: ");
    Serial.println(txParams.snr);
    Serial.print("RSSI: ");
    Serial.println(txParams.rssi);
    Serial.print("SignalRSSI: ");
    Serial.println(txParams.signalrssi);
    Serial.print("Payload: ");
    
    for (char n = 0; n < loraWANParams.rxpayloadLength; n++)
        {
          Serial.print (payload[n],HEX);
          Serial.write (" ");
          
        }
    Serial.println();
  }
  Serial.print("Result: ");
  Serial.println(lsErrorToString(lora_result));

  }
 
  // Uncomment to send satellite message
  //
   delay(200);
   Serial.println("Sending LoraSat message");
   mytext.toCharArray(payload, 255); 
   int sat_result = lsSendLoraSatWAN(&loraWANParams, &SattxParams, (byte *)payload, mytext.length());
   Serial.print("Result: ");
   Serial.println(lsErrorToString(sat_result));

  // wait TX_INTERVAL seconds
  delay(TX_INTERVAL*1000);

}