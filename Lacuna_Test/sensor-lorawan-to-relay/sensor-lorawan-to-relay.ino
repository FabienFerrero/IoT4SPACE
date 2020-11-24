/*
 *  _                                   ____                       
 * | |    __ _  ___ _   _ _ __   __ _  / ___| _ __   __ _  ___ ___ 
 * | |   / _` |/ __| | | | '_ \ / _` | \___ \| '_ \ / _` |/ __/ _ \
 * | |__| (_| | (__| |_| | | | | (_| |  ___) | |_) | (_| | (_|  __/
 * |_____\__,_|\___|\__,_|_| |_|\__,_| |____/| .__/ \__,_|\___\___|
 *                                           |_|                   
 * Copyright (C) 2020 Lacuna Space Ltd.
 *
 * Description: Sensor LoRaWAN to relay example
 * 
 * License: Revised BSD License, see LICENSE-LACUNA.TXT file included in the project
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef REGION
#define REGION R_EU868
#endif

// Interval between transmissions (in seconds)
#define TX_INTERVAL 3*60

// Only use for testing
//#define DEBUGSLEEP

#include <LibLacuna.h>
#include <time.h>
#include <RTC.h>
#include <SparkFunBME280.h>   // http://librarymanager/All#SparkFun_BME280
#include <Sodaq_LSM303AGR.h>  // http://librarymanager/All#Sodaq_LSM303AGR

// Keys and device address are MSB
static byte networkKey[] = { 0x9F, 0x74, 0x4F, 0x40, 0xC3, 0x91, 0x7E, 0x01, 0x6C, 0xD7, 0x87, 0x5E, 0xF5, 0xC0, 0xDB, 0x40 };
static byte appKey[] = { 0x10, 0xF1, 0xA5, 0x34, 0xCB, 0x63, 0x5B, 0xFF, 0x5A, 0xB4, 0x85, 0x7E, 0x2B, 0x3F, 0xD9, 0xC0 };
// Replace with your device address
static byte deviceAddress[] = { 0x26, 0x01, 0x1D, 0x57 };


static char payload[255];
static int payloadLength;

static lsLoraWANParams loraWANParams;
static lsLoraSatTxParams SattxParams;
static lsLoraTxParams txParams;

// BME280 sensor
BME280 bme280;

// Accelerometer
Sodaq_LSM303AGR lsm303(LSM303_WIRE);

void setup() {
  Serial.begin(9600);

  pinMode(LS_LED_BLUE, OUTPUT);

  pinMode(LS_GPS_ENABLE, OUTPUT);
  digitalWrite(LS_GPS_ENABLE, LOW);
  pinMode(LS_GPS_V_BCKP, OUTPUT);
  digitalWrite(LS_GPS_V_BCKP, LOW);

  digitalWrite(LS_VERSION_ENABLE, LOW);

  pinMode(LS_INT_MAG, OUTPUT); // make INT_MAG LOW for low-power 
  digitalWrite(LS_INT_MAG, LOW);

  while (!Serial && millis() < 2000);
  
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

  analogReadResolution(12);

  // LSM303
  LSM303_WIRE.begin();
  lsm303.enableAccelerometer();

  // BME 280
  Wire.begin();
  bme280.setI2CAddress(0x76);
  if (bme280.beginI2C(BME280_WIRE) == false)
  {
    Serial.println(F("BME280 did not respond. Please check wiring."));
  } 

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
  loraWANParams.rxEnable = false;
  
  // transmission parameters for terrestrial LoRa
  lsCreateDefaultLoraTxParams(&txParams);
  txParams.spreadingFactor = lsLoraSpreadingFactor_7;
  //txParams.bandwidth = lsLoraBandwidth_41_66_khz;
  txParams.bandwidth = lsLoraBandwidth_125_khz;
  txParams.preambleRelay = true;
  txParams.frequency = 865000000;
  txParams.power = 14;
  

  Serial.print("Terrestrial Uplink Frequency: ");
  Serial.println(txParams.frequency/1e6);
}

void loop() {

  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(100);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);



txParams.power = txParams.power-1;

if (txParams.power == -10) { 
    txParams.power=14;  
  }
  

  // LoRa to Relay
  generateMessage(false);
  Serial.println("Sending LoRa message");
  int lora_result = lsSendLoraWAN(&loraWANParams, &txParams, (byte *)payload, payloadLength);
  Serial.print("Result: ");
  Serial.println(lsErrorToString(lora_result));

#ifdef DEBUGSLEEP
  delay(TX_INTERVAL*1000);
#else
  setAlarm(RTC.getEpoch() + TX_INTERVAL);
  LS200_sleep();
#endif
}

void alarmMatch() { }

void setAlarm(uint32_t alarmepoch)
{  
  time_t t;
  struct tm tm;

  t = (time_t)alarmepoch;
  gmtime_r(&t, &tm);
  
  RTC.setAlarmTime(tm.tm_hour, tm.tm_min, tm.tm_sec);
  RTC.setAlarmDay(tm.tm_mday);
  
  RTC.enableAlarm(RTC.MATCH_HHMMSS);
  RTC.attachInterrupt(alarmMatch);
}

void LS200_sleep()  
{
  digitalWrite(LS_GPS_ENABLE, LOW);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, LOW);
  
  bme280.setMode(MODE_SLEEP); 
  lsm303.disableMagnetometer();
  lsm303.disableAccelerometer();

  SPI.end();
  delay(10);
  STM32.stop();

  // Sleep...
  
  SPI.begin();
  digitalWrite(LS_GPS_ENABLE, LOW);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, LOW);
  bme280.setMode(MODE_FORCED);
  lsm303.enableAccelerometer();
}

void generateMessage(bool satellite) {
 
  int16_t temp = (int16_t)(100*bme280.readTempC());
  uint8_t humidity = (int8_t)(bme280.readFloatHumidity());
  uint16_t pressure = (uint16_t)(bme280.readFloatPressure()/10);

  uint16_t voltage_adc = (uint16_t)analogRead(LS_BATVOLT_PIN);
  uint16_t voltage = (uint16_t)((LS_ADC_AREF / 4.096) * (LS_BATVOLT_R1 + LS_BATVOLT_R2) / LS_BATVOLT_R2 * (float)voltage_adc);

  int8_t acc_x = (int8_t)(100*lsm303.getX());
  int8_t acc_y = (int8_t)(100*lsm303.getY());
  int8_t acc_z = (int8_t)(100*lsm303.getZ());

  Serial.println("Read sensors");
  Serial.print("  Voltage: ");
  Serial.println(voltage/1000.0f);
  Serial.print("  Temp: ");
  Serial.println(temp/100.0f);
  Serial.print("  Humidity: ");
  Serial.println(humidity);
  Serial.print("  Pressure: ");
  Serial.println(pressure);

  Serial.print("  x = ");
  Serial.print(acc_x);
  Serial.print(", y = ");
  Serial.print(acc_y);
  Serial.print(", z = ");
  Serial.println(acc_z);

  payload[0] = satellite;
  payload[1] = (temp >> 8) & 0xff;
  payload[2] = temp & 0xff;
  payload[3] = (voltage_adc >> 8) & 0xff;
  payload[4] = voltage_adc & 0xff;
  payload[5] = humidity; 
  payload[6] = (pressure >> 8) & 0xff;
  payload[7] = pressure & 0xff;
  payload[8] = acc_x;
  payload[9] = acc_y;
  payload[10] = acc_z;
  
  payloadLength = 11;
}
