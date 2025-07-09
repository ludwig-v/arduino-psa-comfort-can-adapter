/*
Copyright 2019-2022, Ludwig V. <https://github.com/ludwig-v>
Copyright 2021, Nick V. (V3nn3tj3) <https://github.com/v3nn3tj3>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License at <http://www.gnu.org/licenses/> for
more details.

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
*/

/////////////////////
//    Libraries    //
/////////////////////

#include <EEPROM.h>
#include <SPI.h>
#include <Time.h>
#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h> // https://github.com/PaulStoffregen/DS1307RTC
#include <mcp2515.h> // https://github.com/autowp/arduino-mcp2515 + https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

/////////////////////
//  Configuration  //
/////////////////////

#define CS_PIN_CAN0 10
#define CS_PIN_CAN1 9
#define SERIAL_SPEED 115200
#define CAN_SPEED CAN_125KBPS // Entertainment CAN bus - Low speed
#define CAN_FREQ MCP_16MHZ // Switch to 8MHZ if you have a 8Mhz module

///////////////////////
// Private functions //
///////////////////////
byte checksumm_0E6(const byte* frame);

////////////////////
// Initialization //
////////////////////

MCP2515 CAN0(CS_PIN_CAN0); // CAN-BUS Shield N°1
MCP2515 CAN1(CS_PIN_CAN1); // CAN-BUS Shield N°2

////////////////////
//   Variables    //
////////////////////

// My variables
bool debugGeneral = false; // Get some debug informations on Serial
bool debugCAN0 = false; // Read data sent by ECUs from the car to Entertainment CAN bus using https://github.com/alexandreblin/python-can-monitor
bool debugCAN1 = false; // Read data sent by the NAC / SMEG to Entertainment CAN bus using https://github.com/alexandreblin/python-can-monitor
bool EconomyModeEnabled = true; // You can disable economy mode on the Telematic if you want to - Not recommended at all
bool Send_CAN2010_ForgedMessages = false; // Send forged CAN2010 messages to the CAR CAN-BUS Network (useful for testing CAN2010 device(s) from already existent connectors)
bool TemperatureInF = false; // Default Temperature in Celcius
bool mpgMi = false;
bool kmL = false; // km/L statistics instead of L/100
bool fixedBrightness = false; // Force Brightness value in case the calibration does not match your brightness value range
bool noFMUX = false; // If you don't have any useful button on the main panel, turn the SRC button on steering wheel commands into MENU - only works for CAN2010 SMEG / NAC -
byte steeringWheelCommands_Type = 0; // noFMUX extra setting : 0 = Generic, 1 = C4 I / C5 X7 NAV+MUSIC+APPS+PHONE mapping, 2 = C4 I / C5 X7 MENU mapping, 3 = C4 I / C5 X7 MENU mapping + SRC on wiper command button, 4 = C4 I / C5 X7 MENU mapping + TRIP on wiper command button, 5 = C4 I / C5 X7 MENU mapping + SRC on wiper command button + TRIP on ESC button
byte languageID = 0; // Default is FR: 0 - EN: 1 / DE: 2 / ES: 3 / IT: 4 / PT: 5 / NL: 6 / BR: 9 / TR: 12 / RU: 14
bool listenCAN2004Language = false; // Switch language on CAN2010 devices if changed on supported CAN2004 devices, default: no
byte Time_day = 1; // Default day if the RTC module is not configured
byte Time_month = 1; // Default month if the RTC module is not configured
int Time_year = 2022; // Default year if the RTC module is not configured
byte Time_hour = 0; // Default hour if the RTC module is not configured
byte Time_minute = 0; // Default minute if the RTC module is not configured
bool resetEEPROM = false; // Switch to true to reset all EEPROM values
bool CVM_Emul = true; // Send suggested speed from Telematic to fake CVM (Multifunction camera inside the windshield) frame
bool generatePOPups = false; // Generate notifications from alerts journal - useful for C5 (X7)

bool emulateVIN = false; // Replace network VIN by another (donor car for example)
char vinNumber[18] = "VF3XXXXXXXXXXXXXX";

bool hasAnalogicButtons = false; // Analog buttons instead of FMUX
byte menuButton = 4;
byte volDownButton = 5;
byte volUpButton = 6;
byte scrollValue = 0;

// Default variables
bool Ignition = false;
bool SerialEnabled = false;
int Temperature = 0;
bool EconomyMode = false;
bool EngineRunning = false;
byte languageID_CAN2004 = 0;
bool AirConditioningON = false;
byte FanSpeed = 0;
bool FanOff = false;
bool AirRecycle = false;
bool DeMist = false;
bool DeFrost = false;
byte LeftTemp = 0;
byte RightTemp = 0;
bool Mono = false;
bool FootAerator = false;
bool WindShieldAerator = false;
bool CentralAerator = false;
bool AutoFan = false;
byte FanPosition = 0;
bool MaintenanceDisplayed = false;
int buttonState = 0;
int lastButtonState = 0;
long lastDebounceTime = 0;
long buttonPushTime = 0;
long buttonSendTime = 0;
long debounceDelay = 100;
int daysSinceYearStart = 0;
unsigned long customTimeStamp = 0;
int vehicleSpeed = 0;
int engineRPM = 0;
bool darkMode = false;
bool resetTrip1 = false;
bool resetTrip2 = false;
bool pushAAS = false;
bool pushSAM = false;
bool pushDSG = false;
bool pushSTT = false;
bool pushCHECK = false;
bool stopCHECK = false;
bool pushBLACK = false;
bool pushASR = false;
bool pushTRIP = false;
byte personalizationSettings[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte statusCMB[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte statusTRIP[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool TelematicPresent = false;
bool ClusterPresent = false;
bool pushA2 = false;
int alertsCache[] = {0, 0, 0, 0, 0, 0, 0, 0}; // Max 8
byte alertsParametersCache[] = {0, 0, 0, 0, 0, 0, 0, 0}; // Max 8
bool isBVMP = false;
byte statusOpenings = 0;
byte notificationParameters = 0;

// Language & Unit CAN2010 value
byte languageAndUnitNum = (languageID * 4) + 128;

// CAN-BUS Messages
struct can_frame canMsgSnd;
struct can_frame canMsgRcv;

void setup() {
  int tmpVal;

  if (resetEEPROM) {
    EEPROM.update(0, 0);
    EEPROM.update(1, 0);
    EEPROM.update(2, 0);
    EEPROM.update(3, 0);
    EEPROM.update(4, 0);
    EEPROM.update(5, 0);
    EEPROM.update(6, 0);
    EEPROM.update(7, 0);
    EEPROM.update(10, 0);
    EEPROM.update(11, 0);
    EEPROM.update(12, 0);
    EEPROM.update(13, 0);
    EEPROM.update(14, 0);
    EEPROM.update(15, 0);
    EEPROM.update(16, 0);
  }

  if (debugCAN0 || debugCAN1 || debugGeneral) {
    SerialEnabled = true;
  }

  // Read data from EEPROM
  tmpVal = EEPROM.read(0);
  if (tmpVal >= 128) {
    languageAndUnitNum = tmpVal;
  }

  if ((languageAndUnitNum % 2) == 0 && kmL) {
    languageAndUnitNum = languageAndUnitNum + 1;
  }

  tmpVal = EEPROM.read(1);
  if (tmpVal <= 32) {
    languageID_CAN2004 = tmpVal;
  }

  tmpVal = EEPROM.read(2);
  if (tmpVal <= 32) {
    languageID = tmpVal;
  }

  tmpVal = EEPROM.read(3);
  if (tmpVal == 1) {
    TemperatureInF = true;
  }

  tmpVal = EEPROM.read(4);
  if (tmpVal == 1) {
    mpgMi = true;
  }

  tmpVal = EEPROM.read(5);
  if (tmpVal <= 31) {
    Time_day = tmpVal;
  }

  tmpVal = EEPROM.read(6);
  if (tmpVal <= 12) {
    Time_month = tmpVal;
  }

  EEPROM.get(7, tmpVal); // int
  if (tmpVal >= 1872 && tmpVal <= 2127) {
    Time_year = tmpVal;
  }

  personalizationSettings[0] = EEPROM.read(10);
  personalizationSettings[1] = EEPROM.read(11);
  personalizationSettings[2] = EEPROM.read(12);
  personalizationSettings[3] = EEPROM.read(13);
  personalizationSettings[4] = EEPROM.read(14);
  personalizationSettings[5] = EEPROM.read(15);
  personalizationSettings[6] = EEPROM.read(16);

  if (hasAnalogicButtons) {
    //Initialize buttons - MENU/VOL+/VOL-
    pinMode(menuButton, INPUT_PULLUP);
    pinMode(volDownButton, INPUT_PULLUP);
    pinMode(volUpButton, INPUT_PULLUP);
  }

  if (SerialEnabled) {
    // Initalize Serial for debug
    Serial.begin(SERIAL_SPEED);

    // CAN-BUS from car
    Serial.println("Initialization CAN0");
  }

  CAN0.reset();
  CAN0.setBitrate(CAN_SPEED, CAN_FREQ);
  while (CAN0.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
  }

  if (SerialEnabled) {
    // CAN-BUS to CAN2010 device(s)
    Serial.println("Initialization CAN1");
  }

  CAN1.reset();
  CAN1.setBitrate(CAN_SPEED, CAN_FREQ);
  while (CAN1.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
  }

  setSyncProvider(RTC.get); // Get time from the RTC module
  if (timeStatus() != timeSet) {
    if (SerialEnabled) {
      Serial.println("Unable to sync with the RTC");
    }

    // Set default time (01/01/2020 00:00)
    setTime(Time_hour, Time_minute, 0, Time_day, Time_month, Time_year);
    EEPROM.update(5, Time_day);
    EEPROM.update(6, Time_month);
    EEPROM.put(7, Time_year);
  } else if (SerialEnabled) {
    Serial.println("RTC has set the system time");
  }

  // Set hour on CAN-BUS Clock
  canMsgSnd.data[0] = hour();
  canMsgSnd.data[1] = minute();
  canMsgSnd.can_id = 0x228;
  canMsgSnd.can_dlc = 2;
  CAN0.sendMessage( & canMsgSnd);

  // Send fake EMF version
  canMsgSnd.data[0] = 0x25;
  canMsgSnd.data[1] = 0x0A;
  canMsgSnd.data[2] = 0x0B;
  canMsgSnd.data[3] = 0x04;
  canMsgSnd.data[4] = 0x0C;
  canMsgSnd.data[5] = 0x01;
  canMsgSnd.data[6] = 0x20;
  canMsgSnd.data[7] = 0x11;
  canMsgSnd.can_id = 0x5E5;
  canMsgSnd.can_dlc = 8;
  CAN0.sendMessage( & canMsgSnd);

  if (SerialEnabled) {
    Serial.print("Current Time: ");
    Serial.print(day());
    Serial.print("/");
    Serial.print(month());
    Serial.print("/");
    Serial.print(year());

    Serial.print(" ");

    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());

    Serial.println();
  }
}

void loop() {
  int tmpVal;

  if (hasAnalogicButtons) {
    // Receive buttons from the car
    if (((millis() - lastDebounceTime) > debounceDelay)) {
      tmpVal = 0;
      if (!digitalRead(menuButton)) tmpVal += 0b001;
      if (!digitalRead(volDownButton)) tmpVal += 0b010;
      if (!digitalRead(volUpButton)) tmpVal += 0b100;
      if (tmpVal != lastButtonState) {
        buttonPushTime = millis();
        buttonSendTime = 0;
        //buttonPushState = 0;
      }
      if ((millis() - buttonPushTime) > 100) {
        switch (tmpVal) {
        case 0b001:
          //canMsgSnd.data[0] = 0x02; // MENU button
          canMsgSnd.data[0] = 0x02;
          canMsgSnd.data[1] = 0x00;
          canMsgSnd.data[2] = 0x00;
          canMsgSnd.data[3] = 0x00;
          canMsgSnd.data[4] = 0x00;
          canMsgSnd.data[5] = 0xFF;
          canMsgSnd.data[6] = 0x00;
          canMsgSnd.data[7] = 0x00;
          canMsgSnd.can_id = 0x122;
          canMsgSnd.can_dlc = 8;
          // Menu button
          if (buttonSendTime == 0) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Menu");
            }
            lastDebounceTime = millis();
            buttonSendTime = millis();
            //buttonPushState = 1;
          } else if (millis() - buttonPushTime > 800 && ((millis() - buttonPushTime < 2000 && millis() - buttonSendTime > 600) || (millis() - buttonPushTime > 2000 && millis() - buttonSendTime > 350))) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Menu");
            }
            buttonSendTime = millis();
            lastDebounceTime = millis();
          }
          break;
        case 0b010:
          canMsgSnd.data[0] = 0x04; //Volume down
          canMsgSnd.data[1] = scrollValue;
          canMsgSnd.data[2] = 0x00;
          canMsgSnd.can_id = 0x21F;
          canMsgSnd.can_dlc = 3;
          // Menu button
          if (buttonSendTime == 0) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Vol -");
            }
            lastDebounceTime = millis();
            buttonSendTime = millis();
            //buttonPushState = 1;
          } else if (millis() - buttonPushTime > 800 && ((millis() - buttonPushTime < 2000 && millis() - buttonSendTime > 600) || (millis() - buttonPushTime > 2000 && millis() - buttonSendTime > 350))) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Vol -");
            }
            buttonSendTime = millis();
            lastDebounceTime = millis();
          }
          break;
        case 0b100:
          canMsgSnd.data[0] = 0x08; //Volume down
          canMsgSnd.data[1] = scrollValue;
          canMsgSnd.data[2] = 0x00;
          canMsgSnd.can_id = 0x21F;
          canMsgSnd.can_dlc = 3;
          // Menu button
          if (buttonSendTime == 0) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Vol +");
            }
            lastDebounceTime = millis();
            buttonSendTime = millis();
            //buttonPushState = 1;
          } else if (millis() - buttonPushTime > 800 && ((millis() - buttonPushTime < 2000 && millis() - buttonSendTime > 600) || (millis() - buttonPushTime > 2000 && millis() - buttonSendTime > 350))) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Vol +");
            }
            buttonSendTime = millis();
            lastDebounceTime = millis();
          }
          break;
        case 0b110:
          canMsgSnd.data[0] = 0x0C; //Mute
          canMsgSnd.data[1] = scrollValue;
          canMsgSnd.data[2] = 0x00;
          canMsgSnd.can_id = 0x21F;
          canMsgSnd.can_dlc = 3;
          // Menu button
          if (buttonSendTime == 0) {
            CAN1.sendMessage( & canMsgSnd);
            if (SerialEnabled) {
              Serial.println("Mute");
            }
            lastDebounceTime = millis();
            buttonSendTime = millis();
            //buttonPushState = 1;
          }
          break;
        default:
          //buttonPushState = 0;
          lastDebounceTime = millis();
        }
      }
      lastButtonState = tmpVal;
    }
  }

  // Receive CAN messages from the car
  if (CAN0.readMessage( & canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    if (debugCAN0) {
      Serial.print("FRAME:ID=");
      Serial.print(id);
      Serial.print(":LEN=");
      Serial.print(len);

      char tmp[3];
      for (int i = 0; i < len; i++) {
        Serial.print(":");

        snprintf(tmp, 3, "%02X", canMsgRcv.data[i]);

        Serial.print(tmp);
      }

      Serial.println();

      CAN1.sendMessage( & canMsgRcv);
    } else if (!debugCAN1) {
      if (id == 0x15B) {
        // Do not send back converted frames between networks
      } else if (id == 0x36 && len == 8) { // Economy Mode detection
        if (bitRead(canMsgRcv.data[2], 7) == 1) {
          if (!EconomyMode && SerialEnabled) {
            Serial.println("Economy mode ON");
          }

          EconomyMode = true;
        } else {
          if (EconomyMode && SerialEnabled) {
            Serial.println("Economy mode OFF");
          }

          EconomyMode = false;
        }

        tmpVal = canMsgRcv.data[3];

        // Fix brightness when car lights are ON - Brightness Instrument Panel "20" > "2F" (32 > 47) - Depends on your car
        if (fixedBrightness && tmpVal >= 32) {
          canMsgRcv.data[3] = 0x28; // Set fixed value to avoid low brightness due to incorrect CAN2010 Telematic calibration
        }
        CAN1.sendMessage( & canMsgRcv);
      } else if (id == 0xB6 && len == 8) {
        engineRPM = ((canMsgRcv.data[0] << 8) | canMsgRcv.data[1]) * 0.125;
        if (engineRPM > 0) {
          EngineRunning = true;
        } else {
          EngineRunning = false;
        }
        vehicleSpeed = ((canMsgRcv.data[2] << 8) | canMsgRcv.data[3]) * 0.01;
        CAN1.sendMessage( & canMsgRcv);
      } else if (id == 0x336 && len == 3 && emulateVIN) { // ASCII coded first 3 letters of VIN
        canMsgSnd.data[0] = vinNumber[0]; //V
        canMsgSnd.data[1] = vinNumber[1]; //F
        canMsgSnd.data[2] = vinNumber[2]; //3
        canMsgSnd.can_id = 0x336;
        canMsgSnd.can_dlc = 3;
        CAN1.sendMessage( & canMsgSnd);
      } else if (id == 0x3B6 && len == 6 && emulateVIN) { // ASCII coded 4-9 letters of VIN
        canMsgSnd.data[0] = vinNumber[3]; //X
        canMsgSnd.data[1] = vinNumber[4]; //X
        canMsgSnd.data[2] = vinNumber[5]; //X
        canMsgSnd.data[3] = vinNumber[6]; //X
        canMsgSnd.data[4] = vinNumber[7]; //X
        canMsgSnd.data[5] = vinNumber[8]; //X
        canMsgSnd.can_id = 0x3B6;
        canMsgSnd.can_dlc = 6;
        CAN1.sendMessage( & canMsgSnd);
      } else if (id == 0x2B6 && len == 8 && emulateVIN) { // ASCII coded 10-17 letters (last 8) of VIN
        canMsgSnd.data[0] = vinNumber[9]; //X
        canMsgSnd.data[1] = vinNumber[10]; //X
        canMsgSnd.data[2] = vinNumber[11]; //X
        canMsgSnd.data[3] = vinNumber[12]; //X
        canMsgSnd.data[4] = vinNumber[13]; //X
        canMsgSnd.data[5] = vinNumber[14]; //X
        canMsgSnd.data[6] = vinNumber[15]; //X
        canMsgSnd.data[7] = vinNumber[16]; //X
        canMsgSnd.can_id = 0x2B6;
        canMsgSnd.can_dlc = 8;
        CAN1.sendMessage( & canMsgSnd);
      } else if (id == 0xE6 && len < 8) { // ABS status frame, increase length
        canMsgSnd.data[0] = canMsgRcv.data[0]; // Status lights / Alerts
        canMsgSnd.data[1] = canMsgRcv.data[1]; // Rear left rotations
        canMsgSnd.data[2] = canMsgRcv.data[2]; // Rear left rotations
        canMsgSnd.data[3] = canMsgRcv.data[3]; // Rear right rotations
        canMsgSnd.data[4] = canMsgRcv.data[4]; // Rear right rotations
        canMsgSnd.data[5] = canMsgRcv.data[5]; // Battery Voltage measured by ABS
        canMsgSnd.data[6] = canMsgRcv.data[6]; // STT / Slope / Emergency Braking
        canMsgSnd.data[7] = checksumm_0E6(canMsgSnd.data); // Checksum / Counter : Test needed
        canMsgSnd.can_id = 0xE6;
        canMsgSnd.can_dlc = 8;
        CAN1.sendMessage( & canMsgSnd);
      } else if (id == 0x21F && len == 3) { // Steering wheel commands - Generic
        tmpVal = canMsgRcv.data[0];
        scrollValue = canMsgRcv.data[1];

        if (bitRead(canMsgRcv.data[0], 1) && noFMUX && steeringWheelCommands_Type == 0) { // Replace MODE/SRC by MENU (Valid for 208, C-Elysee calibrations for example)
          canMsgSnd.data[0] = 0x80; // MENU button
          canMsgSnd.data[1] = 0x00;
          canMsgSnd.data[2] = 0x00;
          canMsgSnd.data[3] = 0x00;
          canMsgSnd.data[4] = 0x00;
          canMsgSnd.data[5] = 0x02;
          canMsgSnd.data[6] = 0x00; // Volume potentiometer button
          canMsgSnd.data[7] = 0x00;
          canMsgSnd.can_id = 0x122;
          canMsgSnd.can_dlc = 8;
          CAN1.sendMessage( & canMsgSnd);
          if (Send_CAN2010_ForgedMessages) {
            CAN0.sendMessage( & canMsgSnd);
          }
        } else {
          CAN1.sendMessage( & canMsgRcv);

          if (noFMUX || hasAnalogicButtons) { // Fake FMUX Buttons in the car
            canMsgSnd.data[0] = 0x00;
            canMsgSnd.data[1] = 0x00;
            canMsgSnd.data[2] = 0x00;
            canMsgSnd.data[3] = 0x00;
            canMsgSnd.data[4] = 0x00;
            canMsgSnd.data[5] = 0x02;
            canMsgSnd.data[6] = 0x00; // Volume potentiometer button
            canMsgSnd.data[7] = 0x00;
            canMsgSnd.can_id = 0x122;
            canMsgSnd.can_dlc = 8;
            CAN1.sendMessage( & canMsgSnd);
            if (Send_CAN2010_ForgedMessages) {
              CAN0.sendMessage( & canMsgSnd);
            }
          }
        }
      } else if (id == 0xA2 && noFMUX && steeringWheelCommands_Type == 1) { // Steering wheel commands - C4 I / C5 X7
        // Fake FMUX Buttons in the car
        canMsgSnd.data[0] = 0x00;
        canMsgSnd.data[1] = 0x00;
        canMsgSnd.data[2] = 0x00;
        canMsgSnd.data[3] = 0x00;
        canMsgSnd.data[4] = 0x00;
        canMsgSnd.data[5] = 0x02;
        canMsgSnd.data[6] = 0x00; // Volume potentiometer button
        canMsgSnd.data[7] = 0x00;

        if (bitRead(canMsgRcv.data[1], 3)) { // MENU button pushed > MUSIC
          if (!pushA2) {
            canMsgSnd.data[0] = 0x00;
            canMsgSnd.data[1] = 0x20;
            canMsgSnd.data[2] = 0x00;
            canMsgSnd.data[3] = 0x00;
            canMsgSnd.data[4] = 0x00;
            canMsgSnd.data[5] = 0x02;
            canMsgSnd.data[6] = 0x00; // Volume potentiometer button
            canMsgSnd.data[7] = 0x00;
            pushA2 = true;
          }
        } else if (bitRead(canMsgRcv.data[1], 2)) { // MODE button pushed > NAV
          if (!pushA2) {
            canMsgSnd.data[0] = 0x00;
            canMsgSnd.data[1] = 0x08;
            canMsgSnd.data[2] = 0x00;
            canMsgSnd.data[3] = 0x00;
            canMsgSnd.data[4] = 0x00;
            canMsgSnd.data[5] = 0x02;
            canMsgSnd.data[6] = 0x00; // Volume potentiometer button
            canMsgSnd.data[7] = 0x00;
            pushA2 = true;
          }
        } else if (bitRead(canMsgRcv.data[1], 4)) { // ESC button pushed > APPS
          if (!pushA2) {
            canMsgSnd.data[0] = 0x00;
            canMsgSnd.data[1] = 0x40;
            canMsgSnd.data[2] = 0x00;
            canMsgSnd.data[3] = 0x00;
            canMsgSnd.data[4] = 0x00;
            canMsgSnd.data[5] = 0x02;
            canMsgSnd.data[6] = 0x00; // Volume potentiometer button
            canMsgSnd.data[7] = 0x00;
            pushA2 = true;
          }
        } else if (bitRead(canMsgRcv.data[1], 5)) { // OK button pushed > PHONE
          if (!pushA2) {
            canMsgSnd.data[0] = 0x00;
            canMsgSnd.data[1] = 0x04;
            canMsgSnd.data[2] = 0x08;
            canMsgSnd.data[3] = 0x00;
            canMsgSnd.data[4] = 0x00;
            canMsgSnd.data[5] = 0x02;
            canMsgSnd.data[6] = 0x00; // Volume potentiometer button
            canMsgSnd.data[7] = 0x00;
            pushA2 = true;
          }
        } else {
          pushA2 = false;
          CAN1.sendMessage( & canMsgRcv);
        }
        canMsgSnd.can_id = 0x122;
        canMsgSnd.can_dlc = 8;
        CAN1.sendMessage( & canMsgSnd);
        if (Send_CAN2010_ForgedMessages) {
          CAN0.sendMessage( & canMsgSnd);
        }
      } else if (id == 0xA2 && noFMUX && (steeringWheelCommands_Type == 2 || steeringWheelCommands_Type == 3 || steeringWheelCommands_Type == 4 || steeringWheelCommands_Type == 5)) { // Steering wheel commands - C4 I / C5 X7
        // Fake FMUX Buttons in the car
        canMsgSnd.data[0] = 0x00;
        canMsgSnd.data[1] = 0x00;
        canMsgSnd.data[2] = 0x00;
        canMsgSnd.data[3] = 0x00;
        canMsgSnd.data[4] = 0x00;
        canMsgSnd.data[5] = 0x02;
        canMsgSnd.data[6] = 0x00; // Volume potentiometer button
        canMsgSnd.data[7] = 0x00;

        if (bitRead(canMsgRcv.data[1], 3)) { // MENU button pushed > MENU
          if (!pushA2) {
            canMsgSnd.data[0] = 0x80;
            canMsgSnd.data[1] = 0x00;
            canMsgSnd.data[2] = 0x00;
            canMsgSnd.data[3] = 0x00;
            canMsgSnd.data[4] = 0x00;
            canMsgSnd.data[5] = 0x02;
            canMsgSnd.data[6] = 0x00; // Volume potentiometer button
            canMsgSnd.data[7] = 0x00;
            pushA2 = true;
          }
        } else if (bitRead(canMsgRcv.data[1], 2) && (steeringWheelCommands_Type == 3 || steeringWheelCommands_Type == 5)) { // Right push button / MODE/SRC > SRC
          if (!pushA2) {
            canMsgSnd.data[0] = 0x40;
            canMsgSnd.data[1] = 0x00;
            canMsgSnd.data[2] = 0x00;
            canMsgSnd.data[3] = 0x00;
            canMsgSnd.data[4] = 0x00;
            canMsgSnd.data[5] = 0x02;
            canMsgSnd.data[6] = 0x00; // Volume potentiometer button
            canMsgSnd.data[7] = 0x00;
            pushA2 = true;
          }
        } else if (bitRead(canMsgRcv.data[1], 4) && steeringWheelCommands_Type == 4) { // ESC button pushed > SRC
          if (!pushA2) {
            canMsgSnd.data[0] = 0x40;
            canMsgSnd.data[1] = 0x00;
            canMsgSnd.data[2] = 0x00;
            canMsgSnd.data[3] = 0x00;
            canMsgSnd.data[4] = 0x00;
            canMsgSnd.data[5] = 0x02;
            canMsgSnd.data[6] = 0x00; // Volume potentiometer button
            canMsgSnd.data[7] = 0x00;
            pushA2 = true;
          }
        } else if (bitRead(canMsgRcv.data[1], 4) && steeringWheelCommands_Type == 5) { // ESC button pushed > TRIP
          if (!pushA2) {
            pushTRIP = true;
            pushA2 = true;
          }
        } else if (bitRead(canMsgRcv.data[1], 2) && steeringWheelCommands_Type == 4) { // Right push button / MODE/SRC > TRIP
          if (!pushA2) {
            pushTRIP = true;
            pushA2 = true;
          }
        } else {
          pushA2 = false;
          CAN1.sendMessage( & canMsgRcv);
        }
        canMsgSnd.can_id = 0x122;
        canMsgSnd.can_dlc = 8;
        CAN1.sendMessage( & canMsgSnd);
        if (Send_CAN2010_ForgedMessages) {
          CAN0.sendMessage( & canMsgSnd);
        }

        if (pushTRIP) {
          pushTRIP = false;

          canMsgSnd.data[0] = statusTRIP[0];
          bitWrite(canMsgSnd.data[0], 3, 1);
          canMsgSnd.data[1] = statusTRIP[1];
          canMsgSnd.data[2] = statusTRIP[2];
          canMsgSnd.data[3] = statusTRIP[3];
          canMsgSnd.data[4] = statusTRIP[4];
          canMsgSnd.data[5] = statusTRIP[5];
          canMsgSnd.data[6] = statusTRIP[6];
          canMsgSnd.data[7] = statusTRIP[7];
          canMsgSnd.can_id = 0x221;
          canMsgSnd.can_dlc = 8;
          CAN1.sendMessage( & canMsgSnd);
          if (Send_CAN2010_ForgedMessages) {
            CAN0.sendMessage( & canMsgSnd);
          }
        }
      } else if (id == 0x217 && len == 8) { // Cache cluster status (CMB)
        statusCMB[0] = canMsgRcv.data[0];
        statusCMB[1] = canMsgRcv.data[1];
        statusCMB[2] = canMsgRcv.data[2];
        statusCMB[3] = canMsgRcv.data[3];
        statusCMB[4] = canMsgRcv.data[4];
        statusCMB[5] = canMsgRcv.data[5];
        statusCMB[6] = canMsgRcv.data[6];
        statusCMB[7] = canMsgRcv.data[7];

        CAN1.sendMessage( & canMsgRcv);
      } else if (id == 0x1D0 && len == 7 && EngineRunning) { // No fan activated if the engine is not ON on old models
        LeftTemp = canMsgRcv.data[5];
        RightTemp = canMsgRcv.data[6];
        if (LeftTemp == RightTemp) { // No other way to detect MONO mode
          Mono = true;
          LeftTemp = LeftTemp + 64;
        } else {
          Mono = false;
        }

        FanOff = false;
        // Fan Speed BSI_2010 = "41" (Off) > "49" (Full speed)
        tmpVal = canMsgRcv.data[2];
        if (tmpVal == 15) {
          FanOff = true;
          FanSpeed = 0x41;
        } else {
          FanSpeed = (tmpVal + 66);
        }

        // Position Fan
        tmpVal = canMsgRcv.data[3];

        if (tmpVal == 0x40) {
          FootAerator = false;
          WindShieldAerator = true;
          CentralAerator = false;
        } else if (tmpVal == 0x30) {
          FootAerator = false;
          WindShieldAerator = false;
          CentralAerator = true;
        } else if (tmpVal == 0x20) {
          FootAerator = true;
          WindShieldAerator = false;
          CentralAerator = false;
        } else if (tmpVal == 0x70) {
          FootAerator = false;
          WindShieldAerator = true;
          CentralAerator = true;
        } else if (tmpVal == 0x80) {
          FootAerator = true;
          WindShieldAerator = true;
          CentralAerator = true;
        } else if (tmpVal == 0x50) {
          FootAerator = true;
          WindShieldAerator = false;
          CentralAerator = true;
        } else if (tmpVal == 0x10) {
          FootAerator = false;
          WindShieldAerator = false;
          CentralAerator = false;
        } else if (tmpVal == 0x60) {
          FootAerator = true;
          WindShieldAerator = true;
          CentralAerator = false;
        } else {
          FootAerator = false;
          WindShieldAerator = false;
          CentralAerator = false;
        }

        tmpVal = canMsgRcv.data[4];
        if (tmpVal == 0x10) {
          DeMist = true;
          AirRecycle = false;
        } else if (tmpVal == 0x30) {
          AirRecycle = true;
        } else {
          AirRecycle = false;
        }

        AutoFan = false;
        DeMist = false;

        tmpVal = canMsgRcv.data[0];
        if (tmpVal == 0x11) {
          DeMist = true;
          AirConditioningON = true;
          FanOff = false;
        } else if (tmpVal == 0x12) {
          DeMist = true;
          AirConditioningON = false;
          FanOff = false;
        } else if (tmpVal == 0x21) {
          DeMist = true;
          AirConditioningON = true;
          FanOff = false;
        } else if (tmpVal == 0xA2) {
          FanOff = true;
          AirConditioningON = false;
        } else if (tmpVal == 0x22) {
          AirConditioningON = false;
        } else if (tmpVal == 0x20) {
          AirConditioningON = true;
        } else if (tmpVal == 0x02) {
          AirConditioningON = false;
          AutoFan = false;
        } else if (tmpVal == 0x00) {
          AirConditioningON = true;
          AutoFan = true;
        }

        if (!FootAerator && !WindShieldAerator && CentralAerator) {
          FanPosition = 0x34;
        } else if (FootAerator && WindShieldAerator && CentralAerator) {
          FanPosition = 0x84;
        } else if (!FootAerator && WindShieldAerator && CentralAerator) {
          FanPosition = 0x74;
        } else if (FootAerator && !WindShieldAerator && CentralAerator) {
          FanPosition = 0x54;
        } else if (FootAerator && !WindShieldAerator && !CentralAerator) {
          FanPosition = 0x24;
        } else if (!FootAerator && WindShieldAerator && !CentralAerator) {
          FanPosition = 0x44;
        } else if (FootAerator && WindShieldAerator && !CentralAerator) {
          FanPosition = 0x64;
        } else {
          FanPosition = 0x04; // Nothing
        }

        if (DeMist) {
          FanSpeed = 0x10;
          FanPosition = FanPosition + 16;
        } else if (AutoFan) {
          FanSpeed = 0x10;
        }

        if (FanOff) {
          AirConditioningON = false;
          FanSpeed = 0x41;
          LeftTemp = 0x00;
          RightTemp = 0x00;
          FanPosition = 0x04;
        }

        if (AirConditioningON) {
          canMsgSnd.data[0] = 0x01; // A/C ON - Auto Soft : "00" / Auto Normal "01" / Auto Fast "02"
        } else {
          canMsgSnd.data[0] = 0x09; // A/C OFF - Auto Soft : "08" / Auto Normal "09" / Auto Fast "0A"
        }

        canMsgSnd.data[1] = 0x00;
        canMsgSnd.data[2] = 0x00;
        canMsgSnd.data[3] = LeftTemp;
        canMsgSnd.data[4] = RightTemp;
        canMsgSnd.data[5] = FanSpeed;
        canMsgSnd.data[6] = FanPosition;
        canMsgSnd.data[7] = 0x00;
        canMsgSnd.can_id = 0x350;
        canMsgSnd.can_dlc = 8;
        CAN1.sendMessage( & canMsgSnd);
        if (Send_CAN2010_ForgedMessages) {
          CAN0.sendMessage( & canMsgSnd);
        }
      } else if (id == 0xF6 && len == 8) {
        tmpVal = canMsgRcv.data[0];
        if (tmpVal > 128) {
          if (!Ignition && SerialEnabled) {
            Serial.println("Ignition ON");
          }

          Ignition = true;
        } else {
          if (Ignition && SerialEnabled) {
            Serial.println("Ignition OFF");
          }

          Ignition = false;
        }

        tmpVal = (canMsgRcv.data[5] >> 1) - 40; // Temperatures can be negative but we only have 0 > 255, the new range is starting from -40°C
        if (Temperature != tmpVal) {
          Temperature = tmpVal;

          if (SerialEnabled) {
            Serial.print("Ext. Temperature: ");
            Serial.print(tmpVal);
            Serial.println("°C");
          }
        }

        CAN1.sendMessage( & canMsgRcv);
      } else if (id == 0x168 && len == 8) { // Instrument Panel - WIP
        canMsgSnd.data[0] = canMsgRcv.data[0]; // Alerts
        canMsgSnd.data[1] = canMsgRcv.data[1];
        canMsgSnd.data[2] = canMsgRcv.data[2];
        canMsgSnd.data[3] = canMsgRcv.data[3];
        canMsgSnd.data[4] = canMsgRcv.data[4];
        canMsgSnd.data[5] = canMsgRcv.data[5];
        bitWrite(canMsgSnd.data[6], 7, 0);
        bitWrite(canMsgSnd.data[6], 6, 1); // Ambiance
        bitWrite(canMsgSnd.data[6], 5, 1); // EMF availability
        bitWrite(canMsgSnd.data[6], 4, bitRead(canMsgRcv.data[5], 0)); // Gearbox report while driving
        bitWrite(canMsgSnd.data[6], 3, bitRead(canMsgRcv.data[6], 7)); // Gearbox report while driving
        bitWrite(canMsgSnd.data[6], 2, bitRead(canMsgRcv.data[6], 6)); // Gearbox report while driving
        bitWrite(canMsgSnd.data[6], 1, bitRead(canMsgRcv.data[6], 5)); // Gearbox report while driving
        bitWrite(canMsgSnd.data[6], 0, 0);
        canMsgSnd.data[7] = canMsgRcv.data[7];
        canMsgSnd.can_id = 0x168;
        canMsgSnd.can_dlc = 8;

        CAN1.sendMessage( & canMsgSnd);
        if (Send_CAN2010_ForgedMessages) { // Will generate some light issues on the instrument panel
          CAN0.sendMessage( & canMsgSnd);
        }
      } else if (id == 0x120 && generatePOPups) { // Alerts journal / Diagnostic > Popup notifications - Work in progress
        // C5 (X7) Cluster is connected to CAN High Speed, no notifications are sent on CAN Low Speed, let's rebuild alerts from the journal (slighly slower than original alerts)
        // Bloc 1
        if (bitRead(canMsgRcv.data[0], 7) == 0 && bitRead(canMsgRcv.data[0], 6) == 1) {
          sendPOPup(bitRead(canMsgRcv.data[1], 7), 5, 1, 0x00); // Engine oil pressure fault: stop the vehicle (STOP)
          sendPOPup(bitRead(canMsgRcv.data[1], 6), 1, 1, 0x00); // Engine temperature fault: stop the vehicle (STOP)
          sendPOPup(bitRead(canMsgRcv.data[1], 5), 138, 6, 0x00); // Charging system fault: repair needed (WARNING)
          sendPOPup(bitRead(canMsgRcv.data[1], 4), 106, 1, 0x00); // Braking system fault: stop the vehicle (STOP)
          // bitRead(canMsgRcv.data[1], 3); // N/A
          sendPOPup(bitRead(canMsgRcv.data[1], 2), 109, 2, 0x00); // Power steering fault: stop the vehicle (STOP)
          sendPOPup(bitRead(canMsgRcv.data[1], 1), 3, 4, 0x00); // Top up coolant level (WARNING)
          // bitRead(canMsgRcv.data[1], 0); // Fault with LKA (WARNING)
          sendPOPup(bitRead(canMsgRcv.data[2], 7), 4, 4, 0x00); // Top up engine oil level (WARNING)
          // bitRead(canMsgRcv.data[2], 6); // N/A
          notificationParameters = 0x00;
          bitWrite(notificationParameters, 7, bitRead(canMsgRcv.data[2], 5)); // Front right door
          bitWrite(notificationParameters, 6, bitRead(canMsgRcv.data[2], 4)); // Front left door
          bitWrite(notificationParameters, 5, bitRead(canMsgRcv.data[2], 3)); // Rear right door
          bitWrite(notificationParameters, 4, bitRead(canMsgRcv.data[2], 2)); // Rear left door
          bitWrite(notificationParameters, 3, bitRead(canMsgRcv.data[2], 0)); // Boot open
          // bitWrite(notificationParameters, 2, ?); // Hood open
          bitWrite(notificationParameters, 1, bitRead(canMsgRcv.data[3], 7)); // Rear Screen open
          // bitWrite(notificationParameters, 0, ?); // Fuel door open
          sendPOPup((bitRead(canMsgRcv.data[2], 5) || bitRead(canMsgRcv.data[2], 4) || bitRead(canMsgRcv.data[2], 3) || bitRead(canMsgRcv.data[2], 2) || bitRead(canMsgRcv.data[2], 0) || bitRead(canMsgRcv.data[3], 7)), 8, 8, notificationParameters); // Left hand front door opened (WARNING) || Right hand front door opened (WARNING) || Left hand rear door opened (WARNING) || Right hand rear door opened (WARNING) || Boot open (WARNING) || Rear screen open (WARNING)
          // bitRead(canMsgRcv.data[2], 1); // N/A
          sendPOPup(bitRead(canMsgRcv.data[3], 6), 107, 2, 0x00); // ESP/ASR system fault, repair the vehicle (WARNING)
          // bitRead(canMsgRcv.data[3], 5); // Battery charge fault, stop the vehicle (WARNING)
          // bitRead(canMsgRcv.data[3], 4); // N/A
          sendPOPup(bitRead(canMsgRcv.data[3], 3), 125, 6, 0x00); // Water in diesel fuel filter (WARNING)
          sendPOPup(bitRead(canMsgRcv.data[3], 2), 103, 6, 0x00); // Have brake pads replaced (WARNING)
          sendPOPup(bitRead(canMsgRcv.data[3], 1), 224, 10, 0x00); // Fuel level low (INFO)
          sendPOPup(bitRead(canMsgRcv.data[3], 0), 120, 6, 0x00); // Airbag(s) or seatbelt(s) pretensioner fault(s) (WARNING)
          // bitRead(canMsgRcv.data[4], 7); // N/A
          // bitRead(canMsgRcv.data[4], 6); // Engine fault, repair the vehicle (WARNING)
          sendPOPup(bitRead(canMsgRcv.data[4], 5), 106, 2, 0x00); // ABS braking system fault, repair the vehicle (WARNING)
          sendPOPup(bitRead(canMsgRcv.data[4], 4), 15, 4, 0x00); // Particle filter is full, please drive 20min to clean it (WARNING)
          // bitRead(canMsgRcv.data[4], 3); // N/A
          sendPOPup(bitRead(canMsgRcv.data[4], 2), 129, 6, 0x00); // Particle filter additive level low (WARNING)
          // bitRead(canMsgRcv.data[4], 1); // N/A
          sendPOPup(bitRead(canMsgRcv.data[4], 0), 17, 4, 0x00); // Suspension fault, repair the vehicle (WARNING)
          // bitRead(canMsgRcv.data[5], 7); // Preheating deactivated, battery charge too low (INFO)
          // bitRead(canMsgRcv.data[5], 6); // Preheating deactivated, fuel level too low (INFO)
          // bitRead(canMsgRcv.data[5], 5); // Check the centre brake lamp (WARNING)
          // bitRead(canMsgRcv.data[5], 4); // Retractable roof mechanism fault (WARNING)
          // sendPOPup(bitRead(canMsgRcv.data[5], 3), ?, 8, 0x00); // Steering lock fault, repair the vehicle (WARNING)
          sendPOPup(bitRead(canMsgRcv.data[5], 2), 131, 6, 0x00); // Electronic immobiliser fault (WARNING)
          // bitRead(canMsgRcv.data[5], 1); // N/A
          // bitRead(canMsgRcv.data[5], 0); // Roof operation not possible, system temperature too high (WARNING)
          // bitRead(canMsgRcv.data[6], 7); // Roof operation not possible, start the engine (WARNING)
          // bitRead(canMsgRcv.data[6], 6); // Roof operation not possible, apply parking brake (WARNING)
          // bitRead(canMsgRcv.data[6], 5); // Hybrid system fault (STOP)
          // bitRead(canMsgRcv.data[6], 4); // Automatic headlamp adjustment fault (WARNING)
          // bitRead(canMsgRcv.data[6], 3); // Hybrid system fault (WARNING)
          // bitRead(canMsgRcv.data[6], 2); // Hybrid system fault: speed restricted (WARNING)
          sendPOPup(bitRead(canMsgRcv.data[6], 1), 223, 10, 0x00); // Top Up screenwash fluid level (INFO)
          sendPOPup(bitRead(canMsgRcv.data[6], 0), 227, 14, 0x00); // Replace remote control battery (INFO)
          // bitRead(canMsgRcv.data[7], 7); // N/A
          // bitRead(canMsgRcv.data[7], 6); // Preheating deactivated, set the clock (INFO)
          // bitRead(canMsgRcv.data[7], 5); // Trailer connection fault (WARNING)
          // bitRead(canMsgRcv.data[7], 4); // N/A
          // bitRead(canMsgRcv.data[7], 3); // Tyre under-inflation (WARNING)
          // bitRead(canMsgRcv.data[7], 2); // Driving aid camera limited visibility (INFO)
          // bitRead(canMsgRcv.data[7], 1); // N/A
          // bitRead(canMsgRcv.data[7], 0); // N/A
        }

        // Bloc 2
        if (bitRead(canMsgRcv.data[0], 7) == 1 && bitRead(canMsgRcv.data[0], 6) == 0) {
          // bitRead(canMsgRcv.data[1], 7); // N/A
          // bitRead(canMsgRcv.data[1], 6); // Electric mode not available : Particle filter regenerating (INFO)
          // bitRead(canMsgRcv.data[1], 5); // N/A
          notificationParameters = 0x00;
          bitWrite(notificationParameters, 7, bitRead(canMsgRcv.data[1], 4)); // Front left tyre
          bitWrite(notificationParameters, 6, bitRead(canMsgRcv.data[1], 3)); // Front right tyre
          bitWrite(notificationParameters, 5, bitRead(canMsgRcv.data[1], 2)); // Rear right tyre
          bitWrite(notificationParameters, 4, bitRead(canMsgRcv.data[1], 1)); // Rear left tyre
          sendPOPup((bitRead(canMsgRcv.data[1], 4) || bitRead(canMsgRcv.data[1], 3) || bitRead(canMsgRcv.data[1], 2) || bitRead(canMsgRcv.data[1], 1)), 13, 6, notificationParameters); // Puncture: Replace or repair the wheel (STOP)
          notificationParameters = 0x00;
          bitWrite(notificationParameters, 7, bitRead(canMsgRcv.data[1], 0)); // Front right sidelamp
          bitWrite(notificationParameters, 6, bitRead(canMsgRcv.data[2], 7)); // Front left sidelamp
          bitWrite(notificationParameters, 5, bitRead(canMsgRcv.data[2], 6)); // Rear right sidelamp
          bitWrite(notificationParameters, 4, bitRead(canMsgRcv.data[2], 5)); // Rear left sidelamp
          sendPOPup((bitRead(canMsgRcv.data[1], 0) || bitRead(canMsgRcv.data[2], 7) || bitRead(canMsgRcv.data[2], 6) || bitRead(canMsgRcv.data[2], 5)), 160, 6, notificationParameters); // Check sidelamps (WARNING)
          notificationParameters = 0x00;
          bitWrite(notificationParameters, 7, bitRead(canMsgRcv.data[2], 4)); // Right dipped beam headlamp
          bitWrite(notificationParameters, 6, bitRead(canMsgRcv.data[2], 3)); // Left dipped beam headlamp
          sendPOPup((bitRead(canMsgRcv.data[2], 4) || bitRead(canMsgRcv.data[2], 3)), 154, 6, notificationParameters); // Check the dipped beam headlamps (WARNING)
          notificationParameters = 0x00;
          bitWrite(notificationParameters, 7, bitRead(canMsgRcv.data[2], 2)); // Right main beam headlamp
          bitWrite(notificationParameters, 6, bitRead(canMsgRcv.data[2], 1)); // Left main beam headlamp
          sendPOPup((bitRead(canMsgRcv.data[2], 2) || bitRead(canMsgRcv.data[2], 1)), 155, 6, notificationParameters); // Check the main beam headlamps (WARNING)
          notificationParameters = 0x00;
          bitWrite(notificationParameters, 7, bitRead(canMsgRcv.data[2], 0)); // Right brake lamp
          bitWrite(notificationParameters, 6, bitRead(canMsgRcv.data[3], 7)); // Left brake lamp
          sendPOPup((bitRead(canMsgRcv.data[2], 0) || bitRead(canMsgRcv.data[3], 7)), 156, 6, notificationParameters); // Check the RH brake lamp (WARNING) || Check the LH brake lamp (WARNING)
          notificationParameters = 0x00;
          bitWrite(notificationParameters, 7, bitRead(canMsgRcv.data[3], 6)); // Front right foglamp
          bitWrite(notificationParameters, 6, bitRead(canMsgRcv.data[3], 5)); // Front left foglamp
          bitWrite(notificationParameters, 5, bitRead(canMsgRcv.data[3], 4)); // Rear right foglamp
          bitWrite(notificationParameters, 4, bitRead(canMsgRcv.data[3], 3)); // Rear left foglamp
          sendPOPup((bitRead(canMsgRcv.data[3], 6) || bitRead(canMsgRcv.data[3], 5) || bitRead(canMsgRcv.data[3], 4) || bitRead(canMsgRcv.data[3], 3)), 157, 6, notificationParameters); // Check the front foglamps (WARNING) || Check the front foglamps (WARNING) || Check the rear foglamps (WARNING) || Check the rear foglamps (WARNING)
          notificationParameters = 0x00;
          bitWrite(notificationParameters, 7, bitRead(canMsgRcv.data[3], 2)); // Front right direction indicator
          bitWrite(notificationParameters, 6, bitRead(canMsgRcv.data[3], 1)); // Front left direction indicator
          bitWrite(notificationParameters, 5, bitRead(canMsgRcv.data[3], 0)); // Rear right direction indicator
          bitWrite(notificationParameters, 4, bitRead(canMsgRcv.data[4], 7)); // Rear left direction indicator
          sendPOPup((bitRead(canMsgRcv.data[3], 2) || bitRead(canMsgRcv.data[3], 1) || bitRead(canMsgRcv.data[3], 0) || bitRead(canMsgRcv.data[4], 7)), 159, 6, notificationParameters); // Check the direction indicators (WARNING)
          notificationParameters = 0x00;
          bitWrite(notificationParameters, 7, bitRead(canMsgRcv.data[4], 6)); // Right reversing lamp
          bitWrite(notificationParameters, 6, bitRead(canMsgRcv.data[4], 5)); // Left reversing lamp
          sendPOPup((bitRead(canMsgRcv.data[4], 6) || bitRead(canMsgRcv.data[4], 5)), 159, 6, notificationParameters); // Check the reversing lamp(s) (WARNING)
          // bitRead(canMsgRcv.data[4], 4); // N/A
          // bitRead(canMsgRcv.data[4], 3); // N/A
          // bitRead(canMsgRcv.data[4], 2); // N/A
          // bitRead(canMsgRcv.data[4], 1); // N/A
          // bitRead(canMsgRcv.data[4], 0); // N/A
          // bitRead(canMsgRcv.data[5], 7); // N/A
          // bitRead(canMsgRcv.data[5], 6); // N/A
          // bitRead(canMsgRcv.data[5], 5); // N/A
          sendPOPup(bitRead(canMsgRcv.data[5], 4), 136, 8, 0x00); // Parking assistance system fault (WARNING)
          // bitRead(canMsgRcv.data[5], 3); // N/A
          // bitRead(canMsgRcv.data[5], 2); // N/A
          notificationParameters = 0x00;
          bitWrite(notificationParameters, 7, bitRead(canMsgRcv.data[5], 1)); // Front left tyre
          bitWrite(notificationParameters, 6, bitRead(canMsgRcv.data[5], 0)); // Front right tyre
          bitWrite(notificationParameters, 5, bitRead(canMsgRcv.data[6], 7)); // Rear right tyre
          bitWrite(notificationParameters, 4, bitRead(canMsgRcv.data[6], 5)); // Rear left tyre
          sendPOPup((bitRead(canMsgRcv.data[5], 1) || bitRead(canMsgRcv.data[5], 0) || bitRead(canMsgRcv.data[6], 7) || bitRead(canMsgRcv.data[6], 5)), 13, 8, notificationParameters); // Adjust tyre pressures (WARNING)
          // bitRead(canMsgRcv.data[6], 5); // Switch off lighting (INFO)
          // bitRead(canMsgRcv.data[6], 4); // N/A
          sendPOPup((bitRead(canMsgRcv.data[6], 3) || bitRead(canMsgRcv.data[6], 1)), 190, 8, 0x00); // Emissions fault (WARNING)
          sendPOPup(bitRead(canMsgRcv.data[6], 2), 192, 8, 0x00); // Emissions fault: Starting Prevented (WARNING)
          // bitRead(canMsgRcv.data[6], 0); // N/A
          // bitRead(canMsgRcv.data[7], 7); // N/A
          // bitRead(canMsgRcv.data[7], 6); // N/A
          sendPOPup(bitRead(canMsgRcv.data[7], 5), 215, 10, 0x00); // "P" (INFO)
          sendPOPup(bitRead(canMsgRcv.data[7], 4), 216, 10, 0x00); // Ice warning (INFO)
          bitWrite(statusOpenings, 7, bitRead(canMsgRcv.data[7], 3)); // Front right door
          bitWrite(statusOpenings, 6, bitRead(canMsgRcv.data[7], 2)); // Front left door
          bitWrite(statusOpenings, 5, bitRead(canMsgRcv.data[7], 1)); // Rear right door
          bitWrite(statusOpenings, 4, bitRead(canMsgRcv.data[7], 0)); // Rear left door
          sendPOPup((bitRead(canMsgRcv.data[7], 3) || bitRead(canMsgRcv.data[7], 2) || bitRead(canMsgRcv.data[7], 1) || bitRead(canMsgRcv.data[7], 0) || bitRead(statusOpenings, 3) || bitRead(statusOpenings, 1)), 222, 8, statusOpenings); // Front right door opened (INFO) || Front left door opened (INFO) || Rear right door opened (INFO) || Rear left door opened (INFO)
        }

        // Bloc 3
        if (bitRead(canMsgRcv.data[0], 7) == 1 && bitRead(canMsgRcv.data[0], 6) == 1) {
          bitWrite(statusOpenings, 3, bitRead(canMsgRcv.data[1], 7)); // Boot open
          // bitWrite(statusOpenings, 2, ?); // Hood open
          bitWrite(statusOpenings, 1, bitRead(canMsgRcv.data[1], 5)); // Rear Screen open
          // bitWrite(statusOpenings, 0, ?); // Fuel door open
          sendPOPup((bitRead(canMsgRcv.data[1], 7) || bitRead(canMsgRcv.data[1], 5) ||  bitRead(statusOpenings, 7) ||  bitRead(statusOpenings, 6) ||  bitRead(statusOpenings, 5) ||  bitRead(statusOpenings, 4)), 222, 8, statusOpenings); // Boot open (INFO) || Rear Screen open (INFO)
          // bitRead(canMsgRcv.data[1], 6); // Collision detection risk system fault (INFO)
          // bitRead(canMsgRcv.data[1], 4); // N/A
          // bitRead(canMsgRcv.data[1], 3); // N/A
          // bitRead(canMsgRcv.data[1], 2); // N/A
          // bitRead(canMsgRcv.data[1], 1); // N/A
          // bitRead(canMsgRcv.data[1], 0); // N/A
          // bitRead(canMsgRcv.data[2], 7); // N/A
          // bitRead(canMsgRcv.data[2], 6); // N/A
          // bitRead(canMsgRcv.data[2], 5); // N/A
          sendPOPup(bitRead(canMsgRcv.data[2], 4), 100, 6, 0x00); // Parking brake fault (WARNING)
          // bitRead(canMsgRcv.data[2], 3); // Active spoiler fault: speed restricted (WARNING)
          // bitRead(canMsgRcv.data[2], 2); // Automatic braking system fault (INFO)
          // bitRead(canMsgRcv.data[2], 1); // Directional headlamps fault (WARNING)
          // bitRead(canMsgRcv.data[2], 0); // N/A
          // bitRead(canMsgRcv.data[3], 7); // N/A
          // bitRead(canMsgRcv.data[3], 6); // N/A
          // bitRead(canMsgRcv.data[3], 5); // N/A
          // bitRead(canMsgRcv.data[3], 4); // N/A
          // bitRead(canMsgRcv.data[3], 3); // N/A
          if (isBVMP) {
            sendPOPup(bitRead(canMsgRcv.data[3], 2), 122, 4, 0x00); // Gearbox fault (WARNING)
          } else {
            sendPOPup(bitRead(canMsgRcv.data[3], 2), 110, 4, 0x00); // Gearbox fault (WARNING)
          }
          // bitRead(canMsgRcv.data[3], 1); // N/A
          // bitRead(canMsgRcv.data[3], 0); // N/A
          // bitRead(canMsgRcv.data[4], 7); // N/A
          // bitRead(canMsgRcv.data[4], 6); // N/A
          // bitRead(canMsgRcv.data[4], 5); // N/A
          // bitRead(canMsgRcv.data[4], 4); // N/A
          // bitRead(canMsgRcv.data[4], 3); // N/A
          // bitRead(canMsgRcv.data[4], 2); // Engine fault (WARNING)
          sendPOPup(bitRead(canMsgRcv.data[4], 1), 17, 3, 0x00); // Suspension fault: limit your speed to 90km/h (WARNING)
          // bitRead(canMsgRcv.data[4], 0); // N/A
          // bitRead(canMsgRcv.data[5], 7); // N/A
          // bitRead(canMsgRcv.data[5], 6); // N/A
          // bitRead(canMsgRcv.data[5], 5); // N/A
          // bitRead(canMsgRcv.data[5], 4); // N/A
          notificationParameters = 0x00;
          bitWrite(notificationParameters, 7, bitRead(canMsgRcv.data[5], 3)); // Front left tyre
          bitWrite(notificationParameters, 6, bitRead(canMsgRcv.data[5], 2)); // Front right tyre
          bitWrite(notificationParameters, 5, bitRead(canMsgRcv.data[5], 1)); // Rear right tyre
          bitWrite(notificationParameters, 4, bitRead(canMsgRcv.data[5], 0)); // Rear left tyre
          sendPOPup((bitRead(canMsgRcv.data[5], 3) || bitRead(canMsgRcv.data[5], 2) || bitRead(canMsgRcv.data[5], 1) || bitRead(canMsgRcv.data[5], 0)), 229, 10, notificationParameters); // Sensor fault: Left hand front tyre pressure not monitored (INFO)
          sendPOPup(bitRead(canMsgRcv.data[6], 7), 18, 4, 0x00); // Suspension fault: repair the vehicle (WARNING)
          sendPOPup(bitRead(canMsgRcv.data[6], 6), 109, 4, 0x00); // Power steering fault: repair the vehicle (WARNING)
          // bitRead(canMsgRcv.data[6], 5); // N/A
          // bitRead(canMsgRcv.data[6], 4); // N/A
          // bitRead(canMsgRcv.data[6], 3); // Inter-vehicle time measurement fault (WARNING)
          // bitRead(canMsgRcv.data[6], 2); // Engine fault, stop the vehicle (STOP)
          // bitRead(canMsgRcv.data[6], 1); // Fault with LKA (INFO)
          // bitRead(canMsgRcv.data[6], 0); // Tyre under-inflation detection system fault (WARNING)
          notificationParameters = 0x00;
          bitWrite(notificationParameters, 7, bitRead(canMsgRcv.data[7], 7)); // Front left tyre
          bitWrite(notificationParameters, 6, bitRead(canMsgRcv.data[7], 6)); // Front right tyre
          bitWrite(notificationParameters, 5, bitRead(canMsgRcv.data[7], 5)); // Rear right tyre
          //bitWrite(notificationParameters, 4, ?); // Rear left tyre
          sendPOPup((bitRead(canMsgRcv.data[7], 7) || bitRead(canMsgRcv.data[7], 6) || bitRead(canMsgRcv.data[7], 5)), 183, 8, notificationParameters); // Underinflated wheel, ajust pressure and reset (INFO)
          // bitRead(canMsgRcv.data[7], 4); // Spare wheel fitted: driving aids deactivated (INFO)
          // bitRead(canMsgRcv.data[7], 3); // Automatic braking disabled (INFO)
          sendPOPup(bitRead(canMsgRcv.data[7], 2), 188, 6, 0x00); // Refill AdBlue (WARNING)
          sendPOPup(bitRead(canMsgRcv.data[7], 1), 187, 10, 0x00); // Refill AdBlue (INFO)
          sendPOPup(bitRead(canMsgRcv.data[7], 0), 189, 4, 0x00); // Impossible engine start, refill AdBlue (WARNING)
        }

        CAN1.sendMessage( & canMsgRcv); // Forward original frame
      } else if (id == 0x221) { // Trip info
        statusTRIP[0] = canMsgRcv.data[0];
        statusTRIP[1] = canMsgRcv.data[1];
        statusTRIP[2] = canMsgRcv.data[2];
        statusTRIP[3] = canMsgRcv.data[3];
        statusTRIP[4] = canMsgRcv.data[4];
        statusTRIP[5] = canMsgRcv.data[5];
        statusTRIP[6] = canMsgRcv.data[6];
        statusTRIP[7] = canMsgRcv.data[7];
        CAN1.sendMessage( & canMsgRcv); // Forward original frame

        customTimeStamp = (long) hour() * (long) 3600 + minute() * 60 + second();
        daysSinceYearStart = daysSinceYearStartFct();

        canMsgSnd.data[0] = (((1 << 8) - 1) & (customTimeStamp >> (12)));
        canMsgSnd.data[1] = (((1 << 8) - 1) & (customTimeStamp >> (4)));
        canMsgSnd.data[2] = (((((1 << 4) - 1) & (customTimeStamp)) << 4)) + (((1 << 4) - 1) & (daysSinceYearStart >> (8)));
        canMsgSnd.data[3] = (((1 << 8) - 1) & (daysSinceYearStart));
        canMsgSnd.data[4] = 0x00;
        canMsgSnd.data[5] = 0xC0;
        canMsgSnd.data[6] = languageID;
        canMsgSnd.can_id = 0x3F6; // Fake EMF Time frame
        canMsgSnd.can_dlc = 7;

        CAN0.sendMessage( & canMsgSnd);
      } else if (id == 0x128 && len == 8) { // Instrument Panel
        canMsgSnd.data[0] = canMsgRcv.data[4]; // Main driving lights
        bitWrite(canMsgSnd.data[1], 7, bitRead(canMsgRcv.data[6], 7)); // Gearbox report
        bitWrite(canMsgSnd.data[1], 6, bitRead(canMsgRcv.data[6], 6)); // Gearbox report
        bitWrite(canMsgSnd.data[1], 5, bitRead(canMsgRcv.data[6], 5)); // Gearbox report
        bitWrite(canMsgSnd.data[1], 4, bitRead(canMsgRcv.data[6], 4)); // Gearbox report
        bitWrite(canMsgSnd.data[1], 3, bitRead(canMsgRcv.data[6], 3)); // Gearbox report while driving
        bitWrite(canMsgSnd.data[1], 2, bitRead(canMsgRcv.data[6], 2)); // Gearbox report while driving
        bitWrite(canMsgSnd.data[1], 1, bitRead(canMsgRcv.data[6], 1)); // Gearbox report while driving
        bitWrite(canMsgSnd.data[1], 0, bitRead(canMsgRcv.data[6], 0)); // Gearbox report blinking
        bitWrite(canMsgSnd.data[2], 7, bitRead(canMsgRcv.data[7], 7)); // Arrow blinking
        bitWrite(canMsgSnd.data[2], 6, bitRead(canMsgRcv.data[7], 6)); // BVA mode
        bitWrite(canMsgSnd.data[2], 5, bitRead(canMsgRcv.data[7], 5)); // BVA mode
        bitWrite(canMsgSnd.data[2], 4, bitRead(canMsgRcv.data[7], 4)); // BVA mode
        bitWrite(canMsgSnd.data[2], 3, bitRead(canMsgRcv.data[7], 3)); // Arrow type
        bitWrite(canMsgSnd.data[2], 2, bitRead(canMsgRcv.data[7], 2)); // Arrow type
        if (bitRead(canMsgRcv.data[7], 1) == 1 && bitRead(canMsgRcv.data[7], 0) == 0) { // BVMP to BVA
          isBVMP = true;
          bitWrite(canMsgSnd.data[2], 1, 0); // Gearbox type
          bitWrite(canMsgSnd.data[2], 0, 0); // Gearbox type
        } else {
          bitWrite(canMsgSnd.data[2], 1, bitRead(canMsgRcv.data[7], 1)); // Gearbox type
          bitWrite(canMsgSnd.data[2], 0, bitRead(canMsgRcv.data[7], 0)); // Gearbox type
        }
        bitWrite(canMsgSnd.data[3], 7, bitRead(canMsgRcv.data[1], 7)); // Service
        bitWrite(canMsgSnd.data[3], 6, bitRead(canMsgRcv.data[1], 6)); // STOP
        bitWrite(canMsgSnd.data[3], 5, bitRead(canMsgRcv.data[2], 5)); // Child security
        bitWrite(canMsgSnd.data[3], 4, bitRead(canMsgRcv.data[0], 7)); // Passenger Airbag
        bitWrite(canMsgSnd.data[3], 3, bitRead(canMsgRcv.data[3], 2)); // Foot on brake
        bitWrite(canMsgSnd.data[3], 2, bitRead(canMsgRcv.data[3], 1)); // Foot on brake
        bitWrite(canMsgSnd.data[3], 1, bitRead(canMsgRcv.data[0], 5)); // Parking brake
        bitWrite(canMsgSnd.data[3], 0, 0); // Electric parking brake
        bitWrite(canMsgSnd.data[4], 7, bitRead(canMsgRcv.data[0], 2)); // Diesel pre-heating
        bitWrite(canMsgSnd.data[4], 6, bitRead(canMsgRcv.data[1], 4)); // Opening open
        bitWrite(canMsgSnd.data[4], 5, bitRead(canMsgRcv.data[3], 4)); // Automatic parking
        bitWrite(canMsgSnd.data[4], 4, bitRead(canMsgRcv.data[3], 3)); // Automatic parking blinking
        bitWrite(canMsgSnd.data[4], 3, 0); // Automatic high beam
        bitWrite(canMsgSnd.data[4], 2, bitRead(canMsgRcv.data[2], 4)); // ESP Disabled
        bitWrite(canMsgSnd.data[4], 1, bitRead(canMsgRcv.data[2], 3)); // ESP active
        bitWrite(canMsgSnd.data[4], 0, bitRead(canMsgRcv.data[2], 2)); // Active suspension
        bitWrite(canMsgSnd.data[5], 7, bitRead(canMsgRcv.data[0], 4)); // Low fuel
        bitWrite(canMsgSnd.data[5], 6, bitRead(canMsgRcv.data[0], 6)); // Driver seatbelt
        bitWrite(canMsgSnd.data[5], 5, bitRead(canMsgRcv.data[3], 7)); // Driver seatbelt blinking
        bitWrite(canMsgSnd.data[5], 4, bitRead(canMsgRcv.data[0], 1)); // Passenger seatbelt
        bitWrite(canMsgSnd.data[5], 3, bitRead(canMsgRcv.data[3], 6)); // Passenger seatbelt Blinking
        bitWrite(canMsgSnd.data[5], 2, 0); // SCR
        bitWrite(canMsgSnd.data[5], 1, 0); // SCR
        bitWrite(canMsgSnd.data[5], 0, bitRead(canMsgRcv.data[5], 6)); // Rear left seatbelt
        bitWrite(canMsgSnd.data[6], 7, bitRead(canMsgRcv.data[5], 5)); // Rear seatbelt left blinking
        bitWrite(canMsgSnd.data[6], 6, bitRead(canMsgRcv.data[5], 2)); // Rear right seatbelt
        bitWrite(canMsgSnd.data[6], 5, bitRead(canMsgRcv.data[5], 1)); // Rear right seatbelt blinking
        bitWrite(canMsgSnd.data[6], 4, bitRead(canMsgRcv.data[5], 4)); // Rear middle seatbelt
        bitWrite(canMsgSnd.data[6], 3, bitRead(canMsgRcv.data[5], 3)); // Rear middle seatbelt blinking
        bitWrite(canMsgSnd.data[6], 2, bitRead(canMsgRcv.data[5], 7)); // Instrument Panel ON
        bitWrite(canMsgSnd.data[6], 1, bitRead(canMsgRcv.data[2], 1)); // Warnings
        bitWrite(canMsgSnd.data[6], 0, 0); // Passenger protection
        canMsgSnd.data[7] = 0x00;
        canMsgSnd.can_id = 0x128;
        canMsgSnd.can_dlc = 8;

        CAN1.sendMessage( & canMsgSnd);
        if (Send_CAN2010_ForgedMessages) { // Will generate some light issues on the instrument panel
          CAN0.sendMessage( & canMsgSnd);
        }
      } else if (id == 0x3A7 && len == 8) { // Maintenance
        canMsgSnd.data[0] = 0x40;
        // Values are coded with WORD data type HIGH byte fisrt, LOW byte second
        canMsgSnd.data[1] = canMsgRcv.data[5]; // Value x256 +
        canMsgSnd.data[2] = canMsgRcv.data[6]; // Value x1 = Number of days till maintenance (FF FF if disabled)
        canMsgSnd.data[3] = canMsgRcv.data[3]; // Value x256 * 20 +
        canMsgSnd.data[4] = canMsgRcv.data[4]; // Value x20 = km left till maintenance
        canMsgSnd.can_id = 0x3E7; // New maintenance frame ID
        canMsgSnd.can_dlc = 5;

        if (SerialEnabled && !MaintenanceDisplayed) {
          uint16_t tmpVal = (canMsgRcv.data[3] << 8) | canMsgRcv.data[4];
          // Not multiply to 20 to avoid overflow
          Serial.print("Next maintenance in: ");
          if (tmpVal != 0xFFFF) {
            Serial.print(tmpVal);
            Serial.println(" * 20 km");
          }
          tmpVal = (canMsgRcv.data[5] << 8) | canMsgRcv.data[6];
          if (tmpVal != 0xFFFF) {
            Serial.print(tmpVal);
            Serial.println(" days");
          }
          MaintenanceDisplayed = true;
        }

        CAN1.sendMessage( & canMsgSnd);
        if (Send_CAN2010_ForgedMessages) {
          CAN0.sendMessage( & canMsgSnd);
        }
      } else if (id == 0x1A8 && len == 8) { // Cruise control
        CAN1.sendMessage( & canMsgRcv);

        canMsgSnd.data[0] = canMsgRcv.data[1];
        canMsgSnd.data[1] = canMsgRcv.data[2];
        canMsgSnd.data[2] = canMsgRcv.data[0];
        canMsgSnd.data[3] = 0x80;
        canMsgSnd.data[4] = 0x14;
        canMsgSnd.data[5] = 0x7F;
        canMsgSnd.data[6] = 0xFF;
        canMsgSnd.data[7] = 0x98;
        canMsgSnd.can_id = 0x228; // New cruise control frame ID
        canMsgSnd.can_dlc = 8;
        CAN1.sendMessage( & canMsgSnd);
        if (Send_CAN2010_ForgedMessages) {
          CAN0.sendMessage( & canMsgSnd);
        }
      } else if (id == 0x2D7 && len == 5 && listenCAN2004Language) { // CAN2004 Matrix
        tmpVal = canMsgRcv.data[0];
        if (tmpVal > 32) {
          kmL = true;
          tmpVal = tmpVal - 32;
        }

        if (tmpVal <= 32 && languageID_CAN2004 != tmpVal) {
          languageID_CAN2004 = tmpVal;
          EEPROM.update(1, languageID_CAN2004);

          // Change language and unit on ID 608 for CAN2010 Telematic language change
          languageAndUnitNum = (languageID_CAN2004 * 4) + 128;
          if (kmL) {
            languageAndUnitNum = languageAndUnitNum + 1;
          }
          EEPROM.update(0, languageAndUnitNum);

          if (SerialEnabled) {
            Serial.print("CAN2004 Matrix - Change Language: ");
            Serial.print(tmpVal);
            Serial.println();
          }
        } else {
          Serial.print("CAN2004 Matrix - Unsupported language ID: ");
          Serial.print(tmpVal);
          Serial.println();
        }
      } else if (id == 0x361) { // Personalization menus availability
        bitWrite(canMsgSnd.data[0], 7, 1); // Parameters availability
        bitWrite(canMsgSnd.data[0], 6, bitRead(canMsgRcv.data[2], 3)); // Beam
        bitWrite(canMsgSnd.data[0], 5, 0); // Lighting
        bitWrite(canMsgSnd.data[0], 4, bitRead(canMsgRcv.data[3], 7)); // Adaptative lighting
        bitWrite(canMsgSnd.data[0], 3, bitRead(canMsgRcv.data[4], 1)); // SAM
        bitWrite(canMsgSnd.data[0], 2, bitRead(canMsgRcv.data[4], 2)); // Ambiance lighting
        bitWrite(canMsgSnd.data[0], 1, bitRead(canMsgRcv.data[2], 0)); // Automatic headlights
        bitWrite(canMsgSnd.data[0], 0, bitRead(canMsgRcv.data[3], 6)); // Daytime running lights
        bitWrite(canMsgSnd.data[1], 7, bitRead(canMsgRcv.data[5], 5)); // AAS
        bitWrite(canMsgSnd.data[1], 6, bitRead(canMsgRcv.data[3], 5)); // Wiper in reverse
        bitWrite(canMsgSnd.data[1], 5, bitRead(canMsgRcv.data[2], 4)); // Guide-me home lighting
        bitWrite(canMsgSnd.data[1], 4, bitRead(canMsgRcv.data[1], 2)); // Driver welcome
        bitWrite(canMsgSnd.data[1], 3, bitRead(canMsgRcv.data[2], 6)); // Motorized tailgate
        bitWrite(canMsgSnd.data[1], 2, bitRead(canMsgRcv.data[2], 0)); // Selective openings - Rear
        bitWrite(canMsgSnd.data[1], 1, bitRead(canMsgRcv.data[2], 7)); // Selective openings - Key
        bitWrite(canMsgSnd.data[1], 0, 0); // Selective openings
        bitWrite(canMsgSnd.data[2], 7, 1); // TNB - Seatbelt indicator
        bitWrite(canMsgSnd.data[2], 6, 1); // XVV - Custom cruise limits
        bitWrite(canMsgSnd.data[2], 5, bitRead(canMsgRcv.data[1], 4)); // Configurable button
        bitWrite(canMsgSnd.data[2], 4, bitRead(canMsgRcv.data[2], 2)); // Automatic parking brake
        bitWrite(canMsgSnd.data[2], 3, 0); // Sound Harmony
        bitWrite(canMsgSnd.data[2], 2, 0); // Rear mirror index
        bitWrite(canMsgSnd.data[2], 1, 0);
        bitWrite(canMsgSnd.data[2], 0, 0);
        bitWrite(canMsgSnd.data[3], 7, 1); // DSG Reset
        bitWrite(canMsgSnd.data[3], 6, 0); // Front Collision Warning
        bitWrite(canMsgSnd.data[3], 5, 0);
        bitWrite(canMsgSnd.data[3], 4, 1); // XVV - Custom cruise limits Menu
        bitWrite(canMsgSnd.data[3], 3, 1); // Recommended speed indicator
        bitWrite(canMsgSnd.data[3], 2, bitRead(canMsgRcv.data[5], 6)); // DSG - Underinflating (3b)
        bitWrite(canMsgSnd.data[3], 1, bitRead(canMsgRcv.data[5], 5)); // DSG - Underinflating (3b)
        bitWrite(canMsgSnd.data[3], 0, bitRead(canMsgRcv.data[5], 4)); // DSG - Underinflating (3b)
        canMsgSnd.data[4] = 0x00;
        canMsgSnd.data[5] = 0x00;
        canMsgSnd.data[6] = 0x00;
        bitWrite(canMsgSnd.data[6], 5, 1); // Privacy mode
        canMsgSnd.data[7] = 0x00;
        canMsgSnd.can_id = 0x361;
        canMsgSnd.can_dlc = 8;
        CAN1.sendMessage( & canMsgSnd);
        if (Send_CAN2010_ForgedMessages) {
          CAN0.sendMessage( & canMsgSnd);
        }
      } else if (id == 0x260 && len == 8) { // Personalization settings status
        // Do not forward original message, it has been completely redesigned on CAN2010
        // Also forge missing messages from CAN2004

        if (canMsgRcv.data[0] == 0x01) { // User profile 1
          canMsgSnd.data[0] = languageAndUnitNum;
          bitWrite(canMsgSnd.data[1], 7, (mpgMi)?1:0);
          bitWrite(canMsgSnd.data[1], 6, (TemperatureInF)?1:0);
          bitWrite(canMsgSnd.data[1], 5, 0); // Ambiance level
          bitWrite(canMsgSnd.data[1], 4, 1); // Ambiance level
          bitWrite(canMsgSnd.data[1], 3, 1); // Ambiance level
          bitWrite(canMsgSnd.data[1], 2, 1); // Parameters availability
          bitWrite(canMsgSnd.data[1], 1, 0); // Sound Harmony
          bitWrite(canMsgSnd.data[1], 0, 0); // Sound Harmony
          bitWrite(canMsgSnd.data[2], 7, bitRead(canMsgRcv.data[1], 0)); // Automatic parking brake
          bitWrite(canMsgSnd.data[2], 6, bitRead(canMsgRcv.data[1], 7)); // Selective openings - Key
          bitWrite(canMsgSnd.data[2], 5, bitRead(canMsgRcv.data[1], 4)); // Selective openings
          bitWrite(canMsgSnd.data[2], 4, bitRead(canMsgRcv.data[1], 5)); // Selective openings - Rear
          bitWrite(canMsgSnd.data[2], 3, bitRead(canMsgRcv.data[1], 1)); // Driver Welcome
          bitWrite(canMsgSnd.data[2], 2, bitRead(canMsgRcv.data[2], 7)); // Adaptative lighting
          bitWrite(canMsgSnd.data[2], 1, bitRead(canMsgRcv.data[3], 6)); // Daytime running lights
          bitWrite(canMsgSnd.data[2], 0, bitRead(canMsgRcv.data[3], 7)); // Ambiance lighting
          bitWrite(canMsgSnd.data[3], 7, bitRead(canMsgRcv.data[2], 5)); // Guide-me home lighting
          bitWrite(canMsgSnd.data[3], 6, bitRead(canMsgRcv.data[2], 1)); // Duration Guide-me home lighting (2b)
          bitWrite(canMsgSnd.data[3], 5, bitRead(canMsgRcv.data[2], 0)); // Duration Guide-me home lighting (2b)
          bitWrite(canMsgSnd.data[3], 4, bitRead(canMsgRcv.data[2], 6)); // Beam
          bitWrite(canMsgSnd.data[3], 3, 0); // Lighting ?
          bitWrite(canMsgSnd.data[3], 2, 0); // Duration Lighting (2b) ?
          bitWrite(canMsgSnd.data[3], 1, 0); // Duration Lighting (2b) ?
          bitWrite(canMsgSnd.data[3], 0, bitRead(canMsgRcv.data[2], 4)); // Automatic headlights
          bitWrite(canMsgSnd.data[4], 7, bitRead(canMsgRcv.data[5], 6)); // AAS
          bitWrite(canMsgSnd.data[4], 6, bitRead(canMsgRcv.data[6], 5)); // SAM
          bitWrite(canMsgSnd.data[4], 5, bitRead(canMsgRcv.data[5], 4)); // Wiper in reverse
          bitWrite(canMsgSnd.data[4], 4, 0); // Motorized tailgate
          bitWrite(canMsgSnd.data[4], 3, bitRead(canMsgRcv.data[7], 7)); // Configurable button
          bitWrite(canMsgSnd.data[4], 2, bitRead(canMsgRcv.data[7], 6)); // Configurable button
          bitWrite(canMsgSnd.data[4], 1, bitRead(canMsgRcv.data[7], 5)); // Configurable button
          bitWrite(canMsgSnd.data[4], 0, bitRead(canMsgRcv.data[7], 4)); // Configurable button

          personalizationSettings[7] = canMsgSnd.data[1];
          personalizationSettings[8] = canMsgSnd.data[2];
          personalizationSettings[9] = canMsgSnd.data[3];
          personalizationSettings[10] = canMsgSnd.data[4];
        } else { // Cached information if any other profile
          canMsgSnd.data[0] = languageAndUnitNum;
          canMsgSnd.data[1] = personalizationSettings[7];
          canMsgSnd.data[2] = personalizationSettings[8];
          canMsgSnd.data[3] = personalizationSettings[9];
          canMsgSnd.data[4] = personalizationSettings[10];
        }
        canMsgSnd.data[5] = 0x00;
        canMsgSnd.data[6] = 0x00;
        canMsgSnd.can_id = 0x260;
        canMsgSnd.can_dlc = 7;
        CAN1.sendMessage( & canMsgSnd);
        if (Send_CAN2010_ForgedMessages) {
          CAN0.sendMessage( & canMsgSnd);
        }

        bitWrite(canMsgSnd.data[0], 7, 0);
        bitWrite(canMsgSnd.data[0], 6, 0);
        bitWrite(canMsgSnd.data[0], 5, 0);
        bitWrite(canMsgSnd.data[0], 4, 0);
        bitWrite(canMsgSnd.data[0], 3, 0);
        bitWrite(canMsgSnd.data[0], 2, 1); // Parameters validity
        bitWrite(canMsgSnd.data[0], 1, 0); // User profile
        bitWrite(canMsgSnd.data[0], 0, 1); // User profile = 1
        canMsgSnd.data[1] = personalizationSettings[0];
        canMsgSnd.data[2] = personalizationSettings[1];
        canMsgSnd.data[3] = personalizationSettings[2];
        canMsgSnd.data[4] = personalizationSettings[3];
        canMsgSnd.data[5] = personalizationSettings[4];
        canMsgSnd.data[6] = personalizationSettings[5];
        canMsgSnd.data[7] = personalizationSettings[6];
        canMsgSnd.can_id = 0x15B; // Personalization frame status
        canMsgSnd.can_dlc = 8;
        CAN0.sendMessage( & canMsgSnd);

        if (!TelematicPresent && Ignition) {
          canMsgSnd.data[0] = 0x00;
          canMsgSnd.data[1] = 0x10;
          canMsgSnd.data[2] = 0xFF;
          canMsgSnd.data[3] = 0xFF;
          canMsgSnd.data[4] = 0x7F;
          canMsgSnd.data[5] = 0xFF;
          canMsgSnd.data[6] = 0x00;
          canMsgSnd.data[7] = 0x00;
          canMsgSnd.can_id = 0x167; // Fake EMF status frame
          canMsgSnd.can_dlc = 8;
          CAN0.sendMessage( & canMsgSnd);
        }

        // Economy mode simulation
        if (EconomyMode && EconomyModeEnabled) {
          canMsgSnd.data[0] = 0x14;
          if (Ignition) {
            canMsgSnd.data[5] = 0x0E;
          } else {
            canMsgSnd.data[5] = 0x0C;
          }
        } else {
          if (EngineRunning) {
            canMsgSnd.data[0] = 0x54;
          } else {
            canMsgSnd.data[0] = 0x04;
          }
          canMsgSnd.data[5] = 0x0F;
        }
        canMsgSnd.data[1] = 0x03;
        canMsgSnd.data[2] = 0xDE;

        canMsgSnd.data[3] = 0x00; // Increasing value,
        canMsgSnd.data[4] = 0x00; // counter ?

        canMsgSnd.data[6] = 0xFE;
        canMsgSnd.data[7] = 0x00;
        canMsgSnd.can_id = 0x236;
        canMsgSnd.can_dlc = 8;
        CAN1.sendMessage( & canMsgSnd);
        if (Send_CAN2010_ForgedMessages) {
          CAN0.sendMessage( & canMsgSnd);
        }

        // Current Time
        // If time is synced
        if (timeStatus() != timeNotSet) {
          canMsgSnd.data[0] = (year() - 1872); // Year would not fit inside one byte (0 > 255), substract 1872 and you get this new range (1872 > 2127)
          canMsgSnd.data[1] = month();
          canMsgSnd.data[2] = day();
          canMsgSnd.data[3] = hour();
          canMsgSnd.data[4] = minute();
          canMsgSnd.data[5] = 0x3F;
          canMsgSnd.data[6] = 0xFE;
        } else {
          canMsgSnd.data[0] = (Time_year - 1872); // Year would not fit inside one byte (0 > 255), substract 1872 and you get this new range (1872 > 2127)
          canMsgSnd.data[1] = Time_month;
          canMsgSnd.data[2] = Time_day;
          canMsgSnd.data[3] = Time_hour;
          canMsgSnd.data[4] = Time_minute;
          canMsgSnd.data[5] = 0x3F;
          canMsgSnd.data[6] = 0xFE;
        }
        canMsgSnd.can_id = 0x276;
        canMsgSnd.can_dlc = 7;
        CAN1.sendMessage( & canMsgSnd);
        if (Send_CAN2010_ForgedMessages) {
          CAN0.sendMessage( & canMsgSnd);
        }

        if (!EngineRunning) {
          AirConditioningON = false;
          FanSpeed = 0x41;
          LeftTemp = 0x00;
          RightTemp = 0x00;
          FanPosition = 0x04;

          canMsgSnd.data[0] = 0x09;
          canMsgSnd.data[1] = 0x00;
          canMsgSnd.data[2] = 0x00;
          canMsgSnd.data[3] = LeftTemp;
          canMsgSnd.data[4] = RightTemp;
          canMsgSnd.data[5] = FanSpeed;
          canMsgSnd.data[6] = FanPosition;
          canMsgSnd.data[7] = 0x00;
          canMsgSnd.can_id = 0x350;
          canMsgSnd.can_dlc = 8;
          CAN1.sendMessage( & canMsgSnd);
          if (Send_CAN2010_ForgedMessages) {
            CAN0.sendMessage( & canMsgSnd);
          }
        }
      } else if (id == 0x321 && len < 5)  { // Intercept 0x321 and reconstruct it with 5 bytes DrumVlado
        canMsgSnd.can_id = 0x321;  // Set CAN ID to 0x321
        canMsgSnd.can_dlc = 5;     // Set length to 5 bytes
        for (int i = 0; i < 4; i++) {
          canMsgSnd.data[i] = canMsgRcv.data[i];  // Copy the first 4 bytes from the received message
        }
        canMsgSnd.data[4] = 0x00;  // Add the missing byte
        CAN1.sendMessage(&canMsgSnd);
      } else {
        CAN1.sendMessage( & canMsgRcv);
      }
    } else {
      CAN1.sendMessage( & canMsgRcv);
    }
  }

  // Forward messages from the CAN2010 device(s) to the car
  if (CAN1.readMessage( & canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    if (debugCAN1) {
      Serial.print("FRAME:ID=");
      Serial.print(id);
      Serial.print(":LEN=");
      Serial.print(len);

      char tmp[3];
      for (int i = 0; i < len; i++) {
        Serial.print(":");

        snprintf(tmp, 3, "%02X", canMsgRcv.data[i]);

        Serial.print(tmp);
      }

      Serial.println();

      CAN0.sendMessage( & canMsgRcv);
    } else if (!debugCAN0) {
      if (id == 0x260 || id == 0x361) {
        // Do not send back converted frames between networks
      } else if (id == 0x39B && len == 5) {
        Time_year = canMsgRcv.data[0] + 1872; // Year would not fit inside one byte (0 > 255), add 1872 and you get this new range (1872 > 2127)
        Time_month = canMsgRcv.data[1];
        Time_day = canMsgRcv.data[2];
        Time_hour = canMsgRcv.data[3];
        Time_minute = canMsgRcv.data[4];

        setTime(Time_hour, Time_minute, 0, Time_day, Time_month, Time_year);
        RTC.set(now()); // Set the time on the RTC module too
        EEPROM.update(5, Time_day);
        EEPROM.update(6, Time_month);
        EEPROM.put(7, Time_year);

        // Set hour on CAN-BUS Clock
        canMsgSnd.data[0] = hour();
        canMsgSnd.data[1] = minute();
        canMsgSnd.can_id = 0x228;
        canMsgSnd.can_dlc = 1;
        CAN0.sendMessage( & canMsgSnd);

        if (SerialEnabled) {
          Serial.print("Change Hour/Date: ");
          Serial.print(day());
          Serial.print("/");
          Serial.print(month());
          Serial.print("/");
          Serial.print(year());

          Serial.print(" ");

          Serial.print(hour());
          Serial.print(":");
          Serial.print(minute());

          Serial.println();
        }
      } else if (id == 0x1A9 && len == 8) { // Telematic commands
        TelematicPresent = true;

        darkMode = bitRead(canMsgRcv.data[0], 7); // Dark mode
        resetTrip1 = bitRead(canMsgRcv.data[0], 1); // Reset Trip 1
        resetTrip2 = bitRead(canMsgRcv.data[0], 0); // Reset Trip 2
        pushAAS = bitRead(canMsgRcv.data[3], 2); // AAS
        pushSAM = bitRead(canMsgRcv.data[3], 2); // SAM
        pushDSG = bitRead(canMsgRcv.data[5], 0); // Indirect DSG reset
        pushSTT = bitRead(canMsgRcv.data[6], 7); // Start&Stop
        pushCHECK = bitRead(canMsgRcv.data[6], 0); // Check
        stopCHECK = bitRead(canMsgRcv.data[1], 7); // Stop Check
        pushBLACK = bitRead(canMsgRcv.data[5], 0); // Black Panel

        if (Ignition) {
          canMsgSnd.data[0] = 0x00;
          bitWrite(canMsgSnd.data[0], 7, resetTrip1); // Reset Trip 1
          bitWrite(canMsgSnd.data[0], 6, resetTrip2); // Reset Trip 2
          canMsgSnd.data[1] = 0x10;
          bitWrite(canMsgSnd.data[1], 5, darkMode); // Dark mode
          canMsgSnd.data[2] = 0xFF;
          canMsgSnd.data[3] = 0xFF;
          canMsgSnd.data[4] = 0x7F;
          canMsgSnd.data[5] = 0xFF;
          canMsgSnd.data[6] = 0x00;
          canMsgSnd.data[7] = 0x00;
          canMsgSnd.can_id = 0x167; // Fake EMF Status frame
          canMsgSnd.can_dlc = 8;
          CAN0.sendMessage( & canMsgSnd);
        }

        if (!ClusterPresent && Ignition && (resetTrip1 || resetTrip2 || pushAAS || pushSAM || pushDSG || pushSTT || pushCHECK)) {
          canMsgSnd.data[0] = statusCMB[0];
          canMsgSnd.data[1] = statusCMB[1];
          bitWrite(canMsgSnd.data[1], 4, pushCHECK);
          bitWrite(canMsgSnd.data[1], 2, resetTrip1);
          canMsgSnd.data[2] = statusCMB[2];
          bitWrite(canMsgSnd.data[2], 7, pushAAS);
          bitWrite(canMsgSnd.data[2], 6, pushASR);
          canMsgSnd.data[3] = statusCMB[3];
          bitWrite(canMsgSnd.data[3], 3, pushSAM);
          bitWrite(canMsgSnd.data[3], 0, resetTrip2);
          canMsgSnd.data[4] = statusCMB[4];
          bitWrite(canMsgSnd.data[4], 7, pushDSG);
          canMsgSnd.data[5] = statusCMB[5];
          canMsgSnd.data[6] = statusCMB[6];
          bitWrite(canMsgSnd.data[6], 7, pushSTT);
          canMsgSnd.data[7] = statusCMB[7];
          canMsgSnd.can_id = 0x217;
          canMsgSnd.can_dlc = 8;
          CAN0.sendMessage( & canMsgSnd);
        }
      } else if (id == 0x329 && len == 8) {
        pushASR = bitRead(canMsgRcv.data[3], 0); // ESP
      } else if (id == 0x31C && len == 5) { // MATT status
        canMsgSnd.data[0] = canMsgRcv.data[0];
        // Rewrite if necessary to make BTEL commands working
        if (resetTrip1) { // Reset Trip 1
          bitWrite(canMsgSnd.data[0], 3, 1);
        }
        if (resetTrip2) { // Reset Trip 2
          bitWrite(canMsgSnd.data[0], 2, 1);
        }
        canMsgSnd.data[1] = canMsgRcv.data[1];
        canMsgSnd.data[2] = canMsgRcv.data[2];
        canMsgSnd.data[3] = canMsgRcv.data[3];
        canMsgSnd.data[4] = canMsgRcv.data[4];
        canMsgSnd.can_id = 0x31C;
        canMsgSnd.can_dlc = 5;
        CAN0.sendMessage( & canMsgSnd);
      } else if (id == 0x217 && len == 8) { // Rewrite Cluster status (CIROCCO for example) for tactile touch buttons (telematic) because it is not listened by BSI
        ClusterPresent = true;

        canMsgSnd.data[0] = canMsgRcv.data[0];
        canMsgSnd.data[1] = canMsgRcv.data[1];
        bitWrite(canMsgSnd.data[1], 4, pushCHECK);
        bitWrite(canMsgSnd.data[1], 2, resetTrip1);
        canMsgSnd.data[2] = canMsgRcv.data[2];
        bitWrite(canMsgSnd.data[2], 7, pushAAS);
        bitWrite(canMsgSnd.data[2], 6, pushASR);
        canMsgSnd.data[3] = canMsgRcv.data[3];
        bitWrite(canMsgSnd.data[3], 3, pushSAM);
        bitWrite(canMsgSnd.data[3], 0, resetTrip2);
        canMsgSnd.data[4] = canMsgRcv.data[4];
        bitWrite(canMsgSnd.data[4], 7, pushDSG);
        canMsgSnd.data[5] = canMsgRcv.data[5];
        canMsgSnd.data[6] = canMsgRcv.data[6];
        bitWrite(canMsgSnd.data[6], 7, pushSTT);
        canMsgSnd.data[7] = canMsgRcv.data[7];
        canMsgSnd.can_id = 0x217;
        canMsgSnd.can_dlc = 8;
        CAN0.sendMessage( & canMsgSnd);
      } else if (id == 0x15B && len == 8) {
        if (bitRead(canMsgRcv.data[1], 2)) { // Parameters validity
          tmpVal = canMsgRcv.data[0];
          if (tmpVal >= 128) {
            languageAndUnitNum = tmpVal;
            EEPROM.update(0, languageAndUnitNum);

            if (SerialEnabled) {
              Serial.print("Telematic - Change Language and Unit (Number): ");
              Serial.print(tmpVal);
              Serial.println();
            }

            tmpVal = canMsgRcv.data[1];
            if (tmpVal >= 128) {
              mpgMi = true;
              EEPROM.update(4, 1);

              tmpVal = tmpVal - 128;
            } else {
              mpgMi = false;
              EEPROM.update(4, 0);
            }

            if (tmpVal >= 64) {
              TemperatureInF = true;
              EEPROM.update(3, 1);

              if (SerialEnabled) {
                Serial.print("Telematic - Change Temperature Type: Fahrenheit");
                Serial.println();
              }
            } else if (tmpVal >= 0) {
              TemperatureInF = false;
              EEPROM.update(3, 0);

              if (SerialEnabled) {
                Serial.print("Telematic - Change Temperature Type: Celcius");
                Serial.println();
              }
            }
          } else {
            tmpVal = tmpVal >> 2;
            if (canMsgRcv.data[1] >= 128) {
              tmpVal--;
            }
            languageID = tmpVal;

            // CAN2004 Head-up panel is only one-way talking, we can't change the language on it from the CAN2010 Telematic :-(

            if (SerialEnabled) {
              Serial.print("Telematic - Change Language (ID): ");
              Serial.print(tmpVal);
              Serial.println();
            }
          }

          // Personalization settings change
          bitWrite(canMsgSnd.data[0], 7, 0);
          bitWrite(canMsgSnd.data[0], 6, 0);
          bitWrite(canMsgSnd.data[0], 5, 0);
          bitWrite(canMsgSnd.data[0], 4, 0);
          bitWrite(canMsgSnd.data[0], 3, 0);
          bitWrite(canMsgSnd.data[0], 2, 0); // Parameters validity, 0 = Changed parameter(s) the BSI must take into account
          bitWrite(canMsgSnd.data[0], 1, 0); // User profile
          bitWrite(canMsgSnd.data[0], 0, 1); // User profile = 1
          bitWrite(canMsgSnd.data[1], 7, bitRead(canMsgRcv.data[2], 6)); // Selective openings
          bitWrite(canMsgSnd.data[1], 6, 1);
          bitWrite(canMsgSnd.data[1], 5, bitRead(canMsgRcv.data[2], 4)); // Selective rear openings
          bitWrite(canMsgSnd.data[1], 4, bitRead(canMsgRcv.data[2], 5)); // Selective openings
          bitWrite(canMsgSnd.data[1], 3, 0);
          bitWrite(canMsgSnd.data[1], 2, 0);
          bitWrite(canMsgSnd.data[1], 1, bitRead(canMsgRcv.data[2], 3)); // Driver welcome
          bitWrite(canMsgSnd.data[1], 0, bitRead(canMsgRcv.data[2], 7)); // Parking brake
          bitWrite(canMsgSnd.data[2], 7, bitRead(canMsgRcv.data[2], 2)); // Adaptative lighting
          bitWrite(canMsgSnd.data[2], 6, bitRead(canMsgRcv.data[3], 4)); // Beam
          bitWrite(canMsgSnd.data[2], 5, bitRead(canMsgRcv.data[3], 7)); // Guide-me home lighting
          bitWrite(canMsgSnd.data[2], 4, bitRead(canMsgRcv.data[3], 0)); // Automatic headlights
          bitWrite(canMsgSnd.data[2], 3, 0);
          bitWrite(canMsgSnd.data[2], 2, 0);
          bitWrite(canMsgSnd.data[2], 1, bitRead(canMsgRcv.data[3], 6)); // Duration Guide-me home lighting (2b)
          bitWrite(canMsgSnd.data[2], 0, bitRead(canMsgRcv.data[3], 5)); // Duration Guide-me home lighting (2b)
          bitWrite(canMsgSnd.data[3], 7, bitRead(canMsgRcv.data[2], 0)); // Ambiance lighting
          bitWrite(canMsgSnd.data[3], 6, bitRead(canMsgRcv.data[2], 1)); // Daytime running lights
          bitWrite(canMsgSnd.data[3], 5, 0);
          bitWrite(canMsgSnd.data[3], 4, 0);
          bitWrite(canMsgSnd.data[3], 3, 0);
          bitWrite(canMsgSnd.data[3], 2, 0);
          bitWrite(canMsgSnd.data[3], 1, 0);
          bitWrite(canMsgSnd.data[3], 0, 0);
          canMsgSnd.data[4] = 0x00;
          bitWrite(canMsgSnd.data[5], 7, bitRead(canMsgRcv.data[4], 7)); // AAS
          bitWrite(canMsgSnd.data[5], 6, bitRead(canMsgRcv.data[4], 7)); // AAS
          bitWrite(canMsgSnd.data[5], 5, 0);
          bitWrite(canMsgSnd.data[5], 4, bitRead(canMsgRcv.data[4], 5)); // Wiper in reverse
          bitWrite(canMsgSnd.data[5], 3, 0);
          bitWrite(canMsgSnd.data[5], 2, 0);
          bitWrite(canMsgSnd.data[5], 1, 0);
          bitWrite(canMsgSnd.data[5], 0, 0);
          bitWrite(canMsgSnd.data[6], 7, 0);
          bitWrite(canMsgSnd.data[6], 6, bitRead(canMsgRcv.data[4], 6)); // SAM
          bitWrite(canMsgSnd.data[6], 5, bitRead(canMsgRcv.data[4], 6)); // SAM
          bitWrite(canMsgSnd.data[6], 4, 0);
          bitWrite(canMsgSnd.data[6], 3, 0);
          bitWrite(canMsgSnd.data[6], 2, 0);
          bitWrite(canMsgSnd.data[6], 1, 0);
          bitWrite(canMsgSnd.data[6], 0, 0);
          bitWrite(canMsgSnd.data[7], 7, bitRead(canMsgRcv.data[4], 3)); // Configurable button
          bitWrite(canMsgSnd.data[7], 6, bitRead(canMsgRcv.data[4], 2)); // Configurable button
          bitWrite(canMsgSnd.data[7], 5, bitRead(canMsgRcv.data[4], 1)); // Configurable button
          bitWrite(canMsgSnd.data[7], 4, bitRead(canMsgRcv.data[4], 0)); // Configurable button
          bitWrite(canMsgSnd.data[7], 3, 0);
          bitWrite(canMsgSnd.data[7], 2, 0);
          bitWrite(canMsgSnd.data[7], 1, 0);
          bitWrite(canMsgSnd.data[7], 0, 0);
          canMsgSnd.can_id = 0x15B;
          canMsgSnd.can_dlc = 8;
          CAN0.sendMessage( & canMsgSnd);

          // Store personalization settings for the recurring frame
          personalizationSettings[0] = canMsgSnd.data[1];
          personalizationSettings[1] = canMsgSnd.data[2];
          personalizationSettings[2] = canMsgSnd.data[3];
          personalizationSettings[3] = canMsgSnd.data[4];
          personalizationSettings[4] = canMsgSnd.data[5];
          personalizationSettings[5] = canMsgSnd.data[6];
          personalizationSettings[6] = canMsgSnd.data[7];
          EEPROM.update(10, personalizationSettings[0]);
          EEPROM.update(11, personalizationSettings[1]);
          EEPROM.update(12, personalizationSettings[2]);
          EEPROM.update(13, personalizationSettings[3]);
          EEPROM.update(14, personalizationSettings[4]);
          EEPROM.update(15, personalizationSettings[5]);
          EEPROM.update(16, personalizationSettings[6]);
        }
      } else if (id == 0x1E9 && len >= 2 && CVM_Emul) { // Telematic suggested speed to fake CVM frame
        CAN0.sendMessage( & canMsgRcv);

        tmpVal = (canMsgRcv.data[3] >> 2); // POI type - Gen2 (6b)

        canMsgSnd.data[0] = canMsgRcv.data[1];
        canMsgSnd.data[1] = ((tmpVal > 0 && vehicleSpeed > canMsgRcv.data[0]) ? 0x30 : 0x10); // POI Over-speed, make speed limit blink
        canMsgSnd.data[2] = 0x00;
        canMsgSnd.data[3] = 0x00;
        canMsgSnd.data[4] = 0x7C;
        canMsgSnd.data[5] = 0xF8;
        canMsgSnd.data[6] = 0x00;
        canMsgSnd.data[7] = 0x00;
        canMsgSnd.can_id = 0x268; // CVM Frame ID
        canMsgSnd.can_dlc = 8;
        CAN1.sendMessage( & canMsgSnd);
      } else if (id == 0x1E5 && len == 7) {
        // Ambience mapping
        tmpVal = canMsgRcv.data[5];
        if (tmpVal == 0x00) { // User
          canMsgRcv.data[6] = 0x40;
        } else if (tmpVal == 0x08) { // Classical
          canMsgRcv.data[6] = 0x44;
        } else if (tmpVal == 0x10) { // Jazz
          canMsgRcv.data[6] = 0x48;
        } else if (tmpVal == 0x18) { // Pop-Rock
          canMsgRcv.data[6] = 0x4C;
        } else if (tmpVal == 0x28) { // Techno
          canMsgRcv.data[6] = 0x54;
        } else if (tmpVal == 0x20) { // Vocal
          canMsgRcv.data[6] = 0x50;
        } else { // Default : User
          canMsgRcv.data[6] = 0x40;
        }

        // Loudness / Volume linked to speed
        tmpVal = canMsgRcv.data[4];
        if (tmpVal == 0x10) { // Loudness / not linked to speed
          canMsgRcv.data[5] = 0x40;
        } else if (tmpVal == 0x14) { // Loudness / Volume linked to speed
          canMsgRcv.data[5] = 0x47;
        } else if (tmpVal == 0x04) { // No Loudness / Volume linked to speed
          canMsgRcv.data[5] = 0x07;
        } else if (tmpVal == 0x00) { // No Loudness / not linked to speed
          canMsgRcv.data[5] = 0x00;
        } else { // Default : No Loudness / not linked to speed
          canMsgRcv.data[5] = 0x00;
        }

        // Bass
        // CAN2004 Telematic Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
        // CAN2010 Telematic Range: "32" > "88" ("60" = 0)
        tmpVal = canMsgRcv.data[2];
        canMsgRcv.data[2] = ((tmpVal - 32) >> 2) + 57; // Converted value

        // Treble
        // CAN2004 Telematic Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
        // CAN2010 Telematic Range: "32" > "88" ("60" = 0)
        tmpVal = canMsgRcv.data[3];
        canMsgRcv.data[4] = ((tmpVal - 32) >> 2) + 57; // Converted value on position 4 (while it's on 3 on a old amplifier)

        // Balance - Left / Right
        // CAN2004 Telematic Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
        // CAN2010 Telematic Range: "32" > "88" ("60" = 0)
        tmpVal = canMsgRcv.data[1];
        canMsgRcv.data[1] = ((tmpVal - 32) >> 2) + 57; // Converted value

        // Balance - Front / Back
        // CAN2004 Telematic Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
        // CAN2010 Telematic Range: "32" > "88" ("60" = 0)
        tmpVal = canMsgRcv.data[0];
        canMsgRcv.data[0] = ((tmpVal - 32) >> 2) + 57; // Converted value

        // Mediums ?
        canMsgRcv.data[3] = 63; // 0x3F = 63

        CAN0.sendMessage( & canMsgRcv);
      } else {
        CAN0.sendMessage( & canMsgRcv);
      }
    } else {
      CAN0.sendMessage( & canMsgRcv);
    }

  }
}

void sendPOPup(bool present, int id, byte priority, byte parameters) {
  bool clear = false;
  byte firstEmptyPos = 8;

  for (int i = 0; i < 8; i++) {
    if (alertsCache[i] == id) {
      if (!present) { // Remove from cache and clear popup
        alertsCache[i] = alertsParametersCache[i] = firstEmptyPos = 0;
        clear = true;
        break;
      } else if (parameters == alertsParametersCache[i]) { // Already sent
        return;
      } else {
        return sendPOPup(false, id, priority, 0x00); // Clear previous popup first
      }
    } else if (alertsCache[i] == 0 && firstEmptyPos >= 8) {
      firstEmptyPos = i;
    }
  }

  if (firstEmptyPos >= 8) {
    return; // Avoid overflow    
  }
  if (!present && !clear) {
    return;
  } else if (!clear) {
    alertsCache[firstEmptyPos] = id;
    alertsParametersCache[firstEmptyPos] = parameters;

    if (SerialEnabled && present) {
      Serial.print("Notification sent with message ID: ");
      Serial.println(id);
    }
  }

  if (priority > 14) {
    priority = 14;
  }

  if (present) {
    canMsgSnd.data[0] = highByte(id); 
    canMsgSnd.data[1] = lowByte(id);
    bitWrite(canMsgSnd.data[0], 7, present); // New message
  } else { // Close Popup
    canMsgSnd.data[0] = 0x7F; 
    canMsgSnd.data[1] = 0xFF;
  }
  canMsgSnd.data[2] = priority; // Priority (0 > 14)
  bitWrite(canMsgSnd.data[2], 7, 1); // Destination: NAC / EMF / MATT
  bitWrite(canMsgSnd.data[2], 6, 1); // Destination: CMB
  canMsgSnd.data[3] = parameters; // Parameters
  canMsgSnd.data[4] = 0x00; // Parameters
  canMsgSnd.data[5] = 0x00; // Parameters
  canMsgSnd.data[6] = 0x00; // Parameters
  canMsgSnd.data[7] = 0x00; // Parameters
  canMsgSnd.can_id = 0x1A1;
  canMsgSnd.can_dlc = 8;
  CAN1.sendMessage( & canMsgSnd);

  return;
}

int daysSinceYearStartFct() {
  // Given a day, month, and year (4 digit), returns
  // the day of year. Errors return 999.
  int daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  // Check if it is a leap year, this is confusing business
  // See: https://support.microsoft.com/en-us/kb/214019
  if (year()%4  == 0) {
    if (year()%100 != 0) {
      daysInMonth[1] = 29;
    }
    else {
      if (year()%400 == 0) {
        daysInMonth[1] = 29;
      }
    }
   }

  int doy = 0;
  for (int i = 0; i < month() - 1; i++) {
    doy += daysInMonth[i];
  }

  doy += day();
  return doy;
}

byte checksumm_0E6(const byte* frame)
{
    /* Autors:
        organizer of the bacchanal: styleflava
        algorithm: Ilia
        code: Pepelxl
    */
    static byte iter = 0;
    byte cursumm = 0;
    for (byte i = 0; i < 7; i++)
    {
        cursumm += (frame[i] >> 4) + (frame[i] & 0x0F);
    }
    cursumm += iter;
    cursumm = ((cursumm ^ 0xFF) - 3) & 0x0F;
    cursumm ^= iter << 4;
    iter++;
    if (iter >= 16) iter = 0;
    return cursumm;
}
