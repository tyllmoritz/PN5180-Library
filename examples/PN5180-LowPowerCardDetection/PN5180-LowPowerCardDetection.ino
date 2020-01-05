// NAME: PN5180-LowPowerCardDetection.ino
//
// DESC: Example usage of the PN5180 library for the PN5180-NFC Module
//       from NXP Semiconductors.
//		 Uses Low power card detection mode (LPCD)
//
// Copyright (c) 2018 by Andreas Trappmann. All rights reserved.
// Copyright (c) 2019 by Dirk Carstensen.
//
// This file is part of the PN5180 library for the Arduino environment.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public 
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// BEWARE: SPI with an Arduino to a PN5180 module has to be at a level of 3.3V
// use of logic-level converters from 5V->3.3V is absolutly neccessary
// on most Arduinos for all input pins of PN5180!
// If used with an ESP-32, there is no need for a logic-level converter, since
// it operates on 3.3V already.
//
// Arduino <-> Level Converter <-> PN5180 pin mapping:
// 5V             <-->             5V
// 3.3V           <-->             3.3V
// GND            <-->             GND
// 5V      <-> HV
// GND     <-> GND (HV)
//             LV              <-> 3.3V
//             GND (LV)        <-> GND
// SCLK,13 <-> HV1 - LV1       --> SCLK
// MISO,12        <---         <-- MISO
// MOSI,11 <-> HV3 - LV3       --> MOSI
// SS,10   <-> HV4 - LV4       --> NSS (=Not SS -> active LOW)
// BUSY,9         <---             BUSY
// Reset,7 <-> HV2 - LV2       --> RST
// IRQ,6          <---             IRQ
//
// ESP-32    <--> PN5180 pin mapping:
// 3.3V      <--> 3.3V
// GND       <--> GND
// SCLK, 18   --> SCLK
// MISO, 19  <--  MISO
// MOSI, 23   --> MOSI
// SS, 16     --> NSS (=Not SS -> active LOW)
// BUSY, 5   <--  BUSY
// Reset, 17  --> RST
//

/*
 * Pins on ICODE2 Reader Writer:
 *
 *   ICODE2   |     PN5180
 * pin  label | pin  I/O  name
 * 1    +5V
 * 2    +3,3V
 * 3    RST     10   I    RESET_N (low active)
 * 4    NSS     1    I    SPI NSS
 * 5    MOSI    3    I    SPI MOSI
 * 6    MISO    5    O    SPI MISO
 * 7    SCK     7    I    SPI Clock
 * 8    BUSY    8    O    Busy Signal
 * 9    GND     9  Supply VSS - Ground
 * 10   GPIO    38   O    GPO1 - Control for external DC/DC
 * 11   IRQ     39   O    IRQ
 * 12   AUX     40   O    AUX1 - Analog/Digital test signal
 * 13   REQ     2?  I/O   AUX2 - Analog test bus or download
 *
 */

#include <PN5180.h>
#include <PN5180ISO14443.h>
#include <PN5180ISO15693.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_NANO)

#define PN5180_NSS  10
#define PN5180_BUSY 9
#define PN5180_RST  7
#define PN5180_IRQ  6

#elif defined(ARDUINO_ARCH_ESP32)

#define PN5180_NSS  16   
#define PN5180_BUSY 5  
#define PN5180_RST  17

#else
#error Please define your pinout here!
#endif

PN5180ISO14443 nfc(PN5180_NSS, PN5180_BUSY, PN5180_RST);

void setup() {
  Serial.begin(115200);
  Serial.println(F("=================================="));
  Serial.println(F("Uploaded: " __DATE__ " " __TIME__));
  Serial.println(F("PN5180 LPCD Demo Sketch"));

  pinMode(PN5180_IRQ, INPUT);
  nfc.begin();

  Serial.println(F("----------------------------------"));
  Serial.println(F("PN5180 Hard-Reset..."));
  nfc.reset();

  Serial.println(F("----------------------------------"));
  Serial.println(F("Reading product version..."));
  uint8_t productVersion[2];
  nfc.readEEprom(PRODUCT_VERSION, productVersion, sizeof(productVersion));
  Serial.print(F("Product version="));
  Serial.print(productVersion[1]);
  Serial.print(".");
  Serial.println(productVersion[0]);

  if (0xff == productVersion[1]) { // if product version 255, the initialization failed
    Serial.println(F("Initialization failed!?"));
    Serial.println(F("Press reset to restart..."));
    Serial.flush();
    exit(-1); // halt
  }
  
  Serial.println(F("----------------------------------"));
  Serial.println(F("Reading firmware version..."));
  uint8_t firmwareVersion[2];
  nfc.readEEprom(FIRMWARE_VERSION, firmwareVersion, sizeof(firmwareVersion));
  Serial.print(F("Firmware version="));
  Serial.print(firmwareVersion[1]);
  Serial.print(".");
  Serial.println(firmwareVersion[0]);

  if (firmwareVersion[1] < 4) {
    Serial.println("This LPCD demo might work only for firmware version 4.0!!!");
  };
  
  Serial.println(F("----------------------------------"));
  Serial.println(F("Reading EEPROM version..."));
  uint8_t eepromVersion[2];
  nfc.readEEprom(EEPROM_VERSION, eepromVersion, sizeof(eepromVersion));
  Serial.print(F("EEPROM version="));
  Serial.print(eepromVersion[1]);
  Serial.print(".");
  Serial.println(eepromVersion[0]);
  Serial.println(F("----------------------------------"));
  Serial.println(F("Reading IRQ-Pin..."));
  uint8_t irqPin[1];
  nfc.readEEprom(IRQ_PIN_CONFIG, irqPin, sizeof(irqPin));
  Serial.print(F("irqPin="));
  Serial.println(irqPin[0]);

  Serial.println(F("----------------------------------"));
  Serial.println(F("start LPCD..."));//  nfc.loadRFConfig(0x00, 0x80);

  // LPCD threshold
  uint8_t data[255];
  uint8_t response[256];
  uint8_t threshold = 0x04;
  data[0] = threshold;
  nfc.writeEEprom(0x37, data, 1);
  nfc.readEEprom(0x37, response, 1);
  threshold = response[0];
  Serial.print("LPCD-threshold: ");
  Serial.println(threshold, HEX);
  
  // LPCD_FIELD_ON_TIME (0x36)
  uint8_t fieldOn = 0xF0;
  data[0] = fieldOn;
  nfc.writeEEprom(0x36, data, 1);
  nfc.readEEprom(0x36, response, 1);
  fieldOn = response[0];
  Serial.print("LPCD-fieldOn time: ");
  Serial.println(fieldOn, HEX);

  // LPCD_REFVAL_GPO_CONTROL
  uint8_t lpcdMode = 0x01; // 1 = LPCD SELF CALIBRATION
  data[0] = lpcdMode;
  nfc.writeEEprom(0x38, data, 1);
  nfc.readEEprom(0x38, response, 1);
  lpcdMode = response[0];
  Serial.print("lpcdMode: ");
  Serial.println(lpcdMode, HEX);
  delay(100);


  // turn on LPCD
  uint16_t sleepTimeMS = 2500;
  if (nfc.switchToLPCD(sleepTimeMS)) {
    Serial.println("switchToLPCD success");
  } else {
    Serial.println("switchToLPCD failed");
  }
}

uint32_t loopCnt = 0;


// read cards loop
void loop() {
  if (digitalRead(PN5180_IRQ) == HIGH) {
    // LPCD detection irq
    showIRQStatus(nfc.getIRQStatus());
    uint32_t u;
    nfc.readRegister(0x26, &u);
    Serial.print("LPCD_REFERENCE_VALUE: ");
    Serial.println(u, HEX);
    nfc.clearIRQStatus(0xffffffff);
    nfc.reset(); 
    delay(1000);
    // turn on LPCD
    uint16_t sleepTimeMS = 2500;
    if (nfc.switchToLPCD(sleepTimeMS)) {
      Serial.println("switchToLPCD success");
    } else {
      Serial.println("switchToLPCD failed");
    }
  }
}


void showIRQStatus(uint32_t irqStatus) {
  Serial.print(F("IRQ-Status 0x"));
  Serial.print(irqStatus, HEX);
  Serial.print(": [ ");
  if (irqStatus & (1<< 0)) Serial.print(F("RQ "));
  if (irqStatus & (1<< 1)) Serial.print(F("TX "));
  if (irqStatus & (1<< 2)) Serial.print(F("IDLE "));
  if (irqStatus & (1<< 3)) Serial.print(F("MODE_DETECTED "));
  if (irqStatus & (1<< 4)) Serial.print(F("CARD_ACTIVATED "));
  if (irqStatus & (1<< 5)) Serial.print(F("STATE_CHANGE "));
  if (irqStatus & (1<< 6)) Serial.print(F("RFOFF_DET "));
  if (irqStatus & (1<< 7)) Serial.print(F("RFON_DET "));
  if (irqStatus & (1<< 8)) Serial.print(F("TX_RFOFF "));
  if (irqStatus & (1<< 9)) Serial.print(F("TX_RFON "));
  if (irqStatus & (1<<10)) Serial.print(F("RF_ACTIVE_ERROR "));
  if (irqStatus & (1<<11)) Serial.print(F("TIMER0 "));
  if (irqStatus & (1<<12)) Serial.print(F("TIMER1 "));
  if (irqStatus & (1<<13)) Serial.print(F("TIMER2 "));
  if (irqStatus & (1<<14)) Serial.print(F("RX_SOF_DET "));
  if (irqStatus & (1<<15)) Serial.print(F("RX_SC_DET "));
  if (irqStatus & (1<<16)) Serial.print(F("TEMPSENS_ERROR "));
  if (irqStatus & (1<<17)) Serial.print(F("GENERAL_ERROR "));
  if (irqStatus & (1<<18)) Serial.print(F("HV_ERROR "));
  if (irqStatus & (1<<19)) Serial.print(F("LPCD "));
  Serial.println("]");
}
