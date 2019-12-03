// NAME: PN5180ISO14443.h
//
// DESC: ISO14443 protocol on NXP Semiconductors PN5180 module for Arduino.
//
// Copyright (c) 2019 by Dirk Carstensen. All rights reserved.
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
// #define DEBUG 1

#include <Arduino.h>
#include "PN5180ISO14443.h"
#include <PN5180.h>
#include "Debug.h"

PN5180ISO14443::PN5180ISO14443(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin) 
              : PN5180(SSpin, BUSYpin, RSTpin) {
}

bool PN5180ISO14443::setupRF() {
  PN5180DEBUG(F("Loading RF-Configuration...\n"));
  if (loadRFConfig(0x00, 0x80)) {  // ISO14443 parameters
    PN5180DEBUG(F("done.\n"));
  }
  else return false;

  PN5180DEBUG(F("Turning ON RF field...\n"));
  if (setRF_on()) {
    PN5180DEBUG(F("done.\n"));
  }
  else return false;

  return true;
}

uint16_t PN5180ISO14443::rxBytesReceived() {
	uint32_t rxStatus;
	uint16_t len = 0;
	readRegister(RX_STATUS, &rxStatus);
	// Lower 9 bits has length
	len = (uint16_t)(rxStatus & 0x000001ff);
	return len;
}
/*
* buffer : must be 10 byte array
* buffer[0-1] is ATQAa
* buffer[2] is sak
* buffer[3..6] is 4 byte UID
* buffer[7..9] is remaining 3 bytes of UID for 7 Byte UID tags
* kind : 0  we send REQA, 1 we send WUPA
*/
void PN5180ISO14443::activateTypeA(uint8_t *buffer, uint8_t kind) {
	uint8_t cmd[7];

	// Load standard TypeA protocol
	loadRFConfig(0x0, 0x80);

	// OFF Crypto
	writeRegisterWithAndMask(SYSTEM_CONFIG, 0xFFFFFFBF);
	// Clear RX CRC
	writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE);
	// Clear TX CRC
	writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE);
	//Send REQA/WUPA, 7 bits in last byte
	cmd[0] = (kind == 0) ? 0x26 : 0x52;
	sendData(cmd, 1, 0x07);
	// READ 2 bytes ATQA into  buffer
	readData(2, buffer);
	//Send Anti collision 1, 8 bits in last byte
	cmd[0] = 0x93;
	cmd[1] = 0x20;
	sendData(cmd, 2, 0x00);
	//Read 5 bytes, we will store at offset 2 for later usage
	readData(5, cmd+2);
	//Enable RX CRC calculation
	writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01);
	//Enable TX CRC calculation
	writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01);
	//Send Select anti collision 1, the remaining bytes are already in offset 2 onwards
	cmd[0] = 0x93;
	cmd[1] = 0x70;
	sendData(cmd, 7, 0x00);
	//Read 1 byte SAK into buffer[2]
	readData(1, buffer+2);
	// Check if the tag is 4 Byte UID or 7 byte UID and requires anti collision 2
	// If Bit 3 is 0 it is 4 Byte UID
	if ((buffer[2] & 0x04) == 0) {
		// Take first 4 bytes of anti collision as UID store at offset 3 onwards. job done
		for (int i = 0; i < 4; i++) buffer[3+i] = cmd[2 + i];
	}
	else {
		// Take First 3 bytes of UID, Ignore first byte 88(CT)
		for (int i = 0; i < 3; i++) buffer[3+i] = cmd[3 + i];
		// Clear RX CRC
		writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE);
		// Clear TX CRC
		writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE);
		// Do anti collision 2
		cmd[0] = 0x95;
		cmd[1] = 0x20;
		sendData(cmd, 2, 0x00);
		//Read 5 bytes. we will store at offset 2 for later use
		readData(5, cmd+2);
		// first 4 bytes belongs to last 4 UID bytes, we keep it.
		for (int i = 0; i < 4; i++) {
			buffer[6 + i] = cmd[2+i];
		}
		//Enable RX CRC calculation
		writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01);
		//Enable TX CRC calculation
		writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01);
		//Send Select anti collision 2 
		cmd[0] = 0x95;
		cmd[1] = 0x70;
		sendData(cmd, 7, 0x00);
		//Read 1 byte SAK into buffer[2]
		readData(1, buffer + 2);	
	}
}

bool PN5180ISO14443::mifareBlockRead(uint8_t blockno, uint8_t *buffer) {
	bool success = false;
	uint16_t len;
	uint8_t cmd[2];
	// Send mifare command 30,blockno
	cmd[0] = 0x30;
	cmd[1] = blockno;
	sendData(cmd, 2, 0x00);
	//Check if we have received any data from the tag
	delay(5);
	len = rxBytesReceived();
	if (len == 16) {
		// READ 16 bytes into  buffer
		readData(16, buffer);
		success = true;
	}
	return success;
}


uint8_t PN5180ISO14443::mifareBlockWrite16(uint8_t blockno, uint8_t *buffer) {
	uint8_t cmd[1];
	// Clear RX CRC
	writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE);

	// Mifare write part 1
	cmd[0] = 0xA0;
	cmd[1] = blockno;
	sendData(cmd, 2, 0x00);
	readData(1, cmd);

	// Mifare write part 2
	sendData(buffer,16, 0x00);
	delay(10);

	// Read ACK/NAK
	readData(1, cmd);

	//Enable RX CRC calculation
	writeRegisterWithOrMask(CRC_RX_CONFIG, 0x1);
	return cmd[0];
}

bool PN5180ISO14443::mifareHalt() {
	uint8_t cmd[1];
	//mifare Halt
	cmd[0] = 0x50;
	cmd[1] = 0x00;
	sendData(cmd, 2, 0x00);	
	 return true;
}

bool PN5180ISO14443::readCardSerial(uint8_t *buffer) {
  
    uint8_t response[10];
	// Always return 10 bytes
    // Offset 0..1 is ATQA
    // Offset 2 is SAK.
    // UID 4 bytes : offset 3 to 6 is UID, offset 7 to 9 to Zero
    // UID 7 bytes : offset 3 to 9 is UID
    for (int i = 0; i < 10; i++) response[i] = 0;
     activateTypeA(response, 1);
	if ((response[0] == 0xFF) && (response[1] == 0xFF))
	  return false;
    for (int i = 0; i < 7; i++) buffer[i] = response[i+3];
	mifareHalt();
	return true;  
}

bool PN5180ISO14443::isCardPresent() {
	uint8_t cmd[7];
	uint8_t response[2];

	// Load standard TypeA protocol
	loadRFConfig(0x0, 0x80);

	// OFF Crypto
	writeRegisterWithAndMask(SYSTEM_CONFIG, 0xFFFFFFBF);
	// Clear RX CRC
	writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE);
	// Clear TX CRC
	writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE);
	//Send REQA, 7 bits in last byte
	cmd[0] = 0x26;
	sendData(cmd, 1, 0x07);
	// READ 2 bytes ATQA into  buffer
	response[0] = 0;
	response[1] = 0;
	readData(2, response);
	// check valid response
	if ((response[0] == 0x44) && (response[1] == 0x00)) 
      return true;
	 
    mifareHalt();	  
    return true;
}

