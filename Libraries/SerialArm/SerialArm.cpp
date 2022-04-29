#include "SerialArm.h"

SerialArm::SerialArm(HardwareSerial* port, uint8_t enPin) {
    this->serPort = port;
    this->enTxPin = enPin;
}

/*
 * Add joints to the arm by their ID
 */
bool SerialArm::addJoint(uint8_t ID) {
    
    if(ID >= SerialArm::MAX_ID) {
        return 0;
    }

    this->id2joint[ID] = this->numJoints;
    this->joint2id[this->numJoints++] = ID;

    return 1;
}

/*
 * Start communications and setup pins
 */
void SerialArm::start() {
    this->serPort->begin(this->BAUD);
    while(!this->serPort) {};
    pinMode(this->enTxPin, OUTPUT);
    digitalWrite(this->enTxPin, LOW);
}

/*
 * Get the current number of joints
 */
uint8_t SerialArm::getNumJoints() {
    return this->numJoints;
}

/*
 * Send a packet to the arm joint
 */
void SerialArm::sendPacket(uint8_t ID, uint8_t COM, uint8_t* data, uint16_t dataLen) {
    this->serPort->clear(); // clear serial buffers

    uint8_t pack[SerialArm::MAX_PACK_LEN] = {0};

    pack[0] = 0xFF;
    pack[1] = 0xFF;
    pack[2] = 0xFD;
    pack[3] = 0x00;
    pack[4] = ID;
    pack[5] = (uint8_t)(dataLen + 3); // low byte
    pack[6] = (uint8_t)((dataLen + 3) >> 8); // high byte
    pack[7] = COM;

    for(uint8_t i = 0; i < dataLen; i++) {
        pack[8 + i] = data[i];
    }

    uint16_t crc = SerialArm::updateCRC(0, &pack[0], 8 + dataLen);
    pack[dataLen + 8] = (uint8_t)crc;
    pack[dataLen + 9] = (uint8_t)(crc >> 8);  

    /*
    Serial.print("Sent: ");
    for(uint8_t i = 0; i < dataLen + 10; i++) {
        Serial.print(pack[i], HEX);
        Serial.print(',');
    }
    Serial.println();
    */

    // Send the packet
    digitalWrite(this->enTxPin, HIGH);
    this->serPort->write(&pack[0], dataLen + 10);
    this->serPort->flush();
    digitalWrite(this->enTxPin, LOW);
}

/*
 * Parse a response from Serial
 */
bool SerialArm::parseResponse() {

    this->COM_ERR = NO_ERR;

    uint8_t pack[SerialArm::MAX_PACK_LEN] = {0};
    uint8_t packLen = 0;
    
    // Wait until Serial data has come in
    long startTime = micros();
    long del = micros() - startTime;
    while( (del < this->SER_TIMEOUT) && !this->serPort->available()) {
        del = micros() - startTime;
    }
    if(del > this->SER_TIMEOUT) {
        this->COM_ERR = TIMEOUT;
        return 0;
    }
    
    // Collect received bytes until too long of a delay between
    startTime = micros();
    long delAllow = (1e7*8)/this->BAUD;

    while(del < delAllow && (packLen < SerialArm::MAX_PACK_LEN) ) {
        if(this->serPort->available() > 0) {
            pack[packLen++] = this->serPort->read();
            startTime = micros();
        }
        del = micros() - startTime;
    }

    
    /* 
    Serial.print("Received: ");
    for(uint8_t i = 0; i < packLen; i++) {
        Serial.print(pack[i], HEX);
        Serial.print(',');
    }
    Serial.println();
    */

    uint8_t packHeader[4] = {0xFF, 0xFF, 0xFD, 0x00};
    uint8_t headCtr = 0;
    bool inHeader = false;
    uint8_t currInd = 0;
    uint8_t headStartInd = 0;
    for (; currInd < packLen; currInd++) {

        bool headMatch = pack[currInd] == packHeader[headCtr];

        if(!inHeader && headMatch) { // Potentially found the start of header
            headCtr++;
            inHeader = true;
            headStartInd = currInd;
        } else if(inHeader && headMatch) { // In header and matched
            headCtr++;
        } else if(inHeader && !headMatch) { // Started in the header but didn't match
            inHeader = false;
            break;
        }

        // Break out once header is fully matched
        if(headCtr >= sizeof(packHeader)/sizeof(uint8_t)) {
            currInd++;
            break;
        }
    }
    if(!inHeader) { //We didn't get the complete header
        this->COM_ERR = HEADER;
        return 0;
    }

    // Parse out packet data information
    this->lastID = pack[currInd++];
    uint16_t len = (pack[currInd + 1] << 8) + pack[currInd];
    currInd += 2;

    if( (currInd + len) > packLen) {
        this->COM_ERR = DATA_LEN;
        return 0;
    }

    // Check CRC
    uint16_t crc = (pack[currInd + len - 1] << 8) + pack[currInd + len - 2];
    if(crc != SerialArm::updateCRC(0, &pack[headStartInd], 5 + len)) {
        this->COM_ERR = CRC;
        return 0;
    }

    // Parse through params
    this->lastCom = pack[currInd++];
    this->errCode[this->id2joint[this->lastID]] = pack[currInd++];

    if(this->errCode[this->id2joint[this->lastID]] != 0) {
        this->COM_ERR = JNT_ERR;
        return 0;
    }

    // Clear old data and fill it with new
    memset(&this->data[0], 0, SerialArm::MAX_PACK_LEN); 
    for(uint8_t i = 0; i < len - 4; i++) { // len includes com byte, err byte, and 2 crc bytes
        this->data[i] = pack[currInd++]; 
    }
    this->dataLen = len - 4;

    return 1;
}

/*
 * Toggle the LED for a specific joint
 */
void SerialArm::toggleLED(uint8_t j) {

  uint8_t data[3] = {65, 0x00, 0x00};
  if(this->led[j]) {
    data[2] = 0x00;
  } else {
    data[2] = 0x01;
  }
  this->led[j] = !this->led[j];

  // Send the command
  this->sendPacket(this->joint2id[j], 0x03, &data[0], 3);
  // Parse the response
  if(this->parseResponse()) {
    
    Serial.print("Data from ID ");
    Serial.print(this->lastID, HEX);
    Serial.print(": ");
    for(uint8_t i = 0; i < this->dataLen; i++) {
        Serial.print(this->data[i], HEX);
        Serial.print(',');
    }
    Serial.println();
  } else {
    Serial.print("COM_ERR for ID ");
    Serial.print(this->lastID, HEX);
    Serial.print(": ");
    Serial.println(this->COM_ERR);
  }
}
