#include "SerialArm.h"
#include <cmath> //std::abs, std::signbit

/*
 *  Constructor to set the serial port and TX enable pin for this arm
 */
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
    this->packHeader.b[0] = 0xFF;
    this->packHeader.b[1] = 0xFF;
    this->packHeader.b[2] = 0xFD;
    this->packHeader.b[3] = 0x00;
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
void SerialArm::sendPacket(uint8_t id, uint8_t com, uint8_t* data, uint16_t dataLen) {
    this->serPort->clear(); // clear serial buffers

    uint8_t pack[SerialArm::MAX_PACK_LEN] = {0};

    pack[0] = 0xFF;
    pack[1] = 0xFF;
    pack[2] = 0xFD;
    pack[3] = 0x00;
    pack[4] = id;
    pack[5] = (uint8_t)(dataLen + 3); // low byte
    pack[6] = (uint8_t)((dataLen + 3) >> 8); // high byte
    pack[7] = com;

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
 * Fill in a data buffer for the sync write instruction 
 *  buf - pointer to the start of the data buffer to fill. Assumed to be sized correctly
 *  idx - index to write to
 *  data - pointers to 2D array of data (one col per joint, down cols for that joints data)
 *  len_per - length of data for each joint (data is len_per x numJoints)
 *
 *  returns the length of the buffer
 */
uint8_t SerialArm::createSyncWriteBuf(uint8_t* buf, uint16_t IDX, 
                                      uint8_t data[][SerialArm::MAX_JOINTS], uint8_t len_per) {

    buf[0] = IDX;
    buf[1] = 0x00;
    buf[2] = len_per;
    buf[3] = 0x00;
    
    uint8_t currInd = 4;
    for (uint8_t j = 0; j < this->numJoints; j++) {
        buf[currInd++] = this->joint2id[j];

        // Data is already arranged in little endian (LSB first)
        for (uint8_t i = 0; i < len_per; i++) {
            buf[currInd++] = data[i][j];
        }
    }

    return currInd;
}

/*
 * Write the same data to all the joints
 *  idx - address to write to
 *  data - 1 x len_per array of data to write to all joints
 *  len_per - length of data to write in bytes
 */
void SerialArm::writeAllSame(uint8_t idx, uint8_t* data_per, uint8_t len_per) {

    uint8_t data[len_per][SerialArm::MAX_JOINTS];
    for(uint8_t j = 0; j < this->numJoints; j++) {
        for(uint8_t i = 0; i < len_per; i++) {
            data[i][j] = data_per[i];
        }
    }
    this->writeAllDiff(idx, data, len_per);
}

/*
 * Write different data to same address of each joint at the same time
 *  idx - address to write to  
 *  data - len_per x numJoints array of data, each column is data to write for one joint
 *  len_per - length of data to write to each joint in bytes
 */
void SerialArm::writeAllDiff(uint8_t idx, uint8_t data[][SerialArm::MAX_JOINTS], uint8_t len_per) {

    uint8_t buf[SerialArm::MAX_PACK_LEN] = {0};
    uint8_t len = createSyncWriteBuf(&buf[0], idx, data, len_per); 
    this->sendPacket(BCST_ID, SYNC_WT, &buf[0], len);  
}


/*
 * Check for a packet header received over serial
 */
bool SerialArm::checkForHeader() {
    uint8_t headBuf[2*HEADER_LEN] = {0}; // Efficient circular-ish type buffer
    uint8_t headInd = 0;
    long startTime = micros();
    long del = 0;
    while(del < this->SER_TIMEOUT) { // want to get at least the header

        if(this->serPort->available() > 0) {
            headBuf[headInd] = this->serPort->read(); // Write to the first half of the buffer

            /*
            Serial.print(headBuf[headInd], HEX);
            Serial.print(",");
            */
            headBuf[headInd + HEADER_LEN] = headBuf[headInd]; // Copy to the second half of the buffer
            headInd++;

            if(!(headInd < HEADER_LEN)) { // reset ind to 0
                headInd = 0;
            }

            // Check the last <HEADER_LEN> bytes for a match
            SerialArm::f_b_union headCheck;
            memcpy(&headCheck.b[0], &headBuf[headInd], HEADER_LEN);
            // When the motors are on, the header was found to have issues with the first byte
            // So we will relax the requirements to let the first byte be anything
            headCheck.b[0] = 0xFF;

            if(headCheck.f == this->packHeader.f) {
                break;
            }
        }
        del = micros() - startTime;
    }
    //Serial.println();
    if(del >= this->SER_TIMEOUT) { //We didn't get the complete header
        return 0;
    }
    return 1;
}


/*
 * Collect a packet over serial
 *  Should be called ONLY after checkForHeader returns true
 *
 *  Returns the length of data written to pack buffer or 0 if fail
 */
uint8_t SerialArm::collectPacket(uint8_t* pack) {

    // Collect bytes until received complete packet or timeout
    uint8_t packLen = HEADER_LEN;
    memcpy(&pack[0], &this->packHeader.b[0], packLen);
    uint16_t len = MAX_PACK_LEN; // parameter length of packet sent
    long startTime = micros();
    long del = 0;
    while( (del < this->SER_TIMEOUT) && (packLen - 7 < len) ) {
        if(this->serPort->available() > 0) {

            pack[packLen++] = this->serPort->read();
            switch(packLen) {
                case HEADER_LEN + 1: // ID byte
                    this->lastRecID = pack[packLen - 1];
                    this->lastRecJoint = (this->lastRecID < MAX_ID) ? 
                                          this->id2joint[this->lastRecID] : (this->numJoints + 1);
                    break;
                case HEADER_LEN + 3: // Length bytes
                    len = (pack[packLen - 1] << 8) + pack[packLen - 2];
                    if( (packLen + len) > MAX_PACK_LEN) {
                        this->COM_ERR = DATA_LEN;
                        return 0;
                    }
                    break;
            }
        }
        del = micros() - startTime;
    }
    if(del >= this->SER_TIMEOUT) { //We didn't get the complete packet
        return 0;
    }
    return packLen;
}

/*
 * Parse a single response from serial
 */
bool SerialArm::parseSingleResponse() {

    this->COM_ERR = NO_ERR;
    
    // Check for header
    if(!this->checkForHeader()) { //We didn't get the complete header
        Serial.println("hola");
        this->COM_ERR = TIMEOUT;
        return 0;
    }

    // Collect packet
    uint8_t pack[SerialArm::MAX_PACK_LEN] = {0};
    uint8_t packLen = this->collectPacket(&pack[0]);
    if(!packLen) { //We didn't get the complete packet
        this->COM_ERR = INCOMPLETE;
        Serial.println("hola2");
        return 0;
    }

    // Check the ID of the packet
    if(this->lastRecJoint == this->numJoints + 1) {
        this->COM_ERR = BAD_ID;
        return 0;
    }
/* 
    Serial.print("Received ");
    Serial.print(packLen);
    Serial.print(" bytes: ");
    for(uint8_t i = 0; i < packLen; i++) {
        Serial.print(pack[i], HEX);
        Serial.print(',');
    }
    Serial.println();
*/

    uint16_t paramLen = (pack[HEADER_LEN + 2] << 8) + pack[HEADER_LEN + 1];
    // Check CRC
    uint16_t crc = (pack[packLen - 1] << 8) + pack[packLen - 2];
    if(crc != SerialArm::updateCRC(0, &pack[0], 5 + paramLen)) {
        this->COM_ERR = CRC;
        Serial.println("hola3");
        return 0;
    }

    // Parse through params
    uint8_t currInd = HEADER_LEN + 3;
    this->lastCom[lastRecJoint] = pack[currInd++];
    this->errCode[this->lastRecJoint] = pack[currInd++];

    if(this->errCode[this->lastRecJoint] != 0) {
        this->COM_ERR = JNT_ERR;
        return 0;
    }

    // Fill with new data
    for(uint8_t i = 0; i < paramLen - 4; i++) { // len includes com byte, err byte, and 2 crc bytes
        this->recData[i][this->lastRecJoint] = pack[currInd++]; 
    }
    this->recDataLen[this->lastRecJoint] = paramLen - 4;

    return 1;
}

/*
 * Toggle the LED for a specific joint
 */
void SerialArm::toggleLED(uint8_t j) {

  uint8_t data[3] = {IDX_LED, 0x00, 0x00};
  if(this->led[j]) {
    data[2] = 0x00;
  } else {
    data[2] = 0x01;
  }
  this->led[j] = !this->led[j];

  // Send the command
  this->sendPacket(this->joint2id[j], W_REG, &data[0], 3);
}

/*
 * Toggle all the leds
 */
void SerialArm::toggleLED(void) {
    uint8_t data_per[1] = {!this->led[0]};
    this->writeAllSame(IDX_LED, &data_per[0], (uint8_t)1);
    this->led[0] = !this->led[0];
}

/*
 * Enable all the motors
 */
bool SerialArm::enable() {

    uint8_t data_per[1] = {0x01};
    this->writeAllSame(IDX_EN, &data_per[0], (uint8_t)1);
    this->enabled = true;
    return true;
}

/*
 * Disable all the motors
 */
bool SerialArm::disable() {

    uint8_t data_per[1] = {0x00};
    this->writeAllSame(IDX_EN, &data_per[0], (uint8_t)1);
    this->enabled = false;
    return true;
}

/*
 * Configure the joints
 */
void SerialArm::configure(uint8_t del, uint8_t mode, float v_max) {
    uint8_t data_per[1] = {0}; // 0us delay

    // Return delay time
    data_per[0] = del; // 0us delay
    this->writeAllSame(IDX_RETDEL, &data_per[0], (uint8_t)1);

    // Operating mode
    data_per[0] = mode; // PWM mode
    this->writeAllSame(IDX_OPMODE, &data_per[0], (uint8_t)1);

    // Status return level
    data_per[0] = 2; //return status for ping and read instructions only
    this->writeAllSame(IDX_STATRET, &data_per[0], (uint8_t)1);

    // Voltage limit
    SerialArm::f_b_union v;
    v.f = std::abs(v_max);
    this->writeAllSame(IDX_OPMODE, &v.b[0], (uint8_t)1); // Teensy is little endian so this should work
    this->MAX_V = v.f;
}

/*
 * Set control velocity for each individual joint
 *
 *  v assumed to be a 1 x numJoints array
 */
void SerialArm::setV(float* v) {
    if(this->enabled) {
        uint8_t len_per = 2;

        uint8_t data[len_per][SerialArm::MAX_JOINTS];
        for(uint8_t j = 0; j < this->numJoints; j++) {
           
            if(std::abs(v[j]) > MAX_V) {
                v[j] = std::signbit(v[j]) ? -1*MAX_V : MAX_V; //signbit is true if negative
            }
            int16_t pwm = (int16_t)(MAX_PWM * v[j]/MAX_V);

            data[0][j] = (uint8_t)pwm;
            data[1][j] = (uint8_t)(pwm >> 8);
        }
        this->writeAllDiff(IDX_PWMGOAL, data, len_per);
    }
}

/*
 * Get position of a single joint in radians
 */
float SerialArm::getPos(uint8_t j) {

    uint8_t data[4] = {IDX_POS, 0x00, sizeof(float), 0};

    // Send the command
    this->sendPacket(this->joint2id[j], R_REG, &data[0], sizeof(float));

    // Parse the response
    if(this->parseSingleResponse()) {

        /*
        Serial.print("Data from ID ");
        Serial.print(this->lastRecID, HEX);
        Serial.print(": ");
        for(uint8_t i = 0; i < this->recDataLen[this->lastRecJoint]; i++) {
            Serial.print(this->recData[i][this->lastRecJoint], HEX);
            Serial.print(',');
        }
        Serial.println();
        */

        int32_t pos = (this->recData[3][j] << 24) + (this->recData[2][j] << 16) +
                      (this->recData[1][j] << 8) + (this->recData[0][j]); 

        return pos*(2*3.14159)/(ENC_PER_REV);
    }
    /*
    Serial.print("COM_ERR for ID ");
    Serial.print(this->lastRecID, HEX);
    Serial.print(": ");
    Serial.println(this->COM_ERR);
    */
    return std::nanf("");

}

/*
 * Get position from all joints 
 *  Fills the pointer to a 1 x numJoints float array with position in radians
 */
void SerialArm::getAllPos(float* pos) {

    uint8_t data[4 + this->numJoints] = {IDX_POS, 0x00, sizeof(float), 0};
    for(uint8_t j = 0; j < this->numJoints; j++) {
        data[4 + j] = this->joint2id[j]; // Fill buffer with the joint IDs
    }

    // Send the command
    this->sendPacket(BCST_ID, SYNC_RD, &data[0], 4 + this->numJoints);

    // Parse the response
    for (uint8_t i = 0; i < this->numJoints; i++) {
        if(this->parseSingleResponse()) {

            uint8_t j = this->id2joint[this->lastRecID];
            int32_t pos_i = (this->recData[3][j] << 24) + (this->recData[2][j] << 16) +
                            (this->recData[1][j] << 8) + (this->recData[0][j]); 

            pos[j] = pos_i*(2*3.14159)/(ENC_PER_REV); 

        } else {

            for (uint8_t j = 0; j < this->numJoints; j++) {
                pos[j] = std::nanf(""); 
            }
            break;
        }
    }
}

/*
 * Get velocity of a single joint in rad/s
 */
float SerialArm::getVel(uint8_t j) {

    uint8_t data[4] = {IDX_VEL, 0x00, sizeof(float), 0};

    // Send the command
    this->sendPacket(this->joint2id[j], R_REG, &data[0], sizeof(float));

    // Parse the response
    if(this->parseSingleResponse()) {

        int32_t vel = (this->recData[3][j] << 24) + (this->recData[2][j] << 16) +
                      (this->recData[1][j] << 8) + (this->recData[0][j]); 

        return vel*(2*3.14159)*(VEL_UNIT_RPM)/60; // rad/s
    }
    return std::nanf("");
}

/*
 * Get velocity from all joints 
 *  Fills the pointer to a 1 x numJoints float array with velocity in rad/s
 */
void SerialArm::getAllVel(float* vel) {

    uint8_t data[4 + this->numJoints] = {IDX_VEL, 0x00, sizeof(float), 0};
    for(uint8_t j = 0; j < this->numJoints; j++) {
        data[4 + j] = this->joint2id[j]; // Fill buffer with the joint IDs
    }

    // Send the command
    this->sendPacket(BCST_ID, SYNC_RD, &data[0], 4 + this->numJoints);

    // Parse the response
    for (uint8_t i = 0; i < this->numJoints; i++) {
        if(this->parseSingleResponse()) {

            uint8_t j = this->id2joint[this->lastRecID];
            int32_t vel_i = (this->recData[3][j] << 24) + (this->recData[2][j] << 16) +
                            (this->recData[1][j] << 8) + (this->recData[0][j]); 

            vel[j] = vel_i*(2*3.14159)*(VEL_UNIT_RPM)/60; // rad/s
        } else {
            for (uint8_t j = 0; j < this->numJoints; j++) {
                vel[j] = std::nanf(""); 
            }
            break;
        }
    }
}
