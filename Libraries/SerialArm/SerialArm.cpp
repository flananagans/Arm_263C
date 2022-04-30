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
 *  Write different data to same address of each joint at the same time
 */
void SerialArm::writeAllDiff(uint8_t idx, uint8_t data[][SerialArm::MAX_JOINTS], uint8_t len_per) {

    uint8_t buf[SerialArm::MAX_PACK_LEN] = {0};
    uint8_t len = createSyncWriteBuf(&buf[0], idx, data, len_per); 
    this->sendPacket(BCST_ID, SYNC_WT, &buf[0], len);  
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

    Serial.print("Received: ");
    for(uint8_t i = 0; i < packLen; i++) {
        Serial.print(pack[i], HEX);
        Serial.print(',');
    }
    Serial.println();

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
            // Break out once header is fully matched
            if(headCtr++ >= sizeof(packHeader)/sizeof(uint8_t)) {
                currInd++;
                break;
            }
        } else if(inHeader && !headMatch) { // Started in the header but didn't match
            inHeader = false;
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

  uint8_t data[3] = {IDX_LED, 0x00, 0x00};
  if(this->led[j]) {
    data[2] = 0x00;
  } else {
    data[2] = 0x01;
  }
  this->led[j] = !this->led[j];

  // Send the command
  this->sendPacket(this->joint2id[j], W_REG, &data[0], 3);
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
