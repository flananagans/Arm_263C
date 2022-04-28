#include "SerialArm.h"

SerialArm::SerialArm(HardwareSerial& port, uint8_t enPin) : serPort(port) {
    this->enTxPin = enPin;
}

/*
 * Add joints to the arm by their ID
 */
void SerialArm::addJoint(uint8_t ID) {
   this->joints[this->numJoints++] = ID;
}

/*
 * Start communications and setup pins
 */
void SerialArm::start() {
    this->serPort.begin(1000000);
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

void SerialArm::sendPacket(uint8_t ID, uint8_t COM, uint8_t* data, uint16_t len) {
    uint8_t pack[SerialArm::MAX_PACK_LEN];

    pack[0] = 0xFF;
    pack[1] = 0xFF;
    pack[2] = 0xFD;
    pack[3] = 0x00;
    pack[4] = ID;
    pack[5] = (uint8_t)(len + 3); // low byte
    pack[6] = (uint8_t)((len + 3) >> 8); // high byte
    pack[7] = COM;

    for(uint8_t i = 0; i < len; i++) {
        pack[8 + i] = data[i];
    }

    Serial.println("hola1");
    uint16_t crc = SerialArm::updateCRC(0, &pack[0], 8 + len);
    pack[len + 8] = (uint8_t)crc;
    pack[len + 9] = (uint8_t)(crc >> 8);  

    Serial.print("Sent: ");
    for(uint8_t i = 0; i < len + 10; i++) {
        Serial.print(pack[i], HEX);
        Serial.print(',');
    }
    Serial.println();

    digitalWrite(this->enTxPin, HIGH);
    this->serPort.write(&pack[0], len + 10);
    this->serPort.flush();
    digitalWrite(this->enTxPin, LOW);
    Serial.println("hola2");
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

  this->sendPacket(this->joints[j], 0x03, &data[0], 3);
}
