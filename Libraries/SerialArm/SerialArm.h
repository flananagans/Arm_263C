#ifndef SERIALARM_H
#define SERIALARM_H

#include <Arduino.h>
#include <HardwareSerial.h>

class SerialArm {

    public:
        SerialArm(HardwareSerial* port, uint8_t enPin);
        bool addJoint(uint8_t ID);
        uint8_t getNumJoints();
        void start();
        void toggleLED(uint8_t j);
        void toggleLED(void);
        bool enable();
        bool disable();
        void configure(uint8_t del, uint8_t mode, float v_max);
        void setV(float* v);
        float getPos(uint8_t j);
        void getAllPos(float* pos);
        float getVel(uint8_t j);
        void getAllVel(float* vel);

        bool enabled = false;
    private:

        // Configuration
        static const uint8_t MAX_PACK_LEN = 64;
        static const uint8_t MAX_JOINTS = 8;
        static const uint8_t MAX_ID = 32;
        float MAX_V = 12.0;
        static constexpr float MAX_PWM = 885.0;
        static constexpr float ENC_PER_REV = 4096.0;
        static constexpr float VEL_UNIT_RPM = 0.229;

        uint8_t BCST_ID = 0xFE; // Used to broadcast message to all joints
        uint8_t id2joint[MAX_ID] = {0}; // lookup for ID -> joint ind
        uint8_t joint2id[MAX_JOINTS] = {0}; // lookup for joint ind -> ID
        uint8_t numJoints = 0;
        bool led[MAX_JOINTS] = {0}; // LED state of each joint
        uint8_t errCode[MAX_JOINTS] = {0}; // error codes for joint motors
        uint8_t lastRecID = 0; // ID of sender of last packet
        uint8_t lastRecJoint = 0; // Joint number of last joint heard from 
        uint8_t lastCom[MAX_JOINTS] = {0}; // last command received for each joint
        uint8_t recData[MAX_PACK_LEN][MAX_JOINTS]; // last data received from each joint
        uint8_t recDataLen[MAX_JOINTS] = {0};

        HardwareSerial* serPort;
        int enTxPin = 0; // HIGH: Transmitter, LOW: Receiver
        long BAUD = 1000000; // serial baud rate in bps
        long SER_TIMEOUT = 1000; // timeout in us

        // Union for converting floats to bytes
        union f_b_union {
            float f;
            uint8_t b[sizeof(float)];
        };
        
        f_b_union packHeader; // header of each packet is 4 bytes so store in a float union
        static const uint8_t HEADER_LEN = sizeof(float);

        // Error codes for parsing serial responses
        enum COM_ERR_ENUM {
            NO_ERR,
            TIMEOUT,
            INCOMPLETE,
            BAD_ID,
            DATA_LEN,
            CRC,
            JNT_ERR
        } COM_ERR;

        // Command codes
        uint8_t R_REG = 0x02;
        uint8_t W_REG = 0x03;
        uint8_t SYNC_RD = 0x82;
        uint8_t SYNC_WT = 0x83;
        
        // Register indices
        uint8_t IDX_RETDEL = 9; // return delay time
        uint8_t IDX_OPMODE = 11; // operation mode
        uint8_t IDX_EN = 64; // torque enable
        uint8_t IDX_LED = 65; // LED on/off
        uint8_t IDX_STATRET = 68; // status return behavior
        uint8_t IDX_PWMGOAL = 100; // set PWM goal
        uint8_t IDX_VEL = 128; // current velocity
        uint8_t IDX_POS = 132; // current position

        void sendPacket(uint8_t id, uint8_t com, uint8_t* data, uint16_t len);
        uint8_t createSyncWriteBuf(uint8_t* buf, uint16_t IDX, uint8_t data[][MAX_JOINTS], uint8_t len_per);
        void writeAllSame(uint8_t idx, uint8_t* data_per, uint8_t len_per);
        void writeAllDiff(uint8_t idx, uint8_t data[][SerialArm::MAX_JOINTS], uint8_t len_per);
        bool checkForHeader(void);
        uint8_t collectPacket(uint8_t* pack);
        bool parseSingleResponse();

        /*
         * CRC calculation function from Dynamixel datasheet
         */
        static unsigned short updateCRC(unsigned short crc_accum, 
                                        unsigned char *data_blk_ptr,
                                        unsigned short data_blk_size) {
            
            unsigned short i, j;
            unsigned short crc_table[256] = {
                0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
                0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
                0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
                0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
                0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
                0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
                0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
                0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
                0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
                0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
                0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
                0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
                0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
                0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
                0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
                0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
                0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
                0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
                0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
                0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
                0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
                0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
                0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
                0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
                0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
                0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
                0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
            };

            for(j = 0; j < data_blk_size; j++)
            {
                i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
                crc_accum = (crc_accum << 8) ^ crc_table[i];
            }

            return crc_accum;
        };
};


#endif
