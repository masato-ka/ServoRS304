#include<ServoRS304.h>
/* PacketCMD* ServoController::shortPacketFactory(unsigned char servoId, unsigned char flag, unsigned char headerAddr, \
        unsigned char dataLength, unsigned char *data){
    packet_cmd p = {""};
    
    
} */


ServoController::ServoController(HardwareSerial& serial){
    //TODO If begin serial in this place on Wio LTE, do not work.
    hardwareSerial = &serial;
}

ServoController::begin(){
    hardwareSerial->begin(115200);
}

ServoController::begin(int baurate){
    hardwareSerial->begin();
}

//private 
void ServoController::calcCheckSum(unsigned char *cmd, int length){
    unsigned char checkSum = 0;
//    int length = sizeof(cmd)/sizeof(unsigned char)
    for(int i=2; i < length - 1; i++ ){
        checkSum = checkSum ^ cmd [i];
    }
    cmd[length-1] = checkSum;
}

//TODO Serail servo controller class 
//private 
void ServoController::sendCommand(unsigned char *cmd, int length){
    hardwareSerial->write(cmd,length);
    delay(1);
}


void ServoController::clearRxBuffer(){
    while(hardwareSerial->available()){hardwareSerial->read();}
}

//private 
void ServoController::getMemoryMap(unsigned char id, unsigned char *buffer){
    unsigned char txData[8];
    unsigned char checkSum = 0;
    txData[0] = 0xFA;
    txData[1] = 0xAF;
    txData[2] = id;
    txData[3] = 0x09;
    txData[4] = 0x00;
    txData[5] = 0x00;
    txData[6] = 0x01;

    calcCheckSum(txData,8);
    sendCommand(txData,8);
    delay(17);//delay(10);//Wio側のバッファに入らない？
    unsigned char readRaw[26+8];
    int i = 0;
    while(hardwareSerial->available() > 0){
        readRaw[i++] = hardwareSerial->read();
    }
    memcpy(buffer, &readRaw[8], 26);
}

void ServoController::resetServo(unsigned char servoId){
    unsigned char cmd[8];
    cmd[0] = 0xFA;
    cmd[1] = 0xAF;
    cmd[2] = servoId;
    cmd[3] = 0x20;
    cmd[4] = 0xFF;
    cmd[5] = 0x00;
    cmd[6] = 0x00;
    calcCheckSum(cmd,8);
    sendCommand(cmd,8);
}

void ServoController::restartServo(unsigned char servoId){
    unsigned char cmd[8];
    cmd[0] = 0xFA;
    cmd[1] = 0xAF;
    cmd[2] = servoId;
    cmd[3] = 0x20;
    cmd[4] = 0xFF;
    cmd[5] = 0x00;
    cmd[6] = 0x00;
    calcCheckSum(cmd,8);
    sendCommand(cmd,8);
}

void ServoController::changeServoId(unsigned char servoId, unsigned char newServoId) {
    unsigned char cmd[8];
    cmd[0] = 0xFA;
    cmd[1] = 0xAF;
    cmd[2] = servoId;
    cmd[3] = 0x00;
    cmd[4] = 0x04;
    cmd[5] = 0x01;
    cmd[6] = 0x01;//TODO When do change it.
    cmd[7] = newServoId;
    calcCheckSum(cmd,8);
    sendCommand(cmd,8);
}

void ServoController::reverseServoDirection(unsigned char servoId){
    unsigned char cmd[8];
    cmd[0] = 0xFA;
    cmd[1] = 0xAF;
    cmd[2] = servoId;
    cmd[3] = 0x00;
    cmd[4] = 0x05;
    cmd[5] = 0x01;
    cmd[5] = 0x01;
    cmd[6] = 0x01;//TODO reverse if after get current servo direction.
    calcCheckSum(cmd, 8);
    sendCommand(cmd, 8);
}

short ServoController::getCurrentAngle(unsigned char id){
    unsigned char memory[26];
    getMemoryMap(id, memory);
    short msb = (short)memory[8] & 0xffff;
    short lsb = (short)memory[7] & 0xffff;
    short angle =  ((msb << 8) | lsb);
    return angle;
}

void ServoController::turnOnTorque(unsigned char servoId){
    unsigned char cmd[9];

    cmd[0] = 0xFA; // Header
    cmd[1] = 0xAF; // Header
    cmd[2] = servoId; // ID
    cmd[3] = 0x00; // Flags
    cmd[4] = 0x24; // Address
    cmd[5] = 0x01; // Length
    cmd[6] = 0x01; // Count
    cmd[7] = 0x01; // Data
    calcCheckSum(cmd,9);
    sendCommand(cmd,9);
    delay(1);
    clearRxBuffer();    
}

void ServoController::turnOffTorque(unsigned char servoId){
    unsigned char cmd[9];

    cmd[0] = 0xFA; // Header
    cmd[1] = 0xAF; // Header
    cmd[2] = servoId; // ID
    cmd[3] = 0x00; // Flags
    cmd[4] = 0x24; // Address
    cmd[5] = 0x01; // Length
    cmd[6] = 0x01; // Count
    cmd[7] = 0x00; // Data
    calcCheckSum(cmd,9);
    sendCommand(cmd,9);
    delay(1);
    clearRxBuffer();    
}

void ServoController::breakTorque(unsigned char servoId){
    unsigned char cmd[9];

    cmd[0] = 0xFA; // Header
    cmd[1] = 0xAF; // Header
    cmd[2] = servoId; // ID
    cmd[3] = 0x00; // Flags
    cmd[4] = 0x24; // Address
    cmd[5] = 0x01; // Length
    cmd[6] = 0x01; // Count
    cmd[7] = 0x02; // Data
    calcCheckSum(cmd,9);
    sendCommand(cmd,9);
    delay(1);
    clearRxBuffer();    
}

void ServoController::moveServo(unsigned char servoId, int angle, int speed){
    unsigned char cmd[12]; //送信データバッファ [10byte]
    unsigned char CheckSum = 0; // チェックサム計算用変数
    cmd[0] = 0xFA; // Header
    cmd[1] = 0xAF; // Header
    cmd[2] = servoId; // ID
    cmd[3] = 0x00; // Flags
    cmd[4] = 0x1E; // Address
    cmd[5] = 0x04; // Length
    cmd[6] = 0x01; // Count
    // Angle
    cmd[7] = (unsigned char)0x00FF & angle; // Low byte
    cmd[8] = (unsigned char)0x00FF & (angle >> 8); //Hi byte
    // Speed
    cmd[9] = (unsigned char)0x00FF & speed; // Low byte
    cmd[10] = (unsigned char)0x00FF & (speed >> 8); //Hi byte
    
    calcCheckSum(cmd,12);
    sendCommand(cmd,12);
    delayMicroseconds(1250); // データ送信完了待ち
    clearRxBuffer();
}