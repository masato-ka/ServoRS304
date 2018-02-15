/*
ServoRS304.cpp
author masato-ka
Date 2018/02/03
License is LGPL 2.1
Copyright © 2018 Masato Kawamura. All rights reserved.
*/

#include<ServoRS304.h>

ServoController::ServoController(HardwareSerial& serial){
    //TODO If begin serial in this place on Wio LTE, do not work.
    hardwareSerial = &serial;
}

void ServoController::begin(){
    hardwareSerial->begin(115200);
}

void ServoController::begin(long baurate){
    hardwareSerial->begin(baurate);
}

void ServoController::end(){
    hardwareSerial->end();
}

void ServoController::setShortPacketHeader(unsigned char *cmd, unsigned char servoId, \
    unsigned char flag, unsigned char address){
    cmd[headerFAIndex] = FA;
    cmd[headerAFIndex] = AF;
    cmd[headerIdIndex] = servoId;
    cmd[headerFlagIndex] = flag;
    cmd[headerAddressIndex] = address;
}

void ServoController::setShortPacketData(unsigned char *cmd, int length , unsigned char *data){
    cmd[headerDataLengthIndex] = (unsigned char)0x00FF & length;
    cmd[headerCntIndex] = 0x01;
    for(int i=0; i < length; i++){
        cmd[packetHeaderSize+i] = data[i];
    }
}


//private 
void ServoController::calcCheckSum(unsigned char *cmd, int length){
    unsigned char checkSum = 0;
//    int length = sizeof(cmd)/sizeof(unsigned char)
    for(int i = headerIdIndex; i < length - packetCheckSumSize; i++ ){
        checkSum = checkSum ^ cmd [i];
    }
    cmd[length-1] = checkSum;
}

//TODO Serail servo controller class 
//private 
void ServoController::sendCommand(unsigned char *cmd, int length){
    const int oneByteBit = 8;
    hardwareSerial->write(cmd,length);
    int delayTime = sendDelayUSTimeOneByte * length;
    delayMicroseconds(delayTime + 1000);
}


void ServoController::clearRxBuffer(){
    while(hardwareSerial->available()){hardwareSerial->read();}
}

//private 
void ServoController::getMemoryMap(unsigned char servoId, unsigned char *buffer){
    unsigned char cmd[returnRequestPacketSize];
    unsigned char checkSum = 0;

    setShortPacketHeader(cmd, servoId, REQUEST_FLAG, 0x00);
    setShortPacketData(cmd, 0, NULL);
    calcCheckSum(cmd, returnRequestPacketSize);
    sendCommand(cmd, returnRequestPacketSize);
    delay(17);//TODO change parameter with return packet delay time and baudrate of USART.
    unsigned char readRaw[returnPacketSize+returnRequestPacketSize];
    int i = 0;
    while(hardwareSerial->available() > 0){
        readRaw[i++] = hardwareSerial->read();
    }
    memcpy(buffer, &readRaw[returnRequestPacketSize], returnPacketSize);
}

void ServoController::resetServo(unsigned char servoId){
    const int dataSize = 1;
    unsigned char cmd[packetHeaderSize+packetHeaderSize+dataSize];
    unsigned char data[dataSize];
    data[0]=REST_DATA;

    setShortPacketHeader(cmd, servoId, RESET_FLAG, RESET_ADDRESS);
    cmd[headerDataLengthIndex] = RESET_DATALENGTH;
    cmd[headerCntIndex] = RESET_CNT;
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
    sendCommand(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
}

void ServoController::restartServo(unsigned char servoId){
    const int dataSize = 0;
    unsigned char cmd[packetHeaderSize + packetCheckSumSize];
    setShortPacketHeader(cmd, servoId, RESTART_FLAG, RESTART_ADDRESS);
    setShortPacketData(cmd , dataSize, NULL);
    calcCheckSum(cmd, packetHeaderSize + packetCheckSumSize);
    sendCommand(cmd, packetHeaderSize + packetCheckSumSize);

    sendDelayUSTimeOneByte = (1 / usartspeed_set[usartSpeed]) * BIT_PER_BYTE * MICROSECOND;

}

void ServoController::changeServoId(unsigned char servoId, unsigned char newServoId) {
    const int dataSize = 1;
    unsigned char cmd[packetHeaderSize + packetHeaderSize + dataSize];
    unsigned char data[dataSize];
    data[0] = newServoId;
    setShortPacketHeader(cmd,servoId, WRITE_FLAG, SERVOID_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd,packetHeaderSize + packetHeaderSize + dataSize);
    sendCommand(cmd,packetHeaderSize + packetHeaderSize + dataSize);
}

void ServoController::changeUSARTSpeed(unsigned char servoId, unsigned char speed){
    const int dataSize = 1;
    unsigned char cmd[packetHeaderSize+packetCheckSumSize+dataSize];
    unsigned char data[dataSize];
    if(0x05 <= speed && speed <= 0x09){
        data[0] = speed;
    }else{
        data[0] = 0x07;
    }
    usartSpeed = data[0];
    setShortPacketHeader(cmd, servoId, WRITE_FLAG, USARTSPEED_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
    sendCommand(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
}

void ServoController::reverseServoDirection(unsigned char servoId){
    const int dataSize = 1;
    unsigned char cmd[packetHeaderSize+packetCheckSumSize+dataSize];
    unsigned char data[dataSize];
    if(servoDirection == 0x00){
        data[0] = 0x01;
    }else{
        data[0] = 0x00;
    }
    setShortPacketHeader(cmd, servoId, WRITE_FLAG, SERVODIRECTION_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    //cmd[6] = 0x01;//TODO reverse if after get current servo direction.
    calcCheckSum(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
    sendCommand(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
}

void ServoController::setPacketReturnTime(unsigned char servoId, unsigned char time){
    const int dataSize = 1;
    unsigned char cmd[packetHeaderSize+packetCheckSumSize+dataSize];
    unsigned char data[dataSize];
    returnDelayTime = time;
    data[0] = returnDelayTime;
    setShortPacketHeader(cmd, servoId, WRITE_FLAG, DELAYTIME_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
    sendCommand(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
}

void ServoController::setAngleLimit(unsigned char servoId, short cw_angleLimit, short ccw_angleLimit){
    const int dataSize = 4;
    unsigned char cmd[packetHeaderSize+packetCheckSumSize+dataSize];
    unsigned char data[dataSize];
    data[0] = (unsigned char) 0x00FF & cw_angleLimit;
    data[1] = (unsigned char) 0x00FF & (cw_angleLimit >> 8);
    data[2] = (unsigned char) 0x00FF & ccw_angleLimit;
    data[3] = (unsigned char) 0x00FF & (ccw_angleLimit >> 8);
    setShortPacketHeader(cmd, servoId, WRITE_FLAG, ANGLELIMIT_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd,packetHeaderSize+packetCheckSumSize+dataSize);
    sendCommand(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
}

void ServoController::setComplianceMergin(unsigned char servoId, unsigned char cw_mergin, unsigned char ccw_mergin){
    const int dataSize = 2;
    unsigned char cmd[packetHeaderSize+packetCheckSumSize+dataSize];
    unsigned char data[dataSize];
    data[0] = cw_mergin;
    data[1] = ccw_mergin;
    setShortPacketHeader(cmd, servoId, WRITE_FLAG, MERGIN_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd,packetHeaderSize+packetCheckSumSize+dataSize);
    sendCommand(cmd, packetHeaderSize+packetCheckSumSize+dataSize);

}

void ServoController::setComplianceSlope(unsigned char servoId, unsigned char cw_slope, unsigned char ccw_slope){
    const int dataSize = 2;
    unsigned char cmd[packetHeaderSize+packetCheckSumSize+dataSize];
    unsigned char data[dataSize];
    data[0] = cw_slope;
    data[1] = ccw_slope;
    setShortPacketHeader(cmd, servoId, WRITE_FLAG, SLOPE_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
    sendCommand(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
}

void ServoController::setPunch(unsigned char servoId, unsigned char cw_punch, unsigned char ccw_punch){
    const int dataSize = 2;
    unsigned char cmd[packetHeaderSize+packetCheckSumSize+dataSize];
    unsigned char data[dataSize];
    data[0] = cw_punch;
    data[1] = ccw_punch;
    setShortPacketHeader(cmd, servoId, WRITE_FLAG, PUNCH_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
    sendCommand(cmd, packetHeaderSize+packetCheckSumSize+dataSize);
}

void ServoController::setMaxTorque(unsigned char servoId, unsigned char torque){
    const int dataSize = 1;
    unsigned char cmd[packetHeaderSize + packetCheckSumSize + dataSize];
    unsigned char data[dataSize];
    data[0] = torque;
    setShortPacketHeader(cmd, servoId, WRITE_FLAG, MAXTORQUE_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd, packetHeaderSize+ packetCheckSumSize+dataSize);
    sendCommand(cmd, packetHeaderSize+ packetCheckSumSize + dataSize);
}

short ServoController::getCurrentAngle(unsigned char id){
    const int dataSize = 18;
    unsigned char memory[packetHeaderSize+packetCheckSumSize+dataSize];
    getMemoryMap(id, memory);
    short msb = (short)memory[8] & 0xffff;
    short lsb = (short)memory[7] & 0xffff;
    short angle =  ((msb << 8) | lsb);
    return angle;
}

int ServoController::getCurrentTime(unsigned char id){
    const int dataSize = 18;
    unsigned char memory[packetHeaderSize+packetCheckSumSize+dataSize];
    getMemoryMap(id, memory);
    short msb = (short) memory[10] & 0xffff;
    short lsb = (short) memory[9] & 0xffff;
    short currentTime = ((msb<<8) | lsb);
    return currentTime * 10;// 10ms
}

short ServoController::getCurrentSpeed(unsigned char id){
    const int dataSize = 18;
    unsigned char memory[packetHeaderSize+packetCheckSumSize+dataSize];
    getMemoryMap(id, memory);
    short msb = (short) memory[12] & 0xffff;
    short lsb = (short) memory[11] & 0xffff;
    short currentSpeed = ((msb<<8) | lsb);
    return currentSpeed;
}

short ServoController::getCurrentServoLoad(unsigned char id){
    const int dataSize = 18;
    unsigned char memory[packetHeaderSize+packetCheckSumSize+dataSize];
    getMemoryMap(id, memory);
    short msb = (short) memory[14] & 0xffff;
    short lsb = (short) memory[13] & 0xffff;
    short currentLoad = ((msb<<8) | lsb);
    return currentLoad;// 10ms
}

short ServoController::getCurrentThermo(unsigned char id){
    const int dataSize = 18;
    unsigned char memory[packetHeaderSize+packetCheckSumSize+dataSize];
    getMemoryMap(id, memory);
    short msb = (short) memory[16] & 0xffff;
    short lsb = (short) memory[15] & 0xffff;
    short currentThermo = ((msb<<8) | lsb);
    return currentThermo;// 10ms
}

float ServoController::getCurrentVoltage(unsigned char id){
    const int dataSize = 18;
    unsigned char memory[packetHeaderSize+packetCheckSumSize+dataSize];
    getMemoryMap(id, memory);
    short msb = (short) memory[18] & 0xffff;
    short lsb = (short) memory[17] & 0xffff;
    short currentVoltage = ((msb<<8) | lsb);
    return currentVoltage * 0.01;// 10ms
}


void ServoController::turnOnTorque(unsigned char servoId){
    const int dataSize = 1;
    unsigned char cmd[packetHeaderSize+packetCheckSumSize+dataSize];
    unsigned char data[dataSize] = {TORQUE_ON};
    setShortPacketHeader(cmd, servoId, WRITE_FLAG, TORQUE_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd,packetHeaderSize+packetCheckSumSize+dataSize);
    sendCommand(cmd,packetHeaderSize+packetCheckSumSize+dataSize);
    clearRxBuffer();    
}

void ServoController::turnOffTorque(unsigned char servoId){
    const int dataSize = 1;
    unsigned char cmd[packetHeaderSize+packetCheckSumSize+dataSize];
    unsigned char data[dataSize] = {TORQUE_OFF};
    setShortPacketHeader(cmd, servoId, WRITE_FLAG, TORQUE_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd,packetHeaderSize+packetCheckSumSize+dataSize);
    sendCommand(cmd,packetHeaderSize+packetCheckSumSize+dataSize);
    clearRxBuffer();    
}

void ServoController::breakTorque(unsigned char servoId){
    const int dataSize =1;
    unsigned char cmd[packetHeaderSize+packetCheckSumSize+dataSize];
    unsigned char data[dataSize] = {TORQUE_BREAK};
    setShortPacketHeader(cmd, servoId, WRITE_FLAG, TORQUE_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd,packetHeaderSize+packetCheckSumSize+dataSize);
    sendCommand(cmd,packetHeaderSize+packetCheckSumSize+dataSize);
    clearRxBuffer();    
}

void ServoController::moveServo(unsigned char servoId, int angle, int speed){
    const int dataSize = 4;
    unsigned char cmd[packetHeaderSize+packetCheckSumSize+dataSize]; //送信データバッファ [10byte]
    unsigned char data[dataSize];
    // Angle 
    data[0] = (unsigned char)0x00FF & angle; // Low byte
    data[1] = (unsigned char)0x00FF & (angle >> 8); //Hi byte
    // Speed
    data[2] = (unsigned char)0x00FF & speed; // Low byte
    data[3] = (unsigned char)0x00FF & (speed >> 8); //Hi byte 
    setShortPacketHeader(cmd, servoId, WRITE_FLAG, ANGLE_ADDRESS);
    setShortPacketData(cmd, dataSize, data);
    calcCheckSum(cmd,packetHeaderSize+packetCheckSumSize+dataSize);
    sendCommand(cmd,packetHeaderSize+packetCheckSumSize+dataSize);
    delayMicroseconds(1250); // データ送信完了待ち
    clearRxBuffer();
}