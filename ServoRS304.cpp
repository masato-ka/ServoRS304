#include<ServoRS304.h>
/* PacketCMD* ServoController::shortPacketFactory(unsigned char servoId, unsigned char flag, unsigned char headerAddr, \
        unsigned char dataLength, unsigned char *data){
    packet_cmd p = {""};
    
    
} */


ServoController::ServoController(HardwareSerial& serial){
    //TODO If begin serial in this place on Wio LTE, do not work.
    hardwareSerial = &serial;
}

void ServoController::begin(){
    hardwareSerial->begin(115200);
}

void ServoController::begin(int baurate){
    hardwareSerial->begin(baurate);
}

void ServoController::end(){
    hardwareSerial->end();
}

void ServoController::setShortPacketHeader(unsigned char *cmd, unsigned char servoId, \
    unsigned char flag, unsigned char address){
    cmd[headerFAIndex] = headerFA;
    cmd[headerAFIndex] = headerAF;
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
    unsigned char cmd[9];
    unsigned char data[1];
    data[0] = newServoId;
    setShortPacketHeader(cmd,servoId,0x00,0x04);
    setShortPacketData(cmd, 1, data);
    calcCheckSum(cmd,9);
    sendCommand(cmd,9);
}

void ServoController::changeUSARTSpeed(unsigned char servoId, unsigned char speed){
    unsigned char cmd[9];
    unsigned char data[1];
    if(0x05 <= speed && speed <= 0x09){
        data[0] = speed;
    }else{
        data[0] = 0x07;
    }
    usartSpeed = data[0];
    setShortPacketHeader(cmd, servoId, 0x00, 0x06);
    setShortPacketData(cmd, 1, data);
    calcCheckSum(cmd, 9);
    sendCommand(cmd, 9);
}

void ServoController::reverseServoDirection(unsigned char servoId){
    unsigned char cmd[9];
    unsigned char data[1];
    if(servoDirection == 0x00){
        data[0] = 0x01;
    }else{
        data[0] = 0x00;
    }
    setShortPacketHeader(cmd, servoId, 0x00, 0x05);
    setShortPacketData(cmd, 1, data);
    //cmd[6] = 0x01;//TODO reverse if after get current servo direction.
    calcCheckSum(cmd, 9);
    sendCommand(cmd, 9);
}

void ServoController::setPacketReturnTime(unsigned char servoId, unsigned char time){
    unsigned char cmd[9];
    unsigned char data[1];
    returnDelayTime = time;
    data[0] = returnDelayTime;
    setShortPacketHeader(cmd, servoId, 0x60, 0x07);
    setShortPacketData(cmd, 1, data);
    calcCheckSum(cmd, 9);
    sendCommand(cmd, 9);
}

void ServoController::setAngleLimit(unsigned char servoId, short cw_angleLimit, short ccw_angleLimit){
    unsigned char cmd[12];
    unsigned char data[4];
    data[0] = (unsigned char) 0x00FF & cw_angleLimit;
    data[1] = (unsigned char) 0x00FF & (cw_angleLimit >> 8);
    data[2] = (unsigned char) 0x00FF & ccw_angleLimit;
    data[3] = (unsigned char) 0x00FF & (ccw_angleLimit >> 8);
    setShortPacketHeader(cmd, servoId, 0x00, 0x08);
    setShortPacketData(cmd, 4, data);
    calcCheckSum(cmd,12);
    sendCommand(cmd, 12);
}

void ServoController::setComplianceMergin(unsigned char servoId, unsigned char cw_mergin, unsigned char ccw_mergin){
    unsigned char cmd[10];
    unsigned char data[2];
    data[0] = cw_mergin;
    data[1] = ccw_mergin;
    setShortPacketHeader(cmd, servoId, 0x00, 0x18);
    setShortPacketData(cmd, 2, data);
    calcCheckSum(cmd,10);
    sendCommand(cmd, 10);

}

void ServoController::setComplianceSlope(unsigned char servoId, unsigned char cw_slope, unsigned char ccw_slope){
    unsigned char cmd[10];
    unsigned char data[2];
    data[0] = cw_slope;
    data[1] = ccw_slope;
    setShortPacketHeader(cmd, servoId, 0x00, 0x1A);
    setShortPacketData(cmd, 2, data);
    calcCheckSum(cmd, 10);
    sendCommand(cmd, 10);
}

void ServoController::setPunch(unsigned char servoId, unsigned char cw_punch, unsigned char ccw_punch){
    unsigned char cmd[10];
    unsigned char data[2];
    data[0] = cw_punch;
    data[1] = ccw_punch;
    setShortPacketHeader(cmd, servoId, 0x00, 0x1C);
    setShortPacketData(cmd, 2, data);
    calcCheckSum(cmd, 10);
    sendCommand(cmd, 10);
}

short ServoController::getCurrentAngle(unsigned char id){
    unsigned char memory[26];
    getMemoryMap(id, memory);
    short msb = (short)memory[8] & 0xffff;
    short lsb = (short)memory[7] & 0xffff;
    short angle =  ((msb << 8) | lsb);
    return angle;
}

int ServoController::getCurrentTime(unsigned char id){
    unsigned char memory[26];
    getMemoryMap(id, memory);
    short msb = (short) memory[10] & 0xffff;
    short lsb = (short) memory[9] & 0xffff;
    short currentTime = ((msb<<8) | lsb);
    return currentTime * 10;// 10ms
}

short ServoController::getCurrentSpeed(unsigned char id){
    unsigned char memory[26];
    getMemoryMap(id, memory);
    short msb = (short) memory[12] & 0xffff;
    short lsb = (short) memory[11] & 0xffff;
    short currentSpeed = ((msb<<8) | lsb);
    return currentSpeed;
}

short ServoController::getCurrentServoLoad(unsigned char id){
        unsigned char memory[26];
    getMemoryMap(id, memory);
    short msb = (short) memory[14] & 0xffff;
    short lsb = (short) memory[13] & 0xffff;
    short currentLoad = ((msb<<8) | lsb);
    return currentLoad;// 10ms
}

short ServoController::getCurrentThermo(unsigned char id){
    unsigned char memory[26];
    getMemoryMap(id, memory);
    short msb = (short) memory[16] & 0xffff;
    short lsb = (short) memory[15] & 0xffff;
    short currentThermo = ((msb<<8) | lsb);
    return currentThermo;// 10ms
}

float ServoController::getCurrentVoltage(unsigned char id){
        unsigned char memory[26];
    getMemoryMap(id, memory);
    short msb = (short) memory[18] & 0xffff;
    short lsb = (short) memory[17] & 0xffff;
    short currentVoltage = ((msb<<8) | lsb);
    return currentVoltage * 0.01;// 10ms
}


void ServoController::turnOnTorque(unsigned char servoId){
    unsigned char cmd[9];
    unsigned char data[1] = {0x01};
    setShortPacketHeader(cmd, servoId, 0x00, 0x24);
    setShortPacketData(cmd, 1, data);
/*     cmd[0] = 0xFA; // Header
    cmd[1] = 0xAF; // Header
    cmd[2] = servoId; // ID
    cmd[3] = 0x00; // Flags
    cmd[4] = 0x24; // Address
    cmd[5] = 0x01; // Length
    cmd[6] = 0x01; // Count
    cmd[7] = 0x01; // Data
 */ 
    calcCheckSum(cmd,9);
    sendCommand(cmd,9);
    delay(1);
    clearRxBuffer();    
}

void ServoController::turnOffTorque(unsigned char servoId){
    unsigned char cmd[9];
    unsigned char data[1] = {0x00};
    setShortPacketHeader(cmd, servoId, 0x00, 0x24);
    setShortPacketData(cmd, 1, data);
    calcCheckSum(cmd,9);
    sendCommand(cmd,9);
    delay(1);
    clearRxBuffer();    
}

void ServoController::breakTorque(unsigned char servoId){
    unsigned char cmd[9];
    unsigned char data[1] = {0x02};
    setShortPacketHeader(cmd, servoId, 0x00, 0x24);
    setShortPacketData(cmd, 1, data);
    calcCheckSum(cmd,9);
    sendCommand(cmd,9);
    delay(1);
    clearRxBuffer();    
}

void ServoController::moveServo(unsigned char servoId, int angle, int speed){
    unsigned char cmd[12]; //送信データバッファ [10byte]
    unsigned char data[4];
    // Angle 
    data[0] = (unsigned char)0x00FF & angle; // Low byte
    data[1] = (unsigned char)0x00FF & (angle >> 8); //Hi byte
    // Speed
    data[2] = (unsigned char)0x00FF & speed; // Low byte
    data[3] = (unsigned char)0x00FF & (speed >> 8); //Hi byte 
    setShortPacketHeader(cmd, servoId, 0x00, 0x1E);
    setShortPacketData(cmd, 4, data);
    calcCheckSum(cmd,12);
    sendCommand(cmd,12);
    delayMicroseconds(1250); // データ送信完了待ち
    clearRxBuffer();
}