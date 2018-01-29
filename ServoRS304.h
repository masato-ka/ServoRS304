#ifndef ServoRS304_h
#define ServoRS304_h

#include<Arduino.h>
#include <string.h>
//TODO Should be adapt multi servo.

const int headerFAIndex = 0;
const int headerAFIndex = 1;
const int headerIdIndex = 2;
const int headerFlagIndex = 3;
const int headerAddressIndex = 4;
const int headerDataLengthIndex = 5;
const int headerCntIndex = 6;

const unsigned char headerFA = 0xFA;
const unsigned char headerAF = 0xAF;

const int packetHeaderSize = 7;
const int packetCheckSumSize = 1;

typedef struct PacketCMD{

    unsigned char header[2];
    unsigned char servoId;
    unsigned char flag;
    unsigned char headAddr;
    unsigned char dataLength;
    unsigned char servoCount;
    unsigned char data[1];
    unsigned char checkSum;

} packet_cmd;

class ServoController{
    private:
        unsigned char servoDirection = 0x00;
        unsigned char usartSpeed = 0x05;
        unsigned char returnDelayTime = 0x00;
        HardwareSerial *hardwareSerial;
        ServoController();
        void calcCheckSum(unsigned char* cmd, int length);
        void sendCommand(unsigned char *cmd, int length);
        void getMemoryMap(unsigned char id, unsigned char * buffer);
        PacketCMD* shortPacketFactory(unsigned char servoId, unsigned char flag, unsigned char headerAddr, \
        unsigned char dataLength, unsigned char *data);
        PacketCMD* returnPakcetFactory(unsigned char servoId, unsigned char flag, unsigned char headerAddr, \
        unsigned char dataLength, unsigned char *data);
        void clearRxBuffer();
        void setShortPacketHeader(unsigned char *cmd, unsigned char servoId, unsigned char flag, unsigned char address);
        void setShortPacketData(unsigned char *cmd, int length , unsigned char *data);
    public:
        ServoController(HardwareSerial& hardwareSerial);
        void begin();
        void begin(int baurate);
        void end();
        void initializeServo();
        void resetServo(unsigned char servoId);
        void restartServo(unsigned char servoId);
        void changeServoId(unsigned char servoId, unsigned char newId);
        void reverseServoDirection(unsigned char servoId);
        void changeUSARTSpeed(unsigned char servoId, unsigned char speed);
        void setPacketReturnTime(unsigned char servoId, unsigned char time);
        void setMaxTorque(unsigned char servoId, unsigned char torque);
        void setAngleLimit(unsigned char servoId, short cw_angleLimit, short ccw_angleLimit);
        void setThermoLimit(unsigned char servoId, short limit);
        void setNoSignalTorque(unsigned char servoId, short torque);
        void setInitialTime(unsigned char servoId, short time);
        void setComplianceMergin(unsigned char servoId, unsigned char cw_mergin, unsigned char ccw_mergin);
        void setComplianceSlope(unsigned char servoId, unsigned char cw_slope, unsigned char ccw_slope);
        void setPunch(unsigned char servoId, unsigned char cw_punch, unsigned char ccw_punch);

        short getCurrentAngle(unsigned char servoId);
        int getCurrentTime(unsigned char servoId);
        short getCurrentSpeed(unsigned char servoId);
        short getCurrentServoLoad(unsigned char servoId);
        short getCurrentThermo(unsigned char servoId);
        float getCurrentVoltage(unsigned char servoId);

        void turnOnTorque(unsigned char servoId);
        void turnOffTorque(unsigned char servoId);
        void breakTorque(unsigned char servoId);
        void moveServo(unsigned char servoId, int angle, int speed);
};


#endif