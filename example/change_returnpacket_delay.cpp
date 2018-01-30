#include<ServoRS304.h>

ServoController servoController(Serial);

void setup() {
    servoController.begin(115200);
}

void loop() {

    servoController.resetServo(0x01);
    servoController.restartServo(0x01);
    delay(500);
    //1ms=100μs+18(0x12)ｘ50μs
    servoController.setPacketReturnTime(0x01, 0x12);
    servoController.restartServo(0x01);
    delay(500);
    while(true){
        int angle = servoController.getCurrentAngle(0x01);
    }

}