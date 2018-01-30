#include<ServoRS304.h>

ServoController servoController(Serial);

void setup() {
    servoController.begin(115200);
}

void loop() {

    servoController.turnOnTorque(0x01);
    servoController.moveServo(0x01, 450, 100);
    delay(1000);
    servoController.moveServo(0x01, -450, 100);
    delay(1000);
    servoController.moveServo(0x01, 0 , 50);
    servoController.turnOffTorque(0x01);
    delay(5000);
    
}