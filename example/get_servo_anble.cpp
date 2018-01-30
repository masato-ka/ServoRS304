#include<ServoRS304.h>

ServoController servoController(Serial);

void setup() {
    servoController.begin(115200);
}

void loop() {

    int angle;
    servoController.getCurrentAngle(0x01);
    delay(100);
}