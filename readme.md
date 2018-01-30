# RS304 Controll library for Arduino

RS304 is Servo motor for hobby robot. It was developed by FUTABA.
The servo mortor controlled by serial communication (Half-duplex communication).
This library controll full function of the servo.
Servo and Arduino connect using 3-state circuit.



# Usage


## Initialize library.

``` c++
ServoController servoController(&Serial);
void setup() {
  servoController.begin(115200);
  delay(500);
}
```

## Move sarvo and get current angle.

``` c++
void loop() {
    SerialUSB.println("Start servo moving.");
    servoController.turnOnTorque(0x01);
    delay(1000);
    while(1){
        int angle = 0;
        SerialUSB.println("Loop");
        servoController.moveServo(0x01,300,100); //servoId, angle, speed
        delay(2000);
        angle = servoController.getCurrentAngle(0x01);
        SerialUSB.println(angle);
        delay(1000);
        servoController.moveServo(0x01,-300,100);
        delay(2000);
        angle = servoController.getCurrentAngle(0x01);
        SerialUSB.println(angle);
        delay(1000);
    }

}
```

Please show example folder.

# License

* Licensed under the GNU LESSER GENERAL PUBLIC LICENSE Version 2.1


# Author

 * Twitter: @masato-ka
 * Blog: http://masato-ka.hatenablog.com/
 * E-mail: jp6uzv(at)gmail.com