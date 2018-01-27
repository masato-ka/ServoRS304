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
  HardwareSerial *serial;
  serial = &Serial;
  serial->begin(115200);
  delay(500);
}
```

## 

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


# Configuration

TODO.

# Author
Twitter: @masato-ka
e-mail: jp6uzv@gmail.com