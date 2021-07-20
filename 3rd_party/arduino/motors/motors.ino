/*
 * Arduino code for SN754410 H-bridge
 * motor driver control.
 * copyleft Feb. 2010, Fabian Winkler
 *
 */
int speedPin = 3; // H-bridge enable pin for speed control
int motor1APin = 6; // H-bridge leg 1
int motor2APin = 7; // H-bridge leg 2
int ledPin = 13;       // status LED
int speed_value_motor1; // value for motor speed
String val;


void setup() {
 // set digital i/o pins as outputs:
 pinMode(speedPin, OUTPUT);
 pinMode(motor1APin, OUTPUT);
 pinMode(motor2APin, OUTPUT);
 pinMode(ledPin, OUTPUT);
 Serial.begin(9600);
}

//Winkler, DC motor control with the Arduino board, p.2
void loop() {
 // put motor in forward motion
 digitalWrite(motor1APin, HIGH); // set leg 1 of the H-bridge low
 digitalWrite(motor2APin, LOW); // set leg 2 of the H-bridge high
  // just invert the above values for reverse motion,
 // i.e. motor1APin = HIGH and motor2APin = LOW
 
  if (Serial.available()) {
    val = Serial.readString(); //This reads value from python using USB
    int b = val.toInt(); //Because serial sends char so I convert from string to int
    Serial.println(b);
    // control the speed 0- 255
    speed_value_motor1 = b; // half speed
    analogWrite(speedPin, speed_value_motor1); // output speed as
    // PWM value
  }



 // control the speed 0- 255
 //speed_value_motor1 = b; // half speed
 //analogWrite(speedPin, speed_value_motor1); // output speed as
// PWM value
}
