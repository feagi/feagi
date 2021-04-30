
const int trigPin = 9;
const int echoPin = 10;
long duration;
int distance;
String incomingByte ;    

void setup() {

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600);
  pinMode(11, OUTPUT);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);

//  if (Serial.available() > 0) {
//
//  incomingByte = Serial.readStringUntil('\n');
//
//    if (incomingByte == "on") {
//
//      digitalWrite(11, HIGH);
//
//      Serial.write("Led on");
//
//    }
//
//    else if (incomingByte == "off") {
//
//      digitalWrite(11, LOW);
//
//      Serial.write("Led off");
//
//    }
//
//    else{
//
//     Serial.write("invald input");
//
//    }
//
//  }

}
