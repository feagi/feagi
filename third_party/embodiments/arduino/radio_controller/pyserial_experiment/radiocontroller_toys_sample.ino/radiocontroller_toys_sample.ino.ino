#include <ArduinoJson.h>

String store_list[5];
void setup() {
    Serial.begin(9600); // Initialize serial communication
    pinMode(13, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(7, OUTPUT);
}

void loop() {
//  DynamicJsonDocument doc(1024);

  int     size_ = 0;
  String  payload;
  while ( !Serial.available()  ){}
  if ( Serial.available() )
    payload = Serial.readStringUntil( '\n' );
  StaticJsonDocument<512> doc;

  DeserializationError   error = deserializeJson(doc, payload);
  if (error) {
    Serial.println(error.c_str()); 
    return;
  }
  if (doc.containsKey("0")) {
    digitalWrite(13, HIGH);
    store_list[0] = "13";
    Serial.println("13 TRUE!");
  }
  if (doc.containsKey("1")) {
    digitalWrite(12, HIGH);
    store_list[1] = "12";
    Serial.println("12 TRUE!");
  }
  if (doc.containsKey("2")) {
    digitalWrite(8, HIGH);
    store_list[2] = "8";
    Serial.println("8 TRUE!");
  }
  if (doc.containsKey("3")) {
    digitalWrite(7, HIGH);
    store_list[3] = "7";
    Serial.println("7 TRUE!");
  }
  delay(2000);
  for (int i = 0; i < 5; i++) {
      if (store_list[i].length() > 0) { // Check if the string is not empty
          digitalWrite(store_list[i].toInt(), LOW);
          store_list[i] = ""; // empty the value
      }
  }
}
